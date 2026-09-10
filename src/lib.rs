// SPDX-License-Identifier: GPL-3.0-or-later

//! Interactive push and shove PCB router.
//!
//! A design level port of KiCad's PNS router as a standalone library:
//! no GUI, no board file format, no global state. A host hands over a
//! plain data description of its board, drives a session from the mouse,
//! draws the preview frame each event answers with, and applies the
//! commit diff the session ends on.
//!
//! # A whole session
//!
//! ```
//! use pnsrouter::geometry::seg::Seg;
//! use pnsrouter::geometry::shape::Shape;
//! use pnsrouter::geometry::vec2::Vec2;
//! use pnsrouter::item::{HostId, LayerRange, NetId};
//! use pnsrouter::node::World;
//! use pnsrouter::router::{FixOutcome, NewGeometry, Router};
//! use pnsrouter::rules::FixedClearance;
//! use pnsrouter::settings::{RouterMode, RoutingSettings, Sizes};
//! use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};
//!
//! // A round pad on one copper layer, as the host describes it.
//! fn pad(id: HostId, at: Vec2, net: NetId) -> WorldItem {
//!   WorldItem::new(
//!     id,
//!     Some(net),
//!     LayerRange::single(0),
//!     WorldGeometry::Solid {
//!       shape: Shape::circle(at, 400_000),
//!       pos: at,
//!       offset: Vec2::new(0, 0),
//!       orientation_degrees: 0.0,
//!       anchors: Vec::new(),
//!     },
//!   )
//! }
//!
//! let start = Vec2::new(0, 0);
//! let target = Vec2::new(4_000_000, 0);
//!
//! // The board: two pads to route between, and one track of another net
//! // standing across the straight path between them.
//! let mut snapshot = WorldSnapshot::new(2, World::DEFAULT_MAX_CLEARANCE);
//! snapshot.items.push(pad(HostId(1), start, NetId(1)));
//! snapshot.items.push(pad(HostId(2), target, NetId(1)));
//! snapshot.items.push(WorldItem::new(
//!   HostId(3),
//!   Some(NetId(2)),
//!   LayerRange::single(0),
//!   WorldGeometry::Segment {
//!     seg: Seg::new(
//!       Vec2::new(2_000_000, -2_000_000),
//!       Vec2::new(2_000_000, 2_000_000),
//!     ),
//!     width: 200_000,
//!   },
//! ));
//!
//! // What the route is made of, and which layers a via would span.
//! let mut sizes = Sizes {
//!   track_width: 200_000,
//!   board_min_track_width: 100_000,
//!   via_diameter: 600_000,
//!   via_drill: 300_000,
//!   ..Sizes::default()
//! };
//! sizes.add_layer_pair(0, 1);
//!
//! let settings = RoutingSettings {
//!   mode: RouterMode::Walkaround,
//!   ..RoutingSettings::default()
//! };
//!
//! let mut router = Router::new(
//!   &snapshot,
//!   Box::new(FixedClearance::uniform(100_000)),
//!   settings,
//!   sizes,
//! );
//!
//! // Press: start on the first pad, on layer 0.
//! router
//!   .start_routing(start, Some(HostId(1)), 0)
//!   .expect("the pad is routable");
//!
//! // Move: one call per mouse motion. The frame is everything the host
//! // draws, and it replaces the previous one whole.
//! let frame = router.move_to(target, Some(HostId(2)));
//! assert!(!frame.items.is_empty());
//!
//! // Click on the target pad, forcing the placement to finish there.
//! let diff = match router.fix_route(target, Some(HostId(2)), true) {
//!   FixOutcome::Finished(diff) => diff,
//!   FixOutcome::Continue(_) => panic!("a forced fix finishes the route"),
//! };
//!
//! // The host applies the diff in one undo transaction. Here the route
//! // is several segments, because it had to bend around the track.
//! assert!(diff.added.len() > 1);
//! assert!(
//!   diff
//!     .added
//!     .iter()
//!     .all(|item| matches!(item.geometry, NewGeometry::Segment { .. }))
//! );
//! ```
//!
//! # What a host provides
//!
//! Four things, and nothing else. A [`snapshot::WorldSnapshot`], which is
//! every copper object on the board as plain data with the host's own
//! object ids; an implementation of [`rules::RuleResolver`], which answers
//! the clearance between any two items ([`rules::FixedClearance`] is
//! enough for a board with one clearance); already snapped cursor points,
//! because snapping to pads, grids and existing tracks is host work and
//! the engine never guesses; and the application of the
//! [`router::CommitDiff`] a session ends on, deleting, creating and
//! rewriting board objects inside one undo transaction. The host reports
//! the ids it assigned back with [`router::Router::assign_host_ids`] if
//! the session carries on afterwards.
//!
//! Nothing in the engine reads the wall clock, iterates a hash container
//! or holds a callback other than the resolver, so one snapshot plus one
//! sequence of events always produces one answer.
//!
//! # The modules
//!
//! In the order a host meets them:
//!
//! - [`snapshot`]: the plain data description of a board, and the sync
//!   that turns it into a world.
//! - [`rules`]: the clearance oracle a host implements, with a fixed
//!   clearance implementation to start from.
//! - [`settings`]: the routing mode, the effort knobs and the sizes a
//!   placement uses.
//! - [`router`]: the session facade, the preview frame and the commit
//!   diff. This is the whole host API.
//! - [`eventlog`]: recording a session to text and replaying it, which is
//!   how regressions and host bug reports travel.
//! - [`debug`]: the hook an algorithm draws its own internals through.
//!
//! The engine underneath, in the order it is built up:
//!
//! - [`geometry`]: pure value geometry in integer nanometres, ported from
//!   KiCad's `libs/kimath`.
//! - [`arena`]: the generational arena that replaces KiCad's raw item
//!   pointers.
//! - [`item`]: the objects a world is made of, and their layers and nets.
//! - [`collide`]: the item level collision test.
//! - [`index`]: the broad phase, one R-tree per copper layer.
//! - [`joint`]: the connectivity graph laid over the items.
//! - [`node`]: the branching world every query and every speculative edit
//!   goes through.
//! - [`mod@line`]: a run of segments seen as one polyline.
//! - [`topology`]: connectivity queries, including the leading ratline.
//! - [`algo_base`]: what every routing algorithm is handed instead of a
//!   global router.
//! - [`walkaround`]: bending a line around what is in its way.
//! - [`shove`]: pushing what is in the way aside instead.
//! - [`optimizer`]: making a committed line shorter and less cornery.
//! - [`mouse_trail`]: which of the two 45 degree postures the head leaves
//!   in.
//! - [`via`]: walking a via out of what it collides with, which the
//!   placer and the dragger share.
//! - [`placer`]: the interactive placement state machine the facade
//!   drives.
//! - [`dragger`]: moving a segment, a corner or a via of an existing
//!   trace. Its mark obstacles path is complete; the walkaround, shove
//!   and via halves are still stubs, and the facade does not reach it
//!   yet.
//!
//! # Measuring and reproducing
//!
//! `examples/latency.rs` builds a synthetic board of a given size and
//! prints the distribution of per call latency in each routing mode; it is
//! the harness behind `doc/performance.md` and the place to add a
//! measurement. [`eventlog`] is the other entry point: a
//! [`eventlog::Recorder`] installed on a router writes the snapshot, the
//! settings, the sizes, the events and the resulting commit as text, and
//! [`eventlog::SessionRecording::from_text`] replays it against a fresh
//! router. A host that can drop such a file next to a bug report can have
//! the session reproduced exactly.

#![cfg_attr(feature = "fail-on-warnings", deny(warnings))]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(clippy::missing_docs_in_private_items)]

pub mod algo_base;
pub mod arena;
pub mod collide;
pub mod debug;
pub mod dragger;
pub mod eventlog;
pub mod geometry;
pub mod index;
pub mod item;
pub mod joint;
pub mod line;
pub mod mouse_trail;
pub mod node;
pub mod optimizer;
pub mod placer;
pub mod router;
pub mod rules;
pub mod settings;
pub mod shove;
pub mod snapshot;
pub mod topology;
pub mod via;
pub mod walkaround;
