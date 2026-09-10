// SPDX-License-Identifier: GPL-3.0-or-later

//! Interactive latency of a routing session on a large synthetic board.
//!
//! LibrePCB drives the router from the mouse: every motion event is one
//! [`Router::move_to`] and the host redraws the frame it answers with, so
//! a move that takes longer than one 60 Hz frame (16 ms) shows up as lag.
//! This example builds a board of a given size, drives one fixed sequence
//! of moves across it in each of the three routing modes, and prints the
//! distribution of the per call wall time.
//!
//! ```text
//! cargo run --release --example latency          # 2000 and 20000
//! cargo run --release --example latency -- 5000  # one board size
//! ```
//!
//! `PNSROUTER_SHOVE_ITERATION_LIMIT` overrides
//! [`RoutingSettings::shove_iteration_limit`]. That budget is what the
//! tail of the distribution turned out to be made of. See
//! `doc/performance.md`.
//!
//! `PNSROUTER_PARALLELISM` overrides [`Router::set_parallelism`], the
//! thread count of the obstacle query, whose default is 1. It lives here
//! and not in `src/` for the same reason the clock does: the crate reads
//! no environment. The parallel obstacle query section of
//! `doc/performance.md` is what it was added to measure, and what says
//! why the default is what it is.
//!
//! The board is a two layer Manhattan grid: horizontal traces on layer 0,
//! vertical traces on layer 1, one gap per row and per column so that a
//! route has somewhere to slip through, pads and vias on their own nets in
//! the space between the traces, and a rectangular outline of non routable
//! edge segments. Nothing here is random: the gap of a row sits a fixed
//! stride from the gap of the row before it, so one segment count always
//! produces one board and one sequence of measurements.
//!
//! The wall clock is read here and nowhere else. The crate itself never
//! reads it (`DESIGN.md` section 8), which is why this lives in an example
//! rather than behind a feature flag in `src/`.

#![forbid(unsafe_code)]

use std::env;
use std::process;
use std::time::{Duration, Instant};

use pnsrouter::geometry::seg::Seg;
use pnsrouter::geometry::shape::Shape;
use pnsrouter::geometry::vec2::Vec2;
use pnsrouter::item::{HostId, LayerRange, NetId, ViaType};
use pnsrouter::router::{FixOutcome, Router};
use pnsrouter::rules::FixedClearance;
use pnsrouter::settings::{RouterMode, RoutingSettings, Sizes};
use pnsrouter::snapshot::{WorldGeometry, WorldItem, WorldSnapshot};

// ---------------------------------------------------------------------
// The board
// ---------------------------------------------------------------------

/// The distance between two neighbouring traces of the grid, in
/// nanometres. One millimetre, which leaves a channel wide enough for the
/// routed track and its clearance on both sides.
const PITCH: i32 = 1_000_000;

/// The width of every track on the board and of the routed one.
const TRACK_WIDTH: i32 = 200_000;

/// The copper to copper clearance the whole board is built to.
const CLEARANCE: i32 = 200_000;

/// The copper radius of a pad of the grid.
const PAD_RADIUS: i32 = 150_000;

/// The radius of the hole drilled through a pad of the grid.
const PAD_DRILL_RADIUS: i32 = 50_000;

/// The copper diameter of a via of the grid, and of a via the router
/// places.
const VIA_DIAMETER: i32 = 350_000;

/// The drill diameter of a via of the grid, and of a via the router
/// places.
const VIA_DRILL: i32 = 200_000;

/// The width of one edge segment of the board outline.
const OUTLINE_WIDTH: i32 = 100_000;

/// How many segments one side of the board outline is broken into.
const OUTLINE_PIECES: usize = 8;

/// One pad of the grid every this many cells, in both directions.
const PAD_STRIDE: usize = 10;

/// One via of the grid every this many cells, in both directions.
const VIA_STRIDE: usize = 17;

/// The cell of the via lattice inside a [`VIA_STRIDE`] block.
const VIA_PHASE: usize = 3;

/// The cell the route starts on, in both directions.
const START_CELL: usize = 4;

/// How far the gap of one row moves along compared with the row before.
///
/// Coprime with the row count for every board this example builds, which
/// is what spreads the gaps out instead of lining them up in a column.
const GAP_STRIDE: usize = 37;

/// The same for the columns, so that the two layers do not share a
/// pattern.
const GAP_STRIDE_VERTICAL: usize = 53;

/// How many cells of a row the gap swallows.
const GAP_CELLS: usize = 2;

/// The net the route is on. No item of the board carries it, so every
/// obstacle keeps its clearance against the head.
const ROUTE_NET: NetId = NetId(1);

/// The first net id handed out to the board's own items.
const FIRST_BOARD_NET: u32 = 10;

/// How many copper layers the synthetic board has.
const COPPER_LAYERS: u8 = 2;

/// The board sizes measured when the command line names none.
const DEFAULT_SIZES: [usize; 2] = [2_000, 20_000];

/// A board and the handle the route starts on.
struct Board {
  /// The board itself, as a host would hand it over.
  snapshot: WorldSnapshot,
  /// The pad [`Board::start_at`] sits on.
  start_pad: HostId,
  /// Where the route starts.
  start_at: Vec2,
  /// How many track segments the grid holds.
  trace_segments: usize,
  /// How many pads the grid holds, the start pad included.
  pads: usize,
  /// How many vias the grid holds.
  vias: usize,
  /// How many edge segments the outline holds.
  outline_segments: usize,
  /// How long the snapshot took to build.
  build: Duration,
}

/// The state of the board generator: the items so far and the next id.
struct Generator {
  /// Every item built so far.
  items: Vec<WorldItem>,
  /// The next host id to hand out.
  next_host: u64,
  /// The next net id to hand out.
  next_net: u32,
  /// How many track segments were built.
  trace_segments: usize,
  /// How many pads were built.
  pads: usize,
  /// How many vias were built.
  vias: usize,
  /// How many edge segments were built.
  outline_segments: usize,
}

impl Generator {
  /// An empty board.
  fn new() -> Self {
    Self {
      items: Vec::new(),
      next_host: 1,
      next_net: FIRST_BOARD_NET,
      trace_segments: 0,
      pads: 0,
      vias: 0,
      outline_segments: 0,
    }
  }

  /// The next free host id.
  fn next_id(&mut self) -> HostId {
    let id = HostId(self.next_host);

    self.next_host += 1;
    id
  }

  /// The next free net id.
  fn next_net(&mut self) -> NetId {
    let net = NetId(self.next_net);

    self.next_net += 1;
    net
  }

  /// Break one straight run into `pieces` track segments of one net.
  ///
  /// Each segment is its own host object, which is what a board file
  /// holds: a polyline trace is a row of two point segments meeting at
  /// trivial joints.
  fn add_run(
    &mut self,
    net: NetId,
    layer: i32,
    from: Vec2,
    to: Vec2,
    pieces: usize,
  ) {
    for piece in 0..pieces {
      let start = interpolate(from, to, piece, pieces);
      let end = interpolate(from, to, piece + 1, pieces);

      if start == end {
        continue;
      }

      let id = self.next_id();

      self.items.push(WorldItem::new(
        id,
        Some(net),
        LayerRange::single(layer),
        WorldGeometry::Segment {
          seg: Seg::new(start, end),
          width: TRACK_WIDTH,
        },
      ));
      self.trace_segments += 1;
    }
  }

  /// One round pad with a hole, on one layer and on its own net.
  fn add_pad(&mut self, at: Vec2, layer: i32, net: NetId) -> HostId {
    let id = self.next_id();
    let mut item = WorldItem::new(
      id,
      Some(net),
      LayerRange::single(layer),
      WorldGeometry::Solid {
        shape: Shape::circle(at, PAD_RADIUS),
        pos: at,
        offset: Vec2::new(0, 0),
        orientation_degrees: 0.0,
        anchors: Vec::new(),
      },
    );

    item.hole = Some(Shape::circle(at, PAD_DRILL_RADIUS));
    self.items.push(item);
    self.pads += 1;
    id
  }

  /// One through via on its own net.
  fn add_via(&mut self, at: Vec2) {
    let id = self.next_id();
    let net = self.next_net();

    self.items.push(WorldItem::new(
      id,
      Some(net),
      LayerRange::new(0, i32::from(COPPER_LAYERS) - 1),
      WorldGeometry::Via {
        pos: at,
        diameter: VIA_DIAMETER,
        drill: VIA_DRILL,
        via_type: ViaType::Through,
        is_free: false,
      },
    ));
    self.vias += 1;
  }

  /// One piece of the board outline: netless, non routable copper on
  /// every layer, which is how a host syncs an edge cut.
  fn add_edge(&mut self, from: Vec2, to: Vec2) {
    let id = self.next_id();
    let mut item = WorldItem::new(
      id,
      None,
      LayerRange::new(0, i32::from(COPPER_LAYERS) - 1),
      WorldGeometry::Solid {
        shape: Shape::segment(Seg::new(from, to), OUTLINE_WIDTH),
        pos: from,
        offset: Vec2::new(0, 0),
        orientation_degrees: 0.0,
        anchors: Vec::new(),
      },
    );

    item.flags.routable = false;
    self.items.push(item);
    self.outline_segments += 1;
  }
}

/// The point `step / steps` of the way from `from` to `to`.
///
/// The products are formed in `i64`, as every product of coordinates in
/// the crate is (`DESIGN.md` section 2).
fn interpolate(from: Vec2, to: Vec2, step: usize, steps: usize) -> Vec2 {
  let numerator = step as i64;
  let denominator = steps as i64;
  let x = i64::from(from.x)
    + (i64::from(to.x) - i64::from(from.x)) * numerator / denominator;
  let y = i64::from(from.y)
    + (i64::from(to.y) - i64::from(from.y)) * numerator / denominator;

  Vec2::new(x as i32, y as i32)
}

/// The largest integer whose square does not exceed `value`.
fn integer_sqrt(value: usize) -> usize {
  let mut root = 0;

  while (root + 1) * (root + 1) <= value {
    root += 1;
  }
  root
}

/// Which cell of a row the gap starts at.
///
/// A fixed stride walk over the cells that are far enough from both ends
/// to leave a run on either side.
fn gap_cell(index: usize, lines: usize, stride: usize) -> usize {
  let room = lines - GAP_CELLS - 4;

  2 + (index * stride) % room
}

/// Build a board of roughly `target_segments` track segments.
///
/// The grid is square: `lines` traces per layer, each broken into
/// `pieces` segments, so the count is `2 * lines * pieces`, and `pieces`
/// grows with `lines` so that both the number of traces and the length of
/// one trace grow with the board.
fn build_board(target_segments: usize) -> Board {
  let started = Instant::now();
  let lines = integer_sqrt(2 * target_segments).max(16);
  let pieces = (lines / 4).max(1);
  let margin = PITCH;
  let span = (lines as i32 - 1) * PITCH;
  let side = span + 2 * margin;
  let mut generator = Generator::new();

  // The horizontal traces of layer 0 and the vertical ones of layer 1.
  for index in 0..lines {
    let along = margin + index as i32 * PITCH;
    let gap = gap_cell(index, lines, GAP_STRIDE);
    let vertical_gap = gap_cell(index, lines, GAP_STRIDE_VERTICAL);

    add_broken_trace(
      &mut generator,
      Vec2::new(margin, along),
      Vec2::new(margin + span, along),
      0,
      gap,
      pieces,
    );
    add_broken_trace(
      &mut generator,
      Vec2::new(along, margin),
      Vec2::new(along, margin + span),
      1,
      vertical_gap,
      pieces,
    );
  }

  // The pads and the vias, in the middle of a cell so that they keep
  // their clearance to the four traces around them.
  for row in 0..lines.saturating_sub(1) {
    for column in 0..lines.saturating_sub(1) {
      let at = cell_centre(margin, row, column);
      let is_pad = row % PAD_STRIDE == 0 && column % PAD_STRIDE == 0;
      let is_via =
        row % VIA_STRIDE == VIA_PHASE && column % VIA_STRIDE == VIA_PHASE;

      if is_pad {
        let net = generator.next_net();

        generator.add_pad(at, 0, net);
      } else if is_via {
        generator.add_via(at);
      }
    }
  }

  // The pad the route starts on, on a net of its own.
  let start_at = cell_centre(margin, START_CELL, START_CELL);
  let start_pad = generator.add_pad(start_at, 0, ROUTE_NET);

  // The outline.
  let corners = [
    Vec2::new(0, 0),
    Vec2::new(side, 0),
    Vec2::new(side, side),
    Vec2::new(0, side),
  ];

  for (index, from) in corners.iter().enumerate() {
    let to = corners[(index + 1) % corners.len()];

    for piece in 0..OUTLINE_PIECES {
      generator.add_edge(
        interpolate(*from, to, piece, OUTLINE_PIECES),
        interpolate(*from, to, piece + 1, OUTLINE_PIECES),
      );
    }
  }

  let snapshot = WorldSnapshot {
    copper_layer_count: COPPER_LAYERS,
    max_clearance: CLEARANCE,
    items: generator.items,
    edge_exclusions: Vec::new(),
  };

  Board {
    snapshot,
    start_pad,
    start_at,
    trace_segments: generator.trace_segments,
    pads: generator.pads,
    vias: generator.vias,
    outline_segments: generator.outline_segments,
    build: started.elapsed(),
  }
}

/// The centre of one cell of the grid, where a pad or a via goes.
fn cell_centre(margin: i32, row: usize, column: usize) -> Vec2 {
  Vec2::new(
    margin + column as i32 * PITCH + PITCH / 2,
    margin + row as i32 * PITCH + PITCH / 2,
  )
}

/// One trace of the grid: two runs with a gap between them.
///
/// The pieces are split between the two runs in proportion to their
/// length, so the segments of a board are all about the same size.
fn add_broken_trace(
  generator: &mut Generator,
  from: Vec2,
  to: Vec2,
  layer: i32,
  gap: usize,
  pieces: usize,
) {
  let net = generator.next_net();
  let cells = usize::max(
    ((to.x - from.x).abs() + (to.y - from.y).abs()) as usize / PITCH as usize,
    1,
  );
  let before = gap;
  let first = usize::max(pieces * before / cells, 1);
  let second = usize::max(pieces.saturating_sub(first), 1);

  generator.add_run(
    net,
    layer,
    from,
    interpolate(from, to, before, cells),
    first,
  );
  generator.add_run(
    net,
    layer,
    interpolate(from, to, before + GAP_CELLS, cells),
    to,
    second,
  );
}

// ---------------------------------------------------------------------
// The session
// ---------------------------------------------------------------------

/// How many moves one session performs.
const MOVES: usize = 96;

/// How far one move advances the cursor, in both directions.
const MOVE_STEP: i32 = PITCH / 2;

/// How far the cursor path is shifted sideways from the start pad.
///
/// A quarter of a pitch, which is what keeps the diagonal from ever
/// landing exactly on the centre of a pad or a via of the lattice: those
/// all sit at whole multiples of the pitch from each other in both
/// directions, and this offset breaks the tie in `x` only.
const CURSOR_OFFSET: i32 = PITCH / 4;

/// A corner is fixed after this many moves.
const FIX_EVERY: usize = 12;

/// One move that took longer than this is a dropped frame at 60 Hz.
const FRAME_BUDGET: Duration = Duration::from_millis(16);

/// The environment variable that overrides the shove's iteration budget.
const SHOVE_BUDGET_VARIABLE: &str = "PNSROUTER_SHOVE_ITERATION_LIMIT";

/// The environment variable that overrides the obstacle query's threads.
const PARALLELISM_VARIABLE: &str = "PNSROUTER_PARALLELISM";

/// What one session cost, call by call.
struct Timings {
  /// [`Router::new`], which is where the snapshot becomes a world.
  new: Duration,
  /// [`Router::start_routing`].
  start: Duration,
  /// One entry per [`Router::move_to`].
  moves: Vec<Duration>,
  /// One entry per [`Router::fix_route`].
  fixes: Vec<Duration>,
  /// [`Router::stop_routing`].
  stop: Duration,
  /// How many preview items the session drew in total, summed so that
  /// nothing in the measured calls can be optimised away.
  drawn: usize,
  /// How many segments the session committed.
  committed: usize,
  /// Whether a fix ended the session before the moves ran out.
  ended_early: bool,
}

/// The shove budget the sessions run with.
///
/// [`RoutingSettings::shove_iteration_limit`] is KiCad's default of 250
/// unless the environment names another one, which is how
/// `doc/performance.md` measures what the tail of the distribution buys.
fn shove_budget() -> u32 {
  env::var(SHOVE_BUDGET_VARIABLE)
    .ok()
    .and_then(|value| value.parse().ok())
    .unwrap_or_else(|| RoutingSettings::default().shove_iteration_limit)
}

/// The thread count the sessions run the obstacle query on.
///
/// [`None`] leaves the world at its default, which is what a host that
/// never touches the knob gets.
fn parallelism() -> Option<usize> {
  env::var(PARALLELISM_VARIABLE)
    .ok()
    .and_then(|value| value.parse().ok())
}

/// The sizes every session places with.
fn sizes() -> Sizes {
  let mut sizes = Sizes {
    clearance: CLEARANCE,
    min_clearance: CLEARANCE,
    track_width: TRACK_WIDTH,
    board_min_track_width: TRACK_WIDTH,
    via_diameter: VIA_DIAMETER,
    via_drill: VIA_DRILL,
    ..Sizes::default()
  };

  sizes.add_layer_pair(0, i32::from(COPPER_LAYERS) - 1);
  sizes
}

/// Drive one session across the board in one mode.
fn run_session(board: &Board, mode: RouterMode) -> Timings {
  let settings = RoutingSettings {
    mode,
    shove_iteration_limit: shove_budget(),
    ..RoutingSettings::default()
  };
  let started = Instant::now();
  let mut router = Router::new(
    &board.snapshot,
    Box::new(FixedClearance::uniform(CLEARANCE)),
    settings,
    sizes(),
  );
  let new = started.elapsed();

  if let Some(threads) = parallelism() {
    router.set_parallelism(threads);
  }

  let mut timings = Timings {
    new,
    start: Duration::ZERO,
    moves: Vec::with_capacity(MOVES),
    fixes: Vec::new(),
    stop: Duration::ZERO,
    drawn: 0,
    committed: 0,
    ended_early: false,
  };

  let started = Instant::now();
  let frame = router
    .start_routing(board.start_at, Some(board.start_pad), 0)
    .expect("the start pad of the synthetic board is routable");

  timings.start = started.elapsed();
  timings.drawn += frame.items.len();

  for step in 1..=MOVES {
    let at = Vec2::new(
      board.start_at.x + CURSOR_OFFSET + step as i32 * MOVE_STEP,
      board.start_at.y + step as i32 * MOVE_STEP,
    );
    let started = Instant::now();
    let frame = router.move_to(at, None);

    timings.moves.push(started.elapsed());
    timings.drawn += frame.items.len() + frame.violations.len();

    if step % FIX_EVERY != 0 {
      continue;
    }

    let started = Instant::now();
    let outcome = router.fix_route(at, None, false);

    timings.fixes.push(started.elapsed());

    if let FixOutcome::Finished(diff) = outcome {
      timings.committed += diff.added.len();
      timings.ended_early = true;
      break;
    }
  }

  let started = Instant::now();
  let diff = router.stop_routing();

  timings.stop = started.elapsed();
  timings.committed += diff.added.len();
  timings
}

// ---------------------------------------------------------------------
// The report
// ---------------------------------------------------------------------

/// One duration in milliseconds.
fn ms(duration: Duration) -> f64 {
  duration.as_secs_f64() * 1000.0
}

/// The nearest rank percentile of a sorted slice.
fn percentile(sorted: &[Duration], percent: usize) -> Duration {
  if sorted.is_empty() {
    return Duration::ZERO;
  }

  let rank = (sorted.len() * percent).div_ceil(100).max(1);

  sorted[rank - 1]
}

/// The name of a mode as the table prints it.
fn mode_name(mode: RouterMode) -> &'static str {
  match mode {
    RouterMode::MarkObstacles => "mark obstacles",
    RouterMode::Shove => "shove",
    RouterMode::Walkaround => "walkaround",
  }
}

/// Print the header of the per mode table.
fn print_header() {
  println!(
    "  {:<14} {:>9} {:>8} {:>9} {:>9} {:>9} {:>7} {:>9} {:>9} {:>6} {:>7}",
    "mode",
    "new (ms)",
    "start",
    "move p50",
    "move p95",
    "move max",
    ">16 ms",
    "fix max",
    "stop",
    "segs",
    "drawn"
  );
}

/// Print one row of the per mode table.
fn print_row(mode: RouterMode, timings: &Timings) {
  let mut moves = timings.moves.clone();
  let mut fixes = timings.fixes.clone();

  moves.sort_unstable();
  fixes.sort_unstable();

  let slow = timings
    .moves
    .iter()
    .filter(|duration| **duration > FRAME_BUDGET)
    .count();

  println!(
    "  {:<14} {:>9.1} {:>8.3} {:>9.3} {:>9.3} {:>9.3} {:>7} {:>9.3} {:>9.3} \
     {:>6} {:>7}",
    mode_name(mode),
    ms(timings.new),
    ms(timings.start),
    ms(percentile(&moves, 50)),
    ms(percentile(&moves, 95)),
    ms(moves.last().copied().unwrap_or_default()),
    slow,
    ms(fixes.last().copied().unwrap_or_default()),
    ms(timings.stop),
    timings.committed,
    timings.drawn,
  );
}

/// Measure one board size in every mode.
fn measure(target_segments: usize) {
  let board = build_board(target_segments);

  println!();
  println!(
    "board of {} target segments: {} track segments, {} pads, {} vias, \
     {} edge segments, {} items",
    target_segments,
    board.trace_segments,
    board.pads,
    board.vias,
    board.outline_segments,
    board.snapshot.items.len()
  );
  println!(
    "snapshot build {:.1} ms, {} moves of {:.2} mm, a fix every {} moves",
    ms(board.build),
    MOVES,
    f64::from(MOVE_STEP) / 1e6,
    FIX_EVERY
  );
  print_header();

  for mode in [
    RouterMode::MarkObstacles,
    RouterMode::Walkaround,
    RouterMode::Shove,
  ] {
    let timings = run_session(&board, mode);

    print_row(mode, &timings);

    if timings.ended_early {
      println!(
        "    ({}: a fix finished the route after {} moves)",
        mode_name(mode),
        timings.moves.len()
      );
    }
  }
}

/// Read the board sizes from the command line.
fn requested_sizes() -> Vec<usize> {
  let mut sizes = Vec::new();

  for argument in env::args().skip(1) {
    match argument.parse::<usize>() {
      Ok(size) if size >= 100 => sizes.push(size),
      _ => {
        eprintln!(
          "usage: latency [segment count ...]  (each at least 100, \
           default {} and {})",
          DEFAULT_SIZES[0], DEFAULT_SIZES[1]
        );
        process::exit(2);
      }
    }
  }

  if sizes.is_empty() {
    sizes.extend_from_slice(&DEFAULT_SIZES);
  }
  sizes
}

/// Measure every board size the command line names.
fn main() {
  println!(
    "pnsrouter interactive latency, clearance {} nm, track {} nm, \
     max clearance {} nm",
    CLEARANCE, TRACK_WIDTH, CLEARANCE
  );

  for size in requested_sizes() {
    measure(size);
  }
}
