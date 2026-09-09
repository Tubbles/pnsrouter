// SPDX-License-Identifier: GPL-3.0-or-later

//! A reader for the `.kicad_pcb` boards the PNS regression cases route on.
//!
//! It covers what a router needs and nothing else: the layer stack, the
//! nets, the copper items (tracks, arcs, vias and pads), the board outline
//! on `Edge.Cuts` and the keepout areas. Silkscreen, courtyards, fills, 3D
//! models, text and the whole project side of the file are read past.
//!
//! # Units and axes
//!
//! Every coordinate, size and width in this module is an integer count of
//! nanometres, converted from the file's millimetre text by
//! [`super::sexpr::millimetres_to_nanometres`] without going through
//! `f64`. KiCad's board internal unit is the nanometre, so the conversion
//! is exact and lossless in both directions.
//!
//! The Y axis is left exactly as the file has it, which is already the
//! axis the router works in: Y grows downwards. That is verifiable inside
//! the corpus rather than taken on faith. In
//! `boards/shove_same_net_via.kicad_pcb` the footprint at
//! `(at 114.7 74.05 90)` carries a pad at local `(-0.9125 0)` and a track
//! on the same net ends at `(114.7 74.9625)`; the pad therefore sits at
//! `+0.9125` mm in Y from the footprint origin. KiCad's `RotatePoint`
//! (`libs/kimath/src/trigo.cpp:225`) maps `(x, y)` at 90 degrees to
//! `(y, -x)`, which sends local `(-0.9125, 0)` to `(0, +0.9125)`. A
//! rotation that is counter clockwise on screen therefore comes out as
//! that matrix only when Y grows downwards, so the file's Y axis points
//! down and no flip is applied anywhere below.
//!
//! # File format versions in the corpus
//!
//! The ten boards span three eras of the format and the reader handles all
//! three, because the alternative is dropping the two most interesting
//! boards:
//!
//! - `boards/simple.kicad_pcb` is version 20220914. Copper layers are
//!   numbered `F.Cu` 0 and `B.Cu` 31, there is a `(net <number> "<name>")`
//!   table, items refer to a net by number, and the unique identifier
//!   field is `(tstamp <uuid>)`.
//! - `boards/dp_test.kicad_pcb` is version 20250907. Copper layers are
//!   renumbered, `F.Cu` 0 and `B.Cu` 2, with inner layers on the even
//!   numbers upwards from 4 as `boards/video-v10.kicad_pcb` shows. The
//!   net table survives and the identifier field is now `(uuid "<uuid>")`.
//! - The other eight boards drop the net table entirely and refer to a
//!   net by name, `(net "GND")`. Two of them
//!   (`boards/drag-walk-optimize*.kicad_pcb`) also replace a footprint's
//!   `(at x y rot)` with `(transform (translate x y) (rotate deg)
//!   (scale sx sy))`.

use super::sexpr::{self, Node, ParseError};

/// A point on the board, in nanometres, Y growing downwards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Point {
  /// Distance right of the file origin, in nanometres.
  pub x: i64,
  /// Distance below the file origin, in nanometres.
  pub y: i64,
}

/// An axis aligned rectangle in nanometres, Y growing downwards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoundingBox {
  /// Smallest x of any point in the box.
  pub left: i64,
  /// Smallest y of any point in the box.
  pub top: i64,
  /// Largest x of any point in the box.
  pub right: i64,
  /// Largest y of any point in the box.
  pub bottom: i64,
}

impl BoundingBox {
  /// The box that just holds `point`.
  pub fn around(point: Point) -> Self {
    Self {
      left: point.x,
      top: point.y,
      right: point.x,
      bottom: point.y,
    }
  }

  /// Grow the box so that it also holds `point`.
  pub fn include(&mut self, point: Point) {
    self.left = self.left.min(point.x);
    self.top = self.top.min(point.y);
    self.right = self.right.max(point.x);
    self.bottom = self.bottom.max(point.y);
  }

  /// Whether `point` is inside the box or on its border.
  pub fn contains(&self, point: Point) -> bool {
    point.x >= self.left
      && point.x <= self.right
      && point.y >= self.top
      && point.y <= self.bottom
  }

  /// The same box grown by `margin` on every side.
  pub fn grown(&self, margin: i64) -> Self {
    Self {
      left: self.left - margin,
      top: self.top - margin,
      right: self.right + margin,
      bottom: self.bottom + margin,
    }
  }
}

/// One entry of the `(layers ...)` table.
#[derive(Debug, Clone)]
pub struct KicadLayer {
  /// The `PCB_LAYER_ID` the file assigns. Its meaning changed between
  /// format versions, see the module documentation, so nothing downstream
  /// should read arithmetic into it.
  pub id: i64,
  /// The canonical name, for example `F.Cu` or `Edge.Cuts`.
  pub name: String,
  /// The type token: `signal`, `power`, `mixed`, `jumper` or `user`. Only
  /// `user` layers are not copper.
  pub kind: String,
  /// The optional user visible name, present when the board renames a
  /// layer.
  pub user_name: Option<String>,
  /// Index of this layer among the copper layers, or `None` when it is
  /// not copper.
  ///
  /// The index is the layer's position in the file's own copper
  /// declaration order, which in all ten boards is `F.Cu`, the inner
  /// layers ascending, then `B.Cu`. That is exactly the numbering PNS
  /// uses: `PNS_KICAD_IFACE_BASE::GetPNSLayerFromBoardLayer`
  /// (`pcbnew/router/pns_kicad_iface.cpp:3052`) maps `F.Cu` to 0, `B.Cu`
  /// to `copperLayerCount - 1` and an inner layer to `id / 2 - 1`.
  /// Deriving it from the declaration order instead of from that formula
  /// is what lets the same reader handle the 2022 board, whose `B.Cu` is
  /// layer 31.
  pub copper_index: Option<usize>,
}

/// One net of the board.
#[derive(Debug, Clone)]
pub struct KicadNet {
  /// The net number.
  ///
  /// For a board that still carries a `(net <number> "<name>")` table this
  /// is KiCad's netcode. For the newer boards, which name nets inline and
  /// ship no table, it is synthesised: 0 for the unconnected net and then
  /// one number per distinct name in order of first appearance in the
  /// file. It is a stable identity within one read of one file and
  /// nothing more.
  pub number: i64,
  /// The net name, empty for the unconnected net.
  pub name: String,
}

/// A straight track segment.
#[derive(Debug, Clone)]
pub struct KicadSegment {
  /// One end.
  pub start: Point,
  /// The other end.
  pub end: Point,
  /// Track width.
  pub width: i64,
  /// Name of the copper layer it is on.
  pub layer: String,
  /// Index of that layer among the copper layers.
  pub copper_layer: usize,
  /// Index into [`KicadBoard::nets`], or `None` when the item carries no
  /// net at all.
  pub net: Option<usize>,
  /// The item's KIID, the identity the event log refers to items by.
  pub uuid: String,
}

/// A curved track segment.
///
/// Parsed, counted and carried through, but marked unsupported: the router
/// port has no arc track item yet, so a `WorldSnapshot` conversion has to
/// either refuse a board whose [`KicadBoard::arcs`] is non empty or
/// approximate each arc by a polyline. `boards/stickhub-extra-via.kicad_pcb`
/// is the only board in the corpus with any, and it has 180 of them.
/// KiCad's own regression comparison has the same gap from the other side:
/// `comparePnsItems` (`qa/tools/pns/pns_log_file.cpp:320`) has no `ARC_T`
/// branch, so two arcs on the same net and layers compare equal whatever
/// their geometry.
#[derive(Debug, Clone)]
pub struct KicadArc {
  /// One end.
  pub start: Point,
  /// A point on the arc between the ends, which with them fixes the
  /// circle.
  pub mid: Point,
  /// The other end.
  pub end: Point,
  /// Track width.
  pub width: i64,
  /// Name of the copper layer it is on.
  pub layer: String,
  /// Index of that layer among the copper layers.
  pub copper_layer: usize,
  /// Index into [`KicadBoard::nets`], or `None`.
  pub net: Option<usize>,
  /// The item's KIID.
  pub uuid: String,
}

/// A via.
#[derive(Debug, Clone)]
pub struct KicadVia {
  /// Centre.
  pub at: Point,
  /// Outer copper diameter.
  pub size: i64,
  /// Hole diameter.
  pub drill: i64,
  /// Name of the layer the via span starts on, the first name in the
  /// file's `(layers ...)` pair.
  pub layer_top: String,
  /// Name of the layer the via span ends on.
  pub layer_bottom: String,
  /// Copper index of [`KicadVia::layer_top`].
  pub copper_layer_top: usize,
  /// Copper index of [`KicadVia::layer_bottom`].
  pub copper_layer_bottom: usize,
  /// Whether the via carries the `(free yes)` flag, meaning the user
  /// detached it from its net's connectivity.
  pub free: bool,
  /// Index into [`KicadBoard::nets`], or `None`.
  pub net: Option<usize>,
  /// The item's KIID.
  pub uuid: String,
}

/// What a pad is electrically and mechanically.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PadKind {
  /// Surface mount, copper on one side only.
  SurfaceMount,
  /// Plated through hole.
  ThroughHole,
  /// Unplated through hole: a mounting hole, no copper.
  NonPlatedThroughHole,
  /// KiCad's `connect` pad: an edge connector finger, copper on one side
  /// and no solder paste.
  EdgeConnector,
}

/// The outline of a pad's copper.
#[derive(Debug, Clone, PartialEq)]
pub enum PadShape {
  /// A circle of diameter [`KicadPad::size`]`.x`.
  Circle,
  /// A rectangle of [`KicadPad::size`].
  Rectangle,
  /// A rectangle with rounded corners.
  RoundedRectangle {
    /// Corner radius as a fraction of the shorter side, KiCad's
    /// `roundrect_rratio`.
    ratio: f64,
  },
  /// A stadium: a rectangle capped with semicircles on the shorter axis.
  Oval,
  /// A trapezoid, which the corpus never uses but the format allows.
  Trapezoid,
  /// A free outline built from primitives.
  Custom {
    /// The primitives, each already a closed polygon in absolute board
    /// coordinates. See [`KicadPad`] for how they were transformed.
    primitives: Vec<Vec<Point>>,
  },
}

/// The hole of a through hole pad.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PadDrill {
  /// A round hole.
  Round {
    /// Hole diameter.
    diameter: i64,
  },
  /// A slot.
  Oval {
    /// Slot width along the pad's local X axis.
    width: i64,
    /// Slot width along the pad's local Y axis.
    height: i64,
  },
}

/// A footprint pad.
///
/// # How the footprint transform was applied
///
/// [`KicadPad::at`] and every point in [`PadShape::Custom`] are absolute
/// board coordinates. Three facts drive the transform, all of them checked
/// against the corpus rather than assumed:
///
/// 1. The pad's `(at x y)` is the footprint local position, and the
///    absolute position is `footprintOrigin + RotatePoint(local,
///    footprintRotation)` with KiCad's own `RotatePoint`
///    (`libs/kimath/src/trigo.cpp:225`), which for an angle of 90 degrees
///    maps `(x, y)` to `(y, -x)`.
/// 2. **No mirroring is applied for a footprint on the back side.** KiCad
///    bakes the flip into the stored pad positions and layer names when
///    the user flips a footprint, so reading the file back must not flip
///    again. Checked across the whole corpus: of the 681 pads that belong
///    to a back side footprint, 269 land exactly on a track endpoint of
///    their own net when no mirroring is applied and 7 do when the local
///    X is mirrored, and 5 of those 7 are pads at local `x = 0`, where
///    the two readings are the same point.
/// 3. The angle in the pad's own `(at x y rot)` is the **absolute**
///    orientation, not one relative to the footprint. A pad in an
///    unrotated library footprint placed at 90 degrees is written
///    `(at ... 90)`. Across the corpus 3915 of the 4395 pads repeat their
///    footprint's angle exactly and 480 differ, the latter being pads
///    their library footprint already rotates.
///
/// A `WorldSnapshot` conversion therefore needs no transform of its own,
/// only the layer and net mapping.
#[derive(Debug, Clone)]
pub struct KicadPad {
  /// KIID of the footprint the pad belongs to.
  pub footprint_uuid: String,
  /// Reference designator of that footprint, for example `R12`. Empty
  /// when the footprint has none.
  pub footprint_reference: String,
  /// The pad number as text, because KiCad allows `A1` and `CAGE` as
  /// readily as `1`.
  pub number: String,
  /// What the pad is.
  pub kind: PadKind,
  /// The outline of its copper.
  pub shape: PadShape,
  /// Absolute centre on the board.
  pub at: Point,
  /// Copper extent, `x` across and `y` down in the pad's own frame.
  pub size: Point,
  /// Absolute orientation in degrees, counter clockwise on screen.
  pub rotation_degrees: f64,
  /// The layer names exactly as the file lists them, wildcards included.
  pub layer_names: Vec<String>,
  /// The copper layers the pad is on, as copper indices, ascending.
  /// `*.Cu` has already been expanded to every copper layer.
  pub copper_layers: Vec<usize>,
  /// Index into [`KicadBoard::nets`], or `None` for an unconnected pad.
  pub net: Option<usize>,
  /// The hole, for a through hole pad.
  pub drill: Option<PadDrill>,
  /// The pad's KIID.
  pub uuid: String,
}

/// A graphic item, used here for the board outline on `Edge.Cuts`.
#[derive(Debug, Clone)]
pub enum KicadGraphic {
  /// A straight line.
  Line {
    /// One end.
    start: Point,
    /// The other end.
    end: Point,
    /// Stroke width.
    width: i64,
    /// The item's KIID.
    uuid: String,
  },
  /// A circular arc through three points.
  Arc {
    /// One end.
    start: Point,
    /// A point on the arc between the ends.
    mid: Point,
    /// The other end.
    end: Point,
    /// Stroke width.
    width: i64,
    /// The item's KIID.
    uuid: String,
  },
  /// An axis aligned rectangle given by two opposite corners.
  Rectangle {
    /// One corner.
    start: Point,
    /// The opposite corner.
    end: Point,
    /// Stroke width.
    width: i64,
    /// The item's KIID.
    uuid: String,
  },
  /// A closed polygon.
  Polygon {
    /// The corners, in file order.
    points: Vec<Point>,
    /// Stroke width.
    width: i64,
    /// The item's KIID.
    uuid: String,
  },
}

impl KicadGraphic {
  /// Grow `bounds` so that it holds every defining point of this item.
  ///
  /// For an arc that is the three points, not the true extent of the
  /// curve, which is enough for a sanity check and avoids solving for the
  /// circle here.
  fn extend_bounds(&self, bounds: &mut BoundingBox) {
    match self {
      KicadGraphic::Line { start, end, .. }
      | KicadGraphic::Rectangle { start, end, .. } => {
        bounds.include(*start);
        bounds.include(*end);
      }
      KicadGraphic::Arc {
        start, mid, end, ..
      } => {
        bounds.include(*start);
        bounds.include(*mid);
        bounds.include(*end);
      }
      KicadGraphic::Polygon { points, .. } => {
        for point in points {
          bounds.include(*point);
        }
      }
    }
  }

  /// The first defining point, used to seed a bounding box.
  fn first_point(&self) -> Option<Point> {
    match self {
      KicadGraphic::Line { start, .. }
      | KicadGraphic::Rectangle { start, .. }
      | KicadGraphic::Arc { start, .. } => Some(*start),
      KicadGraphic::Polygon { points, .. } => points.first().copied(),
    }
  }
}

/// A rule area: a zone that forbids something rather than filling copper.
///
/// One is produced per `(polygon ...)` outline of a zone that carries a
/// `(keepout ...)` block. Zones nested inside a footprint are included and
/// need no transform: KiCad stores a footprint's rule area outline in
/// absolute board coordinates, checked on
/// `boards/ultrasound.kicad_pcb`, where the footprint at `(92.5 66.275)`
/// holds a keepout whose outline is written as `(xy 93.2 62.675)` and up.
#[derive(Debug, Clone)]
pub struct KicadKeepout {
  /// The zone's KIID. Several keepouts share it when a zone has more than
  /// one outline.
  pub uuid: String,
  /// Names of the layers the rule applies to.
  pub layer_names: Vec<String>,
  /// Those of them that are copper, as copper indices, ascending.
  pub copper_layers: Vec<usize>,
  /// The outline, in absolute board coordinates.
  pub outline: Vec<Point>,
  /// Whether tracks may enter.
  pub tracks_allowed: bool,
  /// Whether vias may enter.
  pub vias_allowed: bool,
  /// Whether pads may enter.
  pub pads_allowed: bool,
  /// Whether a copper pour may fill into the area.
  pub copper_pour_allowed: bool,
  /// Whether footprints may be placed inside.
  pub footprints_allowed: bool,
}

/// Everything a router needs out of a `.kicad_pcb` file.
#[derive(Debug, Clone)]
pub struct KicadBoard {
  /// The file name the board was read from, for test messages.
  pub source_name: String,
  /// The `(version ...)` stamp, a date as `YYYYMMDD`.
  pub file_version: i64,
  /// How many copper layers the stack has.
  pub copper_layer_count: usize,
  /// The whole layer table, copper and otherwise, in file order.
  pub layers: Vec<KicadLayer>,
  /// The nets, see [`KicadNet::number`] for how they are numbered.
  pub nets: Vec<KicadNet>,
  /// Straight track segments.
  pub segments: Vec<KicadSegment>,
  /// Curved track segments, which the router port does not support yet,
  /// see [`KicadArc`].
  pub arcs: Vec<KicadArc>,
  /// Vias.
  pub vias: Vec<KicadVia>,
  /// Pads of every footprint, with the footprint transform applied.
  pub pads: Vec<KicadPad>,
  /// Graphic items on `Edge.Cuts`.
  pub board_outline: Vec<KicadGraphic>,
  /// Rule areas.
  pub keepouts: Vec<KicadKeepout>,
}

impl KicadBoard {
  /// Whether the board holds items no `WorldSnapshot` conversion can
  /// represent yet.
  pub fn has_unsupported_items(&self) -> bool {
    !self.arcs.is_empty()
  }

  /// The name of net `index`, or `""` when the index is out of range.
  pub fn net_name(&self, index: usize) -> &str {
    self.nets.get(index).map_or("", |net| net.name.as_str())
  }

  /// Index of the copper layer called `name`.
  pub fn copper_index(&self, name: &str) -> Option<usize> {
    self
      .layers
      .iter()
      .find(|layer| layer.name == name)
      .and_then(|layer| layer.copper_index)
  }

  /// The bounding box of the `Edge.Cuts` outline, or `None` when the
  /// board has no outline.
  pub fn outline_bounds(&self) -> Option<BoundingBox> {
    let mut items = self.board_outline.iter();
    let mut bounds = loop {
      let item = items.next()?;
      if let Some(point) = item.first_point() {
        break BoundingBox::around(point);
      }
    };
    for item in &self.board_outline {
      item.extend_bounds(&mut bounds);
    }
    Some(bounds)
  }
}

/// Read a board out of the text of a `.kicad_pcb` file.
///
/// `source_name` only ever appears in error and assertion messages.
pub fn read_board(
  source_name: &str,
  text: &str,
) -> Result<KicadBoard, ParseError> {
  let root = sexpr::parse(text)?;
  if root.tag() != Some("kicad_pcb") {
    return Err(ParseError::new(
      root.line(),
      "the file does not start with `(kicad_pcb`",
    ));
  }

  let file_version = root.required_child("version")?.value_integer(0)?;
  let layers = read_layers(&root)?;
  let copper_layer_count = layers
    .iter()
    .filter(|layer| layer.copper_index.is_some())
    .count();
  let nets = read_nets(&root)?;

  let mut board = KicadBoard {
    source_name: source_name.to_string(),
    file_version,
    copper_layer_count,
    layers,
    nets,
    segments: Vec::new(),
    arcs: Vec::new(),
    vias: Vec::new(),
    pads: Vec::new(),
    board_outline: Vec::new(),
    keepouts: Vec::new(),
  };

  for item in root.values() {
    match item.tag() {
      Some("segment") => {
        let segment = read_segment(&board, item)?;
        board.segments.push(segment);
      }
      Some("arc") => {
        let arc = read_arc(&board, item)?;
        board.arcs.push(arc);
      }
      Some("via") => {
        let via = read_via(&board, item)?;
        board.vias.push(via);
      }
      Some("zone") => read_zone(&mut board, item)?,
      Some("footprint") => read_footprint(&mut board, item)?,
      Some("gr_line" | "gr_arc" | "gr_rect" | "gr_poly") => {
        if let Some(graphic) = read_board_outline_graphic(item)? {
          board.board_outline.push(graphic);
        }
      }
      _ => {}
    }
  }

  Ok(board)
}

/// Read the `(layers ...)` table and assign the copper indices.
fn read_layers(root: &Node) -> Result<Vec<KicadLayer>, ParseError> {
  let table = root.required_child("layers")?;
  let mut layers = Vec::new();
  let mut next_copper_index = 0;
  for entry in table.values() {
    let items = entry.items();
    if items.len() < 3 {
      return Err(ParseError::new(entry.line(), "malformed layer entry"));
    }
    let id = items[0]
      .as_str()
      .and_then(|text| text.parse().ok())
      .ok_or_else(|| {
        ParseError::new(entry.line(), "layer entry has no numeric id")
      })?;
    let name = items[1]
      .as_str()
      .ok_or_else(|| ParseError::new(entry.line(), "layer entry has no name"))?
      .to_string();
    let kind = items[2]
      .as_str()
      .ok_or_else(|| ParseError::new(entry.line(), "layer entry has no type"))?
      .to_string();
    let user_name = items
      .get(3)
      .and_then(Node::as_str)
      .map(std::string::ToString::to_string);
    let copper_index = if kind == "user" {
      None
    } else {
      let index = next_copper_index;
      next_copper_index += 1;
      Some(index)
    };
    layers.push(KicadLayer {
      id,
      name,
      kind,
      user_name,
      copper_index,
    });
  }
  Ok(layers)
}

/// Read the net table, synthesising one when the file has none.
fn read_nets(root: &Node) -> Result<Vec<KicadNet>, ParseError> {
  let mut nets: Vec<KicadNet> = Vec::new();
  for entry in root.children("net") {
    if entry.values().len() < 2 {
      continue;
    }
    nets.push(KicadNet {
      number: entry.value_integer(0)?,
      name: entry.value_str(1)?.to_string(),
    });
  }

  if !nets.is_empty() {
    return Ok(nets);
  }

  // The newest boards name their nets inline and ship no table at all.
  // Walk the whole document once, in document order, and build a table out
  // of the names that appear. Net 0 is the unconnected net, as it is in
  // every table KiCad does write.
  nets.push(KicadNet {
    number: 0,
    name: String::new(),
  });
  let mut names = Vec::new();
  collect_inline_net_names(root, &mut names);
  for (offset, name) in names.into_iter().enumerate() {
    let number = i64::try_from(offset)
      .map_err(|_| ParseError::new(root.line(), "too many nets to number"))?
      + 1;
    nets.push(KicadNet { number, name });
  }
  Ok(nets)
}

/// Collect every distinct `(net "<name>")` value in document order.
fn collect_inline_net_names(node: &Node, names: &mut Vec<String>) {
  if node.tag() == Some("net")
    && node.values().len() == 1
    && let Some(name) = node.values()[0].as_text()
    && !name.is_empty()
    && !names.iter().any(|known| known == name)
  {
    names.push(name.to_string());
  }
  for item in node.items() {
    collect_inline_net_names(item, names);
  }
}

/// The KIID of an item, from `(uuid "...")` or the older `(tstamp ...)`.
fn read_uuid(item: &Node) -> String {
  item
    .child("uuid")
    .or_else(|| item.child("tstamp"))
    .and_then(|node| node.values().first())
    .and_then(Node::as_str)
    .unwrap_or_default()
    .to_string()
}

/// Read a two coordinate child such as `(start x y)` or `(xy x y)`.
fn read_point(node: &Node) -> Result<Point, ParseError> {
  Ok(Point {
    x: node.value_millimetres(0)?,
    y: node.value_millimetres(1)?,
  })
}

/// Read the named two coordinate child of `item`.
fn read_point_child(item: &Node, tag: &str) -> Result<Point, ParseError> {
  read_point(item.required_child(tag)?)
}

/// Resolve the `(net ...)` child of an item to an index into the board's
/// net table.
///
/// Both spellings are accepted because both are in the corpus: a number,
/// which is a netcode into the file's own table, and a quoted name, which
/// is how the newest format refers to a net.
fn read_net(
  board: &KicadBoard,
  item: &Node,
) -> Result<Option<usize>, ParseError> {
  let Some(node) = item.child("net") else {
    return Ok(None);
  };
  let Some(value) = node.values().first() else {
    return Ok(None);
  };
  match value {
    Node::Text { text, line } => board
      .nets
      .iter()
      .position(|net| &net.name == text)
      .ok_or_else(|| {
        ParseError::new(*line, format!("no net is named `{text}`"))
      })
      .map(Some),
    Node::Symbol { text, line } => {
      let number: i64 = text.parse().map_err(|_| {
        ParseError::new(*line, format!("`{text}` is not a net number"))
      })?;
      board
        .nets
        .iter()
        .position(|net| net.number == number)
        .ok_or_else(|| {
          ParseError::new(*line, format!("no net has number {number}"))
        })
        .map(Some)
    }
    Node::List { line, .. } => {
      Err(ParseError::new(*line, "expected a net number or name"))
    }
  }
}

/// The copper index of the layer named by the `(layer ...)` child.
fn read_copper_layer(
  board: &KicadBoard,
  item: &Node,
) -> Result<(String, usize), ParseError> {
  let node = item.required_child("layer")?;
  let name = node.value_str(0)?.to_string();
  let index = board.copper_index(&name).ok_or_else(|| {
    ParseError::new(node.line(), format!("`{name}` is not a copper layer"))
  })?;
  Ok((name, index))
}

/// Read a `(segment ...)`.
fn read_segment(
  board: &KicadBoard,
  item: &Node,
) -> Result<KicadSegment, ParseError> {
  let (layer, copper_layer) = read_copper_layer(board, item)?;
  Ok(KicadSegment {
    start: read_point_child(item, "start")?,
    end: read_point_child(item, "end")?,
    width: item.required_child("width")?.value_millimetres(0)?,
    layer,
    copper_layer,
    net: read_net(board, item)?,
    uuid: read_uuid(item),
  })
}

/// Read an `(arc ...)`.
fn read_arc(board: &KicadBoard, item: &Node) -> Result<KicadArc, ParseError> {
  let (layer, copper_layer) = read_copper_layer(board, item)?;
  Ok(KicadArc {
    start: read_point_child(item, "start")?,
    mid: read_point_child(item, "mid")?,
    end: read_point_child(item, "end")?,
    width: item.required_child("width")?.value_millimetres(0)?,
    layer,
    copper_layer,
    net: read_net(board, item)?,
    uuid: read_uuid(item),
  })
}

/// Read a `(via ...)`.
fn read_via(board: &KicadBoard, item: &Node) -> Result<KicadVia, ParseError> {
  let layers = item.required_child("layers")?;
  let layer_top = layers.value_str(0)?.to_string();
  let layer_bottom = layers.value_str(1)?.to_string();
  let copper_layer_top = board.copper_index(&layer_top).ok_or_else(|| {
    ParseError::new(layers.line(), format!("`{layer_top}` is not copper"))
  })?;
  let copper_layer_bottom =
    board.copper_index(&layer_bottom).ok_or_else(|| {
      ParseError::new(layers.line(), format!("`{layer_bottom}` is not copper"))
    })?;
  let free = item.child("free").is_some_and(|node| {
    node.values().first().and_then(Node::as_str) != Some("no")
  });
  Ok(KicadVia {
    at: read_point_child(item, "at")?,
    size: item.required_child("size")?.value_millimetres(0)?,
    drill: item.required_child("drill")?.value_millimetres(0)?,
    layer_top,
    layer_bottom,
    copper_layer_top,
    copper_layer_bottom,
    free,
    net: read_net(board, item)?,
    uuid: read_uuid(item),
  })
}

/// Read a graphic item, keeping it only when it is on `Edge.Cuts`.
fn read_board_outline_graphic(
  item: &Node,
) -> Result<Option<KicadGraphic>, ParseError> {
  let on_edge_cuts = item
    .child("layer")
    .and_then(|node| node.values().first())
    .and_then(Node::as_str)
    == Some("Edge.Cuts");
  if !on_edge_cuts {
    return Ok(None);
  }
  let width = read_stroke_width(item)?;
  let uuid = read_uuid(item);
  let graphic = match item.tag() {
    Some("gr_line") => KicadGraphic::Line {
      start: read_point_child(item, "start")?,
      end: read_point_child(item, "end")?,
      width,
      uuid,
    },
    Some("gr_arc") => KicadGraphic::Arc {
      start: read_point_child(item, "start")?,
      mid: read_point_child(item, "mid")?,
      end: read_point_child(item, "end")?,
      width,
      uuid,
    },
    Some("gr_rect") => KicadGraphic::Rectangle {
      start: read_point_child(item, "start")?,
      end: read_point_child(item, "end")?,
      width,
      uuid,
    },
    Some("gr_poly") => KicadGraphic::Polygon {
      points: read_points(item.required_child("pts")?)?,
      width,
      uuid,
    },
    _ => return Ok(None),
  };
  Ok(Some(graphic))
}

/// The stroke width of a graphic item.
///
/// The newer files nest it as `(stroke (width w) (type solid))`, the older
/// ones write `(width w)` directly, and a pad primitive writes `(width 0)`
/// to mean an unstroked filled outline.
fn read_stroke_width(item: &Node) -> Result<i64, ParseError> {
  if let Some(stroke) = item.child("stroke") {
    return stroke.required_child("width")?.value_millimetres(0);
  }
  match item.child("width") {
    Some(node) => node.value_millimetres(0),
    None => Ok(0),
  }
}

/// Read a `(pts (xy x y) ...)` list.
fn read_points(pts: &Node) -> Result<Vec<Point>, ParseError> {
  let mut points = Vec::new();
  for entry in pts.values() {
    if entry.tag() != Some("xy") {
      return Err(ParseError::new(
        entry.line(),
        format!(
          "`{}` inside a point list is not supported",
          entry.tag().unwrap_or("<atom>")
        ),
      ));
    }
    points.push(read_point(entry)?);
  }
  Ok(points)
}

/// Read a `(zone ...)`, keeping it only when it is a rule area.
///
/// The same routine serves top level zones and zones nested inside a
/// footprint, because both store their outline in absolute board
/// coordinates.
fn read_zone(board: &mut KicadBoard, item: &Node) -> Result<(), ParseError> {
  let Some(keepout) = item.child("keepout") else {
    return Ok(());
  };
  let uuid = read_uuid(item);
  let layer_names = read_layer_name_list(item);
  let copper_layers = expand_copper_layers(board, &layer_names);
  let allowed = |tag: &str| {
    keepout
      .child(tag)
      .and_then(|node| node.values().first())
      .and_then(Node::as_str)
      != Some("not_allowed")
  };
  for polygon in item.children("polygon") {
    board.keepouts.push(KicadKeepout {
      uuid: uuid.clone(),
      layer_names: layer_names.clone(),
      copper_layers: copper_layers.clone(),
      outline: read_points(polygon.required_child("pts")?)?,
      tracks_allowed: allowed("tracks"),
      vias_allowed: allowed("vias"),
      pads_allowed: allowed("pads"),
      copper_pour_allowed: allowed("copperpour"),
      footprints_allowed: allowed("footprints"),
    });
  }
  Ok(())
}

/// The layer names of an item that may spell them `(layer "x")` or
/// `(layers "x" "y" ...)`.
fn read_layer_name_list(item: &Node) -> Vec<String> {
  let node = item.child("layers").or_else(|| item.child("layer"));
  node.map_or_else(Vec::new, |node| {
    node
      .values()
      .iter()
      .filter_map(Node::as_str)
      .map(str::to_string)
      .collect()
  })
}

/// Turn a list of layer names, wildcards included, into ascending copper
/// indices.
///
/// `*.Cu` means every copper layer and `F&B.Cu` means the two outer ones.
/// Names that are not copper, `F.Mask` and the like, drop out.
fn expand_copper_layers(board: &KicadBoard, names: &[String]) -> Vec<usize> {
  let mut indices: Vec<usize> = Vec::new();
  let push = |index: usize, indices: &mut Vec<usize>| {
    if !indices.contains(&index) {
      indices.push(index);
    }
  };
  for name in names {
    match name.as_str() {
      "*.Cu" => {
        for layer in &board.layers {
          if let Some(index) = layer.copper_index {
            push(index, &mut indices);
          }
        }
      }
      "F&B.Cu" => {
        for outer in ["F.Cu", "B.Cu"] {
          if let Some(index) = board.copper_index(outer) {
            push(index, &mut indices);
          }
        }
      }
      other => {
        if let Some(index) = board.copper_index(other) {
          push(index, &mut indices);
        }
      }
    }
  }
  indices.sort_unstable();
  indices
}

/// How a footprint's local frame maps onto the board.
#[derive(Debug, Clone, Copy)]
struct FootprintTransform {
  /// The footprint origin in absolute board coordinates.
  origin: Point,
  /// The footprint orientation in degrees, counter clockwise on screen.
  rotation_degrees: f64,
  /// Whether the local X axis is mirrored, from `(scale sx sy)`.
  mirror_x: bool,
  /// Whether the local Y axis is mirrored.
  mirror_y: bool,
}

impl FootprintTransform {
  /// Map a footprint local point to absolute board coordinates.
  fn apply(&self, local: Point) -> Point {
    let scaled = Point {
      x: if self.mirror_x { -local.x } else { local.x },
      y: if self.mirror_y { -local.y } else { local.y },
    };
    let rotated = rotate_point(scaled, self.rotation_degrees);
    Point {
      x: self.origin.x + rotated.x,
      y: self.origin.y + rotated.y,
    }
  }
}

/// Rotate a point about the origin, exactly as KiCad does.
///
/// A port of `RotatePoint( int*, int*, const EDA_ANGLE& )`
/// (`libs/kimath/src/trigo.cpp:225`): the four cardinal angles are exact
/// integer swaps, everything else goes through sine and cosine and
/// `KiROUND` (`libs/kimath/include/math/util.h:97`), which rounds halfway
/// cases away from zero. Rust's `f64::round` has the same tie rule as
/// C++'s `llround`, so the two agree.
///
/// The matrix is `(x cos + y sin, -x sin + y cos)`, which is a counter
/// clockwise rotation on a screen whose Y axis points down.
#[allow(
  clippy::float_cmp,
  clippy::cast_precision_loss,
  clippy::cast_possible_truncation
)]
fn rotate_point(point: Point, degrees: f64) -> Point {
  let normalized = degrees.rem_euclid(360.0);
  if normalized == 0.0 {
    point
  } else if normalized == 90.0 {
    Point {
      x: point.y,
      y: -point.x,
    }
  } else if normalized == 180.0 {
    Point {
      x: -point.x,
      y: -point.y,
    }
  } else if normalized == 270.0 {
    Point {
      x: -point.y,
      y: point.x,
    }
  } else {
    let radians = normalized.to_radians();
    let (sine, cosine) = radians.sin_cos();
    Point {
      x: ((point.y as f64) * sine + (point.x as f64) * cosine).round() as i64,
      y: ((point.y as f64) * cosine - (point.x as f64) * sine).round() as i64,
    }
  }
}

/// Read a footprint's transform from either spelling.
///
/// The classic spelling is `(at x y [rot])`. The 2026 boards
/// `drag-walk-optimize.kicad_pcb` and
/// `drag-walk-optimize-fix-corners.kicad_pcb` instead write
/// `(transform (translate x y) (rotate deg) (scale sx sy))`. Both of them
/// only ever use `(scale 1 1)`, so the mirroring branch below is not
/// exercised by the corpus; it is written the way KiCad's own transform
/// composes, scale first and rotation second.
fn read_footprint_transform(
  footprint: &Node,
) -> Result<FootprintTransform, ParseError> {
  if let Some(transform) = footprint.child("transform") {
    let translate = transform.required_child("translate")?;
    let rotation_degrees = match transform.child("rotate") {
      Some(node) => node.value_double(0)?,
      None => 0.0,
    };
    let (mirror_x, mirror_y) = match transform.child("scale") {
      Some(node) => (read_mirror_flag(node, 0)?, read_mirror_flag(node, 1)?),
      None => (false, false),
    };
    return Ok(FootprintTransform {
      origin: read_point(translate)?,
      rotation_degrees,
      mirror_x,
      mirror_y,
    });
  }

  let at = footprint.required_child("at")?;
  let rotation_degrees = if at.values().len() > 2 {
    at.value_double(2)?
  } else {
    0.0
  };
  Ok(FootprintTransform {
    origin: read_point(at)?,
    rotation_degrees,
    mirror_x: false,
    mirror_y: false,
  })
}

/// Read one component of a `(scale sx sy)`, which KiCad only ever writes
/// as plus or minus one.
#[allow(clippy::float_cmp)]
fn read_mirror_flag(node: &Node, index: usize) -> Result<bool, ParseError> {
  let value = node.value_double(index)?;
  if value == 1.0 {
    Ok(false)
  } else if value == -1.0 {
    Ok(true)
  } else {
    Err(ParseError::new(
      node.line(),
      format!("a footprint scale of {value} is not a plain mirror"),
    ))
  }
}

/// The reference designator of a footprint, in either spelling.
fn read_footprint_reference(footprint: &Node) -> String {
  for property in footprint.children("property") {
    if property.values().first().and_then(Node::as_str) == Some("Reference") {
      return property
        .values()
        .get(1)
        .and_then(Node::as_str)
        .unwrap_or_default()
        .to_string();
    }
  }
  for text in footprint.children("fp_text") {
    if text.values().first().and_then(Node::as_str) == Some("reference") {
      return text
        .values()
        .get(1)
        .and_then(Node::as_str)
        .unwrap_or_default()
        .to_string();
    }
  }
  String::new()
}

/// Read a `(footprint ...)`, adding its pads and its rule areas to the
/// board.
fn read_footprint(
  board: &mut KicadBoard,
  footprint: &Node,
) -> Result<(), ParseError> {
  let transform = read_footprint_transform(footprint)?;
  let footprint_uuid = read_uuid(footprint);
  let footprint_reference = read_footprint_reference(footprint);

  for item in footprint.values() {
    match item.tag() {
      Some("pad") => {
        let pad = read_pad(
          board,
          item,
          transform,
          &footprint_uuid,
          &footprint_reference,
        )?;
        board.pads.push(pad);
      }
      Some("zone") => read_zone(board, item)?,
      _ => {}
    }
  }
  Ok(())
}

/// Read a `(pad ...)`, resolving it into absolute board coordinates.
fn read_pad(
  board: &KicadBoard,
  item: &Node,
  transform: FootprintTransform,
  footprint_uuid: &str,
  footprint_reference: &str,
) -> Result<KicadPad, ParseError> {
  let number = item.value_str(0)?.to_string();
  let kind = match item.value_str(1)? {
    "smd" => PadKind::SurfaceMount,
    "thru_hole" => PadKind::ThroughHole,
    "np_thru_hole" => PadKind::NonPlatedThroughHole,
    "connect" => PadKind::EdgeConnector,
    other => {
      return Err(ParseError::new(
        item.line(),
        format!("`{other}` is not a pad type this reader knows"),
      ));
    }
  };
  let shape_token = item.value_str(2)?.to_string();

  let at = item.required_child("at")?;
  let local = read_point(at)?;
  let position = transform.apply(local);
  let rotation_degrees = if at.values().len() > 2 {
    at.value_double(2)?
  } else {
    0.0
  };

  let size_node = item.required_child("size")?;
  let size = read_point(size_node)?;

  let layer_names = read_layer_name_list(item);
  let copper_layers = expand_copper_layers(board, &layer_names);

  let shape = match shape_token.as_str() {
    "circle" => PadShape::Circle,
    "rect" => PadShape::Rectangle,
    "oval" => PadShape::Oval,
    "trapezoid" => PadShape::Trapezoid,
    "roundrect" => PadShape::RoundedRectangle {
      ratio: item.required_child("roundrect_rratio")?.value_double(0)?,
    },
    "custom" => PadShape::Custom {
      primitives: read_pad_primitives(item, position, rotation_degrees)?,
    },
    other => {
      return Err(ParseError::new(
        item.line(),
        format!("`{other}` is not a pad shape this reader knows"),
      ));
    }
  };

  let drill = match item.child("drill") {
    None => None,
    Some(node) => {
      if node.values().first().and_then(Node::as_str) == Some("oval") {
        Some(PadDrill::Oval {
          width: node.value_millimetres(1)?,
          height: node.value_millimetres(2)?,
        })
      } else {
        Some(PadDrill::Round {
          diameter: node.value_millimetres(0)?,
        })
      }
    }
  };

  Ok(KicadPad {
    footprint_uuid: footprint_uuid.to_string(),
    footprint_reference: footprint_reference.to_string(),
    number,
    kind,
    shape,
    at: position,
    size,
    rotation_degrees,
    layer_names,
    copper_layers,
    net: read_net(board, item)?,
    drill,
    uuid: read_uuid(item),
  })
}

/// Read the `(primitives ...)` of a custom pad into absolute polygons.
///
/// A primitive's points are stored relative to the pad position, in the
/// pad's own unrotated frame, so the absolute point is
/// `padPosition + RotatePoint(primitivePoint, padRotation)`. That was
/// checked against the corpus by the anchor: every custom pad in it uses
/// `(anchor rect)`, whose rectangle is centred on the pad position with
/// the pad size, and in all five custom pads the primitive outline is
/// flush with or encloses that rectangle only when the points are read as
/// pad relative. In `boards/stickhub-extra-via.kicad_pcb` for instance the
/// pad at local `(-0.975 0.05)` has size `1.5 1.5`, so its anchor spans
/// x from `-1.725` to `-0.225`, and its three primitive triangles span
/// exactly `-0.75` to `0.75` about the pad, that is the same interval.
///
/// The rotation used is the pad's own absolute angle, which is what KiCad
/// applies when it builds the effective shape. No custom pad in the corpus
/// has an angle that differs from its footprint's, so the corpus cannot
/// tell the two apart; KiCad's own code is the authority here.
fn read_pad_primitives(
  item: &Node,
  position: Point,
  rotation_degrees: f64,
) -> Result<Vec<Vec<Point>>, ParseError> {
  let Some(primitives) = item.child("primitives") else {
    return Ok(Vec::new());
  };
  let place = |local: Point| {
    let rotated = rotate_point(local, rotation_degrees);
    Point {
      x: position.x + rotated.x,
      y: position.y + rotated.y,
    }
  };

  let mut polygons = Vec::new();
  for primitive in primitives.values() {
    match primitive.tag() {
      Some("gr_poly") => {
        let points = read_points(primitive.required_child("pts")?)?;
        polygons.push(points.into_iter().map(place).collect());
      }
      Some("gr_rect") => {
        let start = read_point_child(primitive, "start")?;
        let end = read_point_child(primitive, "end")?;
        polygons.push(
          [
            Point {
              x: start.x,
              y: start.y,
            },
            Point {
              x: end.x,
              y: start.y,
            },
            Point { x: end.x, y: end.y },
            Point {
              x: start.x,
              y: end.y,
            },
          ]
          .into_iter()
          .map(place)
          .collect(),
        );
      }
      other => {
        return Err(ParseError::new(
          primitive.line(),
          format!(
            "`{}` is not a pad primitive this reader knows",
            other.unwrap_or("<atom>")
          ),
        ));
      }
    }
  }
  Ok(polygons)
}
