// SPDX-License-Identifier: GPL-3.0-or-later

//! Readers for a PNS regression case: the `.log` recording and the
//! `.settings` sidecar.
//!
//! # What a log is
//!
//! `LOGGER` (`pcbnew/router/pns_logger.h:48`) records the user's input
//! events and nothing else. No geometry is logged: replay reconstructs it
//! by running the real router over a board that is identified separately.
//! On disk the file is pretty printed JSON written by
//! `LOGGER::FormatLogFileAsJSON` (`pcbnew/router/pns_logger.cpp:107`) with
//! six top level keys, `mode`, `events`, `removedItems`, `addedItems`,
//! `headItems` and the two optional ones `test_case_type` and
//! `board_hash`.
//!
//! Two of those are decorative. `LOGGER::ParseEventFromJSON`
//! (`pcbnew/router/pns_logger.cpp:297`) reads back position, type, layer
//! and uuids but never the per event `sizes` block, and
//! `PNS_LOG_FILE::loadJsonLog` (`qa/tools/pns/pns_log_file.cpp:582`) never
//! reads `headItems`. Both are still parsed here, because a reader that
//! silently drops half a file is a reader you cannot use to check the
//! file.
//!
//! `addedItems` and `removedItems` are the golden result. KiCad asserts
//! set equality on the removed KIIDs and multiset equality on the added
//! items after a dedup pass (`PNS_LOG_FILE::COMMIT_STATE::Compare`,
//! `qa/tools/pns/pns_log_file.cpp:411`), which is all a replay is checked
//! against: no DRC run, no end state check.
//!
//! # The legacy text format
//!
//! `PNS_LOG_FILE::Load` falls back to a whitespace delimited grammar when
//! the JSON parse fails (`qa/tools/pns/pns_log_file.cpp:571`), reading
//! lines that start with `mode`, `event`, `added` or `removed` and
//! ignoring everything else, comment lines included.
//! [`read_legacy_log`] implements it for completeness and because it is
//! the only spelling of the event stream that is legible by eye. No file
//! in the corpus uses it: all eleven are JSON.
//!
//! # Board identity
//!
//! A log names its board by content hash, not by file name. The producer
//! stamps `board_hash` with `IO_UTILS::fileHashMMH3` over the dumped board
//! (`pcbnew/router/router_tool.cpp:894`) and the harness hashes every
//! `boards/*.kicad_pcb` at startup to match
//! (`qa/tools/pns/qa_pns_regressions_main.cpp:166`). The hash is carried
//! through as an opaque string here: the hash routine lives outside the
//! part of KiCad checked out as a reference, so a case cannot be resolved
//! to its board without it. `walk-with-teardrops` is the one case that
//! needs no lookup, shipping its own board as a `.dump` beside the log.

use std::io;
use std::path::{Path, PathBuf};

use super::json::{self, JsonError, JsonValue};
use super::kicad_pcb::Point;
use super::sexpr::ParseError;

/// What went wrong while reading a log or a settings file, and where.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LogError {
  /// One based line number the problem was found on, 0 when it has no
  /// line.
  pub line: u32,
  /// Human readable description of the problem.
  pub message: String,
}

impl LogError {
  /// Build an error reported against `line`.
  pub fn new(line: u32, message: impl Into<String>) -> Self {
    Self {
      line,
      message: message.into(),
    }
  }
}

impl std::fmt::Display for LogError {
  fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(formatter, "line {}: {}", self.line, self.message)
  }
}

impl std::error::Error for LogError {}

impl From<JsonError> for LogError {
  fn from(error: JsonError) -> Self {
    Self {
      line: error.line,
      message: error.message,
    }
  }
}

impl From<ParseError> for LogError {
  fn from(error: ParseError) -> Self {
    Self {
      line: error.line,
      message: error.message,
    }
  }
}

/// One recorded user input event.
///
/// `EVENT_TYPE` (`pcbnew/router/pns_logger.h:60`). `Abort` is declared in
/// KiCad and emitted by nothing in the tree.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
  /// Begin routing a new track from a point.
  StartRoute,
  /// Begin dragging one existing item.
  StartDrag,
  /// Commit the route so far up to a point.
  Fix,
  /// Move the cursor.
  Move,
  /// Declared but never emitted.
  Abort,
  /// Toggle the pending via on the head.
  ToggleVia,
  /// Undo the last fixed segment, that is backspace during routing.
  Unfix,
  /// Begin dragging several items at once.
  StartMultiDrag,
}

impl EventKind {
  /// Map the integer written into the log.
  pub fn from_code(code: i64, line: u32) -> Result<Self, LogError> {
    match code {
      0 => Ok(EventKind::StartRoute),
      1 => Ok(EventKind::StartDrag),
      2 => Ok(EventKind::Fix),
      3 => Ok(EventKind::Move),
      4 => Ok(EventKind::Abort),
      5 => Ok(EventKind::ToggleVia),
      6 => Ok(EventKind::Unfix),
      7 => Ok(EventKind::StartMultiDrag),
      other => {
        Err(LogError::new(line, format!("{other} is not an event type")))
      }
    }
  }
}

/// Which router the session was driving.
///
/// `ROUTER_MODE` (`pcbnew/router/pns_router.h:67`). Not to be confused
/// with the `mode` of a `.settings` file, which is [`RoutingMode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouterMode {
  /// Single track routing.
  RouteSingle,
  /// Differential pair routing.
  RouteDiffPair,
  /// Length tuning of a single track.
  TuneSingle,
  /// Length tuning of a differential pair.
  TuneDiffPair,
  /// Skew tuning of a differential pair.
  TuneDiffPairSkew,
}

impl RouterMode {
  /// Map the integer written into the log.
  pub fn from_code(code: i64, line: u32) -> Result<Self, LogError> {
    match code {
      1 => Ok(RouterMode::RouteSingle),
      2 => Ok(RouterMode::RouteDiffPair),
      3 => Ok(RouterMode::TuneSingle),
      4 => Ok(RouterMode::TuneDiffPair),
      5 => Ok(RouterMode::TuneDiffPairSkew),
      other => {
        Err(LogError::new(line, format!("{other} is not a router mode")))
      }
    }
  }
}

/// How strictly a case is meant to be judged.
///
/// `TEST_CASE_TYPE` (`pcbnew/router/pns_logger.h:52`). KiCad round trips
/// the field and then never acts on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestCaseType {
  /// The committed geometry must match exactly.
  StrictGeometry,
  /// Only the connectivity of the result matters.
  ConnectivityOnly,
  /// The case is expected to fail.
  ExpectedFail,
  /// The case reproduces a bug that is still open.
  KnownBug,
}

impl TestCaseType {
  /// Map the integer written into the log.
  pub fn from_code(code: i64, line: u32) -> Result<Self, LogError> {
    match code {
      0 => Ok(TestCaseType::StrictGeometry),
      1 => Ok(TestCaseType::ConnectivityOnly),
      2 => Ok(TestCaseType::ExpectedFail),
      3 => Ok(TestCaseType::KnownBug),
      other => Err(LogError::new(
        line,
        format!("{other} is not a test case type"),
      )),
    }
  }
}

/// The track and via sizes recorded alongside an event.
///
/// `LOGGER::formatSizesAsJSON` (`pcbnew/router/pns_logger.cpp:217`). Only
/// the three session starting events and `ToggleVia` are ever handed a
/// real `SIZES_SETTINGS`; every other event carries the constructor
/// defaults. KiCad never reads the block back, recomputing the sizes from
/// the board through `ImportSizes` instead
/// (`qa/tools/pns/pns_log_player.cpp:135`), so treat it as a hint and not
/// as input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogSizes {
  /// Track width in nanometres.
  pub track_width: i64,
  /// Via copper diameter in nanometres.
  pub via_diameter: i64,
  /// Via hole diameter in nanometres.
  pub via_drill: i64,
  /// Whether the user pinned the track width rather than inheriting it.
  pub track_width_is_explicit: bool,
  /// PNS layer index the via span ends on.
  pub layer_bottom: i64,
  /// PNS layer index the via span starts on.
  pub layer_top: i64,
  /// `VIATYPE` of the pending via.
  pub via_type: i64,
}

/// One entry of the event stream.
///
/// `EVENT_ENTRY` (`pcbnew/router/pns_logger.h:71`).
#[derive(Debug, Clone)]
pub struct LogEvent {
  /// What happened.
  pub kind: EventKind,
  /// Where the cursor was, in nanometres. Zero for `ToggleVia` and
  /// `Unfix`, which record no position.
  pub position: Point,
  /// The PNS layer index the event applies to. Replay only falls back to
  /// it when the event's item does not resolve
  /// (`qa/tools/pns/pns_log_player.cpp:118`).
  pub layer: i64,
  /// KIIDs of the board items the event refers to. `LOGGER::LogM`
  /// (`pcbnew/router/pns_logger.cpp:75`) records only items that have a
  /// parent board item, so the list can be empty: `simple-shove-1` starts
  /// routing in free space and its first event has none.
  pub uuids: Vec<String>,
  /// The decorative sizes block, absent in the legacy format.
  pub sizes: Option<LogSizes>,
}

/// A shape as the logger writes it.
///
/// `LOGGER::formatShapeAsJSON` (`pcbnew/router/pns_logger.cpp:231`) emits
/// exactly these three and a JSON `null` for anything else.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LogShape {
  /// A thick line.
  Segment {
    /// Track width.
    width: i64,
    /// One end.
    start: Point,
    /// The other end.
    end: Point,
  },
  /// A thick circular arc through three points.
  Arc {
    /// Track width.
    width: i64,
    /// One end.
    start: Point,
    /// A point on the arc between the ends.
    mid: Point,
    /// The other end.
    end: Point,
  },
  /// A disc, which is how a via and a hole are logged.
  Circle {
    /// Radius, so half a via diameter.
    radius: i64,
    /// Centre.
    center: Point,
  },
}

/// A router item in the golden result.
///
/// `LOGGER::formatRouterItemAsJSON` (`pcbnew/router/pns_logger.cpp:173`).
/// The net is written by **name**, not by netcode, which is what makes the
/// golden portable to another implementation: nothing in it depends on the
/// board's internal numbering.
#[derive(Debug, Clone)]
pub struct LogItem {
  /// `ITEM::KindStr`, so `segment`, `arc`, `via`, `hole`, `line` and so
  /// on.
  pub kind: String,
  /// The net name.
  pub net: String,
  /// PNS layer index the item starts on.
  pub layer_start: i64,
  /// PNS layer index the item ends on.
  pub layer_end: i64,
  /// The geometry, absent for kinds the logger writes no shape for, such
  /// as the `line` items in `headItems`.
  pub shape: Option<LogShape>,
  /// Hole diameter, present on a via.
  pub drill: Option<i64>,
}

/// A whole `.log` file.
///
/// `LOG_DATA` (`pcbnew/router/pns_logger.h:94`).
#[derive(Debug, Clone)]
pub struct LogFile {
  /// Which router the session drove.
  pub router_mode: RouterMode,
  /// How strictly the case is meant to be judged, when recorded.
  pub test_case_type: Option<TestCaseType>,
  /// Content hash of the board the session ran on, when recorded. Opaque
  /// here, see the module documentation.
  pub board_hash: Option<String>,
  /// The input events, in order.
  pub events: Vec<LogEvent>,
  /// KIIDs of the board items the session removed. Half of the golden.
  pub removed_items: Vec<String>,
  /// The items the session added. The other half of the golden.
  pub added_items: Vec<LogItem>,
  /// The head being routed when the session ended. Recorded and never
  /// read back by KiCad.
  pub head_items: Vec<LogItem>,
}

/// Read a `.log` file, JSON first and the legacy grammar as a fallback,
/// which is the order `PNS_LOG_FILE::Load` uses
/// (`qa/tools/pns/pns_log_file.cpp:571`).
pub fn read_log(text: &str) -> Result<LogFile, LogError> {
  match read_json_log(text) {
    Ok(log) => Ok(log),
    Err(json_error) => read_legacy_log(text).map_err(|legacy_error| {
      LogError::new(
        json_error.line,
        format!(
          "not JSON ({}) and not the legacy format ({})",
          json_error.message, legacy_error.message
        ),
      )
    }),
  }
}

/// Read the JSON spelling of a `.log` file.
pub fn read_json_log(text: &str) -> Result<LogFile, LogError> {
  let document = json::parse(text)?;
  let mode_value = document.required_member("mode")?;
  let router_mode =
    RouterMode::from_code(mode_value.integer()?, mode_value.line())?;

  let test_case_type = match document.member("test_case_type") {
    None => None,
    Some(value) => {
      Some(TestCaseType::from_code(value.integer()?, value.line())?)
    }
  };

  let board_hash = match document.member("board_hash") {
    None => None,
    Some(value) => Some(value.text()?.to_string()),
  };

  let mut events = Vec::new();
  for entry in document.required_member("events")?.array()? {
    events.push(read_json_event(entry)?);
  }

  let mut removed_items = Vec::new();
  for entry in document.required_member("removedItems")?.array()? {
    removed_items.push(entry.text()?.to_string());
  }

  let mut added_items = Vec::new();
  for entry in document.required_member("addedItems")?.array()? {
    added_items.push(read_json_item(entry)?);
  }

  let mut head_items = Vec::new();
  for entry in document.required_member("headItems")?.array()? {
    head_items.push(read_json_item(entry)?);
  }

  Ok(LogFile {
    router_mode,
    test_case_type,
    board_hash,
    events,
    removed_items,
    added_items,
    head_items,
  })
}

/// Read a `{"x": .., "y": ..}` pair, KiCad's `VECTOR2I`
/// (`pcbnew/router/pns_logger.cpp:40`). The values are already board
/// internal units, that is nanometres.
fn read_json_point(value: &JsonValue) -> Result<Point, LogError> {
  Ok(Point {
    x: value.required_member("x")?.integer()?,
    y: value.required_member("y")?.integer()?,
  })
}

/// Read one entry of the `events` array.
fn read_json_event(value: &JsonValue) -> Result<LogEvent, LogError> {
  let type_value = value.required_member("type")?;
  let mut uuids = Vec::new();
  for entry in value.required_member("uuids")?.array()? {
    uuids.push(entry.text()?.to_string());
  }
  let sizes = match value.member("sizes") {
    None => None,
    Some(block) => Some(LogSizes {
      track_width: block.required_member("trackWidth")?.integer()?,
      via_diameter: block.required_member("viaDiameter")?.integer()?,
      via_drill: block.required_member("viaDrill")?.integer()?,
      track_width_is_explicit: block
        .required_member("trackWidthIsExplicit")?
        .boolean()?,
      layer_bottom: block.required_member("layerBottom")?.integer()?,
      layer_top: block.required_member("layerTop")?.integer()?,
      via_type: block.required_member("viaType")?.integer()?,
    }),
  };
  Ok(LogEvent {
    kind: EventKind::from_code(type_value.integer()?, type_value.line())?,
    position: read_json_point(value.required_member("position")?)?,
    layer: value.required_member("layer")?.integer()?,
    uuids,
    sizes,
  })
}

/// Read one entry of `addedItems` or `headItems`.
fn read_json_item(value: &JsonValue) -> Result<LogItem, LogError> {
  let layers = value.required_member("layers")?.array()?;
  if layers.len() != 2 {
    return Err(LogError::new(
      value.line(),
      "an item's `layers` is not a pair",
    ));
  }
  let shape = match value.member("shape") {
    None | Some(JsonValue::Null { .. }) => None,
    Some(block) => Some(read_json_shape(block)?),
  };
  let drill = match value.member("drill") {
    None => None,
    Some(block) => Some(block.integer()?),
  };
  Ok(LogItem {
    kind: value.required_member("kind")?.text()?.to_string(),
    net: value.required_member("net")?.text()?.to_string(),
    layer_start: layers[0].integer()?,
    layer_end: layers[1].integer()?,
    shape,
    drill,
  })
}

/// Read an item's `shape` member.
fn read_json_shape(value: &JsonValue) -> Result<LogShape, LogError> {
  let type_value = value.required_member("type")?;
  match type_value.text()? {
    "segment" => Ok(LogShape::Segment {
      width: value.required_member("width")?.integer()?,
      start: read_json_point(value.required_member("start")?)?,
      end: read_json_point(value.required_member("end")?)?,
    }),
    "arc" => Ok(LogShape::Arc {
      width: value.required_member("width")?.integer()?,
      start: read_json_point(value.required_member("start")?)?,
      mid: read_json_point(value.required_member("mid")?)?,
      end: read_json_point(value.required_member("end")?)?,
    }),
    "circle" => Ok(LogShape::Circle {
      radius: value.required_member("radius")?.integer()?,
      center: read_json_point(value.required_member("center")?)?,
    }),
    other => Err(LogError::new(
      type_value.line(),
      format!("`{other}` is not a logged shape"),
    )),
  }
}

/// Read the legacy whitespace delimited spelling of a `.log` file.
///
/// The grammar is four line kinds, everything else ignored
/// (`qa/tools/pns/pns_log_file.cpp:662`):
///
/// ```text
/// mode <router mode>
/// event <x> <y> <type> <layer> <uuid count> <uuid>...
/// added segment net <netcode> layers <a> <b> shape 4 <ax> <ay> <bx> <by> <width>
/// added via net <netcode> layers <a> <b> shape 2 <cx> <cy> <radius> drill <d>
/// removed <uuid>
/// ```
///
/// The event line shape is `LOGGER::ParseEvent`
/// (`pcbnew/router/pns_logger.cpp:274`), the item lines are
/// `PNS_LOG_FILE::parseLegacyItemFromString`
/// (`qa/tools/pns/pns_log_file.cpp:298`) and the shape numbers are
/// `SHAPE_TYPE`, 2 for a circle and 4 for a segment.
///
/// Two things differ from the JSON path and both matter if a corpus in
/// this format ever turns up. The legacy item names its net by **netcode**
/// where the JSON names it by name, so a legacy net cannot be resolved
/// without the board it was recorded against; the netcode is put into
/// [`LogItem::net`] as decimal text to keep that visible. And the event
/// line carries no sizes block at all, so [`LogEvent::sizes`] is `None`.
pub fn read_legacy_log(text: &str) -> Result<LogFile, LogError> {
  let mut router_mode = RouterMode::RouteSingle;
  let mut saw_mode = false;
  let mut events = Vec::new();
  let mut removed_items = Vec::new();
  let mut added_items = Vec::new();

  for (offset, raw_line) in text.lines().enumerate() {
    let line = u32::try_from(offset + 1).unwrap_or(u32::MAX);
    let mut tokens = raw_line.split_ascii_whitespace();
    let Some(command) = tokens.next() else {
      continue;
    };
    match command {
      "mode" => {
        router_mode =
          RouterMode::from_code(next_integer(&mut tokens, line)?, line)?;
        saw_mode = true;
      }
      "event" => events.push(read_legacy_event(&mut tokens, line)?),
      "removed" => removed_items.push(
        tokens
          .next()
          .ok_or_else(|| LogError::new(line, "`removed` without a uuid"))?
          .to_string(),
      ),
      "added" => {
        if let Some(item) = read_legacy_item(&mut tokens, line)? {
          added_items.push(item);
        }
      }
      _ => {}
    }
  }

  if !saw_mode {
    return Err(LogError::new(1, "no `mode` line"));
  }

  Ok(LogFile {
    router_mode,
    test_case_type: None,
    board_hash: None,
    events,
    removed_items,
    added_items,
    head_items: Vec::new(),
  })
}

/// Take the next whitespace delimited token as an integer.
fn next_integer<'a>(
  tokens: &mut impl Iterator<Item = &'a str>,
  line: u32,
) -> Result<i64, LogError> {
  let token = tokens
    .next()
    .ok_or_else(|| LogError::new(line, "the line ends too early"))?;
  token
    .parse()
    .map_err(|_| LogError::new(line, format!("`{token}` is not an integer")))
}

/// Read the tail of a legacy `event` line, the `event` token consumed.
fn read_legacy_event<'a>(
  tokens: &mut impl Iterator<Item = &'a str>,
  line: u32,
) -> Result<LogEvent, LogError> {
  let position = Point {
    x: next_integer(tokens, line)?,
    y: next_integer(tokens, line)?,
  };
  let kind = EventKind::from_code(next_integer(tokens, line)?, line)?;
  let layer = next_integer(tokens, line)?;
  let count = next_integer(tokens, line)?;
  let mut uuids = Vec::new();
  for _ in 0..count {
    uuids.push(
      tokens
        .next()
        .ok_or_else(|| LogError::new(line, "fewer uuids than promised"))?
        .to_string(),
    );
  }
  Ok(LogEvent {
    kind,
    position,
    layer,
    uuids,
    sizes: None,
  })
}

/// Read the tail of a legacy `added` line, the `added` token consumed.
///
/// Returns `None` for an item kind the legacy grammar cannot express,
/// which is every kind but `segment` and `via`.
fn read_legacy_item<'a>(
  tokens: &mut impl Iterator<Item = &'a str>,
  line: u32,
) -> Result<Option<LogItem>, LogError> {
  let Some(kind) = tokens.next() else {
    return Err(LogError::new(line, "`added` without an item kind"));
  };
  if kind != "segment" && kind != "via" {
    return Ok(None);
  }

  let mut item = LogItem {
    kind: kind.to_string(),
    net: String::new(),
    layer_start: 0,
    layer_end: 0,
    shape: None,
    drill: None,
  };

  while let Some(property) = tokens.next() {
    match property {
      "net" => item.net = next_integer(tokens, line)?.to_string(),
      "layers" => {
        item.layer_start = next_integer(tokens, line)?;
        item.layer_end = next_integer(tokens, line)?;
      }
      "drill" => item.drill = Some(next_integer(tokens, line)?),
      "shape" => {
        let shape_type = next_integer(tokens, line)?;
        item.shape = Some(match shape_type {
          2 => LogShape::Circle {
            center: Point {
              x: next_integer(tokens, line)?,
              y: next_integer(tokens, line)?,
            },
            radius: next_integer(tokens, line)?,
          },
          4 => {
            let start = Point {
              x: next_integer(tokens, line)?,
              y: next_integer(tokens, line)?,
            };
            let end = Point {
              x: next_integer(tokens, line)?,
              y: next_integer(tokens, line)?,
            };
            LogShape::Segment {
              start,
              end,
              width: next_integer(tokens, line)?,
            }
          }
          other => {
            return Err(LogError::new(
              line,
              format!("{other} is not a legacy shape type"),
            ));
          }
        });
      }
      _ => {}
    }
  }

  Ok(Some(item))
}

/// Which algorithm the router runs.
///
/// `PNS_MODE` (`pcbnew/router/pns_routing_settings.h:39`). Distinct from
/// the [`RouterMode`] of a log despite both keys being spelled `mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoutingMode {
  /// Ignore collisions and mark the obstacles.
  MarkObstacles,
  /// Shove obstacles out of the way.
  Shove,
  /// Walk around obstacles.
  Walkaround,
}

/// How hard the optimiser tries.
///
/// `PNS_OPTIMIZATION_EFFORT` (`pcbnew/router/pns_routing_settings.h:47`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OptimizationEffort {
  /// Cheapest.
  Low,
  /// The default.
  Medium,
  /// Everything the optimiser has.
  Full,
}

/// How corners are drawn.
///
/// `DIRECTION_45::CORNER_MODE`
/// (`libs/kimath/include/geometry/direction45.h:66`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CornerMode {
  /// Horizontal, vertical and 45 degrees with mitred corners.
  Mitered45,
  /// The same with filleted corners.
  Rounded45,
  /// Horizontal and vertical only.
  Mitered90,
  /// Horizontal and vertical with filleted corners.
  Rounded90,
}

/// A `.settings` file, the flat JSON of `PNS::ROUTING_SETTINGS`.
///
/// The keys and their defaults are the parameter list built in
/// `PNS::ROUTING_SETTINGS::ROUTING_SETTINGS`
/// (`pcbnew/router/pns_routing_settings.cpp:60` to `:107`). A missing key
/// falls back to the default, matching KiCad, whose loader also only warns
/// when the whole file fails to load
/// (`qa/tools/pns/pns_log_file.cpp:513`).
#[derive(Debug, Clone, PartialEq)]
pub struct LogSettings {
  /// Which algorithm to run.
  pub mode: RoutingMode,
  /// How hard the optimiser tries.
  pub effort: OptimizationEffort,
  /// Remove a loop when a route closes one.
  pub remove_loops: bool,
  /// Route to the centre of a pad rather than to where the cursor is.
  pub smart_pads: bool,
  /// Allow shoving vias, not only tracks.
  pub shove_vias: bool,
  /// Offer to finish the route to the target.
  pub suggest_finish: bool,
  /// Track the mouse rather than snapping to the last committed point.
  pub follow_mouse: bool,
  /// Start a route on the diagonal rather than on the axis.
  pub start_diagonal: bool,
  /// Iteration budget for the shover.
  pub shove_iteration_limit: i64,
  /// Iteration budget for propagating a via push.
  pub via_force_prop_iteration_limit: i64,
  /// Time budget for the shover in milliseconds. Not used by a replay,
  /// which has no clock.
  pub shove_time_limit: i64,
  /// Iteration budget for the walkaround.
  pub walkaround_iteration_limit: i64,
  /// Let a route jump over an obstacle instead of going around it.
  pub jump_over_obstacles: bool,
  /// Smooth a dragged segment after the drag.
  pub smooth_dragged_segments: bool,
  /// Let the user commit a route that violates the design rules.
  pub can_violate_drc: bool,
  /// Route at any angle rather than on the 45 degree grid.
  pub free_angle_mode: bool,
  /// Snap the cursor to nearby tracks.
  pub snap_to_tracks: bool,
  /// Snap the cursor to nearby pads.
  pub snap_to_pads: bool,
  /// Optimise the whole dragged track, not only the dragged part.
  pub optimize_dragged_track: bool,
  /// Pick the starting posture automatically.
  pub auto_posture: bool,
  /// Commit every segment of the head on a fix, not only the first.
  pub fix_all_segments: bool,
  /// Refuse to produce angles the corner mode does not allow.
  pub restrict_angles: bool,
  /// How corners are drawn.
  pub corner_mode: CornerMode,
  /// How far, as a multiple of the obstacle size, hugging is preferred
  /// over walking around.
  pub walkaround_hug_length_threshold: f64,
  /// Keys the file carries that the current KiCad no longer defines, such
  /// as `pad_pushout`, in file order. Recorded rather than rejected: an
  /// old settings file is still a valid input, and a key that quietly
  /// vanished is worth being able to see.
  pub unrecognised_keys: Vec<String>,
}

impl Default for LogSettings {
  /// The defaults of `PNS::ROUTING_SETTINGS`
  /// (`pcbnew/router/pns_routing_settings.cpp:60`).
  fn default() -> Self {
    Self {
      mode: RoutingMode::Walkaround,
      effort: OptimizationEffort::Medium,
      remove_loops: true,
      smart_pads: true,
      shove_vias: true,
      suggest_finish: false,
      follow_mouse: true,
      start_diagonal: false,
      shove_iteration_limit: 250,
      via_force_prop_iteration_limit: 40,
      shove_time_limit: 1000,
      walkaround_iteration_limit: 40,
      jump_over_obstacles: false,
      smooth_dragged_segments: true,
      can_violate_drc: false,
      free_angle_mode: false,
      snap_to_tracks: false,
      snap_to_pads: false,
      optimize_dragged_track: false,
      auto_posture: true,
      fix_all_segments: true,
      restrict_angles: false,
      corner_mode: CornerMode::Mitered45,
      walkaround_hug_length_threshold: 1.5,
      unrecognised_keys: Vec::new(),
    }
  }
}

/// Read a `.settings` file.
pub fn read_settings(text: &str) -> Result<LogSettings, LogError> {
  let document = json::parse(text)?;
  let JsonValue::Object { members, .. } = &document else {
    return Err(LogError::new(document.line(), "settings are not an object"));
  };

  let mut settings = LogSettings::default();
  for (name, value) in members {
    match name.as_str() {
      // `meta` is the settings framework's own version stamp, not a
      // router parameter.
      "meta" => {}
      "mode" => {
        settings.mode = match value.integer()? {
          0 => RoutingMode::MarkObstacles,
          1 => RoutingMode::Shove,
          2 => RoutingMode::Walkaround,
          other => {
            return Err(LogError::new(
              value.line(),
              format!("{other} is not a routing mode"),
            ));
          }
        }
      }
      "effort" => {
        settings.effort = match value.integer()? {
          0 => OptimizationEffort::Low,
          1 => OptimizationEffort::Medium,
          2 => OptimizationEffort::Full,
          other => {
            return Err(LogError::new(
              value.line(),
              format!("{other} is not an optimisation effort"),
            ));
          }
        }
      }
      "corner_mode" => {
        settings.corner_mode = match value.integer()? {
          0 => CornerMode::Mitered45,
          1 => CornerMode::Rounded45,
          2 => CornerMode::Mitered90,
          3 => CornerMode::Rounded90,
          other => {
            return Err(LogError::new(
              value.line(),
              format!("{other} is not a corner mode"),
            ));
          }
        }
      }
      "remove_loops" => settings.remove_loops = value.boolean()?,
      "smart_pads" => settings.smart_pads = value.boolean()?,
      "shove_vias" => settings.shove_vias = value.boolean()?,
      "suggest_finish" => settings.suggest_finish = value.boolean()?,
      "follow_mouse" => settings.follow_mouse = value.boolean()?,
      "start_diagonal" => settings.start_diagonal = value.boolean()?,
      "jump_over_obstacles" => {
        settings.jump_over_obstacles = value.boolean()?
      }
      "smooth_dragged_segments" => {
        settings.smooth_dragged_segments = value.boolean()?;
      }
      "can_violate_drc" => settings.can_violate_drc = value.boolean()?,
      "free_angle_mode" => settings.free_angle_mode = value.boolean()?,
      "snap_to_tracks" => settings.snap_to_tracks = value.boolean()?,
      "snap_to_pads" => settings.snap_to_pads = value.boolean()?,
      "optimize_dragged_track" => {
        settings.optimize_dragged_track = value.boolean()?;
      }
      "auto_posture" => settings.auto_posture = value.boolean()?,
      "fix_all_segments" => settings.fix_all_segments = value.boolean()?,
      "restrict_angles" => settings.restrict_angles = value.boolean()?,
      "shove_iteration_limit" => {
        settings.shove_iteration_limit = value.integer()?;
      }
      "via_force_prop_iteration_limit" => {
        settings.via_force_prop_iteration_limit = value.integer()?;
      }
      "shove_time_limit" => settings.shove_time_limit = value.integer()?,
      "walkaround_iteration_limit" => {
        settings.walkaround_iteration_limit = value.integer()?;
      }
      "walkaround_hug_length_threshold" => {
        settings.walkaround_hug_length_threshold = value.double()?;
      }
      other => settings.unrecognised_keys.push(other.to_string()),
    }
  }
  Ok(settings)
}

/// One regression case on disk.
///
/// The file naming is not uniform. `PNS_LOG_FILE::Load`
/// (`qa/tools/pns/pns_log_file.cpp:484`) derives every sidecar from the
/// log path by substituting the extension, so a case is whatever base name
/// its `.log` happens to have. Eight of the eleven use `pns`, two use the
/// case directory's own name and one uses `pns-no-hug-2`.
#[derive(Debug, Clone)]
pub struct RegressionCase {
  /// The case directory's name, which is also the name KiCad registers
  /// the Boost test under
  /// (`qa/tools/pns/qa_pns_regressions_main.cpp:210`).
  pub name: String,
  /// The `.log` itself.
  pub log_path: PathBuf,
  /// The `.settings` sidecar, when the case ships one.
  pub settings_path: Option<PathBuf>,
  /// The `.dump` board, present only for a case that carries its own
  /// board instead of naming one from the pool by hash.
  pub board_dump_path: Option<PathBuf>,
  /// The `.kicad_dru` custom design rules, when the case ships any.
  pub design_rules_path: Option<PathBuf>,
}

/// Find every regression case under `root`.
///
/// Mirrors `createTestCases`
/// (`qa/tools/pns/qa_pns_regressions_main.cpp:181`): every sub directory
/// except `boards` is a case, and every `*.log` in it is a test. The
/// result is sorted by directory name and then by log file name so that
/// the order does not depend on the file system.
pub fn discover_cases(root: &Path) -> io::Result<Vec<RegressionCase>> {
  let mut directories = Vec::new();
  for entry in std::fs::read_dir(root)? {
    let entry = entry?;
    if !entry.file_type()?.is_dir() {
      continue;
    }
    let name = entry.file_name().to_string_lossy().into_owned();
    if name == "boards" {
      continue;
    }
    directories.push((name, entry.path()));
  }
  directories.sort_by(|left, right| left.0.cmp(&right.0));

  let mut cases = Vec::new();
  for (name, directory) in directories {
    let mut logs = Vec::new();
    for entry in std::fs::read_dir(&directory)? {
      let path = entry?.path();
      if path.extension().is_some_and(|extension| extension == "log") {
        logs.push(path);
      }
    }
    logs.sort();
    for log_path in logs {
      let sidecar = |extension: &str| {
        let candidate = log_path.with_extension(extension);
        candidate.exists().then_some(candidate)
      };
      let settings_path = sidecar("settings");
      let board_dump_path = sidecar("dump");
      let design_rules_path = sidecar("kicad_dru");
      cases.push(RegressionCase {
        name: name.clone(),
        settings_path,
        board_dump_path,
        design_rules_path,
        log_path,
      });
    }
  }
  Ok(cases)
}
