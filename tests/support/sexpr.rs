// SPDX-License-Identifier: GPL-3.0-or-later

//! A reader for the s-expression dialect KiCad writes into `.kicad_pcb`
//! files.
//!
//! The dialect is small. A file is one parenthesised list; an element is a
//! nested list, a double quoted string, or a bare symbol; whitespace
//! separates elements. Numbers are not a token kind of their own, they
//! arrive as symbols. That is deliberate: it lets
//! [`millimetres_to_nanometres`] read the decimal text digit by digit
//! instead of routing a board coordinate through `f64` and back.
//!
//! Every error carries the one based line the offending token starts on,
//! so a fixture that changes shape points at itself.

use std::fmt;

/// Nanometres in one millimetre. KiCad's board internal unit is the
/// nanometre and its files are written in millimetres.
const NANOMETRES_PER_MILLIMETRE: i64 = 1_000_000;

/// Decimal places a millimetre value may use before it stops being
/// representable in whole nanometres.
const MILLIMETRE_FRACTION_DIGITS: usize = 6;

/// What went wrong while reading, and where.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError {
  /// One based line number the offending token starts on.
  pub line: u32,
  /// Human readable description of the problem.
  pub message: String,
}

impl ParseError {
  /// Build an error reported against `line`.
  pub fn new(line: u32, message: impl Into<String>) -> Self {
    Self {
      line,
      message: message.into(),
    }
  }
}

impl fmt::Display for ParseError {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(formatter, "line {}: {}", self.line, self.message)
  }
}

impl std::error::Error for ParseError {}

/// One node of a parsed s-expression.
#[derive(Debug, Clone)]
pub enum Node {
  /// A bare token: a tag such as `segment`, a number such as `-1.27`, or
  /// a keyword such as `yes`.
  Symbol {
    /// The token text exactly as it appears in the file.
    text: String,
    /// One based line the token starts on.
    line: u32,
  },
  /// A double quoted string with its escape sequences resolved.
  Text {
    /// The string contents after unescaping.
    text: String,
    /// One based line the token starts on.
    line: u32,
  },
  /// A parenthesised list. By convention its first element is the tag.
  List {
    /// The elements, tag included.
    items: Vec<Node>,
    /// One based line the opening parenthesis is on.
    line: u32,
  },
}

impl Node {
  /// The one based line this node starts on.
  pub fn line(&self) -> u32 {
    match self {
      Node::Symbol { line, .. } => *line,
      Node::Text { line, .. } => *line,
      Node::List { line, .. } => *line,
    }
  }

  /// The text of a bare symbol, or `None` for strings and lists.
  pub fn as_symbol(&self) -> Option<&str> {
    match self {
      Node::Symbol { text, .. } => Some(text),
      _ => None,
    }
  }

  /// The contents of a quoted string, or `None` for symbols and lists.
  pub fn as_text(&self) -> Option<&str> {
    match self {
      Node::Text { text, .. } => Some(text),
      _ => None,
    }
  }

  /// The text of either atom kind. KiCad quotes inconsistently across
  /// file format versions, `(generator pcbnew)` in one and
  /// `(generator "pcbnew")` in the next, so most callers want this rather
  /// than [`Node::as_symbol`] or [`Node::as_text`].
  pub fn as_str(&self) -> Option<&str> {
    match self {
      Node::Symbol { text, .. } => Some(text),
      Node::Text { text, .. } => Some(text),
      Node::List { .. } => None,
    }
  }

  /// The elements of a list, or an empty slice for an atom.
  pub fn items(&self) -> &[Node] {
    match self {
      Node::List { items, .. } => items,
      _ => &[],
    }
  }

  /// The elements of a list after the tag, or an empty slice for an atom
  /// and for the empty list.
  pub fn values(&self) -> &[Node] {
    self.items().split_first().map_or(&[], |(_, rest)| rest)
  }

  /// The tag of a list, that is the text of its first element when that
  /// element is an atom.
  pub fn tag(&self) -> Option<&str> {
    self.items().first().and_then(Node::as_str)
  }

  /// The first direct child list carrying `tag`.
  ///
  /// Only direct children are searched, which is what makes it safe to
  /// ask a footprint for its `at` without picking up the `at` of a text
  /// item nested inside it.
  pub fn child(&self, tag: &str) -> Option<&Node> {
    self.children(tag).next()
  }

  /// Every direct child list carrying `tag`, in document order.
  pub fn children<'node>(
    &'node self,
    tag: &str,
  ) -> impl Iterator<Item = &'node Node> {
    self
      .items()
      .iter()
      .filter(move |item| item.tag() == Some(tag))
  }

  /// Like [`Node::child`] but an absent child is an error rather than
  /// `None`.
  pub fn required_child(&self, tag: &str) -> Result<&Node, ParseError> {
    self.child(tag).ok_or_else(|| {
      ParseError::new(
        self.line(),
        format!(
          "`{}` has no `{tag}` child",
          self.tag().unwrap_or("<not a list>")
        ),
      )
    })
  }

  /// The value at `index` counted after the tag.
  pub fn value(&self, index: usize) -> Result<&Node, ParseError> {
    self.values().get(index).ok_or_else(|| {
      ParseError::new(
        self.line(),
        format!(
          "`{}` has no value at position {index}",
          self.tag().unwrap_or("<not a list>")
        ),
      )
    })
  }

  /// The value at `index` as text, whether quoted in the file or not.
  pub fn value_str(&self, index: usize) -> Result<&str, ParseError> {
    let node = self.value(index)?;
    node.as_str().ok_or_else(|| {
      ParseError::new(node.line(), "expected an atom, found a list")
    })
  }

  /// The value at `index` as a decimal integer.
  pub fn value_integer(&self, index: usize) -> Result<i64, ParseError> {
    let node = self.value(index)?;
    let text = node.as_str().ok_or_else(|| {
      ParseError::new(node.line(), "expected an integer, found a list")
    })?;
    text.parse().map_err(|_| {
      ParseError::new(node.line(), format!("`{text}` is not an integer"))
    })
  }

  /// The value at `index` as a floating point number.
  ///
  /// Only for quantities KiCad itself keeps as a `double`, such as an
  /// angle in degrees or a rounded rectangle corner ratio. Coordinates go
  /// through [`Node::value_millimetres`] instead.
  pub fn value_double(&self, index: usize) -> Result<f64, ParseError> {
    let node = self.value(index)?;
    let text = node.as_str().ok_or_else(|| {
      ParseError::new(node.line(), "expected a number, found a list")
    })?;
    text.parse().map_err(|_| {
      ParseError::new(node.line(), format!("`{text}` is not a number"))
    })
  }

  /// The value at `index` read as millimetres and returned in whole
  /// nanometres.
  pub fn value_millimetres(&self, index: usize) -> Result<i64, ParseError> {
    let node = self.value(index)?;
    let text = node.as_str().ok_or_else(|| {
      ParseError::new(node.line(), "expected a number, found a list")
    })?;
    millimetres_to_nanometres(text, node.line())
  }

  /// Whether two trees hold the same tags, atoms and nesting, ignoring
  /// the line numbers. Used by the round trip test, where the reformatted
  /// text puts everything on different lines.
  pub fn structurally_equal(&self, other: &Node) -> bool {
    match (self, other) {
      (Node::Symbol { text: a, .. }, Node::Symbol { text: b, .. }) => a == b,
      (Node::Text { text: a, .. }, Node::Text { text: b, .. }) => a == b,
      (Node::List { items: a, .. }, Node::List { items: b, .. }) => {
        a.len() == b.len()
          && a.iter().zip(b.iter()).all(|(x, y)| x.structurally_equal(y))
      }
      _ => false,
    }
  }

  /// Write the tree back out as a single line of s-expression text.
  ///
  /// The output is not byte identical to KiCad's, which indents and wraps,
  /// but it parses back to a structurally equal tree.
  pub fn write_to_string(&self) -> String {
    let mut output = String::new();
    self.write_into(&mut output);
    output
  }

  /// Append this node's text to `output`, see [`Node::write_to_string`].
  fn write_into(&self, output: &mut String) {
    match self {
      Node::Symbol { text, .. } => output.push_str(text),
      Node::Text { text, .. } => {
        output.push('"');
        for character in text.chars() {
          match character {
            '"' => output.push_str("\\\""),
            '\\' => output.push_str("\\\\"),
            '\n' => output.push_str("\\n"),
            '\r' => output.push_str("\\r"),
            '\t' => output.push_str("\\t"),
            other => output.push(other),
          }
        }
        output.push('"');
      }
      Node::List { items, .. } => {
        output.push('(');
        for (index, item) in items.iter().enumerate() {
          if index > 0 {
            output.push(' ');
          }
          item.write_into(output);
        }
        output.push(')');
      }
    }
  }
}

/// Read one top level s-expression out of `input`.
///
/// Trailing whitespace is allowed, trailing tokens are not: a
/// `.kicad_pcb` file is exactly one `(kicad_pcb ...)` list.
pub fn parse(input: &str) -> Result<Node, ParseError> {
  let mut reader = Reader::new(input);
  reader.skip_whitespace();
  let node = reader.read_node()?;
  reader.skip_whitespace();
  if reader.position < reader.bytes.len() {
    return Err(ParseError::new(
      reader.line,
      "trailing text after the top level expression",
    ));
  }
  Ok(node)
}

/// Convert a decimal millimetre literal into whole nanometres.
///
/// The conversion is exact: the digits are read directly, never through
/// `f64`, so `0.075` is 75000 nanometres and not one unit either side of
/// it. A value with more than [`MILLIMETRE_FRACTION_DIGITS`] significant
/// decimals is rejected rather than rounded, because in the fixture corpus
/// every coordinate, size and width fits in six decimals and anything
/// longer means the reader is looking at a field it misidentified.
pub fn millimetres_to_nanometres(
  text: &str,
  line: u32,
) -> Result<i64, ParseError> {
  let malformed = || {
    ParseError::new(line, format!("`{text}` is not a decimal millimetre value"))
  };

  let bytes = text.as_bytes();
  let mut index = 0;
  let negative = match bytes.first() {
    Some(b'-') => {
      index = 1;
      true
    }
    Some(b'+') => {
      index = 1;
      false
    }
    _ => false,
  };

  let integer_start = index;
  while index < bytes.len() && bytes[index].is_ascii_digit() {
    index += 1;
  }
  let integer_text = &text[integer_start..index];

  let mut fraction_text = "";
  if index < bytes.len() && bytes[index] == b'.' {
    index += 1;
    let fraction_start = index;
    while index < bytes.len() && bytes[index].is_ascii_digit() {
      index += 1;
    }
    fraction_text = &text[fraction_start..index];
  }

  if index != bytes.len()
    || (integer_text.is_empty() && fraction_text.is_empty())
  {
    return Err(malformed());
  }

  let whole: i64 = if integer_text.is_empty() {
    0
  } else {
    integer_text.parse().map_err(|_| malformed())?
  };

  let (kept, dropped) = if fraction_text.len() > MILLIMETRE_FRACTION_DIGITS {
    fraction_text.split_at(MILLIMETRE_FRACTION_DIGITS)
  } else {
    (fraction_text, "")
  };
  if dropped.bytes().any(|digit| digit != b'0') {
    return Err(ParseError::new(
      line,
      format!("`{text}` is finer than one nanometre"),
    ));
  }

  let mut fraction: i64 = 0;
  for digit in kept.bytes() {
    fraction = fraction * 10 + i64::from(digit - b'0');
  }
  for _ in kept.len()..MILLIMETRE_FRACTION_DIGITS {
    fraction *= 10;
  }

  let magnitude = whole
    .checked_mul(NANOMETRES_PER_MILLIMETRE)
    .and_then(|scaled| scaled.checked_add(fraction))
    .ok_or_else(|| {
      ParseError::new(line, format!("`{text}` overflows a 64 bit nanometre"))
    })?;

  Ok(if negative { -magnitude } else { magnitude })
}

/// Cursor over the input text.
///
/// It walks bytes rather than characters. Every delimiter of the dialect
/// is ASCII and a UTF-8 continuation byte is never ASCII, so slicing the
/// original string at the recorded byte offsets always lands on a
/// character boundary.
struct Reader<'a> {
  /// The input, kept as text so that slices can be turned back into
  /// `String` without a validity check.
  input: &'a str,
  /// The input as bytes, for scanning.
  bytes: &'a [u8],
  /// Offset of the next unread byte.
  position: usize,
  /// One based line number of the byte at `position`.
  line: u32,
}

impl<'a> Reader<'a> {
  /// Start reading at the beginning of `input`.
  fn new(input: &'a str) -> Self {
    Self {
      input,
      bytes: input.as_bytes(),
      position: 0,
      line: 1,
    }
  }

  /// Advance past whitespace, counting the lines crossed.
  fn skip_whitespace(&mut self) {
    while self.position < self.bytes.len() {
      match self.bytes[self.position] {
        b'\n' => {
          self.line += 1;
          self.position += 1;
        }
        byte if byte.is_ascii_whitespace() => self.position += 1,
        _ => break,
      }
    }
  }

  /// Read one node, whatever kind starts at the cursor.
  fn read_node(&mut self) -> Result<Node, ParseError> {
    match self.bytes.get(self.position) {
      None => Err(ParseError::new(self.line, "unexpected end of input")),
      Some(b'(') => self.read_list(),
      Some(b')') => {
        Err(ParseError::new(self.line, "unbalanced closing parenthesis"))
      }
      Some(b'"') => self.read_text(),
      Some(_) => self.read_symbol(),
    }
  }

  /// Read a parenthesised list, the cursor sitting on the `(`.
  fn read_list(&mut self) -> Result<Node, ParseError> {
    let line = self.line;
    self.position += 1;
    let mut items = Vec::new();
    loop {
      self.skip_whitespace();
      match self.bytes.get(self.position) {
        None => {
          return Err(ParseError::new(line, "unterminated list"));
        }
        Some(b')') => {
          self.position += 1;
          return Ok(Node::List { items, line });
        }
        Some(_) => items.push(self.read_node()?),
      }
    }
  }

  /// Read a double quoted string, the cursor sitting on the opening
  /// quote.
  ///
  /// KiCad escapes the backslash and the double quote. The three control
  /// escapes are accepted too because they cost nothing; any other
  /// backslash pair is kept verbatim, which is what KiCad's own reader
  /// does with, for example, a Windows path inside a property value.
  fn read_text(&mut self) -> Result<Node, ParseError> {
    let line = self.line;
    self.position += 1;
    let mut text = String::new();
    loop {
      let byte = *self
        .bytes
        .get(self.position)
        .ok_or_else(|| ParseError::new(line, "unterminated quoted string"))?;
      match byte {
        b'"' => {
          self.position += 1;
          return Ok(Node::Text { text, line });
        }
        b'\\' => {
          let escaped =
            *self.bytes.get(self.position + 1).ok_or_else(|| {
              ParseError::new(self.line, "unterminated escape sequence")
            })?;
          match escaped {
            b'\\' => text.push('\\'),
            b'"' => text.push('"'),
            b'n' => text.push('\n'),
            b'r' => text.push('\r'),
            b't' => text.push('\t'),
            other => {
              text.push('\\');
              text.push(char::from(other));
            }
          }
          self.position += 2;
        }
        b'\n' => {
          self.line += 1;
          text.push('\n');
          self.position += 1;
        }
        _ => {
          let start = self.position;
          while self.position < self.bytes.len()
            && !matches!(self.bytes[self.position], b'"' | b'\\' | b'\n')
          {
            self.position += 1;
          }
          text.push_str(&self.input[start..self.position]);
        }
      }
    }
  }

  /// Read a bare symbol, which runs until whitespace or a delimiter.
  fn read_symbol(&mut self) -> Result<Node, ParseError> {
    let line = self.line;
    let start = self.position;
    while self.position < self.bytes.len() {
      let byte = self.bytes[self.position];
      if byte.is_ascii_whitespace() || matches!(byte, b'(' | b')' | b'"') {
        break;
      }
      self.position += 1;
    }
    Ok(Node::Symbol {
      text: self.input[start..self.position].to_string(),
      line,
    })
  }
}
