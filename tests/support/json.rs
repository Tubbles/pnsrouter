// SPDX-License-Identifier: GPL-3.0-or-later

//! A reader for the JSON that KiCad's PNS logger and router settings are
//! written in.
//!
//! `pcbnew/router/pns_logger.cpp:107` formats a log through nlohmann's
//! pretty printer and `qa/tools/pns/pns_log_file.cpp:582` reads it back
//! with the same library, so `pns.log` and `pns.settings` are ordinary
//! JSON documents. The router crate takes no dependencies, so this is the
//! smallest reader that covers what those two files use.
//!
//! Two details are worth knowing. Objects keep their members in a `Vec`
//! rather than a map, so iteration order is the document order and never a
//! hash order. Numbers are split into [`JsonValue::Integer`] and
//! [`JsonValue::Double`] on whether the literal has a fraction or an
//! exponent, so a board coordinate such as `111800000` stays an exact
//! `i64`.

use std::fmt;

/// What went wrong while reading, and where.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JsonError {
  /// One based line number the offending token starts on.
  pub line: u32,
  /// Human readable description of the problem.
  pub message: String,
}

impl JsonError {
  /// Build an error reported against `line`.
  pub fn new(line: u32, message: impl Into<String>) -> Self {
    Self {
      line,
      message: message.into(),
    }
  }
}

impl fmt::Display for JsonError {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(formatter, "line {}: {}", self.line, self.message)
  }
}

impl std::error::Error for JsonError {}

/// One JSON value.
#[derive(Debug, Clone)]
pub enum JsonValue {
  /// `null`.
  Null {
    /// One based line the literal starts on.
    line: u32,
  },
  /// `true` or `false`.
  Boolean {
    /// The value.
    value: bool,
    /// One based line the literal starts on.
    line: u32,
  },
  /// A number written without a fraction or an exponent.
  Integer {
    /// The value.
    value: i64,
    /// One based line the literal starts on.
    line: u32,
  },
  /// A number written with a fraction or an exponent.
  Double {
    /// The value.
    value: f64,
    /// One based line the literal starts on.
    line: u32,
  },
  /// A string with its escape sequences resolved.
  Text {
    /// The contents.
    value: String,
    /// One based line the literal starts on.
    line: u32,
  },
  /// An array.
  Array {
    /// The elements, in document order.
    items: Vec<JsonValue>,
    /// One based line the opening bracket is on.
    line: u32,
  },
  /// An object, kept as an ordered list of members.
  Object {
    /// The members, in document order.
    members: Vec<(String, JsonValue)>,
    /// One based line the opening brace is on.
    line: u32,
  },
}

impl JsonValue {
  /// The one based line this value starts on.
  pub fn line(&self) -> u32 {
    match self {
      JsonValue::Null { line } => *line,
      JsonValue::Boolean { line, .. } => *line,
      JsonValue::Integer { line, .. } => *line,
      JsonValue::Double { line, .. } => *line,
      JsonValue::Text { line, .. } => *line,
      JsonValue::Array { line, .. } => *line,
      JsonValue::Object { line, .. } => *line,
    }
  }

  /// A one word name for this value's kind, for error messages.
  fn kind(&self) -> &'static str {
    match self {
      JsonValue::Null { .. } => "null",
      JsonValue::Boolean { .. } => "a boolean",
      JsonValue::Integer { .. } => "an integer",
      JsonValue::Double { .. } => "a number",
      JsonValue::Text { .. } => "a string",
      JsonValue::Array { .. } => "an array",
      JsonValue::Object { .. } => "an object",
    }
  }

  /// The member named `name`, or `None` when the value is not an object
  /// or has no such member.
  pub fn member(&self, name: &str) -> Option<&JsonValue> {
    match self {
      JsonValue::Object { members, .. } => members
        .iter()
        .find(|(member_name, _)| member_name == name)
        .map(|(_, value)| value),
      _ => None,
    }
  }

  /// Like [`JsonValue::member`] but an absent member is an error.
  pub fn required_member(&self, name: &str) -> Result<&JsonValue, JsonError> {
    self
      .member(name)
      .ok_or_else(|| JsonError::new(self.line(), format!("no `{name}` member")))
  }

  /// The elements of an array.
  pub fn array(&self) -> Result<&[JsonValue], JsonError> {
    match self {
      JsonValue::Array { items, .. } => Ok(items),
      other => Err(JsonError::new(
        other.line(),
        format!("expected an array, found {}", other.kind()),
      )),
    }
  }

  /// The value as a string.
  pub fn text(&self) -> Result<&str, JsonError> {
    match self {
      JsonValue::Text { value, .. } => Ok(value),
      other => Err(JsonError::new(
        other.line(),
        format!("expected a string, found {}", other.kind()),
      )),
    }
  }

  /// The value as an integer.
  pub fn integer(&self) -> Result<i64, JsonError> {
    match self {
      JsonValue::Integer { value, .. } => Ok(*value),
      other => Err(JsonError::new(
        other.line(),
        format!("expected an integer, found {}", other.kind()),
      )),
    }
  }

  /// The value as a floating point number. An integer literal is accepted
  /// too, because JSON does not distinguish `1` from `1.0`.
  pub fn double(&self) -> Result<f64, JsonError> {
    match self {
      JsonValue::Double { value, .. } => Ok(*value),
      #[allow(clippy::cast_precision_loss)]
      JsonValue::Integer { value, .. } => Ok(*value as f64),
      other => Err(JsonError::new(
        other.line(),
        format!("expected a number, found {}", other.kind()),
      )),
    }
  }

  /// The value as a boolean.
  pub fn boolean(&self) -> Result<bool, JsonError> {
    match self {
      JsonValue::Boolean { value, .. } => Ok(*value),
      other => Err(JsonError::new(
        other.line(),
        format!("expected a boolean, found {}", other.kind()),
      )),
    }
  }
}

/// Read one JSON document out of `input`.
pub fn parse(input: &str) -> Result<JsonValue, JsonError> {
  let mut reader = Reader::new(input);
  reader.skip_whitespace();
  let value = reader.read_value()?;
  reader.skip_whitespace();
  if reader.position < reader.bytes.len() {
    return Err(JsonError::new(
      reader.line,
      "trailing text after the document",
    ));
  }
  Ok(value)
}

/// Cursor over the input text, byte oriented for the same reason as the
/// s-expression reader: every JSON delimiter is ASCII.
struct Reader<'a> {
  /// The input, kept as text so slices can become `String` cheaply.
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

  /// Consume `literal` at the cursor, or fail.
  fn expect(&mut self, literal: &str) -> Result<(), JsonError> {
    if self.input[self.position..].starts_with(literal) {
      self.position += literal.len();
      Ok(())
    } else {
      Err(JsonError::new(self.line, format!("expected `{literal}`")))
    }
  }

  /// Read one value, whatever kind starts at the cursor.
  fn read_value(&mut self) -> Result<JsonValue, JsonError> {
    let line = self.line;
    match self.bytes.get(self.position) {
      None => Err(JsonError::new(line, "unexpected end of input")),
      Some(b'{') => self.read_object(),
      Some(b'[') => self.read_array(),
      Some(b'"') => {
        let value = self.read_text()?;
        Ok(JsonValue::Text { value, line })
      }
      Some(b't') => {
        self.expect("true")?;
        Ok(JsonValue::Boolean { value: true, line })
      }
      Some(b'f') => {
        self.expect("false")?;
        Ok(JsonValue::Boolean { value: false, line })
      }
      Some(b'n') => {
        self.expect("null")?;
        Ok(JsonValue::Null { line })
      }
      Some(_) => self.read_number(),
    }
  }

  /// Read an object, the cursor sitting on the `{`.
  fn read_object(&mut self) -> Result<JsonValue, JsonError> {
    let line = self.line;
    self.position += 1;
    let mut members = Vec::new();
    loop {
      self.skip_whitespace();
      match self.bytes.get(self.position) {
        None => return Err(JsonError::new(line, "unterminated object")),
        Some(b'}') => {
          self.position += 1;
          return Ok(JsonValue::Object { members, line });
        }
        Some(b',') if !members.is_empty() => {
          self.position += 1;
          continue;
        }
        Some(b'"') => {
          let name = self.read_text()?;
          self.skip_whitespace();
          self.expect(":")?;
          self.skip_whitespace();
          let value = self.read_value()?;
          members.push((name, value));
        }
        Some(_) => {
          return Err(JsonError::new(self.line, "expected a member name"));
        }
      }
    }
  }

  /// Read an array, the cursor sitting on the `[`.
  fn read_array(&mut self) -> Result<JsonValue, JsonError> {
    let line = self.line;
    self.position += 1;
    let mut items = Vec::new();
    loop {
      self.skip_whitespace();
      match self.bytes.get(self.position) {
        None => return Err(JsonError::new(line, "unterminated array")),
        Some(b']') => {
          self.position += 1;
          return Ok(JsonValue::Array { items, line });
        }
        Some(b',') if !items.is_empty() => {
          self.position += 1;
        }
        Some(_) => items.push(self.read_value()?),
      }
    }
  }

  /// Read a string, the cursor sitting on the opening quote.
  fn read_text(&mut self) -> Result<String, JsonError> {
    let line = self.line;
    self.position += 1;
    let mut text = String::new();
    loop {
      let byte = *self
        .bytes
        .get(self.position)
        .ok_or_else(|| JsonError::new(line, "unterminated string"))?;
      match byte {
        b'"' => {
          self.position += 1;
          return Ok(text);
        }
        b'\\' => {
          let escaped =
            *self.bytes.get(self.position + 1).ok_or_else(|| {
              JsonError::new(self.line, "unterminated escape sequence")
            })?;
          self.position += 2;
          match escaped {
            b'"' => text.push('"'),
            b'\\' => text.push('\\'),
            b'/' => text.push('/'),
            b'b' => text.push('\u{8}'),
            b'f' => text.push('\u{c}'),
            b'n' => text.push('\n'),
            b'r' => text.push('\r'),
            b't' => text.push('\t'),
            b'u' => text.push(self.read_unicode_escape()?),
            other => {
              return Err(JsonError::new(
                self.line,
                format!("unknown escape `\\{}`", char::from(other)),
              ));
            }
          }
        }
        b'\n' => {
          return Err(JsonError::new(self.line, "newline inside a string"));
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

  /// Read the four hexadecimal digits of a `\u` escape, the cursor
  /// sitting just past the `u`.
  ///
  /// Surrogate pairs are not joined: nothing KiCad writes into a log needs
  /// them, and silently producing a replacement character would be worse
  /// than saying so.
  fn read_unicode_escape(&mut self) -> Result<char, JsonError> {
    let end = self.position + 4;
    if end > self.bytes.len() {
      return Err(JsonError::new(self.line, "truncated `\\u` escape"));
    }
    let digits = &self.input[self.position..end];
    let code = u32::from_str_radix(digits, 16).map_err(|_| {
      JsonError::new(self.line, format!("`{digits}` is not hexadecimal"))
    })?;
    self.position = end;
    char::from_u32(code).ok_or_else(|| {
      JsonError::new(self.line, format!("`\\u{digits}` is not a character"))
    })
  }

  /// Read a number, splitting integer literals from fractional ones.
  fn read_number(&mut self) -> Result<JsonValue, JsonError> {
    let line = self.line;
    let start = self.position;
    let mut fractional = false;
    while self.position < self.bytes.len() {
      match self.bytes[self.position] {
        b'0'..=b'9' | b'-' | b'+' => self.position += 1,
        b'.' | b'e' | b'E' => {
          fractional = true;
          self.position += 1;
        }
        _ => break,
      }
    }
    let text = &self.input[start..self.position];
    if text.is_empty() {
      return Err(JsonError::new(line, "expected a value"));
    }
    if fractional {
      let value = text.parse().map_err(|_| {
        JsonError::new(line, format!("`{text}` is not a number"))
      })?;
      Ok(JsonValue::Double { value, line })
    } else {
      let value = text.parse().map_err(|_| {
        JsonError::new(line, format!("`{text}` is not an integer"))
      })?;
      Ok(JsonValue::Integer { value, line })
    }
  }
}
