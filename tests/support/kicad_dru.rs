// SPDX-License-Identifier: GPL-3.0-or-later

//! A reader for the `.kicad_dru` custom design rules a project may ship.
//!
//! One case in the regression corpus has one,
//! `pns_regressions/issue24132-shove-same-net-via/pns.kicad_dru`, and its
//! single rule is what makes that case's golden what it is:
//!
//! ```text
//! (version 1)
//!
//! (rule "track_via_phys"
//!     (condition "A.Type == 'track' && B.Type == 'via'")
//!     (constraint physical_clearance (min 2mm))
//! )
//! ```
//!
//! KiCad reads the file with `DRC_RULES_PARSER::Parse`
//! (`pcbnew/drc/drc_rule_parser.cpp:166`) over a DSN lexer and compiles
//! each condition with `PCBEXPR_COMPILER`
//! (`pcbnew/pcbexpr_evaluator.cpp:893`) into a stack machine that can
//! call board geometry functions. Neither is reproduced. What is here is
//! the subset the corpus uses plus the neighbouring forms that cost
//! nothing:
//!
//! - `(version N)`, and `(rule "name" ...)` holding at most one
//!   `(condition "expr")` and any number of
//!   `(constraint TYPE (min L) (opt L) (max L))`.
//! - a length as a decimal literal with an `mm`, `in` or `mil` suffix.
//!   Those are KiCad's three length units in a rule expression
//!   (`PCBEXPR_UNIT_RESOLVER::GetSupportedUnits`,
//!   `pcbnew/pcbexpr_evaluator.cpp:829`), and a literal with no unit at
//!   all is an error there too
//!   (`common/libeval_compiler/libeval_compiler.cpp:1090`). `um` is
//!   **not** one of them: the lexer only takes a suffix that is in that
//!   list (`COMPILER::resolveUnits`, `:430`), so `2um` fails to compile.
//! - a condition built from `A.Type == '...'`, `A.NetClass == '...'`,
//!   the same two on `B`, `&&`, `||`, `!` and parentheses.
//!
//! **Anything else drops the rule** and records its name in
//! [`DesignRules::dropped_rules`]. Dropping rather than approximating is
//! deliberate: a `(layer ...)` clause this reader ignored would silently
//! widen the rule, whereas a dropped rule is visible in the ignore reason
//! of the test that depends on it.
//!
//! # Precedence, which is not "the largest wins"
//!
//! `DRC_ENGINE::EvalRules` (`pcbnew/drc/drc_engine.cpp:1006`) walks the
//! rules of one constraint type in load order and every match overwrites
//! what the previous one left, field by field (`applyConstraint`,
//! `:1048`, and the "Rule applied; overrides previous constraints"
//! report at `:1860`). So the **last** matching rule wins, and a later
//! rule that sets only `min` leaves an earlier rule's `opt` standing.
//! Load order is the implicit rules first and the file's rules after
//! (`DRC_ENGINE::InitEngine`, `:884`), so a rule here overrides the net
//! class value rather than being folded into it. That the corpus rule
//! also happens to be the largest is a coincidence of having one rule.
//!
//! # Both orders, always
//!
//! A condition is evaluated for `(A, B)` and, if it does not hold and
//! there is a second item, again for `(B, A)`
//! (`DRC_RULE_CONDITION::EvaluateFor`,
//! `pcbnew/drc/drc_rule_condition.cpp:111`, "Conditions are
//! commutative"). A one sided query keeps the single order, which is why
//! a rule naming `B` never fires on one: `B.Type` on a null item is
//! undefined and an undefined operand makes `==` false
//! (`pcbnew/pcbexpr_evaluator.cpp:648`,
//! `common/libeval_compiler/libeval_compiler.cpp:124`).

use super::sexpr::{Node, ParseError, parse as parse_sexpr};

/// The clearance ceiling KiCad clamps every clearance constraint to,
/// `MAXIMUM_CLEARANCE` (`include/board_design_settings.h:110`), in
/// nanometres. It exists to keep later arithmetic away from an overflow.
pub const MAXIMUM_CLEARANCE_NANOMETRES: i32 = 500_000_000;

/// Nanometres in one millimetre, `PCB_IU_PER_MM`
/// (`include/base_units.h:68`).
const NANOMETRES_PER_MILLIMETRE: i64 = 1_000_000;

/// Nanometres in one mil, `EDA_IU_SCALE::IU_PER_MILS`
/// (`include/base_units.h:83`, the millimetre scale times 0.0254).
const NANOMETRES_PER_MIL: i64 = 25_400;

/// Nanometres in one inch, a thousand mils.
const NANOMETRES_PER_INCH: i64 = 25_400_000;

/// Decimal places a length literal may carry before this reader gives up
/// rather than silently rounding a value it has misread.
const MAXIMUM_FRACTION_DIGITS: usize = 9;

// ---------------------------------------------------------------------
// The values a rule speaks about
// ---------------------------------------------------------------------

/// Which of the two items a condition term reads.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Side {
  /// The `A` of `A.Type`.
  A,
  /// The `B` of `B.Type`.
  B,
}

/// The `A.Type` name of a board item.
///
/// `A.Type` resolves to `ENUM_MAP<KICAD_T>::ToString` of the item's class
/// (`PCBEXPR_TYPE_REF::GetValue`, `pcbnew/pcbexpr_evaluator.cpp:644`),
/// and that map is `EDA_ITEM_DESC` (`common/eda_item.cpp:557`). Only the
/// classes a board in this corpus can hold are here; the comparison is
/// case insensitive, so the corpus rule's `'track'` matches `Track`
/// (`LIBEVAL::VALUE::EqualTo`,
/// `common/libeval_compiler/libeval_compiler.cpp:136`).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum ItemType {
  /// `PCB_TRACE_T` and `PCB_ARC_T`, which share the one name.
  Track,
  /// `PCB_VIA_T`.
  Via,
  /// `PCB_PAD_T`.
  Pad,
  /// `PCB_ZONE_T`, a rule area or a filled zone.
  Zone,
  /// `PCB_FOOTPRINT_T`.
  Footprint,
  /// `PCB_SHAPE_T`, which is what a board outline stroke is.
  Graphic,
}

impl ItemType {
  /// The name `A.Type` compares against, from `EDA_ITEM_DESC`
  /// (`common/eda_item.cpp:567` and following).
  pub const fn name(self) -> &'static str {
    match self {
      ItemType::Track => "Track",
      ItemType::Via => "Via",
      ItemType::Pad => "Pad",
      ItemType::Zone => "Zone",
      ItemType::Footprint => "Footprint",
      ItemType::Graphic => "Graphic",
    }
  }
}

/// One side of a rule query: everything a supported condition can read.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct RuleItem<'a> {
  /// What `A.Type` answers.
  pub item_type: ItemType,
  /// What `A.NetClass` answers, the effective net class name.
  pub net_class: &'a str,
}

// ---------------------------------------------------------------------
// Conditions
// ---------------------------------------------------------------------

/// The subset of KiCad's expression language this reader understands.
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Condition {
  /// `A.Type == 'track'`.
  TypeIs(Side, String),
  /// `A.NetClass == 'Default'`.
  NetClassIs(Side, String),
  /// `!(...)`.
  Not(Box<Condition>),
  /// `... && ...`.
  And(Box<Condition>, Box<Condition>),
  /// `... || ...`.
  Or(Box<Condition>, Box<Condition>),
}

impl Condition {
  /// Whether the condition holds for this pair in either order.
  ///
  /// Port of `DRC_RULE_CONDITION::EvaluateFor`
  /// (`pcbnew/drc/drc_rule_condition.cpp:39`): the given order first,
  /// and the swapped order only when there is a second item.
  pub fn holds_for(&self, a: RuleItem<'_>, b: Option<RuleItem<'_>>) -> bool {
    if self.holds_in_order(Some(a), b) {
      return true;
    }

    // :111, "Conditions are commutative".
    match b {
      None => false,
      Some(b) => self.holds_in_order(Some(b), Some(a)),
    }
  }

  /// The condition with `A` bound to the first item and `B` to the
  /// second.
  ///
  /// A term reading a side that is not there answers false, which is
  /// what an undefined operand does to `==`
  /// (`common/libeval_compiler/libeval_compiler.cpp:124`).
  fn holds_in_order(
    &self,
    a: Option<RuleItem<'_>>,
    b: Option<RuleItem<'_>>,
  ) -> bool {
    let side = |which: Side| match which {
      Side::A => a,
      Side::B => b,
    };

    match self {
      Condition::TypeIs(which, name) => side(*which)
        .is_some_and(|item| item.item_type.name().eq_ignore_ascii_case(name)),
      Condition::NetClassIs(which, name) => side(*which)
        .is_some_and(|item| item.net_class.eq_ignore_ascii_case(name)),
      Condition::Not(inner) => !inner.holds_in_order(a, b),
      Condition::And(left, right) => {
        left.holds_in_order(a, b) && right.holds_in_order(a, b)
      }
      Condition::Or(left, right) => {
        left.holds_in_order(a, b) || right.holds_in_order(a, b)
      }
    }
  }
}

// ---------------------------------------------------------------------
// Constraints and rules
// ---------------------------------------------------------------------

/// Which design rule a constraint sets, for the three this harness acts
/// on.
///
/// The keyword to constraint mapping is `DRC_RULES_PARSER::parseConstraint`
/// (`pcbnew/drc/drc_rule_parser.cpp:521` and following).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum ConstraintKind {
  /// `clearance`, KiCad's `CLEARANCE_CONSTRAINT` and the router's
  /// `CT_CLEARANCE`.
  Clearance,
  /// `physical_clearance`, `CT_PHYSICAL_CLEARANCE`. Net blind.
  PhysicalClearance,
  /// `physical_hole_clearance`, `CT_PHYSICAL_HOLE_CLEARANCE`. Net blind.
  PhysicalHoleClearance,
  /// Every other keyword. Parsed and kept, never acted on: none of them
  /// reaches the router's clearance ladder
  /// (`pcbnew/router/pns_kicad_iface.cpp:906` to `:964`).
  Other,
}

impl ConstraintKind {
  /// Whether [`MAXIMUM_CLEARANCE_NANOMETRES`] applies to this kind.
  ///
  /// `applyConstraint` caps a clearance and leaves a length, a width or a
  /// spoke count alone (`pcbnew/drc/drc_engine.cpp:1069` to `:1077`).
  const fn is_clearance(self) -> bool {
    matches!(
      self,
      ConstraintKind::Clearance
        | ConstraintKind::PhysicalClearance
        | ConstraintKind::PhysicalHoleClearance
    )
  }
}

/// One `(constraint ...)` of a rule.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Constraint {
  /// Which rule this sets.
  pub kind: ConstraintKind,
  /// The keyword as the file spelled it, kept for
  /// [`ConstraintKind::Other`].
  pub keyword: String,
  /// `(min ...)` in nanometres.
  pub min: Option<i32>,
  /// `(opt ...)` in nanometres.
  pub opt: Option<i32>,
  /// `(max ...)` in nanometres.
  pub max: Option<i32>,
}

/// One `(rule ...)` this reader could model in full.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Rule {
  /// The quoted name.
  pub name: String,
  /// The `(condition ...)`, or [`None`] for an unconditional rule.
  pub condition: Option<Condition>,
  /// Every `(constraint ...)`, in file order.
  pub constraints: Vec<Constraint>,
}

/// A whole `.kicad_dru` file.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct DesignRules {
  /// `(version N)`, zero for a file that carries none.
  pub version: i64,
  /// The rules this reader modelled, in file order, which is the order
  /// `EvalRules` walks them in.
  pub rules: Vec<Rule>,
  /// The names of the rules it did not, see the module documentation.
  pub dropped_rules: Vec<String>,
}

impl DesignRules {
  /// Whether the file holds a rule this reader could not model.
  pub fn has_unsupported(&self) -> bool {
    !self.dropped_rules.is_empty()
  }

  /// The constraint of one kind this pair resolves to, if any rule
  /// matches.
  ///
  /// Port of `DRC_ENGINE::EvalRules`
  /// (`pcbnew/drc/drc_engine.cpp:1006`) reduced to the file's own rules:
  /// every matching rule overwrites the fields it carries, so the last
  /// match wins. See the module documentation.
  pub fn constraint(
    &self,
    kind: ConstraintKind,
    a: RuleItem<'_>,
    b: Option<RuleItem<'_>>,
  ) -> Option<Constraint> {
    let mut resolved: Option<Constraint> = None;

    for rule in &self.rules {
      let matched = match &rule.condition {
        // :1792, an unconditional rule always applies.
        None => true,
        Some(condition) => condition.holds_for(a, b),
      };

      if !matched {
        continue;
      }

      for candidate in
        rule.constraints.iter().filter(|entry| entry.kind == kind)
      {
        // :1048, `applyConstraint`, field by field.
        let target = resolved.get_or_insert_with(|| Constraint {
          kind,
          keyword: candidate.keyword.clone(),
          min: None,
          opt: None,
          max: None,
        });

        target.keyword.clone_from(&candidate.keyword);

        if let Some(min) = candidate.min {
          // :1077, a clearance constraint is capped and nothing else is.
          target.min = Some(if kind.is_clearance() {
            min.min(MAXIMUM_CLEARANCE_NANOMETRES)
          } else {
            min
          });
        }

        if let Some(opt) = candidate.opt {
          target.opt = Some(opt);
        }

        if let Some(max) = candidate.max {
          target.max = Some(max);
        }
      }
    }

    resolved
  }

  /// The `min` of [`DesignRules::constraint`], or zero when nothing
  /// matches.
  ///
  /// Zero is what KiCad's ladder folds in for an unmatched query too:
  /// `EvalRules` always answers a constraint, `QueryConstraint` therefore
  /// returns true (`pcbnew/router/pns_kicad_iface.cpp:752`) and
  /// `MINOPTMAX::Min` gives 0 for a value with no minimum
  /// (`libs/core/include/core/minoptmax.h:29`).
  pub fn minimum(
    &self,
    kind: ConstraintKind,
    a: RuleItem<'_>,
    b: Option<RuleItem<'_>>,
  ) -> i32 {
    self
      .constraint(kind, a, b)
      .and_then(|constraint| constraint.min)
      .unwrap_or(0)
  }

  /// The largest `min` any rule of this kind could ever hand back.
  ///
  /// Port of `DRC_ENGINE::QueryWorstConstraint`
  /// (`pcbnew/drc/drc_engine.cpp:2405`), which is how
  /// `GetBiggestClearanceValue` (`pcbnew/board_design_settings.cpp:1795`)
  /// folds a physical clearance rule into the world's maximum clearance.
  pub fn worst_minimum(&self, kind: ConstraintKind) -> i32 {
    let mut worst = 0;

    for rule in &self.rules {
      for constraint in &rule.constraints {
        if constraint.kind == kind {
          worst = worst.max(constraint.min.unwrap_or(0));
        }
      }
    }

    if kind.is_clearance() {
      worst = worst.min(MAXIMUM_CLEARANCE_NANOMETRES);
    }

    worst
  }

  /// Whether the file defines a net blind physical clearance rule.
  ///
  /// Port of `DRC_ENGINE::HasUserDefinedPhysicalConstraint`
  /// (`pcbnew/drc/drc_engine.cpp:2449`), which is
  /// `HasConditionalConstraint` (`:2432`) over the two physical kinds: a
  /// rule counts when it is not implicit, which every rule in this file
  /// is, **and** carries a condition. An unconditional physical rule does
  /// not set the flag.
  pub fn has_conditional_physical_constraint(&self) -> bool {
    self.rules.iter().any(|rule| {
      rule.condition.is_some()
        && rule.constraints.iter().any(|constraint| {
          matches!(
            constraint.kind,
            ConstraintKind::PhysicalClearance
              | ConstraintKind::PhysicalHoleClearance
          )
        })
    })
  }
}

// ---------------------------------------------------------------------
// Reading the file
// ---------------------------------------------------------------------

/// Read a whole `.kicad_dru`.
///
/// # Errors
///
/// When the text is not the s-expression dialect, when a top level entry
/// is neither `version` nor `rule`, or when a length is not a decimal
/// literal with one of the three units. A rule whose shape is legal but
/// outside the modelled subset is **not** an error; it lands in
/// [`DesignRules::dropped_rules`].
pub fn parse(text: &str) -> Result<DesignRules, ParseError> {
  // A `.kicad_dru` is a sequence of top level expressions and the reader
  // takes exactly one, so wrap them. Nothing is added before the first
  // byte of a line, which keeps every reported line number the file's
  // own.
  let wrapped = format!("({})", strip_comments(text));
  let document = parse_sexpr(&wrapped)?;
  let mut rules = DesignRules::default();

  for entry in document.items() {
    match entry.tag() {
      Some("version") => rules.version = entry.value_integer(0)?,
      Some("rule") => match read_rule(entry)? {
        Ok(rule) => rules.rules.push(rule),
        Err(name) => rules.dropped_rules.push(name),
      },
      _ => {
        return Err(ParseError::new(
          entry.line(),
          "a design rules file holds `version` and `rule` entries",
        ));
      }
    }
  }

  Ok(rules)
}

/// Blank out the comment lines before the s-expression reader sees them.
///
/// KiCad's lexer treats a line whose first non blank character is `#` as
/// a comment (`common/dsnlexer.cpp:571`). The lines are emptied rather
/// than removed so that every following line keeps its number.
fn strip_comments(text: &str) -> String {
  let mut output = String::with_capacity(text.len());

  for (index, line) in text.lines().enumerate() {
    if index > 0 {
      output.push('\n');
    }

    if !line.trim_start().starts_with('#') {
      output.push_str(line);
    }
  }

  output
}

/// Read one `(rule ...)`.
///
/// The outer `Result` is a malformed file; the inner one is a rule this
/// reader will not model, carrying its name for
/// [`DesignRules::dropped_rules`].
fn read_rule(node: &Node) -> Result<Result<Rule, String>, ParseError> {
  let name = node.value_str(0)?.to_string();
  let mut condition = None;
  let mut constraints = Vec::new();

  for element in node.values().iter().skip(1) {
    match element.tag() {
      Some("condition") => {
        let Some(parsed) = read_condition(element.value_str(0)?) else {
          return Ok(Err(name));
        };

        condition = Some(parsed);
      }
      Some("constraint") => constraints.push(read_constraint(element)?),
      // A `(layer ...)`, a `(severity ...)`, a `(priority ...)`: every
      // one of them narrows the rule, so ignoring it would widen it.
      _ => return Ok(Err(name)),
    }
  }

  Ok(Ok(Rule {
    name,
    condition,
    constraints,
  }))
}

/// Read one `(constraint TYPE (min L) (opt L) (max L))`.
fn read_constraint(node: &Node) -> Result<Constraint, ParseError> {
  let keyword = node.value_str(0)?.to_string();
  let kind = match keyword.as_str() {
    "clearance" => ConstraintKind::Clearance,
    // :492, `mechanical_clearance` is the deprecated spelling.
    "physical_clearance" | "mechanical_clearance" => {
      ConstraintKind::PhysicalClearance
    }
    // :497
    "physical_hole_clearance" | "mechanical_hole_clearance" => {
      ConstraintKind::PhysicalHoleClearance
    }
    _ => ConstraintKind::Other,
  };
  let mut constraint = Constraint {
    kind,
    keyword,
    min: None,
    opt: None,
    max: None,
  };

  for element in node.values().iter().skip(1) {
    let tag = element.tag();
    let slot = match tag {
      // :768, :810, :789.
      Some("min") => &mut constraint.min,
      Some("opt") => &mut constraint.opt,
      Some("max") => &mut constraint.max,
      _ => {
        return Err(ParseError::new(
          element.line(),
          "a constraint holds `min`, `opt` and `max` values",
        ));
      }
    };
    let mut literal = String::new();

    for value in element.values() {
      literal.push_str(value.as_str().ok_or_else(|| {
        ParseError::new(value.line(), "expected a length, found a list")
      })?);
    }

    *slot = Some(parse_length(&literal, element.line())?);
  }

  Ok(constraint)
}

/// Read a length literal such as `2mm`, `10mil` or `0.5in` into whole
/// nanometres.
///
/// Port of `DRC_RULES_PARSER::parseValueWithUnits`
/// (`pcbnew/drc/drc_rule_parser.cpp:844`) for the one expression shape
/// that matters, a literal with a unit. The digits are read directly
/// rather than through `f64`, and the result is rounded half away from
/// zero, which is what `KiROUND` does to the evaluator's double
/// (`pcbnew/pcbexpr_evaluator.cpp:933`).
///
/// # Errors
///
/// When the text is not a decimal literal, when the unit is not one of
/// KiCad's three lengths, or when the value does not fit in a nanometre
/// `i32`.
fn parse_length(text: &str, line: u32) -> Result<i32, ParseError> {
  let malformed = || {
    ParseError::new(line, format!("`{text}` is not a length in mm, in or mil"))
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

  if integer_text.is_empty() && fraction_text.is_empty() {
    return Err(malformed());
  }

  if fraction_text.len() > MAXIMUM_FRACTION_DIGITS {
    return Err(ParseError::new(
      line,
      format!("`{text}` carries more decimals than a length can hold"),
    ));
  }

  // The unit resolver takes `mil`, `mm` and `in` for a length
  // (`pcbnew/pcbexpr_evaluator.cpp:831`).
  let scale = match text[index..].trim() {
    "mm" => NANOMETRES_PER_MILLIMETRE,
    "in" => NANOMETRES_PER_INCH,
    "mil" => NANOMETRES_PER_MIL,
    _ => return Err(malformed()),
  };

  let mut mantissa: i128 = 0;

  for digit in integer_text.bytes().chain(fraction_text.bytes()) {
    mantissa = mantissa * 10 + i128::from(digit - b'0');
  }

  let divisor = 10_i128.pow(fraction_text.len() as u32);
  // Half away from zero on the magnitude, which is `KiROUND`.
  let scaled = (mantissa * i128::from(scale) * 2 + divisor) / (divisor * 2);
  let signed = if negative { -scaled } else { scaled };

  i32::try_from(signed).map_err(|_| {
    ParseError::new(line, format!("`{text}` overflows a nanometre `i32`"))
  })
}

// ---------------------------------------------------------------------
// Reading a condition
// ---------------------------------------------------------------------

/// One token of the condition subset.
#[derive(Clone, PartialEq, Eq, Debug)]
enum Token {
  /// A bare word, `A` or `Type`.
  Word(String),
  /// A single quoted literal.
  Text(String),
  /// `.`
  Dot,
  /// `==`
  Equal,
  /// `&&`
  And,
  /// `||`
  Or,
  /// `!`
  Not,
  /// `(`
  Open,
  /// `)`
  Close,
}

/// Read a condition expression, or [`None`] when it is outside the
/// modelled subset.
fn read_condition(text: &str) -> Option<Condition> {
  // An empty expression is KiCad's unconditional rule
  // (`pcbnew/drc/drc_engine.cpp:1792`), but a rule that spells one is not
  // in the corpus and the caller has a shorter spelling for it, so it is
  // not accepted here.
  let tokens = tokenize(text)?;
  let mut cursor = 0;
  let condition = read_or(&tokens, &mut cursor)?;

  (cursor == tokens.len()).then_some(condition)
}

/// Split a condition into tokens, or [`None`] on a character the subset
/// has no meaning for.
fn tokenize(text: &str) -> Option<Vec<Token>> {
  let bytes = text.as_bytes();
  let mut tokens = Vec::new();
  let mut index = 0;

  while index < bytes.len() {
    let byte = bytes[index];

    if byte.is_ascii_whitespace() {
      index += 1;

      continue;
    }

    if byte.is_ascii_alphabetic() || byte == b'_' {
      let start = index;

      while index < bytes.len()
        && (bytes[index].is_ascii_alphanumeric() || bytes[index] == b'_')
      {
        index += 1;
      }

      tokens.push(Token::Word(text[start..index].to_string()));

      continue;
    }

    if byte == b'\'' {
      index += 1;

      let start = index;

      while index < bytes.len() && bytes[index] != b'\'' {
        index += 1;
      }

      if index == bytes.len() {
        return None;
      }

      let literal = &text[start..index];

      index += 1;

      // A literal holding a glob is matched with `WildCompareString`
      // (`common/libeval_compiler/libeval_compiler.cpp:1110`), which is
      // not modelled.
      if literal.contains('*') || literal.contains('?') {
        return None;
      }

      tokens.push(Token::Text(literal.to_string()));

      continue;
    }

    let (token, width) = match (byte, bytes.get(index + 1)) {
      (b'=', Some(b'=')) => (Token::Equal, 2),
      (b'&', Some(b'&')) => (Token::And, 2),
      (b'|', Some(b'|')) => (Token::Or, 2),
      (b'.', _) => (Token::Dot, 1),
      (b'!', Some(b'=')) => return None,
      (b'!', _) => (Token::Not, 1),
      (b'(', _) => (Token::Open, 1),
      (b')', _) => (Token::Close, 1),
      _ => return None,
    };

    tokens.push(token);
    index += width;
  }

  Some(tokens)
}

/// `or := and ( "||" and )*`
fn read_or(tokens: &[Token], cursor: &mut usize) -> Option<Condition> {
  let mut left = read_and(tokens, cursor)?;

  while tokens.get(*cursor) == Some(&Token::Or) {
    *cursor += 1;

    let right = read_and(tokens, cursor)?;

    left = Condition::Or(Box::new(left), Box::new(right));
  }

  Some(left)
}

/// `and := unary ( "&&" unary )*`
fn read_and(tokens: &[Token], cursor: &mut usize) -> Option<Condition> {
  let mut left = read_unary(tokens, cursor)?;

  while tokens.get(*cursor) == Some(&Token::And) {
    *cursor += 1;

    let right = read_unary(tokens, cursor)?;

    left = Condition::And(Box::new(left), Box::new(right));
  }

  Some(left)
}

/// `unary := "!" unary | "(" or ")" | comparison`
fn read_unary(tokens: &[Token], cursor: &mut usize) -> Option<Condition> {
  match tokens.get(*cursor) {
    Some(Token::Not) => {
      *cursor += 1;

      Some(Condition::Not(Box::new(read_unary(tokens, cursor)?)))
    }
    Some(Token::Open) => {
      *cursor += 1;

      let inner = read_or(tokens, cursor)?;

      if tokens.get(*cursor) != Some(&Token::Close) {
        return None;
      }

      *cursor += 1;

      Some(inner)
    }
    _ => read_comparison(tokens, cursor),
  }
}

/// `comparison := ("A" | "B") "." ("Type" | "NetClass") "==" literal`
fn read_comparison(tokens: &[Token], cursor: &mut usize) -> Option<Condition> {
  let Some(Token::Word(receiver)) = tokens.get(*cursor) else {
    return None;
  };
  let side = match receiver.as_str() {
    "A" => Side::A,
    "B" => Side::B,
    _ => return None,
  };

  if tokens.get(*cursor + 1) != Some(&Token::Dot) {
    return None;
  }

  let Some(Token::Word(field)) = tokens.get(*cursor + 2) else {
    return None;
  };

  if tokens.get(*cursor + 3) != Some(&Token::Equal) {
    return None;
  }

  let Some(Token::Text(literal)) = tokens.get(*cursor + 4) else {
    return None;
  };

  *cursor += 5;

  // The field name comparison is case insensitive in KiCad too
  // (`PCBEXPR_UCODE::CreateVarRef`, `pcbnew/pcbexpr_evaluator.cpp:725`).
  if field.eq_ignore_ascii_case("Type") {
    Some(Condition::TypeIs(side, literal.clone()))
  } else if field.eq_ignore_ascii_case("NetClass") {
    Some(Condition::NetClassIs(side, literal.clone()))
  } else {
    None
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  /// The corpus file itself.
  const CORPUS: &str = "(version 1)\n\
    \n\
    (rule \"track_via_phys\"\n\
    \t(condition \"A.Type == 'track' && B.Type == 'via'\")\n\
    \t(constraint physical_clearance (min 2mm))\n\
    )\n";

  /// A track, for the query side of a test.
  fn track(net_class: &str) -> RuleItem<'_> {
    RuleItem {
      item_type: ItemType::Track,
      net_class,
    }
  }

  /// A via, likewise.
  fn via(net_class: &str) -> RuleItem<'_> {
    RuleItem {
      item_type: ItemType::Via,
      net_class,
    }
  }

  /// The one file in the corpus reads, and answers in both orders.
  #[test]
  fn the_corpus_rule_gives_a_track_and_a_via_two_millimetres() {
    let rules = parse(CORPUS).expect("the corpus file parses");

    assert_eq!(rules.version, 1);
    assert_eq!(rules.rules.len(), 1);
    assert!(!rules.has_unsupported());
    assert!(rules.has_conditional_physical_constraint());
    assert_eq!(
      rules.worst_minimum(ConstraintKind::PhysicalClearance),
      2_000_000
    );

    let kind = ConstraintKind::PhysicalClearance;

    assert_eq!(
      rules.minimum(kind, track("Default"), Some(via("Default"))),
      2_000_000
    );
    // The swapped order, `EvaluateFor`'s second try.
    assert_eq!(
      rules.minimum(kind, via("Default"), Some(track("Default"))),
      2_000_000
    );
    // Two tracks match neither order.
    assert_eq!(
      rules.minimum(kind, track("Default"), Some(track("Default"))),
      0
    );
    // A one sided query never satisfies a term naming `B`.
    assert_eq!(rules.minimum(kind, track("Default"), None), 0);
    // And the rule says nothing about the copper clearance.
    assert_eq!(
      rules.minimum(
        ConstraintKind::Clearance,
        track("Default"),
        Some(via("Default"))
      ),
      0
    );
  }

  /// A comment line and a net class condition.
  #[test]
  fn a_net_class_condition_reads_the_class_name() {
    let text = "# a comment the lexer drops\n\
      (version 1)\n\
      (rule \"power\"\n\
        (condition \"A.NetClass == 'Power' && !(B.Type == 'pad')\")\n\
        (constraint clearance (min 0.5mm))\n\
      )\n";
    let rules = parse(text).expect("the file parses");

    assert!(!rules.has_unsupported());
    // Not a physical rule, so the net blind flag stays down.
    assert!(!rules.has_conditional_physical_constraint());

    let kind = ConstraintKind::Clearance;

    assert_eq!(
      rules.minimum(kind, track("Power"), Some(via("Default"))),
      500_000
    );
    // Matched in the swapped order: `B` is then the `Power` track, and
    // `A` the via, which is not a pad.
    assert_eq!(
      rules.minimum(kind, via("Default"), Some(track("Power"))),
      500_000
    );
    assert_eq!(
      rules.minimum(kind, track("Default"), Some(via("Default"))),
      0
    );
  }

  /// A condition outside the subset drops its rule and raises the flag.
  #[test]
  fn an_unsupported_condition_drops_the_rule() {
    let text = "(version 1)\n\
      (rule \"courtyard\"\n\
        (condition \"A.intersectsCourtyard('U1')\")\n\
        (constraint physical_clearance (min 1mm))\n\
      )\n\
      (rule \"layered\"\n\
        (layer \"F.Cu\")\n\
        (condition \"A.Type == 'via'\")\n\
        (constraint physical_clearance (min 3mm))\n\
      )\n";
    let rules = parse(text).expect("the file parses");

    assert!(rules.rules.is_empty());
    assert!(rules.has_unsupported());
    assert_eq!(rules.dropped_rules, vec!["courtyard", "layered"]);
    // A dropped rule contributes nothing, the flag is the whole report.
    assert!(!rules.has_conditional_physical_constraint());
    assert_eq!(rules.worst_minimum(ConstraintKind::PhysicalClearance), 0);
  }

  /// Every unit KiCad's rule expressions take, and none that they do not.
  #[test]
  fn a_length_reads_in_millimetres_inches_and_mils() {
    assert_eq!(parse_length("2mm", 1), Ok(2_000_000));
    assert_eq!(parse_length("0.075mm", 1), Ok(75_000));
    assert_eq!(parse_length("1in", 1), Ok(25_400_000));
    assert_eq!(parse_length("10mil", 1), Ok(254_000));
    assert_eq!(parse_length("0.5 mm", 1), Ok(500_000));
    // Half away from zero, as `KiROUND` rounds the evaluator's double:
    // 0.0005 mil is 12.7 nanometres.
    assert_eq!(parse_length("0.0005mil", 1), Ok(13));
    assert_eq!(parse_length("-0.0005mil", 1), Ok(-13));
    assert_eq!(parse_length("-1mm", 1), Ok(-1_000_000));

    // `um` is not one of KiCad's expression units, nor is a bare number.
    assert!(parse_length("2um", 1).is_err());
    assert!(parse_length("2", 1).is_err());
    assert!(parse_length("2cm", 1).is_err());
    assert!(parse_length("mm", 1).is_err());
  }

  /// The last matching rule wins, field by field, rather than the
  /// largest.
  #[test]
  fn the_last_matching_rule_overrides_the_earlier_ones() {
    let text = "(version 1)\n\
      (rule \"wide\"\n\
        (condition \"A.Type == 'via'\")\n\
        (constraint physical_clearance (min 3mm) (opt 4mm))\n\
      )\n\
      (rule \"narrow\"\n\
        (condition \"A.Type == 'via'\")\n\
        (constraint physical_clearance (min 1mm))\n\
      )\n";
    let rules = parse(text).expect("the file parses");
    let resolved = rules
      .constraint(ConstraintKind::PhysicalClearance, via("Default"), None)
      .expect("the rules match a via");

    assert_eq!(resolved.min, Some(1_000_000));
    // `applyConstraint` only writes the fields the later rule carries.
    assert_eq!(resolved.opt, Some(4_000_000));
    // The broad phase still has to reach for the widest of them.
    assert_eq!(
      rules.worst_minimum(ConstraintKind::PhysicalClearance),
      3_000_000
    );
  }

  /// A constraint keyword the harness does not act on is still kept.
  #[test]
  fn an_unmodelled_constraint_keyword_is_parsed_and_ignored() {
    let text = "(version 1)\n\
      (rule \"thin\"\n\
        (condition \"A.Type == 'track'\")\n\
        (constraint track_width (min 0.1mm) (opt 0.2mm))\n\
      )\n";
    let rules = parse(text).expect("the file parses");

    assert!(!rules.has_unsupported());

    let constraint = &rules.rules[0].constraints[0];

    assert_eq!(constraint.kind, ConstraintKind::Other);
    assert_eq!(constraint.keyword, "track_width");
    assert_eq!(constraint.min, Some(100_000));
    assert_eq!(
      rules.constraint(ConstraintKind::Clearance, track("Default"), None),
      None
    );
  }

  /// A file that is not the dialect at all is an error, not a drop.
  #[test]
  fn a_malformed_file_is_an_error() {
    assert!(parse("(rule \"unterminated\"").is_err());
    assert!(parse("(nonsense 1)").is_err());
    assert!(parse("(rule \"x\" (constraint clearance (min)))").is_err());
  }
}
