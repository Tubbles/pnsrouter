// SPDX-License-Identifier: GPL-3.0-or-later

//! A generational arena, the crate's replacement for KiCad's raw pointers.
//!
//! KiCad's router refers to every stored object by address and keeps
//! removed items alive in `NODE::m_garbageItems`
//! (`pcbnew/router/pns_node.cpp:840`) precisely because a `LINE` may still
//! hold links to them. `doc/reference/kicad/02-item-model-and-node.md`
//! section 10.1 asks for a generational index instead, so that a stale
//! link turns from undefined behaviour into a detectable `None`.
//!
//! The arena is generic because the world model needs three of them:
//! items, joints and nodes. Handles carry a zero sized type tag, so an
//! [`ArenaId<Item>`] cannot be passed where an `ArenaId<Node>` is
//! expected.
//!
//! [`Arena::iter`] walks the live slots in **index order**, never in
//! insertion or free list order, so every traversal of an arena is
//! reproducible. That is `DESIGN.md` section 8.
//!
//! [`ArenaId<Item>`]: ArenaId

use std::cmp::Ordering;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

/// A handle to a value in an [`Arena`].
///
/// The pair of an arena slot index and the generation that slot carried
/// when the handle was made. A handle whose generation no longer matches
/// its slot is stale, and every lookup answers `None` for it.
///
/// The `T` parameter is a type tag only: it costs no space and exists so
/// that handles into different arenas cannot be mixed up.
pub struct ArenaId<T> {
  /// The slot index.
  index: u32,
  /// The slot's generation when this handle was made.
  generation: u32,
  /// The type tag. `fn() -> T` rather than `T` so that the handle is
  /// `Copy`, `Send` and `Sync` whatever `T` is.
  tag: PhantomData<fn() -> T>,
}

impl<T> ArenaId<T> {
  /// The slot index this handle points at.
  ///
  /// Exposed for the spatial index and the debug output, which key on
  /// the numeric pair. It is not a valid handle on its own.
  pub const fn index(self) -> u32 {
    self.index
  }

  /// The generation this handle was made in.
  pub const fn generation(self) -> u32 {
    self.generation
  }
}

impl<T> Clone for ArenaId<T> {
  fn clone(&self) -> Self {
    *self
  }
}

impl<T> Copy for ArenaId<T> {}

impl<T> PartialEq for ArenaId<T> {
  fn eq(&self, other: &Self) -> bool {
    self.index == other.index && self.generation == other.generation
  }
}

impl<T> Eq for ArenaId<T> {}

/// Handles order by `(index, generation)`.
///
/// The order is arbitrary but total and stable: it exists so that a
/// handle can key an ordered container. That is what replaces KiCad's
/// `std::unordered_set<ITEM*>` and `std::unordered_multimap` of joints,
/// whose iteration order is the address order of the heap and therefore
/// changes between runs (`DESIGN.md` section 8,
/// `doc/reference/kicad/02-item-model-and-node.md` section 11 entries 2,
/// 7 and 15).
///
/// It carries no geometric and no temporal meaning. A freed slot is
/// reused by the next insertion, so a handle made later can sort before
/// one made earlier. Anywhere a tie has to break on something the user
/// can see, break it on `Item::uid` instead.
impl<T> PartialOrd for ArenaId<T> {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}

impl<T> Ord for ArenaId<T> {
  fn cmp(&self, other: &Self) -> Ordering {
    (self.index, self.generation).cmp(&(other.index, other.generation))
  }
}

impl<T> Hash for ArenaId<T> {
  fn hash<H: Hasher>(&self, state: &mut H) {
    self.index.hash(state);
    self.generation.hash(state);
  }
}

impl<T> fmt::Debug for ArenaId<T> {
  fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(formatter, "ArenaId({}, {})", self.index, self.generation)
  }
}

/// One arena slot.
#[derive(Clone, Debug)]
struct Slot<T> {
  /// The generation this slot is currently at. It grows by one on every
  /// removal, so a handle made before the removal no longer matches.
  generation: u32,
  /// The value, or `None` when the slot is free.
  value: Option<T>,
}

/// A generational arena.
///
/// Removal keeps the slot and bumps its generation rather than shifting
/// anything, so live handles into other slots stay valid. A freed slot is
/// reused by the next insertion, which is why the generation is what makes
/// a stale handle detectable.
///
/// `Clone` is derived, because `NODE::Branch`
/// (`pcbnew/router/pns_node.cpp:157`) copies whole containers and the
/// port follows it, see `DESIGN.md` section 4.5.
#[derive(Clone, Debug)]
pub struct Arena<T> {
  /// Every slot ever allocated, live or free.
  slots: Vec<Slot<T>>,
  /// The indices of the free slots, most recently freed first.
  free: Vec<u32>,
}

impl<T> Arena<T> {
  /// An empty arena.
  pub const fn new() -> Self {
    Self {
      slots: Vec::new(),
      free: Vec::new(),
    }
  }

  /// The number of live values.
  pub fn len(&self) -> usize {
    self.slots.len() - self.free.len()
  }

  /// Whether the arena holds no live value.
  pub fn is_empty(&self) -> bool {
    self.len() == 0
  }

  /// Store a value and return its handle.
  ///
  /// # Panics
  ///
  /// When the arena would grow past `u32::MAX` slots, which no board can
  /// reach.
  pub fn insert(&mut self, value: T) -> ArenaId<T> {
    if let Some(index) = self.free.pop() {
      let slot = &mut self.slots[index as usize];
      slot.value = Some(value);

      return ArenaId {
        index,
        generation: slot.generation,
        tag: PhantomData,
      };
    }

    let index = u32::try_from(self.slots.len())
      .expect("arena slot count must fit in a u32");
    self.slots.push(Slot {
      generation: 0,
      value: Some(value),
    });

    ArenaId {
      index,
      generation: 0,
      tag: PhantomData,
    }
  }

  /// Take a value out, freeing its slot.
  ///
  /// Returns `None` for a stale or already removed handle. The slot's
  /// generation grows, so the handle that was just used never matches
  /// again.
  pub fn remove(&mut self, id: ArenaId<T>) -> Option<T> {
    let slot = self.slots.get_mut(id.index as usize)?;

    if slot.generation != id.generation {
      return None;
    }

    let value = slot.value.take()?;
    slot.generation = slot.generation.wrapping_add(1);
    self.free.push(id.index);

    Some(value)
  }

  /// Borrow a value.
  pub fn get(&self, id: ArenaId<T>) -> Option<&T> {
    let slot = self.slots.get(id.index as usize)?;

    if slot.generation != id.generation {
      return None;
    }

    slot.value.as_ref()
  }

  /// Borrow a value mutably.
  pub fn get_mut(&mut self, id: ArenaId<T>) -> Option<&mut T> {
    let slot = self.slots.get_mut(id.index as usize)?;

    if slot.generation != id.generation {
      return None;
    }

    slot.value.as_mut()
  }

  /// Whether the handle still names a live value.
  pub fn contains(&self, id: ArenaId<T>) -> bool {
    self.get(id).is_some()
  }

  /// Every live value with its handle, in **slot index order**.
  ///
  /// The order does not depend on when a value was inserted or which
  /// slots were freed before it, which is what makes an arena traversal
  /// reproducible (`DESIGN.md` section 8).
  pub fn iter(&self) -> impl Iterator<Item = (ArenaId<T>, &T)> {
    self.slots.iter().enumerate().filter_map(|(index, slot)| {
      let value = slot.value.as_ref()?;

      Some((
        ArenaId {
          index: index as u32,
          generation: slot.generation,
          tag: PhantomData,
        },
        value,
      ))
    })
  }
}

impl<T> Default for Arena<T> {
  fn default() -> Self {
    Self::new()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  /// The handles of every live value, in iteration order.
  fn ids(arena: &Arena<&'static str>) -> Vec<u32> {
    arena.iter().map(|(id, _)| id.index()).collect()
  }

  /// The values, in iteration order.
  fn values(arena: &Arena<&'static str>) -> Vec<&'static str> {
    arena.iter().map(|(_, value)| *value).collect()
  }

  #[test]
  fn insert_then_get_returns_the_value() {
    let mut arena = Arena::new();
    let first = arena.insert("a");
    let second = arena.insert("b");

    assert_eq!(arena.get(first), Some(&"a"));
    assert_eq!(arena.get(second), Some(&"b"));
    assert_eq!(arena.len(), 2);
    assert!(!arena.is_empty());
  }

  #[test]
  fn a_fresh_arena_is_empty() {
    let arena: Arena<&str> = Arena::new();

    assert_eq!(arena.len(), 0);
    assert!(arena.is_empty());
    assert_eq!(values(&arena), Vec::<&str>::new());
  }

  #[test]
  fn get_mut_writes_through() {
    let mut arena = Arena::new();
    let id = arena.insert("a");

    *arena.get_mut(id).unwrap() = "z";

    assert_eq!(arena.get(id), Some(&"z"));
  }

  #[test]
  fn remove_returns_the_value_and_frees_the_slot() {
    let mut arena = Arena::new();
    let id = arena.insert("a");

    assert_eq!(arena.remove(id), Some("a"));
    assert_eq!(arena.len(), 0);
    assert!(!arena.contains(id));
    assert_eq!(arena.get(id), None);
    assert_eq!(arena.remove(id), None);
  }

  /// The whole point of the generation: a handle into a reused slot does
  /// not silently address the new occupant.
  #[test]
  fn a_reused_slot_rejects_the_old_handle() {
    let mut arena = Arena::new();
    let stale = arena.insert("a");
    arena.remove(stale);

    let fresh = arena.insert("b");

    assert_eq!(fresh.index(), stale.index());
    assert_ne!(fresh.generation(), stale.generation());
    assert_eq!(arena.get(fresh), Some(&"b"));
    assert_eq!(arena.get(stale), None);
    assert!(!arena.contains(stale));
  }

  #[test]
  fn a_handle_past_the_end_is_rejected() {
    let mut arena = Arena::new();
    let id = arena.insert("a");
    let mut other: Arena<&str> = Arena::new();

    assert_eq!(other.get(id), None);
    assert_eq!(other.get_mut(id), None);
    assert_eq!(other.remove(id), None);
  }

  #[test]
  fn iteration_is_in_slot_index_order_after_removals() {
    let mut arena = Arena::new();
    let a = arena.insert("a");
    let b = arena.insert("b");
    let c = arena.insert("c");
    let d = arena.insert("d");

    arena.remove(b);
    arena.remove(d);

    assert_eq!(ids(&arena), vec![a.index(), c.index()]);
    assert_eq!(values(&arena), vec!["a", "c"]);
  }

  /// Reinsertion fills the freed slots, and iteration still runs by slot
  /// index rather than by insertion order.
  #[test]
  fn iteration_is_in_slot_index_order_after_reinsertions() {
    let mut arena = Arena::new();
    arena.insert("a");
    let b = arena.insert("b");
    arena.insert("c");
    let d = arena.insert("d");

    arena.remove(b);
    arena.remove(d);

    let first = arena.insert("e");
    let second = arena.insert("f");

    // The free list is a stack, so "d"'s slot is reused first.
    assert_eq!(first.index(), 3);
    assert_eq!(second.index(), 1);
    assert_eq!(values(&arena), vec!["a", "f", "c", "e"]);
    assert_eq!(ids(&arena), vec![0, 1, 2, 3]);
  }

  #[test]
  fn clone_is_independent() {
    let mut arena = Arena::new();
    let id = arena.insert("a");
    let mut copy = arena.clone();

    *copy.get_mut(id).unwrap() = "z";

    assert_eq!(arena.get(id), Some(&"a"));
    assert_eq!(copy.get(id), Some(&"z"));
  }

  /// The order is slot index first, generation second, which is what
  /// lets a handle key a `BTreeMap`.
  #[test]
  fn handles_order_by_index_then_generation() {
    let mut arena: Arena<&str> = Arena::new();
    let first = arena.insert("a");
    let second = arena.insert("b");

    assert!(first < second);

    arena.remove(first);
    let reused = arena.insert("c");

    assert_eq!(reused.index(), first.index());
    assert!(first < reused);
    assert!(reused < second);

    let mut sorted = vec![second, reused, first];
    sorted.sort_unstable();

    assert_eq!(sorted, vec![first, reused, second]);
  }

  #[test]
  fn handles_of_different_arenas_hash_and_compare_by_value() {
    let mut arena: Arena<&str> = Arena::new();
    let first = arena.insert("a");
    let second = arena.insert("b");

    assert_eq!(first, first);
    assert_ne!(first, second);
    assert_eq!(format!("{first:?}"), "ArenaId(0, 0)");
  }
}
