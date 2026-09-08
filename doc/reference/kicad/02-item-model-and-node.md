# KiCad PNS: item model and the branching "world" (NODE)

Reference architecture note for a from-scratch Rust reimplementation. Source: sparse checkout of KiCad master at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85` (2026-09-07), rooted at `/home/Tubbles/dev/ref/kicad/`. All `path:line` citations below are relative to that root. Line numbers are from that exact commit and will drift.

Sparse checkout caveat: only `libs/kimath`, `pcbnew/router`, `qa/*`, `thirdparty/clipper2` are present. `include/core/typeinfo.h` (which defines `dyn_cast`) and `include/board_item.h` are not on disk, so claims about those are inferred from call sites in the router and are flagged where they matter.

---

## 0. Ten second orientation

PNS keeps the board as a tree of `NODE`s. The root `NODE` holds the synced board. Every speculative operation (a shove pass, a walkaround attempt, a drag preview) allocates a child `NODE` via `Branch()` (`pcbnew/router/pns_node.cpp:157`) and mutates that. `Commit()` (`pcbnew/router/pns_node.cpp:1622`) folds one branch back into the root; `KillChildren()` (`pcbnew/router/pns_node.cpp:1647`) throws the whole speculation away.

The stored items are `SOLID`, `SEGMENT`, `ARC`, `VIA`, `HOLE`. `LINE` is not stored: it is a transient view assembled on demand from a run of `SEGMENT`s and `ARC`s (`pcbnew/router/pns_line.h:47-60`, `pcbnew/router/pns_node.cpp:1132`). `JOINT` is the connectivity node of the graph, keyed by (position, net) and carrying a layer range plus a list of incident items (`pcbnew/router/pns_joint.h:42-65`).

Every cross-object reference in this design is a raw pointer, and in several places the *identity* of that pointer (not the value it points at) is what makes the algorithm correct. Section 11 inventories those.

---

## 1. ITEM

### 1.1 Class shape and the double inheritance

`ITEM` derives from both `OWNABLE_ITEM` and `ITEM_OWNER` (`pcbnew/router/pns_item.h:97`). That is deliberate: an item can be owned (by a `NODE`, an `ITEM_SET`, a `LINE`, or a parent `VIA`/`SOLID`) and can itself own other items (a `VIA` owns its `HOLE`, a `LINE` owns its via copy).

`ITEM_OWNER` is an empty polymorphic base with only a virtual destructor (`pcbnew/router/pns_item.h:57-60`). `NODE` derives from it (`pcbnew/router/pns_node.h:241`), so does `ITEM_SET` (`pcbnew/router/pns_itemset.h:36`).

`OWNABLE_ITEM` holds a single `const ITEM_OWNER* m_owner` (`pcbnew/router/pns_item.h:88`) with `Owner()`, `SetOwner()`, `BelongsTo()` (`pcbnew/router/pns_item.h:72`, `:77`, `:82`). `BelongsTo` is pure pointer equality.

### 1.2 The kind enum and bitmask usage

```
INVALID_T = 0, SOLID_T = 1, LINE_T = 2, JOINT_T = 4, SEGMENT_T = 8,
ARC_T = 16, VIA_T = 32, DIFF_PAIR_T = 64, HOLE_T = 128, ANY_T = 0xffff,
LINKED_ITEM_MASK_T = SOLID_T | SEGMENT_T | ARC_T | VIA_T | HOLE_T
```

Declared at `pcbnew/router/pns_item.h:101-114`. The values are powers of two so `OfKind(int aKindMask)` is a bit test: `( aKindMask & m_kind ) != 0` (`pcbnew/router/pns_item.h:181-184`). `Kind()` returns the single-bit value (`pcbnew/router/pns_item.h:173`). `ANY_T` is `0xffff`, not `-1`, but callers pass `-1` interchangeably: `COLLISION_SEARCH_OPTIONS::m_kindMask` defaults to `-1` (`pcbnew/router/pns_node.h:119`) and `ITEM_SET::Count` special-cases both `-1` and `ANY_T` (`pcbnew/router/pns_itemset.h:74-77`).

Note `LINKED_ITEM_MASK_T` includes `SOLID_T` and `HOLE_T` even though neither `SOLID` nor `HOLE` derives from `LINKED_ITEM` (`pcbnew/router/pns_solid.h:36`, `pcbnew/router/pns_hole.h:33`). The mask is a naming leftover; do not use it to decide whether a `static_cast<LINKED_ITEM*>` is safe.

`KindStr()` maps the enum to a debug string (`pcbnew/router/pns_item.cpp:315-330`).

Runtime downcasts go through two mechanisms. `static_cast` guarded by a manual `Kind()` switch, as in `NODE::add` (`pcbnew/router/pns_node.cpp:658-680`) and `NODE::Remove(ITEM*)` (`pcbnew/router/pns_node.cpp:998-1051`). And KiCad's `dyn_cast<T*>`, which dispatches on a static `ClassOf(const ITEM*)` predicate that each concrete class defines: `LINE` (`pcbnew/router/pns_line.h:111`), `SEGMENT` (`pcbnew/router/pns_segment.h:79`), `ARC` (`pcbnew/router/pns_arc.h:71`), `VIA` (`pcbnew/router/pns_via.h:191`), `SOLID` (`pcbnew/router/pns_solid.h:100`), `DIFF_PAIR` (`pcbnew/router/pns_diff_pair.h:333`). `HOLE` and `JOINT` define no `ClassOf`, so they cannot be `dyn_cast` to. Real C++ `dynamic_cast` is used once in `followTrivialPath` (`pcbnew/router/pns_topology.cpp:420`, `:434`).

### 1.3 Ownership model

Three owners exist in practice, distinguished only by what `m_owner` points at.

**NODE owns stored items.** `addSolid`, `addSegment`, `addVia`, `addArc`, `addHole` all call `SetOwner(this)` before inserting into the index (`pcbnew/router/pns_node.cpp:612`, `:738`, `:636`, `:767`, `:647`). The public `Add()` overloads take `std::unique_ptr` and `release()` it (`pcbnew/router/pns_node.cpp:620`, `:654`, `:759`, `:786`). `~NODE` deletes every indexed item for which `item->BelongsTo( this )` (`pcbnew/router/pns_node.cpp:96-134`).

**LINE owns its optional trailing VIA, sometimes.** `AppendVia` clones the argument and takes ownership (`pcbnew/router/pns_line.cpp:1421-1423`); `LinkVia` stores the pointer without taking ownership and adds it to the link list (`pcbnew/router/pns_line.cpp:1434-1435`). `~LINE` deletes only if `m_via->BelongsTo( this )` (`pcbnew/router/pns_line.cpp:78-79`), and the copy constructor and both assignment operators re-clone the via only when the source owned it, otherwise they alias the same pointer (`pcbnew/router/pns_line.cpp:54-66`, `:96-108`, `:138-150`).

**VIA and SOLID own their HOLE.** `VIA::SetHole` deletes the previous hole if owned, then sets `m_hole->SetParentPadVia(this); m_hole->SetOwner(this)` (`pcbnew/router/pns_via.h:328-337`). `SOLID::SetHole` is the same shape (`pcbnew/router/pns_solid.h:141-151`). Both destructors delete the hole only when `BelongsTo(this)` (`pcbnew/router/pns_via.h:154-155`, `pcbnew/router/pns_solid.h:51-52`).

**ITEM_SET claims ownership but never releases it.** `ITEM_SET::Add(const LINE&)` clones the line and calls `copy->SetOwner( this )` (`pcbnew/router/pns_itemset.cpp:36-41`), but `ITEM_SET::~ITEM_SET()` has an empty body (`pcbnew/router/pns_itemset.cpp:31-33`). The `aBecomeOwner` overloads do the same (`pcbnew/router/pns_itemset.h:140-145`, `:151-157`). Nothing in the router frees those clones. In Rust this becomes a straightforward owned `Vec<Line>`; do not port the "owner pointer" indirection.

The ownership hand-off during removal is the subtle part; see section 4.5.

### 1.4 m_owner / m_parent / m_sourceItem

`m_parent` is a `BOARD_ITEM*` and means "there is a 1:1 mapping between this PNS item and that board item" (`pcbnew/router/pns_item.h:317-318`). `SetParent` also assigns `m_sourceItem` when non-null (`pcbnew/router/pns_item.h:191-197`).

`m_sourceItem` is the progenitor board item when the mapping is not 1:1, for instance when dragging one track produces several segments (`pcbnew/router/pns_item.h:319-322`). `SEGMENT( const LINE&, const SEG& )` sets `m_parent = nullptr` and inherits `m_sourceItem` from the line (`pcbnew/router/pns_segment.h:64-65`). `AssembleLine` does the mirror image: `pl.SetParent( nullptr ); pl.SetSourceItem( aSeg->GetSourceItem() )` (`pcbnew/router/pns_node.cpp:1150-1151`).

`BoardItem()` is virtual and normally returns `m_parent` (`pcbnew/router/pns_item.h:207`); `HOLE` overrides it to fall through to `m_parentPadVia->Parent()` (`pcbnew/router/pns_hole.h:74-83`). The rule resolver's net-tie and temp-clearance-cache logic keys on `BoardItem()` (`pcbnew/router/pns_kicad_iface.cpp:351`, `:169`).

`OwningNode()` is a convenience that returns the hole's parent's owner when the item is a hole, else its own owner (`pcbnew/router/pns_item.cpp:354-360`). It does an unchecked `static_cast<const NODE*>( Owner() )`. If the owner is actually an `ITEM_SET` or a `LINE`, that is undefined behaviour. Callers get away with it because they only call it on items known to be indexed (`pcbnew/router/pns_shove.cpp:1713`, `pcbnew/router/pns_walkaround.cpp:125`).

### 1.5 Marker flags

Declared as `enum LineMarker` at `pcbnew/router/pns_item.h:42-47`:

| Flag | Value | Meaning | Set at | Cleared at |
| --- | --- | --- | --- | --- |
| `MK_HEAD` | `1 << 0` | this line is the routing head | `pcbnew/router/pns_shove.cpp:853` only (propagation from an already-marked line); the one place that would seed it is commented out at `pcbnew/router/pns_shove.cpp:2500` | `NODE::ClearRanks` default mask (`pcbnew/router/pns_node.h:494`), `LINE::Unmark` (`pcbnew/router/pns_shove.cpp:842`) |
| `MK_VIOLATION` | `1 << 3` | DRC violation, for red preview rendering | `ROUTER::markViolations` (`pcbnew/router/pns_router.cpp:729`) | `NODE::ClearRanks` default mask |
| `MK_LOCKED` | `1 << 4` | item is user-locked or belongs to a non-editing generator | `PNS_KICAD_IFACE_BASE::syncTrack/syncArc/syncVia` (`pcbnew/router/pns_kicad_iface.cpp:1758`, `:1763`, `:1780`, `:1785`, `:1854`, `:1859`) | `DRAGGER::Start` clears it on the item being dragged (`pcbnew/router/pns_dragger.cpp:331`) |
| `MK_DP_COUPLED` | `1 << 5` | declared, never read or written anywhere in the tree | nowhere | nowhere |

Bits `1 << 1` and `1 << 2` are unused. There is **no `MK_HOLE`** in this revision; a grep over the whole checkout finds nothing.

`m_marker` is `mutable` (`pcbnew/router/pns_item.h:327`) so `Mark`/`Unmark` can be `const` (`pcbnew/router/pns_item.h:261-263`). This is a real "mutable shared state" hazard for Rust: collision search and preview code mark items through const references.

`LINE` overrides all three: `Mark` writes its own `m_marker` and forwards to every link (`pcbnew/router/pns_line.cpp:174-181`), `Unmark` forwards then zeroes its own (`:184-190`), and `Marker()` returns the bitwise OR of its own marker and every link's marker (`:193-201`). So a `LINE`'s marker is a *derived* value over the segments it views.

`IsLocked()` is `Marker() & MK_LOCKED` (`pcbnew/router/pns_item.h:278-281`); for a `LINE` that means "any constituent segment is locked". There is also `LINE::HasLockedSegments()` which tests only the links (`pcbnew/router/pns_line.cpp:1626-1634`).

`NODE::ClearRanks( int aMarkerMask = MK_HEAD | MK_VIOLATION )` resets `m_rank` to `-1` and clears the masked marker bits for every item in the *local* index only (`pcbnew/router/pns_node.cpp:1682-1689`). `NODE::RemoveByMarker` removes every locally indexed item carrying a marker bit (`:1692-1704`).

### 1.6 Rank

`m_rank` (`pcbnew/router/pns_item.h:328`, initialised to `-1` at `:125`) is the shove priority. `LINE::SetRank` fans out to the links (`pcbnew/router/pns_line.cpp:1439-1446`); `LINE::Rank()` returns the *minimum* rank over the links when linked, else its own (`pcbnew/router/pns_line.cpp:1449-1466`). `NODE::Commit` resets rank to `-1` on every migrated item (`pcbnew/router/pns_node.cpp:1637`).

### 1.7 Layers: PNS_LAYER_RANGE

`PNS_LAYER_RANGE` is a closed integer interval `[m_start, m_end]` over PNS layer indices, defined in `pcbnew/router/pns_layerset.h:31-147`. Key semantics for the port:

- Default construction is `(-1, -1)`, which is the *invalid/empty* range (`pcbnew/router/pns_layerset.h:34-37`).
- The two-argument constructor swaps if `aStart > aEnd` (`:41-42`), so the invariant `start <= end` always holds.
- `Overlaps` returns **false** if either side has a negative endpoint (`:67-79`). This is what makes a dangling `JOINT` (whose layers get reset to `PNS_LAYER_RANGE(-1)` on the last unlink, `pcbnew/router/pns_joint.h:229`) invisible to `FindJoint`.
- `Merge` treats an invalid range as absorbing: merging into `(-1,-1)` just copies (`:96-110`). Otherwise it is a hull, not a union, so a merged range can cover layers neither input had.
- `Intersection` has an asymmetric special case for `m_end < 0` (`:112-126`) that yields the *other* range's end rather than an empty result.
- `All()` is hardcoded as `PNS_LAYER_RANGE( 0, 256 )` with a "fixme: use layer IDs header" comment (`:129-132`).

`ITEM::Layer()` is `Layers().Start()` (`pcbnew/router/pns_item.h:216`), used as "the" layer for single-layer items. `SetLayer` collapses to a one-element range (`:215`).

Because `Overlaps` is interval overlap, PNS cannot express a via that skips a middle layer. Blind and buried vias are a contiguous range; `VIA::ConnectsLayer` adds a `START_END_ONLY` mode on top for the unconnected-layer removal feature (`pcbnew/router/pns_via.cpp:78-84`), but the *collision* layers stay contiguous.

### 1.8 Net handle

`typedef void* NET_HANDLE` (`pcbnew/router/pns_item.h:55`), described as "an opaque net identifier, the internal workings are owned by the ROUTER_IFACE". In the KiCad host it is a `NETINFO_ITEM*`. Nothing in the PNS core dereferences it. It is compared for equality (`pcbnew/router/pns_item.cpp:188`), hashed as a pointer (`pcbnew/router/pns_joint.h:63`), used as a `std::map` key (`pcbnew/router/pns_index.h:155`), and turned into a `NetCode`/`NetName` only through the resolver (`pcbnew/router/pns_node.h:151-152`).

`nullptr` means "no net". `INDEX::Add` skips the net map entirely for a null net (`pcbnew/router/pns_index.cpp:39-41`); see section 6.3.

`HOLE::Net()` is virtual and forwards to the parent pad or via if there is one (`pcbnew/router/pns_hole.h:56-62`). `JOINT::Net()` is virtual and returns `m_tag.net` rather than `ITEM::m_net` (`pcbnew/router/pns_joint.h:298-301`).

### 1.9 Collide() and collideSimple()

`ITEM::Collide` is a thin wrapper (`pcbnew/router/pns_item.cpp:305-312`); all the work is in the private `collideSimple` (`pcbnew/router/pns_item.cpp:104-302`). Signature: `collideSimple( const ITEM* aHead, const NODE* aNode, int aLayer, COLLISION_SEARCH_CONTEXT* aCtx )`. Read `this` as the candidate obstacle and `aHead` as the thing being routed.

Pseudo-code (faithful to `pcbnew/router/pns_item.cpp:104-302`):

```
collideSimple(self, head, node, layer, ctx):
    if self is head: return false                                  # :119
    if not shouldWeConsiderHoleCollisions(self, head): return false # :122

    runPhysicalOnly = node.resolver.HasUserDefinedPhysicalConstraint()   # :127

    # a LINE with a trailing via collides through its via too
    if self is LINE and self.EndsWithVia():  found |= self.Via().collideSimple(head, ...)  # :133-137
    if head is LINE and head.EndsWithVia():  found |= head.Via().collideSimple(self, ...)  # :139-143

    # recurse into holes
    if head.HasHole() and shouldWeConsiderHoleCollisions(self, head.Hole()):   # :146
        if self.Kind()==HOLE_T or self.Net()!=head.Hole().Net() or runPhysicalOnly:  # :150
            found |= collideSimple(self, head.Hole(), ...)
    if self.HasHole() and shouldWeConsiderHoleCollisions(self.Hole(), head):   # :154
        found |= self.Hole().collideSimple(head, ...)

    lineWidthI = self is LINE ? self.Width()/2 : 0        # :161-162
    lineWidthH = head is LINE ? head.Width()/2 : 0        # :164-165

    if not self.Layers().Overlaps(head.Layers()): return false        # :168

    differentNetsOnly = ctx ? ctx.options.m_differentNetsOnly : true  # :178
    if self.Kind()==HOLE_T and head.Kind()==HOLE_T: differentNetsOnly = false   # :182

    # ---- clearance resolution ladder, first match wins ----
    if differentNetsOnly and self.Net()==head.Net() and head.Net() and not runPhysicalOnly:
        clearance = -1                                    # same net, skip            :188
    elif differentNetsOnly and (self.IsFreePad() or head.IsFreePad()) and not runPhysicalOnly:
        clearance = -1                                    # free pad (NIC), skip      :193
    elif resolver.IsKeepout(self, head, &enforce) or resolver.IsKeepout(head, self, &enforce):
        clearance = enforce ? 0 : -1                      # keepout is exact boundary :198-205
    elif iface and not iface.IsFlashedOnLayer(self, head.Layers()): clearance = -1    # :206
    elif iface and not iface.IsFlashedOnLayer(head, self.Layers()): clearance = -1    # :210
    elif ctx and ctx.options.m_overrideClearance >= 0:
        clearance = ctx.options.m_overrideClearance                                    # :214
    else:
        clearance = node.GetClearance(self, head, ctx ? ctx.options.m_useClearanceEpsilon : false)  # :220

    if clearance < 0: return found                        # -1 means "never collides"

    checkCastellation = (self.m_parent and self.m_parent.GetLayer()==Edge_Cuts)
                        or resolver.IsNonPlatedSlot(self)                              # :229
    checkNetTie       = resolver.IsInNetTie(self)                                      # :232

    shapeI = self.Shape(layer); shapeH = head.Shape(layer)
    if not shapeI or not shapeH: return false                                          # :237

    d = clearance + lineWidthH + lineWidthI - 1           # the -1, see below

    if checkCastellation or checkNetTie:                  # slow path, needs the position
        if shapeH.Collide(shapeI, d, &actual, &pos):                                   # :249
            if checkCastellation and node.QueryEdgeExclusions(pos): return false       # :251
            if checkNetTie and resolver.IsNetTieExclusion(head, pos, self): return false # :254
            record_or_return()
    else:                                                 # fast path
        if shapeH.Collide(shapeI, d): record_or_return()                               # :280

    record_or_return():
        if ctx: ctx.obstacles.insert(OBSTACLE{ m_head=head, m_item=self,
                                               m_clearance=clearance,
                                               m_distFirst=0, m_maxFanoutWidth=0 })    # :260-266, :282-291
             ; found = true
        else:   return true
```

The `- 1` at `pcbnew/router/pns_item.cpp:249` and `:280` is documented in place: "the hulls are built to exactly the clearance distance, so we need to allow for no collision when exactly at the clearance distance". Both the hull builders and the collision test must agree on this or the walkaround will oscillate. Preserve it exactly.

`aLayer` is the layer context, threaded down from the R-tree sub-index that produced the candidate (`pcbnew/router/pns_index.h:167`, consumed at `pcbnew/router/pns_node.cpp:256` via `m_layerContext.value_or(-1)`). `-1` means "the item has only one shape". `VIA::Shape(aLayer)` maps through `EffectiveLayer` (`pcbnew/router/pns_via.h:302-307`, `pcbnew/router/pns_via.cpp:33-53`).

Two exits matter for the port. Without a `ctx`, `collideSimple` returns eagerly on the first hit. With a `ctx`, it never returns early: it accumulates every obstacle into `ctx->obstacles` and returns whether anything was found. The `m_limitCount` cut-off lives one level up in the visitor (`pcbnew/router/pns_node.cpp:259-260`).

### 1.10 Hole handling

`HOLE` is a first-class `ITEM` (`pcbnew/router/pns_hole.h:33`) that carries a `SHAPE*` and a back-pointer `ITEM* m_parentPadVia` (`pcbnew/router/pns_hole.h:94-95`). `VIA::HasHole()` is unconditionally true (`pcbnew/router/pns_via.h:339`), `SOLID::HasHole()` tests for null (`pcbnew/router/pns_solid.h:153`), `VVIA::HasHole()` is overridden to false (`pcbnew/router/pns_via.h:376`) even though the base constructor still built one.

Holes are inserted into the spatial index as separate entries by `addHole`, called from `addSolid` and `addVia` (`pcbnew/router/pns_node.cpp:603-607`, `:626-633`, `:642-649`). Connectivity linking for holes is deliberately commented out (`pcbnew/router/pns_node.cpp:644-645`), so a `HOLE` never appears in a `JOINT`.

`shouldWeConsiderHoleCollisions( aItem, aHead )` (`pcbnew/router/pns_item.cpp:38-80`) prunes self-collisions:

- hole vs hole: false if the two parents are the same object, and also false if the two parent vias are geometrically identical, same position, same padstack, same net, same drill (`pcbnew/router/pns_item.cpp:65-69`). The long comment at `:54-63` explains why: a `LINE` carries a *copy* of its via, so checking a `LINE` against a `NODE` that already contains that same via would otherwise self-collide. This is an explicit acknowledgement that `LINE` does not manage via ownership properly.
- hole vs non-hole: false when the non-hole item *is* the hole's parent (`:74-77`).

Because holes live in the index independently, the collision recursion in `collideSimple` at `:146-157` exists only for the *head* side (an item under construction, not yet indexed) and for the head's hole. The comment at `:107-109` states this.

`~NODE` has to special-case holes: a hole whose `ParentPadVia()` is set is not deleted directly because "we will encounter its parent later, disguised as VIA or SOLID" (`pcbnew/router/pns_node.cpp:100-117`). There is an assertion that a hole is always owned by the same `NODE` as its parent (`:107`) with the comment "If a hole is no longer owned by the same NODE as its parent then we're in a heap of trouble."

### 1.11 Shape() and Hull() contracts

`Shape( int aLayer )` returns a borrowed `const SHAPE*` owned by the item, or nullptr (`pcbnew/router/pns_item.h:242-245`). Implementations: `LINE` returns `&m_line` and ignores the layer (`pcbnew/router/pns_line.h:138`), `SEGMENT` returns `&m_seg` (`pcbnew/router/pns_segment.h:86-89`), `ARC` returns `&m_arc` (`pcbnew/router/pns_arc.h:78-81`), `SOLID` returns `m_shape` which may be null (`pcbnew/router/pns_solid.h:107`), `HOLE` returns `m_holeShape` (`pcbnew/router/pns_hole.h:69`), `VIA` looks up `m_shapes.at( EffectiveLayer( aLayer ) )` and returns nullptr through `wxCHECK` if absent (`pcbnew/router/pns_via.h:302-307`).

`UniqueShapeLayers()` returns `{ -1 }` by default (`pcbnew/router/pns_item.h:250`); only `VIA` overrides it, returning `{ALL_LAYERS}`, `{ALL_LAYERS, INNER_LAYERS, m_layers.End()}`, or one entry per layer depending on stack mode (`pcbnew/router/pns_via.cpp:56-75`). `HasUniqueShapeLayers()` is false except for `VIA` (`pcbnew/router/pns_item.h:252`, `pcbnew/router/pns_via.h:204`). `RelevantShapeLayers( other )` short-circuits to `{ -1 }` when neither side has per-layer shapes, else returns the set union (`pcbnew/router/pns_item.cpp:83-101`); the TODO at `:91-93` notes it over-tests when a via meets a track.

`Hull( aClearance, aWalkaroundThickness, aLayer )` returns a **closed, convex, clockwise** `SHAPE_LINE_CHAIN` by value (`pcbnew/router/pns_item.h:164-168`, default empty). The clockwise assumption is stated in `LINE::Walkaround`: "we assume the default orientation of the hulls is clockwise, so just reverse the vertex order if the caller wants a counter-clockwise walkaround" (`pcbnew/router/pns_line.cpp:397-400`).

The shared builders live in `pcbnew/router/pns_utils.cpp`. `OctagonalHull( p0, size, clearance, chamfer )` emits a closed 4- or 8-gon (`pcbnew/router/pns_utils.cpp:40-68`). The octagon chamfer used by round things is `( 2*cl + width ) * ( 1 - M_SQRT1_2 )`, the equilateral-octagon formula, used identically in `VIA::Hull` (`pcbnew/router/pns_via.cpp:246-249`) and `HOLE::Hull` (`pcbnew/router/pns_hole.cpp:68-71`).

`SEGMENT::Hull` delegates to `SegmentHull` (`pcbnew/router/pns_line.cpp:668-677`, builder at `pcbnew/router/pns_utils.cpp:181`). `ARC::Hull` delegates to `ArcHull` (`pcbnew/router/pns_arc.cpp:28-31`, builder at `pcbnew/router/pns_utils.cpp:71`). `SOLID::Hull` handles compound shapes by unioning per-primitive hulls through a `SHAPE_POLY_SET` and taking outline 0 (`pcbnew/router/pns_solid.cpp:39-71`); `HOLE::Hull` does the same (`pcbnew/router/pns_hole.cpp:57-100`). `VIA::Hull` substitutes the hole diameter for the pad diameter when the via is not flashed on that layer (`pcbnew/router/pns_via.cpp:243-244`), and asserts that a complex viastack is never queried with `aLayer < 0` (`:237-238`).

`SegmentHull` contains a set of numerical robustness hacks for near-degenerate segments, keyed on `kinkThreshold = aClearance / 10` (`pcbnew/router/pns_utils.cpp:184`): almost-vertical, almost-horizontal and almost-45 segments get snapped and the clearance bumped by 1 or 2 (`:220-247`). A zero-length segment falls back to an octagon (`:250-260`). Any reimplementation that skips these will produce non-convex or self-intersecting hulls on short segments and the walkaround graph will fail.

`Hull` is called through `RULE_RESOLVER::HullCache` in every hot path, never directly; see section 8.3.

### 1.12 Clone semantics

`ITEM::Clone()` is pure virtual (`pcbnew/router/pns_item.h:154`), and the free function template `PNS::Clone(const T&)` wraps it in a `unique_ptr` (`pcbnew/router/pns_item.h:343-348`). `ItemCast<T>(unique_ptr<S>)` is an unchecked `static_cast` on the released pointer (`pcbnew/router/pns_item.h:335-341`), used to widen `VVIA` to `VIA` (`pcbnew/router/pns_node.cpp:1354`).

Common to all: `ITEM`'s copy constructor copies everything except `m_owner`, which it forces to `nullptr` (`pcbnew/router/pns_item.h:132-147`, specifically `:140`). A clone is therefore always unowned and must be adopted.

Per class:

- `SEGMENT::Clone` uses the copy constructor then re-copies fields (`pcbnew/router/pns_line.cpp:204-215`). Because it goes through `LINKED_ITEM`'s copy constructor (`pcbnew/router/pns_linked_item.h:41-43`), it **keeps the source `m_uid`**.
- `ARC::Clone` constructs a fresh `ARC( m_arc, m_net )` (`pcbnew/router/pns_arc.cpp:34-47`), which runs `LINKED_ITEM( ARC_T )` and therefore **allocates a new `m_uid`**. This asymmetry with `SEGMENT` is a real behavioural difference.
- `VIA::Clone` explicitly copies `m_uid` with the comment "fixme: oop" (`pcbnew/router/pns_via.cpp:257`) and builds a *fresh* hole via `MakeCircularHole` rather than cloning the source's hole (`:278`). It is a hand-written field-by-field copy, so it silently omits whatever the author forgot: `m_isFree` (the via-specific flag) is copied at `:284`, but `ITEM::m_isFreePad` and `ITEM::m_isCompoundShapePrimitive` are not.
- `SOLID::Clone` is `new SOLID( *this )` (`pcbnew/router/pns_solid.cpp:74-78`), and `SOLID`'s copy constructor deep-clones both the shape and the hole (`pcbnew/router/pns_solid.h:62-66`).
- `HOLE::Clone` clones the shape, copies layers, rank, marker, parent and virtual flag, and explicitly sets owner to null; it does **not** propagate `m_parentPadVia` (`pcbnew/router/pns_hole.cpp:41-54`).
- `LINE::Clone` is `new LINE( *this )` (`pcbnew/router/pns_line.cpp:166-171`), and the copy constructor copies the link vector wholesale via `copyLinks` (`pcbnew/router/pns_line.cpp:72`) while conditionally cloning the via as described in 1.3.
- `JOINT::Clone` asserts and returns nullptr (`pcbnew/router/pns_joint.h:90-94`). Joints are copied by value through the map, never cloned.

The `LINE` copy semantics are the trap: a cloned `LINE` shares the *same* `LINKED_ITEM*` link pointers as the original, pointing into whichever `NODE` owns them. Two `LINE` values can therefore be simultaneously live views of the same segments.

`LINKED_ITEM::UNIQ_ID` is a `uint64_t` from a non-atomic function-local static counter, with a "fixme: make atomic" comment (`pcbnew/router/pns_item.cpp:362-366`, declared `pcbnew/router/pns_linked_item.h:33`, `:62`). `ResetUid()` exists (`pcbnew/router/pns_linked_item.h:46-49`). The uid is used by the shove algorithm to correlate an item across branches without pointer identity.

---

## 2. LINE

### 2.1 What a LINE is

A `LINE` is "a track on a PCB, connecting two non-trivial joints (that is, vias, pads, junctions between multiple traces or two traces different widths and combinations of these). PNS_LINEs are NOT stored in the model (NODE). Instead, they are assembled on-the-fly" (`pcbnew/router/pns_line.h:47-56`).

It holds:

- `SHAPE_LINE_CHAIN m_line`, the actual geometry, which may contain arcs (`pcbnew/router/pns_line.h:281`)
- `int m_width` (`:282`)
- `int m_snapThreshhold` (`:285`)
- `VIA* m_via`, optionally owned (`:287`)
- `ITEM* m_blockingObstacle` for mark-obstacle mode (`:288`)
- inherited from `LINK_HOLDER`: `std::vector<LINKED_ITEM*> m_links` (`pcbnew/router/pns_link_holder.h:125`)

`LINE` derives from `LINK_HOLDER` (`pcbnew/router/pns_line.h:61`), which derives from `ITEM` (`pcbnew/router/pns_link_holder.h:38`). `DIFF_PAIR` is the other `LINK_HOLDER`.

### 2.2 LINK_HOLDER and LINKED_ITEM

`LINKED_ITEM` (`pcbnew/router/pns_linked_item.h:29-64`) is the base for anything that can be a member of a `LINE`: it adds `m_uid` and the virtual `Width()`/`SetWidth()` pair (`:53-58`). Concrete subclasses are `SEGMENT`, `ARC`, `VIA`.

`LINK_HOLDER` (`pcbnew/router/pns_link_holder.h:38-127`) is the owner of the link vector:

| Method | Line | Semantics |
| --- | --- | --- |
| `Link(LINKED_ITEM*)` | `:46` | append if not already present; logs a debug warning on duplicate |
| `Unlink(const LINKED_ITEM*)` | `:57` | `std::erase`, with a `wxCHECK_MSG` guard |
| `Links()` | `:66`, `:67` | mutable and const accessors to the raw vector |
| `IsLinked()` | `:69` | vector non-empty |
| `ContainsLink()` | `:75` | linear search |
| `GetLink(int)` | `:80` | negative index wraps from the end |
| `ClearLinks()` | `:89` | virtual; clears the vector without touching the items |
| `LinkCount()` | `:95` | vector size |
| `ShowLinks()` | `:100` | debug dump; whole body is `#if 0` |
| `copyLinks(const LINK_HOLDER*)` | `:118` | protected, plain vector copy |

`LINE` re-declares `void ShowLinks() const;` at `pcbnew/router/pns_line.h:193` and **never defines it** anywhere in the tree. Calling it would be a link error. Treat both `ShowLinks` overloads as dead code.

`LINE::IsLinkedChecked()` is `IsLinked() && LinkCount() == ShapeCount()` (`pcbnew/router/pns_line.h:125-128`), the intended consistency invariant: one link per shape, where a shape is a segment or a whole arc (`SHAPE_LINE_CHAIN::ShapeCount()`).

### 2.3 What "links" are and when they are valid

A link is a raw `LINKED_ITEM*` into some `NODE`'s index. Links are valid only while (a) that `NODE` is alive and (b) those items have not been removed from it. There is no back-pointer from the item to the lines that reference it, and no invalidation mechanism.

Links are set in exactly two places:

- `NODE::AssembleLine` calls `pl.Link( li )` once per distinct traversed item (`pcbnew/router/pns_node.cpp:1188`).
- `NODE::Add( LINE& )` links each newly created or reused `SEGMENT`/`ARC` back into the caller's line (`pcbnew/router/pns_node.cpp:697`, `:702`, `:723`, `:728`). It asserts `!aLine.IsLinked()` on entry (`:685`).

Links are cleared by `NODE::Remove( LINE& )` (`pcbnew/router/pns_node.cpp:1069-1070`), `LINE::Clear()` (`pcbnew/router/pns_line.cpp:1637-1642`), and directly by callers such as `TOPOLOGY::NearestUnconnectedAnchorPoint` (`pcbnew/router/pns_topology.cpp:118`).

An important asymmetry: `NODE::Add( LINE& )` does **not** add `m_via`. Only the segments and arcs are added. Placing a via at the end of a line is a separate `Add( unique_ptr<VIA> )` plus a `LinkVia` (see `pcbnew/router/pns_shove.cpp:2503-2509`). But `NODE::Remove( LINE& )` *does* handle `VIA_T` links (`pcbnew/router/pns_node.cpp:1065-1066`), so a via that was `LinkVia`d is removed with the line.

`NODE::Add( LINE& )` also reuses existing geometry: `findRedundantArc` / `findRedundantSegment` look for an identical item already at those coordinates on that layer and net, and link that instead of creating a new one (`pcbnew/router/pns_node.cpp:694-698`, `:718-724`). The comment "another line could be referencing this segment too :(" at `:721` is the aliasing warning.

### 2.4 SegmentCount vs PointCount vs ShapeCount vs LinkCount

All four are distinct and all four are used (`pcbnew/router/pns_line.h:144-147`):

- `PointCount()` is the number of vertices in the chain, and an arc contributes many vertices.
- `SegmentCount()` is `PointCount() - 1` for an open chain.
- `ArcCount()` is the number of arcs.
- `ShapeCount()` counts straight segments plus whole arcs, so it is the count that matches `LinkCount()`.

`AssembleLine`'s `aOriginSegmentIndex` output is a *point* index, `line.PointCount() - 1` at the moment the origin item is appended (`pcbnew/router/pns_node.cpp:1195`), then clamped to `pl.SegmentCount() - 1` afterwards (`:1207-1208`) with a TODO admitting the index is not maintained under simplification.

### 2.5 AssembleLine and followLine

See section 4.8. From `LINE`'s point of view: `AssembleLine` returns a `LINE` by value whose `m_owner` is set to the assembling `NODE` (`pcbnew/router/pns_node.cpp:1152`) even though the `NODE` does not own or free it. `OwningNode()` on such a line therefore names the node the links point into, which is the actual purpose of that `SetOwner` call.

### 2.6 ClipToNearestObstacle

`pcbnew/router/pns_line.cpp:679-718`. Iterates at most `IterationLimit = 5` times (`:681`):

```
l = copy of self
repeat up to 5 times:
    obs = node.NearestObstacle(&l)
    if not obs: break
    l.RemoveVia()                                   # :691
    segIdx = l.Line().NearestSegment(obs.m_ipFirst)
    if l.Line().IsArcSegment(segIdx):
        l.Line().Clear()                            # refuse to clip inside an arc, :698
    else:
        nearestPt = l.Line().CSegment(segIdx).NearestPoint(obs.m_ipFirst)
        p = l.Line().Split(nearestPt)
        l.Line().Remove(p+1, -1)                    # truncate after the split, :705
if the loop ran out of iterations: l.Line().Clear() # :714-715
return l
```

Note the returned `LINE` keeps the original's `m_links` (it was copy-constructed) but the geometry has been truncated, so `IsLinkedChecked()` no longer holds. Callers must not feed it back to `NODE::Remove`.

### 2.7 Walkaround

`LINE::Walkaround( const SHAPE_LINE_CHAIN& aObstacle, SHAPE_LINE_CHAIN& aPath, bool aCw )` at `pcbnew/router/pns_line.cpp:297-665`. There is also a four-output overload declared at `pcbnew/router/pns_line.h:187-188` (pre/walk/post split) that is not defined in this file.

It builds a small directed graph over the union of the path's vertices and the hull's vertices, then searches it:

```
if line.SegmentCount() < 1: return false                                   # :301
if line.CPoint(0) is strictly inside the hull: return false                # :308-315
   ("We can't really walk around if the beginning of the path lies inside the obstacle hull")

ips = HullIntersection(aObstacle, line)                                    # :341
pnew = copy of the path ; hnew = copy of the hull                          # :343
if pnew self-intersects: split it at the self-intersection                 # :360-364
for each intersection: split it into both pnew and hnew                    # :367-374
for each pnew vertex lying on the hull edge: split hnew there              # :376-388
if not aCw: hnew = hnew.Reverse()                                          # :399-400

vts.reserve( 2 * (hnew.PointCount() + pnew.PointCount()) )                 # :402  <-- required
for each pnew vertex: push VERTEX{type = INSIDE|ON_EDGE|OUTSIDE, isHull=false, indexp=i}  # :405-422
link each path vertex to its successor and predecessor                     # :430-439
for each hnew vertex: reuse an existing coincident VERTEX (mark isHull) or push a new one  # :442-463
link each hull vertex to the next hull vertex                              # :466-473
... BFS from vertex 0, preferring to leave the path and follow the hull when inside ...
out.Simplify2(false)                                                       # :660
restoreUntouchedArcs(out, pnew)                                            # :661
aPath = out ; return true
```

The `vts.reserve` at `:402` is essential for memory safety, not performance: `VERTEX::neighbours` is a `std::vector<VERTEX*>` pointing into `vts` itself (`pcbnew/router/pns_line.cpp:330`), so any reallocation would dangle every neighbour pointer.

`restoreUntouchedArcs` (`pcbnew/router/pns_line.cpp:255-294`) re-splices arc data back into the output: the graph search only carries vertices, so arcs are recovered by finding the common prefix and suffix between the input and the output and slicing the original chain over those ranges.

`WALKAROUND::processCluster` is the main caller (`pcbnew/router/pns_walkaround.cpp:131-200`); it fetches the hull through `HullCache` with `aLine.Width()` as the walkaround thickness (`:156-158`) and optionally reduces the hull to its bounding box for the 90-degree corner modes (`:162-169`).

### 2.8 Reverse, ClipVertexRange, CountCorners, HasLoops

`Reverse()` reverses both the chain and the link vector, keeping them in correspondence (`pcbnew/router/pns_line.cpp:1406-1411`).

`ClipVertexRange( aStart, aEnd )` slices the chain and then rotates and truncates the link vector to the matching sub-range (`pcbnew/router/pns_line.cpp:1469-1513`). It walks shapes via `SHAPE_LINE_CHAIN::NextShape` to map vertex indices to link indices (`:1481-1493`). The doc comment states the precondition: "It is assumed that anything calling this method will have determined the vertex range to clip based on joints, meaning we will never clip in the middle of an arc" (`:1471-1476`).

`CountCorners( int aAngles )` counts consecutive segment pairs whose `DIRECTION_45::Angle` matches the mask (`pcbnew/router/pns_line.cpp:218-237`).

`HasLoops()` is an O(n^2) scan for repeated vertices at distance >= 2 (`pcbnew/router/pns_line.cpp:1516-1528`).

`CompareGeometry` delegates to the chain (`pcbnew/router/pns_line.cpp:1400-1403`). `FindSegment( const SEGMENT* )` finds the chain index whose `SEG` equals the segment's, ignoring links entirely (`pcbnew/router/pns_line.cpp:1668-1678`).

`ChangedArea( const LINE* )` returns the bounding box of the symmetric difference region between two lines (`pcbnew/router/pns_line.cpp:1545`), used to bound redraws.

There is **no `LINE::Merge`**. The head/tail merge in the placer is `LINE_PLACER::mergeHead()` (`pcbnew/router/pns_line_placer.h:317`), which operates on the placer's `m_head`/`m_tail` pair, not on `LINE` itself.

### 2.9 DragCorner, DragSegment, DragArc

`DragCorner( aP, aIndex, aFreeAngle, aPreferredEndingDirection )` dispatches to `dragCornerFree` or `dragCorner45` (`pcbnew/router/pns_line.cpp:884-896`), with a `wxCHECK_RET` that the index is non-negative.

`dragCornerFree` (`:857-882`) just moves the vertex, but first inserts a duplicate vertex if the target is on an arc, so the arc is not deformed; it asserts if asked to drag a point in the *middle* of an arc (`:876`).

`dragCorner45` (`:823-854`) snaps the target through `snapDraggedCorner`, then rebuilds the 45-degree trace. Three cases: dragging vertex 0 (reverse, rebuild, reverse back), dragging the last vertex (rebuild forward), or dragging an interior vertex (rebuild both halves from the dragged point and concatenate). The rebuild helper is the file-local `dragCornerInternal` (`:722-820`), which tries both `BuildInitialTrace` variants from progressively earlier anchor segments and picks the first that either matches the preferred ending direction, matches the starting direction, or is obtuse with respect to the previous segment.

`DragSegment( aP, aIndex, aFreeAngle )` asserts false for the free-angle case and otherwise calls `dragSegment45` (`:898-908`). `dragSegment45` (`:1230-1397`) snaps through `snapToNeighbourSegments`, then inserts zero-length padding segments at the ends or next to arcs so that a valid previous and next segment always exist (`:1242-1250`), computes guide lines, and re-solves.

`DragArc( aP, aIndex )` finds the arc index at that vertex and drags the whole arc (`:911-...`).

All drag operations rewrite `m_line` in place and leave `m_links` untouched, so a dragged line is immediately out of sync with the node. That is fine because the caller then does `Remove(old); Add(new)`.

### 2.10 Width and snapping

`SetWidth` writes both `m_width` and the chain's width (`pcbnew/router/pns_line.h:155-159`); `SetShape` re-applies `m_width` to the incoming chain (`:131-135`). The default width is the dummy `1` (`:71`).

`m_snapThreshhold` (note the spelling) gates two heuristics that keep dragged geometry from producing jagged micro-segments:

- `snapDraggedCorner` (`pcbnew/router/pns_line.cpp:1143-1183`) looks at segments in the window `[aIndex-2, aIndex+2]`, and for each obtuse pair computes the intersection of their infinite lines; if that point is within `m_snapThreshhold` of the drag target it snaps there. Returns `aP` unchanged when the threshold is <= 0 (`:1153-1154`).
- `snapToNeighbourSegments` (`:1185-1228`) looks specifically at segments `aIndex-2` and `aIndex+2`; if either is parallel to the dragged segment and within the threshold, it snaps onto that segment's `A` point. Returns `aP` unchanged when the threshold is exactly 0 (`:1192-1193`).

The threshold is set from the caller via `SetSnapThreshhold` (`pcbnew/router/pns_line.h:252-255`) and is propagated by every `LINE` copy path.

### 2.11 The via at the end

`EndsWithVia()` is `m_via != nullptr` (`pcbnew/router/pns_line.h:195`). `Via()` returns `*m_via` and will dereference null if the caller did not check (`:203-204`).

`AppendVia( const VIA& )` reverses the line first if the via sits at point 0, so the via is always at the *last* point, then clones and adopts (`pcbnew/router/pns_line.cpp:1414-1424`). `LinkVia( VIA* )` does the same reversal, then aliases and registers the via as a link (`:1427-1436`). `RemoveVia()` unlinks if linked, deletes if owned, and nulls (`:1645-1656`).

Semantically the via is *part of the line's collision footprint*: `collideSimple` explicitly recurses into `line->Via()` on both sides (`pcbnew/router/pns_item.cpp:133-143`), `NODE::NearestObstacle` adds a separate query for it (`pcbnew/router/pns_node.cpp:314-315`) and builds a separate via hull per obstacle (`:367-375`), and `NODE::CheckColliding` does the same (`:525-531`). The comment at `pcbnew/router/pns_item.cpp:129-131` notes the limitation: head-via to head-via collisions are not supported, on the grounds that you cannot route two independent tracks at once.

`SetViaDiameter` forces a complex viastack down to `STACK_MODE::NORMAL` with a warning (`pcbnew/router/pns_line.h:206-214`).

`LINE( VIA* aVia )` builds a degenerate `LINE` wrapping a lone stitching via, taking the via's diameter as the width (`pcbnew/router/pns_line.h:96-107`). Note this constructor stores the pointer without setting ownership, so the caller keeps it.

---

## 3. NODE

### 3.1 The branching world model

`NODE` is documented as: "Keep the router world, i.e. all the tracks, vias, solids in a hierarchical and indexed way. Features: spatial-indexed container for PCB item shapes; collision search and clearance checking; assembly of lines connecting joints, finding loops and unique paths; lightweight cloning/branching (for recursive optimization and shove springback)" (`pcbnew/router/pns_node.h:232-240`).

State (`pcbnew/router/pns_node.h:592-610`):

```
JOINT_MAP                 m_joints;          # unordered_multimap<HASH_TAG, JOINT, JOINT_TAG_HASH>
NODE*                     m_parent;          # node this was branched from, null for root
NODE*                     m_root;            # root of the whole hierarchy (self for root)
std::set<NODE*>           m_children;
std::unordered_set<ITEM*> m_override;        # root items shadowed/removed by this branch
int                       m_maxClearance;
RULE_RESOLVER*            m_ruleResolver;    # borrowed
INDEX*                    m_index;           # owned, raw
int                       m_depth;
std::vector<unique_ptr<SHAPE>> m_edgeExclusions;
std::unordered_set<ITEM*> m_garbageItems;    # meaningful only on the root
```

`isRoot()` is `m_parent == nullptr` (`pcbnew/router/pns_node.h:568-571`). `Depth()` returns `m_depth`, the number of ancestors (`pcbnew/router/pns_node.h:303-306`, assigned at `pcbnew/router/pns_node.cpp:163`). `Depth()` is used to decide whether an end item is still reachable in the current branch (`pcbnew/router/pns_line_placer.cpp:1488`, `:1539`) and for debug output (`pcbnew/router/pns_shove.cpp:1031`).

`NODE` is explicitly non-copyable: the copy constructor and assignment operator are declared private and undefined (`pcbnew/router/pns_node.h:536-537`).

### 3.2 Branch()

`pcbnew/router/pns_node.cpp:157-188`:

```
Branch():
    child = new NODE                      # fresh empty index, empty joints, empty override
    m_children.insert(child)
    child.m_depth        = m_depth + 1
    child.m_parent       = this
    child.m_ruleResolver = m_ruleResolver
    child.m_root         = isRoot() ? this : m_root
    child.m_maxClearance = m_maxClearance

    if not isRoot():                      # :171
        child.m_index    = m_index->Clone()   # O(1) CoW R-trees + copied metadata
        child.m_joints   = m_joints           # full multimap copy
        child.m_override = m_override         # full set copy
    return child
```

The comment at `:169-170` states the rule: "Immediate offspring of the root branch needs not copy anything. For the rest, clone the spatial index, joints, and overridden item maps."

This yields the **two-level query invariant** that every read path depends on: a node's own state plus the root's state is the complete world. A depth-1 branch is empty and defers to the root. A depth-N branch inherited everything from its depth-(N-1) parent (which had already inherited from its own parent, and so on down to depth 1) so it too only needs itself plus the root.

The header warns: "If there are any branches in use, their parents must **not** be deleted" (`pcbnew/router/pns_node.h:420`). `~NODE` asserts if `m_children` is non-empty (`pcbnew/router/pns_node.cpp:74-78`).

### 3.3 Add

Public overloads, all in `pcbnew/router/pns_node.h:381-386`:

- `bool Add( unique_ptr<SEGMENT>, bool aAllowRedundant = false )` rejects zero-length segments and, unless `aAllowRedundant`, rejects a segment identical to one already in a joint at that position (`pcbnew/router/pns_node.cpp:747-762`).
- `void Add( unique_ptr<SOLID> )` (`:617-621`).
- `void Add( unique_ptr<VIA> )` (`:652-655`).
- `bool Add( unique_ptr<ARC>, bool aAllowRedundant = false )` (`:776-788`).
- `void Add( LINE&, bool aAllowRedundant = false )` (`:683-733`), described in 2.3.
- `void AddRaw( ITEM*, bool )` is a public escape hatch to the private `add()` dispatcher (`pcbnew/router/pns_node.h:520-523`).

The private `add( ITEM*, bool )` switches on `Kind()` and asserts on anything unexpected; `HOLE_T` is a deliberate no-op with the comment "added by parent VIA_T or SOLID_T (pad)" (`pcbnew/router/pns_node.cpp:658-680`).

Every `addXxx` helper does three things in this order: register holes, link joints, `SetOwner(this)` and `m_index->Add()`.

- `addSolid`: adds the hole, then links a joint **only if the solid is routable** (`pcbnew/router/pns_node.cpp:601-614`, condition at `:609`).
- `addVia`: adds the hole (asserting the hole belongs to the via), links one joint at the via position over the via's full layer range (`:624-639`).
- `addSegment`: links joints at both endpoints (`:736-744`).
- `addArc`: links joints at `Anchor(0)` and `Anchor(1)` (`:765-773`).
- `addHole`: owner and index only, no joint (`:642-649`).

`BeginBulkAdd()` / `FinalizeBulkAdd()` (`pcbnew/router/pns_node.cpp:1257-1267`) toggle `INDEX`'s deferred mode so that the initial board sync can bulk-load the R-trees with a Hilbert-curve build instead of n individual inserts.

### 3.4 Replace

`Replace( ITEM* aOldItem, unique_ptr<ITEM> aNewItem )` is literally `Remove(old); add(new.release())` (`pcbnew/router/pns_node.cpp:951-955`). `Replace( LINE&, LINE&, bool )` is `Remove(old); Add(new, allowRedundant)` (`:958-962`).

### 3.5 Remove, doRemove and m_override

This is the heart of the copy-on-write model. `doRemove` (`pcbnew/router/pns_node.cpp:809-853`):

```
doRemove(item):
    holeRemoved = false

    # case 1: the item lives in the root and we are a branch -> shadow it, do not touch the root
    if item.BelongsTo(m_root) and not isRoot():                # :815
        m_override.insert(item)
        if item.HasHole(): m_override.insert(item.Hole())      # :819-820

    # case 2: the item lives in this branch, in a non-root ancestor branch,
    #         or in the root and we *are* the root -> physically de-index it here
    elif not item.BelongsTo(m_root) or isRoot():               # :825
        m_index.Remove(item)
        if item.HasHole(): m_index.Remove(item.Hole()); holeRemoved = true

    # ownership hand-off: only if this very node owned it
    if item.BelongsTo(this):                                   # :837
        item.SetOwner(nullptr)
        m_root.m_garbageItems.insert(item)                     # :840  deferred free, on the ROOT
        if item.Hole():
            if not holeRemoved: m_index.Remove(item.Hole())    # :847
            item.Hole().SetOwner(item)                         # :850  hole reverts to its parent
```

Two things to internalise. First, `m_override` only ever holds **root** items. A branch item removed by a deeper branch is handled by case 2, because the deeper branch owns a *copy* of the index that contains it. Second, ownership transfer is deferred: the item is not freed, it is parked in the root's `m_garbageItems` with a null owner, so any `LINE` still holding a link to it does not immediately dangle. It is freed later by `releaseGarbage` (`pcbnew/router/pns_node.cpp:1592-1619`), which runs only on the root (`:1594`) and only deletes items the root does not currently own (`:1602`), so an item removed and then re-added survives.

The typed `Remove` overloads first tear down the joint structure, then call `doRemove`:

- `Remove(SEGMENT*)`: `removeSegmentIndex` unlinks both endpoint joints (`pcbnew/router/pns_node.cpp:856-860`), then `doRemove` (`:984-988`).
- `Remove(ARC*)`: same for `Anchor(0)`/`Anchor(1)` (`:863-867`, `:991-995`).
- `Remove(VIA*)`: `removeViaIndex` finds the joint at the via position and calls `rebuildJoint` (`:931-936`), then `doRemove`, then asserts the hole reverted (`:972-981`).
- `Remove(SOLID*)`: `removeSolidIndex` skips non-routable solids entirely (`:939-948`), else `rebuildJoint`.
- `Remove(ITEM*)` dispatches on kind; the `SOLID_T` and `VIA_T` cases recursively `Remove(hole)` and then restore `hole->SetOwner(parent)` before removing the parent (`:1006-1018`, `:1034-1046`). The `LINE_T` case iterates the links (`:1024-1032`).
- `Remove(LINE&)` iterates the links with a manual kind dispatch, then nulls the line's owner and clears the links (`:1054-1071`). The comment at `:1056` says it: "LINE does not have a separate remover, as LINEs are never truly a member of the tree".

`Overrides( ITEM* )` is a hash lookup in `m_override` (`pcbnew/router/pns_node.h:513-516`); `GetOverrides()` exposes the set (`:525-528`).

### 3.6 rebuildJoint

Removing a via or a pad, which binds several layers into one joint, requires splitting that joint back into per-layer joints. `rebuildJoint` (`pcbnew/router/pns_node.cpp:870-928`) takes the lazy route described in its own comment ("As I'm a lazy bastard, I simply delete the via/solid and all its links and re-insert them", `:876-877`):

```
rebuildJoint(joint, item):
    links = copy of joint.LinkList()
    tag   = { pos = joint.Pos(), net = item.Net() }

    # erase every local joint at that tag whose layers overlap the item
    loop:
        found = first f in m_joints.equal_range(tag) with item.LayersOverlap(f.second)
        if none: break
        m_joints.erase(found)

    # a branch must record "there is deliberately nothing here" so FindJoint
    # does not fall through to the root's version
    completelyErased = false
    if not isRoot() and m_joints has no entry for tag:                     # :911
        m_joints.insert(tag, JOINT(tag.pos, PNS_LAYER_RANGE(-1), tag.net)) # :913-915
        completelyErased = true

    for link in links:
        if link != item: linkJoint(tag.pos, link.Layers(), net, link)
        elif not completelyErased: unlinkJoint(tag.pos, link.Layers(), net, link)
```

The dummy joint with layer range `(-1)` at `:913` is the branch-level tombstone. `PNS_LAYER_RANGE(-1).Overlaps(anything)` is false (`pcbnew/router/pns_layerset.h:69-70`), so `FindJoint` walks past it and, because it found *something* under the tag locally, never consults the root.

### 3.7 The index and parent-chain queries

Every read path consults exactly two nodes: `this` and `m_root`. There is no walk up the chain.

- `QueryColliding` (`pcbnew/router/pns_node.cpp:267-295`): query `m_index` with `override = nullptr`, then, if not root and the limit has not been hit, query `m_root->m_index` with `override = this` (`:282-292`).
- `HitTest` (`:572-598`): query own index, then the root's, filtering root hits through `Overrides()` (`:590-594`). Note the stray `visitor.SetWorld( m_root, nullptr )` at `:586` has no effect because a fresh `visitor_root` is used for the root query; and `visitor_root` never gets a `SetWorld` call at all.
- `FindJoint` (`:1359-1383`): look in `m_joints`, and only if not found there and not root, look in `m_root->m_joints`.
- `AllItemsInNet` (`:1653-1679`): own net bucket, then root's net bucket filtered by `Overrides()` and `IsRoutable()`.
- `QueryJoints` (`:1777-1812`): own joint map, then the root's filtered by `Overrides( &j.second )`. That call passes a `JOINT*`, so it is testing joint pointers against a set of `ITEM*` populated only with items, which can never match. Effectively the root joints are never filtered here.
- `FindItemByParent` (`:1815-1833`) and `FindItemsByParent` (`:1836-1847`) look **only** at the local index. `FindItemsByParent` iterates the whole index linearly.

`OBSTACLE_VISITOR` carries the pair (`pcbnew/router/pns_node.h:186-211`); `SetWorld( node, override )` (`pcbnew/router/pns_node.cpp:208-212`) and `visit( candidate )` returns true, meaning "skip this candidate", when the override node shadows it (`:215-223`).

`LAYER_CONTEXT_SETTER` is an RAII helper that stamps the current sub-index layer onto the visitor for the duration of one sub-index query (`pcbnew/router/pns_node.h:214-230`, applied at `pcbnew/router/pns_index.h:167`).

### 3.8 QueryColliding, OBSTACLE, COLLISION_SEARCH_OPTIONS

`OBSTACLE` (`pcbnew/router/pns_node.h:88-111`):

| Field | Meaning | Written where |
| --- | --- | --- |
| `ITEM* m_head` | the item we searched collisions *against* | `pcbnew/router/pns_item.cpp:261`, `:286` |
| `ITEM* m_item` | the item found colliding | `pcbnew/router/pns_item.cpp:262`, `:287` |
| `VECTOR2I m_ipFirst` | first intersection between head and the obstacle's hull | `pcbnew/router/pns_node.cpp:464` |
| `int m_clearance` | the clearance that was applied | `pcbnew/router/pns_item.cpp:263`, `:288` |
| `VECTOR2I m_pos` | never written anywhere in the tree | (unused) |
| `int m_distFirst` | path length from the line start to `m_ipFirst` | `pcbnew/router/pns_node.cpp:463` |
| `int m_maxFanoutWidth` | widest track fanning out of an obstacle via, used to inflate the via for force propagation | `pcbnew/router/pns_shove.cpp:1553`, `:1592` |

`operator==` and `operator<` both key on `(m_head, m_item)` **by pointer address** (`pcbnew/router/pns_node.h:98-110`). The container is `std::set<OBSTACLE>` (`pcbnew/router/pns_node.h:254`). Two consequences, both important:

1. Set iteration order is address order, which is not reproducible across runs. `CheckColliding` returns `*obs.begin()` (`pcbnew/router/pns_node.cpp:522`, `:530`, `:535`), so "the" reported obstacle is arbitrary.
2. In the line paths, `m_head` points at a stack-local `SEGMENT` constructed inside the loop (`pcbnew/router/pns_node.cpp:310`, `:518`), which dangles the moment the iteration ends. In practice `m_head` is never read again (grep finds only writes plus one commented-out debug line at `pcbnew/router/pns_shove.cpp:1735`), and because the temporary reuses the same stack slot every iteration the dedup degenerates to "unique by `m_item`", which is what the algorithm actually wants.

`COLLISION_SEARCH_OPTIONS` (`pcbnew/router/pns_node.h:114-123`):

| Field | Default | Effect |
| --- | --- | --- |
| `m_differentNetsOnly` | `true` | when true, same-net and free-pad pairs get clearance `-1` (`pcbnew/router/pns_item.cpp:188`, `:193`) |
| `m_overrideClearance` | `-1` | if >= 0, bypasses the rule resolver entirely (`pcbnew/router/pns_item.cpp:214`) |
| `m_limitCount` | `-1` | stop after this many obstacles (`pcbnew/router/pns_node.cpp:259`) |
| `m_kindMask` | `-1` | candidate pre-filter (`pcbnew/router/pns_node.cpp:243`) |
| `m_useClearanceEpsilon` | `true` | forwarded to `RULE_RESOLVER::Clearance` |
| `m_filter` | null | `std::function<bool(const ITEM*)>` predicate, false rejects (`pcbnew/router/pns_node.cpp:250`) |
| `m_layer` | `-1` | declared, never read anywhere |

`COLLISION_SEARCH_CONTEXT` bundles a reference to the caller's obstacle set with a *by-value const copy* of the options (`pcbnew/router/pns_node.h:126-136`).

`DEFAULT_OBSTACLE_VISITOR::operator()` filters in order: kind mask, self-identity, user filter, override check, then `Collide` (`pcbnew/router/pns_node.cpp:241-263`). Returning false from the visitor aborts the R-tree walk.

`QueryColliding` early-returns 0 for virtual items ("By default, virtual items cannot collide", `pcbnew/router/pns_node.cpp:272-274`).

### 3.9 NearestObstacle

`pcbnew/router/pns_node.cpp:298-475`. This is the geometric refinement pass on top of `QueryColliding`:

```
NearestObstacle(line, opts):
    obstacleSet = {}
    for i in 0 .. line.CLine().SegmentCount()-1:
        s = SEGMENT(line, line.CLine().CSegment(i))     # stack temporary!  :310
        QueryColliding(&s, obstacleSet, opts)
    if line.EndsWithVia(): QueryColliding(&line.Via(), obstacleSet, opts)   # :314-315
    if obstacleSet empty: return none

    obstacles = vector(obstacleSet)                     # address-ordered

    # SEQUENTIAL phase: GetClearance() and HullCache() are not thread safe    :346-348
    for each obstacle i:
        cl     = GetClearance(obstacle.m_item, line, opts.m_useClearanceEpsilon) + line.Width()/2
        hull_i = maybeBBox( resolver.HullCache(obstacle.m_item, cl, 0, line.Layer()) )
        if line has via:
            vcl     = GetClearance(obstacle.m_item, &via, ...) + via.Diameter(line.Layer())/2
            viaHull = maybeBBox( resolver.HullCache(obstacle.m_item, vcl, 0, line.Layer()) )

    # PARALLEL phase over owned hull copies
    for each obstacle i:
        for each intersection of hull_i with the line path:
            dist = linePath.PathLength(ip.p, ip.index_their)
            keep the minimum
        (same for viaHull)

    pick the obstacle with the smallest dist; break early on dist == 0        :466-467
    if nothing intersected at all: return obstacles[0]                        :471-472
```

`maybeBBox` is the `makeHull` lambda at `:330-344`: for `MITERED_90` and `ROUNDED_90` corner modes the hull is replaced by its axis-aligned bounding box.

Note the walkaround thickness passed to `HullCache` here is `0` and the line's half width is folded into the clearance instead (`:361-362`). `WALKAROUND::processCluster` does the opposite, passing `aLine.Width()` as the thickness (`pcbnew/router/pns_walkaround.cpp:157-158`). Both reach the same geometry via `cl = aClearance + aWalkaroundThickness / 2` inside the builders (`pcbnew/router/pns_via.cpp:240`, `pcbnew/router/pns_utils.cpp:186`), but they produce **different cache keys**, so the same hull is computed and stored twice.

The fallback at `:471-472` ("if nothing intersected, return the address-first obstacle") is a correctness wart: it can report an obstacle that geometrically does not block the path.

Parallelism: obstacles are processed with `thread_pool::submit_loop` when there are more than 8 of them, chunked at 8 per block (`pcbnew/router/pns_node.cpp:434-445`). The comment explains the constant: task submission locks the pool's queue mutex and wakeup latency is 5 to 20 microseconds, so small blocks lose (`:430-433`).

### 3.10 CheckColliding and HitTest

`CheckColliding( const ITEM*, int aKindMask )` builds options with `m_limitCount = 1` and forwards (`pcbnew/router/pns_node.cpp:492-500`).

`CheckColliding( const ITEM*, const COLLISION_SEARCH_OPTIONS& )` (`:502-539`) special-cases `LINE_T` by decomposing into per-segment `SEGMENT` temporaries exactly like `NearestObstacle`, returning as soon as any query yields something, then the trailing via.

`CheckColliding( const ITEM_SET&, int )` is a loop over the set (`:478-489`).

`HitTest( const VECTOR2I& )` (`:572-598`) treats the point as a zero-radius circle ("fixme: we treat a point as an infinitely small circle, this is inefficient", `:576`) and runs a `HIT_VISITOR` that tests `aItem->Shape( -1 )->Collide( &cp, 0 )` (`:557-568`). The `-1` there is flagged in place: "TODO(JE) padstacks, this may not work" (`:563`), because a complex viastack has no shape at layer `-1` and `VIA::Shape` will return nullptr through the `wxCHECK`, then get dereferenced.

### 3.11 Commit, KillChildren, GetUpdatedItems

`GetUpdatedItems( aRemoved, aAdded )` (`pcbnew/router/pns_node.cpp:1560-1576`) returns nothing for a root, else fills `aRemoved` from `m_override` and `aAdded` from the entire local index. For a deep branch, `aAdded` therefore includes items inherited from intermediate branches, not just this node's own additions.

`Commit( NODE* aNode )` (`:1622-1644`) is called on the root with a branch as the argument (`pcbnew/router/pns_router.cpp:911`):

```
Commit(node):
    if node.isRoot(): return
    for item in node.m_override: Remove(item)      # physically remove from the root
    for item in *node.m_index:
        if item.HasHole(): item.Hole().SetOwner(item)   # re-parent the hole  :1632-1635
        item.SetRank(-1)                                #                      :1637
        item.Unmark()                                   #                      :1638
        add(item)                                       # re-owns to the root  :1639
    releaseChildren()                                   # destroys the whole subtree
    releaseGarbage()
```

Note there is no ordering guarantee between the removals and the additions beyond "all removals first". Note also `Unmark()` with the default argument `-1` clears every marker bit (`pcbnew/router/pns_item.h:262`).

`KillChildren()` is `releaseChildren()` (`:1647-1650`), which recursively deletes the subtree, copying `m_children` first because each `~NODE` erases itself from its parent's set via `unlinkParent` (`:1579-1589`, `:191-197`).

The typical placer sequence is `world->KillChildren(); NODE* rootNode = world->Branch();` (`pcbnew/router/pns_line_placer.cpp:1463-1464`, `pcbnew/router/pns_diff_pair_placer.cpp:665-666`).

### 3.12 Max clearance and why it exists

`m_maxClearance` is the R-tree query inflation radius. `INDEX::Query` is called with it in every search path (`pcbnew/router/pns_node.cpp:285`, `:291`, `:581`, `:588`), and `SHAPE_INDEX::Query` inflates the query shape's bounding box by that amount before searching (`libs/kimath/include/geometry/shape_index.h:323-324`).

It is therefore a **soundness parameter**: if it is smaller than the largest clearance any rule can return, the broad phase silently drops candidates and the narrow phase never sees them. The default is `800000` internal units with the comment "fixme: depends on how thick traces are" (`pcbnew/router/pns_node.cpp:62`). KiCad's host overrides it during sync as `board->GetMaxClearanceValue() + resolver->ClearanceEpsilon()` (`pcbnew/router/pns_kicad_iface.cpp:2300`, `:2452`).

Note the broad phase inflates only by `m_maxClearance`, while `collideSimple` tests against `clearance + lineWidthH + lineWidthI - 1` (`pcbnew/router/pns_item.cpp:249`, `:280`). The half-widths are covered because a `SHAPE_LINE_CHAIN`'s and `SHAPE_SEGMENT`'s bounding boxes already account for their own widths, but this is an implicit coupling worth asserting in a reimplementation.

`Branch()` propagates `m_maxClearance` (`pcbnew/router/pns_node.cpp:167`), `SetRuleResolver` and `SetMaxClearance` are plain setters (`pcbnew/router/pns_node.h:280-289`).

### 3.13 GetClearance

`pcbnew/router/pns_node.cpp:143-154`. Returns a hardcoded `100000` when there is no resolver, `0` when either item is virtual, else delegates to `m_ruleResolver->Clearance( aA, aB, aUseClearanceEpsilon )`.

The virtual-item case at `:148-149` is what makes `VVIA`s (see 3.15) participate in force propagation without imposing clearance.

### 3.14 AssembleLine and followLine

`followLine` (`pcbnew/router/pns_node.cpp:1074-1129`) walks the joint graph in one direction from a starting item, filling parallel arrays of corners, items and per-item arc-reversal flags:

```
followLine(current, scanDirection, pos, limit, corners, segments, arcReversed,
           guardHit, stopAtLockedJoints, followLockedSegments, allowSegmentSizeMismatch):
    guard      = current.Anchor(scanDirection)          # the loop detector       :1081
    startWidth = current.Width()                        #                          :1082
    prevReversed = false

    for count = 0, 1, 2, ...:
        p  = current.Anchor(scanDirection ^ prevReversed)
        jt = FindJoint(p, current)
        if not jt: break

        corners[pos]     = jt.Pos()
        segments[pos]    = current
        arcReversed[pos] = (current is ARC and the joint sits at the "wrong" arc end)   :1096-1103
        pos += scanDirection ? +1 : -1

        if count > 0 and guard == p:                    # closed loop            :1107
            if pos in range: segments[pos] = nullptr
            guardHit = true; break

        if (stopAtLockedJoints and jt.IsLocked()) or pos out of range: break     :1116-1119

        next = jt.NextSegment(current, followLockedSegments)                     :1121
        if not next: break                              # non-trivial joint = line end
        if not allowSegmentSizeMismatch and next.Width() != startWidth: break    :1123

        current      = next
        prevReversed = (jt.Pos() == current.Anchor(scanDirection))               :1127
```

The three option flags:

- `aStopAtLockedJoints` terminates the line at the first joint whose `m_locked` flag is set (`:1116`). Joints are locked via `NODE::LockJoint` (`:1386-1390`), which the shove algorithm uses to pin the head endpoints (`pcbnew/router/pns_shove.cpp:2492`, `:2495`).
- `aFollowLockedSegments` is forwarded to `JOINT::NextSegment` (`:1121`) where it means "a locked segment is still a valid continuation, and a virtual via at this joint does not terminate the line" (`pcbnew/router/pns_joint.h:255`, `:261-265`).
- `aAllowSegmentSizeMismatch` (default true, `pcbnew/router/pns_node.h:441`) allows the line to cross a width change. When false the line stops at the first differently sized item.

`AssembleLine` (`pcbnew/router/pns_node.cpp:1132-1213`) is the driver:

```
AssembleLine(seg, originSegmentIndex, stopAtLockedJoints, followLockedSegments, allowSizeMismatch):
    MaxVerts = 1024 * 16                                                  # :1135
    fixed arrays corners[MaxVerts+1], segs[MaxVerts+1], arcReversed[MaxVerts+1]
    i_start = MaxVerts/2 ; i_end = i_start + 1                            # :1144-1145

    pl = new LINE with seg's width, layers, net; parent=null; sourceItem=seg's; owner=this  # :1147-1152

    followLine(seg, backwards, i_start, ...)                              # :1154
    if not guardHit: followLine(seg, forwards, i_end, ...)                # :1157-1161

    for i in i_start+1 .. i_end-1:
        li = segs[i]
        if li is null or not an ARC: line.Append(corners[i])              # :1175-1176
        if li and li != prev_seg:                                         # :1178
            if li is ARC: line.Append(arcReversed[i] ? arcShape.Reversed() : arcShape)   # :1180-1185
            pl.Link(li)                                                   # :1188
            if li == seg and originSegmentIndex and not originSet:
                *originSegmentIndex = line.PointCount() - 1 ; originSet = true           # :1191-1197
        prev_seg = li

    pl.Line().RemoveDuplicatePoints()   # "do NOT remove colinear segments here!"  :1203-1204
    clamp *originSegmentIndex to pl.SegmentCount()-1                      # :1207-1208
    assert pl.SegmentCount() != 0                                         # :1210
    return pl (by value)
```

The bidirectional array trick (`MaxVerts/2` as the origin, grow both ways) is a 128 KiB stack allocation per call for the three `std::array`s. A Rust port should use a `VecDeque` or two `Vec`s.

`FindLineEnds( line, jointA, jointB )` dereferences `FindJoint` without a null check (`:1216-1220`).

`FindLinesBetweenJoints( a, b, out )` assembles a line from every segment or arc linked to joint `a`, discards those whose layers do not overlap `b`, then clips each to the vertex range between the two joint positions (`:1223-1254`). Used for loop removal.

### 3.15 FixupVirtualVias

`pcbnew/router/pns_node.cpp:1270-1356`. Walks every joint and injects `VVIA`s, which are virtual vias used purely as force-propagation anchors (`pcbnew/router/pns_via.h:367-377`). A `VVIA` sets `m_isVirtual = true`, so it never collides (`pcbnew/router/pns_node.cpp:272-274`), never gets a clearance (`:148-149`), and is skipped when reporting updated items to the host (`pcbnew/router/pns_router.cpp:900`).

The trigger condition is `( is_width_change || n_seg >= 3 || is_locked ) && n_solid == 0 && n_vias == 0` (`:1333`). **`n_seg` is declared at `:1282` and never incremented**, so the "three or more segments meet here" branch is dead code in this revision. Only width changes and locked segments produce a VVIA.

The VVIA diameter is `max_w + 2 * PNS_HULL_MARGIN` (`:1338`, `:1348`) with the comment "the hull margin here is an ugly temporary workaround, the real fix is to use octagons for via force propagation" (`:1335-1336`).

`is_locked` and `locked_seg` are assigned unconditionally inside the segment branch (`:1328-1329`), so they reflect only the *last* segment examined at that joint; `locked_seg` is even declared outside the joint loop (`:1272`) and so can leak across joints.

### 3.16 NODE invariants

The set of properties a reimplementation must maintain:

1. `m_root` is reachable and alive for the entire lifetime of every descendant. `~NODE` asserts on live children (`pcbnew/router/pns_node.cpp:74-78`).
2. A node's index plus the root's index, minus `m_override`, is the complete world. Guaranteed by `Branch()` copying everything for non-root parents (`:171-178`).
3. `m_override` contains only items owned by `m_root`. Enforced by the `BelongsTo(m_root)` test in `doRemove` (`:815`).
4. Every indexed `SOLID`/`VIA` with a hole has that hole indexed too, and owned by the same node. Asserted in `~NODE` (`:107`) and in `addSolid`/`addVia` (`:605`, `:628-631`).
5. A `HOLE`'s owner is either the `NODE` (while indexed) or its parent pad/via (while detached). Flipped in `doRemove` (`:850`) and in `Commit` (`:1634`).
6. Every `SEGMENT`/`ARC`/`VIA`/routable `SOLID` in the index has joints at all of its anchors. Maintained by the paired `addXxx`/`removeXxxIndex` helpers.
7. Joint layer ranges are the merge of the layer ranges of all linked items, and a joint with zero links has layer range `(-1,-1)` and is thus invisible (`pcbnew/router/pns_joint.h:227-230`).
8. Joints are never deleted from the map on unlink ("fixme: remove dangling joints", `pcbnew/router/pns_node.cpp:1466`), only emptied. The map grows monotonically within a node.
9. An assembled `LINE`'s `m_links` are all owned by, or visible from, the assembling node. Violated the moment anything is removed from that node.
10. `m_maxClearance` >= any clearance the resolver can return, or the broad phase is unsound.

Illegal mutations, all unenforced:

- Mutating an item's geometry, net or layers while it is in an index. The R-tree entry keys on the bounding box captured at insert time (`libs/kimath/include/geometry/shape_index.h:215-219`), and `Remove` recomputes the box to find the entry (`:243-247`), so a moved item can no longer be removed. `VIA::SetPos` and `SOLID::SetPos` (`pcbnew/router/pns_via.h:208-217`, `pcbnew/router/pns_solid.cpp:81-92`) are only safe on unindexed items.
- Removing an item from a node while any live `LINE` still links it.
- Deleting a `NODE` that has children.
- Committing a branch to anything but the root (`Commit` early-returns for a root argument, `:1624`, but does not check that `this` is the root).
- Holding a `const JOINT*` across any `Add` or `Remove` on the same node, because `touchJoint` erases and reinserts (`:1432`, `:1439`).

---

## 4. JOINT

`pcbnew/router/pns_joint.h:42-371`. A `JOINT` is itself an `ITEM` of kind `JOINT_T`, which is why it can carry `m_layers` and be passed to `LayersOverlap`.

### 4.1 The hash tag

```
struct HASH_TAG { VECTOR2I pos; NET_HANDLE net; };
```

`pcbnew/router/pns_joint.h:47-51`. The layer range is deliberately **not** in the key ("Joints are hashed by their position, layers and net" is the comment at `:45-46`, but the struct only holds position and net). The hash mixes the two coordinates and the net pointer (`:53-65`); equality is componentwise (`:373-376`). `JOINT::operator==` compares only position and net, ignoring layers (`:325-328`).

The container is `std::unordered_multimap<HASH_TAG, JOINT, JOINT_TAG_HASH>` (`pcbnew/router/pns_node.h:589`). Several joints can share a tag and be distinguished only by layer range, which is exactly what `touchJoint` and `rebuildJoint` exploit.

`Overlaps( const JOINT& )` is the real identity test: same position, same net, overlapping layers (`pcbnew/router/pns_joint.h:346-350`).

### 4.2 touchJoint

`pcbnew/router/pns_node.cpp:1393-1440` is the only way a joint is created or modified:

```
touchJoint(pos, layers, net):
    tag = {pos, net}
    if tag not in m_joints and not isRoot():
        copy every root joint with that tag into m_joints        # copy-on-write   :1406-1412
    jt = JOINT(pos, layers, net)
    loop:
        find any local joint with that tag whose layers overlap `layers`
        if none: break
        jt.Merge(that joint)   # union of links, hull of layer ranges, OR of locked
        erase that joint
    return m_joints.insert(tag, jt)->second
```

`JOINT::Merge` is a no-op unless `Overlaps` holds (`pcbnew/router/pns_joint.h:330-344`), merges layer ranges (`:335`), ORs the locked flag (`:337-338`), and appends every link (`:340-343`).

The consequence for the port: **`touchJoint` invalidates any `const JOINT*` previously obtained for the same tag**, because the merged joint is erased and a new one inserted at a new address. `unordered_multimap` is node-based, so unrelated joints keep their addresses across rehash, but merged ones do not.

`linkJoint` and `unlinkJoint` are one-liners over `touchJoint` (`pcbnew/router/pns_node.cpp:1454-1470`). `LockJoint` likewise (`:1386-1390`).

### 4.3 Classification predicates

| Predicate | Line | Definition |
| --- | --- | --- |
| `IsLineCorner( aAllowLockedSegs = false )` | `:101` | exactly two links, both `SEGMENT_T\|ARC_T`, same width, neither locked unless allowed. A second branch handles "more than two links but exactly two are segments/arcs": only valid when `aAllowLockedSegs`, all non-segment links must be virtual, and the widths must match (`:114-144`). The comment at `:119-120` explains why: "There will be multiple VVIAs on joints between two locked segments, because we naively add a VVIA to each end of a locked segment." |
| `IsNonFanoutVia()` | `:149` | after skipping virtual items, exactly 3 real links of which 1 is a via and 2 are segments/arcs |
| `IsStitchingVia()` | `:171` | exactly one link and it is a via |
| `IsTrivialEndpoint()` | `:176` | exactly one link and it is a `SEGMENT_T` (the comment at `:178` flags arcs and endpoint vias as unhandled) |
| `IsTraceWidthChange()` | `:183` | exactly two `SEGMENT_T` links, no via, differing widths |
| `IsLocked()` | `:357` | the `m_locked` flag, set by `Lock()` (`:352`) |

Note `IsLineCorner` is the trivial-joint test and `NextSegment` is the traversal primitive; they are not defined in terms of each other and can disagree in edge cases.

### 4.4 NextSegment

`pcbnew/router/pns_joint.h:235-273`. "For trivial joints, return the segment adjacent to (aCurrent). For non-trivial ones, return NULL, indicating the end of line."

```
NextSegment(current, allowLockedSegs):
    other = null
    for item in links, item != current:
        if item is SEGMENT_T or ARC_T:
            if item.Net() == current.Net() and item.Layers().Overlaps(current.Layers()):
                if other already set: return null        # fanout, three or more branches
                if not item.IsLocked() or allowLockedSegs: other = item
        elif item is SOLID_T or VIA_T:
            if item is a virtual VIA and allowLockedSegs: continue   # skip VVIAs   :261-265
            return null                                             # pad or via terminates
    return other
```

The "if other already set, return null" check runs even when the second candidate would be rejected for being locked, so a locked third branch still terminates the line.

### 4.5 Links, LinkCount and Via

The link container is an `ITEM_SET` (`pcbnew/router/pns_joint.h:367`). `Link` deduplicates by pointer (`:215-221`). `Unlink` erases and resets the layer range to `(-1)` when the last link goes, returning whether the joint became dangling (`:225-231`). `LinkList()` exposes the raw vector (`:303-306`), `CLinks()` and `Links()` expose the set (`:308`, `:313`). `LinkCount( aMask = -1 )` counts by kind mask (`:318-321`).

`Via()` returns the first `VIA_T` link, with a "fixme: const correctness" note because it casts away const (`:275-284`).

`Dump()` is defined out of line in `pcbnew/router/pns_node.cpp:1443-1451`.

---

## 5. INDEX

`pcbnew/router/pns_index.h:46-158`, `pcbnew/router/pns_index.cpp`.

### 5.1 Storage

```
std::deque<std::unique_ptr<SHAPE_INDEX<ITEM*>>> m_subIndices;   # one per layer, indexed by layer id
std::map<NET_HANDLE, std::list<ITEM*>>          m_netMap;
std::unordered_set<ITEM*>                       m_allItems;
bool                                            m_deferred;
```

(`pcbnew/router/pns_index.h:154-157`.) One R-tree per PCB layer, "reducing overlap and improving search time" (`:42-44`). The deque is grown lazily to `range.End()` on the first `Add` that needs it (`pcbnew/router/pns_index.cpp:33-43`). Note the deque position *is* the layer id, so `m_subIndices[i]` is meaningful only for `i >= 0`.

An item spanning layers 0 to 31 is inserted into 32 sub-indices (`pcbnew/router/pns_index.cpp:47-48`). That is why through vias are relatively expensive.

### 5.2 Add, Remove, Replace, Clone

`Add` asserts the layer range is valid, grows the deque, inserts into each covered sub-index (unless deferred), inserts into `m_allItems`, and appends to the net bucket if the net is non-null (`pcbnew/router/pns_index.cpp:28-52`).

`Remove` silently returns if the deque is too short, removes from each covered sub-index, erases from `m_allItems` and from the net list (`:86-102`). The net list is a `std::list` and `remove(aItem)` is O(n).

`Replace` is `Remove` then `Add` (`:105-109`).

`Clone()` (`pcbnew/router/pns_index.h:71-81`) is the copy-on-write branch primitive: it clones each `SHAPE_INDEX`, which is O(1) because `SHAPE_INDEX::Clone` only bumps the refcount on the `COW_RTREE` root (`libs/kimath/include/geometry/shape_index.h:197-206`, `:112`), then copies `m_netMap` and `m_allItems` by value. So the *metadata* is O(n) per branch and only the tree is shared.

`SetDeferred(true)` plus `BuildSpatialIndex()` is the bulk path: `BuildSpatialIndex` groups every registered item by layer and calls `SHAPE_INDEX::BulkLoad`, a Hilbert-curve build (`pcbnew/router/pns_index.cpp:61-83`, `libs/kimath/include/geometry/shape_index.h:274-292`).

### 5.3 The "unconnected" net bucket

In this revision **there is no separate unconnected bucket**. `INDEX::Add` inserts into `m_netMap` only when `aItem->Net()` is non-null (`pcbnew/router/pns_index.cpp:52-55`), so items with a null net handle are reachable only through `m_allItems` and the spatial trees, never through `GetItemsForNet`. `GetItemsForNet( nullptr )` returns null unless something inserted a real null-keyed entry, which nothing does.

The practical effect: `NODE::AllItemsInNet( nullptr, ... )` (`pcbnew/router/pns_node.cpp:1653`) yields nothing, and `TOPOLOGY::NearestUnconnectedItem` is a no-op on netless items (`pcbnew/router/pns_topology.cpp:189`). `TOPOLOGY::NearestUnconnectedAnchorPoint` guards against this by rejecting `NetCode(jt->Net()) <= 0` up front (`pcbnew/router/pns_topology.cpp:123-124`).

### 5.4 Query

Two overloads, both templates in the header.

`Query( const ITEM* aItem, int aMinDistance, Visitor& )` (`pcbnew/router/pns_index.h:172-189`): iterates the item's own layer range, fetches `aItem->Shape(i)` for each layer, and queries sub-index `i` with that shape. Skips layers where the shape is null. This is why per-layer via shapes work: a padstack via searches each layer with the diameter it actually has there.

`Query( const SHAPE* aShape, int aMinDistance, Visitor& )` (`:192-200`): queries every sub-index with the same shape, "treats all layers as colliding" (`:120`).

Both route through `querySingle` (`:162-169`), which stamps the sub-index layer onto the visitor via `LAYER_CONTEXT_SETTER` before delegating to `SHAPE_INDEX::Query`. That is the whole mechanism by which `collideSimple` learns its `aLayer` argument.

`SHAPE_INDEX::Query` inflates the query shape's bounding box by `aMinDistance` and does a pure box search on the R-tree (`libs/kimath/include/geometry/shape_index.h:321-330`). There is no shape-level filtering in the broad phase; every box hit is handed to the visitor.

An item covering N layers is visited N times by a query that also covers those layers. `collideSimple` will be called N times with different `aLayer` values, and each call inserts into the same `std::set<OBSTACLE>` keyed on `(m_head, m_item)`, so duplicates collapse.

---

## 6. TOPOLOGY

`pcbnew/router/pns_topology.h:42-143`, `pcbnew/router/pns_topology.cpp`. A stateless helper over a borrowed `NODE* m_world` (`pcbnew/router/pns_topology.h:142`, constructed at `:54-55`).

### 6.1 What each entry point solves

| Method | Line | Problem |
| --- | --- | --- |
| `SimplifyLine( LINE* )` | `pcbnew/router/pns_topology.cpp:49` | reassemble the line from its first link, run `SHAPE_LINE_CHAIN::Simplify`, and if the point count changed, remove and re-add. Returns whether it changed anything. |
| `ConnectedJoints( const JOINT* )` | `:73` | BFS over the joint graph following only `SEGMENT_T\|ARC_T` links; returns `std::set<const JOINT*>` |
| `NearestUnconnectedItem( const JOINT*, int* aAnchor, int aKindMask )` | `:185` | all items in the joint's net, minus everything reachable via `ConnectedJoints`, then the closest remaining anchor by Euclidean distance |
| `NearestUnconnectedAnchorPoint( const LINE*, ...)` | `:107` | branch a temp node, add a copy of the track, find the joint at its last point; if something is already connected there return that, else fall back to `NearestUnconnectedItem` |
| `LeadingRatLine( const LINE*, SHAPE_LINE_CHAIN& )` | `:168` | two-point chain from the track end to the above point |
| `AssembleTrivialPath( ITEM*, pair<const JOINT*,const JOINT*>*, bool )` | `:461` | assemble a line from the start item, then extend through the longest branch at each end |
| `AssembleTuningPath( ROUTER_IFACE*, ITEM*, SOLID**, SOLID** )` | `:787` | like the above but with pad-entry truncation, matching `BOARD::GetTrackLength()` |
| `AssembleDiffPair( ITEM*, DIFF_PAIR& )` | `:1036` | find the coupled net's parallel partner item and assemble both lines |
| `AssembleCluster( ITEM*, int aLayer, double, NET_HANDLE )` | `:1187` | flood fill of touching items (clearance overridden to 0) used by the walkaround to treat a pad plus its neighbours as one obstacle |
| `ConnectedItems(...)` | `:1021`, `:1027` | **both overloads return an empty `ITEM_SET`**; they are stubs |
| `ShortestConnectionLength( ITEM*, ITEM* )` | declared `pcbnew/router/pns_topology.h:70` | not defined in this file |

### 6.2 AssembleTrivialPath

`pcbnew/router/pns_topology.cpp:461-533`. Resolves the start item to a `LINKED_ITEM*` (for a via, only if `IsNonFanoutVia()`, `:478-482`), assembles a line, then calls `followTrivialPath` (`:363-458`) which runs `followBranch` from each end.

`followBranch` (`:228-360`) is an explicit-stack DFS with a wall-clock timeout from `ADVANCED_CFG::m_FollowBranchTimeout` (`:237`, `:270-276`). Each stack frame carries its own `ITEM_SET pathItems` and `std::set<const JOINT*> visitedJoints` copied from the parent frame (`:328-330`), so it is exponential in memory on a dense mesh; the timeout is the only backstop. It keeps the longest path found (`:344-352`).

`followTrivialPath` seeds `visited` with the initial line's links (`:381-382`) and passes `aLine2->Links().front()` and `.back()` as the "previous item" for the two directions (`:389`, `:395`).

### 6.3 AssembleTuningPath

`pcbnew/router/pns_topology.cpp:787-...`. Same start-item resolution, but when the start via is a fanout it falls back to `findLinesFromVia` (`:536-608`), which does a zero-clearance `QueryColliding` restricted to `SEGMENT_T|ARC_T` (`:540-546`) and then requires at least one anchor to lie inside the via pad, using `LENGTH_DELAY_CALCULATION::IsPointInsideViaPad` when the parent is a real `PCB_VIA` (`:578-583`) and a plain shape hit test otherwise (`:586-589`).

`walkTuningPath` (`:611-784`) is the length-aware walker that finds terminal pads. The results are stitched into one `ITEM_SET` with the start line in the middle (`:867-875`).

### 6.4 What single-line routing actually needs

Needed for basic single-net, single-line routing:

- `AssembleCluster`, used by `WALKAROUND::singleStep` on every iteration (`pcbnew/router/pns_walkaround.cpp:124`).
- `SimplifyLine`, for post-route cleanup.
- `ConnectedJoints` and `NearestUnconnectedItem` only if you want the leading ratline hint.

Not needed:

- `AssembleTuningPath`, `walkTuningPath`, `findLinesFromVia`: length tuning only.
- `AssembleDiffPair` and the `DP_PARALLELITY_THRESHOLD` machinery: differential pairs only.
- `AssembleTrivialPath` and `followBranch`: used by the tuning tools and by the multi-dragger, not by the basic placer.
- `ConnectedItems`: stubs, ignore.

---

## 7. RULE_RESOLVER and CONSTRAINT

### 7.1 The contract

`pcbnew/router/pns_node.h:139-183`. Everything the router needs to know about design rules goes through this one interface. Pure virtuals the host **must** implement:

| Method | Line | Must return |
| --- | --- | --- |
| `int Clearance( const ITEM* aA, const ITEM* aB, bool aUseClearanceEpsilon = true )` | `:144` | required clearance in internal units, or a negative value meaning "these can never collide" |
| `NET_HANDLE DpCoupledNet( NET_HANDLE )` | `:147` | the partner net of a diff pair, or null |
| `int DpNetPolarity( NET_HANDLE )` | `:148` | positive for P, negative for N; `AssembleDiffPair` swaps the lines when negative (`pcbnew/router/pns_topology.cpp:1160-1161`) |
| `bool DpNetPair( const ITEM*, NET_HANDLE& aNetP, NET_HANDLE& aNetN )` | `:149` | resolve an item to its P/N pair |
| `int NetCode( NET_HANDLE )` | `:151` | a small integer id; `<= 0` is treated as "no net" (`pcbnew/router/pns_topology.cpp:123`) |
| `wxString NetName( NET_HANDLE )` | `:152` | display name |
| `bool IsInNetTie( const ITEM* )` | `:154` | the item belongs to a net-tie footprint; enables the slow collision path |
| `bool IsNetTieExclusion( const ITEM*, const VECTOR2I& aCollisionPos, const ITEM* aCollidingItem )` | `:155` | this particular collision at this position is permitted by the net tie |
| `bool IsDrilledHole( const ITEM* )` | `:158` | the item is a plated drilled hole; selects `CT_HOLE_TO_HOLE` |
| `bool IsNonPlatedSlot( const ITEM* )` | `:159` | enables the castellation path |
| `bool IsKeepout( const ITEM* aObstacle, const ITEM* aItem, bool* aEnforce )` | `:165` | "true if aObstacle is a keepout, set aEnforce if said keepout's rules exclude aItem" |
| `bool QueryConstraint( CONSTRAINT_TYPE, const ITEM* aA, const ITEM* aB, int aLayer, CONSTRAINT* out )` | `:167` | fill `out` and return true if a rule of that type applies |

Virtuals with defaults the host may override:

| Method | Line | Default |
| --- | --- | --- |
| `bool HasUserDefinedPhysicalConstraint()` | `:145` | `false` |
| `void ClearCacheForItems( std::vector<const ITEM*>& )` | `:170` | no-op; called by `~NODE` (`pcbnew/router/pns_node.cpp:127`) and `releaseGarbage` (`:1610`) with the items about to be deleted |
| `void ClearCaches()` | `:171` | no-op |
| `void ClearTemporaryCaches()` | `:172` | no-op |
| `int ClearanceEpsilon() const` | `:174` | `0` |
| `const SHAPE_LINE_CHAIN& HullCache( const ITEM*, int aClearance, int aWalkaroundThickness, int aLayer )` | `:176-182` | a non-caching stub that writes into a function-local static and returns a reference to it, which is **not thread safe and not reentrant** |

`ClearCacheForItems` is the invalidation hook that keeps the pointer-keyed caches from returning results for freed items; it is mandatory in practice even though it is not pure virtual.

### 7.2 CONSTRAINT and CONSTRAINT_TYPE

```
struct CONSTRAINT {
    CONSTRAINT_TYPE m_Type;
    MINOPTMAX<int>  m_Value;
    bool            m_Allowed;
    wxString        m_RuleName, m_FromName, m_ToName;
    bool            m_IsTimeDomain;
};
```

(`pcbnew/router/pns_node.h:73-82`.) Only `m_Value.Min()` is consumed by the clearance path (`pcbnew/router/pns_kicad_iface.cpp:912`, `:922`, `:935`, `:945`, `:955`, `:962`).

`CONSTRAINT_TYPE` (`pcbnew/router/pns_node.h:51-66`) has 13 members: `CT_CLEARANCE = 1`, `CT_DIFF_PAIR_GAP`, `CT_LENGTH`, `CT_WIDTH`, `CT_VIA_DIAMETER`, `CT_VIA_HOLE`, `CT_HOLE_CLEARANCE`, `CT_EDGE_CLEARANCE`, `CT_HOLE_TO_HOLE`, `CT_DIFF_PAIR_SKEW`, `CT_MAX_UNCOUPLED`, `CT_PHYSICAL_CLEARANCE`, `CT_PHYSICAL_HOLE_CLEARANCE = 13`.

### 7.3 Clearance epsilon

`ClearanceEpsilon()` is a small slack subtracted from every positive clearance so that geometry sitting exactly at the limit does not read as a violation. In the KiCad host it is `board->GetDesignSettings().GetDRCEpsilon()`, captured once at resolver construction (`pcbnew/router/pns_kicad_iface.cpp:337-340`), and applied as `rv = max( 0, rv - m_clearanceEpsilon )` when `aUseClearanceEpsilon` and `rv > 0` (`:971-972`).

It is also added to `SetMaxClearance` so the broad phase stays conservative (`pcbnew/router/pns_kicad_iface.cpp:2452`).

The flag is threaded from `COLLISION_SEARCH_OPTIONS::m_useClearanceEpsilon` (default true, `pcbnew/router/pns_node.h:120`) through `collideSimple` (`pcbnew/router/pns_item.cpp:220-221`) and `NODE::GetClearance` (`pcbnew/router/pns_node.cpp:151`). `VIA::PushoutForce` deliberately turns it off so force propagation uses the strict clearance (`pcbnew/router/pns_via.cpp:158`, `:128`), as does `WALKAROUND::processCluster`'s hull query (`pcbnew/router/pns_walkaround.cpp:156`).

Because the flag is part of every cache key (`pcbnew/router/pns_kicad_iface.cpp:96`, `:165`), the two settings are cached independently.

### 7.4 The reference Clearance implementation

`PNS_PCBNEW_RULE_RESOLVER::Clearance` (`pcbnew/router/pns_kicad_iface.cpp:865-983`) is worth mirroring because the layering is not obvious:

```
Clearance(A, B, useEpsilon):
    bothOwned = A and B and A.Owner() and B.Owner()
    if bothOwned:   look up m_clearanceCache     keyed on the two ITEM POINTERS       # :873
    else:           look up m_tempClearanceCache keyed on (BoardItem, net, layers, kind, freePad)  # :881

    layers = (no B) ? A.Layers()
           : isEdge(A) ? B.Layers() : isEdge(B) ? A.Layers()
           : A.Layers().Intersection(B.Layers())
    layers = layers.Intersection( PNS_LAYER_RANGE(PCBNEW_LAYER_ID_START, PCB_LAYER_ID_COUNT-1) )  # :901

    rv = 0
    for layer in layers:
        if both are drilled holes:        rv = max(rv, CT_HOLE_TO_HOLE.Min())          # :908
        elif either is a hole and not same net: rv = max(rv, CT_HOLE_CLEARANCE.Min())  # :916
        if isCopper(A) and isCopper(B) and not sameNet and not freePad:
                                          rv = max(rv, CT_CLEARANCE.Min())             # :929
        if either is an edge:             rv = max(rv, CT_EDGE_CLEARANCE.Min())        # :941
        if either is a hole:              rv = max(rv, CT_PHYSICAL_HOLE_CLEARANCE.Min())  # :951
        always:                           rv = max(rv, CT_PHYSICAL_CLEARANCE.Min())    # :960

    if (sameNet or freePad) and rv == 0: rv = -1        # the "never collides" sentinel  :968
    if useEpsilon and rv > 0: rv = max(0, rv - epsilon)                                  :971
    store in the appropriate cache ; return rv
```

The comment at `:928` matters: "No 'else'; plated holes get both HOLE_CLEARANCE and CLEARANCE." And `:950`: "Physical clearances are net-blind: a physical_clearance rule applies regardless." That net-blindness is precisely what `HasUserDefinedPhysicalConstraint()` exists to detect, so that `collideSimple` knows it cannot take the same-net short circuit (`pcbnew/router/pns_item.cpp:125-127`, `:188`, `:193`).

The two-cache split is a direct consequence of PNS's pointer churn: real board items are stable for the session and can be keyed by address; items the router synthesises while routing are transient, so they are keyed by value and the whole temp cache is dropped via `ClearTemporaryCaches` (`pcbnew/router/pns_kicad_iface.cpp:826-829`). See the comment at `:127-129` and `:974-976`.

### 7.5 HullCache

`pcbnew/router/pns_kicad_iface.cpp:832-848`. Key is `{ const ITEM* item, int clearance, int walkaroundThickness, int layer }` (`:222-236`), hashed with a pointer hash on the item (`:246-247`). On a miss it calls `aItem->Hull(...)` and moves the result into the map, returning a reference into the map.

Three consequences the port must respect:

1. The returned reference is invalidated by any later insertion that rehashes the `std::unordered_map`. The router copies immediately: `NODE::NearestObstacle` copies the reference into an owned `SHAPE_LINE_CHAIN` before releasing the sequential phase, with the comment "we populate all caches first and copy the returned hull references into owned values before releasing the sequential phase" (`pcbnew/router/pns_node.cpp:346-348`, `:364`). `WALKAROUND::processCluster` copies too (`pcbnew/router/pns_walkaround.cpp:172`).
2. It is not thread safe. `pcbnew/router/pns_node.cpp:346` says so explicitly for both `GetClearance` and `HullCache`.
3. It is keyed by item address, so it must be invalidated when items die. `ClearCacheForItems` does that (`pcbnew/router/pns_kicad_iface.cpp:792`, called from `pcbnew/router/pns_node.cpp:127`, `:1610`).

The base-class stub at `pcbnew/router/pns_node.h:176-182` is a correctness trap: it returns a reference to a function-local static that the next call overwrites. Any code holding two hulls at once against the default implementation gets the same one twice.

---

## 8. Public method reference

### 8.1 NODE (`pcbnew/router/pns_node.h`)

| Method | Line | One-line semantics |
| --- | --- | --- |
| `NODE()` | `:256` | fresh root: depth 0, self as root, empty index, `m_maxClearance = 800000` |
| `~NODE()` | `:257` | asserts no children; deletes every indexed item it owns; releases garbage (root only); unlinks from parent |
| `GetClearance(a, b, useEpsilon)` | `:260` | 100000 if no resolver, 0 if either is virtual, else the resolver's answer |
| `GetMaxClearance()` | `:263` | the broad-phase inflation radius |
| `BeginBulkAdd()` | `:272` | defer spatial insertion during initial population |
| `FinalizeBulkAdd()` | `:277` | bulk-load the R-trees from everything added since |
| `SetMaxClearance(int)` | `:280` | set the broad-phase inflation radius; must dominate every rule |
| `SetRuleResolver(RULE_RESOLVER*)` | `:286` | attach a borrowed resolver; propagated to branches |
| `GetRuleResolver()` | `:291` | borrowed resolver |
| `JointCount()` | `:297` | size of the local joint map only |
| `Depth()` | `:303` | number of ancestors |
| `QueryColliding(item, out, opts)` | `:317` | broad plus narrow phase over self and root; returns obstacle count |
| `QueryJoints(box, out, layerMask, kindMask)` | `:320` | joints inside a box on overlapping layers with at least one matching link |
| `NearestObstacle(line, opts)` | `:331` | the obstacle whose hull the line hits first, by path length |
| `CheckColliding(item, kindMask)` | `:342` | first obstacle, limit 1 |
| `CheckColliding(itemSet, kindMask)` | `:353` | first obstacle over a whole set |
| `CheckColliding(item, opts)` | `:363` | first obstacle with explicit options; decomposes lines per segment |
| `HitTest(point)` | `:371` | every item whose layer-(-1) shape contains the point |
| `Add(unique_ptr<SEGMENT>, allowRedundant)` | `:381` | rejects zero-length and (optionally) duplicates; links two joints |
| `Add(unique_ptr<SOLID>)` | `:382` | adds hole, links a joint iff routable |
| `Add(unique_ptr<VIA>)` | `:383` | adds hole, links one multilayer joint |
| `Add(unique_ptr<ARC>, allowRedundant)` | `:384` | as segment, using the two arc anchors |
| `Add(LINE&, allowRedundant)` | `:386` | materialises the chain into SEGMENTs/ARCs and links them back into the line; does **not** add the trailing via |
| `AddEdgeExclusion(unique_ptr<SHAPE>)` | `:388` | register a castellation exclusion zone |
| `QueryEdgeExclusions(pos)` | `:389` | is this position inside any exclusion |
| `Remove(ARC*)` | `:394` | unlink both anchors, then `doRemove` |
| `Remove(SOLID*)` | `:395` | rebuild the joint (skipped for non-routable), then `doRemove` |
| `Remove(VIA*)` | `:396` | rebuild the joint, `doRemove`, assert the hole reverted |
| `Remove(SEGMENT*)` | `:397` | unlink both endpoints, then `doRemove` |
| `Remove(ITEM*)` | `:398` | kind dispatch; handles hole detach for SOLID and VIA; iterates links for LINE |
| `Remove(LINE&)` | `:405` | remove every linked item, null the line's owner, clear its links |
| `Replace(ITEM*, unique_ptr<ITEM>)` | `:413` | remove then add |
| `Replace(LINE&, LINE&, allowRedundant)` | `:414` | remove then add |
| `Branch()` | `:424` | new child; copies index, joints and overrides only when this is not the root |
| `AssembleLine(seg, *originIdx, stopAtLocked, followLocked, allowSizeMismatch)` | `:438` | walk the joint graph both ways from `seg` and build a LINE view |
| `Dump(bool)` | `:444` | whole body is `#if 0` |
| `GetUpdatedItems(removed, added)` | `:452` | `m_override` as removed, entire local index as added; empty for a root |
| `Commit(NODE*)` | `:462` | apply a branch's removals and additions to this node, then kill the subtree |
| `FindJoint(pos, layer, net)` | `:469` | first joint at that tag whose layers overlap; falls through to the root |
| `LockJoint(pos, item, lock)` | `:471` | touch the joint at that position for that item's layers/net and set its locked flag |
| `FindJoint(pos, item)` | `:478` | convenience using `item->Layers().Start()` and `item->Net()` |
| `FindLinesBetweenJoints(a, b, out)` | `:484` | every line connecting two joints, clipped to the joint-to-joint vertex range |
| `FindLineEnds(line, a, b)` | `:487` | the joints at the line's first and last points; dereferences without null check |
| `KillChildren()` | `:490` | recursively destroy the whole subtree |
| `AllItemsInNet(net, out, kindMask)` | `:492` | routable items of that net from self and root, root filtered by `Overrides` |
| `ClearRanks(markerMask)` | `:494` | rank to -1 and clear masked markers, local index only |
| `RemoveByMarker(marker)` | `:496` | remove every locally indexed item carrying that marker |
| `FindItemByParent(BOARD_ITEM*)` | `:498` | first local item with that parent, searched through the net bucket |
| `FindItemsByParent(BOARD_ITEM*)` | `:500` | all local items with that parent, linear scan |
| `HasChildren()` | `:502` | any live branches |
| `GetParent()` | `:507` | the node this was branched from |
| `Overrides(ITEM*)` | `:513` | is this root item shadowed by this branch |
| `FixupVirtualVias()` | `:518` | inject VVIA force anchors at width-change and locked joints |
| `AddRaw(ITEM*, allowRedundant)` | `:520` | public escape hatch to the private `add` dispatcher |
| `GetOverrides()` | `:525` | the shadow set |
| `FindViaByHandle(const VIA_HANDLE&)` | `:530` | resolve a pointer-free via identity back to a `VIA*` |

### 8.2 ITEM (`pcbnew/router/pns_item.h`, plus `OWNABLE_ITEM`)

| Method | Line | One-line semantics |
| --- | --- | --- |
| `Owner()` | `:72` | the owning `ITEM_OWNER`, or null |
| `SetOwner(const ITEM_OWNER*)` | `:77` | claim or release ownership; pure pointer assignment |
| `BelongsTo(const ITEM_OWNER*)` | `:82` | pointer equality against the owner |
| `ITEM(PnsKind)` | `:116` | zero-init: no net, movable, no parent, marker 0, rank -1, routable |
| `ITEM(const ITEM&)` | `:132` | copies everything except the owner, which becomes null |
| `~ITEM()` | `:149` | virtual, empty |
| `Clone()` | `:154` | pure virtual deep copy; per-class semantics differ (see 1.12) |
| `Hull(clearance, walkaroundThickness, layer)` | `:164` | closed convex clockwise walkaround boundary; default empty |
| `Kind()` | `:173` | the single-bit kind value |
| `OfKind(int mask)` | `:181` | bit test against the mask |
| `KindStr()` | `:189` | debug string |
| `SetParent(BOARD_ITEM*)` | `:191` | set the 1:1 board item; also sets the source item if non-null |
| `Parent()` | `:199` | the 1:1 board item, may be null |
| `SetSourceItem` / `GetSourceItem` | `:201`, `:202` | the progenitor board item for non-1:1 mappings |
| `BoardItem()` | `:207` | virtual; the board item even if not the direct parent (HOLE overrides) |
| `SetNet` / `Net` | `:209`, `:210` | opaque net handle; `Net` is virtual (HOLE and JOINT override) |
| `Layers()` / `SetLayers()` | `:212`, `:213` | the spanned layer interval |
| `SetLayer(int)` / `Layer()` | `:215`, `:216` | collapse to one layer / read `Layers().Start()` |
| `LayersOverlap(const ITEM*)` | `:221` | interval overlap, false if either is invalid |
| `Collide(head, node, layer, ctx)` | `:235` | full clearance-aware collision test; see 1.9 |
| `Shape(int layer)` | `:242` | borrowed shape for that layer, or null |
| `UniqueShapeLayers()` | `:250` | layers on which this item has a distinct shape; `{-1}` by default |
| `HasUniqueShapeLayers()` | `:252` | whether the above is meaningful; only VIA returns true |
| `RelevantShapeLayers(const ITEM*)` | `:259` | set union of both items' unique shape layers, or `{-1}` |
| `Mark(int)` | `:261` | virtual, const, overwrites `m_marker` (LINE fans out to links) |
| `Unmark(int = -1)` | `:262` | virtual, const, clears masked bits (LINE fans out then zeroes its own) |
| `Marker()` | `:263` | virtual (LINE ORs its links' markers) |
| `SetRank` / `Rank` | `:265`, `:266` | shove priority; LINE fans out / takes the minimum |
| `Anchor(int n)` | `:268` | the n-th connection point; default is the origin |
| `AnchorCount()` | `:273` | how many anchors; default 0 |
| `IsLocked()` | `:278` | `Marker() & MK_LOCKED` |
| `SetRoutable` / `IsRoutable` | `:283`, `:284` | non-routable solids get no joint and are skipped by `AllItemsInNet` |
| `SetIsFreePad` / `IsFreePad` | `:286`, `:288` | a NIC pad with no net yet; also true if the parent pad/via is free |
| `ParentPadVia()` | `:293` | virtual; only HOLE returns non-null |
| `IsVirtual()` | `:295` | VVIA marker; never collides, gets zero clearance, is never reported to the host |
| `SetIsCompoundShapePrimitive` / `IsCompoundShapePrimitive` | `:300`, `:301` | this item is one primitive of a decomposed compound pad |
| `HasHole()` / `Hole()` / `SetHole(HOLE*)` | `:303`, `:304`, `:305` | hole accessors; `SetHole` takes ownership |
| `Format()` | `:307` | virtual debug string: kind, net name, layer range |
| `OwningNode()` | `:309` | the owner as a `NODE*`, hopping through `ParentPadVia()` for holes; unchecked cast |

---

## 9. Magic constants

| Value | Location | What it is |
| --- | --- | --- |
| `MK_HEAD = 1<<0`, `MK_VIOLATION = 1<<3`, `MK_LOCKED = 1<<4`, `MK_DP_COUPLED = 1<<5` | `pcbnew/router/pns_item.h:43-46` | marker bits; `1<<1` and `1<<2` are unused; `MK_DP_COUPLED` is never used at all |
| `ANY_T = 0xffff` | `pcbnew/router/pns_item.h:112` | catch-all kind mask; callers also pass `-1` |
| `LINKED_ITEM_MASK_T` | `pcbnew/router/pns_item.h:113` | includes `SOLID_T` and `HOLE_T` which are not `LINKED_ITEM`s |
| `m_rank = -1` | `pcbnew/router/pns_item.h:125` | "no rank" sentinel |
| `- 1` in `clearance + lineWidthH + lineWidthI - 1` | `pcbnew/router/pns_item.cpp:249`, `:280` | hull-to-collision epsilon; hulls are built to exactly the clearance |
| `m_maxClearance = 800000` | `pcbnew/router/pns_node.cpp:62` | default broad-phase inflation, internal units, "fixme: depends on how thick traces are" |
| `return 100000` | `pcbnew/router/pns_node.cpp:146` | fallback clearance when no resolver is attached |
| `MIN_OBSTACLES_PER_BLOCK = 8`, `PARALLEL_THRESHOLD = 8` | `pcbnew/router/pns_node.cpp:434-435` | thread-pool chunking for hull intersection |
| `MaxVerts = 1024 * 16` | `pcbnew/router/pns_node.cpp:1135` | `AssembleLine` fixed stack buffers; origin at `MaxVerts/2` (`:1144`) |
| `n_seg >= 3` | `pcbnew/router/pns_node.cpp:1333` | dead branch: `n_seg` is never incremented |
| `max_w + 2 * PNS_HULL_MARGIN` | `pcbnew/router/pns_node.cpp:1338`, `:1348` | VVIA diameter, "ugly temporary workaround" |
| `PNS_LAYER_RANGE(-1)` dummy joint | `pcbnew/router/pns_node.cpp:913` | branch-level tombstone joint, invisible because negative ranges never overlap |
| `#define PNS_HULL_MARGIN 10` | `pcbnew/router/pns_line.h:45` | universal hull slack |
| `m_width = 1` | `pcbnew/router/pns_line.h:71` | dummy default line width |
| `IterationLimit = 5` | `pcbnew/router/pns_line.cpp:681` | `ClipToNearestObstacle` retry budget |
| `int d = 2;` then unconditionally `d = 1;` | `pcbnew/router/pns_line.cpp:726`, `:743` | `dragCornerInternal` lookback; the `d = 2` case is disabled by a commented-out `if` at `:742` |
| `PNS_LAYER_RANGE(0, 256)` | `pcbnew/router/pns_layerset.h:131` | `All()`, "fixme: use layer IDs header" |
| `(-1, -1)` default range | `pcbnew/router/pns_layerset.h:35-36` | the invalid/empty layer range sentinel |
| `ALL_LAYERS = 0`, `INNER_LAYERS = 1` | `pcbnew/router/pns_via.h:78-79` | padstack pseudo-layer keys, deliberately colliding with real layer ids 0 and 1 |
| `m_diameters[0] = 2`, `m_drill = 1` | `pcbnew/router/pns_via.h:86-87` | dummy via geometry |
| `Diameter(EffectiveLayer(0)) / 4` | `pcbnew/router/pns_via.cpp:181` | pushout force clamp, "another stupid heuristic" |
| `aMaxIterations = 10` | `pcbnew/router/pns_via.h:298` | `PushoutForce` default iteration cap; the "try the lead vector instead" switch fires past `aMaxIterations / 2` (`pcbnew/router/pns_via.cpp:187`) |
| `( 2*cl + width ) * ( 1.0 - M_SQRT1_2 )` | `pcbnew/router/pns_via.cpp:249`, `pcbnew/router/pns_hole.cpp:71` | equilateral octagon chamfer |
| `2.0 / ( 1.0 + M_SQRT2 ) * d` | `pcbnew/router/pns_utils.cpp:188`, `:86` | octagon side ratio in `SegmentHull` and `ArcHull` |
| `kinkThreshold = aClearance / 10` | `pcbnew/router/pns_utils.cpp:184` | below this length a segment is snapped to an axis or 45 degrees |
| `cl++` / `cl += 2` for near-degenerate segments | `pcbnew/router/pns_utils.cpp:226`, `:231`, `:239` | clearance bumps compensating the snapping above |
| `180.0` degrees, chord shorter than `cl` | `pcbnew/router/pns_utils.cpp:76` | an arc more than half a turn with a short chord is hulled as a circle |
| `DP_PARALLELITY_THRESHOLD = 5` | `pcbnew/router/pns_topology.h:106` | diff-pair parallelism tolerance |
| `0xBADC0FFEE0DDF00D` | `pcbnew/router/pns_kicad_iface.cpp:118`, `:206`, `:245` | hash seed for all three resolver caches |
| `(BOARD*)777` | `pcbnew/router/pns_kicad_iface.cpp:90` | `ENTERED_GROUP_MAGIC_NUMBER`, "keep this odd so that it can never match a real pointer" |
| `CQS_ALL_RULES = 1`, `CQS_IGNORE_HOLE_CLEARANCE = 2` | `pcbnew/router/pns_node.h:248-249` | `COLLISION_QUERY_SCOPE`, declared but never referenced anywhere |

---

## 10. Rust mapping notes

### 10.1 Item storage: arena with generational ids

Replace every `ITEM*` with a generational index into a per-world arena.

```rust
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct ItemId { index: u32, generation: u32 }

pub enum ItemBody { Solid(Solid), Segment(Segment), Arc(Arc), Via(Via), Hole(Hole) }

pub struct Item {
    kind:  Kind,            // one-hot, keep the bitmask for OfKind
    body:  ItemBody,
    net:   Option<NetHandle>,
    layers: LayerRange,
    parent: Option<BoardItemId>,
    source: Option<BoardItemId>,
    marker: MarkerFlags,    // bitflags!
    rank:   i32,
    flags:  ItemFlags,      // routable, virtual, free_pad, compound_primitive, movable
    hole:   Option<ItemId>, // replaces VIA::m_hole / SOLID::m_hole ownership
    parent_pad_via: Option<ItemId>, // replaces HOLE::m_parentPadVia
}
```

Why generational and not plain indices: the C++ code deliberately keeps removed items alive in `m_root->m_garbageItems` (`pcbnew/router/pns_node.cpp:840`) precisely because `LINE`s may still hold links to them. A generation counter turns "stale link" from undefined behaviour into a detectable `None`, which is the single largest safety win available here.

Do not model `m_owner` as data. In the C++ it exists only to answer three questions: "who frees this" (`~NODE` at `pcbnew/router/pns_node.cpp:98`, `~LINE` at `pcbnew/router/pns_line.cpp:78`), "which node do these links point into" (`OwningNode()`), and "is this a synthesised item" (`Clearance`'s `bothOwned` test at `pcbnew/router/pns_kicad_iface.cpp:868`). In Rust, (1) is the arena's job, (2) becomes an explicit `NodeId` field on `Line`, and (3) becomes an explicit `Provenance::Board | Provenance::Synthetic` enum on the item.

The virtual `Clone()` becomes a plain `impl Clone for Item` plus an `arena.insert(item.clone())`. Preserve the per-class differences deliberately or fix them deliberately: `SEGMENT::Clone` keeps the uid, `ARC::Clone` does not, `VIA::Clone` keeps the uid but rebuilds the hole. Pick one rule and document it.

`dynamic_cast`, `dyn_cast` and the `Kind()` switches all collapse into `match item.body`.

### 10.2 The branching NODE: explicit overlay, not persistent maps

The C++ model is already an explicit two-level overlay, and the two-level query invariant (section 3.7) makes it far simpler than a general persistent structure. Mirror it:

```rust
pub struct World {
    arena:  Arena<Item>,            // owns every item, all branches
    nodes:  Arena<Node>,
    root:   NodeId,
}

pub struct Node {
    parent:   Option<NodeId>,
    root:     NodeId,
    children: Vec<NodeId>,
    depth:    u32,
    index:    Index,                // spatial + net + membership, local additions
    joints:   JointMap,
    overrides: HashSet<ItemId>,     // root items shadowed here
    max_clearance: i32,
}
```

`Branch()` becomes: allocate a `Node`; if the parent is not the root, clone the parent's `index`, `joints` and `overrides` (`pcbnew/router/pns_node.cpp:171-178`). Keep that rule verbatim, because every query path assumes it.

Three options for the index clone, in increasing order of effort:

1. **Eager clone with a shared R-tree** (what C++ does). `SHAPE_INDEX::Clone` is O(1) via a refcounted copy-on-write R-tree (`libs/kimath/include/geometry/shape_index.h:197-206`), while the net map and the membership set are copied eagerly (`pcbnew/router/pns_index.h:71-81`). In Rust: `Arc<RTreeNode>` with `Arc::make_mut` on write, plus a plain clone of the `HashMap`/`HashSet`. Lowest risk, closest to the original, and the metadata copy is what dominates anyway.
2. **`im`-style persistent HAMT** for the net map and membership set, plus the same CoW R-tree. Makes branching cheap even with a large board. Worth doing only if profiling shows the metadata copy hurting.
3. **Explicit per-branch delta lists** (`added: Vec<ItemId>`, `removed: HashSet<ItemId>`) with query-time chain walking. Do **not** do this. It breaks the two-level invariant and turns every query into an O(depth) walk. The C++ authors chose eager copying specifically to avoid that.

Removal (`doRemove`, `pcbnew/router/pns_node.cpp:809-853`) maps cleanly:

```rust
fn do_remove(&mut self, world: &mut World, id: ItemId) {
    let owner_is_root = world.arena[id].home == self.root;
    if owner_is_root && !self.is_root() {
        self.overrides.insert(id);
        if let Some(h) = world.arena[id].hole { self.overrides.insert(h); }
    } else {
        self.index.remove(world, id);
        if let Some(h) = world.arena[id].hole { self.index.remove(world, h); }
    }
    if world.arena[id].home == self.id {
        // C++ parks the item in root.m_garbageItems with a null owner.
        // In Rust: either free it now (generational ids make stale links safe),
        // or move it to a per-root graveyard if you want dangling links to still read.
    }
}
```

Because generational ids make a stale reference a recoverable `None`, the entire `m_garbageItems` deferred-free machinery (`pcbnew/router/pns_node.cpp:840`, `:1592-1619`) can be deleted. That is a genuine simplification, not just a translation.

`Commit` (`pcbnew/router/pns_node.cpp:1622-1644`) becomes a method on `World` taking `(root_id, branch_id)`, since it mutates two nodes.

### 10.3 LINE links

`LINE` is a value type in Rust. Its links become `Vec<ItemId>` plus the `NodeId` they are valid in:

```rust
pub struct Line {
    chain: ShapeLineChain,
    width: i32,
    snap_threshold: i32,
    via: Option<LineVia>,
    links: Vec<ItemId>,
    links_valid_in: Option<NodeId>,   // replaces LINE::m_owner set at pns_node.cpp:1152
    blocking_obstacle: Option<ItemId>,
    marker: MarkerFlags,              // the line's own bits; Marker() ORs the links'
    rank: i32,
}

pub enum LineVia { Owned(Via), Linked(ItemId) }
```

`LineVia` makes the C++ `m_via` ownership fork (`pcbnew/router/pns_line.cpp:54-66`, `:1414-1436`, `:1645-1656`) explicit and non-optional, and it removes the need for the geometric self-collision heuristic at `pcbnew/router/pns_item.cpp:65-69`: with an explicit `Linked(id)`, `should_consider_hole_collisions` can compare ids directly instead of comparing position, padstack, net and drill.

Keep `IsLinkedChecked()` (`pcbnew/router/pns_line.h:125`) as a `debug_assert!` invariant on every function that consumes links.

`Mark`/`Unmark`/`Marker` fanning out over links (`pcbnew/router/pns_line.cpp:174-201`) needs `&mut World` in Rust, so they cannot stay `&self` methods. That is the right outcome: the C++ `mutable int m_marker` (`pcbnew/router/pns_item.h:327`) exists purely to hide this mutation behind a `const` interface. Make it `fn mark(&self, world: &mut World, flags: MarkerFlags)`.

### 10.4 Preserving parent-chain query semantics

Every read path in C++ visits exactly `{ self, root }`. Encode that as a single helper so it cannot be forgotten:

```rust
impl World {
    fn visit_candidates<F>(&self, node: NodeId, query: &Query, mut f: F)
        where F: FnMut(ItemId) -> ControlFlow<()>
    {
        self.nodes[node].index.query(self, query, &mut f);
        if node != self.nodes[node].root {
            let root = self.nodes[node].root;
            self.nodes[root].index.query(self, query, &mut |id| {
                if self.nodes[node].overrides.contains(&id) { ControlFlow::Continue(()) }
                else { f(id) }
            });
        }
    }
}
```

Then `query_colliding`, `hit_test`, `all_items_in_net` and `query_joints` all go through it. The C++ versions each reimplement the pattern by hand (`pcbnew/router/pns_node.cpp:284-292`, `:581-595`, `:1655-1678`, `:1784-1809`), which is exactly why `QueryJoints` ended up filtering `JOINT*` against a set of `ITEM*` (`:1801`) and never actually filtering anything, and why `HitTest`'s `SetWorld` call at `:586` is dead.

`FindJoint` (`pcbnew/router/pns_node.cpp:1359-1383`) needs the same treatment, with one wrinkle: the tombstone. Encode it as `Option<Joint>` in the map rather than a joint with layer range `(-1)`, so "deliberately empty here" is a type, not a magic value.

### 10.5 Joint identity: never use addresses

`JOINT` is stored by value in an `unordered_multimap` and identified throughout by the address of the mapped value: `TOPOLOGY::ConnectedJoints` returns `std::set<const JOINT*>` (`pcbnew/router/pns_topology.cpp:73`, `:94`), `followBranch` keeps `std::set<const JOINT*> visitedJoints` (`pcbnew/router/pns_topology.cpp:248`, `:319`), `NearestUnconnectedItem` intersects link pointers against a set (`:195`). But `touchJoint` erases and reinserts on merge (`pcbnew/router/pns_node.cpp:1432`, `:1439`), so those addresses are not stable across any mutation, and a joint found in a branch has a different address from the equal joint in the root.

In Rust give joints their own arena and `JointId`:

```rust
pub struct JointKey { pos: Point, net: Option<NetHandle> }   // matches HASH_TAG exactly
pub struct JointMap { by_key: HashMap<JointKey, SmallVec<[JointId; 2]>>, arena: Arena<Joint> }
pub struct Joint { key: JointKey, layers: LayerRange, links: Vec<ItemId>, locked: bool }
```

The multimap-of-overlapping-layer-ranges is inherent to the design (a via joint spans layers; removing the via splits it back into per-layer joints, `rebuildJoint` at `pcbnew/router/pns_node.cpp:870-928`), so keep the "several joints per key, disambiguated by layer overlap" structure. Just make the disambiguation explicit rather than a `f++` walk over the multimap's bucket chain that runs past `equal_range` into unrelated entries (`pcbnew/router/pns_node.cpp:1374-1380`).

### 10.6 RULE_RESOLVER

A trait with the same shape:

```rust
pub trait RuleResolver {
    fn clearance(&mut self, a: ItemRef<'_>, b: ItemRef<'_>, use_epsilon: bool) -> i32;
    fn has_user_defined_physical_constraint(&mut self) -> bool { false }
    fn dp_coupled_net(&self, net: NetHandle) -> Option<NetHandle>;
    fn dp_net_polarity(&self, net: NetHandle) -> i32;
    fn dp_net_pair(&self, item: ItemRef<'_>) -> Option<(NetHandle, NetHandle)>;
    fn net_code(&self, net: Option<NetHandle>) -> i32;
    fn net_name(&self, net: Option<NetHandle>) -> String;
    fn is_in_net_tie(&self, item: ItemRef<'_>) -> bool;
    fn is_net_tie_exclusion(&self, item: ItemRef<'_>, at: Point, other: ItemRef<'_>) -> bool;
    fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool;
    fn is_non_plated_slot(&self, item: ItemRef<'_>) -> bool;
    fn is_keepout(&self, obstacle: ItemRef<'_>, item: ItemRef<'_>) -> Option<KeepoutVerdict>;
    fn query_constraint(&self, ty: ConstraintType, a: ItemRef<'_>, b: Option<ItemRef<'_>>,
                        layer: i32) -> Option<Constraint>;
    fn clearance_epsilon(&self) -> i32 { 0 }
    fn hull(&mut self, item: ItemRef<'_>, clearance: i32, thickness: i32, layer: i32)
        -> Rc<ShapeLineChain>;
    fn invalidate_items(&mut self, items: &[ItemId]) {}
}
```

Three deliberate changes from the C++:

- `IsKeepout( obstacle, item, bool* aEnforce )` becomes `Option<KeepoutVerdict>` with variants `Enforce` (clearance 0) and `Exempt` (clearance -1), because the C++ out-parameter is only meaningful when the return is true (`pcbnew/router/pns_node.h:161-165`, consumed at `pcbnew/router/pns_item.cpp:198-205`).
- `HullCache` returns `Rc<ShapeLineChain>` instead of `&SHAPE_LINE_CHAIN`. That removes the reference-invalidation hazard (`pcbnew/router/pns_kicad_iface.cpp:844-847`), removes the need for the explicit "copy the hull before going parallel" phase (`pcbnew/router/pns_node.cpp:346-375`), and lets the parallel phase share hulls instead of copying them.
- Cache keys must not be raw pointers. `ItemId` works for the persistent cache; the C++ temp cache already keys by value (`pcbnew/router/pns_kicad_iface.cpp:130-197`), and with generational ids the persistent cache can key on `ItemId` too, with `invalidate_items` on removal.

`ClearanceEpsilon` and `HasUserDefinedPhysicalConstraint` should be cached fields on the concrete resolver, as they already are in KiCad (`pcbnew/router/pns_kicad_iface.cpp:308`, `:312`).

### 10.7 Threading

The C++ parallelises only the hull intersection loop, after an explicit sequential phase that warms the clearance and hull caches, because neither is thread safe (`pcbnew/router/pns_node.cpp:346-348`). In Rust, `&mut dyn RuleResolver` naturally forces the same split. If you want more parallelism later, the clean move is `Rc<ShapeLineChain>` plus a `RwLock`-guarded or sharded hull cache, at which point the sequential phase disappears entirely.

### 10.8 C++ features that need a different design

| C++ mechanism | Where | Rust replacement |
| --- | --- | --- |
| `virtual ITEM* Clone()` | `pcbnew/router/pns_item.h:154` | `#[derive(Clone)]` on `Item`, `arena.insert(item.clone())` |
| `dyn_cast<T*>` via static `ClassOf` | `pcbnew/router/pns_line.h:111` and siblings | `match item.body { ItemBody::Line(l) => .. }` |
| unchecked `static_cast<const NODE*>(Owner())` | `pcbnew/router/pns_item.cpp:357`, `:359` | typed `NodeId` field; the cast cannot be expressed |
| `mutable int m_marker` with `const` `Mark()` | `pcbnew/router/pns_item.h:261-263`, `:327` | `fn mark(&self, world: &mut World, ...)`; the interior mutability disappears |
| `ITEM : OWNABLE_ITEM, ITEM_OWNER` diamond-ish multiple inheritance | `pcbnew/router/pns_item.h:97` | plain data; ownership lives in the arena, not in the item |
| raw `INDEX*` owned by `NODE` | `pcbnew/router/pns_node.h:604` | owned `Index` value inside `Node` |
| `NODE*` back-pointers `m_parent`, `m_root`, `m_children` | `pcbnew/router/pns_node.h:595-597` | `NodeId` into a node arena |
| `std::unique_ptr` in, `release()` immediately | `pcbnew/router/pns_node.cpp:620`, `:654`, `:759`, `:786` | `fn add(&mut self, item: Item) -> ItemId` |
| function-local static uid counter, non-atomic | `pcbnew/router/pns_item.cpp:362-366` | `AtomicU64` on the world, or drop uids entirely in favour of `ItemId` |
| static local in the default `HullCache` | `pcbnew/router/pns_node.h:179-181` | not expressible; the trait returns `Rc<_>` |
| `std::set<OBSTACLE>` ordered by pointer address | `pcbnew/router/pns_node.h:103-110` | `Vec<Obstacle>` deduped by `ItemId`, then sorted by a deterministic geometric key |
| 128 KiB of `std::array` on the stack in `AssembleLine` | `pcbnew/router/pns_node.cpp:1135-1139` | `VecDeque<(Point, ItemId, bool)>` or two `Vec`s grown from the middle |
| `VERTEX*` neighbours into a reserved `std::vector` | `pcbnew/router/pns_line.cpp:330`, `:402` | `Vec<u32>` indices into `vts`; the reserve becomes unnecessary |
| `wxCHECK` / `assert` returning sentinel values mid-expression | `pcbnew/router/pns_via.h:230`, `:305` | `Result` or `Option`, propagated |
| `void*` net handle | `pcbnew/router/pns_item.h:55` | newtype `NetHandle(u32)` or `NetId` into a host-side table |

### 10.9 Suggested build order for the port

1. `LayerRange`, `NetHandle`, `MarkerFlags`, `Kind`. Pure value types, direct translation of `pcbnew/router/pns_layerset.h` and `pcbnew/router/pns_item.h:42-114`.
2. Item arena, `ItemBody`, `Shape`/`Hull` per body. Port the hull builders from `pcbnew/router/pns_utils.cpp` including the degenerate-segment hacks at `:207-260`.
3. `Index`: per-layer R-trees plus a net map plus a membership set. Get `Clone` cheap from day one.
4. `JointMap`, `touch_joint`, `link_joint`, `unlink_joint`, `rebuild_joint`.
5. `Node` with add/remove/branch/commit and the single `visit_candidates` helper.
6. `collide_simple` with the exact clearance ladder from `pcbnew/router/pns_item.cpp:178-222` and the `- 1` at `:249`.
7. `Line`, `assemble_line`, `follow_line`.
8. `nearest_obstacle`, then `Line::walkaround`.
9. `RuleResolver` trait plus a trivial fixed-clearance implementation for tests.
10. Only then the placer, shove and dragger.

Steps 1 through 9 are the whole of this note. `TOPOLOGY` beyond `AssembleCluster` is step 11 or later.

---

## 11. Inventory: where pointer identity is essential

Every entry here is a place where the C++ relies on the *address* of an object rather than its value, and where a naive Rust port using indices will silently change behaviour unless handled.

1. **`OWNABLE_ITEM::BelongsTo`** (`pcbnew/router/pns_item.h:82`) is pure pointer equality against the owner. It drives every free/no-free decision: `~NODE` (`pcbnew/router/pns_node.cpp:98`), `~LINE` (`pcbnew/router/pns_line.cpp:78`), `~VIA` (`pcbnew/router/pns_via.h:154`), `~SOLID` (`pcbnew/router/pns_solid.h:51`), `doRemove` (`pcbnew/router/pns_node.cpp:815`, `:825`, `:837`), `releaseGarbage` (`:1602`).

2. **`NODE::m_override` is a `std::unordered_set<ITEM*>`** (`pcbnew/router/pns_node.h:599`). Shadowing is by address; two structurally identical items are distinct overrides.

3. **`OBSTACLE::operator<` and `operator==` order by `(uintptr_t)m_head` then `(uintptr_t)m_item`** (`pcbnew/router/pns_node.h:103-110`). Consequences: `std::set<OBSTACLE>` iteration order is address order, so `CheckColliding`'s `*obs.begin()` (`pcbnew/router/pns_node.cpp:522`) picks an arbitrary obstacle, and `NearestObstacle`'s no-intersection fallback `obstacles[0]` (`:472`) does too. Neither is reproducible across runs.

4. **`OBSTACLE::m_head` points at a stack temporary** in both line paths (`pcbnew/router/pns_node.cpp:310`, `:518`). It dangles immediately after the loop iteration. Nothing reads it (the only consumer is commented out at `pcbnew/router/pns_shove.cpp:1735`), and because the temporary reuses the same stack slot each iteration, the dedup key degenerates to `m_item` alone, which is the desired behaviour by accident.

5. **`RULE_RESOLVER` clearance cache keys on two `const ITEM*`** (`pcbnew/router/pns_kicad_iface.cpp:92-109`), canonically ordered by address (`:99-100`). A freed-and-reallocated item at the same address would silently inherit stale clearances; `ClearCacheForItems` (`pcbnew/router/pns_node.cpp:127`, `:1610`) exists specifically to prevent that. The temp cache deliberately keys by value instead (`:130-197`).

6. **`HullCache` keys on `const ITEM*`** (`pcbnew/router/pns_kicad_iface.cpp:222-236`) with the same lifetime hazard, and returns a reference into the map that any subsequent insertion can invalidate (`:844-847`).

7. **`const JOINT*` is the joint identity** across `TOPOLOGY::ConnectedJoints` (`pcbnew/router/pns_topology.cpp:73-104`), `followBranch::visitedJoints` (`:248`, `:319`, `:331`), and `NearestUnconnectedItem` (`:195`). But `touchJoint` erases and reinserts merged joints (`pcbnew/router/pns_node.cpp:1432`, `:1439`), and a branch's copy of a root joint has a different address (`:1411`). So two "equal" joints can compare unequal, and one joint's address can change under any mutation.

8. **`ITEM_SET::Contains` / `Erase` / `ExcludeItem`** are linear pointer searches (`pcbnew/router/pns_itemset.h:161`, `:166`, `pcbnew/router/pns_itemset.cpp:122`). `ROUTER::markViolations` uses `draggedItems.Contains( obs.m_item )` to decide what not to mark (`pcbnew/router/pns_router.cpp:726`).

9. **`LINK_HOLDER::ContainsLink` / `Unlink`** are pointer searches (`pcbnew/router/pns_link_holder.h:75`, `:57`). `NODE::Add(LINE&)` uses `!aLine.ContainsLink( rseg )` to avoid double-linking a reused segment (`pcbnew/router/pns_node.cpp:722`).

10. **`JOINT::NextSegment` compares `item != aCurrent` by address** (`pcbnew/router/pns_joint.h:246`), and `followLine` passes the current item through (`pcbnew/router/pns_node.cpp:1121`). Two identical segments at the same joint would break traversal.

11. **`shouldWeConsiderHoleCollisions` falls back to `parentI != parentH`** and `holeI->ParentPadVia() != aHead` (`pcbnew/router/pns_item.cpp:71`, `:75`, `:77`), with the geometric heuristic at `:65-69` explicitly compensating for the fact that a `LINE`'s via *copy* has a different address from the node's via.

12. **`ITEM::collideSimple` starts with `if( this == aHead ) return false`** (`pcbnew/router/pns_item.cpp:119`) and `DEFAULT_OBSTACLE_VISITOR` with `if( m_item == aCandidate )` (`pcbnew/router/pns_node.cpp:247`). Self-collision suppression is by address, so an item and its clone *will* collide with each other. This is why `NearestObstacle` can be called on a `LINE` that is already in the node.

13. **`VERTEX::neighbours` are `VERTEX*` into the `vts` vector** in `LINE::Walkaround` (`pcbnew/router/pns_line.cpp:330`, `:432`, `:438`, `:472`), kept valid only by the `reserve` at `:402`.

14. **`NODE::FindItemByParent` / `FindItemsByParent` compare `item->Parent() == aParent`** (`pcbnew/router/pns_node.cpp:1826`, `:1842`), a `BOARD_ITEM*` from the host.

15. **`INDEX::m_allItems` is `unordered_set<ITEM*>` and `m_netMap` is `map<NET_HANDLE, list<ITEM*>>`** (`pcbnew/router/pns_index.h:155-156`), so `Contains` and net-list removal are address based, and `NET_HANDLE` itself is an opaque host pointer used as a `std::map` key.

16. **`m_root->m_garbageItems`** (`pcbnew/router/pns_node.cpp:840`) is the deferred-free pool that exists solely so that stale `LINKED_ITEM*` links in live `LINE`s remain dereferenceable. Generational ids make this unnecessary; if you keep it, keep it deliberately.

17. **`ITEM_SET` sets itself as owner and never frees** (`pcbnew/router/pns_itemset.cpp:36-41` versus the empty destructor at `:31-33`). Anything relying on "the `ITEM_SET` will clean up" is relying on a leak.
