# KiCad PNS: DRAG_ALGO, DRAGGER, MULTI_DRAGGER and the host drag gestures

Reference architecture note for milestone 9 of this crate. Source tree: sparse checkout of KiCad master at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85`, read 2026-09-10. All `path:line` citations are relative to `/home/Tubbles/dev/ref/kicad/`.

Files read in full: `pcbnew/router/pns_drag_algo.h`, `pcbnew/router/pns_dragger.h`, `pcbnew/router/pns_dragger.cpp`, `pcbnew/router/pns_multi_dragger.h`, `pcbnew/router/pns_multi_dragger.cpp`, and, added on 2026-09-10 with section 11, `pcbnew/router/pns_component_dragger.h` and `pcbnew/router/pns_component_dragger.cpp`. Read in the parts that touch dragging: `pcbnew/router/pns_router.h/.cpp`, `pcbnew/router/router_tool.cpp`, `pcbnew/router/pns_tool_base.cpp`, `pcbnew/router/pns_line.cpp` (the drag primitives), `pcbnew/router/pns_optimizer.cpp` (the drag only passes), `pcbnew/router/pns_shove.h/.cpp` (the call surface), `pcbnew/router/pns_via.cpp` (`PushoutForce`), `pcbnew/router/pns_node.cpp` (`FixupVirtualVias`, `FindJoint`), `libs/kimath/src/geometry/shape_line_chain.cpp` (`PointAlong`), `qa/tools/pns/pns_log_player.cpp` (how the corpus drives a drag).

What is deliberately not here, because an earlier note has it:

- `ROUTER`'s enums, states and the branch tree, note 03 sections 1.1 and 1.2. Section 1 below extends the drag half rather than repeating it.
- `SHOVE`'s internals and its `ClearHeads` / `AddHeads` / `Run` protocol, note 04 sections 1.2 and 5.3 to 5.4. Section 4 below records only what the two draggers ask of it and what the crate already has.
- `OPTIMIZER`'s passes and effort flags, note 04 section 4. Section 6 below covers only `REQUIRE_OBTUSE_ANGLES`, which exists for the dragger and for nothing else.
- `NODE::FixupVirtualVias` and its two errata, note 02 section 3.15.
- The QA log format, the regression harness and the per case table, note 05 sections 6.2 to 6.9. Section 10 below adds the drag specific reading of the seven cases.
- `ROUTER_PREVIEW_ITEM`, snapping and undo granularity, note 05 sections 4.2 to 4.11.

A note on the checkout: `pcbnew/tools/` is not in this sparse clone, so `PCB_ACTIONS::routerInlineDrag`, `drag45Degree`, `dragFreeAngle` and `breakTrack` cannot be read where they are defined. Everything said about them below is read off their use sites in `pcbnew/router/router_tool.cpp`. `common/advanced_config.cpp` is likewise absent, so the default values of `ADVANCED_CFG::m_MaxTangentAngleDeviation` and `m_MaxTrackLengthToKeep` are unknown; both are arc only.

---

## 0. Read this first: three names in the task brief do not exist

`SHOVE::ShoveLines` and `SHOVE::ShoveMultiLines` do not exist anywhere in `pcbnew/router/` at this commit. A full tree grep returns nothing. Note 04 section 0 already records this for `ShoveLines`; it holds for `ShoveMultiLines` too. Neither dragger calls anything of the sort: both drive the shove through `ClearHeads()` / `AddHeads(...)` / `Run()` (`pcbnew/router/pns_shove.h:80` to `:84`), the single dragger with one head and the multi dragger with one head per line.

`SHOVE::ShoveDraggingVia` is declared at `pcbnew/router/pns_shove.h:86` and **has no definition anywhere in the tree**. Its only mention outside the header is the commented out tail of `pcbnew/router/pns_dragger.cpp:919`:

```cpp
SHOVE::SHOVE_STATUS st = m_shove->Run(); //ShoveDraggingVia( m_draggedVia, aP, newVia );
```

so the via drag runs an ordinary `Run()` over a via head. `src/shove.rs`'s module documentation already records it as a dead declaration and does not port it. That decision stands: nothing in this note needs it.

The one shove entry point the draggers do use besides the head protocol is **none**. `SHOVE::ShoveObstacleLine` (`pcbnew/router/pns_shove.h:88`) is public and is called directly by `DIFF_PAIR_PLACER` (note 04 section 5.5), not by either dragger. This crate has it as `Shove::shove_obstacle_line` (`src/shove.rs:2264`) and the dragger will not need it.

---

## 1. `DRAG_ALGO`: the interface, and how `ROUTER` drives it

### 1.1 The virtual list

`DRAG_ALGO` (`pcbnew/router/pns_drag_algo.h:43`) derives from `ALGO_BASE`, so it inherits the router pointer, `Settings()`, `Router()`, `Dbg()` and `Logger()` (note 03 section 1.1). It adds one data member, `NODE* m_world` (`:128`), and twelve methods.

| Method | Line | Contract |
| --- | --- | --- |
| `SetWorld( NODE* )` | `:61` | virtual, default body stores into `m_world`. Neither subclass overrides it. |
| `Start( const VECTOR2I&, ITEM_SET& )` | `:72` | pure. Begin a drag at a point with a set of anchor items. False means the drag never started. |
| `Drag( const VECTOR2I& )` | `:80` | pure. Move to a point. The bool is "this position has a valid solution", not "something happened". |
| `FixRoute( bool aForceCommit )` | `:89` | pure. Commit or refuse. |
| `CurrentNode() const` | `:96` | pure. The node holding everything the drag changed. |
| `CurrentNets() const` | `:103` | pure. |
| `CurrentLayer() const` | `:110` | pure. |
| `Traces()` | `:117` | pure, **non const**. The items the drag is moving. |
| `SetMode( DRAG_MODE )` | `:119` | virtual with an **empty default body**, so `COMPONENT_DRAGGER` silently discards the mode. |
| `Mode() const` | `:121` | pure. **No caller anywhere in the tree.** |
| `GetForceMarkObstaclesMode( bool* ) const` | `:123` | pure. Two answers in one call: the return is "the drag has fallen back to highlighting", the out parameter is the last drag status. |
| `GetLastCommittedLeaderSegments()` | `:125` | virtual, default returns an empty vector. Only `MULTI_DRAGGER` overrides it. |

The signature that matters most for a port is `Traces()`. It returns `const ITEM_SET` **by value** and is not const, because `MULTI_DRAGGER::Traces` returns a member that `Drag` rebuilds. An `ITEM_SET` holds raw `ITEM*` and can hold `LINE`s, which are values and not node items; the set therefore owns nothing and its contents are only valid until the next `Drag`.

### 1.2 `DRAG_MODE` is a request mask on the way in and a single value on the way out

`DRAG_MODE` (`pcbnew/router/pns_router.h:75` to `:84`):

```
DM_CORNER     = 0x01
DM_SEGMENT    = 0x02
DM_VIA        = 0x04
DM_FREE_ANGLE = 0x08
DM_ARC        = 0x10
DM_ANY        = 0x17   // corner | segment | via | free_angle | arc
DM_COMPONENT  = 0x20   // not in DM_ANY
```

The host passes a mask. `DRAGGER::Start` reads exactly **one bit** out of it:

```cpp
m_freeAngleMode = (m_mode & DM_FREE_ANGLE);     // pns_dragger.cpp:314
```

and then `startDragSegment` / `startDragVia` / `startDragArc` **overwrite** `m_mode` with a single value (`:130`, `:144`, `:148`, `:251`, `:262`). So `DM_CORNER`, `DM_SEGMENT`, `DM_VIA` and `DM_ARC` in the request are never consulted: the dragger picks its mode from the clicked item's kind and the click position, and the caller cannot ask for one. `StartDragging( p, item, DM_ANY )` and `StartDragging( p, item, DM_SEGMENT )` behave identically. The QA log player exploits this and passes plain `0` (`qa/tools/pns/pns_log_player.cpp:169`), which is why the corpus replays without recording a drag mode at all.

The one place the mask means something outside the free angle bit is `ROUTER_TOOL::CanInlineDrag` (`pcbnew/router/router_tool.cpp:2742`), which refuses a footprint drag when `DM_FREE_ANGLE` is set (`:2754`).

`DM_COMPONENT` never reaches a `DRAG_ALGO` either. `ROUTER::StartDragging` chooses `COMPONENT_DRAGGER` from the *shape of the item set*, not from the mode (section 1.3), and `COMPONENT_DRAGGER` does not override `SetMode`.

For the port: **model the mode as an output enum, not an input mask.** The only input flag is free angle.

### 1.3 `ROUTER::StartDragging`, both overloads

`pcbnew/router/pns_router.cpp:159`:

```cpp
bool ROUTER::StartDragging( const VECTOR2I& aP, ITEM* aItem, int aDragMode = DM_ANY )
{
    m_leaderSegments.clear();
    return StartDragging( aP, ITEM_SET( aItem ), aDragMode );
}
```

It is a one line forward. The default argument differs between the two declarations: the single item overload defaults to `DM_ANY`, the set overload to `DM_COMPONENT` (`pcbnew/router/pns_router.h:221`, `:222`). Given section 1.2 that difference is inert.

The real one, `pcbnew/router/pns_router.cpp:166`:

```
StartDragging(aP, aStartItems, aDragMode):
    m_leaderSegments.clear()                                   :168
    SetFailureReason("")                                       :169
    if aStartItems.Empty(): return false                       :171
    GetRuleResolver()->ClearCaches()                           :174

    # pick the algorithm from the shape of the set, not from aDragMode
    if aStartItems.Count(SOLID_T) == aStartItems.Size():       :176
        m_dragger = COMPONENT_DRAGGER; m_state = DRAG_COMPONENT
    elif aStartItems.Count(SEGMENT_T | ARC_T) > 1:             :182
        m_dragger = MULTI_DRAGGER;     m_state = DRAG_SEGMENT
    else:                                                      :187
        m_dragger = DRAGGER;           m_state = DRAG_SEGMENT

    m_dragger->SetMode(aDragMode)                              :193
    m_dragger->SetWorld(m_world.get())        # the ROOT       :194
    m_dragger->SetLogger(m_logger)                             :195
    m_dragger->SetDebugDecorator(iface->GetDebugDecorator())   :196

    if m_logger:                                               :198
        m_logger->Clear()
        if size == 1: Log(EVT_START_DRAG,      aP, items[0])   :204
        elif size > 1: LogM(EVT_START_MULTIDRAG, aP, items)    :206

    if m_dragger->Start(aP, aStartItems): return true          :209
    else: m_dragger.reset(); m_state = IDLE; return false      :213
```

Three things to carry over.

1. **The world handed to the dragger is the router's root**, not a branch (`:194`). Every branch the drag makes is the dragger's own business. Contrast `LINE_PLACER`, which is given a branch (note 03 section 1.2).
2. **The dispatch is on the item set, and an empty set is refused before anything else** (`:171`). "All solids" wins over "more than one segment", so a selection of pads is always a component drag.
3. `aStartItems.Count( ITEM::SEGMENT_T | ITEM::ARC_T ) > 1` is a count over a kind **mask**, so two segments, two arcs, or one of each all reach `MULTI_DRAGGER`. A single segment plus a via reaches `DRAGGER`, which then ignores the via: `DRAGGER::Start` reads `aPrimitives[0]` and nothing else (`pcbnew/router/pns_dragger.cpp:309`).

### 1.4 `moveDragging` and what the host sees

`pcbnew/router/pns_router.cpp:656`:

```cpp
bool ROUTER::moveDragging( const VECTOR2I& aP, ITEM* aEndItem )
{
    m_iface->EraseView();
    bool ret = m_dragger->Drag( aP );
    ITEM_SET dragged = m_dragger->Traces();
    m_leaderSegments = m_dragger->GetLastCommittedLeaderSegments();
    updateView( m_dragger->CurrentNode(), dragged, true );
    return ret;
}
```

`aEndItem` is accepted and **never used**. Compare `movePlacing` (`:789`), which draws the head with `PNS_HEAD_TRACE` and handles the trailing via specially: `moveDragging` does none of that, so dragged geometry reaches the host as ordinary added items out of the node delta, with no head flag. A port that wants the dragged line highlighted has to say so itself.

`updateView` is called with `aDragging = true` (`:665`), which reaches `m_iface->DisplayItem( item, clearance, aDragging )` (`:771`). `markViolations` still runs, because the router mode is `PNS_MODE_ROUTE_SINGLE` during a drag (`:757`), and it skips items that `GetDragger()->Traces()` contains (`:718` to `:727`) so that the thing being dragged is never marked as its own violation.

The dispatcher: `ROUTER::Move` (`:494`) routes both `DRAG_SEGMENT` and `DRAG_COMPONENT` to `moveDragging` (`:504` to `:506`).

### 1.5 `FixRoute`, `GetUpdatedItems`, `CommitRouting`, `StopRouting`

`ROUTER::FixRoute` (`:915`) logs `EVT_FIX` and forwards:

```cpp
case DRAG_SEGMENT:
case DRAG_COMPONENT:
    rv = m_dragger->FixRoute( aForceCommit );      // :928 to :931
```

`aP` and `aEndItem` are dropped on this branch: the dragger already knows where it is. `aForceFinish` is placer only. So the whole drag half of the facade is `bool FixRoute( bool aForceCommit )`.

`ROUTER::GetUpdatedItems` (`:833`) is what the QA harness reads, and it has a drag branch:

```cpp
else if ( m_state == DRAG_SEGMENT )          // :844
{
    node    = m_dragger->CurrentNode();
    current = m_dragger->Traces();
}
```

Note `DRAG_COMPONENT` is **not** handled, so a component drag reports nothing. Note also that this reads `CurrentNode()` and not a committed node: **the corpus golden for a drag case is the uncommitted delta after the last move.** No `EVT_FIX` is needed, and none of the seven drag logs has one (section 10).

`CommitRouting( NODE* )` (`:862`) is shared with routing and is unchanged for drags, including the removed plus added fold that preserves host object identity (note 03 section 1.3). The early return `if( m_state == ROUTE_TRACK && !m_placer->HasPlacedAnything() ) return;` (`:864`) is guarded on the state, so a drag always commits.

`StopRouting` (`:967`) resets `m_dragger` (`:985`), erases the view, sets `IDLE`, and calls `m_world->KillChildren()` and `ClearRanks()`. The nets pushed to the host for the ratsnest come from `m_placer->GetModifiedNets` (`:971` to `:979`), so **a drag never updates the ratsnest through this path**; the host has to do it. `GetCurrentNets`/`GetCurrentLayer` (`:1032`, `:1043`) do consult the dragger, but their only callers are in `pns_dp_meander_placer.cpp`, which never runs during a drag, so both dragger implementations of `CurrentLayer()` are unreachable in practice.

`GetLastCommittedLeaderSegments` (`:940`) returns `m_leaderSegments`, populated only by `moveDragging` (`:663`) from `MULTI_DRAGGER` (note 03 section 1.5). Single drag never contributes.

### 1.6 The node tree during a drag

```
world (root, owned by ROUTER, handed to the dragger by SetWorld)
 |
 +-- m_preDragNode = m_world->Branch()            pns_dragger.cpp:321
      |   built once in Start; carries startDragArc's stub segments
      |
      +-- SHOVE's own root = m_preDragNode        pns_dragger.cpp:325
      |    (only when RM_Shove and not free angle)
      |    and the springback stack above it
      |
      +-- m_lastNode
           mark obstacles:  m_preDragNode->Branch()          :390
           walkaround:      m_preDragNode->Branch()          :720
           shove:           m_shove->CurrentNode()->Branch() :851, :894, :939
```

`m_lastNode` is deleted and rebuilt on every `Drag` in the mark obstacles and walkaround paths (`:384` to `:390`, `:714` to `:720`), and in the shove path it is deleted (`:805`) and then re-branched from wherever the shove now stands. `CurrentNode()` is `m_lastNode ? m_lastNode : m_world` (`:1052`), so before the first `Drag` the dragger reports the untouched board.

`MULTI_DRAGGER` is flatter:

```
world
 +-- m_preShoveNode = m_world->Branch()   with every dragged line removed   :267 to :272
 |    +-- SHOVE root, springback stack
 +-- m_lastNode
      mark obstacles: m_world->Branch()                :587
      walkaround:     preWalkNode->Branch(), where preWalkNode = m_world->Branch()   :475, :496
      shove:          m_shove->CurrentNode()->Branch() :658
```

---

## 2. `DRAGGER`

### 2.1 State

`pcbnew/router/pns_dragger.h:154` to `:175`, with what each is really for:

| Member | Line | Role |
| --- | --- | --- |
| `VIA_HANDLE m_initialVia` | `:154` | the via as it was when the drag started. Used by `dragMarkObstacles` and `dragWalkaround` to find the fanout every time (`:438`, `:792`), so those two paths always re-derive from the original position. |
| `VIA_HANDLE m_draggedVia` | `:155` | updated by `dragShove` from `GetModifiedHeadVia` (`:935`). The shove path tracks the via as it moves; the other two do not. |
| `NODE* m_lastNode` | `:157` | the answer of the last `Drag`. |
| `NODE* m_preDragNode` | `:158` | branch of the world made once in `Start` (`:321`). |
| `int m_mode` | `:159` | a `DRAG_MODE` value, an `int` so `SetMode` can store a mask before `Start` narrows it. |
| `LINE m_draggedLine` | `:160` | the **original** assembled line, with its links, never re-dragged. Every drag starts from this. |
| `LINE m_lastDragSolution` | `:161` | the last successful post shove, post optimize line. Written **only** by the shove path (`:858`, `:901`). |
| `unique_ptr<SHOVE> m_shove` | `:162` | only allocated when `RM_Shove && !m_freeAngleMode` (`:323`). |
| `int m_draggedSegmentIndex` | `:163` | a **segment** index in `DM_SEGMENT`, a **point** index in `DM_CORNER`. See section 2.4. |
| `bool m_dragStatus` | `:164` | whether the last drag position is legal. |
| `PNS_MODE m_currentMode` | `:165` | `Settings().Mode()` sampled once in `Start` (`:313`), so changing the routing mode mid drag has no effect. |
| `ITEM_SET m_origViaConnections` | `:166` | **dead.** Declared and never read or written anywhere in the tree. |
| `VECTOR2D m_lastValidPoint` | `:167` | the last point that produced a solution. Declared as `VECTOR2D` although every assignment is a `VECTOR2I` (`:316`, `:1022`, `:1034`) and its only read passes it back to `Drag( const VECTOR2I& )` (`:983`). |
| `ITEM_SET m_draggedItems` | `:170` | what `Traces()` returns. |
| `bool m_freeAngleMode` | `:173` | from the request mask. |
| `bool m_forceMarkObstaclesMode` | `:174` | latched true when the very first drag fails (`:1029`) and never cleared. |
| `MOUSE_TRAIL_TRACER m_mouseTrailTracer` | `:175` | fed on every drag (`:1000`) and read for exactly one thing, the lead vector in `propagateViaForces` (`:67`). The posture half of the tracer is unused. |

The constructor (`:41` to `:54`) sets `m_mode = DM_SEGMENT`, `m_currentMode = RM_MarkObstacles`, everything else false or null. `SetMode` overwrites `m_mode` before `Start` runs, so the constructor's value never survives.

The destructor (`:57`) is **empty**: `m_lastNode` and `m_preDragNode` are raw pointers into the branch tree and are reclaimed by `ROUTER::StopRouting`'s `m_world->KillChildren()` (`pcbnew/router/pns_router.cpp:990`).

### 2.2 `Start`

`pcbnew/router/pns_dragger.cpp:304`:

```
Start(aP, aPrimitives):
    if aPrimitives.Empty(): return false                        :306
    startItem = aPrimitives[0]                                  :309   # only the first is read

    m_lastNode = null                                           :311
    m_draggedItems.Clear()                                      :312
    m_currentMode = Settings().Mode()                           :313
    m_freeAngleMode = (m_mode & DM_FREE_ANGLE)                  :314
    m_forceMarkObstaclesMode = false                            :315
    m_lastValidPoint = aP                                       :316

    m_mouseTrailTracer.Clear(); AddTrailPoint(aP)               :318, :319
    m_preDragNode = m_world->Branch()                           :321

    if m_currentMode == RM_Shove and not m_freeAngleMode:        :323
        m_shove = SHOVE(m_preDragNode, Router())                 :325
        m_shove->SetLogger(Logger()); SetDebugDecorator(Dbg())   :326, :327
        m_shove->SetDefaultShovePolicy(SHP_SHOVE)                :328

    startItem->Unmark(MK_LOCKED)                                 :331

    switch startItem->Kind():                                    :336
        SEGMENT_T:
            vvia = checkVirtualVia(aP, seg)                       :341
            return vvia ? startDragVia(vvia) : startDragSegment(aP, seg)
        VIA_T:  return startDragVia(via)                         :349
        ARC_T:  return startDragArc(aP, arc)                     :352
        default: return false                                    :355
```

Notes for the port.

- **`startItem->Unmark( MK_LOCKED )` mutates the world item** (`:331`). It is how the dragger overrides a lock the host chose to override; `ROUTER_TOOL::performDragging` shows the confirmation dialog first (`pcbnew/router/router_tool.cpp:2525`), and `InlineDrag` clears the board level lock before `SyncWorld` instead, with a comment saying the lock cannot be reliably restored (`:2816` to `:2827`). The unmark is never undone.
- A `SOLID_T` start item falls into `default:` and returns false, which is how a single pad in the set reaches `DRAGGER` and refuses. The all solids case never gets here.
- `SetDefaultShovePolicy( SHP_SHOVE )` (`:328`) is a **no operation in this revision**. `m_defaultPolicy` is read at exactly two places, `pcbnew/router/pns_shove.cpp:1663` and `:1671`, both testing `& SHP_IGNORE`, and the constructor already sets `m_defaultPolicy = SHP_SHOVE` (`:209`). Nothing in the tree ever sets `SHP_IGNORE`. `src/shove.rs` correctly has no `set_default_shove_policy`.

### 2.3 `checkVirtualVia`

`:81` to `:115`. A click near a segment endpoint may really be a click on a virtual via that `NODE::FixupVirtualVias` planted at a width change or a locked segment end (note 02 section 3.15).

```
checkVirtualVia(aP, aSeg):
    w2 = aSeg->Width() / 2
    distA = |aP - aSeg->Seg().A|;  distB = |aP - aSeg->Seg().B|
    if distA <= w2: psnap = A                       :90
    elif distB <= w2: psnap = B                     :94
    else: return null                               :100
    jt = m_world->FindJoint(psnap, aSeg)            :103
    if not jt: return null
    for item in jt->LinkList():                     :108
        if item->IsVirtual() and item->OfKind(VIA_T): return item
    return null
```

`FindJoint( pos, item )` is the two argument overload (`pcbnew/router/pns_node.h:478`), which forwards to `FindJoint( pos, aItem->Layers().Start(), aItem->Net() )`.

The threshold here is `<=` (`:90`, `:94`) where `startDragSegment` uses `<` (`:128`). A click at exactly `w/2` from an endpoint is a via drag if a virtual via is there and a *segment* drag if not, never a corner drag. Cosmetic, but it is a behaviour a golden could pin.

**This crate has no virtual vias at all** (`src/node.rs` module documentation, `src/snapshot.rs:483`), so `checkVirtualVia` has nothing to find. Section 9 records the decision that has to be made about `FixupVirtualVias` before the via drag is finished.

### 2.4 `startDragSegment`: where the mode comes from

`:118` to `:152`, the single most important routine in the file.

```
startDragSegment(aP, aSeg):
    w2 = aSeg->Width() / 2                                                :120
    m_draggedLine = m_world->AssembleLine(aSeg, &m_draggedSegmentIndex)   :122
    m_lastDragSolution = m_draggedLine                                    :123

    distA = |aP - aSeg->Seg().A|;  distB = |aP - aSeg->Seg().B|           :125, :126

    if distA < w2 or distB < w2:                                          :128
        m_mode = DM_CORNER
        if distB <= distA: m_draggedSegmentIndex++                        :132
    elif m_freeAngleMode:                                                 :135
        if distB < distA
           and m_draggedSegmentIndex < m_draggedLine.PointCount() - 2     :138
           and not m_draggedLine.CLine().IsPtOnArc(idx + 1):              :139
            m_draggedSegmentIndex++
        m_mode = DM_CORNER                                                :144
    else:
        m_mode = DM_SEGMENT                                               :148
    return true
```

`NODE::AssembleLine( aSeg, &aOriginSegmentIndex )` returns the whole trivially connected line and writes back the **segment** index of `aSeg` inside it. The dragger then reuses that integer with two different meanings:

- in `DM_SEGMENT` it stays a segment index and goes to `LINE::DragSegment( aP, index )`;
- in `DM_CORNER` it becomes a **point** index. Segment `i` spans points `i` and `i + 1`, so the un-incremented index names the segment's `A` end and `++` names its `B` end. That is what `if( distB <= distA ) m_draggedSegmentIndex++` is doing.

**Free angle mode can never produce a segment drag.** The `elif` at `:135` forces `DM_CORNER` for a mid segment click. That is deliberate and it matches `LINE::DragSegment`, whose free angle branch is `assert( false )` (`pcbnew/router/pns_line.cpp:900` to `:903`).

The free angle branch's extra guards (`PointCount() - 2` and the arc test) are absent from the corner branch above it, so a click within `w/2` of an endpoint in free angle mode goes through the unguarded path. `dragCornerFree` handles the arc case itself by inserting a vertex (`pcbnew/router/pns_line.cpp:863` to `:878`), so this is survivable rather than wrong.

**The host's snapping is what makes corner drag reachable.** `TOOL_BASE::snapToItem` (`pcbnew/router/pns_tool_base.cpp:486` to `:505`) uses the *same* `w/2` threshold and returns the endpoint exactly when the cursor is inside it:

```cpp
SEG::ecoord w_sq = SEG::Square( li->Width() / 2 );
if( distA_sq < w_sq || distB_sq < w_sq )
    return ( distA_sq < distB_sq ) ? A : B;
else if( aItem->Kind() == ITEM::SEGMENT_T )
    return m_gridHelper->AlignToSegment( aP, seg->Seg() );
```

so by the time the point reaches `startDragSegment` it is either exactly an endpoint (distance 0, corner drag) or a point on the segment (segment drag). A port that snaps differently changes which mode the user gets.

### 2.5 `startDragVia`

`:257` to `:265`. Three lines: `m_initialVia = aVia->MakeHandle()`, `m_draggedVia = m_initialVia`, `m_mode = DM_VIA`, return true. It cannot fail.

`VIA::MakeHandle` (`pcbnew/router/pns_via.cpp:310`) builds `{ pos, layers, net }` plus a `valid` flag (`pcbnew/router/pns_via.h:45` to `:57`). The point of the handle is that the shove replaces via items wholesale, so a pointer would dangle; a position plus layer range plus net can be looked up again in whichever node the caller now stands on. This crate has `ViaHandle` with the same three fields and `Option` in place of `valid` (`src/shove.rs:267`), and `World::find_via_by_handle` (`src/node.rs:2262`).

### 2.6 `startDragArc`: out of scope, recorded

`:155` to `:254`. Arcs are not in this crate (`PLAN.md`, "Later, on hold by user decision"), so this is documentation only.

```
startDragArc(aP, aArc):
    maxDeviation = EDA_ANGLE(ADVANCED_CFG::m_MaxTangentAngleDeviation, DEGREES_T)  :157
    centralAngle = |aArc->CArc().GetCentralAngle()|                                :159
    if centralAngle + maxDeviation >= ANGLE_180:                                   :161
        SetFailureReason("Unable to drag arc tracks of %.1f degrees or greater.")  :164
        return false

    probe = m_world->AssembleLine(aArc, &probeIdx)                                 :170
    find the first arc index in probe and its first/last point                     :176 to :193
    isolatedStart = (firstArcPt == 0);  isolatedEnd = (lastArcPt == last)          :195, :196

    if isolatedStart or isolatedEnd:
        stubLen = max(1, KiROUND(m_MaxTrackLengthToKeep * IU_PER_MM) / 2)          :200, :201
        for each isolated end: add a tangent SEGMENT stub of stubLen
                               to m_preDragNode                                    :231, :241
        m_draggedLine = m_preDragNode->AssembleLine(aArc, &m_draggedSegmentIndex)  :244
    else:
        m_draggedLine = m_world->AssembleLine(aArc, &m_draggedSegmentIndex)        :248

    m_mode = DM_ARC
```

Two structural points worth keeping even though the arc geometry is not ported.

- This is the **only** writer to `m_preDragNode` other than `Branch()` itself, and it writes *before* the shove is constructed... no: the shove is constructed at `:325`, `startDragArc` runs at `:352`, so the stubs land in the shove's root node after the shove already holds a pointer to it. That works because `SHOVE` stores the node pointer and reads it lazily.
- The stubs exist so the arc has neighbours to be tangent to. `dragWalkaround` and `dragMarkObstacles` then collide the dragged line against `m_world` (`:545`, `:739`, `:770`), which is the **root** and therefore does not contain the stubs. So the stubs are visible to the shove and to `m_lastNode` but invisible to the collision test that decides whether to walk around. Recorded as erratum E12.

`EDA_ANGLE` is used by `pns_dragger.cpp` at `:157`, `:159`, `:161` and `:163` and **nowhere else in the file**, all four inside `startDragArc`. `RotatePoint` is not used by either dragger at all. `TODO.md` lists both as "needed by the dragger"; section 9 corrects that.

### 2.7 `findViaFanoutByHandle`

`:267` to `:302`. Given a via handle and a node, returns the set of things attached at that joint: every trivially connected `LINE` reachable from a linked segment or arc, plus at most one `VIA`.

```
findViaFanoutByHandle(aNode, handle):
    jt = aNode->FindJoint(handle.pos, handle.layers.Start(), handle.net)   :271
    if not jt: return {}
    foundVia = false
    for item in jt->LinkList():                                           :278
        if item is SEGMENT_T or ARC_T:
            l = aNode->AssembleLine(item, &segIndex)                      :284
            if segIndex != 0: l.Reverse()                                 :286
            rv.Add(l)
        elif item is VIA_T and not foundVia:
            rv.Add(item); foundVia = true                                 :293
    return rv
```

`if( segIndex != 0 ) l.Reverse()` is the invariant the callers depend on: **every line in the fanout starts at the via.** That is what lets `dragViaMarkObstacles` say `origLine.CLine().Find( aHandle.pos )` and get a valid corner index (`:468`).

The reverse condition is `!= 0` rather than "the via is not at point 0", which is the same thing only because the seed segment is linked to the joint. It is worth reproducing verbatim rather than reasoning about.

The `foundVia` guard means a stacked via pair at one joint contributes one item. `LinkList()` order is the joint's link vector order, which is insertion order in KiCad and must be a deterministic order here.
### 2.8 `Drag`: dispatch, the first drag fallback, and the restore

`:998` to `:1049`. This is the state machine, and it is small enough to transcribe whole.

```
Drag(aP):
    m_mouseTrailTracer.AddTrailPoint(aP)                     :1000
    firstDrag = (m_lastNode == null)                         :1002

    if m_freeAngleMode or m_forceMarkObstaclesMode:          :1005
        ret = dragMarkObstacles(aP)
    else:
        switch m_currentMode:                                :1011
            RM_MarkObstacles: ret = dragMarkObstacles(aP)
            RM_Shove:         ret = dragShove(aP)
            RM_Walkaround:    ret = dragWalkaround(aP)
            default:          ret = false

    if ret:
        m_lastValidPoint = aP                                :1022
    else:
        if firstDrag:
            # first collision resolution failed: fall back to highlighting, forever
            m_forceMarkObstaclesMode = true                  :1029
            ret = dragMarkObstacles(aP)
            if ret: m_lastValidPoint = aP
        elif m_lastNode:
            # restore the last solution
            parent = m_lastNode->GetParent()->Branch()       :1039
            delete m_lastNode
            m_lastNode = parent
            m_draggedItems.Clear()
            m_lastDragSolution.ClearLinks()
            m_lastNode->Add(m_lastDragSolution)              :1044
    return ret
```

Four behaviours to reproduce deliberately.

1. **`m_forceMarkObstaclesMode` is one way.** It latches on the first failure and is never cleared, so a drag that starts on top of an obstacle spends the rest of its life in highlight mode even after the cursor moves somewhere legal. The host reads it back through `GetForceMarkObstaclesMode` and shows "Track violates DRC. (Ctrl+click to commit anyway.)" (`pcbnew/router/router_tool.cpp:2572` to `:2587`).
2. **`dragMarkObstacles` returns `true` unconditionally** (`:448`), so the `firstDrag` fallback always succeeds and the restore branch is only ever reached from `dragWalkaround` (returns `ok`) or `dragShove` (returns `m_dragStatus`).
3. **The restore re-adds `m_lastDragSolution`, which only the shove path ever updates** (`:858`, `:901`). In walkaround mode `m_lastDragSolution` is still the *original* line from `startDragSegment` (`:123`), so a failed walkaround drag snaps the trace back to where it started rather than to the last good drag position. In `DM_VIA` shove mode it is likewise never written, so a failed via drag restores the original line and leaves the via wherever `m_draggedVia` last put it. Erratum E4.
4. **The restore branch never restores the via.** Only `m_lastDragSolution`, a `LINE`, is added back.

### 2.9 `dragMarkObstacles`

`:381` to `:449`.

```
dragMarkObstacles(aP):
    delete m_lastNode                                                     :384
    m_lastNode = m_preDragNode->Branch()                                  :390

    case DM_SEGMENT, DM_CORNER:                                           :394
        thresh = Settings().SmoothDraggedSegments() ? width / 4 : 0       :398
        origLine = m_draggedLine;  dragged = m_draggedLine                :399, :400
        dragged.SetSnapThreshhold(thresh)                                 :401
        dragged.ClearLinks()                                              :402
        if DM_SEGMENT: dragged.DragSegment(aP, m_draggedSegmentIndex)     :405
        else:          dragged.DragCorner(aP, idx, m_freeAngleMode)       :407
        m_lastNode->Remove(origLine); m_lastNode->Add(dragged)            :409, :410
        m_draggedItems = { dragged }                                      :412, :413

    case DM_ARC:                                                          :418
        same, with DragArc; a collapsed arc leaves an empty chain and
        Add() is then a no op, which the comment at :426 says is intended

    case DM_VIA:                                                          :437
        dragViaMarkObstacles(m_initialVia, m_lastNode, aP)

    m_dragStatus = Settings().AllowDRCViolations()
                   ? true
                   : !m_lastNode->CheckColliding(m_draggedItems)          :443 to :446
    return true                                                           :448
```

Three details.

- `dragged.ClearLinks()` happens **after** `origLine` is copied and **before** `DragSegment` in the mark obstacles path, whereas in the walkaround and shove paths the links are still present when `Remove` runs. Both work: `NODE::Remove( LINE& )` uses the links when it has them and falls back to geometry when it does not, and here the removal uses `origLine`, which kept its links.
- **The snap threshold is `width / 4` here and in `dragWalkaround` (`:727`), but `width / 2` in `dragShove` (`:818`).** Both carry the same `//TODO: Make threshold configurable` comment (`:397`, `:817`). That is a real per mode difference in how eagerly a dragged corner snaps onto its own neighbouring segments, not a typo to normalise away.
- `CheckColliding( m_draggedItems )` is the `ITEM_SET` overload (`pcbnew/router/pns_node.cpp:478`), and `m_draggedItems` holds `LINE`s, so the set overload decomposes them per segment. This crate's `World::check_colliding_items` (`src/node.rs:1807`) takes items only and its documentation names this exact call site as the reason the gap exists.

### 2.10 `dragViaMarkObstacles`

`:452` to `:489`. No forces, no walkaround: move the via to the cursor and drag each attached line's near corner with it.

```
dragViaMarkObstacles(aHandle, aNode, aP):
    m_draggedItems.Clear()
    fanout = findViaFanoutByHandle(aNode, aHandle)          :456
    if fanout.Empty(): return true                          :458
    for item in fanout:
        if item is LINE:
            draggedLine = copy of the line
            draggedLine.DragCorner(aP, origLine.CLine().Find(aHandle.pos),
                                   m_freeAngleMode)         :468
            draggedLine.ClearLinks()
            m_draggedItems.Add(draggedLine)
            m_lastNode->Remove(origLine); m_lastNode->Add(draggedLine)
        elif item is VIA:
            nvia = Clone(*via); nvia->SetPos(aP)            :478, :480
            m_draggedItems.Add(nvia.get())
            m_lastNode->Remove(via); m_lastNode->Add(move(nvia))
```

The `aNode` parameter is used only for the fanout lookup; every mutation goes to `m_lastNode` directly (`:473`, `:474`, `:483`, `:484`). Both call sites pass `m_lastNode` anyway (`:438`, and `dragShove`'s fallback at `:945` passes it too), so the parameter is redundant.

`m_draggedItems.Add( nvia.get() )` stores a raw pointer into a `unique_ptr` that is then moved into the node (`:484`). The item survives because the node owns it, but `Traces()` is handing out a pointer whose lifetime is the node's. A port with arena ids has no equivalent hazard.

### 2.11 `tryWalkaround` and `dragWalkaround`

`tryWalkaround` (`:685` to `:706`) is four lines of configuration and one call:

```cpp
WALKAROUND walkaround( aNode, Router() );
walkaround.SetSolidsOnly( false );                                 // :688
walkaround.SetIterationLimit( Settings().WalkaroundIterationLimit() );  // :691
walkaround.SetLengthLimit( true, 30.0 );                           // :692
walkaround.SetAllowedPolicies( { WALKAROUND::WP_SHORTEST } );      // :693
aWalk = aOrig;
RESULT wr = walkaround.Route( aWalk );
return wr.status[WP_SHORTEST] == ST_DONE ? (aWalk = wr.lines[WP_SHORTEST], true) : false;
```

The length limit factor is **30.0**, which is twenty times the placer's and ten times `MULTI_DRAGGER::tryWalkaround`'s 3.0 (`pcbnew/router/pns_multi_dragger.cpp:391`). Note 03 section 5.1 has the semantics: the walk is abandoned once the routed length exceeds `factor` times the direct length. A single drag is therefore allowed to take an enormously long detour before giving up, which is the difference the `walk-with-teardrops` and `walk_drag_seg_against_board_edge` cases exercise.

`dragWalkaround` (`:709` to `:799`):

```
dragWalkaround(aP):
    delete m_lastNode; m_lastNode = m_preDragNode->Branch()          :714, :720

    case DM_SEGMENT, DM_CORNER:                                      :724
        thresh = SmoothDraggedSegments() ? width / 4 : 0             :727
        dragged = draggedWalk = origLine = m_draggedLine             :728 to :730
        dragged.SetSnapThreshhold(thresh)
        if DM_SEGMENT: dragged.DragSegment(aP, idx)                  :735
        else:          dragged.DragCorner(aP, idx)   # free angle NOT passed :737
        if m_world->CheckColliding(&dragged):                        :739
            ok = tryWalkaround(m_lastNode, dragged, draggedWalk)
        else:
            draggedWalk = dragged; ok = true
        if draggedWalk.CLine().PointCount() < 2: ok = false           :749
        if ok:
            m_lastNode->Remove(origLine)                              :756
            optimizeAndUpdateDraggedLine(draggedWalk, origLine, aP)   :757

    case DM_ARC: the same with DragArc                                :762 to :790
    case DM_VIA: ok = dragViaWalkaround(m_initialVia, m_lastNode, aP) :791, :792

    m_dragStatus = ok
    return ok
```

Two things.

- `DragCorner( aP, idx )` at `:737` omits the free angle argument, unlike `dragMarkObstacles` at `:407` which passes `m_freeAngleMode`. It does not matter, because free angle mode is routed to `dragMarkObstacles` before the mode switch is reached (`:1005`), so `dragWalkaround` can never run with `m_freeAngleMode` true. Harmless inconsistency, worth not copying.
- **The collision probe is against `m_world`, the root, not `m_lastNode`.** At that moment `m_lastNode` is a fresh branch of `m_preDragNode` that still contains the original line, so for a straight segment drag the two answer the same. They differ for arcs, where `m_preDragNode` carries `startDragArc`'s stubs (erratum E12), and they would differ for any future caller that edits the branch first.

### 2.12 `dragViaWalkaround` and `propagateViaForces`

`propagateViaForces` (`:62` to `:78`):

```cpp
VIA* via = *vias.begin();                                        // :64
VECTOR2I lead = -m_mouseTrailTracer.GetTrailLeadVector();        // :67
const int iterLimit = Settings().ViaForcePropIterationLimit();   // :69
if( via->PushoutForce( node, lead, force, ITEM::ANY_T, iterLimit ) )
{
    via->SetPos( via->Pos() + force );
    return true;
}
return false;
```

The `std::set<VIA*>&` parameter is dead generality: only `*vias.begin()` is read and the one call site builds a set of exactly one (`:513` to `:515`). It is also the crate's determinism rule broken in miniature, since iterating a `std::set<VIA*>` is pointer ordered; with one element it cannot bite. Port it as a single via.

`MOUSE_TRAIL_TRACER::GetTrailLeadVector` (`pcbnew/router/pns_mouse_trail_tracer.cpp:279` to `:289`) is `last point - first point` of the trail, or `(0,0)` for fewer than two points. **Negated** here, so the lead points from the cursor back toward where the drag started: the via is pushed away from the direction of travel when the barycentric force stops working (`pcbnew/router/pns_via.cpp:187`).

`VIA::PushoutForce( NODE*, const VECTOR2I& aDirection, VECTOR2I& aForce, int aCollisionMask, int aMaxIterations )` (`pcbnew/router/pns_via.cpp:143` to `:232`) is the iterative overload this crate already has as the private `via_pushout_force` in `src/placer/line_placer.rs:2121`, including the deliberate reproduction of KiCad's discarded `force.Resize( threshold )` at `:207`. Section 9 records that it has to be lifted out of the placer module.

`dragViaWalkaround` (`:492` to `:566`):

```
dragViaWalkaround(aHandle, aNode, aP):
    m_draggedItems.Clear()
    fanout = findViaFanoutByHandle(aNode, aHandle)                    :496
    if fanout.Empty(): return true                                    :498

    viaPropOk = false
    for item in fanout:                                               :504
        if item is VIA:
            draggedVia = Clone(*via); draggedVia->SetPos(aP)          :508, :510
            m_draggedItems.Add(draggedVia.get())                      :511
            m_lastNode->Remove(via)                                   :517
            if propagateViaForces(m_lastNode, {draggedVia}):          :519
                viaTargetPos = draggedVia->Pos()                      :523
                viaPropOk = true
                m_lastNode->Add(move(draggedVia))                     :525
    if not viaPropOk: return false                                    :530

    for item in fanout:                                               :533
        if item is LINE:
            draggedLine.DragCorner(viaTargetPos,
                                   origLine.CLine().Find(aHandle.pos),
                                   m_freeAngleMode)                   :541
            if m_world->CheckColliding(&draggedLine):                 :545
                if not tryWalkaround(m_lastNode, draggedLine, walkLine): return false
                m_lastNode->Remove(origLine)                          :552
                optimizeAndUpdateDraggedLine(walkLine, origLine, aP)  :553
            else:
                m_draggedItems.Add(draggedLine)                       :557
                m_lastNode->Remove(origLine); m_lastNode->Add(draggedLine)
    return true
```

Three problems, all reproducible and all worth deciding on rather than inheriting.

- **The via is removed from `m_lastNode` before the force propagation and only re-added if it succeeds** (`:517`, `:525`). When `propagateViaForces` fails, the function returns false at `:530` having already deleted the via from the branch. `Drag` then takes the restore path, which re-branches from the parent (`:1039`) and throws the mutilated node away, so nothing leaks to the user. It is still a node that transiently violates "a drag never deletes the thing being dragged".
- **`optimizeAndUpdateDraggedLine` clears `m_draggedItems`** (`:617`). So the moment any fanout line needs a walkaround, the dragged via added at `:511` and every earlier fanout line are wiped out of the set, and `Traces()` reports only the last optimized line. That under-reports to `markViolations` (which then marks the dragged via as a violation of itself) and to the host preview. Erratum E5.
- **The optimizer anchor is `aP`, the raw cursor, not `viaTargetPos`** (`:553` versus `:541`). When the force propagation moved the via, the line was dragged to `viaTargetPos` but the optimizer is told to preserve a vertex at `aP`, which is not on the line. `optimizeAndUpdateDraggedLine` then falls into `bestAnchorForPoint` (`:590`, `:591`) and preserves whatever is nearest. Erratum E6.

`LINE walkLine( *l );` at `:539` is a dead initialization: `tryWalkaround` assigns `aWalk = aOrig` first thing (`:695`).

### 2.13 `dragShove`

`:802` to `:954`. Three cases, and the segment/corner one is the transcription target for this milestone.

```
dragShove(aP):
    delete m_lastNode                                                       :805

    case DM_SEGMENT, DM_CORNER:                                             :813
        thresh = SmoothDraggedSegments() ? width / 2 : 0                    :818   # /2, not /4
        draggedPreShove = m_draggedLine                                     :819
        draggedPreShove.SetSnapThreshhold(thresh)                           :820
        if DM_SEGMENT: draggedPreShove.DragSegment(aP, idx)                 :823
        else:          draggedPreShove.DragCorner(aP, idx)                  :825

        preShoveNode = m_shove->CurrentNode()                               :827
        if preShoveNode: preShoveNode->Remove(draggedPreShove)              :830

        policy = SHP_SHOVE | SHP_DONT_LOCK_ENDPOINTS                        :832
        if DM_CORNER and m_draggedSegmentIndex == 0: policy |= SHP_REVERSED :836, :837

        m_shove->ClearHeads()                                               :839
        m_shove->AddHeads(draggedPreShove, policy)                          :840
        ok = (m_shove->Run() == SH_OK)                                      :841

        draggedPostShove = draggedPreShove                                  :843
        if ok and m_shove->HeadsModified():
            draggedPostShove = m_shove->GetModifiedHead(0)                  :848

        m_lastNode = m_shove->CurrentNode()->Branch()                       :851

        if ok:
            draggedPostShove.ClearLinks(); Unmark()                         :855, :856
            optimizeAndUpdateDraggedLine(draggedPostShove, m_draggedLine, aP) :857
            m_lastDragSolution = move(draggedPostShove)                      :858
        m_dragStatus = ok

    case DM_ARC:  same, plus a guard that a collapsed arc with < 2 points
                  is never fed to AddHeads                                  :872 to :887

    case DM_VIA:                                                            :908
        m_shove->DisablePostShoveOptimizations(OPTIMIZER::LIMIT_CORNER_COUNT) :914
        m_shove->ClearHeads()
        m_shove->AddHeads(m_draggedVia, aP, SHP_SHOVE)                      :917
        st = m_shove->Run()                                                 :919
        if m_shove->HeadsModified():
            m_draggedVia = m_shove->GetModifiedHeadVia(0)                   :926, :935
        m_lastNode = m_shove->CurrentNode()->Branch()                       :939
        m_draggedItems.Clear()                                              :941
        if st != SH_OK: m_dragStatus = dragViaWalkaround(m_draggedVia, m_lastNode, aP)  :945
        else:           m_dragStatus = true
    return m_dragStatus
```

Points that decide behaviour.

- **`preShoveNode->Remove( draggedPreShove )`** (`:830`). `draggedPreShove` has already been re-shaped by `DragSegment`, but `LINE::DragSegment` does not touch the links, so `NODE::Remove( LINE& )` removes the *original* segments by link. The comment in `MULTI_DRAGGER` spells the same trick out (`pcbnew/router/pns_multi_dragger.cpp:731` to `:733`). It is exactly the ordering a port must not "clean up".
- **`SHP_REVERSED` when dragging corner 0** (`:836`). The corner at index 0 is the line's start, and the shove's endpoint locking works from the far end; the flag tells it which end is anchored. `SHP_DONT_LOCK_ENDPOINTS` is set for every segment and corner drag because both endpoints of a dragged line may legitimately move.
- **The optimizer's root line is `m_draggedLine`**, the original (`:857`), not the pre shove line. That is the baseline `LIMIT_CORNER_COUNT` wants, and this crate accepts and ignores it (`src/optimizer.rs`, `optimize`'s `root` parameter).
- **`m_lastNode` is branched off the shove's node whether or not the run succeeded** (`:851`, `:894`, `:939`). On failure the shove has already rewound its own stack, so this is the pre run state.
- **`DisablePostShoveOptimizations( LIMIT_CORNER_COUNT )` is called on every via drag move** (`:914`), with a comment calling it "a hack that disables it, before I figure out a more reliable solution". This crate has `Shove::disable_post_shove_optimizations` (`src/shove.rs:1333`).
- The via case escalates to `dragViaWalkaround` on shove failure (`:944`, `:945`) using `m_draggedVia`, which may already have been moved by a *previous* successful shove. So the fallback looks up the fanout at the last known via position, not the original.

### 2.14 `optimizeAndUpdateDraggedLine`, `bestAnchorForPoint`, `pointHasBadCorner`

`optimizeAndUpdateDraggedLine` (`:569` to `:619`) is where a successful **walkaround** or **shove** drag ends. Its five call sites are `:553` (`dragViaWalkaround`), `:757` and `:786` (`dragWalkaround`) and `:857` and `:900` (`dragShove`); `dragMarkObstacles` never calls it, which is what makes a mark obstacles drag follow the cursor exactly and is repeated in section 2.9. An earlier revision of this line said "in all three modes", which was wrong.

```
optimizeAndUpdateDraggedLine(aDragged, aOrig, aP):
    aDragged.ClearLinks(); aDragged.Unmark()                        :573, :574
    optimizer = OPTIMIZER(m_lastNode)                               :576

    effort = MERGE_SEGMENTS                                         :578
    if SmoothDraggedSegments():  effort |= MERGE_COLINEAR           :580, :581
    if GetRestrictAngles():      effort |= REQUIRE_OBTUSE_ANGLES    :583, :584
    optimizer.SetEffortLevel(effort)

    anchor = aP                                                     :588
    if aDragged.CLine().Find(aP) < 0:                               :590
        anchor = bestAnchorForPoint(aDragged.CLine(), aP)
    optimizer.SetPreserveVertex(anchor)                             :593
    aDragged.Line().Split(anchor)                                   :594

    if not GetOptimizeEntireDraggedTrack():                         :598
        affectedArea = aDragged.ChangedArea(&aOrig)                 :600
        if not affectedArea: affectedArea = BOX2I(aP)               :603
        optimizer.SetRestrictArea(*affectedArea)                    :607

    optimizer.Optimize(&aDragged, &draggedPostOpt, &origLine)       :612
    aDragged = draggedPostOpt

    m_lastNode->Add(draggedPostOpt)                                 :616
    m_draggedItems.Clear(); m_draggedItems.Add(draggedPostOpt)      :617, :618
```

`SetPreserveVertex` also sets the `PRESERVE_VERTEX` flag as a side effect (`pcbnew/router/pns_optimizer.h:138` to `:142`), which is why the effort level above never mentions it. This crate's `Optimizer::set_preserve_vertex` (`src/optimizer.rs:1535`) does the same, and `optimize_line` documents having to work around it.

The degenerate restrict area at `:603`, `BOX2I( aP )`, is a zero size box whose comment says "No valid area yet? set to minimum to disable optimization". `Line::changed_area` returning `None` is the crate's `!affectedArea`.

`pointHasBadCorner` (`:622` to `:636`): a vertex is bad when the two segments meeting there make an angle in `ANG_ACUTE | ANG_RIGHT | ANG_HALF_FULL`. Endpoints are never bad (`:624`). `DIRECTION_45( seg )` uses the default `a90 = false`, so a 90 degree corner classifies as `ANG_RIGHT` and not as straight.

`bestAnchorForPoint` (`:639` to `:682`): when the cursor is not on the line, the anchor to preserve is the nearest point; if that lands on a bad corner, walk outward one vertex at a time and take the first good one, preferring the closer of the left and right candidates:

```
bestAnchorForPoint(aLine, aP):
    nearest = aLine.NearestPoint(aP)                                :641
    vertIdx = aLine.Find(nearest)                                   :642
    if vertIdx < 0 or not pointHasBadCorner(aLine, vertIdx): return nearest   :644

    for offset in 1 .. PointCount()-1:                              :650
        rightIdx = vertIdx + offset
        candidate = (rightIdx in range and good) ? CPoint(rightIdx) : none
        leftIdx = vertIdx - offset
        if leftIdx >= 0 and good(leftIdx):
            if no candidate or |left - aP|^2 < |candidate - aP|^2: return left  :664
        if candidate: return candidate                              :672
    return nearest                                                  :681
```

The comparison at `:664` is on **squared** distances, so no rounding is involved. The tie goes to the right candidate.

### 2.15 `FixRoute`

`:957` to `:995`:

```
FixRoute(aForceCommit):
    node = CurrentNode()
    if not node: return false
    if m_dragStatus:                                        :963
        Router()->CommitRouting(node); return true
    elif m_forceMarkObstaclesMode:                          :968
        if aForceCommit: CommitRouting(node); return true   :970
        return false
    else:
        # shove/walkaround: everything committed will be legal even if the
        # current cursor solution is not
        Drag(m_lastValidPoint)                              :983
        node = CurrentNode()
        if node and m_dragStatus: CommitRouting(node); return true
    return false
```

`aForceCommit` matters **only** on the `m_forceMarkObstaclesMode` branch. In plain `RM_MarkObstacles` mode with a colliding drag and `AllowDRCViolations()` false, `m_dragStatus` is false and `m_forceMarkObstaclesMode` is false, so Ctrl+click does nothing except re-drag to the last valid point. Erratum E7.

The re-drag at `:983` is a full `Drag` call, so it goes through the shove or the walkaround again and can itself fail, and it appends another trail point to the mouse trail tracer.

### 2.16 The small accessors

- `CurrentNode()` (`:1052`): `m_lastNode ? m_lastNode : m_world`.
- `Traces()` (`:1058`): `m_draggedItems`, by value.
- `CurrentNets()` (`:372`): `{ m_draggedVia.net }` in `DM_VIA`, else `{ m_draggedLine.Net() }`. Note it tests `m_mode == PNS::DM_VIA` with `==` on what is nominally a mask, which is correct only because `Start` narrowed it.
- `CurrentLayer()` (`pns_dragger.h:98`): `m_draggedLine.Layer()`. Wrong for a via drag, where `m_draggedLine` was never assigned, but unreachable (section 1.5).
- `Mode()` (`:366`): casts `m_mode` back to `DRAG_MODE`. No caller anywhere.
- `GetForceMarkObstaclesMode` (`pns_dragger.h:124`): writes `m_dragStatus` through the pointer, returns `m_forceMarkObstaclesMode`.
- `GetOriginalLine()` (`pns_dragger.h:103`): one caller, `TOOL_BASE::checkSnap` (`pcbnew/router/pns_tool_base.cpp:308`), which refuses to snap the cursor to a segment that belongs to the line being dragged. A port needs the equivalent or the drag will snap to itself.
- `GetLastDragSolution()` (`pns_dragger.h:108`): **no callers**. Dead.

### 2.17 Free angle mode, end to end

The complete path, so it can be built or skipped as one unit:

1. The host runs `PCB_ACTIONS::dragFreeAngle` and calls `performDragging( DM_ANY | DM_FREE_ANGLE )` (`pcbnew/router/router_tool.cpp:2452` to `:2455`).
2. `DRAGGER::Start` latches `m_freeAngleMode` (`:314`) and **does not construct the shove** (`:323`).
3. `startDragSegment` forces `DM_CORNER` for any click, mid segment included (`:135` to `:145`).
4. `Drag` routes unconditionally to `dragMarkObstacles` (`:1005`), so free angle never walks around and never shoves.
5. `dragMarkObstacles` calls `DragCorner( aP, idx, true )` (`:407`), which is `LINE::dragCornerFree` (`pcbnew/router/pns_line.cpp:888` to `:891`): set the point, `Simplify()`, done. No 45 degree rebuild.
6. `optimizeAndUpdateDraggedLine` is never reached, so a free angle drag is never optimized.
7. `ROUTER_TOOL::CanInlineDrag` refuses free angle for footprints (`pcbnew/router/router_tool.cpp:2754`).

Free angle mode is therefore about 15 lines of new code on top of a working corner drag, and it is completely independent of the shove and the walkaround.

### 2.18 Settings and thresholds the dragger reads, consolidated

| Setting | Read at | Effect |
| --- | --- | --- |
| `Settings().Mode()` | `:313` | sampled once into `m_currentMode`; picks mark obstacles, shove or walkaround. Changing it mid drag does nothing. |
| `SmoothDraggedSegments()` | `:398`, `:727`, `:818`, `:580` | corner snap threshold `width/4` (mark obstacles, walkaround) or `width/2` (shove), and `MERGE_COLINEAR` in the post drag optimizer. |
| `AllowDRCViolations()` | `:443` | mark obstacles reports success regardless of collisions. |
| `GetRestrictAngles()` | `:583` | adds `REQUIRE_OBTUSE_ANGLES` to the post drag optimizer. Three of the seven corpus drag cases set it. |
| `GetOptimizeEntireDraggedTrack()` | `:598` | when false, the optimizer is restricted to `ChangedArea`. Every corpus case has it false. |
| `WalkaroundIterationLimit()` | `:691` | the walkaround budget. |
| `ViaForcePropIterationLimit()` | `:69` | the via pushout budget. |
| length limit factor `30.0` | `:692` | hard coded in `tryWalkaround`. |

Every one of these except the hard coded 30.0 already exists on `RoutingSettings` in this crate (`src/settings.rs:188`, `:194`, `:207`, `:228`, `:278`, and the walkaround limit at `src/walkaround.rs:264`).
---

## 3. The `LINE` drag primitives

`LINE::DragCorner`, `DragSegment` and `DragArc` plus their four private helpers and two snappers live in `pcbnew/router/pns_line.cpp:722` to `:1397`. `src/line.rs`'s "What is not ported" list names them as milestone work, with one exception: `DragCorner` in its `aFreeAngle = false`, no preferred direction form is already there (`src/line.rs:1019`), because the shove's via fanout drag needs it (`pcbnew/router/pns_shove.cpp:1101`).

Declarations (`pcbnew/router/pns_line.h`):

```cpp
void DragSegment( const VECTOR2I& aP, int aIndex, bool aFreeAngle = false );          // :237
void DragCorner( const VECTOR2I& aP, int aIndex, bool aFreeAngle = false,
                 DIRECTION_45 aPreferredEndingDirection = DIRECTION_45() );           // :238
void DragArc( const VECTOR2I& aP, int aIndex );                                       // :240
```

### 3.1 `dragCornerInternal`

`:722` to `:820`. A free function, not a member. Given a chain and a target point, rebuild the chain so it ends at the target, keeping as much of the original as possible.

```
dragCornerInternal(aOrigin, aP, aPreferredEndingDirection = UNDEFINED):
    if aOrigin.PointCount() == 1:                                        :730
        return DIRECTION_45().BuildInitialTrace(aOrigin[0], aP)
    if aOrigin.SegmentCount() == 1:                                      :734
        dir = DIRECTION_45(aOrigin[0] - aOrigin[1])
        return DIRECTION_45().BuildInitialTrace(aOrigin[0], aP, dir.IsDiagonal())

    d = 1                                                                :743
    for i from aOrigin.SegmentCount() - d down to 0:                     :745
        d_start = DIRECTION_45(aOrigin.CSegment(i))
        p_start = aOrigin.CPoint(i)
        d_prev  = (i > 0) ? DIRECTION_45(aOrigin.CSegment(i-1)) : UNDEFINED

        for j in 0, 1:                                                   :755
            paths[j] = d_start.BuildInitialTrace(p_start, aP, j)
            if paths[j].SegmentCount() >= 1: dirs[dirCount++] = DIRECTION_45(paths[j].CSegment(0))

        # 1. a path whose LAST segment matches the preferred direction
        if aPreferredEndingDirection != UNDEFINED:                       :768
            pick the first paths[j] with DIRECTION_45(paths[j].CSegment(-1)) == preferred
        # 2. a path whose FIRST segment continues the segment at i
        if not picked:                                                   :781
            pick the first paths[j] with dirs[j] == d_start
        if picked: break
        # 3. a path whose first segment is obtuse to the segment before i
        for j: if dirs[j].IsObtuse(d_prev): picked = paths[j]; break     :796
        if picked: break

    if picked:                                                           :809
        return aOrigin.Slice(0, i) ++ picked
    dir = DIRECTION_45(aOrigin.CLastPoint() - aOrigin[PointCount()-2])    :817
    return DIRECTION_45().BuildInitialTrace(aOrigin.CPoint(0), aP, dir.IsDiagonal())
```

`int d = 2;` at `:726` is immediately overwritten by `d = 1;` at `:743`, whose guarding `if` is commented out at `:742` with "fixme: constant/parameter?". So the loop always starts at the last segment. Do not port the `d = 2` path.

The fallback at `:817` reads `aOrigin.CPoints()[ PointCount() - 2 ]`, which for a two point chain is index 0, so it is safe given the `SegmentCount() == 1` early return above it.

This crate has it as the private `drag_corner_internal` (`src/line.rs:1675`) with the preferred direction case omitted, because only the multi dragger passes one.

### 3.2 `dragCorner45` and `dragCornerFree`

`dragCorner45` (`:823` to `:854`):

```
dragCorner45(aP, aIndex, preferred):
    width   = m_line.Width()
    snapped = snapDraggedCorner(m_line, aP, aIndex)                          :828
    if aIndex == 0:                                                          :830
        path = dragCornerInternal(m_line.Reverse(), snapped, preferred).Reverse()
    elif aIndex == m_line.SegmentCount():                                    :834
        path = dragCornerInternal(m_line, snapped, preferred)
    else:
        if m_line.IsPtOnArc(aIndex + 1): m_line.Insert(aIndex+1, CPoint(aIndex+1))  :841
        path = dragCornerInternal(m_line.Slice(0, aIndex), snapped, preferred)      :845
             ++ dragCornerInternal(m_line.Slice(aIndex,-1).Reverse(), snapped, preferred).Reverse()
    path.Simplify(); path.SetWidth(width); m_line = path                     :851 to :853
```

The comment at `:844`, "fixme: awkward behaviour for outwards drags", is the known weakness of the middle case.

`dragCornerFree` (`:857` to `:882`) is the whole of free angle mode:

```
dragCornerFree(aP, aIndex):
    if m_line.IsPtOnArc(idx):                          # insert a draggable vertex
        if idx == 0 or not IsPtOnArc(idx-1):  Insert(idx, GetPoint(idx))     :867
        elif idx == numpts-1 or not IsArcSegment(idx): idx++; Insert(idx, GetPoint(idx))  :871, :872
        else: wxASSERT_MSG(false, "Attempt to dragCornerFree in the middle of an arc!")
    m_line.SetPoint(idx, aP)                                                 :880
    m_line.Simplify()                                                        :881
```

Without arcs that is two lines. Note `Simplify()` and not `Simplify2()`: KiCad's plain `Simplify` removes duplicate points and collinear runs with its own tolerance (note 01 section 6.5), which is `LineChain::simplify(0)` here.

`DragCorner` (`:884` to `:896`) is the dispatcher, with a `wxCHECK_RET( aIndex >= 0 )` guard.

### 3.3 `snapDraggedCorner`

`:1143` to `:1183`. Already ported as `Line::snap_dragged_corner` (`src/line.rs:1070`).

```
snapDraggedCorner(aPath, aP, aIndex):
    if m_snapThreshhold <= 0: return aP                     :1153
    s_start = max(aIndex - 2, 0);  s_end = min(aIndex + 2, SegmentCount() - 1)
    best = aP; best_dist = INT_MAX
    for i in s_start..=s_end:
        for j in s_start..i:
            a = CSegment(i); b = CSegment(j)
            if not DIRECTION_45(a).IsObtuse(DIRECTION_45(b)): continue    :1164
            ip = a.IntersectLines(b)                                      :1167
            if ip and |ip - aP| < m_snapThreshhold and < best_dist: best = ip
    return best
```

The early return is **after** `s_start` and `s_end` are computed, which is harmless. The crate hoists it, which changes nothing.

### 3.4 `snapToNeighbourSegments` and `dragSegment45`

`snapToNeighbourSegments` (`:1185` to `:1228`) is the segment drag's snapper, and it is a different rule from `snapDraggedCorner`: it snaps the dragged segment onto the **line** of a parallel segment two positions away.

```
snapToNeighbourSegments(aPath, aP, aIndex):
    if m_snapThreshhold == 0: return aP                       :1192   # note: == 0, not <= 0
    dragDir = DIRECTION_45(aPath.CSegment(aIndex))
    snap_d = {-1, -1}
    if aIndex >= 2:                                           :1195
        s = CSegment(aIndex - 2)
        if DIRECTION_45(s) == dragDir: snap_d[0] = s.LineDistance(aP)
        snap_p[0] = s.A
    if aIndex < SegmentCount() - 2:                           :1205
        s = CSegment(aIndex + 2)
        if DIRECTION_45(s) == dragDir: snap_d[1] = s.LineDistance(aP)
        snap_p[1] = s.A
    return the snap_p[i] with the smallest snap_d[i] that is >= 0 and <= m_snapThreshhold, else aP
```

Two traps. The guard is `== 0` where the corner snapper uses `<= 0`, so a negative threshold behaves differently between the two. And it returns `s.A`, a **point**, as the answer for what is used as a target the dragged segment's supporting line passes through; `dragSegment45` then builds `SEG s_current( target, target + drag_dir.ToVector() )` (`:1330`), so only the projection of that point onto the perpendicular matters.

`dragSegment45` (`:1230` to `:1397`) is the routine this milestone has to write from scratch. The shape of it:

```
dragSegment45(aP, aIndex):
    path = m_line
    target = snapToNeighbourSegments(path, aP, aIndex)                    :1240
    index = aIndex

    # guarantee a previous and a next segment exist, inserting zero length ones if not
    if index == 0 or path.IsPtOnArc(index):                               :1247
        path.Insert(index > 0 ? index+1 : 0, path.CPoint(index)); index++
    if index == path.SegmentCount() - 1:                                  :1253
        path.Insert(path.PointCount()-1, path.CLastPoint())
    elif path.IsPtOnArc(index + 1):                                       :1257
        path.Insert(index+1, path.CPoint(index+1))

    dragged  = path.CSegment(index);      drag_dir = DIRECTION_45(dragged)
    s_prev   = path.CSegment(index - 1);  dir_prev = DIRECTION_45(s_prev)
    s_next   = path.CSegment(index + 1);  dir_next = DIRECTION_45(s_next)

    # a neighbour parallel to the dragged segment cannot bend, so split it
    if dir_prev == drag_dir:  dir_prev = dir_prev.Left();  Insert(index, CPoint(index)); index++   :1271
    elif dir_prev == UNDEFINED: dir_prev = drag_dir.Left()                                          :1277
    if dir_next == drag_dir:  dir_next = dir_next.Right(); Insert(index+1, CPoint(index+1))         :1282
    elif dir_next == UNDEFINED: dir_next = drag_dir.Right()                                         :1287

    re-read s_prev, s_next, dragged after the inserts                     :1292 to :1294

    # two guide lines at each end: the two 45 degree ways out
    if aIndex == 0:            guideA = { dragged.A + drag_dir.Right(), dragged.A + drag_dir.Left() }   :1296
    elif dir_prev.Angle(drag_dir) & (ANG_OBTUSE | ANG_HALF_FULL):
                               guideA = { s_prev.A + drag_dir.Left(), s_prev.A + drag_dir.Right() }     :1306
    else:                      guideA[0] = guideA[1] = SEG(dragged.A, dragged.A + dir_prev)             :1310
    ... the mirror image for guideB at the far end                        :1313 to :1328

    s_current = SEG(target, target + drag_dir.ToVector())                 :1330

    best = the shortest of the four candidates:                           :1335
      for i, j in {0,1} x {0,1}:
        ip1 = s_current.IntersectLines(guideA[i]);  ip2 = s_current.IntersectLines(guideB[j])
        if not ip1 or not ip2: continue
        s1 = SEG(s_prev.A, ip1);  s2 = SEG(ip1, ip2);  s3 = SEG(ip2, s_next.B)
        if s1 intersects s_next: np = [s1.A, ip, s_next.B]                :1353
        elif s3 intersects s_prev: np = [s_prev.A, ip, s3.B]              :1359
        elif s1 intersects s3: np = [s_prev.A, ip, s_next.B]              :1365
        else: np = [s_prev.A, ip1, ip2, s_next.B]                         :1373
        keep np if np.Length() < best_len

    if m_line.PointCount() == 1:                m_line = best             :1387
    elif aIndex == 0:                           m_line.Replace(0, 1, best)          :1390
    elif aIndex == m_line.SegmentCount() - 1:   m_line.Replace(-2, -1, best)        :1392
    else:                                       m_line.Replace(aIndex, aIndex+1, best)  :1394
    m_line.Simplify()
```

Two things about the ending. The three `Replace` cases index into `m_line`, the **original**, using `aIndex`, the original index, while everything above worked on `path`, the copy with vertices inserted. That is deliberate: `best` spans from `s_prev.A` to `s_next.B` in the padded chain, which is the same pair of points as `aIndex` to `aIndex + 1` in the unpadded one whenever a pad was inserted at that end. And `m_line.PointCount() == 1` at `:1387` cannot happen: `wxASSERT( aIndex < m_line.PointCount() )` at `:1235` plus `CSegment(index - 1)` at `:1265` would already have failed. Dead branch.

The three "intersects" cases at `:1353`, `:1359` and `:1365` are the degenerate collapses: when the new segment's supporting line crosses one of the neighbours, the corner disappears and the result has three points instead of four.

### 3.5 `DragArc`, recorded and not ported

`:911` to `:1141`. Rebuild the arc as a circle tangent to two lines and passing through the cursor, clamped so the cursor stays in the region where such a circle exists (and `:1070` also pushes the cursor out of the maximal circle, so when one neighbour is not tangent to the arc the fallback constraint is the arc's own tangent stub, whose far end is the arc's own endpoint, and the drag can only shrink the arc; two long tangent neighbours let it grow. Found in slice 8a of the arcs, 2026-09-12, both directions pinned by `tests/arc_drag.rs`).

The shape, for the record: find the arc's index and its first and last point in the chain (`:916` to `:936`); build tangent lines at both arc endpoints (`:941` to `:947`); decide whether the neighbouring chain segments are collinear with those tangents within `ADVANCED_CFG::m_MaxTangentAngleDeviation` (`:949` to `:987`) and, if so, use them as the tangent constraints instead; intersect the two tangents (`:1017`, bailing out on parallel tangents at `:1019`); build the maximal tangent circle with `CIRCLE::ConstructFromTanTanPt` (`:1037`) and clamp the cursor into the feasible triangle (`:1053` to `:1070`); build the real circle through the clamped cursor (`:1073`); project the new centre onto both tangents for the new endpoints (`:1079`, `:1080`); if the new chord is shorter than `m_MaxTrackLengthToKeep` the arc is **dropped** and the chain is spliced without it (`:1089` to `:1102`); otherwise snap either new endpoint back onto its chain anchor when within the same limit (`:1104` to `:1124`) and splice the new `SHAPE_ARC` in (`:1126` to `:1140`).

This needs `SHAPE_ARC`, `CIRCLE`, `CalcArcMid` and the chain's arc index vector, none of which this crate has. It is the arc milestone, not this one.

---

## 4. What the two draggers ask of `SHOVE`, and what this crate already has

Both draggers use only the head protocol. The complete list of `SHOVE` members either of them touches:

| KiCad | Line | `DRAGGER` | `MULTI_DRAGGER` | This crate |
| --- | --- | --- | --- | --- |
| `SHOVE( NODE*, ROUTER* )` | `pns_shove.h:77` | `:325` | `:274` | `Shove::new(root)`, `src/shove.rs:1273` |
| `SetLogger`, `SetDebugDecorator` | `pns_algo_base.h:73` | `:326`, `:327` | `:275`, `:276` | `AlgoContext`, `src/algo_base.rs` |
| `SetDefaultShovePolicy` | `pns_shove.h:72` | `:328` | `:277`, `:647` | absent, and correctly so (section 2.2) |
| `ClearHeads` | `:80` | `:839`, `:884`, `:916` | `:648` | `Shove::clear_heads`, `:1366` |
| `AddHeads( LINE, policy )` | `:81` | `:840`, `:885` | `:653` | `Shove::add_head_line`, `:1387` |
| `AddHeads( VIA_HANDLE, pos, policy )` | `:82` | `:917` | never | `Shove::add_head_via`, `:1417` |
| `Run` | `:84` | `:841`, `:886`, `:919` | `:656` | `Shove::run`, `:4100` |
| `CurrentNode` | `:99` | `:827`, `:851`, `:894`, `:939` | `:658` | `Shove::current_node` |
| `HeadsModified( i )` | `:101` | `:847`, `:891`, `:924` | `:684` | `Shove::heads_modified(Option<usize>)`, `:1450` |
| `GetModifiedHead( i )` | `:102` | `:848`, `:892` | `:685` | `Shove::modified_head`, `:1466` |
| `GetModifiedHeadVia( i )` | `:103` | `:926` | never | `Shove::head_via`, `:1442` |
| `DisablePostShoveOptimizations` | `:109` | `:914` | never | `Shove::disable_post_shove_optimizations`, `:1333` |

Policies used: `SHP_SHOVE | SHP_DONT_LOCK_ENDPOINTS`, plus `SHP_REVERSED` for a corner 0 drag (`pns_dragger.cpp:832`, `:837`, `:882`); `SHP_SHOVE` alone for a via head (`:917`); `SHP_SHOVE | SHP_DONT_OPTIMIZE` for every multi drag head (`pns_multi_dragger.cpp:653`). `SHP_DONT_OPTIMIZE` has **no other user in the tree**, so the multi dragger is the reason that flag exists. All four are on `ShovePolicy` (`src/shove.rs:334`).

So the shove needs **nothing new** for milestone 9. Every entry point either dragger uses is already public on `Shove`.

---

## 5. `MULTI_DRAGGER`

The class comment is honest: "Dragging algorithm for multiple segments. Very trival version for demonstration purposes." (`pcbnew/router/pns_multi_dragger.h:44`). It is the newest and least finished code in this note, it has no regression coverage in the corpus (no log contains `EVT_START_MULTIDRAG`, note 05 section 6.9), and `PLAN.md` lists multi drag under "Non-goals for now". Everything below is therefore a record rather than a transcription target, with section 10 placing it last.

### 5.1 `MDRAG_LINE`, the per line record

`pcbnew/router/pns_multi_dragger.h:127` to `:154`. One of these per line the user selected.

| Field | Line | Meaning |
| --- | --- | --- |
| `leaderItem` | `:129` | declared, **never read or written anywhere**. Dead. |
| `originalLeaders` | `:130` | every selected primitive that assembled into this line. |
| `isStrict` | `:132` | the cursor is within `width/2` of this line's corner or leader segment. |
| `isMidSeg`, `isCorner` | `:133`, `:134` | which kind of grab this line offers. |
| `isDraggable` | `:135` | set true at construction (`:81`) and never set false. Dead guard at `:820`. |
| `leaderSegIndex` | `:137` | for corner mode a point index, for segment mode a **link** index. See section 5.2. |
| `cornerIsLast` | `:138` | whether the grabbed corner is the chain's last point. |
| `originalLine` | `:140` | as assembled, possibly reversed by `Start`. |
| `preDragLine` | `:141` | `originalLine` at the top of each `tryPosture`, possibly with the last point removed. |
| `draggedLine` | `:142` | the result. |
| `preShoveLine` | `:143` | declared, **never read or written**. Dead. |
| `dragOK` | `:145` | this line produced a usable drag this posture. |
| `isPrimaryLine` | `:146` | the cursor is attached to this one. |
| `clipDone` | `:147` | declared, **never read or written**. Dead. |
| `offset` | `:148` | declared, **never read or written**. Dead. |
| `midSeg` | `:149` | the grabbed segment, in segment mode. |
| `dragDist` | `:150` | signed distance along the perpendicular, used to order the shove heads and the walkaround attempts. Only ever written in the segment branch (`:907`), so in corner mode every line sorts equal. |
| `cornerDistance`, `leaderSegDistance` | `:151`, `:152` | how far the cursor is from this line's corner and from its leader segment. |
| `mdragIndex` | `:153` | index into `m_mdragLines`, added so `multidragShove` can tell which lines were dropped (`:663` to `:676`). |

Five of eighteen fields are dead. That is the strongest single signal about the maturity of this file.

### 5.2 `MULTI_DRAGGER::Start`: choosing the leaders and the mode

`:45` to `:281`. Four phases.

**Phase 1, deduplicate into lines** (`:60` to `:85`). For every primitive, if some already built `MDRAG_LINE`'s `originalLine.ContainsLink( litem )` then record it as another leader of that line and skip; otherwise assemble a new line. So a selection of five segments on one trace becomes one `MDRAG_LINE` with five `originalLeaders`.

**Phase 2, classify each line** (`:90` to `:174`):

```
thr = originalLine.Width() / 2
distFirst = |CPoint(0) - aP|;  distLast = |CLastPoint() - aP|
cornerDistance = min(distFirst, distLast)                              :100

ifirst = aPrimitives.FindVertex(CPoint(0))                             :104
ilast  = aPrimitives.FindVertex(CLastPoint())                          :103
takeFirst = (ifirst and ilast) ? distFirst < distLast : bool(ifirst)   :106 to :111

if ifirst or ilast:                                                    :113
    corner = takeFirst ? first : last
    cornerIsLast   = not takeFirst
    leaderSegIndex = takeFirst ? 0 : SegmentCount() - 1                :118, :131
    cornerDistance = the chosen distance;  isCorner = true
    if that distance <= thr: isStrict = true; cornerDistance = 0       :122, :135

for lidx, link in enumerate(originalLine.Links()):                     :145
    if link is a SEGMENT and aPrimitives.Contains(link):               :150
        d = link->Seg().Distance(aP)
        midSeg = link->Seg(); isMidSeg = true                          :155, :156
        leaderSegIndex = lidx                                          :157
        leaderSegDistance = d + thr                                    :158
        if d < thr and not isStrict:                                   :160
            isCorner = false; isStrict = true; leaderSegDistance = 0

if isStrict: anyStrictCornersFound |= isCorner
             anyStrictMidSegsFound |= not isCorner                     :171, :172
```

`ITEM_SET::FindVertex` (`pcbnew/router/pns_itemset.cpp:137` to `:150`) returns the first selected **segment** with an endpoint exactly equal to the given point, with a "fixme: biconnected concept" comment. So "the user selected a segment that touches this line's end" is what makes the line a corner candidate.

The link loop keeps overwriting `leaderSegIndex` and `midSeg`, so the **last** selected segment on the line wins, not the nearest to the cursor. And note `leaderSegIndex` has now been used with two different meanings on the same field: a point index in the corner branch and a link index in the loop.

**Phase 3, pick the drag mode** (`:176` to `:225`):

```
if anyStrictCornersFound: m_dragMode = DM_CORNER                      :176
elif anyStrictMidSegsFound: m_dragMode = DM_SEGMENT                   :178
else:
    bestCorner = argmin over cornerDistance                           :189
    bestSeg    = argmin over leaderSegDistance                        :194
    pick whichever distance is smaller, and mark that line primary    :201 to :223
    if neither exists: return false   # "can it really happen?"       :224
```

**Phase 4, corner mode sanity, primary selection, and the shove** (`:227` to `:280`):

```
if m_dragMode == DM_CORNER:                                           :227
    for l in m_mdragLines:
        if not l.cornerIsLast: l.originalLine.Reverse(); l.cornerIsLast = true   :232 to :235
        jt = m_world->FindJoint(l.originalLine.CLastPoint(), &l.originalLine)    :239
        if not jt: m_dragMode = DM_SEGMENT; break                     :241 to :245
        if not jt->IsTrivialEndpoint(): m_dragMode = DM_SEGMENT       :247 to :250   # no break

for l in m_mdragLines:                                                :254
    if (anyStrictCornersFound or anyStrictMidSegsFound) and l.isStrict:
        l.isPrimaryLine = true; break                                 :258, :259

m_origDraggedItems = aPrimitives                                      :263

if Settings().Mode() == RM_Shove:                                     :265
    m_preShoveNode = m_world->Branch()                                :267
    for l: m_preShoveNode->Remove(l.originalLine)                     :271
    m_shove = SHOVE(m_preShoveNode, Router())                         :274
    m_shove->SetDefaultShovePolicy(SHP_SHOVE | SHP_DONT_LOCK_ENDPOINTS)  :277
return true
```

`JOINT::IsTrivialEndpoint` (`pcbnew/router/pns_joint.h:176` to `:180`) is `m_linkedItems.Size() == 1 && Count(SEGMENT_T) == 1`, with its own "fixme: Arcs and trivial endpoint vias" comment. The intent at `:237`, "if it's connected (non-trivial fanout), disregard it", is to refuse corner mode when a selected line's end is soldered to something.

Two problems live in phase 4, both erratum material.

- The reversal at `:234` happens **before** the joint test, and switching to `DM_SEGMENT` afterwards leaves already reversed lines reversed while `leaderSegIndex` still refers to the pre reversal link order. The `break` at `:244` makes it worse by leaving the rest un-reversed, so the set ends in a mixed state. (E18)
- The primary selection at `:254` to `:261` takes the first line with `isStrict`, whatever kind of strictness it has. When one line has a strict corner and an earlier one has a strict mid segment, the mode is `DM_CORNER` from `anyStrictCornersFound` while the primary is the mid segment line. (E26)

Only the shove path branches the world here. Mark obstacles and walkaround branch inside their own routines, so mode switching between moves would half work.

### 5.3 `MULTI_DRAGGER::Drag` and `tryPosture`

`:704` to `:997`. The whole move is a local lambda `tryPosture( int aVariant )` run for variants 0, 1 and 2 until one returns true (`:968` to `:974`), followed by a dispatch on the routing mode.

```
tryPosture(aVariant):
    for l in m_mdragLines:                                            :721
        l.dragOK = false;  l.preDragLine = l.originalLine
        if l.isPrimaryLine:
            primaryDragged = l.originalLine; primaryDragged->ClearLinks()   :734, :735
            primaryPreDrag = l.originalLine                           :736
            primaryLine = &l

    if aVariant == 1 and primaryPreDrag->PointCount() > 2:            :742
        drop the last point of primaryPreDrag, primaryDragged and every preDragLine   :744 to :750

    completed.clear()
    snapThreshold = SmoothDraggedSegments() ? primaryDragged->Width()/4 : 0    :755

    if m_dragMode == DM_CORNER:                                       :757
        lastPreDrag = primaryPreDrag->CSegment(-1)                    :762
        primaryDir  = DIRECTION_45(lastPreDrag)                       :763
        primaryDragged->SetSnapThreshhold(snapThreshold)
        primaryDragged->DragCorner(aP, PointCount()-1, false)         :766
        if primaryDragged->SegmentCount() == 0: return false           :791, :792
        lastPrimDrag = primaryDragged->CSegment(-1)                   :771
        if aVariant == 2: lastPrimDrag = lastPreDrag                  :773, :774
        if DIRECTION_45(lastSeg) != primaryDir and lastSeg.Length() < Width():
            lastPrimDrag = lastPreDrag                                :779 to :782
        perp = (lastPrimDrag.B - lastPrimDrag.A).Perpendicular()      :785
        primaryLastSegDir = DIRECTION_45(lastPrimDrag)                :786
    else:                                                             :800
        lastPreDrag = primaryDragged->CSegment(primaryLine->leaderSegIndex)   :805
        primaryDragged->SetSnapThreshhold(snapThreshold)
        primaryDragged->DragSegment(aP, primaryLine->leaderSegIndex)  :807
        perp = (primaryLine->midSeg.B - primaryLine->midSeg.A).Perpendicular()   :808
        m_guide = SEG(aP, aP + perp)                                  :809

    m_leaderSegments = m_origDraggedItems.CItems()                    :813
    m_draggedItems.Clear()

    for l in m_mdragLines:                                            :817
        if l.preDragLine.SegmentCount() >= 1:                         :826
            if m_dragMode == DM_CORNER:                               :837
                parallelDir = DIRECTION_45(l.preDragLine.CSegment(-1))
                if primaryDir.Angle(parallelDir) in {OBTUSE, RIGHT, STRAIGHT}:   :843
                    dist = lastPreDrag.LineDistance(l.preDragLine.CLastPoint(), true)   :849
                    projected = aP + perp.Resize(dist)                :852
                    parallelDragged = l.preDragLine; ClearLinks()
                    parallelDragged.DragCorner(projected, PointCount()-1,
                                               false, primaryLastSegDir)         :863
                    if parallelDragged.SegmentCount() < 1: continue   :871, :872
                    l.dragOK = true
                    if not l.isPrimaryLine:
                        l.draggedLine = parallelDragged; completed.push_back(l)   :878, :879
                        m_draggedItems.Add(parallelDragged)
            elif m_dragMode == DM_SEGMENT:                            :884
                if DIRECTION_45(lastPreDrag).Angle(DIRECTION_45(l.midSeg))
                       & (ANG_HALF_FULL | ANG_STRAIGHT):              :891
                    dist = lastPreDrag.LineDistance(l.preDragLine.CPoint(l.leaderSegIndex), true)  :893
                    projected = aP + perp.Resize(dist)                :895
                    sperp = SEG(aP, aP + perp.Resize(10000000))       :897
                    startProj = sperp.LineProject(m_dragStartPoint)   :898
                    v = projected - startProj
                    l.dragDist = |v| * sign(v.Dot(perp))              :907
                    l.dragOK = true
                    if not l.isPrimaryLine:
                        l.draggedLine = l.preDragLine; ClearLinks()
                        SetSnapThreshhold(snapThreshold)
                        DragSegment(projected, l.leaderSegIndex, false)   :915
                        completed.push_back(l)
        if l.isPrimaryLine:                                           :931
            l.draggedLine = *primaryDragged; l.dragOK = true; completed.push_back(l)

    if m_dragMode == DM_SEGMENT: return true                          :939
    for l in completed:                                               :943
        if not l.dragOK and aVariant < 2: return false                :945
        if l.isPrimaryLine: continue
        if l.draggedLine.SegmentCount() < 1: return false              :953
        if DIRECTION_45(l.draggedLine.CSegment(-1)) != primaryLastSegDir: return false   :958
    return true
```

Then:

```
for variant in 0, 1, 2: res = tryPosture(variant); if res: break      :968 to :974
switch Settings().Mode():                                            :976
    RM_Walkaround:    m_dragStatus = multidragWalkaround(completed)
    RM_Shove:         m_dragStatus = multidragShove(completed)
    RM_MarkObstacles: m_dragStatus = multidragMarkObstacles(completed)
return m_dragStatus
```

The mechanism, stated plainly: **drag the primary line, then move every other line to the point at the same perpendicular offset from the primary's last (or leader) segment that it had before the drag.** `perp` is the perpendicular of the primary's reference segment, `dist` is the signed line distance of the other line's endpoint from that reference segment (`LineDistance( p, true )` is the signed overload, `libs/kimath/include/geometry/seg.h:155`), and `aP + perp.Resize( dist )` is where that endpoint should now be. The comment at `:831` to `:835` says as much and calls the algorithm "quite trival".

The three variants are the entire fallback strategy: variant 0 drags the lines as they are, variant 1 drops each line's last point first, variant 2 keeps the primary's pre drag direction as the reference and, per `:945`, tolerates lines that failed to drag. `res` is assigned and then never read (`:966`, `:970`), so if all three fail the drag proceeds anyway with whatever `completed` the last variant produced. (E20)

`SEG lastPreDrag;` (`:710`) is default constructed to `(0,0)-(0,0)` and used at `:803`, one line **before** it is assigned at `:805`, to build a debug shape. Debug only, but it is read before it is written. (E19)

`m_guide` is written only in the segment branch (`:809`) and read only by `findNewLeaderSegment` (`:417`), which `restoreLeaderSegments` calls only in the non corner branch. Consistent.

`10000000` at `:897` is a hard coded 10 mm perpendicular used to project the drag start point. It only has to be long enough that `LineProject` is numerically sane, but it is a magic constant to record.

### 5.4 `multidragMarkObstacles` and `clipToOtherLine`

`multidragMarkObstacles` (`:573` to `:611`):

```
delete m_lastNode; m_lastNode = m_world->Branch()                     :577, :587
for every ordered pair (l1, l2) with l1 < l2:                         :590
    if clipToOtherLine(m_lastNode, l1.draggedLine, copy of l2.draggedLine):
        l2.draggedLine = the clipped copy                             :597, :598
for l in aCompletedLines:
    m_lastNode->Remove(l.originalLine); m_lastNode->Add(l.draggedLine)    :604, :605
restoreLeaderSegments(aCompletedLines)
```

It is the only mode that shortens lines against each other rather than routing around. The comment at `:583` to `:586` is the best one line description of the branch model in the whole tree: "m_lastNode contains the temporary (post-modification) state. Think of it as of an efficient undo buffer."

`clipToOtherLine` (`:294` to `:346`) is a binary search on arc length for the longest prefix of one line that does not collide with another:

```
clipToOtherLine(aNode, aRef, aClipped):
    clipLengthThreshold = 100                                         :299
    l = aClipped;  curL = l.CLine().Length();  step = curL / 2 - 1     :307, :308
    while step > clipLengthThreshold:                                 :310
        sl_tmp = aClipped.CLine()
        pclip = sl_tmp.PointAlong(curL)                               :313
        idx = sl_tmp.Split(pclip);  sl_tmp = sl_tmp.Slice(0, idx)      :314, :315
        l.SetShape(sl_tmp)
        if l.Collide(&aRef, aNode, l.Layer(), &ctx):                  :321
            didClip = true; curL -= step; step /= 2
        else:
            tightest = sl_tmp
            if didClip: curL += step; step /= 2
            else: break                                               :338
    aClipped.SetShape(tightest)                                       :343
    return didClip
```

Three things.

- `int curL = l.CLine().Length();` narrows a `long long int` (`libs/kimath/include/geometry/shape_line_chain.h:500`) into an `int`. A chain longer than about 2.147 metres overflows. (E25)
- If the very first probe collides and every later one does too, `tightest` is never assigned and `aClipped` is set to an **empty** chain while `didClip` is true, so the caller stores an empty line into `draggedLine`. (E25)
- `LINE::Collide( const LINE*, NODE*, layer, ctx )` at `:321` is a **line against line** collision, `pcbnew/router/pns_item.cpp:133`. `TODO.md` already names this exact call site as one of the four that need a per segment decomposition. This crate has `World::collide_lines` (`src/node.rs:1853`), which decomposes the obstacle side, so it is the right primitive.

`SHAPE_LINE_CHAIN::PointAlong( int aPathLength )` (`libs/kimath/src/geometry/shape_line_chain.cpp:2671` to `:2693`) is the one geometry routine this milestone genuinely adds:

```
PointAlong(aPathLength):
    if aPathLength == 0: return CPoint(0)
    total = 0
    for i in 0..SegmentCount():
        s = CSegment(i); l = s.Length()
        if total + l >= aPathLength: return s.A + (s.B - s.A).Resize(aPathLength - total)
        total += l
    return CLastPoint()
```

Everything it needs, `Seg::length`, `Vec2::resize`, is already in `src/geometry`.

### 5.5 `multidragWalkaround`

`:458` to `:570`. Walk every line in two orders and keep the cheaper one.

```
delete m_lastNode                                                     :461
sort aCompletedLines by dragDist ascending                            :467 to :472
preWalkNode = m_world->Branch()                                       :475
for l: preWalkNode->Remove(l.originalLine)                            :480

for attempt in 0, 1:                                                  :493
    state.node = preWalkNode->Branch()                                :496
    state.postWalkLines.resize(n)
    for lidx in 0..n:
        l = aCompletedLines[attempt ? n-1-lidx : lidx]                :501
        walk = l.draggedLine
        if tryWalkaround(state.node, l.draggedLine, walk):            :504
            state.node->Add(walk)
            state.totalLength += walk.Length() - l.draggedLine.Length()   :512
            state.postWalkLines[lidx] = walk                          :513
        else:
            state.fail = true; break

bestAttempt = the non failing attempt with the smaller totalLength     :523 to :543
if neither: delete both nodes; return false                            :545 to :550
for lidx: aCompletedLines[lidx].draggedLine = best.postWalkLines[lidx]  :553 to :556
m_lastNode = best.node; delete the other                               :558, :559
restoreLeaderSegments(aCompletedLines)
```

The two attempts exist because a walkaround is order dependent: the first line walked has the free space, the last one has to fit around everything already placed. `totalLength` is the sum of the detours, so the winner is the ordering that bends the set least.

**The index bug.** At `:501` the line taken is `aCompletedLines[n-1-lidx]` for attempt 1, but at `:513` its result is stored at `postWalkLines[lidx]`, and at `:555` `postWalkLines[lidx]` is assigned back to `aCompletedLines[lidx]`. So whenever attempt 1 wins, every line gets some other line's walked geometry. (E17)

`std::sort` at `:472` with a comparator declared as returning `int` (`:467`) but used as a strict weak ordering. It compiles because `int` converts to `bool`; the comparator itself is correct. Sorting by `dragDist` is fine in segment mode and a no operation in corner mode, where `dragDist` is never assigned (section 5.1), so `std::sort` on all zeros is order dependent for equal elements. `std::sort` is not stable, so **the corner mode walkaround order is unspecified**. A deterministic port must tie break, for example on `mdragIndex`.

`tryWalkaround` (`:384` to `:405`) is the same as the single dragger's except for `SetLengthLimit( true, 3.0 )` (`:391`) instead of 30.0.

The commented out ASAN block at `:562` to `:565` is a leftover.

### 5.6 `multidragShove`

`:613` to `:701`:

```
delete m_lastNode; if not m_shove: return false                       :615, :621
sort aCompletedLines by dragDist                                      :629
(debug dump of every line's classification)                           :633 to :644
m_shove->SetDefaultShovePolicy(SHP_SHOVE)                             :647
m_shove->ClearHeads()
for l in aCompletedLines: m_shove->AddHeads(l.draggedLine,
                              SHP_SHOVE | SHP_DONT_OPTIMIZE)          :653
status = m_shove->Run()                                               :656
m_lastNode = m_shove->CurrentNode()->Branch()                         :658

# re-add lines removed from m_preShoveNode in Start that are not in aCompletedLines,
# or they would be silently deleted from the board                    :660 to :676
completedIndices = { cl.mdragIndex for cl in aCompletedLines }
for ml in m_mdragLines:
    if ml.mdragIndex not in completedIndices:
        preserved = ml.originalLine; preserved.ClearLinks(); m_lastNode->Add(preserved)

if status == SH_OK:                                                   :678
    for i, l in enumerate(aCompletedLines):
        if m_shove->HeadsModified(i): l.draggedLine = m_shove->GetModifiedHead(i)   :684, :685
        l.draggedLine.ClearLinks()      # "this should not be linked (assert in rt-test)"  :687, :688
        m_lastNode->Add(l.draggedLine)
else:
    return false                                                      :695
restoreLeaderSegments(aCompletedLines)
```

The sort before `AddHeads` is the one place in the tree where a caller deliberately controls head order (note 04 section 5.4). The re-add block at `:660` to `:676` is a repair for a real data loss bug: `Start` removed every `m_mdragLines` entry from `m_preShoveNode`, but `Drag` only rebuilds the ones that passed the direction check, so without this the rejected lines would vanish. It is also why `mdragIndex` exists.

The `else: return false` at `:693` to `:696` leaves without calling `restoreLeaderSegments`, so `m_leaderSegments` keeps whatever `Drag` put there at `:813`, the original selected items. (E27)

### 5.7 `findNewLeaderSegment`, `restoreLeaderSegments` and the selection handoff

`restoreLeaderSegments` (`:429` to `:456`):

```
m_leaderSegments.clear()
for l in aCompletedLines where l.dragOK:
    if m_dragMode == DM_CORNER:
        if l.draggedLine.LinkCount() > 0:
            m_leaderSegments.push_back(l.draggedLine.GetLink(-1))     :441, :442
    else:
        newLeaderIdx = findNewLeaderSegment(l)                        :447
        if 0 <= newLeaderIdx < l.draggedLine.LinkCount():
            m_leaderSegments.push_back(l.draggedLine.GetLink(newLeaderIdx))   :450, :451
```

`GetLink( -1 )` is the last link (`pcbnew/router/pns_link_holder.h:80` to `:86`, negative indices wrap). In corner mode the leader is by construction the segment at the dragged end, which after `Start`'s reversal is the last one.

`findNewLeaderSegment` (`:407` to `:427`) answers "which segment of the dragged line corresponds to the one the user grabbed":

```
origLeader    = aLine.preDragLine.CSegment(aLine.leaderSegIndex)      :409
origLeaderDir = DIRECTION_45(origLeader)
for i in 0..aLine.draggedLine.SegmentCount():
    curSeg = draggedLine.CSegment(i);  curDir = DIRECTION_45(curSeg)
    ip = curSeg.IntersectLines(m_guide)                               :417
    if ip and curSeg.Contains(*ip)                                    :419
       and (curDir == origLeaderDir or curDir == origLeaderDir.Opposite()):   :421
        return i
return -1
```

`m_guide` is the perpendicular ray through the cursor built at `:809`, so the answer is "the segment the cursor's perpendicular actually crosses, running the same way the grabbed one did". `DIRECTION_45::Opposite` is needed here and exists as `Direction45::opposite` (`src/geometry/direction45.rs:424`).

The whole point of this machinery is `GetLastCommittedLeaderSegments` (`pns_multi_dragger.h:112`), which the host reads after the drag to **restore the selection** (`pcbnew/router/router_tool.cpp:3164`, `:3238` to `:3246`): the router deleted the segments the user had selected and made new ones, so it hands back the new segments and the host selects their parents. The comment at `:110` to `:111` says exactly that.

### 5.8 The small members

- `CurrentNode()` (`:1000`): `m_lastNode ? m_lastNode : m_world`.
- `Traces()` (`:1006`): `m_draggedItems`, which `tryPosture` fills with the **non primary** dragged lines only (`:880`, and never for the primary). So the line under the cursor is not in `Traces()` and therefore is not skipped by `markViolations` (`pcbnew/router/pns_router.cpp:726`).
- `CurrentNets()` (`:351`): iterates a `std::set<NET_HANDLE>`, which is pointer ordered. Determinism hazard, unreachable in practice (section 1.5).
- `CurrentLayer()` (`:1012`): `return 0;` with "fixme: should we care?". (E22)
- `SetMode` (`:284`): **empty body**. `Mode()` (`:289`): `return DM_CORNER;` unconditionally, regardless of `m_dragMode`. (E21)
- `GetForceMarkObstaclesMode` (`pns_multi_dragger.h:114`): writes `m_dragStatus` out and always returns **false**, so the host never shows the "Track violates DRC" hint during a multi drag even though `FixRoute` will refuse. (E23)
- `FixRoute( bool aForceCommit )` (`:366` to `:382`): commits when `m_dragStatus || Settings().AllowDRCViolations()`, **ignoring `aForceCommit` entirely**. Ctrl+click cannot force a multi drag commit. (E24)
- The constructor (`:33` to `:37`) sets only `m_world` and `m_lastNode`, leaving `m_dragStatus` and `m_dragMode` indeterminate. `Start` assigns `m_dragStatus` at `:48`, before the empty set refusal at `:53`, but `m_dragMode` is only assigned in phase 3, so a refused start leaves it indeterminate. Nothing reads it in that state, so this is latent only.
---

## 6. The optimizer's drag only passes

`REQUIRE_OBTUSE_ANGLES = 0x400` (`pcbnew/router/pns_optimizer.h:110`, "Try to prevent 90-degree or acute corners in a drag") is set by exactly two callers: `DRAGGER::optimizeAndUpdateDraggedLine` when `GetRestrictAngles()` (`pcbnew/router/pns_dragger.cpp:583`) and `ROUTER_TOOL::OptimizeSelected` for the same reason (note 04 section 5.6). It selects two things at once.

**`OBTUSE_ONLY_CONSTRAINT`** (`pcbnew/router/pns_optimizer.h:350`, added at `:705`, checked at `pcbnew/router/pns_optimizer.cpp:305` to `:346`). A candidate replacement is rejected unless every corner inside it, and both seams where it joins the rest of the path, are obtuse or straight:

```
Check(aVertex1, aVertex2, aOriginLine, aCurrentPath, aReplacement):
    isAngleOk(s1, s2) = DIRECTION_45(s1).Angle(DIRECTION_45(s2)) & (ANG_OBTUSE | ANG_STRAIGHT)
    if aReplacement.SegmentCount() < 1: return true                   :319
    for i in 0 .. replSegs-2:
        if not isAngleOk(replacement[i], replacement[i+1]): return false     :324
    if aVertex1 > 0 and not isAngleOk(aCurrentPath[aVertex1-1], replacement.first): return false   :335
    if aVertex2 < pathSegs and not isAngleOk(replacement.last, aCurrentPath[aVertex2]): return false  :341
    return true
```

Note the mask is `ANG_OBTUSE | ANG_STRAIGHT`, which excludes `ANG_RIGHT`, so a 90 degree corner is rejected. `DIRECTION_45( seg )` defaults to `a90 = false`, so a right angle really does classify as `ANG_RIGHT` and not as straight.

**`dragFixCorners`** (`pcbnew/router/pns_optimizer.cpp:801` to `:846`), called from `Optimize` at `:709`, **before** `mergeFull`:

```
dragFixCorners(aLine):
    anchor = (m_effortLevel & PRESERVE_VERTEX) ? m_preservedVertex : path.CLastPoint()   :807
    anchorIdx = path.Find(anchor)
    if anchorIdx <= 0: return false                                   :811
    if path.IsArcSegment(anchorIdx-1) or IsArcSegment(anchorIdx): return false   :814
    if anchorIdx >= path.PointCount()-1:                              :819
        return dragFixCorner(aLine, anchorIdx - 1)
    angle = DIRECTION_45(path.CSegment(anchorIdx-1)).Angle(DIRECTION_45(path.CSegment(anchorIdx)))
    if angle == ANG_STRAIGHT:                                         :830
        changed  = dragFixCorner(aLine, anchorIdx - 1)
        path.Split(anchor); anchorIdx = path.Find(anchor)             :834, :835
        if 0 < anchorIdx < path.PointCount()-1:
            changed |= dragFixCorner(aLine, anchorIdx + 1)            :838
    else:
        changed = dragFixCorner(aLine, anchorIdx)                     :842
    return changed
```

```
dragFixCorner(aLine, aVIdx):
    if aVIdx <= 0 or aVIdx >= path.PointCount()-1: return false       :742
    if either adjacent segment is an arc: return false                :745
    angle = DIRECTION_45(s1).Angle(DIRECTION_45(s2))    where s1 = CSegment(aVIdx-1), s2 = CSegment(aVIdx)
    if angle not in {ANG_RIGHT, ANG_ACUTE, ANG_HALF_FULL}: return false    :753
    if (m_effortLevel & RESTRICT_AREA) and not m_restrictArea.Contains(path.CPoint(aVIdx)):
        return false                                                  :756 to :760
    for posture in 0, 1:                                              :765
        bypass = DIRECTION_45().BuildInitialTrace(s1.A, s2.B, posture)
        if bypass.SegmentCount() < 1 or checkColliding(aLine, bypass): continue   :769, :772
        loop = closed polygon [s1.A, CPoint(aVIdx), s2.B] + bypass reversed        :775 to :783
        keep bypass with the smallest |loop.Area()|                   :785
    if no bypass: return false
    path.Replace(s1.Index(), s2.Index(), bestBypass); path.Simplify2()    :795, :796
    return true
```

So the pass is: at the vertex the drag anchored on, if the corner is right, acute or a full reversal, replace the two segments around it with the 45 degree bypass whose enclosed area is smallest and which does not collide. The `ANG_STRAIGHT` branch handles the case where the anchor is in the middle of a straight run, fixing the corner on either side of it.

The area comparison is a `double` from `SHAPE_LINE_CHAIN::Area` and is the only floating point in the pass; it is a tie break over at most two candidates, so it does not endanger determinism.

`src/optimizer.rs` deliberately has neither: line `:82` of its module documentation and `:190` of the `EffortFlags` documentation both say the flag arrives with the dragger and that a host passing `0x400` through `EffortFlags::from_bits` keeps the bit and has it ignored. `Optimizer::optimize` has the placeholder comment at the exact right spot (`src/optimizer.rs:1676`, "`:709`. `REQUIRE_OBTUSE_ANGLES` and `dragFixCorners` arrive with the dragger").

This matters for the corpus: `drag-acute-fallback`, `drag-walk-optimize-a` and `drag-walk-optimize-fix-corners` all set `restrict_angles: true` in their `.settings`, and the first one's name says what it is testing.

---

## 7. The host side: `router_tool.cpp`

### 7.1 Which gesture starts which drag

| Gesture | Entry | Mode passed |
| --- | --- | --- |
| Inside the router tool's own loop, `PCB_ACTIONS::dragFreeAngle` | `performDragging` | `DM_ANY \| DM_FREE_ANGLE` (`:2455`) |
| Inside the router tool's loop, `PCB_ACTIONS::drag45Degree` | `performDragging` | `DM_ANY` (`:2460`) |
| From the selection tool, `PCB_ACTIONS::routerInlineDrag` | `ROUTER_TOOL::InlineDrag` | `aEvent.Parameter<int>()` (`:2997`) |
| `PCB_ACTIONS::breakTrack`, inside the loop | `breakTrack()` | not a drag |
| `PCB_ACTIONS::breakTrack`, from outside | `ROUTER_TOOL::InlineBreakTrack` | not a drag |

Bindings at `:3525` and `:3526`. Both drag gestures inside the loop call `updateStartItem( *evt, true )` first (`:2454`, `:2459`), where the `true` is `aIgnorePads` (`pcbnew/router/pns_tool_base.h:67`): a drag never starts on a pad.

Receiving `routerInlineDrag` while the router tool is already routing is treated as a cancel (`:1994`), so the two never overlap.

The mode parameter of `routerInlineDrag` is set where the action is declared, in `pcbnew/tools/pcb_actions.cpp`, which is not in this checkout. Given section 1.2 it can only matter through `CanInlineDrag`'s footprint test.

### 7.2 `performDragging`

`:2516` to `:2666`. Structurally `performRouting` with three differences (note 05 section 4.10 has the summary; this is the sequence).

```
performDragging(aMode):
    ClearViewDecorations(); view()->ClearPreview(); InitPreview()     :2518 to :2521
    if m_startItem->IsLocked(): modal dialog, OK label "Drag Anyway"  :2525 to :2534
    if not m_router->StartDragging(m_startSnapPoint, m_startItem, aMode):   :2536
        show FailureReason in the info bar; return                    :2540, :2541
    highlightNets(true, { m_startItem->Net() })                       :2546, :2547
    ctls->SetAutoPan(true); m_gridHelper->SetAuxAxes(true, m_startSnapPoint)   :2549, :2550
    frame()->UndoRedoBlock(true)                                      :2551

    loop:
        motion:  updateEndItem(evt); m_router->Move(m_endSnapPoint, m_endItem)   :2565, :2566
                 if dragger->GetForceMarkObstaclesMode(&dragStatus) and not dragStatus:
                     add a ROUTER_STATUS_VIEW_ITEM saying "Track violates DRC."
                     with the hint _("(%s to commit anyway.)") formatted with
                     KeyNameFromKeyCode(MD_CTRL + PSEUDO_WXK_CLICK)   :2572 to :2587
        left click: forceCommit = evt->Modifier(MD_CTRL)              :2594
                    if m_router->FixRoute(m_endSnapPoint, m_endItem, false, forceCommit): break   :2596
        right click: context menu
        cancel / activate: break                                      :2603 to :2613
        undo/redo: wxFAIL                                             :2614 to :2618
        cut/copy/paste/zone fill: wxBell()                            :2624 to :2631
        undo action: treated as escape                                :2633 to :2639

    ClearPreview(); ShowPreview(false)                                :2653, :2654
    if m_router->RoutingInProgress(): m_router->StopRouting()         :2656, :2657
    m_startItem = nullptr; SetAuxAxes(false); UndoRedoBlock(false)    :2659 to :2665
```

The whole drag lives inside one `UndoRedoBlock`, so it produces exactly one undo entry, which is what `doc/work/009-dragging.md` asks of the LibrePCB side.

`ROUTER_STATUS_VIEW_ITEM` is the drag specific piece of host rendering: a floating message at the mouse position. Nothing else in the router tool uses it.

### 7.3 `CanInlineDrag` and `NeighboringSegmentFilter`

`CanInlineDrag( int aDragMode )` (`:2742` to `:2762`) is the gate the selection tool asks before offering a drag:

```
run the selection cursor with NeighboringSegmentFilter                :2744
if selection.Size() == 1:   return front is in GENERAL_COLLECTOR::DraggableItems   :2749
if every item is a FOOTPRINT: return not (aDragMode & DM_FREE_ANGLE)  :2751 to :2754
if every item is a PCB_TRACE_T: return true                           :2756 to :2758
return false
```

So a mixed selection of tracks and vias with more than one item is refused: the multi dragger only ever sees pure track selections.

`NeighboringSegmentFilter` (`:2669` to `:2739`) is what turns "the user clicked where two segments meet" into a single item drag rather than a multi drag. It trims the collection to one reference item when:

- there are no arcs at all (`:2685`, "We eliminate arcs because they are not supported in the inline drag code");
- at least one via or trace, at most one via, at most two traces (`:2689` to `:2698`);
- every track in the collection is on the same net as the reference and is co-terminus with it at the reference point, which is snapped to the reference's own endpoint when the click is on one (`:2713` to `:2733`).

Read the other way: **a corner is delivered as one segment**, and the dragger's own `startDragSegment` then decides corner versus segment from the position. The filter is the reason a two segment corner does not become a `MULTI_DRAGGER` drag.

### 7.4 `InlineDrag`

`:2773` to `:3257`. The long one, because it also handles footprint drags with ratsnest and courtyard previews. The track and via path:

```
InlineDrag(aEvent):
    selection = the selection tool's current selection                :2775
    if empty: re-run the selection cursor with NeighboringSegmentFilter   :2778
    if empty or front is not a BOARD_ITEM: return 0                   :2780
    if front is not TRACE / VIA / ARC / FOOTPRINT: return 0           :2788 to :2794

    # clear the lock BEFORE SyncWorld so no virtual via is generated for it,
    # noting the lock cannot be reliably restored afterwards
    if item->IsLocked(): wasLocked = true; item->SetLocked(false)     :2816 to :2827

    selectionClear; Activate()                                        :2829 to :2832
    m_router->SyncWorld()      # the world may be stale after a Move  :2850 to :2852
    SyncLayerVisibilityCache()                                        :2855

    for each selected item: pnsItem = world->FindItemByParent(it)     :2913
        if pnsItem is SEGMENT_T / VIA_T / ARC_T: itemsToDrag.Add(pnsItem)   :2918 to :2923

    # snap: nearest of the items to drag, then snapToItem on it
    closestItem = argmin over itemsToDrag of shape->SquaredDistance(p0, 0)   :2944 to :2958
    p = snapToItem(closestItem, p0);  m_startItem = closestItem       :2962, :2964
    highlightNets(true, { closestItem->Net() })                       :2966, :2967

    dragMode = aEvent.Parameter<int>()                                :2997
    if not m_router->StartDragging(p, itemsToDrag, dragMode):         :2999
        restore the lock, restore the selection, return 0             :3001 to :3016

    SetAuxAxes(true, p); ShowCursor; SetAutoPan; UndoRedoBlock(true)  :3018 to :3021
    m_router->SetVisibleViewArea(BOX2ISafe(visible world extents))    :3036, :3037
    m_router->Move(p, nullptr)         # prime the collision detection :3040

    loop:
        cancel/activate:  restore the lock; hasMultidragCancelled = true; break   :3055 to :3065
        motion or drag:   hasMouseMoved = true; updateEndItem; Move(...)          :3066 to :3070
                          ClearPreview(); the DRC hint, as in performDragging     :3137 to :3155
        mouse up or click, only once the mouse has moved:             :3157
                          forceCommit = evt->Modifier(MD_CTRL)        :3160
                          updateEndItem; FixRoute(..., false, forceCommit)   :3162, :3163
                          leaderSegments = m_router->GetLastCommittedLeaderSegments()   :3164
                          break
        undo/redo: wxFAIL;  undo action: restore the lock and break   :3168 to :3193

    if m_router->RoutingInProgress(): StopRouting()                   :3230, :3231
    if cancelled:            restoreSelection(selection)              :3234 to :3237
    elif leaderSegments:     select the Parent() of every leader segment   :3238 to :3246
    SetAuxAxes(false); UndoRedoBlock(false); highlightNets(false)     :3248 to :3254
```

Five things a port has to copy.

1. **`SyncWorld()` unconditionally at the start** (`:2852`). The world may be stale after an unrelated edit, and `FindItemByParent` and every joint lookup depend on it.
2. **The lock is cleared before the sync** (`:2816` to `:2827`), so `FixupVirtualVias` does not plant a virtual via at the now unlocked segment's ends. That is the only reason the ordering matters, and the comment says the lock cannot be restored because the drag may not end with the same number of segments.
3. **`hasMouseMoved` gates the commit** (`:3157`). A press and release without motion is not a drag and commits nothing.
4. **`SetVisibleViewArea`** (`:3037`) before the first move, which is what clips the shove's post shove optimization (note 04 section 5.1). A headless port passes `None`, which this crate already models (`Shove::set_visible_view_area`).
5. **The priming `Move( p, nullptr )`** at `:3040`, before any user motion, so `m_lastNode` exists and `Traces()` is populated the instant the drag starts.

The selection handoff at `:3238` to `:3246` is the multi drag's payoff: without it a multi drag would end with nothing selected, because the router deleted the selected segments.

### 7.5 `InlineBreakTrack` and `breakTrack`

`InlineBreakTrack` (`:3260` to `:3323`) is not a drag, but it shares the entry style and it is the other way a user cuts a track under the router tool.

```
InlineBreakTrack(aEvent):
    if selection.Size() != 1: return 0                                :3264
    if item is not PCB_TRACE_T and not PCB_ARC_T: return 0            :3269
    selectionClear; Activate(); SyncLayerVisibilityCache()            :3272 to :3277
    m_startItem = m_router->GetWorld()->FindItemByParent(item)        :3279
    m_startSnapPoint = snapToItem(m_startItem,
                          context menu ? GetMenuCursorPos() : GetCursorPosition())   :3289 to :3301
    if m_startItem->IsLocked(): dialog with OK label "Break Track"    :3303 to :3312
    UndoRedoBlock(true); breakTrack(); if RoutingInProgress(): StopRouting()   :3314 to :3318
    UndoRedoBlock(false)
```

`breakTrack()` (`:2113` to `:2131`) refuses to split a via stack's connecting trace and otherwise calls `m_router->BreakSegmentOrArc( m_startItem, m_startSnapPoint )`.

`ROUTER::BreakSegmentOrArc` (`pcbnew/router/pns_router.cpp:1106` to `:1127`) branches the world, constructs a **stack** `LINE_PLACER` purely to reach `SplitAdjacentSegments` / `SplitAdjacentArcs`, and commits or deletes the branch. Note 03 section 1.5 already records it. It is worth listing here because the split is the same primitive a drag needs when the user grabs the middle of a track that is then committed as two, and because this crate's placer already has the splitter.

### 7.6 What the host commits

Nothing drag specific. `FixRoute` reaches `DRAGGER::FixRoute` or `MULTI_DRAGGER::FixRoute`, which call `ROUTER::CommitRouting( node )`, which produces the removed / added / changed triple and calls `m_iface->Commit()` (note 03 section 1.3). The host's contribution is:

- one `UndoRedoBlock` around the whole drag, so it is one undo entry;
- the Ctrl modifier turned into `aForceCommit`;
- the selection restored from `GetLastCommittedLeaderSegments`, for multi drag only;
- the lock flag not restored, deliberately.

`SetCommitFlags( APPEND_UNDO )` is never used for a drag (note 05 section 4.9 lists its three users, all routing), so a drag is always its own undo group.
---

## 8. Errata

Things in KiCad's drag code that are dead, unreachable, or do not do what they look like they do. Each is a decision the port has to make once rather than inherit by accident. The single dragger's list is short and mostly cosmetic; the multi dragger's is not.

### 8.1 Dead code

**E1. `DRAG_ALGO::Mode()` has no caller anywhere in the tree.** Declared pure virtual at `pcbnew/router/pns_drag_algo.h:121` and implemented by both draggers (`pns_dragger.cpp:366`, `pns_multi_dragger.cpp:289`). A grep for a `Mode()` call on a dragger, in `pcbnew/` and `qa/` both, returns nothing; every `Mode()` hit is `ROUTER::Mode()`, which is `ROUTER_MODE`. Do not put it on the trait.

Same paragraph, same evidence: `DRAGGER::GetLastDragSolution()` (`pns_dragger.h:108`) has no caller; `DRAGGER::m_origViaConnections` (`pns_dragger.h:166`) is never read or written anywhere; `MDRAG_LINE::leaderItem`, `preShoveLine`, `clipDone` and `offset` (`pns_multi_dragger.h:129`, `:143`, `:147`, `:148`) are never read or written; `MDRAG_LINE::isDraggable` is set true at construction (`:81`) and never set false, so the guard at `:820` always passes.

**E2. The `DRAG_MODE` request mask is write only except for `DM_FREE_ANGLE`.** `DRAGGER::Start` reads one bit (`pns_dragger.cpp:314`) and then `startDragSegment` / `startDragVia` / `startDragArc` overwrite `m_mode` outright (`:130`, `:144`, `:148`, `:251`, `:262`). `MULTI_DRAGGER::SetMode` has an empty body (`pns_multi_dragger.cpp:284`). `COMPONENT_DRAGGER` does not override `SetMode` at all, so it takes `DRAG_ALGO`'s empty default (`pns_drag_algo.h:119`). The QA log player passes `0` (`qa/tools/pns/pns_log_player.cpp:169`) and the corpus replays correctly, which is the proof.

**E3. `SHOVE::SetDefaultShovePolicy` has no observable effect in this revision.** `m_defaultPolicy` is read at exactly two sites, `pcbnew/router/pns_shove.cpp:1663` and `:1671`, both `& SHP_IGNORE`. Nothing in the tree ever sets `SHP_IGNORE`, and the constructor already stores `SHP_SHOVE` (`:209`). Both dragger calls (`pns_dragger.cpp:328`, `pns_multi_dragger.cpp:277`, `:647`) are therefore no operations. `src/shove.rs` correctly has no equivalent.

**E4. `m_lastDragSolution` is only written by the shove path, but `Drag`'s restore branch always uses it.** Assigned at `pns_dragger.cpp:123` (the original line) and then only at `:858` and `:901`, both inside `dragShove`'s segment/corner and arc branches. The restore at `:1044` re-adds it after any failed non first drag. So in walkaround mode a failed drag snaps the trace back to its original shape rather than to the last good one, and in `DM_VIA` shove mode the restore re-adds a line and leaves the via wherever `m_draggedVia` last put it. Reproduce it if the goldens need it, but record the decision.

### 8.2 Wrong or surprising in the single dragger

**E5. `optimizeAndUpdateDraggedLine` clears `m_draggedItems`, so a via drag under-reports `Traces()`.** `pns_dragger.cpp:617`. `dragViaWalkaround` adds the dragged via at `:511` and each non colliding fanout line at `:557`, then the first line that needs a walkaround reaches `optimizeAndUpdateDraggedLine` at `:553`, which wipes the set and leaves only that line in it. Consequences: `ROUTER::markViolations` no longer skips the dragged via (`pcbnew/router/pns_router.cpp:726`), so the via is drawn as colliding with itself, and the host preview loses items. Fix in the port: have the optimizer helper **add** rather than replace, and clear at the top of `dragViaWalkaround` instead.

**E6. `dragViaWalkaround` optimizes around the cursor, not the via's real position.** The line is dragged to `viaTargetPos`, the post force position (`:541`), but `optimizeAndUpdateDraggedLine` is handed `aP` (`:553`), which is not on the line whenever the force propagation moved the via. `bestAnchorForPoint` then silently substitutes the nearest vertex (`:590`, `:591`), so the preserved vertex is not the one the drag actually anchored on.

**E7. `aForceCommit` is honoured only in forced mark obstacles mode.** `DRAGGER::FixRoute` (`:957` to `:995`) tests it inside the `else if( m_forceMarkObstaclesMode )` branch only (`:968` to `:977`). In plain `RM_MarkObstacles` with a colliding drag and `AllowDRCViolations()` false, `m_dragStatus` is false, `m_forceMarkObstaclesMode` is false, and Ctrl+click falls into the re-drag branch at `:983` instead of committing. `MULTI_DRAGGER::FixRoute` ignores the argument entirely (E24). The host meanwhile advertises "Ctrl+click to commit anyway" from `GetForceMarkObstaclesMode` (`pcbnew/router/router_tool.cpp:2572`), which is consistent with the single dragger and not with the multi one.

**E8. `m_lastValidPoint` is a `VECTOR2D`** (`pns_dragger.h:167`) although all three writes are `VECTOR2I` (`:316`, `:1022`, `:1034`) and its only read passes it to `Drag( const VECTOR2I& )` (`:983`). Round tripping an `int32_t` through a `double` is exact, so nothing is lost; it is the wrong type. Port it as an integer point.

**E9. Two different endpoint thresholds.** `checkVirtualVia` tests `dist <= w2` (`:90`, `:94`); `startDragSegment` tests `dist < w2` (`:128`). A click at exactly `w/2` from an endpoint is a via drag if a virtual via sits there and a segment drag if not, and can never be a corner drag. `TOOL_BASE::snapToItem` uses a third spelling, `distA_sq < w_sq` on squared values (`pcbnew/router/pns_tool_base.cpp:496`), which agrees with `startDragSegment`.

**E10. `propagateViaForces` takes a `std::set<VIA*>&` and reads only `*begin()`** (`pns_dragger.cpp:62`, `:64`). The one call site builds a one element set (`:513` to `:515`). Beyond being dead generality, iterating a set of pointers is address ordered, which `DESIGN.md` section 8 forbids here. Port it as a single via.

**E11. `dragViaWalkaround` leaves the via removed when the force propagation fails.** `m_lastNode->Remove( via )` at `:517`, `Add` only inside `if( ok )` at `:525`, and `return false` at `:530` with the node in that state. `Drag`'s restore branch re-branches from the parent (`:1039`) and discards it, so no user sees it, but a port that keeps the node would.

**E12. `startDragArc`'s stub segments go into `m_preDragNode`, while the collision probes test against `m_world`.** Stubs added at `:231` and `:241`; probes at `:545`, `:739` and `:770` all use `m_world`, the router's root. So the stubs are visible to the shove (whose root is `m_preDragNode`) and to `m_lastNode`, but invisible to the test that decides whether an arc drag needs a walkaround. Arc only, so this crate inherits nothing, but it is the same class of mistake a port could make with any node the drag adds to.

**E13. `dragViaMarkObstacles( aHandle, aNode, aP )` uses `aNode` only for the fanout lookup** (`:456`) and mutates `m_lastNode` directly (`:473`, `:474`, `:483`, `:484`). Both call sites pass `m_lastNode` (`:438`, `:945`), so the parameter is redundant. `dragViaWalkaround` has the same shape (`:496` versus `:517`, `:525`, `:552`).

### 8.3 Wrong or surprising on the router side of a drag

**E14. `ROUTER::moveDragging( aP, aEndItem )` never uses `aEndItem`** (`pcbnew/router/pns_router.cpp:656` to `:667`). The end item matters for routing, where it decides snapping and finishing; a drag has no head to attach.

**E15. `ROUTER::GetUpdatedItems` has no `DRAG_COMPONENT` branch** (`:839` handles `ROUTE_TRACK`, `:844` handles `DRAG_SEGMENT`, nothing handles `DRAG_COMPONENT`), so a component drag reports an empty delta to anything that asks, the QA player included. Component drag is out of scope here; the shape of the bug is worth knowing because the same `switch` is the natural place for a port to add a drag branch.

**E16. `ROUTER::StopRouting` never updates the ratsnest after a drag.** The nets pushed to the host come from `m_placer->GetModifiedNets` inside `if( m_placer )` (`:971` to `:979`), and `m_placer` is null in both drag states. A host that relies on `StopRouting` for ratsnest maintenance will see a stale ratsnest after every drag.

### 8.4 The multi dragger

**E17. `multidragWalkaround` stores attempt 1's results under the wrong indices.** The line is taken as `aCompletedLines[attempt ? n-1-lidx : lidx]` (`pcbnew/router/pns_multi_dragger.cpp:501`) and its walked shape is stored at `state->postWalkLines[lidx]` (`:513`), then written back as `aCompletedLines[lidx].draggedLine = postWalkLines[lidx]` (`:555`). For `attempt == 1` the two indexings disagree, so every line receives a different line's geometry whenever the reverse ordering wins. The fix is one index.

**E18. `Start`'s corner mode check reverses lines and then may abandon corner mode.** `l.originalLine.Reverse()` at `:234` runs before the joint test at `:239`; a missing joint sets `m_dragMode = DM_SEGMENT` and `break`s (`:243`, `:244`), leaving the lines processed so far reversed and the rest not, while `leaderSegIndex` still refers to the pre reversal link order. The `!jt->IsTrivialEndpoint()` case at `:247` to `:250` sets the mode without breaking, so it keeps reversing. Either reverse after the decision, or re-derive the indices.

**E19. `lastPreDrag` is read before it is written in the segment branch of `tryPosture`.** Declared at `:710`, used to build a debug shape at `:803`, assigned at `:805`. `SEG`'s default constructor zeroes `A` and `B`, so this is a garbage debug shape rather than undefined behaviour, but it is a read before write.

**E20. `MULTI_DRAGGER::Drag` ignores whether any posture succeeded.** `res` is assigned at `:970` and never read after the loop (`:966` to `:974`); the mode dispatch at `:976` runs on whatever `completed` the last attempted variant left. So a drag that failed all three postures still goes through the shove or the walkaround.

**E21. `Mode()` always answers `DM_CORNER`** (`:289` to `:292`) regardless of `m_dragMode`, and `SetMode` is an empty body (`:284`). No caller (E1), so it is latent.

**E22. `CurrentLayer()` returns `0`** with a "fixme: should we care?" comment (`:1012` to `:1016`). Unreachable in practice, since `ROUTER::GetCurrentLayer`'s only callers are in `pns_dp_meander_placer.cpp`.

**E23. `GetForceMarkObstaclesMode` always returns false** (`pns_multi_dragger.h:114` to `:118`), so the host never shows the DRC warning during a multi drag, even though `FixRoute` will refuse to commit for exactly that reason.

**E24. `MULTI_DRAGGER::FixRoute` ignores `aForceCommit`** (`:366` to `:382`). Ctrl+click cannot force a multi drag commit; `AllowDRCViolations()` is the only override.

**E25. `clipToOtherLine` narrows the chain length and can produce an empty line.** `int curL = l.CLine().Length();` at `:307` narrows a `long long int` (`libs/kimath/include/geometry/shape_line_chain.h:500`), overflowing above roughly 2.147 metres of chain. Separately, if the first probe collides and every subsequent one does too, `tightest` is never assigned and `aClipped.SetShape( tightest )` at `:343` installs an empty chain while `didClip` is true, so `multidragMarkObstacles` stores an empty line at `:598`.

**E26. The primary line can disagree with the drag mode.** `anyStrictCornersFound` alone forces `DM_CORNER` (`:176`), but the primary is the first `isStrict` line in `m_mdragLines` order (`:254` to `:261`), which may be a strict **mid segment** line. `tryPosture`'s corner branch then reads `primaryLine` only for the segment branch, so the corner path uses `primaryDragged` from a line that was classified as a mid segment grab.

**E27. `multidragShove` returns without restoring the leader segments on failure** (`:693` to `:696`), leaving `m_leaderSegments` as the raw `m_origDraggedItems` that `Drag` assigned at `:813`. Those are the pre drag items; the host would then re-select items the commit may have replaced.

**E28. Corner mode multi drag has an unspecified line order.** `dragDist` is only ever assigned in the segment branch (`:907`), so in corner mode every `MDRAG_LINE` compares equal under `compareDragStartDist`, and `std::sort` (`:472`, `:629`) is not stable. The walkaround attempt order and the shove head order are therefore unspecified in corner mode. A deterministic port must tie break, `mdragIndex` being the obvious key.

### 8.5 Not an erratum, but do not "fix" it

- `preShoveNode->Remove( draggedPreShove )` after `draggedPreShove` was already re-shaped (`pns_dragger.cpp:830`). The links, not the geometry, are what the removal uses. `MULTI_DRAGGER` spells the same trick out in a comment at `:731` to `:733`.
- The snap threshold being `width / 4` in two paths and `width / 2` in the third (`:398`, `:727`, `:818`). Both carry the same `//TODO: Make threshold configurable` comment, so it looks like a slip, but a golden can see the difference.
- `dragWalkaround` colliding against `m_world` rather than `m_lastNode` (`:739`). Equivalent for straight segments today.
- `dragMarkObstacles`'s unconditional `return true` (`:448`). It is what makes the forced mark obstacles fallback terminal.
---

## 9. What the port needs

Read off this crate at the same commit as this note. "Present" means it exists and is public; "addition" means a new item in an existing module; "new module" means a new file under `src/`.

### 9.1 Already present, nothing to do

| Helper | Where | Note |
| --- | --- | --- |
| `Line::drag_corner( at, index )` | `src/line.rs:1019` | KiCad's `dragCorner45` with `aFreeAngle = false` and no preferred direction. |
| `Line::snap_dragged_corner` | `src/line.rs:1070` | private, used by `drag_corner`. |
| `drag_corner_internal` | `src/line.rs:1675` | private free function. |
| `Line::snap_threshold` / `set_snap_threshold` | `src/line.rs:1425`, `:1431` | already propagated through every copy path, which the module documentation says was done for this milestone. |
| `Line::changed_area` | `src/line.rs:1258` | the optimizer's restrict area. |
| `Line::clear_links`, `links`, `link_at`, `contains_link`, `link_count`, `reverse` | `src/line.rs:679`, `:574`, `:636`, `:625`, `:580`, `:997` | `link_at` takes an `isize` and wraps negatives, matching `GetLink(-1)`. |
| `Line::mark` / `unmark` / `is_locked` | `src/line.rs:725`, `:741`, `:756` | for `startItem->Unmark( MK_LOCKED )`. |
| `MarkerFlags::LOCKED` | `src/item.rs:268` | |
| `World::assemble_line( node, segment, origin_segment_index, ... )` | `src/node.rs:2523` | the out parameter is already there. |
| `World::find_joint`, `joint`, `find_via_by_handle` | `src/node.rs:2219`, `:2243`, `:2262` | |
| `Joint::is_trivial_endpoint` | `src/joint.rs:458` | needed by `MULTI_DRAGGER::Start` only. |
| `World::branch`, `kill_children`, `drop_node`, `commit`, `get_updated_items` | `src/node.rs:526`, `:578`, `:606`, `:653`, `:2882` | |
| `World::add_line`, `remove_line`, `replace_line` | `src/node.rs:2353`, `:2437`, `:2461` | |
| `World::check_colliding_line`, `query_colliding_line`, `collide_lines` | `src/node.rs:1768`, `:1735`, `:1853` | `collide_lines` is the line against line primitive `clipToOtherLine` needs. |
| `Walkaround` with `set_allowed_policies`, `set_length_limit`, `route` | `src/walkaround.rs:344`, `:333`, `:436` | `tryWalkaround` is four setters and a call. |
| `Optimizer` with `set_effort_level`, `set_preserve_vertex`, `set_restrict_area`, `optimize` | `src/optimizer.rs:1457`, `:1535`, `:1518`, `:1629` | `set_preserve_vertex` already sets the flag as a side effect, as KiCad does. |
| `EffortFlags::MERGE_SEGMENTS`, `MERGE_COLINEAR`, `RESTRICT_AREA`, `PRESERVE_VERTEX` | `src/optimizer.rs:209`, `:262`, `:271`, `:249` | |
| `Shove`, whole head protocol | table in section 4 | nothing new. |
| `Seg::intersect_lines`, `line_distance`, `line_distance_signed`, `line_project`, `contains_point`, `side`, `length` | `src/geometry/seg.rs:907`, `:618`, `:631`, `:568`, `:814`, `:340`, `:282` | `line_distance_signed` is KiCad's `LineDistance( p, true )`. |
| `Vec2::perpendicular`, `resize`, `dot`, `euclidean_norm` | `src/geometry/vec2.rs:147`, `:167`, `:223`, `:117` | |
| `Direction45::angle`, `is_obtuse`, `left`, `right`, `opposite`, `to_vector`, `is_diagonal`, `build_initial_trace` | `src/geometry/direction45.rs:352`, `:370`, `:401`, `:385`, `:424`, `:449`, `:341`, `:512` | |
| `LineChain::insert`, `set_point`, `replace_with_chain`, `slice`, `split`, `simplify`, `simplify2`, `find`, `nearest_point`, `area`, `set_closed`, `length` | `src/geometry/line_chain.rs:609`, `:913`, `:709`, `:776`, `:831`, `:1067`, `:1165`, `:1036`, `:1848`, `:2146`, `:404`, `:976` | `length` already returns `i64`, so erratum E25's narrowing cannot be inherited. |
| `RoutingSettings::smooth_dragged_segments`, `allow_drc_violations`, `optimize_entire_dragged_track`, `restrict_angles`, `via_force_prop_iteration_limit`, `walkaround_iteration_limit` | `src/settings.rs:188`, `:194`, `:207`, `:228`, `:278`, and `src/walkaround.rs:264` | every knob the dragger reads. |
| `RouterState::DragSegment` | `src/router.rs:109` | already declared and documented as never entered. |
| `SessionEvent::MoveTo`, `FixRoute`, `StopRouting` | `src/eventlog.rs:158`, `:167`, `:240` | reusable for a drag. |

### 9.2 Additions to existing modules

| What | Where it goes | KiCad source | Notes |
| --- | --- | --- | --- |
| `Line::drag_corner_with( at, index, free_angle, preferred_direction )` | `src/line.rs` | `pns_line.cpp:884`, `:857`, `:823` | Widen `drag_corner` rather than add a second entry point. `free_angle` is `dragCornerFree`, three lines without arcs. `preferred_direction` is needed by the multi dragger only (`pns_multi_dragger.cpp:863`) and threads into `drag_corner_internal`'s first pick (`pns_line.cpp:768` to `:779`). |
| `Line::drag_segment( at, index )` | `src/line.rs` | `pns_line.cpp:898`, `:1230` to `:1397` | The single largest new routine of the milestone, about 120 lines. The free angle overload is `assert( false )` in KiCad (`:902`), so the Rust signature takes no flag. |
| `Line::snap_to_neighbour_segments` | `src/line.rs`, private | `pns_line.cpp:1185` to `:1228` | Only `drag_segment` calls it. Keep KiCad's `== 0` guard or record the deviation. |
| `Line::drag_arc` | `src/line.rs` | `pns_line.cpp:911` to `:1141` | **Not this milestone.** Needs `SHAPE_ARC`, `CIRCLE::ConstructFromTanTanPt` and `CalcArcMid`, none of which exist here. |
| `LineChain::point_along( path_length ) -> Vec2` | `src/geometry/line_chain.rs` | `shape_line_chain.cpp:2671` to `:2693` | Ten lines on top of `Seg::length` and `Vec2::resize`. `TODO.md` already lists it as deferred to this milestone. Only `clipToOtherLine` uses it, so it lands with multi drag. |
| `MouseTrailTracer::trail_lead_vector() -> Vec2` | `src/mouse_trail.rs` | `pns_mouse_trail_tracer.cpp:279` to `:289` | Last trail point minus first, `(0,0)` below two points. Four lines. Needed only by the via drag. |
| `via_pushout_force` made reachable | move out of `src/placer/line_placer.rs:2121` | `pns_via.cpp:143` to `:232` | It is private to the placer module today. Either lift it to `src/item.rs` next to `Via::pushout_force`, or to a small shared home, together with its private `single_step_force`, `move_via_by` and `move_via_to`. No behaviour change; this is the one refactor the milestone forces. |
| `EffortFlags::REQUIRE_OBTUSE_ANGLES` plus `Constraint::ObtuseOnly` and an `Optimizer::drag_fix_corners` pass | `src/optimizer.rs` | `pns_optimizer.h:110`, `:350`; `pns_optimizer.cpp:305`, `:738`, `:801` | The placeholder comment is already at the right line (`src/optimizer.rs:1676`). Three of the seven corpus cases set `restrict_angles`, so this is not optional. |
| `World::check_colliding_items` accepting lines | `src/node.rs:1807` | `pns_node.cpp:478`, called from `pns_dragger.cpp:446` | Its documentation already names this call site. Either widen the parameter to an enum of item-or-line, or have the dragger loop over `check_colliding_line` itself. The latter is smaller and keeps the node API narrow. |
| `RouterState::DragSegment` actually entered | `src/router.rs:109` | `pns_router.cpp:185`, `:190` | The doc comment saying it is never entered has to change with it. |
| `Router::pending_update` drag branch | `src/router.rs:1452` | `pns_router.cpp:844` to `:848` | It short circuits on `self.placer` today. This is what the corpus goldens compare against, so it is on the critical path, not a nicety. |

### 9.3 New modules

**`src/dragger.rs`.** `DRAGGER` in full: the mode decision, the three drag routines, the via fanout, `optimize_and_update_dragged_line`, `best_anchor_for_point`, `point_has_bad_corner`, `try_walkaround`, `propagate_via_forces`, `fix_route`. Roughly the size of `src/walkaround.rs`.

**`src/multi_dragger.rs`.** `MULTI_DRAGGER`, `MdragLine`, `clip_to_other_line`, the three per mode routines and the leader restoration. Last, and only if multi drag stays in scope: `PLAN.md`'s non-goals list still names it, while `doc/work/009-dragging.md` has it as a task. That conflict is worth resolving before any of it is written.

There is deliberately **no** `src/drag_algo.rs`. `DRAG_ALGO` is a C++ virtual base with two live implementations, one of which is out of scope; a trait for it would exist to be implemented once. `DESIGN.md` section 11 and note 03 section 9.2 both prefer an enum dispatch, which is what `Router` should do: a `Dragger` and, later, a `MultiDragger` variant behind one small enum in `src/router.rs`, not a `Box<dyn DragAlgo>`.

### 9.4 Session facade additions

`Router` (`src/router.rs:514`) needs a `dragger: Option<Dragger>` field beside `placer`, and:

```rust
/// Port of `ROUTER::StartDragging` (`pcbnew/router/pns_router.cpp:166`).
pub fn start_dragging(
    &mut self,
    at: Vec2,
    items: &[HostId],
    free_angle: bool,
) -> Result<(), StartError>;
```

Taking a slice of host ids rather than one, because `StartDragging`'s set overload is the real one and the multi dragger needs it; taking `free_angle: bool` rather than a mode mask, because that is the only input bit (E2). `StartError` gains at least `NotDraggable(ItemId)` for the `default:` refusal at `pns_dragger.cpp:355`, and `startDragArc`'s arc angle refusal has no Rust counterpart while arcs are out.

`move_to` (`src/router.rs:1062`) needs its `if self.state != RouterState::RouteTrack { return default }` guard replaced by a dispatch, mirroring `ROUTER::Move`'s switch (`pns_router.cpp:504` to `:506`). The `end: Option<HostId>` argument is unused on the drag branch (E14) but stays in the signature, because one facade method has to serve both states and the event log records it either way.

`fix_route` (`src/router.rs:1095`) needs a drag branch. KiCad's four argument `FixRoute( aP, aEndItem, aForceFinish, aForceCommit )` drops `aP` and `aEndItem` on the drag branch and `aForceCommit` on the placer branch (`pns_router.cpp:922` to `:935`), so the crate's `fix_route( at, end, force_finish )` gains a fourth argument, `force_commit: bool`, ignored when routing. Its documentation already says `aForceCommit` is dragger only, so the comment is written.

New events on `SessionEvent` (`src/eventlog.rs:144`), to keep a drag replayable:

```rust
/// `EVT_START_DRAG` / `EVT_START_MULTIDRAG`, `pcbnew/router/pns_router.cpp:204`, `:206`.
StartDragging { at: Vec2, items: Vec<HostId>, free_angle: bool },
```

One variant rather than two: KiCad splits on the item count at the logging site only, and the reader can split the same way. `free_angle` is not in KiCad's log at all, which is why a free angle session cannot be replayed there; recording it costs one bool.

The preview side needs nothing new. `PreviewFrame` (`src/router.rs:300`) is built from the node delta plus violation markers, which is exactly `updateView( m_dragger->CurrentNode(), dragged, true )`. The one gap is `PreviewStyle::Head`: `moveDragging` sets no head flag at all (section 1.4), so either the dragged line comes back as an ordinary added item, matching KiCad, or the crate deviates and marks it. Decide it once and write it down.

The commit side needs nothing new either: `stop_routing` (`src/router.rs:1472`) already produces the `CommitDiff` and `build_commit_plan` already does the removed plus added fold, which is what preserves the host object's identity across a drag that re-splits a track.

### 9.5 Two decisions that have to be taken before the via drag is finished

**Virtual vias.** `NODE::FixupVirtualVias` is not ported (`src/node.rs:86`, `TODO.md`), and `WorldSnapshot` deliberately has no `is_virtual` input (`src/snapshot.rs:317`). Without it, `DRAGGER::checkVirtualVia` has nothing to find, so a click near a width change or a locked segment end becomes a corner drag where KiCad gives a via drag. That is a visible behaviour difference, not a missing optimization. Note 02 section 3.15's two errata (the dead `n_seg >= 3` branch, the `locked_seg` that leaks across joints) are the ones to decide on if it is ported. The via drag itself works without it; only the *entry* to a via drag from a segment click depends on it.

**`Line::drag_corner`'s reach.** KiCad's `LINE::DragCorner` is called from the shove (`pns_shove.cpp:1101`), from both draggers, and from `dragViaMarkObstacles` / `dragViaWalkaround`. Widening the existing `drag_corner` rather than adding a parallel entry point keeps the shove's call site honest.

### 9.6 Things `TODO.md` says the dragger needs that it does not

- **`EDA_ANGLE`.** Used in `pns_dragger.cpp` at `:157`, `:159`, `:161` and `:163` only, all four inside `startDragArc`, all four comparing an arc's central angle against `ANGLE_180`. Nothing outside the arc path touches it. `TODO.md`'s entry can be narrowed to "solid orientation and arcs".
- **`RotatePoint`.** A grep of `pns_dragger.cpp`, `pns_multi_dragger.cpp` and the drag half of `pns_line.cpp` finds no call. The dragger does not need it.
- **Line versus line collisions.** Needed at exactly one dragger call site, `pns_multi_dragger.cpp:321` inside `clipToOtherLine`, which is multi drag and therefore last. `pns_dragger.cpp:446` is `CheckColliding( ITEM_SET )` over lines, which is the *set* overload and is satisfied by looping `check_colliding_line`. So this blocks nothing before multi drag.
- **`PointAlong`.** Likewise `clipToOtherLine` only. Last.
---

## 10. The corpus, and a proposed order of implementation

### 10.1 What the seven drag cases actually exercise

Every field below is read from the fixture: `mode` from the case's `.settings`, the start item from the board the case maps to (`tests/kicad_replay.rs:108` to `:143`), and the resulting `DRAG_MODE` by running `startDragSegment`'s test (`pcbnew/router/pns_dragger.cpp:128`) on the logged start point by hand.

| Case | `mode` | Events | Start item | `w/2` | dist to A, B | Mode taken | Other settings | Golden |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `drag-acute-fallback` | 2, `RM_Walkaround` | 1 drag + 857 move | segment `(137.5, 110)` to `(146.5, 101)`, `F.Cu`, net A, w 0.5 mm | 0.25 mm | 6.36, 6.36 mm | **`DM_SEGMENT`** | `restrict_angles` | 11 added, 6 removed |
| `drag-walk-optimize-a` | 2, `RM_Walkaround` | 1 + 49 | the same segment, same board | 0.25 mm | 6.72, 6.01 mm | **`DM_SEGMENT`** | `restrict_angles` | 9 added, 6 removed |
| `drag-walk-optimize-fix-corners` | 2, `RM_Walkaround` | 1 + 82 | segment `(144.45, 103.05)` to `(164.75, 103.05)`, `F.Cu`, net A, w 0.5 mm | 0.25 mm | 10.65, 9.65 mm | **`DM_SEGMENT`** | `restrict_angles` | 9 added, 3 removed |
| `walk-with-teardrops` | 2, `RM_Walkaround` | 1 + 17 | segment `(59.75, 60)` to `(59.75, 55)`, `B.Cu`, net `/Progger/SWCLK`, w 0.2 mm | 0.1 mm | 2.82, 2.18 mm | **`DM_SEGMENT`** | none | 17 added, 19 removed |
| `simple-drag-shove-singlelayer` | 1, `RM_Shove` | 1 + 34 | segment `(136.017, 81.28)` to `(150.241, 81.28)`, `B.Cu`, net `/{slash}PCWR`, w 0.2 mm | 0.1 mm | 6.58, 7.64 mm | **`DM_SEGMENT`** | none | 134 added, 139 removed |
| `walk_drag_seg_against_board_edge` | 1, `RM_Shove` | 1 + 17 | segment `(130.64, 51.2)` to `(203.811, 51.2)`, `B.Cu`, net `-HV`, w 1.0 mm | 0.5 mm | 18.96, 54.21 mm | **`DM_SEGMENT`** | none | 3 added, 3 removed |
| `issue23449-shove-lone-via-drag-crash` | 1, `RM_Shove` | 1 + 102 | **via** at `(143.3, 80.4)`, 0.5 mm, drill 0.3 mm, `F.Cu` to `B.Cu`, `free yes`, net GND | n/a | n/a | **`DM_VIA`** | none | **empty** |

All seven set `smooth_dragged_segments: true` and `optimize_dragged_track: false`, so the snap threshold is always non zero and the post drag optimizer is always restricted to the changed area.

Six conclusions, each of which changes what has to be built and in what order.

1. **Six of seven are mid segment drags.** Not one corpus case is a corner drag. The nearest endpoint in the whole set is 2.18 mm from the click on a 0.2 mm wide track, more than twenty times the `w/2` threshold.
2. **One is a via drag, and its golden is empty.** It is a pure crash regression (note 05 section 6.9 says the same).
3. **No case uses `RM_MarkObstacles`,** so the simplest of the three drag routines has no golden behind it. It still has to exist, because it is the forced fallback of `Drag` (`pns_dragger.cpp:1029`) and free angle mode's only path.
4. **No case is a multi drag, an arc drag, or a free angle drag.** No log contains `EVT_START_MULTIDRAG`; `free_angle_mode: false` in all seven `.settings`; every start item is a `segment` or a `via`, never an `arc`.
5. **Three of the four walkaround cases set `restrict_angles`,** so `REQUIRE_OBTUSE_ANGLES` (section 6) is required for three of the seven goldens, not optional polish. `drag-acute-fallback`'s name is a direct reference to `dragFixCorner`'s job.
6. **No case contains an `EVT_FIX`.** The harness compares `ROUTER::GetUpdatedItems` after the last move (`qa/tools/pns/pns_log_player.cpp:63` to `:89`, `:322`), which for a drag reads `m_dragger->CurrentNode()` (`pcbnew/router/pns_router.cpp:844`). So `Router::pending_update` with a drag branch is what the goldens are read from, and `fix_route` is exercised only by tests this crate writes itself.

One replay detail: the log player calls `StartDragging( evt.p, ritems, 0 )` (`qa/tools/pns/pns_log_player.cpp:169`) with a drag mode of literal zero, and `ritems` is every board item the event's uuid list resolves to (`:120` to `:124`). Each of the seven logs carries exactly one uuid, so `ITEM_SET::Count( SEGMENT_T | ARC_T )` is at most one and `DRAGGER` is always chosen (`pcbnew/router/pns_router.cpp:187`).

### 10.2 Proposed order

Ten steps, smallest self contained piece first. Each names what it unlocks; "unlocks" means the corpus case can move from `#[ignore]`d to a real tier 1 and tier 2 assertion in `tests/kicad_replay.rs`.

**Step 1. `Line::drag_segment` and `Line::snap_to_neighbour_segments`.**
`pns_line.cpp:1230` to `:1397` and `:1185` to `:1228`. Pure geometry, no node, no router, unit testable next to `drag_corner`'s existing tests (`src/line.rs:2927` onward). Every one of the six segment cases goes through it on every move, so getting it wrong is expensive later and cheap now. Do not port the `m_line.PointCount() == 1` branch at `:1387` (section 3.4).
*Unlocks: nothing directly. Prerequisite for everything.*

**Step 2. Lift `via_pushout_force` out of the placer module.**
`src/placer/line_placer.rs:2121`, together with `single_step_force`, `move_via_by` and `move_via_to`. No behaviour change, no new tests beyond moving the existing ones. Doing it before the dragger exists keeps the diff readable.
*Unlocks: nothing. Prerequisite for step 8.*

**Step 3. `Dragger::start` and the mode decision, plus the mark obstacles path.**
`Start` (`pns_dragger.cpp:304`), `startDragSegment` (`:118`), `Drag`'s dispatch and both fallbacks (`:998`), `dragMarkObstacles`'s segment and corner cases (`:381`), `Traces`, `CurrentNode`. Corner drag comes free here, because `Line::drag_corner` already exists. Skip `checkVirtualVia` (nothing to find, section 9.5), skip the arc cases, skip the via cases.
*Unlocks: nothing in the corpus. Needs crate scenario tests, in the style of `tests/placer.rs`.*

**Step 4. The session facade and the event log.**
`Router::start_dragging`, the drag branches of `move_to`, `fix_route` and `pending_update`, `SessionEvent::StartDragging`, `RouterState::DragSegment` actually entered, and the replay harness reading `EVT_START_DRAG` (`tests/support/pns_log.rs` already parses it as `EventKind::StartDrag`).
*Unlocks: all seven cases replaying and terminating. Not the whole of tier 1: with only `dragMarkObstacles` behind them, six of the seven end with the dragged trace lying across something, because that routine only reports what it runs into. The measured numbers are in `doc/work/009-dragging.md`. The goldens will not match yet. Also the point at which the seven `#[ignore = "dragging is milestone 8"]` reasons in `tests/kicad_replay.rs:366` to `:411` and the module comment at `:63` to `:66` stop being true twice over: dragging is milestone 9, and the ignore reason changes per case from here on.*

**Step 5. `optimize_and_update_dragged_line`, `best_anchor_for_point`, `point_has_bad_corner`.**
`pns_dragger.cpp:569` to `:682`. The optimizer, the preserved vertex and the restricted area. Everything it needs already exists in `src/optimizer.rs`.
*Unlocks: nothing on its own; every later step's geometry ends here, so no golden can match without it.*

**Step 6. `dragWalkaround` and `try_walkaround`.**
`pns_dragger.cpp:685` to `:799`, segment and corner cases only. Four setters and a `route` call, with the length limit factor **30.0** (`:692`), which is the number that distinguishes it from every other walkaround caller.
*Unlocks: `walk-with-teardrops` at tier 2. It is the only walkaround case that does not set `restrict_angles`, so it is the honest first golden.*

**Step 7. `REQUIRE_OBTUSE_ANGLES`: `Constraint::ObtuseOnly` and `drag_fix_corners`.**
`pns_optimizer.cpp:305` to `:346`, `:738` to `:798`, `:801` to `:846`, wired in at `src/optimizer.rs:1676` where the placeholder comment already sits.
*Unlocks: `drag-acute-fallback`, `drag-walk-optimize-a` and `drag-walk-optimize-fix-corners` at tier 2, three goldens for one pass. `drag-acute-fallback` is 857 moves on one drag, so it is also the milestone's performance canary.*

**Step 8. `dragShove`, segment and corner cases.**
`pns_dragger.cpp:802` to `:863`. The `preShoveNode->Remove` ordering (`:830`), `SHP_SHOVE | SHP_DONT_LOCK_ENDPOINTS` plus `SHP_REVERSED` for corner 0 (`:832`, `:837`), the `width / 2` snap threshold rather than `width / 4` (`:818`), and `m_lastDragSolution` (`:858`) which the restore path depends on. Nothing new is needed from `Shove` (section 4).
*Unlocks: `simple-drag-shove-singlelayer` and `walk_drag_seg_against_board_edge` at tier 2. The first is the largest golden in the corpus, 134 added and 139 removed across ten nets, so it is the real test of the shove under a drag head.*

**Step 9. The via drag.**
`startDragVia` (`:257`), `findViaFanoutByHandle` (`:267`), `dragViaMarkObstacles` (`:452`), `propagate_via_forces` (`:62`) with `MouseTrailTracer::trail_lead_vector`, `dragViaWalkaround` (`:492`), and `dragShove`'s `DM_VIA` case (`:908`) with its walkaround fallback. Decide E5 and E6 here rather than inherit them.
*Unlocks: `issue23449-shove-lone-via-drag-crash` at tier 1, and at tier 2 only if the crate's drag also ends with an empty delta. Its golden being empty makes it a weak tier 2 signal, so write a crate scenario test for a via drag that actually moves something.*

**Step 10. Free angle mode, then the LibrePCB side.**
Free angle is section 2.17: `Line::drag_corner_with`'s `free_angle` branch, `startDragSegment`'s second case, and `Drag`'s bypass, about fifteen lines that touch nothing else. Then the LibrePCB integration from `doc/work/009-dragging.md`: a drag from the select tool through `BoardPnsRouter`, one undo entry per drag, which is `performDragging`'s single `UndoRedoBlock` (section 7.2).
*Unlocks: nothing in the corpus. The acceptance criterion's second half.*

**Step 11, only if multi drag stays in scope.**
`LineChain::point_along`, `clip_to_other_line` over `World::collide_lines`, `MdragLine`, `Start`'s four phases, `try_posture`'s three variants, the three per mode routines and the leader restoration. Twelve of the twenty eight errata in section 8 are in this file, five of its eighteen per line fields are dead, and no corpus case covers any of it.

Before starting it, resolve the contradiction between `PLAN.md`, which lists multi drag under "Non-goals for now", and `doc/work/009-dragging.md`, which has "Multi drag" as a task. If it goes ahead, fix E17 (the attempt 1 index), E18 (the reversal before the mode decision) and E28 (the unspecified corner mode order) rather than reproduce them: none of them is pinned by a golden, and E28 is a determinism rule this crate does not get to break.

### 10.3 What is deliberately not in the order

- **Arc drag.** `startDragArc` (`pns_dragger.cpp:155`) and `LINE::DragArc` (`pns_line.cpp:911`) need `SHAPE_ARC`, `CIRCLE::ConstructFromTanTanPt`, `CalcArcMid` and the chain's arc index vector. That is the arc milestone, which `PLAN.md` has on hold.
- **Component drag.** `COMPONENT_DRAGGER` is a third `DRAG_ALGO` implementation, reached only when every start item is a `SOLID` (`pcbnew/router/pns_router.cpp:176`), and `ROUTER::GetUpdatedItems` does not even report it (E15). It was out of the order above when this note was written; section 11 was added on 2026-09-10 when it came back into scope as the tail of the milestone, and it is a step of its own, after step 11 and dependent on nothing in it.
- **`checkVirtualVia`.** Blocked on the `FixupVirtualVias` decision (section 9.5), and it only affects which mode a click near a width change produces.

---

## 11. `COMPONENT_DRAGGER`

Read 2026-09-10 at the same commit. `pcbnew/router/pns_component_dragger.cpp` is 275 lines including the licence header and `pcbnew/router/pns_component_dragger.h` is 141, which makes it the smallest of the three `DRAG_ALGO` implementations by a wide margin: no shove, no walkaround, no optimizer, no mode, no failure path.

The mechanism in one sentence: **clone every selected pad at the cursor offset, move rigidly anything that runs between two selected pads, and drag one corner of every other attached trace to where its pad end went.**

`ROUTER::StartDragging` reaches it when every item of the set is a `SOLID_T` (`pcbnew/router/pns_router.cpp:176`), which section 1.3 already records, and puts the router in `DRAG_COMPONENT`. That state is where erratum E15 bites: `ROUTER::GetUpdatedItems` has no branch for it.

### 11.1 State

`pcbnew/router/pns_component_dragger.h:119` to `:136`.

| Member | Line | Role |
| --- | --- | --- |
| `struct DRAGGED_CONNECTION` | `:120` | one attached trace: `origLine`, `attachedPad`, `p_orig`, `p_next`, `offset`. |
| `std::set<SOLID*> m_solids` | `:128` | the pads being dragged. **Pointer ordered**, see E31. |
| `std::set<ITEM*> m_fixedItems` | `:129` | segments and arcs that move rigidly with the pads. Pointer ordered too. |
| `std::vector<DRAGGED_CONNECTION> m_conns` | `:130` | the traces that get one corner dragged. Insertion ordered. |
| `bool m_dragStatus` | `:132` | written once, in the constructor (`.cpp:38`). See E29. |
| `ITEM_SET m_draggedItems` | `:133` | what `Traces()` answers: the cloned solids, the cloned fixed items and the re-dragged lines, all three in one set. |
| `ITEM_SET m_initialDraggedItems` | `:134` | the primitives as they arrived, removed from the fresh branch at the top of every `Drag`. |
| `NODE* m_currentNode` | `:135` | the one branch, rebuilt on every `Drag`. |
| `VECTOR2I m_p0` | `:136` | where the gesture started. |

`DRAGGED_CONNECTION::p_orig` and `p_next` are **not** filled in by `Start`; only `Drag` writes them (`.cpp:184`, `:185`). `offset` is written by `Start` and is zero for everything except the unconnected trace end case of section 11.2.

The node tree is one level, where the single dragger has two:

```
world (root, handed in by SetWorld)
 +-- m_currentNode = m_world->Branch()      rebuilt on every Drag, .cpp:161
```

There is no pre drag node, because there is no shove to stand one on.

### 11.2 `Start`

`.cpp:48` to `:153`. It classifies, and it never fails: the only `return` is `true` at `:152`.

```
Start(aP, aPrimitives):
    m_currentNode = nullptr                                        :52
    m_initialDraggedItems = aPrimitives                            :53
    m_p0 = aP                                                      :54
    seenItems = {}                     # unordered_set<LINKED_ITEM*>  :56

    for item in aPrimitives.Items():                               :115
        if item->Kind() != SOLID_T: continue                       :117
        solid = (SOLID*) item
        m_solids.insert(solid)                                     :122
        if not item->IsRoutable(): continue                        :124

        jt = m_world->FindJoint(solid->Pos(), solid)                :127
        for link in jt->LinkList():                                :129
            if link->OfKind(SEGMENT_T | ARC_T):                    :131
                addLinked(solid, jt, link)                         :132

        # a trace end that lies inside the pad but is not jointed to it
        m_world->QueryJoints(solid->Hull().BBox(), extraJoints,
                             solid->Layers(), SEGMENT_T | ARC_T)   :137
        for extraJoint in extraJoints:                             :140
            if extraJoint->Net() == jt->Net()
               and extraJoint->LinkCount() == 1:                   :142
                li = extraJoint->LinkList().front()                :144
                if li->Collide(solid, m_world, solid->Layer()):    :146
                    addLinked(solid, extraJoint, li,
                              extraJoint->Pos() - solid->Pos())    :147
    return true                                                    :152
```

Four things to carry over.

1. **A pad that is not routable still moves.** `:124` only skips the connection search, so an NPTH pad or a mask-only pad is cloned at the new position and drags nothing.
2. **`FindJoint( solid->Pos(), solid )`** is the two argument overload, `FindJoint( aPos, aItem->Layers().Start(), aItem->Net() )` (`pcbnew/router/pns_node.h:478`). It is dereferenced unchecked at `:129`; the guard that makes that safe is `:124`, because `NODE::addSolid` links a joint only for a routable solid (`pcbnew/router/pns_node.cpp:609`).
3. **The `extraJoints` block is the whole of the "pad is not connected" handling.** A trace that ends inside the pad's hull, on the pad's net, on the pad's layers, with exactly one link at that joint, and that actually collides with the pad, is dragged along as if it were connected. `LinkCount()` takes the default mask `-1`, so an end that also carries a via is not picked up. The offset it is recorded with is the distance from the pad centre to that dangling end, which is what keeps the trace end in the same place relative to the pad as the pad moves.
4. **`solid->Hull()`** is called with all three defaults, `aClearance = 0`, `aWalkaroundThickness = 0`, `aLayer = -1` (`pcbnew/router/pns_solid.h:110`), so the query box is the bare copper box.

`addLinked` (`:58` to `:113`) is where the two "runs between two dragged pads" cases live:

```
addLinked(aSolid, aJoint, aItem, aOffset = {}):
    if aItem in seenItems: return                                  :61
    seenItems.insert(aItem)                                        :64

    # case 1: this one segment goes straight from pad to pad
    otherEnd = (aJoint->Pos() == aItem->Anchor(0)) ? aItem->Anchor(1)
                                                  : aItem->Anchor(0)   :67
    otherJoint = m_world->FindJoint(otherEnd, aItem->Layer(), aItem->Net())  :69
    if otherJoint and otherJoint->LinkCount(SOLID_T):                  :71
        for otherItem in otherJoint->LinkList():                       :73
            if aPrimitives.Contains(otherItem):                        :75
                m_fixedItems.insert(aItem); return                     :77, :78

    cn.origLine    = m_world->AssembleLine(aItem, &segIndex)           :86
    cn.attachedPad = aSolid                                            :87
    cn.offset      = aOffset                                           :88

    # case 2: the whole assembled line goes from pad to pad
    jA = m_world->FindJoint(line.CPoint(0),     aItem->Layer(), aItem->Net())   :92
    jB = m_world->FindJoint(line.CLastPoint(),  aItem->Layer(), aItem->Net())   :93
    wxASSERT(jA == aJoint or jB == aJoint)                             :95
    jSearch = (jA == aJoint) ? jB : jA                                 :96
    if jSearch and jSearch->LinkCount(SOLID_T):                        :98
        for otherItem in jSearch->LinkList():                          :100
            if aPrimitives.Contains(otherItem):                        :102
                for item in cn.origLine.Links():                       :104
                    m_fixedItems.insert(item)                          :105
                return                                                 :107

    m_conns.push_back(cn)                                              :112
```

`segIndex` is written by `AssembleLine` and never read; it exists only because the out parameter has no default.

Both cases are the same idea at two granularities. Case 1 catches one segment whose far anchor carries a dragged pad, case 2 catches a run of segments whose far *joint* does, and case 2 puts **every** link of the run into `m_fixedItems`, not just the seed. A trace between two dragged pads therefore translates rigidly instead of being re-shaped, which is the only way to keep it straight when both of its ends move by the same vector.

`seenItems` is an `unordered_set` but is only ever asked `count` (`:61`), so its order is not observable. It de-duplicates the *seed segment*, not the assembled line, which is why the same line can be reached a second time from the pad at its other end; the second visit lands in case 2 and inserts into a `std::set` that already holds those links, so it is idempotent.

### 11.3 `Drag`

`.cpp:156` to `:244`. Always answers `true` (`:243`), and it re-derives everything from `Start`'s records rather than from the previous drag, so a component drag never accumulates.

```
Drag(aP):
    m_world->KillChildren()                                        :160
    m_currentNode = m_world->Branch()                              :161
    for item in m_initialDraggedItems: m_currentNode->Remove(item) :163, :164
    m_draggedItems.Clear()                                         :166

    for s in m_solids:                                             :168
        p_next = aP - m_p0 + s->Pos()                              :170
        snew = (SOLID*) s->Clone(); snew->SetPos(p_next)           :171, :172
        m_draggedItems.Add(snew.get())                             :174
        m_currentNode->Add(std::move(snew))                        :175
        if not s->IsRoutable(): continue                           :177
        for l in m_conns where l.attachedPad == s:                 :180, :182
            l.p_orig = s->Pos() + l.offset                         :184
            l.p_next = p_next    + l.offset                        :185

    for item in m_fixedItems:                                      :190
        m_currentNode->Remove(item)                                :192
        SEGMENT_T: s_new = clone; s_new->SetEnds(aP - m_p0 + A,
                                                 aP - m_p0 + B)    :199, :202
        ARC_T:     a_new = clone; a_new->Arc().Move(aP - m_p0)     :213, :216
        default:   wxFAIL_MSG                                      :224
        m_draggedItems.Add(new); m_currentNode->Add(new)           :204, :205

    for cn in m_conns:                                             :228
        l_new = LINE(cn.origLine)                                  :230
        l_new.Unmark()                                             :231
        l_new.ClearLinks()                                         :232
        l_new.DragCorner(cn.p_next, cn.origLine.CLine().Find(cn.p_orig))   :233
        m_draggedItems.Add(l_new)                                  :236
        l_orig = LINE(cn.origLine)                                 :238
        m_currentNode->Remove(l_orig)                              :239
        m_currentNode->Add(l_new)                                  :240
    return true                                                    :243
```

Five details a port cannot paraphrase away.

1. **`SOLID::Clone` deep copies the hole.** The copy constructor clones `m_shape` and `m_hole` (`pcbnew/router/pns_solid.h:63`, `:66`), and `SetPos` moves both (`pcbnew/router/pns_solid.cpp:81` to `:90`). A port whose hole is a separate arena item has to build a fresh hole for the clone and translate it by the same delta, or two pads end up sharing one drill.
2. **`l_new.Unmark()` runs while `l_new` still holds `origLine`'s links** (`:231` before `:232`), and `LINE::Unmark` clears the mask on every link as well as on the line (`pcbnew/router/pns_line.cpp:184`). So the *board's* segments lose their marker bits, not only the copy's. Reproduce the order.
3. **`m_draggedItems.Add( l_new )` copies the line before the node links it** (`:236` before `:240`; `ITEM_SET::Add( const LINE& )` clones, `pcbnew/router/pns_itemset.cpp:36`). What `Traces()` answers is therefore an unlinked snapshot, which is what makes it safe to hold across the next `Drag`.
4. **`Find( cn.p_orig )` can answer `-1`**, and `LINE::DragCorner` guards on it with `wxCHECK_RET( aIndex >= 0 )` (`pcbnew/router/pns_line.cpp:886`), so the line is re-added unchanged rather than having its last corner dragged. See E34.
5. **The removal at `:239` uses `l_orig`, a copy that kept its links**, while `l_new` had them cleared at `:232`. That is the same "the links, not the geometry, are what the removal uses" trick section 8.5 lists for the other two draggers.

There is **no collision test in `Drag` at all**, no walkaround, no shove and no optimizer, so `Settings().Mode()` is ignored outright: a component drag behaves like mark obstacles mode whatever the router is set to, and even the highlighting is left to `ROUTER::markViolations`.

### 11.4 `FixRoute`, `CurrentNode` and `Traces`

```
FixRoute(aForceCommit):                                            :247
    node = CurrentNode()
    if node and (Settings().AllowDRCViolations() or aForceCommit
                 or not node->CheckColliding(m_draggedItems)):     :253
        Router()->CommitRouting(node); return true                 :255, :256
    return false                                                   :260

CurrentNode(): return m_currentNode ? m_currentNode : m_world      :264 to :267
Traces():      return m_draggedItems                               :270 to :273
```

`FixRoute` is the **only** place a component drag looks at collisions, and unlike `DRAGGER::FixRoute` (erratum E7) it honours `aForceCommit` in every mode, because there is no `m_forceMarkObstaclesMode` branch to hide it in. `CheckColliding( ITEM_SET )` is the plain loop of `pcbnew/router/pns_node.cpp:478` over a set that holds cloned solids, cloned fixed segments and lines all at once. There is no re-drag branch: a refused fix simply answers false and the host's loop has already ended.

`CurrentNets()` answers an empty vector and `CurrentLayer()` answers `UNDEFINED_LAYER` (`.h:85`, `:96`), both with the comment "Currently unused for component dragging". `Mode()` answers `DM_COMPONENT` (`.h:108`) and has no caller (E1).

### 11.5 The host side

`ROUTER_TOOL::CanInlineDrag` offers a footprint drag when every selected item is a `FOOTPRINT` and `DM_FREE_ANGLE` is not in the mask (`pcbnew/router/router_tool.cpp:2751` to `:2754`), which is the one place the mode mask means anything (section 1.2).

`ROUTER_TOOL::InlineDrag` then builds the item set out of footprints rather than out of the selection (`:2796` to `:2903`):

```
if selection.Front() is not TRACE / VIA / ARC / FOOTPRINT: return 0     :2788
footprints = { selection.Front() } if it is a FOOTPRINT                 :2798, :2799
if selection.Size() > 1: every other item must be a FOOTPRINT too       :2802 to :2814
                         ("We can drag multiple footprints, but not a grab-bag")
...
m_router->SyncWorld()                                                   :2852
for footprint in footprints:                                            :2865
    for pad in footprint->Pads():   itemsToDrag.Add(FindItemByParent(pad))   :2867 to :2872
    for zone in footprint->Zones(): itemsToDrag.Add(FindItemsByParent(zone)) :2881 to :2884
    for shape in footprint->GraphicalItems():                           :2887
        if layer is Edge_Cuts, Margin or copper:
            itemsToDrag.Add(FindItemsByParent(shape))                   :2893, :2894
```

So the set is not only pads: a footprint's copper zones and its board outline, margin and copper graphics all become solids and all travel with it. That is why `COMPONENT_DRAGGER` skips anything that is not `SOLID_T` at `:117` rather than asserting, and why `:124` has to tolerate an unroutable member.

The rest of the gesture is the track path of section 7.4, plus four footprint only pieces the router does not provide:

- the **courtyard clearance check**, `DRC_INTERACTIVE_COURTYARD_CLEARANCE` initialised at `:2863` and run per motion at `:3124`, with each footprint moved and moved back around it (`:3119`, `:3129`);
- the **dynamic ratsnest**, `dynamicItems` collected at `:2877`, blocked at `:2903` and recomputed per motion at `:3134`;
- the **preview of everything that is not copper**: graphics, non copper pads, the reference and the value are cloned, translated by `m_endSnapPoint - p` and added to the view preview while the originals are hidden (`:3076` to `:3113`). The comment at `:3100` says the rest is the router's: "Pads with copper or holes are handled by the router", meaning through `updateView`'s added loop over the cloned solids;
- for a **single** footprint, the cursor is warped to the footprint anchor first when the user asked for that, so the footprint does not jump at the first motion (`:2971` to `:2989`).

The commit is the interesting half, and it is not `CommitDiff` shaped. `ROUTER::CommitRouting( NODE* )` reports the old solid as a removal and the new one as an addition like any other item, and `PNS_KICAD_IFACE` intercepts both:

- `RemoveItem` on a `SOLID_T` whose parent is a `PAD` records `m_fpOffsets[pad].p_old` and **returns without touching the commit** (`pcbnew/router/pns_kicad_iface.cpp:2629` to `:2636`);
- `createBoardItem` on a `SOLID_T` records `m_fpOffsets[pad].p_new` and returns `nullptr`, with the comment "Don't add to commit; we'll add the parent footprints when processing the m_fpOffsets" (`:2849` to `:2856`);
- `Commit()` then walks `m_fpOffsets`, computes `p_new - p_old` per pad, and moves each pad's **footprint** by that offset once, de-duplicated through `processedFootprints` (`:2918` to `:2933`).

So the board never sees a pad removed and re-added; it sees one footprint moved. A port whose commit is a value has to say the same thing, which is what `CommitDiff::moved_solids` is for: one `(host object, offset)` pair per moved pad, leaving the three item lists to the traces the drag re-shaped.

### 11.6 Errata

**E29. `m_dragStatus` is never assigned after the constructor.** `.cpp:38` sets it false and nothing writes it again. `GetForceMarkObstaclesMode` (`.h:113` to `:117`) writes that false into the out parameter and returns false, so the host's "Track violates DRC. (Ctrl+click to commit anyway.)" hint never appears for a component drag even though `FixRoute` will refuse for exactly that reason. Same shape as E23 for the multi dragger.

**E30. `Drag` has no failure path.** It returns `true` unconditionally (`:243`), tests no collisions, and keeps no "last good" solution, so there is nothing to restore and `ROUTER::moveDragging` always reports success. The routing mode is not read at all: no shove, no walkaround, no optimizer. The dead `class OPTIMIZER;` forward declaration at `.h:32` is the trace of an intention that was never carried out.

**E31. `m_solids` and `m_fixedItems` are `std::set` of raw pointers** (`.h:128`, `:129`), so `Drag`'s two loops (`:168`, `:190`) run in address order. Nothing downstream depends on the order today, because the loops only add to a node and to a set that is later scanned for a yes or a no, but `DESIGN.md` section 8 forbids it on principle. Order them by uid in a port. `m_fpOffsets` on the interface side is a `std::map<PAD*, ...>` with the same property (`pcbnew/router/pns_kicad_iface.h:188`), harmless only because every pad of one footprint carries the same offset.

**E32. `Start` clears none of `m_solids`, `m_fixedItems` or `m_conns`,** and the constructor does not either (`:35` to `:40`); only `m_currentNode`, `m_initialDraggedItems` and `m_p0` are reset (`:52` to `:54`). Calling `Start` twice on one instance would accumulate. Latent, because `ROUTER::StartDragging` allocates a fresh `COMPONENT_DRAGGER` per gesture (`pcbnew/router/pns_router.cpp:178`).

**E33. Both "runs between two dragged pads" tests count solids and then iterate every link.** `:71` tests `otherJoint->LinkCount( ITEM::SOLID_T )` and `:73` walks `otherJoint->LinkList()`, which is every link of the joint, so the `Contains` test at `:75` is offered segments and vias as candidates too; `:98` to `:102` is the same shape. It cannot misfire today, because the set only ever holds solids (`pcbnew/router/pns_router.cpp:176`) and a solid is exactly what the `LinkCount` guard promised was there. Filter the link list by `SOLID_T` in a port and the two tests say what they mean.

**E34. `cn.origLine.CLine().Find( cn.p_orig )` can answer `-1`.** `Find` returns `-1` for a point that is not a vertex (`libs/kimath/src/geometry/shape_line_chain.cpp:1253`), and `LINE::DragCorner` catches it with `wxCHECK_RET( aIndex >= 0 )` (`pcbnew/router/pns_line.cpp:886`) and returns, so the line is removed and re-added unchanged. The lookup cannot miss for a well formed board, because `p_orig` is either the pad's own joint position or a dangling end that `QueryJoints` found, and both are endpoints of the assembled line, but it is an unguarded assumption at the call site.

**E35. `wxASSERT( jA == aJoint || jB == aJoint )` at `:95` is what `jSearch` rests on.** In a release build the assertion is a no operation and `jSearch = ( jA == aJoint ) ? jB : jA` silently picks `jA` when neither end is the pad's joint, which would test the wrong end for a second dragged pad. It holds because `AssembleLine` stops at a pad, so one end of the assembled line is always the joint the seed hangs off.

**E36. The unconnected trace end block is almost unreachable, because its two conditions contradict each other.** `:142` demands `extraJoint->Net() == jt->Net()`, that is the same net as the pad, and `:146` then demands `li->Collide( solid, m_world, solid->Layer() )`. `ITEM::Collide` with no search context runs with `differentNetsOnly` true, and a same net pair takes `clearance = -1` at `pcbnew/router/pns_item.cpp:188` and answers false. Two ways out of the contradiction survive:

- **null nets.** The test at `:188` also requires `aHead->Net()` to be non null, so a netless pad with a netless trace end inside it falls through to the resolver and does collide. That is the case a port can exercise, and it is what `tests/component_dragger.rs` uses.
- **a user defined physical clearance rule.** `runPhysicalOnly` (`pns_item.cpp:125`, over `NODE::HasUserDefinedPhysicalConstraint`, `pcbnew/router/pns_node.h:145`) makes the resolver net blind, so every same net pair falls through as well.

On an ordinary board with nets and no physical rule, a trace end that stops inside a pad without being jointed to it is therefore **not** dragged along. Reproduce the code as it stands; the point of recording this is that a port which "fixes" the net test would change behaviour on every board rather than on the two cases above.

### 11.7 What the port needs

Everything in section 9.1 covers it, with three exceptions.

| What | Where | KiCad source |
| --- | --- | --- |
| `CommitDiff::moved_solids` | `src/router.rs` | `pcbnew/router/pns_kicad_iface.cpp:2634`, `:2854`, `:2918`. A pad is not removed and re-added on the board, its footprint is moved. |
| `RouterState::DragComponent` and the `pending_update` branch for it | `src/router.rs` | `pcbnew/router/pns_router.cpp:179`, and E15, which is the branch KiCad does **not** have. |
| A deep copy of a solid's hole | `src/component_dragger.rs` | `pcbnew/router/pns_solid.h:66`, `pns_solid.cpp:87`. The crate keeps the hole as a separate arena item, so the clone needs a fresh one. |

Nothing new is needed from the shove (it is never built), from the optimizer (it is never called), from the walkaround, from `MouseTrailTracer` or from `Line` beyond `drag_corner`, which milestone 9 already widened.
