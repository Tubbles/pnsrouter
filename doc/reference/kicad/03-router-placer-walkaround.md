# KiCad PNS: router facade, line placer, walkaround

Reference architecture note for a from-scratch Rust reimplementation. Source tree: sparse checkout of KiCad master at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85` (2026-09-06). All `path:line` citations are relative to `/home/Tubbles/dev/ref/kicad/`.

Scope: `ROUTER` (the facade), `LINE_PLACER` (single track interactive placement), `WALKAROUND`, `MOUSE_TRAIL_TRACER` (posture solver), `ROUTING_SETTINGS`, `SIZES_SETTINGS`, `pns_utils` (hull geometry), `LOGGER` and `DEBUG_DECORATOR`. `SHOVE` and `OPTIMIZER` internals are covered by a separate note; only their call surfaces appear here.

A note on the checkout: `common/advanced_config.cpp` and `include/advanced_config.h` are not present in this sparse checkout, so the default value of `ADVANCED_CFG::m_EnableRouterDump` and `m_PNSProcessClusterTimeout` could not be read. Everything else below is read from source in this tree.

---

## 1. ROUTER: the facade

### 1.1 Enums and state

`ROUTER_MODE` (`pcbnew/router/pns_router.h:67`) selects which `PLACEMENT_ALGO` subclass gets instantiated on `StartRouting`:

| value | numeric | placer created |
| --- | --- | --- |
| `PNS_MODE_ROUTE_SINGLE` | 1 | `LINE_PLACER` (`pcbnew/router/pns_router.cpp:444`) |
| `PNS_MODE_ROUTE_DIFF_PAIR` | 2 | `DIFF_PAIR_PLACER` (`pcbnew/router/pns_router.cpp:448`) |
| `PNS_MODE_TUNE_SINGLE` | 3 | `MEANDER_PLACER` (`pcbnew/router/pns_router.cpp:452`) |
| `PNS_MODE_TUNE_DIFF_PAIR` | 4 | `DP_MEANDER_PLACER` (`pcbnew/router/pns_router.cpp:456`) |
| `PNS_MODE_TUNE_DIFF_PAIR_SKEW` | 5 | `MEANDER_SKEW_PLACER` (`pcbnew/router/pns_router.cpp:460`) |

The enum starts at 1, which matters because the mode is serialised as a raw integer into the QA log (`json["mode"] = aLogData.m_Mode` at `pcbnew/router/pns_logger.cpp:112`).

`DRAG_MODE` (`pcbnew/router/pns_router.h:75`) is a bitmask, not an enumeration of alternatives: `DM_CORNER = 0x1`, `DM_SEGMENT = 0x2`, `DM_VIA = 0x4`, `DM_FREE_ANGLE = 0x8`, `DM_ARC = 0x10`, `DM_ANY = 0x17`, `DM_COMPONENT = 0x20`. Note `DM_ANY` is `0x17`, that is corner|segment|via|free_angle|arc, and deliberately excludes `DM_COMPONENT`.

`ROUTER::RouterState` (`pcbnew/router/pns_router.h:157`) has exactly four states: `IDLE`, `DRAG_SEGMENT`, `DRAG_COMPONENT`, `ROUTE_TRACK`. `RoutingInProgress()` is simply `m_state != IDLE` (`pcbnew/router/pns_router.cpp:120`). There is no separate "committing" state; the transition back to `IDLE` happens inside `StopRouting()`.

Which algorithm object exists is a function of the state: `m_placer` is non-null only in `ROUTE_TRACK`, `m_dragger` only in the two drag states. The dispatcher `ROUTER::Move` (`pcbnew/router/pns_router.cpp:494`) switches on the state, not on a null check.

Router construction (`pcbnew/router/pns_router.cpp:60`) sets `m_state = IDLE`, `m_mode = PNS_MODE_ROUTE_SINGLE`, `m_iterLimit = 0`, `m_settings = nullptr`, `m_iface = nullptr`, and `m_visibleViewArea.SetMaximum()`. The logger is allocated only when `ADVANCED_CFG::GetCfg().m_EnableRouterDump` is set (`pcbnew/router/pns_router.cpp:69`), so in a normal build `Logger()` returns null and every `m_logger->Log(...)` call site is guarded.

There is a file-static singleton `theRouter` (`pcbnew/router/pns_router.cpp:58`), set in the constructor and cleared in the destructor, reachable through `ROUTER::GetInstance()`. It is used from code that has no router pointer in hand: the mouse trail tracer fetches the debug decorator through it (`pcbnew/router/pns_mouse_trail_tracer.cpp:78`, `:112`), the optimizer reads the corner mode through it (`pcbnew/router/pns_optimizer.cpp:858`, `:1278`), `NODE` reads it at `pcbnew/router/pns_node.cpp:301`, and `VIA::Hull` uses it to reach `IsFlashedOnLayer` (`pcbnew/router/pns_via.cpp:243`). The source itself calls this "an ugly singleton". For a Rust port this is the single most important structural thing to design out: pass a `&RouterContext` (settings + iface + debug sink) explicitly.

### 1.2 The world node and the placer's branches

`NODE` is a persistent, copy-on-write board model. `NODE::Branch()` creates a lightweight child that records only the added/removed delta with respect to its parent (`pcbnew/router/pns_node.h:416`). `NODE::Commit(aNode)` folds a branch's delta into the root and kills all children of the root (`pcbnew/router/pns_node.h:454`). `NODE::KillChildren()` destroys all child nodes and is only valid on the root (`pcbnew/router/pns_node.h:489`).

`ROUTER::m_world` is a `unique_ptr<NODE>` owning the root (`pcbnew/router/pns_router.h:278`). `SyncWorld` (`pcbnew/router/pns_router.cpp:95`) drops the old root, allocates a fresh one, and fills it through the host interface between `BeginBulkAdd()` and `FinalizeBulkAdd()`, then calls `FixupVirtualVias()`.

The chain of nodes during a single-track routing session is:

```
world (root, owned by ROUTER)
 └── rootNode          = world->Branch()          LINE_PLACER::initPlacement, pns_line_placer.cpp:1464
      │                  becomes m_world inside the placer (setWorld, :1468)
      │                  start item is split here (SplitAdjacentSegments, :1466)
      ├── m_currentNode = m_world initially       pns_line_placer.cpp:1476
      │    └── m_lastNode = m_currentNode->Branch()   rebuilt on every Move, pns_line_placer.cpp:1535
      └── shove root    = m_world->Branch()       SHOVE ctor argument, pns_line_placer.cpp:1478
```

The placer's `m_world` member shadows the router's: inside `LINE_PLACER`, `m_world` is the *branch* created in `initPlacement`, not the router's root. `ROUTER::GetWorld()` still returns the root.

Ownership discipline as written in C++: `m_currentNode` and `m_lastNode` are raw pointers into the branch tree. `m_lastNode` is deleted explicitly at the top of every `Move` (`pcbnew/router/pns_line_placer.cpp:1490`) and reallocated at `:1535`. Everything else is reclaimed by `KillChildren()` on the root, called from `initPlacement` (`:1463`), `AbortPlacement` (`:2152`), and `ROUTER::StopRouting` (`pcbnew/router/pns_router.cpp:990`).

The split of responsibility between the two nodes is:

- `m_currentNode` is the world state the algorithms route against. In shove mode it is repointed to `m_shove->CurrentNode()` before and after each shove run (`pcbnew/router/pns_line_placer.cpp:930`, `:963`).
- `m_lastNode` is a postprocessed copy: the trace under the cursor is added to it, adjacent segments at the end item are split into it, and loops are removed from it (`pcbnew/router/pns_line_placer.cpp:1537`). `CurrentNode( aLoopsRemoved = true )` returns `m_lastNode` when it exists, else `m_currentNode` (`pcbnew/router/pns_line_placer.cpp:1278`).

The host sees `m_lastNode` (via `movePlacing` calling `updateView( m_placer->CurrentNode( true ), ... )`, `pcbnew/router/pns_router.cpp:827`), and `CommitPlacement` commits `m_lastNode` (`pcbnew/router/pns_line_placer.cpp:1820`).

### 1.3 Session lifecycle

**SyncWorld** (`pcbnew/router/pns_router.cpp:95`). Called once before a session. Rebuilds the root from the host board.

**isStartingPointRoutable** (`pcbnew/router/pns_router.cpp:222`). Gate in front of `StartRouting`, skipped entirely if `Settings().AllowDRCViolations()` (`:224`). Steps:

1. For diff pair mode, reject if `m_sizes.DiffPairGap() < m_sizes.MinClearance()` (`:229`).
2. Query hover items at the start point; for each, skip `Edge_Cuts` items (`:242`) and items not on the start layer (`:245`). If any item is routable, all failure reasons are cleared and the loop breaks (`:248`). Otherwise a per-type message is chosen: non-plated hole (`:264`), rule area disallowing tracks, named or unnamed (`:277`, `:282`), text item (`:290`).
3. Single-track mode: build a degenerate two-point `LINE` at the start point with `m_sizes.TrackWidth()` and test `m_world->CheckColliding`. If it collides, retry with `m_sizes.BoardMinTrackWidth()`; only if *that* also collides is the start rejected with "The routing start point violates DRC." (`:336`). The rationale is in the comment at `:322`: a width-only collision should not block starting.
4. Diff pair mode: find the primitive pair via `DIFF_PAIR_PLACER::FindDpPrimitivePair` (`:352`). When starting from a segment or arc, compare the measured anchor-to-anchor gap against `DiffPairGap() + DiffPairWidth()` with a 10 percent tolerance (`tolerance = configuredGap / 10`, `:369`) and reject a mismatch. Then the same two-width collision probe as single mode, on both polarities.

Failure paths call `SetFailureReason` and the host reads it back via `FailureReason()` (`pcbnew/router/router_tool.cpp:1739`).

**StartRouting** (`pcbnew/router/pns_router.cpp:434`). Clears the rule resolver caches, runs `isStartingPointRoutable`, constructs the mode-appropriate placer, pushes sizes, layer, debug decorator and logger into it, then calls `m_placer->Start`. On success sets `m_state = ROUTE_TRACK` and logs `EVT_START_ROUTE` with the sizes and `m_placer->CurrentLayer()` (`:479`). On failure resets the placer and stays `IDLE`.

**Move** (`pcbnew/router/pns_router.cpp:494`). Logs `EVT_MOVE`, then dispatches to `movePlacing` or `moveDragging`. Note that `GetRuleResolver()->ClearTemporaryCaches()` at `:512` is only reached on the `default:` branch, that is when the router is idle; it never runs during an actual move. Worth preserving or fixing deliberately, not by accident.

`movePlacing` (`pcbnew/router/pns_router.cpp:789`) erases the view, calls the placer's `Move`, then draws every trace with `PNS_HEAD_TRACE` flag (`router_preview_item.h:50`), and separately draws the trailing via with a clearance that accounts for excess hole clearance beyond the annular ring (`:811-819`). Finally `updateView( m_placer->CurrentNode( true ), current )`.

**FixRoute** (`pcbnew/router/pns_router.cpp:915`). Logs `EVT_FIX` and forwards to `m_placer->FixRoute( aP, aEndItem, aForceFinish )` or the dragger's `FixRoute( aForceCommit )`. The `aForceCommit` argument is a dragger-only parameter; the placer never sees it.

**ContinueFromEnd** (`pcbnew/router/pns_router.cpp:617`). Captures the current layer and end, finds the nearest unconnected ratsnest anchor, calls `CommitRouting()` (which commits and stops), then `StartRouting` from the far anchor on `currentLayer` if the anchor's layer range overlaps it, else on `otherEndLayers.Start()` (`:642`). Finally does a single `Move( currentEnd, nullptr )` back toward where the user was, and hands the new start item back through the out-parameter.

**Finish** (`pcbnew/router/pns_router.cpp:569`). Only valid in `ROUTE_TRACK`. Finds the nearest unconnected anchor and then iterates `Move( otherEnd, otherEndItem )` up to five times (`int triesLeft = 5;` at `:594`), stopping early once `placer->CurrentEnd()` stops changing. If the settled end equals the target and the layer ranges overlap, calls `FixRoute( otherEnd, otherEndItem, false, false )`. The retry loop exists because a single `Move` may get partially stuck and a second attempt from the new state can get further.

**GetNearestRatnestAnchor** (`pcbnew/router/pns_router.cpp:518`) has two modes: if the user has drawn at least one segment, `TOPOLOGY::NearestUnconnectedAnchorPoint` relative to the drawn trace; otherwise find the joint at `CurrentStart()` and take `TOPOLOGY::NearestUnconnectedItem` from it.

**UndoLastSegment** (`pcbnew/router/pns_router.cpp:946`). Logs `EVT_UNFIX` and returns `m_placer->UnfixRoute()`, an `optional<VECTOR2I>` that the host uses to warp the cursor (`pcbnew/router/router_tool.cpp:1865`). Note it dereferences `m_placer` after only checking `RoutingInProgress()`, so a dragging session would crash here; in practice the host only binds the action while routing.

**CommitRouting()** no-arg (`pcbnew/router/pns_router.cpp:958`) calls `m_placer->CommitPlacement()` then `StopRouting()`.

**CommitRouting(NODE\*)** (`pcbnew/router/pns_router.cpp:862`) is the real commit. It bails immediately if `m_state == ROUTE_TRACK && !m_placer->HasPlacedAnything()`. It diffs the node against the root with `GetUpdatedItems`, then classifies:

```
for item in removed:
    if item has a Parent and some added item shares that Parent:
        move that added item into `changed`   # preserves UUID and pad data
    else if not item->IsVirtual():
        iface->RemoveItem(item)
for item in added   (post-move):  if !IsVirtual: iface->AddItem(item)
for item in changed:              if !IsVirtual: iface->UpdateItem(item)
iface->Commit()
m_world->Commit(aNode)
```

The remove/add to update collapsing (`pcbnew/router/pns_router.cpp:877`) is what keeps board item UUIDs stable across a route, which in turn is what makes the QA log's `removedItems` set meaningful.

**StopRouting** (`pcbnew/router/pns_router.cpp:967`). Pushes modified nets to the host for ratsnest update *before* the early return on `!RoutingInProgress()`, then resets both algorithm objects, erases the view, sets `IDLE`, and calls `m_world->KillChildren()` and `m_world->ClearRanks()`.

**AbortPlacement** exists on the placer (`pcbnew/router/pns_line_placer.cpp:2150`) and just kills children and nulls `m_lastNode`, but `ROUTER` never calls it in this revision.

### 1.4 What the host sees: updateView and markViolations

`updateView` (`pcbnew/router/pns_router.cpp:745`):

1. Only calls `markViolations` when the mode is `PNS_MODE_ROUTE_SINGLE` or `PNS_MODE_ROUTE_DIFF_PAIR` (`:757`). The comment at `:753` explains: length tuning cannot create clearance violations by construction, and `markViolations` is expensive under custom DRC rules (issue 24052 is cited).
2. `aNode->GetUpdatedItems( removed, added )`, then `GetRuleResolver()->ClearCacheForItems( added )`.
3. Each added item is displayed with its own clearance; each removed item is hidden.

`markViolations` (`pcbnew/router/pns_router.cpp:670`) queries colliding items for every current item, plus a separate query for the via if the line ends with one (`:711`). Items currently being dragged are skipped (`:726`). Each obstacle gets `MK_VIOLATION` OR'd into its marker and is redrawn as a clone with the resolved clearance. Two subtleties in the `updateItem` lambda:

- a multilayer obstacle colliding with a single-layer current item gets its clone forced onto the current item's layer, unless it has unique per-layer shapes (`:682`);
- a compound-shape primitive is not removed from the view, because only one primitive of the object is being highlighted (`:688`).

Additionally, `line->GetBlockingObstacle()` is drawn if set (`:738`). Only `rhMarkObstacles` writes that field, and in this revision it only ever writes `nullptr` (`pcbnew/router/pns_line_placer.cpp:811`); the code that would set it is inside the `#if 0` block at `:840`.

`GetUpdatedItems` (`pcbnew/router/pns_router.cpp:833`) is the batch variant used by the QA log player: it returns the node delta plus clones of the current head items.

### 1.5 Small commands

- `FlipPosture` (`pcbnew/router/pns_router.cpp:1001`) forwards to the placer only in `ROUTE_TRACK`.
- `SwitchLayer` (`:1010`) forwards to `m_placer->SetLayer` and returns its bool.
- `ToggleViaPlacement` (`:1019`) reads the current state and inverts it, then logs `EVT_TOGGLE_VIA` carrying `m_sizes` but no position and no item.
- `SetOrthoMode` (`:1085`) forwards unconditionally if a placer exists.
- `ToggleCornerMode` (`:1069`) cycles `MITERED_45 -> ROUNDED_45 -> MITERED_90 -> ROUNDED_90 -> MITERED_45` and writes it back into the settings object, which is shared with the host.
- `BreakSegmentOrArc` (`:1106`) makes a throwaway branch, constructs a stack `LINE_PLACER` purely to reach `SplitAdjacentSegments`/`SplitAdjacentArcs`, and either commits or deletes the branch.
- `SetIterLimit`/`GetIterLimit`/`m_iterLimit` (`pcbnew/router/pns_router.h:224`, `:288`) are initialised to zero (`pcbnew/router/pns_router.cpp:74`) and read by nobody in the tree. This is dead API; do not port it. The real iteration limits live in `ROUTING_SETTINGS` (walkaround, shove, via force propagation) and as hard-coded constants inside the algorithms.
- `GetLastCommittedLeaderSegments` (`pcbnew/router/pns_router.cpp:940`) returns `m_leaderSegments`, which is only ever populated by `moveDragging` from `MULTI_DRAGGER::GetLastCommittedLeaderSegments` (`pcbnew/router/pns_router.cpp:663`, `pcbnew/router/pns_multi_dragger.h:112`). `LINE_PLACER` never contributes to it. Both `StartDragging` overloads clear it (`:161`, `:168`). The single consumer is `pcbnew/router/router_tool.cpp:3164`. So: multi-drag only, despite living on the generic router.

### 1.6 Sequence of one single-track session

```
host                         ROUTER                      LINE_PLACER                 NODE tree
----                         ------                      -----------                 ---------
SyncWorld()             ---> build root                                              world
                              iface->SyncWorld(world)

(mouse move, no click)
updateStartItem()            (host side snapping only, pns_tool_base.cpp:332)

click ->
StartRouting(p, item, layer)
                             GetRuleResolver()->ClearCaches()
                             isStartingPointRoutable(p,item,layer)
                             m_placer = LINE_PLACER
                             UpdateSizes / SetLayer / SetDebugDecorator / SetLogger
                             m_placer->Start(p,item)  -->
                                                          m_currentStart = m_fixStart = m_currentEnd = p
                                                          m_currentNet = item->Net() or orphan handle
                                                          setInitialDirection(Settings().InitialDirection())
                                                          initPlacement():
                                                              world->KillChildren()
                                                              rootNode = world->Branch()          world
                                                              SplitAdjacentSegments(rootNode,..)   +-rootNode
                                                              m_currentNode = rootNode
                                                              m_shove = SHOVE(world->Branch())     +-shoveRoot
                                                          derive lastSegDir / initialDir from start item
                                                          mouseTrailTracer.Clear/AddTrailPoint/SetTolerance
                                                              /SetDefaultDirections/SetMouseDisabled
                                                          m_fixedTail.AddStage(fixStart, layer,
                                                              placingVia, direction, node)
                             m_state = ROUTE_TRACK
                             logger->Log(EVT_START_ROUTE, p, item, &sizes, layer)

mouse move ->
Move(p, endItem)             logger->Log(EVT_MOVE,...)
                             movePlacing:
                               iface->EraseView()
                               m_placer->Move(p,endItem) ->
                                                          delete m_lastNode
                                                          route(p) -> routeStep(p):
                                                              reduceTail(p)
                                                              updatePStart(m_tail)
                                                              routeHead(p,newHead,newTail)
                                                                 mode dispatch: rhMarkObstacles /
                                                                 rhWalkOnly / rhShoveOnly
                                                              handleSelfIntersections / handlePullback
                                                              optimizeTailHeadTransition else mergeHead
                                                          fallback via force-attach if p == m_p_start
                                                          current = Trace() = tail ++ head
                                                          m_lastNode = m_currentNode->Branch()      +-lastNode
                                                          SplitAdjacentSegments(m_lastNode,endItem,..)
                                                          removeLoops(m_lastNode, current)
                                                          updateLeadingRatLine()
                                                          mouseTrailTracer.AddTrailPoint(p)
                               display head trace with PNS_HEAD_TRACE
                               updateView(m_placer->CurrentNode(true) = m_lastNode, traces)
                                 -> markViolations + node delta -> iface->DisplayItem/HideItem

click ->
FixRoute(p,endItem,false,..) logger->Log(EVT_FIX,...)
                             m_placer->FixRoute ->
                                                          collision gate unless AllowDRCViolations
                                                          emit SEGMENT/ARC items into m_lastNode
                                                          emit VIA if head ends with one
                                                          simplifyNewLine(m_lastNode, lastItem)
                                                          if !realEnd:  (intermediate click)
                                                              setInitialDirection(d_last)
                                                              m_currentStart = p_last or p_pre_last
                                                              m_fixedTail.AddStage(...)
                                                              m_currentNode = m_lastNode
                                                              m_lastNode = m_lastNode->Branch()
                                                              shove->AddLockedSpringbackNode(currentNode)
                                                              reset head/tail, reseed mouse trail
                                                          else: lock springback, m_idle = true
                             returns realEnd; host breaks its loop when true

Esc / double click / final FixRoute ->
CommitRouting()              m_placer->CommitPlacement() ->
                                                          (shove mode) shove->RewindToLastLockedNode()
                                                          Router()->CommitRouting(m_lastNode)
                             CommitRouting(node):
                               node->GetUpdatedItems -> iface Remove/Add/UpdateItem
                               iface->Commit()
                               m_world->Commit(node)                                world (folded)
                             StopRouting():
                               iface->UpdateNet(per modified net)
                               m_placer.reset(); m_dragger.reset()
                               iface->EraseView(); m_state = IDLE
                               m_world->KillChildren(); m_world->ClearRanks()
```

---

## 2. Settings

### 2.1 ROUTING_SETTINGS

Declared `pcbnew/router/pns_routing_settings.h:58`, defaults set in the constructor at `pcbnew/router/pns_routing_settings.cpp:32-58`, serialisation parameter names and defaults at `:60-107`. It derives from `NESTED_SETTINGS` and calls `LoadFromFile()` at the end of construction (`:108`), so the host owns the lifetime and the router only holds a raw pointer (`ROUTER::LoadSettings`, `pcbnew/router/pns_router.h:241`).

| accessor | type | default | JSON key | consumed by |
| --- | --- | --- | --- | --- |
| `Mode()` | `PNS_MODE` | `RM_Walkaround` (`:35`) | `mode` | `LINE_PLACER::routeHead` dispatch `pcbnew/router/pns_line_placer.cpp:1022`; `FixRoute` collision node choice `:1589`; `Start` shove node choice `:1431`; `UnfixRoute` `:1792`; `CommitPlacement` `:1812`; `buildInitialLine` free angle gate `:2075`; `FollowMouse()` |
| `OptimizerEffort()` | `PNS_OPTIMIZATION_EFFORT` | `OE_MEDIUM` (`:36`) | `effort` | `rhWalkOnly` `pcbnew/router/pns_line_placer.cpp:747`; `rhShoveOnly` `:967`; `SHOVE::runOptimizer` `pcbnew/router/pns_shove.cpp:2028` |
| `ShoveVias()` | bool | true (`:39`) | `shove_vias` | `pcbnew/router/pns_shove.cpp:1060` only |
| `RemoveLoops()` | bool | true (`:37`) | `remove_loops` | `LINE_PLACER::Move` `pcbnew/router/pns_line_placer.cpp:1545` |
| `SuggestFinish()` | bool | false (`:40`) | `suggest_finish` | nothing in the tree; dead setting |
| `SmartPads()` | bool | true (`:38`) | `smart_pads` | `rhWalkOnly` `:762`, `rhShoveOnly` `:982`, `SHOVE::runOptimizer` `pcbnew/router/pns_shove.cpp:2079` |
| `FollowMouse()` | bool | true (`:41`) | `follow_mouse` | returns `m_followMouse && Mode() != RM_MarkObstacles` (`pcbnew/router/pns_routing_settings.h:100`); gates `reduceTail` and the merge stage in `routeStep` (`pcbnew/router/pns_line_placer.cpp:1134`, `:1198`) |
| `SmoothDraggedSegments()` | bool | true (`:47`) | `smooth_dragged_segments` | dragger snap thresholds only (`pcbnew/router/pns_dragger.cpp:398`, `:580`, `:727`, `:818`; `pcbnew/router/pns_multi_dragger.cpp:755`) |
| `JumpOverObstacles()` | bool | false (`:46`) | `jump_over_obstacles` | `pcbnew/router/pns_shove.cpp:829` only |
| `SetStartDiagonal` / `InitialDirection()` | bool -> `DIRECTION_45` | false, so `DIRECTION_45::N` (`:42`, `pcbnew/router/pns_routing_settings.cpp:112`) | `start_diagonal` | `LINE_PLACER::Start` `pcbnew/router/pns_line_placer.cpp:1393` |
| `AllowDRCViolations()` | bool | false (`:48`) | `can_violate_drc` | returns `m_routingMode == RM_MarkObstacles && m_allowDRCViolations` (`pcbnew/router/pns_routing_settings.h:117`), so it is inert outside mark-obstacles mode. Consumed at `pcbnew/router/pns_router.cpp:224`, `pcbnew/router/pns_line_placer.cpp:841`, `:1587`, and the draggers |
| `GetFreeAngleMode()` | bool | false (`:49`) | `free_angle_mode` | `buildInitialLine` `pcbnew/router/pns_line_placer.cpp:2075`, only together with `RM_MarkObstacles`; host status text `pcbnew/router/router_tool.cpp:3425` |
| `ShoveIterationLimit()` | int | 250 (`:43`) | `shove_iteration_limit` | `pcbnew/router/pns_shove.cpp:1890` |
| `ShoveTimeLimit()` | `TIME_LIMIT` | 1000 ms (`:44`) | `shove_time_limit` (lambda param) | `pcbnew/router/pns_shove.cpp:1891` |
| `WalkaroundIterationLimit()` | int | 40 (`:45`) | `walkaround_iteration_limit` | every `WALKAROUND` construction site, and the `WALKAROUND` constructor default `pcbnew/router/pns_walkaround.h:56` |
| `WalkaroundTimeLimit()` | `TIME_LIMIT` | never assigned in the constructor, so `TIME_LIMIT(0)` (`pcbnew/router/time_limit.h:32`) | none | nothing; dead |
| `GetSnapToTracks()` / `GetSnapToPads()` | bool | false, false (`:50`, `:51`) | `snap_to_tracks`, `snap_to_pads` | host only, `pcbnew/router/pns_tool_base.cpp:323`, `:325`; the router core never reads them, and the host *overwrites* them from `MAGNETIC_SETTINGS` on every `checkSnap` (`pcbnew/router/pns_tool_base.cpp:314`) |
| `GetCornerMode()` | `DIRECTION_45::CORNER_MODE` | `MITERED_45` (`:53`) | `corner_mode` (enum param, range `MITERED_45..ROUNDED_90`) | `buildInitialLine` `:2046`, `rhWalkOnly` smart-pad gate `:759`, `rhMarkObstacles` hull snapping `:825`, `rhShoveOnly` `:979`, `WALKAROUND::singleStep` hull squaring `pcbnew/router/pns_walkaround.cpp:129`, optimizer, shove, node |
| `GetOptimizeEntireDraggedTrack()` | bool | false (`:52`) | `optimize_dragged_track` | `pcbnew/router/pns_dragger.cpp:598` only |
| `GetAutoPosture()` | bool | true (`:55`) | `auto_posture` | inverted into `MOUSE_TRAIL_TRACER::SetMouseDisabled` at `pcbnew/router/pns_line_placer.cpp:1427` |
| `GetFixAllSegments()` | bool | true (`:56`) | `fix_all_segments` | `LINE_PLACER::FixRoute` `:1557`, `DIFF_PAIR_PLACER` `:822` |
| `GetRestrictAngles()` | bool | false (`:57`) | `restrict_angles` | dragger `pcbnew/router/pns_dragger.cpp:583`, host `pcbnew/router/router_tool.cpp:2339` |
| `WalkaroundHugLengthThreshold()` | double | 1.5 (`:54`) | `walkaround_hug_length_threshold` | `rhWalkBase` `pcbnew/router/pns_line_placer.cpp:590`, `:592` |
| `ViaForcePropIterationLimit()` | int | 40 (`:58`) | `via_force_prop_iteration_limit` | `buildInitialLine` `:2115`, dragger `pcbnew/router/pns_dragger.cpp:69` |

`PNS_MODE` (`pcbnew/router/pns_routing_settings.h:39`): `RM_MarkObstacles = 0`, `RM_Shove = 1`, `RM_Walkaround = 2`. `PNS_OPTIMIZATION_EFFORT` (`:47`): `OE_LOW = 0`, `OE_MEDIUM = 1`, `OE_FULL = 2`. In the line placer `OE_MEDIUM` and `OE_FULL` are treated identically (`pcbnew/router/pns_line_placer.cpp:753`, `:973`); only `SHOVE::runOptimizer` distinguishes them, and even there the difference is only the flag composition, both with `n_passes = 2` (`pcbnew/router/pns_shove.cpp:2051-2058`).

### 2.2 SIZES_SETTINGS

Declared `pcbnew/router/pns_sizes_settings.h:40`. Plain value type, copied by value into the placer (`LINE_PLACER::m_sizes`). Constructor defaults (`:43-59`), in internal units (nanometres):

| field | accessor | default | notes |
| --- | --- | --- | --- |
| clearance | `Clearance()` / `SetClearance` | 0 | working clearance from the current net to anything, before per-pair resolution |
| min clearance | `MinClearance()` | 0 | board absolute minimum; used by the diff pair gap gate `pcbnew/router/pns_router.cpp:229` |
| track width | `TrackWidth()` | 155000 | pushed into head and tail in `initPlacement` `pcbnew/router/pns_line_placer.cpp:1452` |
| track width is explicit | `TrackWidthIsExplicit()` | true | gates mid-route width changes, `pcbnew/router/pns_line_placer.cpp:2004` |
| board min track width | `BoardMinTrackWidth()` | 0 | the fallback probe width in `isStartingPointRoutable` `pcbnew/router/pns_router.cpp:324` |
| via type | `ViaType()` | `VIATYPE::THROUGH` | selects the layer span, see below |
| via diameter | `ViaDiameter()` | 600000 | `makeVia` `pcbnew/router/pns_line_placer.cpp:80` |
| via drill | `ViaDrill()` | 250000 | same |
| diff pair width | `DiffPairWidth()` | 125000 | diff pair placer |
| diff pair gap | `DiffPairGap()` | 180000 | |
| diff pair via gap | `DiffPairViaGap()` | 180000, but returns `DiffPairGap()` while `m_diffPairViaGapSameAsTraceGap` is true (default true) (`pcbnew/router/pns_sizes_settings.h:87`) | |
| hole to hole | `GetHoleToHole()` | 0 | |
| diff pair hole to hole | `GetDiffPairHoleToHole()` | 0 | |
| diff pair copper to hole | `GetDiffPairCopperToHole()` | 0 | |
| layer pairs | `AddLayerPair` / `PairedLayer` / `GetLayerTop` / `GetLayerBottom` | empty map, so top = `F_Cu`, bottom = `B_Cu` (`pcbnew/router/pns_sizes_settings.cpp:46`, `:55`) | `AddLayerPair` inserts both directions into the map (`:36`) |
| sources | `GetClearanceSource`, `GetWidthSource`, `GetDiffPairWidthSource`, `GetDiffPairGapSource` | empty strings | pure UI provenance strings, never read by the algorithms |

`EffectiveDiffPairViaGap()` (`pcbnew/router/pns_sizes_settings.h:146`) takes the max of the plain copper gap, `holeToHole - 2*annularRing`, and `copperToHole - annularRing`, with `annularRing = (ViaDiameter() - ViaDrill()) / 2`.

Via layer span is not stored on the settings; it is derived by `ROUTER_IFACE::GetViaLayerRange` (`pcbnew/router/pns_router.h:144`): a `THROUGH` via always spans `F_Cu..B_Cu`, everything else uses `GetLayerTop()..GetLayerBottom()`. `LINE_PLACER::makeVia` calls it (`pcbnew/router/pns_line_placer.cpp:78`).

---

## 3. LINE_PLACER

Declared `pcbnew/router/pns_line_placer.h:113`, implemented in `pcbnew/router/pns_line_placer.cpp` (2213 lines).

### 3.1 State

Fields, from `pcbnew/router/pns_line_placer.h:375-416`:

- `m_direction` (`:375`): the current routing direction, that is the direction the head is expected to leave the tail with. Updated by `handlePullback`, `handleSelfIntersections`, `reduceTail`, `mergeHead`, `optimizeTailHeadTransition`.
- `m_initial_direction` (`:376`): the direction for a fresh trace, set by `setInitialDirection` (`:95` in the cpp) which also writes `m_direction` when the tail is empty. Reset targets after pullback and self-intersection.
- `m_head` (`:378`): the volatile part of the trace, from `m_p_start` to the cursor. Recomputed from scratch every `routeStep`.
- `m_tail` (`:381`): the part already settled by collisions. Grows by `mergeHead`, shrinks by `reduceTail` / `handlePullback` / `handleSelfIntersections`. Note: "fixed" here means fixed *within this placement*, not committed to the node. Only `FixRoute` writes items into a `NODE`.
- `m_world` (`:384`): the placer's own branch of the router root.
- `m_p_start` (`:385`): the boundary between tail and head. `updatePStart` sets it to the tail's last point, or `m_currentStart` when the tail is empty (`:1102`).
- `m_fixStart` (`:386`): start point of the last fix, recorded into `FIXED_TAIL` stages.
- `m_last_p_end` (`:388`): `optional<VECTOR2I>` of the previous cursor position. Cleared in `initPlacement` (`:1457`), written at the end of `routeStep` (`:1216`), read only by the second via pushout attempt (`:2122`).
- `m_shove` (`:390`): owned `SHOVE`, constructed on a fresh branch in `initPlacement` (`:1478`).
- `m_currentNode` (`:392`), `m_lastNode` (`:393`): as described in section 1.2.
- `m_sizes` (`:396`): copy of the router sizes.
- `m_placingVia` (`:398`): user toggle, also cleared automatically at every intermediate fix (`:1724`).
- `m_currentNet` (`:400`), `m_currentLayer` (`:401`).
- `m_currentEnd` (`:403`), `m_currentStart` (`:404`), `m_currentTrace` (`:405`): `m_currentTrace` is a cache written by `Traces()` (`:1257`) and read by `FlipPosture` (`:1267`) and `UpdateSizes` (`:2009`).
- `m_startItem` (`:407`), `m_endItem` (`:408`): the anchors under the cursor. `m_endItem` is written at the top of `Move` (`:1496`) and read by `rhShoveOnly` to pin the springback node (`:938`).
- `m_idle` (`:410`): true before `initPlacement` and after a terminal fix. Gates `SetLayer` and `UpdateSizes`.
- `m_chainedPlacement` (`:411`): true after an intermediate fix that did not end with a via. Blocks layer switching mid-trace (`:1354`).
- `m_orthoMode` (`:412`): forces a single 90/45 segment.
- `m_placementCorrect` (`:413`): "something was successfully fixed", part of `HasPlacedAnything`.
- `m_fixedTail` (`:415`): the undo stack, see below.
- `m_mouseTrailTracer` (`:416`): the posture solver.

`FIXED_TAIL` (`pcbnew/router/pns_line_placer.h:48`) is a stack of `STAGE`, each holding a `NODE* commit` and a vector of `FIX_POINT { layer, placingVias, p, direction }`. For the single track placer each stage always holds exactly one point (`AddStage`, `pcbnew/router/pns_line_placer.cpp:2176`); the vector exists for the diff pair case. `PopStage` (`:2194`) copies the back stage out but *only pops if more than one stage remains* (`:2201`), so the initial stage is sticky and repeated undo eventually becomes idempotent at the session start.

### 3.2 Start

`LINE_PLACER::Start` (`pcbnew/router/pns_line_placer.cpp:1380`):

```
Start(p, startItem):
    m_placementCorrect = false
    m_currentStart = m_fixStart = m_currentEnd = p
    m_currentNet = startItem ? startItem->Net() : iface->GetOrphanedNetHandle()
    m_startItem = startItem;  m_placingVia = false;  m_chainedPlacement = false
    m_fixedTail.Clear();  m_endItem = nullptr
    setInitialDirection( Settings().InitialDirection() )      # N, or NE if start_diagonal
    initPlacement()
    initialDir = m_initial_direction
    lastSegDir = UNDEFINED
    if startItem is SEGMENT:
        seg = startItem.Seg()
        if p == seg.A: lastSegDir = DIRECTION_45(seg.Reversed())
        elif p == seg.B: lastSegDir = DIRECTION_45(seg)
        # landing in the middle of a segment leaves lastSegDir UNDEFINED on purpose,
        # so the posture solver is not biased (comment at :1402)
    elif startItem is SOLID whose parent is a PCB_PAD:
        angle = solid.GetOrientation().AsDegrees()
        initialDir = DIRECTION_45( int( (angle + 22.5) / 45.0 ) )
    mouseTrailTracer.Clear()
    mouseTrailTracer.AddTrailPoint(p)
    mouseTrailTracer.SetTolerance( m_head.Width() )
    mouseTrailTracer.SetDefaultDirections( m_initial_direction, UNDEFINED )
    mouseTrailTracer.SetMouseDisabled( !Settings().GetAutoPosture() )
    n = (Mode() == RM_Shove) ? m_shove->CurrentNode() : m_currentNode
    m_fixedTail.AddStage( m_fixStart, m_currentLayer, m_placingVia, m_direction, n )
    return true
```

Two things to note. First, `initialDir` computed from the pad orientation at `:1415-1417` is a *local variable that is then only used in a debug message* at `:1420`; it is never written back into `m_initial_direction`, and `SetDefaultDirections` at `:1426` passes `m_initial_direction`, not `initialDir`. So pad-orientation-derived posture is currently inert. Likewise `lastSegDir` is computed but the second argument to `SetDefaultDirections` is hard-coded `UNDEFINED`. Both look like regressions; reproduce the observable behaviour, not the apparent intent, and flag it.

Second, `Start` always returns `true`. The gate that can refuse a start is `ROUTER::isStartingPointRoutable`, not the placer.

`initPlacement` (`:1442`):

```
initPlacement():
    m_idle = false
    clear head and tail chains; set net, layer, width on both; remove vias from both
    m_last_p_end.reset()
    m_p_start = m_currentStart
    m_direction = m_initial_direction
    world = Router()->GetWorld()
    world->KillChildren()
    rootNode = world->Branch()
    SplitAdjacentSegments( rootNode, m_startItem, m_currentStart )
    setWorld( rootNode )
    m_lastNode = nullptr
    m_currentNode = m_world
    m_shove = make_unique<SHOVE>( m_world->Branch(), Router() )
```

`SplitAdjacentSegments` (`:1287`) is the "start in the middle of a track" mechanism: if there is no joint at `aP` already linked to at least one item, it clones the segment twice, sets the two halves' endpoints to `(A,p)` and `(p,B)`, removes the original and adds both halves with `aAllowRedundant = true`. `SplitAdjacentArcs` (`:1315`) does the same with `SHAPE_ARC::ConstructFromStartEndCenter`, preserving centre, winding and width. Both return false if the split point already has a joint, which is the "you clicked exactly on an existing endpoint" case.

Layer selection: `ROUTER::StartRouting` calls `m_placer->SetLayer( aLayer )` *before* `Start` (`pcbnew/router/pns_router.cpp:468`). At that moment `m_idle` is still true (constructor, `:50`), so `SetLayer` takes the trivial branch and just assigns `m_currentLayer` (`pcbnew/router/pns_line_placer.cpp:1349`).

### 3.3 SetLayer, ToggleVia, FlipPosture, SetOrthoMode

`SetLayer` (`:1347`) has three branches:

1. `m_idle`: assign and return true.
2. `m_chainedPlacement`: refuse (return false). This is what prevents a layer change in the middle of a chained trace with no via.
3. Otherwise, allowed only if there is no start item, or the start item is a via or a solid whose layer range overlaps the requested layer (`:1358-1360`). On success it resets the whole live state: `m_p_start = m_currentStart`, `m_direction = m_initial_direction`, clears the mouse trail, clears head and tail chains and their vias, sets the layer on head and tail, and re-runs `Move( m_currentEnd, nullptr )` so the preview is regenerated on the new layer.

`ToggleVia` (`:84`) just sets `m_placingVia` and removes the head via when disabling. It never adds one; the via is materialised inside `buildInitialLine` / `rhWalkOnly` / `rhShoveOnly`.

`FlipPosture` (`:1262`) first, when the posture is not already manually forced and a trace exists, copies the *actual* first segment direction of the current trace into the tracer as its default direction (`:1269-1271`, with the comment naming issue 12369), then calls `MOUSE_TRAIL_TRACER::FlipPosture`. Without that resync the flip would toggle relative to a stale internal direction.

`SetOrthoMode` (`:2032`) sets the flag only; the effect is entirely inside `buildInitialLine`.

### 3.4 Move

`LINE_PLACER::Move` (`:1482`):

```
Move(p, endItem):
    eiDepth = endItem && endItem->Owner() ? endItem->Owner()->Depth() : -1
    delete m_lastNode; m_lastNode = nullptr
    m_endItem = endItem
    reachesEnd = route(p)                                  # routeStep + "did the head reach p"
    if m_placingVia and p == m_p_start and !m_head.EndsWithVia():
        v = makeVia(p); v.SetNet(m_currentNet); m_head.AppendVia(v)   # :1505, see below
    current = Trace()
    splitPoint = current.PointCount() ? current.CLastPoint() : m_p_start
    if reachesEnd and endItem is SEGMENT and current has segments:
        if last trace segment is collinear with and overlaps the target segment:
            splitPoint = targetSeg.NearestPoint(lastSeg.A)
            rewrite the last point of both `current` and m_head to splitPoint
    m_currentEnd = current.PointCount() ? splitPoint : m_p_start
    m_lastNode = m_currentNode->Branch()
    if reachesEnd and eiDepth >= 0 and endItem and m_currentNode->Depth() >= eiDepth and current has segments:
        if endItem->Net() == m_currentNet: SplitAdjacentSegments(m_lastNode, endItem, splitPoint)
        if Settings().RemoveLoops(): removeLoops(m_lastNode, current)
    updateLeadingRatLine()
    mouseTrailTracer.AddTrailPoint(p)
    return true
```

The zero-length via fallback at `:1505` is documented in the comment at `:1500`: when the user presses V without moving the mouse, `buildInitialLine`'s pushout has a zero lead vector and cannot resolve, the head then carries no via, and the subsequent commit would silently drop it.

The `eiDepth` guard at `:1537` prevents splitting into a node that is shallower than the node owning the end item, that is, prevents corrupting a node the end item does not exist in.

`Move` always returns true. The "did we reach the cursor" answer lives in `reachesEnd` and is only used internally; the host learns about it indirectly through `CurrentEnd()`.

`route` (`:1222`) is a one-liner wrapper: `routeStep(aP)`, then false if the head is empty, else `m_head.CLastPoint() == aP`. The docstring promises repetition due to mouse smoothing, but the current implementation calls `routeStep` exactly once.

`updateLeadingRatLine` (`:2021`) builds `Trace()`, asks `TOPOLOGY( m_lastNode ).LeadingRatLine`, and pushes the result to `iface->DisplayRatline`.

### 3.5 routeStep, the main loop

`routeStep` (`:1110`):

```
routeStep(p):
    fail = false; go_back = false; n_iter = 1
    for i in 0 .. n_iter-1:
        prevTail = m_tail; prevHead = m_head
        if !go_back and Settings().FollowMouse(): reduceTail(p)
        go_back = false
        updatePStart(m_tail)
        if !routeHead(p, newHead, newTail):
            restore m_tail/m_head from prev
            if m_tail is empty: append p_start twice so the user sees a zero-length line
            fail = true
        updatePStart(m_tail)
        if fail: break
        m_head = newHead; m_tail = newTail
        if handleSelfIntersections(): n_iter++; go_back = true
        if !go_back and handlePullback(): n_iter++; m_head.Clear(); go_back = true
    if !fail and Settings().FollowMouse():
        if !optimizeTailHeadTransition(): mergeHead()
    m_last_p_end = p
```

The `n_iter++` pattern means the loop runs one extra pass every time the tail was mutated by self-intersection handling or pullback, so the head can be recomputed against the shortened tail. There is no absolute bound on `n_iter` other than the fact that each mutation strictly shortens the tail.

The zero-length-line fallback at `:1152` is intentional user feedback, not a geometric result; the comment says it gets pruned later. `Trace()` deliberately does not `Simplify()` a two-point chain for this reason (`:1238`).

### 3.6 routeHead and the three mode routines

`routeHead` (`:1020`) is a pure dispatch on `Settings().Mode()`: `RM_MarkObstacles -> rhMarkObstacles`, `RM_Walkaround -> rhWalkOnly`, `RM_Shove -> rhShoveOnly`.

**rhMarkObstacles** (`:808`):

```
rhMarkObstacles(p, newHead, newTail):
    buildInitialLine(p, m_head, RM_MarkObstacles)
    m_head.SetBlockingObstacle(nullptr)
    obs = m_currentNode->NearestObstacle(&m_head)
    if obs:
        clearance = m_currentNode->GetClearance(obs->m_item, &m_head, false)
        hull = ruleResolver->HullCache(obs->m_item, clearance, m_head.Width(), m_head.Layer())
        nearest = (cornerMode is 90-degree) ? hull.BBox().NearestPoint(p) : hull.NearestPoint(p)
        if |nearest - p| < m_head.Width()/2:
            buildInitialLine(nearest, m_head, RM_MarkObstacles)
    newHead = m_head; newTail = m_tail
    return true
```

The snap-to-hull behaviour at `:815` is described in the comment as letting the user route as tightly as possible without turning on shove or walkaround. It never fails, so mark-obstacles mode never gets "stuck". The `#if 0` block at `:840` is a sketch of a "stop at first obstacle" mode that does not exist.

**rhWalkBase** (`:549`) is the shared walkaround driver used by both walkaround and shove modes. It takes a collision mask (`ITEM::ANY_T` for walk, `ITEM::SOLID_T` for shove) and a `PNS_MODE` used only to steer `buildInitialLine`'s via handling.

```
rhWalkBase(p, out walkLine, collisionMask, mode, out viaOk):
    walkFull = m_head (copy);  l1 = m_head (copy)
    walkP = p
    walkaround = WALKAROUND(m_currentNode, Router())
    walkaround.SetSolidsOnly(false)          # immediately overridden by SetItemMask
    walkaround.SetIterationLimit(Settings().WalkaroundIterationLimit())
    walkaround.SetItemMask(collisionMask)
    walkaround.SetAllowedPolicies({ WP_CCW, WP_CW })
    round = 0
    do:
        l1.Clear()
        round++
        viaOk = buildInitialLine(walkP, l1, mode, /*aForceNoVia=*/ round == 0)   # always false, see note
        initTrack = m_tail ++ l1;  initTrack.Simplify()
        initialLength = initTrack.Length()
        hugThresholdLength         = initialLength * WalkaroundHugLengthThreshold()
        hugThresholdLengthComplete = 2.0 * initialLength * WalkaroundHugLengthThreshold()
        wr = walkaround.Route(initTrack)
        len_cw  = (status[CW]  != ST_STUCK) ? len(lines[CW])  : INT_MAX
        len_ccw = (status[CCW] != ST_STUCK) ? len(lines[CCW]) : INT_MAX
        if status[CW] == ST_DONE:
            OPTIMIZER::Optimize(lines[CW], MERGE_SEGMENTS, m_currentNode)
            if splitHeadTail(lines[CW], m_tail, tmpHead, tmpTail):
                optimizer(MERGE_SEGMENTS, mask=collisionMask).Optimize(tmpHead)
                lines[CW] = tmpTail ++ tmpHead
            len_cw = len(lines[CW]);  bestLine = lines[CW]
        if status[CCW] == ST_DONE:
            same treatment
            if len_ccw < len_cw: bestLine = lines[CCW]
        bestLength = min(len_cw, len_ccw)
        if bestLength < hugThresholdLengthComplete and bestLine:
            walkFull = bestLine;  walkP = walkFull.CLastPoint();  continue   # to the do-while test
        # "hug" fallback: clip each candidate to the point nearest the cursor
        if status[CW]  != ST_STUCK: validCw  = cursorDistMinimum(lines[CW],  p, hugThresholdLength, l_cw)
        if status[CCW] != ST_STUCK: validCcw = cursorDistMinimum(lines[CCW], p, hugThresholdLength, l_ccw)
        distCw  = validCw  ? |p - l_cw.Last()|  : INT_MAX
        distCcw = validCcw ? |p - l_ccw.Last()| : INT_MAX
        if distCw < distCcw and validCw:  walkFull = l_cw;  walkP = l_cw.Last()
        elif validCcw:                    walkFull = l_ccw; walkP = l_ccw.Last()
        else: return false
    while round < 2 and m_placingVia
    if l1.EndsWithVia():
        v = l1.Via();  v.SetPos(walkFull.CLastPoint());  walkFull.AppendVia(v)
    walkLine = walkFull
    return !walkFull.EndsWithVia() or viaOk
```

Three details worth carrying over exactly:

- `round++` happens *before* the `round == 0` test at `:578`, so `aForceNoVia` is always false. The apparent intent was to skip via placement on the first round. Reproducing the intent would change behaviour; reproduce the code.
- The loop runs twice only when placing a via; the second pass re-runs the walkaround from the point the first pass reached, which is how the via gets pushed out of the way and the line re-hugged around it.
- The complete-path acceptance uses *twice* the hug threshold, so a full walkaround up to 3x the direct length is preferred over a partial hug at the default 1.5.

**cursorDistMinimum** (`:429`) picks the point on a candidate walk line that is closest to the cursor, then clips there:

```
cursorDistMinimum(L, cursor, lengthThreshold, out result):
    build parallel arrays dists[], pts[]:
        for each segment s of L:
            push |cursor - s.A|, s.A
            pn = s.NearestPoint(cursor)
            if pn != s.A and pn != s.B: push |pn - cursor|, pn
            accumulate segment length; if accumulated > lengthThreshold: lastP = s.B; break
        push |cursor - lastP|, lastP
    minPGlob = argmin(dists)
    minPLoc  = first local minimum (dists[i] > dists[i+1] < dists[i+2]) over i in 0..len-4,
               with a tail special case at :500
    minPLoc = -1            # :515, hard override: the local-minimum path is disabled
    preferred = minPGlob
    thresholdDist = 0
    if clipAndCheckCollisions(pts[preferred], L, result, thresholdDist): return true
    thresholdDist = 0
    ok = false
    for every candidate pts[i]:   ok |= clipAndCheckCollisions(pts[i], L, result, thresholdDist)
    return ok
```

The `minPLoc = -1` at `:515` with the comment "I didn't make my mind yet if local or global minimum feels better" is live code: the local-minimum computation above it is dead. Keep the global minimum.

The fallback loop keeps the *longest* non-colliding prefix, because `clipAndCheckCollisions` raises `thresholdDist` on every success and rejects anything shorter.

**clipAndCheckCollisions** (`:394`):

```
clipAndCheckCollisions(p, L, out result, inout thresholdDist):
    l = L; idx = l.Split(p)
    if idx < 0: return false
    l2 = l.Slice(0, idx);  dist = l2.Length()
    if dist < thresholdDist: rv = false
    ctest = LINE(m_head, l2)
    if m_currentNode->CheckColliding(&ctest): rv = false
    if rv: result = move(l2); thresholdDist = dist
    return rv
```

**splitHeadTail** (`:860`) reconstructs the head/tail boundary after a walk or shove produced a whole new line. The long comment at `:775` explains why the walk operates on tail+head rather than head alone: with the clearance epsilon in place, a head computed alone can be non-colliding yet have its first point inside a later hull, which breaks the walkaround precondition (`LINE::Walkaround` returns false if the first point is strictly inside the hull, `pcbnew/router/pns_line.cpp:308-315`).

```
splitHeadTail(newLine, oldTail, out newHead, out newTail):
    newTail = oldTail with via removed;  newHead = empty (but inherits oldTail's attributes)
    l2 = newLine
    if l2.PointCount() > 1 and oldTail.PointCount() > 1:
        if l2 has oldTail.CLastPoint() on an edge: l2.Split(oldTail.CLastPoint())
        i = first index into oldTail whose point is NOT present in l2   (:881)
        if no such index: i--                                          (:891)
        clamp i to l2.PointCount()-1
        newTail = (i == 0) ? empty : l2.Slice(0, i)
        newHead = l2.Slice(i, -1)
    else:
        newTail = empty;  newHead = l2
    return true            # always
```

The function never returns false, so the `if( !splitHeadTail(...) ) return false;` guards in the callers are unreachable.

**rhWalkOnly** (`:737`):

```
rhWalkOnly(p, out newHead, out newTail):
    if !rhWalkBase(p, walkFull, ITEM::ANY_T, RM_Walkaround, viaOk): return false
    effort = (OptimizerEffort == OE_LOW) ? 0 : OPTIMIZER::MERGE_SEGMENTS
    if SmartPads() and cornerMode in {MITERED_45, ROUNDED_45} and !tracer.IsManuallyForced():
        effort |= OPTIMIZER::SMART_PADS
    if m_currentNode->CheckColliding(&walkFull): return false
    if !splitHeadTail(walkFull, m_tail, newHead, newTail): return false
    if m_placingVia and viaOk: newHead.AppendVia( makeVia(newHead.CLastPoint()) )
    OPTIMIZER::Optimize(&newHead, effort, m_currentNode)
    return true
```

Smart pads is suppressed in 90-degree corner modes ("incompatible with 90-degree mode for now", `:761`) and while the posture is manually forced.

**rhShoveOnly** (`:921`):

```
rhShoveOnly(p, out newHead, out newTail):
    if !rhWalkBase(p, walkSolids, ITEM::SOLID_T, RM_Shove, viaOk): return false
    m_currentNode = m_shove->CurrentNode()
    m_shove->SetLogger / SetDebugDecorator
    if m_endItem: m_shove->SetSpringbackDoNotTouchNode( m_endItem->Owner() )
    else:         m_shove->SetSpringbackDoNotTouchNode( nullptr )
    newHead = walkSolids
    if m_placingVia and viaOk: newHead.AppendVia( makeVia(newHead.CLastPoint()) )
    m_shove->ClearHeads()
    m_shove->AddHeads(newHead, SHOVE::SHP_SHOVE)
    shoveOk = (m_shove->Run() == SHOVE::SH_OK)
    m_currentNode = m_shove->CurrentNode()
    effort = same computation as rhWalkOnly
    if shoveOk:
        if m_shove->HeadsModified(): newHead = m_shove->GetModifiedHead(0)
        splitHeadTail(newHead, m_tail, aNewHead, aNewTail)
        if newHead.EndsWithVia(): aNewHead.AppendVia(newHead.Via())
        OPTIMIZER::Optimize(&aNewHead, effort, m_currentNode)
        return true
    else:
        return rhWalkOnly(p, aNewHead, aNewTail)     # fall back to pure walkaround
```

So shove mode is "walk around solids first, then shove everything else, and fall back to full walkaround if the shove fails". The `SetSpringbackDoNotTouchNode` calls pin the node owning the item under the cursor so springback cannot delete it while it is being referenced (`:937`, and the else branch at `:942` explicitly unpins when the cursor leaves).

### 3.7 buildInitialLine and posture

`buildInitialLine` (`:2038`) is where posture becomes geometry:

```
buildInitialLine(p, out head, mode, forceNoVia = false):
    guessedDir = m_mouseTrailTracer.GetPosture(p)
    cornerMode = Settings().GetCornerMode()
    if m_orthoMode: cornerMode = MITERED_45        # rounded corners make no sense orthogonally
    if tracer.IsManuallyForced():
        # deterministic posture switching: drop a one-segment tail rather than infer from it
        if m_tail.SegmentCount() == 1 and m_head.SegmentCount() > 0:
            if DIRECTION_45(tail[0]) == DIRECTION_45(head[0]) or m_head.SegmentCount() == 1:
                m_p_start = m_tail.CPoint(0);  m_tail.Clear()
    if m_p_start == p:
        l = empty
    else:
        if GetFreeAngleMode() and Mode() == RM_MarkObstacles:
            l = [ m_p_start, p ]                    # single free-angle segment
        elif m_tail is empty:
            l = guessedDir.BuildInitialTrace(m_p_start, p, false, cornerMode)
        else:
            l = m_direction.BuildInitialTrace(m_p_start, p, false, cornerMode)
        if l.SegmentCount() > 1 and m_orthoMode:
            newLast = l.CSegment(0).LineProject(l.CLastPoint())
            l.Remove(-1,-1);  l.SetPoint(1, newLast)   # collapse to a single ortho segment
    head.SetLayer(m_currentLayer);  head.SetShape(l)
    if !m_placingVia or forceNoVia: return true
    v = makeVia(p);  v.SetNet(head.Net())
    if mode == RM_MarkObstacles: head.AppendVia(v); return true
    collMask = (mode == RM_Walkaround) ? ITEM::ANY_T : ITEM::SOLID_T
    iterLimit = Settings().ViaForcePropIterationLimit()          # 40
    for attempt in 0,1:
        lead = p - m_p_start
        if attempt == 1 and m_last_p_end: lead = p - m_last_p_end
        if v.PushoutForce(m_currentNode, lead, force, collMask, iterLimit):
            line = guessedDir.BuildInitialTrace(m_p_start, p + force, false, cornerMode)
            head = LINE(head, line)
            v.SetPos(v.Pos() + force)
            head.AppendVia(v)
            return true
    return false     # via placement unsuccessful
```

Note the asymmetry: the plain trace uses `m_direction` when a tail exists but `guessedDir` when it does not, while the via-corrected retrace at `:2127` always uses `guessedDir`.

`DIRECTION_45::BuildInitialTrace` (`libs/kimath/src/geometry/direction_45.cpp:24`) is the geometric primitive. Behaviour:

- `startDiagonal` comes from the direction's own `IsDiagonal()` unless the direction is `UNDEFINED`, in which case the caller's flag is used (`:31-34`).
- Degenerate shortcut: `w == 0 || h == 0 || (!is90mode && h == w)` emits a single segment (`:46`).
- 45-degree modes split the delta into a straight leg `mp0` and a diagonal leg `mp1` of length `min(w,h)`, and compute `tangentLength = |longLeg| - |mp1|` (`:69-80`).
- `MITERED_45` emits `aP0, aP0 + (startDiagonal ? mp1 : mp0), aP1` (`:100`).
- `ROUNDED_45` replaces the corner with a `SHAPE_ARC` of radius `diagLength / (2*cos(67.5 deg))` where `diagLength = sqrt(2*diag2 - 2*diag2*cos(3*pi/4))` (`:130-132`), with four cases depending on `startDiagonal` and the sign of `tangentLength`. The negative-tangent, straight-start case constructs the arc from a centre and then snaps the endpoint back onto the axis when it is within `SHAPE_ARC::MIN_PRECISION_IU` (`:202`, `:207`).
- `MITERED_90` emits `aP0, aP0 + mp0, aP1` where `mp0` is horizontal when `startDiagonal == (h >= w)` (`:58-65`).
- `ROUNDED_90` builds a quarter arc of radius `min(w,h)` either before or after the straight leg depending on `startDiagonal` (`:265-306`).
- Every branch except the 90-degree `w == h` early return finishes with `pl.Simplify()` (`:310`).

The direction quantisation itself (`DIRECTION_45::construct_`, `libs/kimath/include/geometry/direction45.h:317`) converts a vector to an angle in degrees measured with north up (y is negated at every construction site, `:97`, `:107`, `:120`), then `dir = (mag + 22.5) / 45.0`, that is nearest-octant rounding with a 22.5 degree half-window. `Angle()` (`:181`) classifies by index distance: 1 or 7 obtuse, 2 or 6 right, 3 or 5 acute, 4 half-full, 0 straight, and undefined if either side is undefined.

### 3.8 Tail management primitives

**reduceTail** (`:256`). Called at the top of each `routeStep` iteration when following the mouse. Walks the tail from the last segment backwards; for each tail segment `s` it builds a hypothetical replacement `dir.BuildInitialTrace(s.A, aEnd)`. It **breaks** out of the scan on the first colliding replacement (`:291-292`), and records a candidate only when the replacement's first segment has the same direction as the segment it replaces (`:294`). The last recorded candidate (the earliest non-colliding one, since the loop keeps going backwards) wins: the tail is truncated after `reduce_index` and the head is cleared. Requires at least 2 tail segments and 1 head segment.

Note the dead statement at `:305`: `reducedLine` is built and never used.

**handlePullback** (`:173`). Compares the first head direction against the last tail direction; if the angle is `ANG_RIGHT` or `ANG_ACUTE` (`:218`), the last tail shape is removed (`RemoveShape(-1)`), `m_direction` is set from that removed segment or arc, and if the tail becomes empty `m_direction` falls back to `m_initial_direction`. `pullback_1`, the "direction mismatch" case, is hard-disabled at `:213`. Special cases: a 1-point tail is simply cleared (`:189`); a head with fewer than 2 points is a no-op.

**handleSelfIntersections** (`:104`). Three outcomes:

1. `tail[0] == head[0]`: the head is a completely new trace, so clear the tail and reset `m_direction` to `m_initial_direction` (`:118`).
2. Intersections exist and the earliest one is at index `n < 2`: clear both tail and head, reset direction (`:151`).
3. Otherwise: `m_direction = DIRECTION_45( tail.CSegment(n-1) )` and `tail.Remove(n, -1)` (`:163`).

Intersections exactly at `head[0]` or `tail.last` are ignored as the normal junction (`:146`).

**mergeHead** (`:320`). Moves the whole head into the tail when it is safe:

```
mergeHead():
    ForbiddenAngles = ANG_ACUTE | ANG_HALF_FULL | ANG_UNDEFINED
    head.Simplify(); tail.Simplify()
    if head.ShapeCount() < 3: return false
    if tail non-empty and head[0] != tail.last: return false          # discontinuous
    if m_head.CountCorners(ForbiddenAngles) != 0: return false
    dir_head = direction of head shape 0 (segment or arc)
    if tail non-empty:
        dir_tail = direction of the last tail shape
        if dir_head.Angle(dir_tail) & ForbiddenAngles: return false
    tail.Append(head); tail.Simplify()
    m_direction = direction of the new last tail shape
    head.Remove(0,-1)
    return true
```

The `n_head < 3` gate at `:335` is the reason the head normally keeps at least two shapes live under the cursor: only a head that has grown to three or more shapes is considered "established".

**optimizeTailHeadTransition** (`:1038`). Tried before `mergeHead`; if it succeeds, `mergeHead` is skipped.

```
optimizeTailHeadTransition():
    linetmp = Trace()
    if !tracer.IsManuallyForced() and OPTIMIZER::Optimize(&linetmp, FANOUT_CLEANUP, m_currentNode):
        if linetmp.SegmentCount() < 1: return false
        m_head = linetmp;  m_direction = DIRECTION_45(linetmp.CSegment(0));  m_tail.Clear()
        return true
    tailLookbackSegments = 3
    threshold = min(tail.PointCount(), tailLookbackSegments + 1)      # <= 4
    if tail.ShapeCount() < 3: return false
    opt_line = tail.Slice(-threshold, -1)
    end = min(2, head.PointCount() - 1)
    opt_line.Append( head.Slice(0, end) )
    new_head = LINE(m_tail, opt_line)
    if OPTIMIZER::Optimize(&new_head, MERGE_SEGMENTS, m_currentNode):
        head.Clear()
        tail.Replace(-threshold, -1, new_head.CLine())
        tail.Simplify()
        m_direction = DIRECTION_45(new_head.CSegment(-1))
        return true
    return false
```

The FANOUT_CLEANUP path can override the user's posture, which is why it is skipped when the posture is manually forced (comment at `:1044`). The dead local `tmp` at `:1088` is never used.

### 3.9 Trace and Traces

`Trace()` (`:1233`) concatenates tail and head into a new chain and simplifies it only when it has more than two points (`:1241`), for the zero-length-feedback reason given above. The returned `LINE` inherits its attributes from `m_head`. `Traces()` (`:1255`) caches the result into `m_currentTrace` and wraps it in a one-element `ITEM_SET`, so the returned set points at placer-owned storage that is invalidated by the next call.

### 3.10 FixRoute

`FixRoute` (`:1555`) is the only place where geometry becomes items in a node.

```
FixRoute(p, endItem, forceFinish):
    fixAll = Settings().GetFixAllSegments()
    realEnd = false
    pl = Trace()

    if Mode() == RM_MarkObstacles and endItem:
        # net adoption: whichever side is netless takes the other's net
        if netcode(m_currentNet) <= 0: m_currentNet = endItem->Net(); pl.SetNet(m_currentNet)
        elif netcode(endItem->Net()) <= 0: endItem->SetNet(m_currentNet)

    if !Settings().AllowDRCViolations():
        checkNode = (Mode() == RM_Shove) ? m_shove->CurrentNode() : m_world
        obs = checkNode->CheckColliding(&pl)
        if obs and (Mode() != RM_Shove or obs->m_item is SOLID): return false
        # the shove-mode relaxation is a documented workaround (:1594): the shove node
        # sometimes reports collisions against objects it itself shoved

    l = pl.CLine()
    if l has no segments:                                  # via-only commit
        if m_lastNode: simplify the last added segment if there is one
        if !pl.EndsWithVia(): return false
        m_lastNode->Add( clone of pl.Via() with ResetUid() )
        m_shove->AddLockedSpringbackNode(m_lastNode)
        m_currentNode = nullptr;  m_idle = true;  m_placementCorrect = true
        return true

    p_last     = l.CLastPoint()
    p_pre_last = (l.PointCount() > 2) ? l.CPoints()[n-2] : p_last
    realEnd = (endItem and m_currentNet and m_currentNet == endItem->Net()) or forceFinish
    if !fixAll and l.ArcCount(): fixAll = true             # rollback is broken for arcs (:1648)
    lastDirSeg = (!fixAll and l.SegmentCount() > 1) ? l.CSegment(-2) : l.CSegment(-1)
    d_last = DIRECTION_45(lastDirSeg)
    lastV = (realEnd or m_placingVia or fixAll) ? l.SegmentCount()
                                                : max(1, l.SegmentCount() - 1)
    lastArc = -1
    for i in 0 .. lastV-1:
        arcIndex = l.ArcIndex(i)
        if arcIndex < 0 or (lastArc >= 0 and i == lastV - 1 and !l.IsPtOnArc(lastV)):
            add SEGMENT(pl.CSegment(i), net, width = pl.Width(), layer = m_currentLayer)
        else:
            if arcIndex == lastArc: continue           # this arc was already emitted
            add ARC(l.Arc(arcIndex), net, width, layer);  lastArc = arcIndex
        lastItem = the item just added, or null if Add() refused it
    if pl.EndsWithVia(): m_lastNode->Add( clone of pl.Via() with ResetUid() )
    if lastItem: simplifyNewLine(m_lastNode, lastItem)

    if !realEnd:                                            # intermediate click
        setInitialDirection(d_last)
        m_currentStart = (m_placingVia or fixAll) ? p_last : p_pre_last
        m_fixedTail.AddStage(m_fixStart, m_currentLayer, m_placingVia, m_direction, m_currentNode)
        m_fixStart = m_currentStart
        m_startItem = nullptr
        m_placingVia = false
        m_chainedPlacement = !pl.EndsWithVia()
        m_p_start = m_currentStart;  m_direction = m_initial_direction
        clear head and tail chains and their vias
        m_currentNode = m_lastNode;  m_lastNode = m_lastNode->Branch()
        m_shove->AddLockedSpringbackNode(m_currentNode)
        lastSegDir = pl.EndsWithVia() ? UNDEFINED : d_last
        tracer.Clear(); tracer.SetTolerance(m_head.Width())
        tracer.AddTrailPoint(m_currentStart)
        tracer.SetDefaultDirections(lastSegDir, lastSegDir)
        m_placementCorrect = true
    else:
        m_shove->AddLockedSpringbackNode(m_lastNode)
        m_placementCorrect = true;  m_idle = true
    return realEnd
```

The "fix all segments" behaviour is entirely in the two expressions at `:1654` and `:1659`: with `fixAll` off and no via and no real end, the last segment is left unfixed (`lastV = SegmentCount() - 1`) and the new start becomes `p_pre_last`, so the final leg stays rubber-banded. With `fixAll` on, everything is emitted and the new start is `p_last`.

Via fixing: the via is cloned with `ResetUid()` (`:1706`, and `:1624` for the via-only case) so the committed via gets a fresh identity rather than aliasing the head's temporary via. After fixing a via, `m_placingVia` is cleared (`:1724`) and `m_chainedPlacement` stays false, which is what re-enables layer switching for the next leg.

The return value is `realEnd`, which the host uses to decide whether to end the interactive loop (`pcbnew/router/router_tool.cpp:1934`).

**removeLoops** (`:1828`), called from `Move` not from `FixRoute`:

```
removeLoops(node, latest):
    if latest has no segments or is a closed loop itself: return
    latest.ClearLinks();  node->Add(latest, allowRedundant = true)
    for each link seg of latest:
        ourLine = node->AssembleLine(seg)
        node->FindLineEnds(ourLine, a, b)
        if a == b: node->FindLineEnds(latest, a, b)
        node->FindLinesBetweenJoints(a, b, lines)
        for each line in lines:
            if line does not contain seg and has segments and none of its links is locked:
                mark all of its links for erasure
    erase everything marked;  node->Remove(latest)
```

The locked-track guard at `:1864` is what keeps a locked parallel route from being silently deleted as a redundant loop.

**simplifyNewLine** (`:1894`) runs in two phases. Phase one walks every added segment in the node and, for each of its joints that is *not* a line corner, looks for a neighbour of the same width and overlapping layers whose segment is contained in (or contains) the reference segment, with the far joint having link count 1; such a stub is queued for removal (`:1906-1957`). The comment at `:1898` explains this: collinear segments on non-corner joints block line assembly and the optimizer cannot clean them up. Phase two assembles the line from `aLatest` with `AssembleLine( ..., false, false, false )`, runs `OPTIMIZER::MERGE_COLINEAR`, additionally `Simplify()`s the chain, and replaces the line in the node if either step changed anything (`:1976-1991`).

### 3.11 UnfixRoute, CommitPlacement, HasPlacedAnything

`UnfixRoute` (`:1759`):

```
UnfixRoute() -> optional<VECTOR2I>:
    if !m_fixedTail.PopStage(st): return nullopt
    ret = m_head has points ? m_head.CPoint(0) : nullopt
    clear head and tail chains; m_startItem = nullptr
    m_p_start = m_fixStart = m_currentStart = st.pts[0].p
    m_direction   = st.pts[0].direction
    m_placingVia  = st.pts[0].placingVias
    m_currentLayer = st.pts[0].layer
    m_currentNode = st.commit
    set head/tail layer, remove their vias
    tracer.Clear(); tracer.SetDefaultDirections(m_initial_direction, m_direction)
    tracer.AddTrailPoint(m_p_start)
    m_shove->RewindSpringbackTo(m_currentNode)
    m_shove->UnlockSpringbackNode(m_currentNode)
    if Mode() == RM_Shove:
        m_currentNode = m_shove->CurrentNode();  m_currentNode->KillChildren()
    m_lastNode = m_currentNode->Branch()
    return ret
```

The returned point is the first head point *before* the undo, which the host uses to place the cursor where the undone segment started.

`CommitPlacement` (`:1810`): in shove mode it first rewinds the shove to the last locked node and adopts it as `m_lastNode`, killing its children; then commits `m_lastNode` through the router and nulls both node pointers.

`HasPlacedAnything` (`:1804`) is `m_placementCorrect || m_fixedTail.StageCount() > 1`. Since `Start` always pushes one stage, the `> 1` is "at least one fix happened".

`UpdateSizes` (`:1995`) refuses to change the width mid-route unless the width is explicit or nothing has been placed yet and the start item is not a segment (`:2004`), the point being that continuing an existing track should inherit its width and that ripping up already-fixed geometry to widen it is not acceptable.

---

## 4. MOUSE_TRAIL_TRACER: the posture solver

Declared `pcbnew/router/pns_mouse_trail_tracer.h:32`, implemented in `pcbnew/router/pns_mouse_trail_tracer.cpp`.

State: `m_trail` (a `SHAPE_LINE_CHAIN` of recent cursor positions), `m_tolerance` (set to the head width by the placer), `m_direction` (the current answer), `m_lastSegDirection` (the direction of the previously fixed segment, or UNDEFINED), `m_forced` (solution locked), `m_disableMouse` (mouse heuristic off), `m_manuallyForced` (user pressed the posture key).

`Clear()` (`:39`) resets `m_forced` and `m_manuallyForced` and empties the trail, but deliberately leaves `m_direction`, `m_lastSegDirection` and `m_tolerance` alone, which is why callers always follow `Clear()` with `SetTolerance` and `SetDefaultDirections`.

`AddTrailPoint` (`:47`) appends and self-prunes: when the trail has more than two segments, the new segment is tested against every segment except the last two, and on the first one whose squared distance is within `m_tolerance^2` the trail is sliced back to that index (`:59-69`). This is the "the user came back over their own path" detector. The trail is `Simplify()`d after every append.

`GetPosture(p)` (`:84`) returns a `DIRECTION_45`. Tuning constants, all local to the function:

| constant | value | line | meaning |
| --- | --- | --- | --- |
| `areaRatioThreshold` | 1.3 | `:87` | how much better the fit must be to switch posture |
| `areaRatioEpsilon` | 0.25 | `:90` | hysteresis band around the threshold |
| `minAreaCutoffDistanceFactor` | 6 | `:93` | trail must be longer than 6 * tolerance before the area test is trusted |
| `lockDistanceFactor` | 30 | `:96` | beyond 30 * tolerance from p0 the solution is locked |
| `unlockDistanceFactor` | 10 | `:99` | back within 10 * tolerance of p0 the lock is released and the trail restarted |

Algorithm:

```
GetPosture(p):
    if trail has < 2 points or m_manuallyForced:
        if !m_manuallyForced and m_lastSegDirection != UNDEFINED:
            m_direction = m_disableMouse ? m_lastSegDirection.Right() : m_lastSegDirection
        return m_direction

    p0 = trail[0];  refLength = |p - p0|
    straight = DIRECTION_45().BuildInitialTrace(p0, p, false)   # UNDEFINED dir, straight first
    straight.SetClosed(true); straight.Append(trail.Reverse()); straight.Simplify()
    areaS = straight.Area()
    diag = DIRECTION_45().BuildInitialTrace(p0, p, true)        # diagonal first
    diag.Append(trail.Reverse()); diag.SetClosed(true); diag.Simplify()
    areaDiag = diag.Area()
    ratio = areaS / (areaDiag + 1.0)

    if m_forced and refLength < unlockDistanceFactor * tolerance:
        m_forced = false;  restart the trail at p0                      # unlock

    areaOk = false
    if !m_forced and refLength > minAreaCutoffDistanceFactor * tolerance:
        areaCutoff = tolerance * refLength
        closed copy of the trail; areaOk = (trail.Area() > areaCutoff)

    straightDirection = DIRECTION_45(straight.CSegment(0))
    diagDirection     = DIRECTION_45(diag.CSegment(0))
    if      !m_forced and areaOk and ratio > areaRatioThreshold + areaRatioEpsilon:  new = diagDirection
    else if !m_forced and areaOk and ratio < 1/areaRatioThreshold - areaRatioEpsilon: new = straightDirection
    else:   new = m_direction.IsDiagonal() ? diagDirection : straightDirection
    if !m_disableMouse and new != m_direction: m_direction = new

    if !m_manuallyForced and !m_disableMouse and m_lastSegDirection != UNDEFINED:
        if   straightDirection == m_lastSegDirection: m_direction = straightDirection
        elif diagDirection     == m_lastSegDirection: m_direction = diagDirection
        else switch m_direction.Angle(m_lastSegDirection):
            ANG_HALF_FULL: m_direction = m_direction.IsDiagonal() ? straightDirection : diagDirection
            ANG_ACUTE:     candidate = the other one; take it if candidate.Angle(lastSeg) == ANG_RIGHT
            ANG_RIGHT:     candidate = the other one; take it if candidate.Angle(lastSeg) == ANG_OBTUSE
    if !m_forced and refLength > lockDistanceFactor * tolerance:
        m_forced = true                                                  # lock
    return m_direction
```

The area test is the core: it closes the polygon formed by the candidate two-segment trace and the reversed mouse trail, and compares the enclosed areas. A large `areaS / areaDiag` ratio means the straight-first candidate deviates far from what the user actually drew, so the diagonal-first posture is chosen. The `+1.0` in the denominator is there purely to avoid a division by zero.

`m_disableMouse` (from `auto_posture = false`) has two effects: the mouse-derived direction is computed but never assigned (`:176`), and the previous-segment fallback returns `m_lastSegDirection.Right()` instead of `m_lastSegDirection` (`:107`), that is, "switch posture every segment".

`FlipPosture` (`:271`) is `m_direction = m_direction.Right(); m_forced = true; m_manuallyForced = true;`. `Right()` on a non-90-degree direction is a 45 degree turn (`libs/kimath/include/geometry/direction45.h:260`), which is exactly the straight/diagonal toggle. `m_manuallyForced` is sticky for the life of the trail and is read by the placer to suppress smart pads (`pcbnew/router/pns_line_placer.cpp:764`, `:984`), suppress FANOUT_CLEANUP (`:1045`), and enable the tail-dropping branch in `buildInitialLine` (`:2053`).

`GetTrailLeadVector` (`:279`) returns `last - first` of the trail, or zero for a degenerate trail. **Correction (2026-09-10):** an earlier revision of this note said it has no callers. It has one, `DRAGGER::propagateViaForces` (`pcbnew/router/pns_dragger.cpp:67`), which negates it to push a dragged via back against the direction of travel; note 06 section 2.12 has the detail.

---

## 5. WALKAROUND

Erratum (2026-09-08, found while porting): in the `MITERED_90` and `ROUNDED_90` corner modes both `WALKAROUND::processCluster` (`pns_walkaround.cpp:160`) and `NODE::NearestObstacle`'s `makeHull` (`pns_node.cpp:335`) build the box hull by appending four points to a default `SHAPE_LINE_CHAIN` and never call `SetClosed( true )`. `SHAPE_LINE_CHAIN::PointInside` answers false for any open chain, and `LINE::Walkaround` relies on it (`pns_line.cpp:308`, `:409`, `:478`), so the box hull has no inside in KiCad. The port closes the box (`src/walkaround.rs`, `src/node.rs`); it is the one place milestone 3 routes differently from KiCad on purpose.

Declared `pcbnew/router/pns_walkaround.h:36`, implemented in `pcbnew/router/pns_walkaround.cpp`. It is an `ALGO_BASE`, so it reads settings and the debug decorator through the router.

### 5.1 Policies, statuses, result

`WALK_POLICY` (`pcbnew/router/pns_walkaround.h:70`): `WP_CW = 0`, `WP_CCW = 1`, `WP_SHORTEST = 2`. `MaxWalkPolicies = 3` (`:38`). The three policies run *simultaneously* in one `Route` call, each with its own line and status, selected by `SetAllowedPolicies` (`:396`).

`STATUS` (`:61`): `ST_IN_PROGRESS = 0`, `ST_ALMOST_DONE`, `ST_DONE`, `ST_STUCK`, `ST_NONE`. `ST_ALMOST_DONE` means "a path exists but it does not reach the requested endpoint"; `ST_DONE` means it reaches both the original start and the original end.

`RESULT` (`:77`) is a pair of parallel arrays `STATUS status[3]` and `LINE lines[3]`, all statuses initialised to `ST_NONE`.

Callers and their policy choice:

- `LINE_PLACER::rhWalkBase`: `{ WP_CCW, WP_CW }`, item mask from the caller, default length limit (`pcbnew/router/pns_line_placer.cpp:567`).
- `DRAGGER`: `{ WP_SHORTEST }` with `SetLengthLimit( true, 30.0 )` (`pcbnew/router/pns_dragger.cpp:692`).
- `MULTI_DRAGGER`: `{ WP_SHORTEST }` with `SetLengthLimit( true, 3.0 )` (`pcbnew/router/pns_multi_dragger.cpp:391`).
- `DIFF_PAIR_PLACER`: `{ WP_SHORTEST }` (`pcbnew/router/pns_diff_pair_placer.cpp:207`).
- `SHOVE`: uses `RestrictToCluster` (`pcbnew/router/pns_shove.cpp:816`).

### 5.2 Route

`Route` (`pcbnew/router/pns_walkaround.cpp:303`):

```
Route(initialPath) -> RESULT:
    m_initialLength = initialPath.Length()
    start(initialPath):                                  # :38
        m_iteration = 0
        for every policy: status = ST_IN_PROGRESS, line = initialPath (links cleared)
    m_processedItems.clear()
    while m_iteration < m_iterationLimit:                # default 40
        singleStep()
        stillInProgress = false
        for each enabled policy:
            lengthFactor = len(line) / len(initialPath)
            if m_lengthLimitOn and status != ST_DONE and lengthFactor > m_lengthExpansionFactor:
                status = ST_ALMOST_DONE                  # bail out of a runaway walk
            if status == ST_IN_PROGRESS: stillInProgress = true
        if !stillInProgress: break
        m_iteration++
    for every policy (enabled or not):
        line.ClearLinks()
        if status == ST_IN_PROGRESS: status = ST_ALMOST_DONE
        if line has no segments or line[0] != initialPath[0]: status = ST_STUCK
        if line has points and line.last != initialPath.last: status = ST_ALMOST_DONE
    return m_currentResult
```

`m_lengthExpansionFactor` defaults to 10.0 and `m_lengthLimitOn` to true (`pcbnew/router/pns_walkaround.h:54-55`), so the line placer, which never calls `SetLengthLimit`, uses a 10x runaway guard. The two draggers tighten it to 30x and 3x respectively, which is the opposite direction from what the names suggest: `SetLengthLimit(true, 30.0)` is *looser* than the default.

Two asymmetries to preserve. `start()` (`:41`) initialises all three policy slots to `ST_IN_PROGRESS` with a copy of the initial path, regardless of `m_enabledPolicies`; only `singleStep` and the in-loop check honour the enabled mask. The final classification loop at `:369` again runs over all three slots, so a *disabled* slot ends as `ST_ALMOST_DONE` carrying an unmodified copy of the initial path (or `ST_STUCK` if that path has no segments). Callers must therefore key off their own policy selection, never off "is this slot not `ST_NONE`".

### 5.3 singleStep

`singleStep` (`:94`) has two halves.

Half one, per enabled policy still in progress: find the nearest obstacle to that policy's current line, and if there is none mark the policy `ST_DONE`; otherwise assemble the obstacle's *cluster* with `TOPOLOGY::AssembleCluster( obstacle->m_item, line.Layer(), 0.0, line.Net() )` (`:124`). Working on a cluster rather than a single item is what lets the walk clear a whole pad row or via group in one pass.

`nearestObstacle` (`:50`) sets `opts.m_kindMask = m_itemMask`, installs a filter restricting the search to `m_restrictedSet` when that set is non-empty, and sets `opts.m_useClearanceEpsilon = true`.

Half two, the `processCluster` lambda (`:131`), run once per policy:

```
processCluster(cluster, line, cw) -> bool:
    start_time = now
    timeout_ms = ADVANCED_CFG::m_PNSProcessClusterTimeout
    for each item in cluster:
        if elapsed > timeout_ms: return false            # wallclock bail-out
        clearance = m_world->GetClearance(item, &line, false)
        cachedHull = ruleResolver->HullCache(item, clearance, line.Width(), line.Layer())
        if cornerMode is MITERED_90 or ROUNDED_90:
            hull = the axis-aligned bbox of cachedHull, as a 4-point chain
        else:
            hull = cachedHull
        line.Line().Simplify2()
        if !line.Walkaround(hull, tmp.Line(), cw): return false
        line.SetShape(tmp.CLine())
    return true
```

`WP_CW` and `WP_CCW` each call it once with `cw = true` / `cw = false` and go `ST_STUCK` on failure (`:199-211`).

`WP_SHORTEST` (`:213`) is different: it runs the cluster both ways from the same starting line, checks each result for residual collisions against the world, and then picks:

```
if both directions succeeded:
    if (neither collides) or (both collide):  shortest = the shorter one, shortest_alt = the other
    elif cw does not collide:                 shortest = cw
    elif ccw does not collide:                shortest = ccw
elif only ccw succeeded: shortest = ccw
elif only cw succeeded:  shortest = cw
if shortest collides with any item already processed in an earlier iteration:
    shortest = shortest_alt        # back off to the other winding
if !shortest: status = ST_STUCK  else: line = shortest
m_processedItems += this cluster's items
```

The check-back against `m_processedItems` (`:267-277`) is the mechanism that stops the shortest-path policy from ping-ponging: a winding that undoes the clearance of an obstacle resolved in an earlier iteration is rejected in favour of the alternate winding.

`singleStep` returns `ST_IN_PROGRESS` cast to bool (`:299`), which is `0`, that is `false`, always. The return value is ignored by `Route`.

### 5.4 The per-hull walk: LINE::Walkaround

The actual geometry is in `LINE::Walkaround( hull, out path, cw )` (`pcbnew/router/pns_line.cpp:297`). It builds a directed graph over the union of path vertices and hull vertices and traverses it:

1. Reject immediately if the path's first point is strictly inside the hull (`:308-315`). This is the precondition that `splitHeadTail` in the placer exists to preserve.
2. `HullIntersection( hull, line, ips )` (`:341`), then split both the path copy `pnew` and the hull copy `hnew` at every intersection point (`:367-374`), plus split the hull at any path point lying on a hull edge (`:376-388`).
3. Handle a self-intersecting path by splitting at the self-intersection (`:360`).
4. If `!cw`, reverse the hull. The comment at `:397` states the invariant: hulls are produced clockwise by construction, so counter-clockwise walking is expressed as a reversed hull, not as different traversal logic.
5. Classify every `pnew` vertex as `INSIDE`, `OUTSIDE`, or `ON_EDGE` (`:420`). Link consecutive path vertices in both directions (`:430-439`). Merge hull vertices into the vertex list, marking shared positions as both path and hull vertices (`:442-463`), then link each hull vertex to the next one around the hull (`:466-473`).
6. Traverse from `vts[0]` until reaching the path's last vertex, with a hard `iterLimit = 1000` (`:498`) and a visited-loop break (`:510`). At an `OUTSIDE` vertex, take the next path-adjacent, non-inside, unvisited neighbour, with a fallback to a visited one that is not the immediate predecessor (`:530-548`). At an `ON_EDGE` vertex, prefer an unvisited `OUTSIDE` neighbour, else the next non-hull path vertex whose hull index is `(current + 1) mod hullPointCount` (`:583`), else any `ON_EDGE` neighbour at the next hull index, in which case all hull vertices are un-visited to allow another lap (`:614-618`).
7. Special case for a cursor inside the hull (`inLast`, `:478`): while walking the hull, track the squared distance from the candidate next vertex to the line's last point, and as soon as it stops decreasing, project the last point onto the current hull edge, append the projection and stop (`:628-643`). This is what makes the preview stick to the hull boundary nearest the cursor instead of orbiting the obstacle.
8. Finish with `out.Simplify2(false)` and `restoreUntouchedArcs( out, pnew )` (`:660-661`), which pushes arc data back into runs of points that the walk did not modify, since the graph only carries vertices.

Failure returns are `false` at `:314` (start inside hull), `:508` (iteration limit), `:556` (no usable neighbour) and `:652` (null next vertex), each of which makes `processCluster` return false and the policy go `ST_STUCK`.

### 5.5 Winding direction, cluster restriction, and the visible area

The "`aWindingDirection`" concept in this codebase is the boolean `aCw` threaded from `singleStep` through `LINE::Walkaround`, realised as hull reversal (`pcbnew/router/pns_line.cpp:399`). There is a `SetForceWinding( aEnabled, aCw )` setter on `WALKAROUND` (`pcbnew/router/pns_walkaround.h:112`) writing `m_forceWinding` and `m_forceCw`, but nothing in the tree calls it and nothing reads those members. The same is true of `SetPickShortestPath` / `m_useShortestPath` (`:118`), `m_forceLongerPath` (`:154`), `m_cursorPos` and `m_lastP` (`:150`, `:151`), and the declared-but-never-defined overload `STATUS Route( const LINE&, LINE&, bool )` (`:125`). All dead; do not port.

`RestrictToCluster( aEnabled, aCluster )` (`pcbnew/router/pns_walkaround.cpp:71`) fills `m_restrictedSet` with the cluster's items plus their holes, which narrows `nearestObstacle`'s search to that cluster only. It also always fills `m_restrictedVertices` with the cluster's solid anchors, though nothing reads that vector. Its only caller is `SHOVE` (`pcbnew/router/pns_shove.cpp:816`).

Restrict-to-visible-area: `WALKAROUND` does **not** implement any visible-area restriction in this revision. The only consumer of `ALGO_BASE::VisibleViewArea()` (`pcbnew/router/pns_algo_base.cpp:40`, backed by `ROUTER::m_visibleViewArea`, `pcbnew/router/pns_router.h:256`) is `SHOVE::runOptimizer`, which intersects the total affected area with the visible view area before handing it to the optimizer (`pcbnew/router/pns_shove.cpp:2040`). The host feeds the value in from the canvas viewport (`pcbnew/router/router_tool.cpp:925`, `:3037`), and the router default is `BOX2I::SetMaximum()` (`pcbnew/router/pns_router.cpp:77`), so a headless replay optimises everywhere.

---

## 6. pns_utils: hull geometry

Erratum (2026-09-08, found while porting): the description of the side test in `HullIntersection` below has the sign reversed. `pns_utils.cpp:455` keeps a corner hit when `d1[i].Side( d2[j] ) > 0` for some hull edge and some neighbouring line point, and `SEG::Side > 0` is the right of the directed edge in screen coordinates, the inner side of a clockwise hull. See `src/geometry/hull.rs` and its tests.

Declared `pcbnew/router/pns_utils.h`, implemented in `pcbnew/router/pns_utils.cpp`. `constexpr int HULL_MARGIN = 10` lives at `pcbnew/router/pns_utils.h:34` and is unused; the macro `PNS_HULL_MARGIN 10` at `pcbnew/router/pns_line.h:45` is the one actually used (`pcbnew/router/pns_node.cpp:1338`, `:1348`, `pcbnew/router/pns_diff_pair_placer.cpp:247`).

There is no `ClipLine` in the PNS namespace. `ClipLine` exists only in `libs/kimath/include/geometry/geometry_utils.h:238` (Cohen-Sutherland box clipping, used by `pcbnew/pcb_track.cpp:2347`), and `KIGEOM::ClipLineToBox` in `libs/kimath/include/geometry/shape_utils.h:112`. Neither is part of the router.

**OctagonalHull( p0, size, clearance, chamfer )** (`pcbnew/router/pns_utils.cpp:40`). Produces a closed chain around the rectangle `[p0, p0+size]` inflated by `clearance`, with each corner cut by `chamfer`. With `chamfer == 0` it degenerates to a rectangle (the four conditional `Append`s are skipped). Vertex order, starting at the left edge just above the bottom-left chamfer:

```
(x0-c, y0-c+ch)
(x0-c+ch, y0-c)                  if ch
(x0+w+c-ch, y0-c)
(x0+w+c, y0-c+ch)                if ch
(x0+w+c, y0+h+c-ch)
(x0+w+c-ch, y0+h+c)              if ch
(x0-c+ch, y0+h+c)
(x0-c, y0+h+c-ch)                if ch
```

**SegmentHull( seg, clearance, walkaroundThickness )** (`:181`). This is the workhorse and it carries the 45-degree snapping tolerances.

```
kinkThreshold = clearance / 10
cl = clearance + walkaroundThickness / 2
d  = seg.width / 2 + cl
x  = 2 / (1 + sqrt(2)) * d        # the octagon "corner cut" length
dr = round(d);  xr2 = round(x / 2)
```

Before building, degenerate short segments are straightened (`:207-248`):

- If the segment is **not** 45-degree (`IsSegment45Degree`, `:157`) and `0 < len <= kinkThreshold`, the endpoint is replaced by `a + (sgn(w)*ll, sgn(h)*ll)` where `ll = max(|w|, |h|)`, that is snapped to an exact diagonal.
- If the segment **is** 45-degree and `len <= kinkThreshold`:
  - `|w| <= 1` (almost vertical): `w = 0` and `cl += 1`;
  - `|h| <= 1` (almost horizontal): `h = 0` and `cl += 1`;
  - `| |w| - |h| | <= 2` (almost 45): both components set to `sgn(.) * max(|w|,|h|)` and `cl += 2`.

`IsSegment45Degree` (`:157`) accepts a segment as 45-degree when `|dx| <= 1`, or `|dy| <= 1`, or `| |dx| - |dy| | <= 1`. These `<= 1` and `<= 2` tolerances are integer-unit slop, not angles.

For a zero-length segment (`a == b` after straightening) the hull is an octagon around a `width x width` square with chamfer `round( 2*(1 - 1/sqrt(2)) * d )` (`:252-257`).

Otherwise the hull is eight points built from the direction vector (`:262-279`):

```
dir = b - a
p0 = perp(dir).Resize(dr)      # full offset, perpendicular
ds = perp(dir).Resize(xr2)     # corner-cut offset, perpendicular
pd = dir.Resize(xr2)           # corner-cut offset, along
dp = dir.Resize(dr)            # full offset, along

b + p0 + pd,  b + dp + ds,  b + dp - ds,  b - p0 + pd,
a - p0 - pd,  a - dp - ds,  a - dp + ds,  a + p0 - pd
```

Finally the chain is reversed if `s.CSegment(0).Side(a) < 0`, guaranteeing clockwise orientation (`:282`). That invariant is what `LINE::Walkaround` relies on for its CW/CCW hull reversal trick.

**ArcHull( arc, clearance, walkaroundThickness )** (`:71`). `cl = clearance + (walkaroundThickness + 1) / 2` (note the rounding-up, different from `SegmentHull`). If the arc's central angle exceeds 180 degrees and its chord is shorter than `cl`, the arc is treated as a full circle and delegated to `OctagonalHull` with chamfer `2*(1 - 1/sqrt(2)) * (r + cl)` (`:76-83`). Otherwise `d = width/2 + cl + SHAPE_ARC::DefaultAccuracyForPCB()` and `x = (2/(1+sqrt(2)) * d) / 2`; the arc is polygonised with `ARC_LOW_DEF` and the hull is built as an outer offset chain plus a reversed inner offset chain, with a four-point cap at each end using the same `p0 / ds / pd / dp` scheme as `SegmentHull` (`:102-105`, `:141-144`). Interior vertices use the intersection of adjacent offset lines, that is a proper miter joint, not a per-segment offset (`:127-128`). Clockwise orientation is enforced the same way (`:150`).

**ConvexHull( convex, clearance )** (`:300`). Builds an octagon around a `SHAPE_SIMPLE` by intersecting four axis-aligned lines derived from the inflated bounding box with four diagonals. Each diagonal starts at a bbox corner with slope +/-1 and length `box.GetHeight()`, and is then slid inward by `MoveDiagonal` (`:289`) until it is exactly `clearance` away from the nearest polygon vertex:

```
MoveDiagonal(diag, vertices, clearance):
    vertices.NearestPoint(diag, dist)
    moveBy = perp(diag.A - diag.B).Resize(dist - clearance)
    diag.A += moveBy;  diag.B += moveBy
```

The eight octagon vertices are the eight pairwise line intersections in order left/bottom-left, bottom/bottom-left, bottom/bottom-right, right/bottom-right, right/top-right, top/top-right, top/top-left, left/top-left (`:343-350`). Unlike the other hull builders, this one does not normalise the winding.

**BuildHullForPrimitiveShape( shape, clearance, walkaroundThickness )** (`:478`) is the dispatcher, with `cl = clearance + (walkaroundThickness + 1) / 2`:

| shape | hull |
| --- | --- |
| `SH_RECT` | `OctagonalHull( pos, size, cl, 0 )`, that is a plain inflated rectangle (`:488`) |
| `SH_CIRCLE` | `OctagonalHull` around the `2r x 2r` square with chamfer `2*(1 - 1/sqrt(2))*(r + cl)` (`:498`) |
| `SH_SEGMENT` | `SegmentHull( seg, aClearance, aWalkaroundThickness )`, note the *raw* arguments, not `cl` (`:507`) |
| `SH_ARC` | `ArcHull( arc, aClearance, aWalkaroundThickness )`, likewise raw (`:513`) |
| `SH_SIMPLE` | `ConvexHull( convex, cl )` (`:520`) |
| `SH_ELLIPSE` | `OctagonalHull` around the bbox with chamfer 0 (`:528`) |
| anything else | `wxFAIL_MSG` and an empty chain |

`aWalkaroundThickness` is always the width of the *moving* line, not of the obstacle: the caller passes `aLine.Width()` (`pcbnew/router/pns_walkaround.cpp:158`, `pcbnew/router/pns_line_placer.cpp:822`, `pcbnew/router/pns_shove.cpp:1282`). Halving it and adding it to the clearance is what turns a centreline-based collision query into a "the line's edge clears the obstacle" query, so the walk can treat the routed line as a zero-width polyline.

Per-item hulls:

- `SEGMENT::Hull` (`pcbnew/router/pns_line.cpp:668`) is `SegmentHull` directly.
- `ARC::Hull` (`pcbnew/router/pns_arc.cpp:28`) is `ArcHull` directly.
- `VIA::Hull` (`pcbnew/router/pns_via.cpp:235`) uses `cl = clearance + walkaroundThickness/2` (not rounded up) and `width = Diameter(aLayer)`, falling back to `2 * hole radius` when the via is not flashed on that layer (`:243`). Chamfer is `(2*cl + width) * (1 - 1/sqrt(2))`, the equilateral octagon value for the inflated square.
- `HOLE::Hull` (`pcbnew/router/pns_hole.cpp:57`) mirrors `VIA::Hull` for circular holes, and otherwise dispatches per primitive.
- `SOLID::Hull` (`pcbnew/router/pns_solid.cpp:39`) dispatches a single-primitive compound directly, and for a multi-primitive compound unions all per-primitive hulls into a `SHAPE_POLY_SET`, simplifies and returns outline 0 (`:55-64`). Same for `HOLE` (`pcbnew/router/pns_hole.cpp:84-93`).

**HullIntersection( hull, line, ips )** (`pcbnew/router/pns_utils.cpp:395`). A filtered intersection: raw intersections are computed with `hull.Intersect(line, ips_raw)` and then each is validated. Non-corner-on-both-sides intersections are accepted unconditionally (`:416`). For corner cases, up to two hull segments (`d1`) and up to two line points (`d2`) around the corner are collected, and the intersection is kept only if some `d1[i].Side(d2[j]) > 0`, that is, only if the line actually crosses to the outside of the hull rather than merely touching it (`:455-464`). Without this filter, tangential contacts would create spurious graph vertices in `LINE::Walkaround`.

**Other helpers.** `ApproximateSegmentAsRect` (`:356`) inflates the segment's endpoints by `width/2` in both axes and returns the normalised `SHAPE_RECT` (a coarse over-approximation, wrong for diagonals but conservative). `ChangedArea` (`:369`, `:389`) returns the bbox of the difference between two vias or two lines, used for optimizer area restriction. `NodeStats` (`:544`) is a debug-only dump of a node's added/removed sets.

**HullCache**. Hulls are memoised by the rule resolver rather than by the geometry layer. `RULE_RESOLVER::HullCache` (`pcbnew/router/pns_node.h:176`) has a default implementation that just forwards to `ITEM::Hull` through a function-local static, so the base class provides no caching and returns a reference that is invalidated by the next call. The KiCad implementation (`pcbnew/router/pns_kicad_iface.cpp:832`) keys an `unordered_map` on `HULL_CACHE_KEY { item pointer, clearance, walkaroundThickness, layer }` (`pcbnew/router/pns_kicad_iface.cpp:222`) and returns a stable reference into the map. `ClearCaches()` empties it (`pcbnew/router/pns_kicad_iface.cpp:821`), `ClearTemporaryCaches()` does not. This matters for a port: the hull cache is keyed by *pointer identity*, so any item mutation must invalidate it.

---

## 7. LOGGER, DEBUG_DECORATOR, and the QA log format

### 7.1 LOGGER

`pcbnew/router/pns_logger.h:48`. The logger records a linear list of `EVENT_ENTRY` and nothing else; geometry is not logged, only the user's input events. Replay reconstructs the geometry by running the real router.

`EVENT_TYPE` (`pcbnew/router/pns_logger.h:60`): `EVT_START_ROUTE = 0`, `EVT_START_DRAG`, `EVT_FIX`, `EVT_MOVE`, `EVT_ABORT`, `EVT_TOGGLE_VIA`, `EVT_UNFIX`, `EVT_START_MULTIDRAG`. `EVT_ABORT` is never emitted by anything in the tree.

`EVENT_ENTRY` (`:71`) carries `VECTOR2I p`, `EVENT_TYPE type`, `vector<KIID> uuids`, `SIZES_SETTINGS sizes`, `int layer`.

Emission sites, all in `ROUTER`:

| event | site | payload |
| --- | --- | --- |
| `EVT_START_ROUTE` | `pcbnew/router/pns_router.cpp:479` | position, start item, `&m_sizes`, `m_placer->CurrentLayer()` |
| `EVT_START_DRAG` | `:204` | position, single start item |
| `EVT_START_MULTIDRAG` | `:206` | position, item vector |
| `EVT_MOVE` | `:497` | position, end item |
| `EVT_FIX` | `:920` | position, end item |
| `EVT_UNFIX` | `:952` | nothing but the type |
| `EVT_TOGGLE_VIA` | `:1027` | `&m_sizes` only, zero position, null item |

`Log` (`:99`) wraps a single item into a vector and calls `LogM` (`:75`), which stores only `item->Parent()->m_Uuid` for items that have a parent board item. That is the key design point: the log identifies items by board UUID, so replay resolves them through `NODE::FindItemByParent` on a freshly synced world (`qa/tools/pns/pns_log_player.cpp:116`).

Note that both `StartRouting` and `StartDragging` call `m_logger->Clear()` (`pcbnew/router/pns_router.cpp:478`, `:199`), so the in-memory log only ever holds one session.

`LOG_DATA` (`pcbnew/router/pns_logger.h:94`) is the on-disk payload: mode, optional board hash, added items, removed item UUIDs, head items, events, optional test case type. `TEST_CASE_TYPE` (`:52`): `TCT_STRICT_GEOMETRY = 0`, `TCT_CONNECTIVITY_ONLY`, `TCT_EXPECTED_FAIL`, `TCT_KNOWN_BUG`.

### 7.2 The JSON log format

`FormatLogFileAsJSON` (`pcbnew/router/pns_logger.cpp:107`) emits, pretty-printed with `setw(2)` under a `LOCALE_IO` guard:

```json
{
  "mode": <int ROUTER_MODE>,
  "test_case_type": <int, optional>,
  "board_hash": "<string, optional>",
  "events": [ { "position": {"x":..,"y":..},
                "type": <int EVENT_TYPE>,
                "layer": <int>,
                "uuids": ["<kiid>", ...],
                "sizes": { "trackWidth":.., "viaDiameter":.., "viaDrill":..,
                           "trackWidthIsExplicit":<bool>,
                           "layerBottom":.., "layerTop":.., "viaType":<int> } } ],
  "removedItems": [ "<kiid>", ... ],
  "addedItems":   [ <item>, ... ],
  "headItems":    [ <item>, ... ]
}
```

An `<item>` (`formatRouterItemAsJSON`, `:173`) is `{ "kind": <KindStr>, "net": "<net name>", "layers": [start, end], "shape": <shape>, "drill": <int for vias> }`. A `<shape>` (`formatShapeAsJSON`, `:231`) is one of `{"type":"segment","width","start","end"}`, `{"type":"arc","width","start","end","mid"}`, `{"type":"circle","radius","center"}`; anything else serialises as `null`. `VECTOR2I` is `{"x":..,"y":..}` (`:40`).

Only `SEGMENT_T`, `ARC_T`, `VIA_T` and `HOLE_T` get a shape; everything else is emitted with kind, net and layers only (`:209`).

The `sizes` block is recorded on *every* event, but only `EVT_START_ROUTE`, `EVT_START_DRAG`/`EVT_START_MULTIDRAG` and `EVT_TOGGLE_VIA` are actually given a non-default `SIZES_SETTINGS` to record, so the rest carry the `SIZES_SETTINGS` constructor defaults.

`ParseEventFromJSON` (`:297`) reads back position, type, layer and uuids, and notably **not** the sizes: the replayer re-derives them from the board via `iface->ImportSizes` (`qa/tools/pns/pns_log_player.cpp:135`). A legacy whitespace-delimited text format is still parsed by `ParseEvent` (`:274`), with the line shape `event <x> <y> <type> <layer> <n_uuids> <uuid>...`.

### 7.3 The QA test case on disk

`PNS_LOG_FILE::Load` (`qa/tools/pns/pns_log_file.cpp:484`) expects a family of files sharing a base name:

| extension | contents |
| --- | --- |
| `.log` | the JSON (or legacy) event log described above |
| `.dump` | the board snapshot, loaded with `PCB_IO_KICAD_SEXPR` (`:531`) |
| `.kicad_pro` | project settings, loaded read-only (`:525`) |
| `.settings` | the `ROUTING_SETTINGS` JSON, loaded via `LoadFromRawFile`; a load failure only warns and falls back to defaults (`:513`) |
| `.kicad_dru` (`FILEEXT::DesignRulesFileExtension`) | optional custom DRC rules for the case (`:555`) |

Loading tries the JSON parser first and falls back to the legacy line-based parser (`:571-576`). `SaveLog` (`:451`) writes only the `.log`; the board and settings are expected to already exist.

The replayer (`qa/tools/pns/pns_log_player.cpp:91`) maps events one to one onto router calls: `EVT_START_ROUTE -> ImportSizes + UpdateSizes + StartRouting(evt.p, item, layer)`, `EVT_START_DRAG`/`EVT_START_MULTIDRAG -> StartDragging(evt.p, items, 0)`, `EVT_FIX -> FixRoute(evt.p, item, false, false)`, `EVT_UNFIX -> UndoLastSegment()`, `EVT_MOVE -> Move(evt.p, item)`, `EVT_TOGGLE_VIA -> ToggleViaPlacement()`. The routing layer is `item->Layers().Start()` when the item resolved, else the logged `evt.layer` (`:118`). Note that `StartDragging` is replayed with drag mode `0`, not `DM_ANY`.

Comparison is `PNS_LOG_FILE::COMMIT_STATE::Compare` (`qa/tools/pns/pns_log_file.cpp`, ending at `:448`), which matches the recorded added items and removed UUIDs against what the replay produced. Two consequences for a reimplementation: the regression suite pins *item-level output*, not intermediate geometry, and it pins it under a specific board UUID assignment, which is exactly why `ROUTER::CommitRouting` goes to the trouble of turning remove+add pairs into updates.

### 7.4 DEBUG_DECORATOR

`pcbnew/router/pns_debug_decorator.h:37`. A pure-virtual sink with default no-op bodies. Surface: `SetIteration`, `Message`, `NewStage`, `BeginGroup` / `EndGroup` (hierarchical, with a level), `AddPoint( p, color, size, name )`, `AddItem( item, color, overrideWidth, name )`, `AddShape( SHAPE* | BOX2I | SEG, color, overrideWidth, name )`, `Clear`. Every method also takes a `SRC_LOCATION_INFO { fileName, funcName, line }` (`:44`).

The `PNS_DBG( dbg, method, ... )` macro (`:126`) appends `SRC_LOCATION_INFO(__FILE__, __FUNCTION__, __LINE__)` to the argument list and guards the whole call on `dbg && dbg->IsDebugEnabled() && !PNS_SILENCE_DEBUG`, so string formatting and geometry copying are skipped entirely when debugging is off. `PNS_DBGN` (`:131`) is the zero-argument variant used for `EndGroup`.

For a Rust port: this is a tracing sink, and the guard is the whole point. Model it as a trait object behind an `Option`, or better as a compile-time-elidable macro over a `Sink` trait, because the call sites are extremely dense inside `routeStep`, `rhWalkBase` and `singleStep` and the geometry arguments are expensive to materialise.

---

## 8. Magic constants

Every non-obvious literal that affects geometry or termination, with location.

| value | meaning | location |
| --- | --- | --- |
| `40` | walkaround iteration limit (default setting) | `pcbnew/router/pns_routing_settings.cpp:45`, `:86` |
| `250` | shove iteration limit (default setting) | `pcbnew/router/pns_routing_settings.cpp:43`, `:72` |
| `1000` | shove time limit, milliseconds (default setting) | `pcbnew/router/pns_routing_settings.cpp:44`, `:84` |
| `40` | via force propagation iteration limit (default setting) | `pcbnew/router/pns_routing_settings.cpp:58`, `:73` |
| `1.5` | walkaround hug length threshold (default setting) | `pcbnew/router/pns_routing_settings.cpp:54`, `:106` |
| `2.0 *` threshold | complete-walkaround acceptance multiplier | `pcbnew/router/pns_line_placer.cpp:592` |
| `10.0` | `WALKAROUND::m_lengthExpansionFactor` default | `pcbnew/router/pns_walkaround.h:55` |
| `30.0` | length expansion factor used by `DRAGGER` | `pcbnew/router/pns_dragger.cpp:692` |
| `3.0` | length expansion factor used by `MULTI_DRAGGER` | `pcbnew/router/pns_multi_dragger.cpp:391` |
| `3` | `MaxWalkPolicies` | `pcbnew/router/pns_walkaround.h:38` |
| `1000` | hard iteration limit inside `LINE::Walkaround`'s graph traversal | `pcbnew/router/pns_line.cpp:498` |
| `ADVANCED_CFG::m_PNSProcessClusterTimeout` | per-cluster wallclock budget; the comment says 100 ms is the empirical sweet spot, the actual default is in `common/advanced_config.cpp` which is not in this checkout | `pcbnew/router/pns_walkaround.cpp:136`, comment `:143-145` |
| `5` | `Finish()` move retries (`triesLeft`) | `pcbnew/router/pns_router.cpp:594` |
| `10` percent | diff pair start gap tolerance (`configuredGap / 10`) | `pcbnew/router/pns_router.cpp:369` |
| `3` | `tailLookbackSegments` in `optimizeTailHeadTransition` | `pcbnew/router/pns_line_placer.cpp:1061` |
| `2` | head slice length in `optimizeTailHeadTransition` (`end = min(2, ...)`) | `pcbnew/router/pns_line_placer.cpp:1074` |
| `3` | minimum head shapes required by `mergeHead` | `pcbnew/router/pns_line_placer.cpp:335` |
| `3` | minimum tail shapes required by `optimizeTailHeadTransition` | `pcbnew/router/pns_line_placer.cpp:1068` |
| `2` | minimum tail segments required by `reduceTail` | `pcbnew/router/pns_line_placer.cpp:267` |
| `2` | `n < 2` self-intersection "restart from scratch" cut-off | `pcbnew/router/pns_line_placer.cpp:151` |
| `width / 2` | hull snap radius in `rhMarkObstacles` | `pcbnew/router/pns_line_placer.cpp:832` |
| `2` | number of via pushout attempts in `buildInitialLine` | `pcbnew/router/pns_line_placer.cpp:2117` |
| `2` | `round < 2` walk rounds when placing a via | `pcbnew/router/pns_line_placer.cpp:713` |
| `22.5` | pad orientation to octant rounding offset (`(angle + 22.5) / 45.0`) | `pcbnew/router/pns_line_placer.cpp:1416` |
| `22.5` | vector to octant rounding offset in `DIRECTION_45::construct_` | `libs/kimath/include/geometry/direction45.h:332` |
| `1.3` | `areaRatioThreshold` (posture switch threshold) | `pcbnew/router/pns_mouse_trail_tracer.cpp:87` |
| `0.25` | `areaRatioEpsilon` (posture hysteresis) | `pcbnew/router/pns_mouse_trail_tracer.cpp:90` |
| `6` | `minAreaCutoffDistanceFactor` (x tolerance) | `pcbnew/router/pns_mouse_trail_tracer.cpp:93` |
| `30` | `lockDistanceFactor` (x tolerance) | `pcbnew/router/pns_mouse_trail_tracer.cpp:96` |
| `10` | `unlockDistanceFactor` (x tolerance) | `pcbnew/router/pns_mouse_trail_tracer.cpp:99` |
| `+1.0` | denominator guard in the posture area ratio | `pcbnew/router/pns_mouse_trail_tracer.cpp:133` |
| `tolerance * refLength` | posture area cutoff | `pcbnew/router/pns_mouse_trail_tracer.cpp:152` |
| `m_tolerance^2` | trail self-pruning squared distance limit | `pcbnew/router/pns_mouse_trail_tracer.cpp:59` |
| `2` | trail segments excluded from the self-pruning scan (`SegmentCount() - 2`) | `pcbnew/router/pns_mouse_trail_tracer.cpp:61` |
| `10` | `PNS_HULL_MARGIN` (used), `HULL_MARGIN` (unused) | `pcbnew/router/pns_line.h:45`, `pcbnew/router/pns_utils.h:34` |
| `clearance / 10` | `kinkThreshold` in `SegmentHull` | `pcbnew/router/pns_utils.cpp:184` |
| `2 / (1 + sqrt(2))` | octagon corner-cut ratio, `x = ratio * d` | `pcbnew/router/pns_utils.cpp:188`, `:86` |
| `2 * (1 - 1/sqrt(2))` | equilateral octagon chamfer factor | `pcbnew/router/pns_utils.cpp:252`, `:82`, `:501` |
| `(1 - 1/sqrt(2))` | via and hole chamfer factor applied to `2*cl + width` | `pcbnew/router/pns_via.cpp:249`, `pcbnew/router/pns_hole.cpp:71` |
| `<= 1` | "almost axis aligned" tolerance in `IsSegment45Degree` and in `SegmentHull` | `pcbnew/router/pns_utils.cpp:161`, `:164`, `:169`, `:224`, `:228` |
| `<= 2` | "almost 45 degree" tolerance (`delta45`) in `SegmentHull` | `pcbnew/router/pns_utils.cpp:233` |
| `cl++`, `cl += 2` | clearance bumps compensating the straightening above | `pcbnew/router/pns_utils.cpp:226`, `:230`, `:239` |
| `ARC_LOW_DEF` | arc polygonisation accuracy in `ArcHull` | `pcbnew/router/pns_utils.cpp:88` |
| `SHAPE_ARC::DefaultAccuracyForPCB()` | extra clearance added to `d` in `ArcHull` | `pcbnew/router/pns_utils.cpp:85` |
| `180.0` deg | central angle above which an arc is hulled as a circle | `pcbnew/router/pns_utils.cpp:76` |
| `diameter / 4` | via pushout per-iteration force cap and lead-switch threshold | `pcbnew/router/pns_via.cpp:181` |
| `aMaxIterations / 2` | iteration after which pushout switches to the lead vector | `pcbnew/router/pns_via.cpp:187` |
| `67.5` deg, `3*pi/4` | `ROUNDED_45` arc radius trigonometry | `libs/kimath/src/geometry/direction_45.cpp:131`, `:132` |
| `SHAPE_ARC::MIN_PRECISION_IU` | endpoint tangency snap in `ROUNDED_45` | `libs/kimath/src/geometry/direction_45.cpp:202`, `:207` |
| `256` | `OPTIMIZER::MaxCachedItems` | `pcbnew/router/pns_optimizer.h:159` |
| `155000 / 600000 / 250000 / 125000 / 180000` | default track width, via diameter, via drill, diff pair width, diff pair gap (nm) | `pcbnew/router/pns_sizes_settings.h:46-54` |

---

## 9. Rust mapping notes

### 9.1 Placer state: enum states, not flag soup

The C++ placer encodes its lifecycle in four booleans (`m_idle`, `m_chainedPlacement`, `m_placementCorrect`, `m_placingVia`) plus the nullness of `m_currentNode` / `m_lastNode` and the depth of `m_fixedTail`. Several combinations are unreachable, and the reachable ones are only discoverable by reading every write site. A faithful but sane Rust shape is a two-level split: an outer enum for the lifecycle, an inner struct for the live geometry.

```rust
enum PlacerState {
    Idle { layer: LayerId },                    // before Start, and after a terminal fix
    Placing(Placing),
    Finished { placed_anything: bool },         // terminal fix or via-only commit
}

struct Placing {
    // geometry
    head: Line,
    tail: Line,
    direction: Direction45,
    initial_direction: Direction45,
    p_start: Point,           // tail/head boundary, derived: tail.last().unwrap_or(current_start)
    last_p_end: Option<Point>,
    // anchors
    current_start: Point,
    current_end: Point,
    fix_start: Point,
    start_item: Option<ItemId>,
    end_item: Option<ItemId>,
    // placement flags that are genuinely orthogonal
    placing_via: bool,
    ortho: bool,
    chained: bool,            // last fix left no via, so the layer is pinned
    net: NetHandle,
    layer: LayerId,
    sizes: Sizes,
    posture: MouseTrailTracer,
    fixed_tail: Vec<FixStage>,
}
```

`m_placementCorrect` and `m_fixedTail.StageCount() > 1` collapse into `placed_anything`, computed rather than stored. `m_idle` disappears into the outer enum. `p_start` should be a method, not a field, since `updatePStart` is called at exactly two points in `routeStep` and its rule is total.

The one place where the C++ flags are genuinely essential and must not be simplified is `chained` (`m_chainedPlacement`): it is the only thing preventing a layer change in the middle of a via-less chained trace (`pcbnew/router/pns_line_placer.cpp:1354`).

### 9.2 Mode dispatch: enum, not trait

`routeHead` dispatches on a runtime setting that can change between two `Move` calls, and the three routines are not independent: `rhShoveOnly` calls `rhWalkOnly` as its failure path, and both call the shared `rhWalkBase`. A trait object here buys nothing and costs the ability to share `rhWalkBase` naturally. Use a plain `match`:

```rust
fn route_head(&mut self, p: Point, ctx: &RouterCtx) -> Option<(Line, Line)> {
    match ctx.settings.mode {
        Mode::MarkObstacles => self.rh_mark_obstacles(p, ctx),
        Mode::Walkaround    => self.rh_walk_only(p, ctx),
        Mode::Shove         => self.rh_shove_only(p, ctx),
    }
}
```

Return `Option<(Line, Line)>` rather than `bool` plus two out-parameters; the C++ out-parameter contract is "only valid when true", which the type system should express. Note that the failure case in `routeStep` restores the *previous* head and tail, so the caller must keep them; do not have `route_head` mutate `self.head` (only `rhMarkObstacles` does today, at `pcbnew/router/pns_line_placer.cpp:810`, which is an inconsistency worth removing).

`PLACEMENT_ALGO` (`pcbnew/router/pns_placement_algo.h:183`) is a real abstraction, since the router genuinely holds one of five unrelated placers. That one deserves a trait, or an enum if the set stays closed. Prefer the enum: the set is closed by `ROUTER_MODE`, and it removes the `dynamic_cast<LINE*>` on `Traces()[0]` that `Finish` and `ContinueFromEnd` do (`pcbnew/router/pns_router.cpp:579`, `:624`).

### 9.3 Owning the node tree

The C++ code has a raw-pointer branch tree with `KillChildren()` as a bulk destructor and one manual `delete m_lastNode` per `Move`. It works because ownership is strictly a tree rooted at `ROUTER::m_world` and nothing outlives a `StopRouting`. In Rust the natural encoding is an arena plus generational indices:

```rust
struct NodeArena { nodes: Slab<Node> }
type NodeId = Key;                       // generational, so a stale id is detectable

struct Node { parent: Option<NodeId>, depth: u32, added: ..., removed: ..., children: Vec<NodeId> }
```

with `branch(&mut arena, parent) -> NodeId`, `kill_children(&mut arena, root)`, `commit(&mut arena, root, node)`. The placer then holds `current_node: NodeId` and `last_node: Option<NodeId>` and never owns a node. Three reasons this is the right shape rather than `Rc<RefCell<Node>>` or nested `Box`:

1. Nodes are referenced from outside the tree while alive. `SHOVE` keeps a springback stack of node pointers (`pcbnew/router/pns_shove.h:275`), `FIXED_TAIL::STAGE::commit` holds one per undo stage (`pcbnew/router/pns_line_placer.h:93`), and `ITEM::Owner()` points back at a node (used at `pcbnew/router/pns_line_placer.cpp:1488` and `:938`). A tree of `Box` cannot express that; an arena can, and a generational key turns the C++ dangling-pointer hazard into a detectable error.
2. `KillChildren` is a subtree drop, which is a single arena sweep.
3. Depth comparison (`pcbnew/router/pns_line_placer.cpp:1539`) is a field read, not a pointer walk.

`m_currentNode` and `m_lastNode` are two different lifetimes and should stay two fields: `current_node` is where the algorithms route, `last_node` is a per-`Move` scratch branch that is dropped and rebuilt every frame. Model the rebuild explicitly:

```rust
if let Some(old) = self.last_node.take() { arena.drop_subtree(old); }
...
self.last_node = Some(arena.branch(self.current_node));
```

The one genuinely tricky invariant to preserve: in shove mode `current_node` is re-pointed to `shove.current_node()` twice inside `rh_shove_only` (`pcbnew/router/pns_line_placer.cpp:930`, `:963`), so the shove owns the node the placer is standing on. Either give the shove the arena too, or make `rh_shove_only` return the new node id alongside the lines.

### 9.4 Host-configurable versus core

The split is not the same as the settings/sizes split, and getting it wrong is what makes a router hard to test headlessly.

**Core, must be deterministic and headless:** everything in `LINE_PLACER`, `WALKAROUND`, `MOUSE_TRAIL_TRACER`, `SHOVE`, `OPTIMIZER`, and every hull function. All of `ROUTING_SETTINGS` except the two snap flags. All of `SIZES_SETTINGS` except the four `...Source` strings. The corner mode, the routing mode, the optimizer effort and all iteration limits are core inputs and must be part of any regression fixture, which is exactly why the QA harness ships a `.settings` file per case (`qa/tools/pns/pns_log_file.cpp:513`).

**Host, must be behind a trait:** everything on `ROUTER_IFACE` (`pcbnew/router/pns_router.h:91`). It splits into four unrelated concerns that should be four traits in Rust rather than one 30-method interface:

- *World sync and commit*: `SyncWorld`, `AddItem`, `UpdateItem`, `RemoveItem`, `Commit`, `UpdateNet`, `GetOrphanedNetHandle`, `GetWorld`.
- *Rendering*: `DisplayItem`, `DisplayPathLine`, `DisplayRatline`, `HideItem`, `EraseView`. Called only from `ROUTER::updateView` / `markViolations` / `movePlacing` and from `LINE_PLACER::updateLeadingRatLine`. Everything else is renderer-free.
- *Rules and layers*: `GetRuleResolver`, `IsAnyLayerVisible`, `IsItemVisible`, `IsFlashedOnLayer`, `IsPNSCopperLayer`, `GetBoardLayerFromPNSLayer`, `GetPNSLayerFromBoardLayer`, `StackupHeight`, `ImportSizes`.
- *Length and delay*: `CalculateRoutedPathLength`, `CalculateRoutedPathDelay`, `CalculateLengthForDelay`, `CalculateDelayForShapeLineChain`, `GetSignalAggregate`, `GetNetBoardLength`. Only the tuners use these; the line placer never does.

**Host only, not core at all:** cursor snapping. `snapToItem`, `checkSnap`, `updateStartItem` and `updateEndItem` all live in `TOOL_BASE` (`pcbnew/router/pns_tool_base.cpp:296-520`), and the router receives an already-snapped point. `GetSnapToTracks` / `GetSnapToPads` live in `ROUTING_SETTINGS` but are written by the host from `MAGNETIC_SETTINGS` on every `checkSnap` call (`:314-318`) and read only by the host. Do not put them in the core settings struct.

**Ambient state that should become explicit parameters:** `ROUTER::GetInstance()`. Its four uses (debug decorator in the posture tracer, corner mode in the optimizer and in `NODE`, `IsFlashedOnLayer` in `VIA::Hull`) all become fields or arguments of a `RouterCtx { settings, iface, dbg }` passed by reference. `VIA::Hull` is the awkward one: it needs layer flashing information, so either the hull functions take a `&dyn LayerInfo` or the flashing decision is pushed up into the caller that already has the resolver in hand.

### 9.5 Things not to port

- `ROUTER::SetIterLimit` / `GetIterLimit` / `m_iterLimit` (`pcbnew/router/pns_router.h:224`), never read.
- `WALKAROUND::SetForceWinding`, `SetPickShortestPath`, `m_forceWinding`, `m_forceCw`, `m_useShortestPath`, `m_forceLongerPath`, `m_cursorPos`, `m_lastP`, and the undefined `Route(const LINE&, LINE&, bool)` overload (`pcbnew/router/pns_walkaround.h:112-158`).
- `ROUTING_SETTINGS::SuggestFinish` and `WalkaroundTimeLimit` (no consumers; the latter is never even initialised).
- `LINE_PLACER::AbortPlacement` (`pcbnew/router/pns_line_placer.cpp:2150`), no callers in this revision.
- `PNS::HULL_MARGIN` (`pcbnew/router/pns_utils.h:34`), shadowed by the macro that is actually used.
- The local-minimum branch of `cursorDistMinimum`, hard-disabled at `pcbnew/router/pns_line_placer.cpp:515`.
- `handlePullback`'s `pullback_1`, hard-disabled at `pcbnew/router/pns_line_placer.cpp:213`.
- The `#if 0` "stop at first obstacle" sketch at `pcbnew/router/pns_line_placer.cpp:840`.

### 9.6 Behaviours to reproduce deliberately, not accidentally

These are places where the code and the apparent intent disagree. Decide explicitly for each one, and record the decision, because a regression suite built from KiCad logs will pin the *code* behaviour.

1. `rhWalkBase` passes `aForceNoVia = (round == 0)` after `round++`, so it is always false (`pcbnew/router/pns_line_placer.cpp:576-578`).
2. `LINE_PLACER::Start` computes `initialDir` from pad orientation and `lastSegDir` from the start segment, then passes neither to `SetDefaultDirections` (`:1415`, `:1408`, `:1426`).
3. `ROUTER::Move` never reaches `ClearTemporaryCaches()` while routing (`pcbnew/router/pns_router.cpp:512`).
4. `splitHeadTail` always returns true (`pcbnew/router/pns_line_placer.cpp:917`), so the two early returns guarded on it are unreachable (`:789` in `rhWalkOnly`, `:1000` in `rhShoveOnly`), and the `if( splitHeadTail(...) )` conditions in `rhWalkBase` (`:619`, `:639`) are unconditional.
5. `WALKAROUND::singleStep` returns `ST_IN_PROGRESS` as a bool, that is `false`, and the value is discarded (`pcbnew/router/pns_walkaround.cpp:299`).
6. `WALKAROUND::Route`'s final classification loop runs over disabled policy slots as well (`pcbnew/router/pns_walkaround.cpp:369`).
7. `ROUTER::UndoLastSegment` dereferences `m_placer` while only checking `RoutingInProgress()` (`pcbnew/router/pns_router.cpp:948-954`).
8. `reduceTail` builds `reducedLine` and discards it (`pcbnew/router/pns_line_placer.cpp:305`); `optimizeTailHeadTransition` builds `tmp` and discards it (`:1088`).
9. `SegmentHull` receives raw `aClearance` / `aWalkaroundThickness` from `BuildHullForPrimitiveShape` while every other branch of that dispatcher passes the precombined `cl` (`pcbnew/router/pns_utils.cpp:507`, `:513` versus `:488`, `:498`, `:520`, `:528`). The segment and arc hull functions recombine them internally, but with *different* rounding (`aWalkaroundThickness / 2` in `SegmentHull` at `:186` versus `(aWalkaroundThickness + 1) / 2` in `ArcHull` at `:73` and in the dispatcher at `:481`). A one-unit clearance difference between a segment hull and a via hull is real and observable.
