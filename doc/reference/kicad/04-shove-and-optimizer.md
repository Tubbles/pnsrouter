# KiCad PNS: SHOVE and OPTIMIZER reference architecture note

Source: sparse checkout of KiCad master at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85` ("Pcbnew: fix formatting for unsigned: zu -> u"), read 2026-09-07. All `path:line` citations below are relative to `/home/Tubbles/dev/ref/kicad/`. The checkout is a shallow single-commit clone (`git log` returns exactly one commit), so no claim in this note is backed by upstream history; everything is read off the working tree.

Files read in full: `pcbnew/router/pns_shove.h`, `pcbnew/router/pns_shove.cpp`, `pcbnew/router/pns_optimizer.h`, `pcbnew/router/pns_optimizer.cpp`, `pcbnew/router/pns_utils.h`, `pcbnew/router/pns_utils.cpp`, `pcbnew/router/time_limit.h`, `pcbnew/router/time_limit.cpp`. Consulted for called APIs: `pns_node.h/.cpp`, `pns_line.h/.cpp`, `pns_via.h/.cpp`, `pns_walkaround.h`, `pns_routing_settings.h/.cpp`, `pns_item.h/.cpp`, `pns_segment.h`, `pns_joint.h`, `pns_itemset.h`, `pns_index.h`, `pns_topology.h`, `pns_debug_decorator.h`, `pns_algo_base.h/.cpp`, plus the three callers `pns_line_placer.cpp`, `pns_dragger.cpp`, `pns_multi_dragger.cpp`, `pns_diff_pair_placer.cpp` and `router_tool.cpp`.

## 0. Read this first: the API in the task brief is partly stale

The brief asks about `ShoveLines`, `ShoveMultiLines`, `SetInitialLine` and `SetDefaultDebugDecorator`. None of those symbols exist anywhere in `pcbnew/router/` at this commit; a full-tree grep for them returns nothing. They were the pre-2024 entry points. The current entry point is a two-phase `ClearHeads()` / `AddHeads(...)` / `Run()` protocol declared at `pcbnew/router/pns_shove.h:80-84`.

`ShoveDraggingVia` is declared at `pcbnew/router/pns_shove.h:86-87` but has no definition anywhere in the tree, and its only mention outside the header is a commented-out call at `pcbnew/router/pns_dragger.cpp:919`. It is a dead declaration. Do not port it.

`SetDefaultDebugDecorator` does not exist. Debug decoration is set through the shared base class: `ALGO_BASE::SetDebugDecorator` at `pcbnew/router/pns_algo_base.h:73-76`, read back via `Dbg()` at `pcbnew/router/pns_algo_base.h:78-81`. `SHOVE`'s constructor installs the router interface's decorator by default at `pcbnew/router/pns_shove.cpp:200`.

There is a second, stale copy of the `SHOVE_POLICY` enum sitting at namespace scope at the bottom of the implementation file, `pcbnew/router/pns_shove.cpp:2607-2615`. It shadows nothing (the real one is the class member enum at `pcbnew/router/pns_shove.h:59-69`) and it is missing `SHP_DONT_LOCK_ENDPOINTS` and `SHP_REVERSED`. `formatPolicy` at `pcbnew/router/pns_shove.cpp:2617-2636` reads the class enum but has two copy-paste bugs: it tests `SHP_WALK_FORWARD` twice (lines 2626 and 2628) and `SHP_IGNORE` twice (lines 2630 and 2632), so "walk-back" and "dont-optimize" are never printed correctly. Cosmetic, but do not copy the bug into the port.

## 1. SHOVE

### 1.1 Status and policy enums

`SHOVE_STATUS` at `pcbnew/router/pns_shove.h:50-57`:

```
SH_OK = 0          // collision resolved, keep going
SH_NULL            // never returned by any code path; only used as the initial value of `st` in shoveIteration (pns_shove.cpp:1637)
SH_INCOMPLETE      // gave up: shove failed, stack push failed, iteration/time limit hit
SH_HEAD_MODIFIED   // vestigial; only ever compared against, never assigned (pns_shove.cpp:2557)
SH_TRY_WALK        // "I cannot shove this, escalate to walkaround"
```

`SH_TRY_WALK` is produced in exactly three places: locked segments on the obstacle line (`pcbnew/router/pns_shove.cpp:650` and `:697`), the "shove vias disabled or via locked" guard in `pushOrShoveVia` (`pcbnew/router/pns_shove.cpp:1060-1061`), a locked segment on a via fanout line (`pcbnew/router/pns_shove.cpp:1092`), and an over-long arc extension (`pcbnew/router/pns_shove.cpp:710-711`). It is consumed only inside `shoveIteration`, where it converts the handler call into a call to `onCollidingSolid` (`pcbnew/router/pns_shove.cpp:1825-1826`, `:1838-1839`, `:1849-1850`).

`SHOVE_POLICY` at `pcbnew/router/pns_shove.h:59-69` is a bitmask attached per root line:

```
SHP_DEFAULT             = 0
SHP_SHOVE               = 0x01
SHP_WALK_FORWARD        = 0x02   // declared, never tested anywhere
SHP_WALK_BACK           = 0x04   // declared, never tested anywhere
SHP_IGNORE              = 0x08   // filters the item out of collision search
SHP_DONT_OPTIMIZE       = 0x10   // skip in runOptimizer
SHP_DONT_LOCK_ENDPOINTS = 0x20   // do not LockJoint the head endpoints
SHP_REVERSED            = 0x40   // the head's *first* point is the pusher, not the last
```

Only `SHP_IGNORE` (`pcbnew/router/pns_shove.cpp:1663`, `:1666`), `SHP_DONT_OPTIMIZE` (`:2107`), `SHP_DONT_LOCK_ENDPOINTS` (`:2489`) and `SHP_REVERSED` (`:253`) are actually read. `SHP_SHOVE` is the default set in the constructor (`pcbnew/router/pns_shove.cpp:209`) and is never tested; it is documentation.

### 1.2 Public API surface, as it exists

| Method | Declared | Defined | Effect |
| --- | --- | --- | --- |
| `SHOVE(NODE* world, ROUTER*)` | `pns_shove.h:77` | `pns_shove.cpp:193-210` | `m_root = m_currentNode = world`; policy defaults to `SHP_SHOVE`; installs the interface debug decorator |
| `ClearHeads()` | `pns_shove.h:80` | `pns_shove.cpp:2248-2251` | clears `m_headLines` |
| `AddHeads(const LINE&, int policy)` | `pns_shove.h:81` | `pns_shove.cpp:2254-2258` | pushes a `HEAD_LINE_ENTRY` and calls `SetShovePolicy(line, policy)` |
| `AddHeads(VIA_HANDLE, VECTOR2I newPos, int policy)` | `pns_shove.h:82` | `pns_shove.cpp:2261-2268` | pushes a via-drag head entry with `viaNewPos` |
| `Run()` | `pns_shove.h:84` | `pns_shove.cpp:2397-2605` | the whole algorithm; see 1.5 |
| `ShoveObstacleLine(cur, obstacle, out)` | `pns_shove.h:88` | `pns_shove.cpp:521-633` | public only because `DIFF_PAIR_PLACER` calls it directly (`pns_diff_pair_placer.cpp:251`) |
| `ForceClearance(bool, int)` | `pns_shove.h:91-97` | inline | sets `m_forceClearance` (or -1 to disable); short-circuits `getClearance` |
| `CurrentNode()` | `pns_shove.h:99` | `pns_shove.cpp:2132-2135` | `m_currentNode ? m_currentNode : m_root`. Note it does *not* consult `m_nodeStack` (the alternative is commented out at `:2134`) |
| `HeadsModified(int idx = -1)` | `pns_shove.h:101` | `pns_shove.cpp:2638-2644` | `idx < 0` gives the OR over all heads; otherwise per-head `geometryModified` |
| `GetModifiedHead(int)` | `pns_shove.h:102` | `pns_shove.cpp:2646-2649` | dereferences `m_headLines[i].newHead` without checking it is engaged |
| `GetModifiedHeadVia(int)` | `pns_shove.h:103` | `pns_shove.cpp:2651-2654` | same, for `theVia` |
| `AddLockedSpringbackNode(NODE*)` | `pns_shove.h:105` | `pns_shove.cpp:2138-2149` | pushes a `SPRINGBACK_TAG` with `m_locked = true` and *no* affected area / seq |
| `UnlockSpringbackNode(NODE*)` | `pns_shove.h:106` | `pns_shove.cpp:2200-2214` | linear scan, clears `m_locked` on the first match |
| `RewindSpringbackTo(NODE*)` | `pns_shove.h:107` | `pns_shove.cpp:2152-2183` | finds the tag, `aNode->KillChildren()`, erases the tail of the stack, resets `m_currentNode` |
| `RewindToLastLockedNode()` | `pns_shove.h:108` | `pns_shove.cpp:2186-2197` | pops until top is locked or size == 1 |
| `DisablePostShoveOptimizations(int mask)` | `pns_shove.h:109` | `pns_shove.cpp:2217-2220` | sets `m_optFlagDisableMask`, subtracted in `runOptimizer` (`:2086`) |
| `SetSpringbackDoNotTouchNode(const NODE*)` | `pns_shove.h:110` | `pns_shove.cpp:2223-2226` | pins a node so `reduceSpringback` will not delete it |
| `SetDefaultShovePolicy(int)` | `pns_shove.h:72` | `pns_shove.cpp:2229-2232` | policy applied to items with no root-line entry |
| `SetShovePolicy(const LINKED_ITEM*, int)` | `pns_shove.h:74` | `pns_shove.cpp:2235-2239` | `touchRootLine(item)->policy = p` |
| `SetShovePolicy(const LINE&, int)` | `pns_shove.h:75` | `pns_shove.cpp:2241-2245` | `touchRootLine(line)->policy = p` |

### 1.3 The three stacks and the root-line index

`SHOVE` state, `pcbnew/router/pns_shove.h:275-298`:

```
std::vector<SPRINGBACK_TAG> m_nodeStack;      // one entry per committed shove result (a NODE branch)
std::vector<LINE>           m_lineStack;      // work stack: lines whose collisions still need resolving
std::vector<LINE>           m_optimizerQueue; // every line touched during this Run(), fed to the optimizer
std::deque<HEAD_LINE_ENTRY> m_headLines;      // the caller's heads for this Run()
std::vector<std::unique_ptr<ROOT_LINE_ENTRY>> m_rootLineHistoryEntries; // owner
std::unordered_map<LINKED_ITEM::UNIQ_ID, ROOT_LINE_ENTRY*> m_rootLineHistory; // index
```

`m_lineStack` is a stack in name only. It is a `std::vector` popped from the back (`pcbnew/router/pns_shove.cpp:1635` reads `back()`, `:1695` and `:1513` pop the back), but `unwindLineStack` erases from the *middle* (`pcbnew/router/pns_shove.cpp:1421`) and `pushLineStack(aL, aKeepCurrentOnTop = true)` inserts one slot below the top (`pcbnew/router/pns_shove.cpp:1465-1468`). `onCollidingSolid` also reads `m_lineStack.front()` as "the last line", which is the *bottom* of the stack, not the top (`pcbnew/router/pns_shove.cpp:865`); that looks like a latent bug but it is the shipped behaviour.

`m_optimizerQueue` is not a queue either: it is a set-like vector kept deduplicated by link identity. `pushLineStack` first calls `pruneLineFromOptimizerQueue(aL)` and then appends (`pcbnew/router/pns_shove.cpp:1475-1476`), so a re-shoved line replaces its stale copy. `popLineStack` prunes but does *not* re-add (`pcbnew/router/pns_shove.cpp:1509-1514`). Crucially, when `shoveIteration` finds no obstacle it pops with a raw `m_lineStack.pop_back()` (`pcbnew/router/pns_shove.cpp:1695`) instead of `popLineStack()`, so a line that has come clean *stays* in the optimizer queue. That asymmetry is essential: it is how finished lines reach the optimizer.

`pruneLineFromOptimizerQueue` (`pcbnew/router/pns_shove.cpp:1482-1507`) removes any queue entry that shares a non-via link with `aLine`. Vias are excluded from the match (`:1493`) so a via link never causes an unrelated line to be dropped.

### 1.4 SPRINGBACK_TAG and the springback stack

`SPRINGBACK_TAG`, `pcbnew/router/pns_shove.h:194-210`:

```
int64_t                 m_length;        // set in the ctor to 0 and never written again
std::vector<VIA_HANDLE> m_draggedVias;   // one slot per head, snapshotted at push time
VECTOR2I                m_p;             // never written
NODE*                   m_node;          // the branched NODE this frame owns
OPT_BOX2I               m_affectedArea;  // union of this frame's and the previous frame's changed areas
int                     m_seq;           // 1-based monotone sequence number
bool                    m_locked;
```

`m_length` and `m_p` are dead fields; do not port them. `m_seq` is written at `pcbnew/router/pns_shove.cpp:1026` and never read anywhere.

Why the nodes are kept at all: each successful `Run()` leaves behind a whole `NODE` branch describing "the world after this shove". The router keeps a stack of them so that when the mouse moves back, the shove can be *undone* by discarding frames rather than recomputed. That is the springback.

`pushSpringback(NODE*, OPT_BOX2I)`, `pcbnew/router/pns_shove.cpp:983-1034`. Preconditions: `m_headLines` must be up to date (the doc comment at `:2568` says it must run after `reconstructHeads`, because it snapshots `head.theVia`). Mutates: nothing in any NODE; it only appends to `m_nodeStack`. For each head it copies the current `theVia` handle into `st.m_draggedVias[n]` (`:1006`), unions the incoming affected area with the previous frame's (`:1014-1024`), assigns `m_seq` (`:1026`), sets `m_locked = false` (`:1027`) and pushes. Always returns `true`.

`reduceSpringback(const ITEM_SET& aHeadSet)`, `pcbnew/router/pns_shove.cpp:924-976`. This is the pop side, and it is called exactly once, at the top of `Run()` (`:2429`). Loop invariant: while more than one frame remains, look at the top frame; stop immediately if it is the pinned `m_springbackDoNotTouchNode` (`:932-933`); otherwise ask `spTag.m_node->CheckColliding(aHeadSet)`. If the new heads do not collide with that frame's world and the frame is not locked, the frame is obsolete: `pruneRootLines(spTag.m_node)` (`:941`), `delete spTag.m_node` (`:944`), pop (`:945`). Otherwise stop. After the loop it restores each head's via handle from the surviving top frame's `m_draggedVias` and marks those heads `geometryModified` (`:958-973`). Returns the surviving node, or `m_root` if the stack emptied (`:953-954`).

Note that `delete spTag.m_node` destroys a `NODE`, which invalidates every `LINKED_ITEM*` that node owned. Any `LINE` still holding those links becomes dangling. `Run()` is explicitly aware of this on its failure path and clears both `m_lineStack` and `m_optimizerQueue` before `delete m_currentNode` (`pcbnew/router/pns_shove.cpp:2593-2600`, with a comment saying exactly that).

The locked-node concept exists for the line placer, not for shove itself. `LINE_PLACER` calls `AddLockedSpringbackNode` when it fixes a segment or commits a piece of route (`pcbnew/router/pns_line_placer.cpp:1626`, `:1737`, `:1750`), so that `reduceSpringback` can never roll the world back past a committed decision. Undo goes the other way: `pcbnew/router/pns_line_placer.cpp:1789-1790` calls `RewindSpringbackTo(m_currentNode)` then `UnlockSpringbackNode(m_currentNode)`, and `:1814-1815` calls `RewindToLastLockedNode()`. The dragger never locks anything; only the placer does.

`SetSpringbackDoNotTouchNode` exists because of one specific crash: `LINE_PLACER::Move` hands the shove an `m_endItem` that is owned by some node in the stack, and if springback deleted that node the placer would use freed memory. The pin is set at `pcbnew/router/pns_line_placer.cpp:938` when an end item exists and cleared at `:944` when it does not, and the comment at `pcbnew/router/pns_shove.cpp:930-931` records the original bug.

`RewindSpringbackTo` (`pcbnew/router/pns_shove.cpp:2152-2183`) does *not* delete the nodes it drops from the stack. It calls `aNode->KillChildren()` (`:2174`), which cascades into `NODE::releaseChildren` (`pcbnew/router/pns_node.cpp:1647-1650`) and destroys the descendants that way. The `SPRINGBACK_TAG`s themselves are then erased (`:2175`). `RewindToLastLockedNode` (`:2186-2197`) pops raw and leaks nothing only because those nodes are children of the surviving one and are killed later.

### 1.5 `Run()`: the whole flow

`pcbnew/router/pns_shove.cpp:2397-2605`.

```
Run():
  m_multiLineMode = false; m_headsModified = false
  m_lineStack.clear(); m_optimizerQueue.clear()                          # :2406-2407

  headSet = for each head: clone of the real VIA (found by handle) or of origHead   # :2413-2426
  parent  = reduceSpringback( headSet )                                  # :2429
  m_currentNode = parent->Branch()                                       # :2430
  m_currentNode->ClearRanks()                                            # :2431

  for headLineEntry in m_headLines:                                      # :2438
      m_currentNode->ClearRanks()                                        # :2440
      if headLineEntry.theVia:                                           # via-drag head
          viaToDrag = m_currentNode->FindViaByHandle(*theVia)            # :2444
          if !viaToDrag: st = SH_INCOMPLETE; break
          viaRoot = touchRootLine(viaToDrag); viaRoot->oldVia = viaToDrag  # :2452-2453
          st = pushOrShoveVia(viaToDrag, viaNewPos - pos, rank 0, dontUnwindStack=true)  # :2456
          if st != SH_OK: break
      else:                                                              # line head
          assert origHead->LinkCount() == 0                              # :2464
          m_currentNode->Add(*origHead, true)                            # :2465
          head = *origHead
          if head has no segments and no via: st = SH_INCOMPLETE; break  # :2481-2485
          if not (policy & SHP_DONT_LOCK_ENDPOINTS):                     # :2489
              LockJoint(head.CPoint(0)); if !EndsWithVia: LockJoint(head.CLastPoint())
          SetShovePolicy(head, policy)                                   # :2498
          head.SetRank(100000)                                           # :2501
          if head.EndsWithVia():
              headVia = Clone(head.Via()); headVia->SetRank(100000)      # :2505-2506
              origHead->LinkVia(headVia); head.LinkVia(headVia); node->Add(headVia)
          headRoot = touchRootLine(*origHead)                            # :2512
          headRoot->isHead = true
          headRoot->rootLine = copy of *origHead                         # :2514
          headRoot->policy = policy
          if head.EndsWithVia(): m_rootLineHistory[head via uid] = headRoot   # :2518
          if !pushLineStack(head): st = SH_INCOMPLETE; break             # :2540
      st = shoveMainLoop()                                               # :2547
      if st != SH_OK: break

  if st == SH_OK:
      runOptimizer(m_currentNode)                                        # :2563
      reconstructHeads(false)                                            # :2565
      removeHeads()                                                      # :2566
      pushSpringback(m_currentNode, m_affectedArea)                      # :2569
  else:
      for each head with prevVia: theVia = prevVia; geometryModified = m_headsModified = true  # :2575-2591
      m_lineStack.clear(); m_optimizerQueue.clear()                      # :2595-2596
      pruneRootLines(m_currentNode)                                      # :2598
      delete m_currentNode; m_currentNode = parent                       # :2600-2601
  return st
```

Two structural consequences worth calling out for the port. First, `shoveMainLoop()` is invoked once *per head* (`:2547`), inside the head loop, not once for all heads. Second, `m_currentNode->ClearRanks()` is called again at the top of every head iteration (`:2440`), which wipes the ranks assigned while shoving the previous head. Ranks therefore do not carry across heads; only the geometry in the node does.

`reconstructHeads(bool)` (`pcbnew/router/pns_shove.cpp:2296-2364`) walks `m_headLines` and, for each line head, looks up its root entry and copies `rootEntry->newLine` into `headEntry.newHead`, setting `geometryModified` by comparing the new line's geometry with the root line's (`:2316-2317`). For via heads it reads `rootEntry->newVia` (or `oldVia`) and rebuilds a `VIA_HANDLE` (`:2337-2357`). It contains three `assert`s that fire on missing root entries, which in a release build become silent nullptr derefs.

`removeHeads()` (`pcbnew/router/pns_shove.cpp:2279-2293`) asks `m_currentNode->GetUpdatedItems(removed, added)` and removes from the node every added item whose root entry has `isHead == true`. The heads are scaffolding: they are added to the node so that the shove can push against them, and deleted again before the node is handed back to the caller. There is also a free function `removeHead(NODE*, LINE&)` at `:2270-2277` that nothing calls.

`preShoveCleanup(LINE* old, LINE* new)` (`pcbnew/router/pns_shove.cpp:2368-2394`) runs `SHAPE_LINE_CHAIN::Simplify2` on the old line and, if the vertex count dropped, installs the simplified line via `replaceLine` and returns true. It is reached only through `assembleLine(..., aPreCleanup = true)` (`:222-228`), and the only caller that passes `true` is `onCollidingSegment` (`:643`).

### 1.6 `shoveMainLoop` and `shoveIteration`

`shoveMainLoop`, `pcbnew/router/pns_shove.cpp:1880-1923`:

```
shoveMainLoop():
    m_affectedArea = {}                                     # :1884
    iterLimit = Settings().ShoveIterationLimit()            # :1890  -> 250
    timeLimit = Settings().ShoveTimeLimit()                 # :1891  -> 1000 ms
    m_iter = 0                                              # :1893
    timeLimit.Restart()                                     # :1895
    if m_lineStack empty and m_draggedVia:                  # :1897
        pushLineStack( LINE(m_draggedVia) )                 # :1901
    while m_lineStack not empty:
        st = shoveIteration(m_iter)                         # :1909
        m_iter++
        if st == SH_INCOMPLETE or timeLimit.Expired() or m_iter >= iterLimit:  # :1913
            st = SH_INCOMPLETE; break
    return st
```

Both budgets are reset per call, and the call is per head, so a 5-head multi-drag gets 5 x 250 iterations and 5 x 1000 ms in the worst case. `m_draggedVia` is a member that is *never assigned* anywhere in the file (only initialised to nullptr at `pcbnew/router/pns_shove.cpp:203`), so the branch at `:1897-1902` is dead in this revision.

`TIME_LIMIT` is a thin wrapper over wall-clock milliseconds: `Expired()` is `wxGetLocalTimeMillis() - m_startTics >= m_limitMs` (`pcbnew/router/time_limit.cpp:87-90`), `Restart()` re-samples the clock (`:93-96`). Note the constructor calls `Restart()` before `m_startTics` is meaningfully used, and `ROUTING_SETTINGS::ShoveTimeLimit()` returns a fresh copy by value (`pcbnew/router/pns_routing_settings.cpp:121-124`), so the clock effectively starts at the copy, then again at `:1895`.

`shoveIteration(int aIter)`, `pcbnew/router/pns_shove.cpp:1633-1871`:

```
shoveIteration(iter):
    currentLine = m_lineStack.back()                                    # :1635  (a COPY, by value)
    for kind in [SOLID_T, VIA_T, SEGMENT_T, HOLE_T]:                    # :1650
        opts.m_kindMask = kind
        opts.m_filter   = drop items whose root policy (or the default policy) has SHP_IGNORE  # :1654-1676
        nearest = m_currentNode->NearestObstacle(&currentLine, opts)     # :1678
        if nearest: break
    if !nearest:
        m_lineStack.pop_back()                                          # :1695  (raw pop; stays in optimizer queue)
        return SH_OK
    viaFixup = fixupViaCollisions(&currentLine, *nearest)               # :1700
    ni = nearest->m_item
    unwindLineStack(ni)                                                 # :1715
    if !ni->OfKind(SOLID_T) and ni->Rank() >= 0 and ni->Rank() > currentLine.Rank():   # :1717
        # reverse collision: we hit something we already shoved
        VIA_T:     patchTadpoleVia; then onCollidingVia(currentLine, ni, rank ni+1)     # :1737
                   or onReverseCollidingVia(currentLine, ni)                            # :1743
        SEGMENT_T: revLine = assembleLine(ni); popLineStack(); unwindLineStack(revLine);
                   patchTadpoleVia; onCollidingVia(revLine,...) or onCollidingLine(revLine, currentLine, revLine.Rank()+1)  # :1776, :1779
                   pushLineStack(revLine)                                                # :1782
        ARC_T:     revLine = assembleLine(ni); popLineStack();
                   onCollidingLine(revLine, currentLine, revLine.Rank()-1); pushLineStack(revLine)  # :1800, :1804
        default:   assert(false)
    else:
        # forward collision: we hit something lower ranked, or a solid
        SEGMENT_T: onCollidingSegment; on SH_TRY_WALK -> onCollidingSolid   # :1823-1826
        ARC_T:     onCollidingArc;     on SH_TRY_WALK -> onCollidingSolid   # :1836-1839
        VIA_T:     onCollidingVia(currentLine, ni, currentLine.Rank()-1); on SH_TRY_WALK -> onCollidingSolid  # :1847-1850
        HOLE_T,
        SOLID_T:   onCollidingSolid                                          # :1859
    return st
```

The kind search order at `pcbnew/router/pns_shove.cpp:1650` is the priority ladder: solids first (immovable, so they must be walked around before anything else is disturbed), then vias, then segments, then holes. Arcs are not in the list; an arc obstacle only shows up because `NearestObstacle` returns it under the `SEGMENT_T` mask (`ITEM::PnsKind` masks are checked in the index) and then the `ni->Kind()` switch dispatches `ARC_T` (`:1793`, `:1833`).

Note that `currentLine` is a by-value copy of the stack top (`:1635`). Every handler that changes the current line therefore has to write the change back explicitly, either by `popLineStack()` + `pushLineStack(newLine)` as `onCollidingSolid` does (`:892-894`), or by relying on shared `LINKED_ITEM*` pointers. This copy-then-mutate-through-shared-pointers pattern is the single hardest thing to translate to Rust; see section 9.

### 1.7 Per-obstacle handlers

Throughout: "mutates" means "calls a NODE mutator", which is always `m_currentNode` unless stated otherwise.

#### `onCollidingSegment(LINE& aCurrent, SEGMENT* aObstacleSeg)` -- `pns_shove.cpp:639-683`

Preconditions: the obstacle is a `SEGMENT` in `m_currentNode`, ranked at or below `aCurrent`. Steps:

1. `obstacleLine = assembleLine(aObstacleSeg, &segIndex, aPreCleanup = true)` (`:643`). The pre-cleanup path can already mutate the node via `preShoveCleanup` -> `replaceLine`.
2. If the assembled line has any `MK_LOCKED` link, bail with `SH_TRY_WALK` (`:647-651`).
3. `shoveOK = ShoveObstacleLine(aCurrent, obstacleLine, shovedLine)` (`:653`).
4. On success: `shovedLine.SetRank(aCurrent.Rank() - 1)` (`:669`), `Simplify2` (`:670`), `unwindLineStack(&obstacleLine)` (`:672`), `replaceLine(obstacleLine, shovedLine, includeInChangedArea = true, allowRedundantSegments = false)` (`:674`), `pushLineStack(shovedLine)` (`:676`).
5. Returns `SH_OK`, or `SH_INCOMPLETE` if the shove failed or the push failed.

Note the rank is set on the *unlinked* line before `replaceLine`; the rank reaches the node because `NODE::Add(LINE&)` constructs each `SEGMENT` from the line and `SEGMENT`'s constructor copies `aParentLine.Rank()` (`pcbnew/router/pns_segment.h:70`).

#### `onCollidingArc(LINE& aCurrent, ARC* aObstacleArc)` -- `pns_shove.cpp:689-732`

Same shape as `onCollidingSegment` but without pre-cleanup (`:692`), plus a length guard: if the shoved chain is more than `extensionWalkThreshold = 1.0` (that is, 100 percent) longer than the original, give up with `SH_TRY_WALK` (`:701-711`). Rank is `aCurrent.Rank() - 1` (`:723`).

Behavioural quirk worth reproducing deliberately or not at all: the function returns `SH_OK` at `:731` **even when `shoveOK` is false**, because the `if( shoveOK )` block at `:720-729` only guards the mutation. A failed arc shove therefore reports success and the iteration loop moves on with the collision unresolved. `onCollidingSegment` returns `SH_INCOMPLETE` in the same situation (`:682`). This asymmetry looks like a bug.

#### `onCollidingLine(LINE& aCurrent, LINE& aObstacle, int aNextRank)` -- `pns_shove.cpp:738-770`

The already-assembled variant, used by the reverse-collision paths. `ShoveObstacleLine`, then `replaceLine(aObstacle, shovedLine, true, false)` (`:759`), then `shovedLine.SetRank(aNextRank)` (`:761`) -- note rank is set *after* the replace here, which still works because the line is now linked and `LINE::SetRank` propagates to links (`pcbnew/router/pns_line.cpp:1439-1446`). Then `pushLineStack`. `SH_OK` on success, `SH_INCOMPLETE` otherwise.

#### `onCollidingSolid(LINE& aCurrent, ITEM* aObstacle, OBSTACLE& aObstacleInfo)` -- `pns_shove.cpp:776-898`

This is the escape hatch: solids cannot be shoved, so the *current* line is re-routed around them.

1. If `aCurrent` ends with a via, look up the real `VIA` at the via joint and, if that via collides with the obstacle, delegate to `onCollidingVia(aObstacle, via, info, aObstacle->Rank() - 1)` (`:780-801`). Yes, the roles are swapped: the solid becomes the "current" item.
2. Build a cluster of items topologically attached to the obstacle: `TOPOLOGY::AssembleCluster(aObstacle, aCurrent.Layers().Start(), 10.0)` (`:804`). The `10.0` is an area expansion limit (`pcbnew/router/pns_topology.h:103`).
3. Configure a `WALKAROUND` restricted to that cluster, solids-not-only, single policy `WP_SHORTEST`, iteration limit `Settings().WalkaroundIterationLimit()` (default 40) (`:813-818`).
4. Two attempts (`:827`). On attempt 0 the target rank is `currentRank + 10000` unless `Settings().JumpOverObstacles()` is on; on attempt 1 it is `currentRank - 1` (`:829-832`). The `+10000` is the "jump the queue" trick: a line that had to walk around an immovable object is promoted far above everything else so that subsequent iterations treat it as a pusher rather than a pushee.
5. `walkaround.Route(aCurrent)`; require `ST_DONE` (`:834-837`); clear links, unmark, `Simplify2`, reject if it has loops (`:841-846`).
6. Accept only if the walked line does not collide with `m_lineStack.front()`, or if it *does* collide but `ShoveObstacleLine(walkaroundLine, lastLine, dummy)` succeeds as a feasibility probe (`:863-882`). If `m_lineStack` is empty, `success` is never set and the function returns `SH_INCOMPLETE` (`:885-886`).
7. `replaceLine(aCurrent, walkaroundLine, true, false)` (`:888`), `SetRank(nextRank)` (`:889`), `popLineStack()` (`:892`), `pushLineStack(walkaroundLine)` (`:894`).

Mutates `m_currentNode` (the current line is replaced in place). Returns `SH_OK` or `SH_INCOMPLETE`.

#### `onCollidingVia(ITEM* aCurrent, VIA* aObstacleVia, OBSTACLE& aObstacleInfo, int aNextRank)` -- `pns_shove.cpp:1175-1261`

Computes a minimum translation vector and hands it to `pushOrShoveVia`.

1. `clearance = getClearance(aCurrent, aObstacleVia)` (`:1179`).
2. If `aCurrent` is a `LINE`: make a scratch copy `vtmp` of the obstacle via; if `aObstacleInfo.m_maxFanoutWidth` exceeds the via diameter, inflate `vtmp` to that width (`:1194-1198`). This is how `fixupViaCollisions` feeds back. Then `lineCollision = vtmp.Shape(layer)->Collide(currentLine->Shape(-1), clearance + width/2, &mtvLine)` (`:1211-1213`). If the current line ends with a via, additionally test via-to-via over `RelevantShapeLayers` and keep the largest per-layer MTV in `mtvVia` (`:1216-1231`).
3. If `aCurrent` is a `SOLID`: `solidCollision` with `mtvSolid` (`:1233-1243`).
4. Priority of the MTV: via beats line beats solid (`:1246-1253`), each negated ("fixme: we may have a sign issue in Collide(CIRCLE, LINE_CHAIN)" at `:1245`), and then `pushOrShoveVia(aObstacleVia, -mtv, aNextRank)` is called with the vector negated *again* (`:1255`). Net effect: the via is pushed by `+mtv` of whichever collision won.

Returns whatever `pushOrShoveVia` returned.

#### `pushOrShoveVia(VIA* aVia, const VECTOR2I& aForce, int aNewRank, bool aDontUnwindStack)` -- `pns_shove.cpp:1041-1168`

The core via mover.

1. Zero force is a no-op returning `SH_OK` (`:1051-1052`).
2. Requires a joint at the via centre; missing joint returns `SH_INCOMPLETE` with a "weird" log (`:1054-1058`).
3. `Settings().ShoveVias() == false || aVia->IsLocked()` returns `SH_TRY_WALK` (`:1060-1061`); a locked *joint* returns `SH_INCOMPLETE` (`:1063-1064`).
4. Anti-snap loop: while the target position lands on an existing joint, nudge by `aForce.Resize(2)` (that is, 2 internal units in the force direction) and retry (`:1067-1075`). Unbounded loop in principle.
5. Clone the via to the new position, carrying the marker (`:1077-1079`).
6. Fanout: for every `SEGMENT`/`ARC` at the joint, assemble its line, bail with `SH_TRY_WALK` if it has locked segments (`:1091-1092`), reverse it if the via is at index 0 so the via end is always last (`:1096-1097`), copy it, `DragCorner(p0_pushed, index of p0)` and `Simplify2` (`:1099-1102`). The `assert` at `:1094` requires the via to be at one end of the line.
7. `pushedVia->SetRank(aNewRank)` (`:1107`); optional `unwindLineStack(aVia)` (`:1115-1116`); `replaceItems(aVia, std::move(pushedVia))` (`:1118`).
8. If there was no fanout at all (a stitching via), push a proxy `LINE` carrying only the via so the main loop does not forget about it (`:1120-1126`).
9. For each dragged line pair: unwind the old, `replaceLine(old, new, true, allowRedundantSegments = true)`, `LinkVia(v2)`, unwind the new, `SetRank(aNewRank)`, write `rootEntry->newLine` ("fixme: it's inelegant", `:1150`), `pushLineStack(new)` (`:1129-1157`). A dragged line that collapsed to zero segments is simply removed from the node (`:1160`).

Mutates `m_currentNode` heavily: one via replacement plus one line replacement per fanout branch. Returns `SH_OK` unless a push failed (`SH_INCOMPLETE`) or a lock was hit (`SH_TRY_WALK`).

#### `onReverseCollidingVia(LINE& aCurrent, VIA* aObstacleVia, OBSTACLE&)` -- `pns_shove.cpp:1267-1387`

Reached when the current line runs into a via that already has a *higher* rank, that is, a via we shoved earlier in this same run. The rule is "the earlier decision wins", so the current line yields.

1. If `aCurrent` ends with a via, test via-to-via across `RelevantShapeLayers`; a real collision escalates to `onCollidingVia(&aCurrent, aObstacleVia, info, aCurrent.Rank() - 1)` (`:1271-1303`). Along the way it fetches the obstacle via's hull from the rule resolver's `HullCache` and logs an inside/outside test purely for the debug view (`:1282-1288`).
2. Otherwise: strip the via from a working copy `cur` (`:1312`), unwind the current line from the stacks (`:1313`), and then for every segment/arc attached to the obstacle via's joint on an overlapping layer, build a synthetic pusher line `head = thatLine + the obstacle via` and call `ShoveObstacleLine(head, cur, shoved)`; each success feeds `cur.SetShape(shoved.CLine())` so the shoves compose (`:1315-1346`).
3. If the via had no attached lines (`n == 0`), build a degenerate head containing only the via and shove against that (`:1348-1366`).
4. Re-append the current line's own via if it had one (`:1368-1369`), then `unwindLineStack(&aCurrent)` again, `replaceLine(aCurrent, shoved, true, false)` (`:1379`), `pushLineStack(shoved)` (`:1381`), and finally `shoved.SetRank(currentRank)` (`:1384`) with the rank captured *before* the replace (`:1377`).

Returns `SH_OK` or `SH_INCOMPLETE`. Note the rank is set after the push, so the `LINE` copy sitting in `m_lineStack` has a stale `m_rank` member; it reads correctly anyway because `LINE::Rank()` prefers the minimum over links when linked (`pcbnew/router/pns_line.cpp:1449-1466`).

#### `fixupViaCollisions(const LINE* aCurrent, OBSTACLE& obs)` -- `pns_shove.cpp:1517-1597`

A pre-pass run on every iteration (`:1700`) whose whole job is to keep the force-propagation assumption ("track ends never move on their own, they are only dragged by vias") true.

Case A, the obstacle is a via (`:1527-1560`): compute the widest track attached to it. If that width is at least the via diameter on the current layer, set `obs.m_maxFanoutWidth = maxw + 1` and return true. `onCollidingVia` then inflates its scratch via to that width so the computed MTV is large enough to also clear the fat tracks.

Case B, the obstacle is a segment with a via on either end (`:1564-1595`): if the via is *narrower* than the segment, build a test via inflated to the segment width and check whether that would collide with the current line. If so, rewrite `obs.m_item` to point at the *via* instead of the segment and set `obs.m_maxFanoutWidth = s->Width() + 1`. The iteration then shoves the via and lets the fanout drag the segment, rather than trying to shove the segment directly.

The function reads `ja->Via()` and `jb->Via()` at `:1573` without checking that `FindJoint` returned non-null at `:1570-1571`. In practice both joints must exist for a segment in the index.

#### `patchTadpoleVia(ITEM* nearest, LINE& current)` -- `pns_shove.cpp:1599-1628`

Called on both reverse-collision branches (`:1727`, `:1762`). If the current line's last point sits on a joint that carries a via, the line does not already end with a via, and that via is currently colliding with something, then link the via into the line (`:1622-1625`). This makes a "tadpole" (a track with a via head that got detached during earlier shoving) behave as a unit again. It always returns false; the return value is ignored at both call sites.

#### `unwindLineStack` -- `pns_shove.cpp:1390-1437` (by link) and `:1440-1453` (by item)

Removes from `m_lineStack` (and `m_optimizerQueue`) any line that references a link which is about to be invalidated. The subtlety at `:1397-1418`, spelled out in a note-to-self comment, is that a line that ends with a via must *not* be dropped outright: it is degraded to a via-only stub (`ClearLinks`, `Line().Clear()`, `LinkVia(via)`) so the via keeps participating in cross-layer collision checks. Only non-via matches are erased outright (`:1421`). The `m_optimizerQueue` pass at `:1430-1436` erases unconditionally for non-via links.

#### `pushLineStack` / `popLineStack` -- `pns_shove.cpp:1456-1479` / `:1509-1514`

`pushLineStack` refuses (returns false) any line that has segments but no links (`:1458-1463`); that guard is what turns "we produced a LINE that was never added to the NODE" into `SH_INCOMPLETE` rather than a later crash. `aKeepCurrentOnTop` inserts one below the top. Every push also refreshes the optimizer queue entry.

#### `assembleLine` -- `pns_shove.cpp:218-231`

Thin wrapper over `NODE::AssembleLine(seg, &index, aStopAtLockedJoints = true)` (`pcbnew/router/pns_node.cpp:1132`), plus the optional `preShoveCleanup`. The `aStopAtLockedJoints = true` is important: it is how the endpoint locks placed in `Run()` (`:2492`, `:2495`) actually restrain the algorithm. A locked joint terminates line assembly, so the assembled obstacle line never extends past a pinned endpoint, and `ShoveObstacleLine`'s "endpoints must not move" checks (`:459`, `:312-316`) then hold trivially.

#### `replaceItems` / `replaceLine` -- `pns_shove.cpp:52-80` / `:83-166`

`replaceItems(old, new)` merges the changed area into `m_affectedArea` (`:54-57`), and for vias touches the root-line entry, records `re->newVia`, then re-indexes the entry under the *new* via's uid (`:62-79`). `m_currentNode->Replace` is `Remove` + `add` (`pcbnew/router/pns_node.cpp:951-955`).

`replaceLine(old, new, aIncludeInChangedArea, aAllowRedundantSegments, aNode)` is the workhorse and also the root-line bookkeeper:

1. Optionally merge the changed area and emit a `shove-changed-area` debug rect (`:85-97`).
2. If the old line ends with a via, unlink the via link first so the via survives the replace (`:99-112`).
3. Look for an existing root entry under any of the old line's link uids (`:124-134`). If found, reuse it; if not, `allocRootLine(clone of old)` and index it under every old link uid (`:137-148`).
4. `aNode ? aNode->Replace(...) : m_currentNode->Replace(...)` (`:151-154`). The comment at `:150` warns that `Replace` invalidates `Links()`.
5. Re-point every *new* link uid at the same root entry (`:157-161`) and store `rootEntry->newLine = aNew` (`:163`).

Returns the root entry, so callers can post-patch `newLine` (as `pushOrShoveVia` does at `:1150`).

#### `sanityCheck` -- `pns_shove.cpp:186-190`

Two `assert`s that the first and last points are unchanged. It has no callers in this revision; a grep for `sanityCheck(` finds only the declaration (`pns_shove.h:269`) and the definition. It documents the invariant the shove is supposed to preserve, which is enforced for real inside `shoveLineToHullSet` at `:459-464` and `shoveLineFromLoneVia` at `:312-316`.

#### `checkShoveDirection` -- `pns_shove.cpp:243-268`

The heuristic that decides whether a candidate shoved line went the right way. It builds a closed region from the obstacle line plus the reversed shoved line, and asks whether the *pusher's reference point* ends up inside it (`:258-262`). Inside means the shoved line wrapped around the pusher, which is wrong, so the function returns `!inside`.

The reference point is the pusher's first vertex by default (`:249`), the via centre for a lone via (`:249`), or the pusher's *last* vertex if the root line's policy carries `SHP_REVERSED` (`:253-256`). The three-comment archaeology block at `:234-243` records that this is admittedly a heuristic: there is no orientation for an open curve, so an external hint (`SHP_REVERSED`) had to be added for corner dragging. Any Rust port needs the same hint plumbed through from the dragger.

## 2. The hull-based pushing geometry

### 2.1 What a hull is

A hull is a closed `SHAPE_LINE_CHAIN` that bounds "everywhere the walking line's *centreline* must not go". It is therefore built at `clearance + walkaroundThickness / 2`, where `walkaroundThickness` is the width of the line being pushed. Once you re-walk the pushed line's centreline around that hull, the pushed line's *edge* sits exactly at the required clearance. The convention is stated in the comment at `pcbnew/router/pns_shove.cpp:565-567`.

`ITEM::Hull(clearance, walkaroundThickness, layer)` is the virtual entry point (`pcbnew/router/pns_item.h:164`). Implementations:

- `SEGMENT::Hull` -> `SegmentHull` (`pcbnew/router/pns_line.cpp:668-677`, `pcbnew/router/pns_utils.cpp:181-286`)
- `ARC::Hull` -> `ArcHull` (`pcbnew/router/pns_arc.cpp:28-31`, `pcbnew/router/pns_utils.cpp:71-154`)
- `VIA::Hull` -> `OctagonalHull` around the via square (`pcbnew/router/pns_via.cpp:235-250`)
- `SOLID::Hull` -> `BuildHullForPrimitiveShape`, or a unioned `SHAPE_POLY_SET` outline for compound shapes (`pcbnew/router/pns_solid.cpp:39-71`)

`OctagonalHull(p0, size, clearance, chamfer)` (`pcbnew/router/pns_utils.cpp:40-68`) is the primitive: an axis-aligned box inflated by `clearance` with its four corners cut by `chamfer`. A `chamfer` of 0 degenerates to a rectangle. For circles the chamfer is `2 * (1 - 1/sqrt(2)) * (r + cl)` (`pcbnew/router/pns_utils.cpp:501`) and for vias `(2 * cl + width) * (1 - 1/sqrt(2))` (`pcbnew/router/pns_via.cpp:249`), both of which make the octagon equilateral.

`SegmentHull` (`pcbnew/router/pns_utils.cpp:181-286`) is an eight-vertex capsule approximation: half-width plus clearance perpendicular (`dr`), and a `x = 2/(1+sqrt(2)) * d` chamfer along the axis (`:187-190`). It contains a "kinky segment" fixup: for segments shorter than `kinkThreshold = clearance / 10` (`:184`) whose direction is not a clean 45-degree multiple, the endpoint is snapped to the nearest axis or diagonal and the clearance is bumped by 1 or 2 units (`:207-248`). Zero-length segments fall back to an octagonal hull around a square of the segment width (`:250-260`). The last step reverses the chain if needed so **the hull outline is always clockwise** (`:281-285`); `ArcHull` does the same (`pcbnew/router/pns_utils.cpp:149-153`). That orientation convention is what makes the `clockwise` flag in `LINE::Walkaround` meaningful.

`ArcHull` (`pcbnew/router/pns_utils.cpp:71-154`) polylines the arc at `ARC_LOW_DEF`, offsets each segment by `d = width/2 + cl + SHAPE_ARC::DefaultAccuracyForPCB()` on both sides, and intersects consecutive offset segments to get the vertex normals. Arcs whose central angle exceeds 180 degrees and whose chord is shorter than the clearance are treated as full circles (`:76-83`).

`ConvexHull(SHAPE_SIMPLE, clearance)` (`pcbnew/router/pns_utils.cpp:300-353`) builds an octagon by taking the bounding box's four sides and four 45-degree diagonals, then sliding each diagonal inward until it is exactly `clearance` from the nearest polygon vertex (`MoveDiagonal`, `:289-297`), and intersecting the eight lines.

`HULL_MARGIN = 10` is declared at `pcbnew/router/pns_utils.h:138` and is **never used**. The live constant is the macro `PNS_HULL_MARGIN 10` at `pcbnew/router/pns_line.h:45`, used at `pcbnew/router/pns_diff_pair_placer.cpp:247` and `pcbnew/router/pns_node.cpp:1338`, `:1348`. Two definitions of the same number in two headers; keep one in the port.

### 2.2 `HullCache`

`RULE_RESOLVER::HullCache(item, clearance, walkaroundThickness, layer)` is virtual, and the base implementation at `pcbnew/router/pns_node.h:176-182` is a trap: it writes into a function-local `static SHAPE_LINE_CHAIN` and returns a reference to it. That is not reentrant and not thread safe. The real implementation, `PNS_PCBNEW_RULE_RESOLVER::HullCache` at `pcbnew/router/pns_kicad_iface.cpp:832-848`, keys a hash map on `{item pointer, clearance, walkaroundThickness, layer}` and returns a reference into the map. Both the comment at `pcbnew/router/pns_node.cpp:346-348` and the parallel loop below it are built around "HullCache is not thread-safe, so populate it sequentially first, then copy the hulls into owned values before going parallel".

Note the cache key includes the raw `ITEM*` pointer (`pcbnew/router/pns_kicad_iface.cpp:837`), so it must be invalidated whenever items are freed; that is what `ClearCacheForItems` at `pcbnew/router/pns_node.cpp:127` and `:1610` is for.

Inside shove, `HullCache` is called in exactly one place, the via-to-via debug/diagnostic path of `onReverseCollidingVia` (`pcbnew/router/pns_shove.cpp:1282-1283`). Everywhere else shove builds hulls directly with `SEGMENT::Hull` / `VIA::Hull` and does not cache them.

### 2.3 `ShoveObstacleLine`: building the hull set

`pcbnew/router/pns_shove.cpp:521-633`.

```
ShoveObstacleLine(curLine, obstacleLine, out):
    jtStart/jtEnd = joints at obstacleLine's endpoints                 # :531-535
    voeStart/voeEnd = "is there a via on that endpoint"                # :537-540
    out.ClearLinks()                                                   # :542
    strip the via off a working copy of obstacleLine, remember it      # :548-552

    if curLine ends with a via AND (layers do not overlap OR curLine has no segments):
        return shoveLineFromLoneVia(curLine, obstacleLine, out)         # :558-562

    clearance = getClearance(&curLine, &obstacleLine)                   # :570
    for attempt in 0..2:                                                # :578
        hulls = []
        for i in 0..curLine.SegmentCount()-1:
            if curLine segment i is an arc:
                clearance += round(SHAPE_ARC::DefaultAccuracyForPCB())  # :593  (note: accumulates!)
            hulls.push( SEGMENT(curLine, curLine.CSegment(i))
                        .Hull(clearance + extraHullExpansion,
                              obstacleLine.Width(), obstacleLine.Layer()) )   # :596
        if curLine ends with a via:
            viaClearance = max(getClearance(via, obstacleLine),
                               holeClearance + drill/2 - diameter/2)    # :604-612
            hulls.push( curLine.Via().Hull(viaClearance, obstacleLine.Width(), obstacleLayer) )  # :614
        permitMovingStart = (attempt >= 2) && !voeStart                 # :617
        permitMovingEnd   = (attempt >= 2) && !voeEnd                   # :618
        if shoveLineToHullSet(curLine, obstacleLine, out, hulls,
                              permitMovingStart, permitMovingEnd):
            re-append the obstacle's own via if it had one              # :622-623
            return true
        extraHullExpansion += 1000                                      # :628  cHullFailureExpansionFactor
    return false
```

Three details matter for a port. The hull set is ordered along the pusher: one hull per segment of `curLine` in index order, and the via hull last (`:614`). The arc clearance bump at `:593` is inside the segment loop and mutates the shared `clearance` variable, so a line with several arcs accumulates the bump for every subsequent segment; that is almost certainly unintended but it is the behaviour. The retry ladder is: attempt 0 plain, attempt 1 with hulls inflated by 1000 IU, attempt 2 with hulls inflated by 2000 IU *and* permission to move the obstacle's endpoints, but only endpoints that do not carry a via.

The via-hole clearance formula at `:609-612` (and again at `pcbnew/router/pns_shove.cpp:289-290`) says: if hole clearance plus hole radius exceeds pad clearance plus pad radius, then the hole is the binding constraint, so rewrite the pad clearance to `holeClearance + drill/2 - diameter/2` so that a hull built around the *pad* still clears the *hole*.

### 2.4 `shoveLineToHullSet`: which side, and the clockwise decision

`pcbnew/router/pns_shove.cpp:328-513`. This is where the actual "which way around" choice happens, and it is a brute-force search over four combinations:

```
for attempt in 0..3:                                          # :338
    invertTraversal = attempt >= 2       # process hulls back-to-front   # :340
    clockwise       = attempt % 2        # walk direction                # :341
    l    = obstacleLine
    path = l.CLine()
    if endpoint adjustment permitted and l has at least one segment:      # :347
        for each endpoint, find the nearest point on any hull within
        c_ENDPOINT_ON_HULL_THRESHOLD = 1000 IU; if found, append/insert
        that point so the endpoint sits exactly on the hull              # :349-401
    for each hull in traversal order:                                     # :406
        if !l.Walkaround(hull, path, clockwise): fail this attempt        # :414-423
        path.Simplify2(); l.SetShape(path)                                # :425-426
    reject if the result does not share its first/last point with the
      original obstacle line                                              # :432-464
    reject if !checkShoveDirection(curLine, obstacleLine, l)               # :466-472
    reject if path.SelfIntersecting()                                      # :474-479
    reject if l collides with curLine in m_currentNode                     # :481, :496-501
    accept: out.SetShape(l.CLine()); return true                           # :503-507
return false
```

So there is no analytic "left or right" decision. The algorithm tries clockwise, counter-clockwise, and both again with the hull list reversed, and takes the first candidate that survives four independent validity tests. `LINE::Walkaround(hull, path, aCw)` (`pcbnew/router/pns_line.cpp:297+`) does the graph search: it refuses outright if the path's first point is strictly inside the hull (`pcbnew/router/pns_line.cpp:308-315`), builds a directed graph of path and hull vertices classified inside/outside/on-edge, and BFS-walks it in the requested winding.

There is a subtle wart at `:466-472`: when the direction check fails, the loop nevertheless writes the rejected line into `aResultLine` before `continue`. So a caller that ignores the return value can read a direction-rejected shape. `onCollidingArc` is exactly such a caller (see 1.7).

The hull-order inversion for `attempt >= 2` exists because walking around hull A then hull B is not the same as B then A when the hulls overlap; a chain of overlapping hulls (a via grid, a SOIC pad row) can be walkable in one order and not the other.

### 2.5 The "multiple hulls" case for solids

Solids are never shoved. Two separate mechanisms handle them.

The hull side: `SOLID::Hull` (`pcbnew/router/pns_solid.cpp:39-71`) handles a `SH_COMPOUND` shape by building one hull per sub-shape, adding them all as outlines to a `SHAPE_POLY_SET`, calling `Simplify()`, and returning outline 0 (`:55-64`). A single-shape compound short-circuits to the primitive path (`:48-52`). So "multiple hulls" for a solid collapses to one merged outline, not a hull set.

The routing side: `onCollidingSolid` does not use hulls at all, it delegates to `WALKAROUND` restricted to a `TOPOLOGY::CLUSTER` around the solid (`pcbnew/router/pns_shove.cpp:803-818`). The cluster exists so that a pad in a chain of touching pads is walked around as a group rather than one pad at a time.

`BuildHullForPrimitiveShape` (`pcbnew/router/pns_utils.cpp:478-541`) is the primitive dispatcher: rect -> octagon with zero chamfer, circle -> equilateral octagon, segment -> `SegmentHull`, arc -> `ArcHull`, simple polygon -> `ConvexHull`, ellipse -> octagon around the bounding box. Anything else trips a `wxFAIL_MSG` and returns an empty chain (`:531-540`), which downstream would silently mean "no obstacle".

### 2.6 Vias

Three distinct via interactions:

**Via as pusher, line as pushee.** `shoveLineFromLoneVia` (`pcbnew/router/pns_shove.cpp:278-322`) is used when the pusher line has no segments on the obstacle's layer, so only its via matters. It builds a single via hull (`:292`), walks the obstacle line around it clockwise *and* counter-clockwise (`:296-300`), picks clockwise unless `checkShoveDirection` rejects it (`:302-307`), and then applies the same endpoint-preservation and collision checks as the hull-set path (`:309-319`).

**Via as pushee.** `onCollidingVia` -> `pushOrShoveVia`. The via is moved by an MTV and every track attached to its joint is dragged along by `LINE::DragCorner` (`pcbnew/router/pns_shove.cpp:1101`). That fanout drag is the reason `fixupViaCollisions` exists: if the attached tracks are wider than the via, the via's own MTV is not enough to clear them.

**Via versus via.** Handled per shape layer over `VIA::RelevantShapeLayers` in `onCollidingVia` (`pcbnew/router/pns_shove.cpp:1222-1230`) and in `onReverseCollidingVia` (`:1292-1297`), keeping the largest per-layer MTV. Via collisions take priority over line collisions when both are present (`:1246-1249`).

**The "shove vias" setting.** `ROUTING_SETTINGS::ShoveVias()` (`pcbnew/router/pns_routing_settings.h:76`, default true at `pcbnew/router/pns_routing_settings.cpp:41`) is read in exactly one place: the guard at `pcbnew/router/pns_shove.cpp:1060-1061`. With vias disabled, every via collision degrades to `SH_TRY_WALK` and therefore to `onCollidingSolid`, that is, the current line walks around the via instead.

## 3. Head versus obstacle priority: what stops the ping-pong

### 3.1 Rank

`ITEM::Rank()` / `SetRank()` are plain accessors on `int m_rank`, default -1 (`pcbnew/router/pns_item.h:125`, `:265-266`, `:328`). `LINE` overrides both: `SetRank` writes the member *and* propagates to every link (`pcbnew/router/pns_line.cpp:1439-1446`); `Rank()` returns the *minimum* rank over links when the line is linked, and the member otherwise, mapping `INT_MAX` back to -1 (`pcbnew/router/pns_line.cpp:1449-1466`). Rank also flows into freshly constructed segments: `SEGMENT(const LINE&, const SEG&)` copies `aParentLine.Rank()` (`pcbnew/router/pns_segment.h:70`), which is how a rank set on an unlinked `LINE` survives `NODE::Add`.

Rank values actually used:

- `-1`: untouched. `NODE::ClearRanks` sets every item in the index to -1 (`pcbnew/router/pns_node.cpp:1682-1689`), and `NODE::Commit`-style item reuse does the same at `:1637`.
- `0`: a via being dragged as a head (`pcbnew/router/pns_shove.cpp:2456`).
- `100000`: a line head and its via (`pcbnew/router/pns_shove.cpp:2501`, `:2506`).
- `currentRank - 1`: the normal "I pushed you, so you are now below me" assignment (`:669`, `:723`, `:830`, `:1847`).
- `currentRank + 10000`: the walkaround promotion in `onCollidingSolid` when `JumpOverObstacles()` is off (`:832`).
- `rank + 1`: the reverse-collision assignments (`:1737`, `:1776`, `:1779`).

The anti-ping-pong rule is the single test at `pcbnew/router/pns_shove.cpp:1717`:

```
if( !ni->OfKind( ITEM::SOLID_T ) && ni->Rank() >= 0 && ni->Rank() > currentLine.Rank() )
```

If the obstacle outranks the current line, we do not shove the obstacle again. Instead the roles are reversed: the *current* line becomes the pushee. Because every forward shove assigns `rank - 1`, ranks strictly decrease as the shove wave propagates outward from the head at 100000, and a cycle would require some item to outrank its own pusher, which the assignment rules forbid. Solids are excluded from the test because they can never be shoved at all; they always take the forward branch and end up in `onCollidingSolid`.

That argument is not a proof of termination, which is why the hard `iterLimit` (250) and `timeLimit` (1000 ms) at `pcbnew/router/pns_shove.cpp:1890-1891` exist as a backstop.

`m_forceClearance` (`pcbnew/router/pns_shove.h:91-97`, `:291`) is orthogonal to rank: when non-negative it makes `getClearance` return a constant regardless of the DRC engine (`pcbnew/router/pns_shove.cpp:169-183`). Its only user is `DIFF_PAIR_PLACER::attemptWalk`, which sets it to `gap - 2 * PNS_HULL_MARGIN` so the two members of a pair are shoved to exactly the coupled gap (`pcbnew/router/pns_diff_pair_placer.cpp:247`).

### 3.2 Root lines and root-line history

A "root line" is the pre-shove shape of a track, kept so that (a) the optimizer can be told "do not make this line worse than it originally was", and (b) `reconstructHeads` can tell whether a head actually changed. The comment at `pcbnew/router/pns_shove.cpp:117-123` states the first purpose explicitly.

The data structure is two-level, and the header comment at `pcbnew/router/pns_shove.h:280` explains why: *"UID entries may alias the same history entry, so ownership lives outside the index."* Every link uid of a line maps to the *same* `ROOT_LINE_ENTRY*`, and when a line is replaced the new links are pointed at the same entry (`pcbnew/router/pns_shove.cpp:157-161`). If the map owned the entries, the aliasing would double-free. So ownership is a separate `std::vector<std::unique_ptr<ROOT_LINE_ENTRY>>` (`pns_shove.h:281`) filled only by `allocRootLine` (`pns_shove.cpp:1942-1948`), and `m_rootLineHistory` holds raw pointers (`pns_shove.h:282`).

I cannot verify the upstream commit message "We need to own PNS shove root-line history" from this checkout: `git log` here has exactly one commit. The code shape is exactly what that message would describe, and the header comment is the in-tree evidence.

`ROOT_LINE_ENTRY` (`pcbnew/router/pns_shove.h:118-131`):

```
std::unique_ptr<LINE> rootLine;   // the pre-shove shape; may be null for via-only entries
VIA*  oldVia   = nullptr;         // set by Run() for via heads (:2453)
VIA*  newVia   = nullptr;         // set by replaceItems when a via moves (:67)
std::optional<LINE> newLine;      // set by replaceLine (:163) and patched by pushOrShoveVia (:1150)
int   policy   = SHP_DEFAULT;
bool  isHead   = false;           // set in Run() (:2513); read by runOptimizer (:2109) and removeHeads (:2288)
```

Accessors: `findRootLine(LINE)` scans the line's links for the first uid present in the map (`pns_shove.cpp:1951-1962`); `findRootLine(LINKED_ITEM*)` is a direct lookup (`:1964-1972`); `touchRootLine(...)` is find-or-create, creating with a *clone of the current line* as the root (`:1975-2000`) or with a null root for a bare item (`:2003-2019`).

Lifetime: `pruneRootLines(NODE*)` (`pcbnew/router/pns_shove.cpp:901-917`) erases the *index* entries for every item that node added, on both the springback-pop path (`:941`) and the `Run()` failure path (`:2598`). It never touches `m_rootLineHistoryEntries`, so the owning vector grows monotonically for the lifetime of the `SHOVE` object. For an interactive router that object is recreated per drag, so it is bounded in practice, but a port should size it deliberately.

`m_rootLineHistory` is an `unordered_map` but is only ever used for point lookups and single-key erases; it is never iterated. Good news for determinism.

### 3.3 `MK_HEAD` and endpoint locking

`MK_HEAD` (`pcbnew/router/pns_item.h:43`) is the legacy mechanism and is now **inert inside shove**. Every use in `pns_shove.cpp` is inside `#if 0` (`:484`, `:751`, `:851-853`) or commented out (`:2500`). The one live consumer elsewhere is `pcbnew/router/pns_topology.cpp:1246`, and `NODE::ClearRanks`' default mask is `MK_HEAD | MK_VIOLATION` (`pcbnew/router/pns_node.h:494`). Do not build the Rust port around `MK_HEAD`.

What replaced it is the combination of (a) `ROOT_LINE_ENTRY::isHead` and (b) joint locking. `Run()` calls `NODE::LockJoint` on the head's first point, and on its last point when the head does not end with a via, unless the policy carries `SHP_DONT_LOCK_ENDPOINTS` (`pcbnew/router/pns_shove.cpp:2489-2496`). `assembleLine` then passes `aStopAtLockedJoints = true` into `NODE::AssembleLine` (`pcbnew/router/pns_shove.cpp:220`), so no obstacle line ever grows through a pinned endpoint. `LINE::HasLockedSegments()` (`pcbnew/router/pns_line.cpp:1626-1634`) is a separate `MK_LOCKED` test on links and is what produces `SH_TRY_WALK` at `:647-651`, `:696-697` and `:1091-1092`.

`SHP_DONT_LOCK_ENDPOINTS` is set by both draggers: `pcbnew/router/pns_dragger.cpp:832` and `:882`, and `pcbnew/router/pns_multi_dragger.cpp:277`. The rationale is that a dragged line's endpoints are supposed to be free to slide, unlike a line being routed.

`SHP_REVERSED` is set in exactly one place, `pcbnew/router/pns_dragger.cpp:836-837`, when dragging corner index 0. It flips which endpoint `checkShoveDirection` treats as the pusher (`pcbnew/router/pns_shove.cpp:253-256`).

## 4. OPTIMIZER

### 4.1 Effort flags

`OPTIMIZER::OptimizationEffort`, `pcbnew/router/pns_optimizer.h:99-111`:

| Flag | Value | Dispatched at | Live callers |
| --- | --- | --- | --- |
| `MERGE_SEGMENTS` | 0x001 | `pns_optimizer.cpp:713-714` -> `mergeFull` | shove (OE_MEDIUM/OE_FULL), placer, dragger, router tool |
| `SMART_PADS` | 0x002 | `:724-725` -> `runSmartPads` | shove, placer, router tool |
| `MERGE_OBTUSE` | 0x004 | `:717-718` -> `mergeObtuse` | shove (OE_LOW only), router tool |
| `FANOUT_CLEANUP` | 0x008 | `:728-729` -> `fanoutCleanup` | placer (`pns_line_placer.cpp:1046`), router tool |
| `KEEP_TOPOLOGY` | 0x010 | `:697-701` -> adds `KEEP_TOPOLOGY_CONSTRAINT` | none in tree |
| `PRESERVE_VERTEX` | 0x020 | `:676-680` -> adds `PRESERVE_VERTEX_CONSTRAINT` | dragger via `SetPreserveVertex` (`pns_dragger.cpp:593`) |
| `RESTRICT_VERTEX_RANGE` | 0x040 | `:682-687` -> adds `RESTRICT_VERTEX_RANGE_CONSTRAINT` | none in tree |
| `MERGE_COLINEAR` | 0x080 | `:720-721` -> `mergeColinear` | dragger, placer (`:1979`), router tool |
| `RESTRICT_AREA` | 0x100 | `:689-695` -> adds `AREA_CONSTRAINT` | shove (`pns_shove.cpp:2072-2073`), dragger (`pns_dragger.cpp:607`) |
| `LIMIT_CORNER_COUNT` | 0x200 | `:663-674` -> adds `CORNER_COUNT_LIMIT_CONSTRAINT` | shove always (`pns_shove.cpp:2064`) |
| `REQUIRE_OBTUSE_ANGLES` | 0x400 | `:703-710` -> adds `OBTUSE_ONLY_CONSTRAINT` and runs `dragFixCorners` | dragger, router tool, gated on `GetRestrictAngles()` |

Two of these are effectively inert even when set. `RESTRICT_VERTEX_RANGE_CONSTRAINT::Check` unconditionally returns true (`pcbnew/router/pns_optimizer.cpp:279-284`), and its constructor ignores both bounds (`pcbnew/router/pns_optimizer.h:318-321`). `CORNER_COUNT_LIMIT_CONSTRAINT::Check` computes the corner count and then returns true on both branches (`pcbnew/router/pns_optimizer.cpp:287-303`), with a "fixme: something fishy with the max corneriness limit" comment at `:299`. It also stores `m_minCorners` and `m_angleMask` but never `m_maxCorners` (`pcbnew/router/pns_optimizer.h:332-347`). So `LIMIT_CORNER_COUNT` currently costs a `LINE` copy, a `Replace`, a `Simplify2` and a `CountCorners` per candidate and changes nothing. That is worth knowing, because `pns_dragger.cpp:912-914` disables `LIMIT_CORNER_COUNT` for via drags with a comment calling it a hack.

`ANGLE_CONSTRAINT_45` (`pcbnew/router/pns_optimizer.h:244-264`) is declared, has no definition anywhere in the tree, and is never instantiated. `mergeStep` does not use it. What `mergeStep` actually uses of `DIRECTION_45` is described in 4.4.

### 4.2 `Optimize`: the driver

`OPTIMIZER::Optimize(const LINE* aLine, LINE* aResult, LINE* aRoot)`, `pcbnew/router/pns_optimizer.cpp:652-732`:

```
Optimize(line, out, root):
    if !out: return false
    *out = *line; out->ClearLinks()                          # :657-658
    hasArcs = line->ArcCount() > 0                           # :660

    # constraint assembly, in this order
    if LIMIT_CORNER_COUNT and root:
        addConstraint(CORNER_COUNT_LIMIT(min = root->CountCorners(ANG_OBTUSE),
                                         max = line->SegmentCount(),
                                         mask = ANG_OBTUSE))  # :663-674
    if PRESERVE_VERTEX:       addConstraint(PRESERVE_VERTEX(m_preservedVertex))   # :676-680
    if RESTRICT_VERTEX_RANGE: addConstraint(RESTRICT_VERTEX_RANGE(...))           # :682-687
    if RESTRICT_AREA:         addConstraint(AREA(m_restrictArea, m_restrictAreaIsStrict))  # :689-695
    if KEEP_TOPOLOGY:         addConstraint(KEEP_TOPOLOGY)                        # :697-701
    if REQUIRE_OBTUSE_ANGLES: addConstraint(OBTUSE_ONLY)                          # :703-707

    # passes, in this order, each OR-ing into the return value
    if REQUIRE_OBTUSE_ANGLES:            rv |= dragFixCorners(out)    # :709-710
    if !hasArcs and MERGE_SEGMENTS:      rv |= mergeFull(out)         # :713-714
    if !hasArcs and MERGE_OBTUSE:        rv |= mergeObtuse(out)       # :717-718
    if MERGE_COLINEAR:                   rv |= mergeColinear(out)     # :720-721
    if !hasArcs and SMART_PADS:          rv |= runSmartPads(out)      # :724-725
    if !hasArcs and FANOUT_CLEANUP:      rv |= fanoutCleanup(out)     # :728-729
    return rv
```

Four of the six passes are skipped entirely when the line contains arcs; only `MERGE_COLINEAR` and `dragFixCorners` run on arc lines, and `dragFixCorner` itself bails on arc segments (`pcbnew/router/pns_optimizer.cpp:745-746`). The `AREA_CONSTRAINT` constructor takes `aAllowedAreaStrict` and drops it on the floor (`pcbnew/router/pns_optimizer.h:269-273`), so `m_restrictAreaIsStrict` is dead.

Constraints are `new`ed here and deleted in the destructor (`pcbnew/router/pns_optimizer.cpp:122-128`). They are *not* cleared between `Optimize` calls on the same `OPTIMIZER`, so an optimizer reused across many lines accumulates duplicate constraints. `SHOVE::runOptimizer` does exactly that: one `OPTIMIZER` for the whole queue, `n_passes` passes, so by the end each line is checked against many copies of the same `CORNER_COUNT_LIMIT` and `AREA` constraints, one per `Optimize` call so far. Functionally harmless because the constraints are pure predicates, but it is O(calls) work per candidate. Do not replicate.

The static convenience overload `OPTIMIZER::Optimize(LINE*, int effort, NODE*, VECTOR2I)` (`pcbnew/router/pns_optimizer.cpp:1258-1270`) builds a throwaway optimizer with `SetCollisionMask(-1)` and optimizes in place. It is what the placer and the router tool use.

### 4.3 Collision checking and the "cache"

`checkColliding(ITEM*, bool aUpdateCache)`, `pcbnew/router/pns_optimizer.cpp:474-479`:

```cpp
bool OPTIMIZER::checkColliding( ITEM* aItem, bool aUpdateCache )
{
    CACHE_VISITOR v( aItem, m_world, m_collisionKindMask );

    return static_cast<bool>( m_world->CheckColliding( aItem ) );
}
```

The visitor is constructed and then discarded. The `SHAPE_INDEX_LIST<ITEM*> m_cache` and `std::unordered_map<ITEM*, CACHED_ITEM> m_cacheTags` (`pcbnew/router/pns_optimizer.h:204-206`) are never populated: `cacheAdd` (`pcbnew/router/pns_optimizer.cpp:160-168`) has no callers anywhere in the tree, `CacheRemove` and `ClearCache` have no callers outside the class, and `MaxCachedItems = 256` (`pcbnew/router/pns_optimizer.h:159`) is never referenced. `aUpdateCache` is unused.

In other words: **there is no working collision cache in the optimizer.** Every candidate is checked against the live `NODE` with a full `NODE::CheckColliding`. Note also that `m_collisionKindMask` (`pcbnew/router/pns_optimizer.h:209`, set to `ITEM::ANY_T` by shove at `pns_shove.cpp:2087` and to -1 by the static overload at `pns_optimizer.cpp:1263`) is only read by the dead `CACHE_VISITOR`, so it has no effect either. `ClearCache(aStaticOnly = true)` at `:206-213` would also be undefined behaviour if it ever ran: it erases from `m_cacheTags` while iterating it.

`checkColliding(LINE*, const SHAPE_LINE_CHAIN&)` (`:502-507`) wraps the candidate path in a temporary `LINE` sharing the original's width/layer/net and forwards. That temporary is unlinked, so `NODE::CheckColliding` sees it as a foreign item and will report collisions with the *original* line's own segments if they are still in the node. Callers therefore always remove the line from the node first (`pns_dragger.cpp:830`, `router_tool.cpp:2328`, `pns_multi_dragger.cpp:271`) or optimize inside a node where the line is not present.

`checkConstraints(v1, v2, originLine, currentPath, replacement)` (`:488-499`) is a plain AND over `m_constraints`. `OPT_CONSTRAINT::GetPriority`/`SetPriority` (`pcbnew/router/pns_optimizer.h:236-237`) exist but nothing ever sets or reads a priority.

### 4.4 `mergeFull` and `mergeStep`

`mergeFull(LINE*)`, `pcbnew/router/pns_optimizer.cpp:587-624`:

```
mergeFull(line):
    step     = line.SegmentCount() - 1                       # :590
    segs_pre = line.SegmentCount()
    line.Simplify2()                                          # :594
    if step < 0: return false
    current_path = line
    loop:
        max_step = current_path.SegmentCount() - 2            # :604
        step = min(step, max_step)
        if step < 1: break
        if !mergeStep(line, current_path, step): step--       # :612-615
        if step == 0: break                                   # :617-618
    line.SetShape(current_path)                               # :621
    return current_path.SegmentCount() < segs_pre
```

Coarse-to-fine: try to bypass a long span first, and only shrink the span when nothing at that span works. Note `mergeStep` mutates `current_path` in place and returns true after the *first* successful replacement, so the outer loop restarts the scan at the same step size after each hit.

`mergeStep(LINE* aLine, SHAPE_LINE_CHAIN& aCurrentPath, int step)`, `pcbnew/router/pns_optimizer.cpp:849-916`:

```
mergeStep(line, path, step):
    cost_orig = COST_ESTIMATOR::CornerCost(path)                       # :853
    if line.SegmentCount() < 2: return false                            # :855-856
    cornerMode = ROUTER::GetInstance()->Settings().GetCornerMode()       # :858
    is90mode   = cornerMode in {MITERED_90, ROUNDED_90}                  # :859
    orig_start = DIRECTION_45(line.CSegment(0),  is90mode)               # :861   (computed, never used)
    orig_end   = DIRECTION_45(line.CSegment(-1), is90mode)               # :862   (computed, never used)

    for n in 0 .. n_segs - step - 1:
        if path segment n or n+step is an arc: continue                  # :868-872
        s1 = path.CSegment(n); s2 = path.CSegment(n + step)
        for i in 0..1:                                                   # the two "postures"
            bypass = DIRECTION_45().BuildInitialTrace(s1.A, s2.B, i, cornerMode)   # :883
            cost[i] = INT_MAX
            if !checkColliding(line, bypass)
               and checkConstraints(n, n + step + 1, line, path, bypass):          # :888-891
                path[i] = path with [s1.Index(), s2.Index()] replaced by bypass
                path[i].Simplify2()
                cost[i] = CornerCost(path[i])                                       # :895-899
        pick path[0] if cost[0] < cost_orig and cost[0] < cost[1]
        else pick path[1] if cost[1] < cost_orig                                    # :902-905
        if picked: aCurrentPath = *picked; return true                              # :907-912
    return false
```

`DIRECTION_45` shows up three ways. `BuildInitialTrace(a, b, posture, cornerMode)` (`:883`) generates the two canonical two-segment 45-degree connections between the span endpoints; the `i` loop over `{0, 1}` is exactly "diagonal-first" versus "straight-first". `cornerMode` comes from the global router settings via the singleton `ROUTER::GetInstance()` (`:858`), which is a hidden global dependency the port should turn into an explicit parameter. And `COST_ESTIMATOR::CornerCost` is itself defined in terms of `DIRECTION_45::Angle` (`:44-57`). The `orig_start` / `orig_end` directions at `:861-862` are computed and never read; dead code.

`fanoutCleanup` (`:1273-1320`) and `dragFixCorner` (`:767`) also call `BuildInitialTrace`, the latter without a corner mode so it uses the 45-degree default.

### 4.5 `mergeObtuse`

`pcbnew/router/pns_optimizer.cpp:510-584`. Same coarse-to-fine skeleton as `mergeFull`, but the candidate is not a generated bypass; it is the intersection point of the two extended segments:

```
mergeObtuse(line):
    step = line.PointCount() - 3                            # :514
    segs_pre = line.SegmentCount()
    current_path = line
    loop:
        step = min(step, current_path.SegmentCount() - 2)
        if step < 2: line = current_path; return line.SegmentCount() < segs_pre   # :530-534
        found = false
        for n in 0 .. n_segs - step - 1:
            s1 = current_path.CSegment(n); s2 = current_path.CSegment(n + step)
            if DIRECTION_45(s1).IsObtuse(DIRECTION_45(s2)):                        # :544
                ip = *s1.IntersectLines(s2)                                        # :546
                s1opt = SEG(s1.A, ip); s2opt = SEG(ip, s2.B)
                if DIRECTION_45(s1opt).IsObtuse(DIRECTION_45(s2opt)):              # :551
                    opt_path = [s1opt.A, ip, s2opt.B]
                    if !checkColliding(LINE(line, opt_path)):                      # :560
                        current_path.Replace(s1.Index()+1, s2.Index(), ip)         # :562
                        found = true; break
        if !found:
            if step <= 2: line = current_path; return ...                          # :575-579
            step--
```

Note `mergeObtuse` bypasses `checkConstraints` entirely. It only checks collision. So `RESTRICT_AREA`, `PRESERVE_VERTEX` and friends do not constrain it. Since shove uses `MERGE_OBTUSE` for `OE_LOW` (`pcbnew/router/pns_shove.cpp:2046`) and always sets `RESTRICT_AREA` (`:2072`), on the lowest effort setting the area restriction is silently not enforced.

`s1.IntersectLines(s2)` at `:546` is dereferenced without checking the optional. Two obtuse-related segments are never parallel in the 45-degree world, so this holds, but a Rust port should return an error rather than assume.

### 4.6 `mergeColinear`

`pcbnew/router/pns_optimizer.cpp:627-649`. A single forward pass removing the shared vertex of any two collinear consecutive segments, skipping zero-length segments (an artefact of abutting arcs) and vertices that are on an arc:

```
for segIdx in 0 .. line.SegmentCount() - 2:
    s1 = line.CSegment(segIdx); s2 = line.CSegment(segIdx + 1)
    if s1.SquaredLength() == 0 or s2.SquaredLength() == 0: continue     # :639-640
    if s1.Collinear(s2) and !line.IsPtOnArc(segIdx + 1):
        line.Remove(segIdx + 1)                                          # :644
return line.SegmentCount() < nSegs
```

The loop bound `line.SegmentCount() - 1` is re-evaluated each iteration and `segIdx` is not rewound after a removal, so the pass can skip over a newly created collinear pair; a second `Optimize` call would catch it. No collision check at all: removing a collinear vertex cannot change the geometry.

### 4.7 `runSmartPads` and `smartPadsSingle`

`runSmartPads(LINE*)`, `pcbnew/router/pns_optimizer.cpp:1231-1255`: requires at least 3 points; finds a pad or via at each end via `findPadOrVia(layer, net, p)` (`:1093-1107`, a joint lookup that returns the first `VIA_T | SOLID_T` link); optimizes the start with `smartPadsSingle(line, startPad, aEnd = false, aEndVertex = 3)`; then the end with a vertex budget of `PointCount()-1` or `PointCount()-1-vtx` depending on whether the start pass consumed vertices (`:1245-1250`); then `Simplify2`. It always returns true, which means `Optimize` reports "changed" whenever `SMART_PADS` is on, whether or not anything changed.

`smartPadsSingle(LINE*, ITEM* aPad, bool aEnd, int aEndVertex)`, `pcbnew/router/pns_optimizer.cpp:1110-1228`:

```
smartPadsSingle(line, pad, isEnd, endVertex):
    ForbiddenAngles = ANG_ACUTE | ANG_RIGHT | ANG_HALF_FULL | ANG_UNDEFINED   # :1114-1115
    if pad is a SOLID with a non-zero Offset: return -1                        # :1123-1124
    if pad is a VIA: return -1                                                 # :1128-1129
    breakouts = computeBreakouts(line.Width(), pad, permitDiagonal = true)     # :1131
    chain = isEnd ? line.reversed() : line                                     # :1132
    p_end = min(endVertex, min(3, chain.PointCount() - 1))                     # :1133
    for p in 1 .. p_end:                                                       # :1136
        if the pad shape does not collide with SEG(chain[0], chain[p]) inflated
           by width/2, the line is inside the pad: skip this p                 # :1139-1143
        for each breakout, for diag in {0,1}:
            connect = BuildInitialTrace(breakout.last, chain[p], diag == 0)    # :1150-1151
            reject if empty, if the breakout-to-connect angle is forbidden,
              or if breakout.Length() > chain.Length()                          # :1155-1164
            candidate = breakout + connect + chain[p+1 ..]                      # :1166-1170
            if LINE(line, candidate).CountCorners(ForbiddenAngles) == 0:
                record variant (p, breakout.Length(), candidate.Simplify2())    # :1175-1183
    # selection
    min_cost   = CornerCost(*line)      # the user's own line is the baseline   # :1193
    max_length = 0
    for each variant, if !checkColliding(candidate):                            # :1205
        accept if cost < min_cost, or cost == min_cost and breakout length > max_length  # :1207
    if found: line->SetShape(best); return best_p
    return -1
```

The tie-break on breakout length is documented at `:1188-1192`: on an oblong pad, two equal-cost exits should resolve to the one that runs along the pad's long axis before leaving.

Breakout generation, `computeBreakouts` (`:1044-1090`) dispatching on shape:

- circle / via -> `circleBreakouts` (`:919-939`): eight rays at 45-degree steps, length `radius * sqrt(2)`.
- rect -> `rectBreakouts` (`:988-1041`): four axis-aligned exits at `size/2 + width`, plus four diagonals when permitted, offset by `d_offset` so an oblong rect exits from the ends of its long axis.
- segment -> `ApproximateSegmentAsRect` (`pcbnew/router/pns_utils.cpp:356-366`) then `rectBreakouts`.
- simple polygon -> `customBreakouts` (`:942-985`): cast a ray from the pad centre every 45 (or 90) degrees, take the first intersection with the polygon boundary as the breakout endpoint.

`circleBreakouts` ignores `aPermitDiagonal` and always emits all eight (`:924`), while `rectBreakouts` and `customBreakouts` honour it.

### 4.8 `fanoutCleanup`

`pcbnew/router/pns_optimizer.cpp:1273-1320`. If both endpoints sit on a pad or via (or the line ends with a via) *and* the line is shorter than `10 * width` (`:1285`, `:1303`), replace the whole line with whichever of the two `BuildInitialTrace` postures does not collide (`:1305-1316`). This is the "two pads next to each other, just draw the L" cleanup. It uses `m_world->CheckColliding` directly rather than `checkColliding` (`:1311`), and it also reads `cornerMode` off the `ROUTER::GetInstance()` singleton (`:1278`).

### 4.9 `COST_ESTIMATOR`

`pcbnew/router/pns_optimizer.h:48-80`, `pcbnew/router/pns_optimizer.cpp:44-110`.

Corner cost is a pure lookup on the angle class between two consecutive segments (`pcbnew/router/pns_optimizer.cpp:44-57`):

| `DIRECTION_45::Angle` | cost |
| --- | --- |
| `ANG_STRAIGHT` | 5 |
| `ANG_OBTUSE` (45 degrees) | 10 |
| `ANG_RIGHT` (90 degrees) | 30 |
| `ANG_ACUTE` (135 degrees) | 50 |
| `ANG_HALF_FULL` (180 degrees, a hairpin) | 60 |
| anything else (`ANG_UNDEFINED`) | 100 |

`CornerCost(SHAPE_LINE_CHAIN)` sums over all consecutive segment pairs (`:60-68`). Because straight (collinear) joints still cost 5, the estimator prefers *fewer vertices* even when the shape is unchanged; that is deliberate.

`Add`/`Remove`/`Replace` (`:77-97`) maintain running `m_lengthCost` (a `double`, the sum of chain lengths) and `m_cornerCost` (an `int`).

`IsBetter(other, lengthTolerance, cornerTolerance)` (`:100-110`):

```cpp
if( aOther.m_cornerCost < m_cornerCost && aOther.m_lengthCost < m_lengthCost )
    return true;
else if( aOther.m_cornerCost < m_cornerCost * aCornerTolerance &&
         aOther.m_lengthCost < m_lengthCost * aLengthTolerance )
    return true;
return false;
```

Strictly better on both axes wins; otherwise both axes must be within a multiplicative tolerance. Note the semantics are inverted from the name: `a.IsBetter(b)` asks whether **b** is better than **a**. `IsBetter` has no callers in this tree; `mergeStep` compares raw `CornerCost` integers instead (`:902-905`) and `smartPadsSingle` does its own cost-plus-length comparison (`:1207`). The whole running-total machinery (`Add`/`Remove`/`Replace`, `m_lengthCost`) is unused. Length is therefore **not** part of any live decision except the breakout tie-break and the diff-pair coupled-length budget.

### 4.10 Diff-pair path

`mergeDpSegments` (`:1497-1535`) is the pair-aware analogue of `mergeFull`, running two independent step counters over the P and N chains and calling `mergeDpStep` (`:1434-1494`) for each. `mergeDpStep` proposes a `BuildInitialTrace` bypass on the reference chain when the span is obtuse, tries to find a matching bypass on the coupled chain via `coupledBypass` (`:1369-1423`), and accepts the change only if the coupled length does not drop by more than `budget = clenPre / 10` (`:1444`, "fixme: come up with something more intelligent here"). `coupledBypass` uses a fixed C array `int vStartIdx[1024]` with an acknowledged overflow risk (`:1373`). Verification is `verifyDpBypass` (`:1350-1366`): the two new lines must not collide with each other or with the world.

`Tighten` / `tightenSegment` / `shovedArea` (`:1544-1703`) are free functions in the `PNS` namespace that binary-search a three-segment span inward while it stays collision free, minimising the area swept relative to the pre-shove line. Nothing in the tree calls `Tighten`; it is dormant.

## 5. What the callers ask for

### 5.1 `SHOVE::runOptimizer` -- `pns_shove.cpp:2022-2129`

```
runOptimizer(node):
    effort = Settings().OptimizerEffort()
    area   = totalAffectedArea()                      # springback frame area U m_affectedArea (:1926-1939)
    maxWidth = max width over m_optimizerQueue        # :2034-2035
    if area: area.Inflate(maxWidth); area = area.Intersect(VisibleViewArea())   # :2037-2041
    switch effort:
        OE_LOW:    optFlags = MERGE_OBTUSE;   n_passes = 1     # :2045-2048
        OE_MEDIUM: optFlags = MERGE_SEGMENTS; n_passes = 2     # :2050-2053
        OE_FULL:   optFlags = MERGE_SEGMENTS; n_passes = 2     # :2055-2058
    optFlags |= LIMIT_CORNER_COUNT                              # :2064
    if area: optFlags |= RESTRICT_AREA; optimizer.SetRestrictArea(*area, false)  # :2066-2074
    if Settings().SmartPads() and cornerMode in {MITERED_45, ROUNDED_45}:
        optFlags |= SMART_PADS                                  # :2079-2083
    optimizer.SetEffortLevel(optFlags & ~m_optFlagDisableMask)  # :2086
    optimizer.SetCollisionMask(ITEM::ANY_T)                     # :2087

    for pass in 0 .. n_passes-1:
        std::reverse(m_optimizerQueue)                          # :2093
        for i in 0 .. queue.size()-1:
            rootEntry = findRootLine(queue[i])
            if rootEntry and (policy & SHP_DONT_OPTIMIZE): continue   # :2107-2108
            if rootEntry and rootEntry->isHead:            continue   # :2109-2110
            if optimizer.Optimize(&queue[i], &optimized, rootEntry->rootLine.get()):
                replaceLine(queue[i], optimized, aIncludeInChangedArea = false, aNode = node)  # :2122
                queue[i] = std::move(optimized)                        # :2123
```

`OE_MEDIUM` and `OE_FULL` are identical here (`:2050-2058`); only the placer's own post-shove optimize distinguishes them, and not really (`pns_line_placer.cpp:967-977` also treats them the same). The restrict-area box is the union of everything this shove and all surviving springback frames touched, inflated by the widest queued line and clipped to the visible viewport, so off-screen geometry is never rearranged. Heads are excluded from optimization because the caller owns the head's shape.

The `std::reverse` per pass (`:2093`) means pass 0 optimizes the queue newest-first and pass 1 oldest-first. Note the call at `:2122` is `replaceLine( lineToOpt, optimized, false, aNode )` -- four arguments, so `false` binds to `aIncludeInChangedArea` and `aNode` binds to `aAllowRedundantSegments` (a `bool` parameter receiving a `NODE*`, which is a non-null pointer, hence `true`), while the real `aNode` parameter stays `nullptr` and the replace goes to `m_currentNode`. Since `runOptimizer` is only ever called with `aNode == m_currentNode` (`:2563`) the outcome is the same, but the argument slot is misaligned. Do not copy this.

### 5.2 `LINE_PLACER`

- Creates its `SHOVE` over a branch of the world once, at `pcbnew/router/pns_line_placer.cpp:1478`.
- Per move in shove mode (`rhShoveOnly`, `:921-1017`): walk around solids only, set the do-not-touch node from `m_endItem` (`:935-945`), `ClearHeads()`, `AddHeads(newHead, SHP_SHOVE)` (`:959-960`), `Run()`, and on success pull `GetModifiedHead(0)`, split head/tail, and post-optimize with `MERGE_SEGMENTS` (plus `SMART_PADS` when enabled and in a 45-degree corner mode) (`:965-1006`). On failure it falls back to `rhWalkOnly` (`:1013`).
- Walk mode uses `MERGE_SEGMENTS` only (`:747-757`, `:799`) and `MERGE_SEGMENTS` on each walkaround candidate (`:617`, `:637`).
- `optimizeTailHeadTransition` uses `FANOUT_CLEANUP` alone (`:1046`), and there is a `MERGE_COLINEAR`-only call at `:1979`.
- Springback locking: `AddLockedSpringbackNode` at `:1626`, `:1737`, `:1750`; `RewindSpringbackTo` + `UnlockSpringbackNode` at `:1789-1790`; `RewindToLastLockedNode` at `:1814`.

### 5.3 `DRAGGER`

- Sets the default policy to `SHP_SHOVE` at `pcbnew/router/pns_dragger.cpp:328`.
- Segment/corner drag (`:813-863`): removes the pre-drag line from the shove's current node (`:829-830`), policy `SHP_SHOVE | SHP_DONT_LOCK_ENDPOINTS`, plus `SHP_REVERSED` when dragging corner 0 (`:832-837`), `AddHeads` + `Run`, then `optimizeAndUpdateDraggedLine`.
- Arc drag (`:865-906`): same, with a guard that a collapsed arc with fewer than two points is not fed to `AddHeads` (`:872-887`).
- Via drag (`:908-950`): `DisablePostShoveOptimizations(OPTIMIZER::LIMIT_CORNER_COUNT)` (`:914`) with a comment calling it a hack, then `AddHeads(m_draggedVia, aP, SHP_SHOVE)` and `Run`; falls back to `dragViaWalkaround` on failure (`:944-945`).
- `optimizeAndUpdateDraggedLine` (`:569-619`) is where the dragger's own optimizer runs: `MERGE_SEGMENTS`, plus `MERGE_COLINEAR` when `SmoothDraggedSegments()` and `REQUIRE_OBTUSE_ANGLES` when `GetRestrictAngles()` (`:578-584`); `SetPreserveVertex(anchor)` where the anchor is the dragged point or the nearest good corner (`:588-594`); and **the restrict-area box is set here**, to `aDragged.ChangedArea(&aOrig)` (or a degenerate box around the drag point when there is no change), only when `GetOptimizeEntireDraggedTrack()` is false (`:598-608`). It passes the original line as `aRoot` so `LIMIT_CORNER_COUNT` has a baseline (`:612`).

### 5.4 `MULTI_DRAGGER`

- Branches the world, removes every dragged line from the branch, and constructs the `SHOVE` over it, with default policy `SHP_SHOVE | SHP_DONT_LOCK_ENDPOINTS` (`pcbnew/router/pns_multi_dragger.cpp:267-277`).
- Per move it flips the default policy back to plain `SHP_SHOVE` (`:647`), then adds every completed line as a head with `SHP_SHOVE | SHP_DONT_OPTIMIZE` (`:650-654`) and runs once. `SHP_DONT_OPTIMIZE` is the only use of that flag in the tree, and it is why multi-drag results are not re-optimized by shove.
- The lines are sorted by `dragDist` before being added (`:624-629`), which makes head order deterministic and is the one place a caller deliberately controls it.

### 5.5 `DIFF_PAIR_PLACER`

- Calls `SHOVE::ShoveObstacleLine` directly, outside `Run()`, as the geometric core of `attemptWalk`, with `ForceClearance(true, gap - 2 * PNS_HULL_MARGIN)` (`pcbnew/router/pns_diff_pair_placer.cpp:247-251`).
- In shove mode it adds both lines of the pair as heads with the default policy and runs once (`:370-374`), then reads back both modified heads (`:382-386`).
- `tryWalkDp` runs the diff-pair optimizer (`optimizer.Optimize(&aPair)` -> `mergeDpSegments`) at `:311-314`.

### 5.6 `ROUTER_TOOL`

The interactive "optimize selected tracks" action uses the widest effort set in the tree: `MERGE_SEGMENTS | MERGE_OBTUSE | MERGE_COLINEAR | SMART_PADS | FANOUT_CLEANUP`, plus `REQUIRE_OBTUSE_ANGLES` when `GetRestrictAngles()` (`pcbnew/router/router_tool.cpp:2333-2342`), on a branch with the original line removed (`:2327-2328`), rejecting the result if it is geometrically identical or collides (`:2344-2354`).

## 6. Every magic constant, with file:line

### 6.1 Budgets and limits

| Value | Meaning | Where |
| --- | --- | --- |
| `250` | shove iteration limit, per `shoveMainLoop` call, per head | `pcbnew/router/pns_routing_settings.cpp:43` (default) and `:72` (JSON param default); read at `pcbnew/router/pns_shove.cpp:1890` |
| `1000` ms | shove wall-clock limit, per `shoveMainLoop` call | `pcbnew/router/pns_routing_settings.cpp:44`; read at `pcbnew/router/pns_shove.cpp:1891`; expiry test at `pcbnew/router/time_limit.cpp:87-90` |
| `40` | walkaround iteration limit | `pcbnew/router/pns_routing_settings.cpp:45`, `:86`; used by shove at `pcbnew/router/pns_shove.cpp:818` |
| `40` | via force propagation iteration limit | `pcbnew/router/pns_routing_settings.cpp:58`, `:73`; not read by shove |
| `4` | `shoveLineToHullSet` attempts (cw/ccw x hull order) | `pcbnew/router/pns_shove.cpp:338` |
| `3` | `ShoveObstacleLine` hull-expansion attempts | `pcbnew/router/pns_shove.cpp:578` |
| `2` | `onCollidingSolid` walkaround attempts | `pcbnew/router/pns_shove.cpp:827` |
| `2` | `mergeStep` / `dragFixCorner` / `fanoutCleanup` posture count | `pcbnew/router/pns_optimizer.cpp:881`, `:765`, `:1305` |
| `1` / `2` | `runOptimizer` pass counts (OE_LOW / OE_MEDIUM and OE_FULL) | `pcbnew/router/pns_shove.cpp:2047`, `:2052`, `:2057` |
| `8` | `MIN_OBSTACLES_PER_BLOCK` and parallel threshold in `NearestObstacle` | `pcbnew/router/pns_node.cpp:434-435` |
| `3` | `Tighten` refinement passes | `pcbnew/router/pns_optimizer.cpp:1671` |

### 6.2 Ranks

| Value | Meaning | Where |
| --- | --- | --- |
| `-1` | unranked / reset | `pcbnew/router/pns_item.h:125`; `pcbnew/router/pns_node.cpp:1686` (`ClearRanks`), `:1637` (`Commit`) |
| `0` | a via being dragged as a head | `pcbnew/router/pns_shove.cpp:2456` |
| `100000` | a line head, and its via | `pcbnew/router/pns_shove.cpp:2501`, `:2506` |
| `rank - 1` | normal forward shove demotion | `pcbnew/router/pns_shove.cpp:669`, `:723`, `:800`, `:830`, `:1301`, `:1847` |
| `rank + 1` | reverse-collision promotion | `pcbnew/router/pns_shove.cpp:1737`, `:1776`, `:1779` |
| `rank - 1` | reverse arc collision (note: not `+1`) | `pcbnew/router/pns_shove.cpp:1800` |
| `rank + 10000` | walkaround promotion when `JumpOverObstacles()` is off | `pcbnew/router/pns_shove.cpp:832` |

### 6.3 Hull geometry

| Value | Meaning | Where |
| --- | --- | --- |
| `10` | `HULL_MARGIN`, declared and never used | `pcbnew/router/pns_utils.h:138` |
| `10` | `PNS_HULL_MARGIN`, the live one | `pcbnew/router/pns_line.h:45`; used at `pcbnew/router/pns_diff_pair_placer.cpp:247`, `pcbnew/router/pns_node.cpp:1338`, `:1348` |
| `clearance + (thickness + 1) / 2` | hull inflation, rounded up | `pcbnew/router/pns_utils.cpp:481` (`BuildHullForPrimitiveShape`), `:73` (`ArcHull`) |
| `clearance + thickness / 2` | hull inflation, truncated | `pcbnew/router/pns_utils.cpp:186` (`SegmentHull`), `pcbnew/router/pns_via.cpp:240` (`VIA::Hull`) |
| `2 / (1 + sqrt(2)) * d` | octagon chamfer length along the axis | `pcbnew/router/pns_utils.cpp:188` (segment), `:86` (arc) |
| `2 * (1 - 1/sqrt(2)) * (r + cl)` | equilateral octagon chamfer for circles | `pcbnew/router/pns_utils.cpp:82`, `:501` |
| `(2 * cl + width) * (1 - 1/sqrt(2))` | via octagon chamfer | `pcbnew/router/pns_via.cpp:249` |
| `clearance / 10` | `kinkThreshold`: below this length a non-45 segment is snapped | `pcbnew/router/pns_utils.cpp:184` |
| `+1`, `+2` | clearance bumps applied to snapped kinky segments | `pcbnew/router/pns_utils.cpp:226`, `:231`, `:239` |
| `SHAPE_ARC::DefaultAccuracyForPCB()` | extra clearance for arcs, in `ArcHull` and in the shove hull loop | `pcbnew/router/pns_utils.cpp:85`; `pcbnew/router/pns_shove.cpp:593` |
| `180.0` degrees | above this central angle an arc hull degenerates to a circle | `pcbnew/router/pns_utils.cpp:76` |
| `ARC_LOW_DEF` | arc-to-polyline accuracy for hull construction | `pcbnew/router/pns_utils.cpp:88` |
| `1000` IU | `c_ENDPOINT_ON_HULL_THRESHOLD`: how close an endpoint must be to a hull to be snapped onto it | `pcbnew/router/pns_shove.cpp:332` |
| `1000` IU | `cHullFailureExpansionFactor`: per-attempt hull inflation on shove failure | `pcbnew/router/pns_shove.cpp:524`, applied at `:628` |
| `1.0` | `extensionWalkThreshold`: an arc shove that doubles the length becomes a walkaround | `pcbnew/router/pns_shove.cpp:701` |
| `10.0` | `AssembleCluster` area expansion limit around a solid | `pcbnew/router/pns_shove.cpp:804` |
| `2` IU | via anti-snap nudge per iteration | `pcbnew/router/pns_shove.cpp:1074` |
| `+1` | `m_maxFanoutWidth` slack over the widest attached track / the segment width | `pcbnew/router/pns_shove.cpp:1553`, `:1592` |
| `width / 2` in each axis | segment-as-rect approximation for breakouts | `pcbnew/router/pns_utils.cpp:360` |

### 6.4 Optimizer weights and thresholds

| Value | Meaning | Where |
| --- | --- | --- |
| `5` | `COST_ESTIMATOR` cost of a straight (collinear) joint | `pcbnew/router/pns_optimizer.cpp:51` |
| `10` | cost of a 45-degree (obtuse) corner | `pcbnew/router/pns_optimizer.cpp:50` |
| `30` | cost of a right-angle corner | `pcbnew/router/pns_optimizer.cpp:53` |
| `50` | cost of an acute corner | `pcbnew/router/pns_optimizer.cpp:52` |
| `60` | cost of a 180-degree hairpin (`ANG_HALF_FULL`) | `pcbnew/router/pns_optimizer.cpp:54` |
| `100` | cost of anything else (`ANG_UNDEFINED`) | `pcbnew/router/pns_optimizer.cpp:55` |
| `256` | `MaxCachedItems`, declared and never referenced | `pcbnew/router/pns_optimizer.h:159` |
| `3` | maximum vertex index smart pads will rewrite from a pad | `pcbnew/router/pns_optimizer.cpp:1133`, `:1246` |
| `1` (squared IU) | `PRESERVE_VERTEX_CONSTRAINT` "on the segment" tolerance | `pcbnew/router/pns_optimizer.cpp:257`, `:271` |
| `10 * width` | `fanoutCleanup` maximum line length | `pcbnew/router/pns_optimizer.cpp:1285` |
| `radius * sqrt(2)` | circle breakout ray length | `pcbnew/router/pns_optimizer.cpp:929` |
| `45` degrees | circle breakout angular step (8 rays, always) | `pcbnew/router/pns_optimizer.cpp:924` |
| `45` / `90` degrees | custom (polygon) breakout step, diagonal-permitting or not | `pcbnew/router/pns_optimizer.cpp:952` |
| `max(bboxW, bboxH) / 2 + 5` | custom breakout ray length | `pcbnew/router/pns_optimizer.cpp:951` |
| `size/2 + width`, `width + min(sx,sy)/2` | rect breakout offsets, orthogonal and diagonal | `pcbnew/router/pns_optimizer.cpp:1003-1004`, `:1013` |
| `12` | `rectBreakouts` reserve | `pcbnew/router/pns_optimizer.cpp:996` |
| `clenPre / 10` | diff-pair coupled-length loss budget per merge step | `pcbnew/router/pns_optimizer.cpp:1444` |
| `1024` | `coupledBypass` fixed start-index array, with an acknowledged overflow risk | `pcbnew/router/pns_optimizer.cpp:1373` |
| `ANG_ACUTE\|ANG_RIGHT\|ANG_HALF_FULL\|ANG_UNDEFINED` | `ForbiddenAngles` in smart pads | `pcbnew/router/pns_optimizer.cpp:1114-1115` |
| `ANG_OBTUSE` | the angle mask used to seed `CORNER_COUNT_LIMIT_CONSTRAINT` | `pcbnew/router/pns_optimizer.cpp:665` |

### 6.5 Debug-only magic numbers

Widths passed to `AddItem` / `AddPoint` / `AddShape` (`0`, `10000`, `100000`, `150000`, `200000`, `1000000`) are render overrides in internal units, not geometry. They appear throughout `pcbnew/router/pns_shove.cpp` (for example `:410`, `:660`, `:745`, `:1110`, `:1287`) and carry no algorithmic meaning.

## 7. Debug decorator hooks and logging

The decorator interface is `pcbnew/router/pns_debug_decorator.h:37-115`, with methods `SetIteration`, `Message`, `NewStage`, `BeginGroup`, `EndGroup`, `AddPoint`, `AddItem`, `AddShape(SHAPE*)`, `AddShape(BOX2I)`, `AddShape(SEG)` and `Clear`. Every call site goes through the `PNS_DBG` / `PNS_DBGN` macros (`:126-133`), which short-circuit on `dbg && dbg->IsDebugEnabled()` and attach `__FILE__`, `__FUNCTION__`, `__LINE__` as a `SRC_LOCATION_INFO`. A `PNS_SILENCE_DEBUG` compile-time switch is available at `:124`.

### 7.1 The iteration marker

`shoveIteration` calls `Dbg()->SetIteration(aIter)` at `pcbnew/router/pns_shove.cpp:1641-1642`. That is the single hook a step-through visual debugger needs: everything emitted after it belongs to iteration `aIter`.

### 7.2 Named groups (the natural tree nodes for a debugger UI)

| Group name | Level / iter | Where |
| --- | --- | --- |
| `"shove-details"` | level 1 | `pcbnew/router/pns_shove.cpp:336`, closed at `:505` and `:510` |
| `"walk-cluster"` | level 1 | `:806`, closed at `:811` |
| `"push-via-by-line"` | level 1 | `:1187`, closed at `:1258` |
| `"on-reverse-via-fail-shove"` | `m_iter` | `:1328`, closed at `:1338` |
| `"on-reverse-via-fail-lonevia"` | `m_iter` | `:1350`, closed at `:1353` |
| `"on-reverse-via"` | `m_iter` | `:1371`, closed at `:1375` |
| `"iter %d: reverse-collide-via"` | 0 | `:1725`, closed at `:1746` |
| `"iter %d: reverse-collide-segment"` | 0 | `:1753`, closed at `:1788` |
| `"iter %d: reverse-collide-arc "` | 0 | `:1796`, closed at `:1802` |
| `"iter %d: collide-segment "` | 0 | `:1821`, closed at `:1828` |
| `"iter %d: collide-arc "` | 0 | `:1834`, closed at `:1841` |
| `"iter %d: collide-via (fixup: %d)"` | 0 | `:1846`, closed at `:1852` |
| `"iter %d: walk-solid "` | 0 | `:1858`, closed at `:1861` |
| `"node:<label>"` | 0 | `NodeStats` in `pcbnew/router/pns_utils.cpp:549`, closed at `:557` |

Note the group at `pcbnew/router/pns_shove.cpp:1187` is opened unconditionally in `onCollidingVia` but the matching `EndGroup` at `:1258` is only reached on the normal return path; the alternative exit at `:1239` is commented out. If the solid branch at `:1233-1243` ever fell through to an early return, the group would leak. It currently does not, but a port should pair the scope with RAII.

### 7.3 Named shapes and items

Stable labels a viewer can filter on, from `pcbnew/router/pns_shove.cpp`: `"shove-changed-area"` (`:92`), `"chkdir %d"` (`:264`), `"hull[%d]"` / `"path[%d]"` / `"obs[%d]"` (`:410-412`), `"colliding-segment"` (`:659`), `"current-line"` (`:660`, `:716`, `:745`, `:825`, `:1202`, `:1330`, `:1352`), `"obstacle-line"` (`:661`, `:717`, `:744`), `"shoved-line"` (`:662`, `:718`, `:746`, `:1331`), `"obstacle-arc"` (`:715`), `"cl-item"` (`:809`), `"walk-line"` (`:848`), `"via-pre"` / `"via-post"` (`:1110-1111`), `"fan-pre"` / `"fan-post"` (`:1163-1164`), `"current-line-via"` (`:1206`, `:1335`), `"orig-via"` (`:1209`), `"obstacle-via-hull"` (`:1287`), `"obstacle-via"` / `"the-via"` (`:1329`, `:1351`), `"rr-the-via"` / `"rr-current-line"` / `"rr-shoved-line"` (`:1372-1374`), `"push line stack failed"` (`:1460`), `"nearest %p %s rank %d"` (`:1682-1686`), `"v2v nearesti"` (`:1734`), `"head"` (`:1756`), `"opt-area"` (`:2070`).

The most useful `Message` traces for reconstructing a run: the per-attempt failure reasons in `shoveLineToHullSet` (`"attempt %d fail vfirst-last" / "fail vend-start" / "fail direction-check" / "fail self-intersect" / "fail coll-check"`, `:454-500`), the springback trace (`"push-sp depth=%d node=%p"` at `:1031`, `"pop-sp node=%p depth=%d"` at `:939`, `"restore-springback-via ..."` at `:964`, `"addLockedSPNode ..."` at `:2146`), the per-iteration header (`"iter %d: node %p stack %d "` at `:1906`), the obstacle description (`"NI: %s (%s) %p %d"` at `:1708`), the root-line trace (`"touch [found]" / "touch [create]"` at `:1983`, `:1994`, `:2009`, `:2015`), and the run summary (`"Shove status : %s after %d iterations, heads: %d"` at `:2555-2558`).

### 7.4 The optimizer emits nothing

Every `PNS_DBG` in `pcbnew/router/pns_optimizer.cpp` is commented out: `:239-241` (area constraint), `:464` (topology constraint), `:669-671` (corner count limit), `:693` (area shape), `:1583-1607` and `:1646-1654` and `:1701-1702` (tighten). The optimizer has no live visual output at all. For a Rust port with a visual debugger this is the biggest gap to fill: instrument `mergeStep` (candidate bypass, both postures, cost, accept/reject reason) and `smartPadsSingle` (breakout list, variants, chosen variant) from the start.

### 7.5 Logging

`SHOVE` inherits `SetLogger` / `Logger()` from `ALGO_BASE` (`pcbnew/router/pns_algo_base.h:63-68`), and both the line placer and the multi dragger install a logger (`pcbnew/router/pns_line_placer.cpp:932`, `pcbnew/router/pns_multi_dragger.cpp:275`). But `pcbnew/router/pns_shove.cpp` never calls `Logger()`; the file opens with `// fixme - move all logger calls to debug decorator` at `:46`, and that migration is complete. The `LOGGER` (`pcbnew/router/pns_logger.h`) is used by the router at a higher level to record events for replay.

`NodeStats(dbg, label, node)` (`pcbnew/router/pns_utils.cpp:544-558`) dumps a node's added and removed items as a debug group. Shove has five commented-out calls to it (`pcbnew/router/pns_shove.cpp:2433`, `:2467`, `:2538`, `:2549`, `:2561`) marking the points a developer most often wants to inspect: right after `Branch()`, after adding a head, before pushing the line stack, after the main loop, and before optimization. Those are good default breakpoints for a port.

## 8. Rust mapping notes

### 8.1 Node ownership and the springback stack

The C++ model is a tree of `NODE`s with raw-pointer parent and child links, where a child overlays its parent (`NODE::Branch` at `pcbnew/router/pns_node.cpp:157-188` clones the spatial index, joints and override map only when the parent is not the root). Deleting a node frees every `ITEM` it owns and silently invalidates every `LINE` still holding those links; the shove code has to defend against that by hand (`pcbnew/router/pns_shove.cpp:2593-2596`).

Recommended shape in Rust:

```
struct World { nodes: Arena<Node>, root: NodeId }
struct Node { parent: Option<NodeId>, depth: u32, items: Arena<Item>, index: SpatialIndex, overrides: HashSet<ItemId>, joints: HashMap<JointKey, Joint> }
struct SpringbackFrame { node: NodeId, dragged_vias: Vec<Option<ViaHandle>>, affected_area: Option<Box2i>, locked: bool }
struct Shove { stack: Vec<SpringbackFrame>, ... }
```

A `Vec<SpringbackFrame>` over arena-allocated nodes is sufficient and is strictly better than the C++ arrangement, because dropping a frame becomes "remove the `NodeId` from the arena" and every stale `ItemId` then fails a lookup instead of dangling. Make `ItemId` a generational index so a stale reference is a detectable error rather than a silent alias. Do not use `Rc<RefCell<Node>>`: the algorithm mutates the current node while holding references derived from it, which is exactly the pattern that makes `RefCell` panic at runtime.

Drop the dead `SPRINGBACK_TAG` fields (`m_length`, `m_p`, `m_seq`; `pcbnew/router/pns_shove.h:203`, `:205`, `:208`). Keep `locked` and `dragged_vias`.

The `SpringbackDoNotTouchNode` pin (`pcbnew/router/pns_shove.h:286`, set at `pcbnew/router/pns_line_placer.cpp:938`) is a workaround for the caller holding a raw pointer into a node the shove may delete. With generational ids the pin is still needed semantically (the placer's end item must stay resolvable), but the failure mode becomes a clean `None` rather than a crash. Model it as `pinned: Option<NodeId>` and check it in `reduce_springback`.

### 8.2 The LINE stack

`LINE` in C++ is a value type that *also* holds `Vec<*LinkedItem>` into a node (`pcbnew/router/pns_link_holder.h:66-67`). Copying a `LINE` copies the link pointers, so two copies alias the same node items; `LINE::Rank()` exploits this by reading through the links (`pcbnew/router/pns_line.cpp:1449-1466`), and `LINE::SetRank` writes through them (`:1439-1446`).

In Rust, split the two concepts:

```
struct LineShape { chain: PolyLine, width: i32, layer: LayerRange, net: NetId }   // pure value
struct LineRef   { shape: LineShape, links: SmallVec<[ItemId; 8]>, via: Option<ItemId> }
```

Then `rank(node, line)` is a free function that takes `&Node` and folds `min` over `line.links`, and `set_rank(node, line, r)` takes `&mut Node`. That removes the "mutate items in place while iterating a copy" hazard entirely, because every mutation is visibly routed through `&mut Node`.

`m_lineStack` should be a `Vec<LineRef>` with three explicit operations rather than the current four ad-hoc mutations: `push(line)`, `push_below_top(line)` (the `aKeepCurrentOnTop` case at `pcbnew/router/pns_shove.cpp:1465-1468`), and `retain_not_referencing(item_id)` (the `unwindLineStack` case at `:1392-1428`). The via-degradation special case inside `unwindLineStack` (`:1399-1418`) must be preserved; write it as an explicit `enum UnwindAction { Keep, DegradeToViaStub(ItemId), Drop }` so it is testable in isolation.

`m_optimizerQueue` should not be a `Vec<LineRef>` scanned linearly. Model it as `IndexMap<RootLineId, LineRef>` keyed by root-line identity, which makes `pruneLineFromOptimizerQueue` (`pcbnew/router/pns_shove.cpp:1482-1507`, currently O(queue x links) per push) an O(1) replace, and makes iteration order insertion-deterministic. Keep the asymmetry deliberately: a line that goes clean must stay in the queue (the raw `pop_back` at `:1695`), a line that is unwound must leave it.

### 8.3 Avoiding "mutate in place while iterating"

Three specific C++ patterns to restructure:

1. `shoveIteration` copies the stack top by value (`pcbnew/router/pns_shove.cpp:1635`) and then hands `&currentLine` to handlers that call `replaceLine` on the node, invalidating the copy's links. In Rust, pass a `LineId` into the stack (an index), have handlers return a small `enum ShoveEffect { ReplaceLine{ old: LineId, new: LineShape, rank: i32 }, MoveVia{ via: ItemId, to: Point, rank: i32 }, PushLine(..), PopLine, Fail }`, and apply the effects in the caller. That is the single biggest structural change and it is worth doing.
2. `unwindLineStack` erases from `m_lineStack` and `m_optimizerQueue` from inside handlers, several times per handler (for example `pcbnew/router/pns_shove.cpp:1313` and again `:1378` inside `onReverseCollidingVia`). With the effect-list design these become explicit effects too, applied once at the end of the iteration.
3. `replaceLine` re-points the root-line index from old link uids to new link uids while the caller still holds the old `LINE` (`pcbnew/router/pns_shove.cpp:157-161`). Make the root-line index own its mapping and expose `rebind(old_links, new_links)` as one atomic operation.

Return a `Result<ShoveStatus, ShoveError>` rather than the five-valued enum; `SH_NULL` and `SH_HEAD_MODIFIED` are dead (see 1.1), so the live set is `{ Ok, Incomplete, TryWalk }`.

### 8.4 Determinism and testability

Make the following explicit, seeded or ordered, because the C++ leaves them to pointer values (details in section 9):

- Obstacle selection must be tie-broken on a stable key (item uid, then the intersection distance), never on address. Give every item a `u64` uid at construction from a per-`World` counter, not a process-global one, and use `(dist, uid)` as the sort key.
- Every container that is iterated must be insertion-ordered (`Vec`, `IndexMap`, `IndexSet`), never `HashMap`/`HashSet`. Where a hash map is genuinely only used for point lookups (the root-line index), that is fine, but assert in tests that it is never iterated.
- Take `cornerMode` and every other setting as an explicit parameter. The C++ reaches for `ROUTER::GetInstance()->Settings()` from inside `mergeStep` (`pcbnew/router/pns_optimizer.cpp:858`) and `fanoutCleanup` (`:1278`), which makes those functions untestable without a global router.
- Replace the wall-clock `TIME_LIMIT` (`pcbnew/router/time_limit.cpp:87-90`) with an injectable budget: a `Budget { max_iterations: u32, deadline: Option<Instant> }` where tests set only the iteration count. A wall-clock limit inside the algorithm makes results machine-dependent and non-reproducible, which is unacceptable for a golden-file test suite.
- Reset budgets once per `Run`, not once per head. The C++ per-head reset (`pcbnew/router/pns_shove.cpp:1893-1895` called from `:2547`) means the total work scales with head count in a way the caller cannot see.

A good test harness: serialise the world plus the head set plus the settings into a fixture, run the shove, and compare the resulting item set (sorted by uid) and the per-iteration effect list against a golden file. The effect-list design from 8.3 makes the second comparison possible and is far more diagnostic than comparing final geometry.

### 8.5 Suggested port order

**Phase 1, single line shove without vias.** Port, in this order: `OctagonalHull` and `SegmentHull` (`pcbnew/router/pns_utils.cpp:40-68`, `:181-286`); `LINE::Walkaround` (`pcbnew/router/pns_line.cpp:297+`) and `HullIntersection` (`pcbnew/router/pns_utils.cpp:395-475`); `checkShoveDirection` (`pcbnew/router/pns_shove.cpp:243-268`); `shoveLineToHullSet` (`:328-513`); the segment-only branch of `ShoveObstacleLine` (`:521-633`); `onCollidingSegment` (`:639-683`); `pushLineStack` / `popLineStack` / `unwindLineStack` without the via special case; `shoveIteration` restricted to `SEGMENT_T` and `SOLID_T`; `shoveMainLoop`; a `Run` with exactly one line head. That is a self-contained, testable router that already exhibits the rank mechanism and the hull walk.

**Phase 2, springback and the optimizer.** `SPRINGBACK_TAG` stack, `pushSpringback` / `reduceSpringback`; the root-line index; `COST_ESTIMATOR::CornerCost` and `mergeStep` / `mergeFull`; `mergeObtuse`; `mergeColinear`; `AREA_CONSTRAINT` and `PRESERVE_VERTEX_CONSTRAINT`. Skip `LIMIT_CORNER_COUNT` and `RESTRICT_VERTEX_RANGE`, which are no-ops (4.1). Skip the dead collision cache entirely (4.3).

**Phase 3, solids.** `TOPOLOGY::AssembleCluster`, `WALKAROUND`, `onCollidingSolid`, the `SH_TRY_WALK` escalation, `SOLID::Hull` including the compound-shape union.

**Phase 4, vias.** `VIA::Hull`, `shoveLineFromLoneVia`, `pushOrShoveVia` with the fanout drag, `onCollidingVia`, `onReverseCollidingVia`, `fixupViaCollisions`, `patchTadpoleVia`, the via-stub degradation in `unwindLineStack`, and via-drag heads in `Run`.

**Phase 5, cosmetics.** `runSmartPads` and the breakout generators; `fanoutCleanup`; arcs (`ArcHull`, `onCollidingArc`, and the arc guards scattered through the optimizer at `pcbnew/router/pns_optimizer.cpp:660`, `:713`, `:717`, `:724`, `:728`, `:745`, `:868`).

Do not port at all: `ShoveDraggingVia` (no definition), `sanityCheck` (no callers), `removeHead` free function (`pcbnew/router/pns_shove.cpp:2270-2277`, no callers), the optimizer's `CACHE_VISITOR` / `m_cache` / `m_cacheTags` / `cacheAdd` / `ClearCache` / `MaxCachedItems`, `ANGLE_CONSTRAINT_45`, `RESTRICT_VERTEX_RANGE_CONSTRAINT`, `CORNER_COUNT_LIMIT_CONSTRAINT` in its current no-op form, `COST_ESTIMATOR::Add`/`Remove`/`Replace`/`IsBetter` (unused), `Tighten` / `tightenSegment` / `shovedArea` (unused), and the duplicate `SHOVE_POLICY` enum at `pcbnew/router/pns_shove.cpp:2607-2615`.

## 9. Where the C++ depends on pointer order or unspecified iteration order

These are the places a port must decide deliberately, because the C++ result can differ between runs, builds and allocators.

1. **`std::set<OBSTACLE>` ordered by raw pointer.** `OBSTACLE::operator<` compares `(uintptr_t)m_head` then `(uintptr_t)m_item` (`pcbnew/router/pns_node.h:103-110`). `NODE::QueryColliding` fills that set (`pcbnew/router/pns_node.cpp:270`, `:294`) and `NODE::NearestObstacle` flattens it into a vector in that order (`pcbnew/router/pns_node.cpp:321`). This is the single most consequential ordering dependency in the whole algorithm.

2. **Nearest-obstacle tie-break.** The winner scan uses a strict `<` on distance (`pcbnew/router/pns_node.cpp:460`), so among obstacles at equal path distance the *first in pointer order* wins. Equal distances are common: a via grid, a pad row, two parallel tracks entering the hull at the same offset. Fix in the port by sorting on `(dist, item_uid)`.

3. **Zero-distance early exit.** `if( results[i].dist == 0 ) break;` (`pcbnew/router/pns_node.cpp:466-467`) stops the scan at the first zero-distance hit in pointer order, discarding any other zero-distance obstacle.

4. **The no-intersection fallback.** `if( nearest.m_distFirst == INT_MAX ) nearest = obstacles[0];` (`pcbnew/router/pns_node.cpp:471-472`) picks a purely pointer-ordered arbitrary obstacle when the hull produced no valid intersection.

5. **`INDEX::m_allItems` is an `std::unordered_set<ITEM*>`** (`pcbnew/router/pns_index.h:51`, `:156`, iterated via `begin()`/`end()` at `:146-147`). Everything that walks a node's items walks it in hash-of-pointer order: `NODE::GetUpdatedItems` (`pcbnew/router/pns_node.cpp:1574-1575`), `NODE::ClearRanks` (`:1684`), `NODE::Commit` (`:1630`), `NODE::RemoveByMarker` (`:1696`). In shove this reaches `removeHeads` (`pcbnew/router/pns_shove.cpp:2283-2292`) and `pruneRootLines` (`:906-916`). Those two are order-insensitive as written, but `NODE::Commit` re-adds items in that order, which sets the *insertion order* of the next node's joint link lists, and joint link lists are iterated in insertion order in several essential places (next item).

6. **Joint link-list order decides via fanout order.** `JOINT::LinkList()` returns `ITEM_SET::CItems()`, a `std::vector<ITEM*>` (`pcbnew/router/pns_joint.h:303-306`, `pcbnew/router/pns_itemset.h:96`), so it is insertion-ordered, not pointer-ordered. But the insertion order is inherited from the index walk in point 5. `pushOrShoveVia` iterates it to build the fanout list (`pcbnew/router/pns_shove.cpp:1081-1105`) and then pushes those lines onto the line stack in that order (`:1129-1165`), which changes the order in which subsequent iterations resolve their collisions. `onReverseCollidingVia` (`:1315-1346`) and `onCollidingSolid` (`:789-796`) iterate the same list.

7. **`OPTIMIZER::findPadOrVia` returns the first `VIA_T | SOLID_T` in joint link order** (`pcbnew/router/pns_optimizer.cpp:1100-1104`). A point with both a pad and a via resolves to whichever was inserted first.

8. **`m_rootLineHistory` is an `std::unordered_map`** (`pcbnew/router/pns_shove.h:282`). It is never iterated in this revision, only point-looked-up and single-key-erased, so it is currently safe. `findRootLine(const LINE&)` does iterate the *line's own links* and returns the first hit (`pcbnew/router/pns_shove.cpp:1953-1959`); link order is insertion order, so that is deterministic given point 6.

9. **`NODE::releaseChildren` copies `std::set<NODE*> kids`** (`pcbnew/router/pns_node.cpp:1582`) and destroys in pointer order. Destruction order is observable through the `ClearCacheForItems` calls at `pcbnew/router/pns_node.cpp:127` and `:1610`.

10. **`LINKED_ITEM::genNextUid` uses a non-atomic process-global counter** (`pcbnew/router/pns_item.cpp:362-366`, with a "fixme: make atomic" comment). Uids are therefore stable within a single-threaded session but depend on everything allocated earlier in the process. A port should scope the counter to the `World` so a fixture always produces the same uids.

11. **`WALKAROUND::m_restrictedSet` and `m_processedItems` are `std::set<const ITEM*>` / `std::set<ITEM*>`** (`pcbnew/router/pns_walkaround.h:152`, `:161`), reached from `onCollidingSolid` through `RestrictToCluster` (`pcbnew/router/pns_shove.cpp:816`).

12. **The base `RULE_RESOLVER::HullCache` returns a reference to a function-local `static`** (`pcbnew/router/pns_node.h:176-182`). Not an ordering issue but the same class of hazard: any two live references alias, and it is not thread safe. The production override is a real map (`pcbnew/router/pns_kicad_iface.cpp:832-848`) keyed on the raw `ITEM*`, so cache hits depend on address reuse after a free.

13. **`NearestObstacle` dispatches to a thread pool above 8 obstacles** (`pcbnew/router/pns_node.cpp:437-445`). The per-obstacle work writes only into `results[i]`, so the parallelism itself is deterministic, but it forced the sequential hull-cache pre-pass at `:346-376`; any port that reorders those phases reintroduces the data race.

Everything else in `pns_shove.cpp` and `pns_optimizer.cpp` iterates `std::vector` and is order-deterministic given the above inputs.
