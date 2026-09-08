# PNS host integration contract, GUI event flow, third party integration, and test infrastructure

Reference note for a from-scratch Rust reimplementation of KiCad's push-and-shove router and its integration into LibrePCB.

## 0. Scope and source provenance

Citations in this note are `path:line` relative to one of two trees. Paths beginning `pcbnew/`, `qa/`, `libs/kimath/`, `common/` or `include/` are relative to `/home/Tubbles/dev/ref/kicad/`, a sparse checkout of KiCad master at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85` (2026-09-07). Paths beginning `src/` or `3rd_party/` are relative to `/home/Tubbles/dev/ref/horizon/`, a shallow clone of Horizon EDA master. Paths beginning `libs/librepcb/` are relative to the LibrePCB working tree at `/var/home/Tubbles/dev/librepcb`.

Citation convention: a bare `:1234` continues the file named most recently in the same section or paragraph. Where a section switches files, the full path is repeated.

Two names recur and are worth fixing up front. `PNS::ITEM` is the router's own geometric object (SEGMENT, ARC, VIA, SOLID, LINE, HOLE, DIFF_PAIR). `BOARD_ITEM` is the host's object. The whole host contract is the translation layer between the two, plus the design-rule oracle. `PNS::NET_HANDLE` is literally `typedef void* NET_HANDLE;` (`pcbnew/router/pns_item.h:55`), an opaque host-owned token that the router only ever compares for equality and passes back to the host.

The router uses integer nanometre coordinates throughout (`VECTOR2I`), and "PNS layers" are a dense zero-based copper-layer index, not the host's layer enum. Both facts are structural and are discussed in section 1.7.

## 1. ROUTER_IFACE on master

### 1.1 The complete virtual list

The interface is declared at `pcbnew/router/pns_router.h:91` through `pcbnew/router/pns_router.h:152`. Every method except `GetNetBoardLength` and the non-virtual `GetViaLayerRange` is pure virtual, so a host must supply all 32 of them (31 pure virtuals plus `GetNetBoardLength`) even if most are stubs. KiCad itself proves this: `PNS_KICAD_IFACE_BASE` (`pcbnew/router/pns_kicad_iface.h:58`) stubs out ten of them with empty bodies or constant returns, and only the GUI subclass `PNS_KICAD_IFACE` (`pcbnew/router/pns_kicad_iface.h:147`) implements them for real.

The grouping below is by responsibility, not by declaration order.

**World synchronisation (1 method).**

```cpp
virtual void SyncWorld( NODE* aNode ) = 0;              // pns_router.h:97
```

**Commit / write-back (4 methods).**

```cpp
virtual void AddItem( ITEM* aItem ) = 0;                // pns_router.h:98
virtual void UpdateItem( ITEM* aItem ) = 0;             // pns_router.h:99
virtual void RemoveItem( ITEM* aItem ) = 0;             // pns_router.h:100
virtual void Commit() = 0;                              // pns_router.h:111
```

**Interactive preview / view decoration (5 methods).**

```cpp
virtual void DisplayItem( const ITEM*, int aClearance, bool aEdit = false, int aFlags = 0 ) = 0;  // pns_router.h:106
virtual void DisplayPathLine( const SHAPE_LINE_CHAIN&, int aImportance ) = 0;                     // pns_router.h:108
virtual void DisplayRatline( const SHAPE_LINE_CHAIN&, NET_HANDLE ) = 0;                           // pns_router.h:109
virtual void HideItem( ITEM* aItem ) = 0;                                                         // pns_router.h:110
virtual void EraseView() = 0;                                                                     // pns_router.h:114
```

**Visibility predicates (2 methods).**

```cpp
virtual bool IsAnyLayerVisible( const PNS_LAYER_RANGE& ) const = 0;   // pns_router.h:101
virtual bool IsItemVisible( const PNS::ITEM* ) const = 0;             // pns_router.h:102
```

**Layer semantics (5 methods).**

```cpp
virtual bool IsFlashedOnLayer( const PNS::ITEM*, int aLayer ) const = 0;                // pns_router.h:103
virtual bool IsFlashedOnLayer( const PNS::ITEM*, const PNS_LAYER_RANGE& ) const = 0;    // pns_router.h:104
virtual bool IsPNSCopperLayer( int aPNSLayer ) const = 0;                               // pns_router.h:105
virtual PCB_LAYER_ID GetBoardLayerFromPNSLayer( int aLayer ) const = 0;                 // pns_router.h:133
virtual int GetPNSLayerFromBoardLayer( PCB_LAYER_ID aLayer ) const = 0;                 // pns_router.h:134
```

**Net identity (4 methods).**

```cpp
virtual int GetNetCode( NET_HANDLE ) const = 0;         // pns_router.h:115
virtual wxString GetNetName( NET_HANDLE ) const = 0;    // pns_router.h:116
virtual void UpdateNet( NET_HANDLE ) = 0;               // pns_router.h:117
virtual NET_HANDLE GetOrphanedNetHandle() = 0;          // pns_router.h:118
```

**Design-rule and geometry parameters (2 methods).**

```cpp
virtual bool ImportSizes( SIZES_SETTINGS&, ITEM* aStartItem, NET_HANDLE, VECTOR2D aStartPosition ) = 0;  // pns_router.h:112
virtual int  StackupHeight( int aFirstLayer, int aSecondLayer ) const = 0;                               // pns_router.h:113
```

**Sub-oracles (3 methods).**

```cpp
virtual PNS::NODE* GetWorld() const = 0;                // pns_router.h:119
virtual RULE_RESOLVER* GetRuleResolver() = 0;           // pns_router.h:121
virtual DEBUG_DECORATOR* GetDebugDecorator() = 0;       // pns_router.h:122
```

**Length and delay for tuning (6 methods).**

```cpp
virtual long long int CalculateRoutedPathLength( const ITEM_SET&, const SOLID* aStartPad,
                                                 const SOLID* aEndPad, const NETCLASS* ) = 0;   // pns_router.h:124
virtual int64_t CalculateRoutedPathDelay( ... ) = 0;                                            // pns_router.h:126
virtual int64_t CalculateLengthForDelay( int64_t aDesiredDelay, int aWidth, bool aIsDiffPairCoupled,
                                         int aDiffPairCouplingGap, int aPNSLayer, const NETCLASS* ) = 0;  // pns_router.h:128
virtual int64_t CalculateDelayForShapeLineChain( const SHAPE_LINE_CHAIN&, ... ) = 0;            // pns_router.h:130
virtual bool GetSignalAggregate( NET_HANDLE aNetP, NET_HANDLE aNetN,
                                 long long& aExtraLength, long long& aExtraDelay ) const = 0;   // pns_router.h:135
virtual long long GetNetBoardLength( NET_HANDLE aNet ) const { return 0; }                      // pns_router.h:138
```

**One non-virtual helper.** `GetViaLayerRange( const SIZES_SETTINGS& )` at `pcbnew/router/pns_router.h:144` is implemented in the header in terms of `GetPNSLayerFromBoardLayer`, and hard-codes the rule that a `VIATYPE::THROUGH` via always spans `F_Cu` to `B_Cu` regardless of the layer pair in the sizes, whereas blind, buried and micro vias use the pair. Callers are `pcbnew/router/pns_line_placer.cpp:78` and `pcbnew/router/pns_diff_pair_placer.cpp:76`. In a Rust port this belongs on the router side, not the host side, since it is pure logic over two host-supplied primitives.

### 1.2 SyncWorld: who calls it, when, and what it must guarantee

`ROUTER::SyncWorld()` (`pcbnew/router/pns_router.cpp:95`) is the only caller:

```cpp
void ROUTER::SyncWorld()
{
    ClearWorld();
    m_world = std::make_unique<NODE>();
    m_world->BeginBulkAdd();
    m_iface->SyncWorld( m_world.get() );
    m_world->FinalizeBulkAdd();
    m_world->FixupVirtualVias();
}
```

Three things follow that a reimplementation must not lose. First, the host is handed a fresh, empty root `NODE` and populates it by calling `NODE::Add`, so the call is push-style, not pull-style. Second, the whole population happens inside a bulk-add window, meaning the spatial index is built once at the end rather than incrementally; a Rust port that takes a plain-data snapshot instead of a callback gets this for free. Third, `FixupVirtualVias()` runs after the host is done, so the router synthesises extra items the host never described. Any host that tries to enumerate "the items I put in" and match them against "the items in the world" will be off by the virtual vias.

The host implementation must also install the rule resolver and the world's `m_maxClearance` before returning. KiCad does both at the tail of `SyncWorld` (`pcbnew/router/pns_kicad_iface.cpp:2448` to `pcbnew/router/pns_kicad_iface.cpp:2452`):

```cpp
delete m_ruleResolver;
m_ruleResolver = new PNS_PCBNEW_RULE_RESOLVER( m_board, this );
aWorld->SetRuleResolver( m_ruleResolver );
aWorld->SetMaxClearance( worstClearance + m_ruleResolver->ClearanceEpsilon() );
```

`m_maxClearance` is not cosmetic. It is the inflation radius used for every spatial index query (`pcbnew/router/pns_node.cpp:285`, `pcbnew/router/pns_node.cpp:291`, `pcbnew/router/pns_node.cpp:581`, `pcbnew/router/pns_node.cpp:588`). If the host understates it, obstacles that are genuinely within clearance are never returned by the broad phase and the router silently routes through them. The default if nobody sets it is a hard-coded `800000` nm with a "fixme" comment (`pcbnew/router/pns_node.cpp:62`). Getting this wrong is one of the easiest ways to produce a router that "mostly works" and then produces DRC violations on boards with one unusually large clearance rule.

`worstClearance` starts as `m_board->GetMaxClearanceValue()` and is then raised by any per-pad clearance override encountered during the pad loop (`pcbnew/router/pns_kicad_iface.cpp:2300`, `pcbnew/router/pns_kicad_iface.cpp:2364` to `pcbnew/router/pns_kicad_iface.cpp:2367`). LibrePCB has exactly the same shape of problem: per-pad `copperClearance` overrides exist at `libs/librepcb/core/geometry/pad.h:112`, so the LibrePCB implementation must fold them into the same maximum.

Called from the GUI, `SyncWorld` runs once per routing session start, not per mouse move.

### 1.3 AddItem, UpdateItem, RemoveItem, Commit

These four are called only from `ROUTER::CommitRouting( NODE* )` (`pcbnew/router/pns_router.cpp:862` through `pcbnew/router/pns_router.cpp:912`). The sequence is worth reading closely because the diff classification is done by the router, not by the host:

```
aNode->GetUpdatedItems( removed, added );
for item in removed:
    if item has a Parent and some item in `added` has the SAME Parent pointer:
        move that added item into `changed`, erase it from `added`
    else if not item->IsVirtual():
        iface->RemoveItem( item )
for item in added:      if not virtual: iface->AddItem( item )
for item in changed:    if not virtual: iface->UpdateItem( item )
iface->Commit()
m_world->Commit( aNode )
```

The parent-pointer identity match at `pcbnew/router/pns_router.cpp:884` is what preserves a track's UUID, net-tie membership, solder-mask override and so on across a shove. Without it, every shoved segment would be a delete plus an insert, which in KiCad terms would churn UUIDs and break the undo diff. A Rust port must keep an equivalent stable host-side identity on each item and must classify remove-plus-add-with-same-identity as an update.

`IsVirtual()` items are skipped entirely; those are the synthetic vias from `FixupVirtualVias` and similar, and must never reach the host.

`iface->Commit()` is the transaction boundary. In KiCad it does three things beyond pushing the transaction (`pcbnew/router/pns_kicad_iface.cpp:2911` to `pcbnew/router/pns_kicad_iface.cpp:2958`): it applies accumulated footprint offsets from component dragging, it re-parents newly created board items into the group that the replaced item belonged to, and only then calls `m_commit->Push( _( "Routing" ), m_commitFlags | SKIP_ENTERED_GROUP )` and allocates a fresh `BOARD_COMMIT` for the next segment. So one `Commit()` equals one undo entry, and the router calls it once per fixed click, not once per session. See section 4 for what that means for undo granularity.

The base class stubs all four (`pcbnew/router/pns_kicad_iface.cpp:2620`, `:2648`, `:2752`, and `Commit()` inline empty at `pcbnew/router/pns_kicad_iface.h:81`). That is what makes headless testing possible: a test can sync a world, run the algorithms, and read the resulting `NODE` directly without ever writing back.

Two subtleties in `AddItem`. It back-patches the router item's parent pointer so subsequent `UpdateItem` calls in the same session find the right board item: `aItem->SetParent( boardItem )` at `pcbnew/router/pns_kicad_iface.cpp:2903`. And `createBoardItem` substitutes an orphan net when the router item has none, `net = NETINFO_LIST::OrphanedItem()` at `pcbnew/router/pns_kicad_iface.cpp:2763`, which is the same object `GetOrphanedNetHandle()` returns. A `SOLID` reaching `createBoardItem` is not a board item at all; it records a pad displacement into `m_fpOffsets` and returns null (`pcbnew/router/pns_kicad_iface.cpp:2849` to `pcbnew/router/pns_kicad_iface.cpp:2856`).

### 1.4 DisplayItem, DisplayPathLine, DisplayRatline, HideItem, EraseView

These are called on every mouse move, from inside the router, and are the reason the interface is a callback interface rather than a pure snapshot API.

`EraseView()` is called first thing in both `moveDragging` (`pcbnew/router/pns_router.cpp:658`) and `movePlacing` (`pcbnew/router/pns_router.cpp:791`), and again in `StopRouting` (`pcbnew/router/pns_router.cpp:987`) and `ClearViewDecorations` (`pcbnew/router/pns_router.cpp:997`). The contract is "drop every preview object you were given since the last EraseView", plus, in KiCad, un-hide every board item that `HideItem` hid (`pcbnew/router/pns_kicad_iface.cpp:2456` to `pcbnew/router/pns_kicad_iface.cpp:2471`). So the preview set is fully rebuilt on each move; there is no incremental preview protocol.

`DisplayItem` has four call sites, and the `aFlags` argument distinguishes them:

- `pcbnew/router/pns_router.cpp:695`, inside `markViolations`, for an item that the current head collides with. The clearance passed is the actual resolved clearance for that pair, so the host can draw the violated clearance ring. Note the special handling just above: if the marked item is multilayer and the current item is not, a clone is retargeted to the current item's layer (`pcbnew/router/pns_router.cpp:682` to `pcbnew/router/pns_router.cpp:686`), and compound-shape primitives are drawn without hiding the original (`pcbnew/router/pns_router.cpp:688` to `pcbnew/router/pns_router.cpp:693`).
- `pcbnew/router/pns_router.cpp:771`, inside `updateView`, for every item the current `NODE` added relative to its parent, with `aEdit = aDragging`.
- `pcbnew/router/pns_router.cpp:804`, for the routing head line itself, with `aFlags = PNS_HEAD_TRACE`.
- `pcbnew/router/pns_router.cpp:821`, for the via at the end of the head line, again with `PNS_HEAD_TRACE`.

The flag constants are `PNS_HEAD_TRACE 1`, `PNS_HOVER_ITEM 2`, `PNS_SEMI_SOLID 4`, `PNS_COLLISION 8` (`pcbnew/router/router_preview_item.h:50` to `pcbnew/router/router_preview_item.h:53`). `PNS_SEMI_SOLID` is added by the host itself, not the router, when the item's parent is a rule area (`pcbnew/router/pns_kicad_iface.cpp:2485` to `pcbnew/router/pns_kicad_iface.cpp:2489`).

There is a non-obvious piece of arithmetic in the head-via case at `pcbnew/router/pns_router.cpp:811` to `pcbnew/router/pns_router.cpp:819`. The clearance drawn for a via is normally the copper clearance, but if the via's hole clearance exceeds the annular ring, the excess hole clearance is drawn instead:

```cpp
int holeClearance = GetRuleResolver()->Clearance( via.Hole(), nullptr );
int annularWidth = std::max( 0, via.Diameter( l->Layer() ) - via.Drill() ) / 2;
int excessHoleClearance = holeClearance - annularWidth;
if( excessHoleClearance > clearance )
    clearance = excessHoleClearance;
```

This is display-only, but it tells you that `Clearance( item, nullptr )` with a null second argument is a supported and used query meaning "this item's own worst-case clearance against anything".

`DisplayPathLine` is used only by the length tuners, to highlight the path being tuned: `pcbnew/router/pns_meander_placer.cpp:307`, `pcbnew/router/pns_dp_meander_placer.cpp:326` and `:336`, `pcbnew/router/pns_meander_skew_placer.cpp:213` and `:223`. `aImportance` is 0 or 1 and KiCad maps it to yellow at 0.6 alpha for 1 and grey at 0.6 alpha for 0 (`pcbnew/router/pns_kicad_iface.cpp:2536` to `pcbnew/router/pns_kicad_iface.cpp:2539`). For skew tuning the two members of the pair get importance 1 and 0 swapped depending on which is being tuned, so "importance" really means "is this the path currently under the cursor's control".

`DisplayRatline` is called from `pcbnew/router/pns_line_placer.cpp:2028` and `pcbnew/router/pns_diff_pair_placer.cpp:920` and `:923`. It draws the guide line from the head to the nearest unconnected anchor. KiCad's implementation is entirely about colour selection from net colours and net classes (`pcbnew/router/pns_kicad_iface.cpp:2548` to `pcbnew/router/pns_kicad_iface.cpp:2591`); the geometry is just the chain.

`HideItem` is called from `updateView` for every item the current node *removed* relative to its parent (`pcbnew/router/pns_router.cpp:775`), and from `isStartingPointRoutable` for items highlighted as blocking the start point (`pcbnew/router/pns_router.cpp:334`) and the analogous diff-pair path (`pcbnew/router/pns_router.cpp:422`). Semantically it means "stop drawing the committed board version of this item, because I am drawing a moved copy of it". KiCad also hides any teardrop zone overlapping the hidden item (`pcbnew/router/pns_kicad_iface.cpp:2606` to `pcbnew/router/pns_kicad_iface.cpp:2615`), which is a good example of a host-specific cosmetic concern the router knows nothing about.

### 1.5 IsAnyLayerVisible and IsItemVisible

Both exist purely so the tool does not snap to or start routing from something the user cannot see. `IsPNSCopperLayer` and `IsAnyLayerVisible` are consulted back to back in `pickSingleItem` (`pcbnew/router/pns_tool_base.cpp:150` and `pcbnew/router/pns_tool_base.cpp:153`), and `IsItemVisible` in `snapToItem` (`pcbnew/router/pns_tool_base.cpp:452`). No router algorithm calls either. `PNS_KICAD_IFACE_BASE` returns `true` unconditionally for both (`pcbnew/router/pns_kicad_iface.h:67` and `pcbnew/router/pns_kicad_iface.h:70`), which is the correct headless behaviour.

`PNS_KICAD_IFACE::IsItemVisible` at `pcbnew/router/pns_kicad_iface.cpp:2261` is more subtle than "is the layer on": it honours high-contrast mode, per-item level-of-detail against the current zoom (`item->ViewGetLOD( layer, m_view ) < m_view->GetScale()`), and, importantly, reports an item as visible if the router itself hid it via `HideItem` (`pcbnew/router/pns_kicad_iface.cpp:2285`). Without that last clause the tool would refuse to snap to the very item it is currently dragging.

### 1.6 IsFlashedOnLayer and IsPNSCopperLayer

`IsFlashedOnLayer` is the surprise. It is not a display concern. It is consulted in the collision inner loop, `ITEM::collideSimple`, at `pcbnew/router/pns_item.cpp:206` and `pcbnew/router/pns_item.cpp:210`:

```cpp
else if( iface && !iface->IsFlashedOnLayer( this, aHead->Layers() ) )
    clearance = -1;
else if( iface && !iface->IsFlashedOnLayer( aHead, Layers() ) )
    clearance = -1;
```

A clearance of `-1` means "no collision at all". So this predicate is what makes a via with removed unconnected annular rings, or a pad that does not flash on an inner layer, stop being an obstacle on that layer while its hole remains one. It is also used in `PNS::VIA::Shape` at `pcbnew/router/pns_via.cpp:243` to decide whether a via has any copper on a given layer.

Getting this wrong in either direction is a correctness bug, not a cosmetic one. Return `true` always and the router treats non-flashed annular rings as copper obstacles, refusing legal routes on inner layers of any board that uses "remove unused pads". Return `false` too eagerly and the router will happily route through real copper. For a first LibrePCB integration where every pad and via is flashed on every layer it spans, returning `aItem->Layers().Overlaps( aLayer )` is correct and safe, but the hook must exist because LibrePCB does have blind and buried via support and will eventually want the same behaviour.

The KiCad implementation delegates to the parent board item when there is one (`via->FlashLayer(...)` at `pcbnew/router/pns_kicad_iface.cpp:2183`, `pad->FlashLayer(...)` at `:2190`), falls back to `PNS::VIA::ConnectsLayer` for parentless router-created vias (`:2199`), and otherwise to a plain layer-range overlap (`:2201`). The range-taking overload at `pcbnew/router/pns_kicad_iface.cpp:2205` intersects the item's range with the query range first and then answers "is it flashed on *any* layer of the intersection". Note the asymmetry: the single-layer overload short-circuits to `true` for `aLayer < 0` (`pcbnew/router/pns_kicad_iface.cpp:2172`), meaning "no layer context, assume flashed".

`IsPNSCopperLayer( int )` is a straight predicate over the PNS layer index (`pcbnew/router/pns_kicad_iface.cpp:2141`), used once, in `pickSingleItem` (`pcbnew/router/pns_tool_base.cpp:150`), to reject candidates that live on non-copper layers. In a host whose PNS layer space contains only copper layers by construction, which is what a LibrePCB mapping should do, this is a constant `true`.

### 1.7 Layer mapping

The router's layer space is a dense zero-based copper index: 0 is the top layer, `copperLayerCount - 1` is the bottom, inner layers are consecutive in between. KiCad's mapping to and from its own sparse `PCB_LAYER_ID` enum is at `pcbnew/router/pns_kicad_iface.cpp:3039` and `pcbnew/router/pns_kicad_iface.cpp:3054`:

```cpp
PCB_LAYER_ID GetBoardLayerFromPNSLayer( int aLayer ) const
{
    if( aLayer < 0 || aLayer >= m_board->GetCopperLayerCount() ) return UNDEFINED_LAYER;
    if( aLayer == 0 ) return F_Cu;
    if( aLayer == m_board->GetCopperLayerCount() - 1 ) return B_Cu;
    return static_cast<PCB_LAYER_ID>( ( aLayer + 1 ) * 2 );
}
```

The inverse is `( aLayer / 2 ) - 1` for inner layers (`pcbnew/router/pns_kicad_iface.cpp:3065`). Both are pure functions of the copper layer count.

Two consequences matter for a port. First, the mapping is board-dependent, so a host cannot precompute it as a constant; adding a layer to the stack renumbers everything. Second, `PNS_LAYER_RANGE` is used as a *closed* interval `[Start, End]`, and code such as `syncPad` explicitly guards against a degenerate inverted range being silently swapped: the `FRONT_INNER_BACK` guard at `pcbnew/router/pns_kicad_iface.cpp:1660` to `pcbnew/router/pns_kicad_iface.cpp:1668` exists because `PNS_LAYER_RANGE(1, 0)` would be normalised to `(0, 1)` and index the pad on both outer layers. A Rust port should make the range type refuse to construct an inverted range rather than reordering.

`SetLayersFromPCBNew( start, end )` (`pcbnew/router/pns_kicad_iface.cpp:3163`) is just the pairwise application of the mapping and is a convenience, not part of the interface.

### 1.8 Net handles

`NET_HANDLE` is `void*`. KiCad passes `NETINFO_ITEM*` through it. The router never dereferences it; it compares handles for equality (`aA->Net() == aB->Net()` in `Clearance`, `pcbnew/router/pns_kicad_iface.cpp:903`), stores them on items, and hands them back to the host.

`GetNetCode` exists because a few places need an ordered or "is this a real net" test rather than pointer equality. It is used in `pickSingleItem` to decide whether the net filter applies at all (`pcbnew/router/pns_tool_base.cpp:167`: `if( m_router->GetInterface()->GetNetCode( aNet ) <= 0 || item->Net() == aNet )`), in `highlightNets` (`pcbnew/router/pns_tool_base.cpp:259`), and in `LINE_PLACER` to decide whether a start or end item with a non-net counts (`pcbnew/router/pns_line_placer.cpp:1571` and `:1576`). The convention that `<= 0` means "not a real net" is baked into these call sites.

`GetNetName` is used only for debug output and logging: `pcbnew/router/pns_logger.cpp:184`, `pcbnew/router/pns_item.cpp:347`, and several `wxLogTrace` sites in `pns_shove.cpp` and `pns_multi_dragger.cpp`. A port can return an empty string and lose nothing but log readability.

`UpdateNet` is called once per modified net at the end of `StopRouting` (`pcbnew/router/pns_router.cpp:978`), and KiCad's implementation is a single `wxLogTrace` (`pcbnew/router/pns_kicad_iface.cpp:3014`). The intent, per the comment at `pcbnew/router/pns_router.cpp:969`, is "update the ratsnest with new changes"; in practice KiCad gets that from the board commit instead. Treat it as an advisory invalidation hook.

`GetOrphanedNetHandle` has exactly one caller, `pcbnew/router/pns_line_placer.cpp:1386`:

```cpp
m_currentNet = aStartItem ? aStartItem->Net() : Router()->GetInterface()->GetOrphanedNetHandle();
```

That is, it supplies the net for a track started in empty space, on no existing item. KiCad returns the singleton `NETINFO_LIST::OrphanedItem()` (`pcbnew/router/pns_kicad_iface.cpp:3020`), and `createBoardItem` re-uses the same object as the fallback net and then assigns the default netclass to it if its netcode is non-positive (`pcbnew/router/pns_kicad_iface.cpp:2862` to `pcbnew/router/pns_kicad_iface.cpp:2868`). It must be a stable, non-null handle whose `GetNetCode` is `<= 0`, because the net-filter logic in `pickSingleItem` and `LINE_PLACER` keys off exactly that. Returning null instead would work for equality comparisons but would break the `GetNetCode( aNet ) <= 0` guard, which dereferences nothing but distinguishes "no net" from "net zero" purely by the code.

### 1.9 ImportSizes

`ImportSizes` is the host's chance to fill a `PNS::SIZES_SETTINGS` from board design rules. It is not called by the router core at all; both call sites are in the GUI tool, `pcbnew/router/router_tool.cpp:1710` and `pcbnew/router/router_tool.cpp:3349`. So it is part of the *tool* contract, not the *engine* contract, and a Rust port could reasonably hoist it out of the engine host trait entirely.

The KiCad implementation (`pcbnew/router/pns_kicad_iface.cpp:1102` to `pcbnew/router/pns_kicad_iface.cpp:1330`) is a good specification of what the router actually needs, so it is worth enumerating what it sets:

- `Clearance` and `MinClearance`, both seeded from the board minimum, with `Clearance` raised by a `CT_CLEARANCE` query against a dummy segment at the start anchor if that query yields something at least as large (`:1111` to `:1143`). `MinClearance` is used to reject a diff pair whose gap is below it (`pcbnew/router/pns_router.cpp:229`), and `Clearance` feeds the hover slop radius (`pcbnew/router/pns_helpers.cpp:32`).
- `TrackWidth`, `BoardMinTrackWidth`, `TrackWidthIsExplicit`. The width resolution order is: inherit from the connected track if the board setting says so (`:1150`), else netclass via a `CT_WIDTH` query (`:1158`), else the current user width (`:1176`). `TrackWidthIsExplicit` is set to `!m_UseConnectedTrackWidth || m_TempOverrideTrackWidth` (`:1188`) and is consumed by the placers to decide whether they may change the width mid-route (`pcbnew/router/pns_line_placer.cpp:2004`, `pcbnew/router/pns_diff_pair_placer.cpp:793`).
- `ViaDiameter` and `ViaDrill`, from netclass `CT_VIA_DIAMETER` and `CT_VIA_HOLE` queries or from the current user values (`:1190` to `:1222`).
- The whole diff-pair block: width, gap, via gap, and `SetDiffPairViaGapSameAsTraceGap( false )` (`:1224` to `:1285`).
- `HoleToHole`, `DiffPairHoleToHole`, `DiffPairCopperToHole` (`:1287` to `:1327`).

The inheritance helper `inheritTrackWidth` (`pcbnew/router/pns_kicad_iface.cpp:986`) has a detail worth copying: when a start position is supplied, it picks the connected track whose *far* end is nearest the cursor, on the grounds that the far-end direction indicates which stub the user is pointing at (`:1029` to `:1071`), and only falls back to "minimum width of all connected tracks on the current layer, else on any layer" (`:1073` to `:1096`).

### 1.10 The length and delay hooks

Six methods, all used only by the tuning placers. Call sites:

- `CalculateRoutedPathLength` and `CalculateRoutedPathDelay`: `pcbnew/router/pns_meander_placer_base.cpp:313` and `:323`.
- `CalculateLengthForDelay`: `pcbnew/router/pns_meander_placer.cpp:169`, `:172`, `:175`; `pcbnew/router/pns_dp_meander_placer.cpp:733`, `:736`, `:739`; `pcbnew/router/pns_meander_skew_placer.cpp:259`.
- `CalculateDelayForShapeLineChain`: `pcbnew/router/pns_meander_placer.cpp:293`, `:327`; `pcbnew/router/pns_dp_meander_placer.cpp:489`, `:492`, `:520`, `:523`.
- `GetSignalAggregate`: `pcbnew/router/pns_meander_placer_base.cpp:68`, `pcbnew/router/pns_meander_skew_placer.cpp:146`.
- `GetNetBoardLength`: `pcbnew/router/pns_meander_placer_base.cpp:88`.

None of the six is reachable from single-track routing or from dragging. A first integration can return zero from all six and lose only the tuning modes. `GetNetBoardLength` already has a default `{ return 0; }` in the header (`pcbnew/router/pns_router.h:138`), which is the maintainers' own acknowledgement that these are optional.

`GetSignalAggregate` deserves a note because its semantics are non-obvious: given the two nets of a pair, it finds every *other* net in the same "net chain" (KiCad's name for a signal that passes through series components) and sums their already-routed length and delay, so the tuner can budget the whole chain rather than just the segment being routed (`pcbnew/router/pns_kicad_iface.cpp:3068` to `pcbnew/router/pns_kicad_iface.cpp:3124`). It returns `true` for a valid chain even when the sum is zero (`:3121` comment). LibrePCB has no equivalent of net chains today, so `false` is the honest answer.

There is a live bug marker in this area: `GetLengthDelayCalculationItems` at `pcbnew/router/pns_kicad_iface.cpp:3283` carries the comment `// TODO: BUG IS HERE!!!` on the line that sets a via's layer span from the previous and next line layers. Worth not replicating.

### 1.11 Two dead virtuals: StackupHeight and GetWorld

`StackupHeight( int, int )` is declared pure virtual at `pcbnew/router/pns_router.h:113` and implemented at `pcbnew/router/pns_kicad_iface.cpp:1333`, but a tree-wide grep finds no caller. The only other hits are the declaration in the header, an unrelated same-named method on `LENGTH_CALCULATION` used by a DRC test (`qa/tests/pcbnew/drc/test_drc_tuner_agreement.cpp:178`), and a stub returning 0 in the log viewer's mock interface (`qa/tools/pns/pns_log_viewer_frame.h:74`). Via height used to be folded into length calculations here and now goes through `LENGTH_CALCULATION` instead. A Rust port should not have this method at all.

`ROUTER_IFACE::GetWorld()` (`pcbnew/router/pns_router.h:119`) is dead in the same way. Every `GetWorld()` call site in the tree (`pcbnew/router/pns_line_placer.cpp:1461`, `pcbnew/router/pns_meander_placer.cpp:81`, `pcbnew/router/pns_dp_meander_placer.cpp:101`, `pcbnew/router/pns_diff_pair_placer.cpp:626` and `:663`) resolves to `ROUTER::GetWorld()`, not to the interface's. The interface version exists only because `PNS_KICAD_IFACE_BASE` caches the node pointer it was handed in `SyncWorld` (`pcbnew/router/pns_kicad_iface.cpp:2302`) and exposes it for the host's own use.

### 1.12 Required subset

For basic single-line routing with shove and walkaround, on a host with simple netclass clearances, the genuinely required methods are:

| Method | Why it is required |
| --- | --- |
| `SyncWorld` | The only way items enter the world |
| `GetRuleResolver` | Every collision query goes through it |
| `GetDebugDecorator` | May return null (the `PNS_DBG` macro null-checks it, `pcbnew/router/pns_debug_decorator.h:126`), but it is reached through the global router singleton with no null check at `pcbnew/router/pns_via.cpp:150` and `pcbnew/router/pns_mouse_trail_tracer.cpp:78`, so the singleton and its interface must be live |
| `IsFlashedOnLayer` (both overloads) | Called in the collision inner loop, `pcbnew/router/pns_item.cpp:206` |
| `GetBoardLayerFromPNSLayer` / `GetPNSLayerFromBoardLayer` | Needed by `QueryConstraint` and by `GetViaLayerRange` |
| `GetOrphanedNetHandle` | Required to start a track in empty space, `pcbnew/router/pns_line_placer.cpp:1386` |
| `GetNetCode` | The `<= 0` convention gates net filtering in the tool and the placer |
| `AddItem` / `UpdateItem` / `RemoveItem` / `Commit` | Write-back; can be no-ops for a read-only or test host |
| `DisplayItem` / `HideItem` / `EraseView` | Can be no-ops headless, but a GUI needs them |
| `GetWorld` | Trivial accessor, and unused by the engine (section 1.11) |

Needed only for tuning, diff pairs or HDI:

| Method | Needed for |
| --- | --- |
| `DisplayPathLine` | Tuning preview only |
| `DisplayRatline` | Single-track and diff-pair guide line, cosmetic |
| The four `Calculate*` methods, `GetSignalAggregate`, `GetNetBoardLength` | Tuning only |
| `ImportSizes` | Tool-level, not engine-level |
| `IsAnyLayerVisible` / `IsItemVisible` / `IsPNSCopperLayer` | Tool-level hover filtering only |
| `UpdateNet` | Advisory |
| `GetNetName` | Debug output only |
| `StackupHeight` | Nothing; dead |

## 2. RULE_RESOLVER

### 2.1 The contract

Declared at `pcbnew/router/pns_node.h:139` through `pcbnew/router/pns_node.h:183`. Fifteen virtuals, of which four have defaults.

```cpp
virtual int Clearance( const ITEM* aA, const ITEM* aB, bool aUseClearanceEpsilon = true ) = 0;   // :144
virtual bool HasUserDefinedPhysicalConstraint() { return false; }                                 // :145
virtual NET_HANDLE DpCoupledNet( NET_HANDLE ) = 0;                                                // :147
virtual int DpNetPolarity( NET_HANDLE ) = 0;                                                      // :148
virtual bool DpNetPair( const ITEM*, NET_HANDLE& aNetP, NET_HANDLE& aNetN ) = 0;                  // :149
virtual int NetCode( NET_HANDLE ) = 0;                                                            // :151
virtual wxString NetName( NET_HANDLE ) = 0;                                                       // :152
virtual bool IsInNetTie( const ITEM* ) = 0;                                                       // :154
virtual bool IsNetTieExclusion( const ITEM*, const VECTOR2I& aCollisionPos, const ITEM* ) = 0;     // :155
virtual bool IsDrilledHole( const PNS::ITEM* ) = 0;                                               // :158
virtual bool IsNonPlatedSlot( const PNS::ITEM* ) = 0;                                             // :159
virtual bool IsKeepout( const ITEM* aObstacle, const ITEM* aItem, bool* aEnforce ) = 0;           // :165
virtual bool QueryConstraint( CONSTRAINT_TYPE, const ITEM* aItemA, const ITEM* aItemB,
                              int aLayer, CONSTRAINT* aConstraint ) = 0;                          // :167
virtual void ClearCacheForItems( std::vector<const ITEM*>& ) {}                                   // :170
virtual void ClearCaches() {}                                                                     // :171
virtual void ClearTemporaryCaches() {}                                                            // :172
virtual int ClearanceEpsilon() const { return 0; }                                                // :174
virtual const SHAPE_LINE_CHAIN& HullCache( const ITEM*, int aClearance,
                                           int aWalkaroundThickness, int aLayer );                // :176
```

The `HullCache` default at `pcbnew/router/pns_node.h:176` to `pcbnew/router/pns_node.h:182` is worth reading because it is a trap: it computes into a function-local `static SHAPE_LINE_CHAIN empty` and returns a reference to it. That is neither reentrant nor thread safe, and it is only correct because callers consume the result immediately. `pcbnew/router/pns_node.cpp:346` explicitly notes "GetClearance() and HullCache() are not thread-safe" and serialises the first phase of a parallel obstacle scan because of it. A Rust port should return an owned value or a handle into an arena and let the borrow checker enforce lifetime, rather than replicating this.

### 2.2 Clearance and the two caches

`PNS_PCBNEW_RULE_RESOLVER::Clearance` is at `pcbnew/router/pns_kicad_iface.cpp:865` to `pcbnew/router/pns_kicad_iface.cpp:983`. Structure:

```
bothOwned = aA && aB && aA->Owner() && aB->Owner()
if bothOwned:      look up m_clearanceCache      keyed by (ptr A, ptr B, epsilonFlag), canonically ordered
else if aA && aB:  look up m_tempClearanceCache  keyed by (properties of A, properties of B, epsilonFlag)
compute layers:
    aB == null            -> aA->Layers()
    isEdge(aA)            -> aB->Layers()
    isEdge(aB)            -> aA->Layers()
    otherwise             -> intersection
clamp layers to [PCBNEW_LAYER_ID_START, PCB_LAYER_ID_COUNT-1]
rv = 0
for layer in layers:
    if both drilled holes:   rv = max(rv, CT_HOLE_TO_HOLE)
    elif either is a hole and nets differ: rv = max(rv, CT_HOLE_CLEARANCE)
    if isCopper(aA) and (aB==null or isCopper(aB)) and !sameNet and !freePad:
                             rv = max(rv, CT_CLEARANCE)
    if either is an edge:    rv = max(rv, CT_EDGE_CLEARANCE)
    if either is a hole:     rv = max(rv, CT_PHYSICAL_HOLE_CLEARANCE)
    always:                  rv = max(rv, CT_PHYSICAL_CLEARANCE)
if (sameNet || freePad) and rv == 0:   rv = -1
if aUseClearanceEpsilon and rv > 0:    rv = max(0, rv - m_clearanceEpsilon)
store in the appropriate cache
```

Several details are essential in the sense that a naive reimplementation gets them wrong.

**The two-cache split.** `m_clearanceCache` (`pcbnew/router/pns_kicad_iface.cpp:314`) is keyed by raw item pointers, canonically ordered so `(A,B)` and `(B,A)` hash the same (`CLEARANCE_CACHE_KEY` at `pcbnew/router/pns_kicad_iface.cpp:92` to `:109`). It is only usable for items the world owns, because only those have stable addresses. Router-internal temporaries get `m_tempClearanceCache` (`:315`), keyed instead by *properties*: board item pointer, net handle, layer start, layer end, kind, free-pad flag (`TEMP_CLEARANCE_CACHE_KEY::SIDE` at `:132` to `:161`). The comment at `:127` to `:129` spells out the rationale: items with the same properties get the same clearance, so they share one entry. This is what makes the shove loop, which creates thousands of transient `LINE` objects, tractable. The temporary cache is cleared separately, `ClearTemporaryCaches()` at `:826`, which the router calls from `ROUTER::Move` at `pcbnew/router/pns_router.cpp:512`.

**Note the layer flag is a `bool` and not the layer.** Despite the task framing, the cache key is *not* keyed by layer: it is keyed by `(A, B, aUseClearanceEpsilon)`. The layer range is derived from the two items inside the computation, and the loop takes the maximum across the whole overlapping range. So the cached value is "worst-case clearance across all layers on which these two items coexist", not a per-layer value. Any port that caches per-layer will get a different, and finer-grained, answer than KiCad; that is arguably better but it is a behavioural difference.

**Edge items borrow the other item's layers.** `isEdge` at `pcbnew/router/pns_kicad_iface.cpp:453` returns true for a `PCB_SHAPE` on `Edge_Cuts` or `Margin`. Because such a shape is synced onto *all* copper layers (section 3.6), intersecting layer ranges would be a no-op; the code instead uses the non-edge item's range so the DRC query gets a meaningful layer. `pcbnew/router/pns_router.cpp:242` makes the same point from the other side: `isStartingPointRoutable` skips edge-cuts items when checking routability, with the comment "Edge cuts are put on all layers, but they're not *really* on all layers".

**`-1` means "no clearance applies".** The same-net and free-pad short circuit at `pcbnew/router/pns_kicad_iface.cpp:968` only fires when nothing raised `rv` above zero, so a physical clearance rule can still bind same-net items. `NODE::GetClearance` (`pcbnew/router/pns_node.cpp:143`) additionally returns `0` for virtual items and `100000` when there is no resolver at all, the latter being a defensive default rather than a meaningful number.

**The clearance epsilon.** `m_clearanceEpsilon` is initialised from `aBoard->GetDesignSettings().GetDRCEpsilon()` (`pcbnew/router/pns_kicad_iface.cpp:338`) and subtracted from any positive clearance when `aUseClearanceEpsilon` is set. Its purpose is to stop the router from reporting a violation on geometry that DRC considers exactly at the limit, given integer rounding. It is added back into the world's max clearance (`pcbnew/router/pns_kicad_iface.cpp:2452`) so the broad phase is not narrowed by it. The related "extra 1" in `collideSimple`, `shapeH->Collide( shapeI, clearance + lineWidthH + lineWidthI - 1 )` at `pcbnew/router/pns_item.cpp:249` and `:280`, exists for the same reason: hulls are built to exactly the clearance distance, so touching at exactly the clearance must not count as a collision.

**Line widths are folded into the clearance.** `pcbnew/router/pns_item.cpp:159` to `:165` comments that "collision routines ignore SHAPE_POLY_LINE widths so we have to pass them in as part of the clearance value", then adds half of each `LINE`'s width. A Rust port with width-aware collision can drop this, but must then not double count.

### 2.3 IsKeepout

`pcbnew/router/pns_kicad_iface.cpp:388` to `pcbnew/router/pns_kicad_iface.cpp:430`. The two-argument plus out-parameter shape is unusual and matters: the return value is "is `aObstacle` a keepout at all", while `*aEnforce` is "does that keepout's rule set actually exclude `aItem`". The caller at `pcbnew/router/pns_item.cpp:198` to `:205` uses them as:

```cpp
else if( aNode->GetRuleResolver()->IsKeepout( this, aHead, &enforce )
         || aNode->GetRuleResolver()->IsKeepout( aHead, this, &enforce ) )
{
    if( enforce ) clearance = 0;   // keepouts are exact boundary; no clearance
    else          clearance = -1;
}
```

So a keepout is an exact-boundary obstacle with zero clearance, and a non-applicable keepout is not an obstacle at all. Note the short-circuit `||`: if the first call returns true, `enforce` from the second call is never computed, which is correct only because a keepout item is never also a routed item.

The inner `checkKeepout` lambda maps the zone's four flags onto the other item's type (`:397` to `:411`): tracks and arcs against `GetDoNotAllowTracks`, vias against `GetDoNotAllowVias`, pads against `GetDoNotAllowPads`, and pads against `GetDoNotAllowFootprints` with an admitted "Incomplete test, but better than nothing" caveat at `:406`. The other item is materialised as a dummy board item via `getBoardItem` so the type test has something to look at.

### 2.4 IsInNetTie and IsNetTieExclusion

`IsInNetTie` (`pcbnew/router/pns_kicad_iface.cpp:349`) is a cheap test: does the item's board item belong to a footprint flagged as a net tie. It is used at `pcbnew/router/pns_item.cpp:232` to decide whether to take the "slow" collision path that computes the collision *position*, because net-tie exclusion is position dependent.

`IsNetTieExclusion` (`pcbnew/router/pns_kicad_iface.cpp:357`) answers "should this particular collision at this particular point be forgiven". Two cases: both items belong to the same net-tie footprint (`:371` to `:375`), or the DRC engine's own net-tie exclusion says so (`:377` to `:382`). LibrePCB has no net-tie concept, so both can return `false` and the router will take the fast collision path everywhere, which is also faster.

### 2.5 IsDrilledHole and IsNonPlatedSlot

Both at `pcbnew/router/pns_kicad_iface.cpp:464` and `:478`. Both start by requiring the item be of kind `HOLE_T` (`isHole` at `:444`), then resolve the parent through `aItem->ParentPadVia()` when the hole has no direct parent (`:471`, `:485`), which is the case for holes attached to router-created vias.

`IsDrilledHole` selects `CT_HOLE_TO_HOLE` versus `CT_HOLE_CLEARANCE` in `Clearance` (`:908`). `IsNonPlatedSlot` is used at `pcbnew/router/pns_item.cpp:230` to enable the castellation check path, and is defined narrowly: NPTH attribute *and* a non-round drill (`:494` to `:495`), with the explicit note at `:498` that "Via holes are (currently) always round, and always plated".

### 2.6 QueryConstraint

`pcbnew/router/pns_kicad_iface.cpp:537` to `pcbnew/router/pns_kicad_iface.cpp:789`. This is the single point where PNS talks to KiCad's DRC rule engine, and it is by far the most host-specific method in the whole contract.

The `CONSTRAINT_TYPE` enum (`pcbnew/router/pns_node.h:51` to `:66`) has thirteen members: `CT_CLEARANCE=1`, `CT_DIFF_PAIR_GAP=2`, `CT_LENGTH=3`, `CT_WIDTH=4`, `CT_VIA_DIAMETER=5`, `CT_VIA_HOLE=6`, `CT_HOLE_CLEARANCE=7`, `CT_EDGE_CLEARANCE=8`, `CT_HOLE_TO_HOLE=9`, `CT_DIFF_PAIR_SKEW=10`, `CT_MAX_UNCOUPLED=11`, `CT_PHYSICAL_CLEARANCE=12`, `CT_PHYSICAL_HOLE_CLEARANCE=13`. The switch at `:548` to `:564` maps each onto a `DRC_CONSTRAINT_T`.

The `CONSTRAINT` struct (`pcbnew/router/pns_node.h:73` to `:82`) carries a `MINOPTMAX<int> m_Value`, a rule name, from and to names, an `m_Allowed` flag and `m_IsTimeDomain`. `Clearance` only ever reads `m_Value.Min()`; `ImportSizes` reads `Min()`, `Opt()` and `PinnedOpt()` depending on the constraint.

Because items handed to `QueryConstraint` frequently have no board item (they are router temporaries), the resolver keeps six preallocated dummy board items, two each of track, arc and via (`pcbnew/router/pns_kicad_iface.cpp:305` to `:307`), all flagged `ROUTER_TRANSIENT` in the constructor (`:328` to `:335`). `getBoardItem` (`:505`) stamps the PNS item's layer, net and anchors onto the right dummy and returns it. Two of each kind is exactly enough for a pairwise query.

The bulk of the function, `:591` to `:738`, is a performance optimisation for custom DRC rules whose conditions depend on geometry, such as `intersectsCourtyard`. A multi-segment `LINE` with no board item is evaluated segment by segment against the other item, taking the *smallest* (most permissive) constraint and short-circuiting as soon as a zero or negative one is found (`pickSmallerConstraint` at `:573` to `:589`). When both sides are multi-segment lines, it is a double loop over segment pairs with a bounding-box proximity filter of `2 * max(widthA, widthB)` (`:676`, `:713`). All of this is gated on `drcEngine->HasGeometryDependentRules()` (`:606`), so a host without a rule language never enters it.

Two behaviours at the tail are worth copying. A constraint whose severity is `RPT_SEVERITY_IGNORE` is reported back as `m_Value.SetMin( -1 )` rather than as "no constraint" (`:755` to `:763`), except for implicit tuning-profile rules. And the value-passing switch at `:765` to `:788` is a whitelist: any constraint type not listed returns `false`, which is how unimplemented types degrade safely.

### 2.7 DpCoupledNet, DpNetPolarity, DpNetPair

All three are pure net-name string matching in KiCad. `DpCoupledNet` (`:1345`) delegates to `BOARD::DpCoupledNet`. `DpNetPolarity` (`:1363`) returns `BOARD::MatchDpSuffix`, whose sign encodes which half of the pair the net is. `DpNetPair` (`:1376`) matches the suffix, orders the two into P and N according to the sign, looks both up by name, and fails if either is missing.

For LibrePCB these can all be stubbed: `DpCoupledNet` returns null, `DpNetPolarity` returns 0, `DpNetPair` returns `false`. Diff-pair placement then fails cleanly at `pcbnew/router/pns_router.cpp:352` with "Cannot start a differential pair" rather than misbehaving.

### 2.8 Cache invalidation

`ClearCaches()` (`:817`) wipes all three caches plus the memoised `m_hasUserPhysicalConstraint`. It is called at the top of `StartRouting` (`pcbnew/router/pns_router.cpp:436`) and `StartDragging` (`pcbnew/router/pns_router.cpp:174`).

`ClearCacheForItems( items )` (`:792`) evicts every clearance entry either of whose sides is in the dirty set, and every hull entry for a dirty item. It is called from `updateView` on the items the current node added (`pcbnew/router/pns_router.cpp:765` to `:766`). This is the mechanism that stops a stale clearance from a deleted item being reused for a freshly allocated item at the same address, which is a real hazard given the pointer-keyed cache. A Rust port using generational indices or slotmap keys sidesteps the class of bug entirely.

`ClearTemporaryCaches()` (`:826`) drops only `m_tempClearanceCache`, from `ROUTER::Move` (`pcbnew/router/pns_router.cpp:512`).

### 2.9 HasUserDefinedPhysicalConstraint

A memoised boolean (`:851`). The comment at `:310` to `:311` explains why it is cached: it "runs in the collideSimple inner loop and walks the DRC engine map otherwise". Its effect at `pcbnew/router/pns_item.cpp:127` is to set `runPhysicalOnly`, which forces hole-versus-hole and same-net collision paths to still consult the resolver, because physical clearance rules are net-blind. A host with no physical clearance concept returns `false` (the base-class default) and gets the fast path everywhere.

### 2.10 HullCache

`:832` to `:848`. Keyed by `(item pointer, clearance, walkaroundThickness, layer)` (`HULL_CACHE_KEY` at `:222` to `:236`). Callers are the line placer (`pcbnew/router/pns_line_placer.cpp:821`), the shove engine (`pcbnew/router/pns_shove.cpp:1282`), the walkaround (`pcbnew/router/pns_walkaround.cpp:157`), and the parallel obstacle scan (`pcbnew/router/pns_node.cpp:364`, `:373`). Hull generation is the dominant cost in a shove iteration, so this cache is a performance requirement, not an optimisation to defer.

### 2.11 Minimal subset for a simple-netclass host

LibrePCB's rule model, read from source, is: a per-net-class `minCopperCopperClearance`, `minCopperWidth` and `minViaDrillDiameter` (`libs/librepcb/core/project/circuit/netclass.h:69` to `:75`), each combined with the board DRC setting by `std::max` (`libs/librepcb/core/project/board/drc/boarddesignrulecheckdata.h` helper methods, for example `getMinCopperCopperClearance` which takes `std::max(settings.getMinCopperCopperClearance(), it->minCopperCopperClearance)`); board-level `minCopperBoardClearance`, `minCopperNpthClearance`, `minDrillDrillClearance`, `minDrillBoardClearance` (`libs/librepcb/core/project/board/drc/boarddesignrulechecksettings.h:112` to `:128`); board-level `defaultTraceWidth` and `defaultViaDrillDiameter` (`libs/librepcb/core/project/board/boarddesignrules.h:57` and `:60`); and a per-pad `copperClearance` override (`libs/librepcb/core/geometry/pad.h:112`).

That maps onto the resolver as:

| Resolver method | LibrePCB implementation |
| --- | --- |
| `Clearance(A, B, eps)` | `max` over: copper-copper if both copper and different nets, using `max(board, netclassA, netclassB, padOverrideA, padOverrideB)`; copper-to-board-edge if either is an outline item; copper-to-NPTH if either is a non-plated hole; drill-to-drill if both are holes; drill-to-board for a hole against the outline. Return `-1` when nothing applies. |
| `ClearanceEpsilon()` | `0` unless a rounding epsilon is introduced |
| `QueryConstraint` | Implement `CT_CLEARANCE`, `CT_WIDTH`, `CT_VIA_DIAMETER`, `CT_VIA_HOLE`, `CT_HOLE_CLEARANCE`, `CT_EDGE_CLEARANCE`, `CT_HOLE_TO_HOLE`. Return `false` for everything else. |
| `IsKeepout` | Zones with a "no copper" rule; `*aEnforce = true`. Return `false` if LibrePCB zones are not modelled as keepouts in v1. |
| `IsDrilledHole` | Item is a hole and its parent is a plated via or PTH pad |
| `IsNonPlatedSlot` | Item is a hole, parent is an NPTH pad, and the drill is non-round |
| `IsInNetTie`, `IsNetTieExclusion` | `false` |
| `HasUserDefinedPhysicalConstraint` | `false` |
| `DpCoupledNet`, `DpNetPolarity`, `DpNetPair` | null, 0, `false` |
| `NetCode`, `NetName` | Stable integer per net signal, and the net name |
| `HullCache`, `ClearCaches`, `ClearCacheForItems`, `ClearTemporaryCaches` | Implement the caches; they are performance-critical |

## 3. SyncWorld, object by object

`PNS_KICAD_IFACE_BASE::SyncWorld` is at `pcbnew/router/pns_kicad_iface.cpp:2292` to `pcbnew/router/pns_kicad_iface.cpp:2453`. Order of traversal: board drawings, then the board outline polygon, then zones, then footprints (pads, reference, value, footprint zones, fields, graphical items), then tracks, arcs and vias. Order matters for one reason only: `worstClearance` is accumulated during the pad loop and consumed at the very end.

### 3.1 Tracks become SEGMENT

`syncTrack` at `pcbnew/router/pns_kicad_iface.cpp:1749`. A `PNS::SEGMENT` from `SEG(start, end)` plus net, width, a single PNS layer, and the parent pointer. Two independent lock sources set `PNS::MK_LOCKED`: the board item's own lock flag (`:1757`), and membership in a `PCB_GENERATOR` group that is not currently being edited (`:1760` to `:1764`). The second is how teardrops and tuning patterns become immovable while routing near them.

Tracks are added with `aWorld->Add( std::move( segment ), true )` (`pcbnew/router/pns_kicad_iface.cpp:2432`); the trailing `true` is the "allow duplicate" flag, which matters for boards containing coincident tracks.

### 3.2 Arcs become ARC

`syncArc` at `:1770`. Built from `SHAPE_ARC(start, mid, end, width)`, so the host must supply a three-point arc representation, not a centre-and-angle one. Same two lock sources. Also added with the duplicate flag (`:2437`).

### 3.3 Vias become VIA plus a separate HOLE

`syncVia` at `:1792` to `:1891`. The essentials:

- Layer span from `SetLayersFromPCBNew( aVia->TopLayer(), aVia->BottomLayer() )` (`:1814`).
- Diameter is per PNS layer, not scalar. `PNS::VIA::STACK_MODE` is `NORMAL`, `FRONT_INNER_BACK` or `CUSTOM` (`:1827` to `:1849`). The long comment at `:1797` to `:1811` explains that `FRONT_INNER_BACK` cannot be used for blind or buried vias because `PNS::VIA` has no idea how many layers the board has and therefore cannot tell its own bottom layer from the board's; such vias are forced to `NORMAL` with the inner-layer diameter.
- `SetUnconnectedLayerMode` (`:1819`) carries the "remove unused pads" policy, which is what `IsFlashedOnLayer` then consults.
- The hole is a *separate* `PNS::HOLE` object attached with `SetHole` (`:1863`), built by `PNS::HOLE::MakeCircularHole( position, drill/2, layerRange )`. Its layer range is set independently via `SetHoleLayers` from the via's primary drill layers if defined, else from the copper span (`:1867` to `:1873`). Secondary drill size and layers are also carried (`:1876` to `:1888`), for back-drilled vias.

The separation of hole from copper is structural. `ITEM::collideSimple` treats a pad's or via's hole as its own indexed item (`pcbnew/router/pns_item.cpp:107` to `:109`) and recurses into hole-versus-item and hole-versus-hole collisions explicitly (`:146` to `:157`). A port that models the hole as an attribute of the via rather than as a first-class item will have to reinvent all of that.

### 3.4 Pads become one or more SOLID

`syncPad` at `:1619` to `:1746`, returning a vector.

Non-copper pads with no drill are skipped outright (`:1626`). `PAD_ATTRIB::PTH` and `NPTH` keep the full copper-layer range `PNS_LAYER_RANGE( 0, BoardCopperLayerCount() - 1 )` set at `:1622`. `CONN` and `SMD` are narrowed to the single front layer of their layer stack and are skipped entirely if that leaves them with no copper (`:1635` to `:1650`). Anything else logs "unsupported pad type" and is skipped (`:1652` to `:1654`).

The per-layer loop is driven by `aPad->Padstack().ForEachUniqueLayer( makeSolidFromPadLayer )` (`:1743`), so a uniform pad yields one SOLID and a custom padstack yields one per distinct layer. Inside:

- NPTH pads get `SetRoutable( false )` (`:1672` to `:1673`), which is what makes `isStartingPointRoutable` report "Cannot start routing from a non-plated hole" (`pcbnew/router/pns_router.cpp:263` to `:264`).
- Layer assignment differs by padstack mode: `CUSTOM` gets the single layer, `FRONT_INNER_BACK` gets the single layer for F and B and the inner range `PNS_LAYER_RANGE( 1, count - 2 )` otherwise, and `NORMAL` gets the full range (`:1675` to `:1689`).
- Pad-to-die length and delay are carried onto the solid (`:1693` to `:1694`), for length tuning.
- Position and offset are stored separately, with the offset rotated by the pad orientation (`:1700` to `:1708`).
- A drill produces a `PNS::HOLE` from `GetEffectiveHoleShape()->Clone()`, spanning the full copper range regardless of the pad's own copper span (`:1710` to `:1714`).
- Free pads (unconnected pins) get `SetIsFreePad()` (`:1697`), which suppresses clearance against them entirely in `Clearance` (`:904`, `:968`) and in `collideSimple` (`pcbnew/router/pns_item.cpp:193`).

The "aperture" and flashing logic is the comment at `:1716` to `:1717`:

```cpp
// We generate a single SOLID for a pad, so we have to treat it as ALWAYS_FLASHED and
// then perform layer-specific flashing tests internally.
const std::shared_ptr<SHAPE>& shape = aPad->GetEffectiveShape( aLayer, FLASHING::ALWAYS_FLASHED );
```

That is, the geometry is always the flashed geometry, and whether it counts on a given layer is decided later by `ROUTER_IFACE::IsFlashedOnLayer`. This is the design decision that makes `IsFlashedOnLayer` mandatory rather than optional.

The shape itself is a single primitive when the pad's effective shape has exactly one indexable subshape, and otherwise a `SHAPE_SIMPLE` polygon from `GetEffectivePolygon( aLayer, ERROR_OUTSIDE )` (`:1720` to `:1735`), with a comment citing KiCad issue 15553: "Multiple shapes have a tendency to confuse the hull generator". A solid with no shape is dropped (`:1737` to `:1738`).

Thermal reliefs are not synced. The router sees the pad's copper and the zone's keepout status, nothing about the connection pattern.

Castellated pads get one extra treatment in the caller: `aWorld->AddEdgeExclusion( hole )` (`pcbnew/router/pns_kicad_iface.cpp:2369` to `:2374`). That registers a region where an edge-cuts collision is forgiven, consumed at `pcbnew/router/pns_item.cpp:251` via `aNode->QueryEdgeExclusions( pos )`.

### 3.5 Zones: only rule areas, never filled copper

This is the answer that most surprises people coming to PNS: **filled zones are not obstacles and are not synced at all.**

`syncZone` at `pcbnew/router/pns_kicad_iface.cpp:1894` opens with:

```cpp
if( !aZone->GetIsRuleArea() || !aZone->HasKeepoutParametersSet() )
    return false;
```

So a normal filled copper pour is invisible to the router. Only rule areas that actually set keepout parameters are synced. The `aBoardOutline` parameter is accepted and never used in the current implementation.

When a rule area *is* synced, it becomes one `PNS::SOLID` per triangle of the zone outline's triangulation, on every copper layer in the zone's layer set (`:1921` to `:1952`). Each triangle solid gets a null net, the zone as parent, `SetIsCompoundShapePrimitive()` and `SetRoutable( false )`. Triangulation is forced with `poly->CacheTriangulation()` (`:1905`) and a self-intersecting polygon that fails to triangulate raises a modal dialog and is skipped (`:1907` to `:1919`), which is a host-policy decision a headless port must replace.

`SetIsCompoundShapePrimitive` matters downstream: `markViolations` refuses to hide the original when a compound primitive is highlighted, "We're only highlighting one (or more) of several primitives so we don't want all the other parts of the object to disappear" (`pcbnew/router/pns_router.cpp:688` to `:693`).

The practical consequence for a shove router is that the router will happily route a trace through a filled pour, and the host is expected to re-fill the pour afterwards. LibrePCB's plane fragments builder occupies the same role and should be treated the same way: do not sync plane fragments as obstacles in v1.

### 3.6 Board outline, graphics and text on copper

`syncGraphicalItem` at `:2047` handles `PCB_SHAPE` and `PCB_TEXTBOX`. It accepts an item if it is on `Edge_Cuts`, on `Margin`, or on a copper layer (`:2049` to `:2051`), and produces one SOLID per effective shape from `MakeEffectiveShapesWithLineEndings`.

For `Edge_Cuts` and `Margin`:

```cpp
solid->SetLayers( PNS_LAYER_RANGE( 0, m_board->GetCopperLayerCount() - 1 ) );
solid->SetRoutable( false );
```

(`:2061` to `:2062`). So yes, the board outline is a SOLID spanning every copper layer, and it is non-routable. Additionally, for `Edge_Cuts` only, the shape's width is forced to zero (`:2070` to `:2079`), so the outline is a zero-width curve and the clearance comes entirely from the `CT_EDGE_CLEARANCE` rule. `Margin` keeps its width. That asymmetry is deliberate and easy to miss.

For copper-layer shapes the solid takes the single layer and is routable unless it is a table cell (`:2066` to `:2067`), carries the shape's net (`:2082`), and gets `SetAnchorPoints( aItem->GetConnectionPoints() )` (`:2081`) so the router can snap to a graphic's endpoints. When one board item yields several shapes, each solid gets `SetIsCompoundShapePrimitive()` (`:2086` to `:2087`).

`syncTextItem` at `:1958` handles `PCB_TEXT`, `PCB_TABLE`, `PCB_FIELD` and footprint reference and value. It refuses non-copper layers (`:1960`) and invisible fields (`:1963`), then converts to a polygon via `TransformShapeToPolygon( ..., ERROR_OUTSIDE )`, simplifies, and takes **only outline 0** (`:1984`). Text with multiple disjoint glyph outlines is therefore under-represented, which is a known simplification. Null net, non-routable.

`syncDimension` at `:1993` does the same but adds *every* outline rather than just the first (`:2001` to `:2016`), and separately converts the dimension's text if it is visible and non-empty (`:2030` to `:2041`). Null net, non-routable.

`syncBarcode` at `:2099` uses `GetBoundingHull` rather than the exact shape, and adds every outline. Null net, non-routable.

### 3.7 What is deliberately not synced

From the `SyncWorld` switch and the sync helpers:

- Filled copper zones, as covered above.
- `PCB_REFERENCE_IMAGE_T`, `PCB_TARGET_T`, `PCB_GRIDITEM_T`, explicitly listed and broken out of the switch with the comment `// ignore` (`pcbnew/router/pns_kicad_iface.cpp:2333` to `:2336`).
- Anything on a non-copper layer other than `Edge_Cuts` and `Margin`. Silkscreen, solder mask, courtyards, fabrication layers are all invisible to the router. Courtyard-based DRC rules are reached only through `QueryConstraint`, which is why the geometry-dependent-rule machinery in `QueryConstraint` exists.
- Thermal relief spokes and any zone-to-pad connection detail.
- Anything not matched by the switch falls into `UNIMPLEMENTED_FOR( gitem->GetClass() )` (`:2339`), which is a runtime complaint rather than a silent skip.

### 3.8 Net handle mapping in the sync

Every sync helper either passes the board object's net straight through (`aTrack->GetNet()` at `:1751`, `aPad->GetNet()` at `:1691`, `aItem->GetNet()` at `:2082`) or sets `nullptr` explicitly for non-conductive obstacles (text at `:1970`, dimensions at `:2007`, barcodes at `:2116`, rule-area triangles at `:1943`). A null net is not the same as the orphan net: null means "this thing has no net and clearance always applies", whereas the orphan handle is a real net object with a non-positive net code, used only for newly started tracks.


## 4. The GUI tool layer: event flow, snapping, undo, and preview

Two classes: `PNS::TOOL_BASE` (`pcbnew/router/pns_tool_base.h:44`), which owns the router, the interface and the grid helper and implements picking and snapping; and `ROUTER_TOOL : public PNS::TOOL_BASE` (`pcbnew/router/router_tool.h`), which implements the event loops.

Three names in common circulation do not exist in this revision and are worth flagging so nobody goes looking for them. `pickSingleItem` has no `aIgnoreNet` and no `aIgnoreLockedItems` parameter; the real signature is `pickSingleItem( const VECTOR2I& aWhere, NET_HANDLE aNet = nullptr, int aLayer = -1, bool aIgnorePads = false, const std::vector<ITEM*> aAvoidItems = {} )` at `pcbnew/router/pns_tool_base.h:61`. There is no `deleteTraces()` anywhere in the tree. There is no singular `highlightNet` on the tool base, only `highlightNets( bool, std::set<NET_HANDLE> )` at `pcbnew/router/pns_tool_base.h:65`. Also `m_startLayer` lives on the interface (`pcbnew/router/pns_kicad_iface.h:144`), not on the tool.

### 4.1 Ownership and lifecycle

`TOOL_BASE` owns three heap objects and the destruction order is commented and essential (`pcbnew/router/pns_tool_base.cpp:66` to `:71`):

```cpp
delete m_gridHelper;
delete m_router;
delete m_iface; // Delete after m_router because PNS::NODE dtor needs m_ruleResolver
```

`Reset( RESET_REASON )` (`pcbnew/router/pns_tool_base.cpp:74` to `:108`) is a full teardown and rebuild: new interface, `SetBoard`, `SetView`, `SetHostTool`, then a new `ROUTER`, `SetInterface`, `ClearWorld()`, `SyncWorld()`, `UpdateSizes( m_savedSizes )`, load `ROUTING_SETTINGS` from app config under the key `"tools.pns"`, and finally a new `PCB_GRID_HELPER` bound to the frame's magnetic settings. `SetHostTool` is what allocates the `BOARD_COMMIT` (`pcbnew/router/pns_kicad_iface.cpp:3035`), and `SetView` is what allocates the preview `VIEW_GROUP` and the debug decorator (`pcbnew/router/pns_kicad_iface.cpp:2967` to `:2993`).

`m_savedSizes` (`pcbnew/router/pns_tool_base.h:71`) persists sizes across router invocations: saved at `pcbnew/router/router_tool.cpp:2509`, restored at `pcbnew/router/pns_tool_base.cpp:98`.

### 4.2 pickSingleItem: the hover priority rules

`pcbnew/router/pns_tool_base.cpp:111` to `:250`. This is the function a reimplementation is most likely to get subtly wrong, because the rules are entirely implicit in the order of a five-element array.

**Top layer.** `int tl = aLayer > 0 ? aLayer : GetPNSLayerFromBoardLayer( getView()->GetTopLayer() )` (`pcbnew/router/pns_tool_base.cpp:114`). The comparison is `> 0`, not `>= 0`, so an explicit request for PNS layer 0 (the front copper layer) silently falls back to the view's top layer. That looks like a latent bug and should not be reproduced literally.

**Slop radius.** `maxSlopRadius = max( gridHelper->GetGrid().x, gridHelper->GetGrid().y )` (`:117`), that is, one grid step, not a fixed constant.

**Two passes.** The loop runs with slop radius 0 and then, only if the first pass found nothing at all, with `maxSlopRadius` (`:141` to `:223`). `QueryHoverItems( aP, 0 )` is an exact geometric hit test (`pcbnew/router/pns_router.cpp:154`, implemented as a zero-radius circle collide at `pcbnew/router/pns_node.cpp:557` to `:568`); `QueryHoverItems( aP, r > 0 )` builds a zero-length width-1 segment spanning all layers and queries with `m_differentNetsOnly = false` and `m_overrideClearance = r` (`pcbnew/router/pns_router.cpp:135` to `:148`). So the slop radius is expressed as a clearance override, meaning it is a circular capture radius on every layer at once.

Also note that the query runs against `m_placer ? m_placer->CurrentNode() : m_world.get()` (`pcbnew/router/pns_router.cpp:128`), so while routing you hover against the speculative node, not the committed board.

**Candidate rejection**, in order, each one a `continue`:

1. `!item->IsRoutable()` (`:147`).
2. `!m_iface->IsPNSCopperLayer( item->Layers().Start() )` (`:150`).
3. `!m_iface->IsAnyLayerVisible( item->Layers() )` (`:153`).
4. `alg::contains( aAvoidItems, item )` (`:156`). The only caller that passes anything is `updateEndItem`, which passes `{ m_startItem }` (`:409`).

**Net filter.** `aIgnorePads` drops all solids (`:163`). Then the main scoring branch is entered only when `GetNetCode( aNet ) <= 0 || item->Net() == aNet` (`:167`), which is the sentinel convention from section 1.8: a null or non-positive net means "any net matches".

**The five priority slots** (`:163` to `:218`), which are the actual answer to "what does the tool prefer":

| Slot | Contents | Distance metric |
| --- | --- | --- |
| 0 | Nearest via or pad whose layer range overlaps the current layer | Distance to shape centre |
| 1 | Nearest track or arc whose layer range overlaps the current layer | Distance to the nearer of the two endpoints |
| 2 | Nearest via or pad on any layer | Distance to shape centre |
| 3 | Nearest track or arc on any layer | Distance to the nearer endpoint |
| 4 | A netless item overlapping the current layer, only in `RM_MarkObstacles` mode | none; last writer wins |

Two extra rules sit outside that table. A free pad on a different net qualifies for slot 0 with distance zero, but only when the cursor is strictly inside the pad shape (`:204` to `:212`, comment "Allow free pads only when already inside pad"). And netless items are only considered at all in mark-obstacles mode (`:213` to `:218`).

**Final resolution** (`:225` to `:239`): walk the slots in order; in high-contrast display mode, null out any candidate that does not overlap `tl`, which effectively disables slots 2 and 3; and apply `aLayer >= 0` as a hard post-filter on the winner.

So the summary rule is: **pads and vias beat tracks, current layer beats other layers, and the layer test dominates the kind test.** A track on the current layer beats a pad on a different layer.

### 4.3 checkSnap and snapToItem

`checkSnap` (`pcbnew/router/pns_tool_base.cpp:296` to `:329`) does two things. It refuses to snap to any link of the line currently being dragged (`:302` to `:310`), which is what stops a drag from snapping to itself. And it pushes the editor's magnetic settings into the PNS settings on every call (`:312` to `:318`), then answers `GetSnapToTracks()` for segments, arcs and vias and `GetSnapToPads()` for solids. It is consulted only from `updateEndItem`, never from `updateStartItem`.

`snapToItem` (`:450` to `:520`) computes the anchor:

- No item, or `!m_iface->IsItemVisible( aItem )`: plain grid align, using `GRID_VIAS` if a via is pending and `GRID_WIRES` otherwise (`:452` to `:455`).
- `SOLID_T`: if the solid has no explicit anchor points, `solid->Anchor( 0 )`, which is `m_pos` (`pcbnew/router/pns_solid.cpp:95` to `:98`). For pads that is the shape position minus the rotated offset (`pcbnew/router/pns_kicad_iface.cpp:1700` to `:1707`), so **pads always snap to their centre**. If anchor points exist, the nearest one wins. Anchor points are only ever populated for copper graphics, from `PCB_SHAPE::GetConnectionPoints()` (`pcbnew/router/pns_kicad_iface.cpp:2081`).
- `VIA_T`: via centre, unconditionally (`:483` to `:484`).
- `SEGMENT_T` or `ARC_T`: endpoint only if the cursor is within **half the track width** of it (`:492` to `:499`, the test is `distA_sq < w_sq || distB_sq < w_sq` where `w_sq = Square( Width()/2 )`); otherwise a grid-aware projection onto the geometry, `m_gridHelper->AlignToSegment` (`:504`) or `AlignToArc` (`:509`).
- Anything else: grid align.

The half-width endpoint rule is a nice piece of design worth copying: it means the snap target is exactly the region a user would perceive as "the end of the track".

Every call site calls `m_toolMgr->GetView()->SyncLayerVisibilityCache()` first, because `IsItemVisible` uses `ViewGetLOD` which reads that cache (`pcbnew/router/pns_tool_base.cpp:348` to `:349`).

### 4.4 updateStartItem versus updateEndItem

`updateStartItem` (`pcbnew/router/pns_tool_base.cpp:332` to `:362`):

- Ctrl+Shift is an escape hatch: no item, no snapping, cursor forced to the raw position, early return (`:340` to `:346`).
- `SetUseGrid( gal->GetGridSnapping() && !aEvent.DisableGridSnapping() )` and `SetSnap( !aEvent.Modifier( MD_SHIFT ) )` (`:352` to `:353`). Shift disables item snapping.
- `pickSingleItem( pos, nullptr, -1, aIgnorePads )` (`:355`): **any net, any layer**.
- If the grid is off and the picked item does not overlap the top layer, discard it (`:357` to `:358`).
- Snap, then `ForceCursorPosition( true, m_startSnapPoint )` (`:360` to `:361`).

`updateEndItem` (`:365` to `:435`) differs in five ways:

- Position comes from the live mouse, not the event (`:378`), with a fast-mouse correction that substitutes `aEvent.DragOrigin()` when a routing-state event arrives as a short drag (`:380` to `:386`).
- If the route has no net and the mode is not `RM_MarkObstacles`, it snaps to grid only and clears `m_endItem` (`:388` to `:396`).
- The layer is `m_router->IsPlacingVia() ? -1 : m_router->GetCurrentLayer()` (`:398` to `:401`); with a via pending, any layer is a valid endpoint.
- The pick is net-restricted and loops over every current net, excluding `m_startItem` (`:403` to `:413`).
- The result is gated by `m_gridHelper->GetSnap() && checkSnap( endItem )`; failing that, the item is dropped **and** the point falls back to plain grid alignment (`:415` to `:425`).

### 4.5 Snapping happens before Move, always

Every `m_router->Move()` call in the tool passes `m_endSnapPoint`, never a raw cursor position, and every one is immediately preceded by `updateEndItem()`. The full list is `pcbnew/router/router_tool.cpp:989`, `:1188`, `:1323`, `:1465`, `:1664`, `:1858`, `:1871`, `:1949`, `:1963`, `:1969`, `:2565`, `:3069`, `:3356`. The two exceptions are `InlineDrag`'s priming call at `pcbnew/router/router_tool.cpp:3040`, whose point was already snapped at `:2962`, and `ROUTER::Finish`'s internal move at `pcbnew/router/pns_router.cpp:600`, which targets a ratsnest anchor.

`updateEndItem` also forces the visible crosshair to the snapped point (`pcbnew/router/pns_tool_base.cpp:427`), so what the user sees and what the engine receives are the same coordinate. That invariant is worth preserving verbatim: a Rust port should make it structurally impossible to hand the engine an unsnapped point, for example by having `Move` take a `SnappedPoint` newtype.

### 4.6 Sequence diagrams

**Start routing on a pad.**

```
user left-clicks over a pad
  ROUTER_TOOL::MainLoop loop iteration
    evt->IsClick( BUT_LEFT )                                  router_tool.cpp:2468
    updateStartItem( evt )                                    pns_tool_base.cpp:332
      SyncLayerVisibilityCache
      pickSingleItem( pos, net=null, layer=-1, ignorePads=false )
        QueryHoverItems( pos, 0 )        -> exact hit         pns_router.cpp:154
        slot 0 <- the pad (SOLID on the current layer)        pns_tool_base.cpp:169
      snapToItem( pad, pos ) -> pad centre                    pns_tool_base.cpp:459
      ForceCursorPosition( true, m_startSnapPoint )
    performRouting( evt->Position() )                         router_tool.cpp:2474
      prepareInteractive()                                    router_tool.cpp:1676
        pcbLayer = getStartLayer( m_startItem )                router_tool.cpp:997
        frame->SetActiveLayer( pcbLayer )
        m_iface->SetStartLayerFromPCBNew( pcbLayer )           router_tool.cpp:1707
        m_iface->ImportSizes( sizes, m_startItem, null, pos )  router_tool.cpp:1710
        sizes.AddLayerPair( routeTop, routeBottom )
        m_router->UpdateSizes( sizes )
        highlightNets( true, { startNet } )                    router_tool.cpp:1721
        controls->SetAutoPan( true )
        m_router->StartRouting( m_startSnapPoint, m_startItem, pnsLayer )
          GetRuleResolver()->ClearCaches()                     pns_router.cpp:436
          isStartingPointRoutable(...)                         pns_router.cpp:438
          m_placer = LINE_PLACER                               pns_router.cpp:444
          m_placer->Start( aP, aStartItem )                    pns_router.cpp:472
          logger->Log( EVT_START_ROUTE, aP, item, &sizes, layer )
        frame->UndoRedoBlock( true )                           router_tool.cpp:1753
      enter the routing while( Wait(...) ) loop
```

**Move the mouse.**

```
TA_MOUSE_MOTION
  evt->IsMotion()                                             router_tool.cpp:1856
  updateEndItem( evt )                                        pns_tool_base.cpp:365
    layer = IsPlacingVia() ? -1 : GetCurrentLayer()
    for net in GetCurrentNets():
        pickSingleItem( mousePos, net, layer, false, { m_startItem } )
    if GetSnap() and checkSnap( endItem ):  m_endSnapPoint = snapToItem(...)
    else:                                    m_endSnapPoint = grid align
    ForceCursorPosition( true, m_endSnapPoint )
  m_router->Move( m_endSnapPoint, m_endItem )                 router_tool.cpp:1859
    logger->Log( EVT_MOVE, aP, endItem )                      pns_router.cpp:497
    movePlacing( aP, endItem )                                pns_router.cpp:789
      iface->EraseView()                                      pns_router.cpp:791
      m_placer->Move( aP, aEndItem )
      for each head LINE:
          iface->DisplayItem( line, clearance, false, PNS_HEAD_TRACE )   pns_router.cpp:804
          if line ends with a via:
              iface->DisplayItem( &via, clearance, false, PNS_HEAD_TRACE ) pns_router.cpp:821
      updateView( placer->CurrentNode( true ), current )      pns_router.cpp:827
        markViolations(...)  -> DisplayItem for each obstacle  pns_router.cpp:760
        GetRuleResolver()->ClearCacheForItems( added )         pns_router.cpp:766
        for item in added:    iface->DisplayItem( item, clearance, aDragging )
        for item in removed:  iface->HideItem( item )
      (LINE_PLACER also emits iface->DisplayRatline(...)       pns_line_placer.cpp:2028)
```

**Left click to fix a segment.**

```
TA_MOUSE_CLICK BUT_LEFT                                        router_tool.cpp:1926
  updateEndItem( evt )
  needLayerSwitch = m_router->IsPlacingVia()
  if m_router->FixRoute( m_endSnapPoint, m_endItem, false, false ):
      break out of the routing loop        # the route reached its real end
  # otherwise a segment was fixed and routing continues:
  if needLayerSwitch:  switchLayerOnViaPlacement()             router_tool.cpp:1016
  else:                updateSizesAfterRouterEvent( currentLayer, m_endSnapPoint )
  syncRouterAndFrameLayer()
  updateEndItem( evt )       # again: the fix changed the placer state
  m_router->Move( m_endSnapPoint, m_endItem )
  m_startItem = nullptr
```

`ROUTER::FixRoute` (`pcbnew/router/pns_router.cpp:915`) logs `EVT_FIX` and forwards to `m_placer->FixRoute( aP, aEndItem, aForceFinish )`. Note that **no board edit happens here.** `LINE_PLACER::FixRoute` adds the segments, arcs and vias to `m_lastNode`, pushes a `FIXED_TAIL` stage, and re-branches (`pcbnew/router/pns_line_placer.cpp:1669` to `:1747`). The host sees nothing until the loop exits.

**Switch layer, which places a via.**

```
user presses a layer hotkey (or V)
  the routing loop's Wait() does not consume it -> evt->SetPassEvent()  router_tool.cpp:2022
  the tool manager re-dispatches to onLayerCommand / onViaCommand      router_tool.cpp:3528, :3536
  handleLayerSwitch( evt, aForceVia )                          router_tool.cpp:1347
    resolve targetLayer (layerNext / layerPrev / layerToggle / explicit)
    if targetLayer == currentLayer: return
    if !aForceVia and m_router->SwitchLayer( pnsTarget ):      router_tool.cpp:1463
        # plain layer change succeeded, no via
        updateEndItem; updateSizesAfterRouterEvent; Move; return
    # SwitchLayer refused, so we must place a via
    viaType = getViaTypeFromFlags( evt->Parameter<int>() )     router_tool.cpp:1149
    resolve the implicit target layer from the nearest ratsnest anchor
                                                               router_tool.cpp:1537-1619
    resolve via diameter and drill (netclass constraints or user values)
                                                               router_tool.cpp:1621-1651
    sizes.SetViaType(...); sizes.AddLayerPair( pnsCurrent, pnsTarget )
    m_router->UpdateSizes( sizes )
    if !m_router->IsPlacingVia(): m_router->ToggleViaPlacement()
    updateEndItem; m_router->Move( m_endSnapPoint, m_endItem )
```

The key mechanism is at `pcbnew/router/router_tool.cpp:1463`: a bare layer change is attempted first, and the via appears exactly because `LINE_PLACER::SetLayer` refuses once anything has been chained. `SetLayer` succeeds only when the placer is idle, or when `!m_chainedPlacement` and the start item is null, a via, or a pad spanning the target layer (`pcbnew/router/pns_line_placer.cpp:1347` to `:1377`). Pressing a layer hotkey and pressing V are therefore the *same* code path with different `aForceVia`.

`AddLayerPair` stores the mapping in both directions (`pcbnew/router/pns_sizes_settings.cpp:36` to `:43`), which is why `PairedLayer()` is symmetric and why `switchLayerOnViaPlacement` can query the pre-switch layer and get the right answer (`pcbnew/router/router_tool.cpp:1016` to `:1034`).

**Undo the last segment (Backspace).**

```
routerUndoLastSegment / ACTIONS::doDelete / ACTIONS::undo     router_tool.cpp:1861
  if last = m_router->UndoLastSegment():                      pns_router.cpp:946
      logger->Log( EVT_UNFIX )
      m_placer->UnfixRoute()                                  pns_line_placer.cpp:1759
        pop a FIXED_TAIL::STAGE
        restore start point, direction, layer, via state
        rewind the shove springback
        re-branch m_lastNode
      WarpMouseCursor( last.value(), true )
      evt->SetMousePosition( last.value() )
  updateEndItem( evt )
  m_router->Move( m_endSnapPoint, m_endItem )
```

Backspace, Delete and Ctrl+Z are all bound to the same behaviour while routing.

**Finish on a pad, or double click.**

```
ACTIONS::finishInteractive or IsDblClick( BUT_LEFT )          router_tool.cpp:1983
  m_router->FixRoute( m_endSnapPoint, m_endItem, /*forceFinish*/ true, /*forceCommit*/ false )
  break
# fall out of the loop:
  m_router->CommitRouting()                                   router_tool.cpp:2026
    m_placer->CommitPlacement()                               pns_router.cpp:961
      (shove mode: rewind to the last locked springback node) pns_line_placer.cpp:1810
      Router()->CommitRouting( m_lastNode )                   pns_router.cpp:862
        classify removed / added / changed by parent identity  pns_router.cpp:877
        iface->RemoveItem / AddItem / UpdateItem
        iface->Commit()          -> BOARD_COMMIT::Push("Routing") pns_kicad_iface.cpp:2956
        m_world->Commit( aNode )
    StopRouting()                                             pns_router.cpp:967
      for net in modified nets: iface->UpdateNet( net )
      m_placer.reset(); m_dragger.reset()
      iface->EraseView()
      m_world->KillChildren(); m_world->ClearRanks()
  m_iface->SetCommitFlags( 0 )                                router_tool.cpp:2028
  finishInteractive()                                         router_tool.cpp:1759
```

**Escape or cancel.**

```
IsCancelInteractive || cancelCurrentItem || IsActivate || routerInlineDrag   router_tool.cpp:1992
  if cancel and ( m_inRouteSelected or not RoutingInProgress ): m_cancelled = true
  if IsActivate and not a move tool:                            m_cancelled = true
  break
# and then the SAME teardown as a normal finish runs:
  m_router->CommitRouting()          # commits everything already fixed
  finishInteractive()
```

This is the single most surprising behaviour in the tool. **Escape does not discard the route.** Because the loop always falls through to `CommitRouting()` at `pcbnew/router/router_tool.cpp:2026`, every segment the user already fixed with a click is committed. Only the un-fixed head geometry is discarded, and that discarding happens inside `LINE_PLACER::CommitPlacement`, which in shove mode rewinds to the last locked springback node (`pcbnew/router/pns_line_placer.cpp:1810` to `:1817`). A first Escape breaks out of `performRouting` back into `MainLoop`; a second Escape leaves the tool.

### 4.7 Auto finish and continue from end

`routerAttemptFinish` (`pcbnew/router/router_tool.cpp:1874` to `:1904`) calls `ROUTER::Finish()`. `Finish` (`pcbnew/router/pns_router.cpp:569` to `:614`) requires the `ROUTE_TRACK` state and a non-empty trace, finds the nearest unconnected ratsnest anchor, then iterates `Move( otherEnd, otherEndItem )` up to five times until `placer->CurrentEnd()` stops changing, and only calls `FixRoute` if the settled point equals the anchor *and* the anchor's layers overlap the current layer. So auto-finish is a fixed-point iteration on the placer, not a separate autorouting algorithm.

`routerContinueFromEnd` (`pcbnew/router/router_tool.cpp:1905` to `:1925`) calls `ROUTER::ContinueFromEnd` (`pcbnew/router/pns_router.cpp:617` to `:653`), which captures the current end, finds the far ratsnest anchor, **commits the current route**, restarts routing from the far anchor, and primes with `Move( currentEnd, nullptr )`. The tool then sets `SetCommitFlags( APPEND_UNDO )` if the previous leg had placed anything, so the two legs land in one undo entry (`pcbnew/router/router_tool.cpp:1919`).

`hasOtherEnd`, the condition that enables both actions in the context menu, tests `board->GetConnectivity()->GetRatsnestForNet( currentNet )` for a non-empty edge list (`pcbnew/router/router_tool.cpp:635` to `:650`). So the host must supply a ratsnest for these features to be reachable at all.

### 4.8 Mode switching

Two orthogonal enums both called "mode".

`PNS::ROUTER_MODE` (`pcbnew/router/pns_router.h:67` to `:73`): `PNS_MODE_ROUTE_SINGLE`, `PNS_MODE_ROUTE_DIFF_PAIR`, `PNS_MODE_TUNE_SINGLE`, `PNS_MODE_TUNE_DIFF_PAIR`, `PNS_MODE_TUNE_DIFF_PAIR_SKEW`. Set through `ROUTER::SetMode` from `MainLoop` (`pcbnew/router/router_tool.cpp:2409`) and `RouteSelected` (`:2234`), and consumed in `ROUTER::StartRouting` to choose the placer class (`pcbnew/router/pns_router.cpp:441` to `:465`).

`PNS::PNS_MODE` (`pcbnew/router/pns_routing_settings.h:39` to `:44`): `RM_MarkObstacles`, `RM_Shove`, `RM_Walkaround`. Changed by `ChangeRouterMode` (`pcbnew/router/router_tool.cpp:2070`) and cycled MarkObstacles to Shove to Walkaround by `CycleRouterMode` (`:2082`). This is the one users think of as "the router mode".

Corner mode is a third axis, cycled MITERED_45, ROUNDED_45, MITERED_90, ROUNDED_90 by `handlePnSCornerModeChange` (`pcbnew/router/router_tool.cpp:946` to `:994`), which always ends with `updateEndItem` plus `Move` to refresh the preview.

### 4.9 Commit and undo granularity

This is the answer most likely to differ from a reader's intuition, so it is worth stating flatly: **the whole route is one undo entry, not one per fixed segment.**

The chain is: the `BOARD_COMMIT` is owned by the interface and created once per `SetHostTool` (`pcbnew/router/pns_kicad_iface.cpp:3035`); `PNS_KICAD_IFACE::Commit()` is the only place that pushes it, then immediately allocates a fresh one (`pcbnew/router/pns_kicad_iface.cpp:2956` to `:2957`); `Commit()` is called exactly once, from `ROUTER::CommitRouting( NODE* )` (`pcbnew/router/pns_router.cpp:910`); and `CommitRouting( NODE* )` is reached only when the interactive loop exits, via `LINE_PLACER::CommitPlacement` (`pcbnew/router/pns_line_placer.cpp:1810` to `:1825`). Intermediate clicks only branch nodes inside the PNS world.

`ROUTER::CommitRouting( NODE* )` early-returns when nothing was placed: `if( m_state == ROUTE_TRACK && !m_placer->HasPlacedAnything() ) return;` (`pcbnew/router/pns_router.cpp:864`).

`SetCommitFlags( int )` (`pcbnew/router/pns_kicad_iface.h:177`) is the only mechanism for merging several *routes* into one undo step. It is OR-ed into the push, and is used with `APPEND_UNDO` by `RouteSelected` for every route after the first in a batch (`pcbnew/router/router_tool.cpp:2247` to `:2261`), by `routerContinueFromEnd` (`:1919`), and by `OptimizeSelected` (`:2356`). `performRouting` resets it to 0 after every route (`:2028`), and `routerAttemptFinish` clears it on failure so a manual intervention starts a fresh undo group (`:1898`).

The whole interactive session runs inside `frame()->UndoRedoBlock( true )` (`:1753`, released at `:1787`), and an undo or redo event reaching the routing loop is a `wxFAIL` (`:2004` to `:2008`).

For a LibrePCB port this maps cleanly: open one `UndoCommandGroup` when the route ends, apply the whole `CommitDiff`, close it. The `APPEND_UNDO` behaviour is an optional refinement for batch routing.

### 4.10 Dragging

`performDragging` (`pcbnew/router/router_tool.cpp:2516` to `:2666`) mirrors `performRouting` with three differences worth noting. It shows a confirmation dialog with an OK label of "Drag Anyway" for locked items (`:2525` to `:2534`). It surfaces a DRC status overlay when `dragger->GetForceMarkObstaclesMode` reports a violation, hinting "Ctrl+click to commit anyway" (`:2563` to `:2590`). And the Ctrl modifier on the fixing click becomes `aForceCommit` in `FixRoute` (`:2591` to `:2598`), which is the "commit despite DRC" gesture.

`ROUTER::StartDragging` picks the dragger class by the shape of the start set: all solids gives `COMPONENT_DRAGGER`, more than one segment or arc gives `MULTI_DRAGGER`, otherwise `DRAGGER` (`pcbnew/router/pns_router.cpp:176` to `:191`).

`InlineDrag` (`pcbnew/router/router_tool.cpp:2773` to `:3257`) is the drag entered from the selection tool. Two details are worth copying: it clears the lock flag *before* `SyncWorld` so virtual vias are not generated for the item being dragged, with a comment noting the lock cannot be reliably restored (`:2816` to `:2827`); and it calls `m_router->SyncWorld()` unconditionally because the world may be stale after an unrelated move (`:2850` to `:2852`).

### 4.11 ROUTER_PREVIEW_ITEM: what the host must be able to draw

`ROUTER_PREVIEW_ITEM` (`pcbnew/router/router_preview_item.h:56`) is the concrete answer to "what rendering capability does the host owe the router".

**Flags.** `PNS_HEAD_TRACE 1`, `PNS_HOVER_ITEM 2`, `PNS_SEMI_SOLID 4`, `PNS_COLLISION 8` (`pcbnew/router/router_preview_item.h:50` to `:53`). Where each is actually set: `PNS_HEAD_TRACE` by `ROUTER::movePlacing` for the head line and its via (`pcbnew/router/pns_router.cpp:804`, `:821`); `PNS_SEMI_SOLID` by the interface for rule-area zones (`pcbnew/router/pns_kicad_iface.cpp:2485`); `PNS_COLLISION` inside `Update()` from the item's own `MK_VIOLATION` marker (`pcbnew/router/router_preview_item.cpp:211`). `PNS_HOVER_ITEM` is never set anywhere in this tree, so its two code paths (`pcbnew/router/router_preview_item.cpp:217`, `:610`) are dead. Do not port it.

**Colour rules** (`pcbnew/router/router_preview_item.cpp:583` to `:614` and `:146` to `:218`). The host must supply: a per-copper-layer colour; an optional per-net or per-netclass override in `NET_COLOR_MODE::ALL`; a saturated variant for the head trace (`color.Saturate( 1.0 )` at `:608`); alpha 0.8 for ordinary preview items (`:147`); a fixed grey `COLOR4D( 0.7, 0.7, 0.7, 0.8 )` for vias (`:172`); an opaque green `COLOR4D( 0, 1, 0, 1 )` for collisions (`:214`); a ratline colour derived from the net or netclass and then brightened (`pcbnew/router/pns_kicad_iface.cpp:2587`); and two path-line colours, yellow at 0.6 alpha for importance 1 and grey at 0.6 alpha for importance 0 (`pcbnew/router/pns_kicad_iface.cpp:2536` to `:2539`).

**Depth ordering.** Everything is drawn onto one view layer, `LAYER_SELECT_OVERLAY`, and the whole board layer stack is compressed into fractional depth steps of `LayerDepthFactor = 0.001` so it fits inside one view-group sublayer (`pcbnew/router/router_preview_item.h:66` to `:78`). `m_depth = m_originDepth - ( ( Layers().Start() + 1 ) * LayerDepthFactor )` (`pcbnew/router/router_preview_item.cpp:148`), so items on deeper PNS layers draw in front. Vias are pushed in front of every copper layer by subtracting `PCB_LAYER_ID_COUNT * LayerDepthFactor` (`:200`), and path lines by `PathOverlayDepth` (`pcbnew/router/pns_kicad_iface.cpp:2532`).

**The two-pass clearance draw.** `ViewDraw` (`pcbnew/router/router_preview_item.cpp:542` to `:580`) carries the comment "The order of draw here is important. Cairo doesn't currently support z-ordering, so we need to draw the clearance first to ensure it is in the background" (`:552` to `:554`). The clearance halo is drawn at `m_originDepth` in a hard-coded `DARKDARKGRAY` at 0.9 stroke and 0.7 fill alpha, then the real geometry at `m_depth` in `m_color`. Semi-solid items that are not in collision are drawn outline-only (`:563` to `:565`).

**Shape repertoire** that `drawShape` (`:292` to `:539`) requires:

| Shape | Body | Clearance halo |
| --- | --- | --- |
| Line chain (and triangle) | polyline at `m_width`, with true arcs for the chain's arc segments | same at `m_width + 2 * clearance` |
| Segment | capsule of the segment's own width | capsule at `width + 2 * clearance` |
| Circle | filled circle, or an **annulus** when the item has a circular hole (stroke width `halfWidth + R - r`, radius `(halfWidth + R + r)/2`, `:373` to `:384`) | circle at `R + clearance` |
| Rect | rectangle | four lines at line width `2 * clearance` |
| Simple polygon | filled convex polygon | polyline with the first point re-appended |
| Arc | true arc at the arc's width | same, widened |
| Ellipse | ellipse or elliptical arc | same, widened |
| Compound, poly set | `wxFAIL_MSG`, unsupported | |

A degenerate zero-length chain segment renders as a filled dot of radius `lineWidth/2` (`:262` to `:269`). Holes not folded into an annulus are stroked at line width 1 (`:523` to `:538`).

So the minimum primitive set a host renderer owes the router is: stroked and filled polyline with width, true circular arc, circle, filled dot, capsule, rectangle, convex polygon, ellipse and elliptical arc, each drawable twice at two depths.

**The one thing that is not obvious.** Edge-cut items are synced with zero width for collision purposes (section 3.6) but the preview item explicitly re-widens the cloned shape back to its true width for display (`pcbnew/router/router_preview_item.cpp:60` to `:73`). A port must keep the display width separate from the collision width for outline items.


## 5. Horizon EDA's integration

Horizon EDA vendored KiCad's PNS at `3rd_party/router/` and wrote a host adapter at `src/router/pns_horizon_iface.hpp` and `.cpp` (43 KB). It is the only public example of a third party integrating this router into a different data model, so it is the closest thing to a template for LibrePCB.

### 5.1 The vendored vintage, and why it matters

`git -C /home/Tubbles/dev/ref/horizon log --oneline -20 -- 3rd_party/router` returns nothing usable: the clone is shallow (`.git/shallow` exists) and only the tip commit is present. So the vendoring history cannot be reconstructed from this checkout.

The vintage is recorded once, in Horizon's third-party table at `README.md:47`, which names KiCad 6.0.4 and links the 6.0.4 tag of `pcbnew/router`. There is no README under `3rd_party/router/` itself. In-tree copyright headers corroborate it: `3rd_party/router/router/pns_node.h:5` still reads `Copyright (C) 2016-2021 KiCad Developers` where master reads `Copyright The KiCad Developers`.

**The module boundary is visible in the file inventory, and it is the single most useful structural fact in this section.** Diffing the two router directories, every file in `3rd_party/router/router/` also exists in `pcbnew/router/`, and the files present only in master split cleanly in two. Four are the KiCad host layer Horizon threw away and replaced: `pns_kicad_iface.{h,cpp}`, `router_tool.{h,cpp}`, `pns_tool_base.{h,cpp}`, `router_preview_item.{h,cpp}`. The rest are post-6.0.4 upstream additions: `pns_hole.{h,cpp}`, `pns_multi_dragger.{h,cpp}`, `pns_helpers.{h,cpp}`, `router_status_view_item.{h,cpp}`. Nothing was deleted. So **the router core is 60 files taken verbatim and the host layer is four files rewritten from scratch**, which is exactly where a Rust port should draw its crate boundary. Horizon's replacement for those four is about 2500 lines against master's roughly 250 KB.

Build-wise the router is one isolated static library with `gtkmm` as its only dependency and no wx (`meson.build:877` to `:937`, flags at `:53` to `:56`), and note that both the adapter (`meson.build:910`) and the driving tool (`meson.build:911`) are compiled **into that library** rather than into the main application.

The interface diff itself is much older than master. Comparing `3rd_party/router/router/pns_router.h:86` to `:113` against `pcbnew/router/pns_router.h:91` to `:152`:

**`ROUTER_IFACE` in the 6.0.4-era copy has 18 pure virtuals.** Master has 31 pure virtuals plus one defaulted.

Present in both, unchanged in spirit (13): `SyncWorld`, `AddItem`, `UpdateItem`, `RemoveItem`, `IsAnyLayerVisible`, `IsItemVisible`, `HideItem`, `Commit`, `StackupHeight`, `EraseView`, `GetWorld`, `GetRuleResolver`, `GetDebugDecorator`.

Present in both but with a changed signature (5):

| 6.0.4 | master | Nature of the change |
| --- | --- | --- |
| `DisplayItem( const ITEM*, int aClearance, bool aEdit = false )` (`3rd_party/router/router/pns_router.h:99`) | adds `int aFlags = 0` (`pcbnew/router/pns_router.h:106`) | the four preview flags |
| `DisplayRatline( const SHAPE_LINE_CHAIN&, int aColor = -1 )` (`:100`) | `DisplayRatline( const SHAPE_LINE_CHAIN&, NET_HANDLE )` (`:109`) | the host now colours by net itself |
| `ImportSizes( SIZES_SETTINGS&, ITEM*, int aNet )` (`:103`) | adds `NET_HANDLE` and `VECTOR2D aStartPosition` (`:112`) | the far-end width heuristic of section 1.9 |
| `UpdateNet( int aNetCode )` (`:107`) | `UpdateNet( NET_HANDLE )` (`:117`) | net identity change |
| `IsFlashedOnLayer( const ITEM*, int )` (`:98`) | a second overload taking a `PNS_LAYER_RANGE` is added (`:104`) | range queries in the collision path |

Added in master (14): `IsPNSCopperLayer`, `DisplayPathLine`, `GetNetCode`, `GetNetName`, `GetOrphanedNetHandle`, `GetBoardLayerFromPNSLayer`, `GetPNSLayerFromBoardLayer`, `CalculateRoutedPathLength`, `CalculateRoutedPathDelay`, `CalculateLengthForDelay`, `CalculateDelayForShapeLineChain`, `GetSignalAggregate`, `GetNetBoardLength`, plus the non-virtual `GetViaLayerRange`.

Removed in master: nothing.

**The reading that matters for a port: the intersection of the two vintages is the essential contract.** Thirteen methods survived unchanged across four years and a major refactor. Of the fourteen additions, eleven are net identity, layer mapping, or length and delay tuning; only `IsPNSCopperLayer`, `DisplayPathLine` and the second `IsFlashedOnLayer` overload touch routing behaviour.

`RULE_RESOLVER` changed more. The 6.0.4 version (`3rd_party/router/router/pns_node.h:78` to `:100`) has 9 pure virtuals plus one defaulted:

```cpp
int Clearance( const ITEM* aA, const ITEM* aB );
int HoleClearance( const ITEM* aA, const ITEM* aB );
int HoleToHoleClearance( const ITEM* aA, const ITEM* aB );
int DpCoupledNet( int aNet );  int DpNetPolarity( int aNet );
bool DpNetPair( const ITEM*, int& aNetP, int& aNetN );
bool IsDiffPair( const ITEM* aA, const ITEM* aB );
bool QueryConstraint( CONSTRAINT_TYPE, const ITEM*, const ITEM*, int aLayer, CONSTRAINT* );
wxString NetName( int aNet );
virtual void ClearCacheForItem( const ITEM* aItem ) {}
```

Removed in master: `HoleClearance` and `HoleToHoleClearance` (folded into `Clearance` via the `CT_HOLE_CLEARANCE` and `CT_HOLE_TO_HOLE` constraint types, `pcbnew/router/pns_kicad_iface.cpp:908` to `:926`), `IsDiffPair` (dead even in 6.0.4, see below), and the singular `ClearCacheForItem`.

Added in master: `HasUserDefinedPhysicalConstraint`, `NetCode`, `IsInNetTie`, `IsNetTieExclusion`, `IsDrilledHole`, `IsNonPlatedSlot`, `IsKeepout`, `ClearCacheForItems` (plural), `ClearCaches`, `ClearTemporaryCaches`, `ClearanceEpsilon`, `HullCache`.

`CONSTRAINT_TYPE` in 6.0.4 (`3rd_party/router/router/pns_node.h:55` to `:66`) runs 1 through 9 and is a **strict prefix** of master's 1 through 13 (`pcbnew/router/pns_node.h:51` to `:66`); the four additions are `CT_DIFF_PAIR_SKEW`, `CT_MAX_UNCOUPLED`, `CT_PHYSICAL_CLEARANCE`, `CT_PHYSICAL_HOLE_CLEARANCE`.

### 5.2 The parent-item wrapper: the single best idea to copy

KiCad's `PNS::ITEM::Parent()` returns a `BOARD_ITEM*` that the router core then downcasts. Horizon changed the type. In the vendored copy, `pns_item.h:149` to `:150` and `:250` read:

```cpp
void SetParent( const PNS_HORIZON_PARENT_ITEM* aParent ) { m_parent = aParent; }
const PNS_HORIZON_PARENT_ITEM* Parent() const { return m_parent; }
...
const PNS_HORIZON_PARENT_ITEM*   m_parent;
```

`PNS_HORIZON_PARENT_ITEM` (`src/router/pns_horizon_iface.hpp:29` to `:64`) is a tagged tuple of six nullable Horizon pointers, one per object kind the router can see, with an equality operator over all six:

```cpp
const horizon::Track *track = nullptr;
const horizon::Via *via = nullptr;
const horizon::BoardPackage *package = nullptr;
const horizon::Pad *pad = nullptr;
const horizon::BoardHole *hole = nullptr;
const horizon::Keepout *keepout = nullptr;
```

Note the pad case carries **two** pointers, package and pad, because a pad has no identity independent of its placement. That is a real modelling need that a single opaque pointer cannot express.

Ownership is a `std::list<PNS_HORIZON_PARENT_ITEM> parents` on the interface (`src/router/pns_horizon_iface.hpp:158`), interned through `get_or_create_parent` (`src/router/pns_horizon_iface.cpp:449` to `:457`), which is a linear `std::find` over the list. A `std::list` is used specifically because pointers into it must stay stable as it grows. `SyncWorld` clears the list first (`src/router/pns_horizon_iface.cpp:767`), so parent pointers do not survive a re-sync.

The cost of *not* doing this in 6.0.4 is visible as four commented-out blocks inside the vendored router core, every one of them a `dynamic_cast` from `Parent()` to a KiCad type:

1. `3rd_party/router/router/pns_item.cpp:58` to `:91`, the entire keepout enforcement inside `ITEM::collideSimple`, including `dynamic_cast<ZONE*>( Parent() )` at `:83` and `:84`. Live code resumes at `:92`. Master has since moved this exact logic behind `RULE_RESOLVER::IsKeepout` (`pcbnew/router/pns_node.h:165`), which is the right answer.
2. `3rd_party/router/router/pns_router.cpp:223` to `:266`, the `switch( parent->Type() )` in `isStartingPointRoutable` that produced the human-readable failure reasons quoted in section 3.4, leaving a bare `return false`.
3. `3rd_party/router/router/pns_topology.cpp:309` to `:428`, `AssembleTuningPath` short-circuited with an inserted `return initialPath;` before a block that used `PAD::GetEffectivePolygon()` and `PAD::FlashLayer()`. Master's fix was to pass the `ROUTER_IFACE*` into `AssembleTuningPath` and add `CalculateRoutedPathLength` to the interface.
4. `3rd_party/router/router/pns_logger.cpp:68`, `ent.uuid = "null";` replacing `item->Parent()->m_Uuid.AsString()`, which makes Horizon's logs non-replayable.

All four are symptoms of one design flaw. **Making the parent an opaque host-defined handle from day one removes the need for every one of these patches**, and master has been converging on the same conclusion by adding `m_sourceItem` alongside `m_parent` (`pcbnew/router/pns_item.h:201` to `:202`) and pushing the remaining downcasts behind resolver methods.

### 5.3 The rule resolver

`PNS_HORIZON_RULE_RESOLVER` (`src/router/pns_horizon_iface.cpp:94` to `:117`) has three members: the rule set, the interface, and a precomputed `std::vector<const horizon::RuleClearanceCopperKeepout*>` sorted by priority (`:115`, populated at `:129`). **There is no clearance cache and no hull cache at all.** Every `Clearance` call goes to `m_rules->get_clearance_copper(...)` afresh.

The 6.0.4 contract does offer an invalidation hook, `virtual void ClearCacheForItem( const ITEM* ) {}` (`3rd_party/router/router/pns_node.h:99`), called for every added item by `ROUTER::updateView` (`3rd_party/router/router/pns_router.cpp:562`). Horizon does not override it, because there is nothing to invalidate.

Two consequences of that are worse than they first look, and both are inside the collision inner loop. `get_clearance_copper_other`, which serves the board-edge and NPTH branches, calls `get_rules_sorted<RuleClearanceCopperOther>()` **inline on every query** (`src/board/board_rules.cpp:675`), and that template builds a map, then a vector, does a `dynamic_cast` per element, and sorts. And `RuleMatch::match` in either regex mode calls `Glib::Regex::create(u)` per invocation (`src/rules/rule_match.cpp:76`, `:82`), compiling a regular expression from scratch inside the same loop. The natural memoisation key here is tiny, `(net_a, net_b, layer, patch_a, patch_b)`, and nothing uses it. This is the clearest defect in the integration, and it is precisely why master added `ClearCaches`, `ClearTemporaryCaches` and `HullCache` (`pcbnew/router/pns_node.h:170` to `:182`).

`Clearance` (`src/router/pns_horizon_iface.cpp:153` to `:270`) maps PNS item kinds onto Horizon's `PatchType` taxonomy through `patch_type_from_kind` (`:134`), then refines it from the parent: a pad on a through padstack becomes `PAD_TH`, a hole on a `HOLE` padstack becomes `HOLE_PTH`, a hole on a `MECHANICAL` padstack becomes `HOLE_NPTH` (`:167` to `:180`). The dispatch is then a four-way chain:

```
if either parent is &parent_dummy_outline:   get_clearance_copper_other( net, layer ).get_clearance( pt, BOARD_EDGE )
elif either is a MECHANICAL padstack:        get_clearance_copper_other( net, layer ).get_clearance( pt, HOLE_NPTH )
elif either parent has a keepout:            first matching RuleClearanceCopperKeepout, else 0
else:                                        get_clearance_copper( netA, netB, layer ).get_clearance( ptA, ptB )
```

and every branch adds a `routing_offset` taken from the rule, overridable per session through `set_override_routing_offset` (`src/router/pns_horizon_iface.hpp:124`). That override is Horizon's answer to KiCad's clearance epsilon, but inverted: it *widens* clearance for routing rather than narrowing it.

Two structural notes. First, `Clearance( aA, nullptr )` returns a hard-coded `1e6` (`src/router/pns_horizon_iface.cpp:155` to `:156`), where KiCad computes the item's own worst-case clearance. Second, the layer selection when one or both items are multilayer is explicitly approximate, with three `fixme` comments (`:179`, `:204`, `:245`) and a final `layer = layers_b.Start(); // fixme, good enough for now` (`:245`).

The rest of the resolver is thin. `HoleClearance` (`:272`) opens with `return 0;` followed by unreachable code. `HoleToHoleClearance` (`:282`) returns 0 with the comment "good enough for now". `IsDiffPair` (`:289`) throws `std::runtime_error("IsDiffPair not implemented")` with the comment "not used anywhere", which is correct: nothing calls it in either vintage. `QueryConstraint` (`:296` to `:306`) handles exactly one type:

```cpp
if (aType == CONSTRAINT_TYPE::CT_CLEARANCE) {
    // only used in  MEANDER_PLACER_BASE::Clearance
    aConstraint->m_Value.SetMin(0);
    return true;
}
throw std::runtime_error("QueryConstraint not implemented");
```

The diff-pair trio is real, though, and simpler than KiCad's suffix matching because Horizon models pairing explicitly: `DpCoupledNet` reads `net->diffpair` (`:308`), `DpNetPolarity` reads `net->diffpair_primary` (`:317`), `DpNetPair` orders the two from the same flag (`:325`). That is worth noting: **a host with an explicit diff-pair relation gets these three for free, whereas KiCad has to parse net names.**

### 5.4 SyncWorld

`PNS_HORIZON_IFACE::SyncWorld` (`src/router/pns_horizon_iface.cpp:759` to `:826`) is 68 lines against KiCad's 161, and the shape is the same: iterate the host model, `aWorld->Add(...)`, then install a fresh rule resolver and set the max clearance.

Order: tracks and arcs, vias, holes, package pads, board outline polygons, keepout contours.

| Horizon object | PNS item | Notes |
| --- | --- | --- |
| `Track`, straight | `SEGMENT` | `syncTrack` at `:459`. Width, single layer, parent, `MK_LOCKED` if `track->locked` |
| `Track`, arc | `ARC` | `syncTrackArc` at `:478`, built with `SHAPE_ARC::ConstructFromStartEndCenter`, so Horizon stores centre-based arcs and converts, where KiCad stores three points |
| `Via` | `VIA` | `syncVia` at `:737`. Layer span from `via->span`; type is `THROUGH` if the span equals the through range, else `BLIND_BURIED` (`:748`). **No separate hole item**, because 6.0.4 has no `HOLE_T` |
| `Pad` | one `SOLID` | `syncPad` at `:559`, delegating to `syncPadstack` at `:605` |
| `BoardHole` | one `SOLID` | `syncHole` at `:577`, same padstack path |
| Outline `Polygon` on `L_OUTLINE` | **many `SEGMENT`s** | `syncOutline` at `:500`, see below |
| `KeepoutContour` on a copper layer | **many `SEGMENT`s** | `syncKeepout` at `:523`, see below |
| Planes, copper pours | **not synced** | Same as KiCad (section 3.5) |
| Text, silkscreen, dimensions | **not synced** | Horizon syncs no graphics at all |

`syncPadstack` (`:605` to `:735`) has two paths, and the fast one is a performance decision worth copying. When a padstack has exactly one copper shape and no copper polygons (`:623`) and that shape sits at the padstack origin (`shape_is_at_origin`, `:590`), a `Shape::Form::CIRCLE` becomes a native `SHAPE_CIRCLE` (`:637`) and a `Shape::Form::RECTANGLE` at a multiple of 90 degrees becomes a native `SHAPE_RECT` (`:655`), with width and height swapped at 90 and 270 degrees (`angle_needs_swap` at `:600`, applied at `:652`). Both have closed-form collision and hull routines in kimath; `SHAPE_SIMPLE` does not. On a board of ordinary SMD rectangles this is the difference between an interactive shove and a slideshow.

The slow path unions every copper shape and polygon of the padstack with Clipper into a single outline and makes one `SHAPE_SIMPLE` from it. It **throws** `std::runtime_error("invalid pad polygons: " + count)` if the union yields more than one outline (`:722`). KiCad's equivalent falls back to a polygon and keeps going (`pcbnew/router/pns_kicad_iface.cpp:1729` to `:1735`); Horizon aborts the whole sync. That is a robustness regression, not a design choice to copy.

**The outline and keepout representation is the biggest divergence from KiCad, and it is instructive.** Where KiCad makes the board edge a zero-width `SOLID` spanning all copper layers (section 3.6), Horizon walks the outline polygon and emits, for every edge and *every* copper layer, a `PNS::SEGMENT` of width 10 nm marked `MK_LOCKED` with a shared static `&parent_dummy_outline` parent (`src/router/pns_horizon_iface.cpp:511` to `:519`). Keepouts get exactly the same treatment (`:543` to `:555`), with a per-keepout parent so the resolver can find the matching rule.

This works, and it is simple, but it has two costs. A polygon with N edges on an 8-layer board becomes 8N indexed items rather than one, which inflates the spatial index and every broad-phase query. And a 10 nm wide segment is a *line*, not an *area*, so nothing prevents the router from placing geometry entirely inside a keepout as long as it does not cross the boundary. Horizon patches the start-of-route case host-side with its own `ClipperLib::PointInPolygon` test (`src/core/tools/tool_route_track_interactive.cpp:756` to `:772`, called at `:868` to `:874`, producing "Can't start routing in keepout"); mid-route entry is unprotected. The keepout filter is also coarse: `syncKeepout` returns early unless `keepout->patch_types_cu.count(horizon::PatchType::TRACK)` (`:528`), so a keepout that forbids vias but permits tracks is simply not synced, which is precisely the per-item-kind discrimination that the commented-out block in `pns_item.cpp` used to provide.

`SetMaxClearance` is `4 * rules->get_max_clearance()` (`:825`), against KiCad's `worstClearance + epsilon` (`pcbnew/router/pns_kicad_iface.cpp:2452`). The factor of four is an unexplained safety margin; it costs broad-phase performance but is safe in the direction that matters (section 1.2).

### 5.5 Net handles and layer mapping

6.0.4 has no `NET_HANDLE`; nets are plain `int` codes. Horizon assigns them lazily and densely from net UUIDs (`src/router/pns_horizon_iface.cpp:377` to `:388`):

```cpp
int PNS_HORIZON_IFACE::get_net_code(const horizon::UUID &uu)
{
    if (net_code_map.count(uu)) return net_code_map.at(uu);
    net_code_map_r.emplace_back(&board->block->nets.at(uu));
    auto nc = net_code_map_r.size() - 1;
    net_code_map.emplace(uu, nc);
    return nc;
}
```

The reverse map is a `std::vector<Net*>` indexed by code (`:390` to `:399`). Exactly the same lazy interning is done for via definitions (`:401` to `:421`), so the router can carry a via style through an `int`.

This has a latent bug worth avoiding: **the first net interned gets code 0**, and `pickSingleItem` in the tool treats `aNet <= 0` as "match any net" (`src/core/tools/tool_route_track_interactive.cpp:336`). One net silently becomes a wildcard. Master avoids this by making the handle an opaque pointer and putting the sentinel behind `GetNetCode`, but the `<= 0` convention survives (section 1.8), so the hazard is inherited rather than removed. A Rust port should use `Option<NetId>` and a real newtype.

Layer mapping is a hard-coded ten-case switch in both directions (`src/router/pns_horizon_iface.cpp:20` to `:48` and `:50` to `:92`), from Horizon's `BoardLayers::TOP_COPPER`, `IN1_COPPER` through `IN8_COPPER`, `BOTTOM_COPPER` onto KiCad's `F_Cu`, `In1_Cu` through `In8_Cu`, `B_Cu`. Both are `static`, so the mapping is board-independent, which is only correct because 6.0.4's PNS layer space *is* KiCad's sparse `PCB_LAYER_ID` enum rather than the dense copper index master introduced (section 1.7). The hard cap at eight inner layers is a real limitation.

### 5.6 How the interface implements the rest of the contract

This is the most useful table in the section, because it shows how much of the contract a working integration can leave as a stub.

| Method | Horizon implementation | Line |
| --- | --- | --- |
| `AddItem` | Mutates the `Board` **immediately**: creates a `Track` with a random UUID for `SEGMENT_T` and `ARC_T`, creates a `Via` for `VIA_T`, connects endpoints to existing pads or junctions via `find_pad` and `find_junction`, and back-patches `aItem->SetParent(...)` | `:983` |
| `RemoveItem` | `board->tracks.erase(uuid)` or `board->vias.erase(uuid)`, recording the endpoints in `junctions_maybe_erased` for later cleanup | `:920` |
| `UpdateItem` | Handles **only** `VIA_T`, moving the via's junction and merging coincident same-net junctions. **Throws** `std::runtime_error` for every other kind | `:1190` |
| `Commit` | `board->update_junction_connections()`, then deletes any junction left with no connections, then `EraseView()`. **It does not touch undo at all** | `:1131` |
| `DisplayItem` | Adds a canvas line, arc or via marker to `m_preview_items`; `SOLID_T` is a deliberate no-op; anything else `assert(false)` | `:840` |
| `DisplayRatline` | One canvas line in `ColorP::AIRWIRE_ROUTER` on a magic layer 10000 | `:1240` |
| `HideItem` | `canvas->hide_obj(ref)` for tracks and vias only | `:905` |
| `EraseView` | `canvas->show_all_obj()` then removes every preview object | `:828` |
| `UpdateNet` | `board->update_airwires(false, {net->uuid})`, that is, a real ratsnest refresh | `:1149` |
| `IsAnyLayerVisible` | **throws** `std::runtime_error("IsAnyLayerVisible not implemented")` | `:1177` |
| `IsItemVisible` | **throws** `std::runtime_error("IsItemVisible not implemented")` | `:1183` |
| `IsFlashedOnLayer` | `return true;` unconditionally | `:1225` |
| `ImportSizes` | `return true;` and nothing else; the tool sets sizes itself | `:1230` |
| `StackupHeight` | `return 0;` | `:1235` |
| `GetDebugDecorator` | **returns `nullptr`** | `:1162` |
| `GetWorld` | returns the cached node | `hpp:91` |

Five observations follow.

**Two virtuals throw and it is fine.** `IsAnyLayerVisible` and `IsItemVisible` are only ever called from KiCad's own tool base (`pcbnew/router/pns_tool_base.cpp:153`, `:452`), never from the engine. Horizon wrote its own tool and therefore never reaches them. That is a clean confirmation of the section 1.12 split between engine methods and tool methods.

**`GetDebugDecorator` returning null is safe** because `PNS_DBG` null-checks (`pcbnew/router/pns_debug_decorator.h:126`). It also means Horizon gets no router debug graphics at all.

**`IsFlashedOnLayer` returning `true` is correct for Horizon and would be correct for LibrePCB v1**, because neither models removed annular rings. But it is the one stub that becomes a correctness bug the moment the host grows that feature (section 1.6).

**The commit model is entirely different from KiCad's, and simpler.** There is no transaction object. `AddItem` and `RemoveItem` mutate the live `Board` as they are called, and `Commit()` only repairs junction bookkeeping. Undo comes from Horizon's document-level snapshot model: `HistoryItemBoard` holds a whole `Block` and a whole `Board` **by value** (`src/core/core_board.cpp:826` to `:834`), so every undo step is a deep copy of the document and a revert is a one-line restore. On top of that, the core wraps every tool in a `catch (const std::exception&)` that calls `history_load(history_manager.get_current())` and `rebuild_internal(true, "undo")` (`src/core/core.cpp:102` to `:112` and `:186` to `:194`), so a `throw` from anywhere in the adapter, including the four `std::runtime_error`s above, rolls the whole document back to the last snapshot. That is why throwing from `UpdateItem` and `QueryConstraint` is not as reckless as it looks, and it is **the single most important thing not to copy** unless your host also has snapshot undo.

**And there is a trap here that is easy to miss when planning a buffered commit.** The obvious fix for a host with command-based undo, like LibrePCB, is to buffer the `AddItem` and `RemoveItem` callbacks into a command group and apply the whole thing at `Commit()`. That does not work as stated, because **the router reads back what it just wrote**. `AddItem` resolves each new segment endpoint by scanning the live board for a coincident pad and then a coincident junction, creating one only if neither is found (`src/router/pns_horizon_iface.cpp:1021` to `:1043`, using `find_pad` at `:940` and `find_junction` at `:962`), and the tool separately looks items up through `NODE::FindItemByParent` (`src/core/tools/tool_route_track_interactive.cpp:270`, `:286`). A pure write-buffer would make every one of those lookups miss. A host that wants a deferred transaction needs a shadow model that the lookups consult, or it must move endpoint resolution out of the callback and into the apply step.

**`Commit()` calls `EraseView()`** (`:1145`), whereas KiCad calls it at the top of `Commit` (`pcbnew/router/pns_kicad_iface.cpp:2916`). Same effect, different place.

### 5.7 The driving tool

`src/core/tools/tool_route_track_interactive.cpp` is Horizon's equivalent of `router_tool.cpp`, and it is roughly 1200 lines against KiCad's 3600.

Startup is worth reading for two host-side decisions. `imp->set_no_update(true)` (`:197`) suppresses the document-driven canvas rebuild that would otherwise fire after every tool event, so a shove does not trigger a full scene rebuild per frame. And `wrapper->settings.SetShoveVias(false)` (`:238`) is hardcoded and never exposed to the user, because Horizon's via-to-junction binding makes shoving vias risky.

Mode selection maps a Horizon tool ID onto `ROUTER::SetMode` at `:216` to `:232`, covering all five `ROUTER_MODE` values including the three tuning modes. The `PNS_MODE` axis is set separately in `prepareInteractive` (`:678`, `:684`, `:690`, `:696`).

The event loop is a switch over Horizon's `ToolArgs`, and the router calls it makes are exactly the same set KiCad makes:

```
begin / LMB on a start item   -> StartRouting( p0, m_startItem, routingLayer )   :288, :702
drag gesture                  -> StartDragging( p0, m_startItem, DM_ANY )        :277
mouse move                    -> Move( m_endSnapPoint, m_endItem )               :779, :915, :963, :985
LMB                           -> FixRoute( m_endSnapPoint, m_endItem )           :784, :927
  ... on true                 -> CommitRouting(); StopRouting(); commit()        :928 to :930
backspace                     -> UndoLastSegment(); Move(...)                    :1020 to :1022
via key                       -> ToggleViaPlacement(); Move(...)                 :990 to :992
layer key                     -> SwitchLayer( layer_to_router( layer ) )          :957, :1030
escape in ROUTING             -> StopRouting(); commit()                          :979 to :980
escape in drag or tune        -> revert()                                          :791, :813
```

Two things are worth extracting.

**Snapping precedes `Move`, exactly as in KiCad.** Every `Move` in the routing branch takes `wrapper->m_endSnapPoint` and `wrapper->m_endItem`, computed by the tool's own `updateEndItem` equivalent, never a raw cursor. The drag and tune branches pass the raw `args.coords` (`:798`, `:836`, `:852`), which is consistent with KiCad only because those paths do their own snapping upstream.

**Escape in route mode commits, in drag and tune mode reverts.** `:979` to `:980` calls `StopRouting()` and returns `ToolResponse::commit()`, keeping everything already committed, while `:791` and `:813` return `ToolResponse::revert()`. This mirrors KiCad's behaviour (section 4.6) from a completely different codebase, which is good evidence that "escape keeps the fixed segments" is inherent to the router's design rather than a KiCad UI quirk.

Horizon's `pickSingleItem` (`:306` onward) carries the same two sentinel bugs as KiCad's: `if (aLayer > 0) tl = aLayer;` at `:306` to `:307` means layer 0 can never be forced, and `if (aNet <= 0 || item->Net() == aNet)` at `:336` makes net code 0 a wildcard. The second is worse in Horizon because 0 is a *valid* lazily-assigned net code (section 5.5).

`ROUTER::LoadSettings` aliases rather than copies (`3rd_party/router/router/pns_router.h:196` to `:199` is `m_settings = aSettings;`), so the tool's own `settings` member is the live object the router reads, and `apply_settings()` mutates it in place. `apply_settings()` opens with `if (!router) return;` (`:101` to `:102`) because the core applies persisted settings before the tool starts (`src/core/core.cpp:63` to `:75`, before `begin()` at `:100`).

There is one dangling-pointer hazard: the tool caches a raw `meander_placer` pointer (`tool_route_track_interactive.hpp:85`) that `ROUTER::StopRouting` invalidates by resetting `m_placer` (`3rd_party/router/router/pns_router.cpp:740`), and the tool never nulls its copy. It happens to be safe only because every tune path that calls `StopRouting` returns immediately.

### 5.8 Other patches to the vendored core

Beyond the four commented-out `Parent()` downcasts in section 5.2, the compiled router carries seven more edits, all at the host boundary and none algorithmic:

- **`ROUTING_SETTINGS` decoupled from KiCad's settings framework.** `3rd_party/router/router/pns_routing_settings.h:57` reads `class ROUTING_SETTINGS //: public NESTED_SETTINGS`, with the base class commented out in place and the `PARAM` registration block deleted. Horizon serialises the tool's own struct as JSON instead (`src/core/tools/tool_route_track_interactive.cpp:77` to `:97`). **This is the right shape for a port: routing settings are a plain value struct and persistence is the host's business.**
- **`kimathLogDebug` gutted to an empty body** (`3rd_party/router/kimath/src/math/util.cpp:34` to `:37`). This is the seam through which `KiROUND` overflow is reported. A port should wire it to a real log call; a silent coordinate overflow is a correctness bug, not a diagnostic.
- **`wxGetLocalTimeMillis()` reimplemented** as a `static int64_t get_millis()` over `Glib::DateTime::create_now_utc()` (`3rd_party/router/router/time_limit.cpp:40` to `:44`). `TIME_LIMIT` gates the shove loop's budget, so this is hot-path code. In Rust it is `std::time::Instant`.
- The via-definition code field added to `PNS::VIA`, the `FindItemByParent` retype, the logger UUID stub, plus mechanical include swaps.

Two upstream structural changes that a port should follow master on rather than 6.0.4:

- **`OBSTACLE` carried a `SHAPE_LINE_CHAIN m_hull` by value** in 6.0.4 (`3rd_party/router/router/pns_node.h:105` to `:113`), with `typedef std::vector<OBSTACLE> OBSTACLES;`. Master dropped the hull and added `m_clearance`, `m_pos`, `m_maxFanoutWidth` and ordering operators so obstacles live in a `std::set` (`pcbnew/router/pns_node.h:88` to `:111`). Every collision query in 6.0.4 copies a polyline per obstacle. Do not reproduce that shape.
- **`VIA` changed base class** from `ITEM` (`3rd_party/router/router/pns_via.h:48`) to `LINKED_ITEM` (`pcbnew/router/pns_via.h:60`), so vias participate in the joint graph the way segments do.

### 5.9 What a third integrator should copy, and what to avoid

**Copy.**

The typed parent handle (section 5.2). It is one small struct and it eliminates four core patches. Make it an opaque host-defined type from the first commit, and let it carry more than one host pointer when the host's identity model needs it.

The lazy dense net interning (section 5.5), minus the zero sentinel. A `UUID` to `u32` map built on demand is exactly right; just make the zero value unusable.

Treating `ROUTING_SETTINGS` as a plain value struct owned and persisted by the host (section 5.8).

Implementing only what the engine calls. Horizon ships a working shove router with `ImportSizes` returning `true`, `StackupHeight` returning zero, `GetDebugDecorator` returning null, and two virtuals that throw. That is a real measurement of how small the required subset is.

Explicit diff-pair modelling (section 5.3). Three trivial methods against KiCad's net-name suffix parsing.

The primitive-shape fast path for pads (section 5.4). Keeping a circle a circle and a rectangle a rectangle, rather than turning every pad into a polygon, is what makes the shove interactive.

The round-trip assertion on the layer mapping: `layer_from_router` asserts `l == layer_to_router(lo)` on every call (`src/router/pns_horizon_iface.cpp:90`). Cheap, and layer-mapping bugs are otherwise silent and catastrophic.

**Do not copy.**

The commit model, unless the host has snapshot undo (section 5.6). Horizon mutates the board from inside `AddItem` and relies on exception-driven document rollback. LibrePCB needs a transaction.

Throwing from adapter methods for the same reason. `UpdateItem` throwing for anything that is not a via (`src/router/pns_horizon_iface.cpp:1220`) is a landmine in any host without whole-document rollback.

Representing areas as chains of thin locked segments (section 5.4). It scales badly and it only guards the boundary, not the interior. KiCad's triangulated non-routable solids are the better model.

Aborting the whole sync on an unexpected pad geometry (`:722`). Degrade, do not throw.

The absent clearance and hull caches (section 5.3). And note Horizon does not actually get away with it: its own rule lookup allocates and sorts, and can compile a regular expression, per collision query.

The linear scans. `get_or_create_parent` is a `std::find` over a `std::list` per synced item (`src/router/pns_horizon_iface.cpp:451`), making `SyncWorld` quadratic, and `find_pad` and `find_junction` are full-board exact-coordinate scans called twice per added segment (`:940`, `:962`, called at `:1042` to `:1043`). Use an arena with a hash index, and a spatial hash keyed on layer and position.

The unguarded null dereference in the Horizon-added `NODE::FindItemByParent`: `INDEX::GetItemsForNet` returns `nullptr` for an unknown net (`3rd_party/router/router/pns_index.cpp:71` to `:72`) and the loop dereferences it unchecked (`3rd_party/router/router/pns_node.cpp:1582` to `:1584`). Master guards the equivalent. It is reachable from the tool's drag and tune entry points.

Open-coding keepout semantics inside `collide` (section 5.2, item 1). Master's `IsKeepout` hook is the right shape.


## 6. Test infrastructure

### 6.1 What is built, and when

The whole PNS test tooling directory is opt-in. `qa/tools/CMakeLists.txt:35` guards `add_subdirectory( pns )` behind `if( KICAD_BUILD_PNS_DEBUG_TOOL )`, so nothing here is compiled in a default build.

Two binaries come out of `qa/tools/pns/CMakeLists.txt`. `pns_debug_tool` (`qa/tools/pns/CMakeLists.txt:62` to `:75`) is the interactive log viewer. `qa_pns_regressions` (`:77` to `:85`) is the headless replay harness, registered with CTest at `qa/tools/pns/CMakeLists.txt:168`.

Both share `COMMON_SRCS` (`:27` to `:60`), which is instructive on its own: it pulls in `pcbnew/net_chain_bridging.cpp`, the entire DRC engine and roughly twenty-four `drc_test_provider_*.cpp` files, `board_stackup_manager/stackup_predefined_prms.cpp`, plus `mock_pcb_tuning_pattern.cpp`. That is, **the headless regression harness links the real KiCad DRC engine**. It is not a mock. Both binaries also link full wxWidgets and GAL (`:22`, `:102` to `:147`), so "headless" here means "no window", not "no GUI toolkit".

The four unit test files are ordinary members of `QA_PCBNEW_SRCS` in `qa/tests/pcbnew/CMakeLists.txt:93`, `:139`, `:140`, `:141`, compiling into the single `qa_pcbnew` executable and running under the catch-all `qa_pcbnew_other` CTest group.

### 6.2 The pns.log format

The data model lives in `pcbnew/router/pns_logger.h`, not in the QA tree, because the producer is the live router.

`EVENT_TYPE` (`pcbnew/router/pns_logger.h:60` to `:69`): `EVT_START_ROUTE = 0`, `EVT_START_DRAG = 1`, `EVT_FIX = 2`, `EVT_MOVE = 3`, `EVT_ABORT = 4`, `EVT_TOGGLE_VIA = 5`, `EVT_UNFIX = 6`, `EVT_START_MULTIDRAG = 7`. `EVT_ABORT` is declared and never emitted anywhere in the tree.

`EVENT_ENTRY` (`:71` to `:92`) has exactly five fields: `VECTOR2I p`, `EVENT_TYPE type`, `std::vector<KIID> uuids`, `SIZES_SETTINGS sizes`, `int layer`.

`LOG_DATA` (`:94` to `:105`) is the whole file: `ROUTER_MODE m_Mode`, `std::optional<wxString> m_BoardHash`, `std::vector<ITEM*> m_AddedItems`, `std::set<KIID> m_RemovedItems`, `std::vector<ITEM*> m_Heads`, `std::vector<EVENT_ENTRY> m_Events`, `std::optional<TEST_CASE_TYPE> m_TestCaseType`.

The on-disk form is pretty-printed JSON written by `LOGGER::FormatLogFileAsJSON` (`pcbnew/router/pns_logger.cpp:107` to `:150`), with top-level keys `mode`, optional `test_case_type`, optional `board_hash`, `events`, `removedItems`, `addedItems`, `headItems`.

An event object (`pcbnew/router/pns_logger.cpp:153` to `:170`):

```json
{ "position": {"x": 111800000, "y": 93600000},
  "type": 0, "layer": 0,
  "uuids": ["ebb07c9c-...."],
  "sizes": { "trackWidth": ..., "viaDiameter": ..., "viaDrill": ...,
             "trackWidthIsExplicit": ..., "layerBottom": ..., "layerTop": ..., "viaType": ... } }
```

An item object (`pcbnew/router/pns_logger.cpp:173` to `:214`) carries `kind`, `net` (by **name**), `layers` as a two-element array, `shape`, and `drill` for vias. A shape is one of exactly three forms (`pcbnew/router/pns_logger.cpp:231` to `:271`): `{"type":"segment","width","start","end"}`, `{"type":"arc","width","start","end","mid"}`, `{"type":"circle","radius","center"}`.

**Two fields are write-only.** `LOGGER::ParseEventFromJSON` (`pcbnew/router/pns_logger.cpp:297` to `:309`) reads only `position`, `type`, `layer` and `uuids`; the `sizes` block is never read back, and at replay the sizes are recomputed from the live board through `ImportSizes` (`qa/tools/pns/pns_log_player.cpp:135`, `:160`). Similarly `PNS_LOG_FILE::loadJsonLog` (`qa/tools/pns/pns_log_file.cpp:582` to `:659`) never touches `headItems`, and the player comments "fixme: update the state with the head trace (not supported in current testsuite)" (`qa/tools/pns/pns_log_player.cpp:83`).

A legacy line-oriented grammar is still parsed as a fallback (`qa/tools/pns/pns_log_file.cpp:662` to `:714`): whitespace-tokenised lines beginning `mode`, `event`, `added` or `removed`. No file in the current corpus uses it. Note it resolves nets by **netcode**, whereas the JSON path resolves by **name** (`qa/tools/pns/pns_log_file.cpp:178`).

### 6.3 The sidecar files

`PNS_LOG_FILE::Load` (`qa/tools/pns/pns_log_file.cpp:484` to `:579`) derives everything from the log path by extension substitution:

| File | Purpose |
| --- | --- |
| `<base>.log` | the events and the golden result |
| `<base>.dump` | the board, **a `.kicad_pcb` payload under a `.dump` extension**, used only when no `board_hash` resolves |
| `<base>.kicad_pro` | project, loaded read-only through a fresh `SETTINGS_MANAGER` |
| `<base>.settings` | `PNS::ROUTING_SETTINGS` as JSON; a load failure only warns |
| `<base>.kicad_dru` | optional custom DRC rules, fed to `DRC_ENGINE::InitEngine` |

Board identity is by **content hash, not by name**. `GetLogBoardHash` (`qa/tools/pns/pns_log_file.cpp:716` to `:720`) returns the `board_hash` field, and the producer stamps it with `IO_UTILS::fileHashMMH3` over the dumped board (`pcbnew/router/router_tool.cpp:894`). The harness hashes every `*.kicad_pcb` under `pns_regressions/boards/` at startup and matches (`qa/tools/pns/qa_pns_regressions_main.cpp:166` to `:191`).

`pns.settings` is a flat JSON of `PNS::ROUTING_SETTINGS`, whose parameter list is built in `pcbnew/router/pns_routing_settings.cpp:60` to `:107`: `mode`, `effort`, `remove_loops`, `smart_pads`, `shove_vias`, `suggest_finish`, `follow_mouse`, `start_diagonal`, `shove_iteration_limit`, `via_force_prop_iteration_limit`, `shove_time_limit`, `walkaround_iteration_limit`, `jump_over_obstacles`, `smooth_dragged_segments`, `can_violate_drc`, `free_angle_mode`, `snap_to_tracks`, `snap_to_pads`, `optimize_dragged_track`, `auto_posture`, `fix_all_segments`, `restrict_angles`, `corner_mode`, `walkaround_hug_length_threshold`. Beware two enums both spelled "mode": `PNS_MODE` in the settings (`RM_MarkObstacles = 0`, `RM_Shove = 1`, `RM_Walkaround = 2`, `pcbnew/router/pns_routing_settings.h:39` to `:44`) and `ROUTER_MODE` in the log (`PNS_MODE_ROUTE_SINGLE = 1` and up, `pcbnew/router/pns_router.h:67` to `:73`).

There is **no view or layer state** in a log beyond the per-event PNS layer index. `SetVisibleViewArea` is set from the GAL in the live tool (`pcbnew/router/router_tool.cpp:924`) and never recorded, so replay runs with the constructor default `m_visibleViewArea.SetMaximum()` (`pcbnew/router/pns_router.cpp:78`).

### 6.4 How a user produces a log

Two gates, both in advanced config. `ROUTER::ROUTER()` allocates the logger only when `ADVANCED_CFG::GetCfg().m_EnableRouterDump` is set (`pcbnew/router/pns_router.cpp:67` to `:70`), and `ROUTER_TOOL::handleCommonEvents` binds the `0` key to `saveRouterDebugLog()` behind the same flag (`pcbnew/router/router_tool.cpp:928` to `:939`). A second flag, `m_RouterTestCaseDirectory` (`pcbnew/router/router_tool.cpp:760`), switches the save path from a file dialog to a directory-plus-name dialog that also asks for a `TEST_CASE_TYPE` and names everything `pns.*`.

Every logging call is in `ROUTER`, none deeper. The complete list is `pcbnew/router/pns_router.cpp:198` (clear), `:204` (`EVT_START_DRAG`), `:206` (`EVT_START_MULTIDRAG`), `:478` (clear) and `:479` (`EVT_START_ROUTE`, the only event carrying a layer and sizes), `:497` (`EVT_MOVE`), `:920` (`EVT_FIX`), `:952` (`EVT_UNFIX`), `:1027` (`EVT_TOGGLE_VIA`). `SetLogger` is propagated into `SHOVE` and `WALKAROUND` but those classes never log; the pointer only travels so it can be forwarded further.

`saveRouterDebugLog` (`pcbnew/router/router_tool.cpp:758` to `:917`) writes the settings, saves the board with `PCB_IO_KICAD_SEXPR::SaveBoard` under a `.dump` extension, saves the project and local settings, copies the `.kicad_dru` if present, then calls `ROUTER::GetUpdatedItems` to capture the golden added and removed sets, hashes the board dump, and writes the log.

### 6.5 Replay

`PNS_LOG_PLAYER_KICAD_IFACE` (`qa/tools/pns/pns_log_player.h:71` to `:86`) derives from `PNS_KICAD_IFACE_BASE` and overrides exactly four methods (`qa/tools/pns/pns_log_player.cpp:336` to `:365`): `HideItem` and `DisplayItem` route into a `PNS_LOG_VIEW_TRACKER`, and `GetNetCode` and `GetNetName` unwrap the `NETINFO_ITEM`. **Everything else is production code**, including `SyncWorld`, all four `sync*` helpers, `ImportSizes`, the layer mapping, and crucially `GetRuleResolver()` returning the real `PNS_PCBNEW_RULE_RESOLVER` driven by a real `DRC_ENGINE`.

That is the single most important fact about this corpus for a Rust port: **the regressions exercise the real KiCad clearance resolver against a real board, headlessly, with no GUI and no mocks.**

`createRouter` (`qa/tools/pns/pns_log_player.cpp:42` to `:60`) does the obvious wiring, then `ReplayLog` (`:91` to `:317`) immediately replaces the settings with the parsed ones (`:98`).

The replay loop per event (`:105` to `:237`):

```
items        = log->ItemsById( evt )              // KIID -> BOARD_CONNECTED_ITEM*, linear scan
ritem        = world->FindItemByParent( items[0] )// BOARD_ITEM* -> PNS::ITEM*
routingLayer = ritem ? ritem->Layers().Start() : evt.layer
dispatch:
  EVT_START_ROUTE      -> SetStartLayerFromPNS; ImportSizes; UpdateSizes; StartRouting( p, ritem, layer )
  EVT_START_DRAG       -> same prologue;         StartDragging( p, ritems, 0 )
  EVT_START_MULTIDRAG  -> same
  EVT_FIX              -> FixRoute( p, ritem, false, false )
  EVT_UNFIX            -> UndoLastSegment()
  EVT_MOVE             -> Move( p, ritem )
  EVT_TOGGLE_VIA       -> ToggleViaPlacement()
  default              -> no-op
```

Note that the drag mode is hard-coded to `0` at `qa/tools/pns/pns_log_player.cpp:169`, not to any `DRAG_MODE` enumerator, because the original mode was never logged.

`GetRouterUpdatedItems` (`:63` to `:89`) calls `ROUTER::GetUpdatedItems` (`pcbnew/router/pns_router.cpp:833` to `:859`) and builds a `COMMIT_STATE` of removed `KIID`s and added `ITEM*`s. Head items are cloned by `GetUpdatedItems` and deleted immediately by the player (`:85` to `:86`).

### 6.6 What the regression harness actually asserts

`qa/tools/pns/qa_pns_regressions_main.cpp`. Discovery (`:181` to `:213`): hash every `*.kicad_pcb` under `pns_regressions/boards/`, then for every sub-directory other than `boards` and every `*.log` in it, register a Boost test case named after the **directory**.

The assertion is `PNS_TEST_FIXTURE::RunTest` (`:81` to `:135`), and it is exactly one thing:

```cpp
player.ReplayLog( &logFile, 0 );
auto cstate   = player.GetRouterUpdatedItems();
auto expected = logFile.GetExpectedResult();
bool pass     = cstate.Compare( expected );
BOOST_REQUIRE( pass );
```

That is: **the set of added and removed items after replaying the events must match the `addedItems` and `removedItems` arrays stored inside the same `pns.log`.** There is no DRC run, no golden geometry file, no router-end-state check. `test_case_type` is round-tripped but never acted on.

`COMMIT_STATE::Compare` (`qa/tools/pns/pns_log_file.cpp:411` to `:448`) is set equality on removed IDs and multiset equality on added items after a dedup pass, using `comparePnsItems` (`:320` to `:383`), which compares kind, net, layers, and then per kind: via diameter, drill and position; segment seg and width; hole radius and centre. **There is no `ARC_T` branch**, so two arcs on the same net and layers compare equal regardless of geometry.

Three traps for anyone reusing this corpus as ground truth:

1. **A missing or hash-mismatched board makes the test pass.** `qa/tools/pns/qa_pns_regressions_main.cpp:101` to `:108` reports "Failed to load test, will skip", calls `BOOST_CHECK( true )` and returns. Green does not mean "ran".
2. The golden is regenerated in place by `qa_pns_regressions --update-golden` (`:112` to `:122`, `:241`), which replays with `aUpdateExpectedResult = true` and calls `SaveLog`. So the golden always encodes whatever the code did on the day it was refreshed.
3. `sizes` in the log is decorative and `headItems` is never read, so neither is part of the contract.

### 6.7 The debug decorator and the viewer

`PNS_TEST_DEBUG_DECORATOR` (`qa/tools/pns/pns_test_debug_decorator.h`, `.cpp`) retains everything in memory as a per-stage tree of `PNS_DEBUG_SHAPE` nodes, each carrying owned cloned shapes, a colour, a width, an iteration number, a message and a `SRC_LOCATION_INFO` recording the file, function and line of the originating `PNS_DBG` call (`qa/tools/pns/pns_test_debug_decorator.cpp:174` to `:266`). `SetDebugEnabled( true )` in the constructor (`:116`) means replay is far more instrumented than a live session.

`LABEL_MANAGER` (`qa/tools/pns/label_manager.cpp`) is purely a viewer helper that draws text labels with leader lines; its collision-avoidance pass is disabled by an unconditional `return` at `:163` to `:165`. It captures nothing about the router.

`PNS_LOG_VIEWER_FRAME` (`qa/tools/pns/pns_log_viewer_frame.cpp`) is the interactive tool: a rewind slider over debug stages, a filter box, an OK/FAIL status from `PNS_DEBUG_STAGE::m_status`, a tree list whose columns are `Type, Value, File, Method, Line, VCount, Non-45` (`:186` to `:192`), a "Go to line in IDE" context action that shells out to `code --goto`, `devenv`, `clion` or `emacsclient` (`:670` to `:677`), and a "Save As" that pops a four-way `TEST_CASE_TYPE` chooser and writes a new regression case (`:432` to `:466`). Note that `SetLogFile` always replays with `aUpdateExpectedResult = true` (`:365` to `:397`), so saving from the viewer rewrites the golden from the current code's behaviour.

`PNS_VIEWER_IFACE` (`qa/tools/pns/pns_log_viewer_frame.h:50` to `:229`) is a **third** `ROUTER_IFACE` implementation, almost entirely stubbed, existing only so `ROUTER_PREVIEW_ITEM` can render. Its `StackupHeight` returns 0 (`qa/tools/pns/pns_log_viewer_frame.h:74`), which is more evidence that the method is dead.

`playground.cpp` is a geometry scratchpad, not a test: it renders twenty-eight hard-coded arcs annotated with the bug each reproduces (`qa/tools/pns/playground.cpp:213` to `:243`). `mock_pcb_tuning_pattern.cpp` is a link-time stub, not a test double: it supplies empty definitions for the whole `PCB_TUNING_PATTERN` API so boards containing tuning-pattern generators can load without dragging in the tool framework (`qa/tools/pns/mock_pcb_tuning_pattern.cpp:194` to `:381`).

### 6.8 The unit tests

`qa/tests/pcbnew/test_pns_basics.cpp` is the only file with mocks, and they are worth studying because they are a working reference for the minimal host.

`MOCK_RULE_RESOLVER` (`qa/tests/pcbnew/test_pns_basics.cpp:91` to `:323`) is roughly 230 lines and reimplements the whole clearance policy by hand: the layer-range intersection with the edge-item special case (`:107` to `:114`), the per-layer walk over `CT_HOLE_TO_HOLE`, `CT_HOLE_CLEARANCE`, `CT_CLEARANCE`, `CT_EDGE_CLEARANCE`, `CT_PHYSICAL_HOLE_CLEARANCE` and `CT_PHYSICAL_CLEARANCE` (`:100` to `:182`), and the `-1` short circuit for same-net and free-pad pairs. `QueryConstraint` (`:205` to `:251`) is a lookup in a map populated by `AddMockRule()`, falling back to tunable defaults: `m_defaultClearance = 200000`, `m_defaultHole2Hole = 220000`, `m_defaultHole2Copper = 210000`, and zero for the two physical constraints. Every diff-pair, net-tie and keepout method is inert.

`MOCK_PNS_KICAD_IFACE` (`:327` to `:352`) derives from `PNS_KICAD_IFACE_BASE` and overrides only three things: `HideItem` and `DisplayItem` become no-ops, and `GetRuleResolver()` returns the mock. It adds two test hatches exposing the protected `inheritTrackWidth` and `syncVia`.

`PNS_TEST_FIXTURE` (`:355` to `:375`) is four members, and the commented-out `std::unique_ptr<BOARD> m_board` at `:374` documents the intent: most cases need no board at all. The dominant pattern is to hand-build a `PNS::NODE`:

```cpp
std::unique_ptr<PNS::NODE> world( new PNS::NODE );
world->SetMaxClearance( 10000000 );
world->SetRuleResolver( &m_ruleResolver );
world->AddRaw( new PNS::VIA( ... ) );
world->QueryColliding( item, obstacles );
```

There is **no shared `pns_test_fixture.h`**; the fixture is local to this file.

Cases worth knowing about: `PNSHoleCollisions` (`:508`) is the canonical executable specification of the copper, hole and hole-to-hole collision matrix; `PNSLayerRangeSwapBehavior` (`:686`) pins the range-swapping behaviour discussed in section 1.7; `PNSInheritTrackWidthCursorProximity` (`:764`) pins the far-end heuristic from section 1.9; the eight physical-clearance cases at `:1257` to `:1468` pin the net-blind physical rule behaviour; and `PNSMarkViolationsKeepsPadstackViaShape` (`:1555`) is the only case that really drives the router end to end.

The other three files use `PNS_KICAD_IFACE_BASE` **directly**, with no mock at all, against a board loaded from disk:

```cpp
KI_TEST::LoadBoard( m_settingsManager, "<name>", m_board );
PNS::ROUTER          router;
PNS_KICAD_IFACE_BASE iface;
iface.SetBoard( m_board.get() );
router.SetInterface( &iface );
router.ClearWorld();
router.SyncWorld();
PNS::ITEM* startItem = router.GetWorld()->FindItemByParent( someBoardItem );
```

(`qa/tests/pcbnew/test_pns_tuning_path_through_pad.cpp:96` to `:104`; `qa/tests/pcbnew/test_pns_diff_pair_tuning_width.cpp:70` to `:78`; `qa/tests/pcbnew/test_pns_via_layer_span.cpp:68` to `:91`.) The fixtures are bare data holders: a `SETTINGS_MANAGER` and a `std::unique_ptr<BOARD>`.

`test_pns_diff_pair_tuning_width.cpp` asserts that a diff-pair tuning path stops at a track width change and that a through via in a diff pair spans the full stack. `test_pns_tuning_path_through_pad.cpp` asserts that `TOPOLOGY::AssembleTuningPath` walks through an in-line pad rather than terminating at it. `test_pns_via_layer_span.cpp` asserts that a through via spans the whole board regardless of the layer pair, and that every synced via has `HoleLayers() == Layers()`.

### 6.9 The regression corpus

Twelve directories under `qa/data/pcbnew/pns_regressions/`. `boards/` is not a case; it is the shared board pool, skipped by name at `qa/tools/pns/qa_pns_regressions_main.cpp:195` and hashed into a lookup table at `:191`. It holds ten `.kicad_pcb` files ranging from a 10 KB two-net test board to a 5.9 MB, 588-net production board.

The eleven actual cases, one line each:

| Case | Router mode | Events | Golden | Covers |
| --- | --- | --- | --- | --- |
| `backspace1` | Shove | 334: 1 start, 314 move, 10 fix, **9 unfix** | 2 segments added | The only case exercising `UndoLastSegment`, that is, backspace during routing |
| `drag-acute-fallback` | Walkaround, `restrict_angles` | 858: 1 start-drag, 857 move | 11 added, 6 removed | Fallback when a drag would produce an acute corner; the longest log in the corpus |
| `drag-walk-optimize-a` | Walkaround, `restrict_angles` | 50: 1 start-drag, 49 move | 9 added, 6 removed | Post-drag walkaround optimiser behaviour |
| `drag-walk-optimize-fix-corners` | Walkaround, `restrict_angles` | 83: 1 start-drag, 82 move | 9 added, 3 removed | The corner-fixing variant of the above |
| `issue22749-shove-weird-drag-track-end` | Shove | 54: 1 start-route on layer 1, 53 move | 11 added, 5 removed | GitLab issue 22749: shove producing a malformed track end |
| `issue23449-shove-lone-via-drag-crash` | Shove | 103: 1 start-drag, 102 move | **empty** | GitLab issue 23449: crash dragging an isolated via; asserts only "did not crash and committed nothing" |
| `issue24132-shove-same-net-via` | Shove | 344: 1 start-route, 342 move, 1 fix | 4 added (segment, via, hole), 1 removed | GitLab issue 24132; the **only** case shipping a `.kicad_dru`, a `physical_clearance` rule of 2 mm between tracks and vias, and the only golden containing a via and a hole |
| `simple-drag-shove-singlelayer` | Shove | 35: 1 start-drag, 34 move | **134 added, 139 removed** across ten nets | A heavy single-layer shove cascade; by far the largest golden |
| `simple-shove-1` | Shove, `optimize_dragged_track`, `smart_pads` off | 27: 1 start-route with an **empty uuids array**, 26 move | 28 added, 13 removed | Routing started in free space, so the replay falls back to the logged layer |
| `walk_drag_seg_against_board_edge` | Shove despite the name | 18: 1 start-drag, 17 move | 3 added, 3 removed | Dragging a segment into the board outline |
| `walk-with-teardrops` | Walkaround | 18: 1 start-drag, 17 move | 17 added, 19 removed | Walkaround around teardrop pads with hugging disabled; the **only self-contained case**, shipping its own board as `pns-no-hug-2.dump` with no `board_hash` |

Corpus-level gaps worth noting. Every one of the eleven logs has `ROUTER_MODE = PNS_MODE_ROUTE_SINGLE`, so nothing exercises diff-pair or tuning replay. No case contains `EVT_TOGGLE_VIA`, `EVT_START_MULTIDRAG` or `EVT_ABORT`, so the multi-dragger and via-toggle replay branches are dead in regression runs. Two cases ship no `.kicad_pro`, and no case ships a `.kicad_prl` even though the loader reads `m_AutoTrackWidth` from it (`qa/tools/pns/pns_log_file.cpp:543`).

### 6.10 Reuse as test vectors for a Rust port

Ranked by value.

**1. Replay the same event logs and compare committed geometry.** This is the highest-value reuse and it is directly feasible. A `pns.log` is a five-field event stream, a `pns.settings` is a flat JSON of twenty-four knobs, and the golden is in the same file. A Rust harness needs: a `.kicad_pcb` reader good enough to build a `WorldSnapshot` (or, better, a one-off C++ tool that dumps each regression board as a `WorldSnapshot` JSON so the Rust side never has to parse `.kicad_pcb`); a JSON reader for the log; a mapping from `KIID` to `HostId`; and a comparison over the added and removed sets. Because the golden is stored as geometry plus net name plus layer range, not as UUIDs of new items, it is portable across implementations.

The honest caveat is that this comparison is exact geometry. Two shove implementations that both produce DRC-clean, topologically equivalent results will disagree on the exact vertex coordinates. So the practical plan is to run the corpus in three tiers: (a) does not crash and terminates, on all eleven; (b) commits a DRC-clean result, on all eleven, checked with LibrePCB's own DRC rather than against the golden; (c) exact geometry match, aspirationally, and treated as a diagnostic rather than a gate.

The `issue23449` case is a free win at tier (a), since its golden is empty and it is purely a crash regression.

**2. Port `MOCK_RULE_RESOLVER` and the collision unit tests.** `qa/tests/pcbnew/test_pns_basics.cpp:91` to `:323` plus `PNSHoleCollisions` (`:508`) and the eight physical-clearance cases (`:1257` to `:1468`) need no board and no GUI. They are an executable specification of the clearance matrix, they are the part of the contract a LibrePCB host must get exactly right, and they translate to Rust almost line for line. Do these first, before any replay work.

**3. Port the invariant tests.** `PNSLayerRangeSwapBehavior` (`:686`), `PNSInheritTrackWidthCursorProximity` (`:764`), `PNSSegmentSplitPreservesLockedState` (`:720`), and the via-layer-span assertions in `qa/tests/pcbnew/test_pns_via_layer_span.cpp:135` and `:176`. Each pins one small behaviour that is easy to get wrong and cheap to test.

**4. Build the equivalent of the log format from day one.** The single best thing KiCad's PNS did for its own maintainability is that a user can reproduce any router bug by pressing one key. A Rust port should define its event log as the *primary* input to the engine (a `Vec<Event>` plus a `WorldSnapshot` plus a `Settings`), so that the interactive tool is a thin producer of that stream and the test harness is a thin consumer of it. That inverts KiCad's arrangement, where logging is bolted onto a live-callback engine, and it makes every session trivially replayable.

**5. Do not copy the harness's failure modes.** Make a missing board a hard error, not a skip. Include arcs in the comparison. Do not store the golden inside the input file, because regenerating it then silently rewrites the test.


## 7. Rust and LibrePCB mapping notes

### 7.1 What the C++ contract actually is, stripped of C++

Read as a specification rather than as C++, the host contract has four distinct responsibilities that happen to be bundled into two classes. Separating them is the single biggest structural improvement available to a Rust port.

1. **World supply.** A one-shot description of every obstacle, its geometry, layer span, net, hole, and mutability. Pure data. Currently `SyncWorld` plus the nine `sync*` helpers.
2. **Rule oracle.** Answer "what clearance applies between these two things on these layers" and a handful of classification predicates. Pure function of the world plus the design rules, called millions of times per session. Currently `RULE_RESOLVER`.
3. **Write-back.** Turn the router's final geometry into host edits under one undo entry. Currently `AddItem`, `UpdateItem`, `RemoveItem`, `Commit`.
4. **View decoration.** Draw a transient preview. Currently `Display*`, `HideItem`, `EraseView`.

Responsibilities 1 and 4 are the ones KiCad has bundled awkwardly. World supply is push-style callbacks into a partially built graph, which is why the base interface has to exist at all and why the world cannot be built off the UI thread. View decoration is a callback firing from inside the geometry kernel, which is why the router depends on a global singleton (`pcbnew/router/pns_item.cpp:172`, `pcbnew/router/pns_via.cpp:243`, `pcbnew/router/pns_optimizer.cpp:858`, `pcbnew/router/pns_node.cpp:301`, and five more) and why `pcbnew/router/pns_item.cpp:171` carries the comment "fixme: this f***ing singleton must go...".

### 7.2 Proposed Rust shape

**Plain data crossing into the engine: a world snapshot.** Replace `SyncWorld` with a value the host builds and hands over. This gets the bulk-add semantics for free, makes the world constructible off the UI thread, makes it trivially serialisable for the regression harness in section 6, and removes an entire class of "the host called Add after FinalizeBulkAdd" bug.

```rust
pub struct WorldSnapshot {
    pub copper_layer_count: u8,
    pub max_clearance: i32,           // see 1.2; understating this is silent corruption
    pub items: Vec<WorldItem>,
    pub edge_exclusions: Vec<Shape>,  // castellated pad holes, pns_kicad_iface.cpp:2373
}

pub struct WorldItem {
    pub id: HostId,                   // opaque, stable, host-owned
    pub kind: ItemKind,               // Segment | Arc | Via | Solid
    pub net: Option<NetId>,
    pub layers: LayerRange,           // closed interval over dense copper indices
    pub geometry: Geometry,
    pub hole: Option<Hole>,           // separate object; see 3.3
    pub flags: ItemFlags,             // LOCKED | NOT_ROUTABLE | FREE_PAD | COMPOUND_PRIMITIVE
    pub flashed_layers: LayerMask,    // materialises IsFlashedOnLayer; see 7.3
}
```

`HostId` should be a generational index, not a pointer. KiCad's clearance cache is keyed by raw `PNS::ITEM*` (`pcbnew/router/pns_kicad_iface.cpp:92`) and needs explicit invalidation on every node update (`pcbnew/router/pns_router.cpp:766`) precisely to avoid address reuse aliasing a stale entry. A generational key removes the hazard by construction.

**Callbacks that must stay callbacks: the rule oracle.** `Clearance` cannot be precomputed because the router asks about items that do not exist yet, at layers that vary, in combinations that are quadratic in the world size. It stays a trait.

```rust
pub trait RuleResolver {
    /// Worst-case clearance across the layers the two items share.
    /// Returns None when no rule applies (KiCad's -1).
    fn clearance(&self, a: ItemRef<'_>, b: Option<ItemRef<'_>>, use_epsilon: bool) -> Option<i32>;

    fn clearance_epsilon(&self) -> i32 { 0 }

    /// Only the constraint types the host actually implements; None otherwise.
    fn constraint(&self, ty: ConstraintType, a: ItemRef<'_>, b: Option<ItemRef<'_>>,
                  layer: LayerIndex) -> Option<Constraint>;

    fn is_keepout(&self, obstacle: ItemRef<'_>, item: ItemRef<'_>) -> Keepout;  // None | Present | Enforced
    fn is_drilled_hole(&self, item: ItemRef<'_>) -> bool;
    fn is_non_plated_slot(&self, item: ItemRef<'_>) -> bool;

    fn net_id(&self, net: NetId) -> i32;
    fn net_name(&self, net: NetId) -> &str { "" }

    // Diff pair, all defaulted so a v1 host implements none of them.
    fn dp_coupled_net(&self, _net: NetId) -> Option<NetId> { None }
    fn dp_net_polarity(&self, _net: NetId) -> i32 { 0 }
    fn dp_net_pair(&self, _item: ItemRef<'_>) -> Option<(NetId, NetId)> { None }

    // Net ties, defaulted off.
    fn is_in_net_tie(&self, _item: ItemRef<'_>) -> bool { false }
    fn is_net_tie_exclusion(&self, _item: ItemRef<'_>, _at: Point, _other: ItemRef<'_>) -> bool { false }
    fn has_user_physical_constraint(&self) -> bool { false }
}
```

Two deliberate changes from the C++. `IsKeepout`'s bool-plus-out-parameter becomes a three-state enum, because the C++ pair of values only has three meaningful combinations and the fourth (`false` return with `*aEnforce` written) is a latent bug waiting to happen given the short-circuiting `||` at `pcbnew/router/pns_item.cpp:198`. And `Clearance` returns `Option<i32>` rather than encoding "no rule" as `-1`, which removes the need for every caller to remember the sentinel.

The caches belong to the engine, not the host. `HullCache`, `m_clearanceCache` and `m_tempClearanceCache` are all pure memoisation of a pure function; in Rust they should live in the engine behind the trait, so hosts cannot get invalidation wrong. That also deletes `ClearCaches`, `ClearCacheForItems` and `ClearTemporaryCaches` from the host contract entirely, which are three of the four methods most likely to be forgotten by an integrator.

**Plain data crossing out of the engine: the commit diff.** Replace `AddItem`/`UpdateItem`/`RemoveItem`/`Commit` with a value the engine returns and the host applies.

```rust
pub struct CommitDiff {
    pub removed: Vec<HostId>,
    pub added: Vec<NewItem>,                 // no HostId yet; host assigns
    pub updated: Vec<(HostId, NewItem)>,     // identity preserved; see 1.3
    pub moved_pads: Vec<(HostId, Point)>,    // component dragger; see pns_kicad_iface.cpp:2918
}
```

The remove-plus-add-with-same-identity fold at `pcbnew/router/pns_router.cpp:877` to `:892` moves into the engine, where it belongs, and the host just applies three lists inside one undo transaction. `NewItem` should carry a provenance field mirroring `GetSourceItem()` (`pcbnew/router/pns_item.h:202`), because KiCad uses it to inherit solder-mask settings (`pcbnew/router/pns_kicad_iface.cpp:2775` to `:2780`, `:2797` to `:2802`), via tenting (`:2838` to `:2843`) and group membership (`:2875` to `:2891`). LibrePCB will want the same for whatever per-trace attributes it grows.

**Callbacks that must stay callbacks: view decoration.** These fire from deep inside `movePlacing` and `updateView` on every mouse move and there is no way to hoist them out without buffering the whole preview, which is in fact exactly what a Rust port should do:

```rust
pub struct PreviewFrame {
    pub items: Vec<PreviewItem>,       // DisplayItem, with clearance and flags
    pub path_lines: Vec<(LineChain, i32)>,   // DisplayPathLine
    pub ratlines: Vec<(LineChain, NetId)>,   // DisplayRatline
    pub hidden: Vec<HostId>,           // HideItem
}
```

`ROUTER::Move` returns a fresh `PreviewFrame` and the host swaps it in wholesale. This matches the existing semantics exactly, because `EraseView` already clears everything at the top of every move (`pcbnew/router/pns_router.cpp:658`, `:791`), so the protocol is already whole-frame replacement pretending to be incremental. The gain is that the engine becomes a pure function of (world, settings, event) and is trivially testable and replayable.

### 7.3 Flashing: the one predicate that must not be a stub

`IsFlashedOnLayer` is the item in the host contract most likely to be dismissed as cosmetic and most damaging to get wrong. It has three distinct effects:

- It suppresses collisions entirely, `clearance = -1`, at `pcbnew/router/pns_item.cpp:206` and `:210`.
- It shrinks a via's *walkaround hull* to the hole diameter when the via is not flashed on the layer being walked around, `pcbnew/router/pns_via.cpp:243` to `:244`. So a non-flashed via is not merely "not an obstacle", it is an obstacle of a different size, and the walkaround geometry differs.
- It is what makes `syncPad`'s "always flashed geometry, decide per layer later" design work at all (`pcbnew/router/pns_kicad_iface.cpp:1716` to `:1718`).

Because it is a pure function of the item and a layer, it should not be a callback in a Rust port. Materialise it as the `flashed_layers: LayerMask` field on `WorldItem` and have the engine consult the mask. For LibrePCB v1 the mask is simply the item's layer span; for a later HDI story it is the span minus the layers where the annular ring is removed.

### 7.4 Coordinates and units

KiCad's `VECTOR2I` is `VECTOR2<int32_t>` (`libs/kimath/include/math/vector2d.h:683`) in nanometres, so the whole router works in signed 32-bit nanometres, about plus or minus 2.15 metres. LibrePCB's `Length` is `int64_t` nanometres (`libs/librepcb/core/types/length.h:82`). The conversion is lossless for any realistic board but the Rust port should either adopt `i32` and validate on ingest, or adopt `i64` throughout and accept that some geometry kernels will need rework. Adopting `i32` and rejecting out-of-range input at the snapshot boundary is the lower-risk choice, since every hull, octagon and clearance computation in PNS assumes intermediate products fit in `ecoord` (`VECTOR2I::extended_type`, aliased at `pcbnew/router/pns_kicad_iface.cpp:87`).

### 7.5 Mapping LibrePCB's rule model

LibrePCB's design rules, as they exist today:

- Per net class: `minCopperCopperClearance`, `minCopperWidth`, `minViaDrillDiameter`, plus optional `defaultTraceWidth` and `defaultViaDrill` (`libs/librepcb/core/project/circuit/netclass.h:63` to `:77`).
- Board DRC settings: `minCopperCopperClearance`, `minCopperBoardClearance`, `minCopperNpthClearance`, `minDrillDrillClearance`, `minDrillBoardClearance`, `minCopperWidth`, `minPthAnnularRing` (`libs/librepcb/core/project/board/drc/boarddesignrulechecksettings.h:112` to `:135`).
- Board design rules: `defaultTraceWidth`, `defaultViaDrillDiameter` (`libs/librepcb/core/project/board/boarddesignrules.h:57`, `:60`).
- Per pad: `copperClearance` (`libs/librepcb/core/geometry/pad.h:112`).
- Combination rule, already implemented for DRC: netclass value combined with board setting by `std::max`, per `BoardDesignRuleCheckData::getMinCopperCopperClearance` and its siblings in `libs/librepcb/core/project/board/drc/boarddesignrulecheckdata.h`.

The resolver is therefore a small table lookup, not a rule engine:

```
clearance(a, b) =
    let mut result = None
    if a and b are copper, different nets, neither is a free pad:
        result = max(result, max over both sides of
                     max(board.minCopperCopperClearance,
                         netclass(side).minCopperCopperClearance,
                         pad_override(side)))
    if either is a board outline item:
        result = max(result, board.minCopperBoardClearance)     // and minDrillBoardClearance if hole
    if either is a non-plated hole and nets differ:
        result = max(result, board.minCopperNpthClearance)
    if both are holes:
        result = max(result, board.minDrillDrillClearance)
    result
```

`QueryConstraint` then answers `CT_CLEARANCE` from the same table, `CT_WIDTH` from `max(board.minCopperWidth, netclass.minCopperWidth)` with `Opt` set to `netclass.defaultTraceWidth` or `boardDesignRules.defaultTraceWidth`, `CT_VIA_HOLE` from `max(board.minPthDrillDiameter, netclass.minViaDrillDiameter)` with `Opt` from `defaultViaDrill`, `CT_VIA_DIAMETER` from the annular ring rules, `CT_EDGE_CLEARANCE` from `minCopperBoardClearance`, `CT_HOLE_CLEARANCE` from `minCopperNpthClearance`, `CT_HOLE_TO_HOLE` from `minDrillDrillClearance`, and `None` for everything else. That is seven of the thirteen constraint types; the other six are diff-pair, tuning and physical-clearance concepts LibrePCB does not have.

`ClearanceEpsilon` should be zero for LibrePCB, since there is no DRC epsilon in the model. Note that this makes the router marginally stricter than KiCad, which is the safe direction.

### 7.6 Mapping LibrePCB's board objects into the snapshot

| LibrePCB object | PNS item | Notes |
| --- | --- | --- |
| `BI_NetLine` (`libs/librepcb/core/project/board/items/bi_netline.h`) | `Segment` | Straight only; LibrePCB has no arc traces today, so the `Arc` kind is initially unused but must exist for the router's own output |
| `BI_Via` (`bi_via.h`) | `Via` plus a separate `Hole` | Copper span from start and end layer; hole span likewise. Model the hole as its own item, per section 3.3 |
| `BI_Pad` (`bi_pad.h`) | one or more `Solid`, plus a `Hole` if drilled | Non-routable if the pad is a non-plated hole. Carry the per-pad copper clearance override into `max_clearance` |
| `BI_Hole` (`bi_hole.h`) | `Solid` marked non-routable, plus a `Hole` | Board-level mechanical holes |
| Board outline polygon | `Solid` spanning every copper layer, non-routable, zero width | Mirrors `pcbnew/router/pns_kicad_iface.cpp:2059` to `:2079`. Clearance comes from the edge rule, not from the shape width |
| `BI_Polygon` on a copper layer (`bi_polygon.h`) | `Solid` on that layer | Routable, carries a net if it has one |
| `BI_StrokeText` on a copper layer (`bi_stroketext.h`) | `Solid`, null net, non-routable | KiCad takes only outline 0 (`pcbnew/router/pns_kicad_iface.cpp:1984`); LibrePCB stroke text is already a set of strokes, so emit one solid per stroke and mark them compound |
| `BI_Zone` with a "no copper" rule (`bi_zone.h`) | triangulated `Solid`s, null net, non-routable, compound | Only if `IsKeepout` is implemented in v1; otherwise skip |
| `BI_Plane` (`bi_plane.h`) | **not synced** | Matches KiCad exactly (section 3.5). Rebuild plane fragments after the commit |
| `BI_AirWire` (`bi_airwire.h`) | not synced | The router computes its own ratlines |
| `BI_Device` | not synced directly | Only its pads |
| Anything on a non-copper, non-outline layer | not synced | Silkscreen, mask, courtyard, documentation |

Nets map to `NetSignal` identity. `NetId` must have a distinguished "orphan" value whose `net_id()` is `<= 0`, because the sentinel convention at `pcbnew/router/pns_tool_base.cpp:167` and `pcbnew/router/pns_line_placer.cpp:1571` is how the engine recognises an unnamed, freshly started track.

### 7.7 Minimal host obligations for LibrePCB's first integration

In dependency order, the shortest path to a working single-track shove router:

1. **A snapshot builder** that walks a `Board` and emits `WorldSnapshot`. Must include: traces, vias with holes, pads with holes and per-layer shapes, board outline as a zero-width all-copper-layer non-routable solid, mechanical holes. Must exclude planes. Must compute `max_clearance` as the maximum over the board rule, every net class rule, and every per-pad override, and must not understate it.
2. **A dense copper layer index** and the two conversions to and from `Layer*`. This is a pure function of the board's copper layer list.
3. **A rule resolver** implementing `clearance` and `constraint` for the seven applicable constraint types, and `is_drilled_hole` / `is_non_plated_slot`. Everything else takes the trait default.
4. **A flashed-layer mask** on every pad and via. For v1 this equals the layer span.
5. **A commit applier** that takes a `CommitDiff` and applies it inside one `UndoCommandGroup`, preserving `BI_NetLine` and `BI_Via` identity for `updated` entries. Note the read-back constraint from section 5.6: if endpoint resolution against existing pads and junctions happens inside the engine's write path, a purely deferred command group will not see its own earlier writes. Resolve endpoints in the apply step, or keep a shadow index the engine can query. LibrePCB's net segment model means a new trace also needs net points and possibly a net segment split or merge, so this is the piece most likely to be underestimated: the engine emits free-floating segments and LibrePCB requires them stitched into a `BI_NetSegment` with junctions. The existing building blocks are `libs/librepcb/editor/project/cmd/cmdboardnetsegmentaddelements.h`, `cmdboardnetsegmentremoveelements.h`, `cmdboardsplitnetline.h` and `cmdcombineboardnetsegments.h`.
6. **A preview renderer** that consumes a `PreviewFrame`: polylines with a width, circles for vias, an outline offset by a clearance, dashed ratlines, and four flag-driven styles (head trace, hover, semi-solid, collision).
7. **A sizes provider** equivalent to `ImportSizes`: track width, via diameter, via drill, board minimum clearance and minimum track width. This lives in the tool, not the engine.

Deliberately out of scope for v1: diff pairs, all three tuning modes, net ties, keepout zones, custom padstacks, blind and buried vias beyond a simple layer pair, and every `Calculate*` hook.

### 7.8 Things to design differently from KiCad

- **No global singleton.** Nine call sites reach the host through `ROUTER::GetInstance()` (`pcbnew/router/pns_item.cpp:172`, `:340`; `pcbnew/router/pns_via.cpp:150`, `:243`; `pcbnew/router/pns_optimizer.cpp:858`, `:1278`; `pcbnew/router/pns_node.cpp:301`; `pcbnew/router/pns_mouse_trail_tracer.cpp:78`, `:112`; `pcbnew/router/pns_utils.cpp:199`; `pcbnew/router/pns_logger.cpp:175`). Two of these are in the collision and hull hot paths. Thread a `&Context` instead, and the router becomes usable from more than one thread and from more than one document at a time.
- **No pointer-keyed caches.** Generational keys, and the caches owned by the engine.
- **No `-1` sentinels.** `Option<i32>` for clearance, a three-state enum for keepout.
- **No stateful out-parameters in the resolver.** `IsKeepout( obstacle, item, &enforce )` with short-circuit evaluation at the call site is a bug farm.
- **Return the preview and the diff rather than pushing them.** Both are already whole-frame replacements in disguise.
- **Do not port `StackupHeight`.** It has no callers (section 1.11).
- **Fix the layer-span bug flagged at `pcbnew/router/pns_kicad_iface.cpp:3283`** if the tuning hooks are ever implemented.
