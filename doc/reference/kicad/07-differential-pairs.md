# KiCad PNS: DIFF_PAIR, DP_GATEWAYS, DIFF_PAIR_PLACER and the pair host surface

Reference architecture note for milestone 10 of this crate. Source tree: sparse checkout of KiCad master at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85`, read 2026-09-10. All `path:line` citations are relative to `/home/Tubbles/dev/ref/kicad/`.

Files read in full: `pcbnew/router/pns_diff_pair.h`, `pcbnew/router/pns_diff_pair.cpp`, `pcbnew/router/pns_diff_pair_placer.h`, `pcbnew/router/pns_diff_pair_placer.cpp`, `pcbnew/router/ranged_num.h`, `pcbnew/router/pns_sizes_settings.h`, `pcbnew/router/pns_sizes_settings.cpp`, `pcbnew/router/pns_placement_algo.h`. Read in the parts that touch pairs: `pcbnew/router/pns_optimizer.h/.cpp` (the `DIFF_PAIR` overload and its four helpers), `pcbnew/router/pns_router.h/.cpp` (the mode enum, the start gate, the placer factory), `pcbnew/router/router_tool.cpp` (the action, the dimension menu, the status bar), `pcbnew/router/pns_kicad_iface.cpp` (the three host hooks, `ImportSizes`, the constraint mapping), `pcbnew/board.cpp` (`MatchDpSuffix`, `DpCoupledNet`, read through `git show HEAD:pcbnew/board.cpp` because `pcbnew/board.cpp` is in the sparse checkout but its callers in `pcbnew/tools/` are not), `pcbnew/router/pns_topology.cpp` (`AssembleDiffPair`, `SimplifyLine`), `pcbnew/router/pns_dragger.cpp` and `pcbnew/router/pns_multi_dragger.cpp` (to establish that neither knows what a pair is), `qa/tests/pcbnew/test_pns_diff_pair_tuning_width.cpp`.

What is deliberately not here, because an earlier note has it:

- `ROUTER`'s enums, states, branch tree and session lifecycle: note 03 sections 1.1 to 1.6. Section 10 below covers only what differs when the mode is `PNS_MODE_ROUTE_DIFF_PAIR`.
- `LINE_PLACER`'s state machine, `routeStep`, tail management and posture: note 03 sections 3 and 4. Section 5 below is written as a diff against it wherever the two placers share a shape.
- `WALKAROUND`'s policies, statuses and `singleStep`: note 03 section 5. Section 11.1 records only what `attemptWalk` asks of it.
- `SHOVE`'s internals, the springback stack and the head protocol: note 04 sections 1 to 3. Section 11.2 records only the two entry points the pair placer uses and the one it does not.
- `OPTIMIZER`'s passes, effort flags and cost estimator: note 04 section 4. Section 6 below expands note 04 section 4.10 into the full pair path, and states which effort flags reach it (none).
- `SIZES_SETTINGS`'s single track half: note 03 section 2.2.
- The QA log format and the regression corpus: note 05 sections 6.2 to 6.9. It contains no pair case, which is why section 15 designs a synthetic fixture instead.

Two notes on the checkout. `pcbnew/tools/pcb_actions.cpp` is absent, so `PCB_ACTIONS::routeDiffPair` and `PCB_ACTIONS::routerDiffPairDialog` cannot be read where they are defined; everything said about them below comes from their use sites in `pcbnew/router/router_tool.cpp`. `pcbnew/dialogs/dialog_pns_diff_pair_dimensions.cpp` is likewise absent, so the dimension dialog is described from its one call site (`pcbnew/router/router_tool.cpp:2041`).

---

## 0. Read this first: four names in the task brief, and what they actually are

**`MatchGateways` does not exist.** A full tree grep returns nothing. The routine that turns two gateway sets into candidate routes is `DP_GATEWAYS::FitGateways` (`pcbnew/router/pns_diff_pair.cpp:336`), and the per candidate geometry is `DIFF_PAIR::BuildInitial` (`:208`). Section 4 covers both.

**`tryCurrentDpRoute` does not exist.** The routine that tries a walkaround for the pair is `DIFF_PAIR_PLACER::tryWalkDp` (`pcbnew/router/pns_diff_pair_placer.cpp:279`), which calls `attemptWalk` (`:200`) four times. Section 5.7.

**There is no pair dragger.** `pcbnew/router/pns_dragger.cpp`, `pns_dragger.h`, `pns_multi_dragger.cpp` and `pns_component_dragger.cpp` contain no reference to `DIFF_PAIR`, `DpNetPair`, `DpCoupledNet` or `DpNetPolarity` at this commit; a grep over the four files returns nothing. Dragging a track that happens to be half of a pair drags exactly that one track, with `DRAGGER` or `MULTI_DRAGGER` chosen by the shape of the item set (note 06 section 1.3), and the other half is left where it was. `MULTI_DRAGGER` is the closest thing KiCad has: selecting both tracks and dragging moves them together and preserves their spacing, but it does so as a generic bundle of parallel lines and not as a coupled pair, and nothing in it consults the pair hooks. Section 9 states this with the lines.

**`DIFF_PAIR_PLACER::UnfixRoute` does not exist either.** `PLACEMENT_ALGO::UnfixRoute` (`pcbnew/router/pns_placement_algo.h:81`) is virtual with a default body of `return std::nullopt;`, `LINE_PLACER` overrides it (`pcbnew/router/pns_line_placer.cpp:1759`) and `DIFF_PAIR_PLACER` does not. So backspace during a pair placement does nothing, silently. Section 5.11.

---

## 1. The data model

Four classes in `pcbnew/router/pns_diff_pair.h`, none of which is ever stored in a `NODE`. `DIFF_PAIR` derives from `LINK_HOLDER` and so is nominally an `ITEM` with kind `DIFF_PAIR_T = 64` (`pcbnew/router/pns_item.h:110`), but `DIFF_PAIR::Clone()` is `assert( false ); return nullptr;` (`pcbnew/router/pns_diff_pair.h:338`), which is the strongest possible statement that it is a value and not a board object. Nothing ever adds one to a node; the placer commits the two `LINE`s it holds and throws the pair away.

### 1.1 `DP_GATEWAY`: a pair of anchor points with a preferred exit

`pcbnew/router/pns_diff_pair.h:43`. Six fields (`:108` to `:113`):

| Field | Line | Meaning |
| --- | --- | --- |
| `m_entryP`, `m_entryN` | `:108` | Optional lead-in chains, from the primitive to the anchors. |
| `m_hasEntryLines` | `:109` | Whether the two above are meaningful. Set by `SetEntryLines` only. |
| `m_anchorP`, `m_anchorN` | `:110` | The two points a route leaves from or arrives at. |
| `m_isDiagonal` | `:111` | Documented at `:60` as "the gateway anchors lie on a diagonal line". Used as the `aStartDiagonal` argument to `BuildInitialTrace` in `buildEntries` (`pns_diff_pair.cpp:590`, `:592`) and nowhere else. |
| `m_allowedEntryAngles` | `:112` | A `DIRECTION_45::AngleType` mask, default `ANG_OBTUSE` (`:47`). |
| `m_priority` | `:113` | The score `FitGateways` sums, default 0 (`:47`). |

The constructor (`:46` to `:53`) takes the two anchors, the diagonal flag, the angle mask and the priority, and clears `m_hasEntryLines`. `SetEntryLines` (`:89`) sets both chains and the flag together; there is no way to set one. `Reverse()` (`pns_diff_pair.cpp:201`) reverses both entry chains in place, which is how a target gateway is turned into an entry gateway in `BuildInitial`.

`Entry()` (`pns_diff_pair.cpp:296`) is the odd one:

```cpp
const DIFF_PAIR DP_GATEWAY::Entry() const
{
    return DIFF_PAIR( m_entryP, m_entryN, 0 );
}
```

It builds a whole `DIFF_PAIR` by value out of the two entry chains, with a gap constraint of zero, purely so that `CheckConnectionAngle` can be called on it. It is called three times per `BuildInitial` (`:223`, `:226` twice through `CP()`/`CN()`, `:244`, `:247`, `:248`), each time constructing and destroying a `DIFF_PAIR`. In the port this is a free function over two chains, not a temporary object.

### 1.2 `DP_PRIMITIVE_PAIR`: the two board objects a pair starts or ends on

`pcbnew/router/pns_diff_pair.h:119`. It **owns** two cloned `ITEM*` plus two anchor points:

```
m_primP, m_primN : ITEM*     owned, deleted in the destructor (:94)
m_anchorP, m_anchorN : VECTOR2I
```

Three constructors:

- `DP_PRIMITIVE_PAIR( ITEM* aPrimP, ITEM* aPrimN )` (`pns_diff_pair.cpp:39`): clones both and takes `Anchor(0)` of each as the anchors.
- `DP_PRIMITIVE_PAIR( const VECTOR2I&, const VECTOR2I& )` (`:56`): anchors only, both item pointers null.
- The copy constructor (`:64`) nulls both pointers first, then clones whichever of the other's are non null.

`Directional()` (`:101`) is `m_primP && m_primP->OfKind( SEGMENT_T | ARC_T )`, testing only the P primitive. `DirP()` / `DirN()` (`:161`, `:167`) go through `anchorDirection` (`:110`):

```
anchorDirection(item, p):
    if item is not SEGMENT_T or ARC_T:  return DIRECTION_45()      # undefined
    if item->Anchor(0) == p:            return DIRECTION_45(Anchor(0) - Anchor(1))
    else:                               return DIRECTION_45(Anchor(1) - Anchor(0))
```

The direction points **away from** the anchor, back along the item, when the anchor is end 0, and **towards** end 1 otherwise. That asymmetry is deliberate: it is the direction the existing track arrives from, which `buildDpContinuation` extends.

`CursorOrientation` (`:122`) is the routine that decides which way "forward" is when the cursor is not on a target:

```
CursorOrientation(cursor, out midpoint, out direction):
    assert m_primP and m_primN                                   :125
    if both primitives are SEGMENT_T:                            :129
        aP = primP->Anchor(1);  aN = primN->Anchor(1)            :131
        if both segments are non degenerate and ApproxParallel:  :139
            midpoint  = (aP + aN) / 2                            :141
            direction = (segP.B - segP.A).Resize(|aP - aN|)      :142
            return                                               # no cursor flip
    else:
        aP = primP->Anchor(0);  aN = primN->Anchor(0)            :149
    midpoint  = (aP + aN) / 2                                    :153
    direction = (aP - aN).Perpendicular()                        :154
    if direction . (cursor - midpoint) < 0: direction = -direction  :156
```

Two things to carry over. First, the parallel-segments early return at `:144` skips the cursor dot product, so the direction is whichever way the P segment happens to run, cursor or not. Second, the `else` at `:147` is reached when the primitives are **not both segments**, which includes the mixed case of one segment and one via; in that case `Anchor(0)` is used for both, whereas the both-segments path uses `Anchor(1)`. There is no path that mixes `Anchor(0)` for one and `Anchor(1)` for the other.

`dump()` (`pns_diff_pair.h:150`) prints raw pointers with `printf`. No caller anywhere in the tree.

### 1.3 `DP_GATEWAYS`: the gateway builder and the fitter

`pcbnew/router/pns_diff_pair.h:167`. Four scalars plus the vector:

```
m_gap          : int    centre to centre spacing, set once by the constructor (:170)
m_viaGap       : int    initialised to m_gap (:172), overwritten by SetFitVias (:186)
m_viaDiameter  : int    0 until SetFitVias (:175, :184)
m_fitVias      : bool   true until SetFitVias says otherwise (:176, :183)
m_gateways     : std::vector<DP_GATEWAY>
```

The constructor takes only the gap, so a `DP_GATEWAYS` built and never given `SetFitVias` claims `m_fitVias == true` with `m_viaDiameter == 0`. That default matters: `routeHead` calls `SetFitVias` on the **target** set (`pns_diff_pair_placer.cpp:712`) but never on the **entry** set, so the entry set runs with `m_fitVias == true, m_viaDiameter == 0, m_viaGap == m_gap`. `m_fitVias` is read in exactly one place, `BuildForCursor` (`pns_diff_pair.cpp:548`, `:574`), which the entry set never reaches, so the stale default is harmless here. It is still a trap for a port that reorders the calls.

`SetFitVias( bool, int aDiameter = 0, int aViaGap = -1 )` (`:181`) treats a negative via gap as "same as the trace gap" (`:186`). The placer always passes an explicit value (`pns_diff_pair_placer.cpp:712`), so the `-1` branch is only reachable from a caller that does not exist.

`Clear()` (`:179`) and `CGateways()` (`:205`) have no caller in the tree.

The private `DP_CANDIDATE` struct (`:210` to `:215`) has five members; `FitGateways` writes only `p` and `n` (`pns_diff_pair.cpp:361`, `:362`) and reads only those two (`:375`). `gw_p`, `gw_n` and `score` are never touched. Section 13 lists it.

### 1.4 `DIFF_PAIR`: two chains, two lines, two vias

`pcbnew/router/pns_diff_pair.h:234`. The fields (`:558` to `:569`):

```
m_n, m_p             : SHAPE_LINE_CHAIN   the authoritative geometry
m_line_p, m_line_n   : LINE               lazily rebuilt views of the above
m_via_p, m_via_n     : VIA                the end vias, valid only when m_hasVias
m_hasVias            : bool
m_net_p, m_net_n     : NET_HANDLE
m_width              : int
m_gap                : int                see section 1.5, the meaning changes
m_viaGap             : int                never read
m_maxUncoupledLength : int                never read
m_chamferLimit       : int                never read
m_gapConstraint      : RANGED_NUM<int>
```

Five constructors. The default (`:259`), one taking a gap (`:273`), one taking two chains and a gap (`:289`), one taking two `LINE`s and a gap (`:307`), and a copy constructor (`:327`) that delegates to `operator=`. All four non-copy constructors zero `m_width`, `m_gap`, `m_viaGap`, `m_maxUncoupledLength` and `m_chamferLimit` by hand, with a comment about the static analyser (`:263`, `:279`, `:297`, `:319`). Only the two-`LINE` constructor sets the nets, from the lines (`:314`, `:315`).

The three gap-taking constructors assign `m_gapConstraint = aGap` directly (`:277`, `:295`, `:313`), which goes through `RANGED_NUM::operator=( const T )` (`ranged_num.h:38`) and therefore leaves both tolerances at zero. `SetGap` (`:434`) does something different:

```cpp
void SetGap( int aGap )
{
    m_gap = aGap;
    m_gapConstraint = RANGED_NUM<int>( m_gap, 10000, 10000 );
}
```

so a pair that went through `SetGap` matches gaps within +/- 10000 nm (10 um), and a pair that was only constructed does not. Any port that collapses the two paths changes `CoupledSegmentPairs`'s answer.

`PLine()` and `NLine()` (`:488`, `:496`) are the lazy views:

```cpp
LINE& PLine()
{
    if( !m_line_p.IsLinked() )
        updateLine( m_line_p, m_p, m_net_p, m_via_p );
    return m_line_p;
}
```

`updateLine` (`:545`) sets the shape, the width, the net, the layer from `Layers().Start()`, the parent and the source item, and appends the via when `m_hasVias`. Because nothing in the placer ever links these lines (`NODE::Add` links a *copy*, `pns_diff_pair_placer.cpp:850`), `IsLinked()` is always false and both accessors rebuild the whole line on every call. `Traces()` calls both (`:412`, `:413`), `rhMarkObstacles` calls both (`:109`, `:110`), `attemptWalk` calls one or the other twice per iteration (`:224`, `:225`), `updateLeadingRatLine` calls both (`:919`, `:922`). In the port these are not accessors; they are constructions, and the shape should be built once per move and passed down.

`AppendVias` (`:445`) copies both vias and clones their holes; `RemoveVias` (`:454`) clears the flag and calls `LINE::RemoveVia()` on the two cached lines but **leaves `m_via_p` and `m_via_n` as they were**. That is benign only because `updateLine` guards on `m_hasVias`.

`SetShape` has two overloads. `SetShape( const SHAPE_LINE_CHAIN& aP, const SHAPE_LINE_CHAIN& aN, bool aSwapLanes = false )` (`:399`) writes the two chains, swapping which is which when asked; `SetShape( const DIFF_PAIR& aPair )` (`:413`) copies the other pair's two chains and nothing else, so the gap, width, nets and vias of the destination survive. `tryWalkDp` relies on exactly that (`pns_diff_pair_placer.cpp:313`).

`Clear()` (`:513`), `Append()` (`:519`) and `Empty()` (`:525`) have no caller in the tree. `EndsWithVias()` (`:461`), `SetViaDiameter` (`:466`) and `SetViaDrill` (`:472`) are used by the placer only (`pns_diff_pair_placer.cpp:799`, `:801`, `:802`, `:821`, `:836`).

### 1.5 `m_gap` means two different things at two different moments

This is the single most confusing thing in the file and the port has to name the two quantities apart.

- **Centre to centre.** `DIFF_PAIR_PLACER::gap()` (`pns_diff_pair_placer.cpp:616`) is `m_sizes.DiffPairGap() + m_sizes.DiffPairWidth()`, the distance between the two centrelines. That is what both `DP_GATEWAYS` are constructed with (`:681`, `:682`), what every gateway builder spaces its anchors by, what `checkGap` compares against, and what `m_currentTrace` carries between `:731` and `:741`.
- **Edge to edge.** `m_sizes.DiffPairGap()` alone is the copper gap. That is what `m_currentTrace` carries after a successful fit (`:741`), what `CoupledSegmentPairs` matches against (because it subtracts `m_width` from the centreline distance, `pns_diff_pair.cpp:855`), and what `attemptWalk` hands the shove as a forced clearance (`:247`).

The switch happens inside `routeHead`:

```
routeHead(p):
    m_currentTrace.SetGap( gap() )            # centre to centre        :731
    result = gwsEntry.FitGateways(...)        # also SetGap(m_gap), c2c  :734, :374
    if result:
        m_currentTrace.SetWidth( DiffPairWidth() )                       :740
        m_currentTrace.SetGap( DiffPairGap() )   # edge to edge          :741
```

and on the failure path (`:756`, `return m_currentTraceOk;`) it does not happen, so a `m_currentTrace` whose fit failed but whose previous fit succeeded is left with the centre to centre value while `tryWalkDp` runs over it. Section 13 erratum E12.

### 1.6 `RANGED_NUM`

`pcbnew/router/ranged_num.h:25`. A value with an asymmetric tolerance:

```cpp
bool Matches( const T& aOther ) const
{
    return ( aOther >= m_value - m_toleranceMinus && aOther <= m_value + m_tolerancePlus );
}
```

with `operator T()` (`:33`) **non const**, which is why `checkGap( p, n, m_gapConstraint )` (`pns_diff_pair.cpp:251`) compiles only inside the non const `BuildInitial`. `T` is `int` at every instantiation, but three of the four `Matches` call sites pass an `int64_t` (`pns_diff_pair.cpp:857`, `:883`, `:931`) and the fourth passes an `int64_t` too (`pns_optimizer.cpp:1338`), so each one narrows at the call. Distances on a board never approach `INT_MAX` nanometres, so the narrowing is inert, but the port should take an `i64` and compare in `i64`.

---

## 2. The coupling geometry

This is the smallest self contained piece of the milestone and the one to port first (section 14). Everything in it is pure integer geometry over two `SHAPE_LINE_CHAIN`s plus a width and a gap constraint; nothing touches a `NODE`, a rule resolver or the router.

### 2.1 `commonParallelProjection`: the overlap of two near-parallel segments

`pcbnew/router/pns_diff_pair.cpp:786`. A free function with **external linkage**, forward declared again in `pcbnew/router/pns_topology.cpp:1033` so `AssembleDiffPair` can call it. It answers "which part of `p` faces `n`, and which part of `n` faces that".

```
commonParallelProjection(p, n, out pClip, out nClip) -> bool:
    n_proj_p = SEG( p.LineProject(n.A), p.LineProject(n.B) )        :788
    t_a     = 0                                                     :790
    t_b     = p.TCoef(p.B)                                          :791
    tproj_a = p.TCoef(n_proj_p.A)                                   :793
    tproj_b = p.TCoef(n_proj_p.B)                                   :794
    if t_b < t_a:         swap(t_a, t_b)                            :796
    if tproj_b < tproj_a: swap(tproj_a, tproj_b)                    :799
    if t_b <= tproj_a: return false        # no overlap             :802
    if t_a >= tproj_b: return false                                 :805

    t[4] = { 0, p.TCoef(p.B), p.TCoef(n_proj_p.A), p.TCoef(n_proj_p.B) }   :808
    sort(t)     # "fixme: awful and disgusting way of finding 2 midpoints" :810
    pLenSq = p.SquaredLength()                                      :812
    dp = p.B - p.A                                                  :814
    pClip.A = p.A + ( rescale(dp.x, t[1], pLenSq), rescale(dp.y, t[1], pLenSq) )   :815
    pClip.B = p.A + ( rescale(dp.x, t[2], pLenSq), rescale(dp.y, t[2], pLenSq) )   :818
    nClip.A = n.LineProject(pClip.A)                                :821
    nClip.B = n.LineProject(pClip.B)                                :822
    return true
```

`SEG::TCoef( aP )` is `(B - A) . (aP - A)`, an `ecoord` (`int64_t`), so `t` is the dot product parameterisation of `p` and `pLenSq` is its full-scale value. The two middle order statistics of the four `t` values are the clipped span, which is the intersection of `[0, |p|^2]` with the projection of `n` onto `p`'s line. Note that `t_a`/`t_b` and the first two entries of `t[]` are recomputed rather than reused, and the swap at `:796` is discarded because `t[]` is rebuilt unsorted; only the two early rejections use the swapped values.

Two consequences for the port. First, the sort of four `int64_t` is the whole "find two midpoints" step: replace it with two `min`/`max` pairs or keep the sort, it makes no difference at four elements. Second, `nClip` is the projection of `pClip` onto `n`'s **line**, not its segment, so `nClip` can lie outside `n` when the two are not truly parallel; every caller has already tested `ApproxParallel`, which bounds the error.

### 2.2 `DIFF_PAIR::CoupledSegmentPairs`: which segments face which

`pcbnew/router/pns_diff_pair.cpp:834`. The only routine that produces `COUPLED_SEGMENTS` records, and the input to the pair meander placer (`pns_dp_meander_placer.cpp:295`).

```
CoupledSegmentPairs(out pairs):
    # "Do not simplify the line chains here, otherwise the indices will be invalid"  :838
    for i in 0 .. m_p.SegmentCount()-1:
        if m_p.IsArcSegment(i): continue                            :842
        for j in 0 .. m_n.SegmentCount()-1:
            if m_n.IsArcSegment(j): continue                        :847
            sp = m_p.Segment(i);  sn = m_n.Segment(j)               :850
            dist = | sp.Distance(sn) - m_width |                    :855
            if sp.ApproxParallel(sn, 2)
               and m_gapConstraint.Matches(dist)
               and commonParallelProjection(sp, sn, p_clip, n_clip):  :857
                pairs.push_back( COUPLED_SEGMENTS(p_clip, sp, i, n_clip, sn, j) )  :860
```

Three details. The parallelism threshold is the explicit `2` (`:857`), where `CoupledLength( chains )` at `:883` uses `ApproxParallel`'s default and `TOPOLOGY::AssembleDiffPair` uses `DP_PARALLELITY_THRESHOLD = 5` (`pns_topology.h:106`, used at `pns_topology.cpp:1088`); three thresholds for the same question. `m_p.Segment(i)` is the non const accessor (`:850`), which is why the method is `const` but the chains are read through a const reference bound at `:836`. And the `dist` is `|centreline distance - width|`, that is the copper edge to edge gap, which is why `m_gapConstraint` has to hold the **edge to edge** value here (section 1.5).

The comment at `:838` is the reason `mergeDpStep` calls `Simplify2` on its own copies rather than on the pair (`pns_optimizer.cpp:1473`, `:1482`): the `indexP`/`indexN` fields are indices into the unsimplified chains and any caller holding them must not reindex underneath.

### 2.3 The three `CoupledLength` overloads

They compute the same quantity three ways and disagree in two places.

**`int64_t CoupledLength( const SHAPE_LINE_CHAIN& aP, const SHAPE_LINE_CHAIN& aN ) const`** (`:868`), the one the optimizer uses:

```
total = 0
for i in aP.SegmentCount(), j in aN.SegmentCount():
    sp = aP.CSegment(i);  sn = aN.CSegment(j)
    dist = | sp.Distance(sn) - m_width |
    if sp.ApproxParallel(sn) and m_gapConstraint.Matches(dist)
       and commonParallelProjection(sp, sn, p_clip, n_clip):
        total += p_clip.Length()
return total
```

No arc guard, default parallelism threshold, `int64_t` accumulator, and it takes the chains as arguments so the optimizer can score a hypothetical shape without mutating the pair.

**`double CoupledLength() const`** (`:893`) runs `CoupledSegmentPairs` and sums `pair.coupledP.Length()`. Same number, computed with the arc guard and threshold 2, accumulated in a `double`. Used by `tryWalkDp` (`pns_diff_pair_placer.cpp:294`).

**`int CoupledLength( const SEG& aP, const SEG& aN ) const`** (`:928`) is the single segment case, returning `p_clip.Length()` or 0. **No caller in the tree.**

`Skew()` (`:828`) is `m_p.Length() - m_n.Length()` as a `double`; `TotalLength()` (`:919`) is their mean; `CoupledLengthFactor()` (`:908`) is `CoupledLength() / TotalLength()` with a zero guard. `CoupledLengthFactor` has no caller, and `TotalLength` has no caller other than `CoupledLengthFactor`, so the pair is dead together.

### 2.4 `checkGap`: the accept test on a candidate route

`pcbnew/router/pns_diff_pair.cpp:182`, a file static:

```cpp
static bool checkGap( const SHAPE_LINE_CHAIN &p, const SHAPE_LINE_CHAIN &n, int gap )
{
    SEG::ecoord gap_sq = SEG::Square( gap - 100 );
    for( int i = 0; i < p.SegmentCount(); i++ )
        for( int j = 0; j < n.SegmentCount() ; j++ )
            if( p.CSegment( i ).SquaredDistance( n.CSegment( j ) ) < gap_sq )
                return false;
    return true;
}
```

This is a **minimum** test, not a coupling test: it rejects a candidate in which any P segment comes closer than `gap - 100` to any N segment. It says nothing about the two staying at the gap; a candidate that diverges wildly passes. The `100` is a bare nanometre slack with no name. `gap` arrives as `m_gapConstraint` (`:251`) through `RANGED_NUM::operator T()`, so only the value is used and the +/- 10000 tolerance is discarded at this call.

The comparison is `O(segments^2)` and it runs inside the innermost loop of `FitGateways`, which is itself `O(entry x target x 2)`. Section 4.3 works out the resulting cost.

### 2.5 Unit tests this section should carry

The whole of section 2 is testable with two hand written chains and no world at all, which is why it comes first in section 14. The cases worth pinning:

- Two parallel horizontal segments at exactly `gap + width` centre to centre: one coupled pair, `coupledP` equal to the overlap.
- The same offset along their common direction so only half overlaps: `commonParallelProjection` clips to the overlap, `CoupledLength` equals the overlap length.
- The same with no overlap at all: `commonParallelProjection` returns false, no pair.
- Antiparallel segments (N reversed): `ApproxParallel` is direction blind, so they still couple; the clip is still the overlap. This is worth an explicit test because it is easy to break with a direction check.
- A segment pair at `gap + width + 10001`: rejected by `Matches` after `SetGap`, accepted at exactly `+ 10000`.
- A diagonal pair, to catch integer rounding in `rescale`.
- `Skew` on chains of different length, and `Skew` on the same chain twice (zero).
- `checkGap` on an L shaped candidate whose two arms pass within `gap/2` at the corner: rejected.

---

## 3. Building gateways

Five public builders and two private ones. Every one of them appends to `m_gateways`; none of them clears it first, so a `DP_GATEWAYS` accumulates across calls. `routeHead` relies on that: `BuildFromPrimitivePair` ends by calling `BuildGeneric`, which appends on top of whatever the diagonal-alignment block already pushed.

### 3.1 `makeGapVector`: the half-gap primitive

`pcbnew/router/pns_diff_pair.cpp:401`, a file static and the key to reading every builder below.

```cpp
static VECTOR2I makeGapVector( VECTOR2I dir, int length )
{
    int l = length / 2;
    VECTOR2I rv;
    if( dir.EuclideanNorm() == 0 )
        return dir;
    do {
        rv = dir.Resize( l );
        l++;
    } while( ( rv * 2 ).EuclideanNorm() < length );
    return rv;
}
```

It returns a vector along `dir` of **half** the requested length, rounded up until doubling it reaches `length`. So `p0 + makeGapVector(d, g)` and `p0 - makeGapVector(d, g)` are `g` apart, not `2g`. The loop exists because `VECTOR2I::Resize` rounds, so `Resize(length/2) * 2` can fall a nanometre short; it increments until it does not. With a zero direction it returns the zero vector and the caller silently gets a degenerate gateway.

For the port: this is a small pure function over `Vec2` and an `i32`, and it deserves its own test (`makeGapVector(v, L)` doubled is at least `L`, and at most `L + 2` for any `v`).

### 3.2 `BuildGeneric`: the fan of exits from two points

`pcbnew/router/pns_diff_pair.cpp:649`. The workhorse. It takes the two primitive anchor points and produces every 45 degree gateway a pair could leave them through. `aViaMode` suppresses everything that is not usable when the gateway has to fit a via pair.

It starts by building eight probe segments (`:658` to `:665`), each 200 nm long and centred on one of the two points: horizontal and vertical through P and through N (`st_p[0..1]`, `st_n[0..1]`), and the two diagonals through P and through N (`d_p[0..1]`, `d_n[0..1]`). They are used only as **lines**, through `Collinear` and `IntersectLines`; the 100 nm half length is arbitrary.

```
BuildGeneric(p0_p, p0_n, aBuildEntries, aViaMode):
    padToGapThreshold = 3                                            :655
    padDist = |p0_n - p0_p|                                          :656

    # --- part one: the two points already lie on a common 45 degree line
    for i in 0..1:                                                   :668
        straightColl = st_p[i].Collinear(st_n[i])                    :670
        diagColl     = d_p[i].Collinear(d_n[i])                      :671
        if straightColl or diagColl:                                 :673
            dir  = makeGapVector(p0_n - p0_p, m_gap / 2)             :675
            m    = (p0_p + p0_n) / 2                                 :676
            prio = (padDist > 3 * m_gap) ? 2 : 1                     :677
            if not aViaMode:                                         :679
                push( m - dir, m + dir, diagColl, ANG_RIGHT, prio )  :681
                dir = makeGapVector(p0_n - p0_p, 2 * m_gap)          :684
                push( p0_p - dir, p0_p - dir + perp(dir), diagColl ) :685
                push( p0_p - dir, p0_p - dir - perp(dir), diagColl ) :686
                push( p0_n + dir + perp(dir), p0_n + dir, diagColl ) :687
                push( p0_n + dir - perp(dir), p0_n + dir, diagColl ) :688

    # --- part two: intersections of the eight probe lines
    for i in 0..1, j in 0..1:                                        :693
        ips[0] = d_n[i].IntersectLines(d_p[j])                       :699
        ips[1] = st_p[i].IntersectLines(st_n[j])                     :700
        if d_n[i].Collinear(d_p[j]):   ips[0] = none                 :702
        if st_p[i].Collinear(st_p[j]): ips[1] = none                 :705   # see E5

        # diagonal-diagonal and straight-straight
        for k in 0..1:                                               :710
            if ips[k] and ips[k] != p0_p and ips[k] != p0_n:         :716
                m    = *ips[k]
                prio = (padDist > 3 * m_gap) ? 10 : 20               :718
                g_p  = (p0_p - m).Resize( ceil(m_gap * 1/sqrt(2)) )  :719
                g_n  = (p0_n - m).Resize( ceil(m_gap * 1/sqrt(2)) )  :720
                push( m + g_p, m + g_n, k == 0, ANG_OBTUSE, prio )   :722

        ips[0] = st_n[i].IntersectLines(d_p[j])                      :728
        ips[1] = st_p[i].IntersectLines(d_n[j])                      :729

        # diagonal-straight: "8 possibilities of weirder exits"
        for k in 0..1:                                               :732
            if ips[k] and not aViaMode and ips[k] != p0_p and != p0_n:  :738
                m = *ips[k]
                g_p = (p0_p - m).Resize( ceil(m_gap * sqrt(2)) )     :742
                g_n = (p0_n - m).Resize( ceil(m_gap) )               :743
                if angle(g_p, g_n) != ANG_ACUTE: push(m+g_p, m+g_n, true)  :745
                g_p = (p0_p - m).Resize( m_gap )                     :748
                g_n = (p0_n - m).Resize( ceil(m_gap * sqrt(2)) )     :749
                if angle(g_p, g_n) != ANG_ACUTE: push(m+g_p, m+g_n, true)  :751

    if aBuildEntries: buildEntries(p0_p, p0_n)                       :759
```

The spacing arithmetic is worth doing once, because it is the invariant the rest of the file assumes: **every gateway's two anchors are `m_gap` apart**, except one.

- `:685` to `:688`: `|dir| ~ m_gap` because `makeGapVector(v, 2*m_gap)` halves, and the perpendicular offset is `|dir|`, so the separation is `m_gap`. Good.
- `:719` to `:722`: `m` is the intersection of a P line and an N line, so `p0_p - m` and `p0_n - m` are perpendicular in the straight-straight and diagonal-diagonal cases; two legs of `m_gap/sqrt(2)` at 90 degrees give a hypotenuse of `m_gap`. Good.
- `:742` to `:752`: 45 degrees between the two directions, legs `m_gap*sqrt(2)` and `m_gap`; the law of cosines gives `sqrt(2g^2 + g^2 - 2*g*sqrt(2)*g*cos45) = g`. Good.
- `:675` to `:682`: `makeGapVector(v, m_gap / 2)` has length `m_gap/4`, so `m - dir` and `m + dir` are `m_gap/2` apart. **Half the gap.** Section 13 erratum E4.

`angle( a, b )` at `:745` and `:751` is the file static at `:173`, `DIRECTION_45(a).Angle(DIRECTION_45(b))`.

The priority ladder that comes out of this function is `20` (close intersection exits), `10` (far intersection exits), `2` and `1` (the collinear midpoint exit), and `0` (everything else, including the four side-by exits and both diagonal-straight families). `BuildFromPrimitivePair` adds `100` and `99` on top, and `buildDpContinuation` adds `100`, `20` and `5`.

### 3.3 `buildEntries`: leads from the primitive to the anchors

`pcbnew/router/pns_diff_pair.cpp:583`.

```
buildEntries(p0_p, p0_n):
    for g in m_gateways:
        if g.HasEntryLines(): continue
        lead_p = DIRECTION_45().BuildInitialTrace(g.AnchorP(), p0_p, g.IsDiagonal()).Reverse()
        lead_n = DIRECTION_45().BuildInitialTrace(g.AnchorN(), p0_n, g.IsDiagonal()).Reverse()
        g.SetEntryLines(lead_p, lead_n)
```

It runs over **every** gateway in the set, including ones an earlier builder appended, and skips only those that already have entry lines. So calling `BuildGeneric(..., aBuildEntries = true)` twice on the same set is idempotent for the first batch and fills in the second. The trace is built from the anchor back to the primitive and then reversed, which is not the same as building it forward: `BuildInitialTrace` is not symmetric in its endpoints (note 01 covers this; the first segment's direction is chosen relative to the *start*). The reversal is therefore load bearing and a port must not "simplify" it to a forward build.

`g.IsDiagonal()` is used here as the `aStartDiagonal` hint, meaning "the lead's first segment out of the anchor is diagonal".

### 3.4 `buildDpContinuation`: extending an existing pair of tracks

`pcbnew/router/pns_diff_pair.cpp:599`. Reached from `BuildFromPrimitivePair` when both primitives are segments or arcs (`:445`).

```
buildDpContinuation(pair, isDiagonal):
    push( DP_GATEWAY(pair.AnchorP(), pair.AnchorN(), isDiagonal), priority 100 )   :601
    if not pair.Directional(): return                                             :605

    EPSILON  = 5          # 0.005 um                                              :612
    SIN_22_5 = 0.38268                                                            :613
    SIN_23_5 = 0.39875                                                            :614

    addAngledGateways(length, priority):                                          :616
        entryP = [ anchorP, anchorP + DirP().ToVector().Resize(length) ]          :620
        push( DP_GATEWAY(entryP.last, anchorN, isDiagonal), priority,
              entry lines (entryP, empty) )                                       :622
        entryN = [ anchorN, anchorN + DirN().ToVector().Resize(length) ]          :627
        push( DP_GATEWAY(anchorP, entryN.last, isDiagonal), priority,
              entry lines (empty, entryN) )                                       :630

    delta = anchorP - anchorN                                                     :636
    if |delta.x| < EPSILON or |delta.y| < EPSILON or |delta.x - delta.y| < EPSILON:  :638
        addAngledGateways( round(m_gap * SIN_22_5), 20 )                          :640
        addAngledGateways( round(m_gap * SIN_23_5), 5 )                           :644
```

The first gateway is the identity: leave the pair exactly where the existing tracks end. Priority 100 makes it beat everything `BuildGeneric` produces, which is why continuing an existing pair goes straight on by default.

The four angled gateways exist so the pair can turn 45 degrees without the inner track having to double back. Stepping **one** anchor forward by `gap * sin(22.5)` rotates the anchor line by 22.5 degrees, which is exactly half of a 45 degree turn, so the pair enters the turn already half rotated. The second call with `sin(23.5)` and priority 5 is an admitted fudge, with the comment at `:642` and a link to KiCad issue 12459: "sin(22.5) doesn't always work, so we also add some lower priority ones with a bit of wiggle room".

The guard at `:638` accepts only pairs whose anchor line is horizontal, vertical or at 45 degrees (`|delta.x - delta.y| < EPSILON` catches the `+45` diagonal but **not** the `-45` one, where `delta.x + delta.y` is near zero). Section 13 erratum E6.

Note the entry lines are set with one chain empty (`:624` passes an empty `SHAPE_LINE_CHAIN` for N, `:632` for P). `BuildInitial` then appends an empty chain to one side, and `CheckConnectionAngle` short circuits to `true` for the empty side because `SegmentCount() == 0` (`pns_diff_pair.cpp:268`, `:280`). That is the mechanism by which one lane leads and the other does not.

### 3.5 `BuildFromPrimitivePair`: the entry point for both ends

`pcbnew/router/pns_diff_pair.cpp:418`. The dispatcher on what kind of board objects the pair sits on.

```
BuildFromPrimitivePair(pair, preferDiagonal):
    if pair.PrimP() == nullptr:                                      :426
        BuildGeneric(pair.AnchorP(), pair.AnchorN(), entries=true)
        return

    if both primitives are SOLID_T or VIA_T:                         :434
        p0_p = pair.AnchorP();  p0_n = pair.AnchorN()
        shP  = pair.PrimP()->Shape(-1)          # TODO(JE) padstacks  :440
    elif both are SEGMENT_T or ARC_T:                                :442
        buildDpContinuation(pair, preferDiagonal); return            :445

    majorDirection = (p0_p - p0_n).Perpendicular()                   :450
    if shP == nullptr: return                                        :452

    switch shP->Type():                                              :455
      SH_CIRCLE:  BuildGeneric(p0_p, p0_n, entries=true); return     :457
      SH_RECT:    w,h = width,height of the rect; if w<h swap        :461
                  orthoFanDistance = (w+1)*3/2 ; diagFanDistance = w-h
      SH_SEGMENT: w = shape width; s = shape segment                 :474
                  orthoFanDistance = w + |s.B - s.A|
                  diagFanDistance  = |s.B - s.A|
      SH_SIMPLE, SH_COMPOUND: same as SH_RECT over the bounding box  :484
      default:    wxFAIL_MSG("Unsupported starting primitive")       :499

    if checkDiagonalAlignment(p0_p, p0_n):                           :506
        padDist = |p0_p - p0_n|
        for k in 0..1:                                               :510
            dir = makeGapVector(majorDirection, k==0 ? orthoFanDistance
                                                     : diagFanDistance)
            d  = max(0, padDist - m_gap)                             :519
            dp = makeGapVector(dir, d)                               :520
            dv = makeGapVector(p0_n - p0_p, d)                       :521
            for i in 0..1:                                           :523
                sign = i ? -1 : 1
                gw_p  = p0_p + sign*(dir + dp) + dv                  :527
                gw_n  = p0_n + sign*(dir + dp) - dv                  :528
                entryP = [ p0_p, p0_p + sign*dir, gw_p ]             :530
                entryN = [ p0_n, p0_n + sign*dir, gw_n ]             :531
                push( DP_GATEWAY(gw_p, gw_n, false), priority 100-k,
                      entry lines (entryP, entryN) )                 :533

    BuildGeneric(p0_p, p0_n, entries=true)                           :542
```

The mixed case is silently unhandled: if `PrimP()` is a pad and `PrimN()` is a segment, neither `if` at `:434` nor `elif` at `:442` fires, `shP` stays null, and the function returns at `:453` having pushed nothing. `FindDpPrimitivePair` prevents that by requiring `item->Kind() == aItem->Kind()` (`pns_diff_pair_placer.cpp:559`), so in practice the two are always the same kind. Section 13 erratum E10.

The fan block at `:506` is the "walk the pair out of two pads side by side" case, and its arithmetic is the one that produces the classic staircase breakout. `dir` is perpendicular to the anchor line, half the fan distance long. `dp` is along `dir` and `dv` is along the anchor line, both half of `padDist - m_gap`, which is exactly the amount the two anchors have to converge to reach the gap. So `gw_p` and `gw_n` end up `m_gap` apart (section 3.2's invariant), offset from the pads by `dir + dp`, with a two segment lead through `p0 + sign*dir`. `k = 0` uses the ortho fan distance and gets priority 100, `k = 1` uses the diagonal one and gets 99.

`checkDiagonalAlignment( a, b )` (`:383`) is:

```cpp
VECTOR2I dir( abs(a.x - b.x), abs(a.y - b.y) );
return (dir.x == 0 && dir.y != 0) || (dir.x == dir.y) || (dir.y == 0 && dir.x != 0);
```

that is, the two points are on a common horizontal, vertical or 45 degree line. Note `dir.x == dir.y` is true when both are zero, so two coincident anchors pass; nothing else guards against that.

The `SH_RECT` branch swaps `w` and `h` so `w` is the long side (`:466`), then `diagFanDistance = w - h` is zero for a square pad, and `makeGapVector(majorDirection, 0)` returns a vector of length 0 or 1. So for square pads the `k = 1` fan collapses onto the pad centres and produces a degenerate gateway with priority 99. It is filtered out later only by `BuildInitial`'s `checkGap` and self intersection tests.

`shP = aPair.PrimP()->Shape( -1 )` (`:440`) is the P primitive's shape on the "all layers" pseudo layer, with a `TODO(JE) padstacks` comment at `:439`. The N primitive's shape is never consulted, so a pair of differently shaped pads is fanned as if both were shaped like P.

### 3.6 `BuildForCursor`: gateways at a free cursor position

`pcbnew/router/pns_diff_pair.cpp:546`. Used when the cursor is not over a target pair.

```
BuildForCursor(cursor):
    gap = m_fitVias ? m_viaGap + m_viaDiameter : m_gap               :548
    for diagonal in { false, true }:                                 :550
        for i in 0..3:                                               :552
            if not diagonal:                                         :556
                dir = makeGapVector( VECTOR2I(gap, gap), gap )
                if i % 2 == 0: dir.x = -dir.x
                if i / 2 == 0: dir.y = -dir.y
            else:                                                    :566
                if i / 2 == 0: dir = ( (gap+1)/2 * (i%2 ? -1 : 1), 0 )
                else:          dir = ( 0, (gap+1)/2 * (i%2 ? -1 : 1) )
            if m_fitVias: BuildGeneric(cursor + dir, cursor - dir, true, viaMode=true)  :575
            else:         push( DP_GATEWAY(cursor + dir, cursor - dir, diagonal) )      :577
```

Eight gateways when not fitting vias: four with the anchor line on a diagonal (the `!diagonal` branch, `dir` has both components non zero) and four with it axis aligned (the `diagonal` branch). The naming is inverted relative to `DP_GATEWAY`'s documented meaning of `m_isDiagonal` ("the anchors lie on a diagonal line", `pns_diff_pair.h:60`): the branch that puts the anchors on a diagonal passes `false`, and the branch that puts them on an axis passes `true`. Section 13 erratum E7. Whether it is a bug depends on reading `m_isDiagonal` as "the lead out of this gateway starts diagonal", which is how `buildEntries` uses it (`:590`), and under that reading the values are right: a pair sitting on a diagonal wants a straight-first lead.

Only four of the eight `!diagonal` cases are distinct, because `i % 2` and `i / 2` cover all four sign combinations, and the `diagonal` branch produces four distinct axis offsets. There is no deduplication anywhere.

When `m_fitVias` is on, each of the eight positions is expanded by a full `BuildGeneric` in via mode, so the set can reach roughly eight times the generic fan. The spacing used is `m_viaGap + m_viaDiameter`, that is centre to centre for a pair of vias, and not `m_gap`.

### 3.7 `FilterByOrientation`: dropping gateways that face the wrong way

`pcbnew/router/pns_diff_pair.cpp:391`.

```cpp
void DP_GATEWAYS::FilterByOrientation( int aAngleMask, DIRECTION_45 aRefOrientation )
{
    std::erase_if( m_gateways,
        [&]( const DP_GATEWAY& dp )
        {
            DIRECTION_45 orient( dp.AnchorP() - dp.AnchorN() );
            return ( orient.Angle( aRefOrientation ) & aAngleMask );
        } );
}
```

It **removes** the gateways whose anchor line angle to the reference is in the mask, which reads backwards from the name but is what the one caller wants: `routeHead` passes `ANG_STRAIGHT | ANG_HALF_FULL` with the cursor direction as reference (`pns_diff_pair_placer.cpp:724`), that is, "drop every gateway whose anchor line is parallel or antiparallel to the direction of travel", leaving the ones whose anchor line is across the direction of travel. The predicate returns an `int` implicitly converted to `bool`, so any bit in the mask counts.

The lambda captures `aRefOrientation` by value, and `DIRECTION_45( VECTOR2I )` snaps to the nearest octant, so two anchors 1 nm apart still produce a defined direction; two coincident anchors produce `DIR_UNDEFINED` whose `Angle` is `ANG_UNDEFINED` (`libs/kimath/include/geometry/direction45.h:184`), which is not in the mask, so degenerate gateways survive the filter.

### 3.8 `BuildOrthoProjections`: dead

`pcbnew/router/pns_diff_pair.cpp:302`, declared at `pns_diff_pair.h:194`. **No caller anywhere in the tree.** For each gateway in `aEntries` it projects the cursor onto a horizontal and a diagonal guide through the gateway midpoint, keeps the nearer projection, builds a fresh `DP_GATEWAYS` for that point with the via settings copied across (`:321` to `:323`, direct private member access on another instance of the same class), runs `BuildForCursor` on it and appends every result with `aOrthoScore` as the priority.

It is the ortho mode counterpart of the `lead_dist` branch in `routeHead`, and `DIFF_PAIR_PLACER::SetOrthoMode` (`pns_diff_pair_placer.cpp:84`) sets `m_orthoMode` which **is never read anywhere in the placer**. So ortho mode for pairs is wired up at the host end, stored, and ignored. Section 13 errata E1 and E13. Do not port either.

---

## 4. Fitting: turning two gateway sets into one route

### 4.1 `DIFF_PAIR::BuildInitial`: one candidate

`pcbnew/router/pns_diff_pair.cpp:208`. Given one entry gateway and one target gateway, build the two chains and say whether the result is acceptable.

```
BuildInitial(entry, target, prefDiagonal) -> bool:
    p = BuildInitialTrace(entry.AnchorP(), target.AnchorP(), prefDiagonal)   :211
    n = BuildInitialTrace(entry.AnchorN(), target.AnchorN(), prefDiagonal)   :213

    mask = entry.AllowedAngles() | ANG_STRAIGHT | ANG_OBTUSE                 :216
    m_p = p ; m_n = n                                                        :218   (redundant, see below)

    if entry.HasEntryLines():                                                :221
        if not entry.Entry().CheckConnectionAngle(*this, mask): return false  :223
        m_p = entry.Entry().CP() ; m_p.Append(p)                             :226, :228
        m_n = entry.Entry().CN() ; m_n.Append(n)                             :227, :229
    else:
        m_p = p ; m_n = n                                                    :233

    mask = target.AllowedAngles() | ANG_STRAIGHT | ANG_OBTUSE                :237
    if target.HasEntryLines():                                               :239
        t = copy of target ; t.Reverse()                                     :241, :242
        if not CheckConnectionAngle(t.Entry(), mask): return false           :244
        m_p.Append(t.Entry().CP())                                           :247
        m_n.Append(t.Entry().CN())                                           :248

    if not checkGap(p, n, m_gapConstraint): return false                     :251
    if p.SelfIntersecting() or n.SelfIntersecting(): return false            :254
    if p.Intersects(n): return false                                         :257
    return true
```

Four things a port must get right.

**The three accept tests run on `p` and `n`, not on `m_p` and `m_n`.** Lines `:251`, `:254` and `:257` all use the raw middle traces. The entry and target leads are never gap checked, never self intersection checked, and never checked against the other lane. That is deliberate to the extent that the leads come from a builder that already spaced them, but it means a lead that crosses the other lane, or a two segment fan lead that doubles back on itself, is accepted.

**`CheckConnectionAngle` is called in two different directions.** At `:223` it is `entryDiffPair.CheckConnectionAngle( *this, mask )`, so the last segment of the *entry lead* is compared to the first segment of the *middle trace* which was just stored into `m_p`/`m_n` at `:218`. That store at `:218` exists only to feed this call, and the `else` branch at `:233` repeats it, which is why `:218` looks redundant but is not. At `:244` it is `this->CheckConnectionAngle( t.Entry(), mask )` where `this` now holds entry lead plus middle, so the last segment of the middle is compared to the first segment of the reversed target lead.

**The angle mask always includes straight and obtuse.** `AllowedAngles()` defaults to `ANG_OBTUSE` and the only gateways that set something else are the collinear midpoint exit with `ANG_RIGHT` (`:681`) and the intersection exits with `ANG_OBTUSE` (`:722`). So the mask is `ANG_STRAIGHT | ANG_OBTUSE` almost everywhere and `ANG_STRAIGHT | ANG_OBTUSE | ANG_RIGHT` for the midpoint exit.

**`DP_GATEWAY t( aTarget ); t.Reverse();`** at `:241` copies the whole gateway, including both entry chains, and reverses them so the lead runs from the middle out to the target primitive rather than in. That copy is per candidate, inside the triple loop.

`CheckConnectionAngle` itself (`:264`):

```
CheckConnectionAngle(other, allowedAngles) -> bool:
    checkP = (m_p empty or other.m_p empty)
             or ( DIRECTION_45(m_p.CSegment(-1)).Angle(DIRECTION_45(other.m_p.CSegment(0))) & allowedAngles ) != 0
    checkN = same for the N chains
    return checkP and checkN
```

An empty chain on either side passes, which is what makes the one sided angled gateways of `buildDpContinuation` work (section 3.4).

### 4.2 `DP_GATEWAYS::FitGateways`: the candidate search

`pcbnew/router/pns_diff_pair.cpp:336`.

```
FitGateways(entrySet, targetSet, prefDiagonal, out dp) -> bool:
    best : DP_CANDIDATE
    bestScore = -1000 ; found = false                                :341, :342
    for g_entry in entrySet.Gateways():                              :344
        for g_target in targetSet.Gateways():                        :346
            for preferred in { false, true }:                        :348
                score = (preferred ? 0 : -3) + g_entry.Priority() + g_target.Priority()  :350
                if score >= bestScore:                               :354
                    l = DIFF_PAIR(m_gap)                             :356
                    if l.BuildInitial(g_entry, g_target,
                                      preferred ? prefDiagonal : !prefDiagonal):  :358
                        best.p = l.CP() ; best.n = l.CN()            :361
                        bestScore = score ; found = true             :363
    if found:
        dp.SetGap(m_gap)                                             :374
        dp.SetShape(best.p, best.n)                                  :375
        return true
    return false
```

The signature is `FitGateways( DP_GATEWAYS& aEntry, DP_GATEWAYS& aTarget, ... )` on a `DP_GATEWAYS` instance, and the only call site is `gwsEntry.FitGateways( gwsEntry, gwsTarget, ... )` (`pns_diff_pair_placer.cpp:734`), passing the receiver as the first argument. So `this` is used only for `m_gap` (`:356`, `:374`) and `aEntry` shadows it. It should be a static or a free function; a port should make it one, taking the gap explicitly.

The scoring is worth spelling out because it is not a maximisation of route quality, it is a maximisation of gateway priority with a tie break:

- `score` depends only on the two priorities and the posture flag. Geometry contributes nothing.
- The guard is `score >= bestScore`, not `>`, so among equal scoring candidates the **last one that builds** wins. Iteration order over the two gateway vectors therefore decides the route. Both vectors are `std::vector` filled in a deterministic order by the builders, so KiCad is deterministic here; a port must preserve builder push order exactly to get the same routes.
- `preferred` runs `false` before `true`, and `false` scores `-3`. So for a fixed gateway pair, the `prefDiagonal` posture is tried second and wins the `>=` tie unless it fails to build. The `-3` only matters against gateway priority differences smaller than 3, which is the gap between the `100` and `99` fan priorities and between `20` and `5` in `buildDpContinuation`.
- `BuildInitial` is called **only when the score already beats the best**, so a high scoring gateway pair that fails to build does not stop a lower scoring one from being tried later, but a lower scoring one that appears earlier is skipped entirely. Ordering and scoring interact.

### 4.3 The cost of the search

`|entry gateways| x |target gateways| x 2` calls to `BuildInitial`, each of which does two `BuildInitialTrace`s, up to three `DIFF_PAIR` temporaries for the `Entry()` calls, one `O(sp * sn)` `checkGap`, two `SelfIntersecting` and one chain-chain `Intersects`.

Rough sizes: `BuildGeneric` pushes up to 5 gateways from the collinear block plus up to 8 from the intersection block plus up to 16 from the diagonal-straight block, so tens. `BuildFromPrimitivePair` on a pad pair adds 4 and then calls `BuildGeneric`. `BuildForCursor` without vias pushes exactly 8; with vias it runs `BuildGeneric` eight times, so hundreds. The worst case is therefore a via placement move, at a few hundred entry times a few hundred target, which is tens of thousands of `BuildInitial` calls per mouse move. There is no early exit, no spatial pruning and no memoisation.

For the port this is the first thing to measure. One cheap win that cannot change the result: hoist the `DP_GATEWAY t( aTarget ); t.Reverse();` copy out of `BuildInitial` so each target gateway is reversed once instead of once per candidate. A second, bigger one is to walk the candidates in descending score order and stop once the remaining maximum cannot beat the best found. That is exact for the score, which is pure gateway priority, but it is **not** neutral for the route: the `>=` at `:354` makes the last equal scoring candidate that builds the winner, so reordering changes which one that is. Take it only with a deliberate log entry, or prune on strictly lower scores and keep the original order.

---

## 5. `DIFF_PAIR_PLACER`

`pcbnew/router/pns_diff_pair_placer.h:52`, deriving from `PLACEMENT_ALGO` (`pns_placement_algo.h:45`). It implements the same 17 method interface as `LINE_PLACER` and shares its overall shape: `Start` picks anchors and branches the world, `Move` re-routes from scratch, `FixRoute` writes into the node and either finishes or starts a new leg. Everything below is written as a diff against note 03 section 3.

### 5.1 State

`pcbnew/router/pns_diff_pair_placer.h:217` to `:281`. Compared to `LINE_PLACER` there is no tail, no head, no mouse trail tracer, no fixed tail stack and no direction member.

| Member | Line | Live? | Meaning |
| --- | --- | --- | --- |
| `m_state` (`RT_START`/`RT_ROUTE`/`RT_FINISH`) | `:223` | **dead** | Set to `RT_START` in the constructor (`:37`) and never read or written again. |
| `m_chainedPlacement` | `:225` | live | Set at `:840`/`:844`, read at `:444` to refuse a layer change mid chain. |
| `m_initialDiagonal` | `:226` | live | The posture carried across legs; `:817` derives it from the committed shape, `:661` restores it. |
| `m_startDiagonal` | `:227` | live | The posture of the current leg; `FlipPosture` toggles it (`:421`), all three gateway calls take it (`:687`, `:693`, `:734`). |
| `m_fitOk` | `:228` | live | Whether the last `route()` produced a collision free pair. Gates `FixRoute` (`:810`). |
| `m_netP`, `m_netN` | `:230` | live | |
| `m_start` | `:232` | live | The pair the session started on. |
| `m_prevPair` | `:233` | live | `std::optional`; the pair the current leg starts from. `routeHead` seeds it from `m_start` (`:684`), `FixRoute` advances it (`:856`). |
| `m_iteration` | `:236` | **dead** | Zeroed at `:44`, never touched again. |
| `m_world` | `:239` | live | The branch `initPlacement` made. |
| `m_p_start` | `:242` | **dead** | Never written and never read. |
| `m_shove` | `:245` | live | Rebuilt by `initPlacement` (`:673`) and again at the end of `FixRoute` (`:861`). |
| `m_currentNode` | `:248` | live | |
| `m_lastNode` | `:251` | live | The per move branch `Move` makes for display. |
| `m_lastFixNode` | `:252` | live | The node `CommitPlacement` commits. |
| `m_sizes` | `:254` | live | |
| `m_placingVia` | `:257` | live | |
| `m_viaDiameter`, `m_viaDrill`, `m_currentWidth` | `:260`, `:263`, `:266` | **dead** | All three zeroed in the constructor and never read; the placer goes through `m_sizes` everywhere. |
| `m_currentLayer` | `:268` | live | |
| `m_startsOnVia` | `:270` | **dead** | Zeroed at `:55`, never touched again. |
| `m_orthoMode` | `:271` | **dead** | Written at `:56`, `:86` and `:659`; never read. |
| `m_snapOnTarget` | `:272` | live | Whether `routeHead` found a target pair (`:694`) or fell back to the cursor (`:728`). Decides whether `FixRoute` ends the session. |
| `m_currentEnd`, `m_currentStart` | `:274` | live | |
| `m_currentTrace` | `:275` | live | The pair being placed. |
| `m_currentTraceOk` | `:276` | live | Sticky "a fit has succeeded at least once this leg"; see E12. |
| `m_currentEndItem` | `:278` | live | |
| `m_idle` | `:280` | live | |
| `m_hasFixedAnything` | `:281` | live | Gates the connected-track-width preservation in `UpdateSizes` (`:793`). |

`setInitialDirection` (`pns_diff_pair_placer.h:196`) is declared and **never defined**; `LINE_PLACER` has one with the same name (`pns_line_placer.h:280`) and that one is real. Section 13 erratum E2.

Note there is no `m_currentTrace` reset in `AbortPlacement` and no `m_idle = true` either, so an aborted placement leaves the placer claiming to be busy; the router destroys the placer instead (`pns_router.cpp:967` onwards), which is why nobody notices.

### 5.2 `FindDpPrimitivePair` and `getDanglingAnchor`

`pcbnew/router/pns_diff_pair_placer.cpp:515`, a **public static** so `ROUTER::isStartingPointRoutable` can call it before any placer exists (`pns_router.cpp:352`).

```
FindDpPrimitivePair(world, p, item, out pair, out errorMsg) -> bool:
    if not world->GetRuleResolver()->DpNetPair(item, netP, netN):        :520
        errorMsg = "Unable to find complementary differential pair nets. Make sure
                    the names of the nets belonging to a differential pair end with
                    either N/P or +/-."                                  :526
        return false

    refNet     = item->Net()                                             :533
    coupledNet = (refNet == netP) ? netN : netP                          :534
    refAnchor  = getDanglingAnchor(world, item)                          :536
    if not refAnchor:
        errorMsg = "Can't find a suitable starting point.  If starting from an
                    existing differential pair make sure you are at the end."  :543
        return false

    coupledItems = world->AllItemsInNet(coupledNet)                      :553
    bestDist = infinity ; found = false
    for cand in coupledItems:                                            :557
        if cand->Kind() != item->Kind(): continue                        :559
        anchor = getDanglingAnchor(world, cand)                          :561
        if not anchor: continue
        dist = |*anchor - *refAnchor|                                    :566
        shapeMatches = not (cand is SOLID_T or VIA_T and cand->Layers() != item->Layers())  :570
        if dist < bestDist and shapeMatches:                             :575
            found = true ; bestDist = dist
            if refNet != netP: pair = DP_PRIMITIVE_PAIR(cand, item)  ; anchors (*anchor, *refAnchor)
            else:              pair = DP_PRIMITIVE_PAIR(item, cand)  ; anchors (*refAnchor, *anchor)
    if not found:
        errorMsg = "Can't find a suitable starting point for coupled net \"%s\"."  :598
        return false
    return true
```

`getDanglingAnchor` (`:462`) is a free function with external linkage in the same file:

| Kind | Line | Answer |
| --- | --- | --- |
| `LINE_T` | `:466` | `CPoint(0)`, or none when the line has no points. |
| `VIA_T`, `SOLID_T` | `:475` | `Anchor(0)`, unconditionally. |
| `ARC_T` | `:479` | `GetP0()` if the joint at anchor 0 has exactly one link, else `GetP1()` if the joint at anchor 1 does, else none. |
| `SEGMENT_T` | `:493` | `Seg().A` or `Seg().B` under the same one-link test. |
| anything else | `:508` | none. |

So "dangling" means "an end of this track that nothing else connects to", which is why the error message tells the user to click at the end of an existing pair. For pads and vias there is no such test: any pad qualifies, and `Anchor(0)` is its centre.

Three properties a port has to preserve. The **kind equality** test at `:559` means a pad can only pair with a pad and a segment only with a segment; it is what keeps `BuildFromPrimitivePair`'s unhandled mixed case unreachable. The **layer equality** test at `:570` applies to solids and vias only, so two segments on different layers can pair. And the search is a linear scan over every item in the coupled net with a `dist < bestDist` strict comparison, so on an exact tie the **first** item in `AllItemsInNet`'s order wins. `NODE::AllItemsInNet` fills a `std::set<ITEM*>`, which is ordered by pointer, so KiCad's tie break is allocation order. Section 13 erratum E8: the port must tie break on something stable, and note 02's `(distance, uid)` convention is the obvious choice.

The polarity handling at `:580` uses `refNet != netP` rather than `DpNetPolarity`, so `FindDpPrimitivePair` needs only `DpNetPair`. `DpNetPolarity` is used by the topology, not by the placer (section 7.4).

### 5.3 `Start`, `initPlacement`, `setWorld`

```
Start(p, startItem) -> bool:                                             :622
    setWorld(Router()->GetWorld())                                       :626
    m_currentNode = m_world                                              :627
    if not FindDpPrimitivePair(m_currentNode, p, startItem, m_start, &err):  :631
        Router()->SetFailureReason(err); return false
    m_netP = m_start.PrimP()->Net()                                      :637
    m_netN = m_start.PrimN()->Net()                                      :638
    m_currentStart = m_currentEnd = p                                    :640
    m_placingVia = false ; m_chainedPlacement = false                    :642
    m_hasFixedAnything = false ; m_currentTraceOk = false                :644
    m_currentTrace = DIFF_PAIR()                                         :646
    m_currentTrace.SetNets(m_netP, m_netN)                               :647
    m_lastFixNode = nullptr                                              :648
    initPlacement()                                                      :650
    return true

initPlacement():                                                        :656
    m_idle = false ; m_orthoMode = false                                 :658
    m_currentEndItem = nullptr                                           :660
    m_startDiagonal = m_initialDiagonal                                  :661
    world = Router()->GetWorld()                                         :663
    world->KillChildren()                                                :665
    rootNode = world->Branch()                                           :666
    setWorld(rootNode)                                                   :668
    m_lastNode = nullptr ; m_currentNode = rootNode                      :670
    m_shove = make_unique<SHOVE>(m_currentNode, Router())                :673
```

Differences from `LINE_PLACER::Start` (note 03 section 3.2) worth listing:

- **No snapping and no splitting.** `LINE_PLACER::Start` calls `splitAdjacentSegments` and `SetupOptimizerNode`-ish work; the pair placer does neither. A pair started in the middle of an existing pair does not break the existing tracks.
- **`m_start` is found on the root world, not on the branch.** `:627` sets `m_currentNode = m_world` (the router's root) and the search runs there; `initPlacement` then branches and overwrites `m_currentNode` (`:671`). The `ITEM*` clones inside `m_start` therefore point at clones of root items, which is fine because `DP_PRIMITIVE_PAIR` owns clones, but the anchors were taken before the branch existed.
- **`m_prevPair` is not reset.** `Start` never touches it; `routeHead` seeds it from `m_start` only when it is empty (`:684`). The router constructs a fresh placer per `StartRouting` (`pns_router.cpp:448`), so `m_prevPair` is always empty at that point, but nothing in the placer enforces it.
- **`m_start` is not consulted for the layer.** `SetLayer` is called by the router before `Start` (`pns_router.cpp:468`), so `m_currentLayer` is whatever the host asked for and no check ties it to the primitives' layers.
- `initPlacement` calls `world->KillChildren()` on the router's **root**, which destroys every branch anyone else holds. `LINE_PLACER` does the same (note 03 section 1.2), so this is shared behaviour, not a pair quirk.

### 5.4 Sizes: `gap()`, `viaGap()`, `makeVia`, `UpdateSizes`

```cpp
int DIFF_PAIR_PLACER::viaGap() const { return m_sizes.EffectiveDiffPairViaGap(); }   // :610
int DIFF_PAIR_PLACER::gap() const
{
    return m_sizes.DiffPairGap() + m_sizes.DiffPairWidth();                          // :616
}
```

`gap()` is centre to centre (section 1.5). `viaGap()` goes through `SIZES_SETTINGS::EffectiveDiffPairViaGap` (`pns_sizes_settings.h:146`):

```cpp
int annularRing = ( ViaDiameter() - ViaDrill() ) / 2;
return std::max( { DiffPairViaGap(),
                   GetDiffPairHoleToHole() - 2 * annularRing,
                   GetDiffPairCopperToHole() - annularRing } );
```

that is, the copper edge to copper edge gap needed so that neither the hole to hole nor the copper to hole rule is violated between the two vias of the pair. `DiffPairViaGap()` itself (`:87`) returns `DiffPairGap()` while `m_diffPairViaGapSameAsTraceGap` is set, and the host clears that flag in `ImportSizes` (`pns_kicad_iface.cpp:1285`).

`makeVia` (`:74`) asks the interface for the layer range and builds a `VIA` from the sizes; identical in shape to `LINE_PLACER::makeVia`.

`UpdateSizes` (`:782`):

```
UpdateSizes(sizes):
    prevDiffPairWidth = m_sizes.DiffPairWidth()                          :784
    m_sizes = sizes                                                      :786
    if not m_idle:
        if not m_sizes.TrackWidthIsExplicit() and m_hasFixedAnything:    :793
            m_sizes.SetDiffPairWidth(prevDiffPairWidth)                  :794
        m_currentTrace.SetWidth(m_sizes.DiffPairWidth())                 :796
        m_currentTrace.SetGap(m_sizes.DiffPairGap())                     :797
        if m_currentTrace.EndsWithVias():                                :799
            m_currentTrace.SetViaDiameter(m_sizes.ViaDiameter())         :801
            m_currentTrace.SetViaDrill(m_sizes.ViaDrill())               :802
```

The guard at `:793` is the pair analogue of `LINE_PLACER::UpdateSizes`'s, with the comment at `:790` saying so: in "use connected track width" mode, once a leg has been fixed the inherited width must not revert to the netclass value. Note `SetGap` here writes the **edge to edge** value even if the last `routeHead` left the centre to centre one in place, so an `UpdateSizes` between two moves silently repairs the state E12 describes.

### 5.5 `routeHead`: from cursor to a fitted pair

`pcbnew/router/pns_diff_pair_placer.cpp:677`. The core of the placer.

```
routeHead(p) -> bool:
    m_fitOk = false                                                      :679
    gwsEntry  = DP_GATEWAYS(gap())                                       :681
    gwsTarget = DP_GATEWAYS(gap())                                       :682
    if not m_prevPair: m_prevPair = m_start                              :684
    gwsEntry.BuildFromPrimitivePair(*m_prevPair, m_startDiagonal)        :687

    if FindDpPrimitivePair(m_currentNode, p, m_currentEndItem, target):  :691
        gwsTarget.BuildFromPrimitivePair(target, m_startDiagonal)        :693
        m_snapOnTarget = true                                            :694
    else:
        if not propagateDpHeadForces(p, fp): return false                :700
        m_prevPair->CursorOrientation(fp, midp, dirV)                    :704
        fpProj    = SEG(midp, midp + dirV).LineProject(fp)               :706
        lead_dist = |fpProj - fp|                                        :710
        gwsTarget.SetFitVias(m_placingVia, m_sizes.ViaDiameter(), viaGap())  :712
        if lead_dist > (DiffPairGap() + DiffPairWidth()) / 2:            :715
            gwsTarget.BuildForCursor(fp)                                 :717
        else:
            gwsTarget.BuildForCursor(fpProj)                             :723
            gwsTarget.FilterByOrientation(ANG_STRAIGHT | ANG_HALF_FULL,
                                          DIRECTION_45(dirV))            :724
        m_snapOnTarget = false                                           :728

    m_currentTrace.SetGap(gap())                                         :731
    m_currentTrace.SetLayer(m_currentLayer)                              :732
    result = gwsEntry.FitGateways(gwsEntry, gwsTarget, m_startDiagonal, m_currentTrace)  :734
    if result:
        m_currentTraceOk = true                                          :738
        m_currentTrace.SetNets(m_netP, m_netN)                           :739
        m_currentTrace.SetWidth(m_sizes.DiffPairWidth())                 :740
        m_currentTrace.SetGap(m_sizes.DiffPairGap())                     :741
        if m_placingVia:
            m_currentTrace.AppendVias(makeVia(CP().CLastPoint(), m_netP),
                                      makeVia(CN().CLastPoint(), m_netN))  :745
        else:
            m_currentTrace.RemoveVias()                                  :750
        return true
    return m_currentTraceOk                                              :756
```

Five points.

**The target is found by the same routine as the start.** `FindDpPrimitivePair( m_currentNode, aP, m_currentEndItem, target )` with the error message pointer left null. So hovering over one pad of a destination pair snaps the whole pair to it, and hovering over anything that is not half of a pair falls through to the cursor branch. `m_currentEndItem` is whatever the host passed to `Move` and may be null, in which case `DpNetPair` returns false immediately (`pns_kicad_iface.cpp:1379`) and the cursor branch runs.

**The `lead_dist` test is the pair's posture rule.** `dirV` is the direction of travel from `CursorOrientation`; `fpProj` is the cursor projected onto the line through the pair midpoint along that direction. When the cursor is far off that line (more than half the centre to centre gap), gateways are built at the cursor and the pair is free to turn. When it is close, gateways are built at the **projection** and then every gateway whose anchor line runs along the direction of travel is filtered out, which forces the pair to stay straight and end as close to the cursor as the 45 degree regime allows. This is the pair equivalent of `LINE_PLACER`'s posture solver, and it is far simpler: there is no mouse trail, no hysteresis and no lock.

**`SetFitVias` is called on the target set only** (`:712`), and only in the cursor branch. Placing a via while snapped to a target pair therefore builds the target gateways with the trace gap rather than the via gap, and `BuildForCursor`'s via expansion never runs. The vias are still appended at `:745`.

**The failure return is sticky.** `return m_currentTraceOk` at `:756` means "the fit failed, but if an earlier move in this leg succeeded, report success and leave the old shape in `m_currentTrace`". The caller (`rhMarkObstacles`, `rhWalkOnly`, `rhShoveOnly`) then proceeds to collision test or walk **the stale shape**. Section 13 erratum E12; note 03 section 9.6's list of "behaviours to reproduce deliberately" is where this belongs.

**`SetLayer` on the trace** (`:732`) is `ITEM::SetLayer`, giving the pair a single layer range; `updateLine` reads `Layers().Start()` back out for each line (`pns_diff_pair.h:550`). The vias get their range from `GetViaLayerRange( m_sizes )` instead (`:76`).

### 5.6 `propagateDpHeadForces`: walking the cursor out of obstacles

`pcbnew/router/pns_diff_pair_placer.cpp:118`. The pair's answer to "the cursor is inside something, where should the head actually go". The comment at `:144` calls it lazy and the one at `:146` says it is lifted from `VIA::PushoutForce` and specialised.

```
propagateDpHeadForces(p, out newP) -> bool:
    virtHead = makeVia(p, nullptr)                                       :120
    if m_placingVia:
        virtHead.SetDiameter(0, viaGap() + 2 * virtHead.Diameter(0))     :124
    else:
        virtHead.SetLayer(m_currentLayer)                                :128
        virtHead.SetDiameter(0, DiffPairGap() + 2 * DiffPairWidth())     :129

    solidsOnly = true                                                    :132
    if Settings().Mode() == RM_MarkObstacles: newP = p; return true      :134
    elif Settings().Mode() == RM_Walkaround:  solidsOnly = false         :139

    maxIter = 40 ; iter = 0 ; collided = false                           :150
    force, totalForce : VECTOR2I                                         :153
    handled : set<const ITEM*>                                           :154
    while iter < maxIter:                                                :156
        obs = m_currentNode->CheckColliding(&virtHead,
                                            solidsOnly ? SOLID_T : ANY_T)  :158
        if not obs or handled.count(obs->m_item): break                  :161
        clearance = m_currentNode->GetClearance(obs->m_item,
                                                &m_currentTrace.PLine(), false)  :164
        collided = false                                                 :166
        for viaLayer in virtHead.RelevantShapeLayers(obs->m_item):       :168
            collided |= obs->m_item->Shape(viaLayer)->Collide(
                            virtHead.Shape(viaLayer), clearance, &layerForce)  :170
            if |layerForce|^2 > |force|^2: force = layerForce            :173
        if collided:
            totalForce += force                                          :179
            virtHead.SetPos(virtHead.Pos() + force)                      :180
        handled.insert(obs->m_item)                                      :183
        iter++
    succeeded = (not collided or iter != maxIter)                        :188
    if succeeded: newP = p + force ; return true                         :192
    return false
```

The virtual head is a circle whose diameter is the **whole pair's width**: `gap + 2 * width` for tracks (`:129`), or `viaGap + 2 * viaDiameter` for a via pair (`:124`). Approximating the pair as one fat round object is exactly what the comment at `:144` admits to.

Three defects to carry knowingly or fix. `force` is declared outside the loop and only ever replaced when a longer one appears (`:173`), so it is a running maximum across all iterations and all layers, never reset; `virtHead` is displaced by that running maximum on every iteration, so it walks further than any single obstacle asked for. `totalForce` accumulates the same values and is **never read**. And the answer at `:192` is `aP + force`, a single application of the running maximum, which does not equal `virtHead.Pos()` (`aP` plus the sum) whenever more than one iteration ran. Section 13 erratum E9.

The clearance at `:164` is resolved between the obstacle and the **P line**, not between the obstacle and the virtual head, with the comment at `:148` explaining why: a via's resolved clearance to an item can differ from the pair's, and the point of the routine is to respect the pair's.

In mark obstacles mode the whole loop is skipped and the cursor is used as is (`:134`), which is what makes that mode's preview follow the mouse exactly.

### 5.7 `attemptWalk` and `tryWalkDp`

`attemptWalk` (`pcbnew/router/pns_diff_pair_placer.cpp:200`) is the pair's walkaround: walk one lane around the obstacle, then shove the other lane away from the walked one so the gap is preserved, and alternate.

```
attemptWalk(node, current, out walk, pFirst, windCw, solidsOnly) -> bool:
    walkaround = WALKAROUND(node, Router())                              :203
    walkaround.SetSolidsOnly(solidsOnly)                                 :205
    walkaround.SetIterationLimit(Settings().WalkaroundIterationLimit())   :206
    walkaround.SetAllowedPolicies({ WP_SHORTEST })                       :207
    shove = SHOVE(node, Router())                                        :209
    walk = *current                                                      :212
    cur  = DIFF_PAIR(*current)                                           :216
    currentIsP = pFirst ; iter = 0                                       :218, :214
    mask = solidsOnly ? SOLID_T : ANY_T                                  :220

    do:                                                                  :222
        preWalk  = currentIsP ? cur.PLine() : cur.NLine()                :224
        preShove = currentIsP ? cur.NLine() : cur.PLine()                :225
        if not node->CheckColliding(&preWalk, mask):                     :228
            currentIsP = !currentIsP                                     :230
            if not node->CheckColliding(&preShove, mask): break          :232
            else: continue                                               :235
        wf1 = walkaround.Route(preWalk)                                  :238
        if wf1.status[WP_SHORTEST] != ST_DONE: return false              :240
        postWalk = wf1.lines[WP_SHORTEST]                                :243
        postShove = LINE(preShove)                                       :245
        shove.ForceClearance(true, cur.Gap() - 2 * PNS_HULL_MARGIN)      :247
        if not shove.ShoveObstacleLine(postWalk, preShove, postShove): return false  :251
        postWalk.Line().Simplify()                                       :256
        postShove.Line().Simplify()                                      :257
        cur.SetShape(postWalk.CLine(), postShove.CLine(), !currentIsP)   :259
        currentIsP = !currentIsP                                         :261
        if not node->CheckColliding(&postShove, mask): break             :263
        iter++
    while iter < 3                                                       :268

    if iter == 3: return false                                           :270
    walk.SetShape(cur.CP(), cur.CN())                                    :273
    return true
```

Notes. `aWindCw` is **never used in the body**; section 13 erratum E3. The `continue` at `:235` jumps to the `while` condition without incrementing `iter`, so a pair where neither lane collides on the first pass exits through the `break` at `:233`, and a pair where the non-walk lane collides but the walk lane does not loops with the roles swapped, still without incrementing `iter`; that cannot spin forever because the swap is its own inverse and the second visit takes the other branch. The `SetShape( ..., !currentIsP )` at `:259` uses the swap-lanes overload so that `postWalk` lands in P when `currentIsP` and in N otherwise.

`ForceClearance( true, cur.Gap() - 2 * PNS_HULL_MARGIN )` (`:247`) is the whole coupling mechanism: the shove is told to use the pair's copper gap, minus 20 nm of hull slack, as the clearance between the two lanes, regardless of what the rule resolver would say about two different nets. `PNS_HULL_MARGIN` is 10 (`pns_line.h:45`). `cur.Gap()` must be edge to edge here, which it is after a successful `routeHead` and is not after a failed one (E12).

A fresh `WALKAROUND` **and a fresh `SHOVE`** are constructed per call (`:203`, `:209`), and `tryWalkDp` calls `attemptWalk` four times per move, each on its own branch. That is four `SHOVE` constructions and four `NODE::Branch()` per mouse move on top of whatever the mode does afterwards.

`tryWalkDp` (`:279`):

```
tryWalkDp(aNode, pair, solidsOnly) -> bool:
    best : DIFF_PAIR ; bestScore = 1e14                                  :281, :282
    for attempt in 0..3:                                                 :284
        p = DIFF_PAIR() ; tmp = m_currentNode->Branch()                  :286, :287
        pfirst  = attempt & 1                                            :289
        wind_cw = attempt & 2                                            :290
        if attemptWalk(tmp, &pair, p, pfirst, wind_cw, solidsOnly):      :292
            cl    = 1 + p.CoupledLength()                                :294
            skew  = p.Skew()                                             :295
            score = cl + |skew| * 3.0                                    :297
            if score < bestScore: bestScore = score ; best = move(p)     :299
        delete tmp                                                       :306
    if bestScore > 0.0:                                                  :309
        optimizer = OPTIMIZER(m_currentNode)                             :311
        pair.SetShape(best)                                              :313
        optimizer.Optimize(&pair)                                        :314
        return true
    return false
```

`aNode` is **never used**; the body goes through `m_currentNode` (`:287`, `:311`). Both call sites pass `m_currentNode` anyway (`:328`, `:363`). Section 13 erratum E3.

Because `aWindCw` is inert, the four attempts are two distinct computations run twice each, and the `score < bestScore` strict comparison means the **first** of each duplicated pair wins. So `tryWalkDp` is exactly "try P first, try N first, keep the better", at twice the cost.

The score `1 + CoupledLength() + 3 * |Skew()|` is **minimised** (`:299`). Minimising skew is right; minimising coupled length is the opposite of what every other coupled-length comparison in the tree does (`coupledBypass` keeps `coupledLength > bestLength`, `pns_optimizer.cpp:1408`; `mergeDpStep` accepts only when the coupled length does not drop, `:1471`). Section 13 erratum E11.

`if( bestScore > 0.0 )` at `:309` is **always true**, including when every attempt failed: `bestScore` starts at `1e14` and `cl` is at least 1 so no successful attempt can bring it to zero either. So `tryWalkDp` always returns true, and when nothing succeeded it assigns `best`, a default constructed `DIFF_PAIR` with two empty chains, into `pair` and optimises that. `m_fitOk` is then true with an empty trace. Section 13 erratum E11, and this is the most consequential defect in the file: it is what makes walk mode report success on a pair it could not route.

`OPTIMIZER::Optimize( DIFF_PAIR* )` is called with the optimizer's **default** effort level; section 6 shows the pair path ignores it entirely.

### 5.8 The three mode routines

```
route(p):                                                                :334
    switch Settings().Mode():
        RM_MarkObstacles: return rhMarkObstacles(p)                      :338
        RM_Walkaround:    return rhWalkOnly(p)                           :340
        RM_Shove:         return rhShoveOnly(p)                          :342
        default:          return false                                   :344
```

**`rhMarkObstacles`** (`:104`):

```
if not routeHead(p): return false
collP = m_currentNode->CheckColliding(&m_currentTrace.PLine())
collN = m_currentNode->CheckColliding(&m_currentTrace.NLine())
m_fitOk = not (collP or collN)
return m_fitOk
```

It returns `false` when the pair collides, where `LINE_PLACER::rhMarkObstacles` returns `true` and lets the host paint the violations (note 03 section 3.6). `ROUTER::movePlacing` passes the bool straight through to the host as the return of `Move` (`pns_router.cpp:793`, `:829`), and `ROUTER_TOOL` does not act on it, so the visible difference is nil; the difference that matters is `FixRoute`'s gate on `m_fitOk` (`:810`), which in mark obstacles mode refuses to commit a colliding pair unless `AllowDRCViolations()`. For single tracks the same gate exists but `m_fitOk`'s meaning differs.

**`rhWalkOnly`** (`:323`):

```
if not routeHead(p): return false
m_fitOk = tryWalkDp(m_currentNode, m_currentTrace, false)
return m_fitOk
```

`aSolidsOnly = false`, so the walkaround walks around everything. Given E11, `m_fitOk` is always true here.

**`rhShoveOnly`** (`:352`):

```
m_currentNode = m_shove->CurrentNode()                                   :354
ok = routeHead(p)                                                        :356
m_fitOk = false                                                          :358
if not ok: return false                                                  :360
if not tryWalkDp(m_currentNode, m_currentTrace, true): return false      :363
pLine = m_currentTrace.PLine() ; nLine = m_currentTrace.NLine()          :366
m_shove->ClearHeads()                                                    :370
m_shove->AddHeads(pLine)                                                 :371
m_shove->AddHeads(nLine)                                                 :372
status = m_shove->Run()                                                  :374
m_currentNode = m_shove->CurrentNode()                                   :376
if status == SH_OK:
    if m_shove->HeadsModified(0): pLine = m_shove->GetModifiedHead(0)    :382
    if m_shove->HeadsModified(1): nLine = m_shove->GetModifiedHead(1)    :385
    m_currentTrace.SetShape(pLine.CLine(), nLine.CLine())                :389
    if not colliding(pLine) and not colliding(nLine): m_fitOk = true     :391
else:
    m_currentTrace.SetShape(pLine.CLine(), nLine.CLine())                :400
return m_fitOk
```

The pre-shove `tryWalkDp` with `aSolidsOnly = true` is the pair analogue of `LINE_PLACER`'s "walk around solids first, then shove the rest" (note 04 section 5.2). Both lanes go in as heads through `AddHeads( const LINE&, int aPolicy = SHP_DEFAULT )` (`pns_shove.h:81`), in P then N order, and the two modified heads come back by index. The placer never calls `SetDefaultShovePolicy`, so the policy is the constructor's `SHP_SHOVE` (`pns_shove.cpp:209`); unlike the draggers it does not add `SHP_DONT_LOCK_ENDPOINTS` (note 04 sections 5.3 and 5.4), so both lane endpoints are pinned during the shove. The `ITEM_SET head;` declared at `:368` is never used.

Two differences from the single track placer. There is **no fallback to walk mode** on shove failure: `LINE_PLACER::rhShoveOnly` ends with `return rhWalkOnly( aP )` (note 04 section 5.2, `pns_line_placer.cpp:1013`); the pair placer just reports `m_fitOk == false`. And there is **no springback locking**: `AddLockedSpringbackNode`, `RewindSpringbackTo`, `UnlockSpringbackNode` and `RewindToLastLockedNode` appear nowhere in `pns_diff_pair_placer.cpp`, so the pair placer never pins a shove result and `CommitPlacement` cannot rewind to one (section 5.10).

The `else` branch at `:397` is labelled "bring back previous state" but `pLine` and `nLine` still hold the pre-shove copies taken at `:366`, and `m_currentTrace` was never modified by the shove, so the assignment restores what is already there. Harmless, and a port should just drop the branch (recording the decision).

### 5.9 `Move`

```
Move(p, endItem) -> bool:                                                :760
    m_currentEndItem = endItem                                           :762
    m_fitOk = false                                                      :763
    delete m_lastNode ; m_lastNode = nullptr                             :765
    retval = route(p)                                                    :768
    latestNode = m_currentNode                                           :770
    m_lastNode = latestNode->Branch()                                    :771
    assert(m_lastNode != nullptr)                                        :773
    m_currentEnd = p                                                     :774
    updateLeadingRatLine()                                               :776
    return retval
```

Shorter than `LINE_PLACER::Move` (note 03 section 3.4) because there is no mouse trail to feed and no tail to reduce. There is no `m_idle` guard, so `Move` on an idle placer dereferences a null `m_currentNode` at `:771`; the router never does that because `m_state` gates it (`pns_router.cpp:494`).

The unconditional `Branch()` at `:771` allocates a node per mouse move whether or not anything changed, and `updateLeadingRatLine` runs a `TOPOLOGY` over it. Note 03 section 1.2 covers the same pattern for the single placer.

### 5.10 `FixRoute`

`pcbnew/router/pns_diff_pair_placer.cpp:808`.

```
FixRoute(p, endItem, forceFinish) -> bool:
    if not m_fitOk and not Settings().AllowDRCViolations(): return false  :810
    if CP().SegmentCount() < 1 or CN().SegmentCount() < 1: return false   :813
    if CP().SegmentCount() > 1:
        m_initialDiagonal = not DIRECTION_45(CP().CSegment(-2)).IsDiagonal()  :817
    topo = TOPOLOGY(m_lastNode)                                           :819

    # drop the last segment unless this fix ends the route
    if not m_snapOnTarget and not EndsWithVias() and not forceFinish
       and not Settings().GetFixAllSegments():                            :821
        newP = CP() ; newN = CN()
        if newP.SegmentCount() > 1 and newN.SegmentCount() > 1:            :827
            newP.Remove(-1, -1) ; newN.Remove(-1, -1)                      :829
        m_currentTrace.SetShape(newP, newN)                                :833

    if EndsWithVias():                                                     :836
        m_lastNode->Add(Clone(PLine().Via()))                              :838
        m_lastNode->Add(Clone(NLine().Via()))                              :839
        m_chainedPlacement = false                                         :840
    else:
        m_chainedPlacement = not m_snapOnTarget and not forceFinish        :844

    lineP = PLine() ; lineN = NLine()                                      :847
    m_lastNode->Add(lineP)                                                 :850
    m_lastNode->Add(lineN)                                                 :851
    topo.SimplifyLine(&lineP)                                              :853
    topo.SimplifyLine(&lineN)                                              :854
    m_prevPair = m_currentTrace.EndingPrimitives()                         :856
    m_lastFixNode = m_lastNode                                             :857

    # "avoid an use-after-free error (CommitPlacement calls NODE::Commit which will
    #  invalidate the shove heads state. Need to rethink the memory management)."   :859
    if Settings().Mode() == RM_Shove:
        m_shove = make_unique<SHOVE>(m_world, Router())                    :861
    CommitPlacement()                                                      :863
    m_placingVia = false                                                   :864
    m_lastFixNode = nullptr                                                :865
    if m_snapOnTarget or forceFinish:
        m_idle = true ; return true                                        :869
    else:
        m_hasFixedAnything = true ; initPlacement() ; return false          :874
```

The return contract is the same as the single placer's: `true` means "the session is over", `false` means "a leg was committed and another begins" (note 03 section 3.10). `ROUTER::FixRoute` hands it straight to the host (`pns_router.cpp:925`).

The last segment is dropped so the next leg starts from a corner the user has actually committed to, exactly as `LINE_PLACER` does, but the guard at `:827` requires **both** chains to have more than one segment; a pair where one lane came out with a single segment keeps its last segment on both lanes.

`m_prevPair = m_currentTrace.EndingPrimitives()` (`:856`) is how the next leg knows where to start. `EndingPrimitives` (`pns_diff_pair.cpp:764`) returns a via pair when the trace ends with vias, and otherwise builds two throwaway `SEGMENT`s from the last segment of each lane and sets the anchors to their `B` ends. Those segments are stack objects; `DP_PRIMITIVE_PAIR`'s constructor clones them, so nothing dangles.

The shove is thrown away and rebuilt over `m_world` (`:861`) rather than being rewound, with a comment admitting the memory management is the reason. `LINE_PLACER::CommitPlacement` instead calls `RewindToLastLockedNode()` (`pns_line_placer.cpp:1814`). Because the pair placer never locks a springback node, rewinding would have nothing to rewind to, so the two are consistent in effect: a fixed leg drops the shove's history entirely.

`CommitPlacement` (`:895`) is three lines: commit `m_lastFixNode` through the router if set, then null all three node members. `AbortPlacement` (`:881`) kills the root's children and nulls `m_lastNode`; it does **not** set `m_idle`. `HasPlacedAnything` (`:889`) is `CP().SegmentCount() > 0 || CN().SegmentCount() > 0`, an `or` where the pair is only meaningful with both.

### 5.11 The small commands

| Method | Line | Behaviour |
| --- | --- | --- |
| `ToggleVia( bool )` | `:93` | Sets `m_placingVia`, re-runs `Move( m_currentEnd, nullptr )` when not idle, always returns true. Note the re-run passes a null end item, so a via toggled while hovering a target pair loses the snap for that frame. |
| `SetLayer( int )` | `:437` | Idle: store and succeed. Otherwise refuse if `m_chainedPlacement` or `m_prevPair` is empty (`:444`); otherwise succeed only if `m_prevPair->PrimP()` is null **or** is a via whose layers overlap the target (`:448`), in which case it restarts the leg from `m_prevPair` with `initPlacement()` and a `Move`. So a layer change is possible exactly at a via pair or at a cursor-only start, which is the pair analogue of note 03 section 3.3. |
| `FlipPosture()` | `:419` | Toggles `m_startDiagonal` and re-runs `Move`. |
| `SetOrthoMode( bool )` | `:84` | Stores into `m_orthoMode`, which nothing reads, and re-runs `Move`. Inert except for the re-run. |
| `Traces()` | `:408` | An `ITEM_SET` of `&PLine()` and `&NLine()`, that is, pointers into `m_currentTrace`'s two cached `LINE`s. Valid until the next call to either accessor. |
| `CurrentNode( bool )` | `:428` | `m_lastNode` when set, else `m_currentNode`. The `aLoopsRemoved` parameter is ignored, as it is in `LINE_PLACER`. |
| `CurrentNets()` | `:927` | `{ m_netP, m_netN }`. |
| `GetModifiedNets()` | `:907` | Pushes the same two. |
| `updateLeadingRatLine()` | `:914` | One `TOPOLOGY` over `m_lastNode`, `LeadingRatLine` for each lane, each drawn through `DisplayRatline` with its own net. |
| `UnfixRoute()` | -- | **Not overridden.** `PLACEMENT_ALGO::UnfixRoute` returns `std::nullopt` (`pns_placement_algo.h:81`), so `ROUTER::UndoLastSegment` (`pns_router.cpp:946`) logs an `EVT_UNFIX` and does nothing. |

---

## 6. The optimizer's pair path

`OPTIMIZER::Optimize( DIFF_PAIR* aPair )` (`pcbnew/router/pns_optimizer.cpp:1538`) is two lines:

```cpp
bool OPTIMIZER::Optimize( DIFF_PAIR* aPair )
{
    return mergeDpSegments( aPair );
}
```

**No effort flag reaches it.** `m_effortLevel` is never read on this path: `mergeDpSegments`, `mergeDpStep`, `coupledBypass`, `verifyDpBypass`, `findCoupledVertices` and `checkDpColliding` contain no reference to it, and none of them calls `checkConstraints`. So `KEEP_TOPOLOGY`, `PRESERVE_VERTEX`, `RESTRICT_VERTEX_RANGE`, `RESTRICT_AREA`, `LIMIT_CORNER_COUNT`, `SMART_PADS`, `FANOUT_CLEANUP`, `MERGE_COLINEAR` and `REQUIRE_OBTUSE_ANGLES` are all inert for pairs, and so is `SetCollisionMask` (the pair helpers call `NODE::CheckColliding` with its default mask). The answer to the brief's question about diff pair specific optimizer flags is that there are none, in either direction: the pair path neither reads the flags nor adds any of its own.

`OPTIMIZER::Optimize( DIFF_PAIR* )` also never populates the item cache, so the `CACHED_ITEM` machinery (`pns_optimizer.h:165`) is bypassed too.

### 6.1 `mergeDpSegments`

`pcbnew/router/pns_optimizer.cpp:1497`.

```
mergeDpSegments(pair):
    step_p = pair->CP().SegmentCount() - 2                               :1499
    step_n = pair->CN().SegmentCount() - 2                               :1500
    while true:
        max_step_p = pair->CP().SegmentCount() - 2                       :1507
        max_step_n = pair->CN().SegmentCount() - 2                       :1508
        step_p = min(step_p, max_step_p)                                 :1510
        step_n = min(step_n, max_step_n)
        if step_p < 1 and step_n < 1: break                              :1516
        found_p = (step_p > 1) and mergeDpStep(pair, true,  step_p)      :1522
        found_n = (step_n > 1) and mergeDpStep(pair, false, step_n)      :1525
        if not found_p and not found_n: step_p-- ; step_n--              :1528
    return true
```

The same descending-span shape as `mergeFull` (note 04 section 4.4), run independently on the two lanes. Two traps. The loop only terminates through `:1516`, and the counters only decrease at `:1528`, so a `mergeDpStep` that keeps succeeding without shrinking the chain spins forever; nothing bounds it the way `MERGE_PASS_LIMIT` bounds the single line path. And the guards are `step > 1` at `:1522` and `:1525` while the exit test is `step < 1` at `:1516`, so `step == 1` neither steps nor exits, it just decrements both counters once more. Section 13 erratum E14.

### 6.2 `mergeDpStep`

`pcbnew/router/pns_optimizer.cpp:1434`.

```
mergeDpStep(pair, tryP, step) -> bool:
    currentPath = tryP ? pair->CP() : pair->CN()                         :1438
    coupledPath = tryP ? pair->CN() : pair->CP()                         :1439
    n_segs  = currentPath.SegmentCount() - 1                             :1441
    clenPre = pair->CoupledLength(currentPath, coupledPath)              :1443
    budget  = clenPre / 10   # "fixme: come up with something more intelligent"  :1444
    n = 1
    while n < n_segs - step:                                             :1446
        s1 = currentPath.CSegment(n) ; s2 = currentPath.CSegment(n+step) :1448
        if DIRECTION_45(s1).IsObtuse(DIRECTION_45(s2)):                  :1454
            bypass = BuildInitialTrace(s1.A, s2.B, DIRECTION_45(s1).IsDiagonal())  :1456
            newRef = currentPath with [s1.Index() .. s2.Index()] replaced by bypass  :1462
            deltaUni = pair->CoupledLength(newRef, coupledPath) - clenPre + budget   :1465
            if coupledBypass(m_world, pair, tryP, newRef, bypass, coupledPath, newCoup):  :1467
                deltaCoupled = pair->CoupledLength(newRef, newCoup) - clenPre + budget   :1469
                if deltaCoupled >= 0:
                    newRef.Simplify2() ; newCoup.Simplify2()             :1473
                    pair->SetShape(newRef, newCoup, !tryP)               :1476
                    return true
            elif deltaUni >= 0 and verifyDpBypass(m_world, pair, tryP, newRef, coupledPath):  :1480
                newRef.Simplify2() ; coupledPath.Simplify2()             :1482
                pair->SetShape(newRef, coupledPath, !tryP)               :1485
                return true
        n++
    return false
```

The acceptance rule is "the coupled length may drop by at most a tenth of what it was". `budget` is computed once from the pre-merge coupled length and both deltas are `new - old + budget >= 0`.

Note `coupledPath` is a **copy** taken at `:1439`, so `coupledPath.Simplify2()` at `:1483` mutates the copy before it is written back at `:1485`, and the `else if` at `:1480` is reached only when `coupledBypass` failed, in which case the coupled lane keeps its shape apart from the simplify.

### 6.3 `coupledBypass`, `verifyDpBypass`, `findCoupledVertices`, `checkDpColliding`

`findCoupledVertices` (`:1323`) finds, for one vertex of the reference lane, every segment index of the coupled lane that is `ApproxParallel` to the reference segment and at the pair's gap from that vertex:

```
findCoupledVertices(vertex, origSeg, coupled, pair, out indices) -> count:
    for i in coupled.SegmentCount():
        s = coupled.CSegment(i)
        if s.ApproxParallel(origSeg):
            dist = |s.LineProject(vertex) - vertex| - pair->Width()
            if pair->GapConstraint().Matches(dist): indices[count++] = i
```

This is the only reader of `DIFF_PAIR::GapConstraint()` in the tree, and it needs the **edge to edge** value again.

`coupledBypass` (`:1369`) then searches for a bypass on the coupled lane that maximises coupled length:

```
coupledBypass(node, pair, refIsP, ref, refBypass, coupled, out newCoupled) -> bool:
    int vStartIdx[1024]   # "fixme: possible overflow"                   :1373
    nStarts = findCoupledVertices(refBypass.CPoint(0), refBypass.CSegment(0),
                                  coupled, pair, vStartIdx)              :1374
    dir = DIRECTION_45(refBypass.CSegment(0))                            :1377
    bestLength = -1 ; found = false
    for i in 0..nStarts-1:                                               :1384
        for j in 1 .. coupled.PointCount()-2:                            :1386
            if |vStartIdx[i] - j| > 1:                                   :1390
                bypass = dir.BuildInitialTrace(coupled.CPoint(vStartIdx[i]),
                                               coupled.CPoint(j), dir.IsDiagonal())  :1393
                coupledLength = pair->CoupledLength(ref, bypass)         :1396
                newCoupled = coupled with [min..max] replaced by bypass (reversed if needed)  :1398
                if coupledLength > bestLength and verifyDpBypass(node, pair, refIsP, ref, newCoupled):  :1408
                    bestBypass = newCoupled ; bestLength = coupledLength ; found = true
    if found: newCoupled = bestBypass
    return found
```

The `int vStartIdx[1024]` with the acknowledged overflow comment is a fixed C array written by `findCoupledVertices` with no bound; a coupled lane with more than 1024 parallel segments at the gap overruns it. In the port this is a `Vec<usize>` and the comment goes away.

`verifyDpBypass` (`:1350`) rebuilds two `LINE`s from the pair's lanes with the candidate shapes and rejects when they collide with each other or with the node:

```
refLine     = LINE(refIsP ? pair->PLine() : pair->NLine(), newRef)
coupledLine = LINE(refIsP ? pair->NLine() : pair->PLine(), newCoupled)
return not refLine.Collide(&coupledLine, node, refLine.Layer())
       and not node->CheckColliding(&refLine)
       and not node->CheckColliding(&coupledLine)
```

The lane to lane test here is an ordinary clearance test through the rule resolver, not the forced pair gap that `attemptWalk` uses. So the optimizer will happily bring the two lanes closer than the pair gap as long as the netclass clearance allows it; only `CoupledLength` (through `GapConstraint`) pushes back, and only as a score.

`checkDpColliding` (`:1426`) is a one line helper with **no caller**. Section 13 erratum E15.

---

## 7. Pair identification: the three host hooks

### 7.1 The declarations

`RULE_RESOLVER` (`pcbnew/router/pns_node.h:139`) declares three pure virtuals for pairs:

```cpp
virtual NET_HANDLE DpCoupledNet( NET_HANDLE aNet ) = 0;                  // :147
virtual int        DpNetPolarity( NET_HANDLE aNet ) = 0;                 // :148
virtual bool       DpNetPair( const ITEM* aItem, NET_HANDLE& aNetP,
                              NET_HANDLE& aNetN ) = 0;                   // :149
```

They are on the **rule resolver**, not on `ROUTER_IFACE`. That matters for the port: the crate already has them on `RuleResolver` (`src/rules.rs:450`, `:460`, `:468`), which is the right place.

### 7.2 `BOARD::MatchDpSuffix`: KiCad's naming convention

`pcbnew/board.cpp:2780`. This is the whole of KiCad's notion of what a differential pair is. There is no pair object on the board, no pair property on a net and no user declaration; a pair is two nets whose names differ in one suffix character.

```
MatchDpSuffix(netName, out complementNet) -> int:
    rv = 0 ; count = 0
    for ch in reverse(netName), while rv == 0, count++ :                 :2785
        if ch is a digit or '_':   continue                              :2789
        elif ch == '+':  complementNet = "-" ; rv =  1                   :2793
        elif ch == '-':  complementNet = "+" ; rv = -1                   :2798
        elif ch == 'N':  complementNet = "P" ; rv = -1                   :2803
        elif ch == 'P':  complementNet = "N" ; rv =  1                   :2808
        else: break                                                      :2813
    if rv != 0 and count >= 1:                                           :2819
        complementNet = netName.Left(len - count) + complementNet
                        + netName.Right(count - 1)                       :2821
    return rv
```

The return is the **polarity**: `+1` for the positive half, `-1` for the negative half, `0` for "not part of a pair". The scan runs from the end of the name over digits and underscores until it hits a polarity character, so `USB_D+`, `USB_D+_1`, `CLK_P`, `CLK_N_3` and `LVDS0-` all match, and the complement is built by replacing exactly that one character. `count` is the number of characters consumed **including** the polarity character, which is why the tail is `Right( count - 1 )`.

Note `'N'` and `'P'` are matched case sensitively and only as uppercase, and the scan stops at the first non digit, non underscore, non polarity character, so `clk_p` does not match and `DATA0` does not either.

`BOARD::DpCoupledNet` (`:2828`) is `MatchDpSuffix` plus a `FindNet` lookup of the complement name, returning null when either fails.

### 7.3 The three implementations

`pcbnew/router/pns_kicad_iface.cpp`:

```cpp
PNS::NET_HANDLE PNS_PCBNEW_RULE_RESOLVER::DpCoupledNet( PNS::NET_HANDLE aNet )   // :1345
{
    return m_board->DpCoupledNet( static_cast<NETINFO_ITEM*>( aNet ) );
}

int PNS_PCBNEW_RULE_RESOLVER::DpNetPolarity( PNS::NET_HANDLE aNet )              // :1363
{
    wxString refName;
    if( NETINFO_ITEM* net = static_cast<NETINFO_ITEM*>( aNet ) )
        refName = net->GetNetname();
    wxString dummy1;
    return m_board->MatchDpSuffix( refName, dummy1 );
}

bool PNS_PCBNEW_RULE_RESOLVER::DpNetPair( const PNS::ITEM* aItem,
                                          PNS::NET_HANDLE& aNetP,
                                          PNS::NET_HANDLE& aNetN )              // :1376
{
    if( !aItem || !aItem->Net() ) return false;                                 // :1379
    netNameP = aItem->Net()->GetNetname();
    r = m_board->MatchDpSuffix( netNameP, netNameCoupled );                     // :1385
    if( r == 0 )      return false;                                             // :1387
    else if( r == 1 ) netNameN = netNameCoupled;                                // :1391
    else            { netNameN = netNameP; netNameP = netNameCoupled; }         // :1395
    netInfoP = m_board->FindNet( netNameP );
    netInfoN = m_board->FindNet( netNameN );
    if( !netInfoP || !netInfoN ) return false;                                  // :1404
    aNetP = netInfoP; aNetN = netInfoN;
    return true;
}
```

`DpNetPair` is the only one that normalises: it always answers with `aNetP` the positive half and `aNetN` the negative half, whichever half the item belongs to. `DpNetPolarity` returns the raw `MatchDpSuffix` value, so `0` means "not a pair" and the sign means the half. `DpCoupledNet` answers the *other* net without saying which is which.

### 7.4 Who calls what

| Hook | Caller | Line | Why |
| --- | --- | --- | --- |
| `DpNetPair` | `DIFF_PAIR_PLACER::FindDpPrimitivePair` | `pns_diff_pair_placer.cpp:520` | Establish the two nets and reject a non pair item. The only caller. |
| `DpCoupledNet` | `TOPOLOGY::AssembleDiffPair` | `pns_topology.cpp:1039` | Find the coupled net so the other lane's items can be collected. |
| `DpCoupledNet` | `PNS_KICAD_IFACE_BASE::ImportSizes` | `pns_kicad_iface.cpp:1198`, `:1240` | Build a dummy via and a dummy track on the coupled net so the DRC engine can be asked for pair scoped constraints. |
| `DpCoupledNet` | `ROUTER_TOOL::performRouting` | `router_tool.cpp:1720` | Highlight both nets when a pair placement starts. |
| `DpNetPolarity` | `TOPOLOGY::AssembleDiffPair` | `pns_topology.cpp:1160` | Swap the two assembled lines when the start item is the negative half, so `PLine()` really is P. The only caller. |

So the **placer needs only `DpNetPair`**, and needs it to answer for a `SEGMENT`, an `ARC`, a `SOLID` (pad) and a `VIA`. `DpCoupledNet` is needed by the topology and by size import; `DpNetPolarity` is needed only by the topology, that is, only by the length tuners and by `AssembleDiffPair`, which milestone 10 does not require.

For the crate: `RuleResolver::dp_net_pair` (`src/rules.rs:468`) already has the right shape, `Option<(NetId, NetId)>` instead of a bool plus two out parameters. `dp_coupled_net` (`:450`) and `dp_net_polarity` (`:460`) are defaulted to "not supported", which is the correct default for a host that has no pair concept. The pair placer must fail cleanly and with a message when `dp_net_pair` answers `None`, which is what `StartError` needs a new variant for (section 12.4).

### 7.5 `TOPOLOGY::AssembleDiffPair`, for completeness

`pcbnew/router/pns_topology.cpp:1036`. Not needed by the placer, but it is the routine that recovers a `DIFF_PAIR` from committed board items and is therefore what a pair dragger would have to start from, and what the two pair length tuners do start from (`pns_dp_meander_placer.cpp:105`, `pns_meander_skew_placer.cpp:77`).

```
AssembleDiffPair(start, out pair) -> bool:
    refNet     = start->Net()                                            :1038
    coupledNet = resolver->DpCoupledNet(refNet)                          :1039
    if not coupledNet or start is not a LINKED_ITEM: return false        :1042
    lp = world->AssembleLine(startItem)                                  :1045
    pItems = segments and arcs of lp on startItem's layers               :1050
    nItems = every segment or arc of coupledNet on startItem's layers    :1059

    findNItem(p_item):                                                   :1071
        for n_item in nItems with the same Kind:
            if SEGMENT: require equal width, ApproxParallel within
                        DP_PARALLELITY_THRESHOLD (5), and a common
                        parallel projection; dist_sq = seg to seg        :1080
            if ARC:     require equal width, centres within 5,
                        dist_sq = (radius difference)^2                  :1098
            keep the candidate minimising (dist_sq, distance to the
            centre of the start item)                                    :1115

    findNItem(startItem)                                                 :1130
    if nothing found: retry over every link of the joints at both
                      anchors of startItem                               :1132
    if still nothing: return false                                       :1155
    ln = world->AssembleLine(coupledItem)                                :1158
    if resolver->DpNetPolarity(refNet) < 0: swap(lp, ln)                 :1160
    gap = (perpendicular distance between the two reference items)
          - lp.Width()                                                   :1165
    pair = DIFF_PAIR(lp, ln) ; SetWidth ; SetLayers ; SetGap(gap)        :1179
```

Two things to note for a future pair dragger. The **gap is measured, not configured** (`:1170`, `:1176`), so `AssembleDiffPair` describes the pair as built rather than as specified. And the layer test at `:1052` and `:1061` is `item->Layers() == startItem->Layers()`, exact equality of the range, not overlap.

The overload `const DIFF_PAIR TOPOLOGY::AssembleDiffPair( SEGMENT* aStart )` declared at `pns_topology.h:99` **has no definition anywhere in the tree**. Section 13 erratum E16.

---

## 8. The pair constraints

### 8.1 Which types exist

`CONSTRAINT_TYPE` (`pcbnew/router/pns_node.h:51`) has three that are pair specific:

| Type | Line | Host constraint | Queried where |
| --- | --- | --- | --- |
| `CT_DIFF_PAIR_GAP = 2` | `:54` | `DIFF_PAIR_GAP_CONSTRAINT` (`pns_kicad_iface.cpp:552`) | `ImportSizes` (`pns_kicad_iface.cpp:1262`), `ROUTER_TOOL::updateSizesAfterRouterEvent` (`router_tool.cpp:1125`). |
| `CT_DIFF_PAIR_SKEW = 10` | `:62` | `SKEW_CONSTRAINT` (`:554`) | Nowhere in the router. Length tuning only. |
| `CT_MAX_UNCOUPLED = 11` | `:63` | `MAX_UNCOUPLED_CONSTRAINT` (`:555`) | `ROUTER_TOOL::UpdateMessagePanel` (`router_tool.cpp:3475`), **for display only**. |

The crate's `ConstraintType` (`src/rules.rs:190`) already carries all three with matching discriminants: `DiffPairGap = 2` (`:194`), `DiffPairSkew = 10` (`:211`), `MaxUncoupled = 11` (`:214`).

**`CT_MAX_UNCOUPLED` does not influence routing at all.** Its single query site builds a status bar line ("DP Max Uncoupled-length: ...") from `constraint.m_Value.Max()` and the rule name (`router_tool.cpp:3478`). Nothing in `DIFF_PAIR_PLACER`, `DP_GATEWAYS`, `DIFF_PAIR` or `OPTIMIZER` reads it, `DIFF_PAIR::m_maxUncoupledLength` is never read (section 1.4), and no code path rejects a route for being uncoupled too long. The DRC engine catches it after the fact. A port that implements the placer faithfully implements no max-uncoupled enforcement, and should say so in its documentation rather than silently omit it.

`CT_DIFF_PAIR_SKEW` is likewise absent from the placer; `DIFF_PAIR::Skew()` is used only as a term in `tryWalkDp`'s score (`pns_diff_pair_placer.cpp:295`), with the hard coded weight 3.0 and no constraint behind it.

### 8.2 How the gap reaches the placer

The placer never queries a constraint. It reads `m_sizes` and nothing else. The constraint work happens in the host, twice.

**`PNS_KICAD_IFACE_BASE::ImportSizes`** (`pns_kicad_iface.cpp:1224` to `:1285`), run once at the start of a placement:

```
diffPairWidth  = bds.m_TrackMinWidth                                     :1224
diffPairGap    = bds.m_MinClearance                                      :1225
diffPairViaGap = bds.m_MinClearance                                      :1226
found = false
if bds.m_UseConnectedTrackWidth and startItem:
    found = inheritTrackWidth(startItem, &diffPairWidth, startPos)       :1235
if bds.UseNetClassDiffPair() and startItem:                              :1238
    coupledNet = DpCoupledNet(startItem->Net())                          :1240
    dummyTrack   = degenerate SEGMENT at startItem->Anchor(0), start layer, item's net   :1242
    coupledTrack = the same on coupledNet                                :1247
    if not found and QueryConstraint(CT_WIDTH, dummyTrack, coupledTrack, layer):        :1253
        diffPairWidth = max(diffPairWidth, constraint.Opt())             :1256
    if QueryConstraint(CT_DIFF_PAIR_GAP, dummyTrack, coupledTrack, layer):              :1262
        diffPairGap    = max(diffPairGap,    constraint.PinnedOpt())     :1265
        diffPairViaGap = max(diffPairViaGap, constraint.PinnedOpt())     :1266
else:
    diffPairWidth  = bds.GetCurrentDiffPairWidth()                       :1274
    diffPairGap    = bds.GetCurrentDiffPairGap()                         :1275
    diffPairViaGap = bds.GetCurrentDiffPairViaGap()                      :1276
aSizes.SetDiffPairWidth(diffPairWidth)                                   :1282
aSizes.SetDiffPairGap(diffPairGap)                                       :1283
aSizes.SetDiffPairViaGap(diffPairViaGap)                                 :1284
aSizes.SetDiffPairViaGapSameAsTraceGap(false)                            :1285
```

Two structural points. The pair scoped constraints are asked as **two item queries**: a dummy track on the item's net against a dummy track on the coupled net, which is how a DRC rule scoped to "these two nets" gets selected. And the netclass path takes `max( board minimum, rule optimum )` so the board minimums are a floor; the "user choice" path (`:1272`) bypasses the rules entirely.

Below that, the pair via clearances (`:1287` to `:1327`): `CT_HOLE_TO_HOLE` between the two dummy vias gives `SetDiffPairHoleToHole` (`:1303`), and `CT_HOLE_CLEARANCE` plus `CT_PHYSICAL_HOLE_CLEARANCE` in **both orderings** (`:1311` to `:1325`, with the comment at `:1309` explaining that a net scoped rule may bind the two vias asymmetrically) give `SetDiffPairCopperToHole` (`:1327`). Those two feed `EffectiveDiffPairViaGap` (section 5.4).

**`ROUTER_TOOL::updateSizesAfterRouterEvent`** (`router_tool.cpp:1097` to `:1143`), run again on every layer change and size event, re-evaluates `TRACK_WIDTH_CONSTRAINT` and `DIFF_PAIR_GAP_CONSTRAINT` on the current nets (which for a pair is two, `:1097`) and pushes the result through `ROUTER::UpdateSizes` (`:1145`), which forwards to `PLACEMENT_ALGO::UpdateSizes` (`pns_router.cpp:779` onwards). The "only change the size if we are explicitly using the netclass or we are out of range" guard (`:1111`, `:1130`) is what stops a user's manual gap from being clobbered on every layer change.

### 8.3 How the gap combines with the netclass clearance

They do not combine; they are used in different places.

- The **pair gap** is geometry. It sets the spacing of every gateway (`gap()`, section 5.4), the accept threshold in `checkGap`, the `GapConstraint` that decides which segments count as coupled, and the forced clearance `attemptWalk` hands the shove (`:247`).
- The **netclass clearance** is what `NODE::CheckColliding` and the shove use for every other pair of items, including the pair's lanes against the rest of the board, and including the two lanes against **each other** inside `verifyDpBypass` (section 6.3) and inside `rhMarkObstacles`/`rhShoveOnly`'s collision tests. There is no suppression of P-versus-N collisions anywhere: the two lanes are two nets and collide normally.

The one place the two meet is the start gate: `ROUTER::isStartingPointRoutable` refuses to begin when `m_sizes.DiffPairGap() < m_sizes.MinClearance()` (`pns_router.cpp:229`), with the message "Diff pair gap is less than board minimum clearance." That is the only consistency check between them.

A consequence worth writing down for the port: if the resolver's clearance between the P and N nets exceeds the configured pair gap, every fitted pair collides with itself and `m_fitOk` is false forever, in mark obstacles and shove mode. Nothing detects or reports that beyond the start gate's board-minimum test.

---

## 9. The pair dragger: there is not one

At this commit KiCad has **no differential pair dragger**. Establishing that took four greps and they are worth recording:

- `grep -n 'DIFF_PAIR\|DpNet\|DpCoupled\|diff pair\|DiffPair' pcbnew/router/pns_dragger.cpp pcbnew/router/pns_dragger.h pcbnew/router/pns_multi_dragger.cpp pcbnew/router/pns_line_placer.cpp` returns nothing.
- `pcbnew/router/pns_component_dragger.cpp` likewise.
- `ROUTER::StartDragging` (`pcbnew/router/pns_router.cpp:166`) chooses between `COMPONENT_DRAGGER`, `MULTI_DRAGGER` and `DRAGGER` on the shape of the item set alone (`:176`, `:182`, `:187`); the router mode is not consulted, so `PNS_MODE_ROUTE_DIFF_PAIR` changes nothing about a drag.
- `DRAG_MODE` (`pcbnew/router/pns_router.h:75`) has no pair bit.

What a user gets today:

- Dragging one track of a pair drags that track. `DRAGGER` shoves or walks the other lane out of the way like any other net, so the gap is destroyed.
- Selecting both tracks and dragging invokes `MULTI_DRAGGER` (`pns_router.cpp:182`, more than one `SEGMENT_T`/`ARC_T` in the set), which moves both and does preserve their relative spacing, because it drags every selected line by the same displacement and shoves the rest of the board (note 06 section 8). That is the closest thing to pair dragging in the tree, and it is pair agnostic: it never asks the resolver whether the two lines are coupled, never measures the gap, and would behave identically on two unrelated parallel tracks.
- `MULTI_DRAGGER::GetLastCommittedLeaderSegments` (`pns_drag_algo.h:125`) exists so the host can continue routing from a multi drag, and `ROUTER_TOOL` uses it, but again with no pair semantics.

So milestone 10's "the pair dragger" task has no KiCad reference to port. The honest options, in the order this note recommends them:

1. **Drop it from the milestone.** The acceptance criterion is "a pair routes in all three modes and replays from a recording", which the placer satisfies. Pair dragging can become its own work item once there is a consumer.
2. **Document `MULTI_DRAGGER` as the answer.** The crate already has it (`src/multi_dragger.rs`), and selecting both lanes gives KiCad's exact behaviour. This costs nothing and is what KiCad users actually do.
3. **Invent one.** A pair dragger would be `MULTI_DRAGGER` plus `AssembleDiffPair` to find the partner from one clicked segment, plus a gap preserving constraint on the two dragged lines. That is new design, not a port, and it belongs in a design note rather than a reference note.

---

## 10. The host side

### 10.1 Entering pair mode

`PNS_MODE_ROUTE_DIFF_PAIR` is the second value of `ROUTER_MODE` (`pcbnew/router/pns_router.h:69`). The mode is a plain field set by `ROUTER::SetMode` (`pns_router.cpp:1094`) and read by `Mode()` (`pns_router.h:171`); it never changes during a session.

`ROUTER_TOOL::MainLoop` is registered for both routing actions (`router_tool.cpp:3514`, and the single track action alongside it), takes the mode out of the event parameter (`:2378`), stops any running session of a different mode (`:2382` to `:2388`), and calls `m_router->SetMode( mode )` (`:2409`). `ROUTER_TOOL::RouteSelected` does the same for the "route selected" and autoroute actions (`:2136`, `:2234`). Inside the loop the pair action is one of the three things that trigger `performRouting` (`:2470`).

So the host's whole contribution to entering pair mode is: set the mode before `StartRouting`, and highlight two nets instead of one (`:1718` to `:1727`, using `DpCoupledNet`).

### 10.2 The toolbar sizes

`DIFF_PAIR_MENU` (`router_tool.cpp:476`) is the context submenu, shown only while `m_router->Mode() == PNS_MODE_ROUTE_DIFF_PAIR` (`:728` to `:737`). It lists `bds.m_DiffPairDimensionsList` (`:513`), each entry a `DIFF_PAIR_DIMENSION` of width, gap and via gap formatted into a label (`:518` to `:546`), plus a "use custom" item that opens the dialog (`:565` to `:567`) and a "use netclass" item that clears the custom flag (`:571`).

`ROUTER_TOOL::DpDimensionsDialog` (`:2038`):

```cpp
PNS::SIZES_SETTINGS sizes = m_router->Sizes();
DIALOG_PNS_DIFF_PAIR_DIMENSIONS settingsDlg( frame(), sizes );
if( settingsDlg.ShowModal() == wxID_OK )
{
    m_router->UpdateSizes( sizes );
    m_savedSizes = sizes;
    bds.SetCustomDiffPairWidth( sizes.DiffPairWidth() );
    bds.SetCustomDiffPairGap( sizes.DiffPairGap() );
    bds.SetCustomDiffPairViaGap( sizes.DiffPairViaGap() );
}
```

Three numbers, edited by hand, pushed into the live placer through `UpdateSizes` and mirrored into the board settings. `SIZES_SETTINGS` carries them as `m_diffPairWidth`, `m_diffPairGap`, `m_diffPairViaGap` plus the `m_diffPairViaGapSameAsTraceGap` flag (`pns_sizes_settings.h:166` to `:169`), with defaults 125000, 180000, 180000 and `true` (`:52` to `:55`).

`ROUTER_TOOL::UpdateMessagePanel` shows, in pair mode only (`:3455`): the pair width with its source string, the clearance with its source, the pair gap with its source, and the max uncoupled length if a rule provides one (`:3457` to `:3482`). The source strings come from `SIZES_SETTINGS::GetDiffPairWidthSource` / `GetDiffPairGapSource` (`pns_sizes_settings.h:132`, `:135`), which `ImportSizes` fills with either a rule name or one of "board minimum track width", "board minimum clearance", "user choice".

### 10.3 The start gate

`ROUTER::isStartingPointRoutable` (`pns_router.cpp:222`) has a pair branch that is much larger than the single track one.

```
isStartingPointRoutable(where, startItem, layer) -> bool:
    if Settings().AllowDRCViolations(): return true                      :224
    if mode == PNS_MODE_ROUTE_DIFF_PAIR:                                 :227
        if DiffPairGap() < MinClearance():
            fail("Diff pair gap is less than board minimum clearance.")  :231
    ... shared per item routability scan ...                             :236
    if mode == PNS_MODE_ROUTE_SINGLE:  ... single track probe ...
    elif mode == PNS_MODE_ROUTE_DIFF_PAIR:                               :341
        if not startItem:
            fail("Cannot start a differential pair in the middle of nowhere.")  :345
        if not DIFF_PAIR_PLACER::FindDpPrimitivePair(world, startPoint,
                                                     startItem, dpPair, &err):  :352
            fail(err)
        if startItem is SEGMENT_T or ARC_T:                              :363
            actualGap     = |dpPair.AnchorP() - dpPair.AnchorN()|        :365
            configuredGap = DiffPairGap() + DiffPairWidth()              :366
            tolerance     = configuredGap / 10                           :369
            if |actualGap - configuredGap| > tolerance:
                fail("The differential pair gap at the start point does not
                      match the configured gap. ...")                    :373
        build two degenerate one point LINEs on the two anchors, at
        DiffPairWidth(), on aLayer                                       :382
        if either collides with anything:                                :403
            retry both at BoardMinTrackWidth()                           :408
            if still colliding: markViolations, hide, fail
                                 ("The routing start point violates DRC.")  :424
    return true
```

Three pair specific gates the port needs: the gap-versus-min-clearance check, "cannot start in the middle of nowhere" (a pair placement **requires** a start item, where a single track does not), and the 10 percent gap tolerance check that only applies when starting from a track. The comment at `:359` explains the last one: starting from pads or vias, the anchor spacing is fixed by placement and not by routing rules, so it is not checked.

The two probe lines are built with `Append( anchor ); Append( anchor, true )` (`:387`, `:388`), that is, a chain of one point duplicated, which is how a zero length line is made; they carry the two primitives' nets (`:395`, `:400`).

### 10.4 What differs in `Move` and `FixRoute`

Nothing, in the router. `ROUTER::Move` (`:494`) dispatches on `m_state`, not on `m_mode`, and `movePlacing` (`:789`) iterates `m_placer->Traces()` generically, drawing each `LINE_T` item with its resolved clearance and each line's via if it has one (`:796` to `:823`). For a pair that loop runs twice and draws two vias. `ROUTER::FixRoute` (`:915`) is a straight forward to `m_placer->FixRoute` (`:925`).

The two places the mode does leak into the router are `updateView`, which runs `markViolations` for `PNS_MODE_ROUTE_SINGLE` and `PNS_MODE_ROUTE_DIFF_PAIR` but not for the tuning modes (`:757`), and the start gate above.

Two places assume a single trace and therefore behave oddly for pairs. `ROUTER::Finish` (`:569`) and `ROUTER::ContinueFromEnd` (`:617`) both do `dynamic_cast<LINE*>( placer->Traces()[0] )` and drive the whole placement from that one line's ratline, which for a pair is the P lane only; `GetNearestRatnestAnchor` (`:518`) does the same (`:530`) and falls back to `CurrentNets()[0]` (`:549`). So "finish route" and "continue from other end" in pair mode aim the pair at whatever the P net's nearest unconnected anchor is, and the N lane follows because the placer snaps the pair to the target pair once the cursor lands on it. Note 03 section 9.2 already flags this downcast as the reason to make the placer set an enum in the port.

---

## 11. What the pair placer asks of `WALKAROUND`, `SHOVE` and `OPTIMIZER`

### 11.1 `WALKAROUND`

One call, in `attemptWalk` (`pns_diff_pair_placer.cpp:238`), on a freshly constructed instance configured with three setters:

| Call | Line | Value |
| --- | --- | --- |
| `SetSolidsOnly` | `:205` | `aSolidsOnly`, true from the shove path, false from the walk path. |
| `SetIterationLimit` | `:206` | `Settings().WalkaroundIterationLimit()`. |
| `SetAllowedPolicies` | `:207` | `{ WP_SHORTEST }` only. |
| `Route( preWalk )` | `:238` | Result read as `status[WP_SHORTEST]` and `lines[WP_SHORTEST]`. |

No cluster restriction, no visible area, no winding preference (`aWindCw` never reaches it, erratum E3). `ST_DONE` is required; `ST_ALMOST_DONE` and `ST_STUCK` both fail the attempt (`:240`).

The crate has all of this: `Walkaround::new` (`src/walkaround.rs:260`), `set_allowed_policies` (`:344`), `route` (`:436`), `WalkPolicy::Shortest` (`:95`), `WalkaroundStatus` (`:124`), `WalkaroundResult::into_line` (`:181`). The solids-only flag and the iteration limit are already there (note 03 section 5). Nothing new is needed.

### 11.2 `SHOVE`

Two entry points, and one that is conspicuously absent.

**`ShoveObstacleLine`, outside any `Run()`** (`:251`), preceded by `ForceClearance( true, cur.Gap() - 2 * PNS_HULL_MARGIN )` (`:247`). This is the pair's coupling primitive: push the non walked lane away from the walked one at exactly the pair gap. The crate has `Shove::shove_obstacle_line` (`src/shove.rs:2264`), already public for precisely this caller (its doc comment says so), and `Shove::set_force_clearance` (`:1324`). Nothing new.

**The head protocol** in `rhShoveOnly` (`:370` to `:386`): `ClearHeads()`, `AddHeads(pLine)`, `AddHeads(nLine)`, `Run()`, then `HeadsModified(0)`/`GetModifiedHead(0)` and the same for index 1. The crate has `clear_heads` (`src/shove.rs:1366`), `add_head_line` (`:1387`), `run` (`:4100`), `heads_modified` (`:1450`), `modified_head` (`:1466`). Nothing new.

**No springback locking.** `AddLockedSpringbackNode`, `RewindSpringbackTo`, `UnlockSpringbackNode` and `RewindToLastLockedNode` appear nowhere in the pair placer, where the single placer uses all four (note 04 section 5.2). Instead `FixRoute` throws the shove away and builds a new one (`:861`). A port should do the same, and record the decision rather than "improving" it, because locking would change which shove results survive a fixed leg.

### 11.3 `OPTIMIZER`

One call, `optimizer.Optimize( &aPair )` on a default constructed `OPTIMIZER( m_currentNode )` (`:311`, `:314`), which is `mergeDpSegments` and nothing else (section 6). No effort level, no collision mask, no preserved vertex, no restricted range or area.

The crate's `Optimizer` (`src/optimizer.rs`) has the whole single line side. The pair side is entirely new: `merge_dp_segments`, `merge_dp_step`, `coupled_bypass`, `verify_dp_bypass`, `find_coupled_vertices`. They need `LineChain::replace_with_chain` (`src/geometry/line_chain.rs:709`), `simplify2` (`:1224`), `Direction45::build_initial_trace` (`src/geometry/direction45.rs:512`) and `is_obtuse`, all of which exist.

---

## 12. What the port needs

### 12.1 What already exists, verbatim

Read out of the crate on 2026-09-10.

**Sizes.** `Sizes` (`src/settings.rs:427` onwards) already carries the complete pair set with KiCad's defaults and the two derived accessors:

| Crate | KiCad | Line |
| --- | --- | --- |
| `Sizes::diff_pair_width` | `DiffPairWidth()` | `src/settings.rs:429` |
| `Sizes::diff_pair_gap` | `DiffPairGap()` | `:432` |
| `Sizes::diff_pair_via_gap` | `m_diffPairViaGap` | `:439` |
| `Sizes::diff_pair_via_gap_same_as_trace_gap` | `m_diffPairViaGapSameAsTraceGap` | `:441` |
| `Sizes::diff_pair_hole_to_hole` | `GetDiffPairHoleToHole()` | `:446` |
| `Sizes::diff_pair_copper_to_hole` | `GetDiffPairCopperToHole()` | `:449` |
| `Sizes::diff_pair_via_gap()` | `DiffPairViaGap()` | `:590` |
| `Sizes::effective_diff_pair_via_gap()` | `EffectiveDiffPairViaGap()` | `:606` |

with the defaults asserted at `:737` to `:743` and the two accessors tested at `:803` and `:819`. **Nothing has to change in `Sizes`.** The one thing missing is `DIFF_PAIR_PLACER::gap()`, the centre to centre sum, which should become a named accessor (`Sizes::diff_pair_pitch()` or similar) rather than an open coded addition, precisely because of section 1.5.

**Rule hooks.** `RuleResolver::dp_coupled_net` (`src/rules.rs:450`), `dp_net_polarity` (`:460`), `dp_net_pair` (`:468`), all defaulted to "not supported" and unit tested at `:918` to `:920`. `ConstraintType::DiffPairGap`, `DiffPairSkew` and `MaxUncoupled` (`:194`, `:211`, `:214`). Nothing new.

**Geometry.** Everything `BuildInitial`, the gateway builders and the optimizer's pair path need:

| Need | Crate | Line |
| --- | --- | --- |
| `DIRECTION_45::BuildInitialTrace` | `Direction45::build_initial_trace` | `src/geometry/direction45.rs:512` |
| `DIRECTION_45::Angle` and the masks | `Direction45::angle`, `AngleType::{OBTUSE,RIGHT,ACUTE,STRAIGHT,HALF_FULL,UNDEFINED}` | `:352`, `:156` to `:186` |
| `DIRECTION_45::IsDiagonal`, `ToVector` | `is_diagonal`, `to_vector` | `:341`, `:449` |
| `SEG::LineProject` | `Seg::line_project` | `src/geometry/seg.rs:568` |
| `SEG::ApproxParallel` | `Seg::approx_parallel` (threshold already a parameter) | `:783` |
| `SEG::Collinear` | `Seg::collinear` | `:701` |
| `SEG::IntersectLines` | `Seg::intersect_lines` | `:907` |
| `SEG::SquaredDistance`, `Distance` | `squared_distance_to_segment`, `distance_to_segment` | `:406`, `:450` |
| `SEG::SquaredLength`, `Length` | `squared_length`, `length` | `:291`, `:282` |
| `rescale` | `geometry::math::rescale` | `src/geometry/math.rs:63` |
| `VECTOR2I::Perpendicular`, `Resize` | `Vec2::perpendicular`, `resize` | `src/geometry/vec2.rs:147`, `:167` |
| `SHAPE_LINE_CHAIN::SelfIntersecting` | `LineChain::self_intersecting` | `src/geometry/line_chain.rs:1662` |
| `SHAPE_LINE_CHAIN::Intersects` | `LineChain::intersects_chain` | `:1629` |
| `SHAPE_LINE_CHAIN::Simplify`, `Simplify2` | `simplify`, `simplify2` | `:1126`, `:1224` |
| `SHAPE_LINE_CHAIN::Replace` | `replace_with_chain` | `:709` |
| `SHAPE_LINE_CHAIN::Reverse` | `reverse`, `reversed` | `:871`, `:883` |
| `PNS_HULL_MARGIN` | `geometry::hull::HULL_MARGIN` | `src/geometry/hull.rs:66` |

**Engine.** `World::branch` (`src/node.rs:526`), `check_colliding` / `check_colliding_line` (`:1696`, `:1768`), `all_items_in_net` (`:2166`), `find_joint` (`:2220`), `assemble_line` (`:2524`); `topology::leading_rat_line` (`src/topology.rs:493`); `via::via_pushout_force` (`src/via.rs:151`) and `move_via_to` (`:45`); `Line` with `set_shape`, `with_chain`, `append_via`, `remove_via`, `set_via_diameter`, `set_via_drill`, `set_width`, `anchor`, `links` (`src/line.rs:503`, `:385`, `:896`, `:938`, `:962`, `:981`, `:1709`, `:1671`, `:579`).

### 12.2 What is genuinely new

Nine items, in dependency order.

1. **`Seg::t_coef`.** KiCad's `SEG::TCoef( aP ) = (B - A) . (aP - A)` as an `i64`. Not in `src/geometry/seg.rs`; the module documentation at `:42` explicitly lists `TCoef` among the members not exposed. Needed only by `common_parallel_projection`, so it can be a private helper in the new module or a `pub(crate)` on `Seg`.
2. **`common_parallel_projection`.** Section 2.1. A free function returning `Option<(Seg, Seg)>` rather than a bool plus two out parameters.
3. **`RangedNum<i32>`** or, better, a two field `GapConstraint { value: i32, tolerance_plus: i32, tolerance_minus: i32 }` with `matches(&self, other: i64) -> bool`. Section 1.6. Three lines, but it must be a distinct type so the two gap meanings (section 1.5) cannot be confused.
4. **The `diff_pair` module**: `DiffPair`, `DpGateway`, `DpGateways`, `DpPrimitivePair`, `CoupledSegments`, plus the coupling geometry of section 2 and the builders of section 3. `DpPrimitivePair` holds `Option<ItemId>` plus anchors rather than owned clones, because the crate's items live in an arena (note 02 section 11); the KiCad clone-and-delete dance and its leaking `operator=` (erratum E17) disappear.
5. **`Optimizer` pair path**: `merge_dp_segments`, `merge_dp_step`, `coupled_bypass`, `verify_dp_bypass`, `find_coupled_vertices`. Section 6.
6. **`topology::simplify_line`.** `src/topology.rs:28` records it as not ported precisely because its only callers are `pns_diff_pair_placer.cpp:853` and `:854`. Milestone 10 is when it arrives. It is ten lines over `World::assemble_line`, `LineChain::simplify`, `World::remove`, `World::add`.
7. **The pair placer** itself, `src/placer/diff_pair_placer.rs`. Section 12.3.
8. **A placer enum** in the session facade. Section 12.4.
9. **A test resolver that answers the pair hooks.** Section 15.

### 12.3 The placer module's shape

`DESIGN.md` section 11 and note 03 section 9.2 ask for enum dispatch over trait objects, and `src/placer/mod.rs` already records the decision: "the set of placers is closed by the router mode, so an enum wrapping the five is a better fit than a trait object, and it removes the downcast KiCad's `Finish` and `ContinueFromEnd` perform on `Traces()[0]`". Milestone 10 is where that enum finally has two variants and stops being hypothetical.

```rust
// src/placer/mod.rs
pub enum Placer {
  Line(line_placer::LinePlacer),
  DiffPair(diff_pair_placer::DiffPairPlacer),
}
```

with the eleven methods the facade actually calls forwarded by a `match`. Which eleven: `start`, `move_to`, `fix_route`, `commit_placement`, `abort_placement`, `has_placed_anything`, `toggle_via`, `set_layer`, `traces`, `current_start`/`current_end`/`current_layer`/`current_net(s)`/`current_node`, `flip_posture`, `update_sizes`, `undo_last_segment`. Two of those need shape changes:

- **`current_nets`, not `current_net`.** `Router::current_net` (`src/router.rs:883`) forwards to `LinePlacer::current_net`. A pair has two. Either the facade returns a `Vec<NetId>` like KiCad, or, better given `DESIGN.md` section 11's preference for enums over sentinels, an enum `RoutedNets::Single(NetId) | RoutedNets::Pair { p: NetId, n: NetId }`.
- **`traces`.** `LinePlacer::trace` returns one optional line. The pair returns two. The natural signature is a small enum or a `&[Line]` slice into the placer; a slice is enough because both variants can hold their lines contiguously, and it keeps `Router::frame` (`src/router.rs:1064`) a single loop.

Inside the pair placer, follow note 03 section 9.2's advice again for `route`:

```rust
fn route(&mut self, ...) -> bool {
  match context.settings.mode {
    RouterMode::MarkObstacles => self.rh_mark_obstacles(...),
    RouterMode::Walkaround    => self.rh_walk_only(...),
    RouterMode::Shove         => self.rh_shove_only(...),
  }
}
```

and prefer `Option<DiffPair>` returns to KiCad's "bool plus a mutated member", which is what makes erratum E12's sticky `m_currentTraceOk` expressible as an explicit `Option` the caller has to handle.

The state struct drops eight of KiCad's members outright (`m_state`, `m_iteration`, `m_p_start`, `m_startsOnVia`, `m_orthoMode`, `m_viaDiameter`, `m_viaDrill`, `m_currentWidth`; section 5.1) and models `m_idle` as the enum discriminant the way `LinePlacer` already does.

### 12.4 The session facade

`Router::start_routing` (`src/router.rs:1115`) is `(at, start, layer) -> Result<PreviewFrame, StartError>` and hard codes `LinePlacer` (`:1140`). KiCad's equivalent is `StartRouting` plus a separate `SetMode` set earlier by the tool (`pns_router.cpp:441`). Two options:

- **A mode field on the router**, set by a `set_mode` before `start_routing`, exactly as KiCad does. Faithful, but it makes `start_routing` fallible in a way the caller cannot see from the signature, and it adds a mode to the recording format as a separate event.
- **A second entry point**, `start_routing_diff_pair(at, start, layer)`. This is what the crate's own idiom points at: the facade already has `start_routing` and `start_dragging` as two entry points for two placer families, `RouterState` (`src/router.rs:98`) already discriminates them, and a pair start has genuinely different preconditions (a start item is **required**, `pns_router.cpp:343`). The recording then needs `SessionEvent::StartRoutingDiffPair` alongside `StartRouting` (`src/eventlog.rs:147`), which is one more variant and no format ambiguity.

This note recommends the second, and recommends recording the decision in `doc/log/`.

New `StartError` variants needed, from section 10.3:

| Variant | KiCad message | Line |
| --- | --- | --- |
| `PairNeedsStartItem` | "Cannot start a differential pair in the middle of nowhere." | `pns_router.cpp:345` |
| `NotADiffPair` | "Unable to find complementary differential pair nets..." | `pns_diff_pair_placer.cpp:526` |
| `NoDanglingAnchor` | "Can't find a suitable starting point. If starting from an existing differential pair make sure you are at the end." | `:543` |
| `NoCoupledStartItem(NetId)` | "Can't find a suitable starting point for coupled net ..." | `:598` |
| `PairGapBelowMinClearance` | "Diff pair gap is less than board minimum clearance." | `pns_router.cpp:231` |
| `PairGapMismatch` | "The differential pair gap at the start point does not match the configured gap..." | `:373` |

The existing `StartPointViolatesRules` covers the two probe lines at `:424`.

### 12.5 Determinism

Three places in the KiCad code depend on unspecified order and must be pinned in the port (`DESIGN.md` section 8):

- `FindDpPrimitivePair`'s scan over `AllItemsInNet` with a strict `dist < bestDist` (`pns_diff_pair_placer.cpp:575`): KiCad's `std::set<ITEM*>` orders by pointer. Tie break on `(distance, uid)`.
- `FitGateways`'s `score >= bestScore` (`pns_diff_pair.cpp:354`): the winner among equal scores is the last one built, so the push order of every gateway builder is part of the answer. The port must reproduce push order exactly, and a unit test should pin the produced gateway list for a fixed input.
- `tryWalkDp`'s `score < bestScore` over four attempts (`pns_diff_pair_placer.cpp:299`): the first of the two duplicated attempts wins. Once `aWindCw` is dropped (erratum E3), the loop becomes two attempts and the tie break is explicit.

`TOPOLOGY::AssembleDiffPair`'s `dist_sq <= minDist_sq` plus a secondary distance-to-target test (`pns_topology.cpp:1115`) is the same class of problem, but it is not on milestone 10's path.

---

## 13. Errata

Everything here was verified against the tree at `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85`. "No caller" means a grep over `pcbnew/`, `qa/` and the rest of the sparse checkout returns only the definition and, where applicable, the declaration.

### E1. `DP_GATEWAYS::BuildOrthoProjections` is dead

`pcbnew/router/pns_diff_pair.cpp:302`, declared `pcbnew/router/pns_diff_pair.h:194`. No caller. It is the pair counterpart of ortho mode, and ortho mode for pairs is stored and ignored (E13). Do not port.

### E2. `DIFF_PAIR_PLACER::setInitialDirection` is declared and never defined

`pcbnew/router/pns_diff_pair_placer.h:196`. `LINE_PLACER` has a real one (`pns_line_placer.h:280`); this is a copy of the declaration that was never given a body. Nothing references it, so it links. Do not port.

### E3. Three unused parameters in the walk path

- `DIFF_PAIR_PLACER::attemptWalk( ..., bool aWindCw, ... )` (`pcbnew/router/pns_diff_pair_placer.cpp:201`): never read in the body (`:202` to `:276`). The walkaround is configured with `{ WP_SHORTEST }` only (`:207`) and no winding preference is expressible through that policy.
- `DIFF_PAIR_PLACER::tryWalkDp( NODE* aNode, ... )` (`:279`): never read; the body uses `m_currentNode` (`:287`, `:311`). Both call sites pass `m_currentNode` (`:328`, `:363`).
- Consequence: `tryWalkDp`'s four attempts (`:284`) are two distinct computations run twice, because `attempt & 2` only feeds `aWindCw`. Half the walk cost of every move in walk and shove mode is wasted work.

Port as two attempts (`p_first` true and false) and record the halving in `doc/log/`.

### E4. The collinear midpoint gateway is spaced at half the gap

`pcbnew/router/pns_diff_pair.cpp:675` to `:682`:

```cpp
VECTOR2I dir = makeGapVector( p0_n - p0_p, m_gap / 2 );
VECTOR2I m = ( p0_p + p0_n ) / 2;
m_gateways.emplace_back( m - dir, m + dir, diagColl, DIRECTION_45::ANG_RIGHT, prio );
```

`makeGapVector( v, L )` returns a vector of length about `L / 2` (`:401`), so `|dir| ~ m_gap / 4` and the two anchors are `m_gap / 2` apart. Every other gateway in the file is `m_gap` apart: `:685` to `:688` use `makeGapVector( v, 2 * m_gap )`, `:719` to `:722` use two perpendicular legs of `m_gap / sqrt(2)`, `:742` to `:752` use legs of `m_gap * sqrt(2)` and `m_gap` at 45 degrees, `BuildForCursor` uses `makeGapVector( (gap, gap), gap )` and `(gap + 1) / 2`, and the fan block of `BuildFromPrimitivePair` converges the pads to exactly `m_gap`.

A route built from this gateway has its two lanes `m_gap / 2` apart at the anchors, and `checkGap` (`:182`) rejects any candidate whose lanes come closer than `m_gap - 100` anywhere. Since the anchors are endpoints of the first segments of `p` and `n`, `BuildInitial` (`:251`) rejects it for any `m_gap` above 200 nm, which is every real board. So this gateway appears to be unreachable in practice. It is either a typo for `2 * m_gap` or a deliberate half spacing whose purpose was lost. Port it as written, with a test that asserts it never wins, or omit it with a logged decision; do not "fix" it silently, because fixing it would add a gateway KiCad never chooses and change routes.

### E5. Copy-paste in `BuildGeneric`'s collinearity guard

`pcbnew/router/pns_diff_pair.cpp:705`:

```cpp
ips[0] = d_n[i].IntersectLines( d_p[j] );
ips[1] = st_p[i].IntersectLines( st_n[j] );

if( d_n[i].Collinear( d_p[j] ) )
    ips[0] = OPT_VECTOR2I();

if( st_p[i].Collinear( st_p[j] ) )     // <- st_p twice
    ips[1] = OPT_VECTOR2I();
```

`ips[1]` was produced from `st_p[i]` and `st_n[j]`, so the guard should read `st_p[i].Collinear( st_n[j] )` to match the line above it. As written it tests `st_p[i]` against `st_p[j]`.

The guards exist because `SEG::IntersectLines` on two **collinear** infinite lines does not answer "no intersection": it answers the midpoint of the two segments' start points (`libs/kimath/src/geometry/seg.cpp:339` to `:369`), an arbitrary value the caller must discard. On two parallel but non collinear lines it does answer nothing (`:345`).

Working the four index combinations through, the typo turns out to be harmless. `st_p[0]` and `st_n[0]` are horizontal, `st_p[1]` and `st_n[1]` vertical, so `st_p[i]` and `st_n[j]` can only be collinear when `i == j`. The written guard is true exactly when `i == j` (a segment is collinear with itself) and false otherwise (horizontal against vertical). So it nulls `ips[1]` in a superset of the cases the intended guard would, and the extra cases (`i == j` with P and N **not** sharing that axis) are parallel non collinear, where `IntersectLines` already returned nothing. The two guards therefore agree on every input.

Port either form. The crate's `Seg::intersect_lines` reproduces the collinear midpoint (`src/geometry/seg.rs:907` through `intersects_impl`), so the guard is still required, whichever way it is written.

### E6. `buildDpContinuation`'s alignment guard misses the minus 45 diagonal

`pcbnew/router/pns_diff_pair.cpp:638`:

```cpp
if( abs( delta.x ) < EPSILON || abs( delta.y ) < EPSILON || abs( delta.x - delta.y ) < EPSILON )
```

with `delta = anchorP - anchorN`. The three tests catch a vertical anchor line, a horizontal one, and the `+45` diagonal. The `-45` diagonal has `delta.x == -delta.y`, so `abs( delta.x - delta.y )` is `2 * abs( delta.x )`, not near zero, and the angled gateways are not built. A pair whose anchor line runs at `-45` degrees therefore cannot make the 22.5 degree assisted turn that a `+45` pair can. Reproduce as written.

### E7. `BuildForCursor` passes the diagonal flag inverted relative to its documentation

`pcbnew/router/pns_diff_pair.cpp:550` to `:577`. When `diagonal` is false the offset vector has both components non zero, so the anchors lie on a diagonal line, and `false` is passed as `DP_GATEWAY`'s `aIsDiagonal`; when `diagonal` is true the offset is axis aligned and `true` is passed. `DP_GATEWAY::IsDiagonal` is documented as "the gateway anchors lie on a diagonal line" (`pns_diff_pair.h:60`), so the values contradict the documentation. Its only use is as the `aStartDiagonal` hint to `BuildInitialTrace` in `buildEntries` (`:590`, `:592`), under which reading the values are sensible (anchors on a diagonal want a straight-first lead). Port the values as written and fix the doc comment.

### E8. `FindDpPrimitivePair` tie breaks on pointer order

`pcbnew/router/pns_diff_pair_placer.cpp:575`, `dist < bestDist` over `NODE::AllItemsInNet`, which fills a `std::set<ITEM*>`. Two candidate items exactly equidistant from the reference anchor are resolved by allocation address. Port with a `(distance, uid)` tie break per `DESIGN.md` section 8.

### E9. `propagateDpHeadForces` returns a position that does not match the head it computed

`pcbnew/router/pns_diff_pair_placer.cpp:150` to `:196`.

- `force` is declared outside the loop (`:153`) and only replaced when a longer one is seen (`:173`). It is never reset per iteration, so it is a running maximum over every obstacle and every layer.
- `virtHead` is displaced by that running maximum on every colliding iteration (`:180`), so after `n` iterations it sits at `aP + sum of the running maxima`.
- The answer is `aNewP = aP + force` (`:192`), a single application of the final running maximum.
- `totalForce` (`:153`, `:179`) accumulates the same displacements and is never read.

So for any move where more than one obstacle pushes, the position handed to `CursorOrientation` and `BuildForCursor` is not the position the loop walked the virtual head to. Port as written to reproduce KiCad's routes, with `total_force` dropped (it is unobservable) and the aliasing spelled out in a doc comment.

### E10. `BuildFromPrimitivePair` silently does nothing for a mixed primitive pair

`pcbnew/router/pns_diff_pair.cpp:434` and `:442` handle solid-or-via on both sides and segment-or-arc on both sides. A pad paired with a segment matches neither, `shP` stays null and the function returns at `:453` having appended nothing, so `FitGateways` finds no entry gateway and the fit fails with no diagnostic. Unreachable today because `FindDpPrimitivePair` requires equal kinds (`pns_diff_pair_placer.cpp:559`). In the port, make it an explicit `None` return so the placer can say why.

### E11. `tryWalkDp` always reports success, and minimises coupled length

`pcbnew/router/pns_diff_pair_placer.cpp:281` to `:319`. Two defects in nine lines.

`bestScore` starts at `100000000000000.0` (`:282`) and the final test is `if( bestScore > 0.0 )` (`:309`). When every `attemptWalk` fails, `bestScore` is still `1e14`, the test passes, and `aPair.SetShape( best )` (`:313`) writes a **default constructed** `DIFF_PAIR`'s two empty chains into the pair. `tryWalkDp` returns true, `rhWalkOnly` sets `m_fitOk = true` (`:328`) and the placer reports a routed pair with no geometry. `FixRoute` then bails at `:813` because both segment counts are zero, so nothing wrong is committed, but the preview and the return of `Move` both lie. The test was presumably meant to be `if( found )` or `bestScore < initial`.

The score is `1 + CoupledLength() + 3 * |Skew()|` (`:294` to `:297`) and the comparison is `score < bestScore` (`:299`), so among successful attempts the one with the **least** coupling wins. Every other coupled length comparison in the tree maximises: `coupledBypass` keeps `coupledLength > bestLength` (`pns_optimizer.cpp:1408`), `mergeDpStep` accepts only when the coupled length does not fall by more than a tenth (`:1465`, `:1471`). Either the sign of the `cl` term is wrong or the comparison is.

Both are behaviour changing to fix. Port as written first, so the crate reproduces KiCad, add a scenario test that pins the current answer, then change them behind an explicit decision in `doc/log/` with the test updated in the same commit.

### E12. `routeHead`'s sticky success, and the gap left in the wrong unit

`pcbnew/router/pns_diff_pair_placer.cpp:756`, `return m_currentTraceOk;`. When `FitGateways` fails, the placer reports success if any earlier fit in this leg succeeded, and `m_currentTrace` keeps the **previous** move's shape. All three mode routines then operate on that stale shape: `rhMarkObstacles` collision tests it, `rhWalkOnly` walks it, `rhShoveOnly` shoves it. The visible effect is that the pair preview freezes at the last routable position rather than disappearing, which is arguably the nicer behaviour and is certainly the one users are used to.

Compounding it: the `SetGap( DiffPairGap() )` at `:741` is inside the success branch, while `SetGap( gap() )` at `:731` is not. So on the sticky path `m_currentTrace.Gap()` is the centre to centre pitch, and `attemptWalk` hands that to `SHOVE::ForceClearance` (`:247`) as a copper gap, and `CoupledSegmentPairs` matches against it as a copper gap. The lanes are pushed `DiffPairWidth()` further apart than intended and nothing is reported as coupled.

Note that `UpdateSizes` (`:797`) rewrites the gap to the edge to edge value whenever it runs, so the window closes on the next size event.

Reproduce, with the `Option` return of section 12.3 making the sticky path explicit at the call site rather than implicit in a member.

### E13. `m_orthoMode` is written three times and never read

`pcbnew/router/pns_diff_pair_placer.cpp:56`, `:86`, `:659`; declared `pns_diff_pair_placer.h:271`. `SetOrthoMode` is a `PLACEMENT_ALGO` virtual the host calls on the shift key, and for pairs it does nothing except trigger a redundant `Move`. Related to E1: `BuildOrthoProjections` is the machinery it would have driven.

### E14. `mergeDpSegments` has no iteration bound and a dead `step == 1` state

`pcbnew/router/pns_optimizer.cpp:1502` to `:1533`. The `while( 1 )` exits only through `step_p < 1 && step_n < 1` (`:1516`), and the counters decrease only when neither lane found a merge (`:1528`). A `mergeDpStep` that returns true without shrinking either chain spins forever. The single line `mergeFull` path is bounded by `MERGE_PASS_LIMIT` (note 04 section 4.4); the pair path is not. Additionally `step == 1` satisfies neither the `step > 1` merge guards (`:1522`, `:1525`) nor the `step < 1` exit, so it costs one wasted iteration per lane.

In the port, bound it with the crate's existing `MERGE_PASS_LIMIT` (`src/optimizer.rs:181`) and record the deviation.

### E15. `checkDpColliding` has no caller

`pcbnew/router/pns_optimizer.cpp:1426`. Two lines, builds a `LINE` from a lane and a path and asks the node. `verifyDpBypass` does the same work inline (`:1359`, `:1362`). Do not port.

### E16. `TOPOLOGY::AssembleDiffPair( SEGMENT* )` is declared and never defined

`pcbnew/router/pns_topology.h:99`. The `bool AssembleDiffPair( ITEM*, DIFF_PAIR& )` overload at `:101` is the real one. Same class of defect as E2.

### E17. `DP_PRIMITIVE_PAIR::operator=` leaks and can leave a stale pointer

`pcbnew/router/pns_diff_pair.cpp:79`:

```cpp
DP_PRIMITIVE_PAIR& DP_PRIMITIVE_PAIR::operator=( const DP_PRIMITIVE_PAIR& aOther )
{
    if( aOther.m_primP ) m_primP = aOther.m_primP->Clone();
    if( aOther.m_primN ) m_primN = aOther.m_primN->Clone();
    m_anchorP = aOther.m_anchorP;
    m_anchorN = aOther.m_anchorN;
    return *this;
}
```

The previous `m_primP` / `m_primN` are never deleted, so every assignment over a populated pair leaks two cloned items; and when the source's pointer is null the destination keeps its own old pointer while taking the source's anchors, so the pair describes one thing and points at another. The copy constructor (`:64`) gets this right by nulling both first. Reachable from `m_prevPair = m_currentTrace.EndingPrimitives()` (`pns_diff_pair_placer.cpp:856`), which runs once per fixed leg through `std::optional::operator=`, and from `m_start = *m_prevPair` (`:452`). The arena based port has no owned clones and the whole class of problem disappears.

### E18. Dead members and dead methods, collected

| Symbol | Line | Note |
| --- | --- | --- |
| `DIFF_PAIR::m_viaGap` | `pns_diff_pair.h:566` | Assigned in four constructors and both assignment operators, never read. |
| `DIFF_PAIR::m_maxUncoupledLength` | `:567` | Same. The max uncoupled rule is display only (section 8.1). |
| `DIFF_PAIR::m_chamferLimit` | `:568` | Same. Nothing in the tree chamfers a pair. |
| `DIFF_PAIR::Clear`, `Append`, `Empty` | `:513`, `:519`, `:525` | No caller. |
| `DIFF_PAIR::CoupledLengthFactor`, `TotalLength` | `:508`, `:507` | `TotalLength` is called only by `CoupledLengthFactor`, which has no caller. |
| `DIFF_PAIR::CoupledLength( const SEG&, const SEG& )` | `:535` | No caller. |
| `DP_GATEWAYS::Clear`, `CGateways` | `:179`, `:205` | No caller. |
| `DP_GATEWAYS::DP_CANDIDATE::gw_p`, `gw_n`, `score` | `:213`, `:214` | Written nowhere; `FitGateways` uses only `p` and `n`. |
| `DP_PRIMITIVE_PAIR::dump` | `:150` | `printf` of raw pointers. No caller. |
| `DIFF_PAIR_PLACER::m_state`, `m_iteration`, `m_p_start`, `m_startsOnVia`, `m_viaDiameter`, `m_viaDrill`, `m_currentWidth` | `pns_diff_pair_placer.h:223`, `:236`, `:242`, `:270`, `:260`, `:263`, `:266` | Section 5.1. |
| `ITEM_SET head;` in `rhShoveOnly` | `pns_diff_pair_placer.cpp:368` | Declared, never used. |
| The `else` branch of `rhShoveOnly` | `:397` to `:401` | Restores values that were never changed. |

---

## 14. Proposed order of implementation

Ten steps, each of which leaves the crate compiling, clippy clean and tested. The first four need no world, no resolver and no router, which is why they come first: they are the whole of section 2 and section 3 and they carry the risk (integer geometry, the two gap meanings, the gateway push order).

**Step 1: the coupling geometry.** `Seg::t_coef`, `common_parallel_projection`, `GapConstraint`. Unit tests as listed in section 2.5. No new module is strictly needed yet; put `t_coef` on `Seg` and the other two in a new `src/diff_pair.rs`.

Verify: `dev/in-container.sh cargo test diff_pair`.

**Step 2: `DiffPair` and its measurements.** The struct, `set_shape`, `set_gap`, `set_width`, `coupled_segment_pairs`, the two live `coupled_length` overloads, `skew`. Deliberately omit `total_length`, `coupled_length_factor`, the single segment `coupled_length`, `clear`, `append`, `empty` and the three dead members (erratum E18). Name the two gap quantities apart per section 1.5: `gap` (edge to edge) on `DiffPair`, `pitch` (centre to centre) on `Sizes`.

Verify: the section 2.5 cases now run through `DiffPair`, plus a round trip test that `set_gap` gives the +/- 10000 tolerance and construction from a pitch gives none.

**Step 3: `DpGateway`, `DpPrimitivePair` and `check_connection_angle`.** Pure data plus the angle test. `DpPrimitivePair` holds `Option<ItemId>` and anchors; `directional`, `dir_p`, `dir_n`, `cursor_orientation` need a `&World` to read the items, which is the first world dependency.

Verify: `cursor_orientation`'s three branches (both segments parallel, both segments not parallel, not both segments) with hand built worlds.

**Step 4: the gateway builders.** `make_gap_vector`, `build_generic`, `build_entries`, `build_dp_continuation`, `build_from_primitive_pair`, `build_for_cursor`, `filter_by_orientation`, `check_diagonal_alignment`. Omit `build_ortho_projections` (E1), `clear` and `c_gateways` (E18).

Verify: this is where the port is most likely to diverge silently, so pin the output. For each of five inputs (two round pads on a horizontal, two round pads on a diagonal, two rectangular pads, a segment pair continuation, a bare cursor) assert the **complete gateway list** as an ordered vector of `(anchor_p, anchor_n, is_diagonal, allowed_angles, priority, has_entry_lines)`. Push order is part of the answer (section 12.5), so the assertion has to be ordered. Add the E4 assertion here: the midpoint gateway is present and its anchors are `pitch / 2` apart.

**Step 5: `build_initial` and `fit_gateways`.** Including `check_gap`. `fit_gateways` becomes a free function taking the pitch, the two sets and the posture (section 4.2).

Verify: given the step 4 gateway lists, assert the fitted chains for the same five inputs. Assert that the E4 midpoint gateway never wins.

**Step 6: the optimizer's pair path.** `find_coupled_vertices`, `verify_dp_bypass`, `coupled_bypass`, `merge_dp_step`, `merge_dp_segments`, bounded by `MERGE_PASS_LIMIT` (E14). Omit `check_dp_colliding` (E15).

Verify: a pair with a deliberate staircase on one lane, asserted to merge; a pair already optimal, asserted unchanged; a pair whose merge would cost more than a tenth of the coupled length, asserted rejected.

**Step 7: `topology::simplify_line`.** Ten lines, needed by `fix_route`. Update the "what is not here" list in `src/topology.rs:28` in the same commit.

**Step 8: the placer, mark obstacles mode only.** `DiffPairPlacer` with `start`, `find_dp_primitive_pair`, `dangling_anchor`, `init_placement`, `route_head`, `propagate_dp_head_forces`, `rh_mark_obstacles`, `move_to`, `fix_route`, `commit_placement`, `traces`, the small commands. This is the first step that needs the test resolver of section 15.

Verify: the synthetic fixture routes a pair between two pad pairs with no obstacle, in mark obstacles mode, and the committed diff has four segments (two per lane) or fewer.

**Step 9: walk mode.** `attempt_walk` (two attempts, E3) and `try_walk_dp`, `rh_walk_only`. Reproduce E11 exactly at first, with a test that pins "walk mode reports success on an unroutable pair and hands back empty chains", then decide whether to fix it and update the test in the same commit as the `doc/log/` entry.

Verify: the fixture with the obstacle routes around it and both lanes stay at the gap along the straight runs.

**Step 10: shove mode, vias, and the facade.** `rh_shove_only`, the via path (`make_via`, `append_vias`, `toggle_via`, `set_layer`), the `Placer` enum, `start_routing_diff_pair`, the new `StartError` variants, `SessionEvent::StartRoutingDiffPair` and the recording round trip.

Verify: the milestone's acceptance criterion. The fixture routes in all three modes, and the recording of each replays to the same commit diff.

Steps 1 to 7 are engine work with no host surface and can land as separate commits without touching `src/router.rs`. Steps 8 to 10 each change the facade, so each needs the `CHANGELOG.md` entry and the `doc/work/010-differential-pairs.md` checkbox.

---

## 15. A synthetic fixture

The KiCad regression corpus under `qa/data/pcbnew/pns_regressions` contains no differential pair case: `qa/tools/pns/pns_log_player.cpp` has no pair event and `ROUTER_MODE` is not in the log format at all (note 05 section 6.2), so a pair session cannot be recorded there. The only pair test in KiCad's tree is `qa/tests/pcbnew/test_pns_diff_pair_tuning_width.cpp`, which loads a real board (`issue23550/issue23550`) and exercises `AssembleDiffPair` and the through via layer span, not the placer. Milestone 10's fixture therefore has to be built by hand, which the work item already says (`doc/work/010-differential-pairs.md`, the LibrePCB task is on hold and "the fixture is a synthetic board in the crate's tests").

### 15.1 The board

Two nets, two pairs of pads, one obstacle. All coordinates in nanometres, one copper layer for the base case and two for the via case.

```
NetId(1)  = "P"    the positive half
NetId(2)  = "N"    the negative half
NetId(3)  = the obstacle's net

pitch = diff_pair_gap + diff_pair_width = 200000 + 200000 = 400000

Start pads, layer 0:
  pad A_P : circle r = 150000 at ( 0,      -200000 )   net 1
  pad A_N : circle r = 150000 at ( 0,       200000 )   net 2

Target pads, layer 0:
  pad B_P : circle r = 150000 at ( 8000000, -200000 )  net 1
  pad B_N : circle r = 150000 at ( 8000000,  200000 )  net 2

Obstacle, layer 0:
  segment O : ( 4000000, -3000000 ) to ( 4000000, 3000000 ), width 200000, net 3
```

The two pad pairs are `400000` apart, which is exactly the pitch, so `ROUTER::isStartingPointRoutable`'s gap tolerance check does not apply (it only fires for a segment start, `pns_router.cpp:363`) and `build_from_primitive_pair`'s `check_diagonal_alignment` passes on the vertical (`dir.x == 0 && dir.y != 0`).

Round pads take the `SH_CIRCLE` branch of `build_from_primitive_pair` (`pns_diff_pair.cpp:457`), which goes straight to `build_generic` and skips the fan block. A second fixture with rectangular pads exercises the `SH_RECT` fan and its `w - h` degeneracy for squares; make that one non square (say 600000 by 300000) so `diag_fan_distance` is not zero.

Sizes:

```rust
Sizes {
  diff_pair_width: 200_000,
  diff_pair_gap:   200_000,
  diff_pair_via_gap: 200_000,
  diff_pair_via_gap_same_as_trace_gap: false,
  via_diameter: 600_000,
  via_drill:    300_000,
  track_width:  200_000,
  board_min_track_width: 100_000,
  min_clearance: 100_000,
  ..Sizes::default()
}
```

with `min_clearance` below `diff_pair_gap` so the start gate passes, and a `FixedClearance::uniform(100_000)` resolver (`src/rules.rs:543`) underneath so the two lanes at a 200000 copper gap do not collide with each other.

### 15.2 The resolver

`FixedClearance` (`src/rules.rs`) answers clearances and defaults all three pair hooks to "not supported", so the fixture needs a wrapper. It is small enough to live in `tests/support/`:

```rust
/// A rule resolver that knows about exactly one differential pair.
struct PairRules {
  inner: FixedClearance,
  net_p: NetId,
  net_n: NetId,
}

impl RuleResolver for PairRules {
  // clearance, constraint, etc: forward to inner.

  fn dp_coupled_net(&self, net: NetId) -> Option<NetId> {
    if net == self.net_p { Some(self.net_n) }
    else if net == self.net_n { Some(self.net_p) }
    else { None }
  }

  fn dp_net_polarity(&self, net: NetId) -> i32 {
    if net == self.net_p { 1 } else if net == self.net_n { -1 } else { 0 }
  }

  fn dp_net_pair(&self, item: ItemRef<'_>) -> Option<(NetId, NetId)> {
    let net = item.item().net()?;   // ItemRef borrows, Item carries the net
    if net == self.net_p || net == self.net_n { Some((self.net_p, self.net_n)) }
    else { None }
  }
}
```

This deliberately does **not** implement KiCad's `MatchDpSuffix`. The crate has no net names, LibrePCB has no pair concept and no convention is being invented here (`doc/work/010-differential-pairs.md`), so the engine's contract is "the host says which nets are coupled" and the test harness answers from a table. A second resolver that returns `None` from `dp_net_pair` is worth having too, to pin the `StartError::NotADiffPair` path.

### 15.3 The cases

| Case | Mode | What it pins |
| --- | --- | --- |
| `pair_routes_straight_between_pad_pairs` | MarkObstacles | No obstacle: both lanes straight, gap held, four segments or fewer in the diff, `fix_route` finishes because the cursor is on the target pair. |
| `pair_start_requires_a_start_item` | any | `start_routing_diff_pair(at, None, 0)` is `Err(PairNeedsStartItem)` (`pns_router.cpp:345`). |
| `pair_start_rejects_an_uncoupled_net` | any | Start on the obstacle: `Err(NotADiffPair)`. |
| `pair_start_rejects_gap_below_min_clearance` | any | `min_clearance` raised above `diff_pair_gap`: `Err(PairGapBelowMinClearance)` (`:231`). |
| `pair_walks_around_an_obstacle` | Walkaround | With segment O: the route clears it, both lanes stay coupled on the straight runs, `coupled_length` is at least some fraction of `total_length`. |
| `pair_shoves_an_obstacle` | Shove | Same board, obstacle net routable: the obstacle moves, both heads come back through `modified_head(0)` and `modified_head(1)`. |
| `pair_marks_a_collision` | MarkObstacles | Same board: `move_to` reports the collision, `fix_route` refuses without `allow_drc_violations`. |
| `pair_places_a_via` | Shove | Two layers, `toggle_via`, then `fix_route`: two vias in the diff, `effective_diff_pair_via_gap` respected between them. |
| `pair_chains_legs` | Walkaround | `fix_route` mid route returns `Continue`, the next leg starts from the previous `ending_primitives`, the second `fix_route` on the target finishes. |
| `pair_session_replays` | all three | Record with `Recorder`, replay through `SessionRecording::from_text`, compare the commit diff. This is the milestone's acceptance criterion. |
| `unroutable_pair_reports_honestly` | Walkaround | The E11 pin: an obstacle the pair cannot get past, asserting KiCad's current answer, so a later fix is a visible test change. |

The coupling assertions should go through `DiffPair::coupled_length` and `skew` rather than through raw coordinates wherever possible: raw coordinate assertions on a 45 degree router break on every gateway ordering change, whereas "the two lanes are coupled over at least 6 mm and the skew is under 100 um" survives.

### 15.4 What the fixture cannot cover

- `AssembleDiffPair` and therefore anything that recovers a pair from committed geometry: not on milestone 10's path (section 7.5).
- The pair length tuners (`DP_MEANDER_PLACER`, `MEANDER_SKEW_PLACER`): milestone 11.
- Pair dragging: KiCad has none (section 9).
- Arcs: the crate's `LineChain` has no arc companion yet (`src/geometry/line_chain.rs:26`), so `CoupledSegmentPairs`'s two `IsArcSegment` guards (`pns_diff_pair.cpp:842`, `:847`) port as no-ops and the `ARC_T` branches of `getDanglingAnchor` and `anchorDirection` are unreachable.
- A real board comparison against KiCad's own routes: there is no pair case in the corpus and no way to record one, so the port's fidelity to KiCad rests on the step 4 and step 5 gateway and candidate assertions rather than on end to end replay.
