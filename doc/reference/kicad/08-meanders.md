# KiCad PNS: MEANDER_SETTINGS, MEANDER_SHAPE, MEANDERED_LINE and the three length tuning placers

Reference architecture note for milestone 11 of this crate. Source tree: sparse checkout of KiCad master at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85`, read 2026-09-10. All `path:line` citations are relative to `/home/Tubbles/dev/ref/kicad/`.

Files read in full: `pcbnew/router/pns_meander.h`, `pcbnew/router/pns_meander.cpp`, `pcbnew/router/pns_meander_placer_base.h`, `pcbnew/router/pns_meander_placer_base.cpp`, `pcbnew/router/pns_meander_placer.h`, `pcbnew/router/pns_meander_placer.cpp`, `pcbnew/router/pns_dp_meander_placer.h`, `pcbnew/router/pns_dp_meander_placer.cpp`, `pcbnew/router/pns_meander_skew_placer.h`, `pcbnew/router/pns_meander_skew_placer.cpp`, `qa/tools/pns/mock_pcb_tuning_pattern.cpp`, `qa/tests/pcbnew/test_meander_corner_radius.cpp`. Read in the parts that touch tuning: `pcbnew/router/pns_topology.cpp` (`AssembleTuningPath`, `AssembleTrivialPath`, `walkTuningPath`, `AssembleDiffPair`), `pcbnew/router/pns_helpers.cpp` (`GetSnappedStartPoint`), `pcbnew/router/pns_router.h/.cpp` (the three tuning modes, the placer factory, the start gate, the six tuning hooks on `ROUTER_IFACE`), `pcbnew/router/pns_kicad_iface.cpp` (those six hooks' implementations), `pcbnew/router/pns_placement_algo.h`, `libs/kimath/include/math/vector2d.h` (`VECTOR2<double>`), `libs/kimath/include/trigo.h` and `libs/kimath/src/trigo.cpp` (`RotatePoint( double*, double*, EDA_ANGLE )`), `libs/kimath/include/geometry/eda_angle.h` (`Normalize`), `libs/kimath/include/geometry/seg.h`, `libs/kimath/include/geometry/shape_line_chain.h` and `libs/kimath/src/geometry/shape_line_chain.cpp` (`Append`, `Length`, `Split`, `Mirror`), `libs/kimath/include/geometry/shape_arc.h`, `libs/core/include/core/minoptmax.h`, `include/base_units.h`. Read through `git show HEAD:<path>` because they are outside the sparse checkout: `pcbnew/generators/pcb_tuning_pattern.h`, `pcbnew/generators/pcb_tuning_pattern.cpp`, `pcbnew/dialogs/dialog_tuning_pattern_properties.cpp`.

What is deliberately not here, because an earlier note has it:

- `ROUTER`'s enums, states, branch tree and session lifecycle: note 03 sections 1.1 to 1.6. Section 10 below covers only the three tuning modes.
- `PLACEMENT_ALGO`'s contract and `LINE_PLACER`'s state machine: note 03 sections 2 and 3. Sections 5 to 7 are written as a diff against that contract, and the short answer is that a meander placer implements about a third of it.
- `NODE::Branch`, `Add`, `Remove`, `CheckColliding`, `AssembleLine`: note 02 sections 5 to 8.
- `DIFF_PAIR`, `DP_GATEWAYS`, the coupling geometry and `CoupledSegmentPairs`: note 07 sections 1 and 2. Section 6 uses `DIFF_PAIR::COUPLED_SEGMENTS` as a given.
- `SHAPE_LINE_CHAIN`, `SEG` and `VECTOR2<int>`: note 01. Section 8 covers only `VECTOR2<double>`, which note 01 line 33 lists as deferred to this milestone and which `TODO.md:18` records as "used only by the meander placers; port with length tuning".
- The QA log format and the regression corpus: note 05 sections 6.2 to 6.9. It contains no tuning case at all, which is why section 14 designs a synthetic fixture instead.

---

## 0. Read this first: six names in the task brief that do not exist

A full tree grep over the checkout at this commit returns nothing for any of these. They were real in older KiCad and are gone.

**`MEANDER_PLACER_BASE::cutTunedLine` does not exist.** The line is cut by `SHAPE_LINE_CHAIN::Split( aStart, aEnd, aPre, aMid, aPost )` called directly from each placer's move: `pcbnew/router/pns_meander_placer.cpp:241` for a single track, `pcbnew/router/pns_dp_meander_placer.cpp:262` and `:263` for a pair. Section 5.4.

**`compareWithTolerance` does not exist.** The tolerance comparison is three inline branches against `MINOPTMAX::Min()` and `Max()`, at `pcbnew/router/pns_meander_placer.cpp:332` to `:337` and `pcbnew/router/pns_dp_meander_placer.cpp:283` to `:288`. Section 4.7.

**`TunedLength` does not exist.** The accessor is `TuningLengthResult()`, pure virtual at `pcbnew/router/pns_meander_placer_base.h:61`, with three overrides. Section 4.7.

**`AmplitudeSettings` does not exist.** There is `AmplitudeStep( int aSign )` (`pcbnew/router/pns_meander_placer_base.cpp:96`) and the two amplitude fields on `MEANDER_SETTINGS`. Section 4.2.

**`TuningInfo` is not on the placer.** It is `PCB_TUNING_PATTERN::m_tuningInfo`, a `wxString` on the board item, composed at `pcbnew/generators/pcb_tuning_pattern.cpp:1351` from `TuningLengthResult()` and `TuningStatus()`. Section 10.3.

**`MEANDER_PLACER_BASE::Constraint` does not exist.** The one constraint query in the tuning core is inside `Clearance()` (`pcbnew/router/pns_meander_placer_base.cpp:122`); the length and skew constraints are queried by the host, in `PCB_TUNING_PATTERN::EditStart` (`pcbnew/generators/pcb_tuning_pattern.cpp:709`, `:746`, `:771`). Section 10.2.

Two more absences worth stating up front, because they shape the whole milestone.

**`router_tool.cpp` contains no tuning code.** A grep for `PNS_MODE_TUNE`, `MEANDER`, `TuningStatus` or `tuning` over `pcbnew/router/router_tool.cpp` returns nothing. Length tuning at this commit is a board **generator**, `PCB_TUNING_PATTERN` (`pcbnew/generators/pcb_tuning_pattern.h:106`), driven by `DRAWING_TOOL::PlaceTuningPattern` (`pcbnew/generators/pcb_tuning_pattern.cpp:2497`). Section 10.

**There is no meander unit test.** `qa/tests/pcbnew/test_meander_corner_radius.cpp` includes `router/pns_meander.h` and then re-implements the corner radius arithmetic inline (`:122` to `:126`); it never constructs a `MEANDER_SHAPE` and never calls `Fit`. The only assertion it makes about KiCad's own code is on four `MEANDER_SETTINGS` defaults (`:43` to `:52`). The other three tuning tests exercise the topology walk (`test_pns_tuning_path_through_pad.cpp:110`), the generator's layer handling (`test_tuning_pattern_layer.cpp:93`) and `AssembleDiffPair` (`test_pns_diff_pair_tuning_width.cpp:81`), never the shape generator. Section 14 is therefore not optional.

---

## 1. `MEANDER_SETTINGS`

`pcbnew/router/pns_meander.h:69`. A plain settings bag, copied by value into the placer (`MEANDER_PLACER_BASE::m_settings`, `pcbnew/router/pns_meander_placer_base.h:183`) and back out through `MeanderSettings()` (`:104`).

### 1.1 The five constants

| Constant | Line | Value at this commit |
| --- | --- | --- |
| `DEFAULT_LENGTH_TOLERANCE` | `pcbnew/router/pns_meander.cpp:31` | `pcbIUScale.mmToIU( 0.1 )` = 100000 nm |
| `LENGTH_UNCONSTRAINED` | `:32` | `1000000 * IU_PER_MM` = 1e12 nm, one kilometre |
| `DEFAULT_DELAY_TOLERANCE` | `:34` | `0.1 * IU_PER_PS` = 100000 attoseconds |
| `DELAY_UNCONSTRAINED` | `:35` | `1000000 * IU_PER_PS` = 1e12 as, one microsecond |
| `SKEW_UNCONSTRAINED` | `:37` | `std::numeric_limits<int>::max()` = 2147483647 |

`IU_PER_MM = 1e6` and `IU_PER_PS = 1e6` (`include/base_units.h:68`, `:76`), so `DEFAULT_LENGTH_TOLERANCE` and `DEFAULT_DELAY_TOLERANCE` are the same number, 100000. That coincidence hides erratum E2.

The three "unconstrained" values are sentinels, not clamps. `LENGTH_UNCONSTRAINED` is a length the router will happily try to reach; the meander simply runs out of baseline first and the status settles on `TOO_SHORT`.

### 1.2 The fields

`pcbnew/router/pns_meander.h:100` to `:166`, with the defaults from the constructor (`pcbnew/router/pns_meander.cpp:40`).

| Field | Line | Default | What reads it |
| --- | --- | --- | --- |
| `m_minAmplitude` | `pns_meander.h:101` | 200000 | `MEANDER_SHAPE::MinAmplitude` (`pns_meander.cpp:413`), `AmplitudeStep` (`pns_meander_placer_base.cpp:99`) |
| `m_maxAmplitude` | `:104` | 1000000 | `Fit`'s search start (`pns_meander.cpp:781`), `AmplitudeStep` (`:98`) |
| `m_spacing` | `:107` | 600000 | `MEANDER_SHAPE::spacing` (`pns_meander.cpp:460`, `:465`), `SpacingStep`, `MEANDER_PLACER::CheckFit` (`pns_meander_placer.cpp:408`) |
| `m_step` | `:110` | 50000 | `Fit`'s amplitude decrement (`pns_meander.cpp:789`), `MeanderSegment`'s two break tests (`:317`, `:385`) and its skip advance (`:394`), both step methods |
| `m_lenPadToDie` | `:113` | 0 | **Nothing.** Erratum E1. |
| `m_signalExtraLength` | `:117` | 0 | `MEANDER_PLACER::origPathLength` (`pns_meander_placer.cpp:123`) only |
| `m_signalExtraDelay` | `:122` | 0 | `origPathDelay` (`:130`), `calculateTimeDomainTargets` (`:142`) |
| `m_targetLength` | `:125` | `MINOPTMAX<long long>` set to unconstrained | `MEANDER_PLACER::Move` (`:223`), `doMove`'s early test (`:282`) |
| `m_targetLengthDelay` | `:128` | unconstrained | `calculateTimeDomainTargets` (`:148`) |
| `m_targetSignalLength` | `:131` | unconstrained | the chain budget block (`:199` to `:219`) |
| `m_targetSignalLengthDelay` | `:134` | unconstrained | `calculateTimeDomainTargets` (`:145`, `:147`) |
| `m_targetSkew` | `:137` | opt 0, min `-100000`, max `+100000` | `MEANDER_SKEW_PLACER::Move` (`pns_meander_skew_placer.cpp:234` to `:236`) |
| `m_targetSkewDelay` | `:140` | opt 0, min `-100000`, max `+100000` | `MEANDER_SKEW_PLACER::calculateTimeDomainTargets` (`:271`, `:274`, `:277`) |
| `m_overrideCustomRules` | `:142` | false | host only (`pcb_tuning_pattern.cpp:699`) |
| `m_cornerStyle` | `:145` | `MEANDER_STYLE_ROUND` | `MinAmplitude` (`pns_meander.cpp:415`), `cornerRadius` (`:436`), `makeMiterShape` (`:489`) |
| `m_cornerRadiusPercentage` | `:148` | 80 | `cornerRadius` (`:449`) |
| `m_singleSided` | `:151` | false | `MeanderSegment` (`pns_meander.cpp:258`, `:302`, `:320`, `:364`) |
| `m_initialSide` | `:154` | `MEANDER_SIDE_LEFT` (-1) | both placers' side choice (`pns_meander_placer.cpp:265`, `pns_dp_meander_placer.cpp:456`), `flipInitialSide` (`pns_meander.cpp:286`) |
| `m_lengthTolerance` | `:157` | 0 | **Nothing.** Erratum E1. |
| `m_keepEndpoints` | `:160` | false, forced true by the host | `doMove`'s reassembly (`pns_meander_placer.cpp:342`, `pns_dp_meander_placer.cpp:536`) |
| `m_isTimeDomain` | `:163` | false | the delay paths throughout |
| `m_netClass` | `:166` | nullptr | passed straight to the host length hooks (`pns_meander_placer_base.cpp:313`, `:323`) |

`MEANDER_STYLE` is a two value enum, `MEANDER_STYLE_ROUND = 1` and `MEANDER_STYLE_CHAMFER` (`pns_meander.h:53`). `MEANDER_SIDE` is `LEFT = -1`, `DEFAULT = 0`, `RIGHT = 1` (`:59`). `MEANDER_TYPE` has nine members (`:40`): `MT_SINGLE`, `MT_START`, `MT_FINISH`, `MT_TURN`, `MT_CHECK_START`, `MT_CHECK_FINISH`, `MT_CORNER`, `MT_ARC`, `MT_EMPTY`.

Note the split between the enum's zero and the constructor's default: `MEANDER_SIDE_DEFAULT` is 0, but neither `MEANDER_SETTINGS` (`pns_meander.cpp:59`) nor `PCB_TUNING_PATTERN` (`pcb_tuning_pattern.cpp:191`) ever leaves it there. `m_initialSide == 0` selects the "follow the cursor" branch (`pns_meander_placer.cpp:266`, `pns_dp_meander_placer.cpp:457`), which is therefore reachable only from a host that asks for it explicitly.

### 1.3 The six setter pairs

Each target has a `Set...( scalar )` that fills all three of min, opt and max, and a `Set...( const MINOPTMAX<int>& )` that calls the scalar form with `Opt()` and then overwrites min and max from the constraint when it has them.

```
SetTargetLength( long long aOpt ):                       pns_meander.cpp:67
    m_targetLength.SetOpt( aOpt )
    if aOpt == LENGTH_UNCONSTRAINED:
        SetMin( 0 ); SetMax( aOpt )                      # min 0, max one kilometre
    else:
        SetMin( aOpt - DEFAULT_LENGTH_TOLERANCE )        # +/- 0.1 mm
        SetMax( aOpt + DEFAULT_LENGTH_TOLERANCE )

SetTargetLength( const MINOPTMAX<int>& c ):              :84
    SetTargetLength( c.Opt() )
    if c.HasMin(): m_targetLength.SetMin( c.Min() )
    if c.HasMax(): m_targetLength.SetMax( c.Max() )
```

`SetTargetLengthDelay` (`:96`, `:113`), `SetTargetSignalLengthDelay` (`:124`, `:170`) and `SetTargetSignalLength` (`:141`, `:158`) are the same shape with `DELAY_UNCONSTRAINED` / `DEFAULT_DELAY_TOLERANCE` and `LENGTH_UNCONSTRAINED` / `DEFAULT_LENGTH_TOLERANCE` respectively.

`SetTargetSkew( int aOpt )` (`:182`) compares against `SKEW_UNCONSTRAINED` and pads with `DEFAULT_LENGTH_TOLERANCE`, which is right: a skew is a length. `SetTargetSkewDelay( int aOpt )` (`:211`) compares against `SKEW_UNCONSTRAINED` (an int sentinel used for a delay) and pads with `DEFAULT_LENGTH_TOLERANCE` rather than `DEFAULT_DELAY_TOLERANCE`. Erratum E2; numerically invisible at this commit because both constants are 100000.

`MINOPTMAX` (`libs/core/include/core/minoptmax.h:25`) answers `Min()` as 0 when unset, `Max()` as `numeric_limits<T>::max()` when unset and `Opt()` as `Min()` when unset (`:29` to `:31`). Every setter above sets all three, so the unset answers only matter for a `MINOPTMAX` that arrives from a DRC constraint.

### 1.4 What the crate needs of this type

Nine of the twenty two fields are for KiCad's time domain and net chain features, which this crate has no concept of: `m_signalExtraLength`, `m_signalExtraDelay`, `m_targetLengthDelay`, `m_targetSignalLength`, `m_targetSignalLengthDelay`, `m_targetSkewDelay`, `m_isTimeDomain`, `m_netClass`, `m_overrideCustomRules`. Two more are dead in KiCad itself (`m_lenPadToDie`, `m_lengthTolerance`, erratum E1). That leaves eleven that the port genuinely needs, and one of those, `m_cornerStyle`, can only take one of its two values until arcs exist (section 9.4).

---

## 2. `MEANDER_SHAPE`

`pcbnew/router/pns_meander.h:172`. One meander: a type, an amplitude, the base segment it sits on, and one or two generated line chains. It is a **value**: `MEANDERED_LINE` stores `MEANDER_SHAPE*` and news and deletes them (`pns_meander.cpp:297`, `:938`), but the copy constructor is the implicit one and the class is copied by value all over `tuneLineLength` (`pns_meander_placer_base.cpp:213`, `:183`, `:958`).

### 2.1 State

`pns_meander.h:414` to `:462`. Eighteen members, of which four are the turtle's scratch state.

| Member | Line | Meaning |
| --- | --- | --- |
| `m_type` | `:414` | Which of the nine `MEANDER_TYPE`s. |
| `m_placer` | `:417` | Back pointer, used only to reach `MeanderSettings()` (`pns_meander.cpp:242`), `Clearance()` (`:460`) and `CheckFit()` (`:815`). |
| `m_dual` | `:420` | Two chains rather than one. |
| `m_width` | `:423` | Track width, for the corner radius floors and `CheckFit`'s clearance. |
| `m_amplitude` | `:426` | The excursion height. |
| `m_baselineOffset` | `:429` | Half the pair pitch, signed. Zero for a single track. |
| `m_meanCornerRadius` | `:432` | **Output** of `genMeanderShape` (`pns_meander.cpp:610`), read back by `Fit` (`:812`) and by `makeMiterShape` (`:506`). |
| `m_targetBaseLen` | `:435` | When non zero, widens `top` so a resized meander keeps its baseline footprint. |
| `m_p0` | `:438` | Where the meander starts on the base segment. |
| `m_baseSeg` | `:441` | The whole segment being meandered. |
| `m_clippedBaseSeg` | `:444` | The part of it this meander consumes, computed by `updateBaseSegment`. |
| `m_side` | `:447` | True means mirrored across the base line. |
| `m_shapes[2]` | `:450` | The generated chains; `[1]` is used only when dual. |
| `m_baseIndex` | `:453` | Index of the base segment in the original line. Set by `MeanderSegment` (`pns_meander.cpp:275`) and by `Fit`'s check path (`:769`); **never read** by anything in the tree. Erratum E1. |
| `m_currentDir`, `m_currentPos`, `m_currentTarget` | `:456`, `:459`, `:462` | Turtle scratch. |

The constructor (`pns_meander.h:181`) takes the placer, the width and the dual flag and zeroes the rest.

### 2.2 The three derived dimensions

`spacing()` (`pns_meander.cpp:456`) is the meander period, floored by what physically fits:

```
spacing():
    if not dual:  return max( m_width + placer->Clearance(),                 :460
                              Settings().m_spacing )
    else:         return max( m_width + placer->Clearance() + 2*|offset|,    :464
                              Settings().m_spacing )
```

`placer->Clearance()` is a rule resolver query (section 4.2). `spacing()` is called from `cornerRadius()` (`:442`), from `genMeanderShape()` (`:591`) and twice per iteration of `MeanderSegment`'s loop (`:277`, `:394`), so the query is on the hot path and is not cached. Erratum E11.

`cornerRadius()` (`:429`):

```
cornerRadius():
    if m_amplitude == 0: return 0                                            :431
    minCr = |offset| + m_width/2                            if ROUND         :437
    minCr = |offset| + m_width/2 * (1 - tan(22.5 deg))      if CHAMFER       :439
    maxCr = min( (m_amplitude + |offset|)/2, spacing()/2 )                    :441
    if maxCr < minCr: return maxCr                          # wxCHECK2_MSG    :445
    optCr = spacing() * m_cornerRadiusPercentage / 200                        :450
    return clamp( optCr, minCr, maxCr )                                       :452
```

The `/200` rather than `/100` is deliberate: the percentage is of the **half** period, so 100 percent means a corner radius of exactly half the spacing, which is the maximum that fits. `1 - tan(22.5 deg) = 0.5857864`, so for a 200000 nm track the chamfer floor is 58578 nm against the round floor of 100000 nm.

`MinAmplitude()` (`:411`):

```
MinAmplitude():
    a = Settings().m_minAmplitude
    if ROUND:    return max( a, |offset| + m_width )                          :417
    else:        return max( a, |offset| + m_width * tan(1 - tan(22.5 deg)) ) :422
```

The chamfer branch is a typo, erratum E3: `tan( 1 - tan( DEG2RAD( 22.5 ) ) ) = 0.6634703`, where the same expression written correctly one function down (`:439`) is `1 - tan( DEG2RAD( 22.5 ) ) = 0.5857864`. For a 200000 nm track the correction is 132694 instead of 117157, 13.3 percent too large. It is masked whenever `m_minAmplitude` (default 200000) dominates, which is the usual case. The same typo is copied into the host at `pcbnew/generators/pcb_tuning_pattern.cpp:1626`.

### 2.3 The turtle

Five private members implement a Logo style turtle over a `SHAPE_LINE_CHAIN*`.

```
start( target, where, dir ):                                                  :530
    m_currentTarget = target;  target->Clear();  target->Append( where )
    m_currentDir = dir;  m_currentPos = where

forward( length ):                                                            :540
    if length < 5: return                     # also swallows negatives       :543
    m_currentPos += m_currentDir.Resize( length )
    m_currentTarget->Append( m_currentPos )

turn( angle ):                                                                :551
    RotatePoint( m_currentDir, angle )        # only ever +/- ANGLE_90

miter( radius, side ):                                                        :557
    if radius <= 0:
        turn( side ? +90 : -90 );  return                                     :561
    dir = m_currentDir.Resize( radius )
    lc  = makeMiterShape( m_currentPos, dir, side )
    m_currentPos = lc.CLastPoint()
    turn( side ? +90 : -90 )
    m_currentTarget->Append( lc )

uShape( sides, corner, top ):                                                 :575
    forward( sides ); miter( corner, true ); forward( top )
    miter( corner, true ); forward( sides )
```

Three facts a port depends on.

**`turn` is exact.** The only two arguments in the tree are `ANGLE_90` and `-ANGLE_90` (`:561`, `:569`, `:643`, `:661`). `RotatePoint( double*, double*, EDA_ANGLE )` normalises first (`libs/kimath/src/trigo.cpp:296`; `Normalize()` maps -90 to 270, `libs/kimath/include/geometry/eda_angle.h:229`) and then takes the exact branch: 90 gives `(y, -x)` (`trigo.cpp:305`), 270 gives `(-y, x)` (`:313`). No trigonometry runs. `m_currentDir` is therefore always a component permutation with signs of the initial direction, which is an integer vector.

**`forward` has a 5 nm dead zone.** `if( aLength < 5 ) return` (`:543`) is documented as "very small segments cause problems", and it also silently swallows negative lengths, which `startSide = amplitude - 2*cr + |offset|` can produce.

**`miter` appends the corner chain after moving the turtle**, so `m_currentPos` is already the corner's far end when `Append( lc )` runs. The seam duplicate is dropped by `SHAPE_LINE_CHAIN::Append( const SHAPE_LINE_CHAIN& )` (`libs/kimath/src/geometry/shape_line_chain.cpp:1572`).

### 2.4 `makeMiterShape`: where the two corner styles diverge

`pns_meander.cpp:470`. This is the only function in the router that branches on `m_cornerStyle`, and it is the only place `SHAPE_ARC` is constructed.

```
makeMiterShape( P, dir, side ):
    if |dir| == 0:  return chain{ P }                                         :475
    dir_u = dir
    dir_v = dir.Perpendicular()                    # (-y, x)
    endPoint = P + dir_u + dir_v * (side ? -1 : +1)                           :484
    append (int)P.x, (int)P.y                                                 :486

    ROUND:                                                                    :491
        arc.ConstructFromStartEndAngle( P, (VECTOR2I) endPoint,
                                        side ? -ANGLE_90 : ANGLE_90 )         :496
        chain.Append( arc )                                                   :497

    CHAMFER:                                                                  :501
        radius = |dir|
        correction = 0
        if dual and radius > m_meanCornerRadius:                              :506
            correction = -2 * |m_baselineOffset| * tan(22.5 deg)
        dir_cu = dir_u.Resize( correction )
        dir_cv = dir_v.Resize( correction )
        append P - dir_cu                                                     :512
        append P + dir_u + (dir_v + dir_cv) * (side ? -1 : +1)                :514
        append endPoint                                                       :517
```

For a **single** track `m_dual` is false, so `correction` is 0, `dir_cu` and `dir_cv` are the zero vector, and the three chamfer appends collapse to `P`, `endPoint`, `endPoint`. With the leading `append P` at `:486` and `Append`'s duplicate suppression (`shape_line_chain.h:539`) the chamfer corner is exactly two points: `P` and `endPoint`, a single 45 degree chord of length `radius * sqrt(2)` replacing a right angle whose apex would have been at `P + dir_u`.

For a **dual** meander the correction pulls the chamfer's start back along the direction of travel and pushes its middle point out sideways, by `2 * |offset| * tan(22.5 deg)`, so that the inner and outer lanes of a pair stay a constant gap apart around the corner. The `radius > m_meanCornerRadius` guard makes the correction apply only to corners wider than the mean, which is the outer lane.

`dir_u` and `dir_v` are both of length `radius`, so the round corner is a quarter circle of that radius and the chamfer cuts `radius` off each leg. The two styles therefore consume the same amount of baseline and differ only in the corner's own length: `radius * pi/2` against `radius * sqrt(2)`.

### 2.5 `genMeanderShape`: the five shapes

`pns_meander.cpp:585`. Takes the start point, the **whole base segment vector** as the direction (not a unit vector; `Resize` makes the magnitude irrelevant), the side, the type and the baseline offset.

```
genMeanderShape( P, dir, side, type, offset ):
    cr  = cornerRadius()                                                      :589
    spc = spacing()
    amplitude = m_amplitude
    targetBaseLen = m_targetBaseLen
    if side: offset = -offset                                                 :595

    dir_u_b = dir.Resize( offset )                                            :598
    dir_v_b = dir_u_b.Perpendicular()

    if 2*cr > amplitude + |offset|:  cr = (amplitude + |offset|)/2            :601
    if 2*cr > spc:                   cr = spc/2                               :604
    if cr - offset < 0:              cr = offset                              :607
    m_meanCornerRadius = cr                                                   :610

    sCorner   = cr - offset                                                   :612
    uCorner   = cr + offset
    startSide = amplitude - 2*cr + |offset|
    turnSide  = amplitude - cr
    top       = spc - 2*cr                                                    :616

    start( &lc, P + dir_v_b, dir )                                            :620
    switch type: ...
    if side: lc.Mirror( SEG( P, P + dir ) )                                   :681
    m_currentTarget = nullptr                                                 :689
    return lc
```

The five bodies, with `offset == 0` (the single track case) shown after each:

```
MT_EMPTY:                                                                     :624
    append P + dir_v_b + dir            # the clipped base segment, translated

MT_START:                                                                     :628
    if targetBaseLen: top = max( top, targetBaseLen - sCorner - 2*uCorner + offset )
    miter( sCorner, false )
    uShape( startSide, uCorner, top )
    forward( min( sCorner, uCorner ) )
    forward( |offset| )
    # offset 0:  miter(cr,false); uShape(A-2cr, cr, spc-2cr); forward(cr)

MT_FINISH:                                                                    :638
    if targetBaseLen: top = max( top, targetBaseLen - cr - spc )
    start( &lc, P - dir_u_b, dir );  turn( -90 )
    forward( min( sCorner, uCorner ) );  forward( |offset| )
    uShape( startSide, uCorner, top )
    miter( sCorner, false )
    append P + dir_v_b + dir.Resize( targetBaseLen >= spc + cr
                                     ? targetBaseLen : 2*spc - cr )           :649

MT_TURN:                                                                      :656
    if targetBaseLen: top = max( top, targetBaseLen - 2*uCorner + 2*offset )
    start( &lc, P - dir_u_b, dir );  turn( -90 )
    forward( |offset| )
    uShape( turnSide, uCorner, top )
    forward( |offset| )

MT_SINGLE:                                                                     :667
    if targetBaseLen: top = max( top, (targetBaseLen - 2*sCorner - 2*uCorner)/2 )
    miter( sCorner, false )
    uShape( startSide, uCorner, top )
    miter( sCorner, false )
    append P + dir_v_b + dir.Resize( 2*spc )                                   :674
```

`MT_CHECK_START`, `MT_CHECK_FINISH`, `MT_CORNER` and `MT_ARC` fall through the `default:` at `:677` and produce a chain holding only the start point.

Two of the five end with an **absolute** append rather than a turtle move: `MT_SINGLE` at `:674` and `MT_FINISH` at `:650`/`:652`. That is what pins their baseline footprint to exactly `2 * spacing` and `2 * spacing - cr`. `MT_START` and `MT_TURN` end wherever the turtle happens to be, which is on the base line for `MT_START` (it comes back down) and on the base line for `MT_TURN` (it crosses).

### 2.6 The five shapes, worked out for a single track

With `offset == 0`, base segment along `+x` from `(0,0)`, `side == false`, `A` the amplitude, `s` the spacing, `c` the corner radius after the three clamps, and `targetBaseLen == 0`, the turtle produces these exact integer point lists. `d` below is `KiROUND( c * sqrt(2) )`, the length of one chamfered corner as `SHAPE_LINE_CHAIN::Length` counts it (`libs/kimath/src/geometry/shape_line_chain.cpp:964`, which sums per segment `SEG::Length`, and `VECTOR2<int>::EuclideanNorm` takes the exact 45 degree branch, `libs/kimath/include/math/vector2d.h:282`).

**`MT_SINGLE`**, chamfered:

```
(0,0) (c,c) (c,A-c) (2c,A) (s,A) (s+c,A-c) (s+c,c) (s+2c,0) (2s,0)
CurrentLength  = 4d + 2(A - 2c) + 2(s - 2c)
BaselineLength = 2s
elongation     = 4d + 2A - 8c        =  2A - 2c(4 - 2*sqrt(2))  approx  2A - 2.343 c
```

**`MT_START`**, chamfered:

```
(0,0) (c,c) (c,A-c) (2c,A) (s,A) (s+c,A-c) (s+c,c) (s+c,0)
CurrentLength  = 3d + 2(A - 2c) + (s - 2c) + c
BaselineLength = s + c
elongation     = 3d + 2A - 6c        approx  2A - 1.757 c
```

**`MT_TURN`**, chamfered:

```
(0,0) (0,A-c) (c,A) (s-c,A) (s,A-c) (s,0)
CurrentLength  = 2d + 2(A - c) + (s - 2c)
BaselineLength = s
elongation     = 2d + 2A - 4c        approx  2A - 1.172 c
```

**`MT_FINISH`**, chamfered:

```
(0,0) (0,c) (0,A-c) (c,A) (s-c,A) (s,A-c) (s,c) (s+c,0) (2s-c,0)
CurrentLength  = 3d + c + 2(A - 2c) + (s - 2c) + (s - 2c)
BaselineLength = 2s - c
elongation     = 3d + 2A - 6c        approx  2A - 1.757 c
```

**`MT_EMPTY`**: the clipped base segment itself, elongation zero.

For the round style, replace each `d` with `KiROUND` of the arc length that `SHAPE_ARC::GetLength()` reports for a 90 degree arc of radius `c`, which is `c * pi / 2` up to KiCad's arc discretisation. The point lists are otherwise identical, because `makeMiterShape` puts the arc between exactly the same two endpoints as the chord.

A concrete case for the port's first unit test, with `width = 200000`, `clearance = 100000`, `m_spacing = 600000`, `m_cornerRadiusPercentage = 80`, `amplitude = 1000000`, chamfered corners:

```
spacing()      = max( 200000 + 100000, 600000 )              = 600000
cornerRadius() : minCr = 100000 * 0.5857864                  = 58578
                 maxCr = min( 1000000/2, 600000/2 )          = 300000
                 optCr = 600000 * 80 / 200                   = 240000
                 clamp                                       = 240000
genMeanderShape: 2*240000 <= 1000000 and <= 600000           -> c stays 240000
Fit's reject test: m_meanCornerRadius 240000 >= width/2 100000  -> accepted

MT_SINGLE points: (0,0) (240000,240000) (240000,760000) (480000,1000000)
                  (600000,1000000) (840000,760000) (840000,240000)
                  (1080000,0) (1200000,0)
d              = KiROUND( 240000 * sqrt(2) ) = KiROUND( 339411.2550 ) = 339411
CurrentLength  = 4*339411 + 2*520000 + 2*120000              = 2637644
BaselineLength = 1200000
elongation     = 1437644
```

### 2.7 `Fit`: the amplitude search and the two check types

`pns_meander.cpp:723`. This is the only entry point that produces a fitted meander.

```
Fit( type, seg, P, side ):
    if type == MT_CHECK_START:   prim1, prim2 = MT_START, MT_TURN;   check    :731
    if type == MT_CHECK_FINISH:  prim1, prim2 = MT_TURN,  MT_FINISH; check    :737

    if check:                                                                 :744
        m1, m2 = two fresh MEANDER_SHAPEs with this baseline offset
        c1 = m1.Fit( prim1, seg, P, side )                                    :752
        c2 = c1 and m2.Fit( prim2, seg, m1.End(), !side )                     :756
        if c1 and c2:
            adopt m1's shapes, amplitude, dual flag, base segment, base index :760
            m_type = prim1;  m_p0 = P;  m_side = side
            updateBaseSegment()
            return true
        return false

    minAmpl = MinAmplitude()                                                  :780
    maxAmpl = max( Settings().m_maxAmplitude, minAmpl )                       :781
    minCornerRadius = m_width / 2                                             :787

    for ampl = maxAmpl down to minAmpl step -m_step:                          :789
        m_amplitude = ampl
        if dual:  shapes[0] = gen(P, seg.B-seg.A, side, type, +offset)        :795
                  shapes[1] = gen(P, seg.B-seg.A, side, type, -offset)        :796
        else:     shapes[0] = gen(P, seg.B-seg.A, side, type, 0)              :800
        m_type = type;  m_baseSeg = seg;  m_p0 = P;  m_side = side
        updateBaseSegment()                                                   :808
        if m_meanCornerRadius < minCornerRadius: continue                     :812
        if placer->CheckFit( this ): return true                              :815
    return false
```

Three points.

The **check types never produce geometry of their own**. `MT_CHECK_START` asks "can I fit a START here and a TURN after it", and on success the shape becomes a plain `MT_START`. That two step lookahead is what lets `MeanderSegment` decide whether to open a turning run or to place an isolated single (section 3.2).

The **amplitude search is a linear scan downwards** in `m_step` increments, largest first, and it stops at the first amplitude that both clears the corner radius floor and passes `CheckFit`. So a meander is always as tall as it can be at fitting time; `tuneLineLength` shrinks it afterwards (section 4.5). With `m_step == 0` the loop never terminates; nothing in KiCad's tree can set it to zero, but the port should reject it at the settings boundary.

The **corner radius rejection at `:812` applies to both styles**. It reads `m_meanCornerRadius`, which `genMeanderShape` wrote at `:610` after its own three clamps, and compares against `m_width / 2`. For the chamfer style the floor inside `cornerRadius()` is `m_width/2 * 0.5857864`, strictly below `m_width/2`, so a chamfered meander whose optimal radius is clamped down to that floor is then rejected here. The two floors disagree by design; the comment at `:810` cites issue 8629 and the intent is visual, not geometric.

### 2.8 The mutators and the measurements

```
Recalculate():                                                                :823
    shapes[0] = gen( m_p0, m_baseSeg.B - m_baseSeg.A, m_side, m_type,
                     m_dual ? m_baselineOffset : 0 )
    if dual: shapes[1] = gen( ..., -m_baselineOffset )
    updateBaseSegment()

Resize( ampl ):                                                               :836
    if ampl < 0: return
    m_amplitude = max( ampl, MinAmplitude() )                                 :844
    Recalculate()

MakeEmpty():                                                                  :850
    updateBaseSegment()                       # note: before, not after       :852
    dir = m_clippedBaseSeg.B - m_clippedBaseSeg.A
    m_type = MT_EMPTY;  m_amplitude = 0
    shapes[0] = gen( m_p0, dir, m_side, MT_EMPTY, m_dual ? offset : 0 )
    if dual: shapes[1] = gen( m_p0, dir, m_side, MT_EMPTY, -offset )

MakeCorner( p1, p2 ):                                                         :904
    m_type = MT_CORNER
    shapes[0] = { p1 };  shapes[1] = { p2 }
    m_clippedBaseSeg = SEG( p1, p1 )          # degenerate, length 0

MakeArc( arc1, arc2 ):                                                        :916
    m_type = MT_CORNER                        # not MT_ARC; erratum E4
    shapes[0] = { arc1 };  shapes[1] = { arc2 }
    m_clippedBaseSeg = SEG( arc1.GetP1(), arc1.GetP1() )    # the arc END

updateBaseSegment():                                                          :967
    if dual:                                                                  :969
        midA = ( shapes[0].CPoint(0)  + shapes[1].CPoint(0)  ) / 2
        midB = ( shapes[0].CLastPoint() + shapes[1].CLastPoint() ) / 2
        m_clippedBaseSeg = SEG( m_baseSeg.LineProject( midA ),
                                m_baseSeg.LineProject( midB ) )
    else:                                                                     :979
        m_clippedBaseSeg = SEG( m_baseSeg.LineProject( shapes[0].CPoint(0) ),
                                m_baseSeg.LineProject( shapes[0].CLastPoint() ) )

BaselineLength()  = m_clippedBaseSeg.Length()                                 :944
CurrentLength()   = CLine(0).Length()                                         :950
MinTunableLength():                                                           :956
    copy = *this
    copy.SetTargetBaselineLength( BaselineLength() )
    copy.Resize( copy.MinAmplitude() )
    return copy.CurrentLength()
```

`MakeEmpty` uses the **clipped** base segment as its direction (`:854`) where `Recalculate` uses the **unclipped** one (`:825`), and it calls `updateBaseSegment` before generating rather than after, so an emptied meander keeps the clipped segment it had. That is what makes an emptied meander a straight bypass of exactly the baseline it used to consume.

`MinTunableLength` is the shortest this meander can be made without changing its footprint, and it is what `tuneLineLength` uses to decide whether a meander is worth keeping at all.

`Amplitude()` (`pns_meander.h:232`), `Side()` (`:279`), `End()` (`:287`, which is `m_clippedBaseSeg.B`), `CLine( int )` (`:295`), `BaseSegment()` (`:322`), `Width()` (`:355`), `Type()` (`:208`), `SetType` (`:200`), `SetBaselineOffset` (`:366`), `SetTargetBaselineLength` (`:377`), `IsDual` (`:271`), `Settings()` (`pns_meander.cpp:240`) are one liners. `BaseIndex()` / `SetBaseIndex` (`pns_meander.h:224`, `:216`) are written and never read, erratum E1.

---

## 3. `MEANDERED_LINE`

`pcbnew/router/pns_meander.h:469`. An ordered list of `MEANDER_SHAPE*` covering one stretch of a line, plus the fitting loop that produces it.

### 3.1 State and the adders

Five members (`:611` to `:618`): `m_last` (the point the next meander starts from), `m_placer`, `m_meanders`, `m_dual`, `m_width`, `m_baselineOffset`. It **owns** the shapes: `Clear()` deletes every one (`pns_meander.cpp:938`) and the destructor calls `Clear()` (`pns_meander.h:496`). There is a move assignment operator (`:593`) and no copy assignment, which is what lets `doMove` write `m_result = MEANDERED_LINE( this, false )` (`pns_meander_placer.cpp:243`) without leaking.

```
AddCorner( a, b = (0,0) ):                                                    :866
    m = new MEANDER_SHAPE( placer, width, dual )
    m->MakeCorner( a, b );  m_last = a;  push_back( m )

AddArc( arc1, arc2 = SHAPE_ARC() ):                                           :877
    m->MakeArc( arc1, arc2 );  m_last = arc1.GetP1();  push_back( m )

AddArcAndPt( arc1, pt2 ):  AddArc( arc1, SHAPE_ARC( pt2, pt2, pt2, 0 ) )      :888
AddPtAndArc( pt1, arc2 ):  AddArc( SHAPE_ARC( pt1, pt1, pt1, 0 ), arc2 )      :896

AddMeander( shape ):                                                          :928
    m_last = shape->BaseSegment().B;  push_back( shape )
```

`AddPtAndArc` has no caller anywhere in the tree; `AddArcAndPt` has none either. `AddArc` is called from `MEANDER_PLACER::doMove` (`pns_meander_placer.cpp:252`) and from the pair placer's corner walk (`pns_dp_meander_placer.cpp:398`, `:412`, `:433`). Erratum E5.

Note the asymmetry between `AddCorner`, which advances `m_last` to the point it was given, and `AddMeander`, which advances it to the **clipped base segment's** far end rather than to the last point of the generated chain. That is what keeps the meanders marching along the base line rather than along the meandered path.

### 3.2 `MeanderSegment`: the fitting loop

`pns_meander.cpp:252`. Given one base segment, fill it with as many meanders as fit. This is the heart of the milestone.

```
MeanderSegment( base, side, baseIndex = 0 ):
    base_len = base.Length()                       # int, in a double         :254
    singleSided = Settings().m_singleSided                                    :258
    dir = VECTOR2D( base.B - base.A )                                         :260
    if not dual: AddCorner( base.A )                                          :263
    turning = false;  started = false;  m_last = base.A                       :265

    loop:
        m = MEANDER_SHAPE( placer, width, dual )                              :272
        m.SetBaselineOffset( m_baselineOffset );  m.SetBaseIndex( baseIndex )
        thr = m.spacing()                                                     :277
        fail = false
        remaining = base_len - |m_last - base.A|                              :280

        flipInitialSide():                                                    :282
            s = placer->MeanderSettings();  s.m_initialSide = -s.m_initialSide
            placer->UpdateSettings( s )                                       :287

        addSingleIfFits():                                                    :290
            fail = true
            if m.Fit( MT_SINGLE, base, m_last, side ):
                AddMeander( new MEANDER_SHAPE( m ) );  fail = false; started = false
            if fail and not singleSided:                                      :302
                if m.Fit( MT_SINGLE, base, m_last, !side ):
                    if not started: flipInitialSide()                         :307
                    AddMeander( ... );  fail = false; started = false; side = !side

        if remaining < Settings().m_step:  break                              :317

        if not singleSided and remaining > 3.0 * thr:                         :320
            if not turning:                                                   :322
                for checkSide in [ side, !side ]:                             :324
                    if m.Fit( MT_CHECK_START, base, m_last, checkSide ):      :328
                        if not started and checkSide != side: flipInitialSide()
                        turning = true;  AddMeander( new MEANDER_SHAPE( m ) )
                        side = !checkSide;  started = true;  break            :336
                if not turning: addSingleIfFits()                             :342
            else:                                                             :344
                if m.Fit( MT_CHECK_FINISH, base, m_last, side ):              :346
                    m.Fit( MT_TURN, base, m_last, side )                      :350
                    AddMeander( new MEANDER_SHAPE( m ) )
                    side = !side;  started = true
                else:
                    m.Fit( MT_FINISH, base, m_last, side )                    :357
                    started = false;  AddMeander( ... );  turning = false
        elif not singleSided and started:                                     :364
            if m.Fit( MT_FINISH, base, m_last, side ): AddMeander( ... )
            break                                                             :371
        elif not turning and remaining > thr * 2.0:                           :374
            addSingleIfFits()
        else:
            fail = true                                                       :380

        remaining = base_len - |m_last - base.A|                              :383
        if remaining < Settings().m_step: break                               :385

        if fail:                                                              :388
            tmp = MEANDER_SHAPE( placer, width, dual )    # amplitude 0       :390
            nextP = tmp.spacing() - 2 * tmp.cornerRadius() + Settings().m_step :394
            pn = m_last + dir.Resize( nextP )                                 :395
            if base.Contains( pn ) and not dual: AddCorner( pn )              :397
            else: break

    if not dual: AddCorner( base.B )                                          :407
```

Six things to carry over exactly.

**`tmp.cornerRadius()` at `:394` is always zero.** `tmp` is a fresh shape whose `m_amplitude` is 0, and `cornerRadius()` returns 0 for that (`:431`). So the skip advance is always `spacing() + m_step`, and the `- 2 * tmp.cornerRadius()` term is dead. Erratum E6.

**`flipInitialSide` writes back through the placer.** It reads `m_settings`, negates `m_initialSide` and calls `UpdateSettings` (`pns_meander_placer_base.cpp:131`), which overwrites the placer's whole settings struct. Every later read of `Settings()` in the same loop sees the new value, the next base segment's side choice in `doMove` sees it (`pns_meander_placer.cpp:265`), and the host copies it back onto the board item after the move (`pcb_tuning_pattern.cpp:1319`). It is a deliberate feedback path, not a bug, but it means the shape generator mutates its placer, which no other part of the router does. `MEANDER_SIDE_DEFAULT` is 0 and `-0 == 0`, so a default initial side never flips.

**The three arms of the `if` chain are mutually exclusive but not exhaustive in an obvious way.** With `singleSided` true, the first two arms are unreachable and the third fires whenever `remaining > 2 * spacing`, so a single sided run is a plain sequence of `MT_SINGLE` shapes with no turning. With `singleSided` false, the first arm handles the long stretch (`> 3 * spacing`), the second closes an open turning run when the stretch got short, and the third places an isolated single on a medium stretch.

**`MT_CHECK_START` is tried on both sides**, current first (`:324` to `:326`). `addSingleIfFits` likewise tries the current side then the other (`:295`, `:304`). The side that wins is recorded, and if the run had not started yet, the initial side is flipped so that the next `doMove` starts on the side that actually worked.

**All the doubles here are integer valued.** `base_len` is `SEG::Length()`, an `int` (`libs/kimath/include/geometry/seg.h:339`). `|m_last - base.A|` is `VECTOR2<int>::EuclideanNorm()`, also an `int`. `thr` is `spacing()`, an `int`. `3.0 * thr` and `thr * 2.0` are exact for any board sized value. Only `dir.Resize( nextP )` at `:395` is genuinely fractional, and its result is truncated into a `VECTOR2I` on the next line. Section 8.

**`SHAPE_LINE_CHAIN lc;` at `:256` is declared and never used.** Erratum E1.

### 3.3 `CheckSelfIntersections`

`pns_meander.cpp:695`. Called from both placers' `CheckFit`.

```
CheckSelfIntersections( shape, clearance ):
    for m in m_meanders reversed:                                             :697
        if m->Type() in { MT_EMPTY, MT_CORNER }: continue                     :701
        if shape->BaseSegment().ApproxParallel( m->BaseSegment() ): continue  :707
        for j in m->CLine(0).SegmentCount() reversed:                         :712
            if shape->CLine(0).Collide( m->CLine(0).CSegment(j), clearance ):
                return false
    return true
```

The parallel skip at `:707` is what makes the routine cheap: meanders on the same base segment are all parallel to each other and are skipped wholesale, so only meanders from **other** base segments of the same line are actually tested. `SEG::ApproxParallel` defaults its threshold to 1 (`libs/kimath/include/geometry/seg.h:294`).

`MT_ARC` is not in the skip set at `:701` even though `MakeArc` sets `MT_CORNER` (erratum E4), so the two are consistent by accident. Only chain `0` is tested, so for a dual meander the N lane is never checked against anything. Erratum E7.

---

## 4. `MEANDER_PLACER_BASE`

`pcbnew/router/pns_meander_placer_base.h:45`, deriving from `PLACEMENT_ALGO`. Holds the settings, the world branch, the current width and the four pad pointers, and implements the length arithmetic all three placers share.

### 4.1 State

`:166` to `:191`.

| Member | Line | Meaning |
| --- | --- | --- |
| `m_baselineLength`, `m_baselineDelay` | `:166`, `:167` | The path length captured at `Start`, for `TuningLengthDelta` (`:70`). |
| `m_chainExtrasLength`, `m_chainExtrasDelay`, `m_chainExtrasValid` | `:172` to `:174` | The net chain aggregate, section 4.4. |
| `m_world` | `:177` | The branch the placer owns. |
| `m_currentWidth` | `:180` | Width of the tuned track, from the assembled line. |
| `m_settings` | `:183` | The `MEANDER_SETTINGS` copy. |
| `m_currentEnd` | `:186` | Written once to `(0,0)` by two of the three `Start`s and never again. Erratum E1. |
| `m_startPad_p`, `m_endPad_p`, `m_startPad_n`, `m_endPad_n` | `:188` to `:191` | Terminal pads found by the topology walk, for the host's pad to die length. |

`TUNING_STATUS` is `TOO_SHORT = 0`, `TOO_LONG`, `TUNED` (`:49`).

### 4.2 The small members

```
AmplitudeStep( sign ):                                pns_meander_placer_base.cpp:96
    a = m_settings.m_maxAmplitude + sign * m_settings.m_step
    m_settings.m_maxAmplitude = max( a, m_settings.m_minAmplitude )

SpacingStep( sign ):                                  :105
    s = m_settings.m_spacing + sign * m_settings.m_step
    m_settings.m_spacing = max( s, m_currentWidth + Clearance() )

Clearance():                                          :114
    itemToCheck = Traces().CItems().front()                                   :119
    QueryConstraint( CT_CLEARANCE, itemToCheck, nullptr, CurrentLayer(), &c ) :122
    wxCHECK_MSG( c.m_Value.HasMin(), m_currentWidth, "No minimum clearance?" ) :125
    return c.m_Value.Min()

UpdateSettings( s ):    m_settings = s                :131
MeanderSettings():      return m_settings             :301
CheckFit( shape ):      return false                  pns_meander_placer_base.h:120
```

`Clearance()` calls `Traces()`, which on both concrete placers is **not** a pure query: `MEANDER_PLACER::Traces` rebuilds `m_currentTrace` from `m_originLine` and `m_finalShape` (`pns_meander_placer.cpp:416`) and `DP_MEANDER_PLACER::Traces` rebuilds two lines (`pns_dp_meander_placer.cpp:628`). `Clearance()` is reached from `MEANDER_SHAPE::spacing()` (`pns_meander.cpp:460`), which runs several times per candidate amplitude inside `Fit`. So every amplitude trial rebuilds the placer's trace from a stale `m_finalShape` and issues an uncached rule resolver query. Erratum E11.

The `wxCHECK_MSG` fallback returns the **track width** as the clearance when the host has no minimum clearance rule, which then feeds `spacing()`'s floor as `2 * width`.

`CheckFit`'s base implementation returns false, so a placer that forgets to override it fits nothing at all and every `Fit` fails.

### 4.3 `lineLength`, `lineDelay` and the host hooks

```
lineLength( items, startPad, endPad ):                :307
    if items.Empty(): return 0
    return Router()->GetInterface()->CalculateRoutedPathLength(
               items, startPad, endPad, m_settings.m_netClass )               :313

lineDelay( items, startPad, endPad ):                 :317   same shape       :323
```

`CalculateRoutedPathLength` is a pure virtual on `ROUTER_IFACE` (`pcbnew/router/pns_router.h:124`). KiCad's implementation (`pcbnew/router/pns_kicad_iface.cpp:3169`) converts the item set into length calculation items and calls the board's length calculator with `OptimiseVias = false, MergeTracks = false, OptimiseTracesInPads = false, InferViaInPad = true` (`:3183`). The pad entry optimisation is off there because `AssembleTuningPath` has already applied it to the path's chains in place (`pns_topology.cpp:928`, `:1010`).

The six tuning hooks on `ROUTER_IFACE` are `CalculateRoutedPathLength` (`pns_router.h:124`), `CalculateRoutedPathDelay` (`:126`), `CalculateLengthForDelay` (`:128`), `CalculateDelayForShapeLineChain` (`:130`), `GetSignalAggregate` (`:135`) and `GetNetBoardLength` (`:138`, the only one with a default body, `return 0`). Five of the six are pure virtual, so a host that wants any routing at all has to implement them even if it never tunes.

### 4.4 `initChainExtras` and `chainNarrowingOffset`

```
initChainExtras():                                    :51
    m_chainExtras* = 0;  m_chainExtrasValid = false
    nets = CurrentNets();  if empty: return                                   :59
    first  = nets[0]
    second = nets.size() >= 2 ? nets[1] : nets[0]                             :63
    if iface->GetSignalAggregate( first, second, extraLen, extraDelay ):      :68
        m_chainExtrasLength = extraLen;  m_chainExtrasDelay = extraDelay
    m_chainExtrasValid = true

chainNarrowingOffset():                               :78
    if not m_chainExtrasValid: return 0
    tunedNetBoardLen = iface->GetNetBoardLength( CurrentNets()[0] )           :88
    unmeasured = max( 0, tunedNetBoardLen - m_baselineLength )                :90
    return m_chainExtrasLength + unmeasured
```

This is KiCad's "net chain" feature: a logical signal that runs through several nets separated by series components. The aggregate is the copper already contributed by the *other* nets in the chain, and the offset is subtracted from the user facing target so the meander does not try to make up length the chain already has.

Nothing in this crate has a net chain concept, and nothing in LibrePCB does either. In the port both functions collapse to a constant zero, which removes `GetSignalAggregate`, `GetNetBoardLength`, `m_chainExtras*`, `m_signalExtraLength`, `m_signalExtraDelay`, `m_targetSignalLength` and `m_targetSignalLengthDelay` from the milestone.

### 4.5 `tuneLineLength`: three passes over the meander list

`pns_meander_placer_base.cpp:203`. Given a fitted `MEANDERED_LINE` whose meanders are all at maximum amplitude, and the number of nanometres the line must gain, shrink or delete meanders until the total lands on target.

```
tuneLineLength( tuned, elongation ):
    maxElongation = 0;  minElongation = 0;  finished = false

    # Pass 1: truncate the run and turn the last surviving meander into an end shape.
    for m in tuned.Meanders():                                                :209
        if m->Type() in { MT_CORNER, MT_ARC }: continue                       :211
        endType = MT_SINGLE  if m->Type() in { MT_START, MT_SINGLE }          :216
                  else MT_FINISH
        end = copy of *m;  end.SetType( endType );  end.Recalculate()         :213
        maxEndElongation = end.CurrentLength() - end.BaselineLength()         :224

        if maxElongation + maxEndElongation > elongation:                     :226
            if not finished:                                                  :228
                m->SetType( endType );  m->Recalculate()                      :230
                if endType == MT_SINGLE:                                      :233
                    endMinElongation = m->MinTunableLength() - m->BaselineLength()
                    if minElongation + endMinElongation >= elongation:        :239
                        m->MakeEmpty()
                finished = true
            else:
                m->MakeEmpty()                                                :247

        maxElongation += m->CurrentLength() - m->BaselineLength()             :251
        minElongation += m->MinTunableLength() - m->BaselineLength()          :252

    # Pass 2: how much the survivors actually give.
    remainingElongation = elongation;  meanderCount = 0                       :256
    for m in tuned.Meanders():
        if m->Type() not in { MT_CORNER, MT_ARC, MT_EMPTY }:                  :261
            remainingElongation -= m->CurrentLength() - m->BaselineLength()
            meanderCount += 1

    lenReductionLeft = -remainingElongation;  meandersLeft = meanderCount     :268
    if lenReductionLeft < 0 or meandersLeft == 0: return                      :271

    # Pass 3: take an equal share off each survivor.
    for m in tuned.Meanders():                                                :274
        if m->Type() not in { MT_CORNER, MT_ARC, MT_EMPTY }:
            lenReductionHere = lenReductionLeft / meandersLeft                :278
            initialLen = m->CurrentLength()
            minAmpl = m->MinAmplitude()
            amp = findAmplitudeForLength( m, initialLen - lenReductionHere,
                                          minAmpl, m->Amplitude() )           :282
            amp = max( amp, minAmpl )                                         :285
            m->SetTargetBaselineLength( m->BaselineLength() )                 :288
            m->Resize( amp )
            lenReductionLeft -= initialLen - m->CurrentLength()               :291
            meandersLeft -= 1
            if meandersLeft == 0: break
```

The early return at `:271` is the "line is still too short even at full amplitude" case: nothing is shrunk, every meander stays at the amplitude `Fit` gave it, and the status ends up `TOO_SHORT`. The reverse case, `elongation` negative because the line is already longer than the target, is handled before `tuneLineLength` is ever called (section 5.4).

`SetTargetBaselineLength` before `Resize` at `:288` is what keeps a shrunk meander from also shrinking its footprint: `genMeanderShape`'s `top = max( top, ... )` widens the flat top to compensate for the shorter sides. Without it the meanders would slide together and leave a gap at the end of the tuned stretch.

Pass 1 accumulates `maxElongation` and `minElongation` **after** the possible mutation at `:230` or `:247`, so an emptied meander contributes zero to both from that point on. `finished` never resets, so everything after the first overshoot is emptied.

### 4.6 `findAmplitudeForLength` and `findAmplitudeBinarySearch`

Two free functions in the `PNS` namespace, `pns_meander_placer_base.cpp:181` and `:137`, with `LENGTH_TARGET_TOLERANCE = 20` nanometres at `:32`.

```
findAmplitudeForLength( m, targetLength, minAmp, maxAmp ):                    :181
    copy = *m
    copy.SetTargetBaselineLength( m->BaselineLength() )                       :186
    initialGuess = m->Amplitude() - ( m->CurrentLength() - targetLength ) / 2  :188
    if minAmp <= initialGuess <= maxAmp:                                      :190
        copy.Resize( minAmp )                    # <- minAmp, not initialGuess :192
        if |copy.CurrentLength() - targetLength| < LENGTH_TARGET_TOLERANCE:
            return initialGuess                  # <- returns the untested value :195
    return findAmplitudeBinarySearch( copy, targetLength, minAmp, maxAmp )

findAmplitudeBinarySearch( copy, target, minAmp, maxAmp ):                    :137
    if minAmp == maxAmp: return maxAmp           # no length test at all      :139
    copy.Resize( minAmp );  minLen = copy.CurrentLength()                     :142
    copy.Resize( maxAmp );  maxLen = copy.CurrentLength()
    if minLen > target: return 0                                              :148
    if maxLen < target: return 0                                              :151
    if |minLen - target| < TOL or |maxLen - target| < TOL:                    :157
        return the closer of minAmp, maxAmp
    left = findAmplitudeBinarySearch( copy, target, minAmp, (minAmp+maxAmp)/2 )
    if left: return left                                                      :167
    right = findAmplitudeBinarySearch( copy, target, (minAmp+maxAmp)/2, maxAmp )
    if right: return right
    return 0
```

`initialGuess` is `amplitude - deltaLength / 2`, which is the right first order estimate because a meander's elongation is `2 * amplitude` plus a corner correction (section 2.6). But the fast path measures `copy.Resize( minAmp )` and then returns `initialGuess`, so it fires only when the **minimum** amplitude already hits the target and then returns a completely different amplitude. Erratum E8. Almost certainly `copy.Resize( initialGuess )` was meant.

The bisection is a depth first search that returns the first amplitude any branch produces, and its base case `minAmp == maxAmp` returns that amplitude without checking that it achieves anything. Interval halving with integer division reaches that base case in about `log2( maxAmp - minAmp )` levels, so it terminates, but the returned amplitude can miss the target by far more than the 20 nm tolerance. `0` doubles as "not found" and as a legal amplitude, which the caller papers over with `amp = max( amp, minAmpl )` at `:285`. `int minLen = copy.CurrentLength()` narrows a `long long` at `:143` and `:146`.

Both functions are free functions with external linkage in the `PNS` namespace and no declaration in any header, so nothing outside this translation unit can call them.

### 4.7 The status protocol

`TuningLengthResult()` is pure virtual (`pns_meander_placer_base.h:61`), `TuningDelayResult()` defaults to 0 (`:66`), `TuningStatus()` is pure virtual (`:76`). Two deltas are computed from the baseline captured at `Start`: `TuningLengthDelta()` (`:70`) and `TuningDelayDelta()` (`:71`), gated by `HasBaseline()` (`:68`).

`TunedPath()` is pure virtual (`:125`) and returns the item set the tuned line runs over, which the host draws as a highlight (`pcb_tuning_pattern.cpp:2014`).

The three status values are turned into user text in exactly two places, both in the host: `pcb_tuning_pattern.cpp:1328` to `:1330` ("too long", "too short", "tuned", plus an unreachable "unknown") and again at `:1914` to `:1916` when a saved pattern is reloaded. The QA mock has the same mapping as strings, `qa/tools/pns/mock_pcb_tuning_pattern.cpp:140` to `:142` ("too_long", "too_short", "tuned") and back at `:150` to `:155`.

---

## 5. `MEANDER_PLACER`: the single track placer

`pcbnew/router/pns_meander_placer.h:45`.

### 5.1 State

`:117` to `:141`. `m_currentStart` (the snapped point the tuned stretch begins at), `m_currentNode` (the scratch branch of the last move), `m_originLine` (the assembled line being tuned), `m_currentTrace` (the rebuilt line handed to the host), `m_tunedPath` (the item set the length is measured over), `m_finalShape` (pre plus tuned plus post), `m_result` (the `MEANDERED_LINE`), `m_initialSegment`, `m_padToDieLength`, `m_padToDieDelay`, `m_netClass`, `m_lastLength`, `m_lastDelay`, `m_lastStatus`.

### 5.2 `Start`

`pns_meander_placer.cpp:69`.

```
Start( P, startItem ):
    if not startItem or not OfKind( SEGMENT_T | ARC_T ):                      :71
        SetFailureReason( "Please select a track whose length you want to tune." )
        return false
    m_initialSegment = startItem;  m_currentNode = nullptr
    m_currentStart = HELPERS::GetSnappedStartPoint( m_initialSegment, P )     :79
    m_world = Router()->GetWorld()->Branch()                                  :81
    m_originLine = m_world->AssembleLine( m_initialSegment )                  :82
    m_tunedPath = TOPOLOGY( m_world ).AssembleTuningPath(
                      iface, m_initialSegment, &m_startPad_n, &m_endPad_n )   :85
    m_padToDieLength / Delay = sum of the two pads' pad to die                :90 to :100
    m_world->Remove( m_originLine )                                           :102
    m_currentWidth = m_originLine.Width()                                     :104
    m_currentEnd = (0,0)
    m_netClass = startItem->GetSourceItem()->GetEffectiveNetClass()           :108
    m_baselineLength = origPathLength()                                       :110
    m_baselineDelay  = isTimeDomain ? origPathDelay() : 0
    initChainExtras();  calculateTimeDomainTargets()                          :113
    return true
```

`GetSnappedStartPoint` (`pcbnew/router/pns_helpers.cpp:187`) is `Seg::NearestPoint` for a segment and the nearer of the two anchors for an arc.

`AssembleTuningPath` (`pns_topology.cpp:787`) is a longest path walk out of both ends of the assembled line, through intermediate pads and stopping at the terminal pads, with two in place fixups afterwards: `OptimiseTraceInPad` on every line touching a terminal or intermediate pad (`:928`, `:974`) and `OptimiseTraceInVia` on the lines either side of every via (`:1010`). Both mutate the path's chains, which is why `CalculateRoutedPathLength` is asked not to repeat them. It differs from `AssembleTrivialPath` (`:461`) by walking through pads rather than terminating at them and by choosing the longest branch at a junction rather than refusing to guess.

Note that the placer removes only `m_originLine` from its branch (`:102`), not the whole tuned path. Everything else on the path stays and is what `CheckFit` collides against.

`origPathLength()` (`:121`) is `m_padToDieLength + m_signalExtraLength + lineLength( m_tunedPath, ... )`; `origPathDelay()` (`:128`) is the delay counterpart.

### 5.3 `Move` and the chain budget

`pns_meander_placer.cpp:188`. Everything before the call to `doMove` is net chain arithmetic.

```
Move( P, endItem ):
    m_settings.m_signalExtraDelay = m_chainExtrasValid ? m_chainExtrasDelay : 0 :195
    if m_targetSignalLength.Opt() != LENGTH_UNCONSTRAINED:                    :199
        otherLen = chainNarrowingOffset()                                     :201
        budgetMin = max( 0, targetSignalLength.Min() - otherLen )             :203
        budgetOpt = max( 0, targetSignalLength.Opt() - otherLen )
        budgetMax = max( budgetOpt, targetSignalLength.Max() - otherLen )
        if m_targetLength.Opt() == LENGTH_UNCONSTRAINED:                      :207
            m_targetLength = ( budgetMin, budgetOpt, budgetMax )
        else:                                                                 :213
            m_targetLength.SetMin( max( Min, budgetMin ) )
            m_targetLength.SetOpt( min( Opt, budgetOpt ) )
            m_targetLength.SetMax( min( Max, budgetMax ) )
    calculateTimeDomainTargets()                                              :221
    return doMove( P, endItem, m_targetLength.Opt(),
                   m_targetLength.Min(), m_targetLength.Max() )               :223
```

`calculateTimeDomainTargets` (`:135`) converts a delay target into a length target by asking the host for a length per unit delay at the current width, layer and net class (`:169` to `:177`) and writing the result into `m_targetLength`. It is a no op when `m_isTimeDomain` is false, which is the only case this crate will have.

### 5.4 `doMove`: cut, meander, reassemble

`pns_meander_placer.cpp:228`. The one routine the milestone is really about.

```
doMove( P, endItem, targetLength, targetMin, targetMax ):
    if m_currentStart == P: return false                                      :231
    delete m_currentNode;  m_currentNode = m_world->Branch()                  :234

    m_originLine.CLine().Split( m_currentStart, P, pre, tuned, post )         :241

    m_result = MEANDERED_LINE( this, false )                                  :243
    m_result.SetWidth( m_originLine.Width() );  m_result.SetBaselineOffset( 0 )

    for i in tuned.SegmentCount():                                            :247
        if tuned.IsArcSegment( i ):                                           :249
            m_result.AddArc( tuned.Arc( tuned.ArcIndex( i ) ) )               :252
            i = tuned.NextShape( i );  if i < 0: i = tuned.SegmentCount()
            continue
        s = tuned.CSegment( i )
        side = ( m_settings.m_initialSide == 0 ) ? ( s.Side( P ) < 0 )        :266
                                                 : ( m_settings.m_initialSide < 0 )
        m_result.AddCorner( s.A )                                             :270
        m_result.MeanderSegment( s, side )
        m_result.AddCorner( s.B )

    lineLen = origPathLength();  lineDelay = origPathDelay()                  :275
    m_lastLength = lineLen;  m_lastDelay = lineDelay;  m_lastStatus = TUNED   :278

    if lineLen > m_settings.m_targetLength.Max():                             :282
        m_lastStatus = TOO_LONG
    else:
        m_lastLength = lineLen - tuned.Length()                               :288
        ( delay counterpart )                                                 :290
        tuneLineLength( m_result, targetLength - lineLen )                    :298

    ( draw the tuned path highlight )                                         :301

    if m_lastStatus != TOO_LONG:                                              :311
        tuned.Clear()
        for m in m_result.Meanders():                                         :315
            if m->Type() != MT_EMPTY: tuned.Append( m->CLine( 0 ) )           :319
        m_lastLength += tuned.Length()                                        :323
        ( delay counterpart )                                                 :325
        m_lastStatus = TOO_LONG  if m_lastLength > targetMax                  :332
                       TOO_SHORT if m_lastLength < targetMin
                       TUNED     otherwise

    m_finalShape.Clear()                                                      :340
    if m_settings.m_keepEndpoints:                                            :342
        pre.Simplify(); tuned.Simplify(); post.Simplify()
        m_finalShape = pre + tuned + post
    else:
        m_finalShape = pre + tuned + post;  m_finalShape.Simplify()           :357
    return true
```

Six observations.

**The tuned stretch is re-meandered from scratch on every move.** There is no incremental state; `m_result` is a fresh `MEANDERED_LINE` each time (`:243`).

**Corners are added twice.** `doMove` calls `AddCorner( s.A )` at `:270` and `MeanderSegment` immediately calls `AddCorner( aBase.A )` at `pns_meander.cpp:263`; the same at the other end (`:272` against `:407`). The duplicates are harmless because a corner shape is a one point chain and `Append` drops duplicates, but the meander list carries four corner shapes per base segment where two would do.

**The early `TOO_LONG` test at `:282` reads `m_settings.m_targetLength.Max()`, not the `targetMax` argument.** For `MEANDER_PLACER::Move` the two are the same value (`:224`). For `MEANDER_SKEW_PLACER::Move` they are not, and the consequence is erratum E9.

**`m_lastLength` is built by subtraction then addition**: the whole path length minus the straight stretch about to be replaced (`:288`), then plus the meandered stretch (`:323`). That works because `origPathLength()` measures `m_tunedPath`, whose chains still hold the *pre meander* geometry; the placer never writes the meanders back into the path it measures.

**`keepEndpoints` changes only where `Simplify` runs**: per part when true, over the concatenation when false. The host forces it true (`pcb_tuning_pattern.cpp:1297`) with the comment "Required for re-grouping", because the generator needs the vertices at the pre/tuned and tuned/post seams to survive so it can tell which segments belong to the pattern.

**Arc segments in the tuned stretch pass through as `MT_CORNER` shapes** carrying the arc (`:252`), so an existing arc inside the tuned range is preserved and not meandered. The `i = tuned.NextShape( i )` advance at `:253` skips the rest of the arc's segments.

### 5.5 `CheckFit`

```
CheckFit( shape ):                                    pns_meander_placer.cpp:400
    l = LINE( m_originLine, shape->CLine( 0 ) )
    if m_currentNode->CheckColliding( &l ): return false                      :404
    clearance = shape->Width() + m_settings.m_spacing                         :408
    return m_result.CheckSelfIntersections( shape, clearance )
```

The collision test is against the branch with the origin line removed, so the candidate meander is checked against everything on the board except the track being tuned. The self intersection clearance is `width + spacing`, which is not a rule clearance at all but a heuristic; the pair placer uses `4 * width` instead (`pns_dp_meander_placer.cpp:620`). Erratum E10.

### 5.6 Fix, commit, abort

```
FixRoute( P, endItem, forceFinish ):                  :364
    if not m_currentNode: return false                                        :366
    m_currentTrace = LINE( m_originLine, m_finalShape )                       :369
    m_currentNode->Add( m_currentTrace )
    CommitPlacement();  return true

CommitPlacement():   if m_currentNode: Router()->CommitRouting( m_currentNode ) :390
                     m_currentNode = nullptr
AbortPlacement():    m_world->KillChildren();  return true                    :377
HasPlacedAnything(): return m_currentTrace.SegmentCount() > 0                 :384
```

`FixRoute` ignores both `aP` and `aEndItem` and commits whatever the last `doMove` produced. There is no walkaround, no shove, no optimizer and no via anywhere in this placer.

### 5.7 The queries

`CurrentNode` returns `m_world` until the first move (`:60`). `Traces()` rebuilds `m_currentTrace` (`:414`). `TunedPath()` returns `m_tunedPath` (`:420`). `CurrentStart` / `CurrentEnd` (`:425`, `:430`), the second of which is always `(0,0)`. `CurrentNets()` is a one element vector holding `m_originLine.Net()` (`pns_meander_placer.h:86`). `CurrentLayer()` is `m_initialSegment->Layers().Start()` (`pns_meander_placer.cpp:435`). `TuningLengthResult()` returns `m_lastLength` or, when that is zero, `origPathLength()` (`:441`); `TuningDelayResult()` the same for the delay (`:450`); `TuningStatus()` returns `m_lastStatus` (`:459`).

### 5.8 What it does not implement

`PLACEMENT_ALGO`'s `UnfixRoute` (`pns_placement_algo.h:81`), `ToggleVia` (`:94`), `IsPlacingVia` (`:104`), `SetLayer` (`:114`), `FlipPosture` (`:167`), `UpdateSizes` (`:178`), `SetOrthoMode` (`:189`) and `GetModifiedNets` (`:198`) are all left at their defaults by all three meander placers. So during a tuning session backspace does nothing, the via key does nothing, the layer keys do nothing, posture does nothing and the size settings the router pushes in `StartRouting` (`pns_router.cpp:467`) are discarded. The two brief items `SetLayer` and `ToggleVia` therefore have exactly one line of answer each, and it is "not overridden".

---

## 6. `DP_MEANDER_PLACER`: the pair length placer

`pcbnew/router/pns_dp_meander_placer.h:48`. It does **not** derive from `MEANDER_PLACER`; it derives straight from `MEANDER_PLACER_BASE` and duplicates a good deal of it.

### 6.1 State

`:142` to `:165`. `m_currentStart`, `m_currentNode`, `m_originPair` (a `DIFF_PAIR`), `m_coupledSegments` (a `COUPLED_SEGMENTS_VEC`), `m_currentTraceN` / `m_currentTraceP`, `m_tunedPath` / `m_tunedPathP` / `m_tunedPathN`, `m_finalShapeP` / `m_finalShapeN`, `m_result`, `m_initialSegment`, `m_lastLength`, `m_lastDelay`, the four pad to die values, `m_lastStatus`, `m_netClass`.

`m_coupledSegments` (`:148`) and `m_tunedPath` (`:151`) are never written or read: `Move` uses a local `coupledSegments` (`pns_dp_meander_placer.cpp:252`) and the length comes from `m_tunedPathP` and `m_tunedPathN`. `totalLength()` (`:109`), `meanderSegment()` (`:124`) and `setWorld()` (`:132`) are declared and never defined; `release()` (`:133`) is defined empty (`pns_dp_meander_placer.cpp:175`) and never called; `Trace()` (`:85`) is defined (`:68`) and never called. Erratum E12.

### 6.2 `Start`

`pns_dp_meander_placer.cpp:89`.

```
Start( P, startItem ):
    if not startItem or not OfKind( SEGMENT_T | ARC_T ): fail, same message   :91
    m_currentStart = HELPERS::GetSnappedStartPoint( startItem, P )            :99
    m_world = Router()->GetWorld()->Branch()                                  :101
    if not TOPOLOGY( m_world ).AssembleDiffPair( startItem, m_originPair ):   :105
        SetFailureReason( "Unable to find complementary differential pair
                           net for length tuning..." )
        return false
    if m_originPair.Gap() < 0: m_originPair.SetGap( Sizes().DiffPairGap() )   :114
    if either lane has no segments: return false                              :117
    m_tunedPathP = AssembleTuningPath( iface, m_originPair.PLine().GetLink(0),
                                       &m_startPad_p, &m_endPad_p )           :120
    ( pad to die for P )                                                      :126 to :136
    m_tunedPathN = AssembleTuningPath( iface, m_originPair.NLine().GetLink(0),
                                       &m_startPad_n, &m_endPad_n )           :138
    ( pad to die for N )                                                      :144 to :154
    m_world->Remove( m_originPair.PLine() )                                   :156
    m_world->Remove( m_originPair.NLine() )
    m_currentWidth = m_originPair.Width()                                     :159
    m_netClass = startItem->GetSourceItem()->GetEffectiveNetClass()
    m_baselineLength = origPathLength();  initChainExtras()                   :164
    calculateTimeDomainTargets();  return true
```

`TOPOLOGY::AssembleDiffPair` (`pns_topology.cpp:1036`) is the routine note 07 section 12.2 deferred: it takes the clicked item's net, asks the rule resolver for the coupled net (`:1039`), collects the same layer segments of the clicked line and every same layer segment of the coupled net (`:1050`, `:1059`), and picks the coupled item that is parallel within `DP_PARALLELITY_THRESHOLD`, has the same width, overlaps in the common parallel projection, and is nearest to the clicked item, tie broken by distance to the clicked shape's centre (`:1088` to `:1126`). If nothing matches it retries from the items joined to the clicked one (`:1132` to `:1153`). The gap is recovered from the cross product of the reference direction and the displacement, minus the width (`:1170`). It is the only way this placer or the skew placer can start, so the port needs it.

Both lanes are removed from the branch (`:156`, `:157`), so a meander on one lane can collide with nothing of the other.

`origPathLength()` (`:180`) is `max( totalP, totalN )`, the **longer** lane, and `origPathDelay()` (`:188`) the same. The pair's length for tuning purposes is therefore the longer half, which is why tuning a pair shortens nothing and only ever adds to whichever lane is being meandered.

### 6.3 The baseline and the offset

```
baselineSegment( coupled ):                           :196
    return SEG( ( coupled.coupledP.A + coupled.coupledN.A ) / 2,
                ( coupled.coupledP.B + coupled.coupledN.B ) / 2 )

pairOrientation( coupled ):                           :205
    midp = ( coupled.coupledP.A + coupled.coupledN.A ) / 2
    return coupled.coupledP.Side( midp ) > 0
```

The baseline is the centreline of the coupled span, integer, with the `/ 2` truncating toward zero. `pairOrientation` answers which of the two lanes is on which side of that centreline, and the offset is negated when P is on the positive side:

```
offset = ( tuned.Gap() + tuned.Width() ) / 2                                  :313
if pairOrientation( coupledSegments[0] ): offset *= -1                        :315
m_result.SetBaselineOffset( offset )                                          :318
```

`offset` is half the pitch. It reaches `MEANDER_SHAPE::m_baselineOffset` through `MeanderSegment` (`pns_meander.cpp:274`) and from there into `genMeanderShape` as `+offset` for chain 0 and `-offset` for chain 1 (`pns_meander.cpp:795`, `:796`). Only the **first** coupled span's orientation is consulted, so a pair that swaps sides part way along the tuned stretch gets the wrong sign for the rest of it.

That single number is the whole of "how it keeps the coupling". Everything else follows from it:

- `spacing()` grows by `2 * |offset|` for a dual meander (`pns_meander.cpp:464`), so the two lanes do not touch on the straights.
- `cornerRadius()`'s floor grows by `|offset|` (`:437`, `:439`), so the inner lane's corner never inverts.
- `MinAmplitude()` grows by `|offset|` (`:417`, `:422`), so the excursion is at least as tall as the pair is wide.
- `sCorner = cr - offset` and `uCorner = cr + offset` (`:612`, `:613`) give the inner and outer lanes different corner radii, differing by exactly the pitch, which is what keeps the gap constant around a corner.
- `startSide` gains `|offset|` and the start and finish shapes gain a `forward( |offset| )` (`:635`, `:645`, `:662`, `:664`), which is the lead in and lead out that brings the two lanes back onto the centreline.
- `makeMiterShape`'s chamfer correction pulls the outer lane's chamfer back by `2 * |offset| * tan(22.5 deg)` (`:507`), which is the amount a 45 degree chamfer on the outer radius overshoots.
- `updateBaseSegment` projects the **midpoint** of the two chains' ends rather than one chain's ends (`:971`, `:972`), so the baseline footprint is the pair's, not a lane's.

Nothing enforces the gap after the fact. There is no `checkGap` here as there is in the pair router (note 07 section 2.4); the coupling is a property of the construction.

### 6.4 `Move`

`pns_dp_meander_placer.cpp:215`. The net chain block at `:217` to `:247` is identical to the single placer's. Then:

```
    if m_currentStart == P: return false                                      :249
    delete m_currentNode;  m_currentNode = m_world->Branch()                  :254
    m_originPair.CP().Split( m_currentStart, P, preP, tunedP, postP )         :262
    m_originPair.CN().Split( m_currentStart, P, preN, tunedN, postN )         :263
    tunedP.Simplify();  tunedN.Simplify()                                     :265

    if tunedP.PointCount() == 0 or tunedN.PointCount() == 0:                  :270
        m_finalShape* = the original chains;  m_lastLength = origPathLength()
        m_lastStatus = TOO_SHORT;  return false                               :277

    tuned = DIFF_PAIR( m_originPair );  tuned.SetShape( tunedP, tunedN )      :291
    tuned.CoupledSegmentPairs( coupledSegments )                              :295
    if coupledSegments.empty():                                               :297
        m_finalShape* = the original chains;  m_lastLength = origPathLength()
        updateStatus();  return false                                         :305

    m_result = MEANDERED_LINE( this, true );  m_result.SetWidth( tuned.Width() )
    offset as in section 6.3                                                  :313
    ( draw both tuned path highlights )                                       :320 to :338

    for sp in coupledSegments:                                                :451
        base = baselineSegment( sp )
        side = ( m_initialSide == 0 ) ? ( base.Side( P ) < 0 )
                                      : ( m_initialSide < 0 )                 :456
        addCornersUntilIndex( sp.indexP, sp.indexN )                          :463
        m_result.MeanderSegment( base, side )                                 :465
    addCornersUntilIndex( tunedP.PointCount()-1, tunedN.PointCount()-1 )      :468
    m_result.AddCorner( tunedP.CLastPoint(), tunedN.CLastPoint() )            :470

    dpLen = origPathLength();  dpDelay = origPathDelay()                      :472
    m_lastStatus = TUNED
    if dpLen > m_targetLength.Max():                                          :477
        m_lastStatus = TOO_LONG;  m_lastLength = dpLen;  m_lastDelay = dpDelay
    else:
        m_lastLength = dpLen - max( tunedP.Length(), tunedN.Length() )        :485
        ( delay counterpart )                                                 :487
        tuneLineLength( m_result, m_targetLength.Opt() - dpLen )              :499

    if m_lastStatus != TOO_LONG:                                              :502
        tunedP.Clear();  tunedN.Clear()
        for m in m_result.Meanders():                                         :507
            if m->Type() != MT_EMPTY:
                tunedP.Append( m->CLine(0) );  tunedN.Append( m->CLine(1) )   :511
        m_lastLength += max( tunedP.Length(), tunedN.Length() )               :516
        ( delay counterpart );  updateStatus()                                :530
    ( reassemble both final shapes exactly as the single placer does )        :533 to :565
    return true
```

`addCornersUntilIndex` (`:378`) is a lockstep walk over the two tuned chains that emits one `AddCorner( p, n )` per pair of aligned vertices, or one `AddArc( arcP, arcN )` when both sides are on an arc, and hunts forward on one side when only one side is on an arc (`:400` to `:441`). Its purpose is to carry the uncoupled stretches between coupled spans through unchanged, with the two lanes' vertices paired up. With no arcs it reduces to "emit `AddCorner( tunedP.CPoint( i ), tunedN.CPoint( j ) )` for each index up to the coupled span's start, advancing both cursors".

`DIFF_PAIR::CoupledSegmentPairs` is note 07 section 2.2; each `COUPLED_SEGMENTS` carries `coupledP`, `coupledN` and the two segment indices `indexP`, `indexN` that the walk uses as its stop points.

`updateStatus` (`:280`) is the same three way comparison as the single placer's, against `m_settings.m_targetLength` rather than against arguments, because this placer has no `doMove` and no target arguments.

### 6.5 `CheckFit`, `FixRoute` and the queries

```
CheckFit( shape ):                                    :608
    l1 = LINE( m_originPair.PLine(), shape->CLine(0) )
    l2 = LINE( m_originPair.NLine(), shape->CLine(1) )
    if m_currentNode->CheckColliding( &l1 ): return false                     :613
    if m_currentNode->CheckColliding( &l2 ): return false                     :616
    clearance = shape->Width() + shape->Width() * 3                           :620
    return m_result.CheckSelfIntersections( shape, clearance )

FixRoute( P, endItem, forceFinish ):                  :571
    m_currentNode->Add( LINE( m_originPair.PLine(), m_finalShapeP ) )         :576
    m_currentNode->Add( LINE( m_originPair.NLine(), m_finalShapeN ) )
    CommitPlacement();  return true
```

`FixRoute` does not check `m_currentNode` for null where the single placer does (`pns_meander_placer.cpp:366`). `Move` returns before branching whenever `m_currentStart == P` (`:249`), so a host that fixes without a successful move dereferences null. Erratum E13.

`CheckSelfIntersections` only ever tests chain 0 (`pns_meander.cpp:714`), so the N lane of a dual meander is never checked against earlier meanders. Erratum E7.

`Traces()` returns P then N (`:626`). `TunedPath()` returns N's items then P's, in that order (`:640`). `CurrentNets()` returns P then N (`:696`). `HasPlacedAnything()` tests the **origin** pair's segment counts (`:592`), so it is true from the first successful `Start` onwards, unlike the single placer's, which tests the trace it built.

---

## 7. `MEANDER_SKEW_PLACER`: the skew placer

`pcbnew/router/pns_meander_skew_placer.h:39`, deriving from `MEANDER_PLACER`. It reuses the single placer's `doMove` entirely and changes only what the target is measured against.

### 7.1 It does not pick a lane

The brief asks how it picks the shorter lane. It does not pick a lane at all. `Start` assembles the line of the item the user clicked (`pns_meander_skew_placer.cpp:72`), removes **that** line from the branch (`:128`), and every subsequent step meanders it. The other lane is only measured.

```
Start( P, startItem ):                                :59
    if not startItem or not OfKind( SEGMENT_T | ARC_T ):
        SetFailureReason( "Please select a differential pair track you want to tune." )
    m_currentStart = GetSnappedStartPoint( startItem, P )                     :69
    m_world = Router()->GetWorld()->Branch()
    m_originLine = m_world->AssembleLine( m_initialSegment )                  :72
    m_tunedPath = TOPOLOGY( m_world ).AssembleTrivialPath( startItem, nullptr, true ) :75
    if not AssembleDiffPair( startItem, m_originPair ): fail with the skew message :77
    if m_originPair.Gap() < 0: SetGap( Sizes().DiffPairGap() )                :85
    if either lane has no segments: return false                              :88
    m_tunedPathP = AssembleTuningPath( ..., PLine().GetLink(0), &m_startPad_p, &m_endPad_p ) :92
    ( pad to die for P )                                                      :98 to :108
    m_tunedPathN = AssembleTuningPath( ..., NLine().GetLink(0), &m_startPad_n, &m_endPad_n ) :110
    ( pad to die for N )                                                      :116 to :126
    m_world->Remove( m_originLine )                                           :128
    m_currentWidth = m_originLine.Width();  m_currentEnd = (0,0)
    m_netClass = ...;  m_settings.m_netClass = m_netClass                     :134
    pIsActive = ( m_originPair.NetP() == m_originLine.Net() )                 :137
    lenP, lenN, delayP, delayN from the two tuned paths plus pad to die       :138 to :141
    GetSignalAggregate( NetP, NetN, extraSignalLen, extraSignalDelay )        :146
    if pIsActive:  m_coupledLength = lenN + extra;  m_lastLength = lenP + extra
                   m_tunedPath = m_tunedPathP                                 :155
    else:          m_coupledLength = lenP + extra;  m_lastLength = lenN + extra
                   m_tunedPath = m_tunedPathN                                 :163
    m_baselineLength = origPathLength();  initChainExtras()                   :166
    calculateTimeDomainTargets();  return true
```

`m_tunedPath` is written twice: once from `AssembleTrivialPath` at `:75`, then overwritten from the matching `AssembleTuningPath` result at `:155` or `:163`. The first assignment is dead. Erratum E14.

`origPathLength()` (`:177`) overrides the base and answers the **active** lane's length only, choosing the pad pair by comparing `m_originPair.NetP()` against `m_originLine.Net()`. `CurrentSkew()` (`:195`) is `m_lastLength - m_coupledLength`. `TuningLengthResult()` (`:240`) overrides the base to return the skew rather than a length, and `TuningDelayResult()` (`:246`) the delay skew, which is why the host labels the readout "current skew" in this mode (`pcb_tuning_pattern.cpp:2147`).

`CurrentNets()` (`pns_meander_skew_placer.h:60`) returns the active net first and the coupled net second, with a comment at `:62` explaining that the order matters because `chainNarrowingOffset` looks up `nets[0]`'s board length and `initChainExtras` must exclude both.

### 7.2 `Move`

```
Move( P, endItem ):                                   :201
    calculateTimeDomainTargets()                                              :203
    ( draw both lanes' highlights, active one at importance 1 )               :207 to :225
    offset = chainNarrowingOffset()                                           :232
    return doMove( P, endItem,
                   m_coupledLength + m_targetSkew.Opt() - offset,             :234
                   m_coupledLength + m_targetSkew.Min() - offset,
                   m_coupledLength + m_targetSkew.Max() - offset )
```

So the target handed to the single placer's `doMove` is "the other lane's length, plus the requested skew". If the user clicked the **longer** lane and asks for zero skew, the target is below the current length, `tuneLineLength` is called with a negative elongation, pass 1 empties every meander on the first iteration (`maxEndElongation > elongation` is true for any meander when `elongation < 0`), pass 2 counts zero survivors, pass 3 returns early, and the final comparison at `pns_meander_placer.cpp:332` reports `TOO_LONG`. The user's remedy is to click the other lane. There is no automatic lane choice anywhere in the class.

`calculateTimeDomainTargets` (`:252`) converts a delay skew target into a length skew target through the host's `CalculateLengthForDelay`, writing into `m_settings.m_targetSkew`; it is a no op outside the time domain.

### 7.3 The wrong bound in the inherited `doMove`

`doMove`'s early bail out compares `lineLen > m_settings.m_targetLength.Max()` (`pns_meander_placer.cpp:282`), not `lineLen > aTargetMax`. In skew mode `m_targetLength` is whatever the default constructor left, which is `LENGTH_UNCONSTRAINED` for both opt and max (`pns_meander.cpp:49`, `:74`), one kilometre. So the early test never fires in skew mode, every skew move runs `tuneLineLength`, and the status always comes from the argument based comparison at `:332`. The two are inconsistent but the inconsistency is benign at this commit. Erratum E9.

---

## 8. The floating point inventory

`DESIGN.md` section 2 fixes coordinates at `i32` nanometres with `i64` products, `CLAUDE.md` says "do not use f64 where KiCad uses integers", and note 01 line 33 deferred `VECTOR2D` to this milestone. This section is the promised accounting.

### 8.1 Where the floating point actually is

**All of it is in `pcbnew/router/pns_meander.cpp` and the five declarations in `pns_meander.h`.** A grep for `double`, `VECTOR2D`, `tan(` or a `(int)` cast over `pns_meander_placer_base.cpp`, `pns_meander_placer.cpp`, `pns_dp_meander_placer.cpp` and `pns_meander_skew_placer.cpp` returns nothing at all. The three placers, the length bookkeeping, `tuneLineLength` and the amplitude search are entirely integer.

The header declarations: `start`, `makeMiterShape` and `genMeanderShape` take `const VECTOR2D&` (`pns_meander.h:383`, `:398`, `:401`), and the turtle's two scratch members are `VECTOR2D` (`:456`, `:459`).

### 8.2 Every use, with a verdict

The verdicts are: **exact** means an integer transcription produces bit identical results; **exact after rescaling** means the value is fractional only through a `Resize`, `EuclideanNorm` or `rescale` that the crate already implements in integers with KiCad's own rounding (`src/geometry/vec2.rs:167`, `:117`, `src/geometry/math.rs:63`); **needs a real f64** means the expression is transcendental or accumulates fractional state across steps.

| Line | Expression | Verdict |
| --- | --- | --- |
| `:254` | `double base_len = aBase.Length()` | **Exact.** `SEG::Length()` returns `int` (`libs/kimath/include/geometry/seg.h:339`). Transcribe as `i64`. |
| `:260` | `VECTOR2D dir( aBase.B - aBase.A )` | **Exact.** Built from a `VECTOR2I` difference and only ever used at `:395`. Transcribe as `Vec2`. |
| `:277` | `double thr = (double) m.spacing()` | **Exact.** `spacing()` returns `int`. |
| `:280`, `:383` | `base_len - ( m_last - aBase.A ).EuclideanNorm()` | **Exact.** `VECTOR2<int>::EuclideanNorm()` returns `int` (`vector2d.h:279`, with the 45 degree and axis special cases the crate already reproduces at `src/geometry/vec2.rs:117`). Both operands integral, difference integral. |
| `:320`, `:374` | `remaining > 3.0 * thr`, `remaining > thr * 2.0` | **Exact.** Both sides integral and well under 2^53. Transcribe as `remaining > 3 * thr` in `i64`. |
| `:395` | `VECTOR2I pn = m_last + dir.Resize( nextP )` | **Exact after rescaling, with one caveat.** `operator+` on mixed `VECTOR2<int>` and `VECTOR2<double>` yields `VECTOR2<double>` (`vector2d.h:437`), which is then **truncated toward zero** by the converting constructor (`:97`). KiCad's integer `Resize` rounds (`:405`) where the double one does not, so `Vec2::resize` plus an integer add differs from KiCad by at most 1 nm per component, and only for a base segment that is neither axis aligned nor 45 degrees. The value feeds only `SEG::Contains( pn )` (`:397`), whose tolerance is `SquaredDistance <= 3` (`libs/kimath/src/geometry/seg.cpp:625`), and `AddCorner( pn )`, a skip advance. Use `Vec2::resize`. |
| `:421` | `m_width * tan( 1 - tan( DEG2RAD( 22.5 ) ) )` | **Needs an f64, or a constant.** `tan(1 - tan(22.5 deg)) = 0.6634702554`. It is a compile time constant times an `i32`; the crate's own idiom (`src/item.rs:76`, `OCTAGON_CHAMFER_HALF_FACTOR`) is a `const f64` multiplied and cast. Truncating, not rounding: KiCad assigns to `int`. Erratum E3 says the constant is a typo. |
| `:439` | `m_width / 2 * ( 1 - tan( DEG2RAD( 22.5 ) ) )` | **Needs an f64, or a constant.** `1 - tan(22.5 deg) = 0.5857864376`. Note the integer division happens **first**: `(m_width / 2)` then times the constant, then truncated to `int`. |
| `:475` | `aDir.EuclideanNorm() == 0.0f` | **Exact.** Replace with `dir == Vec2::ZERO`; the norm of a non zero integer vector is never zero. |
| `:481`, `:482` | `dir_u( aDir )`, `dir_v( aDir.Perpendicular() )` | **Exact.** `aDir` at every call site is `m_currentDir.Resize( radius )` (`:565`), so both are integer valued if `Resize` is. `Perpendicular` is `(-y, x)`. |
| `:484` | `endPoint = aP + dir_u + dir_v * ( aSide ? -1.0 : 1.0 )` | **Exact.** A signed sum of the three. |
| `:486`, `:513`, `:515`, `:518` | `lc.Append( (int) p.x, (int) p.y )` | **Truncation toward zero, not rounding.** Reproduce literally. For a meander on the negative side of the origin this is a systematic 1 nm bias; see erratum E15. |
| `:493` | `VECTOR2I arcEnd( (int) endPoint.x, (int) endPoint.y )` | Round corner only, not ported. |
| `:503` | `double radius = (double) aDir.EuclideanNorm()` | **Exact.** Integral; used only in the comparison at `:506`. |
| `:507` | `(-2 * abs( m_baselineOffset )) * tan( DEG2RAD( 22.5 ) )` | **Needs an f64, or a constant.** `tan(22.5 deg) = 0.4142135624`, which is `sqrt(2) - 1`. Dual meanders only. |
| `:509`, `:510` | `dir_u.Resize( correction )`, `dir_v.Resize( correction )` | **Needs an f64 argument, integer result.** `correction` is fractional and negative. `VECTOR2<double>::Resize` multiplies by `sign( aNewLength )` (`vector2d.h:407`), so a negative length reverses the vector. The crate's `Vec2::resize` takes an `i32`; either round `correction` before calling it, which changes the result by under a nanometre, or add an `f64` overload. Dual meanders only. |
| `:514` | `aP + dir_u + (dir_v + dir_cv) * ( aSide ? -1.0 : 1.0 )` | **Exact** once `dir_cv` is an integer vector. |
| `:546` | `m_currentPos += m_currentDir.Resize( aLength )` | **Exact after rescaling, and this is the only accumulating state.** `m_currentDir` is an integer vector (section 2.3), `aLength` is an `int`, so `Resize` is the same rescale the crate already has. The `+=` accumulates in `double`, and the result is truncated when it reaches `Append`. Transcribing `m_currentPos` as a `Vec2` and rounding at each step differs from KiCad by at most 1 nm per step in the general case; for an axis aligned or 45 degree base segment it is bit identical, because `Resize` returns exact integers there (`vector2d.h:389` for the diagonal, and the `rescale` path yields an exact square root when one component is zero). |
| `:565` | `VECTOR2D dir = m_currentDir.Resize( (double) aRadius )` | **Exact after rescaling.** Same argument. |
| `:598`, `:599` | `dir_u_b( aDir.Resize( offset ) )`, `dir_v_b( dir_u_b.Perpendicular() )` | **Exact after rescaling.** For a single track `offset == 0` and `Resize( 0 )` returns the zero vector (`vector2d.h:398`, `newLength_sq == 0`), so both are zero and the whole offset algebra vanishes. |
| `:650`, `:652`, `:674` | `lc.Append( aP + dir_v_b + aDir.Resize( n ) )` | **Exact after rescaling**, then truncated by the `VECTOR2D` to `VECTOR2I` conversion in `Append( const VECTOR2I& )`. |
| `pns_meander.h:456`, `:459` | `VECTOR2D m_currentDir`, `m_currentPos` | The two members that make the turtle floating point. |

### 8.3 The recommended integer turtle

`m_currentDir` never needs to be anything but a `Vec2`: it starts as an integer vector and only ever takes exact 90 degree rotations (section 2.3). Model `turn` as a component swap with signs and delete `RotatePoint` and `EDA_ANGLE` from the milestone entirely; note 06 section 9.6 already established that rotation and angle types in this router are otherwise arc only.

`m_currentPos` is the only genuinely accumulating quantity. Two options:

1. **Keep it a `Vec2` and round at each `forward`.** `pos += dir.resize( length )`, using the crate's existing `Vec2::resize` with KiCad's `KiROUND`. Exact for axis aligned and 45 degree base segments, which is every trace LibrePCB or KiCad's 45 degree router can produce, and within 1 nm per turtle step otherwise. This is what `DESIGN.md` section 2 asks for and what this note recommends.
2. **Carry an `f64` pair and truncate on append.** Bit identical to KiCad on all inputs, at the cost of a floating point coordinate inside the crate for the first time.

Choose 1, record the deviation in `doc/log/`, and pin it with a unit test on a non axis aligned base segment so a future change to `resize` is visible.

Three constants have to be `f64` literals whatever else happens, because they are transcendental: `tan(22.5 deg) = sqrt(2) - 1`, `1 - tan(22.5 deg) = 2 - sqrt(2)` and, if erratum E3 is reproduced rather than fixed, `tan(1 - tan(22.5 deg))`. The first two have exact closed forms in terms of `sqrt(2)`, so `SQRT_2` from `std::f64::consts` covers them; the third does not, and the port should write the intended `2 - sqrt(2)` and note the deviation, or write the KiCad value `0.663_470_255_4` as a named constant with a comment pointing at E3.

### 8.4 What that leaves

The port needs no `Vec2D` type, no `Seg` in doubles and no floating point vector algebra. It needs:

- `Vec2::resize` and `Vec2::euclidean_norm`, both already present (`src/geometry/vec2.rs:167`, `:117`).
- `Vec2::perpendicular`, present (`:147`).
- Two or three `const f64` chamfer factors used the way `OCTAGON_CHAMFER_HALF_FACTOR` already is (`src/item.rs:76`).
- A decision about truncation versus rounding at the four `(int)` casts, recorded per erratum E15.

`VECTOR2D` can stay unported. `TODO.md:18` should be updated in the same commit to say so.

---

## 9. Arcs

### 9.1 Where `SHAPE_ARC` enters

Nine places in the tuning code, and only two of them are on the chamfered path.

| Site | Line | On the chamfered path? |
| --- | --- | --- |
| `MEANDER_SHAPE::MakeArc` | `pns_meander.cpp:916` | **Yes**, when the tuned stretch already contains an arc. |
| `MEANDERED_LINE::AddArc` | `:877` | **Yes**, same reason. |
| `MEANDERED_LINE::AddArcAndPt`, `AddPtAndArc` | `:888`, `:896` | No caller at all (erratum E5). |
| `makeMiterShape`'s `MEANDER_STYLE_ROUND` branch | `:491` to `:499` | No. This is the round corner style itself. |
| `MEANDER_PLACER::doMove`'s arc passthrough | `pns_meander_placer.cpp:249` to `:259` | **Yes.** |
| `DP_MEANDER_PLACER::Move`'s `getItem` and `addCornersUntilIndex` | `pns_dp_meander_placer.cpp:353`, `:396` to `:441` | **Yes.** |
| `tuneLineLength`'s `MT_ARC` skips | `pns_meander_placer_base.cpp:211`, `:261`, `:276` | **Yes**, as dead branches. |
| `CheckSelfIntersections`'s type skip | `pns_meander.cpp:701` | **Yes**, as a dead branch. |

### 9.2 What the chamfered style needs

Nothing arc shaped. `makeMiterShape`'s `MEANDER_STYLE_CHAMFER` branch (`:501` to `:519`) produces at most four points and no arc. The five shape bodies of `genMeanderShape` do not mention arcs. `Fit`, `Resize`, `Recalculate`, `MakeEmpty`, `updateBaseSegment`, `BaselineLength`, `CurrentLength`, `MinTunableLength`, `MeanderSegment`, `CheckSelfIntersections`, `tuneLineLength`, `findAmplitudeForLength` and all three placers' length arithmetic are arc free.

The one thing the chamfered style needs and the crate does not have is `SHAPE_LINE_CHAIN::Length()`'s arc term (`libs/kimath/src/geometry/shape_line_chain.cpp:967`), and it needs it only to be **absent**: with no arcs in the chain the loop at `:960` is the whole answer, which is exactly `LineChain::length` (`src/geometry/line_chain.rs:976`).

### 9.3 What is skipped, and what it costs

Three behaviours are lost while arcs are on hold, and all three are about **pre existing** arcs on the track being tuned, not about the meanders themselves:

1. **A tuned stretch containing an arc.** KiCad carries the arc through as an `MT_CORNER` shape holding the arc (`pns_meander_placer.cpp:252`) and meanders only the straight segments around it. Without arcs the crate's `LineChain` cannot hold one, so the case cannot arise: a line assembled from the crate's world has only straight segments.
2. **A pair whose two lanes have arcs at different indices.** `addCornersUntilIndex`'s four way branch (`pns_dp_meander_placer.cpp:392` to `:441`) exists only for that. Without arcs it collapses to the `!p_item.arc && !n_item.arc` case at `:392`, a plain lockstep `AddCorner( p, n )`.
3. **`MT_ARC` as a meander type.** It is never produced: `MakeArc` sets `MT_CORNER` (`pns_meander.cpp:918`), erratum E4. The three `MT_ARC` tests in `tuneLineLength` and the one in `CheckSelfIntersections` are dead in KiCad too.

So the cost of deferring arcs in this milestone is zero behaviour that the crate could otherwise exhibit. That is a stronger statement than note 07 could make about the pair router, where the arc branches at least had reachable inputs.

### 9.4 Can `MEANDER_STYLE_ROUND` be refused cleanly?

Yes, at the settings boundary, and it should be.

`m_cornerStyle` is read in exactly three places: `MinAmplitude()` (`pns_meander.cpp:415`), `cornerRadius()` (`:436`) and `makeMiterShape()` (`:489`). The first two only choose between two floors; the third is the only one that needs a `SHAPE_ARC`. So a port could in principle accept `Round` and silently draw chamfers, and the geometry would still be valid, merely not what was asked for. Do not do that: the corner style changes the meander's length by `radius * (pi/2 - sqrt(2))` per corner, about 0.155 of the radius, so silently substituting chamfers would make a "tuned" trace miss its target by roughly `0.62 * radius` per meander.

The clean refusal is a settings type that cannot express the round style yet:

```rust
/// Port of `MEANDER_STYLE` (`pcbnew/router/pns_meander.h:53`).
///
/// KiCad's `MEANDER_STYLE_ROUND` (`:54`) is a 90 degree `SHAPE_ARC`
/// built in `makeMiterShape` (`pns_meander.cpp:496`). Arcs are on hold
/// (`PLAN.md`), so this enum has one variant; adding `Round` is the
/// change that lands with them.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
#[non_exhaustive]
pub enum CornerStyle {
  /// A 45 degree chord across the corner. `MEANDER_STYLE_CHAMFER`.
  #[default]
  Chamfer,
}
```

`#[non_exhaustive]` keeps a later `Round` from being a breaking change for a host that matches on it. A host that has a stored "rounded" flag, which KiCad's board file does (`pcb_tuning_pattern.cpp:1802`, `:1857`), maps it to `Chamfer` and says so; there is nothing for it to fail on, because the value is a preference and not a constraint.

The one thing that must not be silently defaulted is `m_cornerRadiusPercentage`. It is meaningful for both styles (it sets `optCr` at `pns_meander.cpp:450`), so it stays, and it keeps KiCad's default of 80.

---

## 10. The host side

### 10.1 The three modes and the placer factory

`ROUTER_MODE` gains three members after the two routing ones: `PNS_MODE_TUNE_SINGLE`, `PNS_MODE_TUNE_DIFF_PAIR`, `PNS_MODE_TUNE_DIFF_PAIR_SKEW` (`pcbnew/router/pns_router.h:70` to `:72`). `ROUTER::StartRouting` switches on the mode and news the matching placer (`pcbnew/router/pns_router.cpp:451` to `:461`), then does the same four calls it does for a routing placer: `UpdateSizes`, `SetLayer`, `SetDebugDecorator`, `SetLogger` (`:467` to `:470`). The first two are no ops on a meander placer (section 5.8), so the sizes the host just imported are dropped on the floor.

`isStartingPointRoutable` (`:294`) branches on `m_mode` with `if( m_mode == PNS_MODE_ROUTE_SINGLE )` (`:307`) and `else if( m_mode == PNS_MODE_ROUTE_DIFF_PAIR )` (`:341`) and has no arm for the three tuning modes, so **no start gate runs for tuning**. The only refusal is the placer's own `Start`, which checks that the clicked item is a segment or an arc (`pns_meander_placer.cpp:71`, `pns_dp_meander_placer.cpp:91`, `pns_meander_skew_placer.cpp:61`) and, for the two pair placers, that `AssembleDiffPair` succeeds.

`ROUTER::Move` goes through `movePlacing` for `ROUTE_TRACK` (`:502`) exactly as it does for routing, so the state machine is unchanged.

The three `TOOL_ACTION`s are `PCB_ACTIONS::tuneSingleTrack` (`pcbnew/tools/pcb_actions.cpp:3013`, hotkey `7`), `tuneDiffPair` (`:3025`, hotkey `8`) and `tuneSkew` (`:3037`, hotkey `9`), each carrying its `PNS::ROUTER_MODE` as the action parameter (`:3023`, `:3035`, `:3047`). They activate `DRAWING_TOOL::PlaceTuningPattern` (`pcbnew/generators/pcb_tuning_pattern.cpp:2497`), **not** the router tool.

### 10.2 `PCB_TUNING_PATTERN`, the board item that drives the placer

`pcbnew/generators/pcb_tuning_pattern.h:106`, a `PCB_GENERATOR`. It is a group of tracks plus the settings that produced them, and it survives in the board file, so a saved tuning pattern can be re edited later.

Its state (`:600` to `:630`): `m_end` (the far end of the tuned stretch; `m_origin` comes from `PCB_GENERATOR`), `m_settings` (a whole `PNS::MEANDER_SETTINGS`), `m_baseLine` and `m_baseLineCoupled` (the un meandered chains, `std::optional<SHAPE_LINE_CHAIN>`), `m_trackWidth`, `m_assembledLineWidth`, `m_diffPairGap`, `m_tuningMode`, `m_lastNetName`, `m_tuningInfo`, `m_tuningLength`, `m_tuningStatus`, `m_updateSideFromEnd`, and a bridging length cache.

`LENGTH_TUNING_MODE` is a separate three value enum (`:38`) that maps onto `PNS::ROUTER_MODE` through `GetPNSMode()` (`:437`) and back through `fromPNSMode` (`pcb_tuning_pattern.cpp:342`).

`CreateNew` (`:482`) seeds `m_settings` from one of three board wide defaults, `bds.m_SingleTrackMeanderSettings`, `m_DiffPairMeanderSettings` or `m_SkewMeanderSettings` (`:496` to `:498`), then folds in the DRC length or chain length constraint.

`EditStart` (`:627`) is where the settings are refreshed from the design rules on every edit:

```
EditStart( tool, board, commit ):
    commit->Add or Modify;  SetFlags( IN_EDIT )                               :629
    router->SyncWorld()                                                       :643
    if not baselineValid(): initBaseLines( router, layer, board )             :648
    if m_updateSideFromEnd:                                                   :651
        pick MEANDER_SIDE_LEFT or RIGHT from which side of the baseline's
        last segment m_end lies on                                            :670
    m_origin = HELPERS::SnapToNearestTrack( m_origin, board, nullptr, &track ) :682
    m_settings.m_netClass = track->GetEffectiveNetClass()                     :685
    if not m_settings.m_overrideCustomRules:                                  :699
        SINGLE:         QueryConstraint( CT_LENGTH, item, nullptr, layer )    :709
                        -> SetTargetLengthDelay or SetTargetLength
        DIFF_PAIR:      QueryConstraint( CT_LENGTH, item, coupledItem, layer ) :746
        DIFF_PAIR_SKEW: QueryConstraint( CT_DIFF_PAIR_SKEW, item, coupledItem, layer ) :771
                        -> SetTargetSkewDelay or SetTargetSkew
```

So the **length and skew targets come from the design rules through the host**, not from the placer. The only constraint the placer itself queries is the clearance (section 4.2).

### 10.3 Settings in, status out

`Update` (`:1193`) is the whole interaction with the router, and it runs on every mouse move of the generator tool:

```
Update( tool, board, commit ):
    if not IN_EDIT: return false                                              :1195
    if router->RoutingInProgress(): router->StopRouting()                     :1237
    reset the copper to m_baseLine (and m_baseLineCoupled for a pair)         :1251, :1264
    startItem = HELPERS::PickSegment( router, m_origin, layer, snap, *m_baseLine ) :1278
    endItem   = HELPERS::PickSegment( router, m_end,    layer, snap, *m_baseLine ) :1279
    router->SetMode( GetPNSMode() )                                           :1287
    if not router->StartRouting( startSnapPoint, startItem, pnslayer ): return false :1289
    placer = static_cast<MEANDER_PLACER_BASE*>( router->Placer() )            :1295
    m_settings.m_keepEndpoints = true                                         :1297
    placer->UpdateSettings( m_settings )                                      :1298
    router->Move( m_end, nullptr )                                            :1300
    if m_trackWidth == 0: take width and gap from the placer or the start item :1305 to :1317
    m_settings     = placer->MeanderSettings()          # read the flip back  :1319
    m_lastNetName  = iface->GetNetName( startItem->Net() )
    m_tuningStatus = placer->TuningStatus()                                   :1321
    m_tuningLength = placer->TuningLengthResult()                             :1322
    statusMessage  = "too long" | "too short" | "tuned" | "unknown"           :1328
    m_tuningInfo.Printf( "%s (%s)", formatted length, statusMessage )         :1351
```

The round trip at `:1298` and `:1319` is what carries `flipInitialSide` (section 3.2) back onto the board item.

The status readout the user sees is `TUNING_STATUS_VIEW_ITEM` (`pcb_tuning_pattern.h:46`), built in `GetPreviewItems` (`pcb_tuning_pattern.cpp:2003`). It shows the tuned path as hover items (`:2014` to `:2019`), a scope line naming the net and chain (`:2047`), a min and max taken from the DRC constraint rather than from `m_settings` (`:2054` for skew, `:2085` for length), a header label that is "current skew", "current delay" or "current length" (`:2147` to `:2151`) and the value from `TuningLengthResult()` or `TuningDelayResult()` (`:2155`). `m_tuningInfo` also appears in the properties grid row (`pcb_tuning_pattern.h:540`) and in `GetMsgPanelInfo` (`pcb_tuning_pattern.cpp:2233`).

Two live adjustments run through the placer rather than the item: `PCB_ACTIONS::spacingIncrease` / `spacingDecrease` call `placer->SpacingStep( +/-1 )` and copy the result back onto the pattern (`:2764`, `:2765`), and `amplIncrease` / `amplDecrease` call `placer->AmplitudeStep( +/-1 )` and copy `m_maxAmplitude` back (`:2785`, `:2786`). Both then re run `Update`.

`GetProperties` / `SetProperties` (`:1790`, `:1833`) persist twenty three keys, of which the meander relevant ones are `tuning_mode`, `initial_side`, `last_status`, `is_time_domain`, `end`, `corner_radius_percent`, `single_sided`, `rounded`, `max_amplitude`, `min_amplitude`, `min_spacing`, the nine target min/opt/max triples, `last_track_width`, `last_diff_pair_gap`, `last_tuning_length`, `last_netname`, `override_custom_rules`, `base_line` and `base_line_coupled`. `rounded` is the corner style as a bool (`:1802`, `:1858`).

### 10.4 What the host commits

`EditFinish` (`:1357`):

```
EditFinish( tool, board, commit ):
    ClearFlags( IN_EDIT );  iface->EraseView()                                :1361
    if router->RoutingInProgress():                                           :1371
        router->FixRoute( m_end, nullptr, forceFinish = true, forceCommit = false ) :1376
        router->StopRouting()
    for each GENERATOR_PNS_CHANGES pnsCommit in tool->GetRouterChanges():      :1397
        for item in pnsCommit.removedItems:                                    :1406
            if tool->ItemCreatedBySession( item ): continue                     :1409
            view->Hide( item, false );  commit->Remove( item )                  :1415
        for item in pnsCommit.addedItems:                                       :1418
            if world->FindItemByParent( item ) == nullptr: continue              :1421
            if not withinBounds( item ) and it is a track:                       :1431
                restore m_assembledLineWidth                                     :1436
            commit->Add( item )
            if withinBounds( item ): AddItem( item )       # into the group       :1443
```

So the router's own commit produces the tracks, and the generator then sorts them: the ones inside the pattern's outline join the group, the ones outside are the reconstructed remainder of the original line and get their original width forced back. `withinBounds` tests both endpoints against `getOutline()` inflated by the DRC epsilon (`:1207`).

`getOutline()` (`:1612`) is an offset of the baseline by `maxAmplitude + width/2` for a single track, or `maxAmplitude + gap/2 + width` for a pair (`:1715`, `:1717`), with the same `MinAmplitude` floor arithmetic as `pns_meander.cpp:411`, typo included (`:1626`). It is the pattern's selection and hit test shape and has no effect on the router.

`EditCancel` (`:1449`) just un hides and calls `StopRouting()`.

### 10.5 The QA surface

`qa/tools/pns/mock_pcb_tuning_pattern.cpp` exists so that the PNS debug tool and the regression runner link without pulling in the whole generator stack. **Every method is a stub**: `CreateNew` returns null (`:199`), `EditStart`, `Update`, `EditFinish`, `EditCancel`, `Remove` are empty (`:209`, `:270`, `:276`, `:281`, `:251`), `getOutline` returns an empty chain (`:306`), `DRAWING_TOOL::PlaceTuningPattern` returns 0 (`:360`). The only real content is the six string conversion helpers (`:80` to `:173`), which are the QA log's vocabulary for tuning: `"single"`, `"diff_pair"`, `"diff_pair_skew"`; `"default"`, `"left"`, `"right"`; `"too_long"`, `"too_short"`, `"tuned"`.

`qa/data/pcbnew/pns_regressions/` contains twelve cases and none of them is a tuning case; the log format has no tuning event and `ROUTER_MODE` is not in it at all (note 05 section 6.2). So there is nothing to replay a tuning session against, exactly as note 07 found for pairs.

---

## 11. What the port needs

Read out of the crate on 2026-09-10.

### 11.1 What already exists, verbatim

**Geometry.** Every primitive the meander generator and the three placers touch is present, and two of them were ported for this milestone specifically.

| Need | Crate | Line |
| --- | --- | --- |
| `SHAPE_LINE_CHAIN::Split( start, end, pre, mid, post )` | `LineChain::split_three_way` | `src/geometry/line_chain.rs:2274`, whose doc already cites `pns_meander_placer.cpp:241` and `pns_dp_meander_placer.cpp:262` |
| `SHAPE_LINE_CHAIN::Mirror( const SEG& )` | `LineChain::mirror` | `:931`, whose doc already cites `pns_meander.cpp:685` as its only caller |
| `SHAPE_LINE_CHAIN::Append( VECTOR2I )` with duplicate suppression | `LineChain::append` | `:556` |
| `SHAPE_LINE_CHAIN::Append( const SHAPE_LINE_CHAIN& )` | `LineChain::append_chain` | `:584` |
| `SHAPE_LINE_CHAIN::Length()` | `LineChain::length` | `:976`, per segment integer sum, as KiCad does |
| `SHAPE_LINE_CHAIN::Simplify` | `LineChain::simplify` | `:1126` |
| `SHAPE_LINE_CHAIN::Collide( SEG, clearance )` | `LineChain::collide_seg` | `:1808` |
| `SEG::Length`, `SquaredLength` | `Seg::length`, `squared_length` | `src/geometry/seg.rs:284`, `:293` |
| `SEG::Side` | `Seg::side` | `:361` |
| `SEG::LineProject` | `Seg::line_project` | `:589` |
| `SEG::ApproxParallel` | `Seg::approx_parallel` | `:804` |
| `SEG::Contains( VECTOR2I )` | `Seg::contains_point` | `:835` |
| `SEG::NearestPoint` | `Seg::nearest_point_to_point` | `:483` |
| `VECTOR2I::Resize`, `EuclideanNorm`, `Perpendicular` | `Vec2::resize`, `euclidean_norm`, `perpendicular` | `src/geometry/vec2.rs:167`, `:117`, `:147` |
| `KiROUND` | `geometry::math::kiround` | `src/geometry/math.rs:23` |

**Engine.** `World::branch` (`src/node.rs:526`), `World::kill_children` (`:578`), `World::add_line` (`:2354`), `World::remove_line` (`:2438`), `World::check_colliding_line` (`:1768`), `World::assemble_line` (`:2524`), `Line::with_chain` (`src/line.rs:385`), `Line::set_shape` (`:503`), `Line::width` and `Line::net`. `topology::assemble_trivial_path` (`src/topology.rs:618`). `AlgoContext` (`src/algo_base.rs`).

**Pairs.** `DiffPair` with `chain_p` / `chain_n` (`src/diff_pair.rs:413`, `:418`), `set_shape` (`:428`), `gap` (`:480`), `set_gap` (`:491`), `width` (`:464`), `p_line` / `n_line` (`:634`, `:640`), `nets` (`:451`), `coupled_segment_pairs` (`:782`) returning `CoupledSegments { coupled_p, coupled_n, index_p, index_n }` (`:273`), `skew` (`:761`), `coupled_length` (`:831`). `common_parallel_projection` (`:209`).

**Rules.** `RuleResolver::constraint` (`src/rules.rs:329`) with `ConstraintType::Clearance` (`:192`), `Length` (`:196`) and `DiffPairSkew` (`:211`), plus `dp_coupled_net` (`:450`) and `dp_net_polarity` (`:460`). The doc comment on `constraint` at `:322` already names `pns_meander_placer_base.cpp:122` as one of its two callers and says "A host that implements nothing here still routes; it only loses length tuning". That sentence stops being hypothetical in this milestone.

**Sizes.** Nothing new. The tuning placers ignore `UpdateSizes` entirely (section 5.8); the only size a pair tuner reads is `Sizes::diff_pair_gap` (`src/settings.rs:432`), and only as the fallback when `AssembleDiffPair` could not recover a gap (`pns_dp_meander_placer.cpp:115`, `pns_meander_skew_placer.cpp:86`).

**RoutingSettings.** Nothing new either. None of `RouterMode`, `OptimizerEffort`, `shove_vias`, `remove_loops`, `smart_pads`, `follow_mouse`, `corner_mode` or any of the iteration limits is read by any tuning code path. A tuning session runs with no walkaround, no shove and no optimizer.

### 11.2 What is genuinely new

Nine items, in dependency order.

1. **`topology::assemble_tuning_path`** and its helper `walk_tuning_path` (`pns_topology.cpp:787`, `:611`). `src/topology.rs:29` lists both, plus `findLinesFromVia` (`:536`), under "what is not here: length tuning only". This is the milestone that removes that line. The two `OptimiseTraceInPad` / `OptimiseTraceInVia` fixups (`pns_topology.cpp:928`, `:1010`) are host geometry helpers that live outside the sparse checkout; the port should either leave the path chains alone and document the deviation or expose the fixup as a host hook.
2. **`topology::assemble_diff_pair`** (`pns_topology.cpp:1036`). Note 07 section 12.2 deferred it because milestone 10 never needed it; both pair tuners start with it and cannot start without it. It needs `World::all_items_in_net` (already present, `src/node.rs:2166`), `common_parallel_projection` (present) and `Seg::approx_parallel` with a threshold of `DP_PARALLELITY_THRESHOLD` (present).
3. **A length metric.** KiCad delegates to the host (`ROUTER_IFACE::CalculateRoutedPathLength`). The crate has no such interface. The honest port is a free function over the item set: sum `LineChain::length` for every line, add nothing for vias unless a host hook supplies a stackup height. Give the host an optional `RuleResolver` method (`fn path_length_extra(&self, ...) -> i64`, defaulted to 0) if via height is wanted later, and record in `doc/log/` that pad to die, pad entry optimisation, net chains and propagation delay are out of scope.
4. **`MeanderSettings`** and `CornerStyle`, `MeanderSide`, `MeanderType`, `TuningStatus`. Eleven of KiCad's twenty two fields (section 1.4), one corner style (section 9.4), and a `LengthTarget { min, opt, max }` value type in place of `MINOPTMAX` with its "unconstrained" sentinels, which should become `Option` per `DESIGN.md` section 11.
5. **`MeanderShape`.** The turtle, `make_miter_shape` (chamfer only), `gen_meander_shape`'s five bodies, `fit`, `recalculate`, `resize`, `make_empty`, `make_corner`, `update_base_segment`, `spacing`, `corner_radius`, `min_amplitude`, `baseline_length`, `current_length`, `min_tunable_length`. Deliberately omit `MakeArc`, `MT_ARC`, `BaseIndex` / `SetBaseIndex` and the round style (errata E1, E4).
6. **`MeanderedLine`.** `add_corner`, `add_meander`, `meander_segment`, `check_self_intersections`, `meanders`, `set_width`, `set_baseline_offset`. Omit `AddArc`, `AddArcAndPt`, `AddPtAndArc` (erratum E5). Ownership is a `Vec<MeanderShape>` by value, which deletes `Clear()` and the move assignment operator outright.
7. **The shared placer arithmetic.** `tune_line_length`, `find_amplitude_for_length`, `find_amplitude_binary_search`, `amplitude_step`, `spacing_step`, `clearance`, and the `TuningStatus` comparison. These are `MEANDER_PLACER_BASE` minus the net chain half (section 4.4) and minus the delay half.
8. **The three placers**, `src/placer/meander_placer.rs`, `dp_meander_placer.rs`, `skew_meander_placer.rs`. Section 11.4.
9. **The facade entry points and their recorded events.** Section 11.5.

### 11.3 The settings and status types

```rust
/// Port of `MEANDER_SETTINGS` (`pcbnew/router/pns_meander.h:69`), minus
/// the time domain and net chain halves KiCad's host feeds it.
pub struct MeanderSettings {
  pub min_amplitude: i32,           // :101, default 200_000
  pub max_amplitude: i32,           // :104, default 1_000_000
  pub spacing: i32,                 // :107, default 600_000
  pub step: i32,                    // :110, default 50_000, must be > 0
  pub corner_style: CornerStyle,    // :145
  pub corner_radius_percentage: i32,// :148, default 80
  pub single_sided: bool,           // :151, default false
  pub initial_side: MeanderSide,    // :154, default Left
  pub keep_endpoints: bool,         // :160, the host forces true
  pub target_length: LengthTarget,  // :125
  pub target_skew: LengthTarget,    // :137
}

/// Port of `MEANDER_PLACER_BASE::TUNING_STATUS`
/// (`pcbnew/router/pns_meander_placer_base.h:49`).
pub enum TuningStatus { TooShort, TooLong, Tuned }
```

`LengthTarget` replaces `MINOPTMAX<long long>` plus `LENGTH_UNCONSTRAINED`. KiCad's setters (section 1.3) fill min and max from an opt with a fixed 100000 nm tolerance, so the natural shape is:

```rust
pub struct LengthTarget { pub min: i64, pub opt: i64, pub max: i64 }
impl LengthTarget {
  /// `MEANDER_SETTINGS::SetTargetLength( long long )`
  /// (`pcbnew/router/pns_meander.cpp:67`). `DEFAULT_LENGTH_TOLERANCE`
  /// is `pcbIUScale.mmToIU( 0.1 )`, 100000 nm (`:31`).
  pub const fn around(opt: i64) -> Self { ... }
  /// The three values a design rule supplied.
  pub const fn explicit(min: i64, opt: i64, max: i64) -> Self { ... }
}
```

`Option<LengthTarget>` says "unconstrained" without a one kilometre sentinel. Where KiCad tests `Opt() != LENGTH_UNCONSTRAINED`, the port matches on `Some`. Note that a `None` target and a `Some` target with a huge opt behave differently in `doMove`: the former should short circuit to `TooShort` without meandering at all, where KiCad meanders as hard as it can. Pick one, and pin it in a test.

`m_step` must be rejected at zero, because `Fit`'s loop decrements by it (`pns_meander.cpp:789`).

### 11.4 The `Placer` enum and the placer modules

`src/placer/mod.rs:48` currently has two variants and its module doc already says "the set of placers is closed by the router mode, so `Placer`, an enum wrapping them, is a better fit than a trait object". Milestone 11 takes it to five, which is KiCad's full set:

```rust
pub enum Placer {
  Line(Box<line_placer::LinePlacer>),
  DiffPair(Box<diff_pair_placer::DiffPairPlacer>),
  Meander(Box<meander_placer::MeanderPlacer>),
  DpMeander(Box<dp_meander_placer::DpMeanderPlacer>),
  SkewMeander(Box<skew_meander_placer::SkewMeanderPlacer>),
}
```

Eight of the twenty two methods `Placer` already forwards are no ops on all three new variants and should be written as such with a one line citation each: `undo_last_segment`, `set_layer`, `toggle_via`, `flip_posture`, `set_ortho_mode`, `update_sizes`, `is_placing_via`, and `leading_rat_line` / `leading_rat_line_n` (a meander placer never draws one). That is section 5.8 as code.

Three methods need a new answer rather than a forward:

- **`traces`** returns one line for `Meander` and `SkewMeander`, two for `DpMeander`, P first, which is what `DP_MEANDER_PLACER::Traces` does (`pns_dp_meander_placer.cpp:626`).
- **`commit_node`** currently asks `has_placed_anything` on the line placer and reads `fixed_node` on the pair placer. A meander placer's `FixRoute` commits directly (`pns_meander_placer.cpp:371`), so the natural answer is `m_currentNode` when a move has produced one and `None` otherwise; that also fixes erratum E13 by construction, because the pair meander placer's null `m_currentNode` becomes a `None`.
- **`current_net`** and **`current_net_n`**: the single meander placer has one, the pair meander placer has two, the skew placer has two with the **active** lane first (`pns_meander_skew_placer.h:60`), which matters and should be documented on the method.

Three new methods are needed on the enum, all of them `Option` returning so that they answer `None` for the two routing placers:

```rust
pub fn tuning_status(&self) -> Option<TuningStatus>;      // TuningStatus()
pub fn tuning_length_result(&self) -> Option<i64>;        // TuningLengthResult(), a skew for SkewMeander
pub fn tuned_path(&self) -> Option<&[Line]>;              // TunedPath()
pub fn meander_settings(&self) -> Option<&MeanderSettings>;
pub fn update_meander_settings(&mut self, s: MeanderSettings);
pub fn amplitude_step(&mut self, sign: i32);
pub fn spacing_step(&mut self, sign: i32);
```

Inside the three placers, follow the shape the crate already uses: no back pointer to the placer from the shape. KiCad's `MEANDER_SHAPE::m_placer` (`pns_meander.h:417`) exists to reach three things, `MeanderSettings()`, `Clearance()` and `CheckFit()`. Pass a small borrowed context instead:

```rust
struct MeanderContext<'a> {
  settings: &'a MeanderSettings,
  clearance: i32,          // resolved once per move, not per Fit; erratum E11
  width: i32,
}
```

and make `check_fit` a closure or a trait the placer implements over that context. Resolving the clearance once per `move_to` rather than once per `spacing()` call removes erratum E11's repeated rule query and the trace rebuild it drags behind it, and it makes the shape generator a pure function of its context, which is what makes step 1 of section 13 unit testable without a world.

`flipInitialSide` (section 3.2) is the one piece of the design that resists this: it is the shape generator writing back into the placer's settings. Model it as a return value. `meander_segment` answers `MeanderSegmentResult { shapes: Vec<MeanderShape>, initial_side_flipped: bool }` and the caller applies the flip. That keeps the generator pure and makes the feedback visible in the signature.

### 11.5 The facade

`Router::start_routing` and `start_routing_diff_pair` (`src/router.rs:1403`, `:1471`) are the precedent. Note 07 section 12.4 chose separate entry points over a mode field and gave the reasons; the same reasons hold here, and more strongly, because a tuning start has a genuinely different signature: it needs the item to tune and the two points that bound the tuned stretch, and it has no layer argument to speak of, since `CurrentLayer()` is read off the clicked segment (`pns_meander_placer.cpp:435`).

```rust
pub fn start_tuning(
  &mut self,
  at: Vec2,
  start: HostId,
  mode: TuningMode,
  settings: MeanderSettings,
) -> Result<PreviewFrame, StartError>;
```

with `TuningMode { SingleLength, PairLength, PairSkew }` mapping onto the three placers, mirroring `LENGTH_TUNING_MODE` (`pcbnew/generators/pcb_tuning_pattern.h:38`). A single entry point with a mode is right here where it was wrong for pairs, because all three tuning modes take exactly the same arguments and differ only in which placer is built, which is precisely `ROUTER::StartRouting`'s switch (`pns_router.cpp:451`).

New `StartError` variants, from the three `SetFailureReason` calls:

| Variant | KiCad message | Line |
| --- | --- | --- |
| `NotATrack` | "Please select a track whose length you want to tune." | `pns_meander_placer.cpp:73`, `pns_dp_meander_placer.cpp:93` |
| `NotADiffPairForTuning` | "Unable to find complementary differential pair net for length tuning..." | `pns_dp_meander_placer.cpp:107` |
| `NotADiffPairForSkew` | "Please select a differential pair track you want to tune." / "...for skew tuning..." | `pns_meander_skew_placer.cpp:63`, `:79` |

`PreviewFrame` (`src/router.rs:405`) needs one addition, because a tuning session's whole point is the readout:

```rust
/// The tuning readout, when the session is a tuning session.
///
/// KiCad's host reads `TuningStatus()` and `TuningLengthResult()` off
/// the placer after every `Move` (`pcb_tuning_pattern.cpp:1321`, `:1322`)
/// and composes them into `m_tuningInfo` (`:1351`).
pub tuning: Option<TuningReadout>,

pub struct TuningReadout {
  pub status: TuningStatus,
  /// A length for the two length modes, a skew for `PairSkew`
  /// (`pns_meander_skew_placer.cpp:240`).
  pub result: i64,
  /// `TuningLengthDelta()` (`pns_meander_placer_base.h:70`), `None`
  /// when `HasBaseline()` is false (`:68`).
  pub delta: Option<i64>,
  /// The settings after the move, so a host can persist the
  /// `initial_side` flip (`pcb_tuning_pattern.cpp:1319`).
  pub settings: MeanderSettings,
}
```

The recording gains one variant, `SessionEvent::StartTuning { at, start, mode, settings }`, alongside the existing `StartRouting` and `StartRoutingDiffPair` (`src/eventlog.rs:147`, `:169`), plus a pair of `AmplitudeStep { sign }` and `SpacingStep { sign }` events for the two live adjustments (`pcb_tuning_pattern.cpp:2764`, `:2785`), because a replay that cannot reproduce them cannot reproduce the geometry.

### 11.6 Determinism

Three places to pin (`DESIGN.md` section 8):

- **`MeanderSegment`'s side flip.** `flipInitialSide` writes into the placer's settings mid loop, so the meanders on the second base segment of a move depend on what happened on the first. That is deterministic but order sensitive; the port must keep the base segments in chain order and must apply the flip at the same points (`pns_meander.cpp:307`, `:331`).
- **`Fit`'s amplitude scan.** Largest first in `m_step` decrements (`:789`), stopping at the first success. The scan order is the answer, not just a search strategy.
- **`findAmplitudeBinarySearch`'s left before right recursion** (`:164`, `:170`). The first branch that returns non zero wins, so the traversal order is part of the result. Reproduce it exactly, errata E8 included, and pin it with a table test before deciding whether to fix it.

`CheckSelfIntersections` iterates the meander vector backwards (`:697`) and the inner segment loop backwards too (`:712`); since the answer is a bool, order does not change it, but it does change which collision is found first if the port ever wants to report one.

---

## 12. Errata

Everything here was verified against the tree at `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85`. "No caller" means a grep over `pcbnew/`, `qa/` and the rest of the sparse checkout, plus `git grep` over the whole tree for the files outside it, returns only the definition and, where applicable, the declaration.

### E1. Dead members, dead fields and dead locals, collected

- `MEANDER_SETTINGS::m_lenPadToDie` (`pcbnew/router/pns_meander.h:113`): written by nothing, read by nothing. The pad to die length that is actually used lives on the placers as `m_padToDieLength` and comes from `SOLID::GetPadToDie` (`pns_meander_placer.cpp:92`).
- `MEANDER_SETTINGS::m_lengthTolerance` (`:157`): written by nothing, read by nothing. The tolerance in force is `DEFAULT_LENGTH_TOLERANCE` baked into the setters.
- `MEANDER_SHAPE::m_baseIndex` with `SetBaseIndex` and `BaseIndex()` (`:453`, `:216`, `:224`): written at `pns_meander.cpp:275` and `:769`, read nowhere.
- `MEANDER_PLACER_BASE::m_currentEnd` (`pns_meander_placer_base.h:186`): assigned `(0,0)` at `pns_meander_placer.cpp:105` and `pns_meander_skew_placer.cpp:131`, never assigned anything else, returned by `CurrentEnd()` (`pns_meander_placer.cpp:430`, `pns_dp_meander_placer.cpp:660`). The host never reads it.
- `SHAPE_LINE_CHAIN lc;` in `MEANDERED_LINE::MeanderSegment` (`pns_meander.cpp:256`): declared, never touched.
- `m_last = aBase.A` at `pns_meander.cpp:268` is redundant: `AddCorner( aBase.A )` two lines earlier already set it (`:871`).

Do not port any of them.

### E2. `SetTargetSkewDelay` uses the length constants

`pcbnew/router/pns_meander.cpp:211`. It compares `aOpt == SKEW_UNCONSTRAINED` where every other delay setter compares against `DELAY_UNCONSTRAINED`, and it pads with `DEFAULT_LENGTH_TOLERANCE` (`:222`, `:223`) where every other delay setter pads with `DEFAULT_DELAY_TOLERANCE`. The padding bug is numerically invisible because `mmToIU( 0.1 ) == 0.1 * IU_PER_PS == 100000` (`:31`, `:34`, from `include/base_units.h:68` and `:76`). The sentinel bug is real but harmless, because `m_targetSkewDelay` is a `MINOPTMAX<int>` and `DELAY_UNCONSTRAINED` is 1e12, which no `int` can hold. Not ported: the crate has no delay concept.

### E3. `MinAmplitude`'s chamfer correction is `tan( 1 - tan( 22.5 deg ) )`

`pcbnew/router/pns_meander.cpp:421`. The same expression written correctly eighteen lines later (`:439`) is `1 - tan( DEG2RAD( 22.5 ) )`. The values are 0.6634702554 against 0.5857864376, so the chamfered minimum amplitude is 13.3 percent larger than intended: 132694 nm instead of 117157 nm for a 200000 nm track. It is masked whenever `m_minAmplitude` dominates, which is the default case (200000 > 132694), so it only bites for a host that lowers the minimum amplitude below the correction. The typo is copied verbatim into the host at `pcbnew/generators/pcb_tuning_pattern.cpp:1626`, where it sizes the pattern's selection outline. Reproduce or fix by explicit decision, and record it in `doc/log/`.

### E4. `MakeArc` sets `MT_CORNER`, so `MT_ARC` is never produced

`pcbnew/router/pns_meander.cpp:918` sets `MT_CORNER` where the type exists and is named `MT_ARC` (`pns_meander.h:48`). Nothing anywhere in the tree assigns `MT_ARC`. The four sites that test for it are therefore dead: `tuneLineLength`'s three skips (`pns_meander_placer_base.cpp:211`, `:261`, `:276`) and, by omission, `CheckSelfIntersections`'s skip set, which lists `MT_EMPTY` and `MT_CORNER` only (`pns_meander.cpp:701`) and happens to be correct because arcs arrive as `MT_CORNER`. Do not port `MT_ARC`.

### E5. `AddArcAndPt` and `AddPtAndArc` have no caller

`pcbnew/router/pns_meander.cpp:888` and `:896`, declared at `pns_meander.h:524` and `:533`. Both construct a degenerate `SHAPE_ARC( pt, pt, pt, 0 )` for the side that has no arc. `AddArc` itself does have callers (`pns_meander_placer.cpp:252`, `pns_dp_meander_placer.cpp:398`, `:412`, `:433`). Do not port either.

### E6. The skip advance's corner radius term is always zero

`pcbnew/router/pns_meander.cpp:394`: `nextP = tmp.spacing() - 2 * tmp.cornerRadius() + Settings().m_step`. `tmp` is a `MEANDER_SHAPE` constructed three lines earlier (`:390`), so its `m_amplitude` is 0 and `cornerRadius()` returns 0 at its first line (`:431`). The advance is always `spacing() + m_step`. Transcribe the simplified form and note why.

### E7. `CheckSelfIntersections` never looks at the N lane

`pcbnew/router/pns_meander.cpp:710` and `:714` both index `CLine( 0 )`. For a dual meander the second chain is never tested against anything, either as the candidate or as the obstacle. `DP_MEANDER_PLACER::CheckFit` does test both lanes against the **node** (`pns_dp_meander_placer.cpp:613`, `:616`), so the gap is only in the meander against meander test. Since the two lanes are a fixed offset apart and the offset is baked into `spacing()` and `cornerRadius()`, a self intersection of lane N without one of lane P is hard to construct; it is still a hole. The port should test both chains and note the deviation.

### E8. `findAmplitudeForLength`'s fast path measures one amplitude and returns another

`pcbnew/router/pns_meander_placer_base.cpp:192` resizes the working copy to `minAmp`; `:195` returns `initialGuess`. The guard at `:190` has already established `minAmp <= initialGuess <= maxAmp`, so the two are different numbers except by coincidence. The intent is plainly `copy.Resize( initialGuess )`. As written the fast path fires only when the **minimum** amplitude is already within 20 nm of the target, and then returns a larger amplitude than the one that was measured. Reproduce first with a pinning test, then decide.

### E9. `doMove`'s early "too long" test ignores its own argument

`pcbnew/router/pns_meander_placer.cpp:282` reads `m_settings.m_targetLength.Max()` where the final comparison twelve lines later reads the `aTargetMax` argument (`:332`). `MEANDER_PLACER::Move` passes `m_targetLength.Max()` as that argument (`:224`), so the two agree. `MEANDER_SKEW_PLACER::Move` passes `m_coupledLength + m_targetSkew.Max() - offset` (`:236`), so they do not. Because the default `m_targetLength.Max()` is `LENGTH_UNCONSTRAINED`, one kilometre (`pns_meander.cpp:49`, `:74`), the early test simply never fires during skew tuning and the behaviour is correct by accident. Port `:282` reading the argument, and note the deviation.

### E10. Two different ad hoc self intersection clearances

`MEANDER_PLACER::CheckFit` uses `width + m_settings.m_spacing` (`pcbnew/router/pns_meander_placer.cpp:408`); `DP_MEANDER_PLACER::CheckFit` uses `width + width * 3`, that is four widths (`pns_dp_meander_placer.cpp:620`). Neither is a design rule clearance, and neither is documented. The single track number scales with the user's spacing setting, so raising the spacing makes the self intersection test stricter, which is backwards. Transcribe both verbatim and flag them in the port's doc comments.

### E11. `Clearance()` rebuilds the trace and re-queries the rules on every amplitude trial

`MEANDER_PLACER_BASE::Clearance` (`pcbnew/router/pns_meander_placer_base.cpp:114`) calls `Traces()` at `:119`, and `Traces()` is not a query: it assigns `m_currentTrace` from `m_originLine` and the possibly stale `m_finalShape` (`pns_meander_placer.cpp:416`) or two lines for a pair (`pns_dp_meander_placer.cpp:628`). It then issues an uncached `QueryConstraint` (`:122`). `Clearance()` is reached from `MEANDER_SHAPE::spacing()` (`pns_meander.cpp:460`, `:464`), which runs from `cornerRadius()` (`:442`), from `genMeanderShape()` (`:591`) and twice per iteration of `MeanderSegment`'s loop (`:277`, `:394`). So a single move issues one rule query per corner radius evaluation per candidate amplitude per meander. The port should resolve the clearance once per move into a `MeanderContext` (section 11.4) and note the deviation, which is a pure speedup with identical results as long as the host's rules do not change mid move.

The fallback when the host has no minimum clearance rule is the **track width** (`:125`), which then floors `spacing()` at twice the width. That is a surprising default and worth reproducing deliberately.

### E12. `DP_MEANDER_PLACER` carries five dead declarations and two dead members

- `long long int totalLength()` (`pcbnew/router/pns_dp_meander_placer.h:109`): declared, never defined, never called. It would not link if anything called it.
- `void meanderSegment( const SEG& )` (`:124`): declared, never defined.
- `void setWorld( NODE* )` (`:132`): declared, never defined.
- `void release()` (`:133`): defined empty (`pns_dp_meander_placer.cpp:175`), never called.
- `const LINE Trace() const` (`:85`): defined (`:68`), never called.
- `DIFF_PAIR::COUPLED_SEGMENTS_VEC m_coupledSegments` (`:148`): never written or read; `Move` uses a local (`:252`).
- `ITEM_SET m_tunedPath` (`:151`): never written or read in this class; the two per lane sets are.

Do not port any of them.

### E13. `DP_MEANDER_PLACER::FixRoute` dereferences `m_currentNode` without a null check

`pcbnew/router/pns_dp_meander_placer.cpp:576`. `MEANDER_PLACER::FixRoute` guards the same dereference (`pns_meander_placer.cpp:366`). `m_currentNode` is null after `Start` and stays null if `Move` returns at its first line, `if( m_currentStart == aP ) return false` (`:249`), which is exactly what happens when the user clicks and fixes without moving. `PCB_TUNING_PATTERN::EditFinish` calls `FixRoute` whenever `RoutingInProgress()` (`pcb_tuning_pattern.cpp:1371`, `:1376`) and `Update` calls `Move` unconditionally (`:1300`) but does not check its return. In the port this cannot arise: `commit_node` answers `None` (section 11.4).

### E14. `MEANDER_SKEW_PLACER::Start` writes `m_tunedPath` twice

`pcbnew/router/pns_meander_skew_placer.cpp:75` assigns it from `AssembleTrivialPath`; `:155` or `:163` overwrites it from whichever `AssembleTuningPath` result matches the active lane. The first assignment is dead, and it is the only call to `AssembleTrivialPath` in any tuning code. Do not port it.

### E15. The turtle truncates toward zero where the rest of the router rounds

Four appends in `makeMiterShape` cast a `double` coordinate with `( int )`: `pcbnew/router/pns_meander.cpp:486`, `:513`, `:515`, `:518`. The three implicit `VECTOR2D` to `VECTOR2I` conversions in `genMeanderShape` (`:625`, `:650`, `:652`, `:674`) and the one at `:395` do the same through `VECTOR2`'s converting constructor, which is a `std::clamp` and a `static_cast` (`libs/kimath/include/math/vector2d.h:97`). Everywhere else in the router a fractional coordinate goes through `KiROUND`. The consequence is a systematic bias toward the origin of up to 1 nm per vertex, asymmetric between positive and negative coordinates, so a meander on the left of the origin is not the mirror image of the same meander on the right. Reproduce it literally, or round and record the deviation; either way pin it with a test that meanders the same segment at `x > 0` and at `x < 0` and compares the two shapes.

### E16. `findAmplitudeBinarySearch` can return an amplitude it never measured

`pcbnew/router/pns_meander_placer_base.cpp:139`: the `minAmp == maxAmp` base case returns `maxAmp` with no length test. The recursion tries the left half first and returns the first non zero answer (`:164` to `:169`), so a narrow interval bottoms out immediately and reports success. `0` is both the "not found" sentinel and a legal amplitude, which the caller papers over with `amp = max( amp, minAmpl )` (`:285`). `int minLen = aCopy.CurrentLength()` narrows a `long long` (`:143`, `:146`), harmless for board sized values. `LENGTH_TARGET_TOLERANCE = 20` (`:32`) is a namespace scope `const int` with no declaration in any header.

### E17. `Fit` assigns `m_baseSeg` twice in its check path

`pcbnew/router/pns_meander.cpp:763` sets `m_baseSeg = aSeg` and `:768` sets `m_baseSeg = m1.m_baseSeg`. `m1` was fitted against the same `aSeg` (`:752`), so the two are equal and the first assignment is dead. Also note the missing space in `m_baseSeg =aSeg;`.

### E18. `doMove` adds four corner shapes per base segment where two would do

`pcbnew/router/pns_meander_placer.cpp:270` and `:272` call `AddCorner( s.A )` and `AddCorner( s.B )`, and `MEANDERED_LINE::MeanderSegment` adds the same two itself for a non dual line (`pns_meander.cpp:263`, `:407`). The duplicates are harmless, because a corner shape is a one point chain and `SHAPE_LINE_CHAIN::Append` drops a repeat of the last point (`libs/kimath/include/geometry/shape_line_chain.h:539`), but they double the corner entries `tuneLineLength` walks past. The pair placer does not have the problem, because `MeanderSegment` skips its own `AddCorner` calls when `m_dual` is true (`pns_meander.cpp:262`, `:406`) and the placer emits the corner pairs itself.

### E19. The properties dialog compares a skew against the length sentinel

`pcbnew/dialogs/dialog_tuning_pattern_properties.cpp:95` tests `m_targetLength.GetValue() == PNS::MEANDER_SETTINGS::LENGTH_UNCONSTRAINED` in the **skew**, non time domain branch, where the value came from `m_settings.m_targetSkew.Opt()` (`:93`) and the sentinel is `SKEW_UNCONSTRAINED` (`pns_meander.cpp:37`). `m_targetSkew` is a `MINOPTMAX<int>`, so it can never hold 1e12, and the "clear the field when unconstrained" branch never fires: an unconstrained skew shows in the dialog as 2147483647 nm, 2.147 metres. The time domain branch six lines up gets it right (`:87`). Host side only.

### E20. `DP_MEANDER_PLACER::Move`'s first bail out leaves the delay stale and forces `TOO_SHORT`

`pcbnew/router/pns_dp_meander_placer.cpp:270` to `:277`, the empty tuned chain guard added for issue 22041. It sets `m_lastLength` and `m_lastStatus = TOO_SHORT` unconditionally, where the second bail out eight lines down calls `updateStatus()` (`:305`) and therefore reports honestly. Neither writes `m_lastDelay`, which keeps whatever the previous move left. Reproduce the second form for both.

### E21. `pairOrientation` is consulted for the first coupled span only

`pcbnew/router/pns_dp_meander_placer.cpp:315` reads `coupledSegments[0]` and sets the baseline offset's sign once for the whole move (`:318`). A pair that crosses over part way along the tuned stretch, so that P and N swap sides of the centreline, gets the wrong sign for every span after the crossing, and the two lanes' meanders are generated on top of each other. Constructing one requires a crossover inside the tuned range, which is unusual but legal. The port should either compute the sign per span or refuse a tuned range that contains a crossover, and either way say so.

### E22. `Fit`'s amplitude loop does not terminate when `m_step` is zero

`pcbnew/router/pns_meander.cpp:789`: `for( int ampl = maxAmpl; ampl >= minAmpl; ampl -= st.m_step )`. Nothing in KiCad's tree can set `m_step` to zero (the dialog does not expose it, and `CreateNew` seeds it from the board defaults whose own default is 50000, `:44`), so this is a latent hazard rather than a live bug. `MeanderSegment` also compares `remaining < Settings().m_step` twice (`:317`, `:385`), which with a zero step turns two loop exits into no exits. The port should reject a non positive step at the settings boundary.

### E23. The one meander unit test does not test the meander code

`qa/tests/pcbnew/test_meander_corner_radius.cpp` includes `router/pns_meander.h` and asserts four `MEANDER_SETTINGS` defaults (`:43` to `:52`). Its other two cases re-implement `cornerRadius`'s clamp arithmetic inline (`:122` to `:126`) and assert that the re-implementation agrees with a table; no `MEANDER_SHAPE` is constructed and `Fit` is never called anywhere in KiCad's test suite. There is therefore **no upstream test the port can mirror** for the shape generator, which is why section 13 step 1 asks for hand computed expectations and section 2.6 supplies them.

---

## 13. Proposed order of implementation

Nine steps, each of which leaves the crate compiling, clippy clean and tested. The first four need no world, no resolver and no router: they are the whole of sections 1 to 3 and they carry all of the risk, because the shape generator is where the integer transcription happens and where KiCad has no test to mirror (erratum E23).

**Step 1: the settings and status types.** `MeanderSettings`, `LengthTarget`, `CornerStyle` (one variant, `#[non_exhaustive]`), `MeanderSide`, `MeanderType` (seven variants; omit `MT_ARC`, erratum E4), `TuningStatus`. Reject a non positive `step` (erratum E22). No behaviour yet.

Verify: `dev/in-container.sh cargo test meander_settings`; a round trip test that `LengthTarget::around( n )` gives `n +/- 100000` per `pns_meander.cpp:78`, and that the defaults match `pns_meander.cpp:42` to `:63` exactly, which is the one thing KiCad's own suite does assert (`test_meander_corner_radius.cpp:43`).

**Step 2: the shape generator, chamfered corners only.** `MeanderShape` with `spacing`, `corner_radius`, `min_amplitude`, the turtle, `make_miter_shape`, `gen_meander_shape`'s five bodies, `recalculate`, `resize`, `make_empty`, `make_corner`, `update_base_segment`, `baseline_length`, `current_length`, `min_tunable_length`. Takes a borrowed `MeanderContext` (section 11.4) rather than a placer back pointer, so the whole step is testable with no world at all.

Verify: this is where the port is most likely to diverge silently, so pin the geometry. For each of `MT_SINGLE`, `MT_START`, `MT_TURN`, `MT_FINISH` and `MT_EMPTY`, on a base segment along `+x` with `offset == 0`, assert the **complete point list** against section 2.6's closed forms, and assert `current_length` and `baseline_length` against the hand computed integers there. The concrete case with `width = 200000`, `clearance = 100000`, `spacing = 600000`, `radius = 80 percent`, `amplitude = 1000000` gives corner radius 240000, the nine point list ending at `(1200000, 0)`, `current_length == 2637644` and `baseline_length == 1200000`; those four numbers are the anchor for everything above them. Repeat each on a base segment along `-x` and along `+y` to pin the side and rotation handling, and once at `x < 0` to pin the truncation decision of erratum E15. Add a dual case with `offset == 200000` asserting that the two chains stay `400000` apart on the straights.

**Step 3: the fitting loop.** `MeanderedLine` with `add_corner`, `add_meander`, `meander_segment`, `check_self_intersections`, plus `MeanderShape::fit` and the two check types. `fit` needs a `check_fit` callback; in this step pass one that always accepts, so the loop is exercised without a world.

Verify: on a 6 mm base segment with the step 2 settings, assert the produced sequence of `(MeanderType, side, baseline_length)` triples, both single sided and turning. Assert that `meander_segment` reports `initial_side_flipped` when the first fit succeeds only on the other side. Assert the skip advance is `spacing + step` (erratum E6).

**Step 4: the length arithmetic.** `tune_line_length`, `find_amplitude_for_length`, `find_amplitude_binary_search`, with `LENGTH_TARGET_TOLERANCE = 20`. Reproduce errata E8 and E16 exactly, each behind a test that names the erratum, so a later fix is a visible test change.

Verify: a list of three meanders and an elongation smaller than one of them gives one truncated meander and two empty ones; an elongation larger than all three leaves all three at full amplitude; an elongation between the two shrinks every survivor by an equal share; a negative elongation empties everything.

**Step 5: `assemble_tuning_path`.** `topology::assemble_tuning_path` and `walk_tuning_path` (`pns_topology.cpp:787`, `:611`), and the length metric of section 11.2 item 3. Update the "what is not here" list at `src/topology.rs:29` in the same commit.

Verify: on the section 14 board, the path from the middle segment reaches both pads and its length is the sum of the track lengths; a path that runs through a via includes the via's neighbours; a path that hits a junction takes the longer branch.

**Step 6: the single trace placer.** `MeanderPlacer` with `start`, `move_to`, `do_move`, `check_fit`, `fix_route`, `commit_placement`, `abort_placement`, `traces`, `tuned_path`, `tuning_status`, `tuning_length_result`, `amplitude_step`, `spacing_step`, and the `Placer::Meander` variant with its eight no ops. This is the first step that needs a world and the section 14 resolver.

Verify: the fixture's `a_trace_reaches_a_longer_target` case, plus the too short and too long cases.

**Step 7: `assemble_diff_pair`.** `topology::assemble_diff_pair` (`pns_topology.cpp:1036`), which note 07 section 12.2 deferred. Update `src/topology.rs:31` in the same commit.

Verify: on the section 14 pair board, a click on either lane recovers both lines, the gap and the layer range; a click on a net with no coupled partner answers `None`; a click on a lane whose partner is not parallel at that point falls back to the joined items path (`pns_topology.cpp:1132`).

**Step 8: the pair length placer.** `DpMeanderPlacer`, `baseline_segment`, `pair_orientation`, the offset, the corner walk of `addCornersUntilIndex` reduced to its no arc case, and the `Placer::DpMeander` variant. Reproduce erratum E21's single sign decision with a test that names it.

Verify: the fixture's `a_pair_reaches_a_longer_target` case; both lanes stay `pitch` apart on every straight run; `DiffPair::coupled_length` over the committed geometry is at least some fraction of the total.

**Step 9: the skew placer and the facade.** `SkewMeanderPlacer` deriving nothing but reusing `do_move`, `current_skew`, the overridden `origin_path_length` and `tuning_length_result`; then `Router::start_tuning`, the three new `StartError` variants, `PreviewFrame::tuning`, `SessionEvent::StartTuning`, `AmplitudeStep`, `SpacingStep` and the recording round trip.

Verify: the milestone's acceptance criterion. The fixture's skew cases pass, and a recorded tuning session replays to the same commit diff.

Steps 1 to 5 and 7 are engine work with no host surface and can land as separate commits without touching `src/router.rs`. Steps 6, 8 and 9 each change the facade, so each needs its `CHANGELOG.md` entry and its `doc/work/011-meanders.md` checkbox. The LibrePCB task in that work item stays on hold: the pair half of the milestone has no LibrePCB counterpart while pairs themselves are on hold (`PLAN.md`), and the single trace half needs a tool state and a settings panel that `doc/librepcb-integration.md` does not yet describe.

---

## 14. A synthetic fixture

There is no tuning case in `qa/data/pcbnew/pns_regressions/` and the log format cannot express one (section 10.5), and KiCad's unit test suite never constructs a `MEANDER_SHAPE` (erratum E23). So, as in note 07 section 15, the fixture is built by hand. `tests/meander.rs` for the shape generator's own tests and `tests/meander_placer.rs` for the three placers, following the split `tests/diff_pair.rs` and `tests/diff_pair_placer.rs` already use.

### 14.1 Common constants

```rust
const CLEARANCE: i32 = 100_000;   // the rule every scenario routes to
const WIDTH: i32     = 200_000;   // one lane
const GAP: i32       = 200_000;   // copper gap between the lanes
const PITCH: i32     = WIDTH + GAP;          // 400_000
const PAD_RADIUS: i32 = 150_000;

fn settings() -> MeanderSettings {
  MeanderSettings {
    min_amplitude: 200_000,
    max_amplitude: 1_000_000,
    spacing: 600_000,
    step: 50_000,
    corner_style: CornerStyle::Chamfer,
    corner_radius_percentage: 80,
    single_sided: false,
    initial_side: MeanderSide::Left,
    keep_endpoints: true,          // the host forces it (pcb_tuning_pattern.cpp:1297)
    target_length: LengthTarget::around( 10_000_000 ),
    target_skew: LengthTarget::around( 0 ),
  }
}
```

With those, section 2.6's derived numbers hold: `spacing()` is 600000, the corner radius is 240000, one full amplitude `MT_SINGLE` is 2637644 nm long over 1200000 nm of baseline, and its elongation is 1437644 nm.

### 14.2 The single trace board

One copper layer. A straight 8 mm track between two pads, and one obstacle to make `CheckFit` do something.

```
NetId(1) = the tuned net
NetId(2) = the obstacle's net

pad A  : circle r = 150000 at (       0, 0 )        net 1, layer 0
pad B  : circle r = 150000 at ( 8000000, 0 )        net 1, layer 0
track T: (0,0) to (8000000,0), width 200000         net 1, layer 0

obstacle (second fixture only):
  segment O: ( 4000000, 900000 ) to ( 5000000, 900000 ), width 200000, net 2
```

`O` sits 900000 nm above the base line, which is 900000 - 100000 (half the meander width) - 100000 (half its own width) = 700000 nm of clear space; a full amplitude meander on that side reaches 1000000 nm and collides, so `Fit` has to step the amplitude down or flip the side. That is the only way to exercise `CheckFit` and the `flipInitialSide` path with a board.

The tuned stretch is always `(1000000, 0)` to `(7000000, 0)`, 6 mm of baseline, which holds five full amplitude `MT_SINGLE` meanders at 1200000 nm each with 0 nm left over, or a turning run of `MT_START`, `MT_TURN`s and `MT_FINISH`.

### 14.3 The pair board

Same layer, two nets, both lanes straight and coupled over the whole span.

```
NetId(1) = P, NetId(2) = N

pad A_P : circle r = 150000 at (       0, -200000 )   net 1
pad A_N : circle r = 150000 at (       0,  200000 )   net 2
pad B_P : circle r = 150000 at ( 8000000, -200000 )   net 1
pad B_N : circle r = 150000 at ( 8000000,  200000 )   net 2
track P : ( 0, -200000 ) to ( 8000000, -200000 ), width 200000, net 1
track N : ( 0,  200000 ) to ( 8000000,  200000 ), width 200000, net 2
```

The two lanes are `PITCH` apart, which is what `assemble_diff_pair` recovers as `gap = 400000 - 200000 = 200000` (`pns_topology.cpp:1170`).

For a dual meander the derived numbers change and should be asserted directly, because they are what keeps the coupling: `offset = ( 200000 + 200000 ) / 2 = 200000`; `spacing() = max( 200000 + 100000 + 2 * 200000, 600000 ) = 700000`; `min_amplitude = max( 200000, 200000 + 132694 ) = 332694` with erratum E3's constant, or `317157` without it, which makes this the cheapest place to pin that decision; `corner_radius` floors at `200000 + 58578 = 258578`, `optCr = 700000 * 80 / 200 = 280000`, `maxCr = min( ( A + 200000 ) / 2, 350000 )`, so at full amplitude the radius is 280000 and `sCorner = 80000`, `uCorner = 480000`.

### 14.4 The unequal lane board, for the skew placer

The skew placer needs the two lanes coupled where the user clicks and unequal overall. Put the extra length in a detour near the start, outside the tuned range.

```
pad A_P : circle r = 150000 at (       0, -200000 )   net 1
pad A_N : circle r = 150000 at (       0,  200000 )   net 2
pad B_P : circle r = 150000 at ( 8000000, -200000 )   net 1
pad B_N : circle r = 150000 at ( 8000000,  200000 )   net 2

track P : one segment,  ( 0, -200000 ) to ( 8000000, -200000 )

track N : four segments, a rectangular detour that adds exactly 2 mm
          (       0,  200000 ) to (       0, 1200000 )
          (       0, 1200000 ) to ( 2000000, 1200000 )
          ( 2000000, 1200000 ) to ( 2000000,  200000 )
          ( 2000000,  200000 ) to ( 8000000,  200000 )
```

P is 8000000 nm, N is 1000000 + 2000000 + 1000000 + 6000000 = 10000000 nm. The two lanes are coupled and `PITCH` apart from `x = 2000000` onwards, so a click anywhere at `x > 2000000` lets `assemble_diff_pair` pair the clicked P segment with N's last segment. The tuned stretch is `(3000000, 0 - 200000)` to `(7000000, -200000)`, 4 mm, which holds three full amplitude singles.

### 14.5 The resolver

`FixedClearance` (`src/rules.rs:526`) and `CoupledNets` (`:682`) both leave `RuleResolver::constraint` at its default `None` (`:329`). That matters here in a way it did not for any earlier milestone: `MEANDER_PLACER_BASE::Clearance()` is a `CT_CLEARANCE` constraint query (`pns_meander_placer_base.cpp:122`), and when it comes back without a minimum KiCad falls back to the **track width** (`:125`), which then floors `spacing()` at `2 * width = 400000`. With the section 14.1 spacing of 600000 the floor never binds, so the fixture would pass either way and the fallback would go untested.

So the fixture needs a resolver that answers the constraint, and a second case that deliberately does not:

```rust
/// A resolver that answers the clearance both ways: as a pairwise
/// clearance and as a `CT_CLEARANCE` constraint, which is what
/// `MEANDER_PLACER_BASE::Clearance` asks for
/// (`pcbnew/router/pns_meander_placer_base.cpp:122`).
struct TuningRules { inner: CoupledNets, clearance: i32 }

impl RuleResolver for TuningRules {
  fn constraint( &self, kind: ConstraintType, _a: ItemRef<'_>,
                 _b: Option<ItemRef<'_>>, _layer: i32 ) -> Option<Constraint> {
    match kind {
      ConstraintType::Clearance => Some( Constraint {
        constraint_type: kind, min: Some( self.clearance ),
        opt: None, max: None, allowed: true } ),
      _ => None,
    }
  }
  // everything else forwards to `inner`
}
```

The port should also decide, and pin, what happens when the constraint is missing. KiCad's `wxCHECK_MSG` returns the width and logs; the crate has no logging channel in an algorithm, so either return the width silently and document it, or make the missing clearance a `StartError`. This note recommends the first, because it matches KiCad and because a host that has no clearance rule at all is already unable to route.

### 14.6 The cases

| Case | Board | What it pins |
| --- | --- | --- |
| `a_single_meander_matches_the_hand_computed_shape` | none | Section 2.6's nine point list, 2637644 and 1200000. Pure geometry, no world. |
| `the_four_shape_types_have_the_documented_elongations` | none | `MT_START`, `MT_TURN`, `MT_FINISH`, `MT_SINGLE` against the closed forms of section 2.6. |
| `a_meander_on_the_negative_side_mirrors_the_positive_one` | none | Erratum E15's truncation decision. |
| `the_fitting_loop_produces_a_turning_run` | none | Section 3.2's arm selection, and the sequence of types on a 6 mm segment. |
| `a_single_sided_run_is_all_singles` | none | The `singleSided` path, which skips both turning arms. |
| `tune_line_length_truncates_then_shrinks` | none | Section 4.5's three passes, one test per pass outcome. |
| `the_amplitude_fast_path_returns_an_untested_value` | none | Erratum E8, pinned deliberately. |
| `a_trace_reaches_a_longer_target` | 14.2, no obstacle | Target 10 mm on an 8 mm trace: status `Tuned`, result within `10_000_000 +/- 100_000`. |
| `a_target_that_cannot_be_reached_reports_too_short` | 14.2, no obstacle | Target 30 mm: status `TooShort`, result is the maximum the 6 mm stretch can give, and every meander is at `max_amplitude`. |
| `a_target_below_the_current_length_reports_too_long` | 14.2, no obstacle | Target 5 mm: `doMove`'s early test fires (`pns_meander_placer.cpp:282`), status `TooLong`, geometry untouched. |
| `an_obstacle_forces_a_smaller_amplitude` | 14.2, with `O` | `CheckFit` rejects the full amplitude on that side; the committed geometry clears `O` by at least `CLEARANCE`. |
| `an_obstacle_flips_the_initial_side` | 14.2, with `O` | `flipInitialSide` fires and the returned settings carry the flip (section 11.5's `TuningReadout::settings`). |
| `a_pair_reaches_a_longer_target` | 14.3 | Status `Tuned`; both lanes `PITCH` apart on every straight run; `DiffPair::coupled_length` over the commit is at least 80 percent of the shorter lane. |
| `a_pair_start_on_an_uncoupled_net_is_refused` | 14.2 | `Err( NotADiffPairForTuning )` (`pns_dp_meander_placer.cpp:107`). |
| `a_start_on_something_that_is_not_a_track_is_refused` | any | `Err( NotATrack )` (`pns_meander_placer.cpp:73`). |
| `the_skew_placer_lengthens_the_shorter_lane` | 14.4 | Click P, target skew 0: status `Tuned`, `tuning_length_result` (the skew) within `+/- 100_000` of zero. |
| `the_skew_placer_reports_too_long_on_the_longer_lane` | 14.4 | Click N, target skew 0: status `TooLong`, nothing meandered. Section 7.1's "it does not pick a lane". |
| `a_tuning_session_replays` | 14.2 | Record with `Recorder`, replay through `SessionRecording::from_text`, compare the commit diff. The milestone's acceptance criterion. |
| `amplitude_and_spacing_steps_replay` | 14.2 | Two `AmplitudeStep` and one `SpacingStep` between moves; the replay reproduces the geometry. |

Assert lengths and statuses, not raw coordinates, everywhere the board is involved: a coordinate assertion on a fitted meander breaks on every change to the amplitude scan, where "the result is within the tolerance of the target" survives. The raw coordinate assertions belong in the three world free cases at the top of the table, where they are hand computed and are the whole point.

### 14.7 What the fixture cannot cover

- **The round corner style**, and therefore any comparison against KiCad's default output: `MEANDER_STYLE_ROUND` needs `SHAPE_ARC` (section 9.4).
- **A tuned stretch that already contains an arc**, and the pair placer's four way arc walk (`pns_dp_meander_placer.cpp:392` to `:441`): the crate's `LineChain` cannot hold an arc (`src/geometry/line_chain.rs:26`).
- **Time domain tuning and net chains**: not ported (section 4.4).
- **Pad to die length**: the crate has no pad to die concept, so `origPathLength`'s first term is always zero.
- **A comparison against KiCad's own tuned geometry.** There is no corpus case, no log event and no unit test upstream, so the port's fidelity rests entirely on the hand computed shape assertions of section 2.6 and on the errata being reproduced deliberately rather than on any end to end replay. That is a weaker footing than milestones 3, 4 and 9 had, and it should be said out loud in `doc/work/011-meanders.md`.
