# KiCad PNS: arcs, from `SHAPE_ARC` to the host boundary

Reference architecture note for the arc milestone of this crate. Source tree: sparse checkout of KiCad master at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85`, read 2026-09-12. All `path:line` citations are relative to `/home/Tubbles/dev/ref/kicad/` unless another root is named.

Files read in full: `libs/kimath/include/geometry/shape_arc.h`, `libs/kimath/src/geometry/shape_arc.cpp`, `libs/kimath/src/geometry/direction_45.cpp`, `pcbnew/router/pns_arc.h`, `pcbnew/router/pns_arc.cpp`.

Read in the parts that touch arcs: `libs/kimath/src/geometry/shape_line_chain.cpp` and `libs/kimath/include/geometry/shape_line_chain.h` (every method whose behaviour changes with arcs, listed in section 2), `libs/kimath/src/geometry/shape_collisions.cpp` (the six arc cases and the dispatcher rows), `libs/kimath/src/trigo.cpp` (`CalcArcCenter`, both overloads), `libs/kimath/src/geometry/geometry_utils.cpp` (`GetArcToSegmentCount`, `CircleToEndSegmentDeltaRadius`), `libs/kimath/include/geometry/direction45.h`, `libs/kimath/include/geometry/shape.h`, `include/base_units.h`, `pcbnew/router/pns_utils.cpp` (`ArcHull`, `BuildHullForPrimitiveShape`), `pcbnew/router/pns_line_placer.cpp`, `pcbnew/router/pns_shove.cpp`, `pcbnew/router/pns_optimizer.cpp`, `pcbnew/router/pns_walkaround.cpp`, `pcbnew/router/pns_dragger.cpp`, `pcbnew/router/pns_component_dragger.cpp`, `pcbnew/router/pns_diff_pair.cpp`, `pcbnew/router/pns_diff_pair_placer.cpp`, `pcbnew/router/pns_meander.cpp`, `pcbnew/router/pns_meander_placer.cpp`, `pcbnew/router/pns_dp_meander_placer.cpp`, `pcbnew/router/pns_meander_skew_placer.cpp`, `pcbnew/router/pns_meander_placer_base.cpp`, `pcbnew/router/pns_helpers.cpp`, `pcbnew/router/pns_line.cpp`, `pcbnew/router/pns_node.cpp`, `pcbnew/router/pns_topology.cpp`, `pcbnew/router/pns_router.cpp`, `pcbnew/router/pns_tool_base.cpp`, `pcbnew/router/pns_logger.cpp`, `pcbnew/router/router_tool.cpp`, `pcbnew/router/router_preview_item.cpp`, `pcbnew/router/pns_kicad_iface.cpp`, `pcbnew/router/pns_joint.h`, `pcbnew/router/pns_item.h`.

Read through `git show HEAD:<path>` because they are outside the sparse checkout: `qa/tests/libs/kimath/geometry/test_shape_arc.cpp`, `qa/tests/libs/kimath/geometry/test_shape_line_chain.cpp`, `qa/tests/pcbnew/test_meander_corner_radius.cpp`, `common/io/io_utils.cpp`, `libs/kimath/include/mmh3_hash.h`, `libs/kimath/include/hash_128.h`.

Also read: `qa/tools/pns/qa_pns_regressions_main.cpp`, `qa/tools/pns/pns_log_file.cpp`, every board under `qa/data/pcbnew/pns_regressions/`; `/home/Tubbles/dev/ref/horizon/src/router/pns_horizon_iface.cpp`, `/home/Tubbles/dev/ref/horizon/src/board/track.hpp`, `/home/Tubbles/dev/ref/horizon/src/core/tools/tool_route_track_interactive.cpp`, `/home/Tubbles/dev/ref/horizon/src/dialogs/router_settings_window.cpp`; `/home/Tubbles/dev/librepcb/libs/librepcb/core/geometry/vertex.h`, `.../geometry/path.h`, `.../geometry/trace.cpp`, `.../utils/toolbox.h`, `.../utils/toolbox.cpp`, `.../types/angle.h`, `.../project/board/drc/boarddesignrulecheck.h`, `/home/Tubbles/dev/librepcb/libs/librepcb/rust-core/src/ffi/router_ffi.rs`.

What is deliberately not repeated here, because an earlier note has it:

- The `m_shapes` / `m_arcs` data model, the `SHAPE_IS_PT` sentinel, `ArcIndex`, `IsArcSegment` and the per operation arc survival table: note 01 section 6.2. Section 2 below extends it rather than restating it.
- `SHAPE_ARC`'s three point representation, `ConvertToPolyline`'s accuracy constants and the zero width rule inside a chain: note 01 section 7.1. Section 1 below is the full reading that section summarises.
- `ArcHull`'s shape and the half width rounding inconsistency across the hull family: note 01 section 12.2. Section 4.2 adds only what note 01 did not cover.
- `BuildInitialTrace`'s 45 and 90 degree skeleton and the `MIN_PRECISION_IU` re-snap: note 01 section 8.3. Section 3 is the line by line reading of the two rounded branches.
- The planned `ArcRef` representation: note 01 section 14.3. Section 11.1 evaluates it against what this note found.
- `DRAGGER::startDragArc` and `LINE::DragArc`: note 06 sections 2.6 and 3.5. Section 5.5 records only what those two sections did not, plus the three `DM_ARC` sites in `Drag`.
- Arcs in the meander placers, `MakeArc`, `MT_ARC` and the round corner style's cost: note 08 section 9 and errata E4, E5. Section 5.8 adds the two things note 08 did not need.
- The diff pair value layer and `CoupledSegmentPairs`: note 07. Section 5.7 adds the arc branches.
- The regression harness, the log format and the per case table: note 05 sections 6.2 to 6.9. Section 8.3 adds the arc specific reading and one new measurement.

---

## 0. Read this first: three summary facts

**Arcs are a first class item in the world but a second class citizen in every algorithm.** `PNS::ARC` is a full `LINKED_ITEM` with a hull, joints, an index entry and a shove handler, but every geometric transformation the router performs on a line either refuses to touch arcs (`pns_optimizer.cpp:713` to `:729`, `pns_line.cpp:695`, `pns_line_placer.cpp:1650`) or works on the polyline approximation and then tries to glue the arcs back on (`pns_line.cpp:661`). The router does not bend an arc; it moves it whole, drops it, or walks around it.

**The arc is carried inside the chain, the approximation is carried alongside it.** A `SHAPE_LINE_CHAIN` holding an arc holds both the exact `SHAPE_ARC` and a polyline of it at 1000 nm accuracy (`shape_line_chain.cpp:57` to `:62`, `:1614`). Which of the two a routine uses is not a matter of policy, it is a matter of which routine: `Length()` uses the arc (`:967`), `Intersect()` uses only the polyline (`:1741`, `:1841`), `Collide()` uses both in sequence (`:445` to `:489`, `:834` to `:893`). Getting this per routine table right is most of the port.

**Every arc result that crosses an integer boundary goes through `double`.** The centre, the radius, both angles and the length are all computed in floating point from the three integer points, and the centre is then *snapped to a round number* if the computed uncertainty allows it (`libs/kimath/src/trigo.cpp:529` to `:550`). Section 1.3 and erratum E1 are about that. Nothing in the crate's numeric model today has this property, and no amount of care in the Rust code removes it: it is inherent to the three point representation.

---

## 1. `SHAPE_ARC`

`libs/kimath/include/geometry/shape_arc.h:35`, implemented in `libs/kimath/src/geometry/shape_arc.cpp` (1168 lines).

### 1.1 Representation and derived values

Four stored fields and three cached ones (`shape_arc.h:327` to `:334`):

```cpp
VECTOR2I m_start; VECTOR2I m_mid; VECTOR2I m_end; int m_width;
BOX2I m_bbox;  VECTOR2I m_center;  double m_radius;   // Calculated values
```

The three points are the representation. `m_center`, `m_radius` and `m_bbox` are recomputed by `update_values()` (`shape_arc.cpp:411` to `:459`) at the end of every constructor and every mutator. Equality compares the three points and the width only (`shape_arc.h:299` to `:303`), so two arcs with the same endpoints and different cached centres cannot exist.

`update_values()` does three things: `m_center = CalcArcCenter( m_start, m_mid, m_end )` (`:413`), `m_radius = sqrt( |start - center|^2 )` in `VECTOR2D` (`:414`), and a bounding box from the three points plus every axis quadrant point the sweep crosses (`:416` to `:458`). The quadrant walk is skipped entirely when `m_radius >= INT_MAX / 2` (`:435`), so a near straight arc gets the bounding box of its three points, which is correct for that case.

`IsCCW()` (`shape_arc.h:310` to `:317`) is the one predicate that is exact: `(end - mid) x (start - mid) > 0` computed in `VECTOR2L`. It is the only handedness answer that does not go through a `double`.

`IsEffectiveLine()` (`shape_arc.cpp:248` to `:254`) is `SEG(start, mid).ApproxCollinear( SEG(mid, end) ) && (mid - start) . (end - mid) > 0`. It is the guard that sends five of the six arc collision cases down a segment path (section 6).

### 1.2 Constructors and the two `ConstructFrom` methods

| Form | Line | Notes |
| --- | --- | --- |
| default | `shape_arc.h:39` | width 0, radius 0, the three points default constructed. `update_values()` is **not** called, so `m_center` is `(0,0)` and `m_bbox` is default. |
| `(center, start, centralAngle, width)` | `shape_arc.cpp:41` | rotates `start` about `center` by `-angle/2` and `-angle` in `VECTOR2D`, then `KiROUND`s both (`:52` to `:56`). Centre is not stored. |
| `(start, mid, end, width)` | `:62` | the canonical one. Just stores and calls `update_values()`. |
| `(segA, segB, radius, width)` | `:74` | tangent to two segments. Uses `EDA_ANGLE`, `sin`, `LineProject` and `RotatePoint` (`:145` to `:171`). On non intersecting or zero length input it asserts and falls back to a 180 degree arc around `segA` (`:121` to `:133`). **No caller in `pcbnew/router/`.** |
| copy, copy with new width | `:178`, `:191` | copy the cached values directly; the width overload does not re-run `update_values()`, which is correct because width is not an input to it. |

`ConstructFromStartEndAngle( start, end, angle, width )` (`shape_arc.cpp:198` to `:213`): computes `center = CalcArcCenter( start, end, angle )` (the two point plus angle overload, `trigo.cpp:329`), then `m_mid = start` rotated about that centre by `-angle/2`. Start and end survive exactly; mid is a rounded rotation. Note `aWidth` is a `double` parameter assigned into an `int` member (`:204`), a silent truncation.

`ConstructFromStartEndCenter( start, end, center, clockwise, width )` (`:216` to `:245`): takes both radial angles, normalises them, subtracts, then normalises the difference into `[0, 360)` or `(-360, 0]` according to `clockwise`, and rotates `m_mid = start` about `center` by `-angle/2`. **The given centre is not stored**; `update_values()` recomputes a centre from the three points, and that recomputed centre generally differs from the one passed in. This is the single most surprising property of the type for a port, and it is what makes `amendArc` and `Slice`'s arc re-cut lossy (section 2.2).

### 1.3 `CalcArcCenter`, the numeric heart

`trigo.cpp:562` to `:580` widens the three integer points to `VECTOR2D`, calls the `VECTOR2D` overload, clamps each coordinate into `[INT_MIN + 100, INT_MAX - 100]` and `KiROUND`s.

The `VECTOR2D` overload (`trigo.cpp:371` to `:559`) has four early exits and one long numeric body:

- all three points inside a 5.0 unit box: the centroid of the three (`:386` to `:390`);
- start and mid, or mid and end, within 2.0 units: the midpoint of start and end (`:398`, `:399`);
- start and end within 2.0 units: the midpoint of start and mid (`:401`, `:402`);
- the axis aligned special case where one chord is horizontal and the other vertical: the midpoint of start and end (`:414` to `:420`).

Otherwise it forms the two perpendicular bisector slopes, guards each zero denominator with `numeric_limits<double>::epsilon()` or `1e-10` (`:423` to `:427`, `:434` to `:438`, `:471` to `:474`), nudges the two slopes apart by an epsilon each when they are equal and the points are not coincident (`:455` to `:459`), propagates a first order uncertainty through every term (`:485` to `:527`), and then does this:

```cpp
double rounded100CenterX = std::floor( ( centerX + 50.0 ) / 100.0 ) * 100.0;   // :529
...
if( std::abs( rounded100CenterX - centerX ) < dCenterX && ... )   // :539
    center.x = rounded100CenterX;                                  // :542
else if( ... rounded10CenterX ... )                                // :545
```

The centre is snapped to a multiple of 100 nm, or failing that 10 nm, whenever the propagated uncertainty covers the round value. The comment at `:534` to `:538` says why: "ALL values within the uncertainty range are equally true". For a port this means **the centre is not a continuous function of the three points**. A one nanometre move of an endpoint can move the reported centre by up to 50 nm, and the radius with it.

`CalcArcCenter( start, end, angle )` (`trigo.cpp:329` to `:368`) is a different algorithm: swap to make the angle positive and at most 180 degrees, compute the chord, `r = (chord/2) / sin(angle/2)`, `d = sqrt(r*r - chord*chord/4)` clamped at zero, then `start + chord/2 along + d perpendicular`. Zero `sin(angle/2)` returns the chord midpoint (`:352`, `:353`). It has no round number snapping.

So `ConstructFromStartEndAngle` and the three point constructor place the centre by two different routines with two different rounding behaviours, and `ConstructFromStartEndAngle` immediately throws its own centre away and re-derives one through the snapping routine.

### 1.4 Angles, radius and length

`GetStartAngle()` / `GetEndAngle()` (`shape_arc.cpp:936`, `:944`) build an `EDA_ANGLE` from `point - GetCenter()` in `VECTOR2L` and normalise into `[0, 360)`. `EDA_ANGLE` from a vector is an `atan2`, so both are `double`.

`GetCentralAngle()` (`:971` to `:1004`) has three cases: `start == end` returns `ANGLE_360` (`:976`), `IsEffectiveLine()` returns `ANGLE_0` (`:982`), otherwise the angular difference about the centre, pushed into the same sign as `IsCCW()` (`:992` to `:1001`). The comment at `:979` to `:982` is candid that a straight arc has no circumcircle and that any angle measured about the stand in centre is fabricated.

`GetRadius()` (`:1007`) returns the cached `double`. **There is no integer radius accessor**, so every caller that wants an `int` truncates or rounds itself; `ArcHull` truncates (`pns_utils.cpp:78`), `router_preview_item.cpp:284` passes the `double` straight to the GAL.

`GetLength()` (`:958` to `:968`) is `|radius * centralAngle_in_radians|`, with a special case returning the chord length when the central angle is exactly zero (`:964`, `:965`). It is a `double`, and `SHAPE_LINE_CHAIN::Length()` adds it into an `long long int` accumulator (`shape_line_chain.cpp:968`), truncating.

### 1.5 `ConvertToPolyline` and `GetArcToSegmentCount`

`ConvertToPolyline( aMaxError = ARC_HIGH_DEF, aActualError = nullptr )` (`shape_arc.h:296`, implemented `shape_arc.cpp:1013` to `:1076`).

```
r = GetRadius(); sa = GetStartAngle(); c = GetCenter(); ca = GetCentralAngle()
halfMaxError = max( 1.0, aMaxError / 2.0 )                                     :1022
external_radius = r + m_width / 2.0                                            :1028
if external_radius < halfMaxError or ca == 0 or dist(chord, mid) < halfMaxError:
    n = 0; effectiveError = external_radius                                    :1032 to :1039
else:
    n = GetArcToSegmentCount( external_radius, aMaxError, ca )                  :1042
    seg360 = n * 360.0 / |ca in degrees|                                        :1045
    effectiveError = CircleToEndSegmentDeltaRadius( external_radius, seg360 )   :1046
r += effectiveError / 2;  n *= 2                                               :1052, :1053
append m_start                                                                 :1055
for i in 1, 3, 5, ... < n:  append KiROUND( c + r * unit( sa + ca*i/n ) )       :1057 to :1068
append m_end                                                                   :1070
```

The doubling at `:1053` with the odd stride at `:1057` is what makes the first and last sub-segments half length, so the exact endpoints stay on the arc while the interior points sit on the inflated radius. The degenerate branch produces a two point chain, which `Append(SHAPE_ARC)` then refuses to tag as an arc (`shape_line_chain.cpp:1622`).

`GetArcToSegmentCount( radius, errorMax, arcAngle )` (`geometry_utils.cpp:38` to `:60`) clamps radius and error to at least 1, computes `arc_increment = 2 * acos( 1 - errorMax/radius )` in degrees, clamps it to at most `360/8 = 45` degrees, `KiROUND`s the quotient and returns at least 2. `CircleToEndSegmentDeltaRadius` (`:63` to `:79`) is `KiROUND( |radius * (1 - 1/cos(pi/segCount))| )` with the segment count floored at 3.

The two accuracies the router uses: 1000 nm for anything stored in a chain (`shape_line_chain.cpp:57` to `:62`, which is `ARC_HIGH_DEF / 5`, and `ARC_HIGH_DEF` is `pcbIUScale.mmToIU( 0.005 )` = 5000 nm, `include/base_units.h:128`, `:137`, `:68`), and 20000 nm for `ArcHull` (`pns_utils.cpp:88`, `ARC_LOW_DEF` = `mmToIU( 0.02 )`, `include/base_units.h:127`, `:136`).

### 1.6 `Collide`

Two overloads, both non virtual entry points for the dispatcher.

`Collide( const VECTOR2I& aP, clearance, aActual, aLocation )` (`shape_arc.cpp:844` to `:933`):

1. `minDist = aClearance + m_width / 2` (`:847`), bounding box reject (`:851`).
2. If `radius >= INT_MAX / 2.0`, approximate the arc as the two chords `start-mid` and `mid-end` and take the nearer (`:859` to `:879`).
3. Otherwise nearest point on the full circle, `dist = KiROUND( |nearestPt - aP| )` (`:882`, `:883`). A zero distance is recomputed as `KiROUND( radius - sqrt( |aP - center|^2 ) )` with the comment that `EuclideanNorm` would truncate first (`:886` to `:893`).
4. If `m_start != m_end`, an angular containment test decides whether the point is outside the sweep, and if so `dist` becomes the distance to the nearer endpoint (`:896` to `:919`).
5. Hit when `dist <= minDist`; `*aActual = max( 0, dist - m_width / 2 )` (`:927`).

`Collide( const SEG& aSeg, clearance, aActual, aLocation )` (`shape_arc.cpp:257` to `:338`) is a candidate point method, not a closed form:

- `radius >= INT_MAX/2.0` builds nine candidates from the two chords and the segment and tests each with the point overload (`:264` to `:292`).
- A sweep over 180 degrees whose chord is shorter than the clearance is treated as a full circle, with an early false when both segment endpoints are strictly inside `radius - clearance` (`:298` to `:310`).
- Otherwise the candidates are the circle/segment intersections, the segment's nearest point to the centre, its nearest points to the two arc endpoints, and the two segment endpoints (`:318` to `:324`); each goes through the point overload (`:328` to `:335`).

The loop short circuits on the first candidate with `*aActual == 0` when `aActual` is requested, and on the first hit otherwise (`:333`). With `aActual` requested and no exact touch, **every** candidate is tested and `*aActual` holds whichever candidate was evaluated last, not the minimum. That is erratum E2.

### 1.7 `NearestPoint`, `NearestPoints`, `IntersectLine`, `Intersect`

`NearestPoint( aP )` (`:476` to `:496`): nearest point on the full circle, snapped to `m_start` or `m_end` when within a squared epsilon of 8 (`:478`, `:483`, `:486`), then `sliceContainsPoint`, then the nearer endpoint. The epsilon is a squared distance, so the snap radius is about 2.8 nm.

Four `NearestPoints` overloads, all returning both points and a squared distance:

| Against | Line | Width handling |
| --- | --- | --- |
| `SHAPE_CIRCLE` | `:499` | arc half width applied to `aPtA`, `aDistSq` zeroed when under `Square(width/2)` (`:543` to `:550`) |
| `SEG` | `:556` | same treatment (`:622` to `:629`) |
| `SHAPE_RECT` | `:635` | **none.** Delegates to `SHAPE_LINE_CHAIN::NearestPoints` on `aRect.Outline()` and returns the raw squared distance (`:640` to `:645`). Erratum E3. |
| `SHAPE_ARC` | `:649` | both arcs' half widths applied by the `adjustForArcWidths` lambda (`:652` to `:663`) |

`sliceContainsPoint( p )` (`:1139` to `:1162`) is the angular containment test used by all the intersection routines: normalise the point's radial angle, walk it by full turns until it is on the right side of the start angle, then compare against the end angle. It works on `EDA_ANGLE`, so it is `double` throughout and has no tolerance of its own.

`IntersectLine( aSeg, aIpsBuffer )` (`:341` to `:362`) treats `aSeg` as infinite, intersects the full circle, and keeps the points inside the sweep. `Intersect( CIRCLE )` (`:365`) and `Intersect( SHAPE_ARC )` (`:386`) are the same shape, the latter requiring containment in both sweeps (`:403`). All three return 0 immediately when any radius reaches `INT_MAX / 2.0` (`:346`, `:367`, `:388`), because `CIRCLE` stores an `int` radius. **None of the three has a caller inside `pcbnew/router/`**; `Intersect( SHAPE_ARC )` is reached only from `SHAPE_LINE_CHAIN::SelfIntersectingWithArcs` (`shape_line_chain.cpp:2273`) and from the QA playground.

### 1.8 Transformations

`Move` (`:1079`), `Rotate` (`:1088`), both `Mirror` overloads (`:1098`, `:1117`) transform all three points and re-run `update_values()`. `Mirror( ref, flipDirection )` negates one coordinate about twice the reference (`:1102` to `:1110`); `Mirror( SEG axis )` uses `SEG::ReflectPoint` (`:1119` to `:1121`). Mirroring flips the handedness, and because the three points move together the representation stays consistent without any reordering.

`Reverse()` (`:1127` to `:1130`) swaps start and end **and leaves the mid alone**, which is correct: the mid stays the mid. `Reversed()` (`:1133`) builds a fresh arc from `(end, mid, start)` and therefore re-runs `CalcArcCenter` on a permuted input, which by section 1.3 can land on a different rounded centre than the original. `Reverse()` does not re-run `update_values()` at all, so the cached centre, radius and bounding box survive the swap unchanged. The two are therefore **not** equivalent. `SHAPE_LINE_CHAIN::Reverse` uses the in place one (`shape_line_chain.cpp:941`); `NODE::AssembleLine` uses `Reversed()` (`pns_node.cpp:1185`). Erratum E4.

### 1.9 Width

`m_width` is the full track width. `SetWidth`/`GetWidth` are the `SHAPE` overrides (`shape_arc.h:206`, `:211`). Three rules hold in the router:

- An arc stored in a `SHAPE_LINE_CHAIN` has width 0. It is forced on insertion (`shape_line_chain.cpp:1625`, `:1697`) and asserted during collision (`:485`, `:879`, `shape_collisions.cpp:686`).
- A `PNS::ARC` item's arc carries the track width (`pns_arc.h:63`, `pns_line_placer.cpp:1691`).
- `BBox( clearance )` inflates by `KiROUND( m_width / 2.0 ) + 1` and then by the clearance (`shape_arc.cpp:462` to `:473`). The `+ 1` is unconditional whenever the width is non zero.

### 1.10 What is exact and what is not

Exact, integer, reproducible on any machine:

- the three points themselves and every transformation of them except `ConstructFromStartEndAngle` / `ConstructFromStartEndCenter`, which rotate through `double`;
- `IsCCW()` (`VECTOR2L` cross product, `shape_arc.h:310`);
- `operator==` (`shape_arc.h:299`);
- `GetChord()` (`shape_arc.h:243`).

Double, and therefore a source of platform and compiler dependent last bits:

- `m_center` (`CalcArcCenter`, with the round number snapping of section 1.3);
- `m_radius`, `GetStartAngle`, `GetEndAngle`, `GetCentralAngle`, `GetLength`;
- `sliceContainsPoint`, hence every `Intersect` and `NearestPoints` result;
- `ConvertToPolyline`'s interior points (`KiROUND` of a trigonometric product, `shape_arc.cpp:1067`);
- `ArcHull`'s offset distances.

Places where one of those doubles reaches an exact comparison the router later makes:

1. `ConstructFromStartEndCenter`'s recomputed centre versus the centre it was given. `splitArc` (`shape_line_chain.cpp:339`, `:342`) and `amendArc` (`:285`) both assume the centre survives; it does not, so splitting an arc twice at the same point does not give the same geometry as splitting it once.
2. `SHAPE_LINE_CHAIN::IsArcStart` / `IsArcEnd` compare `arc.GetP0() == m_points[i]` exactly (`:3278`, `:3299`). Those points came out of `ConvertToPolyline`'s exact endpoint append (`shape_arc.cpp:1055`, `:1070`), so they agree, but any routine that rebuilds an arc through a centre and then leaves the chain's points alone breaks the predicate.
3. `NODE::findRedundantArc` compares the two anchors exactly (`pns_node.cpp:1760`) and ignores the mid point entirely. Erratum E8.
4. `SHAPE_ARC::GetCentralAngle().AsDegrees() > 180.0` gates `ArcHull`'s circle degeneration (`pns_utils.cpp:76`) and `onCollidingArc`'s... no, `startDragArc`'s refusal (`pns_dragger.cpp:161`). A half turn arc lands on whichever side the double happens to fall.
5. `TOPOLOGY::AssembleDiffPair` requires two arc centres to be within 5 nm (`pns_topology.cpp:1109`), a comparison between two independently snapped centres. Erratum E9.

---

## 2. `SHAPE_LINE_CHAIN` with arcs

Note 01 section 6.2 has the data model (`m_shapes` parallel to `m_points`, `SHAPE_IS_PT == -1`, the shared point convention, `ArcIndex`, `IsArcSegment`, the arc order invariant) and a per operation survival table. This section is the behaviour, method by method, and ends with the table of which methods the router actually reaches.

In this section `slc.cpp` = `libs/kimath/src/geometry/shape_line_chain.cpp` and `slc.h` = `libs/kimath/include/geometry/shape_line_chain.h`.

### 2.1 The predicates, with their bounds

All five are at the end of the file and all five are bound checked, unlike `ArcIndex` and `Arc` which are not (`slc.h:856`, `:864`).

- `IsSharedPt( i )` (`slc.cpp:3232`): `i < m_shapes.size()` and both halves set.
- `IsPtOnArc( i )` (`:3240`): `i < m_shapes.size()` and the pair is not `{-1,-1}`. **True for an arc's last point even when the segment leaving that point is straight.**
- `IsArcSegment( s )` (`:3246`): `IsPtOnArc(s) && ArcIndex(s) == m_shapes[s+1].first`, with the wrap to index 0 only when `s+1 == m_shapes.size() && m_closed && IsSharedPt(0)` (`:3255` to `:3261`). On an empty chain `m_shapes.size() - 1` underflows to `SIZE_MAX`, the guard is not taken, and `IsPtOnArc` then returns false, so the underflow is harmless.
- `IsArcStart( i )` (`:3268`): `IsArcSegment(i)` first, so it is bound checked through that; then shared, then `arc.GetP0() == m_points[i]`.
- `IsArcEnd( i )` (`:3282`): `prevIndex = i - 1`, **wrapping to the last point when `i == 0` unconditionally, open chain or not** (`:3286`, `:3287`). Then `IsArcSegment(prevIndex)`, shared, `arc.GetP1() == m_points[i]`.

The difference between `IsPtOnArc` and `IsArcSegment` is the single most frequent source of arc bugs in the router: see erratum E11.

`reversedArcIndex` (`slc.h:941`) has **no caller anywhere in the tree**. Dead.

### 2.2 The mutators

**`Append( const SHAPE_ARC& )`** (`slc.cpp:1612`) forwards to the `aMaxError` overload with 1000 nm. **`Append( const SHAPE_ARC&, int aMaxError )`** (`:1618` to `:1634`) polygonises, and only if the result has more than two points does it record the arc: push a width zeroed copy into `m_arcs` and set every shape entry's `.first` to 0 (`:1622` to `:1629`). A two point approximation silently becomes a plain segment. It then delegates to `Append( SHAPE_LINE_CHAIN )`.

**`Append( const SHAPE_LINE_CHAIN& )`** (`:1546` to `:1609`) offsets the other chain's arc indices by the current arc count (`:1558` to `:1570`), then has a special case worth keeping: when the other chain's first point equals this chain's last point and the other chain's first segment is an arc segment, the arc reference is grafted onto the existing last point rather than duplicating it (`:1579` to `:1586`). It ends with `mergeFirstLastPointIfNeeded()` (`:1606`).

**`Insert( size_t, const VECTOR2I& )`** (`:1637`) splits any arc at that index first (`:1647`, `:1648`) and inserts a plain point.

**`Insert( size_t aVertex, const SHAPE_ARC&, int aMaxError )`** (`:1664` to `:1712`) finds the insertion position in `m_arcs` by scanning backwards from `m_shapes.rbegin()` to `m_shapes.rend() + aVertex` (`:1674` to `:1676`). `rend() + aVertex` is past the reverse end for any non zero `aVertex`, so the loop reads out of bounds. Note 01 section 14.3 already flags this; it is erratum E5 here because it is an arc only path. The method also does not apply `Append`'s "more than two points" rule, so a degenerate arc gets an `m_arcs` entry with a two point run.

**`Remove( int aStart, int aEnd )`** (`:1077` to `:1164`) is the arc aware eraser. It unwraps the chain (`SetClosed(false)` at `:1084`, restored at `:1163`), splits a partially covered arc at each end (`:1100`, `:1101`, `:1106`, `:1107`), pulls each index in by one when it lands on a shared point (`:1103`, `:1104`, `:1109`, `:1110`), collects every arc index fully inside the range into a `std::set<size_t>` and `convertArc`s them (`:1118` to `:1157`), and only then erases the points and shapes. Iterating a `std::set<size_t>` in increasing order while `convertArc` decrements every higher index is the reason the loop at `:1156` is correct only by accident: `convertArc` renumbers, so removing index 3 then index 5 removes what was originally index 6. Erratum E6.

**`Replace( int, int, const VECTOR2I& )`** (`:999`) is `Remove` then `Insert`, so contained arcs are dropped.

**`Replace( int, int, const SHAPE_LINE_CHAIN& )`** (`:1007` to `:1074`) trims coincident endpoints off the incoming chain (`:1029` to `:1046`), `Remove`s the range, offsets the incoming arc indices by the surviving arc count and **appends the incoming arcs at the end of `m_arcs`** (`:1055` to `:1071`). That breaks the chain order invariant that `Reverse` depends on (note 01 section 6.2). Erratum E7.

**`Remove( int )`**, **`RemoveShape( int )`** (`:1380` to `:1409`): `RemoveShape` removes the whole arc containing the index, walking back to the arc start and forward via `NextShape` (`:1398` to `:1406`). `RemoveShape( -1 )` is what the line placer's pullback uses (`pns_line_placer.cpp:244`), which is why a pullback removes a whole arc rather than one approximation segment.

**`SetPoint( int, const VECTOR2I& )`** (`:1362` to `:1377`) destroys every arc touching the point via `convertArc`. Note that it does **not** convert the arc's other points back; `convertArc` clears the references but leaves the points (`:246` to `:272`), so the arc's polyline survives as a plain polyline. This is the intended degradation.

**`Split( const VECTOR2I&, bool aExact )`** (`:1181` to `:1234`): finds the nearest segment within a distance of 2 (`:1184`, `:1194`, `:1198`), and when that segment is an arc segment inserts the point with the arc's index and calls `splitArc( newIndex, true )` to make it a shared point (`:1219` to `:1224`), otherwise plain `Insert`. So splitting on an arc produces two arcs sharing a vertex, and by section 1.2 the two rebuilt arcs do not have the parent's centre.

**`splitArc( aPtIndex, aCoincident )`** (`:292` to `:370`) and **`amendArc( idx, newStart, newEnd )`** (`:275` to `:289`) are the two places that rebuild an arc through `ConstructFromStartEndCenter`. `amendArc` passes `theArc.GetCenter()` and `theArc.IsClockwise()`; both survive the call only up to the recomputation described in section 1.2.

**`Slice( int aStart, int aEnd, int aMaxError = 1000 )`** (`:1418` to `:1543`) is the most intricate. Five `wxCHECK`s that return an empty chain on a bad index (`:1429` to `:1433`). Three behaviours matter for a port:

1. Starting inside an arc: copy the points from `aStartIndex` forward **while they belong to the same arc**, with no `aEndIndex` bound (`:1444`), build a new arc from `m_points[aStartIndex]` to the parent's `GetP1()` about the parent's centre (`:1456`), and advance `aStartIndex` by the number of points copied (`:1463`). A slice whose start and end both fall inside the same arc therefore **extends past `aEndIndex` to the end of that arc**. Erratum E10.
2. Ending inside an arc: the mirror image at `:1486` to `:1513`, correctly bounded by `aEndIndex`.
3. A whole arc that fits: `rv.Append( currentArc, aMaxError )` (`:1519`), which **re-polygonises**. So `chain.Slice(0, n-1)` of an arc bearing chain does not reproduce the original interior points, even though the arc itself is identical.

**`ClearArcs()`** (`:949` to `:953`) is `convertArc` for every arc back to front, which is the only ordering that does not need renumbering. It has no caller in `pcbnew/router/`.

### 2.3 Reverse, Mirror, Rotate, Move

**`Reverse()`** (`:910` to `:946`) reverses points, shapes and arcs, remaps each index to `m_arcs.size() - index - 1`, swaps `first` and `second` on every shared point, and calls `SHAPE_ARC::Reverse()` on each arc (`:940`, `:941`). By section 1.8 that swap leaves the cached centre and bounding box stale in each arc, which nothing downstream notices because `GetCenter()` returns the cached value and the geometry is unchanged.

**`Mirror( ref, flipDirection )`** (`:974` to `:986`) and **`Mirror( SEG axis )`** (`:989` to `:996`) mirror points and arcs but **do not reverse the chain**, so the chain's winding flips. For a closed chain that inverts the orientation that `HullIntersection` and the walkaround depend on (note 01 section 12.4). Neither updates `m_bbox`.

**`Rotate( angle, center )`** (`:495` to `:502`) rotates points and arcs, no bbox update. **No caller in `pcbnew/router/`**; note 06 section 9.6 already established that rotation is arc only in this router.

**`Move`** is inline (`slc.h:776`) and translates points, arcs and the cached bbox.

### 2.4 Simplify, Simplify2, RemoveDuplicatePoints

**`Simplify( int aTolerance = 0 )`** (`:2782` to `:2874`, declared `slc.h:358`). The run extension loop refuses to drop any *intermediate* candidate whose `m_shapes[test].first != SHAPE_IS_PT` (`:2816` to `:2820`), so no arc point is ever removed and `m_arcs` never needs renumbering. The comment at `:2813` to `:2815` is explicit that arc endpoints are allowed to be the run's start or end, so a straight run abutting an arc still collapses. `m_arcs` is untouched by the method.

**`Simplify2( bool aRemoveColinear = true )`** (`:2906` to `:3005`). Stage 1 merges duplicate points when the two shape entries agree or one of them is plain, keeping the non plain one (`:2934` to `:2951`). Stage 2's colinear removal checks that `shapes_unique[i]` and `shapes_unique[i + 1]` are both plain before entering the run (`:2968`, `:2969`) but the run extension at `:2971` to `:2974` does **not** re-check the shape of `pts_unique[n + 1]` as `n` advances. A shallow arc whose consecutive approximation points are within 1 nm of the chord can therefore lose interior points while its `m_arcs` entry survives, leaving an arc with too few points. Erratum E12.

**`RemoveDuplicatePoints()`** (`:2720` to `:2778`) is stage 1 of `Simplify2` on its own, with the same shape merging rule. `NODE::AssembleLine` calls it (`pns_node.cpp:1204`) with the comment "do NOT remove colinear segments here".

Neither `Simplify` nor `Simplify2` ever erases an entry from `m_arcs`. An arc that loses every reference stays in the vector, and `Collide` and `router_preview_item.cpp:278` both iterate `ArcCount()` directly, so an orphaned arc still collides and still draws. Erratum E13.

### 2.5 Queries

**`Length()`** (`:956` to `:971`): sums the straight segments, skipping arc segments, then adds each arc's `GetLength()`. A `double` per arc into a `long long`, truncated.

**`PointAlong( int aPathLength )`** (`:2671` to `:2693`): walks the **polyline** segments only, arc aware in no way. So a point "40 percent along" a chain containing an arc is 40 percent along the polyline, which is shorter than the arc by the sagitta error. Its only router caller is `MULTI_DRAGGER::clipToOtherLine` (note 06 section 9.6).

**`NearestPoint( aP, bool aAllowInternalShapePoints )`** (`:2401` to `:2456`). The flag defaults to true (`slc.h`, checked at the call sites below). Finding the nearest **segment** is arc unaware (`:2413` to `:2422`, plain `CSegment(i).Distance`). When the flag is false and the winning segment is an arc segment, the result is snapped: advance to the nearer of the segment's two endpoints (`:2429` to `:2433`), and if that is an arc start or end return it, otherwise return the nearer of the containing arc's two true endpoints (`:2436` to `:2450`). The `nearest++` at `:2433` can reach `PointCount()`; `IsArcStart` and `IsArcEnd` are bound checked so they answer false, and the `else` branch then calls `ArcIndex( PointCount() )`, which is **not** bound checked (`slc.h:856`). Reachable only on a closed chain whose last segment is an arc segment. Erratum E14.

Router call sites: `pns_line_placer.cpp:830` (`hull.NearestPoint( aP )`, default flag) and `pns_optimizer.cpp` via `SHAPE_LINE_CHAIN::NearestPoint` inside the smart pad pass. Hulls contain no arcs, so the snapping branch is unreachable from the router at this commit.

**`Intersect( const SEG&, INTERSECTIONS& )`** (`:1731` to `:1770`) and **`Intersect( const SHAPE_LINE_CHAIN&, ... )`** (`:1802` to `:1949`) are **completely arc unaware**. Both iterate `SegmentCount()` and build `SEG( m_points[s], m_points[s+1] )`. So `PNS::HullIntersection`, and therefore the entire walkaround, sees an arc as its 1000 nm polyline. This is the single most important fact in section 2 for a port: the walkaround needs no arc geometry at all.

**`Intersects( const SEG& )`** (`:1773`) and **`Intersects( const SHAPE_LINE_CHAIN& )`** (`:2610`) likewise.

**`SelfIntersecting()`** (`:2135` to `:2218`) is arc unaware, polyline only. It is the one the router uses, at three sites: `pns_line.cpp:360` inside `LINE::Walkaround`, `pns_shove.cpp:474` inside `shoveLineToHullSet`, and `pns_diff_pair.cpp:254`.

**`SelfIntersectingWithArcs()`** (`:2234` to `:2398`) is the arc aware one: it builds a shape cache by walking `NextShape` (`:2337` to `:2344`), each entry carrying the shape's first point index, its arc index or -1, and a bounding box taken from the `SEG` or the `SHAPE_ARC`, then runs a quadratic pass with three lambdas, `collideSegSeg` (`:2293`), `collideArcSeg` (`:2242`) and `collideArcArc` (`:2268`). The arc lambdas skip shared endpoints within a squared tolerance of 2.0 (`:2237` to `:2240`, `:2253`, `:2256`, `:2278`, `:2281`). `ArcIndex( si )` at `:2339` is unchecked, but `NextShape` never returns an out of range index.

**The router never calls it.** Its one caller in the whole tree is the STEP exporter (`pcbnew/exporters/step/step_pcb_model.cpp:454`). So every self intersection test the router makes on an arc bearing line sees the 1000 nm polyline, and two arcs that cross without their approximations crossing are accepted. Erratum E16.

**`Collide( const VECTOR2I&, ... )`** (`:426` to `:492`) and **`Collide( const SEG&, ... )`** (`:815` to `:907`) are the two that use both representations. Both run the straight segments first, `continue`ing on `IsArcSegment( i )` (`:447`, `:836`), return early if the polyline alone already collides (`:468`, `:858`), and only then loop `ArcCount()` and delegate to `SHAPE_ARC::Collide` (`:480` to `:489`, `:873` to `:893`). The `SEG` overload carries the polyline's best distance into the arc phase as `closest_dist = sqrt( closest_dist_sq )` (`:870`) so the arc phase can only improve on it. Both assert the stored arc's width is zero (`:485`, `:879`).

Because the polyline phase runs first and returns early, a chain whose polyline is inside the clearance reports the **polyline** distance as `*aActual`, which is up to 1000 nm larger than the true arc distance. This is why the shove adds 5000 nm of clearance for arc segments (section 5.2).

**`CompareGeometry( other, aCyclicalCompare, aEpsilon )`** (`:2535` to `:2607`) copies both chains, `Simplify()`s both, and compares `m_points` only. Arcs are not compared at all, so two chains with the same points and opposite bulges compare equal. No router caller.

### 2.6 `Format` and `Parse` are not inverses and `Format` drops arcs

`Format( bool aCplusPlus )` (`:2506` to `:2532`) emits `SHAPE_LINE_CHAIN( { VECTOR2I( x, y), ... }, closed );`. The arc half is commented out under `/* fixme: arcs` (`:2525` to `:2531`).

`Parse( std::stringstream& )` (`:2623` to `:2668`) reads a completely different format: a point count, the closed flag, an arc count, then `x y shapeIndex` per point, then `cx cy px py angleDegrees` per arc, building each arc through the `(center, start, angle, width)` constructor with the width defaulted to 0 (`:2664`). It clears `m_points` but neither `m_shapes` nor `m_arcs` (`:2628`), and it writes every shape entry as `{ ind, SHAPE_IS_PT }` (`:2650`), so shared points never come back. Erratum E15.

For the port this matters because the crate's event log is the analogue of KiCad's router log, and KiCad's router log does **not** go through `Format`: `LOGGER::formatShapeAsJSON` writes an arc as `{ type, width, start, end, mid }` (`pns_logger.cpp:245` to `:255`). That is the format to copy.

### 2.7 Which of these the router actually reaches

| Chain method | Reached from | Notes |
| --- | --- | --- |
| `Append( SHAPE_ARC )` | `NODE::AssembleLine` (`pns_node.cpp:1185`), `Slice` (`slc.cpp:1519`), `MEANDER_SHAPE::makeMiterShape` (`pns_meander.cpp:497`), `MakeArc` (`:921`, `:922`), `DIRECTION_45::BuildInitialTrace` (`direction_45.cpp:152`, `:167`, `:188`, `:216`, `:258`, `:273`, `:283`, `:295`, `:304`) | the only way an arc enters a chain in the router |
| `ArcIndex`, `Arc`, `CArcs`, `ArcCount` | placer `:200`, `:207`, `:231`, `:355`, `:365`, `:380`, `:1671`, `:1690`; shove via `IsArcSegment`; optimizer `:660`; line `:916`, `:926`, `:938`; dragger `:178`; meander placer `:251`, `:252`; dp meander placer `:355`; node `:689`, `:691`; preview `:278`, `:280` | all unchecked accessors |
| `IsPtOnArc` | placer `:197`, `:204`, `:352`, `:362`, `:1673`; line `:841`, `:863`, `:865`, `:1247`, `:1257`; optimizer `:642`; dragger `:139` | see erratum E11 |
| `IsArcSegment` | placer `:222`, `:377`; shove `:589`; optimizer `:745`, `:814`, `:868`, `:869`; line `:695`, `:869`; diff pair `:842`, `:847`; meander placer `:249`; dp meander placer `:353` | |
| `Remove`, `RemoveShape` | placer `:244` (`RemoveShape(-1)`), `:382`; line `:705`; optimizer `:644` | arc aware paths all reachable |
| `Slice` | line `:285`, `:288`, `:291`, `:845`, `:847`; dragger and multi dragger via `LINE::ClipVertexRange` | erratum E10 reachable through `restoreUntouchedArcs` |
| `Split` | optimizer `:834`; line `:704` | |
| `Simplify` | placer `:373`, `:387`, `:388`; line `:851`, `:881`; `BuildInitialTrace` `:310` | |
| `Simplify2` | walkaround `:177`; line `:660`; shove `:2378` | erratum E12 reachable |
| `Length` | meander length arithmetic, `onCollidingArc` (`pns_shove.cpp:703`, `:704`) | the only arc aware length in the router |
| `Intersect` | `PNS::HullIntersection` (`pns_utils.cpp:403`), `LINE::Walkaround` | polyline only |
| `SelfIntersecting` | line `:360`, shove `:474`, diff pair `:254` | polyline only |
| `Collide` | `ITEM::collideSimple` through the dispatcher | both phases reachable |
| `NearestPoint` | placer `:830` on a hull | snapping branch unreachable, hulls have no arcs |
| `PointAlong` | `MULTI_DRAGGER::clipToOtherLine` | polyline only |
| `Mirror`, `Rotate`, `ClearArcs`, `CompareGeometry`, `Format`, `Parse`, `SelfIntersectingWithArcs` | **no router caller** | do not port for the router's sake |

---

## 3. `DIRECTION_45` and the two rounded corner modes

`CORNER_MODE` is `MITERED_45 = 0`, `ROUNDED_45 = 1`, `MITERED_90 = 2`, `ROUNDED_90 = 3` (`libs/kimath/include/geometry/direction45.h:66` to `:72`). Note 01 section 8.3 has the shared skeleton of `BuildInitialTrace` (`libs/kimath/src/geometry/direction_45.cpp:24` to `:312`): the single segment shortcut at `:46`, the `mp0` / `mp1` / `tangentLength` setup at `:56` to `:81`, and the trailing `pl.Simplify()` at `:310`. This section is the two rounded branches.

### 3.1 `ROUNDED_45`, `direction_45.cpp:105` to `:222`

```
if w == h: append aP0, aP1; break                                       :123 to :128
diag2      = tangentLength >= 0 ? |mp1|^2 : |mp0|^2                     :130
diagLength = sqrt( 2*diag2 - 2*diag2*cos( 3*pi/4 ) )                    :131
arcRadius  = KiROUND( diagLength / ( 2 * cos( 67.5 deg ) ) )            :132
```

Four sub-cases on `startDiagonal` and `sign( tangentLength )`:

| case | arc endpoints | angle | result chain | line |
| --- | --- | --- | --- | --- |
| diagonal, `tangentLength >= 0` | `aP0` to `aP1 - mp0.Resize( tangentLength )` | `+45 * rotationSign` | arc, then `aP1` | `:143` to `:155` |
| diagonal, `tangentLength < 0` | `aP0 + mp1.Resize( |tangentLength| )` to `aP1` | `+45 * rotationSign` | `aP0`, then arc | `:156` to `:168` |
| straight, `tangentLength >= 0` | `aP0 + mp0.Resize( tangentLength )` to `aP1` | `-45 * rotationSign` | `aP0`, then arc | `:177` to `:189` |
| straight, `tangentLength < 0` | built **from a centre**: `SHAPE_ARC( aP0 + centerDir.Resize( arcRadius ), aP0, -45 * rotationSign )` | | arc, then `aP1` | `:190` to `:219` |

`rotationSign = ( w > h ) ? -sw*sh : sw*sh`, computed identically in both halves (`:141` and `:172`). `centerDir` is `mp0` rotated by `90 * rotationSign` in `VECTOR2D` (`:173` to `:175`).

Three things a port must keep:

1. **`arcRadius` is used in exactly one of the four sub-cases**, the fourth. The other three derive the arc from two endpoints and an angle and never touch it. The `diagLength` computation at `:131` is therefore dead work three times out of four, and `diagLength` itself is only ever consumed by `arcRadius`. Erratum E17.
2. **The `MIN_PRECISION_IU` re-snap** (`:197` to `:211`). Constructing from a centre loses endpoint precision, so if the constructed end is within `SHAPE_ARC::MIN_PRECISION_IU` (4 nm, `libs/kimath/include/geometry/shape.h:129`) of `aP1` in one axis, the arc is rebuilt through `ConstructFromStartEndAngle( ca.GetP0(), fixedEnd, +45 * rotationSign )` with that axis forced to `aP1`'s value. Note the sign: the rebuild uses `+ANGLE_45 * rotationSign` where the original used `-ANGLE_45 * rotationSign` (`:194` versus `:205`, `:210`). Since `ConstructFromStartEndAngle` places the centre from the angle's sign, this **flips the bulge** relative to the arc it is correcting. Erratum E18.
3. **The degenerate guard**, `if( arc.GetP0() == arc.GetP1() ) append the point instead` (`:149`, `:164`, `:185`, `:213`). A zero length arc would otherwise enter the chain with a two point approximation and be silently demoted by `Append`'s rule (section 2.2).

The `if( w == h )` block at `:123` to `:128` is **unreachable**: the shortcut at `:46` returns for `!is90mode && h == w`, and `ROUNDED_45` is not a 90 mode. Erratum E19.

### 3.2 `ROUNDED_90`, `direction_45.cpp:239` to `:307`

Radius is the shorter of the two extents. Five cases, all built with `ConstructFromStartEndCenter`:

| case | arc | rest | line |
| --- | --- | --- | --- |
| `w == h` | `aP0` to `aP1` about `aP1 - mp0`, clockwise `(sh == sw) != startDiagonal` | **returns immediately**, skipping `pl.Simplify()` | `:255` to `:260` |
| diagonal, `h > w` | `aP0` to `(aP1.x, aP0.y + w*sh)` about `(aP0.x, aP0.y + w*sh)`, clockwise `sh != sw` | then `aP1` | `:267` to `:275` |
| diagonal, `h <= w` | `aP0` to `(aP0.x + h*sw, aP1.y)` about `(aP0.x + h*sw, aP0.y)`, clockwise `sh == sw` | then `aP1` | `:276` to `:284` |
| straight, `w > h` | `(aP1.x - h*sw, aP0.y)` to `aP1` about `(aP1.x - h*sw, aP1.y)`, clockwise `sh != sw` | `aP0` first | `:288` to `:296` |
| straight, `w <= h` | `(aP0.x, aP1.y - w*sh)` to `aP1` about `(aP1.x, aP1.y - w*sh)`, clockwise `sh == sw` | `aP0` first | `:297` to `:305` |

The `w == h` early return at `:259` is the only path out of `BuildInitialTrace` that does not run `Simplify()`. Every centre here is an exact integer point, so `ROUNDED_90` is the better behaved of the two branches; the only `double` in it is `ConstructFromStartEndCenter`'s own mid point rotation and the centre recomputation of section 1.2.

`VECTOR2I arcEnd;` and `VECTOR2I arcCenter;` are declared at `:262`, `:263` inside the `case` label with no enclosing braces, legal only because `ROUNDED_90` is the last case.

### 3.3 `DIRECTION_45( const SHAPE_ARC&, bool a90 )`

`direction45.h:116` to `:122`: takes `aArc.GetP1() - aArc.GetP0()`, negates y, and classifies. So the direction of an arc is the direction of its **chord**, not of either tangent. For a 45 degree arc that is the bisector of the two tangents and lands on a diagonal octant when the tangents are axis aligned and vice versa. This is what the placer reads at `pns_line_placer.cpp:200`, `:207`, `:232`, `:355`, `:365`, `:380`.

### 3.4 Who selects the mode

`ROUTING_SETTINGS::m_cornerMode` (`pcbnew/router/pns_routing_settings.h:181`), default `MITERED_45` (`pns_routing_settings.cpp:53`), persisted through a JSON param whose range is `MITERED_45` to `ROUNDED_90` (`pns_routing_settings.cpp:104`).

`ROUTER::ToggleCornerMode` cycles `MITERED_45 -> ROUNDED_45 -> MITERED_90 -> ROUNDED_90 -> MITERED_45` (`pns_router.cpp:1075` to `:1078`). `router_tool.cpp` binds the same cycle to a keystroke (`:955` to `:960`) plus two direct actions, "Track Corner Mode Arc 45" (`:317`, `:318`, setting `ROUNDED_45` at `:972`) and "Track Corner Mode Arc 90" (`:324`, `:325`, setting `ROUNDED_90` at `:982`), with checked state predicates at `:713` and `:719` and status text at `:3434`, `:3436`.

Six places read the mode and branch on "is it a 90 mode" or "is it a 45 mode":

| Site | Branch | Effect |
| --- | --- | --- |
| `pns_line_placer.cpp:763`, `:983` | 45 modes only | enable the optimizer's `SMART_PADS` |
| `pns_line_placer.cpp:827` | 90 modes | snap to the hull's **bounding box** nearest point instead of the hull's |
| `pns_line_placer.cpp:2048` | ortho mode | force `MITERED_45`, so rounded corners never appear in ortho |
| `pns_node.cpp:326` | 90 modes | replace every cached hull with its bounding box in `NearestObstacle` |
| `pns_walkaround.cpp:162` | 90 modes | same replacement in `processCluster` |
| `pns_optimizer.cpp:859` | 90 modes | pass `is90mode` into the two `DIRECTION_45` constructors in `mergeStep` |
| `pns_shove.cpp:2080` | 45 modes only | enable `SMART_PADS` in the post shove optimizer |

Note what is **not** there: nothing in the shove, the walkaround or the optimizer treats `ROUNDED_45` differently from `MITERED_45`, or `ROUNDED_90` differently from `MITERED_90`. The rounded modes change exactly one thing, the shape `BuildInitialTrace` returns, and everything downstream copes with the arc that shape contains or refuses to touch it.

---

## 4. `PNS::ARC`, the item

### 4.1 The class

`pcbnew/router/pns_arc.h:37`, deriving from `LINKED_ITEM`. One data member, `SHAPE_ARC m_arc` (`:119`).

| Member | Line | Notes |
| --- | --- | --- |
| `ARC()` | `:40` | default, kind `ARC_T` |
| `ARC( SHAPE_ARC, NET_HANDLE )` | `:44` | the sync constructor |
| `ARC( const ARC& parent, SHAPE_ARC )` | `:51` | copies net, layers, marker, rank; **not** the parent pointer or the source item |
| `ARC( const LINE& parent, SHAPE_ARC )` | `:61` | rebuilds the arc from `(P0, mid, P1)` with the **line's** width, copies net, layers, marker, rank |
| `Shape( int aLayer )` | `:78` | returns `&m_arc`, layer ignored |
| `SetWidth` / `Width` | `:83`, `:88` | forward to the arc |
| `CLine()` | `:93` | `SHAPE_LINE_CHAIN( m_arc )`, polygonised at 1000 nm. **No caller in the tree.** Erratum E20. |
| `Hull` | `:98` | `ArcHull( m_arc, clearance, walkaroundThickness )` (`pns_arc.cpp:28` to `:31`), layer ignored |
| `Anchor( n )` | `:100` | `GetP0()` for 0, `GetP1()` for anything else |
| `AnchorCount()` | `:108` | 2 |
| `ChangedArea( const ARC* )` | `:113` | union of the two bounding boxes (`pns_arc.cpp:51` to `:56`) |
| `Arc()` / `CArc()` | `:115`, `:116` | mutable and const access |
| `Clone()` | `pns_arc.cpp:34` | copies parent, source item, movable, layers, marker, rank, routable. **Does not copy the width**, which is inside `m_arc` and therefore carried by the `ARC( m_arc, m_net )` delegation at `:36`. |

`ARC_T = 16` in the kind bitmask (`pns_item.h:108`) and is part of `LINKED_ITEM_MASK_T` (`:113`). `ITEM::KindStr()` returns `"arc"` (`pns_item.cpp:319`).

### 4.2 The hull

`ArcHull( arc, clearance, walkaroundThickness )` (`pns_utils.cpp:71` to `:154`). Note 01 section 12.2 has the shape. Four things it does not say:

- `int r = aArc.GetRadius();` at `:78` truncates a `double` into an `int` and then builds `OctagonalHull( center - (r,r), (2r,2r), cl, 2*(1 - 1/sqrt2)*(r + cl) )`, where the chamfer argument is a `double` implicitly converted to the `int` parameter. For a radius near `INT_MAX` this overflows; nothing guards it.
- `int x = (int)( 2.0 / ( 1.0 + M_SQRT2 ) * d ) / 2;` at `:86` truncates, then integer divides. `SegmentHull` computes the same quantity as `KiROUND( x / 2.0 )` (`:190`). The two differ by up to 1 nm, which matters because the shove's termination depends on hulls and collisions agreeing to the nanometre (note 01 section 14.6).
- The mitre loop dereferences `sa_out.IntersectLines( sb_out )` and `sa_in.IntersectLines( sb_in )` without checking the optionals (`:131`, `:132`). Two exactly collinear consecutive approximation segments return `nullopt` and the dereference is undefined. `ConvertToPolyline` at `ARC_LOW_DEF` produces at least four points for any non degenerate arc (`GetArcToSegmentCount` floors at 2, then the doubling at `shape_arc.cpp:1053`), and consecutive points on a circle are never collinear, so it is not reachable today. Erratum E21.
- The reversal test at `:150` uses `line.Segment( 0 ).A`, which is `m_start` exactly.

### 4.3 `NODE`

**`Add( std::unique_ptr<ARC>, bool aAllowRedundant )`** (`pns_node.cpp:776` to `:788`): refuses when `findRedundantArc` finds one, otherwise `addArc`. **`addArc`** (`:765` to `:773`) sets the owner, links a joint at each anchor and adds to the index. Identical in shape to `addSegment` (`:736`), except that `Add( SEGMENT )` also refuses a zero length segment (`:749` to `:754`) and `Add( ARC )` has no analogous degenerate check.

**`Add( LINE&, bool aAllowRedundant )`** (`:683` to `:733`) is where a placed line becomes items. It runs **two** loops: first every arc in `m_arcs` order (`:689` to `:705`), then every non arc segment in chain order (`:707` to `:732`). So `LINE::m_links` ends up in arcs first order, not chain order, for any line that contains both. `LINE::ClipVertexRange` (`pns_line.cpp:1469` to `:1500`) walks shapes with `NextShape` and counts link indices in lockstep, which is only correct for chain order. A line built by `NODE::Add` and then clipped therefore maps the wrong links. `NODE::AssembleLine` links in chain order (`pns_node.cpp:1188`), which is the path the shove and the dragger use, so the hazard is latent rather than live. Erratum E22.

**`Remove( ARC* )`** (`:991` to `:994`) is `removeArcIndex` (`:863`) plus the common tail; `Remove( ITEM* )` dispatches on `ARC_T` at `:1002`; `Remove( LINE& )` dispatches on each link at `:1063`, `:1064`.

**`findRedundantArc( A, B, layerRange, net )`** (`:1742` to `:1768`) walks the joint at `A` and returns any `ARC_T` link whose two anchors are `A` and `B` in either order and whose layer start matches. **It never looks at the mid point**, so an arc bulging the other way between the same two endpoints is "redundant" and gets silently reused. Erratum E8.

**`NODE::AssembleLine`** (`:1132` to `:1213`) and its helper `followLine` (`:1074` to `:1129`). `followLine` records, per position, the joint position, the linked item, and an `aArcReversed` flag set when the scan reaches an arc from the anchor that is not the one the scan direction expects (`:1096` to `:1103`). The assembly loop then:

```cpp
if( !li || li->Kind() != ITEM::ARC_T )      // :1175
    line.Append( p );                        // :1176
if( li && prev_seg != li )                   // :1178
{
    if( li->Kind() == ITEM::ARC_T )          // :1180
        line.Append( arcReversed[i] ? sa->Reversed() : *sa );   // :1185
    pl.Link( li );                           // :1188
    ...
}
```

So an arc contributes no plain corner point of its own; its polyline comes entirely from `Append( SHAPE_ARC )`. `Reversed()` rather than `Reverse()` means the reversed copy re-runs `CalcArcCenter` on the permuted points (section 1.8), so assembling the same physical line in the two directions can produce two arcs with different cached centres and radii. Erratum E4.

`*aOriginSegmentIndex = line.PointCount() - 1` (`:1195`) is a **point** index assigned to a variable every caller treats as a segment index, clamped afterwards to `pl.SegmentCount() - 1` (`:1207`, `:1208`) with the comment "TODO: maintain actual segment index under simplification system" (`:1206`). For a line whose arc contributes a dozen approximation points, the origin index after an arc is off by that many. It is the value `DRAGGER::m_draggedSegmentIndex` is built from (`pns_dragger.cpp:244`, `:248`).

`RemoveDuplicatePoints()` at `:1204` is the only cleanup; colinear removal is deliberately skipped.

**`NODE::Dump( bool )`** (`:1473` to the end of the function) is entirely inside `#if 0` and uses an API that no longer exists (`GetKind()`, `GetPos()`, `GetLinkList()`, `AssembleLine` returning a pointer). It is dead code that would not compile. Do not port it. Erratum E23.

### 4.4 `INDEX` and `JOINT`

`INDEX` has no arc specific code; an `ARC` is indexed by `BBox()` like every other item, and `SHAPE_ARC::BBox` is quadrant exact (section 1.1) so the index is tight.

`JOINT` treats `ARC_T` exactly like `SEGMENT_T` in all five of its predicates and accessors: `IsLineCorner` (`pns_joint.h:103`, `:114`), `IsNonFanoutVia` (`:129`), `IsStitchingVia`/`NextSegment` (`:162`, `:200`), `LinkList` filtering (`:248`). The mask is always written `SEGMENT_T | ARC_T`. There is no joint level notion of tangency: a segment meeting an arc at a right angle is a perfectly ordinary line corner.

### 4.5 `LINE`

`LINE` stores the chain, so arcs live inside it. `ArcCount()` forwards (`pns_line.h:146`).

`restoreUntouchedArcs( aPath, aOriginal )` (`pns_line.cpp:255` to `:294`) is the glue that puts arcs back after a walkaround. It finds the longest common prefix and suffix **by exact point equality** (`:265` to `:277`), then rebuilds the path as `original.Slice( 0, head-1 )` plus `path.Slice( max(head-1,0), pathCount-tail-1 )` plus `original.Slice( origCount-tail, origCount-1 )`. Its only caller is the tail of `LINE::Walkaround`'s inner routine (`:661`), immediately after `out.Simplify2( false )` (`:660`). Two consequences:

- Because it works on exact point equality, any arc whose approximation the walkaround touched at all is lost, and the polyline stands in for it.
- Because it uses `Slice`, it inherits erratum E10: a prefix or suffix that ends inside an arc extends to that arc's far end.

`LINE::ClipToNearestObstacle` (`pns_line.cpp:680` to `:716`): when the nearest segment to the collision point is an arc segment the whole line is cleared with the comment "Don't clip at arcs, start again" (`:695` to `:699`).

`dragCorner45` inserts a duplicate point before slicing when the next point is on an arc (`:841`, `:842`); `dragCornerFree` inserts a duplicate on whichever side of an arc endpoint is free and asserts when asked to drag the middle of an arc (`:863` to `:878`); `dragSegment45` does the same at both ends (`:1247` to `:1260`). `DragArc` is note 06 section 3.5.

### 4.6 `TOPOLOGY`

Fourteen of `pns_topology.cpp`'s arc mentions are the `SEGMENT_T | ARC_T` mask in a filter (`:88`, `:301`, `:418`, `:432`, `:488`, `:496`, `:544`, `:678`, `:809`, `:830`, `:838`, `:1052`, `:1061`, `:1229`). `AssembleTrivialPath`, `AssembleTuningPath`, `AssembleCluster` and `LeadingRatLine` need nothing arc specific: they work on joints and links.

The two that are genuinely arc aware are both in `AssembleDiffPair`:

- `findNItem`'s arc branch (`:1098` to `:1113`), guarded by `n_item->Kind() == p_item->Kind()` at `:1077`. It requires the two arc centres to be within `DP_PARALLELITY_THRESHOLD` (5, `pns_topology.h:106`) and then takes `dist_sq = Square( radiusP - radiusN )` with both radii `double`. Five nanometres of centre coincidence between two independently computed and independently snapped centres (section 1.3) is a far stricter test than the segment branch's `ApproxParallel( ..., 5 )`. Erratum E9.
- The gap computation at `:1172` to `:1177`: `gap = |radiusRef - radiusCoupled| - width`, again on two `double` radii.

### 4.7 The one thing that is missing

There is no `ARC` analogue of `SEGMENT( const LINE&, const SEG& )`, the constructor the shove uses to build a per segment hull (`pns_shove.cpp:586`). The shove builds a `SEGMENT` for every chain segment including the ones that approximate an arc, and compensates with extra clearance (section 5.2). So an arc's hull, `ArcHull`, is used only when the arc is an obstacle item in the node, never when it is part of the line being shoved.

---

## 5. Every arc branch in the algorithms

Method: `grep -n -i "SHAPE_ARC\|IsArcSegment\|ArcIndex\|ArcHull\|CArcs\|IsPtOnArc\|ArcCount\|PNS::ARC\|ARC_T\|ROUNDED\|arc" pcbnew/router/*.cpp pcbnew/router/*.h`, minus the hits on "search" and "Search". Every remaining hit is accounted for below, either individually or by a class ("kind mask", "include").

### 5.1 `pns_line_placer.cpp`

Forty three hits. One include (`:29`). Six are corner mode branches already covered in section 3.4 (`:763`, `:827`, `:983`, `:2047` plus the two `SMART_PADS` guards). Two are kind masks (`:1896`, `:1919`). The rest:

**Posture from an arc, `:197` to `:207` and `:352` to `:365`.** `reduceTail` and `mergeHead` both take the head's leading direction and the tail's trailing direction, and both use the **arc chord** when the relevant point is on an arc:

```cpp
if( !head.IsPtOnArc( 0 ) )          first_head = DIRECTION_45( head.CSegment( 0 ) );
else                                 first_head = DIRECTION_45( head.CArcs()[head.ArcIndex(0)] );
int lastSegIdx = tail.PointCount() - 2;
if( !tail.IsPtOnArc( lastSegIdx ) )  last_tail = DIRECTION_45( tail.CSegment( lastSegIdx ) );
else                                 last_tail = DIRECTION_45( tail.CArcs()[tail.ArcIndex(lastSegIdx)] );
```

The test is `IsPtOnArc`, which is true for an arc's **last** point even when the segment leaving it is straight (section 2.1). In that case the code reports the arc's chord direction for a straight segment that is not part of the arc. Six lines later the same file uses `IsArcSegment` for the same question (`:222`, `:377`), which is the correct predicate. Erratum E11.

`:222` to `:233` is the pullback: on a right or acute angle, take the new direction from the last tail shape (segment or arc chord), then `tail.RemoveShape( -1 )` at `:244`, which removes a whole arc rather than one approximation segment. `:377` to `:380` is the same choice after `mergeHead` appends.

**`SplitAdjacentArcs( NODE*, ITEM*, VECTOR2I )`, `:1315` to `:1344`.** The arc twin of `SplitAdjacentSegments`. Refuses when the point already carries a joint with any link (`:1323` to `:1326`), then clones the arc twice and rebuilds each half with `ConstructFromStartEndCenter( ..., o_arc.GetCenter(), o_arc.IsClockwise(), o_arc.GetWidth() )` (`:1333` to `:1337`), removes the old and adds both halves with `aAllowRedundant = true`. `aP` is **not** checked to lie on the arc; a point off the arc produces two arcs that do not meet it. Its only caller is `ROUTER::BreakSegmentOrArc` (`pns_router.cpp:1117`), which is the host's "break track here" gesture (`router_tool.cpp` `breakTrack`, note 06 section 7.5).

**`FixRoute`, `:1648` to `:1702`.** Three arc facts:

1. `if( !fixAll && l.ArcCount() ) fixAll = true;` (`:1650`, `:1651`) with the comment "Rollback doesn't work properly if fix-all isn't enabled and we are placing arcs". So placing in a rounded corner mode silently commits the whole trace on every click.
2. `SEG lastDirSeg = ( !fixAll && l.SegmentCount() > 1 ) ? l.CSegment( -2 ) : l.CSegment( -1 );` (`:1654`) with the comment "lastDirSeg will be calculated incorrectly if we end on an arc" (`:1653`). Because of point 1, `fixAll` is always true when arcs are present, so the `-1` branch is taken and `d_last` is the direction of the last **approximation chord**, not the arc's chord.
3. The item emission loop (`:1669` to `:1702`):

```cpp
ssize_t arcIndex = l.ArcIndex( i );
if( arcIndex < 0 || ( lastArc >= 0 && i == lastV - 1 && !l.IsPtOnArc( lastV ) ) )
    ... emit SEGMENT( pl.CSegment( i ) ) ...
else
{
    if( arcIndex == lastArc ) continue;
    ... emit ARC( l.Arc( arcIndex ) ) ...
    lastArc = arcIndex;
}
```

`ArcIndex( i )` with `i` a segment index is read as a point index, which is in range because `lastV <= SegmentCount()`. The `continue` at `:1688` is what stops an arc being emitted once per approximation segment. But it also swallows a **straight segment whose first point is an arc's last point**: that point has `m_shapes = { N, -1 }`, so `ArcIndex` returns `N >= 0`, the else branch is taken, `arcIndex == lastArc` holds, and the segment is never emitted. The rescue condition only covers the case where that segment is the last one in the trace. So a chain shaped arc, straight, arc loses the middle straight segment on commit. Erratum E24.

**`simplifyNewLine`, `:1894` to `:1950`.** The assertion at `:1896` accepts `SEGMENT_T | ARC_T`, and `lastItem` can be an `ARC` (`:1695`). The `processJoint` lambda then does `static_cast<SEGMENT*>( aItem )->Seg()` (`:1912`), `static_cast<const SEGMENT*>( neighbor )->Width()` (`:1925`) and `static_cast<const SEGMENT*>( neighbor )->Seg()` (`:1931`) on anything passing the `SEGMENT_T | ARC_T` filter at `:1919`. `ARC` and `SEGMENT` are sibling classes under `LINKED_ITEM` (`pns_arc.h:37`, `pns_segment.h:38`), not related by inheritance, so those casts read a `SHAPE_SEGMENT` out of the bytes of a `SHAPE_ARC`. Erratum E25.

**`buildInitialLine`, `:2038` to `:2094`.** Reads the corner mode, forces `MITERED_45` in ortho mode (`:2048`, `:2049`), and passes the mode to `BuildInitialTrace` at `:2082`, `:2084` and `:2127`. The ortho post-processing at `:2087` to `:2093` uses `SetPoint( 1, ... )`, which would destroy an arc, but ortho has forced a mitered mode so no arc exists.

### 5.2 `pns_shove.cpp`

Twenty nine hits after the include. Six are kind masks (`:1083`, `:1317`, `:1442`, `:1658`, and the two in `:1542` to `:1545`, which takes `arc->Width()` into a running maximum). One is a corner mode branch (`:2080`, section 3.4). One is a commented out line with `// fixme arcs` (`:2373`). The rest are three genuine arc paths:

**The clearance bump, `:588` to `:594`.** Inside the per segment hull loop of `ShoveObstacleLine`:

```cpp
for( int i = 0; i < currentLineSegmentCount; i++ )
{
    SEGMENT seg( aCurLine, aCurLine.CSegment( i ) );
    if( aCurLine.CLine().IsArcSegment( i ) )
        clearance += KiROUND( SHAPE_ARC::DefaultAccuracyForPCB() );   // :593
    SHAPE_LINE_CHAIN hull = seg.Hull( clearance + extraHullExpansion, ... );
```

`clearance` is the loop's own variable, declared outside it at `:570`, so the bump **accumulates**: an arc spanning twelve approximation segments adds 60000 nm of clearance, and every hull built after it in the same pass keeps that inflation. The comment at `:588` says the intent is "Arcs need additional clearance to ensure the hulls are always bigger than the arc", which is a per hull statement. Erratum E26. Note also that the constant is `ARC_HIGH_DEF` = 5000 nm while the chain's approximation error is `ARC_HIGH_DEF / 5` = 1000 nm, a factor of five of headroom.

**`onCollidingArc( LINE& aCurrent, ARC* aObstacleArc )`, `:689` to `:732`.** Assemble the obstacle's line, bail with `SH_TRY_WALK` on locked segments (`:696`, `:697`), `ShoveObstacleLine`, then a length gate: `extensionFactor = shovedLen / obsLen - 1.0` and `> 1.0` means `SH_TRY_WALK` (`:701` to `:711`). Both lengths are `SHAPE_LINE_CHAIN::Length()`, so they include the true arc lengths. On success, rank the shoved line at `rank - 1`, replace, push. Note 04 section 2.3 already records the erratum that it returns `SH_OK` even when `shoveOK` is false (`:731`), where `onCollidingSegment` returns `SH_INCOMPLETE` (`:682`).

**The two `ARC_T` cases in `shoveIteration`, `:1793` and `:1833`.** The forward case (`:1833` to `:1843`) is symmetric with `SEGMENT_T`: call `onCollidingArc`, convert `SH_TRY_WALK` to `onCollidingSolid`. The reverse case (`:1793` to `:1808`) is **not** symmetric with `SEGMENT_T` (`:1751` to `:1791`). It omits four things:

| `SEGMENT_T` does | `ARC_T` does |
| --- | --- |
| `unwindLineStack( &revLine )` (`:1761`) | nothing |
| `patchTadpoleVia( ni, currentLine )` (`:1762`) | nothing |
| handles "current line ends with a via that collides with the obstacle" (`:1764` to `:1777`) | nothing |
| `onCollidingLine( revLine, currentLine, revLine.Rank() + 1 )` (`:1779`) | `onCollidingLine( revLine, currentLine, revLine.Rank() - 1 )` (`:1800`) |

The rank sign is the anti ping pong mechanism (note 04 section 1.4): a reverse collision raises the obstacle's rank above the pusher, a forward one lowers it. Passing `- 1` on the reverse path makes the arc's line rank below the pusher, which is the forward convention. The `//TODO(snh): Handle Arc shove separate from track` comment at `:1795` suggests the branch was written as a copy of the forward case. Erratum E27.

### 5.3 `pns_optimizer.cpp`

Eighteen hits after the include. `ClearCache` at `:197` is a false positive on the substring. The rest:

**The `hasArcs` gate, `:660` and `:712` to `:729`.** `bool hasArcs = aLine->ArcCount();` then four of the six passes are skipped outright when it is true: `MERGE_SEGMENTS` (`mergeFull`), `MERGE_OBTUSE`, `SMART_PADS`, `FANOUT_CLEANUP`, each with a `// TODO: Fix for arcs` comment. Only `MERGE_COLINEAR` and, through `REQUIRE_OBTUSE_ANGLES`, `dragFixCorners` still run on a line with arcs.

**`mergeColinear`, `:627` to `:649`.** Skips zero length segments with the comment "Skip zero-length segs caused by abutting arcs" (`:638`, `:639`), and removes a colinear join point only when `!line.IsPtOnArc( segIdx + 1 )` (`:642`). Here `IsPtOnArc` is the right predicate, because the point itself must not carry an arc reference for `Remove` to be safe. The loop does not step `segIdx` back after a removal, so it skips one candidate per removal; that is a pre-existing wart, not arc specific.

**`dragFixCorner`, `:745`, `:746` and `dragFixCorners`, `:814`, `:815`.** Both bail when either adjacent segment is an arc segment. Both are live: `REQUIRE_OBTUSE_ANGLES` is not gated on `hasArcs` (`:703` to `:710`), and it is the dragger's flag (note 06 section 6).

**`mergeStep`, `:859` and `:868`, `:869`.** `:859` is the corner mode `is90mode` computation. `:867` to `:872` skips a pair of candidate segments when either is an arc segment. **That guard is unreachable**: `mergeStep`'s only caller is `mergeFull` (`:612`), and `mergeFull` runs only when `!hasArcs` (`:713`). Erratum E28.

### 5.4 `pns_walkaround.cpp`

One hit, `:162`, the corner mode bounding box substitution of section 3.4. The walkaround itself is arc unaware by construction: it works through `HullIntersection` (`pns_utils.cpp:403`), which calls `SHAPE_LINE_CHAIN::Intersect`, which is polyline only (section 2.5). The one place arcs come back is `LINE::Walkaround`'s tail, `restoreUntouchedArcs` (`pns_line.cpp:661`, section 4.5).

### 5.5 `pns_dragger.cpp`

Note 06 sections 2.6 and 3.5 cover `startDragArc` and `LINE::DragArc`. Three sites they do not:

- `:139`, inside `startDragSegment`'s free angle branch: the drag index advances to the far endpoint only when that point is not on an arc, so a free angle drag never picks an arc's endpoint as its corner.
- `:418` to `:435`, `DM_ARC` in `dragMarkObstacles`: copy the line, `DragArc`, remove the original from `m_lastNode` and add the dragged one. The comment at `:426`, `:427` records that a collapsed arc drag leaves an empty chain, `Add()` is then a no-op, and the arc is simply dropped from the route, which it calls the intended outcome.
- `:762` to `:776`, `DM_ARC` in `dragWalkaround`, and `:865` to `:885`, `DM_ARC` in `dragShove`. The shove path guards against a degenerate result with `if( draggedPreShove.CLine().PointCount() >= 2 )` (`:875`) and the comment at `:872` to `:874`.

`:280` and `:351`, `:352` are the kind mask and the `Start` dispatch to `startDragArc`.

### 5.6 `pns_component_dragger.cpp`

Eleven hits, of which two are kind masks (`:131`, `:138`). The arc case of the per item move (`:210` to `:221`) clones the `ARC`, calls `SHAPE_ARC::Move( aP - m_p0 )` on the clone's arc, adds it to the dragged set and to the current node. `SHAPE_ARC::Move` translates all three points and re-runs `update_values()` (`shape_arc.cpp:1079`), so a translated arc gets a freshly computed centre. Because of `CalcArcCenter`'s round number snapping (section 1.3), translating an arc by `(d, 0)` does **not** in general move its centre by exactly `(d, 0)`. For a component drag that is cosmetic; for a port that wants "drag by d then drag by -d is the identity" it is not.

### 5.7 `pns_diff_pair.cpp` and `pns_diff_pair_placer.cpp`

`pns_diff_pair.cpp`: four kind masks (`:106`, `:112`, `:442`, `:443`) and one real branch, `CoupledSegmentPairs` (`:834` to `:865`), which `continue`s past any arc segment on either lane (`:842`, `:847`) with the comment at `:838` that the chains must not be simplified or the indices go stale. So an arc contributes nothing to the coupled length and the pair's gap is measured only on straight stretches.

`pns_diff_pair_placer.cpp`: one include and one branch, the `ARC_T` case of `getDanglingAnchor` (`:479` to `:492`), which returns whichever arc endpoint sits on a joint with exactly one link. Identical in shape to the `SEGMENT_T` case below it.

### 5.8 The meander placers

Note 08 section 9 has this in full: `MakeArc` sets `MT_CORNER` not `MT_ARC` (erratum E4 there), `AddArcAndPt` and `AddPtAndArc` have no caller (E5 there), `makeMiterShape`'s `MEANDER_STYLE_ROUND` branch is the only `SHAPE_ARC` construction in the meander code (`pns_meander.cpp:491` to `:499`), and the three `MT_ARC` skips in `tuneLineLength` are dead.

Two things note 08 did not need:

- **`MEANDER_PLACER::doMove`'s arc passthrough** (`pns_meander_placer.cpp:247` to `:260`) advances with `i = tuned.NextShape( i )` and then `if( i < 0 ) i = tuned.SegmentCount();` before the `continue`, so the `for` loop's own `i++` runs on top of `NextShape`'s answer. The next iteration therefore starts one shape **past** the one `NextShape` named. For a chain of alternating arcs and segments that skips the segment after every arc. Erratum E29.
- **`makeMiterShape`'s round branch** mixes an explicit and an implicit narrowing on the same value. `lc.Append( (int) p.x, (int) p.y )` at `pns_meander.cpp:486` casts, and `arc.ConstructFromStartEndAngle( aP, arcEnd, ... )` at `:496` passes the `VECTOR2D` through `VECTOR2<int>`'s converting constructor. That constructor clamps into the `int` range and then `static_cast`s (`libs/kimath/include/math/vector2d.h:92` to `:99`), so it truncates toward zero exactly as the explicit cast does and the two agree. Checked because a mismatch here would leave a one nanometre segment between the chain's last point and the arc's start; there is none. Not an erratum, recorded so the port does not have to re-derive it.

`pns_dp_meander_placer.cpp:342` to `:441` is `addCornersUntilIndex`, the lockstep walk over both lanes with its four way branch on which side is on an arc (note 08 section 6.6). `pns_meander_skew_placer.cpp:61` and `pns_meander_placer.cpp:71` are the `SEGMENT_T | ARC_T` start gates. `pns_meander_placer_base.cpp:211`, `:261`, `:276` are the three dead `MT_ARC` skips.

### 5.9 Helpers, router, tool base, router tool, preview, logger

**`pns_helpers.cpp`.** `FindBestStartPoint`'s arc branch (`:64` to `:93`) uses `SHAPE_ARC::NearestPoint` for the cursor distance and the **arc mid point** for the baseline distance, where the segment branch uses the segment's nearest point for both. `:158` to `:163` builds a `SHAPE_ARC` from a `PCB_ARC` to snap against. `GetSnappedStartPoint` (`:195` to `:205`) asserts the item is an arc and returns the nearer of the two anchors.

**`pns_router.cpp`.** `:182` counts `SEGMENT_T | ARC_T` to decide between the single and multi dragger; `:363` is the same mask in `isStartingPointRoutable`; `:1075` to `:1078` is `ToggleCornerMode`; `:1106` to `:1117` is `BreakSegmentOrArc`, which dispatches to `SplitAdjacentArcs`. `:174`, `:436` and `:766` are `ClearCaches` substring false positives.

**`pns_tool_base.cpp`.** `:185` and `:322` are kind masks. `:487` to `:510` is the snap: for a segment or an arc, snap to the nearer anchor when the cursor is within half the width of it, otherwise `AlignToSegment` for a segment and `m_gridHelper->AlignToArc( aP, *shape )` for an arc.

**`router_tool.cpp`.** Besides the corner mode actions of section 3.4, three gates that matter:

- `:2682` to `:2686`: `NeighboringSegmentFilter` returns early when the collector holds any `PCB_ARC_T`, with the comment "We eliminate arcs because they are not supported in the inline drag code". So the selection tool's inline drag never sees an arc.
- `:2316` to `:2325`: the "optimize selected tracks" path skips any assembled line with `ArcCount() > 0`, with the comment "TODO: could allow these once we have arc-aware drag/optimize".
- `:2918` to `:2923`: the multi drag selection **does** accept `ARC_T`.

So in KiCad's UI at this commit, an arc reaches the dragger only through the router tool's own drag gesture and through multi drag, never through the selection tool's inline drag.

**`router_preview_item.cpp`.** `:69` zeroes a `SHAPE_ARC`'s width when the preview takes ownership of a shape. `:157` to `:159` takes an `ARC` item's width. `drawLineChain` (`:252` to `:289`) draws **every** polyline segment first, arc approximation segments included, and then draws each arc in `CArcs()` on top (`:278` to `:285`). So an arc is drawn twice, once as its 1000 nm chords and once as a true arc. Erratum E31. `drawShape`'s `SH_ARC` case (`:453` to `:471`) is the single arc path and does not have the problem.

**`pns_logger.cpp`.** `:192` groups `ARC_T` with `SEGMENT_T` and logs `aItem->Shape( aItem->Layer() )`; `formatShapeAsJSON`'s `SH_ARC` case (`:245` to `:255`) writes `{ "type": "arc", "width", "start", "end", "mid" }`. That is the three point form, so the log format is lossless for arcs. It is also the only place in the tree that serialises an arc inside a router artefact, since `SHAPE_LINE_CHAIN::Format` drops them (section 2.6).

---

## 6. Collision dispatch with arcs

`libs/kimath/src/geometry/shape_collisions.cpp`. Six `SHAPE_ARC` cases plus the dispatcher rows.

| Pair | Line | Method | MTV |
| --- | --- | --- | --- |
| arc, circle | `:597` | `NearestPoints`, both widths applied | yes, `delta.Resize( clearance - sqrt(dist_sq) + 3 )` (`:626`) |
| arc, line chain | `:636` | straight segments first skipping arc segments, then arc against each stored arc (`:658`, `:683` to `:688`) | asserted unimplemented (`:639`) |
| arc, rect | `:721` | rounded rect delegates to the outline (`:724`, `:725`); otherwise `NearestPoints( SHAPE_RECT )`, **width ignored** | yes, same `+ 3` (`:753`) |
| arc, segment | `:763` | `aA.Collide( aB.GetSeg(), clearance + aB.GetWidth()/2 )`, then subtract the half width from `*aActual` (`:777` to `:780`) | asserted unimplemented (`:766`) |
| arc, line chain base | `:786` | `PointInside` shortcut for a closed chain, else `aA.Collide( segment )` per segment (`:803` to `:832`) | asserted unimplemented (`:796`) |
| arc, arc | `:850` | `NearestPoints( SHAPE_ARC )`, both widths applied | yes, same `+ 3` |
| ellipse, arc | `:998` | present in the dispatcher | the router degrades ellipses to bounding boxes before this can be reached (note 01 section 7.5) |

Five of the six begin with `if( aA.IsEffectiveLine() )` and hand off to a `SHAPE_SEGMENT` path, negating the MTV where one is produced (`:600` to `:609`, `:727` to `:736`, `:771` to `:775`, `:790` to `:794`, `:853` to `:868`, the last testing both arcs). The arc/line-chain case at `:636` does not, because it walks the chain itself.

Which does the router reach? `ITEM::collideSimple` collides the two items' `Shape()` values. An `ARC`'s shape is a `SHAPE_ARC` carrying the track width; the other party is one of:

- `SHAPE_SEGMENT` from a `SEGMENT` (`pns_segment.h:146`): the arc/segment case, **reached**.
- `SHAPE_CIRCLE` from a `VIA` (`pns_via.h:349`): the arc/circle case, **reached**.
- `SHAPE_SIMPLE` from a `SOLID` pad (`pns_kicad_iface.cpp:1734`): `SH_SIMPLE` routes to the `SHAPE_LINE_CHAIN_BASE` row (`:1222`, `:1223`), **reached**.
- `SHAPE_COMPOUND` from a compound pad or a slot hole: flattened by the dispatcher into its children, each of which lands in one of the rows above.
- `SHAPE_ARC` from another `ARC`: **reached**.
- `SHAPE_LINE_CHAIN` from a `LINE`: the arc/line-chain case at `:636`, **reached** wherever a line is collided as an item rather than through its segments.
- `SHAPE_RECT`: built by the router only in `PNS::ApproximateSegmentAsRect` (`pns_utils.cpp:356`), which is not part of any collision path, so the arc/rect case at `:721` is **not reached from the router**. Its `NearestPoints` width omission (erratum E3) therefore costs KiCad's router nothing, and costs a host that hands the router `SHAPE_RECT` pads everything.

None of the arc rows produce an MTV except against a circle, a rect and another arc. The shove's via pushout (`onCollidingVia`) is the only MTV consumer (note 01 section 9.2), and it collides a `SHAPE_CIRCLE` against things, so the arc/circle row is the one that matters there.

---

## 7. The host side

### 7.1 KiCad, `pcbnew/router/pns_kicad_iface.cpp`

**Sync in.** `syncArc( PCB_ARC* )` (`:1770` to `:1788`) is four lines of substance:

```cpp
auto arc = std::make_unique<PNS::ARC>(
        SHAPE_ARC( aArc->GetStart(), aArc->GetMid(), aArc->GetEnd(), aArc->GetWidth() ),
        aArc->GetNet() );
arc->SetLayer( GetPNSLayerFromBoardLayer( aArc->GetLayer() ) );
arc->SetParent( aArc );
```

plus the lock marker from `IsLocked()` (`:1779`, `:1780`) and from a parent `PCB_GENERATOR` (`:1782` to `:1786`). It is called from the track loop at `:2434` to `:2437` with `aAllowRedundant = true`. `PCB_ARC` stores the same three points, so the sync is **exact and lossless in both directions**: no centre, no angle, no rounding.

**Commit back.** `UpdateItem`'s `ARC_T` case (`:2659` to `:2671`) writes `SetStart( arc_shape->GetP0() )`, `SetEnd( GetP1() )`, `SetMid( GetArcMid() )`, `SetWidth( arc->Width() )` onto the existing `PCB_ARC`. `AddItem`'s `ARC_T` case (`:2767` to `:2782`) constructs `new PCB_ARC( m_board, shape )` directly from the `SHAPE_ARC`, then width, layer, net, and the solder mask fields inherited from the source track when there is one (`:2775` to `:2779`).

**The one lossy place.** `PNS_PCBNEW_RULE_RESOLVER` keeps two `PCB_ARC m_dummyArcs[2]` (`:306`) to hand the DRC engine a board item for a router item that has no parent. The `ARC_T` case (`:509` to `:514`) sets the layer, the net, the start and the end, and **never sets the mid**. A default constructed `PCB_ARC`'s mid is `(0,0)`, so every rule query about a parentless arc is answered about an arc bulging through the origin. Erratum E32. For clearance rules that depend only on layer, net class and item type it makes no difference, which is presumably why nobody noticed.

### 7.2 Horizon EDA

Horizon stores an arc track as `from`, `to` and `std::optional<Coordi> center` on the ordinary `Track` class (`/home/Tubbles/dev/ref/horizon/src/board/track.hpp:60` to `:62`), with `width`, `layer`, `net` and `locked` shared with straight tracks. There is no mid point and no handedness flag.

**Sync in**, `syncTrackArc` (`/home/Tubbles/dev/ref/horizon/src/router/pns_horizon_iface.cpp:478` to `:498`):

```cpp
sarc.ConstructFromStartEndCenter( VECTOR2I(from.x, from.y), VECTOR2I(to.x, to.y),
                                  VECTOR2I(track->center->x, track->center->y),
                                  false, track->width);
```

`aClockwise` is hardcoded `false`.

**Commit back**, `AddItem`'s shared `SEGMENT_T | ARC_T` case (`:983` to `:1017`), arc half at `:1004` to `:1016`:

```cpp
from = Coordi(p0.x, p0.y);  to = Coordi(p1.x, p1.y);
if (arc.IsClockwise()) std::swap(from, to);
track->center = Coordi(c.x, c.y);
```

So Horizon's convention is "an arc track always sweeps counter-clockwise from `from` to `to`", enforced by swapping the endpoints on the way out and assumed on the way in. The two halves agree, and the preview path at `:881` to `:896` does the same swap. This is a working design, not a bug, and it is the cheapest possible centre based encoding: one bit of information (handedness) is carried by the order of the two endpoints.

The round trip is **not** exact. `ConstructFromStartEndCenter` throws away the given centre and recomputes one from the three points it derives (section 1.2), and the mid point it derives is a rounded rotation. Committing that arc back writes `GetCenter()`, the recomputed one. So sync, commit, sync moves the centre by up to the `CalcArcCenter` snapping granularity.

**Corner mode in the UI.** The task brief said Horizon does not expose one. **It does.** `src/dialogs/router_settings_window.cpp:44` to `:60` builds a "Corner style" `Gtk::ComboBoxText` offering all four modes, and `tool_route_track_interactive.cpp:1086` to `:1101` cycles them from a keystroke, with the current mode in the status bar at `:1223` and `:1264`. The setting round trips through JSON as `corner_mode` (`:85`, `:96`) and reaches the engine through `apply_settings` (`:107` to `:119`). Nothing matches under `src/imp/`, which is where the brief looked; the dialog lives under `src/dialogs/`.

### 7.3 LibrePCB

LibrePCB has no arc **traces**. `Trace` holds a layer, a width and two `TraceAnchor`s, and `Trace::serialize` writes exactly those (`/home/Tubbles/dev/librepcb/libs/librepcb/core/geometry/trace.cpp:236` to `:245`): no angle, no centre, no mid. Adding arcs to the router therefore does not give LibrePCB arc traces for free; it gives the engine the ability to carry them, and the file format would have to change first.

LibrePCB does have arcs everywhere else, encoded as a **bulge angle on the starting vertex**. A `Vertex` is a position plus "angle of the line between this vertex and the following vertex" (`libs/librepcb/core/geometry/vertex.h:89` to `:91`). `Angle` is an `qint32` of microdegrees taken modulo 360000000 (`libs/librepcb/core/types/angle.h:101`, `:102`, `:116`), so the resolution is 1e-6 degrees and the full circle is exactly representable.

The conversions LibrePCB already ships:

- `Toolbox::arcCenter( p1, p2, angle )` and `Toolbox::arcRadius( p1, p2, angle )` (`libs/librepcb/core/utils/toolbox.h:217` to `:220`, implemented `toolbox.cpp:66` to `:101`). Both map the angle to `[-180, 180]`, refuse a zero angle or coincident points with `std::nullopt`, and delegate the arithmetic to `rs::ffi_math_arc_radius` / `ffi_math_arc_radius_and_center`, that is to LibrePCB's own Rust core. Both return `std::nullopt` when the result does not fit a `Length`.
- `Toolbox::arcAngle( p1, p2, center )` (`toolbox.h:231`), counter-clockwise from `p1` to `p2` in `[0, 360)`, zero when undetermined.
- `Toolbox::arcAngleFrom3Points( start, mid, end )` (`toolbox.h:243` area, implemented `toolbox.cpp:118` to `:134`), which is exactly the three point to angle direction. Its own doc comment warns it "might not be 100% accurate and thus should not be used for important things" (`toolbox.h:237`, `:238`).
- `Path::arcObround( p1, p2, angle, width )` and `Path::flattenArcs( maxTolerance )` (`libs/librepcb/core/geometry/path.h:156`, `:107`).

The DRC flattens every arc at a 5000 nm tolerance: `BoardDesignRuleCheck::maxArcTolerance()` returns `PositiveLength( 5000 )` (`libs/librepcb/core/project/board/drc/boarddesignrulecheck.h:171`, `:172`). That is the same number as KiCad's `ARC_HIGH_DEF`, five times looser than the 1000 nm a `SHAPE_LINE_CHAIN` uses.

### 7.4 Converting between the three representations

| Host | Stored | Handedness from | Exactness |
| --- | --- | --- | --- |
| KiCad | `start`, `mid`, `end` (three `VECTOR2I`) | the mid point's side of the chord | exact integers, no derived quantity stored on the board |
| Horizon | `from`, `to`, `center` (three `Coordi`) | endpoint order (always counter-clockwise from `from`) | exact integers, but the centre over-determines the arc |
| LibrePCB | `pos` per vertex plus `angle` to the next | the angle's sign | endpoints exact, the arc between them defined by an exact microdegree angle |

**Three point to centre (KiCad to Horizon).** `CalcArcCenter` (section 1.3). Lossy: the centre is a `double` result rounded to an integer, and then snapped to a multiple of 100 or 10 nm when the propagated uncertainty allows. Two arcs whose endpoints differ by one nanometre can report centres 50 nm apart.

**Centre to three point (Horizon to KiCad).** `ConstructFromStartEndCenter` (section 1.2). Lossy in the mid point, which is a rounded rotation, and lossy in the centre, which is then recomputed from the three points rather than kept. The endpoints survive exactly.

**Three point to angle (KiCad to LibrePCB).** `SHAPE_ARC::GetCentralAngle()` (`shape_arc.cpp:971`), a difference of two `atan2` results about the derived centre, or `Toolbox::arcAngleFrom3Points` on the LibrePCB side. Both lossy, both carrying the derived centre's error. The endpoints survive exactly.

**Angle to three point (LibrePCB to KiCad).** `ConstructFromStartEndAngle` (`shape_arc.cpp:198`), which uses the two point plus angle `CalcArcCenter` (`trigo.cpp:329`, no round number snapping) and then rotates the start by half the angle. Endpoints exact, mid rounded.

The practical consequence for a LibrePCB host: **the crate should take and return the three point form, and the host should convert at the boundary**, because every conversion is lossy and doing it once per crossing is the minimum. A host that stores angles converts angle to three points on the way in and three points to angle on the way out, and must accept that an untouched arc coming back is not bit identical to the one it sent. The way to make that harmless is the same one KiCad uses for segments: the commit diff carries the host's own id, and a host that sees an item it did not ask to change can leave its stored form alone. The crate's `CommitDiff` already distinguishes added, removed and updated items (DESIGN.md section 7), so the rule is "do not rewrite an arc the diff did not list as updated".

For the round trip that does matter, the crate's own: keep the three points and compute nothing else eagerly. Every derived quantity (centre, radius, both angles, length) is a function of them, and caching a derived value means having to decide what happens when it disagrees with a recomputation. KiCad caches and does not decide.

---

## 8. Tests and fixtures

### 8.1 `qa/tests/libs/kimath/geometry/test_shape_arc.cpp`

1499 lines, 27 test cases. Read through `git show HEAD:` because `qa/tests/libs/kimath/` is not in this sparse checkout.

| Case | Line | What it pins |
| --- | --- | --- |
| `NullCtor` | `:146` | a default `SHAPE_ARC` is degenerate and does not crash |
| `BasicSMEGeom` | `:254` | the (start, mid, end) constructor against a data table |
| `BasicCPAGeom` | `:436` | the (center, start, angle) constructor |
| `BasicTTRGeom` | `:537` | the tangent-tangent-radius constructor |
| `BasicSECGeom` | `:620` | `ConstructFromStartEndCenter` |
| `CollideCircle` | `:649` | arc against circle |
| `CollidePt` | `:742` | `Collide( VECTOR2I )` |
| `CollideSeg` | `:799` | `Collide( SEG )` |
| `CollideArc` | `:948` | arc against arc |
| `CollideArcToShapeLineChain` | `:990` | the chain dispatch row |
| `CollideArcToPolygonApproximation` | `:1015` | arc against a polygonised arc |
| `ArcToPolyline` | `:1150` | `ConvertToPolyline` against a table |
| `TransformShallowArcToPolygon`, `TransformVeryShallowArcToPolygon`, `TransformIssue22475ArcToPolygon` | `:1192`, `:1230`, `:1261` | the near straight cases |
| `CollideNearlyFlatArcDoesNotOverflow` | `:1305` | the `INT_MAX/2` guards of section 1.6 |
| `DegenerateArcCoincidentPoints` | `:1343` | `CalcArcCenter`'s coincidence exits |
| `CollinearArcSweepIsNotAFullTurn` | `:1361` | `IsEffectiveLine` feeding `GetCentralAngle` |
| `CurvedArcsKeepTheirSweep` | `:1380` | the same, the other way |
| `CalcArcCenterTwoCoincidentStartMid` etc., six cases | `:1395` to `:1495` | every `CalcArcCenter` early exit and the board scale sanity check |

**Mirror these.** They are pure geometry, need no board and no GUI, and they are the executable specification of the part a port is most likely to get subtly wrong. In priority order: the six `CalcArcCenter` cases, `BasicSMEGeom`, `ArcToPolyline`, `CollideSeg`, `CollidePt`, `CollideArc`, `CollideNearlyFlatArcDoesNotOverflow`.

There is **no** test for `Reverse` versus `Reversed`, none for `Mirror`, none for `Rotate`, and none for `GetLength`.

### 8.2 The arc cases of `test_shape_line_chain.cpp`

| Case | Line | What it pins |
| --- | --- | --- |
| `ArcToPolyline` | `:179` | the chain constructor from an arc |
| `ArcToPolylineLargeCoords` | `:214` | the same at board extremes |
| `RemoveShape`, `RemoveShapeAfterSimplify` | `:557`, `:576` | `RemoveShape` on arcs, which is the placer's pullback |
| `ShapeCount`, `NextShape` | `:599`, `:616` | the shape walk |
| `AppendArc` | `:675` | `Append( SHAPE_ARC )` including the two point demotion |
| `ArcWrappingToStartSharedPoints` | `:764` | `fixIndicesRotation` and the closed chain shared point |
| `Split` | `:822` | `Split` on an arc segment producing two coincident arcs |
| `Slice` | `:896` | the three `Slice` behaviours of section 2.2 |
| `NearestPointPt` | `:1186` | `NearestPoint` including the arc snapping |
| `ReplaceChain` | `:1216` | `Replace( int, int, SHAPE_LINE_CHAIN )` |
| `CompareGeometry`, `CompareGeometryReversed` | `:1250`, `:1299` | |
| `SimplifyWithArcs` | `:1524` | `Simplify`'s arc preservation |
| `SimplifyWithToleranceIssue22597` | `:1329` | the tolerance overload |

**Mirror these**, all of them. `AppendArc`, `Slice`, `Split`, `RemoveShape` and `SimplifyWithArcs` are the five that pin the invariants the router depends on.

### 8.3 Router level tests and fixtures

`qa/tests/pcbnew/test_meander_corner_radius.cpp` has three cases. `DefaultSettings` (`:38`) is a real test: it checks `MEANDER_SETTINGS`'s defaults, including `m_cornerStyle == MEANDER_STYLE_ROUND` and `m_cornerRadiusPercentage == 80`, which the crate's `MeanderSettings` reproduces at `src/meander.rs:509` and `:566`, asserted at `:2938`. The other two, `MinCornerRadiusThreshold` (`:60`) and `GeometricConstraints` (`:93`), **call no KiCad code at all**: they compute `width / 2` and `min( amplitude/2, spacing/2 )` inline and assert their own arithmetic against a hand written table. They cannot fail for any change to `pns_meander.cpp`. Erratum E33. Mirror `DefaultSettings` only.

`qa/tests/pcbnew/` has four PNS tests (`test_pns_basics.cpp`, `test_pns_diff_pair_tuning_width.cpp`, `test_pns_tuning_path_through_pad.cpp`, `test_pns_via_layer_span.cpp`). None of the four mentions `SHAPE_ARC`, `ARC_T` or `PNS::ARC`.

`qa/tools/pns/playground.cpp` is a scratch pad, not a test: it builds a table of arc pairs in millimetres (`:239`), draws them into an overlay, and calls a local `collideArc2Arc` (`:325`) plus `SHAPE_ARC::Intersect` (`:326`). It is the development harness the arc collision work of MR 1009 was written against, and it is worth reading for the geometric cases it enumerates (`:140` to `:170`), but there is nothing to mirror.

**The regression corpus.** Note 05 section 6.9 has the eleven cases. The arc question needed a measurement, so here it is.

`qa/data/pcbnew/pns_regressions/boards/` holds ten `.kicad_pcb` files. Counting top level `(arc` forms:

| Board | arc tracks |
| --- | --- |
| `stickhub-extra-via.kicad_pcb` | **180** |
| every other board, and `walk-with-teardrops/pns-no-hug-2.dump` | 0 |

So the brief's "I found none" is nearly right: there is exactly one board with arcs, `stickhub-extra-via.kicad_pcb` (`:36285` onwards), and its arcs are ordinary copper tracks with `start`, `mid`, `end`, `width`, `layer`, `net`.

Whether any case **exercises** it is a second question, because the harness matches boards to logs by content hash, not by name (`qa/tools/pns/qa_pns_regressions_main.cpp:166` to `:191`, hashing with `IO_UTILS::fileHashMMH3`, which is MurmurHash3 x64 128 with seed `0x68AF835D`, `common/io/io_utils.cpp:86` to `:113`, `libs/kimath/include/mmh3_hash.h`). Reimplementing that hash and running it over the pool and the eleven logs gives:

| Case | resolves to |
| --- | --- |
| `drag-acute-fallback` | `drag-walk-optimize.kicad_pcb` |
| `drag-walk-optimize-a` | `drag-walk-optimize.kicad_pcb` |
| `drag-walk-optimize-fix-corners` | `drag-walk-optimize-fix-corners.kicad_pcb` |
| `issue24132-shove-same-net-via` | `shove_same_net_via.kicad_pcb` |
| `walk-with-teardrops` | its own `.dump`, no hash |
| `backspace1`, `issue22749-...`, `issue23449-...`, `simple-drag-shove-singlelayer`, `simple-shove-1`, `walk_drag_seg_against_board_edge` | **nothing in the pool** |

Six of the eleven cases carry a `board_hash` that matches no board at this commit, and `qa_pns_regressions_main.cpp:101` to `:108` turns that into `BOOST_CHECK( true )` and a pass. Note 05 section 6.8 already records that failure mode in the abstract; this is the measurement. And `stickhub-extra-via.kicad_pcb`, `pic_programmer.kicad_pcb`, `ultrasound.kicad_pcb`, `video-v10.kicad_pcb`, `simple.kicad_pcb`, `backspace1.kicad_pcb` and `dp_test.kicad_pcb` are referenced by no case at all. Erratum E34.

**Conclusion for the crate: no KiCad regression case exercises an arc.** The four cases that actually run use three arc free boards, and no `.log` in the corpus contains a `"type": "arc"` shape or a `"kind": "arc"` item. Arc behaviour in this port has to be tested against tests the port writes itself, mirrored from sections 8.1 and 8.2, plus new session level fixtures.

### 8.4 What to mirror, consolidated

1. The six `CalcArcCenter` cases from `test_shape_arc.cpp:1395` to `:1495`. Do these first; they pin the one routine whose behaviour is neither obvious nor continuous.
2. `BasicSMEGeom`, `BasicSECGeom`, `ArcToPolyline`, `CollideNearlyFlatArcDoesNotOverflow`, `DegenerateArcCoincidentPoints`, `CollinearArcSweepIsNotAFullTurn`, `CurvedArcsKeepTheirSweep`.
3. `CollidePt`, `CollideSeg`, `CollideArc`, `CollideCircle`, `CollideArcToShapeLineChain`.
4. `AppendArc`, `Slice`, `Split`, `RemoveShape`, `RemoveShapeAfterSimplify`, `ShapeCount`, `NextShape`, `ArcWrappingToStartSharedPoints`, `SimplifyWithArcs`, `NearestPointPt`, `ReplaceChain`.
5. `test_meander_corner_radius.cpp:38` only.
6. New, with no KiCad counterpart: a `BuildInitialTrace` table for `ROUNDED_45` and `ROUNDED_90` over the sign and magnitude combinations of `(dx, dy)` crossed with `startDiagonal`, because KiCad has no `test_direction45.cpp` at all (note 01 section 15); an `ArcHull` table; and a session level fixture that routes over `stickhub-extra-via.kicad_pcb`, which is the only real board with arc tracks anywhere in the corpus.

---

## 9. Known upstream issues

| Issue | Title | State | Milestone | Relevance |
| --- | --- | --- | --- | --- |
| 9023 | PCBNew shove router goes into collission when in Arc corners mode | closed 2022-09-02 | 6.0.8 | the symptom section 5.2's clearance bump exists to suppress |
| 15607 | Poor Performance / Freezing When Routing In Router Walkaround Mode | closed 2023-11-15 | 8.0 | "mitigated by using non-rounded corner styles" |
| 4270 | V6: "switch corner rounding" for arc tracks? | closed 2020-04-28 | none | the corner mode UI, closed the day after it was filed |
| 9611 | Merge free-angle mode with corner style; enable inside shove/walk | **open** | none | wishlist, and the reason free angle mode and corner mode are two settings |

All four read via `https://gitlab.com/api/v4/projects/kicad%2Fcode%2Fkicad/issues/<n>` on 2026-09-12.

**The fix for 9023.** The issue's `related_merge_requests` endpoint names one merge request, `!1009` "Shape Line Chain arc collision fixes". Its six commits, all dated 2021-11-15:

| Commit | Title |
| --- | --- |
| `cb7e57fb43f95d3263cc6a35820c297c2f6d5939` | CIRCLE::IntersectLine fix incorrect algorithm documentation comments |
| `d47bd3a04dffa6b19eaa1cbb3713b3dc2e956db6` | **Rewrite broken collision routine SHAPE_ARC::Collide( SEG& aSeg )** |
| `9b43689a76da0ced2473039e3f3a0330d2b54274` | Add SHAPE_ARC to SEG collision test cases |
| `0c3da0f0724953330c3a2b3ac87e4354959a64f0` | **Implement true arc collisions for arcs inside a SHAPE_LINE_CHAIN** |
| `ad3b4f25c237bb731408458c4b0a389bf5e1f8eb` | Add tests for shape_line_chain collision containing arcs |
| `3f60765016433f181b03b661fa43ede3593905d1` | Fix incorrect tolerance applied to CollideArcToPolygonApproximation qa test |

The merge request's file list is `libs/kimath/include/geometry/shape_line_chain.h`, `libs/kimath/src/geometry/circle.cpp`, `libs/kimath/src/geometry/shape_arc.cpp`, `libs/kimath/src/geometry/shape_collisions.cpp`, `libs/kimath/src/geometry/shape_line_chain.cpp`, and the three QA files. **It does not touch `pcbnew/router/` at all.** So the fix for 9023 was to make the geometry layer tell the truth about arcs: the candidate point `SHAPE_ARC::Collide( SEG )` of section 1.6 and the two phase `SHAPE_LINE_CHAIN::Collide` of section 2.5 are that merge request's output, and the two comments in `shape_collisions.cpp` and `shape_line_chain.cpp` that assert a chain's arcs have zero width (`:686`, `:485`, `:879`) date from it.

The shove's clearance bump at `pns_shove.cpp:588` to `:594` is therefore **not** part of the 9023 fix and its provenance is not established here; the shallow clone carries one commit and the GitLab notes endpoint refused an unauthenticated read. What can be said is that it addresses the residual of the same symptom from the router side, by making the hulls big enough that the polyline can never be the reason a collision is missed.

---

## 10. Errata

Defects, dead code and surprising behaviour found in this reading. Numbering is local to this note.

### E1. `CalcArcCenter` snaps the centre to a round number

`libs/kimath/src/trigo.cpp:529` to `:550`. After computing the circumcentre and a first order uncertainty for it, the routine replaces the result with the nearest multiple of 100 nm, or failing that 10 nm, whenever the uncertainty covers that value. The comment at `:534` to `:538` justifies it ("ALL values within the uncertainty range are equally true"). Consequence: the centre is a discontinuous function of the three points, a one nanometre endpoint move can shift it by 50 nm, and `SHAPE_ARC::GetRadius`, `GetCentralAngle` and `GetLength` inherit the discontinuity. Not a bug in KiCad's own terms; a decision a port has to take deliberately, because dropping it changes every arc the router touches.

### E2. `SHAPE_ARC::Collide( SEG )` reports the last candidate's distance, not the minimum

`libs/kimath/src/geometry/shape_arc.cpp:328` to `:335`. The candidate loop calls `Collide( candidate, aClearance, aActual, aLocation )` for every candidate, each call overwriting `*aActual` and `*aLocation`. It returns early only on `*aActual == 0`. With `aActual` requested and no exact touch, the reported distance is whichever candidate happened to be evaluated last, which is `aSeg.B`. Every caller that wants a minimum distance from an arc against a segment gets an arbitrary one.

### E3. `SHAPE_ARC::NearestPoints( const SHAPE_RECT& )` ignores the arc's width

`libs/kimath/src/geometry/shape_arc.cpp:635` to `:646`. The three sibling overloads (`SHAPE_CIRCLE` at `:543`, `SEG` at `:622`, `SHAPE_ARC` at `:652`) all pull `aPtA` in by half the arc's width and zero the distance when it falls inside. This one returns the raw centre line distance. Its consumer is `Collide( SHAPE_ARC, SHAPE_RECT )` (`shape_collisions.cpp:740`), so a wide arc track against a rectangle under-reports the collision by half the track width. The router does not reach that row (section 6), but a host that hands the engine `SHAPE_RECT` pads would.

### E4. `SHAPE_ARC::Reverse()` and `Reversed()` are not equivalent

`libs/kimath/src/geometry/shape_arc.cpp:1127` and `:1133`. `Reverse()` swaps start and end in place and does **not** call `update_values()`, so the cached centre, radius and bounding box survive. `Reversed()` builds a fresh arc from `(end, mid, start)`, which re-runs `CalcArcCenter` on a permuted input and, through E1, can land on a different rounded centre. `SHAPE_LINE_CHAIN::Reverse` uses the first (`shape_line_chain.cpp:941`), `NODE::AssembleLine` uses the second (`pns_node.cpp:1185`). Assembling the same physical line in the two scan directions can therefore produce arcs with different derived geometry.

### E5. `SHAPE_LINE_CHAIN::Insert( size_t, const SHAPE_ARC&, int )` reads out of bounds

`libs/kimath/src/geometry/shape_line_chain.cpp:1674` to `:1676`: `for( auto arc_it = m_shapes.rbegin(); arc_it != m_shapes.rend() + aVertex; arc_it++ )`. `rend() + aVertex` is past the reverse end for any non zero `aVertex`. Note 01 section 14.3 flags it; repeated here because it is an arc only path and a port will have to write the loop from scratch anyway. The same method also skips `Append`'s "more than two points" demotion rule, so a degenerate arc gets an `m_arcs` entry.

### E6. `Remove`'s arc dropping iterates a `std::set` while renumbering

`libs/kimath/src/geometry/shape_line_chain.cpp:1118` to `:1157`. Arc indices inside the removed range are collected into a `std::set<size_t>` and then `convertArc`ed in increasing order. `convertArc` decrements every index above the one it erases (`:263`, `:264`), so the second call operates on a renumbered vector while using the old index. Removing arcs 3 and 5 removes what were originally 3 and 6. Reverse iteration would be correct, which is exactly what `ClearArcs` does (`:951`).

### E7. `Replace( int, int, const SHAPE_LINE_CHAIN& )` breaks the arc order invariant

`libs/kimath/src/geometry/shape_line_chain.cpp:1071`: `m_arcs.insert( m_arcs.end(), newLine.m_arcs.begin(), newLine.m_arcs.end() )`. The incoming arcs go to the end of `m_arcs` regardless of where their points went. `Reverse()` remaps index `i` to `size - i - 1` (`:926`), which is only a correct reversal when `m_arcs` is in chain order. Note 01 section 6.2 records the invariant; this is the one operation that violates it.

### E8. `NODE::findRedundantArc` ignores the mid point

`pcbnew/router/pns_node.cpp:1742` to `:1768`. It matches on the two anchors and the layer start only. Two arcs between the same endpoints with opposite bulges are indistinguishable to it, so `NODE::Add( ARC )` refuses the second (`:780` to `:784`) and `NODE::Add( LINE& )` links the line to the wrong one (`:694` to `:698`). `findRedundantSegment` has no analogous hole because a segment is fully determined by its endpoints.

### E9. The diff pair arc coupling test requires centres within 5 nanometres

`pcbnew/router/pns_topology.cpp:1109`: `if( centerDist_sq > SEG::Square( DP_PARALLELITY_THRESHOLD ) ) continue;`, with `DP_PARALLELITY_THRESHOLD = 5` (`pns_topology.h:106`). The constant is a distance threshold for `SEG::ApproxParallel` in the segment branch above it (`:1088`); reused here it means two concentric arcs must have centres within 5 nm of each other. Both centres come out of `CalcArcCenter` with its round number snapping (E1), so two lanes of the same pair drawn at different radii routinely differ by more than that. The arc branch of `AssembleDiffPair` is close to unreachable in practice.

### E10. `Slice` extends past `aEndIndex` when the slice starts inside an arc

`libs/kimath/src/geometry/shape_line_chain.cpp:1437` to `:1464`. The point copying loop is `for( size_t i = aStartIndex; i < m_points.size() && arcToSplitIndex == ArcIndex( i ); i++ )`, with no `aEndIndex` bound, and the new arc runs to the parent arc's `GetP1()` (`:1456`). The mirror case at `:1491` is correctly bounded. So `chain.Slice( a, b )` where `a` and `b` are both interior to the same arc returns the arc from `a` all the way to its end. `LINE::restoreUntouchedArcs` reaches it (`pns_line.cpp:285`, `:288`, `:291`).

### E11. `IsPtOnArc` is used where `IsArcSegment` is meant

`pcbnew/router/pns_line_placer.cpp:197`, `:204`, `:352`, `:362`. All four ask "is the shape leaving this point an arc" and answer with `IsPtOnArc`, which is also true for an arc's last point when the segment leaving it is straight (`shape_line_chain.cpp:3240`). In that case `ArcIndex` returns the arc **ending** there and the code takes the routing direction from that arc's chord rather than from the straight segment. The same file uses `IsArcSegment` for the same question twenty five lines later (`:222`, `:377`), which is correct. The two predicates also disagree at an arc start when the previous shape was an arc without a shared vertex.

### E12. `Simplify2`'s colinear run can drop an arc's interior points

`libs/kimath/src/geometry/shape_line_chain.cpp:2968` to `:2974`. The guard checks `shapes_unique[i]` and `shapes_unique[i + 1]` before entering the run, but the extension loop at `:2971` advances `n` without re-checking `shapes_unique[n + 1]`. A shallow arc whose consecutive approximation points sit within 1 nm of the chord loses interior points while its `m_arcs` entry survives, so the chain claims an arc across a run that no longer approximates it. Reachable from the walkaround (`pns_walkaround.cpp:177`), `LINE::Walkaround` (`pns_line.cpp:660`) and the shove (`pns_shove.cpp:2378`).

### E13. Orphaned arcs stay in `m_arcs` and still collide and still draw

Neither `Simplify` (`shape_line_chain.cpp:2782`) nor `Simplify2` (`:2906`) nor `RemoveDuplicatePoints` (`:2720`) ever erases from `m_arcs`; only `convertArc` does. `SHAPE_LINE_CHAIN::Collide` iterates `ArcCount()` directly (`:480`, `:873`), as does `Collide( SHAPE_ARC, SHAPE_LINE_CHAIN )` (`shape_collisions.cpp:681`, `:683`) and `ROUTER_PREVIEW_ITEM::drawLineChain` (`router_preview_item.cpp:252`, `:278`). So an arc whose every reference was dropped still contributes a collision and still gets drawn.

### E14. `NearestPoint`'s arc snapping can index `m_shapes` out of range

`libs/kimath/src/geometry/shape_line_chain.cpp:2427` to `:2450`. The guard is `nearest > 0 && nearest < PointCount() && IsArcSegment( nearest )`; `nearest++` at `:2433` can then make it equal `PointCount()`. `IsArcStart` and `IsArcEnd` are bound checked and answer false, so control reaches `Arc( ArcIndex( nearest ) )` at `:2442`, and neither `ArcIndex` (`shape_line_chain.h:856`) nor `Arc` (`:864`) is bound checked. Reachable only on a closed chain whose last segment is an arc segment, which the router does not build.

### E15. `Format` drops arcs and `Parse` is not its inverse

`libs/kimath/src/geometry/shape_line_chain.cpp:2506` to `:2532` emits points and the closed flag only; the arc half sits behind `/* fixme: arcs` at `:2525`. `Parse` (`:2623` to `:2668`) reads an entirely different, whitespace separated format, clears `m_points` but not `m_shapes` or `m_arcs` (`:2628`), never reconstructs a shared point because it writes `{ ind, SHAPE_IS_PT }` unconditionally (`:2650`), and builds each arc from a centre, a start and an angle with the width defaulted to 0 (`:2664`). The two cannot round trip anything, arcs or not.

### E16. `SelfIntersectingWithArcs` has no router caller

`libs/kimath/src/geometry/shape_line_chain.cpp:2234`, declared `shape_line_chain.h:713`. Its one caller in the whole tree is `pcbnew/exporters/step/step_pcb_model.cpp:454`. The router's three self intersection tests all use the polyline only `SelfIntersecting` (`pns_line.cpp:360`, `pns_shove.cpp:474`, `pns_diff_pair.cpp:254`), so two arcs whose 1000 nm approximations do not cross are accepted even when the true arcs do.

### E17. `ROUNDED_45` computes an arc radius it uses in one branch out of four

`libs/kimath/src/geometry/direction_45.cpp:130` to `:132`. `diag2`, `diagLength` and `arcRadius` are computed unconditionally; `arcRadius` is read only at `:193`, in the `!startDiagonal && tangentLength < 0` sub-case, and `diagLength` only feeds `arcRadius`. The other three sub-cases derive the arc from two endpoints and an angle.

### E18. The `MIN_PRECISION_IU` re-snap rebuilds the arc with the opposite angle sign

`libs/kimath/src/geometry/direction_45.cpp:194` builds `SHAPE_ARC ca( arcCenter, aP0, -ANGLE_45 * rotationSign )`. The two correction paths at `:205` and `:210` rebuild it with `ConstructFromStartEndAngle( ca.GetP0(), fixedEnd, ANGLE_45 * rotationSign )`, positive where the original was negative. `ConstructFromStartEndAngle` places the centre from the angle's sign (`trigo.cpp:336` to `:346`), so the corrected arc bulges the other way. Whether that is intended is not discoverable from the code; the comment at `:197`, `:198` speaks only of endpoint precision, and the `TODO` at `:199` says the math should produce the endpoint directly.

### E19. `ROUNDED_45`'s `w == h` block is unreachable

`libs/kimath/src/geometry/direction_45.cpp:123` to `:128`. The single segment shortcut at `:46` returns for `!is90mode && h == w`, and `ROUNDED_45` is not a 90 mode, so the block can never be entered.

### E20. `PNS::ARC::CLine()` has no caller

`pcbnew/router/pns_arc.h:93`. Returns `SHAPE_LINE_CHAIN( m_arc )`, a 1000 nm polygonisation. A grep of the tree finds no use. `SEGMENT` has no analogue.

### E21. `ArcHull` dereferences two optionals without checking

`pcbnew/router/pns_utils.cpp:127` to `:132`. `sa_out.IntersectLines( sb_out )` and `sa_in.IntersectLines( sb_in )` return `OPT_VECTOR2I` and are dereferenced at `:131` and `:132`. Two exactly collinear consecutive polyline segments return `nullopt`. `ConvertToPolyline` at `ARC_LOW_DEF` never produces those for a non degenerate arc, so it is latent rather than live, but a port that changes the polygonisation accuracy inherits the hazard.

### E22. `NODE::Add( LINE& )` links arcs before segments, `ClipVertexRange` assumes chain order

`pcbnew/router/pns_node.cpp:689` to `:732` runs the arc loop first and the segment loop second, so `LINE::m_links` is in "all arcs, then all segments" order for a line containing both. `LINE::ClipVertexRange` (`pns_line.cpp:1469` to `:1500`) walks shapes with `NextShape` and increments a link index in lockstep, which requires chain order. `NODE::AssembleLine` does link in chain order (`pns_node.cpp:1188`), and that is the path every clipping caller uses, so the mismatch is latent.

### E23. `NODE::Dump` is `#if 0` and would no longer compile

`pcbnew/router/pns_node.cpp:1473` onwards. The body uses `GetKind()`, `GetPos()`, `GetLinkList()`, `GetSeg()`, `GetLine()`, `GetNet()`, `GetLinkedSegments()` and an `AssembleLine` returning `LINE*`, none of which exist. Do not port it; the crate's debug decorator (`src/debug.rs`) is the live equivalent.

### E24. `FixRoute` drops a straight segment whose first point is an arc's last point

`pcbnew/router/pns_line_placer.cpp:1669` to `:1702`. For such a point `ArcIndex( i )` returns the arc ending there, so the else branch runs, `arcIndex == lastArc` holds and the `continue` at `:1688` skips it. The rescue condition at `:1673` only covers `i == lastV - 1`. A chain shaped arc, straight, arc therefore commits the two arcs and loses the straight between them. `BuildInitialTrace` never produces that shape (it returns at most one arc), but `mergeHead`'s `tail.Append( head )` (`:371`) can.

### E25. `simplifyNewLine` casts an `ARC` to `SEGMENT`

`pcbnew/router/pns_line_placer.cpp:1912`, `:1925`, `:1931`. `processJoint` accepts anything matching `SEGMENT_T | ARC_T` (`:1919`) and then reads `->Seg()` and `->Width()` through `static_cast<const SEGMENT*>`. `ARC` and `SEGMENT` are siblings under `LINKED_ITEM` (`pns_arc.h:37`, `pns_segment.h:38`), so the cast is undefined and `Seg()` reads a `SHAPE_SEGMENT` out of a `SHAPE_ARC`'s bytes. `lastItem` reaching `simplifyNewLine` can be an `ARC` (`:1695`, `:1713`), and the joint's neighbours can be arcs regardless.

### E26. The shove's arc clearance bump accumulates across the hull loop

`pcbnew/router/pns_shove.cpp:588` to `:594`. `clearance` is declared outside the loop at `:570` and `+=` inside it, so every arc segment permanently inflates the clearance used for every subsequent hull in the same pass. An arc spanning twelve approximation segments adds 60000 nm. The comment's intent is per hull.

### E27. `shoveIteration`'s reverse `ARC_T` case diverges from `SEGMENT_T`

`pcbnew/router/pns_shove.cpp:1793` to `:1808` versus `:1751` to `:1791`. The arc case omits `unwindLineStack`, omits `patchTadpoleVia`, omits the "current line ends with a colliding via" handling, and passes `revLine.Rank() - 1` where the segment case passes `revLine.Rank() + 1`. The sign is the anti ping pong rank convention (note 04 section 1.4): `+ 1` on a reverse collision, `- 1` on a forward one. The `//TODO(snh): Handle Arc shove separate from track` at `:1795` suggests the branch was never finished.

### E28. `mergeStep`'s arc guard is unreachable

`pcbnew/router/pns_optimizer.cpp:867` to `:872`. `mergeStep` is called only from `mergeFull` (`:612`), and `Optimize` gates `mergeFull` on `!hasArcs` (`:713`). A line with arcs never reaches it.

### E29. `MEANDER_PLACER::doMove`'s arc passthrough skips the shape after every arc

`pcbnew/router/pns_meander_placer.cpp:247` to `:260`. The body sets `i = tuned.NextShape( i )` and then `continue`s, which runs the `for` loop's own `i++`. `NextShape` returns the first vertex index of the next shape, so the next iteration starts one past it. For a tuned stretch of alternating arcs and straight segments, the segment following each arc is never meandered and never emitted as a corner.

### E30. `ConstructFromStartEndAngle` and `ConstructFromStartEndCenter` take a `double` width into an `int` member

`libs/kimath/include/geometry/shape_arc.h:99` and `:112` declare `double aWidth`; `shape_arc.cpp:204` and the corresponding assignment store it into `int m_width`. Narrowing without a round. Every router call site passes an integer or omits the argument, so it costs nothing today, but the signature invites a fractional width that silently truncates.

### E31. The preview draws every arc twice

`pcbnew/router/router_preview_item.cpp:252` to `:289`. `drawLineChain` strokes every polyline segment, including the ones approximating an arc, and then strokes each entry of `CArcs()` as a true arc on top. At the router's line widths the two are within 1000 nm of each other, so it reads as a slightly thickened arc rather than an obvious double image.

### E32. The rule resolver's dummy arc never gets a mid point

`pcbnew/router/pns_kicad_iface.cpp:509` to `:514`. `m_dummyArcs[aIdx]` gets a layer, a net, a start and an end. Its mid stays at the `PCB_ARC` default, so every DRC query about a parentless router arc describes an arc bulging through the origin. The `SEGMENT_T` case above it has no such hole because a segment needs only two points.

### E33. Two of the three meander corner radius tests assert their own arithmetic

`qa/tests/pcbnew/test_meander_corner_radius.cpp:60` (`MinCornerRadiusThreshold`) and `:93` (`GeometricConstraints`) compute `width / 2` and `min( amplitude / 2, spacing / 2 )` inline and compare against a hand written expectation table. Neither constructs a `MEANDER_SHAPE` or calls `cornerRadius()`. They cannot fail for any change to `pns_meander.cpp`. Only `DefaultSettings` (`:38`) tests KiCad code.

### E34. Six of the eleven regression cases resolve to no board

Measured by reimplementing `IO_UTILS::fileHashMMH3` (`common/io/io_utils.cpp:86`, MurmurHash3 x64 128, seed `0x68AF835D`) and running it over `qa/data/pcbnew/pns_regressions/boards/` and the eleven `board_hash` fields. `backspace1`, `issue22749-shove-weird-drag-track-end`, `issue23449-shove-lone-via-drag-crash`, `simple-drag-shove-singlelayer`, `simple-shove-1` and `walk_drag_seg_against_board_edge` match nothing in the pool, and `qa_pns_regressions_main.cpp:101` to `:108` turns that into a pass. Seven of the ten boards, including the only one with arc tracks, are referenced by no case. Note 05 section 6.8 records the failure mode; this is the count.

### E35. `IsArcEnd( 0 )` wraps to the last point even on an open chain

`libs/kimath/src/geometry/shape_line_chain.cpp:3286`, `:3287`. Unconditional, with the `aIndex > size - 1` bound check in the `else if` below it. On an open chain the first point can therefore be reported as an arc end because the **last** segment is an arc segment. Note 01 section 6.2 records it; repeated here because the port's `ArcRef` design (section 11.1) removes the need for the predicate to be geometric at all.

### E36. `NODE::Add( ARC )` has no degenerate check

`pcbnew/router/pns_node.cpp:776` to `:788`. `Add( SEGMENT )` refuses a segment whose two ends coincide (`:749` to `:754`) with a log line. The arc overload accepts an arc whose three points coincide, which then joins the joint map twice at the same position and appears in the index with a degenerate bounding box.

### E37. `SplitAdjacentArcs` does not check that the split point is on the arc

`pcbnew/router/pns_line_placer.cpp:1315` to `:1344`. `aP` is used directly as the end of the first half and the start of the second (`:1333`, `:1336`) with no containment test. An off-arc point produces two arcs that meet at a point the original arc never passed through. `SplitAdjacentSegments` has the same shape and the same hole, so this is not arc specific, but the arc version's consequence is larger because both halves are rebuilt through a centre.

### E38. `ArcHull` and `SegmentHull` round the same octagon quantity differently

`pcbnew/router/pns_utils.cpp:86` computes `int x = (int)( 2.0 / ( 1.0 + M_SQRT2 ) * d ) / 2;`, a truncation followed by an integer divide. `SegmentHull` computes the same quantity as `KiROUND( x / 2.0 )` (`:190`). The two can differ by 1 nm. Combined with the half width inconsistency note 01 section 12.2 records (`( t + 1 ) / 2` in `ArcHull` at `:73` versus `t / 2` in `SegmentHull` at `:186`), an arc's hull and a segment's hull of the same width and clearance are not built to the same tolerance. Port both verbatim; the walkaround's termination depends on hulls and collisions agreeing to the nanometre (note 01 section 14.6).

---

## 11. Rust mapping

### 11.1 Note 01 section 14.3's `ArcRef` plan, evaluated

The plan was:

```rust
pub struct LineChain {
    points: Vec<Vec2>,
    shapes: Vec<ArcRef>,     // parallel to points
    arcs:   Vec<ShapeArc>,
    closed: bool,
    width:  i32,
}

#[derive(Copy, Clone, PartialEq, Eq, Default)]
pub enum ArcRef {
    #[default] Plain,
    On(ArcIdx),
    Shared { ends: ArcIdx, starts: ArcIdx },
}
```

**Keep**, with four changes.

**Keep the enum over the `(isize, isize)` pair.** It removes `SHAPE_IS_PT` and makes "second is set while first is not" unrepresentable, which is the invariant `convertArc` has to re-establish by hand (`shape_line_chain.cpp:267`, `:268`). It also removes the need for `reversedArcIndex`, which is dead anyway (E35's neighbour, `shape_line_chain.h:941`).

**Change 1: `On(ArcIdx)` should carry the point's role.** KiCad answers "is this the arc's first point" by comparing `arc.GetP0() == m_points[i]` (`shape_line_chain.cpp:3278`) and "is it the last" by the same against `GetP1()` (`:3299`). Those are exact point comparisons against a value that `amendArc` and `Slice`'s re-cut can change (section 1.2), and they are why `IsArcEnd( 0 )` has to wrap unconditionally (E35). Carry the role instead:

```rust
pub enum PointRole { Start, Interior, End }
pub enum ArcRef {
    Plain,
    On { arc: ArcIdx, role: PointRole },
    Shared { ends: ArcIdx, starts: ArcIdx },
}
```

`is_arc_start`, `is_arc_end` and `is_pt_on_arc` then become field reads, maintained by the paired mutators, and `is_arc_segment` becomes `matches!(shapes[i], Plain) == false && shapes[i].leaving_arc() == shapes[i+1].entering_arc()` with no wrap special case and no geometry. That also removes E11's class of bug by construction, because `leaving_arc()` returns `None` for an `End` and the placer's two predicates collapse into one correct one.

**Change 2: enforce the chain order invariant.** KiCad's `Reverse` remaps `i` to `len - i - 1` (`shape_line_chain.cpp:926`), which is a correct reversal only while `arcs` is in chain order, and `Replace( range, chain )` violates that (E7). Make `replace_with_chain` splice the incoming arcs into position rather than appending, and put a debug assertion on the invariant in a private `check_invariants` that the test build calls after every mutator.

**Change 3: never iterate `arcs` directly.** E13 is the direct consequence of `Collide` and the preview walking `ArcCount()`. Expose `arcs()` only as an iterator derived from `shapes`, or keep the `Vec` private and add `fn live_arcs(&self) -> impl Iterator<Item = (ArcIdx, &ShapeArc)>` that yields each arc once, in chain order, skipping any with no reference. Then port `collide_point` and `collide_seg` against that iterator and an orphan cannot collide.

**Change 4: `arc(i)` returns `Option`.** `Arc` and `ArcIndex` are both unchecked in KiCad (`shape_line_chain.h:856`, `:864`) and E14 is reachable through the second. Note 01 section 14.3 already asks for this for `Slice`; extend it to the arc accessors.

### 11.2 The arc value type

**Recommendation: KiCad's three point form, with no cached derived values.**

```rust
pub struct ShapeArc {
    start: Vec2,
    mid:   Vec2,
    end:   Vec2,
    width: i32,
}
```

Why the three points and not a centre or an angle:

- It is exactly representable in `i32` nanometres, which is the crate's whole numeric premise (DESIGN.md section 2). A centre over-determines the arc and an angle needs a second unit.
- Handedness is free and exact: `is_ccw` is one `i64` cross product (`shape_arc.h:310`).
- It is the only form that survives a round trip with KiCad's board file and with `PCB_ARC` without loss (section 7.1), which matters for replaying KiCad's corpus.
- Translation, mirroring and 90 degree rotation are exact on all three points.
- `PartialEq` is exact and cheap.

What the hosts need at the boundary: **the three points plus the width**, and nothing else. Horizon and LibrePCB both convert, and both conversions are lossy in the same direction (section 7.4), so the crate should not take a centre or an angle even as a convenience constructor for a host. Give hosts `ShapeArc::from_start_end_center( start, end, center, clockwise )` and `ShapeArc::from_start_end_angle( start, end, angle )` as named, documented, explicitly lossy constructors, and make the documentation say that the centre they pass is not stored.

Why no cached values: KiCad caches `m_center`, `m_radius` and `m_bbox` and then has to decide what happens when they go stale. `Reverse()` leaves them stale deliberately (E4) and nothing else in the class does, which is exactly the kind of inconsistency the crate has been removing everywhere else. Compute `center()`, `radius()`, `central_angle()`, `start_angle()`, `end_angle()`, `length()` and `bbox()` on demand. `center()` is the expensive one; the two hot paths, `ArcHull` and `ConvertToPolyline`, each need it once and can take it as an argument internally.

The centre needs a name for its degeneracy:

```rust
pub enum ArcCenter {
    Circumcentre(Vec2),
    Degenerate(Vec2),   // one of CalcArcCenter's four stand-ins
}
```

because `GetCentralAngle`'s `IsEffectiveLine` branch (`shape_arc.cpp:982`) depends on the stand-in existing, and a port that returns `Option` there has to re-derive that dependency at every call site.

**The round number snapping (E1) has to be a logged decision.** Porting it verbatim keeps the crate's arcs bit compatible with KiCad's and keeps any future golden comparison against KiCad's corpus meaningful. Dropping it moves every arc centre by up to 50 nm relative to KiCad and makes `AssembleDiffPair`'s 5 nm centre test (E9) behave differently. Recommendation: **port it verbatim**, in a function called `calc_arc_center` with a doc comment quoting `trigo.cpp:534` to `:538`, and write the decision into `doc/log/`.

### 11.3 The crate modules that change

**`src/geometry/arc.rs`, new.** `ShapeArc` and its methods listed in 11.2, plus the two named constructors, `slice_contains_point`, `nearest_point`, `collide_point`, `collide_seg`, `convert_to_polyline`, `is_effective_line`, `reverse`, `reversed`, `move_by`, `mirror`, `rotate`, `chord`. Plus the free functions `calc_arc_center` (three point) and `calc_arc_center_from_angle` (two point plus angle), `arc_to_segment_count`, `circle_to_end_segment_delta_radius`. Mirror `test_shape_arc.cpp` here.

**`src/geometry/math.rs`.** Needs an angle type. Note 06 section 9.6 and note 08 section 8 both concluded that `EDA_ANGLE` and `RotatePoint` are arc only; this is where that debt comes due. Add a `Degrees(f64)` newtype with KiCad's `Normalize` (into `[0, 360)`) and `Normalize180` semantics (`libs/kimath/include/geometry/eda_angle.h`), and `rotate_point( p, center, angle )` in `f64`.

**`src/geometry/line_chain.rs`.** The largest change. New fields `shapes: Vec<ArcRef>` and `arcs: Vec<ShapeArc>`; new methods `arc_count`, `arc`, `arc_index`, `is_pt_on_arc`, `is_arc_segment`, `is_arc_start`, `is_arc_end`, `is_shared_pt`, `next_shape`, `shape_count`, `remove_shape`, `clear_arcs`, and the private `convert_arc`, `split_arc`, `amend_arc`, `fix_indices_rotation`, `merge_first_last_point_if_needed`, `live_arcs`. Arc behaviour into the existing `append` (`:556`), `append_chain` (`:584`), `insert` (`:609`), `remove` (`:629`), `remove_range` (`:649`), `replace` (`:679`), `replace_with_chain` (`:709`), `slice` (`:776`), `split` (`:831`), `set_point` (`:913`), `reverse` (`:871`), `reversed` (`:883`), `mirror` (`:931`), `move_by` (`:894`), `length` (`:976`), `simplify` (`:1126`), `simplify2` (`:1224`), `remove_duplicate_points` (`:1311`), `collide_point` (`:1745`), `collide_seg` (`:1808`), `nearest_point` (`:1907`). New overload `append_arc( &ShapeArc, max_error )`.

Deliberately **unchanged and polyline only**, matching KiCad: `intersect_seg` (`:1396`), `intersect_chain` (`:1475`), `intersects_chain` (`:1629`), `self_intersecting` (`:1662`), `point_along` (`:1052`), `path_length` (`:1001`), `point_inside` (`:2037`), `area` (`:2205`), `split_three_way` (`:2274`). Put a doc comment on each saying so and citing section 2.5, because a future reader will otherwise "fix" one of them.

Not ported at all: KiCad's `Format`, `Parse`, `CompareGeometry`, `Rotate`, `SelfIntersectingWithArcs` (E15, E16), which have no router caller.

**`src/geometry/shape.rs`.** `ShapeKind::Arc` and `Shape::Arc(ShapeArc)`. Touches `kind` (`:357`), `is_solid` (`:380`), `bbox` (`:411`), `center` (`:460`), `move_by` (`:475`), `subshapes` (`:508`).

**`src/geometry/collision.rs`.** Six new arms in `collide_single` (`:486`) and the corresponding rows in `collide_shapes` (`:399`): arc against circle, rect, segment, simple, line chain and arc. Each begins with the `is_effective_line` hand off to a segment (section 6). MTV for the circle, rect and arc rows only, with the `+ 3` of `shape_collisions.cpp:626`; the other three return no MTV, matching KiCad's assertion. `collide_point` (`:341`) and `collide_seg` (`:365`) gain an arc arm.

**`src/geometry/direction45.rs`.** `CornerMode` gains `Rounded45 = 1` and `Rounded90 = 3` in the discriminant gaps the current enum already leaves (`:240` to `:245`). `build_initial_trace` gains the two branches of section 3, including the `MIN_PRECISION_IU` re-snap with its sign flip (E18) reproduced deliberately or fixed deliberately, logged either way. New `Direction45::from_arc( &ShapeArc, ninety_deg )` (section 3.3). New tests: there is no KiCad counterpart (note 01 section 15), so this is where the port's own table goes.

**`src/geometry/hull.rs`.** New `arc_hull( &ShapeArc, clearance, walkaround_thickness )` reproducing `pns_utils.cpp:71` to `:154` including both roundings of E38, and returning a `Result` rather than dereferencing the mitre intersections blind (E21). `build_hull_for_primitive_shape` (`:644`) gains the `Shape::Arc` row.

**`src/item.rs`.** `ItemBody::Arc( Arc )` with `pub struct Arc { arc: ShapeArc }` next to `Segment` (`:837`), `Via` (`:1015`), `Solid` (`:1459`) and `Hole` (`:1630`). `Kind::ARC` **already exists** with KiCad's bit value (`src/item.rs:173`, `Kind(16)`), so every kind mask in the crate is already correct and none of them changes. The item level `shape`, `hull`, `anchor`, `anchor_count`, `width`, `set_width` and `changed_area` gain an arm each.

**`src/node.rs`.** `add_arc` and `do_add_arc` next to `add_segment` (`:870`) and `do_add_segment` (`:1092`); `remove_arc_index` next to `remove_segment_index` (`:1262`); `find_redundant_arc` next to `find_redundant_segment` (`:1145`), **with the mid point in the comparison** (E8 fixed, logged). `add_line` (`:2354`) gains the arc loop, **in chain order rather than KiCad's arcs first order** (E22 fixed, logged). `assemble_line` (`:2524`) and `follow_line` (`:2623`) gain the reversed flag and the `append_arc` call, using an in place reverse rather than a rebuild (E4 fixed, logged). `nearest_obstacle` (`:1984`) extends its corner mode match with the two rounded variants.

**`src/topology.rs`.** The `SEGMENT | ARC` kind masks are already written that way, because `Kind::ARC` already exists. `assemble_diff_pair`'s `find_n_item` gains the arc branch, with E9's 5 nm threshold either reproduced or widened by an explicit decision.

**`src/line.rs`.** `arc_count`; `restore_untouched_arcs` called from the tail of `walkaround`; the arc guards in `drag_corner` (`:1023`) and `drag_segment` (`:1266`); the arc bail in `clip_to_nearest_obstacle`; `clip_vertex_range`'s shape walk; and `drag_arc`, which is note 06 section 3.5 and is the largest single new function in the milestone.

**`src/placer/line_placer.rs`.** `reduce_tail` (`:592`) and `merge_head` (`:680`) gain the direction from arc branches, using the correct predicate (E11 fixed, logged). The pullback's `remove_shape(-1)`. `build_initial_line` (`:884`) passes the corner mode through, forces `Mitered45` in ortho mode, and the 90 mode hull bounding box snap in `rh_mark_obstacles` (`:1558`). `fix_route` (`:3033`) gains the arc emission loop and the `fix_all` override, **with E24 fixed**. New `split_adjacent_arcs` next to `split_adjacent_segments` (`:2582`). `simplify_new_line` (`:3607`) needs its neighbour handling to work on `LinkedItem` rather than assuming a segment (E25 is unrepresentable in Rust, so this is free).

**`src/shove.rs`.** `shove_obstacle_line` (`:2264`) gains the arc clearance bump, **per hull rather than accumulating** (E26 fixed, logged), which retires the `TODO(arcs)` at `:2261`. New `on_colliding_arc` next to `on_colliding_segment` (`:2484`). `shove_iteration` (`:3527`) and `reverse_collision` (`:3674`) gain the two arc arms, retiring the `TODO(arcs)` at `:3526` and `:3738`; the reverse arm should be written as the segment arm's twin (E27 fixed, logged).

**`src/optimizer.rs`.** A `has_arcs` gate over `merge_full` (`:2196`), `merge_obtuse` (`:2406`), `run_smart_pads` (`:3322`) and `fanout_cleanup` (`:3412`) in `optimize_line` (`:1849`). `merge_colinear` (`:2523`) gains the `is_pt_on_arc` guard and the zero length skip. `drag_fix_corner` (`:1969`) and `drag_fix_corners` (`:2091`) gain their guards. `merge_step` (`:2290`) does **not** need one (E28).

**`src/walkaround.rs`.** Only the corner mode bounding box substitution, which is already keyed on `CornerMode` and extends by two match arms.

**`src/dragger.rs`.** `DragMode::Arc` (`:129`), `start_drag_arc` next to `start_drag_segment` (`:544`), and the three arc arms in `drag_mark_obstacles` (`:765`), `drag_walkaround` (`:909`) and `drag_shove` (`:1058`). Note 06 section 2.6 has the shape, including the tangent stub trick and its erratum E12 there.

**`src/component_dragger.rs`.** One arc arm in the per item move.

**`src/diff_pair.rs`.** `coupled_segment_pairs` gains the arc skip on both lanes.

**`src/placer/diff_pair_placer.rs`.** `get_dangling_anchor` gains the arc arm.

**`src/meander.rs`.** `CornerStyle` gains `Round` and stops being a one variant `#[non_exhaustive]` enum (`:280` to `:287`); `MeanderSettingsError::RoundCornersUnsupported` (`:438`) and its branch in `MeanderSettings::new` (`:600` to `:602`) are deleted; `make_miter_shape` (`src/meander.rs:1024`) gains its round branch, retiring the doc comment at `:1012` that records the gap. `MeanderShape::make_arc` and `MeanderedLine::add_arc` arrive; note 08 errata E4 and E5 say to keep `MT_ARC` and the two unused `Add*` wrappers out.

**`src/placer/meander_placer.rs`.** The arc passthrough in `do_move`, **without E29's index skip**.

**`src/placer/dp_meander_placer.rs`.** `add_corners_until_index`'s four way branch (note 08 section 6.6).

**`src/settings.rs`.** Nothing beyond the two new `CornerMode` variants being accepted; the JSON round trip already carries the full range.

**`src/snapshot.rs`.** `WorldGeometry` (`:248`) gains an `Arc { start, mid, end, width }` variant, and the world builder an arm.

**`src/router.rs`.** `NewGeometry` (`:718`) gains `Arc { start, mid, end, width }`. `PreviewItem` (`:434`) already carries a `LineChain`, which will simply contain arcs once `LineChain` can, so **no change is needed there**, but the host has to be told that a preview chain can now hold arcs and that drawing the polyline alone is a valid, slightly wrong, fallback.

**`src/eventlog.rs`.** `CornerStyle::Round` in the name table (`:1155`), the two new `CornerMode` names, and an arc case in the golden geometry records. Use KiCad's log form, `{ start, mid, end, width }` (`pns_logger.cpp:245`), not `SHAPE_LINE_CHAIN::Format`'s (E15).

**LibrePCB's FFI, `libs/librepcb/rust-core/src/ffi/router_ffi.rs`.** `PnsShapeKind` (`:86`) gains `Arc = 4` and `PnsShape` (`:119`) gains a `mid: PnsPoint` field, reusing `p1` and `p2` as start and end. `PnsNewGeometryKind` (`:1604`) gains `Arc = 2` and `PnsNewItem` (`:1784`) gains `mid`. A new `PnsArcGeometry` next to `PnsSegmentGeometry` (`:176`) for the snapshot side. `PnsPreviewItem` (`:1716`) carries a point list today, so the C++ side either flattens arcs before drawing or gains a parallel arc list; flattening at `maxArcTolerance()` (5000 nm, `boarddesignrulecheck.h:172`) matches what LibrePCB's own DRC does and is the cheaper first step.

**What LibrePCB cannot use yet.** `Trace` serialises no angle (`trace.cpp:236`), so a committed arc has nowhere to go. Until the file format carries one, the honest integration is: the engine may carry arcs that came from the host, and the host does not enable a rounded corner mode. That is a one line refusal at the settings boundary, exactly like the meander round style refusal the crate already has (`src/meander.rs:600`).

---

## 12. A recommended slice order

Each slice is independently reviewable, leaves the crate green, and has an exit criterion that can fail.

**Slice 1: `ShapeArc` as a value type.** `src/geometry/arc.rs` and the angle helpers in `src/geometry/math.rs`. The three points, the two named constructors, `calc_arc_center` both overloads, `is_ccw`, `is_effective_line`, `center`, `radius`, both endpoint angles, `central_angle`, `length`, `bbox`, `chord`, `nearest_point`, `slice_contains_point`, `convert_to_polyline`, `arc_to_segment_count`, `move_by`, `mirror`, `reverse`, `reversed`. No `LineChain` change, no `Shape` variant, nothing else in the crate touched.

*Exit:* the twelve `test_shape_arc.cpp` cases of section 8.4 items 1 and 2 pass, including all six `CalcArcCenter` cases. Write the E1 decision into `doc/log/` before the commit.

**Slice 2: `ShapeArc::collide_point` and `collide_seg`.** The two candidate point routines of section 1.6, plus `nearest_point` and the four `nearest_points` overloads. Still no `LineChain` change.

*Exit:* `CollidePt` (`test_shape_arc.cpp:742`), `CollideSeg` (`:799`), `CollideArc` (`:948`), `CollideCircle` (`:649`) and `CollideNearlyFlatArcDoesNotOverflow` (`:1305`) pass. Decide E2 and E3 explicitly and log both.

**Slice 3: arcs inside `LineChain`.** The `shapes` and `arcs` fields, `ArcRef` with its role, the five predicates, `next_shape`, `shape_count`, `convert_arc`, `split_arc`, `amend_arc`, `remove_shape`, `live_arcs`, and arc behaviour in `append_arc`, `append_chain`, `insert`, `remove`, `remove_range`, `replace_with_chain`, `slice`, `split`, `set_point`, `reverse`, `mirror`, `length`, `simplify`, `simplify2`, `remove_duplicate_points`, `collide_point`, `collide_seg`, `nearest_point`. The largest slice; consider splitting the mutators from the queries.

*Exit:* the eleven `test_shape_line_chain.cpp` cases of section 8.4 item 4 pass, plus a property test that `append_arc` then `slice(0, n-1)` preserves the arc and that `reverse` twice is the identity on points, shapes and arcs. E5, E6, E7, E10, E12, E13, E14 each get a test naming them, whether they are reproduced or fixed.

**Slice 4: `Shape::Arc`, the collision rows and `arc_hull`.** The `Shape` variant, the six rows in `src/geometry/collision.rs`, `arc_hull` in `src/geometry/hull.rs`, the `build_hull_for_primitive_shape` row.

*Exit:* `CollideArcToShapeLineChain` (`test_shape_arc.cpp:990`) and `CollideArcToPolygonApproximation` (`:1015`) pass, plus a new hull table checking that `arc_hull` is clockwise, encloses the arc at the requested clearance, and reproduces `pns_utils.cpp:86`'s truncation exactly.

**Slice 5: `ItemBody::Arc` and the world.** `src/item.rs`, `src/node.rs` (`add_arc`, `remove_arc_index`, `find_redundant_arc`, the arc loop in `add_line`, the arc branch of `assemble_line` and `follow_line`), `WorldGeometry::Arc` in `src/snapshot.rs`. The kind masks in `src/topology.rs` and `src/joint.rs` need nothing, `Kind::ARC` being already present. No algorithm changes yet, so an arc in the snapshot is an obstacle and nothing more.

*Exit:* a scenario test that builds a world containing an arc track, queries `nearest_obstacle` against a line that crosses it, assembles the line the arc belongs to and gets the arc back with its three points intact, and commits a diff that round trips the arc unchanged. This is the first point at which `stickhub-extra-via.kicad_pcb` can be loaded, so add it as a fixture here.

**Slice 6: `CornerMode::Rounded45` and `Rounded90`.** `build_initial_trace`'s two branches, `Direction45::from_arc`, the corner mode branches in `src/node.rs`, `src/walkaround.rs`, `src/optimizer.rs` and `src/placer/line_placer.rs`, and the `has_arcs` gate in `src/optimizer.rs`. The placer can now produce arcs and the walkaround can now route around them, because the walkaround is polyline only (section 5.4) and needs nothing.

*Exit:* the crate's own `build_initial_trace` table (section 8.4 item 6) passes for all four modes, and a scenario test routes a two segment trace in `Rounded45` and in `Rounded90` between two pads with an obstacle between them, in walkaround mode, and commits an arc. E17, E18 and E19 each get a decision and a test. KiCad has no test that covers this, so the table is the port's own contract.

**Slice 7: the placer's commit path and the shove.** `fix_route`'s arc emission with E24 fixed, `split_adjacent_arcs`, `reduce_tail` and `merge_head`'s direction branches with E11 fixed, then `shove_obstacle_line`'s per hull clearance bump with E26 fixed, `on_colliding_arc`, and the two arc arms of `shove_iteration` with E27 fixed. Retires all six `TODO(arcs)` markers.

*Exit:* a scenario test routes in `Rounded45` in shove mode against an existing arc track and pushes it aside, and a second one routes an arc, straight, arc chain and checks that all three items reach the commit diff (the E24 regression). Replay the four resolving regression cases and check they are unchanged, since they contain no arcs and must not move.

**Slice 8: the dragger, the meanders and the host.** `drag_arc` and `DragMode::Arc` with note 06 section 3.5's geometry; the component dragger's arc arm; `CornerStyle::Round` and `make_miter_shape`'s round branch with the `RoundCornersUnsupported` refusal deleted; the meander placers' arc passthrough with E29 fixed; `NewGeometry::Arc`; the FFI additions of section 11.3.

*Exit:* a drag of an arc track in all three modes reaches a committed geometry, `test_meander_corner_radius.cpp:38`'s defaults are reproduced with `Round` as the default, a tuned trace in the round style hits its target length within the tolerance the chamfered style already meets in `tests/meander_placer.rs`, and LibrePCB builds against the new FFI with arcs refused at the settings boundary (section 11.3's last paragraph).

**What is deliberately not in the order.** `SHAPE_ARC::Intersect` and `IntersectLine` (no router caller, section 1.7), `SelfIntersectingWithArcs` (E16), `SHAPE_LINE_CHAIN::Format` and `Parse` (E15), `Rotate`, `ClearArcs`, `CompareGeometry`, `ARC::CLine` (E20), `NODE::Dump` (E23), the `SHAPE_ARC( SEG, SEG, radius, width )` constructor, and `MT_ARC` (note 08 erratum E4). None of them is reachable from the router, and each would need tests of its own.
