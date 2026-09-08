# KiCad PNS router: the kimath geometry foundation

Reference architecture note for a from-scratch Rust reimplementation of KiCad's push-and-shove router.

Source: `/home/Tubbles/dev/ref/kicad` at commit `302b2ba1014b2f116ab38d69ffa8c6d1c633ed85` (2026-09-07). All `path:line` citations below are relative to that root.

Scope: the geometry primitives the router *core* depends on. The router core is every file in `pcbnew/router/` except the KiCad-GUI glue (`router_tool.cpp`, `router_preview_item.cpp`, `router_status_view_item.cpp`, `pns_kicad_iface.cpp`, `pns_tool_base.cpp`). Where a glue file is the only user of something, that is called out explicitly.

Units: all integer coordinates are PCB internal units = nanometres (`include/base_units.h:68`, `constexpr double PCB_IU_PER_MM = 1e6;`).

Checkout caveat: this is a sparse checkout containing only `libs/`, `pcbnew/`, `qa/` and `thirdparty/`. Citations into `include/`, `common/` and `qa/tests/libs/kimath/` were read out of git at the same commit (`git show HEAD:<path>`, `git ls-tree`) and are noted where that matters.

## 1. What the router core actually uses

### 1.1 Header dependency counts (router core only)

Counted by `#include <geometry/...>` / `#include <math/...>` occurrences across the 76 core files, descending: `math/vector2d.h` 26, `geometry/shape_line_chain.h` 24, `math/box2.h` 11, `geometry/shape.h` 9, `geometry/shape_rect.h` 6, `geometry/seg.h` 5, `geometry/shape_poly_set.h` 4, `geometry/direction45.h` 4, `geometry/shape_simple.h` 3, `geometry/shape_segment.h` 3, `geometry/shape_index.h` 3, `geometry/shape_compound.h` 3, `geometry/shape_circle.h` 3, `geometry/shape_arc.h` 3, `geometry/shape_index_list.h` 1, `geometry/shape_ellipse.h` 1, `geometry/eda_angle.h` 1, `geometry/circle.h` 1.

Notably absent: `geometry/convex_hull.h` (the router has its own `PNS::ConvexHull`, see 12.3), `geometry/shape_null.h`, `geometry/polygon_triangulation.h`, `geometry/intersection.h`, `geometry/nearest.h`, `geometry/shape_utils.h`.

### 1.2 Type usage table

"Rough count" = textual identifier occurrences across the router core; treat as an order-of-magnitude signal, not an exact call count.

| Type | Defined in | Rough count | Heaviest users |
|---|---|---|---|
| `VECTOR2I` | `libs/kimath/include/math/vector2d.h:683` | 689 | `pns_diff_pair.cpp` 63, `pns_line.cpp` 54, `pns_utils.cpp` 41, `pns_optimizer.cpp` 38, `pns_line_placer.cpp` 35 |
| `SHAPE_LINE_CHAIN` | `libs/kimath/include/geometry/shape_line_chain.h` | 341 | `pns_optimizer.cpp` 82, `pns_line_placer.cpp` 31, `pns_line.cpp` 29, `pns_optimizer.h` 22 |
| `DIRECTION_45` | `libs/kimath/include/geometry/direction45.h:36` | 261 | `pns_line_placer.cpp` 57, `pns_optimizer.cpp` 49, `pns_line.cpp` 34, `pns_diff_pair.cpp` 27 |
| `SEG` | `libs/kimath/include/geometry/seg.h:37` | 231 | `pns_line.cpp` 48, `pns_optimizer.cpp` 37, `pns_diff_pair.cpp` 32, `pns_utils.cpp` 19 |
| `SHAPE_ARC` | `libs/kimath/include/geometry/shape_arc.h:35` | 44 | `pns_meander.cpp` 9, `pns_meander.h` 8, `pns_arc.h` 6 |
| `SHAPE` (base) | `libs/kimath/include/geometry/shape.h:123` | 40 | `pns_index.h` 5, `pns_item.cpp` 3, `pns_optimizer.cpp` 3 |
| `VECTOR2D` | `libs/kimath/include/math/vector2d.h:682` | 34 | `pns_meander.cpp` 16, `pns_meander.h` 8 |
| `BOX2I` | `libs/kimath/include/math/box2.h:927` | 28 | `pns_optimizer.h` 4, `pns_router.h` 3, `pns_line.cpp` 3 |
| `SHAPE_CIRCLE` | `libs/kimath/include/geometry/shape_circle.h:33` | 19 | `pns_hole.cpp` 6, `pns_via.h` 5 |
| `SHAPE_RECT` | `libs/kimath/include/geometry/shape_rect.h:34` | 15 | `pns_utils.cpp` 5, `pns_optimizer.cpp` 4 |
| `SHAPE_SEGMENT` | `libs/kimath/include/geometry/shape_segment.h:33` | 14 | `pns_utils.cpp` 4, `pns_segment.h` 3 |
| `EDA_ANGLE` | `libs/kimath/include/geometry/eda_angle.h:36` | 11 | `pns_dragger.cpp` 3, `pns_optimizer.cpp` 3, `pns_solid.h` 3 |
| `SHAPE_SIMPLE` | `libs/kimath/include/geometry/shape_simple.h:37` | 6 | `pns_utils.cpp` 3 |
| `SHAPE_COMPOUND` | `libs/kimath/include/geometry/shape_compound.h:34` | 4 | `pns_hole.cpp` 2, `pns_solid.cpp` 2 |
| `CIRCLE` | `libs/kimath/include/geometry/circle.h:32` | 3 | `pns_line.cpp` 2 |
| `SHAPE_POLY_SET` | `libs/kimath/include/geometry/shape_poly_set.h` | 2 | `pns_hole.cpp` 1, `pns_solid.cpp` 1 |
| `SHAPE_ELLIPSE` | `libs/kimath/include/geometry/shape_ellipse.h:33` | 1 | `pns_utils.cpp` (hull dispatch only) |
| `SHAPE_NULL` | `libs/kimath/include/geometry/shape_null.h` | 0 | unused by the router |

The shape of this table is the headline finding: **the router is 95% `VECTOR2I` + `SEG` + `SHAPE_LINE_CHAIN` + `DIRECTION_45`.** The full `SHAPE` polymorphic hierarchy is touched only at the boundary (obstacle items carry a `const SHAPE*` supplied by the board interface) and at hull construction time. `SHAPE_POLY_SET` shows up in exactly two functions.

### 1.3 Method usage, `SEG`

Counts include textual collisions with same-named methods on other types (`Length`, `Reverse`, `Angle`, `Contains`, `NearestPoint`, `Intersect`, `Distance`, `SquaredDistance` are all shared names).

`Length` 61 (`seg.h:339`), `Reverse` 24 (`seg.h:364`), `IntersectLines` 22 (`seg.h:216`), `Angle` 22 (`seg.h:163`, `seg.cpp:107`), `LineProject` 21 (`seg.h:131`, `seg.cpp:681`), `NearestPoint` 18 (`seg.h:170`/`:177`, `seg.cpp:629`/`:116`), `Contains` 17 (`seg.h:362`/`:320`, `seg.cpp:623`), `Side` 12 (`seg.h:139`), `SquaredDistance` 8 (`seg.h:249`/`:259`), `Intersect` 7 (`seg.h:205`, `seg.cpp:442`), `ApproxParallel` 7 (`seg.h:294`, `seg.cpp:803`), `Collinear` 6 (`seg.h:282`), `Distance` 4 (`seg.h:257`/`:267`), `LineDistance` 4 (`seg.h:155`, `seg.cpp:742`), `SquaredLength` 3 (`seg.h:344`), `Reversed` 2 (`seg.h:369`). Heaviest users are `pns_optimizer.cpp`, `pns_line.cpp`, `pns_diff_pair.cpp`, `pns_utils.cpp`, `pns_topology.cpp`.

`SEG::Collide` (`seg.h:247`, `seg.cpp:538`) is never called directly; it is reached through `SHAPE::Collide`. Unused by the router: `ApproxCollinear`, `ApproxPerpendicular`, `PerpendicularSeg`, `ParallelSeg`, `Center`, `ReflectPoint`, `NearestPoints`, `IntersectsLine`, `Overlaps`, `TCoef`, `CanonicalCoefs`, `Index`, `Square`.

### 1.4 Method usage, `VECTOR2I` and `BOX2I`

`VECTOR2I`: `Resize` 49 (`vector2d.h:186`, impl `:381`), `EuclideanNorm` 47 (`:161`, impl `:279`), `Format` 32 (`:193`, debug only), `SquaredEuclideanNorm` 23 (`:170`, impl `:303`), `Perpendicular` 19 (`:178`, impl `:310`), `Dot` 2 (`:203`), `Cross` 1 (`:198`). There is no member `Rotate`; the free `RotatePoint` in `trigo.h` is used instead.

`BOX2I`: `GetWidth` 25 / `GetHeight` 16 (`box2.h:211`/`:212`, 13 each in `pns_utils.cpp`), `Contains` 17 (`box2.h:165`/`:191`/`:198`), `Merge` 12 (`box2.h:653`/`:687`), `GetCenter` 11 (`box2.h:227`), `SquaredDistance` 8 (`box2.h:781`/`:808`), `GetOrigin`/`GetX`/`GetY` 6 each (`box2.h:207`/`:204`/`:205`, all in `pns_utils.cpp`), `GetEnd` 5 (`box2.h:214`), `GetSize` 4 (`box2.h:203`), `GetLeft`/`GetRight`/`GetTop`/`GetBottom` 4 each, `GetPosition` 3 (`box2.h:208`), `Inflate` 2 (`box2.h:553`/`:624`), `Intersects` 1 (`box2.h:308`), `Normalize` 1 (`box2.h:143`), `SetMaximum` 1 (`box2.h:77`, in `pns_router.cpp`).

### 1.5 Method usage, `SHAPE_LINE_CHAIN`

Full detail in section 5. Highest-traffic members, ordered:

`SetWidth`/`Width` (32/78, but `Width` collides textually with `PNS::LINE::Width`), `Append` 170, `SegmentCount` 138, `PointCount` 131, `CPoint` 125, `CSegment` 125, `CLastPoint` 76, `Clear` 65 (shared name), `Remove` 49 (shared name), `Simplify` 36, `Clone` 31 (shared name), `Collide` 28, `Reverse` 24, `Slice` 20, `Simplify2` 18, `NearestPoint` 18, `Find` 16, `IsArcSegment` 16, `Replace` 15, `Split` 13, `ArcIndex` 12, `IsPtOnArc` 12, `Arc` 12, `Insert` 9, `SetClosed` 9, `Intersect` 7, `ShapeCount` 6, `NextShape` 6, `ArcCount` 5, `PointInside` 5, `PointOnEdge` 5, `Move` 5, `SelfIntersecting` 4, `Area` 4, `CompareGeometry` 3, `PathLength` 2, `CPoints` 2, `RemoveShape` 1, `NearestSegment` 1, `IsClosed` 1, `Mirror` 1.

Never called by the router: `Point` (non-const), `FindSegment`, `EdgeContainingPoint`, `SelfIntersectingWithArcs`, `GenerateBBoxCache`, `Rotate`, `OffsetLine`, `ClearArcs`, `IsArcStart`, `IsArcEnd`, `IsSharedPt`, `Convert`.

### 1.6 Method usage, `DIRECTION_45`

The enum `CORNER_MODE` (`direction45.h:66`) is referenced 48 times, more than any method. Then `Angle(DIRECTION_45)` 22 (`direction45.h:181`), `BuildInitialTrace` 19 (`direction45.h:234`, impl `src/geometry/direction_45.cpp:24`), `ToVector` 14 (`direction45.h:287`), `IsDiagonal` 11 (`direction45.h:213`), `Left` 8 / `Right` 8 (`direction45.h:269`/`:251`), `IsObtuse` 7 (`direction45.h:203`), `Opposite` 1 (`direction45.h:170`), `Format` 32 (debug). The `AngleType` values are referenced individually: `ANG_OBTUSE` 11 (`direction45.h:79`), `ANG_RIGHT` 10 (`:80`), `ANG_HALF_FULL` 10 (`:83`), `ANG_ACUTE` 9 (`:81`), `ANG_STRAIGHT` 8 (`:82`), `UNDEFINED` 10 (`:59`). Unused: `Mask` (`:305`), `IsDefined` (`:218`).

## 2. Numeric foundation

### 2.1 `VECTOR2<T>` and the `extended_type` (ECOORD) trait

`VECTOR2<T>` is a plain `{ T x, y; }` (`libs/kimath/include/math/vector2d.h:73`) with a traits-provided widening type.

* `VECTOR2_TRAITS<T>::extended_type` defaults to `T` (`vector2d.h:43`), specialized to `int64_t` for `int` (`vector2d.h:49`).
* `typedef typename VECTOR2_TRAITS<T>::extended_type extended_type;` (`vector2d.h:69`), `typedef T coord_type;` (`vector2d.h:70`).
* `static constexpr extended_type ECOORD_MAX / ECOORD_MIN` (`vector2d.h:72-73`) are `INT64_MAX` / `INT64_MIN` for `VECTOR2I`. The router uses `VECTOR2I::ECOORD_MAX` as a "no result yet" sentinel throughout.
* Instantiations: `VECTOR2D = VECTOR2<double>` (`vector2d.h:682`), `VECTOR2I = VECTOR2<int32_t>` (`vector2d.h:683`), `VECTOR2L = VECTOR2<int64_t>` (`vector2d.h:684`). The router uses only the first two directly; `VECTOR2L` appears inside `SEG`.
* `SEG::ecoord` is an alias for `VECTOR2I::extended_type` (`geometry/seg.h:40`), `SHAPE::ecoord` is the same (`geometry/shape.h:299`).

Where widening is and is not done:

* Exact (widened before multiply): `SquaredEuclideanNorm` (`vector2d.h:303-306`), `Cross` (`vector2d.h:534-538`), `Dot` (`vector2d.h:542-546`), `SquaredDistance` (`vector2d.h:558+`), `operator*(VECTOR2, VECTOR2)` which is a dot product returning `extended_type` (`vector2d.h:502-507`).
* **Not widened**: `operator+`/`operator-` between two `VECTOR2I` produce `VECTOR2<common_type_t<T,U>>` computed in 32-bit (`vector2d.h:436-441`, `:466-471`). `operator*(VECTOR2, scalar)` likewise (`vector2d.h:510-514`). Adding two coordinates near `INT32_MAX` silently wraps. This bites in midpoint expressions like `( a + b ) / 2` used at `shape_collisions.cpp:55`, `:618`, `:745`, `:877`.
* Cross-type construction *does* saturate: the converting constructor clamps through `int64_t` (`vector2d.h:81-108`) and the `operator()<U>()` cast does the same (`vector2d.h:124-140`).

`EuclideanNorm()` (`vector2d.h:279-299`) is float-backed with three special cases: `|x| == |y|` uses `|x| * M_SQRT2`, `x == 0` or `y == 0` returns the other absolute value, otherwise `KiROUND(std::hypot(x, y))`. The 45 degree special case exists because KiCad boards are full of exact diagonals and `hypot` would round inconsistently there.

`Resize(T aNewLength)` (`vector2d.h:381-413`) returns `(0,0)` for a zero vector, uses `|x| == |y| -> |len| * M_SQRT1_2` for exact diagonals, otherwise `sqrt(rescale(len^2, x^2, l^2))` per component with sign preserved, then multiplied by `sign(aNewLength)`. **A negative length reverses the direction.** This function is called 49 times in the router core and is the workhorse of every hull builder.

`Perpendicular()` returns `(-y, x)` (`vector2d.h:310-314`), i.e. a 90 degree counter-clockwise rotation in math coordinates (clockwise on screen, since y is down).

`std::hash<VECTOR2I>` is explicitly `= delete`d (`vector2d.h:708`) with a comment telling callers to use ordered containers instead; `std::less<VECTOR2I>` is provided (`vector2d.h:715`).

### 2.2 Rounding and rescaling

* `KiROUND<fp_type, ret_type>(v)` (`math/util.h:98-124`): `std::llround` (half away from zero), then `std::clamp` to the target type's range, logging an overflow when clamping fired. NaN returns 0 but only under C++23 (`math/util.h:102-113`), so on a C++20 build a NaN reaches `llround` with implementation-defined results.
* `KiCheckedCast<in,ret>` (`math/util.h:65-90`) clamps `long long -> int` and logs.
* `rescale(T num, T val, T den)` generic is `num * val / den` (`math/util.h:135-138`), i.e. **truncating and overflow-prone**. The `int` specialization (`src/math/util.cpp:62-72`) widens to `int64_t` and rounds to nearest. The `int64_t` specialization (`src/math/util.cpp:76-113`) uses `__int128` where available, `_mul128`/`_div128` on MSVC x64, and a decomposed fallback otherwise. **Any Rust port of `rescale(i64,i64,i64)` needs `i128` intermediates.**
* `sign(T)` is `(0 < val) - (val < 0)` (`math/util.h:141-144`).
* Integer square root: `isqrt<T>` at `src/geometry/seg.cpp:57-72`, seeded from `std::sqrt((double) x)` and corrected by an increment/decrement loop, clamped to a compile-time `ct_sqrt(numeric_limits<T>::max())` (`seg.cpp:44-54`). It **truncates**. `SEG::Distance` uses it (`seg.cpp:698-707`), but the shape-level collision code uses `double sqrt` instead, so three rounding conventions coexist (see 9.6).

### 2.3 `EDA_ANGLE`

`EDA_ANGLE` (`geometry/eda_angle.h:36`) stores a `double` in **degrees** (`eda_angle.h:116`), with `DEGREES_TO_RADIANS = M_PI / 180.0` (`eda_angle.h:122`). Constructors accept degrees, radians, or tenths of a degree (`eda_angle.h:30`, `:51-59`). The router touches it in only 11 places, all through `SEG::Angle()`, `SHAPE_ARC::GetCentralAngle()`, and `SOLID`'s orientation field. It is not on any hot path.

## 3. `SEG`

Declared `libs/kimath/include/geometry/seg.h:37`. Layout is `{ VECTOR2I A; VECTOR2I B; int m_index; }` (`seg.h:45-46`, `:398`), i.e. 20 bytes. `A` and `B` are deliberately public (`seg.h:43-44`).

`m_index` is a back-reference to the segment's position in a parent shape, set by the 3-argument constructor (`seg.h:83-88`) and read via `Index()` (`seg.h:357`). `SHAPE_LINE_CHAIN::Segment` populates it (`shape_line_chain.cpp:1285-1299`). The router core never calls `Index()`, but a Rust port that returns segments by value from a chain should still consider carrying the index, because KiCad code elsewhere depends on it.

### 3.1 Predicates and their exact tolerances

| Function | Definition | Exactness |
|---|---|---|
| `Side(p)` | `seg.h:139-144`: sign of `(B-A).Cross(p-A)`. Negative = left, 0 = on the line, positive = right | exact, `i64` |
| `Contains(VECTOR2I)` | `seg.cpp:623-626`: `SquaredDistance(aP) <= 3` | **tolerance of sqrt(3) nm, hard-coded** |
| `Collinear(SEG)` | `seg.h:282-291`: canonical coefficients, then `abs(d1) <= 1 && abs(d2) <= 1` | **tolerance 1 in the un-normalized determinant, so the effective distance tolerance scales inversely with segment length** |
| `ApproxCollinear(SEG, thr=1)` | `seg.cpp:791-801` via `mutualDistanceSquared` | signed squared perpendicular distances of the shorter segment's endpoints to the longer segment's line, both `<= thr^2` |
| `ApproxParallel(SEG, thr=1)` | `seg.cpp:803-813` | `abs(d1_sq - d2_sq) <= thr^2`; note it compares **signed squared** distances, so this is not a rotation-invariant angular tolerance |
| `ApproxPerpendicular(SEG)` | `seg.cpp:815-820` | `aSeg.ApproxParallel(PerpendicularSeg(A))` |
| `Overlaps` / `Contains(SEG)` | `seg.h:297-332` | `Collinear` + endpoint containment |

`mutualDistanceSquared` (`seg.cpp:762-789`) swaps so that `a` is the longer segment, then computes `d = sgn(det) * rescale(det, det, l)` for each endpoint of `b`. Returns false when the longer segment has zero length.

### 3.2 Distances

`SquaredDistance(VECTOR2I)` (`seg.cpp:710-740`) is the standard point-segment projection in `ecoord`, except the interior case which computes `g = |ap|^2 - (e*e)/f` in `double`, `KiROUND`s back and clamps a negative `g` (a rounding artifact) to 0. `SquaredDistance(SEG)` (`seg.cpp:76-104`) special-cases both degenerate inputs first (the cross-product test in `intersects()` gives false positives for zero-length segments, comment at `seg.cpp:78-80`), returns 0 if `Intersects`, and otherwise takes the minimum over four endpoint projections. `Distance(...)` is `isqrt(SquaredDistance(...))` (`seg.cpp:698-707`), truncating. `LineDistance(p, aDetermineSide=false)` (`seg.cpp:742-760`) measures to the **infinite line**, optionally signed by `sgn(det)`.

### 3.3 Intersection

`intersects(SEG, aIgnoreEndpoints, aLines, VECTOR2I* out)` (`seg.cpp:308-434`) is the single primitive: AABB rejection first (skipped in `aLines` mode, `:312-331`), direction vectors in `VECTOR2L` so the determinant is exact in `i64` (`:333-336`), parallel and collinear handling delegated to `checkCollinearOverlap` (`:216-306`) which returns the **midpoint of the overlap interval** (`:279-289`). The intersection point is computed with `rescale` in `i64` and then **range-checked**: if it does not fit in `int32` the function returns `false` even though an intersection mathematically exists (`seg.cpp:412-430`). That silent failure mode needs a deliberate decision in a port.

`Intersect(aSeg, aIgnoreEndpoints=false, aLines=false)` returns `std::optional<VECTOR2I>` (`seg.h:205`, `seg.cpp:442-450`); `IntersectLines(aSeg)` is `Intersect(aSeg, false, true)` (`seg.h:216-219`) and is called 22 times, mostly to intersect offset lines when building hulls.

`SEG::Collide(SEG, clearance, int* aActual)` (`seg.cpp:538-620`) rejects negative clearance up front, special-cases both zero-length inputs, returns a hit with `*aActual = 0` when the segments intersect, then compares the minimum of four endpoint distances against `clearance_sq` (strict). Note it writes `*aActual` **even when returning false** (`seg.cpp:616-617`), unlike the shape-level `Collide` family. `NearestPoint(VECTOR2I)` (`seg.cpp:629-658`) is manually inlined with `VECTOR2L` arithmetic and returns `A` for a degenerate segment.

## 4. `BOX2<Vec>` and `BOX2I`

`typedef BOX2<VECTOR2I> BOX2I;` (`math/box2.h:927`), also `BOX2D` (`:928`) and `BOX2L` (`:929`). `OPT_BOX2I = std::optional<BOX2I>` (`box2.h:931`), used by `PNS::ChangedArea` (`pns_utils.h:62`).

Layout (`box2.h:919-923`):

```
Vec     m_Pos;   // VECTOR2I  -> i32 origin
SizeVec m_Size;  // VECTOR2<extended_type> -> i64 size !
bool    m_init;  // "has this box ever been assigned"
```

Three things a reimplementer will get wrong:

1. **The size is `i64`, the origin is `i32`.** `typedef typename Vec::extended_type size_type;` then `typedef VECTOR2<size_type> SizeVec;` (`box2.h:48-50`). So a `BOX2I` can represent a box wider than `INT32_MAX` and `GetWidth()` returns `int64_t`. `SetMaximum()` exploits exactly this: `m_Pos = -INT32_MAX`, `m_Size = 2 * INT32_MAX` (`box2.h:87-88`) with the comment "We want to be able to invert the box, so don't use lowest()".
2. **`m_init` is a real tri-state.** `Merge(BOX2I)` and `Merge(VECTOR2I)` treat an uninitialized box as absorbing (`box2.h:655-666`, `:689-694`), and `IsValid()` (`box2.h:914-917`) exposes it. Without this flag, "merge into an empty accumulator" needs a sentinel.
3. **Sizes may be negative** unless `Normalize()` (`box2.h:143-160`) has been called. `Contains` (`box2.h:165-189`) and `Intersects` (`box2.h:308`) handle negative sizes explicitly; `SquaredDistance` (`box2.h:781-790`) does **not** and gives wrong answers on a non-normalized box.

`Inflate(dx, dy)` (`box2.h:553-616`) clamps deflation so the box collapses to zero size rather than inverting.

`Contains` treats edges as inside (`box2.h:161-163` doc). `Intersects` is closed-interval as well (`box2.h:319-334`).

`SquaredDistance(Vec)` (`box2.h:781-790`) returns 0 for a point inside; `SquaredDistance(BOX2I)` at `:808`.

## 5. `SHAPE` hierarchy

### 5.1 The type tag

`enum SHAPE_TYPE` (`geometry/shape.h:41-54`), values are sequential from `SH_RECT = 0`:

| Value | Name | Class |
|---|---|---|
| 0 | `SH_RECT` | `SHAPE_RECT` (`shape_rect.h:34`) |
| 1 | `SH_SEGMENT` | `SHAPE_SEGMENT` (`shape_segment.h:33`) |
| 2 | `SH_LINE_CHAIN` | `SHAPE_LINE_CHAIN` (`shape_line_chain.h:77`) |
| 3 | `SH_CIRCLE` | `SHAPE_CIRCLE` (`shape_circle.h:33`) |
| 4 | `SH_SIMPLE` | `SHAPE_SIMPLE` (`shape_simple.h:37`) |
| 5 | `SH_POLY_SET` | `SHAPE_POLY_SET` |
| 6 | `SH_COMPOUND` | `SHAPE_COMPOUND` (`shape_compound.h:34`) |
| 7 | `SH_ARC` | `SHAPE_ARC` (`shape_arc.h:35`) |
| 8 | `SH_NULL` | `SHAPE_NULL` |
| 9 | `SH_POLY_SET_TRIANGLE` | `SHAPE_POLY_SET::TRIANGULATED_POLYGON::TRI` (`shape_poly_set.h:81`) |
| 10 | `SH_ELLIPSE` | `SHAPE_ELLIPSE` (`shape_ellipse.h:33`) |

The numeric values are part of the debug serialization format (`SHAPE::Format` writes `m_type` as an integer, `src/geometry/shape.cpp:43-48`), so they are not free to renumber.

`SHAPE_BASE` (`shape.h:78`) holds only `SHAPE_TYPE m_type` (`shape.h:117`) plus the indexable-subshape hooks `HasIndexableSubshapes` / `GetIndexableSubshapeCount` / `GetIndexableSubshapes` (`shape.h:106-113`). `SHAPE_COMPOUND` is the only implementor (`shape_compound.h:143-152`).

### 5.2 The `SHAPE` virtual interface

`class SHAPE : public SHAPE_BASE` (`shape.h:123`). `static const int MIN_PRECISION_IU = 4;` (`shape.h:129`).

Pure virtual: `Collide(const SEG&, int, int*, VECTOR2I*)` (`shape.h:213-214`), `BBox(int aClearance = 0)` (`shape.h:223`), `TransformToPolygon` (`shape.h:276`), `Rotate` (`shape.h:282`), `Move` (`shape.h:290`), `IsSolid` (`shape.h:292`).

Defaulted: `Clone()` asserts and returns null (`shape.h:146-150`), `Collide(VECTOR2I, ...)` forwards to `Collide(SEG(aP, aP), ...)` (`shape.h:179-183`), `Centre()` returns `BBox(0).Centre()` (`shape.h:230-233`), `Distance` and `SquaredDistance` (`shape.h:242`, `:247`, implemented in `src/geometry/shape.cpp:105`, `:111`) fall back to polygonizing the shape and measuring against `COutline(0)`, `PointInside` (`shape.h:268`, `shape.cpp:123`) does the same. Those generic fallbacks are extremely slow and exist only so that every shape answers every question.

`GetClearance(const SHAPE*)` (`shape.h:157`, `shape.cpp:94`) calls `Collide(other, INT_MAX/2, &temp_dist)`, which is the origin of the overflow exposure documented in 9.6.

`SHAPE_LINE_CHAIN_BASE : public SHAPE` (`shape.h:303`) adds the vtable that both `SHAPE_LINE_CHAIN` and `SHAPE_SIMPLE` share: `GetPoint`, `GetSegment`, `GetPointCount`, `GetSegmentCount`, `IsClosed`, `GetCachedBBox` (`shape.h:360-366`), plus non-arc-aware `Collide`, `SquaredDistance`, `PointInside`, `PointOnEdge`, `EdgeContainingPoint` (`shape.h:324-358`).

### 5.3 What the router stores

The router never owns a `SHAPE` polymorphically except for `PNS::SOLID` and `PNS::HOLE`:

* `PNS::SEGMENT` holds `SHAPE_SEGMENT m_seg` by value (`pns_segment.h:146`).
* `PNS::ARC` holds `SHAPE_ARC m_arc` by value (`pns_arc.h:119`).
* `PNS::VIA` holds `std::map<int, SHAPE_CIRCLE> m_shapes` keyed by layer (`pns_via.h:349`), for padstacks with per-layer diameters.
* `PNS::LINE` holds `SHAPE_LINE_CHAIN m_line` by value (`pns_line.h:281`).
* `PNS::SOLID` holds a raw owning `SHAPE* m_shape` (`pns_solid.h:158`), deep-copied through `Clone()` in its copy constructor (`pns_solid.h:62-63`) and `delete`d in `SetShape` and the destructor (`pns_solid.h:54`, `:115`).
* `PNS::HOLE` holds a raw owning `SHAPE* m_holeShape` (`pns_hole.h:94`).

`ITEM::Shape(int aLayer)` (`pns_item.h:242`) returns `const SHAPE*` and is the single entry point the index and the collision code use.

## 6. `SHAPE_LINE_CHAIN`

This is the type the router lives in. It is a polyline that can carry true arcs inline, that can be open or closed, and that carries a nominal width. Everything the placer, the optimizer, the shover and the dragger manipulate is a `SHAPE_LINE_CHAIN`.

In this section, `slc.h` = `libs/kimath/include/geometry/shape_line_chain.h` and `slc.cpp` = `libs/kimath/src/geometry/shape_line_chain.cpp`.

### 6.1 Data model

Declared `slc.h:77`, deriving from `SHAPE_LINE_CHAIN_BASE`.

| Member | Type | Declaration |
|---|---|---|
| `m_points` | `std::vector<VECTOR2I>` | `slc.h:973` |
| `m_shapes` | `std::vector<std::pair<ssize_t, ssize_t>>` | `slc.h:989` |
| `m_arcs` | `std::vector<SHAPE_ARC>` | `slc.h:991` |
| `m_accuracy` | `int` | `slc.h:994` |
| `m_closed` | `bool` | `slc.h:997` |
| `m_width` | `int` | `slc.h:1004` |
| `m_bbox` | `mutable BOX2I` | `slc.h:1007` |

`m_accuracy` is dead: initialized to 0 in every constructor (`slc.h:154`, `:164`, `slc.cpp:67`, `:80`, `:92`, `:109`) and in the move assignment (`slc.h:260`), but read nowhere. Omit it in a port.

### 6.2 The arc model, i.e. what `ArcIndex` and `IsArcSegment` actually mean

**`m_shapes` has one entry per point, not per segment**, and `m_shapes.size() == m_points.size()` is a hard invariant asserted in a dozen places (`slc.cpp:149`, `:193`, `:1003`, `:1073`, `:1079`, `:1161`, `:1271`, `:1540`, `:1548`, `:1608`, `:1633`, `:1654`, `:1711`, `:3002`).

Each entry is a pair `(first, second)` of indices into `m_arcs`:

* `SHAPE_IS_PT` is the sentinel `-1`, declared `slc.h:968`, defined `slc.cpp:42`.
* `SHAPES_ARE_PT` is `{-1, -1}`, declared `slc.h:970`, defined `slc.cpp:43`, meaning "this vertex is a plain polyline point".
* A vertex that lies in the interior of arc N has `{N, -1}`.
* A vertex that is simultaneously the **end** of arc N and the **start** of arc N+1 has `{N, N+1}`. This is the "shared point". The documented convention (`slc.h:975-988`) is that `.first` is the arc ending here and `.second` the arc starting here, with the invariant that `.second` must be `-1` whenever `.first` is `-1` (`slc.h:987`, re-established in `convertArc` at `slc.cpp:267-268`).

`ArcIndex(size_t aSegment)` (`slc.h:856-862`) therefore returns `m_shapes[i].second` when `IsSharedPt(i)` and `.first` otherwise. It is **unchecked**: an out-of-range index is UB.

`IsArcSegment(size_t aSegment)` (`slc.h:878`, `slc.cpp:3246-3265`) asks whether the *segment* from point `aSegment` to point `aSegment+1` lies on an arc: `IsPtOnArc(aSegment) && ArcIndex(aSegment) == m_shapes[nextIdx].first`. There is one wrap exception: `nextIdx` becomes 0 when `aSegment+1 == m_shapes.size() && m_closed && IsSharedPt(0)` (`slc.cpp:3257-3258`). The comment at `slc.cpp:3248-3252` gives the reason: two adjacent arcs that do *not* share a vertex have a genuine straight segment between them, and comparing `ArcIndex` on both ends is the only way to tell.

Related predicates: `IsPtOnArc` (`slc.cpp:3240-3243`), `IsSharedPt` (`slc.cpp:3232-3237`), `IsArcStart` (`slc.cpp:3268-3279`), `IsArcEnd` (`slc.cpp:3282-3299`). Note `IsArcEnd(0)` wraps to the last point unconditionally, even for an open chain (`slc.cpp:3286-3287`).

There is an implicit invariant that arcs are stored in `m_arcs` **in chain order**, because `Reverse()` remaps index `i` to `m_arcs.size() - i - 1` after reversing the arc vector (`slc.cpp:918-938`). `Replace(range, chain)` breaks this (see 6.8).

Arc mutators:

* `convertArc(ssize_t)` (`slc.cpp:246-272`) degrades an arc to its polyline: it clears every `m_shapes` reference to that arc, decrements higher references, restores the first/second invariant, and erases from `m_arcs`. **The points stay.**
* `ClearArcs()` (`slc.cpp:949-953`) is `convertArc` for every arc, back to front.
* `amendArc(idx, newStart, newEnd)` (`slc.cpp:275-289`) rebuilds the arc with `ConstructFromStartEndCenter(newStart, newEnd, oldCenter, oldClockwise)`, so **the center and handedness survive and the endpoints move**. Wrappers `amendArcStart` / `amendArcEnd` at `slc.h:928-936`. There is no plural `amendArcs` in this revision.
* `splitArc(ssize_t aPtIndex, bool aCoincident = false)` (`slc.cpp:292-370`) either shortens the preceding arc (leaving a short straight segment, `aCoincident == false`) or splits into two arcs sharing the point (`aCoincident == true`), renumbering all following arc indices.
* `mergeFirstLastPointIfNeeded()` (`slc.cpp:214-243`) folds a duplicated first/last point when closing, and **duplicates point 0 at the end** when opening a chain whose point 0 is shared. This is called by `SetClosed` (`slc.h:291`) and by `Append(SHAPE_LINE_CHAIN)` (`slc.cpp:1606`).
* `fixIndicesRotation()` (`slc.cpp:191-211`) rotates the point/shape arrays so no arc straddles the wrap seam, with a rotation-count guard against infinite loops (`slc.cpp:207-209`).

Arc polygonization accuracy: `getArcPolygonizationMaxError()` (`slc.cpp:57-62`) returns `SHAPE_ARC::DefaultAccuracyForPCB() / 5`, i.e. `ARC_HIGH_DEF / 5` = **1000 nm = 1 um**. It is the default for `Append(SHAPE_ARC)` (`slc.cpp:1614`), `Insert(size_t, SHAPE_ARC)` (`slc.cpp:1660`) and `Slice(int, int)` (`slc.cpp:1414`).

Arc survival per operation:

| Operation | Arcs |
|---|---|
| `Append(VECTOR2I)` | new point is `SHAPES_ARE_PT`, existing arcs untouched (`slc.h:542`) |
| `Append(SHAPE_LINE_CHAIN)` | preserved, other chain's arcs appended and indices offset (`slc.cpp:1555-1570`) |
| `Append(SHAPE_ARC[, int])` | creates an arc, but **only when the polyline has more than 2 points** (`slc.cpp:1622`); a 2-point approximation silently becomes a plain segment |
| `Insert(size_t, VECTOR2I)` | splits any arc at that index first (`slc.cpp:1647-1648`) |
| `Insert(size_t, SHAPE_ARC[, int])` | creates an arc, inserted into `m_arcs` in chain order (`slc.cpp:1671-1698`) |
| `Remove(int, int)` | arc-aware: splits at boundaries, `convertArc`s arcs fully inside the range (`slc.cpp:1099-1157`) |
| `RemoveShape(int)` | removes the whole arc containing the index (`slc.cpp:1380-1409`) |
| `Replace(int, int, VECTOR2I)` | Remove + Insert, so contained arcs are dropped (`slc.cpp:999-1004`) |
| `Replace(int, int, SHAPE_LINE_CHAIN)` | preserved but appended at the end of `m_arcs` (`slc.cpp:1071`), breaking chain order |
| `Slice` | preserved, partial arcs re-cut via `ConstructFromStartEndCenter` (`slc.cpp:1451-1461`, `:1501-1511`) |
| `Reverse` | preserved and individually reversed (`slc.cpp:910-946`) |
| `Simplify(int)` | preserved; an intermediate candidate point on an arc aborts that run (`slc.cpp:2816-2820`) |
| `Simplify2(bool)` | preserved; colinear removal requires both shape entries to be `SHAPES_ARE_PT` (`slc.cpp:2968-2969`) |
| `SetPoint(int, VECTOR2I)` | **destroys** any arc touching that point (`slc.cpp:1371-1376`) |
| `Split(VECTOR2I, bool)` | preserved: inserts the point with the arc index then `splitArc(idx, true)` (`slc.cpp:1219-1224`) |
| `Move` | points and arcs moved, bbox translated (`slc.h:776-783`) |
| `Rotate` / `Mirror` | points and arcs transformed, **bbox not updated** (`slc.cpp:495-502`, `:974-996`) |

The router relies on all of that. `pns_line_placer.cpp:200`, `:207`, `:231`, `:355`, `:365`, `:380` all do `DIRECTION_45( chain.CArcs()[chain.ArcIndex(i)] )`, i.e. take the posture from the true arc rather than from the approximation chord. `pns_line_placer.cpp:244` calls `tail.RemoveShape( -1 )` to pull back exactly one shape, arc or segment, from the tail.

### 6.3 Counting: points, segments, shapes

* `PointCount()` (`slc.h:369-372`) is `m_points.size()`.
* `SegmentCount()` (`slc.h:327-335`) is `max(0, PointCount() - 1 + (m_closed ? 1 : 0))`. **For a closed chain it equals `PointCount()`.** A closed 1-point chain reports 1 segment and `Segment(0)` is the degenerate `SEG(p0, p0)` (`slc.cpp:1295-1296`). The `max(0, ...)` relies on `size_t` underflow producing `-1` when assigned to an `int` first (`slc.h:329`); a Rust port must do this in signed arithmetic.
* `ShapeCount()` (`slc.cpp:1269-1282`) counts an arc as one, by walking `NextShape` from 0.
* `NextShape(int)` (`slc.cpp:1302-1359`) returns the first vertex index of the next shape, or `-1` at the end. It never wraps past the end even for a closed chain except in the `m_closed && !IsArcSegment(lastIndex)` case (`slc.cpp:1350-1355`).

### 6.4 Accessors and mutation

`CPoint(int)` (`slc.h:420-428`) does a **single-step** wrap: one `+= PointCount()` for a negative index, one `-= PointCount()` for an over-range one, so `CPoint(-PointCount()-1)` is out of bounds. `SetPoint` behaves the same (`slc.cpp:1364-1367`), `CLastPoint()` is UB on an empty chain (`slc.h:435-438`), and `Segment(int)` wraps negatives by `SegmentCount()` then `wxCHECK`s, returning `SEG(back, back)` or `SEG(0,0,0,0)` on failure (`slc.cpp:1285-1299`). `Remove`, `Replace`, `Slice` and `RemoveShape` wrap negatives by `PointCount()` (`slc.cpp:1086-1090`, `:1009-1013`, `:1422-1426`, `:1382-1383`). `ArcIndex`, `Arc`, `CPoint` and `CLastPoint` are **unchecked**; `Segment`, `Slice`, `Insert`, `Remove`, `RemoveShape`, `amendArc` and `splitArc` carry `wxCHECK`/`wxASSERT` guards.

`Append(const VECTOR2I& aP, bool aAllowDuplication = false)` (`slc.h:534-545`) **silently drops the point** when it equals `CLastPoint()` and `aAllowDuplication` is false. `Insert(aVertex, ...)` with `aVertex == PointCount()` delegates to `Append` (`slc.cpp:1639-1643`), so appending via `Insert` also dedupes while inserting elsewhere does not (TODO at `slc.cpp:1650`).

`Remove(int aStartIndex, int aEndIndex)` (`slc.cpp:1077-1164`) is inclusive on both ends and **temporarily forces the chain open** (`SetClosed(false)` at `:1083-1084`, restored at `:1163`), so `mergeFirstLastPointIfNeeded()` runs twice and can add then remove a trailing point; the index arithmetic inside runs on the unwrapped chain. It also shrinks the range to protect shared points, `aStartIndex += 1` if the start is shared and `aEndIndex -= 1` if the end is (`:1103-1110`), becoming a no-op if the range empties (`:1112-1116`). Note the asymmetric split calls: `splitArc(aStartIndex, false)` but `splitArc(aEndIndex + 1, true)` (`:1101`, `:1107`).

`Replace(int, int, const SHAPE_LINE_CHAIN&)` (`slc.cpp:1007-1074`) trims coincident endpoints of the incoming chain against the boundary points before removing, can degrade to a plain `Remove` in three places (`:1024`, `:1037`, `:1052`), and does **not** call `mergeFirstLastPointIfNeeded`. `Split(const VECTOR2I& aP, bool aExact = false)` (`slc.cpp:1181-1234`) inserts `aP` onto the chain and returns the new index or `-1`, with a hard-coded hit threshold of `min_dist = 2` so a segment matches when `seg.Distance(aP) < 2` (`:1184`).

`Slice(int, int[, int aMaxError])` (`slc.cpp:1418-1543`) returns an inclusive `[start, end]` subchain. Two properties matter: **the result is always open** (`m_closed` is never set on the output) and it **cannot wrap across the seam** because `aEndIndex >= aStartIndex` is required after normalization (`:1433`); all five `wxCHECK`s return an empty chain rather than signaling. Slicing across the seam of a closed chain requires two slices and a concatenation, which the router does at 20 call sites in `pns_line.cpp` and `pns_line_placer.cpp`. `Split(aStart, aEnd, aPre, aMid, aPost)` (`slc.cpp:2877-2902`) is the three-way split the walkaround and shove code use: it snaps both points with `NearestPoint(..., false)` (arc-endpoint snapping), splits a copy at both, reverses the order if needed, then slices. `Reverse()` (`slc.cpp:910-946`) returns a reversed **copy**, preserving `m_closed`, reversing all three vectors, remapping arc indices and swapping `.first`/`.second` on shared points.

### 6.5 `Simplify` versus `Simplify2`

`Simplify(int aTolerance = 0)` (`slc.h:358`, `slc.cpp:2782-2874`) is a greedy "farthest reachable endpoint" walk. Pseudo-code:

```
start = 0
out = [points[0]]
while start < n:
    end = start + 2
    while end reachable:
        # every intermediate point must be plain and within tolerance
        for test in (start+1 .. end-1):
            if m_shapes[test].first != SHAPE_IS_PT: fail        # arc guard
            if !TestSegmentHit(points[test], points[start], points[end], tol): fail
        if all ok: end += 1 (mod n) else: break
    out.push(points[end-1])
    start = end - 1
```

Key details: indices are taken modulo `m_points.size()`, so a closed chain wraps (`slc.cpp:2809-2811`, `:2833-2845`); the tolerance test is `TestSegmentHit` (`src/trigo.cpp:171`), a **distance-to-segment** test, not distance-to-infinite-line; `aTolerance == 0` means exact colinearity; **start and end are allowed to be arc endpoints, only intermediate points must be plain** (`slc.cpp:2813-2820`), so straight runs adjacent to arcs are still collapsible. `m_arcs` is left untouched, so it can end up holding arcs no longer referenced by any point. `m_bbox` is not updated.

`Simplify2(bool aRemoveColinear = true)` (`slc.h:362`, `slc.cpp:2906-3005`) is a separate, legacy path. The header comment at `slc.h:360-361` says outright: *"legacy function, used by the router. Please do not remove until I'll figure out the root cause of rounding errors - Tom"*. It runs two stages: duplicate-vertex removal gated on matching shape entries (`slc.cpp:2928-2954`), then colinear removal gated on both neighbouring shape entries being `SHAPES_ARE_PT` (`slc.cpp:2963-2991`). Its colinearity predicate is `SEG(p0, p[n+2]).LineDistance(p[n+1]) <= 1 || SEG(p0, p[n+2]).Collinear(SEG(p0, p[n+1]))` (`slc.cpp:2972-2973`), i.e. a hard-coded **1 nm** tolerance against the infinite line. It ignores `m_closed` entirely, so the closing segment is never simplified.

The router uses both, and the split is not arbitrary: `Simplify()` (36 sites) is used where geometry is being normalized for output or comparison; `Simplify2()` (18 sites, all in `pns_optimizer.cpp`, `pns_shove.cpp`, `pns_line.cpp`, `pns_walkaround.cpp`) is used inside the optimizer loops where the 1 nm slack matters. A Rust port must keep both behaviours or it will change optimizer convergence.

### 6.6 Intersections

`struct INTERSECTION` (`slc.h:86-119`):

| Field | Line | Meaning |
|---|---|---|
| `VECTOR2I p` | `slc.h:89` | the intersection point |
| `int index_our` | `slc.h:92` | segment (or corner, see below) index in **this** chain |
| `int index_their` | `slc.h:96` | segment (or corner) index in the **argument** chain |
| `bool is_corner_our` | `slc.h:99` | our corner `index_our` lies exactly on their line |
| `bool is_corner_their` | `slc.h:104` | their corner `index_their` lies exactly on our line |
| `bool valid` | `slc.h:109` | auxiliary flag for downstream filtering |

Default constructor sets both indices to `-1` and all flags false including `valid` (`slc.h:111-118`).

The index aliasing trap: when an intersection lands exactly on `SEG::B`, the code **increments the index** so it refers to the corner rather than the segment (`slc.cpp:1896`, `:1911`, `:1929`, `:1940`). So an index equal to `SegmentCount()` is reachable. `PNS::HullIntersection` compensates by wrapping `p.index_our` modulo `hull.SegmentCount()` (`pcbnew/router/pns_utils.cpp:423-424`).

`Intersect(const SEG&, INTERSECTIONS&)` (`slc.cpp:1731-1770`) sets `index_their = -1`, both corner flags false, `valid = true`, **appends** to the output vector and then sorts the **entire** vector by squared distance from `aSeg.A`. It returns `aIp.size()`, i.e. the total, not the number added.

`Intersect(const SHAPE_LINE_CHAIN&, INTERSECTIONS&, bool aExcludeColinearAndTouching = false, BOX2I* aChainBBox = nullptr)` (`slc.cpp:1802-1949`) builds a sorted segment-extent array in `thread_local` scratch vectors (`slc.cpp:1816-1817`) and uses `std::upper_bound` on x for pruning (`slc.cpp:1861-1870`). The flag reads backwards from its name: the colinear-overlap branch is guarded by `!aExcludeColinearAndTouching` (`slc.cpp:1883`), so **the default `false` includes** colinear and touching results as up to four intersection records (`slc.cpp:1885-1913`). Also appends and returns the total size.

`SelfIntersecting()` (`slc.cpp:2135-2218`) is line-segments-only, tests pairs `s1 < s2` with AABBs expanded by **2** to cover `SEG::Contains`'s `<= 3` squared tolerance (`slc.cpp:2156-2160`), and exempts the legitimate closing joint (`slc.cpp:2189-2193`). `SelfIntersectingWithArcs()` (`slc.cpp:2234-2398`) is the arc-exact version, using `CIRCLE::Intersect` for arc/segment and `SHAPE_ARC::Intersect` for arc/arc, with a `pointsClose` tolerance of squared norm `<= 2.0` (`slc.cpp:2237-2240`). **Both leave `valid == false`** on the returned record, unlike `Intersect()` which sets it true. The router calls only `SelfIntersecting()` (4 sites).

### 6.7 `PointInside`

`SHAPE_LINE_CHAIN::PointInside(const VECTOR2I& aPt, int aAccuracy = 0, bool aUseBBoxCache = false)` (`slc.h:896`, `slc.cpp:1986-2020`):

```
if aUseBBoxCache and cached bbox exists and !bbox.Contains(pt): return false
if !m_closed or PointCount() < 3: return false
inside = false
for each edge (p1, p2) over all PointCount() edges including the closing one:
    diff = p2 - p1
    if diff.y == 0: continue                       # skip horizontal edges
    d = rescale(diff.x, pt.y - p1.y, diff.y)       # rounds to NEAREST, not truncating
    if ((p1.y >= pt.y) != (p2.y >= pt.y)) and (pt.x - p1.x < d): inside = !inside
if aAccuracy <= 1: return inside
return inside or PointOnEdge(pt, aAccuracy)
```

This is a **crossing number**, not a winding number, cast in `+x`. The `>=` half-open y-rule makes vertex-on-ray cases consistent. The strict `<` on `pt.x - p1.x < d` matters. The `rescale` there is the `int` specialization with round-to-nearest (`src/math/util.cpp:62-72`), not truncation. An **open chain always returns false** (`slc.cpp:1994-1995`).

The `aAccuracy <= 1` fast path is explained in the base-class copy of the routine (`slc.cpp:2065-2066`): `OnEdge(accuracy)` is used as a proxy for `Inside(accuracy)`.

The bbox cache is **not** maintained automatically (see 6.8), so `aUseBBoxCache = true` requires a prior `GenerateBBoxCache()`.

`PointOnEdge` (`slc.cpp:2074-2077`) is `EdgeContainingPoint(...) >= 0`; `EdgeContainingPoint` (`slc.cpp:2080-2110`) uses `threshold = aAccuracy + 1` compared squared, and walks the arc **approximation** segments (not arc-exact).

### 6.8 Traps

Ordered roughly by how likely they are to bite a Rust port.

1. **`Append` silently drops a duplicate of the last point** (`slc.h:539`). N appends do not imply N points.
2. **`SegmentCount() == PointCount()` for a closed chain** (`slc.h:327-335`), not `PointCount()-1`.
3. **Index wrapping is single-step, not modular** (`slc.h:422-425`, `slc.cpp:1364-1367`).
4. **`Slice` never returns a closed chain and cannot cross the seam** (`slc.cpp:1418-1543`).
5. **`Remove` toggles the closed flag and can transiently add or remove a point** (`slc.cpp:1083-1084`, `:1163`, via `mergeFirstLastPointIfNeeded` at `:234-241`).
6. **`m_bbox` is never invalidated.** It is only written by `Append` (`slc.h:537`, `:543`, `slc.cpp:1577`, `:1603`), the internal arc append inside `Slice` (`slc.cpp:1448`, `:1498`), `Move` (`slc.h:782`) and explicit `GenerateBBoxCache()` (`slc.h:468`). It goes **stale after** `Remove`, `Insert`, `Replace`, `SetPoint`, `Simplify`, `Simplify2`, `RemoveDuplicatePoints`, `Rotate`, `Mirror`, `Clear`, `SetClosed`, `ClearArcs`. Meanwhile `BBox()` (`slc.h:457-466`) always recomputes from scratch and ignores the cache, so `BBox()` and `GetCachedBBox()` can disagree. Since `PointInside(aUseBBoxCache = true)` consults the cache, a stale one produces **false negatives**. Also note `BBox()` inflates by `aClearance + m_width`, not `aClearance + m_width/2`.
7. **`Clear()` resets neither `m_width` nor `m_bbox`** (`slc.h:274-280`).
8. **`SetPoint` destroys arcs** (`slc.cpp:1371-1376`). There is no "move an arc endpoint" operation; the router uses `amendArc` indirectly through `splitArc`/`Slice` instead.
9. **`Replace(range, chain)` breaks the chain-order invariant on `m_arcs`** (`slc.cpp:1071`), which makes a subsequent `Reverse()` remap arc indices wrongly.
10. **`Intersect()` appends and returns the total size**, and the `SEG` overload re-sorts entries a caller had already placed in the vector (`slc.cpp:1766-1769`).
11. **`aExcludeColinearAndTouching` defaults to including them** (`slc.cpp:1883`).
12. **`SelfIntersecting*()` leave `valid == false`** (`slc.h:116`).
13. **`NearestPoint(const SEG&, int&)` uses `LineDistance` over vertices only** (`slc.cpp:2459-2483`), so it is not the nearest point of the chain to the segment. `PNS::MoveDiagonal` (`pcbnew/router/pns_utils.cpp:289-297`) depends on exactly this behaviour.
14. **`PathLength(aP, aIndex = -1)` returns on the first segment** because `indexMatch` starts true (`slc.cpp:1959-1977`), so it silently means "distance from point 0" unless `aIndex` is passed. `aIndex == SegmentCount()` is remapped to the last segment (`slc.cpp:1963-1966`).
15. **`operator!=` ignores arcs, closedness and width** (`slc.h:749-761`); there is no `operator==`.
16. **`Format()` does not serialize arcs** (the code is commented out at `slc.cpp:2525-2531`) while `Parse()` does read them and only fills `.first` (`slc.cpp:2650`). The pair is not round-trip compatible.
17. **`TransformToPolygon` ignores `aError` and `aErrorLoc`** and is just `aBuffer.AddOutline(*this)` (`slc.cpp:3124-3128`).
18. **`Insert(size_t aVertex, const SHAPE_ARC&, int)` has an out-of-bounds iterator bug**: `for( auto arc_it = m_shapes.rbegin(); arc_it != m_shapes.rend() + aVertex; arc_it++ )` at `slc.cpp:1674-1676`. Adding to a reverse iterator moves it past `rend()`. It was clearly meant to be `rend() - aVertex`. Do not replicate it; scan the slice explicitly.
19. **`ClosestSegmentsFast` assumes a closed chain**, indexing `myPts[size()-1]` as the "previous point" when a bucket starts at 0 (`slc.cpp:639`, `:647`). Unused by the router.

## 7. The primitive shapes

### 7.1 `SHAPE_ARC`

`libs/kimath/include/geometry/shape_arc.h:35`. Stored as **three points plus a width**: `VECTOR2I m_start, m_mid, m_end; int m_width;` (`shape_arc.h:327-330`), with `BOX2I m_bbox`, `VECTOR2I m_center` and `double m_radius` as cached derived values (`shape_arc.h:332-334`) recomputed by `update_values()` (`shape_arc.h:325`).

The three-point representation is the important choice: it is exactly representable in integers, unambiguous about handedness, and stable under mirroring. `IsCCW()` (`shape_arc.h:315-322`) is `((m_end - m_mid) x (m_start - m_mid)) > 0` computed in `VECTOR2L`.

Constructors the router reaches: `ConstructFromStartEndAngle(start, end, angle)` (`shape_arc.h:98`) and `ConstructFromStartEndCenter(start, end, center, clockwise)` (`shape_arc.h:110`). `SHAPE_LINE_CHAIN::splitArc` and `amendArc` use the latter; `DIRECTION_45::BuildInitialTrace` uses both.

`ConvertToPolyline(int aMaxError = DefaultAccuracyForPCB(), int* aActualError = nullptr)` (`shape_arc.h:296`, `src/geometry/shape_arc.cpp:1013-1073`) is the polygonizer. `halfMaxError = max(1.0, aMaxError / 2.0)` (`shape_arc.cpp:1022`); degenerate cases collapse to one segment (`shape_arc.cpp:1032-1039`); segment count from `GetArcToSegmentCount` (`src/geometry/geometry_utils.cpp:38-60`, which clamps radius and error to at least 1 and enforces at least 2 segments); the radius is inflated by `effectiveError/2` and the point count doubled so the first and last sub-segments are shorter and the endpoints land exactly on the arc (`shape_arc.cpp:1049-1053`).

`DefaultAccuracyForPCB()` returns `ARC_HIGH_DEF` (`shape_arc.h:279`), which is `pcbIUScale.mmToIU(0.005)` = **5000 nm** (`include/base_units.h:128`, `:137`, with `PCB_IU_PER_MM = 1e6` at `:68`). `ARC_LOW_DEF` is `pcbIUScale.mmToIU(0.02)` = **20000 nm** (`include/base_units.h:127`, `:136`). `PNS::ArcHull` uses `ARC_LOW_DEF` (`pcbnew/router/pns_utils.cpp:88`); `SHAPE_LINE_CHAIN` uses `ARC_HIGH_DEF / 5` = 1000 nm.

Arcs stored inside a `SHAPE_LINE_CHAIN` always have width 0: it is forced on insertion (`slc.cpp:1620`, `:1697`) and asserted during collision (`shape_collisions.cpp:686`, `slc.cpp:485`).

The router constructs `DIRECTION_45` directly from a `SHAPE_ARC` chord via the dedicated constructor (`direction45.h:116-122`).

### 7.2 `SHAPE_CIRCLE`

`shape_circle.h:33`, wrapping `CIRCLE m_circle` (`shape_circle.h:143`), which is `{ VECTOR2I Center; int Radius; }` (`geometry/circle.h:150-151`).

`Collide(SEG, clearance, aActual, aLocation)` is inline in the header (`shape_circle.h:73-102`): `minDist = aClearance + Radius`; hit when `dist_sq == 0 || dist_sq < Square(minDist)`; `*aActual = max(0, (int) sqrt(dist_sq) - Radius)`; `*aLocation` is the segment's nearest point to the center, or the first circle/segment intersection when `dist_sq == 0`.

`CIRCLE::Contains` and `CIRCLE::Intersect` use `SHAPE::MIN_PRECISION_IU = 4` (`shape.h:129`) as a band tolerance: `src/geometry/circle.cpp:192-193`, `:351`, `:355-356`.

`PNS::VIA` stores one `SHAPE_CIRCLE` per layer in a `std::map<int, SHAPE_CIRCLE>` (`pcbnew/router/pns_via.h:349`) to support padstacks with per-layer diameters.

### 7.3 `SHAPE_RECT`

`shape_rect.h:34`, holding `{ VECTOR2I m_p0; int m_w; int m_h; int m_radius; }` (`shape_rect.h:249-252`). **It has a corner radius**, i.e. it is really a rounded rect; the radius is 0 in every constructor except copy and `GetInflated(aOffset)` bumps it by the offset (`shape_rect.h:116-125`). `BBox(aClearance)` builds `BOX2I(p0 - (c,c), (w + 2c, h + 2c))` (`shape_rect.h:105-111`), where the second argument is a size, not a corner. The router builds one only in `PNS::ApproximateSegmentAsRect` (`pcbnew/router/pns_utils.cpp:356-366`).

### 7.4 `SHAPE_SEGMENT`

`shape_segment.h:33`, holding `{ SEG m_seg; int m_width; }` (`shape_segment.h:187-188`): a capsule with a round cap of `m_width/2` at each end. Its `Collide` overloads are inline (`shape_segment.h:78-116`) and use `min_dist = ( m_width + 1 ) / 2 + aClearance`, **rounding the half-width up**, which is inconsistent with `shape_collisions.cpp`'s truncating `aB.GetWidth() / 2` (`:336`, `:339`, `:536`, `:552`, `:571`, `:777`). The two differ by 1 nm for odd widths. `PNS::SEGMENT` holds one by value (`pcbnew/router/pns_segment.h:146`).

### 7.5 `SHAPE_SIMPLE`, `SHAPE_COMPOUND`, `SHAPE_ELLIPSE`

`SHAPE_SIMPLE` (`shape_simple.h:37`) derives from `SHAPE_LINE_CHAIN_BASE` and is a thin wrapper over a `SHAPE_LINE_CHAIN m_points` forced closed in every constructor (`shape_simple.h:46`, `:53`). It is what KiCad's board interface hands the router for pad outlines (`pns_kicad_iface.cpp:1734`) and for each zone triangle (`pns_kicad_iface.cpp:1933`). It is assumed **convex** by `PNS::ConvexHull` (12.3), though nothing enforces that.

`SHAPE_COMPOUND` (`shape_compound.h:34`) holds `{ std::vector<SHAPE*> m_shapes; BOX2I m_cachedBBox; bool m_dirty; }` (`shape_compound.h:159-161`) and owns its children: `AddShape` clones (`shape_compound.h:79-115`) and flattens any child reporting indexable subshapes (`:85-90`, `:106-111`). It represents a complex pad or a slot hole. The router touches it in exactly two places, both `Hull()` implementations (`pns_solid.cpp:46`, `pns_hole.cpp:75`). **Nested compounds are unsupported by the collision dispatcher** (9.1).

`SHAPE_ELLIPSE` is fully wired into collision dispatch but the router touches it only in the hull dispatcher, where it degrades to its bounding box: `OctagonalHull( bbox.GetPosition(), bbox.GetSize(), cl, 0 )` (`pns_utils.cpp:523-529`), a plain rectangle with no chamfer.

## 8. `DIRECTION_45`

`libs/kimath/include/geometry/direction45.h:36`. State is `{ Directions m_dir; bool m_90deg; }` (`direction45.h:346-349`). This is the type that makes PNS a 45-degree router.

### 8.1 The octant encoding

`enum Directions : int { N=0, NE=1, E=2, SE=3, S=4, SW=5, W=6, NW=7, LAST=8, UNDEFINED=-1 }` (`direction45.h:48-60`). **North is up on screen, which is negative y in world space** (`direction45.h:45-46`). Every constructor from a vector or segment therefore flips y before classifying (`direction45.h:96`, `:107`, `:120`).

`construct_(VECTOR2I)` (`direction45.h:317-343`) is float-based: it computes `mag = 360 - atan2(y, x) * 180/pi + 90`, normalizes into `[0, 360)`, then `dir = (mag + 22.5) / 45.0`, i.e. **rounds to the nearest octant**. There is no exactness check; `DIRECTION_45(v)` never returns `UNDEFINED` for a non-zero vector, only for `(0,0)` (`direction45.h:321-322`).

`m_90deg` is set by the second constructor argument and only affects `Left()`/`Right()`, which step by 2 instead of 1 (`direction45.h:257-260`, `:275-278`).

### 8.2 Angle classification

`AngleType Angle(const DIRECTION_45& aOther)` (`direction45.h:181-198`) is pure integer arithmetic on `d = abs(m_dir - aOther.m_dir)`:

| `d` | result | value |
|---|---|---|
| 1 or 7 | `ANG_OBTUSE` | `0x01` |
| 2 or 6 | `ANG_RIGHT` | `0x02` |
| 3 or 5 | `ANG_ACUTE` | `0x04` |
| 4 | `ANG_HALF_FULL` | `0x10` |
| 0 | `ANG_STRAIGHT` | `0x08` |
| either undefined | `ANG_UNDEFINED` | `0x20` |

The values are powers of two so they can be OR'd into masks; `pns_optimizer.cpp:1114` builds `ForbiddenAngles = ANG_ACUTE | ANG_RIGHT | ...` and `pns_optimizer.cpp:665` uses `angleMask = ANG_OBTUSE`.

`IsDiagonal()` is `(m_dir % 2) == 1` (`direction45.h:213-216`), i.e. NE/SE/SW/NW.

`ToVector()` returns the unit octant vector with the screen-to-world y flip already applied: N is `(0,-1)`, SE is `(1,1)`, and so on (`direction45.h:287-303`).

`Opposite()` is a lookup table (`direction45.h:172`). `Mask()` is `1 << m_dir` (`direction45.h:305-308`) and is unused by the router.

### 8.3 `BuildInitialTrace`

`const SHAPE_LINE_CHAIN BuildInitialTrace(const VECTOR2I& aP0, const VECTOR2I& aP1, bool aStartDiagonal = false, CORNER_MODE aMode = MITERED_45) const` (`direction45.h:234-236`, implemented `src/geometry/direction_45.cpp:24-312`).

This is the single most important function in the whole 45-degree regime: it produces the two-segment (or arc-plus-segment) trace between two points that obeys the routing posture. It is called 19 times in the router core, from the placer, the line, the optimizer and the mouse trail tracer.

```
startDiagonal = (m_dir == UNDEFINED) ? aStartDiagonal : IsDiagonal()
w = |dx|, h = |dy|, sw = sign(dx), sh = sign(dy)
is90mode = (aMode is MITERED_90 or ROUNDED_90)

# single-segment shortcut: axis-aligned, or an exact diagonal in 45 mode
if w == 0 or h == 0 or (!is90mode and h == w):
    return chain(aP0, aP1)

if is90mode:
    mp0 = (startDiagonal == (h >= w)) ? (w*sw, 0) : (0, sh*h)
else:
    if w > h:  mp0 = ((w-h)*sw, 0);  mp1 = (h*sw, h*sh);  tangentLength = (w-h) - |mp1|
    else:      mp0 = (0, sh*(h-w));  mp1 = (sw*w, sh*w);  tangentLength = (h-w) - |mp1|

MITERED_45:  chain(aP0, aP0 + (startDiagonal ? mp1 : mp0), aP1)
MITERED_90:  chain(aP0, aP0 + mp0, aP1)
ROUNDED_45:  replace the corner with a 45-degree arc of radius
             diagLength / (2 cos 67.5 deg), placed at the start or the end
             depending on startDiagonal and sign(tangentLength)
ROUNDED_90:  replace the corner with a quarter arc whose radius is min(w, h)
finally: pl.Simplify()
```

The `ROUNDED_45` branch (`direction_45.cpp:105-222`) is the fiddly one. `diagLength = sqrt(2*diag2 - 2*diag2*cos(3*pi/4))` (`direction_45.cpp:131`) and `arcRadius = KiROUND(diagLength / (2 cos(67.5 deg)))` (`direction_45.cpp:132`). Four sub-cases keyed on `startDiagonal` and `sign(tangentLength)`. In the "negative tangent length, straight start" case the arc is constructed from a center, which loses endpoint precision, so the code re-snaps the endpoint when it is within `SHAPE_ARC::MIN_PRECISION_IU` (= 4 nm, `shape.h:129`) of the target coordinate (`direction_45.cpp:202-211`). The developer comment at `direction_45.cpp:134-137` is candid that the math "could probably be condensed and optimized but I'm tired of staring at it".

The trailing `pl.Simplify()` (`direction_45.cpp:310`) removes any degenerate zero-length segment the corner construction introduced.

`CORNER_MODE` (`direction45.h:66-72`) is `MITERED_45 = 0`, `ROUNDED_45 = 1`, `MITERED_90 = 2`, `ROUNDED_90 = 3`. It is carried by `PNS::ROUTING_SETTINGS::m_cornerMode` (`pcbnew/router/pns_routing_settings.h:181`), defaulting to `MITERED_45` (`pns_routing_settings.cpp:53`, `:103-104`).

### 8.4 Role in the placer

`PNS::ROUTING_SETTINGS::InitialDirection()` returns `NE` or `N` depending on a bool (`pns_routing_settings.cpp:112-118`). `PNS::LINE_PLACER` keeps `m_direction` and `m_initial_direction` as `DIRECTION_45` and uses `Angle()` to decide when to pull the tail back: `pullback_2 = (angle == ANG_RIGHT || angle == ANG_ACUTE)` (`pns_line_placer.cpp:217`), after which it does `tail.RemoveShape(-1)` (`pns_line_placer.cpp:244`). `PNS::MOUSE_TRAIL_TRACER::GetPosture` (`pns_mouse_trail_tracer.cpp:85+`) picks between the straight-first and diagonal-first postures using an area ratio with `areaRatioThreshold = 1.3` and `areaRatioEpsilon = 0.25` (`pns_mouse_trail_tracer.cpp:87`, `:90`).

## 9. Collision dispatch: `libs/kimath/src/geometry/shape_collisions.cpp`

In this section `sc.cpp` = `libs/kimath/src/geometry/shape_collisions.cpp`.

### 9.1 Structure

Two public entry points, both defined at the bottom of the file:

* `SHAPE::Collide(const SHAPE*, int aClearance, VECTOR2I* aMTV)` (`sc.cpp:1432`, declared `shape.h:198`) forwards to `collideShapes(this, aShape, aClearance, nullptr, nullptr, aMTV)`.
* `SHAPE::Collide(const SHAPE*, int aClearance = 0, int* aActual = nullptr, VECTOR2I* aLocation = nullptr)` (`sc.cpp:1438`, declared `shape.h:200-201`) forwards with `aMTV = nullptr`.

**They are mutually exclusive**: through the public API you can never ask for MTV and actual-distance in the same call.

`collideShapes` (`sc.cpp:1307`) handles `SH_COMPOUND` only. It expands compound operands into per-subshape pairs (`sc.cpp:1359-1410`), keeps the **minimum** `actual` across sub-collisions (`sc.cpp:1342`) and the **maximum-magnitude** MTV (`sc.cpp:1348`). Its early-exit predicate `canExit` (`sc.cpp:1315`) fires only when there is no `aActual` request or the actual is already 0, and no MTV was requested.

`collideSingleShapes` (`sc.cpp:1047`) short-circuits `SH_POLY_SET` on either side into `SHAPE_POLY_SET::Collide` (`sc.cpp:1050-1063`, asserting `!aMTV` at `:1054` and `:1060`), returns false for `SH_NULL` (`sc.cpp:1067-1068`), and otherwise runs a two-level `switch` on the concrete pair (`sc.cpp:1065-1298`).

**Nested compounds are unsupported.** `collideCompoundSubshapes` calls `collideSingleShapes` directly (`sc.cpp:1337`), which has no `SH_COMPOUND` case, so a compound-in-compound falls through to `wxFAIL_MSG( "Unsupported collision: %s with %s" )` (`sc.cpp:1300-1304`) and returns false. That is also the fallback for any pair the switch does not cover: **assert in debug, return false in release, no generic swap-and-retry.**

Argument swapping is done statically per switch arm through two helpers: `CollCase<Ta,Tb>(x, y, ...)` (`sc.cpp:895`) casts and calls in the given order, and `CollCaseReversed<Ta,Tb>(x, y, ...)` (`sc.cpp:905`) calls with the operands reversed and **negates the MTV** (`sc.cpp:912-913`).

### 9.2 The pair matrix

There are 22 concrete `Collide(A, B, clearance, aActual, aLocation, aMTV)` overloads. Those that can produce an MTV: `CIRCLE x CIRCLE` (`sc.cpp:40`), `RECT x CIRCLE` (`:67`, asserted off when the rect has a corner radius, `:72`), `CIRCLE x LINE_CHAIN_BASE` (`:236`), `CIRCLE x SEGMENT` (`:333`), `ARC x CIRCLE` (`:597`), `ARC x RECT` (`:721`), `ARC x ARC` (`:850`). Every other overload asserts `!aMTV`: `LCB x LCB` (`:351`), `RECT x LCB` (`:469`), `SEGMENT x SEGMENT` (`:529`), `LCB x SEGMENT` (`:545`), `RECT x SEGMENT` (`:561`), `RECT x RECT` (`:580`, delegating to `LCB x LCB`), `ARC x LINE_CHAIN` (`:636`), `ARC x SEGMENT` (`:763`), `ARC x LCB` (`:786`), and six `ELLIPSE x *` overloads (`:919`, `:943`, `:970`, `:984`, `:998`, `:1018`). Pairs with no dedicated overload (`SEGMENT x LCB`, `SEGMENT x RECT`, `SEGMENT x CIRCLE`, `LINE_CHAIN x CIRCLE`, `RECT x ARC`, `CIRCLE x RECT`) are reached by explicit operand reordering in the switch, which spans `sc.cpp:1065-1298`.

**There is an MTV sign inconsistency in the tree.** Several switch cells call `CollCase<..>(aB, aA, ...)`, swapping the operands **without** negating the MTV (`sc.cpp:1140`, `:1143`, `:1173`, `:1179`, `:1186`, `:1207`, `:1210`, `:1213`); two of those (`:1143`, `:1210`) dispatch to `CIRCLE x LINE_CHAIN_BASE`, which does compute one. Separately, that overload writes the pushout applied to **A** (`sc.cpp:323`) whereas every other MTV overload writes the translation for **B**. The router acknowledges this at `pcbnew/router/pns_shove.cpp:1244`: `// fixme: we may have a sign issue in Collide(CIRCLE, LINE_CHAIN)`.

### 9.3 Parameter semantics

`aClearance` is a required **edge-to-edge** separation in nm, added to shape inflation terms before the geometric test (circle radii at `sc.cpp:43`, segment half-widths at `:336`, `:536`, `:552`, `:571`, `:777`, circle-versus-rect at `:83`). The predicate is **strict**: `dist_sq == 0 || dist_sq < min_dist_sq` (`sc.cpp:49`, `:129`, `:615`, `:742`, `:874`), so a gap exactly equal to `aClearance` is not a collision while a gap of exactly 0 always is, even at `aClearance == 0`. Negative clearance is rejected by `SEG::Collide` (`seg.cpp:538-546`) but is not guarded at the shape level.

`aActual` is the **edge-to-edge gap distance, clamped to at least 0**, written **only on a true return**, always `sqrt(squared distance)` minus the inflation terms under a `std::max(0, ...)` (`sc.cpp:52`, `:135`, `:342`, `:539`, `:555`, `:574`, `:621`, `:748`, `:780`, `:880`, `:933`, `:959`). Known defect: for `RECT x CIRCLE` with the circle center strictly inside the rect it reports the distance to the nearest side rather than 0 (`sc.cpp:98-99`, `:135`).

`aLocation` is "a point near the collision", but **which** shape it lies on varies per overload with no consistent contract: `CIRCLE x CIRCLE` writes the midpoint of the two centers, on neither shape (`sc.cpp:55`); the three arc overloads write the midpoint of the two nearest points (`sc.cpp:618`, `:745`, `:877`); `RECT x CIRCLE` writes a point on the rect boundary (`sc.cpp:132`); `LCB x LCB` writes a point on chain A (`sc.cpp:412`) or vertex 0 of either chain in the containment branches (`sc.cpp:364`, `:369`); `ELLIPSE x CIRCLE` writes the circle center (`sc.cpp:962`).

`aMTV` is the minimum translation vector. The prevailing convention is **the translation to apply to B (the `aShape` argument) to separate it from A**, verified at `sc.cpp:58`, `:142`, `:144`, `:339`, `:626`, `:753`, `:885`, with `sc.cpp:323` as the exception noted above. Its magnitude is penetration depth plus `aClearance` plus a bias, not a pure minimum-overlap distance.

### 9.4 The three MTV algorithms

Analytic radial, for `CIRCLE x CIRCLE` (`sc.cpp:58`):

```
delta = B.center - A.center            # i32 subtraction, can overflow
dist_sq = delta.SquaredEuclideanNorm() # exact i64
min_dist = aClearance + rA + rB        # i32 addition, can overflow
*aMTV = delta.Resize( min_dist - sqrt(dist_sq) + 3 )   // fixme: apparent rounding error
```

The `+ 3` bias carries an in-source `fixme` comment. If the centers coincide, `delta` is zero, `Resize` returns `(0,0)` (`vector2d.h:383-384`) and the collision is **unresolvable**. `PNS::VIA::PushoutForce` handles that case explicitly at `pns_via.cpp:139`, and its caller comments at `pns_via.cpp:172-173`: *"might happen (although rarely) that we see a collision, but the MTV is zero... Assume force propagation has failed in such case."*

Analytic nearest-side, for `RECT x CIRCLE` (`sc.cpp:137-145`): `nearest` is the closest point on any of the four sides (`sc.cpp:105-127`); the loop cannot early-exit when an MTV is requested (`sc.cpp:117-118`). The magnitude carries **two independent `+1` biases**. The `inside` test is inclusive axis-aligned containment (`sc.cpp:98-99`).

Iterative pushout, for `CIRCLE x LINE_CHAIN_BASE` (`sc.cpp:299-323`, using `pushoutForce` at `sc.cpp:154-179`):

```
pushoutForce(circle, seg, clearance):
    nearest = seg.NearestPoint(circle.center)
    dist = |nearest - center|;  min_dist = clearance + r
    if dist >= min_dist: return (0,0)
    for corr in 0..4:                       # integer search, 1 nm steps
        f = (center - nearest).Resize(min_dist - dist + corr)
        if seg.Distance(center + f) >= min_dist: break
    return f                                # falls through with corr == 4 if none worked

chain MTV:
    if the circle center is inside the closed chain:
        cs = segment with minimum Distance to center
        np = cs.NearestPoint(center)
        f = (np - center) + (np - center).Resize(radius)   # to boundary, then one radius past
        apply f, accumulate into f_total
    for each segment in order:               # single pass, order dependent, no convergence check
        f = pushoutForce(cmoved, seg, aClearance)
        apply f to cmoved; f_total += f
    *aMTV = f_total
```

The 5-iteration cap (`sc.cpp:168`) can silently return an insufficient MTV.

Nearest-points based, for the three arc overloads (`sc.cpp:623-627`, `:750-754`, `:882-886`): `delta = ptB - ptA` from `SHAPE_ARC::NearestPoints` (declared `shape_arc.h:131`, `:141`, `:151`, `:161`), then `delta.Resize( aClearance - sqrt(dist_sq) + 3 )`, the same `+3` bias.

`SHAPE_LINE_CHAIN::ClosestPoints` (`slc.h:222`, `slc.cpp:718`, `:751`) is **not** used by the collision code, and there is no `-aClearance` biasing anywhere in `sc.cpp`: every bias is positive.

### 9.5 Holes

**kimath has no hole concept in the collision code at all.** Grepping `HasHole|GetHole|Hole()` across `libs/kimath/{include,src}/geometry/` yields only `SHAPE_POLY_SET::HasHoles()` (`shape_poly_set.h:1114`, `shape_poly_set.cpp:1843`), the Clipper import test at `shape_poly_set.cpp:1147`, and one call in `shape_line_chain.cpp:3023`. None of them is referenced from `sc.cpp`. `SHAPE_CIRCLE` is `{ CIRCLE }`, `SHAPE_SEGMENT` is `{ SEG, int }`, `SHAPE_COMPOUND` is a plain vector of shapes.

Holes live one level up, as **independent shapes with their own identity**:

* `PNS::HOLE` (`pcbnew/router/pns_hole.h:33`) wraps a raw `SHAPE* m_holeShape` (`pns_hole.h:94`), typically a `SHAPE_CIRCLE` built by `MakeCircularHole` (`pns_hole.cpp:131-135`) or a `SHAPE_SEGMENT` for a slot.
* `PNS::ITEM::HasHole()` defaults to false (`pns_item.h:303`) and is overridden by `SOLID` and `VIA`.
* A pad's or via's hole is **already a separate `PNS::ITEM` in the node index** (comment at `pns_item.cpp:106-108`), so `collideSimple` only has to recurse explicitly for the routing head's hole (`pns_item.cpp:146-157`).
* `shouldWeConsiderHoleCollisions` (`pns_item.cpp:38`) gates the recursion.

The one place hole semantics leak into kimath is `SHAPE_POLY_SET::Collide(const SHAPE*, ...)` (`shape_poly_set.cpp:2468`): its `SH_SEGMENT` and `SH_CIRCLE` fast paths (`:2472-2502`) measure distance to **every** contour including hole boundaries, whereas the general triangulation path (`:2504-2534`) correctly excludes hole interiors. The router never hits this because it never puts a `SHAPE_POLY_SET` in its world.

### 9.6 Arithmetic and overflow

Exact `i64` paths: `SquaredEuclideanNorm` (`vector2d.h:303`), `Cross`/`Dot` (`vector2d.h:534`, `:542`), `SEG::Square` (`seg.h:119-122`), `SEG::SquaredDistance` (`seg.cpp:76`, `:710`), `SEG::Collide`'s `clearance_sq` (`seg.cpp:581`).

Floating point paths: every `sqrt` on an `int64_t` in `sc.cpp` (`:52`, `:58`, `:135`, `:142`, `:144`, `:621`, `:626`, `:748`, `:753`, `:880`, `:885`, `:958`), plus `VECTOR2::Resize` and `VECTOR2::EuclideanNorm`. `double` has 53 mantissa bits, so `sqrt(dist_sq)` stops being exact once `dist_sq` exceeds 2^53, i.e. once the distance exceeds roughly 95 mm. A port that wants bit-identical results must reproduce this; a port that wants correctness should use an exact integer sqrt everywhere.

**Three different rounding conventions for the same quantity coexist**: `(int) sqrt(...)` truncation (`sc.cpp:52`, `:135`), `KiROUND(sqrt(...))` round-to-nearest (`sc.cpp:621`, `:748`, `:880`), and `isqrt` truncation (`seg.cpp:698-707`).

Overflow sites, none guarded:

* **32-bit inflation sums**: `aClearance + rA + rB` (`sc.cpp:43`), `aClearance + r` (`:83`, `:164`, `:950`), `aClearance + aHalfWidth` (`:186`, `:925`, `:1004`), `aClearance + width/2` (`:336`, `:339`, `:536`, `:552`, `:571`, `:777`). All computed in `int` before any promotion. This is reachable: `SHAPE::GetClearance` calls `Collide(b, INT_MAX/2, &temp_dist)` (`src/geometry/shape.cpp:94`), so any pair whose radii sum past about 1.07e9 nm overflows.
* **Squares of the result**: `min_dist * min_dist` at `sc.cpp:44` is `i64 * i64`, safe given a correct `min_dist`, but marginal (`3e9^2` slightly exceeds `INT64_MAX`).
* **Narrowing back to `int`**: `(int) sqrt(dist_sq)` for `dist_sq` near `INT64_MAX` produces a value above `INT32_MAX` (`sc.cpp:52`, `:135`, `:621`, `:748`, `:880`, `:958`).
* **Coordinate arithmetic**: every `VECTOR2I +/- VECTOR2I` is 32-bit component-wise. The midpoint forms `(a + b) / 2` (`sc.cpp:55`, `:618`, `:745`, `:877`) overflow **before** the divide.

### 9.7 How the router calls into it

The single call site that matters is `PNS::ITEM::collideSimple` (`pcbnew/router/pns_item.cpp:104-302`). It resolves a clearance from the rule resolver (`pns_item.cpp:220`), then:

```
shapeH->Collide( shapeI, clearance + lineWidthH + lineWidthI - 1 )
```

at `pns_item.cpp:280` (fast path) and `:249` (slow path, which also asks for `aActual` and `aLocation` so it can test castellation and net-tie exclusions at the collision point).

Two details worth transplanting verbatim. First, **line widths are folded into the clearance** because "collision routines ignore SHAPE_POLY_LINE widths" (comment at `pns_item.cpp:159-160`); `lineWidthI`/`lineWidthH` are `LINE::Width() / 2` (`pns_item.cpp:162`, `:165`). Second, the **`- 1`**: the comment at `pns_item.cpp:246-248` and `:277-279` says *"the hulls are built to exactly the clearance distance, so we need to allow for no collision when exactly at the clearance distance."* That one-nanometre subtraction is the hinge between the collision predicate and the hull geometry, and getting it wrong makes the walkaround loop non-terminating.

A clearance of `-1` means "no clearance rule applies, skip entirely" (`pns_item.cpp:191`, `:196`, `:204`, `:208`, `:212`, gated at `:224`).

MTV is consumed in exactly two places: `PNS::VIA::PushoutForce(NODE*, const ITEM*, VECTOR2I&)` (`pcbnew/router/pns_via.cpp:126-140`), which takes the maximum-magnitude MTV across the via's relevant layers, and `PNS::SHOVE::onCollidingVia` (`pcbnew/router/pns_shove.cpp:1180-1258`), which does the same and then applies `-mtv` twice (`pns_shove.cpp:1247-1255`).

## 10. Spatial index

### 10.1 What the router uses

`PNS::INDEX` (`pcbnew/router/pns_index.h`) is a thin layer over `SHAPE_INDEX<ITEM*>` (`pns_index.h:50`), which is `KIRTREE::COW_RTREE<T, int, 2>` (`libs/kimath/include/geometry/shape_index.h:112`) with the default `TMAXNODES = 16` (`libs/kimath/include/geometry/rtree/dynamic_rtree_cow.h:50`). So the full instantiation is `KIRTREE::COW_RTREE<PNS::ITEM*, int, 2, 16>`.

The payload is a **raw non-owning `PNS::ITEM*`** stored by value in a leaf `data[]` array (`rtree/rtree_node.h:315`). Ownership lives in `PNS::NODE`.

The legacy single-header Guttman R-tree still exists in the repo at `thirdparty/rtree/geometry/rtree.h` (2022 lines) but is marked **DEPRECATED** in its own header, and its only remaining consumer is the benchmark `qa/tests/libs/kimath/geometry/bench_spatial_index.cpp`. `pcbnew/router/CMakeLists.txt:13` still lists `../../thirdparty/rtree` on the include path.

The bbox is obtained through a free-function customization point rather than a member: `boundingBox<T>(T, int aLayer)` -> `shapeFunctor<T>(aItem, aLayer)` -> `aItem->Shape(aLayer)->BBox()` (`shape_index.h:43-46`, `:58-63`, with `ITEM::Shape(int)` at `pns_item.h:242`).

### 10.2 Operations `pns_index` needs

Layer partitioning is **one tree per layer index**, held in `std::deque<std::unique_ptr<ITEM_SHAPE_INDEX>> m_subIndices` (`pns_index.h:154`), grown lazily (`pns_index.cpp:33-39`). An item spanning layers `[Start, End]` is inserted into **every** sub-index in that range (`pns_index.cpp:43-44`), so an N-layer via appears N times; duplicate hits are absorbed because the consumer collects into a `std::set<OBSTACLE>` (`pns_node.h:254`).

Alongside the trees, `PNS::INDEX` keeps `std::map<NET_HANDLE, std::list<ITEM*>> m_netMap` (`pns_index.h:155`, populated `pns_index.cpp:48-51`, torn down with an O(n) `std::list::remove` at `:100-101`) and `std::unordered_set<ITEM*> m_allItems` (`pns_index.h:156`) backing `Contains()`, `Size()` and iteration (`pns_index.h:136`, `:144`, `:146-147`).

| Operation | Location |
|---|---|
| `Add(ITEM*)` | `pns_index.cpp:28` |
| `Remove(ITEM*)` | `pns_index.cpp:86` |
| `Replace(old, new)` | `pns_index.cpp:105` (Remove + Add) |
| `SetDeferred(bool)` | `pns_index.cpp:55` |
| `BuildSpatialIndex()` | `pns_index.cpp:61` |
| `Clone()` | `pns_index.h:71-81` |
| `Query(const ITEM*, int aMinDistance, Visitor&)` | `pns_index.h:110`, impl `:171-189` |
| `Query(const SHAPE*, int aMinDistance, Visitor&)` | `pns_index.h:124`, impl `:191-200` |
| `GetItemsForNet` | `pns_index.cpp:112` |

The deferred/bulk-load path is a real requirement, not a micro-optimization: `NODE::BeginBulkAdd()` sets deferred and `NODE::FinalizeBulkAdd()` clears it and bulk-loads (`pns_node.cpp:1257-1267`). While deferred, `Add()` still updates `m_allItems`/`m_netMap` and still grows the deque but skips tree insertion (`pns_index.cpp:41-45`); `BuildSpatialIndex()` then regroups by layer and calls `SHAPE_INDEX::BulkLoad` per layer (`pns_index.cpp:63-82`).

### 10.3 Query shape

**There is no BOX2I query on `PNS::INDEX`.** Both public queries are shape-based and templated on the visitor.

`Query(const ITEM*, ...)` (`pns_index.h:171-189`) iterates `aItem->Layers().Start() .. End()`, fetches `aItem->Shape(i)` per layer and queries **only sub-index `i`**, so layer overlap filtering is implicit in which trees get visited. `Query(const SHAPE*, ...)` (`pns_index.h:191-200`) queries **all** sub-indices. Both funnel through `querySingle` (`pns_index.h:162-169`), which wraps the call in an RAII `LAYER_CONTEXT_SETTER` (`pns_node.h:214-231`) that stashes the layer id on the visitor so the exact per-layer collision test can run later.

The "query by shape with clearance" is exactly inflate-then-filter, at `shape_index.h:320-330`:

```
BOX2I box = aShape->BBox();
box.Inflate( aMinDistance );
int min[2] = { box.GetX(), box.GetY() };
int max[2] = { box.GetRight(), box.GetBottom() };
return m_tree.Search( min, max, aVisitor );
```

Only the **query** box is inflated; stored boxes are not. That is correct only because the visitor re-tests exactly.

The tree-level signature is `template <class VISITOR> int Search(const ELEMTYPE aMin[NUMDIMS], const ELEMTYPE aMax[NUMDIMS], VISITOR&) const` (`dynamic_rtree_cow.h:197-207`). The visitor is a stateful mutable lvalue reference with `bool operator()(DATATYPE)`; **returning `false` stops the search**. The return value counts items **reported to the visitor**, incremented before the call (`dynamic_rtree_cow.h:809-812`), so it is a bbox-hit count including the aborting item, not a true-collision count.

All clearance refinement happens in the visitor. `DEFAULT_OBSTACLE_VISITOR::operator()` (`pns_node.cpp:241-263`) applies, in order: kind mask, self-collision skip, user filter callback, branch-override check (`OBSTACLE_VISITOR::visit`, `pns_node.cpp:215-223`), then the real `aCandidate->Collide( m_item, m_node, m_layerContext.value_or(-1), m_ctx )`, then the `m_limitCount` early-out.

`queryCallback<T,V>` / `acceptVisitor` (`shape_index.h:98-106`) and `collide<T,U>` (`shape_index.h:92-96`) are dead code; nothing calls them.

The router also uses the unrelated brute-force `SHAPE_INDEX_LIST<ITEM*>` for the optimizer's cache (`pcbnew/router/pns_optimizer.h:204`). Its `Query` (`shape_index_list.h:237-261`) has a **different contract**: it does the exact `Collide` test inside the query when `aExact` (default true), not in the visitor.

### 10.4 R-tree internals worth knowing for a port

* `MINNODES = MAXNODES * 2 / 5` (`rtree/rtree_node.h:303`), so 6 at fanout 16. `REINSERT_COUNT = MAXNODES * 3 / 10` = 4 (`rtree/dynamic_rtree.h:68`), used only by the non-CoW tree.
* `static_assert( FANOUT <= 31 )` and `static_assert( FANOUT % 4 == 0 )` (`rtree/rtree_node.h:156-157`) for the SIMD overlap mask, which is a `uint32_t`.
* Two trees exist. `DYNAMIC_RTREE` is a full R*-tree with forced reinsert and margin/overlap split (`dynamic_rtree.h:738-800`, `:806-980`, `:1045-1250`). `COW_RTREE`, the one PNS uses, deliberately is not: `dynamic_rtree_cow.h:149` says *"Use a simplified insertion for CoW (no forced reinsert to avoid complexity)"*. `insertSimple` (`:549-603`) descends by minimum area enlargement; `simpleSplit` (`:607-749`) is a median split along the longest axis, sort key `min[axis] + max[axis]`, `splitIdx = total/2` (`:673`).
* **`simpleSplit`'s non-root case is admittedly broken.** `dynamic_rtree_cow.h:723-725`: *"For non-root splits in CoW trees, we need to find the parent. This is a limitation of the simplified CoW approach... For now, create a new root."* It grafts a leaf sibling as a direct child of a brand-new root (`:727-747`), producing inconsistent leaf depths. Search still works because it dispatches on `IsLeaf()` per node. `insertSimple:597-602` similarly detects a child split and does nothing. **Do not replicate this.**
* Bulk load is Hilbert-curve packed at 100% fill: `BulkLoad` at `dynamic_rtree_cow.h:222-350`, sorting by `KIRTREE::HilbertXY2D( 16, hx, hy )` (`:273`) and packing bottom-up (`:300`, `:327`). Note it **reorders the caller's vector in place** (`:282-284`). The non-CoW tree uses `HilbertND2D<NUMDIMS>( 32, coords )` (`dynamic_rtree.h:284`), a different function and a different curve order.
* **Removal keys on the insertion bbox, not the current one.** Every entry stores both `bounds` and a frozen `insertBounds` (`rtree_node.h:310`, `:320`). `DYNAMIC_RTREE::Remove` falls back to a full-tree scan on failure (`dynamic_rtree.h:151-168`); **`COW_RTREE::Remove` has no such fallback** (`dynamic_rtree_cow.h:157-190`, match test at `:757-762`). Meanwhile `SHAPE_INDEX::Remove` recomputes the bbox from the item's *current* shape (`shape_index.h:241-248`). So **if a `PNS::ITEM`'s shape mutates between Add and Remove, the removal silently fails and leaves a stale pointer in the tree** while `m_allItems`/`m_netMap` are updated anyway (`pns_index.cpp:97-101`). PNS avoids this by treating indexed items as immutable.

### 10.5 Copy-on-write is the whole point

`dynamic_rtree_cow.h:33-49` states the rationale: *"Provides O(1) Clone() for the PNS router's branching pattern. The router frequently creates speculative branches that share most of their spatial index with the parent."* The chain is `PNS::NODE::Branch()` (`pns_node.cpp:174`) to `INDEX::Clone` (`pns_index.h:71-81`) to `SHAPE_INDEX::Clone` (`shape_index.h:201-206`) to `COW_RTREE::Clone` (`dynamic_rtree_cow.h:114-131`), which only bumps a root refcount. Note the asymmetry: the **tree** share is O(1), but `m_allItems` (an `unordered_set`) and `m_netMap` (a map of lists) are copied eagerly, so `INDEX::Clone` is O(items) despite the doc comment at `pns_index.h:69`.

Refcounts are `std::atomic<int>` initialized to 1 (`rtree_node.h:322-324`), incremented `relaxed` (`dynamic_rtree_cow.h:121`, `:176`, `:477`) and decremented `acq_rel` (`:500`). That makes refcounting sound but **the container is not thread-safe**: `ensureWritable` (`dynamic_rtree_cow.h:449-489`) loads then copies without a CAS. Treat it as `Send` but not `Sync`. Node memory uses a per-clone `SLAB_ALLOCATOR` prepended onto an immutable shared linked list `ALLOC_CHAIN` (`dynamic_rtree_cow.h:59-63`, `:123-130`), with freeing walking the chain calling `allocator->Owns(node)` (`:520-535`) and `NODES_PER_PAGE = 256` (`rtree_node.h:557`). A Rust port should discard this entirely; per-node `Arc` gives the same lifetime guarantee.

### 10.6 The query radius

`PNS::NODE::m_maxClearance` is initialized to **`800000` nm = 0.8 mm** with the comment `// fixme: depends on how thick traces are.` (`pcbnew/router/pns_node.cpp:62`). It is the `aMinDistance` passed to **every** index query (`pns_node.cpp:285`, `:291`, `:581`, `:588`), i.e. the amount every query box is inflated by. It propagates to child branches (`pns_node.cpp:167`) and is settable (`pns_node.h:282`); the KiCad interface does set it to `worstClearance + ClearanceEpsilon()` at `pns_kicad_iface.cpp:2452`, but the constructor default is what a standalone port would inherit. Keep it configurable.

## 11. `SHAPE_POLY_SET`, only as far as the router needs it

The router core uses `SHAPE_POLY_SET` in **two functions**, for the same purpose, and calls exactly **three** of its members.

* `PNS::SOLID::Hull` (`pcbnew/router/pns_solid.cpp:39-71`): `SHAPE_POLY_SET hullSet;` (`:55`), `hullSet.AddOutline( BuildHullForPrimitiveShape(...) )` per child of a `SHAPE_COMPOUND` (`:59`), `hullSet.Simplify()` (`:63`), `return hullSet.Outline( 0 )` (`:64`).
* `PNS::HOLE::Hull` (`pcbnew/router/pns_hole.cpp:57-100`): identical block at `:84`, `:88`, `:92`, `:93`.

That is the whole surface. No `BooleanAdd`/`Subtract`/`Intersection`/`Xor`, no `Inflate`/`Deflate`, no `Fracture`/`Unfracture`, no `CacheTriangulation`, no `Chamfer`/`Fillet`, no `OutlineCount`. The includes in `pns_item.cpp:28` and `pns_utils.cpp:33` are dead.

**Both call sites keep only outline 0 and discard any holes.** If the compound's primitive hulls are disjoint, the union produces more than one outline and the rest are silently dropped. `Outline(int)` is `m_polys[aIndex][0]` with no bounds check (`shape_poly_set.h:760`), so an empty result is UB.

Semantics of the three members:

* `AddOutline(const SHAPE_LINE_CHAIN&)` (`shape_poly_set.cpp:549-565`) pushes a new single-contour `POLYGON` and auto-closes the chain (`:558-560`). No Clipper involvement. The PNS hull builders already set closed, so the auto-close is a no-op there.
* `Simplify()` (`shape_poly_set.cpp:2226-2234`, declared `shape_poly_set.h:1120`) is `splitCollinearOutlines(); booleanOp( Clipper2Lib::ClipType::Union, empty );`. `splitCollinearOutlines` (`:1912`) builds a `KIRTREE::DYNAMIC_RTREE` over the outline segments and splits at "waists" where two non-adjacent segments are `ApproxCollinear(other, 10)` (`:1966`) and pass `isExteriorWaist` (`:1867`). Then the boolean does the real cleanup.
* `Outline(int)` returns `m_polys[aIndex][0]` by reference (`shape_poly_set.h:760`).

Data model: `typedef std::vector<SHAPE_LINE_CHAIN> POLYGON;` where entry 0 is the outline and the rest are holes (`shape_poly_set.h:73`), and `std::vector<POLYGON> m_polys;` (`shape_poly_set.h:1603`).

Clipper2 delegation, since `Simplify()` is a boolean:

| Public | Clipper2 `ClipType` | Line |
|---|---|---|
| `BooleanAdd` | `Union` | `shape_poly_set.cpp:867` |
| `BooleanSubtract` | `Difference` | `:873` |
| `BooleanIntersection` | `Intersection` | `:879` |
| `BooleanXor` | `Xor` | `:885` |
| `Simplify` | `Union` against an empty set | `:2231` |

`booleanOp` (`shape_poly_set.cpp:759-864`) converts each contour with `poly[i].convertToClipper2( i == 0, zValues, arcBuffer )` (`:783`, `:791`), so **outline 0 is forced to positive area and holes to negative** (enforced in `shape_line_chain.cpp:165-169`). The fill rule is **always `FillRule::NonZero`** (`shape_poly_set.cpp:860`); there is no `EvenOdd` path and no legacy `PolyFillType` anywhere. It `wxFAIL_MSG`es (but does not bail) if arcs are present in a multi-outline boolean (`:762-768`).

Arc preservation across Clipper uses the 64-bit `Z` field as an **index into a side table**, not as a coordinate. `CLIPPER_Z_VALUE { ssize_t m_FirstArcIdx, m_SecondArcIdx; }` (`shape_line_chain.h:37-62`, defaulting to `{-1,-1}`) is pushed per vertex by `convertToClipper2` (`shape_line_chain.cpp:157-188`), and the `ZCallback64` (`shape_poly_set.cpp:806-856`, registered at `:858`) reconstructs one for every manufactured intersection vertex, treating an edge as belonging to an arc only when **both** endpoints agree (`arcSegment`, `:825-836`). It carries a `@todo` at `:858` that the X/Y of the new point is Clipper's straight-segment intersection, **not** the true arc intersection. Net effect: **an arc survives a boolean round trip only when a whole edge stays inside it.**

Since PNS hulls are pure segment polygons (`OctagonalHull`/`SegmentHull` build straight chains, `ArcHull` flattens first), every `CLIPPER_Z_VALUE` on the router's path is `{-1,-1}` and the arc machinery is dead weight there.

`SHAPE_LINE_CHAIN::BuildPolygon` **does not exist**. The conversion is `TransformToPolygon` (`shape_line_chain.cpp:3124-3128`, which ignores its error arguments and is just `aBuffer.AddOutline(*this)`) or `SHAPE_POLY_SET::AddOutline` directly.

The GUI glue is where the real poly-set traffic lives, and it is all one-way: `pns_kicad_iface.cpp:1731-1734` turns a pad's effective polygon into a `SHAPE_SIMPLE`; `pns_kicad_iface.cpp:1905-1935` triangulates a zone with `CacheTriangulation()` and turns **every triangle into its own `PNS::SOLID`** with a `SHAPE_SIMPLE`; `syncTextItem` and `syncDimension` do `TransformShapeToPolygon` + `Simplify()` + per-outline `SHAPE_SIMPLE`. `router_preview_item.cpp:515-516` refuses to draw a `SH_POLY_SET` at all. **The router's world model contains no `SHAPE_POLY_SET`.** That is a significant simplification for a port: zones arrive pre-triangulated.

## 12. Hulls

Hulls are the router's own geometry layer, built on `SHAPE_LINE_CHAIN` and `SEG`. All of it lives in `pcbnew/router/pns_utils.cpp`. Everything is an **octagon**, because an octagon is the tightest convex shape whose edges are all on the 45-degree grid the router routes on.

### 12.1 `OctagonalHull`

`pns_utils.cpp:40-68`, declared `pns_utils.h:44-45`. The bbox inflated by `aClearance`, with each corner cut back by `aChamfer` along both axes. Every diagonal is guarded by `if( aChamfer )` (`:49`, `:54`, `:59`, `:64`), so `aChamfer == 0` degenerates to a rectangle. Closed on construction (`:45`). This is the primitive everything else reduces to.

The recurring chamfer formula for a circle of radius `r` at clearance `cl` is `2.0 * (1.0 - M_SQRT1_2) * (r + cl)` (`pns_utils.cpp:82`, `:501`), i.e. the equilateral-octagon corner cut, with the via/hole variant written as `(2*cl + width) * (1.0 - M_SQRT1_2)` (`pns_via.cpp:249`, `pns_hole.cpp:69`).

### 12.2 `SegmentHull` and `ArcHull`

`SegmentHull` (`pns_utils.cpp:181-286`) approximates a stadium (capsule) as an octagon. Geometry: `d = width/2 + cl`, `x = 2.0 / (1.0 + M_SQRT2) * d` (the octagon side/apothem ratio), `dr = KiROUND(d)`, `xr2 = KiROUND(x / 2.0)` (`:187-190`), then eight points built from the along and perpendicular vectors resized to `dr` and `xr2` (`:262-279`).

Most of the function is **kink correction** (`:207-248`), which exists because a very short segment's direction is numerically unreliable. `kinkThreshold = aClearance / 10` (`:184`). If the segment is shorter than that:

* not near 45 degrees: snap the endpoint so the segment becomes an exact 45 (`:211-216`);
* near vertical (`|w| <= 1`): force `w = 0` and `cl++` (`:223-227`);
* near horizontal (`|h| <= 1`): force `h = 0` and `cl++` (`:228-232`);
* near 45 (`||w| - |h|| <= 2`): snap both and `cl += 2` (`:233-242`).

The `cl` bumps compensate for the error the snapping introduces. `IsSegment45Degree` (`pns_utils.cpp:157-173`) is the 1 nm slop test that drives the branch.

`ArcHull` (`pns_utils.cpp:71-154`) flattens with `ConvertToPolyline( ARC_LOW_DEF )` (`:88`) and then miters offset lines at each vertex by intersecting them (`:110-131`), building the outer boundary forward and the inner boundary in reverse. Offset distance `d = width/2 + cl + SHAPE_ARC::DefaultAccuracyForPCB()` (`:85`), the last term paying for the flattening error. If the arc sweeps more than 180 degrees and its chord is shorter than `cl` it degenerates to a circle hull (`:76-83`).

Both end by forcing clockwise orientation via `if( s.CSegment( 0 ).Side( a ) < 0 ) return s.Reverse();` (`pns_utils.cpp:150-153`, `:282-285`).

Half-width rounding is **inconsistent** across the family: `ArcHull` and `BuildHullForPrimitiveShape` use `(aWalkaroundThickness + 1) / 2` (`pns_utils.cpp:73`, `:481`), while `SegmentHull`, `VIA::Hull` and `HOLE::Hull` use `aWalkaroundThickness / 2` (`pns_utils.cpp:186`, `pns_via.cpp:240`, `pns_hole.cpp:65`).

### 12.3 `PNS::ConvexHull` versus kimath's `BuildConvexHull`

`PNS::ConvexHull(const SHAPE_SIMPLE& aConvex, int aClearance)` (`pns_utils.cpp:300-353`, declared `pns_utils.h:58`) is **not a convex hull algorithm**; the input is assumed convex already. It takes the four axis-aligned lines of `aConvex.BBox(aClearance)` (`:306-312`), adds four 45-degree diagonals, slides each diagonal inward with `MoveDiagonal` (`:289-297`) until it is exactly `aClearance` from the nearest vertex, and intersects consecutive lines to get the eight octagon corners (`:343-350`). The diagonals are seeded with length `box.GetHeight()` on both axes (`:319`, `:325`, `:331`, `:337`), an implicit assumption that height is large enough to span the box.

kimath's `BuildConvexHull` (`libs/kimath/include/geometry/convex_hull.h:38`, `:46`, `:56`; implementation `libs/kimath/src/geometry/convex_hull.cpp:83-155`) is Andrew's monotone chain, O(n log n), returning `std::vector<VECTOR2I>` counter-clockwise, with `typedef long long coord2_t` for the cross products (`convex_hull.cpp:62`) and `<= 0` in the pop test so **collinear points are dropped** (`:106`, `:115`). The polyset overloads flatten outlines only, ignoring holes (`:142-148`). **The router never calls it.** Its only consumers are `pcbnew/footprint.cpp:2476`, `:2541` and `pcbnew/zone_filler.cpp:1924`.

### 12.4 The dispatcher and `HullIntersection`

`BuildHullForPrimitiveShape(const SHAPE*, int aClearance, int aWalkaroundThickness)` (`pns_utils.cpp:478-541`), with `cl = aClearance + (aWalkaroundThickness + 1) / 2` (`:481`):

| Type | Result | Line |
|---|---|---|
| `SH_RECT` | `OctagonalHull( pos, size, cl, 0 )`, i.e. a plain rectangle | `:485-492` |
| `SH_CIRCLE` | `OctagonalHull( c - (r,r), (2r,2r), cl, 2(1 - 1/sqrt2)(r + cl) )` | `:494-502` |
| `SH_SEGMENT` | `SegmentHull( *seg, aClearance, aWalkaroundThickness )` (raw args) | `:504-508` |
| `SH_ARC` | `ArcHull( *arc, aClearance, aWalkaroundThickness )` (raw args) | `:510-514` |
| `SH_SIMPLE` | `PNS::ConvexHull( *convex, cl )` (combined `cl`) | `:516-521` |
| `SH_ELLIPSE` | `OctagonalHull( bbox.pos, bbox.size, cl, 0 )` | `:523-529` |
| default | `wxFAIL_MSG`, empty chain | `:531-540` |

`HullIntersection(hull, line, ips)` (`pns_utils.cpp:395-475`, declared `pns_utils.h:65-66`) is the filter between `SHAPE_LINE_CHAIN::Intersect` and the walkaround. It takes the raw intersections, passes through any hit that is not a corner on either side (`:416-421`), and for corner hits keeps the record only if some adjacent hull segment has a neighbouring line point on its **positive** side (`:455-464`). It wraps `p.index_our` modulo `hull.SegmentCount()` first (`:423-424`), which is the compensation for the corner-index aliasing described in 6.6.

`PNS::HULL_MARGIN` is `10` nm (`pns_utils.h:34`), duplicated as `#define PNS_HULL_MARGIN 10` in `pns_line.h:45` and used for joint hull sizing at `pns_node.cpp:1338`, `:1348` and diff-pair clearance at `pns_diff_pair_placer.cpp:247`.

## 13. Magic constants and epsilons, consolidated

Every value below was read from the tree at the cited line.

**kimath, numeric.**

| Value | Meaning | Location |
|---|---|---|
| `int64_t` | `VECTOR2<int>::extended_type` | `math/vector2d.h:49` |
| `INT64_MAX` / `INT64_MIN` | `VECTOR2I::ECOORD_MAX` / `ECOORD_MIN` | `math/vector2d.h:72-73` |
| `M_SQRT2`, `M_SQRT1_2` | exact-diagonal fast paths in `EuclideanNorm`, `Resize` | `math/vector2d.h:284`, `:391` |
| round-half-away-from-zero, clamped | `KiROUND` | `math/util.h:98-124` |
| round-to-nearest, `i64` intermediate | `rescale(int,int,int)` | `src/math/util.cpp:62-72` |
| `__int128` intermediate | `rescale(int64_t,...)` | `src/math/util.cpp:106-113` |

**kimath, geometry tolerances.**

| Value | Meaning | Location |
|---|---|---|
| `4` | `SHAPE::MIN_PRECISION_IU` | `geometry/shape.h:129` |
| `4` | `CIRCLE::Contains` / `Intersect` band | `src/geometry/circle.cpp:192-193`, `:351`, `:355-356` |
| `4` | endpoint re-snap in `BuildInitialTrace` ROUNDED_45 | `src/geometry/direction_45.cpp:202`, `:207` |
| `<= 3` (squared) | `SEG::Contains(VECTOR2I)` | `src/geometry/seg.cpp:623-626` |
| `<= 1` (determinant) | `SEG::Collinear` | `geometry/seg.h:290` |
| `1` | default `aDistanceThreshold` for `ApproxCollinear`/`ApproxParallel` | `geometry/seg.h:293-294` |
| `5000` nm | `ARC_HIGH_DEF` = `mmToIU(0.005)` | `include/base_units.h:128`, `:137` |
| `20000` nm | `ARC_LOW_DEF` = `mmToIU(0.02)` | `include/base_units.h:127`, `:136` |
| `ARC_HIGH_DEF / 5` = `1000` nm | `getArcPolygonizationMaxError` | `src/geometry/shape_line_chain.cpp:57-62` |
| `max(1.0, aMaxError/2.0)` | `ConvertToPolyline` half-error | `src/geometry/shape_arc.cpp:1022` |
| `8` | `MIN_SEGCOUNT_FOR_CIRCLE` | `src/geometry/geometry_utils.cpp:36` |
| `>= 2` segments | `GetArcToSegmentCount` floor | `src/geometry/geometry_utils.cpp:58` |
| `10` | `ApproxCollinear` threshold in `splitCollinearOutlines` | `src/geometry/shape_poly_set.cpp:1966` |

**SHAPE_LINE_CHAIN` sentinels and thresholds.**

| Value | Meaning | Location |
|---|---|---|
| `-1` | `SHAPE_IS_PT` | decl `shape_line_chain.h:968`, def `shape_line_chain.cpp:42` |
| `{-1,-1}` | `SHAPES_ARE_PT` | decl `:970`, def `:43` |
| `{-1,-1}` | `CLIPPER_Z_VALUE` "no arc" | `shape_line_chain.h:41-42` |
| `-1` | `INTERSECTION::index_our`/`index_their` default | `shape_line_chain.h:112-113` |
| `-1` | `NextShape`/`Find`/`FindSegment`/`PathLength`/`EdgeContainingPoint` failure | `shape_line_chain.cpp:1308`, `:1253`, `:1265`, `:1982`, `:2109` |
| `2` | `Split` hit threshold (`Distance < 2`) | `shape_line_chain.cpp:1184` |
| `1` | `FindSegment` default threshold | `shape_line_chain.h:633` |
| `aAccuracy + 1`, squared | `EdgeContainingPoint` | `shape_line_chain.cpp:2082-2083` |
| `aAccuracy <= 1` | `PointInside` fast path | `shape_line_chain.cpp:2016` |
| `0` | `Simplify` default tolerance (exact colinearity) | `shape_line_chain.h:358` |
| `<= 1` | `Simplify2` colinearity tolerance | `shape_line_chain.cpp:2972` |
| `+/- 2` | `SelfIntersecting` bbox padding | `shape_line_chain.cpp:2156-2160` |
| `<= 2.0` (squared) | `SelfIntersectingWithArcs::pointsClose` | `shape_line_chain.cpp:2237-2240` |
| `100`, `20`, `5` | `ClosestSegmentsFast` bucketing (unused by the router) | `shape_line_chain.cpp:511`, `:512`, `:620` |

There are **no `ARC_START` / `ARC_END` / `ARC_MID` sentinels**. Arc start and end are determined structurally by comparing `m_points[i]` against `SHAPE_ARC::GetP0()`/`GetP1()`, or by `IsSharedPt`.

**Collision.**

| Value | Meaning | Location |
|---|---|---|
| `+3` | MTV magnitude bias, with an in-source `fixme` | `shape_collisions.cpp:58`, `:626`, `:753`, `:885` |
| `+1` twice | MTV bias, rect vs circle | `shape_collisions.cpp:142`, `:144` |
| `corr < 5` | pushout length search cap, 1 nm steps | `shape_collisions.cpp:168`, `:170` |
| `max(1, clearance/4)` | arc/ellipse tessellation error | `shape_collisions.cpp:1005`, `:1040` |
| `w / 2` vs `(w + 1) / 2` | inconsistent half-widths | `shape_collisions.cpp:336` etc vs `shape_segment.h:83`, `:90` |
| `INT_MAX/2` | clearance passed by `SHAPE::GetClearance` | `src/geometry/shape.cpp:94` |
| max magnitude, not sum | compound MTV aggregation | `shape_collisions.cpp:1348` |

**Index.**

| Value | Meaning | Location |
|---|---|---|
| `800000` nm | `NODE::m_maxClearance`, the query inflation, with a `fixme` | `pcbnew/router/pns_node.cpp:62` |
| `16` | R-tree fanout `TMAXNODES` | `rtree/dynamic_rtree_cow.h:50` |
| `MAXNODES * 2 / 5` = 6 | `MINNODES` | `rtree/rtree_node.h:303` |
| `MAXNODES * 3 / 10` = 4 | `REINSERT_COUNT` (non-CoW tree only) | `rtree/dynamic_rtree.h:68` |
| `<= 31`, `% 4 == 0` | fanout static asserts | `rtree/rtree_node.h:156-157` |
| Hilbert order 16 vs 32 | CoW vs non-CoW bulk load | `dynamic_rtree_cow.h:273` vs `dynamic_rtree.h:284` |
| `256`, `alignas(64)` | slab allocator page | `rtree/rtree_node.h:557` |
| `-1` | "any layer" sentinel | `pcbnew/router/pns_node.cpp:256`, `geometry/shape_index_list.h:34` |

**Router-side geometry.**

| Value | Meaning | Location |
|---|---|---|
| `10` nm | `HULL_MARGIN` / `PNS_HULL_MARGIN` | `pns_utils.h:34`, `pns_line.h:45` |
| `-1` | the "hulls are built to exactly the clearance" collision fudge | `pns_item.cpp:249`, `:280` (comment `:246-248`) |
| `-1` | "no clearance rule, skip" | `pns_item.cpp:191`, gated `:224` |
| `aClearance / 10` | `SegmentHull` kink threshold | `pns_utils.cpp:184` |
| `<= 1`, `<= 2` | 45-degree slop tests, with `cl++` / `cl += 2` compensation | `pns_utils.cpp:161-169`, `:223-243` |
| `2.0 * (1.0 - M_SQRT1_2)` | octagon chamfer factor | `pns_utils.cpp:82`, `:252`, `:501` |
| `2.0 / (1.0 + M_SQRT2)` | octagon side/apothem ratio | `pns_utils.cpp:86`, `:188` |
| `1.3`, `0.25`, `6` | posture solver thresholds | `pns_mouse_trail_tracer.cpp:87`, `:90`, `:93` |
| `5` nm, `0.38268`, `0.39875` | diff-pair gateway epsilon and sin(22.5)/sin(23.5) | `pns_diff_pair.cpp:612-614` |
| `1000` | walkaround iteration limit in `LINE::Walkaround` | `pns_line.cpp:498` |
| `Diameter / 4` | via pushout magnitude heuristic, commented "another stupid heuristic" | `pns_via.cpp:181` |

## 14. Rust mapping notes

### 14.1 The numeric core

`VECTOR2I` maps to `#[derive(Copy, Clone, PartialEq, Eq, Hash)] struct Vec2 { pub x: i32, pub y: i32 }`. Nanometre coordinates in `i32` cover +/- 2.1 m, which is enough for any board, so keep the width.

Products of coordinates must be `i64`. Everything KiCad computes in `ecoord` (`Cross`, `Dot`, `SquaredEuclideanNorm`, `SEG::SquaredDistance`, all determinants) fits in `i64` for `i32` inputs, but with almost no headroom: the worst case `2 * (2^31)^2` is about `9.2e18` against `i64::MAX` of `9.223e18`. Two places genuinely need `i128`: `rescale(i64, i64, i64)`, which KiCad itself does in `__int128` (`src/math/util.cpp:106-113`) and which is called from `SEG::NearestPoint`, `LineProject`, `LineDistance` and `intersects`, all hot; and `min_dist * min_dist` where `min_dist` is a clearance plus two radii (`shape_collisions.cpp:44`), which KiCad computes in `i64` and can marginally overflow.

`SquaredEuclideanNorm` and friends are safe in `i64` **provided the difference is computed first in `i64`**. KiCad's `operator-` on two `VECTOR2I` is 32-bit and wraps (`vector2d.h:466-471`); make `Vec2::sub` return a widened `Vec2L { x: i64, y: i64 }` instead. Do not reproduce the wrap.

`EuclideanNorm`, `Resize` and every `sqrt` in the collision code are `f64`-backed. Reproduce the 45-degree special cases in `EuclideanNorm` (`vector2d.h:281-289`) and `Resize` (`vector2d.h:389-392`) exactly, or diagonals will round differently from KiCad and hulls will drift by a nanometre in ways that change optimizer decisions. `KiROUND` is `llround` (half **away from zero**) then clamp; Rust's `f64::round` has the same tie rule and `as i32` saturates rather than being UB, which is what you want. `isqrt` (`seg.cpp:57-72`) becomes `u64::isqrt`; note that `SEG::Distance` truncates while `shape_collisions.cpp` sometimes rounds, so one exact integer sqrt will not be bit-identical to KiCad everywhere. Decide up front whether bit-compatibility or correctness is the goal, because you cannot have both.

`BOX2I` should become `Box2i { min: Vec2L, max: Vec2L }` wrapped in `Option`, which removes the `m_init` tri-state (`box2.h:923`) and the negative-size mode in one move; the router only needs `Merge`, `Contains`, `Intersects`, `Inflate`, `GetCenter`, the four edge accessors and `SquaredDistance`. `SetMaximum()` (`box2.h:77`) becomes a constant. `EDA_ANGLE` can be a newtype over `f64` degrees, or dropped entirely given its 11 uses.

### 14.2 The shape hierarchy

Do **not** reproduce the `SHAPE` virtual hierarchy; use an enum `Shape { Rect, Segment, LineChain, Circle, Simple, Arc, Compound(Vec<Shape>), Null }`. This buys three things: the collision dispatcher becomes a `match` over the pair and the compiler names the missing cells instead of letting them reach `wxFAIL_MSG` and return `false` at runtime (`shape_collisions.cpp:1300-1304`); `Compound(Vec<Shape>)` makes nesting representable and forces a decision about what it means, instead of silently asserting (9.1); and `Clone` stops being a virtual that asserts and returns null (`shape.h:146-150`).

`SHAPE_POLY_SET` and `SHAPE_ELLIPSE` can be left out of the router's enum entirely: the router's world model contains neither (11), and `SH_ELLIPSE` appears only in the hull dispatcher where it degrades to a bbox. `SHAPE_SIMPLE` is a `LineChain` with a forced-closed invariant, so make it a newtype. Keep `SHAPE_RECT::m_radius` (`shape_rect.h:252`) since the collision code branches on it (`shape_collisions.cpp:70-73`), even though the router never sets it. `PNS::SOLID` and `PNS::HOLE` hold raw owning `SHAPE*` with manual `delete` (`pns_solid.h:54`, `:115-116`, `pns_hole.h:94`); with the enum these become `Shape` by value and the deep-copy-on-clone semantics (`pns_solid.h:62-63`) become `#[derive(Clone)]`.

### 14.3 `LineChain`

This deserves the most design attention, because it is where the router lives and where the C++ has the most accidental behaviour.

```rust
pub struct LineChain {
    points: Vec<Vec2>,
    shapes: Vec<ArcRef>,     // parallel to points, invariant enforced by construction
    arcs:   Vec<ShapeArc>,
    closed: bool,
    width:  i32,
}

#[derive(Copy, Clone, PartialEq, Eq, Default)]
pub enum ArcRef {
    #[default] Plain,
    On(ArcIdx),                              // interior or endpoint of one arc
    Shared { ends: ArcIdx, starts: ArcIdx }, // end of arc N, start of arc N+1
}
```

Making `ArcRef` an enum rather than an `(isize, isize)` pair removes the `SHAPE_IS_PT == -1` sentinel and makes the "second must be -1 if first is" invariant (`shape_line_chain.h:987`) unrepresentable. Keep `points.len() == shapes.len()` enforced by mutating only through paired methods.

Things to change deliberately rather than port: **drop the bbox cache** (it is never invalidated, 6.8 item 6, `BBox()` recomputes anyway, and a stale cache silently produces false negatives in `PointInside`); **make indices explicit**, since `CPoint`'s single-step wrap (`shape_line_chain.h:422-425`) is a footgun and `ArcIndex`/`Arc`/`CLastPoint` are UB out of range; **return `Option`/`Result` from `Slice`** instead of an empty chain on every `wxCHECK` failure (`shape_line_chain.cpp:1429-1433`), and consider a wrapping slice for closed chains so callers stop doing two-slice-and-concatenate; **fix the `rend() + aVertex` bug** in `Insert(usize, ShapeArc, i32)` (`shape_line_chain.cpp:1674-1676`); have `Intersect()` return a `Vec` rather than appending to an out-parameter and returning the total length (6.8 item 10); invert or rename `aExcludeColinearAndTouching` so it reads correctly (6.8 item 11); and drop `INTERSECTION::valid`, which is only meaningful inside `HullIntersection`'s filtering (`pns_utils.cpp:414`) and belongs there.

Things to keep exactly: `Append`'s duplicate suppression (`shape_line_chain.h:539`), which looks like a wart but the placer depends on it; `SegmentCount() == PointCount()` for a closed chain (`shape_line_chain.h:327-335`), computed in signed arithmetic rather than relying on `size_t` underflow; and **both** `Simplify` and `Simplify2` with their different tolerances (6.5), because the optimizer's convergence depends on `Simplify2`'s 1 nm slack.

The index aliasing in `INTERSECTION` (an index equal to `SegmentCount()` when the hit is on a `B` endpoint, `shape_line_chain.cpp:1896` and friends) should become an explicit `enum Hit { Segment(usize), Corner(usize) }`, which makes `HullIntersection`'s modulo compensation (`pns_utils.cpp:423-424`) unnecessary.

### 14.4 `Direction45`

`enum Octant { N, NE, E, SE, S, SW, W, NW }` plus `struct Direction45 { dir: Option<Octant>, ninety_deg: bool }`. `Angle()` is a `match` on `(a as i8 - b as i8).abs()` (`direction45.h:186-197`), and `AngleType` should be a `bitflags!` type since the router ORs the values into masks (`pns_optimizer.cpp:665`, `:1114`).

`construct_` (`direction45.h:317-343`) uses `atan2` and rounds to the nearest octant. An integer classification from `(sign(x), sign(y), |x| vs |y|)` gives the same answer for every exact-octant vector and differs only in how near-octant vectors round; since `MOUSE_TRAIL_TRACER` feeds it raw cursor deltas, verify against the float version on near-45 cases before switching. `BuildInitialTrace` (8.3) is a direct port; its `ROUNDED_45` branch is the only part needing `f64`, and the `MIN_PRECISION_IU` endpoint re-snap (`direction_45.cpp:202-211`) must survive.

### 14.5 Crates: what to reuse and what to hand-roll

**R-tree: `rstar`, with one caveat.** It is a proper R*-tree with insert, remove and bulk load, and its `RTreeObject` trait maps onto "give me an AABB". It is a better tree than KiCad's `COW_RTREE`, whose `simpleSplit` is admittedly broken for non-root splits (`dynamic_rtree_cow.h:723-725`). The caveat is the copy-on-write requirement (10.5): PNS branches its `NODE` constantly and needs O(1) index cloning, while `rstar` gives O(n). Three options in order of preference: (1) do not clone the tree at all, keeping one immutable base index for the root board plus a small per-branch overlay of added and removed items merged at query time, which is close to what `PNS::NODE` already does logically (`pns_node.cpp:215-223`, `:291`, `:588` query the root index separately) and removes the CoW machinery entirely; (2) `Arc<RTree<..>>` rebuilt on structural change; (3) hand-roll a persistent R-tree with `Arc<Node>` and `Arc::make_mut` path copying, which is the direct translation but is real work and KiCad's own version has known defects. Whatever you pick, **store the insertion bbox alongside the handle** so removal cannot silently fail when an item's shape has mutated (10.4). `SHAPE_INDEX_LIST` (the optimizer's cache, `pns_optimizer.h:204`) is a `Vec` scan and should stay one.

**Polygon booleans: probably not needed at all.** The router core performs exactly one boolean, `SHAPE_POLY_SET::Simplify()` (a self-union), and only in `SOLID::Hull` and `HOLE::Hull` for multi-primitive compound pads (11). If you must have it, `i_overlay` is the closest match: integer coordinates, the same fill rules, fast. `geo` is float-based and would reintroduce a rounding boundary the rest of the design avoids. KiCad's use here does not need arc preservation (all inputs are already flattened), so the entire `CLIPPER_Z_VALUE` mechanism can be skipped. An alternative worth measuring: replace the union with a genuine convex hull of the union of the primitive hull vertices, which is a conservative superset and removes the dependency completely.

**Everything else: hand-roll.** `SEG`, `Vec2`, `Box2i`, `LineChain`, `Direction45`, the octagonal hull builders and the collision dispatcher are all small, all exact-integer, and all carry KiCad-specific tolerances no general-purpose crate will match. Andrew's monotone chain is 40 lines (`convex_hull.cpp:83-127`) and the router does not call kimath's version anyway. `bitflags` and `smallvec` are worth having, the latter because most chains are short.

### 14.6 Things that are genuinely tricky to port

**Raw shared pointers and mutation through the index.** `PNS::INDEX` stores `ITEM*` (`pns_index.h:50`), `PNS::NODE` owns items, and `NODE::Branch()` hands a cloned index to a child while parent and child hold the same pointers (`pns_node.cpp:174`). Use arena indices (`ItemId(u32)` into a `Vec<Item>` owned by the router) rather than `Arc<Item>` with interior mutability: the index then stores plain integers, clone-on-branch is cheap, and the stale-bbox removal hazard disappears because removal keys on the id.

**Integer overflow in the collision inflation sums** (9.6). `aClearance + rA + rB` in `i32` is reachable through `SHAPE::GetClearance`'s `INT_MAX/2` (`src/geometry/shape.cpp:94`). In Rust this panics in debug and wraps in release, both worse than the C++ silent wrap; take the clearance as `i64` at the collision boundary and do the arithmetic there.

**No NaN handling.** `KiROUND` only guards NaN under C++23 (`math/util.h:102-113`). The float paths that could produce one are all guarded elsewhere: `Resize` on a zero vector (`vector2d.h:383-384`), `atan2(0,0)` in `construct_` (`direction45.h:321-322`), and `sqrt` of a negative in `SEG::SquaredDistance` (`seg.cpp:735-738`). Assert rather than rely on Rust's saturating cast matching.

**The `-1` in the collision clearance** (`pns_item.cpp:249`, `:280`, comment at `:246-248`). This single nanometre is what makes "touching the hull exactly" not a collision. If the hull builders and the collision predicate disagree by one nanometre, the walkaround loop can fail to terminate. Port it verbatim and put a test on it.

**MTV sign convention** (9.2). The C++ is inconsistent and the router compensates with a double negation (`pns_shove.cpp:1247-1255`). Pick one convention ("the vector to apply to the second argument"), enforce it with a newtype that knows which shape it displaces, and port `SHOVE::onCollidingVia` against the fixed convention rather than transcribing the negations.

**`Resize` with a negative length reverses direction** (`vector2d.h:410`). Several hull builders pass values that can go negative, notably `MoveDiagonal`'s `dist - aClearance` (`pns_utils.cpp:294`). This is intentional behaviour, not an accident.

**Order-dependent iterative MTV** (9.4). The circle-versus-chain pushout is a single relaxation pass over segments in chain order with no convergence check, capped at 5 correction steps. It is not a true minimum translation vector and its result depends on segment ordering, so a "better" implementation will change shove behaviour.

**`thread_local` scratch buffers** in `SHAPE_LINE_CHAIN::Intersect` (`shape_line_chain.cpp:1816-1817`). Use a caller-provided scratch struct or just allocate; the `thread_local` is a micro-optimization that will fight the borrow checker.

## 15. KiCad's own tests, for the port to mirror

`qa/tests/libs/kimath/` is present in the repository but not in this sparse checkout. From `git ls-tree -r --name-only HEAD qa/tests/libs/kimath`:

`qa/tests/libs/kimath/CMakeLists.txt`, `kimath_test_module.cpp`, `test_kimath.cpp`; under `math/`: `test_box2.cpp`, `test_matrix3x3.cpp`, `test_util.cpp`, `test_vector2.cpp`, `test_vector3.cpp`; under `geometry/`: `fixtures_geometry.h`, `geom_test_utils.{h,cpp}`, `bench_spatial_index.cpp`, `test_arc_chord_params.cpp`, `test_bezier.cpp`, `test_chamfer.cpp`, `test_circle.cpp`, `test_distribute.cpp`, `test_dogbone.cpp`, `test_dynamic_rtree.cpp`, `test_eda_angle.cpp`, `test_ellipse_to_bezier.cpp`, `test_fillet.cpp`, `test_half_line.cpp`, `test_intersection.cpp`, `test_oval.cpp`, `test_packed_rtree.cpp`, `test_poisson_disk.cpp`, `test_poly_triangulation.cpp`, `test_poly_ystripes_index.cpp`, `test_roundrect.cpp`, `test_segment.cpp`, `test_shape_arc.cpp`, `test_shape_compound_collision.cpp`, `test_shape_ellipse.cpp`, `test_shape_line_chain.cpp`, `test_shape_line_chain_collision.cpp`, `test_shape_nearest_points.cpp`, `test_shape_poly_set.cpp`, `test_shape_poly_set_arcs.cpp`, `test_shape_poly_set_collision.cpp`, `test_shape_poly_set_distance.cpp`, `test_shape_poly_set_iterator.cpp`, `test_shape_poly_set_split_outlines.cpp`, `test_shape_rect_corner.cpp`, `test_transform_trs.cpp`, `test_triangulation_benchmark.cpp`, `test_vector_utils.cpp`.


Directly relevant to the router's geometry foundation, in rough priority order for a port to mirror: `test_shape_line_chain.cpp`, `test_shape_line_chain_collision.cpp`, `test_segment.cpp`, `test_shape_arc.cpp`, `test_vector2.cpp`, `test_box2.cpp`, `test_util.cpp` (which covers `KiROUND` and `rescale`), `test_circle.cpp`, `test_shape_compound_collision.cpp`, `test_shape_nearest_points.cpp`, `test_dynamic_rtree.cpp`, `test_intersection.cpp`. `fixtures_geometry.h` and `geom_test_utils.h` carry the shared fixtures and are worth reading before writing the Rust equivalents.

There is **no `test_direction45.cpp` and no test for `shape_collisions.cpp` as a unit**. `DIRECTION_45::BuildInitialTrace` and the collision dispatch matrix are covered only indirectly, through `qa/tests/pcbnew/` and through the router's own regression fixtures. A Rust port should treat both as untested territory and write its own tables.

Router-level tests live elsewhere: `pcbnew/router/pns_logger.cpp` writes the router's event log, and KiCad replays recorded router sessions in its PNS regression suite. That is the right harness to mirror for behavioural parity, but it is outside the scope of this note.
