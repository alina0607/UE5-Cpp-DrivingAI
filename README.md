# Spline-Based AI Traffic Navigation in Unreal Engine 5

> M.S. Thesis Project — DigiPen Institute of Technology · 2025–2026

A scalable city traffic simulation in Unreal Engine 5 supporting **100+ concurrent AI vehicles** with smooth lane-change logic, dynamic spawning/despawning, and sub-frame recovery behaviours.

**[▶ Watch Demo on YouTube](https://www.youtube.com/watch?v=XKZPfm5rRmM)**

[![Demo Thumbnail](https://img.youtube.com/vi/XKZPfm5rRmM/maxresdefault.jpg)](https://www.youtube.com/watch?v=XKZPfm5rRmM)

---

## Overview

Building a convincing city traffic simulation means solving a chain of interconnected problems at runtime: Where do roads cross? What is the shortest path from here to there? How does a vehicle smoothly follow that path, know when to signal, and recover when it gets stuck behind another car?

This project walks through two performance-critical optimisations and the steering model that ties them together:

| Optimisation | Before | After |
|---|---|---|
| Junction Detection | O(R² × S²) brute-force | O(N) Grid Hash spatial index |
| A\* Pathfinding | O(n²) linear scan | O((V+E) log V) binary min-heap |
| Startup cost (R=100, S=200) | 400,000,000 operations | ~1,000,000 operations (**400× speedup**) |

---

## System Architecture

The system runs across three layers at different frequencies:

```
Level Load   →  DetectMidSplineJunctions  →  builds road graph (nodes + edges)
On Demand    →  FindPathAStar             →  returns ordered node path
Every Tick   →  TickComponent            →  pursuit steering, speed control, lane blending
```

---

## Optimisation 1 — Grid Hash Junction Detection

### The Problem (Brute-Force O(R² × S²))

With R roads and S sample points per road, naive detection compares every sample pair across every road combination — 4 nested loops.

```
With R=100, S=200 → 100² × 200² = 400,000,000 distance computations at startup
```

### The Solution (Grid Hash O(N))

Divides the world into uniform cells of side `CellSize`. Each sample is inserted into exactly one cell. Only the **3×3×3 neighbourhood (27 cells)** is searched — all other cells are skipped with zero work.

```cpp
// Insert sample into grid
FIntVector Cell(
    FMath::FloorToInt(S.Location.X / CellSize),
    FMath::FloorToInt(S.Location.Y / CellSize),
    FMath::FloorToInt(S.Location.Z / CellSize));
Grid.FindOrAdd(Cell).Add(Idx);

// Query only the 3×3×3 neighbourhood
for (int32 DX = -1; DX <= 1; ++DX)
  for (int32 DY = -1; DY <= 1; ++DY)
    for (int32 DZ = -1; DZ <= 1; ++DZ)
      if (auto* List = Grid.Find({Cell.X+DX, Cell.Y+DY, Cell.Z+DZ}))
        // compare pairs in *List only
```

K (average points per neighbouring cell) is **constant** — it does not grow as more roads are added. Total work: **O(N × K) ≈ O(N)**.

---

## Optimisation 2 — A\* Binary Min-Heap

### Before (O(n²))

Open set stored as plain `TArray<int32>`. Finding the lowest F-score required a full linear scan; removing it required `TArray::Remove` — both O(n), repeated once per expanded node.

### After (O((V+E) log V)) with Lazy Deletion

Uses Unreal's `HeapPush` / `HeapPop` on `TArray<TPair<float,int32>>`. When a shorter path is found, a new entry is pushed — stale entries are discarded on pop via `TSet<int32> ClosedSet` in O(1).

```cpp
while (OpenHeap.Num() > 0) {
    FEntry Top;
    OpenHeap.HeapPop(Top, Pred);         // O(log N)
    if (ClosedSet.Contains(Top.Value))   // discard stale
        continue;
    ClosedSet.Add(Top.Value);

    for (const auto& Edge : Edges) {
        const float NewG = GScore[Cur] + Edge.Cost;
        if (NewG < GScore.FindRef(Nb)) {
            GScore[Nb] = NewG;
            OpenHeap.HeapPush({NewG + H(Nb, Goal), Nb}, Pred); // lazy push
        }
    }
}
```

| Operation | Before | After |
|---|---|---|
| Find min F-score | Linear scan O(n) | HeapPop O(log n) |
| Remove processed node | TArray::Remove O(n) | ClosedSet skip O(1) |
| Open set membership | TArray::Contains O(n) | TSet::Contains O(1) |

---

## Pursuit Point Steering

Each tick the vehicle steers toward a **look-ahead point** ahead on the path — not the nearest spline position. Look-ahead distance scales with speed:

```
AheadDist = max(CurrentSpeed × LookAheadTime, MinLookAheadDist)
```

`GetPursuitPoint` walks forward through the segment array, subtracting each segment's remaining length from the budget until spent, then samples the spline at the exact position — handling cross-segment look-ahead seamlessly.

---

## Turn Signal & Direction Detection

`ComputeTurnAtJunction` classifies each upcoming junction using **dot product** and **cross product**:

```cpp
const float Dot    = FVector::DotProduct(In2D, Out2D);
const float CrossZ = FVector::CrossProduct(In2D, Out2D).Z;

if      (Dot > JunctionStraightDot)  OutTurn = ETurnSignal::None;       // straight
else if (Dot < JunctionUTurnDot)     bOutUTurn = true;                  // U-turn
else    OutTurn = (CrossZ > 0.f) ? ETurnSignal::Right : ETurnSignal::Left;
```

Result drives both the **blinker activator** and the **Hermite arc radius scaler** for junction curve smoothing.

---

## Tech Stack

- **Language:** C++
- **Engine:** Unreal Engine 5
- **Key Systems:** USplineComponent, TMap, TSet, TArray heap operations

---

## References

- Hart et al. (1968). *A formal basis for the heuristic determination of minimum cost paths.* IEEE TSC.
- Coulter (1992). *Implementation of the pure pursuit path tracking algorithm.* CMU-RI-TR-92-01.
- Reynolds (1999). *Steering behaviors for autonomous characters.* GDC.
- Lefebvre & Hoppe (2006). *Perfect spatial hashing.* ACM TOG.

---

*Part of [Alina Chuang's Portfolio](https://alina0607.github.io)*
