
#include "RoadNetworkSubsystem.h"
#include "Kismet/GameplayStatics.h"
#include "RoadBPReflectionLibrary.h"
#include "RoadWorldSettings.h"
#include "Engine/World.h"
#include "Engine/Engine.h"

#include "RoadRuleLibrary.h"
#include "DrawDebugHelpers.h"

#include "Components/SplineComponent.h"
#include "Algo/Reverse.h"
#include "EngineUtils.h"
#include "ParkingLotActor.h"

/// <summary>
/// Initialization happens when the world creates this subsystem.
/// </summary>
void URoadNetworkSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    // Register a callback when the world begins play.
    if (UWorld* World = GetWorld())
    {
        World->OnWorldBeginPlay.AddUObject(this, &URoadNetworkSubsystem::HandleWorldBeginPlay);
    }
}

/// <summary>
/// Clear cached data before shutdown.
/// </summary>
void URoadNetworkSubsystem::Deinitialize()
{
    if (UWorld* World = GetWorld())
    {
        World->OnWorldBeginPlay.RemoveAll(this);
    }
    AllRoads.Empty();
    Super::Deinitialize();
}

/// <summary>
/// Get the current world's WorldSettings and cast it to our custom road settings type.
/// </summary>
const ARoadWorldSettings* URoadNetworkSubsystem::GetRoadWorldSettings() const
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return nullptr;
    }

    return Cast<ARoadWorldSettings>(World->GetWorldSettings());
}

void URoadNetworkSubsystem::HandleWorldBeginPlay() {

    BuildRoadCache();
}

/// <summary>
/// Scan the level for Blueprint road actors, extract their data,
/// and rebuild the entire road graph from scratch.
/// </summary>
void URoadNetworkSubsystem::BuildRoadCache()
{

    AllRoads.Empty();

    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Warning, TEXT("BuildRoadCache: World is null"));
        return;
    }
    const ARoadWorldSettings* RoadSettings = GetRoadWorldSettings();
    if (!RoadSettings)
    {
        UE_LOG(LogTemp, Warning, TEXT("BuildRoadCache: WorldSettings is not ARoadWorldSettings"));
        return;
    }
    if (!RoadSettings->RoadActorClass)
    {
        UE_LOG(LogTemp, Warning, TEXT("BuildRoadCache: RoadActorClass is not assigned in WorldSettings"));
        return;
    }


    TArray<AActor*> FoundRoadActors;
    UGameplayStatics::GetAllActorsOfClass(World, RoadSettings->RoadActorClass, FoundRoadActors);

    // Extract runtime road data from each Blueprint road actor.
    for (AActor* RoadActor : FoundRoadActors)
    {
        if (!RoadActor)
        {
            continue;
        }

        if (!RoadActor->ActorHasTag(TEXT("DriveRoad")))
        {
            continue;
        }

        FRoadRuntimeData RoadData;

        // Convert one Blueprint road actor into one FRoadRuntimeData entry.
        const bool bSuccess = URoadBPReflectionLibrary::ExtractRoadCoreData(RoadActor, RoadData);

        // Only store valid road data.
        if (bSuccess)
        {
            AllRoads.Add(RoadData);
        }
    }
    BuildEdgeCandidates();
    DetectMidSplineJunctions();
    BuildNodeCandidates();
    BuildGraphEdges();
    BuildGraphNodes();
    BindParkingActorsToEdges();
    DrawDebugGraph();

}

/// <summary>
/// Convert each cached road runtime entry into one edge candidate.
/// </summary>
void URoadNetworkSubsystem::BuildEdgeCandidates()
{
    EdgeCandidates.Empty();

    for (const FRoadRuntimeData& RoadData : AllRoads)
    {
        // Skip invalid entries that do not have a spline.
        if (!RoadData.InputSpline)
        {
            continue;
        }

        FRoadEdgeCandidate Candidate;

        // Copy the core road data into the edge candidate.
        Candidate.RoadActor = RoadData.RoadActor;
        Candidate.InputSpline = RoadData.InputSpline;
        Candidate.StartWorldLocation = RoadData.StartWorldLocation;
        Candidate.EndWorldLocation = RoadData.EndWorldLocation;
        Candidate.LengthMeters = RoadData.LengthMeters;
        Candidate.RoadType = RoadData.RoadType;
        Candidate.bClosedLoop = RoadData.bClosedLoop;
        Candidate.bTwoRoads = RoadData.bTwoRoads;
        Candidate.TwoRoadsGapM = RoadData.TwoRoadsGapM;
        Candidate.RoadWidthMultiplier = RoadData.RoadWidthMultiplier;
        Candidate.AdditionalWidthM = RoadData.AdditionalWidthM;
        Candidate.GuardrailSideOffsetCm = RoadData.GuardrailSideOffsetCm;
        Candidate.AutoLaneWidthCm = RoadData.AutoLaneWidthCm;
        Candidate.AutoMedianCm = RoadData.AutoMedianCm;

        EdgeCandidates.Add(Candidate);
    }
}

/// <summary>
/// Detect where road splines come close at non-endpoint locations.
/// Uses Grid Hash spatial index for O(N×K) performance instead of brute-force
/// Must be called after BuildEdgeCandidates().
/// </summary>
void URoadNetworkSubsystem::DetectMidSplineJunctions()
{
    JunctionCandidates.Empty();

    const ARoadWorldSettings* RoadSettings = GetRoadWorldSettings();
    if (!RoadSettings)
    {
        return;
    }

    const float SampleStep = RoadSettings->SplineSampleStep;
    const float DetectionRadius = RoadSettings->JunctionDetectionRadius;
    const float DetectionRadiusSq = FMath::Square(DetectionRadius);
    const float NodeMergeDist = RoadSettings->NodeMergeDistance;
    const float NodeMergeDistSq = FMath::Square(NodeMergeDist);

    // Step A: Sample each road's spline
    // Each sample needs to know which road it belongs to (for grid lookup later)
    struct FSplineSample
    {
        FVector Location;   //World position
        float Distance;     //Distance along spline
        int32 EdgeIndex;    //Which road this sample belongs to
    };

    // Collect all samples from all roads into one flat array (Grid Hash indexes into this)
    TArray<FSplineSample> AllSamples;

    for (int32 EdgeIdx = 0; EdgeIdx < EdgeCandidates.Num(); ++EdgeIdx)
    {
        const FRoadEdgeCandidate& Edge = EdgeCandidates[EdgeIdx];
        if (!Edge.InputSpline)
        {
            continue;
        }

        const float SplineLength = Edge.InputSpline->GetSplineLength();

        // Sample from start to end at regular intervals
        for (float Dist = 0.0f; Dist <= SplineLength; Dist += SampleStep)
        {
            FSplineSample Sample;
            Sample.Location = Edge.InputSpline->GetLocationAtDistanceAlongSpline(
                Dist, ESplineCoordinateSpace::World);
            Sample.Distance = Dist;
            Sample.EdgeIndex = EdgeIdx;
            AllSamples.Add(Sample);
        }
    }

    // ============================================================
    // Step B: Use Grid Hash spatial index instead of brute-force
    // Principle: divide world into CellSize×CellSize×CellSize cells,
    // each point only compares with points in same + 26 neighbor cells (3×3×3).
    // Points in distant cells are skipped entirely — no distance calc needed.
    //
    //   ┌─────┬─────┬─────┬─────┐
    //   │     │ ●A  │     │     │  ← A in cell(1,0)
    //   ├─────┼─────┼─────┼─────┤
    //   │     │  ●B │     │     │  ← B (1,1) neighbor → compare
    //   ├─────┼─────┼─────┼─────┤
    //   │     │     │     │ ●C  │  ← C (3,2) → far → skip
    //   └─────┴─────┴─────┴─────┘
    // ============================================================

    // ---- Step B.1：Build the grid ----
    // Cell size = DetectionRadius, so adjacent cells are at most 2×CellSize apart.
    // Checking 3×3×3 = 27 cells guarantees no missed pairs within detection radius.

    const float CellSize = DetectionRadius;

    // Lambda: convert world position → grid cell coordinate
    auto WorldToCell = [CellSize](const FVector& Pos) -> FIntVector
    {
        return FIntVector(
            FMath::FloorToInt(Pos.X / CellSize),
            FMath::FloorToInt(Pos.Y / CellSize),
            FMath::FloorToInt(Pos.Z / CellSize)
        );
    };

    // Grid: Key = cell coordinate, Value = indices into AllSamples for that cell
    TMap<FIntVector, TArray<int32>> Grid;

    // Insert each sample into its corresponding grid cell
    for (int32 SampleIdx = 0; SampleIdx < AllSamples.Num(); ++SampleIdx)
    {
        const FIntVector CellKey = WorldToCell(AllSamples[SampleIdx].Location);
        Grid.FindOrAdd(CellKey).Add(SampleIdx);
    }


    // ---- Step B.2：Iterate cells, compare only neighbor cell points ----
    // For each occupied cell, check itself + 26 neighbors (3×3×3 cube).
    // Only compare pairs from DIFFERENT roads.

    for (const auto& CellPair : Grid)
    {
        const FIntVector& CellKey = CellPair.Key;
        const TArray<int32>& CellSamples = CellPair.Value;

        // Iterate 3×3×3 neighborhood (including self)
        for (int32 DX = -1; DX <= 1; ++DX)
        {
            for (int32 DY = -1; DY <= 1; ++DY)
            {
                for (int32 DZ = -1; DZ <= 1; ++DZ)
                {
                    const FIntVector NeighborKey(
                        CellKey.X + DX,
                        CellKey.Y + DY,
                        CellKey.Z + DZ
                    );

                    // Skip if neighbor cell has no samples
                    const TArray<int32>* NeighborSamples = Grid.Find(NeighborKey);
                    if (!NeighborSamples)
                    {
                        continue;
                    }

                    // Compare each sample in this cell vs each in neighbor cell
                    for (int32 IdxA : CellSamples)
                    {
                        for (int32 IdxB : *NeighborSamples)
                        {
                            const FSplineSample& SampleA = AllSamples[IdxA];
                            const FSplineSample& SampleB = AllSamples[IdxB];

                            // Skip same-road pairs (only want inter-road junctions)
                            if (SampleA.EdgeIndex == SampleB.EdgeIndex)
                            {
                                continue;
                            }

                            // Avoid duplicates: only compare when EdgeA < EdgeB
                            if (SampleA.EdgeIndex > SampleB.EdgeIndex)
                            {
                                continue;
                            }

                            // Distance check (squared to avoid sqrt)
                            const float DistSq = FVector::DistSquared(
                                SampleA.Location, SampleB.Location);

                            if (DistSq > DetectionRadiusSq)
                            {
                                continue;
                            }

                            // Endpoint filter: use distance along spline, NOT world-space distance.
                            // If close to start (dist~0) or end (dist~splineLength) → near endpoint.

                            const FRoadEdgeCandidate& EdgeA = EdgeCandidates[SampleA.EdgeIndex];
                            const FRoadEdgeCandidate& EdgeB = EdgeCandidates[SampleB.EdgeIndex];

                            const float SplineLengthA = EdgeA.InputSpline
                                ? EdgeA.InputSpline->GetSplineLength() : 0.0f;
                            const float SplineLengthB = EdgeB.InputSpline
                                ? EdgeB.InputSpline->GetSplineLength() : 0.0f;

                            // Use DetectionRadius as threshold (not NodeMergeDist).
                            // If a sample is within detection range AND near its own endpoint,
                            // it's an endpoint connection, not a junction.
                            const bool bSampleANearOwnEndpoint =
                                SampleA.Distance <= DetectionRadius
                                || (SplineLengthA - SampleA.Distance) <= DetectionRadius;

                            const bool bSampleBNearOwnEndpoint =
                                SampleB.Distance <= DetectionRadius
                                || (SplineLengthB - SampleB.Distance) <= DetectionRadius;

                            // Both near their own endpoints → endpoint-to-endpoint → skip
                            if (bSampleANearOwnEndpoint && bSampleBNearOwnEndpoint)
                            {
                                continue;
                            }

                            // At least one side is mid-spline → potential real junction

                            const FVector JunctionLoc =
                                (SampleA.Location + SampleB.Location) * 0.5f;

                            // Merge into existing junction or create new one.
                            // This handles crossroads: if 3+ roads meet at one point,
                            // they all get added to the same junction.
                            int32 ExistingJunctionIdx = INDEX_NONE;
                            for (int32 JIdx = 0; JIdx < JunctionCandidates.Num(); ++JIdx)
                            {
                                if (FVector::DistSquared(JunctionCandidates[JIdx].WorldLocation, JunctionLoc)
                                    <= DetectionRadiusSq)
                                {
                                    ExistingJunctionIdx = JIdx;
                                    break;
                                }
                            }

                            if (ExistingJunctionIdx != INDEX_NONE)
                            {
                                // Existing junction → add new edges if not already present
                                FRoadJunctionCandidate& Existing = JunctionCandidates[ExistingJunctionIdx];

                                if (!Existing.ConnectedEdgeIndices.Contains(SampleA.EdgeIndex))
                                {
                                    Existing.ConnectedEdgeIndices.Add(SampleA.EdgeIndex);
                                    Existing.DistancesOnSpline.Add(SampleA.Distance);
                                }
                                if (!Existing.ConnectedEdgeIndices.Contains(SampleB.EdgeIndex))
                                {
                                    Existing.ConnectedEdgeIndices.Add(SampleB.EdgeIndex);
                                    Existing.DistancesOnSpline.Add(SampleB.Distance);
                                }
                            }
                            else
                            {
                                // New junction → add both edges
                                FRoadJunctionCandidate NewJunction;
                                NewJunction.WorldLocation = JunctionLoc;
                                NewJunction.ConnectedEdgeIndices.Add(SampleA.EdgeIndex);
                                NewJunction.DistancesOnSpline.Add(SampleA.Distance);
                                NewJunction.ConnectedEdgeIndices.Add(SampleB.EdgeIndex);
                                NewJunction.DistancesOnSpline.Add(SampleB.Distance);

                                JunctionCandidates.Add(NewJunction);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// <summary>
/// Build node candidates by merging nearby endpoints within NodeMergeDistance.
/// </summary>
void URoadNetworkSubsystem::BuildNodeCandidates()
{
    NodeCandidates.Empty();

    const ARoadWorldSettings* RoadSettings = GetRoadWorldSettings();
    if (!RoadSettings)
    {
        UE_LOG(LogTemp, Warning, TEXT("BuildNodeCandidates: WorldSettings is not ARoadWorldSettings"));
        return;
    }

    const float MergeDistance = RoadSettings->NodeMergeDistance;
    const float MergeDistanceSq = FMath::Square(MergeDistance);

    /*
    Helper lambda that either merges a point into an existing node candidate
    or creates a new node candidate if no nearby node exists.
    function inside function: [] capture list
    */
    auto AddOrMergePoint = [this, MergeDistanceSq](const FVector& Point, bool bFromJunction = false)
        {
            for (FRoadGraphNodeCandidate& ExistingNode : NodeCandidates)
            {
                // Compare squared distance for efficiency.
                const float DistSq = FVector::DistSquared(ExistingNode.WorldLocation, Point);

                if (DistSq <= MergeDistanceSq)
                {
                    // Merge this point into the existing node candidate.
                    // Recompute the representative world location using a running average.
                    const int32 OldCount = ExistingNode.MergedEndpointCount;
                    const int32 NewCount = OldCount + 1;

                    ExistingNode.WorldLocation =
                        ((ExistingNode.WorldLocation * OldCount) + Point) / NewCount;

                    ExistingNode.MergedEndpointCount = NewCount;

                    // If this point came from a junction, mark the node as junction
                    if (bFromJunction)
                    {
                        ExistingNode.bIsJunction = true;
                    }
                    return;
                }
            }

            // No nearby node candidate found, so create a new one.
            FRoadGraphNodeCandidate NewNode;
            NewNode.NodeId = NodeCandidates.Num();
            NewNode.WorldLocation = Point;
            NewNode.MergedEndpointCount = 1;
            NewNode.bIsJunction = bFromJunction;

            NodeCandidates.Add(NewNode);
        };

    // Process start and end point of each edge candidate
    for (int32 i = 0; i < EdgeCandidates.Num(); ++i)
    {
        const FRoadEdgeCandidate& Edge = EdgeCandidates[i];

        AddOrMergePoint(Edge.StartWorldLocation);
        AddOrMergePoint(Edge.EndWorldLocation);
    }

    // Add junction locations as node candidates so A* can route through junctions
    for (int32 JIdx = 0; JIdx < JunctionCandidates.Num(); ++JIdx)
    {
        const FRoadJunctionCandidate& Junction = JunctionCandidates[JIdx];
        AddOrMergePoint(Junction.WorldLocation, /*bFromJunction=*/ true);
    }
}

/// <summary>
/// Search all node candidates and find the nearest one.
/// </summary>
int32 URoadNetworkSubsystem::FindNearestNodeId(const FVector& WorldLocation) const
{
    int32 BestNodeId = INDEX_NONE;
    float BestDistSq = TNumericLimits<float>::Max();

    for (const FRoadGraphNodeCandidate& Node : NodeCandidates)
    {
        const float DistSq = FVector::DistSquared(Node.WorldLocation, WorldLocation);

        if (DistSq < BestDistSq)
        {
            BestDistSq = DistSq;
            BestNodeId = Node.NodeId;
        }
    }

    return BestNodeId;
}

/// <summary>
/// Build finalized graph edges by assigning start/end node ids to each edge candidate.
/// If a junction falls on an edge's spline mid-section, the edge is split into sub-edges.
/// </summary>
void URoadNetworkSubsystem::BuildGraphEdges()
{
    GraphEdges.Empty();

    // For each EdgeCandidate, collect all junction split points on its spline
    for (int32 EdgeIdx = 0; EdgeIdx < EdgeCandidates.Num(); ++EdgeIdx)
    {
        const FRoadEdgeCandidate& EdgeCandidate = EdgeCandidates[EdgeIdx];

        const float SplineLengthCm = EdgeCandidate.InputSpline
            ? EdgeCandidate.InputSpline->GetSplineLength() : 0.0f;

        // ---- Collect split points ----
        // Each split point = (distance along spline, world location)
        struct FSplitPoint
        {
            float DistanceOnSpline;
            FVector WorldLocation;
        };

        TArray<FSplitPoint> SplitPoints;

        //Start point
        FSplitPoint StartSplit;
        StartSplit.DistanceOnSpline = 0.0f;
        StartSplit.WorldLocation = EdgeCandidate.StartWorldLocation;
        SplitPoints.Add(StartSplit);

        // Add all junctions that fall on this edge
        for (const FRoadJunctionCandidate& Junction : JunctionCandidates)
        {
            for (int32 k = 0; k < Junction.ConnectedEdgeIndices.Num(); ++k)
            {
                if (Junction.ConnectedEdgeIndices[k] == EdgeIdx)
                {
                    FSplitPoint JunctionSplit;
                    JunctionSplit.DistanceOnSpline = Junction.DistancesOnSpline[k];
                    JunctionSplit.WorldLocation = Junction.WorldLocation;
                    SplitPoints.Add(JunctionSplit);
                    break; //dont add same junction at same time
                }
            }
        }

        //End point
        FSplitPoint EndSplit;
        EndSplit.DistanceOnSpline = SplineLengthCm;
        EndSplit.WorldLocation = EdgeCandidate.EndWorldLocation;
        SplitPoints.Add(EndSplit);

        //Sort by distance along spline
        SplitPoints.Sort([](const FSplitPoint& A, const FSplitPoint& B)
        {
            return A.DistanceOnSpline < B.DistanceOnSpline;
        });

        // ---- Create sub-edges between adjacent split points ----
        for (int32 i = 0; i < SplitPoints.Num() - 1; ++i)
        {
            const FSplitPoint& From = SplitPoints[i];
            const FSplitPoint& To = SplitPoints[i + 1];

            // Skip zero-length sub-edges (when split points coincide)
            if (FMath::IsNearlyEqual(From.DistanceOnSpline, To.DistanceOnSpline, 1.0f))
            {
                continue;
            }

            const int32 StartNodeId = FindNearestNodeId(From.WorldLocation);
            const int32 EndNodeId = FindNearestNodeId(To.WorldLocation);

            if (StartNodeId == INDEX_NONE || EndNodeId == INDEX_NONE)
            {
                continue;
            }

            // Skip self-loops
            if (StartNodeId == EndNodeId)
            {
                continue;
            }

            FRoadGraphEdge NewEdge;
            NewEdge.EdgeId = GraphEdges.Num();
            NewEdge.StartNodeId = StartNodeId;
            NewEdge.EndNodeId = EndNodeId;
            NewEdge.RoadActor = EdgeCandidate.RoadActor;
            NewEdge.InputSpline = EdgeCandidate.InputSpline;
            NewEdge.StartDistanceOnSpline = From.DistanceOnSpline;
            NewEdge.EndDistanceOnSpline = To.DistanceOnSpline;
            NewEdge.StartWorldLocation = From.WorldLocation;
            NewEdge.EndWorldLocation = To.WorldLocation;
            NewEdge.RoadType = EdgeCandidate.RoadType;
            NewEdge.bTwoRoads = EdgeCandidate.bTwoRoads;
            NewEdge.TwoRoadsGapM = EdgeCandidate.TwoRoadsGapM;
            NewEdge.RoadWidthMultiplier = EdgeCandidate.RoadWidthMultiplier;
            NewEdge.AdditionalWidthM = EdgeCandidate.AdditionalWidthM;
            NewEdge.GuardrailSideOffsetCm = EdgeCandidate.GuardrailSideOffsetCm;
            NewEdge.AutoLaneWidthCm = EdgeCandidate.AutoLaneWidthCm;
            NewEdge.AutoMedianCm = EdgeCandidate.AutoMedianCm;

            // Sub-edge length = spline segment length (cm → meters)
            NewEdge.LengthMeters = (To.DistanceOnSpline - From.DistanceOnSpline) / 100.0;

            GraphEdges.Add(NewEdge);
        }
    }
}

/// <summary>
/// Build finalized graph nodes from node candidates and attach edge connections.
/// </summary>
void URoadNetworkSubsystem::BuildGraphNodes()
{
    GraphNodes.Empty();

    // Create graph nodes from node candidates
    for (const FRoadGraphNodeCandidate& Candidate : NodeCandidates)
    {
        FRoadGraphNode Node;

        Node.NodeId = Candidate.NodeId;
        Node.WorldLocation = Candidate.WorldLocation;
        Node.MergedEndpointCount = Candidate.MergedEndpointCount;
        Node.bIsJunction = Candidate.bIsJunction;

        GraphNodes.Add(Node);
    }

    // Attach edges to corresponding nodes
    for (const FRoadGraphEdge& Edge : GraphEdges)
    {
        if (GraphNodes.IsValidIndex(Edge.StartNodeId))
        {
            GraphNodes[Edge.StartNodeId].ConnectedEdgeIds.Add(Edge.EdgeId);
        }

        if (GraphNodes.IsValidIndex(Edge.EndNodeId))
        {
            GraphNodes[Edge.EndNodeId].ConnectedEdgeIds.Add(Edge.EdgeId);
        }
    }
}

// ============================================================================
//  BindParkingActorsToEdges
//  Scan all parking actors, assign correct Graph EdgeId per spot
// ============================================================================
void URoadNetworkSubsystem::BindParkingActorsToEdges()
{
    UWorld* World = GetWorld();
    if (!World) return;

    auto AssignSpotsForActor = [this](AActor* ParkingActor,
        const FString& TargetLabel,
        TArray<FVector>& SpotPositions,
        TArray<FVector>& SpotForwards,
        TFunctionRef<void(int32 SpotIndex, int32 EdgeId)> Assign)
    {
        if (!ParkingActor) return;

        // ---- Collect all GraphEdges whose RoadActor matches the label ----
        TArray<int32> CandidateEdgeIds;
        CandidateEdgeIds.Reserve(8);
        for (const FRoadGraphEdge& E : GraphEdges)
        {
            if (!E.RoadActor || !E.InputSpline) continue;
#if WITH_EDITOR
            if (E.RoadActor->GetActorLabel().Contains(TargetLabel))
#else
            if (E.RoadActor->GetName().Contains(TargetLabel))
#endif
            {
                CandidateEdgeIds.Add(E.EdgeId);
            }
        }

        if (CandidateEdgeIds.Num() == 0)
        {
            return;
        }


        // ---- For each spot, find best edge from candidates ----
        for (int32 SpotIdx = 0; SpotIdx < SpotPositions.Num(); ++SpotIdx)
        {
            const FVector SpotPos = SpotPositions[SpotIdx];
            const FVector SpotFwd2D = SpotForwards[SpotIdx].GetSafeNormal2D();

            float BestDist = TNumericLimits<float>::Max();
            int32 BestEdgeId = INDEX_NONE;
            float BestDot = -2.0f;

            for (int32 EdgeId : CandidateEdgeIds)
            {
                const FRoadGraphEdge& E = GraphEdges[EdgeId];
                if (!E.InputSpline) continue;

                // Project spot to spline, clamp to [StartDist, EndDist] of this edge
                const float Key = E.InputSpline->FindInputKeyClosestToWorldLocation(SpotPos);
                float ProjDist = E.InputSpline->GetDistanceAlongSplineAtSplineInputKey(Key);
                const float Lo = FMath::Min(E.StartDistanceOnSpline, E.EndDistanceOnSpline);
                const float Hi = FMath::Max(E.StartDistanceOnSpline, E.EndDistanceOnSpline);
                ProjDist = FMath::Clamp(ProjDist, Lo, Hi);

                const FVector ClosestPt = E.InputSpline->GetLocationAtDistanceAlongSpline(
                    ProjDist, ESplineCoordinateSpace::World);
                const FVector SplineDir = E.InputSpline->GetDirectionAtDistanceAlongSpline(
                    ProjDist, ESplineCoordinateSpace::World).GetSafeNormal2D();

                // Edge is undirected (one segment represents both travel directions),
                // so accept |dot| ≥ 0.866. The actual travel direction is decided
                // at NavigateTo* time based on dot sign (StartNode vs EndNode as goal).
                const float Dot = FVector::DotProduct(SpotFwd2D, SplineDir);
                if (FMath::Abs(Dot) < 0.866f) continue;

                const float Dist = FVector::Dist(SpotPos, ClosestPt);
                if (Dist < BestDist)
                {
                    BestDist = Dist;
                    BestEdgeId = EdgeId;
                    BestDot = Dot;
                }
            }

            if (BestEdgeId != INDEX_NONE)
            {
                Assign(SpotIdx, BestEdgeId);
            }
        }
    };

    // ---- [PARK-NAV] For each ParkingLot: use Arrow[0] as the single anchor, pick nearest candidate sub-edge ----
    for (TActorIterator<AParkingLotActor> It(World); It; ++It)
    {
        AParkingLotActor* Lot = *It;
        if (!Lot || Lot->SpotCount <= 0) continue;

        const FString& TargetLabel = Lot->TargetRoadActorLabel;
        const FVector AnchorPos = Lot->GetAnchorArrowPosition();


        // Step 1: collect candidate edges by actor label
        TArray<int32> CandidateEdgeIds;
        CandidateEdgeIds.Reserve(8);
        for (const FRoadGraphEdge& E : GraphEdges)
        {
            if (!E.RoadActor || !E.InputSpline) continue;
#if WITH_EDITOR
            const bool bMatch = E.RoadActor->ActorHasTag(TEXT("DriveRoad"));
#else
            const bool bMatch = E.RoadActor->ActorHasTag(TEXT("DriveRoad"));
#endif
            if (bMatch) CandidateEdgeIds.Add(E.EdgeId);
        }

        if (CandidateEdgeIds.Num() == 0)
        {
            continue;
        }


        // Step 2: project Anchor onto each sub-edge clamped to its [Lo,Hi]; pick smallest dist.
        //         No direction filter — edge is undirected; direction picked at Navigate time.
        int32 BestEdgeId = INDEX_NONE;
        float BestDist = TNumericLimits<float>::Max();
        FVector BestProjPt = FVector::ZeroVector;

        for (int32 EdgeId : CandidateEdgeIds)
        {
            const FRoadGraphEdge& E = GraphEdges[EdgeId];
            if (!E.InputSpline) continue;

            const float Key = E.InputSpline->FindInputKeyClosestToWorldLocation(AnchorPos);
            float ProjDist = E.InputSpline->GetDistanceAlongSplineAtSplineInputKey(Key);
            const float Lo = FMath::Min(E.StartDistanceOnSpline, E.EndDistanceOnSpline);
            const float Hi = FMath::Max(E.StartDistanceOnSpline, E.EndDistanceOnSpline);
            ProjDist = FMath::Clamp(ProjDist, Lo, Hi);

            const FVector ClosestPt = E.InputSpline->GetLocationAtDistanceAlongSpline(
                ProjDist, ESplineCoordinateSpace::World);
            const float Dist = FVector::Dist(AnchorPos, ClosestPt);


            if (Dist < BestDist)
            {
                BestDist = Dist;
                BestEdgeId = EdgeId;
                BestProjPt = ClosestPt;
            }
        }

        if (BestEdgeId == INDEX_NONE)
        {
            continue;
        }

        Lot->AssignBoundEdge(BestEdgeId, BestProjPt);

    }

}


/// <summary>
/// Returns all neighbors of the specified node by traversing its connected edges.
/// </summary>
TArray<FRoadGraphNeighbor> URoadNetworkSubsystem::GetNeighborNodes(int32 NodeId) const
{
    TArray<FRoadGraphNeighbor> Result;

    if (!GraphNodes.IsValidIndex(NodeId))
    {
        return Result;
    }

    const FRoadGraphNode& Node = GraphNodes[NodeId];

    for (int32 EdgeId : Node.ConnectedEdgeIds)
    {
        if (!GraphEdges.IsValidIndex(EdgeId))
        {
            continue;
        }

        const FRoadGraphEdge& Edge = GraphEdges[EdgeId];

        int32 NeighborNodeId = INDEX_NONE;

        // If this node is the start of the edge, then the end node is the neighbor.
        if (Edge.StartNodeId == NodeId)
        {
            NeighborNodeId = Edge.EndNodeId;
        }
        // If this node is the end of the edge, then the start node is the neighbor.
        else if (Edge.EndNodeId == NodeId)
        {
            NeighborNodeId = Edge.StartNodeId;
        }

        if (NeighborNodeId == INDEX_NONE)
        {
            continue;
        }

        FRoadGraphNeighbor Neighbor;
        Neighbor.NeighborNodeId = NeighborNodeId;
        Neighbor.EdgeId = EdgeId;
        Neighbor.Cost = static_cast<float>(Edge.LengthMeters);

        Result.Add(Neighbor);
    }

    return Result;
}

/// <summary>
/// Find the nearest finalized graph node to the specified world location.
/// </summary>
int32 URoadNetworkSubsystem::FindNearestGraphNode(const FVector& WorldLocation) const
{
    int32 BestNodeId = INDEX_NONE;
    float BestDistSq = TNumericLimits<float>::Max();

    for (const FRoadGraphNode& Node : GraphNodes)
    {
        const float DistSq = FVector::DistSquared(Node.WorldLocation, WorldLocation);

        if (DistSq < BestDistSq)
        {
            BestDistSq = DistSq;
            BestNodeId = Node.NodeId;
        }
    }

    return BestNodeId;
}

/// <summary>
/// Heuristic function used by A*.
/// First version uses straight-line world-space distance.
/// </summary>
float URoadNetworkSubsystem::EstimateHeuristicCost(int32 FromNodeId, int32 ToNodeId) const
{
    if (!GraphNodes.IsValidIndex(FromNodeId) || !GraphNodes.IsValidIndex(ToNodeId))
    {
        return 0.0f;
    }

    const FVector& FromLocation = GraphNodes[FromNodeId].WorldLocation;
    const FVector& ToLocation = GraphNodes[ToNodeId].WorldLocation;

    return FVector::Distance(FromLocation, ToLocation);
}

/// <summary>
/// Run A* pathfinding on the road graph from a start node to a goal node.
/// Returns FRoadGraphPath containing the node sequence and total cost.
/// </summary>
FRoadGraphPath URoadNetworkSubsystem::FindPathAStar(int32 StartNodeId, int32 GoalNodeId) const
{
    FRoadGraphPath Result;

    // Validate start and goal node ids.
    if (!GraphNodes.IsValidIndex(StartNodeId) || !GraphNodes.IsValidIndex(GoalNodeId))
    {
        return Result;
    }

    // If start equals goal, return a trivial one-node path.
    if (StartNodeId == GoalNodeId)
    {
        Result.bPathFound = true;
        Result.NodePath.Add(StartNodeId);
        Result.TotalCost = 0.0f;
        return Result;
    }

    /*
    OpenSet stores nodes to be explored.
    First version uses a simple TArray and manually selects the lowest FScore.
    */
    TArray<int32> OpenSet;
    OpenSet.Add(StartNodeId);

    // Record which previous node each node came from.
    TMap<int32, int32> CameFrom;

    // GScore = best known cost from start node to this node.
    TMap<int32, float> GScore;
    GScore.Add(StartNodeId, 0.0f);

    // FScore = GScore + heuristic estimate.
    TMap<int32, float> FScore;
    FScore.Add(StartNodeId, EstimateHeuristicCost(StartNodeId, GoalNodeId));

    // Main A* search loop.
    while (OpenSet.Num() > 0)
    {
        // Select the node with the lowest FScore in the OpenSet as the current node.
        int32 CurrentNodeId = OpenSet[0];
        float CurrentBestFScore = FScore.Contains(CurrentNodeId)
            ? FScore[CurrentNodeId]
            : TNumericLimits<float>::Max();

        for (int32 CandidateNodeId : OpenSet)
        {
            const float CandidateFScore = FScore.Contains(CandidateNodeId)
                ? FScore[CandidateNodeId]
                : TNumericLimits<float>::Max();

            if (CandidateFScore < CurrentBestFScore)
            {
                CurrentBestFScore = CandidateFScore;
                CurrentNodeId = CandidateNodeId;
            }
        }

        // If the current node is the goal, reconstruct the path.
        if (CurrentNodeId == GoalNodeId)
        {
            TArray<int32> ReversedPath;
            ReversedPath.Add(CurrentNodeId);

            int32 WalkNodeId = CurrentNodeId;

            while (CameFrom.Contains(WalkNodeId))
            {
                WalkNodeId = CameFrom[WalkNodeId];
                ReversedPath.Add(WalkNodeId);
            }

            // Reverse because the path was reconstructed backwards.
            Algo::Reverse(ReversedPath);

            Result.bPathFound = true;
            Result.NodePath = ReversedPath;
            Result.TotalCost = GScore.Contains(GoalNodeId) ? GScore[GoalNodeId] : 0.0f;
            return Result;
        }

        // Remove the current node from the OpenSet.
        OpenSet.Remove(CurrentNodeId);

        // Get all neighbors of the current node.
        const TArray<FRoadGraphNeighbor> Neighbors = GetNeighborNodes(CurrentNodeId);

        for (const FRoadGraphNeighbor& Neighbor : Neighbors)
        {
            // TentativeG = candidate cost of reaching the neighbor through the current node.
            const float CurrentG = GScore.Contains(CurrentNodeId)
                ? GScore[CurrentNodeId]
                : TNumericLimits<float>::Max();

            const float TentativeG = CurrentG + Neighbor.Cost;

            const bool bNeighborHasG = GScore.Contains(Neighbor.NeighborNodeId);
            const float ExistingNeighborG = bNeighborHasG
                ? GScore[Neighbor.NeighborNodeId]
                : TNumericLimits<float>::Max();

            // If this new path is better, update the neighbor's records.
            if (TentativeG < ExistingNeighborG)
            {
                CameFrom.Add(Neighbor.NeighborNodeId, CurrentNodeId);
                GScore.Add(Neighbor.NeighborNodeId, TentativeG);

                const float Heuristic =
                    EstimateHeuristicCost(Neighbor.NeighborNodeId, GoalNodeId);

                FScore.Add(Neighbor.NeighborNodeId, TentativeG + Heuristic);

                // Add the neighbor to OpenSet if it is not already there.
                if (!OpenSet.Contains(Neighbor.NeighborNodeId))
                {
                    OpenSet.Add(Neighbor.NeighborNodeId);
                }
            }
        }
    }

    // If OpenSet becomes empty, no path was found.
    return Result;
}

/// <summary>
/// Draw debug visualization of the road graph: nodes, edges, and junction candidates.
/// </summary>
void URoadNetworkSubsystem::DrawDebugGraph()
{
}

// ============================================================================
//  Look up a graph edge by its EdgeId
// ============================================================================
const FRoadGraphEdge* URoadNetworkSubsystem::GetGraphEdgeById(int32 EdgeId) const
{
    for (const FRoadGraphEdge& Edge : GraphEdges)
    {
        if (Edge.EdgeId == EdgeId)
        {
            return &Edge;
        }
    }
    return nullptr;
}

// ============================================================================
//  IsUTurnAllowedAtNode — check if a node is a valid U-turn point
// ============================================================================
bool URoadNetworkSubsystem::IsUTurnAllowedAtNode(int32 NodeId) const
{
    if (!GraphNodes.IsValidIndex(NodeId))
    {
        return false;
    }

    const FRoadGraphNode& Node = GraphNodes[NodeId];

    // Junction nodes always allow U-turns
    if (Node.bIsJunction)
    {
        return true;
    }

    // Dead-end nodes (single spline endpoint = true road end) allow U-turns
    if (Node.MergedEndpointCount == 1)
    {
        return true;
    }

    // Merged endpoint nodes (2+ spline endpoints meeting = road middle) do NOT allow U-turns
    return false;
}

// ============================================================================
//  FindNearestUTurnNode — BFS forward to find nearest valid U-turn node
// ============================================================================
int32 URoadNetworkSubsystem::FindNearestUTurnNode(int32 StartNodeId, int32 ExcludeNodeId) const
{
    if (!GraphNodes.IsValidIndex(StartNodeId))
    {
        return INDEX_NONE;
    }

    // Check if StartNodeId itself allows U-turn
    if (IsUTurnAllowedAtNode(StartNodeId))
    {
        return StartNodeId;
    }

    // BFS to find nearest valid U-turn node
    TArray<int32> Queue;
    TSet<int32> Visited;

    Queue.Add(StartNodeId);
    Visited.Add(StartNodeId);

    int32 Head = 0;
    while (Head < Queue.Num())
    {
        const int32 CurrentId = Queue[Head++];
        const TArray<FRoadGraphNeighbor> Neighbors = GetNeighborNodes(CurrentId);

        for (const FRoadGraphNeighbor& Nb : Neighbors)
        {
            // On the first hop from StartNodeId, skip ExcludeNodeId (don't go backward)
            if (CurrentId == StartNodeId && Nb.NeighborNodeId == ExcludeNodeId)
            {
                continue;
            }

            if (Visited.Contains(Nb.NeighborNodeId))
            {
                continue;
            }

            Visited.Add(Nb.NeighborNodeId);

            if (IsUTurnAllowedAtNode(Nb.NeighborNodeId))
            {
                return Nb.NeighborNodeId;
            }

            Queue.Add(Nb.NeighborNodeId);
        }
    }

    // No valid U-turn node found — fallback: allow at StartNodeId anyway
    UE_LOG(LogTemp, Warning,
        TEXT("[U-TURN] No valid U-turn node found via BFS from Node %d — fallback to StartNodeId"),
        StartNodeId);
    return StartNodeId;
}
