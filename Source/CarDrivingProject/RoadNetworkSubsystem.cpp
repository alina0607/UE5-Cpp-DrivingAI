// Fill out your copyright notice in the Description page of Project Settings.


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

/// <summary>
/// Initialization happens when the world creates this subsystem.
/// </summary>
void URoadNetworkSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
    UE_LOG(LogTemp, Warning, TEXT("RoadNetworkSubsystem: Initialize"));

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
    UE_LOG(LogTemp, Warning, TEXT("RoadNetworkSubsystem: Deinitialize"));
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
    UE_LOG(LogTemp, Warning, TEXT("RoadNetworkSubsystem: World BeginPlay"));

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

    UE_LOG(LogTemp, Warning, TEXT("BuildRoadCache: Found %d road actors"), FoundRoadActors.Num());

    // Extract runtime road data from each Blueprint road actor.
    for (AActor* RoadActor : FoundRoadActors)
    {
        if (!RoadActor)
        {
            continue;
        }

        if (!RoadActor->GetActorLabel().Contains(TEXT("BP_DriveRoad")))
        {
            UE_LOG(LogTemp, Warning, TEXT("BuildRoadCache: Skipping non-road actor %s | Class=%s"),
                *RoadActor->GetActorLabel(),
                *RoadActor->GetClass()->GetName());
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

        EdgeCandidates.Add(Candidate);
    }

    // 印出每條邊的詳細資訊（含 TwoRoads 狀態）以便驗證
    // Log each edge candidate's details including TwoRoads status
    for (int32 i = 0; i < EdgeCandidates.Num(); ++i)
    {
        const FRoadEdgeCandidate& EC = EdgeCandidates[i];
        UE_LOG(LogTemp, Warning,
            TEXT("  Edge[%d] %s | RoadType=%d | TwoRoads=%s | Gap=%.1fm | Length=%.1fm"),
            i,
            EC.RoadActor ? *EC.RoadActor->GetActorLabel() : TEXT("NULL"),
            EC.RoadType,
            EC.bTwoRoads ? TEXT("true") : TEXT("false"),
            EC.TwoRoadsGapM,
            EC.LengthMeters);
    }

    UE_LOG(LogTemp, Warning, TEXT("BuildEdgeCandidates: Built %d edge candidates"), EdgeCandidates.Num());
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

    UE_LOG(LogTemp, Log,
        TEXT("DetectMidSplineJunctions: %d samples in %d grid cells (CellSize=%.0f)"),
        AllSamples.Num(), Grid.Num(), CellSize);

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
    auto AddOrMergePoint = [this, MergeDistanceSq](const FVector& Point)
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
                    return;
                }
            }

            // No nearby node candidate found, so create a new one.
            FRoadGraphNodeCandidate NewNode;
            NewNode.NodeId = NodeCandidates.Num();
            NewNode.WorldLocation = Point;
            NewNode.MergedEndpointCount = 1;

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
        AddOrMergePoint(Junction.WorldLocation);
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

            // Sub-edge length = spline segment length (cm → meters)
            NewEdge.LengthMeters = (To.DistanceOnSpline - From.DistanceOnSpline) / 100.0;

            GraphEdges.Add(NewEdge);
        }
    }

    UE_LOG(LogTemp, Warning,
        TEXT("BuildGraphEdges: Built %d graph edges (from %d edge candidates, %d junctions)"),
        GraphEdges.Num(), EdgeCandidates.Num(), JunctionCandidates.Num());
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

    UE_LOG(LogTemp, Warning,
        TEXT("BuildGraphNodes: Built %d nodes"),
        GraphNodes.Num());
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
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Warning, TEXT("DrawDebugGraph: World is null"));
        return;
    }

    const ARoadWorldSettings* RoadSettings = GetRoadWorldSettings();
    if (!RoadSettings)
    {
        UE_LOG(LogTemp, Warning, TEXT("DrawDebugGraph: WorldSettings is not ARoadWorldSettings"));
        return;
    }

    const float Duration = RoadSettings->DebugDrawDuration;
    const float NodeRadius = RoadSettings->DebugNodeRadius;
    const float EdgeThickness = RoadSettings->DebugEdgeThickness;

    const bool bHasFinalNodes = GraphNodes.Num() > 0;
    const bool bHasFinalEdges = GraphEdges.Num() > 0;

    /*
    Draw edges first.
    Prefer finalized GraphEdges if available;
    otherwise fall back to EdgeCandidates.
    */
    if (RoadSettings->bDrawDebugEdges)
    {
        if (bHasFinalEdges)
        {
            for (const FRoadGraphEdge& Edge : GraphEdges)
            {
                if (Edge.InputSpline)
                {
                    /*Sample along the spline and approximate the curved edge
                    using multiple line segments.
                    */
                    const float StartDist = Edge.StartDistanceOnSpline;
                    const float EndDist = Edge.EndDistanceOnSpline;

                    const float SegmentStep = 300.0f;
                    const int32 NumSegments = FMath::Max(
                        1,
                        FMath::CeilToInt((EndDist - StartDist) / SegmentStep)
                    );

                    FVector PrevPoint =
                        Edge.InputSpline->GetLocationAtDistanceAlongSpline(
                            StartDist,
                            ESplineCoordinateSpace::World
                        );

                    for (int32 SegmentIndex = 1; SegmentIndex <= NumSegments; ++SegmentIndex)
                    {
                        const float Alpha = static_cast<float>(SegmentIndex) / static_cast<float>(NumSegments);
                        const float CurrentDist = FMath::Lerp(StartDist, EndDist, Alpha);

                        const FVector CurrentPoint =
                            Edge.InputSpline->GetLocationAtDistanceAlongSpline(
                                CurrentDist,
                                ESplineCoordinateSpace::World
                            );

                        DrawDebugLine(
                            World,
                            PrevPoint,
                            CurrentPoint,
                            FColor::Cyan,
                            false,
                            Duration,
                            0,
                            EdgeThickness
                        );

                        PrevPoint = CurrentPoint;
                    }
                }
                else
                {
                    DrawDebugLine(
                        World,
                        Edge.StartWorldLocation,
                        Edge.EndWorldLocation,
                        FColor::Cyan,
                        false,
                        Duration,
                        0,
                        EdgeThickness
                    );
                }

                // Optionally draw edge id and node connection info at the midpoint.
                if (RoadSettings->bDrawDebugEdgeLabels)
                {
                    const FVector MidPoint =
                        (Edge.StartWorldLocation + Edge.EndWorldLocation) * 0.5f;

                    DrawDebugString(
                        World,
                        MidPoint + FVector(0.0f, 0.0f, 150.0f),
                        FString::Printf(
                            TEXT("E%d  %d->%d"),
                            Edge.EdgeId,
                            Edge.StartNodeId,
                            Edge.EndNodeId),
                        nullptr,
                        FColor::Cyan,
                        Duration,
                        false
                    );
                }
            }
        }
        else
        {
            for (const FRoadEdgeCandidate& Edge : EdgeCandidates)
            {
                DrawDebugLine(
                    World,
                    Edge.StartWorldLocation,
                    Edge.EndWorldLocation,
                    FColor::Blue,
                    false,
                    Duration,
                    0,
                    EdgeThickness
                );
            }
        }
    }

    /*
    Draw nodes next.
    Prefer finalized GraphNodes if available;
    otherwise fall back to NodeCandidates.
    */
    if (RoadSettings->bDrawDebugNodes)
    {
        if (bHasFinalNodes)
        {
            for (const FRoadGraphNode& Node : GraphNodes)
            {
                DrawDebugSphere(
                    World,
                    Node.WorldLocation,
                    NodeRadius,
                    12,
                    FColor::Green,
                    false,
                    Duration,
                    0,
                    10.0f
                );

                // Optionally draw node id and connected edge count.
                if (RoadSettings->bDrawDebugNodeLabels)
                {
                    DrawDebugString(
                        World,
                        Node.WorldLocation + FVector(0.0f, 0.0f, 250.0f),
                        FString::Printf(
                            TEXT("N%d  E:%d"),
                            Node.NodeId,
                            Node.ConnectedEdgeIds.Num()),
                        nullptr,
                        FColor::White,
                        Duration,
                        false
                    );
                }
            }
        }
        else
        {
            for (const FRoadGraphNodeCandidate& Node : NodeCandidates)
            {
                DrawDebugSphere(
                    World,
                    Node.WorldLocation,
                    NodeRadius,
                    12,
                    FColor::Yellow,
                    false,
                    Duration,
                    0,
                    10.0f
                );

                // Optionally draw candidate node id and merged endpoint count.
                if (RoadSettings->bDrawDebugNodeLabels)
                {
                    DrawDebugString(
                        World,
                        Node.WorldLocation + FVector(0.0f, 0.0f, 250.0f),
                        FString::Printf(
                            TEXT("NC%d  M:%d"),
                            Node.NodeId,
                            Node.MergedEndpointCount),
                        nullptr,
                        FColor::Yellow,
                        Duration,
                        false
                    );
                }
            }
        }
    }

    // ---- Draw junction candidates (red, larger than nodes) ----
    for (int32 i = 0; i < JunctionCandidates.Num(); ++i)
    {
        const FRoadJunctionCandidate& J = JunctionCandidates[i];

        DrawDebugSphere(
            World,
            J.WorldLocation,
            NodeRadius * 1.5f,   // Slightly larger than normal nodes
            12,
            FColor::Red,         // Red = junction
            false,
            Duration,
            0,
            12.0f
        );

        // Print junction index and connected edges above the sphere
        FString JEdgesStr;
        for (int32 k = 0; k < J.ConnectedEdgeIndices.Num(); ++k)
        {
            if (k > 0) { JEdgesStr += TEXT(","); }
            JEdgesStr += FString::Printf(TEXT("E%d"), J.ConnectedEdgeIndices[k]);
        }

        DrawDebugString(
            World,
            J.WorldLocation + FVector(0.0f, 0.0f, 350.0f),
            FString::Printf(TEXT("J%d [%s]"), i, *JEdgesStr),
            nullptr,
            FColor::Red,
            Duration,
            false
        );
    }

    UE_LOG(
        LogTemp,
        Warning,
        TEXT("DrawDebugGraph: Mode=%s | Nodes=%d | Edges=%d | Junctions=%d"),
        (bHasFinalNodes || bHasFinalEdges) ? TEXT("FinalGraph") : TEXT("Candidates"),
        bHasFinalNodes ? GraphNodes.Num() : NodeCandidates.Num(),
        bHasFinalEdges ? GraphEdges.Num() : EdgeCandidates.Num(),
        JunctionCandidates.Num()
    );

    // ---- 寫死測試：用 A* 找路徑並畫黃色粗線 ----
    // Hardcoded test: run A* between two nodes and draw the path in yellow
    if (bHasFinalEdges && bHasFinalNodes)
    {
        const int32 TestStartNode = 9;
        const int32 TestGoalNode = 2;

        const FRoadGraphPath TestPath = FindPathAStar(TestStartNode, TestGoalNode);

        UE_LOG(LogTemp, Warning,
            TEXT("Test AStar | Start=%d | Goal=%d | Found=%s | Cost=%.2f | Path=%s"),
            TestStartNode, TestGoalNode,
            TestPath.bPathFound ? TEXT("true") : TEXT("false"),
            TestPath.TotalCost,
            *FString::JoinBy(TestPath.NodePath, TEXT(" "),
                [](int32 N) { return FString::Printf(TEXT("%d"), N); }));

        if (TestPath.bPathFound)
        {
            // 對路徑中每對相鄰 node，找到連接的 edge 並沿 spline 畫黃色粗線
            // For each adjacent node pair in the path, find the connecting edge
            // and draw it as a thick yellow line along the spline
            for (int32 i = 0; i < TestPath.NodePath.Num() - 1; ++i)
            {
                const int32 FromNode = TestPath.NodePath[i];
                const int32 ToNode = TestPath.NodePath[i + 1];

                for (const FRoadGraphEdge& Edge : GraphEdges)
                {
                    const bool bMatch =
                        (Edge.StartNodeId == FromNode && Edge.EndNodeId == ToNode)
                        || (Edge.StartNodeId == ToNode && Edge.EndNodeId == FromNode);

                    if (!bMatch)
                    {
                        continue;
                    }

                    if (!Edge.InputSpline)
                    {
                        break;
                    }

                    // 沿 spline 取樣畫線（抬高 80cm 避免跟路面重疊）
                    // Sample along spline and draw (raised 80cm to avoid z-fighting)
                    const float FromDist = Edge.StartDistanceOnSpline;
                    const float ToDist = Edge.EndDistanceOnSpline;
                    const int32 NumSteps = FMath::Max(1,
                        FMath::CeilToInt(FMath::Abs(ToDist - FromDist) / 300.0f));

                    FVector PrevPt = Edge.InputSpline->GetLocationAtDistanceAlongSpline(
                        FromDist, ESplineCoordinateSpace::World);

                    for (int32 S = 1; S <= NumSteps; ++S)
                    {
                        const float D = FMath::Lerp(FromDist, ToDist,
                            static_cast<float>(S) / static_cast<float>(NumSteps));

                        const FVector Pt = Edge.InputSpline->GetLocationAtDistanceAlongSpline(
                            D, ESplineCoordinateSpace::World);

                        DrawDebugLine(
                            World,
                            PrevPt + FVector(0.0f, 0.0f, 80.0f),
                            Pt + FVector(0.0f, 0.0f, 80.0f),
                            FColor::Yellow,
                            false,
                            Duration,
                            0,
                            EdgeThickness * 2.5f
                        );

                        PrevPt = Pt;
                    }

                    break;  // 找到了就不用繼續搜尋
                }
            }
        }
    }
}
