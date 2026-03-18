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
    UE_LOG(LogTemp, Warning, TEXT("BuildRoadCache: Cached %d roads"), AllRoads.Num());

    for (const FRoadRuntimeData& RoadData : AllRoads)
    {
        FRoadDrivingRule Rule = URoadRuleLibrary::GetDrivingRuleFromRoadData(RoadData);

        UE_LOG(LogTemp, Warning, TEXT("Road %s | RoadType=%d | Rule=%s | Implemented=%s | AllowOvertaking=%s"),
            RoadData.RoadActor ? *RoadData.RoadActor->GetActorLabel() : TEXT("NULL"),
            (int32)RoadData.RoadType,
            *Rule.RuleName,
            Rule.bImplemented ? TEXT("true") : TEXT("false"),
            Rule.bAllowOvertaking ? TEXT("true") : TEXT("false"));
    }

    BuildEdgeCandidates();
    DetectMidSplineJunctions();
    BuildNodeCandidates();
    BuildGraphEdges();
    BuildGraphNodes();
    DrawDebugGraph();

    for (const FRoadGraphNode& Node : GraphNodes)
    {
        TArray<FRoadGraphNeighbor> Neighbors = GetNeighborNodes(Node.NodeId);

        FString NeighborText;
        for (const FRoadGraphNeighbor& Neighbor : Neighbors)
        {
            NeighborText += FString::Printf(TEXT("%d "), Neighbor.NeighborNodeId);
        }

        UE_LOG(
            LogTemp,
            Warning,
            TEXT("Node %d | NeighborCount=%d | Neighbors=%s"),
            Node.NodeId,
            Neighbors.Num(),
            *NeighborText
        );
    }

    if (GraphNodes.Num() > 0)
    {
        /*
        中文：用第一個 graph node 自己的位置測試最近節點查詢。
        English: Test nearest node lookup using the first graph node's own location.
        */
        const FVector TestLocation = GraphNodes[0].WorldLocation;
        const int32 NearestNodeId = FindNearestGraphNode(TestLocation);

        UE_LOG(
            LogTemp,
            Warning,
            TEXT("Test FindNearestGraphNode | Input=(%.2f, %.2f, %.2f) | Result=%d"),
            TestLocation.X,
            TestLocation.Y,
            TestLocation.Z,
            NearestNodeId
        );
    }

    if (GraphNodes.Num() >= 2)
    {
        const int32 StartNodeId = 0;
        const int32 GoalNodeId = 3;

        const FRoadGraphPath TestPath = FindPathAStar(StartNodeId, GoalNodeId);

        FString PathText;
        for (int32 NodeId : TestPath.NodePath)
        {
            PathText += FString::Printf(TEXT("%d "), NodeId);
        }

        UE_LOG(
            LogTemp,
            Warning,
            TEXT("Test AStar | Start=%d | Goal=%d | Found=%s | Cost=%.2f | Path=%s"),
            StartNodeId,
            GoalNodeId,
            TestPath.bPathFound ? TEXT("true") : TEXT("false"),
            TestPath.TotalCost,
            *PathText
        );
    }
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

        EdgeCandidates.Add(Candidate);
    }

    UE_LOG(LogTemp, Warning, TEXT("BuildEdgeCandidates: Built %d edge candidates"), EdgeCandidates.Num());
}

/// <summary>
/// Detect where road splines come close at non-endpoint locations.
/// Uses Grid Hash spatial index for O(N×K) performance instead of brute-force O(R²×S²).
/// Must be called after BuildEdgeCandidates().
///
/// 偵測 spline 中段的交會點。使用 Grid Hash 空間索引加速，
/// 時間複雜度從暴力的 O(R²×S²) 降到 O(N×K)。
/// 必須在 BuildEdgeCandidates() 之後呼叫。
/// </summary>
void URoadNetworkSubsystem::DetectMidSplineJunctions()
{
    JunctionCandidates.Empty();

    // ---- 取得設定 / Get settings ----
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

    // ============================================================
    // Step A：對每條路的 spline 取樣
    // Step A: Sample each road's spline
    // ============================================================

    // 每個取樣點除了位置和距離，還需要知道它屬於哪條路
    // Each sample needs to know which road it belongs to (for grid lookup later)
    struct FSplineSample
    {
        FVector Location;   // 世界座標 / World position
        float Distance;     // 在 spline 上的距離(cm) / Distance along spline
        int32 EdgeIndex;    // 屬於第幾條路 / Which road this sample belongs to
    };

    // 收集所有路的所有取樣點到一個扁平陣列（Grid Hash 需要用索引引用點）
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

        // 從起點到終點，每隔 SampleStep 取一個點
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
    // Step B：用 Grid Hash 空間索引取代暴力比較
    // Step B: Use Grid Hash spatial index instead of brute-force
    //
    // 原理：把世界切成 CellSize×CellSize×CellSize 的格子，
    // 每個點只跟同格和 26 個鄰格（3×3×3）裡的點比較。
    // 遠處格子的點直接跳過，不需要計算距離。
    //
    // Principle: divide world into CellSize×CellSize×CellSize cells,
    // each point only compares with points in same + 26 neighbor cells (3×3×3).
    // Points in distant cells are skipped entirely — no distance calc needed.
    //
    //   ┌─────┬─────┬─────┬─────┐
    //   │     │ ●A  │     │     │  ← A 在格子(1,0) / A in cell(1,0)
    //   ├─────┼─────┼─────┼─────┤
    //   │     │  ●B │     │     │  ← B 在鄰格(1,1) → 比較 ✓ / neighbor → compare
    //   ├─────┼─────┼─────┼─────┤
    //   │     │     │     │ ●C  │  ← C 在遠格(3,2) → 跳過 ✗ / far → skip
    //   └─────┴─────┴─────┴─────┘
    // ============================================================

    // ---- Step B.1：建立格子 / Build the grid ----
    //
    // 格子大小 = DetectionRadius，這樣鄰格內的點距離最多 2×CellSize，
    // 檢查 3×3×3 = 27 格就保證不漏掉任何在偵測半徑內的配對。
    //
    // Cell size = DetectionRadius, so adjacent cells are at most 2×CellSize apart.
    // Checking 3×3×3 = 27 cells guarantees no missed pairs within detection radius.

    const float CellSize = DetectionRadius;

    // 把世界座標轉成格子座標的 helper lambda
    // Lambda: convert world position → grid cell coordinate
    auto WorldToCell = [CellSize](const FVector& Pos) -> FIntVector
    {
        return FIntVector(
            FMath::FloorToInt(Pos.X / CellSize),
            FMath::FloorToInt(Pos.Y / CellSize),
            FMath::FloorToInt(Pos.Z / CellSize)
        );
    };

    // Grid：Key = 格子座標，Value = 該格子裡所有取樣點在 AllSamples 中的索引
    // Grid: Key = cell coordinate, Value = indices into AllSamples for that cell
    TMap<FIntVector, TArray<int32>> Grid;

    // 把每個取樣點丟進對應的格子
    // Insert each sample into its corresponding grid cell
    for (int32 SampleIdx = 0; SampleIdx < AllSamples.Num(); ++SampleIdx)
    {
        const FIntVector CellKey = WorldToCell(AllSamples[SampleIdx].Location);
        Grid.FindOrAdd(CellKey).Add(SampleIdx);
    }

    UE_LOG(LogTemp, Log,
        TEXT("DetectMidSplineJunctions: %d samples in %d grid cells (CellSize=%.0f)"),
        AllSamples.Num(), Grid.Num(), CellSize);

    // ---- Step B.2：遍歷每個格子，只比較鄰格的點 ----
    // ---- Iterate cells, compare only neighbor cell points ----
    //
    // 對每個有點的格子，檢查自己和 26 個鄰格（3×3×3 立方體），
    // 只比較來自「不同路」的點配對。
    //
    // For each occupied cell, check itself + 26 neighbors (3×3×3 cube).
    // Only compare pairs from DIFFERENT roads.

    for (const auto& CellPair : Grid)
    {
        const FIntVector& CellKey = CellPair.Key;
        const TArray<int32>& CellSamples = CellPair.Value;

        // 遍歷 3×3×3 鄰域（包含自己）
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

                    // 鄰格不存在就跳過（那裡沒有任何取樣點）
                    // Skip if neighbor cell has no samples
                    const TArray<int32>* NeighborSamples = Grid.Find(NeighborKey);
                    if (!NeighborSamples)
                    {
                        continue;
                    }

                    // 比較本格的每個點 vs 鄰格的每個點
                    // Compare each sample in this cell vs each in neighbor cell
                    for (int32 IdxA : CellSamples)
                    {
                        for (int32 IdxB : *NeighborSamples)
                        {
                            const FSplineSample& SampleA = AllSamples[IdxA];
                            const FSplineSample& SampleB = AllSamples[IdxB];

                            // 跳過同一條路的點（我們只找不同路的交會）
                            // Skip same-road pairs (we only want inter-road junctions)
                            if (SampleA.EdgeIndex == SampleB.EdgeIndex)
                            {
                                continue;
                            }

                            // 避免重複比較：只比較 EdgeA < EdgeB 的配對
                            // Avoid duplicates: only compare when EdgeA < EdgeB
                            if (SampleA.EdgeIndex > SampleB.EdgeIndex)
                            {
                                continue;
                            }

                            // 距離檢查（用距離平方避免開根號）
                            // Distance check (squared to avoid sqrt)
                            const float DistSq = FVector::DistSquared(
                                SampleA.Location, SampleB.Location);

                            if (DistSq > DetectionRadiusSq)
                            {
                                continue;
                            }

                            // 過濾：太靠近端點的不算（已被 BuildNodeCandidates 的 merge 處理）
                            // Filter: skip if too close to spline endpoints (handled by node merge)
                            const FRoadEdgeCandidate& EdgeA = EdgeCandidates[SampleA.EdgeIndex];
                            const FRoadEdgeCandidate& EdgeB = EdgeCandidates[SampleB.EdgeIndex];

                            // 取兩點的平均位置作為叉路位置
                            // Average of the two sample positions = junction location
                            const FVector JunctionLoc =
                                (SampleA.Location + SampleB.Location) * 0.5f;

                            const bool bNearEndpointA =
                                FVector::DistSquared(JunctionLoc, EdgeA.StartWorldLocation) <= NodeMergeDistSq
                                || FVector::DistSquared(JunctionLoc, EdgeA.EndWorldLocation) <= NodeMergeDistSq;

                            const bool bNearEndpointB =
                                FVector::DistSquared(JunctionLoc, EdgeB.StartWorldLocation) <= NodeMergeDistSq
                                || FVector::DistSquared(JunctionLoc, EdgeB.EndWorldLocation) <= NodeMergeDistSq;

                            if (bNearEndpointA || bNearEndpointB)
                            {
                                continue;
                            }

                            // 合併：跟已有的候選太近就不重複加入
                            // Merge: skip if too close to an existing junction candidate
                            bool bAlreadyExists = false;
                            for (const FRoadJunctionCandidate& Existing : JunctionCandidates)
                            {
                                if (FVector::DistSquared(Existing.WorldLocation, JunctionLoc)
                                    <= DetectionRadiusSq)
                                {
                                    bAlreadyExists = true;
                                    break;
                                }
                            }

                            if (bAlreadyExists)
                            {
                                continue;
                            }

                            // 新增叉路候選 / Add new junction candidate
                            FRoadJunctionCandidate NewJunction;
                            NewJunction.WorldLocation = JunctionLoc;
                            NewJunction.EdgeCandidateIndexA = SampleA.EdgeIndex;
                            NewJunction.DistanceOnSplineA = SampleA.Distance;
                            NewJunction.EdgeCandidateIndexB = SampleB.EdgeIndex;
                            NewJunction.DistanceOnSplineB = SampleB.Distance;

                            JunctionCandidates.Add(NewJunction);
                        }
                    }
                }
            }
        }
    }

    UE_LOG(LogTemp, Warning,
        TEXT("DetectMidSplineJunctions: Found %d junction candidates"),
        JunctionCandidates.Num());

    // 印出每個叉路的詳細資訊方便除錯
    // Print details of each junction for debugging
    for (int32 i = 0; i < JunctionCandidates.Num(); ++i)
    {
        const FRoadJunctionCandidate& J = JunctionCandidates[i];
        UE_LOG(LogTemp, Warning,
            TEXT("  Junction[%d] Loc=(%.0f,%.0f,%.0f) | EdgeA=%d dist=%.0f | EdgeB=%d dist=%.0f"),
            i,
            J.WorldLocation.X, J.WorldLocation.Y, J.WorldLocation.Z,
            J.EdgeCandidateIndexA, J.DistanceOnSplineA,
            J.EdgeCandidateIndexB, J.DistanceOnSplineB);
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

    // For the first version, only use the start and end point of each edge candidate.
    for (const FRoadEdgeCandidate& Edge : EdgeCandidates)
    {
        AddOrMergePoint(Edge.StartWorldLocation);
        AddOrMergePoint(Edge.EndWorldLocation);
    }

    UE_LOG(LogTemp, Warning, TEXT("BuildNodeCandidates: Built %d node candidates"), NodeCandidates.Num());
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
/// </summary>
void URoadNetworkSubsystem::BuildGraphEdges()
{
    GraphEdges.Empty();

    // For each edge candidate, find its start and end node.
    for (const FRoadEdgeCandidate& EdgeCandidate : EdgeCandidates)
    {
        const int32 StartNodeId = FindNearestNodeId(EdgeCandidate.StartWorldLocation);
        const int32 EndNodeId = FindNearestNodeId(EdgeCandidate.EndWorldLocation);

        // Skip this edge if either side cannot find a valid node.
        if (StartNodeId == INDEX_NONE || EndNodeId == INDEX_NONE)
        {
            continue;
        }

        FRoadGraphEdge NewEdge;
        NewEdge.EdgeId = GraphEdges.Num();
        NewEdge.StartNodeId = StartNodeId;
        NewEdge.EndNodeId = EndNodeId;
        NewEdge.RoadActor = EdgeCandidate.RoadActor;
        NewEdge.StartWorldLocation = EdgeCandidate.StartWorldLocation;
        NewEdge.EndWorldLocation = EdgeCandidate.EndWorldLocation;
        NewEdge.LengthMeters = EdgeCandidate.LengthMeters;
        NewEdge.RoadType = EdgeCandidate.RoadType;

        NewEdge.InputSpline = EdgeCandidate.InputSpline;
        float SplineLengthCm = 0.0f;
        if (EdgeCandidate.InputSpline)
        {
            SplineLengthCm = EdgeCandidate.InputSpline->GetSplineLength();
        }

        NewEdge.StartDistanceOnSpline = 0.0f;
        NewEdge.EndDistanceOnSpline = SplineLengthCm;

        GraphEdges.Add(NewEdge);
    }

    UE_LOG(LogTemp, Warning, TEXT("BuildGraphEdges: Built %d graph edges"), GraphEdges.Num());
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
                    /*
                    中文：沿 spline 取樣，用多段直線近似曲線 edge。
                    English: Sample along the spline and approximate the curved edge
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

    // 畫叉路候選（紅色球體）/ Draw junction candidates (red spheres)
    for (int32 i = 0; i < JunctionCandidates.Num(); ++i)
    {
        const FRoadJunctionCandidate& J = JunctionCandidates[i];

        DrawDebugSphere(
            World,
            J.WorldLocation,
            NodeRadius * 1.5f,   // 比普通 node 大一點，好辨認 / Slightly larger than normal nodes
            12,
            FColor::Red,         // 紅色 = 叉路 / Red = junction
            false,
            Duration,
            0,
            12.0f
        );

        // 在球體上方印出叉路編號
        // Print junction index above the sphere
        DrawDebugString(
            World,
            J.WorldLocation + FVector(0.0f, 0.0f, 350.0f),
            FString::Printf(TEXT("J%d  A:%d B:%d"), i,
                J.EdgeCandidateIndexA, J.EdgeCandidateIndexB),
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
}
