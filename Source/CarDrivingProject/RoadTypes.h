// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "RoadTypes.generated.h"

class AActor;
class USplineComponent;

/// <summary>
/// FRoadRuntimeData stores the runtime information of one road segment
/// extracted from Blueprint road actors.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadRuntimeData
{
	GENERATED_BODY()

	/// <summary>
	/// Original road Blueprint actor placed in the level
	/// </summary>
	UPROPERTY(BlueprintReadOnly)
    TObjectPtr<AActor> RoadActor = nullptr;

	/// <summary>
	/// Spline component used by this road
	/// </summary>
	UPROPERTY(BlueprintReadOnly)
	TObjectPtr<USplineComponent> InputSpline = nullptr;

    /// <summary>
    /// Start position of the road in world space
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector StartWorldLocation = FVector::ZeroVector;

    /// <summary>
    /// End position of the road in world space
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector EndWorldLocation = FVector::ZeroVector;

    /// <summary>
    /// Road length in meters
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double LengthMeters = 0.0;

    /// <summary>
    /// Whether this road uses spline mode
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bUseSpline = false;

    /// <summary>
    /// Whether this road forms a closed loop
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bClosedLoop = false;

    /// <summary>
    /// Road type value read from the Blueprint
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    uint8 RoadType = 0;
};

/// <summary>
/// One candidate graph edge generated from one road runtime entry.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadEdgeCandidate
{
    GENERATED_BODY()

    /// <summary>
    /// Original road actor that produced this edge candidate.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TObjectPtr<AActor> RoadActor = nullptr;

    /// <summary>
    /// Spline component used by this road.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TObjectPtr<class USplineComponent> InputSpline = nullptr;

    /// <summary>
    /// Start position of this edge candidate in world space.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector StartWorldLocation = FVector::ZeroVector;

    /// <summary>
    /// End position of this edge candidate in world space.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector EndWorldLocation = FVector::ZeroVector;

    /// <summary>
    /// Road length in meters.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double LengthMeters = 0.0;

    /// <summary>
    /// Road type copied from FRoadRuntimeData.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    uint8 RoadType = 0;

    /// <summary>
    /// Whether this road is a closed loop.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bClosedLoop = false;
};

/// <summary>
/// FRoadGraphNodeCandidate represents one candidate graph node
/// built from road endpoints before final graph connectivity is assigned.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadGraphNodeCandidate
{
    GENERATED_BODY()

    /// <summary>
    /// Temporary node id assigned during node candidate generation.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 NodeId = INDEX_NONE;

    /// <summary>
    /// Representative world position of this node candidate.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector WorldLocation = FVector::ZeroVector;

    /// <summary>
    /// Number of endpoints merged into this node candidate.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 MergedEndpointCount = 0;
};

/// <summary>
/// FRoadGraphEdge represents one finalized edge in the road graph.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadGraphEdge
{
    GENERATED_BODY()

    /// <summary>
    /// Unique id of this graph edge.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 EdgeId = INDEX_NONE;

    /// <summary>
    /// Start node id of this edge.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 StartNodeId = INDEX_NONE;

    /// <summary>
    /// End node id of this edge.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 EndNodeId = INDEX_NONE;

    /// <summary>
    /// Original road actor associated with this edge.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TObjectPtr<AActor> RoadActor = nullptr;

    /// <summary>
    /// Spline component used by this edge.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TObjectPtr<class USplineComponent> InputSpline = nullptr;

    /// <summary>
    /// Start distance along the spline in centimeters.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float StartDistanceOnSpline = 0.0f;

    /// <summary>
    /// End distance along the spline in centimeters.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float EndDistanceOnSpline = 0.0f;

    /// <summary>
    /// World-space start position of this edge.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector StartWorldLocation = FVector::ZeroVector;

    /// <summary>
    /// World-space end position of this edge.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector EndWorldLocation = FVector::ZeroVector;

    /// <summary>
    /// Length of this edge in meters.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double LengthMeters = 0.0;

    /// <summary>
    /// Road type associated with this edge.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    uint8 RoadType = 0;
};

/// <summary>
/// Final node inside the road graph
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadGraphNode
{
    GENERATED_BODY()

    /// <summary>
    /// Node identifier
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 NodeId = INDEX_NONE;

    /// <summary>
    /// World location of node
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector WorldLocation = FVector::ZeroVector;

    /// <summary>
    /// All edges connected to this node
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TArray<int32> ConnectedEdgeIds;
};

/// <summary>
/// FRoadGraphNeighbor represents one reachable neighboring node.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadGraphNeighbor
{
    GENERATED_BODY()

    /// <summary>
    /// Neighbor node id.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 NeighborNodeId = INDEX_NONE;

    /// <summary>
    /// Edge id used to reach this neighbor.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 EdgeId = INDEX_NONE;

    /// <summary>
    /// Traversal cost of this edge. First version uses road length.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float Cost = 0.0f;
};

/// <summary>
/// FRoadGraphPath represents a node path returned by A* search.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadGraphPath
{
    GENERATED_BODY()

    /// <summary>
    /// Whether a path was successfully found.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bPathFound = false;

    /// <summary>
    /// Ordered node id sequence of the path.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TArray<int32> NodePath;

    /// <summary>
    /// Total path cost.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float TotalCost = 0.0f;
};

/// <summary>
/// 叉路候選：多條路在同一位置交會
/// Junction candidate: multiple roads meeting at one location.
/// Supports T-junctions (2 roads), crossroads (3-4 roads), etc.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadJunctionCandidate
{
    GENERATED_BODY()

    /// <summary>
    /// 交會點的世界座標（所有交會取樣點的平均值）
    /// World position of the junction (average of all contributing sample points)
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector WorldLocation = FVector::ZeroVector;

    /// <summary>
    /// 在此路口交會的所有邊候選索引（EdgeCandidates 陣列的 index）
    /// Indices of all edge candidates that meet at this junction
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TArray<int32> ConnectedEdgeIndices;

    /// <summary>
    /// 每條邊在交會點的 spline 距離（cm），與 ConnectedEdgeIndices 一一對應
    /// Distance along each edge's spline where the junction occurs (cm).
    /// Parallel array with ConnectedEdgeIndices.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TArray<float> DistancesOnSpline;
};
