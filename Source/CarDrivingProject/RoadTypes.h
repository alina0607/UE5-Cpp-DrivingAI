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

    /// <summary>
    /// Whether this road uses the "Two Roads" mode (two parallel road meshes).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bTwoRoads = false;

    /// <summary>
    /// Gap between the two parallel roads in meters (only relevant when bTwoRoads is true).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double TwoRoadsGapM = 0.0;

    /// <summary>
    /// Road width multiplier from the Blueprint asset.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double RoadWidthMultiplier = 1.0;

    /// <summary>
    /// Additional road width in meters from the Blueprint asset.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double AdditionalWidthM = 0.0;

    /// <summary>
    /// GuardrailSideOffset from Road Settings struct (cm).
    /// = road half-width (distance from spline center to road edge).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double GuardrailSideOffsetCm = 350.0;

    /// <summary>
    /// Auto-calculated lane width from CatsEyesPositions (cm).
    /// Average of gaps between consecutive positive marker positions.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float AutoLaneWidthCm = 0.0f;

    /// <summary>
    /// Auto-calculated median offset from CatsEyesPositions (cm).
    /// Position of the first non-negative marker on the right side.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float AutoMedianCm = 0.0f;
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

    /// <summary>
    /// Whether this road uses "Two Roads" mode.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bTwoRoads = false;

    /// <summary>
    /// Gap between the two parallel roads in meters.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double TwoRoadsGapM = 0.0;

    /// <summary>
    /// Road width multiplier from BP.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double RoadWidthMultiplier = 1.0;

    /// <summary>
    /// Additional road width in meters from BP.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double AdditionalWidthM = 0.0;

    /// <summary>
    /// Road half-width (cm) from BP Road Settings.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double GuardrailSideOffsetCm = 350.0;

    /// <summary>
    /// Auto-calculated lane width from CatsEyesPositions (cm).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float AutoLaneWidthCm = 0.0f;

    /// <summary>
    /// Auto-calculated median offset from CatsEyesPositions (cm).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float AutoMedianCm = 0.0f;
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

    /// <summary>
    /// Whether this edge uses "Two Roads" mode (two parallel road meshes).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bTwoRoads = false;

    /// <summary>
    /// Gap between the two parallel roads in meters (only when bTwoRoads=true).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double TwoRoadsGapM = 0.0;

    /// <summary>
    /// Road width multiplier from BP.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double RoadWidthMultiplier = 1.0;

    /// <summary>
    /// Additional road width in meters from BP.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double AdditionalWidthM = 0.0;

    /// <summary>
    /// Road half-width (cm) from BP Road Settings.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    double GuardrailSideOffsetCm = 350.0;

    /// <summary>
    /// Auto-calculated lane width from CatsEyesPositions (cm).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float AutoLaneWidthCm = 0.0f;

    /// <summary>
    /// Auto-calculated median offset from CatsEyesPositions (cm).
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    float AutoMedianCm = 0.0f;
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
/// Junction candidate: multiple roads meeting at one location.
/// Supports T-junctions (2 roads), crossroads (3-4 roads), etc.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadJunctionCandidate
{
    GENERATED_BODY()

    /// <summary>
    /// World position of the junction (average of all contributing sample points)
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FVector WorldLocation = FVector::ZeroVector;

    /// <summary>
    /// Indices of all edge candidates that meet at this junction
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TArray<int32> ConnectedEdgeIndices;

    /// <summary>
    /// Distance along each edge's spline where the junction occurs (cm).
    /// Parallel array with ConnectedEdgeIndices.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    TArray<float> DistancesOnSpline;
};
