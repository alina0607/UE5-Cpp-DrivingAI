// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "RoadTypes.h"
#include "RoadNetworkSubsystem.generated.h"

class ARoadWorldSettings;

/// <summary>
/// World subsystem that manages the road graph:
/// scanning Blueprint road actors, building graph nodes/edges, and running A* pathfinding.
/// </summary>
UCLASS()
class CARDRIVINGPROJECT_API URoadNetworkSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:

	/// <summary>
	/// Called when this subsystem is created for a world.
	/// </summary>
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;

	/// <summary>
	/// Called when this subsystem is being destroyed.
	/// </summary>
	virtual void Deinitialize() override;

	/// <summary>
	/// Scan the level and rebuild the road cache.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	void BuildRoadCache();

	/// <summary>
	/// Returns all neighbors of the specified node
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	TArray<FRoadGraphNeighbor> GetNeighborNodes(int32 NodeId) const;

	/// <summary>
	/// Find the nearest graph node id to the specified world location.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	int32 FindNearestGraphNode(const FVector& WorldLocation) const;

	/// <summary>
	/// Run A* on the graph from a start node to a goal node.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	FRoadGraphPath FindPathAStar(int32 StartNodeId, int32 GoalNodeId) const;

	/// <summary>
	/// Draw the current graph debug visualization in the world.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Debug")
	void DrawDebugGraph();

	// ---- Getters ----

	/// <summary>
	/// Returns all cached roads.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	const TArray<FRoadRuntimeData>& GetAllRoads() const
	{
		return AllRoads;
	}

	/// <summary>
	/// Returns all edge candidates.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	const TArray<FRoadEdgeCandidate>& GetEdgeCandidates() const
	{
		return EdgeCandidates;
	}

	/// <summary>
	/// Returns all node candidates.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	const TArray<FRoadGraphNodeCandidate>& GetNodeCandidates() const
	{
		return NodeCandidates;
	}

	/// <summary>
	/// Returns all finalized graph edges.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	const TArray<FRoadGraphEdge>& GetGraphEdges() const
	{
		return GraphEdges;
	}

	/// <summary>
	/// Returns all finalized graph nodes.
	/// </summary>
	const TArray<FRoadGraphNode>& GetGraphNodes() const
	{
		return GraphNodes;
	}

private:

	/// <summary>
	/// Helper function to get this world's road settings object.
	/// </summary>
	const ARoadWorldSettings* GetRoadWorldSettings() const;

	/// <summary>
	/// Stores all extracted road runtime data for the whole level.
	/// </summary>
	UPROPERTY()
	TArray<FRoadRuntimeData> AllRoads;

	/// <summary>
	/// Candidate graph edges built from cached road runtime data.
	/// </summary>
	UPROPERTY()
	TArray<FRoadEdgeCandidate> EdgeCandidates;

	/// <summary>
	/// Candidate graph nodes built from edge candidate endpoints.
	/// </summary>
	UPROPERTY()
	TArray<FRoadGraphNodeCandidate> NodeCandidates;

	/// <summary>
	/// Finalized graph edge data.
	/// </summary>
	UPROPERTY()
	TArray<FRoadGraphEdge> GraphEdges;

	/// <summary>
	/// Final graph nodes
	/// </summary>
	UPROPERTY()
	TArray<FRoadGraphNode> GraphNodes;

	/// <summary>
	/// 偵測到的中段叉路候選清單
	/// Detected mid-spline junction candidates
	/// </summary>
	UPROPERTY()
	TArray<FRoadJunctionCandidate> JunctionCandidates;

	void HandleWorldBeginPlay();

	/// <summary>
	/// 偵測 spline 中段的交會點，必須在 BuildEdgeCandidates() 之後呼叫
	/// Detect where road splines come close at non-endpoint locations.
	/// Must be called after BuildEdgeCandidates().
	/// </summary>
	void DetectMidSplineJunctions();

	/// <summary>
	/// Build edge candidates from cached road runtime data.
	/// </summary>
	void BuildEdgeCandidates();

	/// <summary>
	/// Build node candidates from edge candidate start and end points.
	/// </summary>
	void BuildNodeCandidates();

	/// <summary>
	/// Find the nearest node candidate id to a given world position.
	/// </summary>
	int32 FindNearestNodeId(const FVector& WorldLocation) const;

	/// <summary>
	/// Build finalized graph edges from edge candidates.
	/// </summary>
	void BuildGraphEdges();

	/// <summary>
	/// Build finalized graph nodes from node candidates and attach edges.
	/// </summary>
	void BuildGraphNodes();

	/// <summary>
	/// Heuristic function used by A*, first version uses straight-line distance.
	/// </summary>
	float EstimateHeuristicCost(int32 FromNodeId, int32 ToNodeId) const;

};
