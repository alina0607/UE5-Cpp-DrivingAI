#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ParkingLotActor.generated.h"

class UArrowComponent;
class UBoxComponent;

/// <summary>
/// Parking Lot Actor — place in level, define multiple parking spots.
///
///   1. Place in level
///      Add child UArrowComponent in BP Components panel, name them "Spot_0", "Spot_1" ...
///      Drag Arrow in viewport to adjust spot position and forward direction
///      System auto-collects all Arrow children as parking spots at BeginPlay
/// </summary>
UCLASS(Blueprintable, BlueprintType)
class CARDRIVINGPROJECT_API AParkingLotActor : public AActor
{
	GENERATED_BODY()

public:
	AParkingLotActor();

	///Building mesh (set mesh in BP)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mesh")
	TObjectPtr<UStaticMeshComponent> BuildingMesh;

	///Parking floor mesh
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mesh")
	TObjectPtr<UStaticMeshComponent> ParkingMesh;

	// ================================================================
	// Configuration
	// ================================================================

	///Custom name displayed on map
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parking Lot")
	FString ParkingLotName = TEXT("Parking Lot");



	/// Target road actor label (set in editor), e.g. "BP_DriveRoad_5".
	/// Matched via GetActorLabel().Contains() during BindParkingActorsToEdges.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parking Lot")
	FString TargetRoadActorLabel = TEXT("BP_DriveRoad");

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	int32 NearestEdgeId = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	float DistanceOnNearestEdge = 0.0f;

	/// How many spots (valid after BeginPlay)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	int32 SpotCount = 0;

	// ================================================================
	//  API
	// ================================================================

	/// Get spot world position
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	FVector GetSpotWorldPosition(int32 SpotIndex) const;

	/// Get spot world forward direction
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	FVector GetSpotWorldForward(int32 SpotIndex) const;

	/// Find first available spot (-1 = all full)
	UFUNCTION(BlueprintCallable, Category = "Parking Lot")
	int32 FindAvailableSpot() const;

	/// Occupy spot
	UFUNCTION(BlueprintCallable, Category = "Parking Lot")
	bool OccupySpot(int32 SpotIndex, AActor* Vehicle);

	/// Release spot
	UFUNCTION(BlueprintCallable, Category = "Parking Lot")
	void ReleaseSpot(int32 SpotIndex);

	/// Is spot occupied?
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	bool IsSpotOccupied(int32 SpotIndex) const;

	/// Get the actor currently occupying a spot (nullptr if empty / released)
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	AActor* GetSpotOccupant(int32 SpotIndex) const;

	/// Get the Graph EdgeId for a spot (set by BindParkingActorsToEdges)
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	int32 GetSpotEdgeId(int32 SpotIndex) const;

	/// Called by RoadNetworkSubsystem to assign EdgeId for a spot
	void AssignSpotEdgeId(int32 SpotIndex, int32 EdgeId);

	// ================================================================
	//  Simplified: use first Arrow as the single navigation anchor
	// ================================================================

	/// Bound edge for the whole lot (derived from Arrow[0])
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	int32 BoundEdgeId = INDEX_NONE;

	/// Projection of Arrow[0] onto BoundEdge
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	FVector BoundProjectionPoint = FVector::ZeroVector;

	/// Arrow[0] world position
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	FVector GetAnchorArrowPosition() const { return GetSpotWorldPosition(0); }

	/// Arrow[0] world forward
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	FVector GetAnchorArrowForward() const { return GetSpotWorldForward(0); }

	/// Called by RoadNetworkSubsystem: set bound edge + projection point for the whole lot
	void AssignBoundEdge(int32 EdgeId, const FVector& ProjectionPoint);

	bool IsLeftSide() const { return bIsLeftSide; }

protected:
	virtual void BeginPlay() override;


private:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USceneComponent> SceneRoot;

	/// Spot arrow components (collected from children at BeginPlay)
	UPROPERTY()
	TArray<TObjectPtr<UArrowComponent>> SpotArrows;

	/// Occupation state (parallel to SpotArrows)
	TArray<bool> SpotOccupied;

	///Occupying vehicles
	UPROPERTY()
	TArray<TWeakObjectPtr<AActor>> SpotVehicles;

	/// Per-spot Graph EdgeId (assigned by BindParkingActorsToEdges)
	TArray<int32> SpotEdgeIds;

	bool bIsLeftSide = false;


	void CollectSpotArrows();
	void DetectRoadSide();
};
