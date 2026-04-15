#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ParkingLotActor.generated.h"

class UArrowComponent;
class UBoxComponent;

/// <summary>
/// 停車場 Actor — 在關卡中放置，定義多個停車格
/// Parking Lot Actor — place in level, define multiple parking spots.
///
/// 使用方式 / Usage:
///   1. 放置到場景中 / Place in level
///   2. 在 BP 的 Components 面板新增子元件 UArrowComponent，命名 "Spot_0", "Spot_1" ...
///      Add child UArrowComponent in BP Components panel, name them "Spot_0", "Spot_1" ...
///   3. 在場景中拖拉 Arrow 調整停車格位置和朝向
///      Drag Arrow in viewport to adjust spot position and forward direction
///   4. 系統會在 BeginPlay 自動收集所有 Arrow 子元件作為停車格
///      System auto-collects all Arrow children as parking spots at BeginPlay
/// </summary>
UCLASS(Blueprintable, BlueprintType)
class CARDRIVINGPROJECT_API AParkingLotActor : public AActor
{
	GENERATED_BODY()

public:
	AParkingLotActor();

	// ================================================================
	//  Mesh外觀
	// ================================================================

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



	/// 此停車場關聯的 BP_DriveRoad Actor 名稱（編輯器設定）
	/// 例如填 "BP_DriveRoad_5"，BindParkingActorsToEdges 時用 GetActorLabel().Contains() 配對
	/// Target road actor label (set in editor), e.g. "BP_DriveRoad_5".
	/// Matched via GetActorLabel().Contains() during BindParkingActorsToEdges.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parking Lot")
	FString TargetRoadActorLabel = TEXT("BP_DriveRoad");

	/// 關聯的最近 Graph Edge ID（BeginPlay 自動偵測）
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	int32 NearestEdgeId = INDEX_NONE;

	/// 停車場在最近 Edge Spline 上的距離
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	float DistanceOnNearestEdge = 0.0f;

	/// 有多少個停車格（BeginPlay 後有效）/ How many spots (valid after BeginPlay)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	int32 SpotCount = 0;

	// ================================================================
	//  API
	// ================================================================

	/// 取得停車格世界位置 / Get spot world position
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	FVector GetSpotWorldPosition(int32 SpotIndex) const;

	/// 取得停車格世界前方方向 / Get spot world forward direction
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	FVector GetSpotWorldForward(int32 SpotIndex) const;

	/// 找第一個空閒停車格 / Find first available spot (-1 = all full)
	UFUNCTION(BlueprintCallable, Category = "Parking Lot")
	int32 FindAvailableSpot() const;

	/// 佔用停車格 / Occupy spot
	UFUNCTION(BlueprintCallable, Category = "Parking Lot")
	bool OccupySpot(int32 SpotIndex, AActor* Vehicle);

	/// 釋放停車格 / Release spot
	UFUNCTION(BlueprintCallable, Category = "Parking Lot")
	void ReleaseSpot(int32 SpotIndex);

	/// 停車格是否被佔用 / Is spot occupied?
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	bool IsSpotOccupied(int32 SpotIndex) const;

	/// 取得指定停車格對應的 Graph EdgeId（由 BindParkingActorsToEdges 設定）
	/// Get the Graph EdgeId for a spot (set by BindParkingActorsToEdges)
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	int32 GetSpotEdgeId(int32 SpotIndex) const;

	/// 由 RoadNetworkSubsystem 呼叫，指派 Spot 對應的 EdgeId
	/// Called by RoadNetworkSubsystem to assign EdgeId for a spot
	void AssignSpotEdgeId(int32 SpotIndex, int32 EdgeId);

	// ================================================================
	//  [PARK-NAV] 簡化版：整個停車場用第一個 Arrow 當導航錨點
	//  Simplified: use first Arrow as the single navigation anchor
	// ================================================================

	/// 整個停車場綁定的 Edge（由 Arrow[0] 決定，在 BindParkingActorsToEdges 時設定）
	/// Bound edge for the whole lot (derived from Arrow[0])
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	int32 BoundEdgeId = INDEX_NONE;

	/// Arrow[0] 投影到綁定 Edge 上的世界位置 / Projection of Arrow[0] onto BoundEdge
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Parking Lot|Debug")
	FVector BoundProjectionPoint = FVector::ZeroVector;

	/// Arrow[0] 世界座標 / Arrow[0] world position
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	FVector GetAnchorArrowPosition() const { return GetSpotWorldPosition(0); }

	/// Arrow[0] 世界前方 / Arrow[0] world forward
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Parking Lot")
	FVector GetAnchorArrowForward() const { return GetSpotWorldForward(0); }

	/// 由 RoadNetworkSubsystem 呼叫：一次設定整個停車場的綁定 Edge + 投影點
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
