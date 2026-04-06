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
	//  Mesh / 外觀
	// ================================================================

	/// 建物 Mesh（在 BP 裡設定 Static Mesh）/ Building mesh (set mesh in BP)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mesh")
	TObjectPtr<UStaticMeshComponent> BuildingMesh;

	/// 停車場地板 Mesh / Parking floor mesh
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mesh")
	TObjectPtr<UStaticMeshComponent> ParkingMesh;

	// ================================================================
	//  設定 / Configuration
	// ================================================================

	/// 停車場自訂名稱（地圖上顯示）/ Custom name displayed on map
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parking Lot")
	FString ParkingLotName = TEXT("Parking Lot");

	/// 停車格大小（半徑，cm）— 用於視覺化 Box / Spot half-extent (cm) for visualization box
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parking Lot")
	FVector SpotHalfExtent = FVector(250.0f, 125.0f, 10.0f);

	/// 停車場在道路的哪一側（true=左側 false=右側）
	/// Which side of the road (true=left, false=right).
	/// Auto-detected from position relative to nearest road, or override manually.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parking Lot")
	bool bIsLeftSide = false;

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

protected:
	virtual void BeginPlay() override;

#if WITH_EDITOR
	virtual void OnConstruction(const FTransform& Transform) override;
#endif

private:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USceneComponent> SceneRoot;

	/// 停車格 Arrow 元件列表（BeginPlay 時從子元件收集）
	/// Spot arrow components (collected from children at BeginPlay)
	UPROPERTY()
	TArray<TObjectPtr<UArrowComponent>> SpotArrows;

	/// 佔用狀態 / Occupation state (parallel to SpotArrows)
	TArray<bool> SpotOccupied;

	/// 佔用車輛 / Occupying vehicles
	UPROPERTY()
	TArray<TWeakObjectPtr<AActor>> SpotVehicles;

	void CollectSpotArrows();
	void DetectRoadSide();
};
