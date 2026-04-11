#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RoadsideParkingActor.generated.h"

class UArrowComponent;

/// <summary>
/// 路邊停車 Actor — 在關卡中放置，用 UArrowComponent 子元件定義停車格
/// Roadside Parking Actor — place in level, use UArrowComponent children to define parking spots.
///
/// 每個 Arrow = 一個停車格（位置 = 停車位置，前方 = 車頭朝向）
/// Each Arrow = one parking spot (position = spot location, forward = car heading).
///
/// 使用方式 / Usage:
///   1. 放置到場景中 / Place in level
///   2. 在 BP 的 Components 面板新增子元件 UArrowComponent，命名 "Spot_0", "Spot_1" ...
///      Add child UArrowComponent in BP Components panel
///   3. 在場景中拖拉 Arrow 調整停車格位置和朝向
///      Drag Arrow in viewport to adjust spot position and forward direction
///   4. Arrow 的位置和方向 = 停車格位置和車頭朝向，系統自動偵測左右側
///      Arrow position and direction = spot location and car heading, system auto-detects road side
/// </summary>
UCLASS(Blueprintable, BlueprintType)
class CARDRIVINGPROJECT_API ARoadsideParkingActor : public AActor
{
	GENERATED_BODY()

public:
	ARoadsideParkingActor();

	// ================================================================
	//  設定 / Configuration
	// ================================================================

	/// 自訂名稱（地圖上顯示）/ Custom name displayed on map
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Roadside Parking")
	FString ZoneName = TEXT("Roadside Parking");

	/// 停車範圍在道路的哪一側（true=左側 false=右側，BeginPlay 自動偵測）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Roadside Parking")
	bool bIsLeftSide = false;

	/// 此路邊停車關聯的 BP_DriveRoad Actor 名稱（編輯器設定）
	/// 例如填 "BP_DriveRoad_5"，BindParkingActorsToEdges 時用 GetActorLabel().Contains() 配對
	/// Target road actor label (set in editor), e.g. "BP_DriveRoad_5".
	/// Matched via GetActorLabel().Contains() during BindParkingActorsToEdges.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Roadside Parking")
	FString TargetRoadActorLabel = TEXT("BP_DriveRoad");

	/// 最近 Graph Edge ID（BeginPlay 自動偵測）
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Roadside Parking|Debug")
	int32 NearestEdgeId = INDEX_NONE;

	/// 有多少個停車格（BeginPlay 後有效）
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Roadside Parking|Debug")
	int32 SpotCount = 0;

	// ================================================================
	//  API — 幾何 / Geometry
	// ================================================================

	/// 取得停車格世界位置 / Get spot world position
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	FVector GetSpotWorldPosition(int32 SpotIndex) const;

	/// 取得停車格世界前方方向 / Get spot world forward direction
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	FVector GetSpotWorldForward(int32 SpotIndex) const;

	/// 總共幾個停車格 / Total spot count
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	int32 GetSpotCount() const { return SpotArrows.Num(); }

	// ================================================================
	//  API — 佔用 / Occupation (每個 Arrow = 一格 / each Arrow = one spot)
	// ================================================================

	/// 找第一個未被佔用的停車格（回傳 index，-1 = 全滿）
	/// Find first unoccupied spot (returns index, -1 = all full)
	UFUNCTION(BlueprintCallable, Category = "Roadside Parking")
	int32 FindAvailableSpot() const;

	/// 佔用指定停車格 / Occupy a spot
	UFUNCTION(BlueprintCallable, Category = "Roadside Parking")
	bool OccupySpot(int32 SpotIndex, AActor* Vehicle);

	/// 釋放指定停車格 / Release a spot
	UFUNCTION(BlueprintCallable, Category = "Roadside Parking")
	void ReleaseSpot(int32 SpotIndex);

	/// 停車格是否被佔用 / Is spot occupied?
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	bool IsSpotOccupied(int32 SpotIndex) const;

	/// 取得指定停車格對應的 Graph EdgeId（由 BindParkingActorsToEdges 設定）
	/// Get the Graph EdgeId for a spot (set by BindParkingActorsToEdges)
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	int32 GetSpotEdgeId(int32 SpotIndex) const;

	/// 由 RoadNetworkSubsystem 呼叫，指派 Spot 對應的 EdgeId
	/// Called by RoadNetworkSubsystem to assign EdgeId for a spot
	void AssignSpotEdgeId(int32 SpotIndex, int32 EdgeId);

protected:
	virtual void BeginPlay() override;

private:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USceneComponent> SceneRoot;

	/// Arrow 元件列表（BeginPlay 時從子元件收集）
	/// Spot arrow components (collected from children at BeginPlay)
	UPROPERTY()
	TArray<TObjectPtr<UArrowComponent>> SpotArrows;

	/// 佔用狀態（平行於 SpotArrows）/ Occupation state (parallel to SpotArrows)
	TArray<bool> SpotOccupied;

	/// 佔用車輛 / Occupying vehicles
	UPROPERTY()
	TArray<TWeakObjectPtr<AActor>> SpotVehicles;

	/// 每個停車格對應的 Graph EdgeId（由 BindParkingActorsToEdges 設定）
	/// Per-spot Graph EdgeId (assigned by BindParkingActorsToEdges)
	TArray<int32> SpotEdgeIds;

	void CollectSpotArrows();
	void DetectRoadSide();
};
