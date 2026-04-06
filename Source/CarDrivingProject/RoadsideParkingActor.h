#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RoadsideParkingActor.generated.h"

class UBoxComponent;

/// <summary>
/// 路邊停車 Actor — 在關卡中放置，用 UBoxComponent 子元件定義可停車範圍
/// Roadside Parking Actor — place in level, use UBoxComponent children to define parking zones.
///
/// 使用方式 / Usage:
///   1. 放置到場景中 / Place in level
///   2. 在 BP 的 Components 面板新增子元件 UBoxComponent，命名 "Zone_0", "Zone_1" ...
///      Add child UBoxComponent in BP Components panel
///   3. 在場景中拖拉、縮放 Box 來定義路邊停車範圍
///      Drag & scale Box in viewport to define roadside parking zones
///   4. Box 的位置和大小 = 可停車的範圍，系統自動偵測左右側
///      Box position and size = parking zone, system auto-detects road side
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

	/// 最近 Graph Edge ID（BeginPlay 自動偵測）
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Roadside Parking|Debug")
	int32 NearestEdgeId = INDEX_NONE;

	/// 有多少個停車區（BeginPlay 後有效）
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Roadside Parking|Debug")
	int32 ZoneCount = 0;

	// ================================================================
	//  API
	// ================================================================

	/// 取得 Zone 的世界中心 / Get zone world center
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	FVector GetZoneWorldCenter(int32 ZoneIndex) const;

	/// 取得 Zone 的世界範圍 (半徑) / Get zone world half-extent
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	FVector GetZoneWorldExtent(int32 ZoneIndex) const;

	/// 取得 Zone 起點（Box 左邊緣世界座標）/ Get zone start (box left edge world)
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	FVector GetZoneWorldStart(int32 ZoneIndex) const;

	/// 取得 Zone 終點（Box 右邊緣世界座標）/ Get zone end (box right edge world)
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	FVector GetZoneWorldEnd(int32 ZoneIndex) const;

	/// 總共幾個 Zone / Total zone count
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Roadside Parking")
	int32 GetZoneCount() const { return ZoneBoxes.Num(); }

protected:
	virtual void BeginPlay() override;

private:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USceneComponent> SceneRoot;

	/// Box 元件列表（BeginPlay 時從子元件收集）
	UPROPERTY()
	TArray<TObjectPtr<UBoxComponent>> ZoneBoxes;

	void CollectZoneBoxes();
	void DetectRoadSide();
};
