#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrafficManager.generated.h"

class ADrivingVehiclePawn;
class AParkingLotActor;
class ARoadsideParkingActor;

/// 車輛生成配置 — 每種車的 BP 類別、數量、速度範圍
/// Vehicle spawn config — BP class, count, and speed range per type
USTRUCT(BlueprintType)
struct FVehicleSpawnConfig
{
	GENERATED_BODY()

	/// 車輛 BP 子類別（不同外觀的車）/ Vehicle BP subclass (different appearance)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	TSubclassOf<ADrivingVehiclePawn> VehicleClass;

	/// 這種車生成幾台 / How many of this type to spawn
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	int32 Count = 2;

	/// 此車種的最低隨機速度（cm/s）/ Min random speed for this type
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "100"))
	float MinSpeed = 1000.0f;

	/// 此車種的最高隨機速度（cm/s）/ Max random speed for this type
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "100"))
	float MaxSpeed = 2000.0f;
};

/// 交通管理器 — 車輛從停車場/路邊停車位置出發，目的地也是停車場或路邊停車
///
/// Traffic manager — vehicles spawn from parking lots/roadside zones,
/// destinations are also parking lots or roadside zones.
UCLASS()
class CARDRIVINGPROJECT_API ATrafficManager : public AActor
{
	GENERATED_BODY()

public:
	ATrafficManager();

	// ================================================================
	//  配置 / Configuration
	// ================================================================

	/// 車輛配置陣列 — 每個元素代表一種車型
	/// Vehicle config array — each element represents one vehicle type
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	TArray<FVehicleSpawnConfig> VehicleConfigs;

	/// 到達目的地後等多久再重新導航（秒）— 隨機範圍
	/// Delay before re-navigating after arrival (seconds) — random range
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float RenavigateDelayMin = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float RenavigateDelayMax = 3.0f;

	// ================================================================
	//  API
	// ================================================================

	/// 取得所有已生成的車輛 / Get all spawned vehicles
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Traffic")
	const TArray<ADrivingVehiclePawn*>& GetSpawnedVehicles() const { return SpawnedVehicles; }

protected:
	virtual void BeginPlay() override;

private:

	/// 已生成的車輛列表 / List of all spawned vehicles
	UPROPERTY()
	TArray<ADrivingVehiclePawn*> SpawnedVehicles;

	/// 場景中所有停車場（BeginPlay 收集）/ All parking lots in level (collected at BeginPlay)
	UPROPERTY()
	TArray<AParkingLotActor*> AllParkingLots;

	/// 場景中所有路邊停車（BeginPlay 收集）/ All roadside parking in level (collected at BeginPlay)
	UPROPERTY()
	TArray<ARoadsideParkingActor*> AllRoadsideParking;

	/// 收集場景中所有停車場和路邊停車 / Collect all parking actors from level
	void CollectParkingActors();

	/// 生成所有車輛 / Spawn all vehicles
	void SpawnAllVehicles();

	/// 為指定車輛分配隨機目的地（停車場或路邊停車）
	/// Assign random parking destination to a vehicle
	void AssignRandomDestination(ADrivingVehiclePawn* Vehicle);

	/// OnPathComplete 回呼：到達後延遲重新導航
	/// OnPathComplete callback: re-navigate after delay
	UFUNCTION()
	void OnVehiclePathComplete(bool bSuccess);

	/// 已排程重導航的車輛（避免重複排程）
	/// Vehicles with pending renavigate timers (avoid duplicates)
	TSet<ADrivingVehiclePawn*> PendingRenavigateVehicles;
};
