#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrafficManager.generated.h"

class ADrivingVehiclePawn;
class AParkingLotActor;

/// Vehicle spawn config — BP class, count, and speed range per type
USTRUCT(BlueprintType)
struct FVehicleSpawnConfig
{
	GENERATED_BODY()

	/// Vehicle BP subclass (different appearance)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	TSubclassOf<ADrivingVehiclePawn> VehicleClass;

	/// How many of this type to spawn
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	int32 Count = 2;

	/// Min random speed for this type
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "100"))
	float MinSpeed = 1000.0f;

	/// Max random speed for this type
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "100"))
	float MaxSpeed = 2000.0f;
};

/// Traffic manager — vehicles spawn from parking lots,
/// destinations are also parking lots.
UCLASS()
class CARDRIVINGPROJECT_API ATrafficManager : public AActor
{
	GENERATED_BODY()

public:
	ATrafficManager();

	// ================================================================
	// Configuration
	// ================================================================

	/// Vehicle config array — each element represents one vehicle type
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	TArray<FVehicleSpawnConfig> VehicleConfigs;

	/// Delay before re-navigating after arrival (seconds) — random range
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float RenavigateDelayMin = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float RenavigateDelayMax = 3.0f;

	// ================================================================
	//  API
	// ================================================================

	/// Get all spawned vehicles
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Traffic")
	const TArray<ADrivingVehiclePawn*>& GetSpawnedVehicles() const { return SpawnedVehicles; }

protected:
	virtual void BeginPlay() override;

private:

	/// List of all spawned vehicles
	UPROPERTY()
	TArray<ADrivingVehiclePawn*> SpawnedVehicles;

	///  All parking lots in level (collected at BeginPlay)
	UPROPERTY()
	TArray<AParkingLotActor*> AllParkingLots;

	///  Collect all parking actors from level
	void CollectParkingActors();

	/// Spawn all vehicles
	void SpawnAllVehicles();

	/// Assign random parking destination to a vehicle
	void AssignRandomDestination(ADrivingVehiclePawn* Vehicle);

	/// OnPathComplete callback: re-navigate after delay
	UFUNCTION()
	void OnVehiclePathComplete(bool bSuccess);

	/// Vehicles with pending renavigate timers (avoid duplicates)
	TSet<ADrivingVehiclePawn*> PendingRenavigateVehicles;
};
