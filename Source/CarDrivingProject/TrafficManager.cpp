#include "TrafficManager.h"
#include "DrivingVehiclePawn.h"
#include "RoadPathFollowerComponent.h"
#include "RoadNetworkSubsystem.h"
#include "RoadTypes.h"
#include "ParkingLotActor.h"
#include "Components/SplineComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "TimerManager.h"

// ============================================================================
//  Constructor
// ============================================================================
ATrafficManager::ATrafficManager()
{
	PrimaryActorTick.bCanEverTick = false;
}

// ============================================================================
//  BeginPlay — 延遲到下一幀 spawn（確保 RoadNetworkSubsystem 已 BuildRoadCache）
//  Delay spawn to next frame (ensure road graph is built).
// ============================================================================
void ATrafficManager::BeginPlay()
{
	Super::BeginPlay();

	GetWorldTimerManager().SetTimerForNextTick(this, &ATrafficManager::SpawnAllVehicles);
}

// ============================================================================
//  CollectParkingActors — 收集場景中所有停車場
//  Collect all parking lot actors from the level.
// ============================================================================
void ATrafficManager::CollectParkingActors()
{
	UWorld* World = GetWorld();
	if (!World) return;

	AllParkingLots.Empty();

	for (TActorIterator<AParkingLotActor> It(World); It; ++It)
	{
		AllParkingLots.Add(*It);
	}

	UE_LOG(LogTemp, Warning,
		TEXT("TrafficManager: Found %d ParkingLots"),
		AllParkingLots.Num());
}

// ============================================================================
//  SpawnAllVehicles — 從停車場位置生成車輛
//  Spawn vehicles from parking lots.
// ============================================================================
void ATrafficManager::SpawnAllVehicles()
{
	UWorld* World = GetWorld();
	if (!World) return;

	CollectParkingActors();

	// 計算可用出生點總數 / Count total available spawn points
	int32 TotalParkingSpots = 0;
	for (AParkingLotActor* Lot : AllParkingLots)
	{
		if (Lot) TotalParkingSpots += Lot->SpotCount;
	}

	const int32 TotalSpawnPoints = TotalParkingSpots;
	if (TotalSpawnPoints == 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrafficManager: No parking lots — cannot spawn vehicles"));
		return;
	}

	if (VehicleConfigs.Num() == 0) return;

	// 建出生點列表 / Build spawn point list
	// 每個出生點存: 位置、旋轉、是否已使用
	struct FSpawnPoint
	{
		FVector Position;
		FRotator Rotation;
		bool bUsed = false;
	};

	TArray<FSpawnPoint> SpawnPoints;

	// 停車場的停車格 / Parking lot spots
	for (AParkingLotActor* Lot : AllParkingLots)
	{
		if (!Lot) continue;
		for (int32 i = 0; i < Lot->SpotCount; ++i)
		{
			FSpawnPoint SP;
			SP.Position = Lot->GetSpotWorldPosition(i) + FVector(0, 0, 30.0f);
			SP.Rotation = Lot->GetSpotWorldForward(i).Rotation();
			SpawnPoints.Add(SP);
		}
	}

	// 打亂出生點順序 / Shuffle spawn points
	for (int32 i = SpawnPoints.Num() - 1; i > 0; --i)
	{
		const int32 j = FMath::RandRange(0, i);
		SpawnPoints.Swap(i, j);
	}

	int32 SpawnPointIdx = 0;
	bool bOutOfSpawnPoints = false;

	for (int32 CfgIdx = 0; CfgIdx < VehicleConfigs.Num(); ++CfgIdx)
	{
		if (bOutOfSpawnPoints) break;
		const FVehicleSpawnConfig& Config = VehicleConfigs[CfgIdx];
		if (!Config.VehicleClass)
		{
			UE_LOG(LogTemp, Warning, TEXT("[TRAFFIC] Config[%d] has null VehicleClass, skipping"), CfgIdx);
			continue;
		}

		UE_LOG(LogTemp, Warning, TEXT("[TRAFFIC] Config[%d] class=%s Count=%d MinSpeed=%.0f MaxSpeed=%.0f"),
			CfgIdx, *Config.VehicleClass->GetName(), Config.Count, Config.MinSpeed, Config.MaxSpeed);

		for (int32 i = 0; i < Config.Count; ++i)
		{
			// 找一個未用的出生點 / Find an unused spawn point
			if (SpawnPointIdx >= SpawnPoints.Num())
			{
				UE_LOG(LogTemp, Warning,
					TEXT("TrafficManager: Not enough spawn points, spawned %d vehicles"),
					SpawnedVehicles.Num());
				bOutOfSpawnPoints = true;
				break;
			}

			FSpawnPoint& SP = SpawnPoints[SpawnPointIdx++];

			FActorSpawnParameters SpawnParams;
			SpawnParams.SpawnCollisionHandlingOverride =
				ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

			ADrivingVehiclePawn* Vehicle = World->SpawnActor<ADrivingVehiclePawn>(
				Config.VehicleClass, SP.Position, SP.Rotation, SpawnParams);

			if (!Vehicle) continue;

			URoadPathFollowerComponent* PF = Vehicle->PathFollower;
			if (PF)
			{
				const float RandSpeed = FMath::FRandRange(Config.MinSpeed, Config.MaxSpeed);
				PF->MaxSpeed = RandSpeed;
				PF->bAutoStart = false;
				PF->ParkingMode = EParkingMode::RoadsideStop;

				PF->OnPathComplete.AddDynamic(this, &ATrafficManager::OnVehiclePathComplete);
			}

			SpawnedVehicles.Add(Vehicle);

			UE_LOG(LogTemp, Warning,
				TEXT("TrafficManager: Spawned vehicle [%d] class=%s speed=%.0f at (%.0f,%.0f,%.0f)"),
				SpawnedVehicles.Num() - 1,
				*Config.VehicleClass->GetName(),
				PF ? PF->MaxSpeed : 0.0f,
				SP.Position.X, SP.Position.Y, SP.Position.Z);

			// 延遲一小段時間再分配目的地（讓車先停好在出生點）
			// Short delay before assigning destination (let car settle at spawn)
			FTimerHandle TimerHandle;
			FTimerDelegate TimerDelegate;
			TimerDelegate.BindLambda([this, Vehicle]()
			{
				if (Vehicle && IsValid(Vehicle))
				{
					AssignRandomDestination(Vehicle);
				}
			});
			GetWorldTimerManager().SetTimer(TimerHandle, TimerDelegate, 0.5f, false);
		}
	}

	UE_LOG(LogTemp, Warning,
		TEXT("TrafficManager: Total %d vehicles spawned"), SpawnedVehicles.Num());
}

// ============================================================================
//  AssignRandomDestination — 隨機選停車場作為目的地
//  Randomly pick a parking lot as destination.
// ============================================================================
void ATrafficManager::AssignRandomDestination(ADrivingVehiclePawn* Vehicle)
{
	if (!Vehicle) return;

	URoadPathFollowerComponent* PF = Vehicle->PathFollower;
	if (!PF) return;

	const int32 TotalLots = AllParkingLots.Num();

	if (TotalLots == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("[TRAFFIC] No parking destinations available"));
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("[TRAFFIC] AssignRandomDestination — %d lots, vehicle at (%.0f,%.0f,%.0f)"),
		TotalLots, Vehicle->GetActorLocation().X, Vehicle->GetActorLocation().Y, Vehicle->GetActorLocation().Z);

	// 隨機選一個停車場 / Pick random parking lot
	const int32 RandIdx = FMath::RandRange(0, TotalLots - 1);
	AParkingLotActor* Lot = AllParkingLots[RandIdx];
	if (Lot)
	{
		const int32 AvailableSpot = Lot->FindAvailableSpot();
		if (AvailableSpot != INDEX_NONE)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("TrafficManager: Vehicle → ParkingLot '%s' spot=%d"),
				*Lot->ParkingLotName, AvailableSpot);
			PF->NavigateToParkingLot(Lot, AvailableSpot);
			return;
		}
	}

	// 再試其它停車場 / Retry other parking lots
	for (AParkingLotActor* OtherLot : AllParkingLots)
	{
		if (!OtherLot) continue;
		const int32 Spot = OtherLot->FindAvailableSpot();
		if (Spot != INDEX_NONE)
		{
			PF->NavigateToParkingLot(OtherLot, Spot);
			return;
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("[TRAFFIC] All destinations full, cannot assign"));
}

// ============================================================================
//  OnVehiclePathComplete — 到達後延遲重新導航
//  Re-navigate after delay when path completes.
// ============================================================================
void ATrafficManager::OnVehiclePathComplete(bool bSuccess)
{
	for (ADrivingVehiclePawn* Vehicle : SpawnedVehicles)
	{
		if (!Vehicle) continue;
		if (PendingRenavigateVehicles.Contains(Vehicle)) continue;

		URoadPathFollowerComponent* PF = Vehicle->PathFollower;
		if (!PF) continue;

		if (PF->GetNavState() == ENavState::Parked || PF->GetNavState() == ENavState::Idle)
		{
			PendingRenavigateVehicles.Add(Vehicle);

			const float Delay = FMath::FRandRange(RenavigateDelayMin, RenavigateDelayMax);

			FTimerHandle TimerHandle;
			FTimerDelegate TimerDelegate;
			TimerDelegate.BindLambda([this, Vehicle]()
			{
				if (Vehicle && IsValid(Vehicle))
				{
					PendingRenavigateVehicles.Remove(Vehicle);
					AssignRandomDestination(Vehicle);
				}
			});

			GetWorldTimerManager().SetTimer(TimerHandle, TimerDelegate, Delay, false);
		}
	}
}
