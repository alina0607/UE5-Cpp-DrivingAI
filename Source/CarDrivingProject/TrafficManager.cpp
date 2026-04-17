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
//  Delay spawn to next frame (ensure road graph is built).
// ============================================================================
void ATrafficManager::BeginPlay()
{
	Super::BeginPlay();

	GetWorldTimerManager().SetTimerForNextTick(this, &ATrafficManager::SpawnAllVehicles);
}

// ============================================================================
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

}

// ============================================================================
//  Spawn vehicles from parking lots.
// ============================================================================
void ATrafficManager::SpawnAllVehicles()
{
	UWorld* World = GetWorld();
	if (!World) return;

	CollectParkingActors();

	// Count total available spawn points
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

	// Build spawn point list — now tracks source lot + spot so departure order can
	// be enforced (spot 0 leaves before spot 1, etc.)
	struct FSpawnPoint
	{
		FVector Position;
		FRotator Rotation;
		AParkingLotActor* SourceLot = nullptr;
		int32 SpotIndex = INDEX_NONE;
		bool bUsed = false;
	};

	TArray<FSpawnPoint> SpawnPoints;

	// Parking lot spots
	for (AParkingLotActor* Lot : AllParkingLots)
	{
		if (!Lot) continue;
		for (int32 i = 0; i < Lot->SpotCount; ++i)
		{
			FSpawnPoint SP;
			SP.Position = Lot->GetSpotWorldPosition(i) + FVector(0, 0, 30.0f);
			SP.Rotation = Lot->GetSpotWorldForward(i).Rotation();
			SP.SourceLot = Lot;
			SP.SpotIndex = i;
			SpawnPoints.Add(SP);
		}
	}

	// Shuffle spawn points (which physical spots we choose is still random,
	// but each spawn keeps its lot/spot context — ordering enforced by PathFollower)
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


		for (int32 i = 0; i < Config.Count; ++i)
		{
			// Find an unused spawn point
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
				PF->MaxSpeed = 1500.f;
				PF->bAutoStart = false;
				PF->ParkingMode = EParkingMode::RoadsideStop;

				// Register source-lot context so sequential-departure gating works
				PF->SetDepartureContext(SP.SourceLot, SP.SpotIndex);

				PF->OnPathComplete.AddDynamic(this, &ATrafficManager::OnVehiclePathComplete);
			}

			SpawnedVehicles.Add(Vehicle);

			UE_LOG(LogTemp, Warning,
				TEXT("TrafficManager: Spawned vehicle [%d] class=%s speed=%.0f at (%.0f,%.0f,%.0f)"),
				SpawnedVehicles.Num() - 1,
				*Config.VehicleClass->GetName(),
				PF ? PF->MaxSpeed : 0.0f,
				SP.Position.X, SP.Position.Y, SP.Position.Z);

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
}

// ============================================================================
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

	// 初次出場用 RequestDepartToParkingLot（會做 spot 順序控管）；
	// 若 HasDepartedFromSource 已為 true（已離開原 spot），後續重新導航就直接呼叫
	// NavigateToParkingLot — gating 只對出場的那一次有意義。
	auto DispatchNavigate = [&](AParkingLotActor* DestLot, int32 DestSpot)
	{
		if (PF->HasDepartedFromSource())
		{
			PF->NavigateToParkingLot(DestLot, DestSpot);
		}
		else
		{
			PF->RequestDepartToParkingLot(DestLot, DestSpot);
		}
	};

	//Pick random parking lot
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
			DispatchNavigate(Lot, AvailableSpot);
			return;
		}
	}

	// Retry other parking lots
	for (AParkingLotActor* OtherLot : AllParkingLots)
	{
		if (!OtherLot) continue;
		const int32 Spot = OtherLot->FindAvailableSpot();
		if (Spot != INDEX_NONE)
		{
			DispatchNavigate(OtherLot, Spot);
			return;
		}
	}

}

// ============================================================================
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
