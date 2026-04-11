#include "TrafficManager.h"
#include "DrivingVehiclePawn.h"
#include "RoadPathFollowerComponent.h"
#include "RoadNetworkSubsystem.h"
#include "RoadTypes.h"
#include "ParkingLotActor.h"
#include "RoadsideParkingActor.h"
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
//  CollectParkingActors — 收集場景中所有停車場和路邊停車
//  Collect all parking lot and roadside parking actors from the level.
// ============================================================================
void ATrafficManager::CollectParkingActors()
{
	UWorld* World = GetWorld();
	if (!World) return;

	AllParkingLots.Empty();
	AllRoadsideParking.Empty();

	for (TActorIterator<AParkingLotActor> It(World); It; ++It)
	{
		AllParkingLots.Add(*It);
	}

	for (TActorIterator<ARoadsideParkingActor> It(World); It; ++It)
	{
		AllRoadsideParking.Add(*It);
	}

	UE_LOG(LogTemp, Warning,
		TEXT("TrafficManager: Found %d ParkingLots, %d RoadsideParking"),
		AllParkingLots.Num(), AllRoadsideParking.Num());
}

// ============================================================================
//  SpawnAllVehicles — 從停車場/路邊停車位置生成車輛
//  Spawn vehicles from parking lots and roadside parking zones.
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

	int32 TotalRoadsideSpots = 0;
	for (ARoadsideParkingActor* RS : AllRoadsideParking)
	{
		if (RS) TotalRoadsideSpots += RS->GetSpotCount();
	}

	const int32 TotalSpawnPoints = TotalParkingSpots + TotalRoadsideSpots;
	if (TotalSpawnPoints == 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrafficManager: No parking lots or roadside zones — cannot spawn vehicles"));
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

	// 路邊停車格位置 / Roadside spot positions
	for (ARoadsideParkingActor* RS : AllRoadsideParking)
	{
		if (!RS) continue;
		for (int32 i = 0; i < RS->GetSpotCount(); ++i)
		{
			FSpawnPoint SP;
			SP.Position = RS->GetSpotWorldPosition(i) + FVector(0, 0, 30.0f);
			// 用 Arrow 的前方方向作為朝向
			// Use Arrow's forward direction as facing direction
			SP.Rotation = RS->GetSpotWorldForward(i).Rotation();
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
//  AssignRandomDestination — 隨機選停車場或路邊停車作為目的地
//  Randomly pick a parking lot or roadside zone as destination.
// ============================================================================
void ATrafficManager::AssignRandomDestination(ADrivingVehiclePawn* Vehicle)
{
	if (!Vehicle) return;

	URoadPathFollowerComponent* PF = Vehicle->PathFollower;
	if (!PF) return;

	const int32 TotalLots = AllParkingLots.Num();
	const int32 TotalRoadside = AllRoadsideParking.Num();
	const int32 TotalOptions = TotalLots + TotalRoadside;

	if (TotalOptions == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("[TRAFFIC] No parking destinations available"));
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("[TRAFFIC] AssignRandomDestination — %d lots, %d roadside, vehicle at (%.0f,%.0f,%.0f)"),
		TotalLots, TotalRoadside, Vehicle->GetActorLocation().X, Vehicle->GetActorLocation().Y, Vehicle->GetActorLocation().Z);

	// 隨機選一個目的地（停車場或路邊停車）
	// Random destination (parking lot or roadside)
	const int32 RandIdx = FMath::RandRange(0, TotalOptions - 1);

	if (RandIdx < TotalLots)
	{
		// 停車場目的地 / Parking lot destination
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
	}

	// 路邊停車目的地（或停車場滿了的 fallback）
	// Roadside destination (or fallback if lot was full)
	if (TotalRoadside > 0)
	{
		const int32 RSIdx = FMath::RandRange(0, TotalRoadside - 1);
		ARoadsideParkingActor* RS = AllRoadsideParking[RSIdx];
		if (RS)
		{
			const int32 SpotIdx = RS->FindAvailableSpot();
			if (SpotIdx != INDEX_NONE)
			{
				const FVector SpotPos = RS->GetSpotWorldPosition(SpotIdx);
				UE_LOG(LogTemp, Warning,
					TEXT("[TRAFFIC] Vehicle → Roadside '%s' spot=%d pos=(%.0f,%.0f,%.0f)"),
					*RS->ZoneName, SpotIdx, SpotPos.X, SpotPos.Y, SpotPos.Z);
				PF->NavigateToRoadside(RS, SpotPos, RS->bIsLeftSide, SpotIdx);
				return;
			}
		}
	}

	// 再試停車場 / Retry parking lots
	for (AParkingLotActor* Lot : AllParkingLots)
	{
		if (!Lot) continue;
		const int32 Spot = Lot->FindAvailableSpot();
		if (Spot != INDEX_NONE)
		{
			PF->NavigateToParkingLot(Lot, Spot);
			return;
		}
	}

	// 再試路邊停車 / Retry roadside
	for (ARoadsideParkingActor* RS : AllRoadsideParking)
	{
		if (!RS) continue;
		const int32 SpotIdx = RS->FindAvailableSpot();
		if (SpotIdx != INDEX_NONE)
		{
			const FVector Pos = RS->GetSpotWorldPosition(SpotIdx);
			PF->NavigateToRoadside(RS, Pos, RS->bIsLeftSide, SpotIdx);
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
