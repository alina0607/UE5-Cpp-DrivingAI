#include "TrafficManager.h"
#include "DrivingVehiclePawn.h"
#include "RoadPathFollowerComponent.h"
#include "RoadNetworkSubsystem.h"
#include "RoadTypes.h"
#include "Components/SplineComponent.h"
#include "Engine/World.h"
#include "TimerManager.h"

// ============================================================================
//  Constructor
// ============================================================================
ATrafficManager::ATrafficManager()
{
	PrimaryActorTick.bCanEverTick = false;

	// 預設一個空配置讓 Editor 裡看到結構
	// Default empty config so Editor shows the structure
}

// ============================================================================
//  BeginPlay — 延遲到下一幀 spawn（確保 RoadNetworkSubsystem 已 BuildRoadCache）
//  Delay spawn to next frame (ensure road graph is built).
// ============================================================================
void ATrafficManager::BeginPlay()
{
	Super::BeginPlay();

	// RoadNetworkSubsystem 在 HandleWorldBeginPlay 建圖
	// 用 NextTick timer 確保圖已建好
	// Road graph is built in HandleWorldBeginPlay.
	// Use NextTick timer to ensure graph is ready.
	GetWorldTimerManager().SetTimerForNextTick(this, &ATrafficManager::SpawnAllVehicles);
}

// ============================================================================
//  SpawnAllVehicles — 遍歷配置，生成所有車輛
//  Iterate configs and spawn all vehicles.
// ============================================================================
void ATrafficManager::SpawnAllVehicles()
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* RoadSub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSub)
	{
		UE_LOG(LogTemp, Warning, TEXT("TrafficManager: RoadNetworkSubsystem not found"));
		return;
	}

	const TArray<FRoadGraphEdge>& Edges = RoadSub->GetGraphEdges();
	if (Edges.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("TrafficManager: No graph edges — cannot spawn vehicles"));
		return;
	}

	// 暫時只生成 index 0 方便 debug / Temporarily only spawn index 0 for debugging
	if (VehicleConfigs.Num() == 0) return;

	for (int32 CfgIdx = 0; CfgIdx < 1; ++CfgIdx) // TODO: 改回 VehicleConfigs.Num()
	{
		const FVehicleSpawnConfig& Config = VehicleConfigs[CfgIdx];

		if (!Config.VehicleClass)
		{
			UE_LOG(LogTemp, Warning, TEXT("TrafficManager: Skipping config with null VehicleClass"));
			continue;
		}

		for (int32 i = 0; i < Config.Count; ++i)
		{
			// 隨機選一條 Edge / Pick random edge
			const int32 EdgeIdx = FMath::RandRange(0, Edges.Num() - 1);
			const FRoadGraphEdge& Edge = Edges[EdgeIdx];

			if (!Edge.InputSpline) continue;

			// 在 spline 上隨機取一個位置（避開頭尾 10%）
			// Random position on spline (avoid first/last 10%)
			const float SplineLen = Edge.InputSpline->GetSplineLength();
			const float RandDist = FMath::FRandRange(SplineLen * 0.1f, SplineLen * 0.9f);

			const FVector SpawnPos = Edge.InputSpline->GetLocationAtDistanceAlongSpline(
				RandDist, ESplineCoordinateSpace::World);
			const FRotator SpawnRot = Edge.InputSpline->GetRotationAtDistanceAlongSpline(
				RandDist, ESplineCoordinateSpace::World);

			// 稍微抬高避免卡地板 / Slightly elevate to avoid ground clipping
			FVector AdjustedPos = SpawnPos + FVector(0.0f, 0.0f, 30.0f);

			FActorSpawnParameters SpawnParams;
			SpawnParams.SpawnCollisionHandlingOverride =
				ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

			ADrivingVehiclePawn* Vehicle = World->SpawnActor<ADrivingVehiclePawn>(
				Config.VehicleClass, AdjustedPos, SpawnRot, SpawnParams);

			if (!Vehicle) continue;

			// 設定隨機速度 / Set random speed
			URoadPathFollowerComponent* PF = Vehicle->PathFollower;
			if (PF)
			{
				const float RandSpeed = FMath::FRandRange(Config.MinSpeed, Config.MaxSpeed);
				PF->MaxSpeed = RandSpeed;
				PF->bAutoStart = false; // TrafficManager 控制導航 / We control navigation
				PF->ParkingMode = EParkingMode::RoadsideStop; // 路邊停車，不擋路 / Roadside parking, don't block road

				// 綁定 OnPathComplete / Bind path complete
				PF->OnPathComplete.AddDynamic(this, &ATrafficManager::OnVehiclePathComplete);
			}

			SpawnedVehicles.Add(Vehicle);

			UE_LOG(LogTemp, Warning,
				TEXT("TrafficManager: Spawned vehicle [%d] class=%s speed=%.0f at Edge %d dist=%.0f"),
				SpawnedVehicles.Num() - 1,
				*Config.VehicleClass->GetName(),
				PF ? PF->MaxSpeed : 0.0f,
				EdgeIdx, RandDist);

			// 立刻分配隨機目���地 / Immediately assign random destination
			AssignRandomDestination(Vehicle);
		}
	}

	UE_LOG(LogTemp, Warning,
		TEXT("TrafficManager: Total %d vehicles spawned"), SpawnedVehicles.Num());
}

// ============================================================================
//  AssignRandomDestination — 為指定車輛分配隨機目的地
//  Assign a random destination to the given vehicle.
// ============================================================================
void ATrafficManager::AssignRandomDestination(ADrivingVehiclePawn* Vehicle)
{
	if (!Vehicle) return;

	URoadPathFollowerComponent* PF = Vehicle->PathFollower;
	if (!PF) return;

	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* RoadSub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSub) return;

	// 排除車輛目前最近的 node（避免原地導航）
	// Exclude the node nearest to the vehicle (avoid navigating to current location)
	const int32 NearestNode = RoadSub->FindNearestGraphNode(Vehicle->GetActorLocation());
	const int32 GoalNode = PickRandomGoalNode(NearestNode);
	if (GoalNode == INDEX_NONE)
	{
		UE_LOG(LogTemp, Warning, TEXT("TrafficManager: No valid goal node found"));
		return;
	}

	UE_LOG(LogTemp, Warning,
		TEXT("TrafficManager: AssignRandomDest — Vehicle at nearest Node %d → Goal Node %d"),
		NearestNode, GoalNode);

	PF->NavigateToNode(GoalNode);
}

// ============================================================================
//  OnVehiclePathComplete — 到達後延���重新導航
//  Re-navigate after delay when path completes.
// ============================================================================
void ATrafficManager::OnVehiclePathComplete(bool bSuccess)
{
	// 遍歷所有停著的車，排程重新導航（跳過已排程的）
	// Iterate all stopped vehicles, schedule renavigate (skip already scheduled)
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

// ============================================================================
//  PickRandomGoalNode — 隨機選一個 Graph Node（排除指定 Node）
//  Pick a random graph node, excluding the given node.
// ============================================================================
int32 ATrafficManager::PickRandomGoalNode(int32 ExcludeNodeId) const
{
	UWorld* World = GetWorld();
	if (!World) return INDEX_NONE;

	URoadNetworkSubsystem* RoadSub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSub) return INDEX_NONE;

	const TArray<FRoadGraphNode>& Nodes = RoadSub->GetGraphNodes();
	if (Nodes.Num() <= 1) return INDEX_NONE;

	// 嘗試最多 20 次避免選到相同 node / Try up to 20 times to avoid same node
	for (int32 Attempt = 0; Attempt < 20; ++Attempt)
	{
		const int32 Idx = FMath::RandRange(0, Nodes.Num() - 1);
		if (Nodes[Idx].NodeId != ExcludeNodeId)
		{
			return Nodes[Idx].NodeId;
		}
	}

	// 退底：回傳第一個不同的 / Fallback: return first different
	for (const FRoadGraphNode& Node : Nodes)
	{
		if (Node.NodeId != ExcludeNodeId) return Node.NodeId;
	}

	return INDEX_NONE;
}
