#include "RoadsideParkingActor.h"
#include "RoadNetworkSubsystem.h"
#include "RoadTypes.h"
#include "Components/ArrowComponent.h"
#include "Components/SplineComponent.h"

ARoadsideParkingActor::ARoadsideParkingActor()
{
	PrimaryActorTick.bCanEverTick = false;

	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	RootComponent = SceneRoot;
}

void ARoadsideParkingActor::BeginPlay()
{
	Super::BeginPlay();
	CollectSpotArrows();
	DetectRoadSide();
}

// ================================================================
//  收集 Arrow 子元件 / Collect child arrow components
// ================================================================

void ARoadsideParkingActor::CollectSpotArrows()
{
	SpotArrows.Reset();

	TArray<UArrowComponent*> Arrows;
	GetComponents<UArrowComponent>(Arrows);

	// 按名稱排序 / Sort by name
	Arrows.Sort([](const UArrowComponent& A, const UArrowComponent& B)
	{
		return A.GetName() < B.GetName();
	});

	for (UArrowComponent* Arrow : Arrows)
	{
		SpotArrows.Add(Arrow);
	}

	SpotCount = SpotArrows.Num();

	// 初始化佔用狀態 / Initialize occupation state
	SpotOccupied.SetNumZeroed(SpotCount);
	SpotVehicles.SetNum(SpotCount);
	SpotEdgeIds.Init(INDEX_NONE, SpotCount);  // 預設 INDEX_NONE，由 BindParkingActorsToEdges 填入

	UE_LOG(LogTemp, Warning, TEXT("RoadsideParking '%s': Found %d ArrowComponents as parking spots"), *ZoneName, SpotCount);
	for (int32 i = 0; i < SpotCount; ++i)
	{
		UE_LOG(LogTemp, Warning, TEXT("  Spot[%d] = '%s' at %s fwd=%s"),
			i, *SpotArrows[i]->GetName(),
			*SpotArrows[i]->GetComponentLocation().ToString(),
			*SpotArrows[i]->GetForwardVector().ToString());
	}
}

// ================================================================
//  API — 幾何 / Geometry
// ================================================================

FVector ARoadsideParkingActor::GetSpotWorldPosition(int32 SpotIndex) const
{
	if (SpotArrows.IsValidIndex(SpotIndex) && SpotArrows[SpotIndex])
	{
		return SpotArrows[SpotIndex]->GetComponentLocation();
	}
	return GetActorLocation();
}

FVector ARoadsideParkingActor::GetSpotWorldForward(int32 SpotIndex) const
{
	if (SpotArrows.IsValidIndex(SpotIndex) && SpotArrows[SpotIndex])
	{
		return SpotArrows[SpotIndex]->GetForwardVector();
	}
	return GetActorForwardVector();
}

// ================================================================
//  佔用 / Occupation
// ================================================================

int32 ARoadsideParkingActor::FindAvailableSpot() const
{
	for (int32 i = 0; i < SpotOccupied.Num(); ++i)
	{
		if (!SpotOccupied[i]) return i;
	}
	return INDEX_NONE;
}

bool ARoadsideParkingActor::OccupySpot(int32 SpotIndex, AActor* Vehicle)
{
	if (!SpotOccupied.IsValidIndex(SpotIndex)) return false;
	if (SpotOccupied[SpotIndex]) return false;
	SpotOccupied[SpotIndex] = true;
	SpotVehicles[SpotIndex] = Vehicle;
	UE_LOG(LogTemp, Warning, TEXT("[RS-OCCUPY] '%s' Spot[%d] occupied by %s"),
		*ZoneName, SpotIndex, Vehicle ? *Vehicle->GetName() : TEXT("null"));
	return true;
}

void ARoadsideParkingActor::ReleaseSpot(int32 SpotIndex)
{
	if (!SpotOccupied.IsValidIndex(SpotIndex)) return;
	UE_LOG(LogTemp, Warning, TEXT("[RS-RELEASE] '%s' Spot[%d] released"), *ZoneName, SpotIndex);
	SpotOccupied[SpotIndex] = false;
	SpotVehicles[SpotIndex] = nullptr;
}

bool ARoadsideParkingActor::IsSpotOccupied(int32 SpotIndex) const
{
	if (!SpotOccupied.IsValidIndex(SpotIndex)) return true;
	return SpotOccupied[SpotIndex];
}

int32 ARoadsideParkingActor::GetSpotEdgeId(int32 SpotIndex) const
{
	if (!SpotEdgeIds.IsValidIndex(SpotIndex)) return INDEX_NONE;
	return SpotEdgeIds[SpotIndex];
}

void ARoadsideParkingActor::AssignSpotEdgeId(int32 SpotIndex, int32 EdgeId)
{
	if (!SpotEdgeIds.IsValidIndex(SpotIndex)) return;
	SpotEdgeIds[SpotIndex] = EdgeId;
}

// ================================================================
//  道路側邊偵測 / Road side detection
// ================================================================

void ARoadsideParkingActor::DetectRoadSide()
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* RoadSub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSub) return;

	const FVector ActorLoc = GetActorLocation();
	const TArray<FRoadGraphEdge>& Edges = RoadSub->GetGraphEdges();

	// ---- 用第一個 Arrow 的前方方向找「同向」的 Edge ----
	// ---- Use first Arrow's forward to find direction-aligned edge ----
	FVector RefFwd = GetActorForwardVector();
	if (SpotArrows.Num() > 0 && SpotArrows[0])
	{
		RefFwd = SpotArrows[0]->GetForwardVector();
	}

	float BestScore = TNumericLimits<float>::Max();
	int32 BestEdgeId = INDEX_NONE;
	FVector BestSplinePoint = FVector::ZeroVector;
	FVector BestSplineTangent = FVector::ForwardVector;
	USplineComponent* BestSpline = nullptr;

	for (const FRoadGraphEdge& Edge : Edges)
	{
		if (!Edge.InputSpline) continue;

		const float ClosestKey = Edge.InputSpline->FindInputKeyClosestToWorldLocation(ActorLoc);
		const float SplineDist = Edge.InputSpline->GetDistanceAlongSplineAtSplineInputKey(ClosestKey);
		const FVector ClosestPoint = Edge.InputSpline->GetLocationAtDistanceAlongSpline(
			SplineDist, ESplineCoordinateSpace::World);
		const FVector SplineDir = Edge.InputSpline->GetDirectionAtDistanceAlongSpline(
			SplineDist, ESplineCoordinateSpace::World);

		// 只選方向夾角 < 30° 的 edge（cos30° ≈ 0.866）
		// Only consider edges within 30° of Arrow direction (cos30° ≈ 0.866)
		const float DirDot = FVector::DotProduct(RefFwd.GetSafeNormal2D(), SplineDir.GetSafeNormal2D());
		if (DirDot < 0.866f) continue;

		const float Dist = FVector::Dist(ActorLoc, ClosestPoint);
		// 方向已被門檻過濾，距離為主要排序依據
		// Direction filtered by threshold, sort primarily by distance
		const float Score = Dist;
		if (Score < BestScore)
		{
			BestScore = Score;
			BestEdgeId = Edge.EdgeId;
			BestSplinePoint = ClosestPoint;
			BestSplineTangent = SplineDir;
			BestSpline = Edge.InputSpline;
		}
	}

	NearestEdgeId = BestEdgeId;

	if (BestEdgeId != INDEX_NONE)
	{
		const FVector ToActor = (ActorLoc - BestSplinePoint).GetSafeNormal2D();
		const FVector SplineRight = FVector::CrossProduct(FVector::UpVector, BestSplineTangent).GetSafeNormal2D();
		const float DotRight = FVector::DotProduct(ToActor, SplineRight);
		bIsLeftSide = (DotRight < 0.0f);

		UE_LOG(LogTemp, Log, TEXT("RoadsideParking '%s': NearestEdge=%d, Side=%s (Arrow-aligned)"),
			*ZoneName, BestEdgeId, bIsLeftSide ? TEXT("Left") : TEXT("Right"));

		// ---- 按沿道路行進順序排序 Spots ----
		// ---- Sort spots by their projected distance along the road ----
		if (BestSpline && SpotArrows.Num() > 1)
		{
			// 計算每個 Arrow 在 spline 上的投影距離
			// Compute each arrow's projected distance along spline
			TArray<TPair<float, int32>> SpotDistances;
			SpotDistances.Reserve(SpotArrows.Num());
			for (int32 i = 0; i < SpotArrows.Num(); ++i)
			{
				if (!SpotArrows[i]) { SpotDistances.Add(TPair<float, int32>(0.0f, i)); continue; }
				const FVector SpotLoc = SpotArrows[i]->GetComponentLocation();
				const float Key = BestSpline->FindInputKeyClosestToWorldLocation(SpotLoc);
				const float Dist = BestSpline->GetDistanceAlongSplineAtSplineInputKey(Key);
				SpotDistances.Add(TPair<float, int32>(Dist, i));
			}

			// 按 spline 距離排序（車行進方向上先遇到的排前面）
			// Sort by spline distance (earlier in travel direction first)
			SpotDistances.Sort([](const TPair<float, int32>& A, const TPair<float, int32>& B)
			{
				return A.Key < B.Key;
			});

			// 重建排序後的陣列 / Rebuild sorted arrays
			TArray<TObjectPtr<UArrowComponent>> SortedArrows;
			TArray<bool> SortedOccupied;
			TArray<TWeakObjectPtr<AActor>> SortedVehicles;
			SortedArrows.Reserve(SpotArrows.Num());
			SortedOccupied.Reserve(SpotOccupied.Num());
			SortedVehicles.Reserve(SpotVehicles.Num());

			for (const auto& Pair : SpotDistances)
			{
				SortedArrows.Add(SpotArrows[Pair.Value]);
				SortedOccupied.Add(SpotOccupied[Pair.Value]);
				SortedVehicles.Add(SpotVehicles[Pair.Value]);
			}

			SpotArrows = MoveTemp(SortedArrows);
			SpotOccupied = MoveTemp(SortedOccupied);
			SpotVehicles = MoveTemp(SortedVehicles);

			UE_LOG(LogTemp, Log, TEXT("RoadsideParking '%s': Sorted %d spots by road travel order"),
				*ZoneName, SpotArrows.Num());
		}
	}
}
