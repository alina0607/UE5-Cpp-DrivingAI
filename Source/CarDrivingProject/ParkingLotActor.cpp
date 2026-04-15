#include "ParkingLotActor.h"
#include "RoadNetworkSubsystem.h"
#include "RoadTypes.h"
#include "Components/ArrowComponent.h"
#include "Components/BoxComponent.h"
#include "Components/SplineComponent.h"
#include "DrawDebugHelpers.h"

AParkingLotActor::AParkingLotActor()
{
	PrimaryActorTick.bCanEverTick = false;

	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	RootComponent = SceneRoot;

	// 建物 / 停車場地板 Mesh — 在 BP 裡設定 Static Mesh
	// Building / parking floor mesh — set Static Mesh in BP
	BuildingMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BuildingMesh"));
	BuildingMesh->SetupAttachment(SceneRoot);

	ParkingMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ParkingMesh"));
	ParkingMesh->SetupAttachment(SceneRoot);
}

void AParkingLotActor::BeginPlay()
{
	Super::BeginPlay();
	CollectSpotArrows();
	DetectRoadSide();
}

// ================================================================
//  收集子元件中的 Arrow 作為停車格 / Collect child arrows as spots
// ================================================================

void AParkingLotActor::CollectSpotArrows()
{
	SpotArrows.Reset();

	// 收集所有 UArrowComponent 子元件
	// Collect all UArrowComponent children
	TArray<UArrowComponent*> Arrows;
	GetComponents<UArrowComponent>(Arrows);

	// 按名稱排序（Spot_0, Spot_1, ...）/ Sort by name
	Arrows.Sort([](const UArrowComponent& A, const UArrowComponent& B)
	{
		return A.GetName() < B.GetName();
	});

	for (UArrowComponent* Arrow : Arrows)
	{
		SpotArrows.Add(Arrow);
	}

	SpotCount = SpotArrows.Num();
	SpotOccupied.SetNumZeroed(SpotCount);
	SpotVehicles.SetNum(SpotCount);
	SpotEdgeIds.Init(INDEX_NONE, SpotCount);  // 預設 INDEX_NONE，由 BindParkingActorsToEdges 填入

	UE_LOG(LogTemp, Warning, TEXT("ParkingLot '%s': Found %d ArrowComponents as parking spots"), *ParkingLotName, SpotCount);
	for (int32 i = 0; i < SpotArrows.Num(); ++i)
	{
		UE_LOG(LogTemp, Warning, TEXT("  Spot[%d] = '%s' at %s fwd=%s"),
			i, *SpotArrows[i]->GetName(),
			*SpotArrows[i]->GetComponentLocation().ToString(),
			*SpotArrows[i]->GetForwardVector().ToString());
	}
}

// ================================================================
//  API
// ================================================================

FVector AParkingLotActor::GetSpotWorldPosition(int32 SpotIndex) const
{
	if (SpotArrows.IsValidIndex(SpotIndex) && SpotArrows[SpotIndex])
	{
		return SpotArrows[SpotIndex]->GetComponentLocation();
	}
	return GetActorLocation();
}

FVector AParkingLotActor::GetSpotWorldForward(int32 SpotIndex) const
{
	if (SpotArrows.IsValidIndex(SpotIndex) && SpotArrows[SpotIndex])
	{
		return SpotArrows[SpotIndex]->GetForwardVector();
	}
	return GetActorForwardVector();
}

int32 AParkingLotActor::FindAvailableSpot() const
{
	for (int32 i = 0; i < SpotOccupied.Num(); ++i)
	{
		if (!SpotOccupied[i]) return i;
	}
	return INDEX_NONE;
}

bool AParkingLotActor::OccupySpot(int32 SpotIndex, AActor* Vehicle)
{
	if (!SpotOccupied.IsValidIndex(SpotIndex)) return false;
	if (SpotOccupied[SpotIndex]) return false;

	SpotOccupied[SpotIndex] = true;
	SpotVehicles[SpotIndex] = Vehicle;
	return true;
}

void AParkingLotActor::ReleaseSpot(int32 SpotIndex)
{
	if (!SpotOccupied.IsValidIndex(SpotIndex)) return;
	SpotOccupied[SpotIndex] = false;
	SpotVehicles[SpotIndex] = nullptr;
}

bool AParkingLotActor::IsSpotOccupied(int32 SpotIndex) const
{
	if (!SpotOccupied.IsValidIndex(SpotIndex)) return true;
	return SpotOccupied[SpotIndex];
}

int32 AParkingLotActor::GetSpotEdgeId(int32 SpotIndex) const
{
	if (!SpotEdgeIds.IsValidIndex(SpotIndex)) return INDEX_NONE;
	return SpotEdgeIds[SpotIndex];
}

void AParkingLotActor::AssignSpotEdgeId(int32 SpotIndex, int32 EdgeId)
{
	if (!SpotEdgeIds.IsValidIndex(SpotIndex)) return;
	SpotEdgeIds[SpotIndex] = EdgeId;
}

// [PARK-NAV] 一次設定整個停車場的綁定 Edge + 投影點
// [PARK-NAV] Set bound edge + projection point for the whole lot
void AParkingLotActor::AssignBoundEdge(int32 EdgeId, const FVector& ProjectionPoint)
{
	BoundEdgeId = EdgeId;
	BoundProjectionPoint = ProjectionPoint;
	// 同步寫到所有 SpotEdgeIds，讓舊 API 也能用 / Mirror to all spot slots for legacy API
	for (int32 i = 0; i < SpotEdgeIds.Num(); ++i)
	{
		SpotEdgeIds[i] = EdgeId;
	}

	UE_LOG(LogTemp, Warning,
		TEXT("[PARK-NAV] '%s' AssignBoundEdge EdgeId=%d ProjPt=(%.0f,%.0f,%.0f)"),
		*ParkingLotName, EdgeId, ProjectionPoint.X, ProjectionPoint.Y, ProjectionPoint.Z);
}

// ================================================================
//  道路側邊偵測 / Road side detection
// ================================================================

void AParkingLotActor::DetectRoadSide()
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* RoadSub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSub) return;

	const FVector LotLocation = GetActorLocation();
	const TArray<FRoadGraphEdge>& Edges = RoadSub->GetGraphEdges();

	float BestDistSq = TNumericLimits<float>::Max();
	int32 BestEdgeId = INDEX_NONE;
	float BestDistOnSpline = 0.0f;
	FVector BestSplinePoint = FVector::ZeroVector;
	FVector BestSplineTangent = FVector::ForwardVector;

	for (const FRoadGraphEdge& Edge : Edges)
	{
		if (!Edge.InputSpline) continue;

		const float ClosestKey = Edge.InputSpline->FindInputKeyClosestToWorldLocation(LotLocation);
		const float SplineDist = Edge.InputSpline->GetDistanceAlongSplineAtSplineInputKey(ClosestKey);
		const FVector ClosestPoint = Edge.InputSpline->GetLocationAtDistanceAlongSpline(
			SplineDist, ESplineCoordinateSpace::World);

		const float DistSq = FVector::DistSquared(LotLocation, ClosestPoint);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestEdgeId = Edge.EdgeId;
			BestDistOnSpline = SplineDist;
			BestSplinePoint = ClosestPoint;
			BestSplineTangent = Edge.InputSpline->GetDirectionAtDistanceAlongSpline(
				SplineDist, ESplineCoordinateSpace::World);
		}
	}

	NearestEdgeId = BestEdgeId;
	DistanceOnNearestEdge = BestDistOnSpline;

	if (BestEdgeId != INDEX_NONE)
	{
		const FVector ToLot = (LotLocation - BestSplinePoint).GetSafeNormal2D();
		const FVector SplineRight = FVector::CrossProduct(FVector::UpVector, BestSplineTangent).GetSafeNormal2D();
		const float DotRight = FVector::DotProduct(ToLot, SplineRight);
		bIsLeftSide = (DotRight < 0.0f);

		UE_LOG(LogTemp, Log, TEXT("ParkingLot '%s': NearestEdge=%d, Side=%s"),
			*ParkingLotName, BestEdgeId, bIsLeftSide ? TEXT("Left") : TEXT("Right"));
	}
}

// ================================================================
//  編輯器視覺化 / Editor visualization
// ================================================================

