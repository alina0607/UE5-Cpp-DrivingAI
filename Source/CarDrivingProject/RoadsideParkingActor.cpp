#include "RoadsideParkingActor.h"
#include "RoadNetworkSubsystem.h"
#include "RoadTypes.h"
#include "Components/BoxComponent.h"
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
	CollectZoneBoxes();
	DetectRoadSide();
}

// ================================================================
//  收集 Box 子元件 / Collect child box components
// ================================================================

void ARoadsideParkingActor::CollectZoneBoxes()
{
	ZoneBoxes.Reset();

	TArray<UBoxComponent*> Boxes;
	GetComponents<UBoxComponent>(Boxes);

	// 按名稱排序 / Sort by name
	Boxes.Sort([](const UBoxComponent& A, const UBoxComponent& B)
	{
			return A.GetComponentLocation().X < B.GetComponentLocation().X;
	});

	for (UBoxComponent* Box : Boxes)
	{
		ZoneBoxes.Add(Box);
	}

	ZoneCount = ZoneBoxes.Num();
	UE_LOG(LogTemp, Log, TEXT("RoadsideParking '%s': Found %d zone boxes"), *ZoneName, ZoneCount);
}

// ================================================================
//  API
// ================================================================

FVector ARoadsideParkingActor::GetZoneWorldCenter(int32 ZoneIndex) const
{
	if (ZoneBoxes.IsValidIndex(ZoneIndex) && ZoneBoxes[ZoneIndex])
	{
		return ZoneBoxes[ZoneIndex]->GetComponentLocation();
	}
	return GetActorLocation();
}

FVector ARoadsideParkingActor::GetZoneWorldExtent(int32 ZoneIndex) const
{
	if (ZoneBoxes.IsValidIndex(ZoneIndex) && ZoneBoxes[ZoneIndex])
	{
		return ZoneBoxes[ZoneIndex]->GetScaledBoxExtent();
	}
	return FVector(100.0f);
}

FVector ARoadsideParkingActor::GetZoneWorldStart(int32 ZoneIndex) const
{
	if (ZoneBoxes.IsValidIndex(ZoneIndex) && ZoneBoxes[ZoneIndex])
	{
		UBoxComponent* Box = ZoneBoxes[ZoneIndex];
		const FVector Center = Box->GetComponentLocation();
		const FVector Forward = Box->GetForwardVector();
		const FVector Extent = Box->GetScaledBoxExtent();
		// 起點 = 中心 - 前方 × X半徑 / Start = center - forward × X extent
		return Center - Forward * Extent.X;
	}
	return GetActorLocation();
}

FVector ARoadsideParkingActor::GetZoneWorldEnd(int32 ZoneIndex) const
{
	if (ZoneBoxes.IsValidIndex(ZoneIndex) && ZoneBoxes[ZoneIndex])
	{
		UBoxComponent* Box = ZoneBoxes[ZoneIndex];
		const FVector Center = Box->GetComponentLocation();
		const FVector Forward = Box->GetForwardVector();
		const FVector Extent = Box->GetScaledBoxExtent();
		// 終點 = 中心 + 前方 × X半徑 / End = center + forward × X extent
		return Center + Forward * Extent.X;
	}
	return GetActorLocation();
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

	float BestDistSq = TNumericLimits<float>::Max();
	int32 BestEdgeId = INDEX_NONE;
	FVector BestSplinePoint = FVector::ZeroVector;
	FVector BestSplineTangent = FVector::ForwardVector;

	for (const FRoadGraphEdge& Edge : Edges)
	{
		if (!Edge.InputSpline) continue;

		const float ClosestKey = Edge.InputSpline->FindInputKeyClosestToWorldLocation(ActorLoc);
		const float SplineDist = Edge.InputSpline->GetDistanceAlongSplineAtSplineInputKey(ClosestKey);
		const FVector ClosestPoint = Edge.InputSpline->GetLocationAtDistanceAlongSpline(
			SplineDist, ESplineCoordinateSpace::World);

		const float DistSq = FVector::DistSquared(ActorLoc, ClosestPoint);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestEdgeId = Edge.EdgeId;
			BestSplinePoint = ClosestPoint;
			BestSplineTangent = Edge.InputSpline->GetDirectionAtDistanceAlongSpline(
				SplineDist, ESplineCoordinateSpace::World);
		}
	}

	NearestEdgeId = BestEdgeId;

	if (BestEdgeId != INDEX_NONE)
	{
		const FVector ToActor = (ActorLoc - BestSplinePoint).GetSafeNormal2D();
		const FVector SplineRight = FVector::CrossProduct(FVector::UpVector, BestSplineTangent).GetSafeNormal2D();
		const float DotRight = FVector::DotProduct(ToActor, SplineRight);
		bIsLeftSide = (DotRight < 0.0f);

		UE_LOG(LogTemp, Log, TEXT("RoadsideParking '%s': NearestEdge=%d, Side=%s"),
			*ZoneName, BestEdgeId, bIsLeftSide ? TEXT("Left") : TEXT("Right"));
	}
}
