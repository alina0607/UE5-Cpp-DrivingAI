#include "RoadPathFollowerComponent.h"
#include "RoadNetworkSubsystem.h"
#include "RoadRuleLibrary.h"
#include "RoadWorldSettings.h"
#include "CollisionChannels.h"
#include "ParkingLotActor.h"
#include "Components/SplineComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "TimerManager.h"
#include "UObject/UObjectIterator.h"

// ============================================================================
//  ComputeLaneOffsetCm — 靜態工具函式（跟之前一樣，不變）
//  Static helper for lane offset calculation (unchanged).
// ============================================================================
static float ComputeLaneOffsetCm(
	bool bTwoRoads,
	double TwoRoadsGapM,
	uint8 RoadType,
	int32 LaneIndex,
	float AutoLaneWidthCm,
	float AutoMedianCm,
	float TwoRoadsMedianAdjust,
	float TwoRoadsLaneWidthAdjust,
	float SharedMedianAdjust,
	float SharedLaneWidthAdjust)
{
	if (bTwoRoads)
	{
		const float GapHalfCm = static_cast<float>(FMath::Max(TwoRoadsGapM, 0.0)) * 50.0f;
		const float Median = AutoMedianCm + TwoRoadsMedianAdjust;
		const float LaneWidth = FMath::Max(AutoLaneWidthCm + TwoRoadsLaneWidthAdjust, 1.0f);
		return GapHalfCm + Median + LaneWidth * (LaneIndex + 0.5f);
	}

	// SharedRoadMedianAdjust 只在 2+2-Lane Wide Road (RoadType=4) 有效
	// SharedRoadMedianAdjust only applies to 2+2-Lane Wide Road (RoadType=4)
	const float MedianAdjust = (RoadType == 4) ? SharedMedianAdjust : 0.0f;
	const float Median = AutoMedianCm + MedianAdjust;
	const float LaneWidth = FMath::Max(AutoLaneWidthCm + SharedLaneWidthAdjust, 1.0f);
	return Median + LaneWidth * (LaneIndex + 0.5f);
}

// ============================================================================
//  Constructor
// ============================================================================
URoadPathFollowerComponent::URoadPathFollowerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

// ============================================================================
//  BeginPlay
// ============================================================================
void URoadPathFollowerComponent::BeginPlay()
{
	Super::BeginPlay();

	if (bAutoStart)
	{
		// Road graph 在 HandleWorldBeginPlay 才建好，比 Component BeginPlay 晚
		// 所以設 flag，第一次 Tick 時才啟動
		// Road graph isn't ready during BeginPlay. Defer to first Tick.
		bPendingStart = true;
	}
}

// ============================================================================
//  ResetFollowingState — 重置所有跟隨/導航狀態（重導航時使用）
//  Reset all following/navigation state (used when re-routing).
// ============================================================================
// ============================================================================
//  LogEvent — push to ring buffer + UE_LOG with unified tag
// ============================================================================
void URoadPathFollowerComponent::LogEvent(const FString& EventText)
{
	const float T = (GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f);
	const FString Line = FString::Printf(TEXT("[%6.1fs] %s"), T, *EventText);
	RecentEvents.Add(Line);
	if (RecentEvents.Num() > MaxRecentEvents)
	{
		RecentEvents.RemoveAt(0, RecentEvents.Num() - MaxRecentEvents);
	}

	AActor* Owner = GetOwner();
}

// ============================================================================
//  HandleEnteredNewSegment — called right after CurrentSegmentIndex++.
//  Detects final-edge entry and forces a pull-over lane change + disables overtake.
// ============================================================================
void URoadPathFollowerComponent::HandleEnteredNewSegment()
{
	if (!PathSegments.IsValidIndex(CurrentSegmentIndex)) return;

	const bool bFinalNow = (CurrentSegmentIndex == PathSegments.Num() - 1);
	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];

	if (bFinalNow && !bIsFinalEdge)
	{
		bIsFinalEdge = true;

		// Cancel any in-progress overtake so we don't leave lanes halfway.
		if (OvertakeState != EOvertakeState::None)
		{
			OvertakeState = EOvertakeState::None;
			OvertakeTargetActor = nullptr;
			OvertakeReturnConfirmTimer = 0.0f;
		}

		// Snap to outermost forward lane for pull-over alignment. Don't
		// commit the lane change immediately — defer to the per-tick
		// safety check so we don't cut in front of a car behind us in
		// the outer lane. Timeout after FinalEdgeLaneForceMaxWait seconds.
		const int32 ForwardLanes = FMath::Max(1, Seg.DrivingRule.ForwardLaneCount);
		const int32 OuterLane = ForwardLanes - 1;
		if (TargetLaneIndex != OuterLane)
		{
			bPendingFinalEdgeForce = true;
			PendingFinalEdgeTargetLane = OuterLane;
			FinalEdgeLaneForceWait = 0.0f;
			CurrentTurnSignal = (OuterLane > CurrentLaneIndex) ? ETurnSignal::Right : CurrentTurnSignal;
		}

		LogEvent(FString::Printf(TEXT("ENTER FINAL EDGE seg=%d edge=%d pendingLane=%d"),
			CurrentSegmentIndex, Seg.EdgeId, OuterLane));
	}
	else
	{
		LogEvent(FString::Printf(TEXT("Seg %d/%d edge=%d lane=%d"),
			CurrentSegmentIndex, PathSegments.Num() - 1, Seg.EdgeId, CurrentLaneIndex));
	}
}

// ============================================================================
void URoadPathFollowerComponent::ResetFollowingState()
{
	bIsFollowing = false;
	bIsFinalEdge = false;
	bOnJunctionCurve = false;
	bIsUTurnCurve = false;
	bIsGapBridgeCurve = false;
	bJustExitedJunctionCurve = false;
	bIsChangingLane = false;
	bOnParkingCurve = false;
	ParkCurveProgress = 0.0f;
	bOnDepartureCurve = false;
	DepCurveProgress = 0.0f;
	JCurveProgress = 0.0f;
	ParkingTargetOffset = 0.0f;
	CurrentSpeed = 0.0f;
	SmoothedDesiredSpeed = 0.0f;
	CurrentTurnSignal = ETurnSignal::None;
	NavState = ENavState::Idle;
	PathSegments.Empty();
	CurrentSegmentIndex = 0;
	ObstacleDistance = -1.0f;
	ObstacleActor = nullptr;
	OvertakeState = EOvertakeState::None;
	OvertakePassedDistAccum = 0.0f;
	OvertakeTargetActor = nullptr;
	OvertakeReturnConfirmTimer = 0.0f;
	StuckTimer = 0.0f;
	bIsStuckReversing = false;
	StuckReversedSoFar = 0.0f;
	CurrentStuckReverseTarget = StuckReverseDistance;
	StuckCooldownTimer = 0.0f;
	ConsecutiveStuckCount = 0;
	TimeSinceLastStuck = 999.0f;
	RearWaitAccum = 0.0f;
	LastYieldReason.Empty();
	LastYieldLogTime = -999.0f;

	// 注意：停車格的釋放與目的地清除不在這裡做。
	// ResetFollowingState 會在 NavigateTo* 設定完 Target* 之後（OccupySpot 之後）才呼叫，
	// 在這裡釋放會把剛佔用的「新」格子也釋掉。釋放交給 ReleasePreviousDestinationSpot()，
	// 由各 NavigateTo* 在「設定 Target* 之前」呼叫。
	// NOTE: spot release / Target* clearing is intentionally NOT done here.
	// ResetFollowingState runs AFTER the new NavigateTo* has already OccupySpot'd and assigned
	// Target*, so doing it here would release the just-occupied new spot. The release is
	// handled by ReleasePreviousDestinationSpot(), called by each NavigateTo* BEFORE
	// it touches Target*.
}

// ============================================================================
//  ReleasePreviousDestinationSpot — 釋放上一個目的地佔用的停車格並清空 Target*
//  Release any spot held by the previous destination and clear Target* state.
// ============================================================================
void URoadPathFollowerComponent::ReleasePreviousDestinationSpot()
{
	if (DestinationType == EDestinationType::ParkingLot
		&& TargetParkingLot.IsValid()
		&& TargetParkingSpotIndex != INDEX_NONE)
	{
		TargetParkingLot->ReleaseSpot(TargetParkingSpotIndex);
	}

	DestinationType = EDestinationType::None;
	TargetParkingLot = nullptr;
	TargetParkingSpotIndex = INDEX_NONE;
}

// ============================================================================
//  FindForwardStartNode — 找位於車前方的最近 graph node 作為 A* 起點
//  Find nearest graph node in the forward cone for use as A* start.
// ============================================================================
int32 URoadPathFollowerComponent::FindForwardStartNode(const FVector& CarPos, const FVector& CarFwd) const
{
	UWorld* World = GetWorld();
	if (!World) return INDEX_NONE;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub) return INDEX_NONE;

	const TArray<FRoadGraphNode>& Nodes = Sub->GetGraphNodes();
	const FVector CarFwd2D = FVector(CarFwd.X, CarFwd.Y, 0.0f).GetSafeNormal();

	// 第一輪：只考慮在前方錐內 (~70° cone, dot >= 0.34) 的 node
	// First pass: only consider nodes inside ~70° forward cone
	int32 BestId = INDEX_NONE;
	float BestDistSq = TNumericLimits<float>::Max();

	for (const FRoadGraphNode& N : Nodes)
	{
		const FVector Delta = N.WorldLocation - CarPos;
		const FVector Delta2D = FVector(Delta.X, Delta.Y, 0.0f);
		const float DistSq = Delta2D.SizeSquared();
		if (DistSq < 1.0f) continue;  // 自己/重疊跳過 / skip self/overlapping

		const float Dot = FVector::DotProduct(Delta2D.GetSafeNormal(), CarFwd2D);
		if (Dot < 0.34f) continue;  // 不在前方錐 / not in forward cone

		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestId = N.NodeId;
		}
	}

	// No forward node → fallback to absolute nearest (rare, e.g. parked in dead-end)
	if (BestId == INDEX_NONE)
	{
		BestId = Sub->FindNearestGraphNode(CarPos);
	}

	return BestId;
}

// ============================================================================
//  FindStartBoundEdge — 找車最近的 edge 並投影，決定 ExitNode
//  Find nearest edge to car, project, determine forward exit node.
// ============================================================================
bool URoadPathFollowerComponent::FindStartBoundEdge(
	const FVector& CarPos,
	const FVector& CarFwd,
	const FRoadGraphEdge*& OutEdge,
	float& OutProjDistAbs,
	bool& OutForward,
	int32& OutExitNodeId) const
{
	OutEdge = nullptr;
	OutProjDistAbs = 0.0f;
	OutForward = true;
	OutExitNodeId = INDEX_NONE;

	UWorld* World = GetWorld();
	if (!World) return false;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub) return false;

	const TArray<FRoadGraphEdge>& Edges = Sub->GetGraphEdges();
	const FVector CarFwd2D = FVector(CarFwd.X, CarFwd.Y, 0.0f).GetSafeNormal();

	const FRoadGraphEdge* BestEdge = nullptr;
	float BestDistSq = TNumericLimits<float>::Max();
	float BestProjDistAbs = 0.0f;
	FVector BestProjPt = FVector::ZeroVector;
	FVector BestSplineDir = FVector::ForwardVector;

	for (const FRoadGraphEdge& E : Edges)
	{
		if (!E.InputSpline) continue;

		const float InputKey = E.InputSpline->FindInputKeyClosestToWorldLocation(CarPos);
		const float RawDist = E.InputSpline->GetDistanceAlongSplineAtSplineInputKey(InputKey);

		const float LoDist = FMath::Min(E.StartDistanceOnSpline, E.EndDistanceOnSpline);
		const float HiDist = FMath::Max(E.StartDistanceOnSpline, E.EndDistanceOnSpline);
		const float ClampedDist = FMath::Clamp(RawDist, LoDist, HiDist);

		const FVector ProjPt = E.InputSpline->GetLocationAtDistanceAlongSpline(
			ClampedDist, ESplineCoordinateSpace::World);

		const float DistSq = FVector::DistSquared2D(CarPos, ProjPt);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestEdge = &E;
			BestProjDistAbs = ClampedDist;
			BestProjPt = ProjPt;
			BestSplineDir = E.InputSpline->GetDirectionAtDistanceAlongSpline(
				ClampedDist, ESplineCoordinateSpace::World);
		}
	}

	if (!BestEdge)
	{
		return false;
	}

	// Car forward vs spline direction determines travel direction along the edge.
	const float DirDot = FVector::DotProduct(
		CarFwd2D, BestSplineDir.GetSafeNormal2D());
	OutForward = (DirDot >= 0.0f);
	OutExitNodeId = OutForward ? BestEdge->EndNodeId : BestEdge->StartNodeId;
	OutEdge = BestEdge;
	OutProjDistAbs = BestProjDistAbs;
	return true;
}

// ============================================================================
//  PrependStartBoundEdgeSegment — 把部分 BoundEdge 插在 PathSegments 最前面
//  Prepend a partial BoundEdge segment to the front of PathSegments.
// ============================================================================
void URoadPathFollowerComponent::PrependStartBoundEdgeSegment(
	const FRoadGraphEdge& BoundEdge, bool bForward, float ProjDistAbs, bool bTriangleExitOffset)
{
	if (!BoundEdge.InputSpline)
	{
		UE_LOG(LogTemp, Error, TEXT("[PREPEND-BOUND] BoundEdge has no spline!"));
		return;
	}

	// Build new segment: from ProjDistAbs to exit end (direction-aware)
	FPathSegmentInternal Seg;
	Seg.Spline = BoundEdge.InputSpline;
	Seg.bTwoRoads = BoundEdge.bTwoRoads;
	Seg.TwoRoadsGapM = BoundEdge.TwoRoadsGapM;
	Seg.RoadType = BoundEdge.RoadType;
	Seg.DrivingRule = URoadRuleLibrary::GetDrivingRuleFromRoadType(BoundEdge.RoadType);
	Seg.RoadWidthMultiplier = BoundEdge.RoadWidthMultiplier;
	Seg.AdditionalWidthM = BoundEdge.AdditionalWidthM;
	Seg.GuardrailSideOffsetCm = BoundEdge.GuardrailSideOffsetCm;
	Seg.AutoLaneWidthCm = BoundEdge.AutoLaneWidthCm;
	Seg.AutoMedianCm = BoundEdge.AutoMedianCm;
	Seg.EdgeId = BoundEdge.EdgeId;

	if (bForward)
	{
		Seg.StartDist = ProjDistAbs;
		Seg.EndDist = BoundEdge.EndDistanceOnSpline;
		Seg.Direction = 1.0f;
		Seg.EndNodeId = BoundEdge.EndNodeId;
	}
	else
	{
		Seg.StartDist = ProjDistAbs;
		Seg.EndDist = BoundEdge.StartDistanceOnSpline;
		Seg.Direction = -1.0f;
		Seg.EndNodeId = BoundEdge.StartNodeId;
	}

	// 如果是從停車場出發（偏移 n 後從斜邊 2 尾端接回道路），
	// 把 StartDist 往前推 ParkingTriangleBaseLength，讓 spline 跟隨從 ProjPt+n 開始。
	// If we're exiting from a parking lot (bTriangleExitOffset), shift StartDist forward by n so the
	// spline segment begins at (ProjPt + n along edge) — the far end of exit hypotenuse 2.
	if (bTriangleExitOffset)
	{
		const float BoundMinDist = FMath::Min(BoundEdge.StartDistanceOnSpline, BoundEdge.EndDistanceOnSpline);
		const float BoundMaxDist = FMath::Max(BoundEdge.StartDistanceOnSpline, BoundEdge.EndDistanceOnSpline);
		const float Shifted = Seg.StartDist + Seg.Direction * ParkingTriangleBaseLength;
		Seg.StartDist = FMath::Clamp(Shifted, BoundMinDist, BoundMaxDist);
	}

	// If A*'s first segment is the same edge as BoundEdge, drop A*'s first segment to avoid
	// the car traversing the same edge twice.
	if (PathSegments.Num() > 0
		&& PathSegments[0].EdgeId == BoundEdge.EdgeId)
	{
		PathSegments.RemoveAt(0);
	}

	// Continuity check omitted (PathSegments doesn't store StartNodeId); rely on caller to
	// pass consistent A* target.
	PathSegments.Insert(Seg, 0);

	// Recompute TurnAtEnd between the prepended segment and the next segment
	if (PathSegments.Num() >= 2)
	{
		ComputeTurnAtJunction(
			PathSegments[0], PathSegments[1],
			PathSegments[0].TurnAtEnd, PathSegments[0].bUTurnAtEnd);
	}
	else
	{
		PathSegments[0].TurnAtEnd = ETurnSignal::None;
		PathSegments[0].bUTurnAtEnd = false;
	}

}

// ============================================================================
//  Shared internal: run A*, build segments, initialize state.
// ============================================================================
void URoadPathFollowerComponent::StartNavigationInternal(int32 FromNodeId, int32 ToNodeId)
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: RoadNetworkSubsystem not found"));
		return;
	}

	//If already following, reset first
	if (bIsFollowing)
	{
		ResetFollowingState();
	}

	// ---- Find nearest edge; use it as the start bound edge ----
	AActor* OwnerActor = GetOwner();
	const FVector CarPos = OwnerActor ? OwnerActor->GetActorLocation() : FVector::ZeroVector;
	const FVector CarFwd = OwnerActor ? OwnerActor->GetActorForwardVector() : FVector::ForwardVector;

	const FRoadGraphEdge* StartBoundEdge = nullptr;
	float StartBoundProjDistAbs = 0.0f;
	bool bStartBoundForward = true;
	int32 StartBoundExitNode = INDEX_NONE;
	bool bHaveStartBoundEdge = false;

	// ---- Parking lot departure: if car is near a parking lot spot, prefer that lot's BoundEdge ----
	// FindStartBoundEdge searches ALL edges globally and may pick a wrong nearby edge
	// (e.g., a coastal road passing close to the parking lot), causing the departure path
	// to go completely off-road. Using the parking lot's BoundEdge guarantees the correct start edge.
	bool bTriangleExit = false;
	AParkingLotActor* DepartureLot = nullptr;
	{
		const float SpotSearchRadiusSq = FMath::Square(ParkingTriangleBaseLength * 2.0f);
		float BestSpotDistSq = SpotSearchRadiusSq;

		for (TActorIterator<AParkingLotActor> It(World); It; ++It)
		{
			AParkingLotActor* Lot = *It;
			if (!Lot || Lot->SpotCount <= 0 || Lot->BoundEdgeId == INDEX_NONE) continue;

			// Check all spots, not just anchor (car could be parked at any spot)
			for (int32 SpotIdx = 0; SpotIdx < Lot->SpotCount; ++SpotIdx)
			{
				const FVector SpotPos = Lot->GetSpotWorldPosition(SpotIdx);
				const float DistSq = FVector::DistSquared2D(CarPos, SpotPos);
				if (DistSq < BestSpotDistSq)
				{
					BestSpotDistSq = DistSq;
					DepartureLot = Lot;
				}
			}
		}

		if (DepartureLot)
		{
			bTriangleExit = true;

			// Use the parking lot's BoundEdge as the start edge
			const FRoadGraphEdge* LotBoundEdge = Sub->GetGraphEdgeById(DepartureLot->BoundEdgeId);
			if (LotBoundEdge && LotBoundEdge->InputSpline)
			{
				StartBoundEdge = LotBoundEdge;

				// Use the lot's BoundProjectionPoint for projection distance, NOT car position.
				// Car is at a parking spot (far from road); projecting car onto spline may land on
				// wrong sub-edge region. BoundProjectionPoint was computed during BuildRoadCache
				// as the exact projection of anchor onto the edge.
				const FVector& ProjPt = DepartureLot->BoundProjectionPoint;
				const float ProjKey = LotBoundEdge->InputSpline->FindInputKeyClosestToWorldLocation(ProjPt);
				const float ProjRawDist = LotBoundEdge->InputSpline->GetDistanceAlongSplineAtSplineInputKey(ProjKey);
				const float LoDist = FMath::Min(LotBoundEdge->StartDistanceOnSpline, LotBoundEdge->EndDistanceOnSpline);
				const float HiDist = FMath::Max(LotBoundEdge->StartDistanceOnSpline, LotBoundEdge->EndDistanceOnSpline);
				StartBoundProjDistAbs = FMath::Clamp(ProjRawDist, LoDist, HiDist);

				// Use parking lot ANCHOR forward (not car forward) to determine travel direction.
				// When parked, car faces perpendicular to road (into the spot), making dot ≈ 0
				// and the direction pick essentially random. Anchor forward is the same reference
				// NavigateToParkingLot uses for arrival, ensuring consistent direction.
				const FVector AnchorFwd = DepartureLot->GetAnchorArrowForward();
				const FVector AnchorFwd2D = FVector(AnchorFwd.X, AnchorFwd.Y, 0.0f).GetSafeNormal();
				const FVector SplineDir = LotBoundEdge->InputSpline->GetDirectionAtDistanceAlongSpline(
					StartBoundProjDistAbs, ESplineCoordinateSpace::World);
				const float AnchorDot = FVector::DotProduct(AnchorFwd2D, SplineDir.GetSafeNormal2D());

				// Arrival logic in NavigateToParkingLot:
				//   bForwardDrive = (AnchorDot >= 0)
				//   EntryNode = bForwardDrive ? Start : End
				// Departure: car drives in same direction, away from entry side,
				//   so ExitNode = bForwardDrive ? End : Start
				bStartBoundForward = (AnchorDot >= 0.0f);
				StartBoundExitNode = bStartBoundForward ? LotBoundEdge->EndNodeId : LotBoundEdge->StartNodeId;
				bHaveStartBoundEdge = true;
			}
		}
	}

	// If no parking lot departure detected, fall back to global nearest edge search
	if (!bHaveStartBoundEdge && OwnerActor)
	{
		bHaveStartBoundEdge = FindStartBoundEdge(
			CarPos, CarFwd,
			StartBoundEdge, StartBoundProjDistAbs, bStartBoundForward, StartBoundExitNode);
	}

	// If start bound edge found, A* starts from its exit node (caller's FromNodeId becomes fallback)
	const int32 AStarFromNode = (bHaveStartBoundEdge && StartBoundExitNode != INDEX_NONE)
		? StartBoundExitNode : FromNodeId;

	FRoadGraphPath AStarPath;
	if (AStarFromNode == ToNodeId)
	{
		AStarPath.bPathFound = true;
		AStarPath.NodePath.Add(ToNodeId);
		AStarPath.TotalCost = 0.0f;
	}
	else
	{
		AStarPath = Sub->FindPathAStar(AStarFromNode, ToNodeId);
	}

	if (!AStarPath.bPathFound)
	{
		OnPathComplete.Broadcast(false);
		return;
	}

	BuildPathSegments(AStarPath);

	// ---- Prepend the partial start bound edge as the first segment ----
	if (bHaveStartBoundEdge && StartBoundEdge)
	{
		PrependStartBoundEdgeSegment(*StartBoundEdge, bStartBoundForward, StartBoundProjDistAbs, bTriangleExit);
	}

	if (PathSegments.Num() == 0)
	{
		OnPathComplete.Broadcast(false);
		return;
	}

	//Initialize all state
	CurrentSegmentIndex = 0;
	ReferenceDistance = PathSegments[0].StartDist;
	CurrentSpeed = 0.0f;  //start from standstill
	TargetLaneIndex = CurrentLaneIndex;
	CurrentLateralOffset = ComputeTargetLaneOffset(CurrentLaneIndex);
	bIsChangingLane = false;
	bOnJunctionCurve = false;
	bJustExitedJunctionCurve = false;
	JCurveProgress = 0.0f;
	CurrentTurnSignal = ETurnSignal::None;
	NavState = ENavState::Driving;
	bIsFollowing = true;

	// ---- Departure curve: project car pos onto first segment's spline (nearest point), then Hermite merge ----
	if (OwnerActor && PathSegments.Num() > 0)
	{
		FPathSegmentInternal& FirstSeg = PathSegments[0];
		const int32 OutermostLane = FMath::Max(0, FirstSeg.DrivingRule.ForwardLaneCount - 1);
		const float OuterLaneOffset = ComputeLaneOffsetCm(
			FirstSeg.bTwoRoads, FirstSeg.TwoRoadsGapM, FirstSeg.RoadType,
			OutermostLane,
			FirstSeg.AutoLaneWidthCm, FirstSeg.AutoMedianCm,
			TwoRoadsMedianAdjustCm, TwoRoadsLaneWidthAdjustCm,
			SharedRoadMedianAdjustCm, SharedRoadLaneWidthAdjustCm);

		// Project car pos onto first segment's spline to find nearest distance
		float ProjDistAbs = FirstSeg.StartDist;
		if (FirstSeg.Spline)
		{
			const float InputKey = FirstSeg.Spline->FindInputKeyClosestToWorldLocation(CarPos);
			const float RawDist = FirstSeg.Spline->GetDistanceAlongSplineAtSplineInputKey(InputKey);

			// Clamp within segment distance range (direction-aware)
			const float LoDist = FMath::Min(FirstSeg.StartDist, FirstSeg.EndDist);
			const float HiDist = FMath::Max(FirstSeg.StartDist, FirstSeg.EndDist);
			ProjDistAbs = FMath::Clamp(RawDist, LoDist, HiDist);
		}

		// Sample projection point at outermost-lane offset — this is the nearest point on the edge
		FVector RoadStartPos, RoadStartDir, RoadStartRight;
		SampleSplineAtDist(FirstSeg, ProjDistAbs, OuterLaneOffset,
			RoadStartPos, RoadStartDir, RoadStartRight);

		const float DistToRoad = FVector::Dist2D(CarPos, RoadStartPos);

		// Set ReferenceDistance to the projected distance (not the segment start)
		ReferenceDistance = ProjDistAbs;

		if (DistToRoad > DepartureCurveTriggerDistance)
		{
			DepCurveP0 = CarPos;
			DepCurveT0 = OwnerActor->GetActorForwardVector() * DistToRoad * DepartureCurveStartTangentScale;
			DepCurveP1 = RoadStartPos;
			DepCurveT1 = RoadStartDir.GetSafeNormal() * DistToRoad * DepartureCurveTangentScale;
			DepCurveLength = FMath::Max(DistToRoad * DepartureCurveLengthMultiplier, DepartureCurveMinLength);
			DepCurveProgress = 0.0f;
			bOnDepartureCurve = true;

			// After departure, merge into outermost lane
			TargetLaneIndex = OutermostLane;
			CurrentLaneIndex = OutermostLane;
			CurrentLateralOffset = OuterLaneOffset;
		}
	}
}

// ============================================================================
//  Auto-navigate to a random parking lot.
// ============================================================================
void URoadPathFollowerComponent::StartFollowing()
{
	UWorld* World = GetWorld();
	if (!World) return;

	//Find parking lots with free spots
	TArray<AParkingLotActor*> Lots;
	for (TActorIterator<AParkingLotActor> It(World); It; ++It)
	{
		if (*It && (*It)->FindAvailableSpot() != INDEX_NONE)
			Lots.Add(*It);
	}

	if (Lots.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("[START] No parking destinations found — cannot auto-start"));
		return;
	}

	const int32 Pick = FMath::RandRange(0, Lots.Num() - 1);
	AParkingLotActor* Lot = Lots[Pick];
	NavigateToParkingLot(Lot);
}

// ============================================================================
//  Called when a car is stuck repeatedly; picks a different parking lot
//  (excluding the current one) so the A* path diverges from the congested zone.
// ============================================================================
void URoadPathFollowerComponent::SwitchToNewRandomDestination()
{
	UWorld* World = GetWorld();
	if (!World) return;

	AActor* Owner = GetOwner();
	const FString OwnerName = Owner ? Owner->GetName() : TEXT("?");

	AParkingLotActor* CurrentLot = TargetParkingLot.Get();

	// Collect all parking lots with free spots, excluding the current target
	TArray<AParkingLotActor*> Candidates;
	for (TActorIterator<AParkingLotActor> It(World); It; ++It)
	{
		AParkingLotActor* Lot = *It;
		if (!Lot) continue;
		if (Lot == CurrentLot) continue;
		if (Lot->FindAvailableSpot() == INDEX_NONE) continue;
		Candidates.Add(Lot);
	}

	if (Candidates.Num() == 0)
	{
		// No alternative available → fall back to re-navigating to the same dest.
		UE_LOG(LogTemp, Warning,
			TEXT("[STUCK-SWITCH] %s: No alternative parking lot found — falling back to current dest"),
			*OwnerName);
		if (DestinationNodeId != INDEX_NONE)
		{
			NavigateToNode(DestinationNodeId);
		}
		return;
	}

	const int32 Pick = FMath::RandRange(0, Candidates.Num() - 1);
	AParkingLotActor* NewLot = Candidates[Pick];

	UE_LOG(LogTemp, Warning,
		TEXT("[STUCK-SWITCH] %s: Switching destination from '%s' → '%s' (%d candidates)"),
		*OwnerName,
		CurrentLot ? *CurrentLot->ParkingLotName : TEXT("<none>"),
		*NewLot->ParkingLotName,
		Candidates.Num());

	//Release currently reserved spot if any
	if (CurrentLot && TargetParkingSpotIndex != INDEX_NONE)
	{
		CurrentLot->ReleaseSpot(TargetParkingSpotIndex);
		TargetParkingSpotIndex = INDEX_NONE;
	}

	NavigateToParkingLot(NewLot);
}

// ============================================================================
//  ZeroDistReNavigateCallback — fires after ZeroDistTeleportGraceDuration
//  The car was teleported to its parking spot and has been sitting there for
//  the grace period.  Now pick a completely new random destination so the car
//  never returns to the same (potentially congested) spot it just escaped from.
// ============================================================================
void URoadPathFollowerComponent::ZeroDistReNavigateCallback()
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	UE_LOG(LogTemp, Warning,
		TEXT("[STUCK] %s: ZERO-DIST grace expired → switching to new random destination"),
		*Owner->GetName());
	LogEvent(TEXT("ZERO-DIST grace done → SwitchToNewRandomDestination"));

	// Arm the in-tick grace timer so that if the new start location also has a
	// car nearby, the ZeroDistStuckTimer won't fire immediately again.
	ZeroDistTeleportGraceTimer = ZeroDistTeleportGraceDuration;
	ZeroDistStuckTimer  = 0.0f;
	ZeroDistClearTimer  = 0.0f;

	// Pick a different parking lot and start navigating there.
	// SwitchToNewRandomDestination releases the current spot, picks a new lot,
	// and calls NavigateToParkingLot (which sets bIsFollowing = true).
	//SwitchToNewRandomDestination();
}

// ============================================================================
//  SetDepartureContext
//  Must be called at spawn BEFORE the first NavigateTo*.
// ============================================================================
void URoadPathFollowerComponent::SetDepartureContext(AParkingLotActor* SourceLot, int32 SpotIdx)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	SourceParkingLot = SourceLot;
	SourceSpotIndex = SpotIdx;
	SourceSpawnLocation = Owner->GetActorLocation();
	bHasDepartedFromSource = false;
	DepartureWaitAccum = 0.0f;

	// Also mark the source lot's bookkeeping so other cars see this spot occupied
	if (SourceLot && SpotIdx != INDEX_NONE)
	{
		SourceLot->OccupySpot(SpotIdx, Owner);
	}
}

// ============================================================================
//  IsPerformingManeuver — in a junction / U-turn / parking / departure curve?
// ============================================================================
bool URoadPathFollowerComponent::IsPerformingManeuver() const
{
	return bOnJunctionCurve || bOnParkingCurve || bOnDepartureCurve;
}

// ============================================================================
//  IsChainHeadBlockedBySignal — walk ObstacleActor links forward; return true
//  if any car in the chain (including this one) is directly blocked by a
//  TrafficSignal.  Used to suppress teleport when the whole queue is just
//  waiting at a red light.
// ============================================================================
bool URoadPathFollowerComponent::IsChainHeadBlockedBySignal() const
{
	const URoadPathFollowerComponent* Current = this;
	for (int32 Depth = 0; Depth < MaxChainWalkDepth; ++Depth)
	{
		const EFrontObstacleType Type = Current->ClassifyFrontObstacle();
		if (Type == EFrontObstacleType::TrafficSignal)
			return true;
		if (Type != EFrontObstacleType::Vehicle)
			return false;

		const AActor* FrontActor = Current->ObstacleActor.Get();
		if (!FrontActor) return false;
		const URoadPathFollowerComponent* FrontPF =
			FrontActor->FindComponentByClass<URoadPathFollowerComponent>();
		if (!FrontPF) return false;
		Current = FrontPF;
	}
	return false;
}

// ============================================================================
//  IsDepartureLaneClear — scan backward along the source edge spline for cars
//  approaching from behind the merge point. Only vehicles count; traffic signal
//  colliders are ignored (we don't want to misread an opposing signal as traffic).
// ============================================================================
bool URoadPathFollowerComponent::IsDepartureLaneClear() const
{
	if (!SourceParkingLot.IsValid()) return true;
	AParkingLotActor* Lot = SourceParkingLot.Get();
	if (!Lot) return true;

	UWorld* World = GetWorld();
	if (!World) return true;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub) return true;

	const FRoadGraphEdge* Edge = Sub->GetGraphEdgeById(Lot->BoundEdgeId);
	if (!Edge || !Edge->InputSpline) return true;

	USplineComponent* S = Edge->InputSpline;
	const FVector& ProjPt = Lot->BoundProjectionPoint;
	const float ProjKey = S->FindInputKeyClosestToWorldLocation(ProjPt);
	const float ProjDist = S->GetDistanceAlongSplineAtSplineInputKey(ProjKey);

	// Determine the spot's facing relative to spline — "backward" on our lane
	// is the direction cars come FROM when they'd hit us merging in.
	const FVector SpotFwd = Lot->GetSpotWorldForward(SourceSpotIndex);
	const FVector SpotFwd2D = FVector(SpotFwd.X, SpotFwd.Y, 0).GetSafeNormal();
	const FVector SplineDir = S->GetDirectionAtDistanceAlongSpline(
		ProjDist, ESplineCoordinateSpace::World).GetSafeNormal2D();
	const bool bForwardAlongSpline = (FVector::DotProduct(SpotFwd2D, SplineDir) >= 0.0f);
	const float BackStep = bForwardAlongSpline ? -1.0f : 1.0f;  // backward rel. departure
	const float FwdStep  = -BackStep;                            // forward rel. departure
	const float SplineLen = S->GetSplineLength();

	FCollisionQueryParams Params;
	if (AActor* Owner = GetOwner()) Params.AddIgnoredActor(Owner);

	// Scan both directions on our lane:
	//  * BEHIND the merge point for approaching cars (cars would hit us)
	//  * AHEAD of the merge point for queued / stopped cars (we'd hit them)
	// Both windows use DepartureLaneCheckDistance; ahead uses half that so we
	// don't refuse to depart just because downstream traffic is normal-spaced.
	auto ScanDirection = [&](float Direction, float Distance) -> bool
	{
		const int32 NumSamples = FMath::Max(2, FMath::CeilToInt(Distance / 250.0f));
		for (int32 i = 1; i <= NumSamples; ++i)
		{
			const float Offset = (Distance * i) / NumSamples;
			const float SampleDist = FMath::Clamp(ProjDist + Direction * Offset, 0.0f, SplineLen);
			const FVector P = S->GetLocationAtDistanceAlongSpline(
				SampleDist, ESplineCoordinateSpace::World)
				+ FVector(0, 0, ObstacleTraceZOffset);

			FHitResult Hit;
			const bool bHit = World->SweepSingleByChannel(
				Hit, P, P + FVector(0, 0, 1), FQuat::Identity,
				ECC_ObstacleDetect,
				FCollisionShape::MakeSphere(DepartureLaneTraceRadius),
				Params);

			if (bHit)
			{
				AActor* A = Hit.GetActor();
				if (A && A->FindComponentByClass<URoadPathFollowerComponent>())
				{
					return false;
				}
			}
		}
		return true;
	};

	// Behind: full window for approaching traffic.
	if (!ScanDirection(BackStep, DepartureLaneCheckDistance)) return false;
	// Ahead: shorter window — just enough to avoid merging into the back of a queue.
	if (!ScanDirection(FwdStep, DepartureLaneCheckDistance * 0.5f)) return false;
	return true;
}

// ============================================================================
//  Before a LEFT-turn junction curve, scan the oncoming lane of NextSeg looking
//  for straight-through vehicles. "Oncoming" = along NextSeg's spline in the
//  REVERSE of our driving direction, offset to the lane that crosses the same
//  junction space we'd sweep through.
// ============================================================================
bool URoadPathFollowerComponent::IsLeftTurnOncomingClear(
	const FPathSegmentInternal& NextSeg,
	AActor*& OutBlockingActor,
	float& OutDistance) const
{
	OutBlockingActor = nullptr;
	OutDistance = -1.0f;

	if (LeftTurnOncomingScanDistance <= 0.0f) return true;  // disabled
	if (!NextSeg.Spline) return true;

	UWorld* World = GetWorld();
	if (!World) return true;

	AActor* Owner = GetOwner();
	if (!Owner) return true;

	USplineComponent* S = NextSeg.Spline;
	const float SplineLen = S->GetSplineLength();

	// Oncoming traffic travels in the OPPOSITE direction along the same spline.
	// We sample starting at NextSeg.StartDist (junction entry point) and walk
	// AGAINST our driving direction, because cars coming head-on are approaching
	// us from "further along the spline in the other direction".
	//
	// NextSeg.Direction is our travel direction (±1). Oncoming step = -Direction.
	// We also offset laterally into the opposing lane (OPPOSITE side of spline).
	const float OncomingStep = -NextSeg.Direction;
	const float StartDist = NextSeg.StartDist;

	// Opposing lane offset: mirror our next-seg lane across the spline centerline.
	// Use innermost opposing lane (lane 0 on the other side) — that's the one a
	// straight-through oncoming car would be in just before the junction.
	const float OpposingLaneOffset = -ComputeLaneOffsetCm(
		NextSeg.bTwoRoads, NextSeg.TwoRoadsGapM, NextSeg.RoadType,
		0,
		NextSeg.AutoLaneWidthCm, NextSeg.AutoMedianCm,
		TwoRoadsMedianAdjustCm, TwoRoadsLaneWidthAdjustCm,
		SharedRoadMedianAdjustCm, SharedRoadLaneWidthAdjustCm);

	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Owner);

	const int32 NumSamples = FMath::Max(3, FMath::CeilToInt(LeftTurnOncomingScanDistance / 250.0f));
	for (int32 i = 1; i <= NumSamples; ++i)
	{
		const float Offset = (LeftTurnOncomingScanDistance * i) / NumSamples;
		const float SampleAbs = FMath::Clamp(StartDist + OncomingStep * Offset, 0.0f, SplineLen);

		// Lateral offset along the opposing lane
		FVector SampleCenter, SampleDir, SampleRight;
		SampleSplineAtDist(NextSeg, SampleAbs, OpposingLaneOffset,
			SampleCenter, SampleDir, SampleRight);

		const FVector P = SampleCenter + FVector(0, 0, ObstacleTraceZOffset);

		FHitResult Hit;
		const bool bHit = World->SweepSingleByChannel(
			Hit, P, P + FVector(0, 0, 1), FQuat::Identity,
			ECC_ObstacleDetect,
			FCollisionShape::MakeSphere(LeftTurnOncomingScanRadius),
			Params);

		if (bHit)
		{
			AActor* A = Hit.GetActor();
			if (A && A != Owner)
			{
				URoadPathFollowerComponent* OtherPF =
					A->FindComponentByClass<URoadPathFollowerComponent>();
				if (OtherPF)
				{
					// Only yield to moving / non-reversing oncoming cars. If the
					// oncoming car is itself stopped & stuck, we'll deadlock
					// head-on; the LeftTurnYieldMaxSec timeout breaks that.
					OutBlockingActor = A;
					OutDistance = Offset;
					return false;
				}
			}
		}
	}
	return true;
}

// ============================================================================
//  All spots with index < SourceSpotIndex must be either empty (occupant drove
//  off) or occupied by a car that itself has already departed from its spot.
// ============================================================================
bool URoadPathFollowerComponent::IsSourceLotReadyForDeparture() const
{
	if (!SourceParkingLot.IsValid() || SourceSpotIndex == INDEX_NONE) return true;
	if (SourceSpotIndex == 0) return true;

	AParkingLotActor* Lot = SourceParkingLot.Get();
	for (int32 i = 0; i < SourceSpotIndex; ++i)
	{
		AActor* Occupant = Lot->GetSpotOccupant(i);
		if (!Occupant) continue;

		URoadPathFollowerComponent* OtherPF =
			Occupant->FindComponentByClass<URoadPathFollowerComponent>();
		if (OtherPF && OtherPF->HasDepartedFromSource()) continue;

		return false;
	}
	return true;
}

void URoadPathFollowerComponent::RequestDepartToParkingLot(AParkingLotActor* DestLot, int32 DestSpotIdx)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	if (!SourceParkingLot.IsValid() || SourceSpotIndex == INDEX_NONE)
	{
		NavigateToParkingLot(DestLot, DestSpotIdx);
		return;
	}

	PendingDestLot = DestLot;
	PendingDestSpotIndex = DestSpotIdx;
	DepartureWaitAccum = 0.0f;

	TryDepartPoll();
}

void URoadPathFollowerComponent::TryDepartPoll()
{
	UWorld* World = GetWorld();
	AActor* Owner = GetOwner();
	if (!World || !Owner) return;

	if (!PendingDestLot.IsValid())
	{
		World->GetTimerManager().ClearTimer(DepartureCheckTimerHandle);
		return;
	}

	const bool bOrderReady = IsSourceLotReadyForDeparture();
	const bool bLaneClear = IsDepartureLaneClear();
	const bool bReady = bOrderReady && bLaneClear;
	const bool bTimedOut = (DepartureWaitAccum >= DepartureMaxWaitTime);

	if (bReady || bTimedOut)
	{
		World->GetTimerManager().ClearTimer(DepartureCheckTimerHandle);

		LogEvent(FString::Printf(TEXT("DEPART GO (order=%s lane=%s waited=%.1fs)"),
			bOrderReady ? TEXT("Y") : TEXT("N"),
			bLaneClear ? TEXT("Y") : TEXT("N"),
			DepartureWaitAccum));

		AParkingLotActor* Dest = PendingDestLot.Get();
		const int32 DestSpot = PendingDestSpotIndex;
		PendingDestLot = nullptr;
		PendingDestSpotIndex = INDEX_NONE;

		NavigateToParkingLot(Dest, DestSpot);
		return;
	}

	DepartureWaitAccum += DepartureCheckInterval;

	// Log the wait reason every ~2s so it's visible in the M-panel.
	if (FMath::Fmod(DepartureWaitAccum, 2.0f) < DepartureCheckInterval)
	{
		LogEvent(FString::Printf(TEXT("DEPART WAIT order=%s lane=%s (%.1fs)"),
			bOrderReady ? TEXT("Y") : TEXT("N"),
			bLaneClear ? TEXT("Y") : TEXT("N"),
			DepartureWaitAccum));
	}

	World->GetTimerManager().SetTimer(
		DepartureCheckTimerHandle,
		FTimerDelegate::CreateUObject(this, &URoadPathFollowerComponent::TryDepartPoll),
		DepartureCheckInterval,
		false);
}

// ============================================================================
//  Called once per tick while bHasDepartedFromSource is false.
// ============================================================================
void URoadPathFollowerComponent::UpdateDepartureProgress()
{
	if (bHasDepartedFromSource) return;
	if (SourceSpotIndex == INDEX_NONE) return;

	AActor* Owner = GetOwner();
	if (!Owner) return;

	const float DistSq = FVector::DistSquared2D(Owner->GetActorLocation(), SourceSpawnLocation);
	const float ThreshSq = DepartureClearDistance * DepartureClearDistance;
	if (DistSq < ThreshSq) return;

	bHasDepartedFromSource = true;

	AParkingLotActor* Lot = SourceParkingLot.Get();
	if (Lot && SourceSpotIndex != INDEX_NONE)
	{
		if (Lot->GetSpotOccupant(SourceSpotIndex) == Owner)
		{
			Lot->ReleaseSpot(SourceSpotIndex);
		}
	}
}

// ============================================================================
//  Navigate from car's current position to a specific graph node.
// ============================================================================
void URoadPathFollowerComponent::NavigateToNode(int32 TargetNodeId)
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: RoadNetworkSubsystem not found"));
		return;
	}

	AActor* Owner = GetOwner();
	if (!Owner) return;

	//Store destination info
	DestinationNodeId = TargetNodeId;
	const TArray<FRoadGraphNode>& Nodes = Sub->GetGraphNodes();
	if (TargetNodeId >= 0 && TargetNodeId < Nodes.Num())
	{
		DestinationWorldLocation = Nodes[TargetNodeId].WorldLocation;
	}

	// If currently driving → mid-drive reroute (no U-turn)
	if (bIsFollowing && NavState == ENavState::Driving
		&& CurrentSegmentIndex < PathSegments.Num())
	{
		RerouteToNode(TargetNodeId);
		return;
	}

	// ====================================================================
	// If car is near a parking lot → bootstrap onto its BoundEdge, then
	// call RerouteToNode — the exact same code path as a manual map click.
	// ====================================================================
	const FVector CarPos = Owner->GetActorLocation();
	const FVector CarFwd = Owner->GetActorForwardVector();

	// ---- Find nearby parking lot ----
	AParkingLotActor* NearLot = nullptr;
	{
		const float SearchRadiusSq = FMath::Square(ParkingTriangleBaseLength * 2.0f);
		float BestDistSq = SearchRadiusSq;
		for (TActorIterator<AParkingLotActor> It(World); It; ++It)
		{
			AParkingLotActor* Lot = *It;
			if (!Lot || Lot->SpotCount <= 0 || Lot->BoundEdgeId == INDEX_NONE) continue;
			for (int32 SpotIdx = 0; SpotIdx < Lot->SpotCount; ++SpotIdx)
			{
				const float DistSq = FVector::DistSquared2D(CarPos, Lot->GetSpotWorldPosition(SpotIdx));
				if (DistSq < BestDistSq)
				{
					BestDistSq = DistSq;
					NearLot = Lot;
				}
			}
		}
	}

	if (NearLot)
	{
		const FRoadGraphEdge* LotEdge = Sub->GetGraphEdgeById(NearLot->BoundEdgeId);
		if (LotEdge && LotEdge->InputSpline)
		{
			// ---- Use anchor forward for direction (same as NavigateToParkingLot arrival) ----
			const FVector AnchorFwd2D = FVector(
				NearLot->GetAnchorArrowForward().X,
				NearLot->GetAnchorArrowForward().Y, 0.0f).GetSafeNormal();

			const FVector& ProjPt = NearLot->BoundProjectionPoint;
			const float ProjKey = LotEdge->InputSpline->FindInputKeyClosestToWorldLocation(ProjPt);
			float ProjDist = LotEdge->InputSpline->GetDistanceAlongSplineAtSplineInputKey(ProjKey);
			const float Lo = FMath::Min(LotEdge->StartDistanceOnSpline, LotEdge->EndDistanceOnSpline);
			const float Hi = FMath::Max(LotEdge->StartDistanceOnSpline, LotEdge->EndDistanceOnSpline);
			ProjDist = FMath::Clamp(ProjDist, Lo, Hi);

			const FVector SplineDir = LotEdge->InputSpline->GetDirectionAtDistanceAlongSpline(
				ProjDist, ESplineCoordinateSpace::World).GetSafeNormal2D();
			const float AnchorDot = FVector::DotProduct(AnchorFwd2D, SplineDir);
			const bool bFwd = (AnchorDot >= 0.0f);

			FPathSegmentInternal Seg;
			Seg.Spline = LotEdge->InputSpline;
			Seg.bTwoRoads = LotEdge->bTwoRoads;
			Seg.TwoRoadsGapM = LotEdge->TwoRoadsGapM;
			Seg.RoadType = LotEdge->RoadType;
			Seg.DrivingRule = URoadRuleLibrary::GetDrivingRuleFromRoadType(LotEdge->RoadType);
			Seg.RoadWidthMultiplier = LotEdge->RoadWidthMultiplier;
			Seg.AdditionalWidthM = LotEdge->AdditionalWidthM;
			Seg.GuardrailSideOffsetCm = LotEdge->GuardrailSideOffsetCm;
			Seg.AutoLaneWidthCm = LotEdge->AutoLaneWidthCm;
			Seg.AutoMedianCm = LotEdge->AutoMedianCm;
			Seg.EdgeId = LotEdge->EdgeId;

			// Start from projection + triangle base length (same as PrependStartBoundEdgeSegment
			// with bTriangleExitOffset), drive toward exit node
			const float ShiftedStart = ProjDist + (bFwd ? 1.0f : -1.0f) * ParkingTriangleBaseLength;
			const float ClampedStart = FMath::Clamp(ShiftedStart, Lo, Hi);

			if (bFwd)
			{
				Seg.StartDist = ClampedStart;
				Seg.EndDist = LotEdge->EndDistanceOnSpline;
				Seg.Direction = 1.0f;
				Seg.EndNodeId = LotEdge->EndNodeId;
			}
			else
			{
				Seg.StartDist = ClampedStart;
				Seg.EndDist = LotEdge->StartDistanceOnSpline;
				Seg.Direction = -1.0f;
				Seg.EndNodeId = LotEdge->StartNodeId;
			}

			if (bIsFollowing) ResetFollowingState();
			PathSegments.Empty();
			PathSegments.Add(Seg);
			CurrentSegmentIndex = 0;
			ReferenceDistance = Seg.StartDist;
			CurrentSpeed = 0.0f;
			TargetLaneIndex = CurrentLaneIndex;
			CurrentLateralOffset = ComputeTargetLaneOffset(CurrentLaneIndex);
			bIsChangingLane = false;
			bOnJunctionCurve = false;
			bJustExitedJunctionCurve = false;
			JCurveProgress = 0.0f;
			CurrentTurnSignal = ETurnSignal::None;
			NavState = ENavState::Driving;
			bIsFollowing = true;

			// ---- Call RerouteToNode — exact same code path as manual map click ----
			RerouteToNode(TargetNodeId);

			// ---- Departure curve: Hermite from parking spot to first segment's spline ----
			if (PathSegments.Num() > 0)
			{
				FPathSegmentInternal& FirstSeg = PathSegments[0];
				const int32 OutermostLane = FMath::Max(0, FirstSeg.DrivingRule.ForwardLaneCount - 1);
				const float OuterLaneOffset = ComputeLaneOffsetCm(
					FirstSeg.bTwoRoads, FirstSeg.TwoRoadsGapM, FirstSeg.RoadType,
					OutermostLane,
					FirstSeg.AutoLaneWidthCm, FirstSeg.AutoMedianCm,
					TwoRoadsMedianAdjustCm, TwoRoadsLaneWidthAdjustCm,
					SharedRoadMedianAdjustCm, SharedRoadLaneWidthAdjustCm);

				FVector RoadStartPos, RoadStartDir, RoadStartRight;
				SampleSplineAtDist(FirstSeg, FirstSeg.StartDist, OuterLaneOffset,
					RoadStartPos, RoadStartDir, RoadStartRight);

				const float DistToRoad = FVector::Dist2D(CarPos, RoadStartPos);

				if (DistToRoad > DepartureCurveTriggerDistance)
				{
					DepCurveP0 = CarPos;
					DepCurveT0 = Owner->GetActorForwardVector() * DistToRoad * DepartureCurveStartTangentScale;
					DepCurveP1 = RoadStartPos;
					DepCurveT1 = RoadStartDir.GetSafeNormal() * DistToRoad * DepartureCurveTangentScale;
					DepCurveLength = FMath::Max(DistToRoad * DepartureCurveLengthMultiplier, DepartureCurveMinLength);
					DepCurveProgress = 0.0f;
					bOnDepartureCurve = true;

					TargetLaneIndex = OutermostLane;
					CurrentLaneIndex = OutermostLane;
					CurrentLateralOffset = OuterLaneOffset;
				}
			}
			return;
		}
	}

	// ---- Fallback: not near a parking lot → original StartNavigationInternal ----
	const int32 FromNode = FindForwardStartNode(CarPos, CarFwd);
	if (FromNode == INDEX_NONE)
	{
		OnPathComplete.Broadcast(false);
		return;
	}

	StartNavigationInternal(FromNode, TargetNodeId);
}

// ============================================================================
//  Navigate to a world position (auto snap to nearest graph node).
// ============================================================================
void URoadPathFollowerComponent::NavigateToLocation(const FVector& Destination)
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: RoadNetworkSubsystem not found"));
		return;
	}

	// Release previously-held spot — switching destination shouldn't keep the old reservation
	ReleasePreviousDestinationSpot();

	// Snap destination to nearest graph node
	const int32 GoalNode = Sub->FindNearestGraphNode(Destination);
	if (GoalNode == INDEX_NONE)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("PathFollower: Cannot find nearest node for destination (%.0f, %.0f, %.0f)"),
			Destination.X, Destination.Y, Destination.Z);
		OnPathComplete.Broadcast(false);
		return;
	}

	//Store destination world location
	DestinationWorldLocation = Destination;

	NavigateToNode(GoalNode);
}

// ============================================================================
//  Navigate to parking lot and park in a spot.
// ============================================================================
void URoadPathFollowerComponent::NavigateToParkingLot(AParkingLotActor* ParkingLot, int32 SpotIndex)
{
	if (!ParkingLot)
	{
		UE_LOG(LogTemp, Error, TEXT("[PARK-NAV] NavigateToParkingLot — null ParkingLot!"));
		OnPathComplete.Broadcast(false);
		return;
	}

	UWorld* World = GetWorld();
	if (!World) { UE_LOG(LogTemp, Error, TEXT("[PARK-NAV] No World!")); return; }

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub) { UE_LOG(LogTemp, Error, TEXT("[PARK-NAV] No RoadNetworkSubsystem!")); return; }

	// Release any previously-held spot BEFORE occupying a new one
	ReleasePreviousDestinationSpot();

	//Find available spot
	if (SpotIndex == INDEX_NONE)
	{
		SpotIndex = ParkingLot->FindAvailableSpot();
	}
	if (SpotIndex == INDEX_NONE)
	{
		OnPathComplete.Broadcast(false);
		return;
	}

	// Occupy spot
	if (!ParkingLot->OccupySpot(SpotIndex, GetOwner()))
	{
		OnPathComplete.Broadcast(false);
		return;
	}

	const FVector SpotWorldPos = ParkingLot->GetSpotWorldPosition(SpotIndex);

	// Store destination state
	DestinationType = EDestinationType::ParkingLot;
	TargetParkingLot = ParkingLot;
	TargetParkingSpotIndex = SpotIndex;

	// ============================================================================
	// Key: A* goal is the ENTRY node (not exit) of BoundEdge. The final segment is the
	// BoundEdge itself, truncated at the anchor projection — so the car drives the bound
	// edge from its entry to the projection point without stopping at the intermediate node.
	// ============================================================================
	const int32 NavEdgeId = ParkingLot->BoundEdgeId;
	if (NavEdgeId == INDEX_NONE)
	{
		ParkingLot->ReleaseSpot(SpotIndex);
		OnPathComplete.Broadcast(false);
		return;
	}

	const FRoadGraphEdge* BoundEdge = Sub->GetGraphEdgeById(NavEdgeId);
	if (!BoundEdge || !BoundEdge->InputSpline)
	{
		ParkingLot->ReleaseSpot(SpotIndex);
		OnPathComplete.Broadcast(false);
		return;
	}

	// Arrow[0] = navigation anchor
	const FVector AnchorPos = ParkingLot->GetAnchorArrowPosition();
	const FVector AnchorFwd = ParkingLot->GetAnchorArrowForward();
	const FVector AnchorProjPt = ParkingLot->BoundProjectionPoint;

	// Use Arrow[0] forward vs spline tangent at anchor projection to pick drive direction
	const float AnchorKey = BoundEdge->InputSpline->FindInputKeyClosestToWorldLocation(AnchorPos);
	const float AnchorSplineDist = BoundEdge->InputSpline->GetDistanceAlongSplineAtSplineInputKey(AnchorKey);
	const FVector AnchorSplineDir = BoundEdge->InputSpline->GetDirectionAtDistanceAlongSpline(
		AnchorSplineDist, ESplineCoordinateSpace::World);
	const float AnchorDot = FVector::DotProduct(
		AnchorFwd.GetSafeNormal2D(), AnchorSplineDir.GetSafeNormal2D());

	// (Car reaches entry, then drives along BoundEdge to the anchor projection in the edge middle.)
	const bool bForwardDrive = (AnchorDot >= 0.0f);
	const int32 EntryNodeId = bForwardDrive ? BoundEdge->StartNodeId : BoundEdge->EndNodeId;

	// Clamp anchor projection to bound edge's spline distance range
	const float BoundMinDist = FMath::Min(BoundEdge->StartDistanceOnSpline, BoundEdge->EndDistanceOnSpline);
	const float BoundMaxDist = FMath::Max(BoundEdge->StartDistanceOnSpline, BoundEdge->EndDistanceOnSpline);
	const float ProjDistAbs = FMath::Clamp(AnchorSplineDist, BoundMinDist, BoundMaxDist);


	if (EntryNodeId == INDEX_NONE)
	{
		ParkingLot->ReleaseSpot(SpotIndex);
		OnPathComplete.Broadcast(false);
		return;
	}

	ParkingMode = EParkingMode::RoadsideStop;


	NavigateToNode(EntryNodeId);

	// ---- Manually append BoundEdge as the final segment, truncated to projection dist ----
	AppendBoundEdgeFinalSegment(*BoundEdge, bForwardDrive, ProjDistAbs);

	// NavigateToNode overwrites DestinationWorldLocation, so set it AFTER
	// Destination = Arrow[0] projection on bound edge
	DestinationWorldLocation = AnchorProjPt;

}

// ============================================================================
//  Truncate path so it ends at the spline projection of TargetPos.
// ============================================================================
void URoadPathFollowerComponent::TruncatePathToWorldPosition(const FVector& TargetPos, int32 TargetEdgeId)
{
	if (PathSegments.Num() == 0)
	{
		return;
	}

	// Find segment matching EdgeId
	int32 TargetSegIdx = INDEX_NONE;
	for (int32 i = PathSegments.Num() - 1; i >= 0; --i)
	{
		if (PathSegments[i].EdgeId == TargetEdgeId)
		{
			TargetSegIdx = i;
			break;
		}
	}

	// If no matching edge, use last segment
	if (TargetSegIdx == INDEX_NONE)
	{
		TargetSegIdx = PathSegments.Num() - 1;
	}

	FPathSegmentInternal& Seg = PathSegments[TargetSegIdx];
	if (!Seg.Spline)
	{
		return;
	}

	// Project target position onto spline
	const float InputKey = Seg.Spline->FindInputKeyClosestToWorldLocation(TargetPos);
	const float ProjectedDist = Seg.Spline->GetDistanceAlongSplineAtSplineInputKey(InputKey);

	// Get projected world pos for verification
	const FVector ProjectedWorldPos = Seg.Spline->GetLocationAtDistanceAlongSpline(
		ProjectedDist, ESplineCoordinateSpace::World);

	const float OldEndDist = Seg.EndDist;
	const float MinDist = FMath::Min(Seg.StartDist, OldEndDist);
	const float MaxDist = FMath::Max(Seg.StartDist, OldEndDist);

	// Clamp to segment's valid range
	const float ClampedDist = FMath::Clamp(ProjectedDist, MinDist, MaxDist);
	Seg.EndDist = ClampedDist;


	//Remove all segments after this one
	const int32 RemoveCount = PathSegments.Num() - TargetSegIdx - 1;
	if (RemoveCount > 0)
	{
		PathSegments.RemoveAt(TargetSegIdx + 1, RemoveCount);
	}
}

// ============================================================================
//  Append the bound edge as the final segment with EndDist clamped to projection.
// ============================================================================
void URoadPathFollowerComponent::AppendBoundEdgeFinalSegment(
	const FRoadGraphEdge& BoundEdge, bool bForward, float ProjDistAbs)
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub) return;

	if (!BoundEdge.InputSpline)
	{
		UE_LOG(LogTemp, Error, TEXT("[APPEND-BOUND] BoundEdge has no spline!"));
		return;
	}

	// If NavigateToNode produced no segments (A* failed or from==to), bail out —
	// we'd need full StartNavigationInternal init (departure curve, state…) to handle it.
	if (PathSegments.Num() == 0)
	{
		return;
	}

	FPathSegmentInternal Seg;
	Seg.Spline = BoundEdge.InputSpline;
	Seg.bTwoRoads = BoundEdge.bTwoRoads;
	Seg.TwoRoadsGapM = BoundEdge.TwoRoadsGapM;
	Seg.RoadType = BoundEdge.RoadType;
	Seg.DrivingRule = URoadRuleLibrary::GetDrivingRuleFromRoadType(BoundEdge.RoadType);
	Seg.RoadWidthMultiplier = BoundEdge.RoadWidthMultiplier;
	Seg.AdditionalWidthM = BoundEdge.AdditionalWidthM;
	Seg.GuardrailSideOffsetCm = BoundEdge.GuardrailSideOffsetCm;
	Seg.AutoLaneWidthCm = BoundEdge.AutoLaneWidthCm;
	Seg.AutoMedianCm = BoundEdge.AutoMedianCm;
	Seg.EdgeId = BoundEdge.EdgeId;

	if (bForward)
	{
		Seg.StartDist = BoundEdge.StartDistanceOnSpline;
		Seg.Direction = 1.0f;
		Seg.EndNodeId = BoundEdge.EndNodeId;
	}
	else
	{
		Seg.StartDist = BoundEdge.EndDistanceOnSpline;
		Seg.Direction = -1.0f;
		Seg.EndNodeId = BoundEdge.StartNodeId;
	}

	// Truncate to (projection − n along travel direction) — this is the entry hypotenuse's base point.
	// The remaining Hermite "parking curve" then runs from here (edge direction) to AnchorPos (arrow direction),
	// forming hypotenuse 1 of the isosceles triangle.
	const float BoundMinDist = FMath::Min(BoundEdge.StartDistanceOnSpline, BoundEdge.EndDistanceOnSpline);
	const float BoundMaxDist = FMath::Max(BoundEdge.StartDistanceOnSpline, BoundEdge.EndDistanceOnSpline);
	const float ClampedProj = FMath::Clamp(ProjDistAbs, BoundMinDist, BoundMaxDist);
	const float PulledBack = ClampedProj - Seg.Direction * ParkingTriangleBaseLength;
	Seg.EndDist = FMath::Clamp(PulledBack, BoundMinDist, BoundMaxDist);

	Seg.TurnAtEnd = ETurnSignal::None;
	Seg.bUTurnAtEnd = false;

	// ---- Recompute previously-last segment's TurnAtEnd — it now feeds into BoundEdge ----
	FPathSegmentInternal& PrevLast = PathSegments.Last();

	const int32 EntryNodeId = bForward ? BoundEdge.StartNodeId : BoundEdge.EndNodeId;

	if (PrevLast.EndNodeId != EntryNodeId)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[APPEND-BOUND] Discontinuity! PrevLast EdgeId=%d EndNode=%d but BoundEdge entry=%d — car may take a weird turn"),
			PrevLast.EdgeId, PrevLast.EndNodeId, EntryNodeId);
	}

	// Use actual spline tangents: PrevLast end tangent vs new Seg start tangent
	ComputeTurnAtJunction(PrevLast, Seg, PrevLast.TurnAtEnd, PrevLast.bUTurnAtEnd);

	PathSegments.Add(Seg);

}

// ============================================================================
//  Compute turn type from actual spline tangents at the junction
// ============================================================================
void URoadPathFollowerComponent::ComputeTurnAtJunction(
	const FPathSegmentInternal& InSeg,
	const FPathSegmentInternal& OutSeg,
	ETurnSignal& OutTurn,
	bool& bOutUTurn,
	float* OutDot,
	float* OutCrossZ) const
{
	OutTurn = ETurnSignal::None;
	bOutUTurn = false;

	if (!InSeg.Spline || !OutSeg.Spline)
	{
		if (OutDot) *OutDot = 1.0f;
		if (OutCrossZ) *OutCrossZ = 0.0f;
		return;
	}

	// Incoming dir: InSeg travel dir at EndDist
	FVector InPos, InDir, InRight;
	SampleSplineAtDist(InSeg, InSeg.EndDist, 0.0f, InPos, InDir, InRight);

	// Outgoing dir: OutSeg travel dir at StartDist
	FVector OutPos, OutDir, OutRight;
	SampleSplineAtDist(OutSeg, OutSeg.StartDist, 0.0f, OutPos, OutDir, OutRight);

	FVector In2D = InDir;  In2D.Z = 0.0f;  In2D.Normalize();
	FVector Out2D = OutDir; Out2D.Z = 0.0f; Out2D.Normalize();

	const float Dot = FVector::DotProduct(In2D, Out2D);
	const float CrossZ = FVector::CrossProduct(In2D, Out2D).Z;

	if (OutDot) *OutDot = Dot;
	if (OutCrossZ) *OutCrossZ = CrossZ;

	if (Dot > JunctionStraightDot)
	{
		OutTurn = ETurnSignal::None;
		bOutUTurn = false;
	}
	else if (Dot < JunctionUTurnDot)
	{
		// U-turn
		OutTurn = (CrossZ > 0.0f) ? ETurnSignal::Right : ETurnSignal::Left;
		bOutUTurn = true;
	}
	else
	{
		OutTurn = (CrossZ > 0.0f) ? ETurnSignal::Right : ETurnSignal::Left;
		bOutUTurn = false;
	}
}

// ============================================================================
//  Build path segments from A* result.
// ============================================================================
void URoadPathFollowerComponent::BuildPathSegments(const FRoadGraphPath& AStarPath)
{
	PathSegments.Empty();

	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub) return;

	const TArray<FRoadGraphEdge>& Edges = Sub->GetGraphEdges();
	const TArray<int32>& Nodes = AStarPath.NodePath;

	for (int32 i = 0; i < Nodes.Num() - 1; ++i)
	{
		const int32 From = Nodes[i];
		const int32 To = Nodes[i + 1];

		for (const FRoadGraphEdge& E : Edges)
		{
			const bool bFwd = (E.StartNodeId == From && E.EndNodeId == To);
			const bool bRev = (E.StartNodeId == To && E.EndNodeId == From);
			if (!bFwd && !bRev) continue;
			if (!E.InputSpline) break;

			FPathSegmentInternal Seg;
			Seg.Spline = E.InputSpline;
			Seg.bTwoRoads = E.bTwoRoads;
			Seg.TwoRoadsGapM = E.TwoRoadsGapM;
			Seg.RoadType = E.RoadType;
			Seg.DrivingRule = URoadRuleLibrary::GetDrivingRuleFromRoadType(E.RoadType);
			Seg.RoadWidthMultiplier = E.RoadWidthMultiplier;
			Seg.AdditionalWidthM = E.AdditionalWidthM;
			Seg.GuardrailSideOffsetCm = E.GuardrailSideOffsetCm;
			Seg.AutoLaneWidthCm = E.AutoLaneWidthCm;
			Seg.AutoMedianCm = E.AutoMedianCm;
			Seg.EdgeId = E.EdgeId;
			Seg.EndNodeId = bFwd ? E.EndNodeId : E.StartNodeId;

			if (bFwd)
			{
				Seg.StartDist = E.StartDistanceOnSpline;
				Seg.EndDist = E.EndDistanceOnSpline;
				Seg.Direction = 1.0f;
			}
			else
			{
				Seg.StartDist = E.EndDistanceOnSpline;
				Seg.EndDist = E.StartDistanceOnSpline;
				Seg.Direction = -1.0f;
			}

			PathSegments.Add(Seg);

			break;
		}
	}

	// ---- Precompute turn direction at each junction using actual spline tangents ----
	for (int32 i = 0; i + 1 < PathSegments.Num(); ++i)
	{
		float DbgDot = 0.0f, DbgCross = 0.0f;
		ComputeTurnAtJunction(
			PathSegments[i], PathSegments[i + 1],
			PathSegments[i].TurnAtEnd, PathSegments[i].bUTurnAtEnd,
			&DbgDot, &DbgCross);
	}
}

// ============================================================================
//  Mid-drive reroute: keep current segment, A* from segment end to new dest.
// ============================================================================
void URoadPathFollowerComponent::RerouteToNode(int32 TargetNodeId)
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub) return;

	const FPathSegmentInternal& CurSeg = PathSegments[CurrentSegmentIndex];
	const int32 FromNodeId = CurSeg.EndNodeId;

	if (FromNodeId == INDEX_NONE)
	{
		return;
	}

	// If destination is already the end of current segment → no reroute needed
	if (FromNodeId == TargetNodeId)
	{
		PathSegments.RemoveAt(CurrentSegmentIndex + 1, PathSegments.Num() - CurrentSegmentIndex - 1);
		return;
	}

	// Find current segment's start node (the node behind the car)
	const TArray<FRoadGraphEdge>& Edges = Sub->GetGraphEdges();
	int32 CurSegStartNode = INDEX_NONE;
	for (const FRoadGraphEdge& E : Edges)
	{
		if (E.EdgeId == CurSeg.EdgeId)
		{
			CurSegStartNode = (CurSeg.Direction > 0.0f) ? E.StartNodeId : E.EndNodeId;
			break;
		}
	}

	FRoadGraphPath AStarPath = Sub->FindPathAStar(FromNodeId, TargetNodeId);
	if (!AStarPath.bPathFound)
	{
		return;
	}

	// U-turn is only allowed at:
	//   1. Dead-end nodes (MergedEndpointCount == 1) — true road ends
	//   2. Junction nodes (bIsJunction == true) — intersections
	// If FromNodeId is not a valid U-turn point, BFS forward to find the nearest one,
	// drive there first, then U-turn.
	if (AStarPath.NodePath.Num() >= 2 && CurSegStartNode != INDEX_NONE
		&& AStarPath.NodePath[1] == CurSegStartNode)
	{
		// Find nearest valid U-turn node (BFS forward from FromNodeId, excluding CurSegStartNode direction)
		const int32 UTurnNodeId = Sub->FindNearestUTurnNode(FromNodeId, CurSegStartNode);

		if (UTurnNodeId == FromNodeId)
		{
			// FromNodeId itself is a valid U-turn point → use original A* path (which includes the U-turn)
		}
		else if (UTurnNodeId != INDEX_NONE)
		{
			// Drive forward to valid U-turn node, then A* from there to destination
			FRoadGraphPath ForwardPath = Sub->FindPathAStar(FromNodeId, UTurnNodeId);
			FRoadGraphPath ReturnPath = Sub->FindPathAStar(UTurnNodeId, TargetNodeId);

			if (ForwardPath.bPathFound && ReturnPath.bPathFound)
			{
				// Merge paths: FromNode → ... → UTurnNode → ... → TargetNode
				FRoadGraphPath MergedPath;
				MergedPath.bPathFound = true;
				MergedPath.NodePath = ForwardPath.NodePath;

				// ReturnPath's first node is UTurnNode (already at ForwardPath end), skip it
				for (int32 i = 1; i < ReturnPath.NodePath.Num(); ++i)
				{
					MergedPath.NodePath.Add(ReturnPath.NodePath[i]);
				}
				MergedPath.TotalCost = ForwardPath.TotalCost + ReturnPath.TotalCost;
				AStarPath = MergedPath;

			}
		}
	}

	// Clear all segments after current
	PathSegments.RemoveAt(CurrentSegmentIndex + 1, PathSegments.Num() - CurrentSegmentIndex - 1);

	// Build new segments and append
	AppendPathSegments(AStarPath);

	// Cancel overtake
	if (OvertakeState != EOvertakeState::None)
	{
		OvertakeState = EOvertakeState::None;
	}

	for (int32 i = CurrentSegmentIndex; i + 1 < PathSegments.Num(); ++i)
	{
		ComputeTurnAtJunction(
			PathSegments[i], PathSegments[i + 1],
			PathSegments[i].TurnAtEnd, PathSegments[i].bUTurnAtEnd);
	}

	if (PathSegments.Num() > 0)
	{
		PathSegments.Last().TurnAtEnd = ETurnSignal::None;
		PathSegments.Last().bUTurnAtEnd = false;
	}

}

// ============================================================================
//  Build segments from A* result and append to PathSegments.
// ============================================================================
void URoadPathFollowerComponent::AppendPathSegments(const FRoadGraphPath& AStarPath)
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub) return;

	const TArray<FRoadGraphEdge>& Edges = Sub->GetGraphEdges();
	const TArray<int32>& NodePath = AStarPath.NodePath;

	for (int32 i = 0; i < NodePath.Num() - 1; ++i)
	{
		const int32 From = NodePath[i];
		const int32 To = NodePath[i + 1];

		for (const FRoadGraphEdge& E : Edges)
		{
			const bool bFwd = (E.StartNodeId == From && E.EndNodeId == To);
			const bool bRev = (E.StartNodeId == To && E.EndNodeId == From);
			if (!bFwd && !bRev) continue;
			if (!E.InputSpline) break;

			FPathSegmentInternal Seg;
			Seg.Spline = E.InputSpline;
			Seg.bTwoRoads = E.bTwoRoads;
			Seg.TwoRoadsGapM = E.TwoRoadsGapM;
			Seg.RoadType = E.RoadType;
			Seg.DrivingRule = URoadRuleLibrary::GetDrivingRuleFromRoadType(E.RoadType);
			Seg.RoadWidthMultiplier = E.RoadWidthMultiplier;
			Seg.AdditionalWidthM = E.AdditionalWidthM;
			Seg.GuardrailSideOffsetCm = E.GuardrailSideOffsetCm;
			Seg.AutoLaneWidthCm = E.AutoLaneWidthCm;
			Seg.AutoMedianCm = E.AutoMedianCm;
			Seg.EdgeId = E.EdgeId;
			Seg.EndNodeId = bFwd ? E.EndNodeId : E.StartNodeId;

			if (bFwd)
			{
				Seg.StartDist = E.StartDistanceOnSpline;
				Seg.EndDist = E.EndDistanceOnSpline;
				Seg.Direction = 1.0f;
			}
			else
			{
				Seg.StartDist = E.EndDistanceOnSpline;
				Seg.EndDist = E.StartDistanceOnSpline;
				Seg.Direction = -1.0f;
			}

			PathSegments.Add(Seg);
			break;
		}
	}
}

// ============================================================================
//  RequestLaneChange
// ============================================================================
void URoadPathFollowerComponent::RequestLaneChange(int32 TargetLane)
{
	if (TargetLane == TargetLaneIndex) return;

	TargetLaneIndex = FMath::Max(0, TargetLane);
	bIsChangingLane = true;
	SmoothedLateralVel = 0.0f;  // reset so blend-in starts from zero each lane change
}

// ============================================================================
//  Sample position at given segment distance with lateral offset.
// ============================================================================
void URoadPathFollowerComponent::SampleSplineAtDist(
	const FPathSegmentInternal& Seg, float Dist, float LateralOffset,
	FVector& OutPosition, FVector& OutTravelDir, FVector& OutTravelRight) const
{
	// IsValid catches both null AND pending-kill/stale UObject pointers — the latter
	// can happen if a road actor gets destroyed while a PathFollower still references
	// its spline. A raw `!Seg.Spline` check passes stale-but-non-null through.
	if (!IsValid(Seg.Spline))
	{
		OutPosition = FVector::ZeroVector;
		OutTravelDir = FVector::ForwardVector;
		OutTravelRight = FVector::RightVector;
		return;
	}

	const float SplineLen = Seg.Spline->GetSplineLength();
	const float Clamped = FMath::Clamp(Dist, 0.0f, SplineLen);

	OutPosition = Seg.Spline->GetLocationAtDistanceAlongSpline(
		Clamped, ESplineCoordinateSpace::World);

	const FRotator SplineRot = Seg.Spline->GetRotationAtDistanceAlongSpline(
		Clamped, ESplineCoordinateSpace::World);

	const FVector SplineFwd = SplineRot.Vector();
	const FVector SplineRight = FRotationMatrix(SplineRot).GetUnitAxis(EAxis::Y);

	OutTravelDir = SplineFwd * Seg.Direction;
	OutTravelRight = SplineRight * Seg.Direction;

	// Apply lateral offset
	OutPosition += OutTravelRight * LateralOffset;
}

// ============================================================================
//  Pursuit point: look ahead along path, crossing segment boundaries.
// ============================================================================
FVector URoadPathFollowerComponent::GetPursuitPoint(float LateralOffset) const
{
	// Look-ahead distance = max(speed × time, minimum)
	const float AheadDist = FMath::Max(CurrentSpeed * LookAheadTime, MinLookAheadDist);

	float Remaining = AheadDist;
	int32 SegIdx = CurrentSegmentIndex;
	float Dist = ReferenceDistance;

	while (Remaining > 0.0f && SegIdx < PathSegments.Num())
	{
		const FPathSegmentInternal& Seg = PathSegments[SegIdx];

		// Distance from current position to segment end
		const float DistToEnd = FMath::Abs(Seg.EndDist - Dist);

		if (Remaining <= DistToEnd)
		{
			// Pursuit point is within this segment
			const float FinalDist = Dist + Remaining * Seg.Direction;

			FVector Pos, Dir, Right;
			SampleSplineAtDist(Seg, FinalDist, LateralOffset, Pos, Dir, Right);
			return Pos;
		}

		// Cross to next segment
		Remaining -= DistToEnd;
		SegIdx++;

		if (SegIdx < PathSegments.Num())
		{
			Dist = PathSegments[SegIdx].StartDist;
		}
	}

	// If beyond path end, return the last segment's end position
	if (PathSegments.Num() > 0)
	{
		const FPathSegmentInternal& LastSeg = PathSegments.Last();
		FVector Pos, Dir, Right;
		SampleSplineAtDist(LastSeg, LastSeg.EndDist, LateralOffset, Pos, Dir, Right);
		return Pos;
	}

	return FVector::ZeroVector;
}

// ============================================================================
//  Remaining distance to end of path.
// ============================================================================
float URoadPathFollowerComponent::GetRemainingDistance() const
{
	if (!bIsFollowing || PathSegments.Num() == 0) return 0.0f;

	// Remaining in current segment
	float Total = FMath::Abs(PathSegments[CurrentSegmentIndex].EndDist - ReferenceDistance);

	// Add all subsequent segments
	for (int32 i = CurrentSegmentIndex + 1; i < PathSegments.Num(); ++i)
	{
		Total += PathSegments[i].GetTravelLength();
	}

	return Total;
}

// ============================================================================
//  Current spline curvature (1/cm). Higher = sharper turn.
// ============================================================================
float URoadPathFollowerComponent::GetCurrentCurvature() const
{
	if (CurrentSegmentIndex >= PathSegments.Num()) return 0.0f;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	if (!Seg.Spline) return 0.0f;

	const float SplineLen = Seg.Spline->GetSplineLength();
	const float Clamped = FMath::Clamp(ReferenceDistance, 0.0f, SplineLen);

	// Estimate curvature from tangent direction change between two nearby points
	const float SampleDelta = 100.0f; //1 meter
	const float DistA = FMath::Clamp(Clamped - SampleDelta, 0.0f, SplineLen);
	const float DistB = FMath::Clamp(Clamped + SampleDelta, 0.0f, SplineLen);

	if (FMath::IsNearlyEqual(DistA, DistB)) return 0.0f;

	const FVector TangentA = Seg.Spline->GetDirectionAtDistanceAlongSpline(
		DistA, ESplineCoordinateSpace::World);
	const FVector TangentB = Seg.Spline->GetDirectionAtDistanceAlongSpline(
		DistB, ESplineCoordinateSpace::World);

	// Angle difference / arc length = curvature
	const float AngleRad = FMath::Acos(FMath::Clamp(
		FVector::DotProduct(TangentA, TangentB), -1.0f, 1.0f));
	const float ArcLength = DistB - DistA;

	return (ArcLength > 1.0f) ? (AngleRad / ArcLength) : 0.0f;
}

// ============================================================================
//  Target lateral offset for given lane index.
// ============================================================================
float URoadPathFollowerComponent::ComputeTargetLaneOffset(int32 LaneIdx) const
{
	if (CurrentSegmentIndex >= PathSegments.Num()) return 0.0f;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];

	return ComputeLaneOffsetCm(
		Seg.bTwoRoads, Seg.TwoRoadsGapM, Seg.RoadType, LaneIdx,
		Seg.AutoLaneWidthCm, Seg.AutoMedianCm,
		TwoRoadsMedianAdjustCm, TwoRoadsLaneWidthAdjustCm,
		SharedRoadMedianAdjustCm, SharedRoadLaneWidthAdjustCm);
}

// ============================================================================
//  Distance to next segment boundary (junction).
// ============================================================================
float URoadPathFollowerComponent::GetDistanceToNextJunction() const
{
	if (CurrentSegmentIndex >= PathSegments.Num()) return FLT_MAX;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	return FMath::Abs(Seg.EndDist - ReferenceDistance);
}

// ============================================================================
//  Compute desired speed considering acceleration, curves, junctions, braking.
// ============================================================================
float URoadPathFollowerComponent::ComputeDesiredSpeed() const
{
	// Boost speed during overtake so the pass actually completes
	float EffectiveMaxSpeed = MaxSpeed;
	if (OvertakeState == EOvertakeState::Passing
		|| OvertakeState == EOvertakeState::Returning)
	{
		EffectiveMaxSpeed = MaxSpeed * OvertakeSpeedBoost;
	}

	float Desired = EffectiveMaxSpeed;

	// ---- Curve speed reduction ----
	const float Curvature = GetCurrentCurvature();
	if (Curvature > 0.0001f)
	{
		const float CurveFactor = 1.0f / (1.0f + CurveSensitivity * Curvature * 10000.0f);
		const float CurveSpeed = MaxSpeed * FMath::Max(CurveFactor, CurveMinSpeedRatio);
		Desired = FMath::Min(Desired, CurveSpeed);
	}

	// ---- Junction approach slowdown ----
	// Reach min speed at JunctionBlendDistance (not junction center).
	// Ensures car enters Hermite curve already at low speed — no speed discontinuity.
	const float DistToJunction = GetDistanceToNextJunction();
	const bool bNextSegExists = (CurrentSegmentIndex + 1 < PathSegments.Num());

	if (bNextSegExists && DistToJunction < JunctionSlowdownDistance)
	{
		// Use CurrentTurnSignal (updated in Step 0 each tick) for slowdown decision
		if (CurrentTurnSignal != ETurnSignal::None)
		{
			// U-turn pre-deceleration target is higher, maintaining natural motion
			const bool bUpcomingUTurn = PathSegments.IsValidIndex(CurrentSegmentIndex)
				&& PathSegments[CurrentSegmentIndex].bUTurnAtEnd;
			const float MinUTurnSpeed = 400.0f; // ~14.4 km/h
			const float TargetSpeedRatio = bUpcomingUTurn
				? FMath::Max(UTurnSpeedRatio, MinUTurnSpeed / FMath::Max(MaxSpeed, 1.0f))
				: JunctionMinSpeedRatio;

			float Alpha;
			if (DistToJunction <= JunctionBlendDistance)
			{
				// Inside curve trigger zone → hold minimum
				Alpha = 1.0f;
			}
			else
			{
				// SmoothStep deceleration from SlowdownDistance to BlendDistance
				Alpha = 1.0f - (DistToJunction - JunctionBlendDistance)
						/ (JunctionSlowdownDistance - JunctionBlendDistance);
				Alpha = FMath::SmoothStep(0.0f, 1.0f, Alpha);
			}
			const float JunctionSpeed = FMath::Lerp(
				MaxSpeed, MaxSpeed * TargetSpeedRatio, Alpha);
			Desired = FMath::Min(Desired, JunctionSpeed);
		}
	}

	// Hold low speed for entire curve. Post-curve FInterpTo ramps up naturally.
	// Gap bridge curves don't slow down — visually straight, just non-contiguous spline distance.
	if (bOnJunctionCurve && !bIsGapBridgeCurve)
	{
		if (bIsUTurnCurve)
		{
			// U-turn: use UTurnSpeedRatio as target but enforce minimum speed
			// Normal cars U-turn at ~10-20 km/h (278-556 cm/s), never stopping
			const float UTurnTargetSpeed = MaxSpeed * UTurnSpeedRatio;
			const float MinUTurnSpeed = 400.0f;  // ~14.4 km/h min U-turn speed
			Desired = FMath::Min(Desired, FMath::Max(UTurnTargetSpeed, MinUTurnSpeed));
		}
		else
		{
			Desired = FMath::Min(Desired, MaxSpeed * JunctionMinSpeedRatio);
		}
	}

	// ---- Obstacle braking ----
	{
		const float ObstacleLimit = ComputeObstacleSpeedLimit();
		Desired = FMath::Min(Desired, ObstacleLimit);
	}

	// ---- End-of-path braking ----
	const float RemainDist = GetRemainingDistance();
	if (RemainDist < BrakeDistance && BrakeDeceleration > 0.0f)
	{
		const float BrakeSpeed = FMath::Sqrt(
			2.0f * BrakeDeceleration * FMath::Max(RemainDist, 0.0f));
		Desired = FMath::Min(Desired, BrakeSpeed);
	}

	return FMath::Max(Desired, 0.0f);
}

// ============================================================================
//  Sample world positions from current position to end of path.
// ============================================================================
FString URoadPathFollowerComponent::GetDestinationName() const
{
	if (DestinationType == EDestinationType::ParkingLot && TargetParkingLot.IsValid())
	{
		return TargetParkingLot->ParkingLotName;
	}
	return FString();
}

int32 URoadPathFollowerComponent::GetTargetSpotIndex() const
{
	if (DestinationType == EDestinationType::ParkingLot)
	{
		return TargetParkingSpotIndex;
	}
	return INDEX_NONE;
}

FString URoadPathFollowerComponent::GetObstacleActorName() const
{
	if (ObstacleActor.IsValid())
	{
		return ObstacleActor->GetName();
	}
	return FString();
}

FString URoadPathFollowerComponent::GetObstacleTypeString() const
{
	if (!ObstacleActor.IsValid())
	{
		return TEXT("None");
	}

	// Has PathFollowerComponent → Vehicle
	if (ObstacleActor->FindComponentByClass<URoadPathFollowerComponent>())
	{
		return TEXT("Vehicle");
	}

	// Check if it's a traffic control object (red light blocker, etc.)
	const FString ClassName = ObstacleActor->GetClass()->GetName();
	if (ClassName.Contains(TEXT("Traffic")) || ClassName.Contains(TEXT("Light"))
		|| ClassName.Contains(TEXT("Signal")) || ClassName.Contains(TEXT("Blocker")))
	{
		return TEXT("Traffic");
	}

	return TEXT("Static");
}

FVector URoadPathFollowerComponent::GetDestinationDisplayLocation() const
{
	// Parking lot destination → return parking lot actor location (not road projection)
	if (DestinationType == EDestinationType::ParkingLot && TargetParkingLot.IsValid())
	{
		return TargetParkingLot->GetActorLocation();
	}
	return DestinationWorldLocation;
}

void URoadPathFollowerComponent::GetRouteWorldPoints(TArray<FVector>& OutPoints, int32 SamplesPerSegment) const
{
	OutPoints.Empty();
	if (!bIsFollowing || PathSegments.Num() == 0) return;

	for (int32 SegIdx = CurrentSegmentIndex; SegIdx < PathSegments.Num(); ++SegIdx)
	{
		const FPathSegmentInternal& Seg = PathSegments[SegIdx];
		if (!Seg.Spline) continue;

		// Start/end for sampling this segment
		const float SampleStart = (SegIdx == CurrentSegmentIndex) ? ReferenceDistance : Seg.StartDist;
		const float SampleEnd = Seg.EndDist;
		const float TotalDist = FMath::Abs(SampleEnd - SampleStart);
		if (TotalDist < 1.0f) continue;

		for (int32 s = 0; s <= SamplesPerSegment; ++s)
		{
			const float Alpha = static_cast<float>(s) / SamplesPerSegment;
			const float Dist = SampleStart + Alpha * (SampleEnd - SampleStart);

			FVector Pos, Dir, Right;
			SampleSplineAtDist(Seg, Dist, CurrentLateralOffset, Pos, Dir, Right);
			OutPoints.Add(Pos);
		}
	}
}

// ============================================================================
//  Main update loop: speed control → advance reference → pursuit → interpolate.
// ============================================================================
void URoadPathFollowerComponent::TickComponent(
	float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// First Tick: road graph is ready, start now.
	// Must also check bAutoStart — TrafficManager sets it to false after spawn
	// to prevent hardcoded StartNodeId/GoalNodeId from overriding the random destination.
	if (bPendingStart)
	{
		bPendingStart = false;
		if (bAutoStart)
		{
			StartFollowing();
		}
	}

	if (!bIsFollowing || PathSegments.Num() == 0) return;

	AActor* Owner = GetOwner();
	if (!Owner) return;

	// Update departure progress so subsequent spot cars see this one as cleared
	UpdateDepartureProgress();

	FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	if (!Seg.Spline)
	{
		bIsFollowing = false;
		return;
	}

	// ================================================================
	//  0a. Pending final-edge lane force — we marked an outer-lane snap
	//  request at segment-entry and now try to apply it once the rear is
	//  clear. Timeout after FinalEdgeLaneForceMaxWait so we don't sit on
	//  the inner lane forever when traffic keeps tailing us.
	// ================================================================
	if (bPendingFinalEdgeForce && bIsFinalEdge)
	{
		const int32 DesiredLane = PendingFinalEdgeTargetLane;
		const bool bAlreadyAtTarget = (TargetLaneIndex == DesiredLane);

		if (bAlreadyAtTarget)
		{
			bPendingFinalEdgeForce = false;
			FinalEdgeLaneForceWait = 0.0f;
		}
		else
		{
			const bool bRearSafe  = IsLaneChangeRearSafe(DesiredLane, LaneChangeRearCheckDistance);
			const bool bFrontClear = IsTargetLaneFrontClear(DesiredLane, LaneChangeFrontCheckDistance);
			FinalEdgeLaneForceWait += DeltaTime;
			// Timeout only overrides the REAR check (rear car will yield).
			// If the FRONT lane is occupied we never force — just stay in current lane
			// and let the parking curve run from wherever we end up.
			const bool bTimedOut  = (FinalEdgeLaneForceWait >= FinalEdgeLaneForceMaxWait);

			if (bFrontClear && (bRearSafe || bTimedOut))
			{
				TargetLaneIndex = DesiredLane;
				bIsChangingLane = true;
				bPendingFinalEdgeForce = false;

				LogEvent(FString::Printf(
					TEXT("FINAL-EDGE LANE apply (lane=%d, %s, wait=%.1fs)"),
					DesiredLane,
					bRearSafe ? TEXT("rear-safe") : TEXT("TIMEOUT-FORCED"),
					FinalEdgeLaneForceWait));

				FinalEdgeLaneForceWait = 0.0f;
			}
			else if (!bFrontClear)
			{
				// Front occupied — don't cut in, keep waiting for a gap
				const float NowT = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
				if (NowT - LastAutoLaneRearBlockLogTime > 2.0f)
				{
					LastAutoLaneRearBlockLogTime = NowT;
					LogEvent(FString::Printf(
						TEXT("FINAL-EDGE WAIT: front of lane %d blocked (wait=%.1fs)"),
						DesiredLane, FinalEdgeLaneForceWait));
				}
			}
		}
	}

	// ================================================================
	//     Turn signal + auto lane change (precomputed, no distance threshold)
	// ================================================================
	{
		// Turn signal: only when approaching the next junction (gated by AutoLaneChangeDistance).
		// Prevents fire-on-entry where a far-away upcoming turn wrongly lights the blinker.
		const ETurnSignal UpcomingTurn = Seg.TurnAtEnd;
		const float DistToJunction = GetDistanceToNextJunction();
		const bool bWithinSignalDist = (DistToJunction < AutoLaneChangeDistance);

		if (UpcomingTurn != ETurnSignal::None && bWithinSignalDist
			&& CurrentTurnSignal != UpcomingTurn)
		{
			CurrentTurnSignal = UpcomingTurn;
		}
		else if ((UpcomingTurn == ETurnSignal::None || !bWithinSignalDist)
			&& !bOnJunctionCurve)
		{
			CurrentTurnSignal = ETurnSignal::None;
		}

		// Auto lane change: use AutoLaneChangeDistance (earlier than slowdown)
		if (UpcomingTurn != ETurnSignal::None && !bIsChangingLane
			&& DistToJunction < AutoLaneChangeDistance)
		{
			const int32 CurLaneCount = Seg.DrivingRule.ForwardLaneCount;
			int32 DesiredLane = TargetLaneIndex;

			if (UpcomingTurn == ETurnSignal::Left)
			{
				DesiredLane = 0;
			}
			else
			{
				DesiredLane = CurLaneCount - 1;
			}

			if (DesiredLane != TargetLaneIndex && CurLaneCount > 1)
			{
				// Two-sided safety gate:
				//   Rear  — don't cut in front of a following car.
				//           Can be overridden when very close to the junction.
				//   Front — don't merge into an occupied lane.
				//           NEVER overridden: if blocked, stay in current lane
				//           and take the junction from there.
				const float ForceCommitDist = AutoLaneChangeDistance * 0.4f;
				const bool bRearSafe   = IsLaneChangeRearSafe(DesiredLane, LaneChangeRearCheckDistance);
				const bool bFrontClear = IsTargetLaneFrontClear(DesiredLane, LaneChangeFrontCheckDistance);
				const bool bMustCommit = (DistToJunction < ForceCommitDist);

				if (bFrontClear && (bRearSafe || bMustCommit))
				{
					RequestLaneChange(DesiredLane);
				}
				else
				{
					const float NowT = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
					if (NowT - LastAutoLaneRearBlockLogTime > 2.0f)
					{
						LastAutoLaneRearBlockLogTime = NowT;
					}
				}
			}
		}
	}

	if (NavState == ENavState::Driving
		&& CurrentSegmentIndex == PathSegments.Num() - 1
		&& ParkingMode == EParkingMode::RoadsideStop)
	{
		const float RemainDist = GetRemainingDistance();
		if (RemainDist < ParkingPullOverDistance)
		{
			NavState = ENavState::Parking;

			if (DestinationType == EDestinationType::ParkingLot
				&& TargetParkingLot.IsValid())
			{
				// --- [PARK-NAV] ParkingLot: target = Arrow[0], bIsLeftSide picks shoulder ---
				const FVector AnchorPos = TargetParkingLot->GetAnchorArrowPosition();
				const FVector AnchorFwd = TargetParkingLot->GetAnchorArrowForward();
				ParkCurveP1 = AnchorPos;

				FVector SplineEndPos, EndTravelDir, EndTravelRight;
				SampleSplineAtDist(Seg, Seg.EndDist, 0.0f, SplineEndPos, EndTravelDir, EndTravelRight);

				// Turn signal: directly from bIsLeftSide
				CurrentTurnSignal = TargetParkingLot->IsLeftSide() ? ETurnSignal::Left : ETurnSignal::Right;

				const float CurveDist = (ParkCurveP1 - SplineEndPos).Size();
				ParkCurveT1 = AnchorFwd * CurveDist * ParkingCurveTangentScale;

			}
			else
			{
				// --- Default: right shoulder ---
				CurrentTurnSignal = ETurnSignal::Right;
				const int32 ShoulderLane = Seg.DrivingRule.ForwardLaneCount;
				ParkingTargetOffset = ComputeTargetLaneOffset(ShoulderLane);
			}
		}
	}

	// ================================================================
	//      Obstacle detection (unified: vehicles, red light blockers, any obstacle)
	// ================================================================
	UpdateObstacleDetection();

	// ================================================================
	//      Stuck recovery — detect overlapping/stuck vehicles and unstick
	// ================================================================
	UpdateStuckRecovery(DeltaTime);

	// UpdateStuckRecovery may have teleported + parked the car (ResetFollowingState),
	// which empties PathSegments and sets bIsFollowing=false.  Re-check before any
	// code that dereferences PathSegments / Seg.Spline, or we crash on a dangling ptr.
	if (!bIsFollowing || PathSegments.Num() == 0) return;

	// If actively reversing to unstick, skip normal speed control / overtaking
	if (!bIsStuckReversing)
	{
		// ================================================================
		//      Overtaking logic (slow car → pass → return)
		// ================================================================
		// DISABLED — overtake suspected to cause SmoothedLateralVel accumulation bugs.
		UpdateOvertakeLogic();

		// ================================================================
		//     Speed control: accelerate toward desired or brake
		// ================================================================
		const float RawDesiredSpeed = ComputeDesiredSpeed();

		// Smooth the desired speed on the way UP only — braking stays instant.
		// This prevents a jarring lurch when resuming from a full stop.
		if (RawDesiredSpeed >= SmoothedDesiredSpeed)
			SmoothedDesiredSpeed = FMath::FInterpTo(SmoothedDesiredSpeed, RawDesiredSpeed, DeltaTime, AccelerationSmoothRate);
		else
			SmoothedDesiredSpeed = RawDesiredSpeed;  // drop instantly for braking

		const float DesiredSpeed = SmoothedDesiredSpeed;

		if (CurrentSpeed < DesiredSpeed)
		{
			// Accelerate using Acceleration param, capped at DesiredSpeed
			CurrentSpeed = FMath::Min(CurrentSpeed + Acceleration * DeltaTime, DesiredSpeed);
		}
		else
		{
			// Emergency brake zone — apply stronger braking when too close
			const float EmergencyZoneDist = ObstacleSlowdownDistance * EmergencyBrakeZoneFraction;
			const bool bEmergencyBrake = (ObstacleDistance >= 0.0f && ObstacleDistance < EmergencyZoneDist);
			const float EffectiveBrakeDecel = bEmergencyBrake
				? BrakeDeceleration * EmergencyBrakeMultiplier
				: BrakeDeceleration;

			CurrentSpeed = FMath::Max(CurrentSpeed - EffectiveBrakeDecel * DeltaTime, DesiredSpeed);
		}

		// Ensure non-negative
		CurrentSpeed = FMath::Max(CurrentSpeed, 0.0f);
	} // end if (!bIsOverlapReversing)

	// ================================================================
	//      Departure curve: smoothly merge from parking spot onto road
	// ================================================================
	if (bOnDepartureCurve)
	{
		// Departure curve speed controlled by DepartureCurveSpeedRatio
		const float DepSpeed = MaxSpeed * DepartureCurveSpeedRatio;
		CurrentSpeed = FMath::FInterpTo(CurrentSpeed, DepSpeed, DeltaTime, 3.0f);

		DepCurveProgress += CurrentSpeed * DeltaTime;
		const float T = FMath::Clamp(DepCurveProgress / FMath::Max(DepCurveLength, 1.0f), 0.0f, 1.0f);

		const FVector CurvePos = FMath::CubicInterp(DepCurveP0, DepCurveT0, DepCurveP1, DepCurveT1, T);
		const FVector CurveTangent = FMath::CubicInterpDerivative(DepCurveP0, DepCurveT0, DepCurveP1, DepCurveT1, T);

		FRotator FinalRot = Owner->GetActorRotation();
		if (CurveTangent.SizeSquared() > 1.0f)
		{
			const FRotator TargetRot = CurveTangent.Rotation();
			FinalRot = FMath::RInterpTo(FinalRot, TargetRot, DeltaTime, 5.0f);
		}

		Owner->SetActorLocationAndRotation(CurvePos, FinalRot);

		if (T >= 1.0f)
		{
			// Departure curve done → continue normal spline following.
			// Sync SmoothedDesiredSpeed to CurrentSpeed so the post-curve
			// acceleration ramps up from the current low speed instead of
			// from the pre-accumulated high value (which causes a lurch).
			bOnDepartureCurve = false;
			SmoothedDesiredSpeed = CurrentSpeed;
		}
		return; // Departure curve in progress → skip normal logic
	}

	// ================================================================
	//     Junction curve: if on Hermite curve, follow it to completion
	// ================================================================
	if (bOnJunctionCurve)
	{
		const float PrevProgress = JCurveProgress;
		JCurveProgress += CurrentSpeed * DeltaTime;
		const float T = FMath::Clamp(JCurveProgress / FMath::Max(JCurveLength, 1.0f), 0.0f, 1.0f);

		if (T < 1.0f)
		{
			FVector CurvePos;
			FVector CurveTangent;

			if (bIsUTurnCurve)
			{
				// ====== Parametric semicircle ======
				// P(t)  = Center + R·(cos(πt)·U + sin(πt)·V)
				// P'(t) = R·π·(-sin(πt)·U + cos(πt)·V) 
				const float Angle = PI * T;
				const float CosA = FMath::Cos(Angle);
				const float SinA = FMath::Sin(Angle);

				const FVector FlatPos =
					UTurnCenter + UTurnArcRadius * (CosA * UTurnAxisU + SinA * UTurnAxisV);

				CurvePos = FVector(FlatPos.X, FlatPos.Y,
					FMath::Lerp(UTurnArcZ0, UTurnArcZ1, T));

				CurveTangent =
					UTurnArcRadius * (-SinA * UTurnAxisU + CosA * UTurnAxisV);
			}
			else
			{
				// Hermite / Normal junction Hermite
				CurvePos = FMath::CubicInterp(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T);
				CurveTangent = FMath::CubicInterpDerivative(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T);
			}

			// Rotation: use curve tangent as target, RInterpTo smoothing
			const FVector PrevPos = Owner->GetActorLocation();
			FRotator FinalRot = Owner->GetActorRotation();
			if (CurveTangent.SizeSquared2D() > 1.0f)
			{
				const FRotator TargetRot = CurveTangent.Rotation();
				FinalRot = FMath::RInterpTo(
					FinalRot, TargetRot, DeltaTime,
					bIsUTurnCurve ? UTurnRotationSpeed : RotationInterpSpeed);
			}

			Owner->SetActorLocationAndRotation(CurvePos, FinalRot);


			return;// Still on curve → skip normal spline
		}

		// ---- T >= 1.0: Curve done, fall through to new segment's spline logic (same frame) ----
		const bool bWasUTurn = bIsUTurnCurve;
		const FVector UTurnFinalCarPos = Owner->GetActorLocation();  // for reprojection

		LogEvent(FString::Printf(TEXT("JUNCTION EXIT %s"), bWasUTurn ? TEXT("(U-turn)") : TEXT("")));

		bOnJunctionCurve = false;
		bIsUTurnCurve = false;
		bIsGapBridgeCurve = false;

		// Sync SmoothedDesiredSpeed to CurrentSpeed so post-curve acceleration
		// ramps up gradually from the current (slow) junction speed instead of
		// from the pre-accumulated high target (which causes a sudden lurch).
		SmoothedDesiredSpeed = CurrentSpeed;

		CurrentSegmentIndex++;

		if (CurrentSegmentIndex >= PathSegments.Num())
		{
			bIsFollowing = false;
			CurrentSpeed = 0.0f;
			NavState = ENavState::Parked;
			CurrentTurnSignal = ETurnSignal::None;
			LogEvent(TEXT("PATH COMPLETE (after junction curve)"));
			OnPathComplete.Broadcast(true);
			return;
		}

		Seg = PathSegments[CurrentSegmentIndex];
		HandleEnteredNewSegment();

		// ================================================================
		// U-turn special: reproject the car's actual position onto the new
		// segment spline to prevent teleport. The semicircle end point is pure
		// geometry and may not sit on the spline; the old code forced RefDist
		// back to JCurveNextRefDist (= NextPos), causing a huge jump. Instead
		// we find the nearest spline distance to the car and let lateral
		// VInterpTo clean up the residual.
		// ================================================================
		if (bWasUTurn && Seg.Spline)
		{
			// Search nearest point ONLY within Seg[1]'s [StartDist, EndDist] range.
			// Reason: the spline may be shared across multiple edges, so
			// FindInputKeyClosestToWorldLocation can return a point outside this
			// segment's own range — hundreds of metres away — causing Step 6 to
			// snap there. We do a coarse + refine sample sweep inside the range.
			const float Lo = FMath::Min(Seg.StartDist, Seg.EndDist);
			const float Hi = FMath::Max(Seg.StartDist, Seg.EndDist);

			auto DistSqAt = [&](float D)
			{
				const FVector P = Seg.Spline->GetLocationAtDistanceAlongSpline(
					D, ESplineCoordinateSpace::World);
				return FVector::DistSquared(UTurnFinalCarPos, P);
			};

			// 64-way coarse sweep
			const int32 CoarseN = 64;
			float BestDist = Lo;
			float BestDistSq = DistSqAt(Lo);
			for (int32 i = 1; i <= CoarseN; ++i)
			{
				const float D = Lo + (Hi - Lo) * (float(i) / float(CoarseN));
				const float Dsq = DistSqAt(D);
				if (Dsq < BestDistSq) { BestDistSq = Dsq; BestDist = D; }
			}

			// Refine: Refine around best candidate
			float Step = (Hi - Lo) / float(CoarseN);
			for (int32 iter = 0; iter < 4; ++iter)
			{
				const float Near = FMath::Max(Lo, BestDist - Step);
				const float Far  = FMath::Min(Hi, BestDist + Step);
				const int32 RefineN = 16;
				for (int32 i = 0; i <= RefineN; ++i)
				{
					const float D = Near + (Far - Near) * (float(i) / float(RefineN));
					const float Dsq = DistSqAt(D);
					if (Dsq < BestDistSq) { BestDistSq = Dsq; BestDist = D; }
				}
				Step /= float(RefineN);
			}

			const float NewDist = BestDist;
			const FVector NewSplinePt = Seg.Spline->GetLocationAtDistanceAlongSpline(
				NewDist, ESplineCoordinateSpace::World);
			const float ReprojDelta = FMath::Sqrt(BestDistSq);
			const float OldRefDist = JCurveNextRefDist;


			// If the gap is huge (next segment's spline is far from the car),
			// the A* path is spatially disconnected after the U-turn. Following
			// that spline would drive through void for 20+ seconds. Instead,
			// re-plan the entire path from the car's current position.
			const float UTurnReprojMaxGap = 2000.0f; // 20m 閾值 / threshold (cm)
			if (ReprojDelta > UTurnReprojMaxGap && DestinationNodeId != INDEX_NONE)
			{
				// Clear curve state
				bOnJunctionCurve = false;
				bIsUTurnCurve = false;
				bIsGapBridgeCurve = false;
				bJustExitedJunctionCurve = false;
				CurrentTurnSignal = ETurnSignal::None;

				// Re-plan from car's position (auto: FindStartBoundEdge → A* → BuildPathSegments)
				StartNavigationInternal(INDEX_NONE, DestinationNodeId);
				return;
			}

			ReferenceDistance = NewDist;
		}
		else
		{
			ReferenceDistance = JCurveNextRefDist;
		}

		// Reset turn signal
		CurrentTurnSignal = ETurnSignal::None;

		// Update lane and lateral offset for new segment
		TargetLaneIndex = JCurveNextLaneIndex;
		CurrentLaneIndex = JCurveNextLaneIndex;
		bIsChangingLane = false;
		CurrentLateralOffset = ComputeTargetLaneOffset(TargetLaneIndex);

		// Flag just-exited so Step 6 uses high interp speed for seamless transition
		bJustExitedJunctionCurve = true;

		// DON'T return — fall through to Step 3~6 below
	}

	// ================================================================
	//     Advance reference point along spline
	// ================================================================
	ReferenceDistance += CurrentSpeed * DeltaTime * Seg.Direction;

	// 3a. Natural segment advance: when ReferenceDistance passes EndDist and a next
	//     segment exists, carry overshoot forward. This is the ONE place straight
	//     junctions hand off — no teleport, because the two segments meet at the
	//     junction node (world positions are continuous).
	const FVector PreAdvancePos = Owner->GetActorLocation(); 
	const int32 PreAdvanceSegIdx = CurrentSegmentIndex;       
	const float PreAdvanceRefDist = ReferenceDistance;       

	while (CurrentSegmentIndex + 1 < PathSegments.Num())
	{
		const FPathSegmentInternal& CurSeg = PathSegments[CurrentSegmentIndex];
		const bool bPassedEnd =
			(CurSeg.Direction > 0.0f && ReferenceDistance >= CurSeg.EndDist)
			|| (CurSeg.Direction < 0.0f && ReferenceDistance <= CurSeg.EndDist);
		if (!bPassedEnd) break;

		// Turn / U-turn → clamp to EndDist, let Step 5 curve handle it
		if (CurSeg.TurnAtEnd != ETurnSignal::None || CurSeg.bUTurnAtEnd)
		{
			ReferenceDistance = CurSeg.EndDist;
			break;
		}

		// Pre-check junction gap: world distance between old-end and new-start.
		// If gap > 100cm, the two segments are NOT spatially contiguous on the spline —
		// clamp to EndDist and let Step 5 create a Hermite curve for smooth transition.
		const FPathSegmentInternal& NextSeg = PathSegments[CurrentSegmentIndex + 1];

		FVector OldEndPos, OldEndDir, OldEndRight;
		SampleSplineAtDist(CurSeg, CurSeg.EndDist, CurrentLateralOffset,
			OldEndPos, OldEndDir, OldEndRight);

		FVector NewStartPos, NewStartDir, NewStartRight;
		SampleSplineAtDist(NextSeg, NextSeg.StartDist, CurrentLateralOffset,
			NewStartPos, NewStartDir, NewStartRight);

		const float JunctionGap = FVector::Dist(OldEndPos, NewStartPos);

		if (JunctionGap > 100.0f)
		{
			// Non-contiguous , clamp, Step 5 will create Hermite curve
			ReferenceDistance = CurSeg.EndDist;
			break;
		}

		// Gap is small , safe to advance
		const float Overshoot = (CurSeg.Direction > 0.0f)
			? (ReferenceDistance - CurSeg.EndDist)
			: (CurSeg.EndDist - ReferenceDistance);

		// ----------------------------------------------------------------
		// Position-continuity reprojection — PART 1: sample OLD seg's end
		// BEFORE we overwrite CurSeg's underlying slot via `Seg = ...`.
		// ----------------------------------------------------------------
		FVector OldEndPosSaved, OldEndDirSaved, OldEndRightSaved;
		SampleSplineAtDist(CurSeg, CurSeg.EndDist, CurrentLateralOffset,
			OldEndPosSaved, OldEndDirSaved, OldEndRightSaved);

		CurrentSegmentIndex++;
		// Re-read new segment data into Seg via operator= (Seg is a C++ reference
		// to the original slot; this copies new fields into that slot).
		Seg = PathSegments[CurrentSegmentIndex];
		ReferenceDistance = Seg.StartDist + Overshoot * Seg.Direction;

		const int32 NewLaneCount = FMath::Max(1, Seg.DrivingRule.ForwardLaneCount);
		TargetLaneIndex = FMath::Clamp(TargetLaneIndex, 0, NewLaneCount - 1);
		CurrentLaneIndex = TargetLaneIndex;
		CurrentTurnSignal = ETurnSignal::None;

		HandleEnteredNewSegment();

		// ----------------------------------------------------------------
		// Part 2: on the NEW segment, compute how much lateral offset we need
		// to add so world position stays continuous. Uses PathSegments[CurrentSegmentIndex]
		// directly (not the potentially-confusing Seg reference) to be explicit.
		// ----------------------------------------------------------------
		if (PathSegments.IsValidIndex(CurrentSegmentIndex)
			&& PathSegments[CurrentSegmentIndex].Spline)
		{
			const FPathSegmentInternal& NewSegRef = PathSegments[CurrentSegmentIndex];

			FVector NewSamplePos, NewSampleDir, NewSampleRight;
			SampleSplineAtDist(NewSegRef, ReferenceDistance, CurrentLateralOffset,
				NewSamplePos, NewSampleDir, NewSampleRight);

			FVector Delta = OldEndPosSaved - NewSamplePos;
			Delta.Z = 0.0f;
			FVector NewRight2D = NewSampleRight; NewRight2D.Z = 0.0f;
			NewRight2D = NewRight2D.GetSafeNormal();

			if (!NewRight2D.IsNearlyZero())
			{
				const float OffsetCorrection = FVector::DotProduct(Delta, NewRight2D);

				const float MaxCorrection = FMath::Max(NewSegRef.AutoLaneWidthCm, 200.0f) * 2.0f;
				if (FMath::Abs(OffsetCorrection) <= MaxCorrection)
				{
					CurrentLateralOffset += OffsetCorrection;
				}
			}
		}
	}

	// ================================================================
	//     Lane change interpolation
	// ================================================================
	const float TargetOffset = ComputeTargetLaneOffset(TargetLaneIndex);

	// Saved BEFORE this frame's lateral update — the rotation system computes
	// FrameLateralDelta from this, but ONLY applies it when bIsChangingLane is true
	// (see heading update below). This means no tilt ever leaks out of a real lane change.
	const float PrevLateralOffset = CurrentLateralOffset;


	// Approach-phase lane change: shift lateral position during normal driving so the
	// car arrives at the junction already in the correct lane.
	// Gated by no obstacle ahead — CurrentSpeed is unreliable here because the car
	// can still have speed while decelerating for an obstacle in front.
	if (bIsChangingLane && !bOnJunctionCurve)
	{
		if (!ObstacleActor.IsValid()) {
			CurrentLateralOffset = FMath::FInterpTo(
				CurrentLateralOffset, TargetOffset, DeltaTime, LaneChangeSmoothRate);

			LogEvent(FString::Printf(TEXT("(bIsChangingLane)")));
		}
		if (FMath::IsNearlyEqual(CurrentLateralOffset, TargetOffset, 10.0f))
		{
			CurrentLaneIndex = TargetLaneIndex;

		}
		if (FMath::IsNearlyEqual(CurrentLateralOffset, TargetOffset, 1.0f))
		{
			CurrentLateralOffset = TargetOffset;
			CurrentLaneIndex = TargetLaneIndex;
			bIsChangingLane = false;
			SmoothedLateralVel = 0.0f;  // clear residual steering after lane change completes
		}

		


	}

	// ================================================================
	//     Sample position + detect junction → generate Hermite curve
	// ================================================================
	FVector RefPos, RefDir, RefRight;
	SampleSplineAtDist(Seg, ReferenceDistance, CurrentLateralOffset,
		RefPos, RefDir, RefRight);

	const float DistToEnd = FMath::Abs(Seg.EndDist - ReferenceDistance);
	const bool bNextSegExists = (CurrentSegmentIndex + 1 < PathSegments.Num());

	// Approaching junction with next segment → create Hermite curve
	//
	// Dynamic blend for multi-lane turns: outer lanes are closer to the corner,
	// need a bigger arc. Scale up by CurrentLateralOffset.
	// Left turn uses full BlendDistance, right turn uses × RightTurnBlendRatio (smaller arc)
	float EffectiveBlendDist = JunctionBlendDistance;
	if (CurrentTurnSignal == ETurnSignal::Right)
	{
		EffectiveBlendDist *= RightTurnBlendRatio;
	}
	if (CurrentTurnSignal != ETurnSignal::None)
	{
		// Per 100cm lane offset, boost BlendDistance (multi-lane dynamic adjustment)
		const float OffsetBoost = (CurrentLateralOffset / 100.0f) * OffsetBoostRate;
		EffectiveBlendDist *= (1.0f + FMath::Max(OffsetBoost, 0.0f));
	}

	if (bNextSegExists && DistToEnd < EffectiveBlendDist)
	{
		const FPathSegmentInternal& NextSeg = PathSegments[CurrentSegmentIndex + 1];

		// ---- Dot gate: InSeg end tangent vs OutSeg start tangent, filter straight segments ----
		// Note: U-turns still use the curve (needed to actually turn around);
		// only near-straight junctions skip the curve.
		float JDot = 1.0f, JCross = 0.0f;
		ETurnSignal TmpTurn;
		bool bTmpU;
		ComputeTurnAtJunction(Seg, NextSeg, TmpTurn, bTmpU, &JDot, &JCross);

		// Check junction gap: even if Dot says "straight", if the two segments are
		// non-contiguous in world space (gap > 100cm), we still need a Hermite curve.
		FVector Step5OldEnd, Step5OldEndDir, Step5OldEndR;
		SampleSplineAtDist(Seg, Seg.EndDist, CurrentLateralOffset,
			Step5OldEnd, Step5OldEndDir, Step5OldEndR);
		FVector Step5NewStart, Step5NewStartDir, Step5NewStartR;
		SampleSplineAtDist(NextSeg, NextSeg.StartDist, CurrentLateralOffset,
			Step5NewStart, Step5NewStartDir, Step5NewStartR);
		const float Step5JunctionGap = FVector::Dist(Step5OldEnd, Step5NewStart);

		const bool bSkipStraight = (JDot > JunctionStraightDot) && (Step5JunctionGap < 100.0f);

		if (bSkipStraight)
		{
			// Truly straight AND spatially contiguous → no curve, Step 3a handles it
		}
		else
		{
			// ----------------------------------------------------------
			// LEFT-TURN YIELD: sphere-trace the oncoming lane of NextSeg
			// for straight-through traffic. If any, stop & wait this
			// frame; next Tick re-checks. LeftTurnYieldMaxSec caps the
			// wait so two head-on stopped cars can't freeze forever.
			// ----------------------------------------------------------
			if (CurrentTurnSignal == ETurnSignal::Left && !Seg.bUTurnAtEnd)
			{
				AActor* OncomingBlocker = nullptr;
				float   OncomingDist = -1.0f;
				const bool bOncomingClear = IsLeftTurnOncomingClear(NextSeg, OncomingBlocker, OncomingDist);

				if (!bOncomingClear)
				{
					LeftTurnYieldAccum += DeltaTime;

					if (LeftTurnYieldAccum < LeftTurnYieldMaxSec)
					{
						// Yield: zero speed, log (throttled), return without building curve
						CurrentSpeed = 0.0f;

						const float NowT = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
						const FString YStr = TEXT("left-turn oncoming");
						if (YStr != LastYieldReason || (NowT - LastYieldLogTime) >= 3.0f)
						{
							LastYieldReason = YStr;
							LastYieldLogTime = NowT;
							LogEvent(FString::Printf(
								TEXT("YIELD L-TURN: oncoming=%s @ %.0fcm (waited %.1fs)"),
								OncomingBlocker ? *OncomingBlocker->GetName() : TEXT("?"),
								OncomingDist,
								LeftTurnYieldAccum));
						}
						return;
					}
					else
					{
						// Timeout — force commit, log once
						LogEvent(FString::Printf(
							TEXT("L-TURN FORCED (yield timeout %.1fs, oncoming=%s)"),
							LeftTurnYieldAccum,
							OncomingBlocker ? *OncomingBlocker->GetName() : TEXT("?")));
						LeftTurnYieldAccum = 0.0f;
						// fall through: build curve
					}
				}
				else
				{
					// Oncoming clear — reset accumulator
					LeftTurnYieldAccum = 0.0f;
				}
			}

			// ----------------------------------------------------------
			// PRE-CURVE RED-LIGHT SCAN: before starting a U-turn or any
			// junction curve, sphere-trace forward for red-light colliders
			// (TrafficSignal-tagged, non-vehicle). If blocked, stop and
			// wait. Once clear the curve starts normally.
			// "已經開始了就不管" is naturally satisfied: bOnJunctionCurve is
			// false here, so this block is never reached mid-curve.
			// ----------------------------------------------------------
			if (JunctionRedLightScanDistance > 0.0f)
			{
				UWorld* ScanWorld = GetWorld();
				AActor* ScanOwner = GetOwner();
				if (ScanWorld && ScanOwner)
				{
					const FVector ScanStart = ScanOwner->GetActorLocation()
						+ FVector(0, 0, ObstacleTraceZOffset);
					const FVector ScanEnd = ScanStart
						+ ScanOwner->GetActorForwardVector() * JunctionRedLightScanDistance;

					FCollisionQueryParams ScanParams;
					ScanParams.AddIgnoredActor(ScanOwner);

					// Use Multi sweep so a vehicle or static object in front of the
					// traffic signal doesn't shadow it (Single only returns the first hit).
					TArray<FHitResult> ScanHits;
					ScanWorld->SweepMultiByChannel(
						ScanHits, ScanStart, ScanEnd, FQuat::Identity,
						ECC_ObstacleDetect,
						FCollisionShape::MakeSphere(JunctionRedLightScanRadius),
						ScanParams);

					bool bRedLightAhead = false;
					const FHitResult* SignalHit = nullptr;
					for (const FHitResult& H : ScanHits)
					{
						AActor* HitAct = H.GetActor();
						if (!HitAct || HitAct == ScanOwner) continue;
						const bool bIsVehicle = HitAct->FindComponentByClass<URoadPathFollowerComponent>() != nullptr;
						const bool bIsSignal = HitAct->Tags.Contains(FName(TrafficSignalTagName));
						if (bIsSignal && !bIsVehicle)
						{
							bRedLightAhead = true;
							SignalHit = &H;
							break;
						}
					}

					if (bRedLightAhead && SignalHit)
					{
						JunctionRedLightWaitAccum += DeltaTime;
						CurrentSpeed = 0.0f;

						const float NowT = ScanWorld->GetTimeSeconds();
						const FString YStr = TEXT("pre-curve red-light");
						if (YStr != LastYieldReason || (NowT - LastYieldLogTime) >= 3.0f)
						{
							LastYieldReason = YStr;
							LastYieldLogTime = NowT;
							LogEvent(FString::Printf(
								TEXT("YIELD PRE-CURVE: red-light=%s @ %.0fcm (waited %.1fs)"),
								*SignalHit->GetActor()->GetName(),
								SignalHit->Distance,
								JunctionRedLightWaitAccum));
						}
						return;
					}
					else
					{
						JunctionRedLightWaitAccum = 0.0f;
					}
				}
			}


			if (NavState == ENavState::Parking
				&& DestinationType == EDestinationType::None)
			{
				// Default parking: gradual lateral offset to target
				CurrentLateralOffset = FMath::FInterpConstantTo(
					CurrentLateralOffset, ParkingTargetOffset, DeltaTime, LaneChangeSpeed);
			}
			else
			{
				CurrentLateralOffset = FMath::FInterpTo(
					CurrentLateralOffset, TargetOffset, DeltaTime, PositionInterpSpeed);
			}

		// Curve end: normal turns go EffectiveBlendDist into next seg; U-turn stays near start
		const bool bUpcomingUTurn = Seg.bUTurnAtEnd;
		const float EntryDist = bUpcomingUTurn ? 0.0f : EffectiveBlendDist;
		float NextSampleDistRaw = NextSeg.StartDist + EntryDist * NextSeg.Direction;

		// Clamp within NextSeg's own [StartDist, EndDist] so a large OffsetBoost
		// on a short next segment can't push NextSampleDist past the segment end
		// (where it would get clamped to the next junction, causing the Hermite
		// curve to swing across the whole segment).
		const float NextLo = FMath::Min(NextSeg.StartDist, NextSeg.EndDist);
		const float NextHi = FMath::Max(NextSeg.StartDist, NextSeg.EndDist);
		const float NextSampleDist = FMath::Clamp(NextSampleDistRaw, NextLo, NextHi);

		// Determine target lane in next segment based on turn direction.
		// MaxNextLane guards against NextLaneCount == 0 (single-lane roads map to lane 0).
		const int32 NextLaneCount = NextSeg.DrivingRule.ForwardLaneCount;
		const int32 MaxNextLane   = FMath::Max(0, NextLaneCount - 1);
		if (CurrentTurnSignal == ETurnSignal::Right)
		{
			// Right turn → outermost lane of next segment (already the max)
			JCurveNextLaneIndex = MaxNextLane;
		}
		else if (CurrentTurnSignal == ETurnSignal::Left)
		{
			// Left turn → same target lane, hard-clamped to next segment's lane range
			JCurveNextLaneIndex = FMath::Clamp(TargetLaneIndex, 0, MaxNextLane);
		}
		else
		{
			// Straight → keep current lane, clamped to next segment's lane range
			JCurveNextLaneIndex = FMath::Clamp(TargetLaneIndex, 0, MaxNextLane);
		}

		// Sample curve endpoint using next segment's target lane offset
		const float NextSegLaneOffset = ComputeLaneOffsetCm(
			NextSeg.bTwoRoads, NextSeg.TwoRoadsGapM, NextSeg.RoadType,
			JCurveNextLaneIndex,
			NextSeg.AutoLaneWidthCm, NextSeg.AutoMedianCm,
			TwoRoadsMedianAdjustCm, TwoRoadsLaneWidthAdjustCm,
			SharedRoadMedianAdjustCm, SharedRoadLaneWidthAdjustCm);

		FVector NextPos, NextDir, NextRight;
		SampleSplineAtDist(NextSeg, NextSampleDist, NextSegLaneOffset,
			NextPos, NextDir, NextRight);

		// P0 = car's ACTUAL position (not spline ref), eliminates VInterpTo lag jump
		// T0 = car's ACTUAL forward, ensures curve starts from current heading
		JCurveP0 = Owner->GetActorLocation();

		// Mark gap bridge: Dot says straight but world positions non-contiguous , no slowdown
		bIsGapBridgeCurve = (JDot > JunctionStraightDot) && (Step5JunctionGap >= 100.0f);

		// ================================================================
		// U-turn: pure geometric parametric semicircle (no Hermite)
		// ----------------------------------------------------------------
		// Center = P0 + R · SideDir (SideDir from turn signal L/R)
		// U axis = (P0 - Center)/R = -SideDir   → P(0) = P0
		// V axis = car forward (flattened)      → tangent at t=0 = Forward
		// P(t)   = Center + R·(cos(πt)·U + sin(πt)·V)
		// P(1)   = P0 + 2R · SideDir, facing backward
		// Arc length = π·R (pure geometry, independent of NextPos)
		// ================================================================
		bIsUTurnCurve = Seg.bUTurnAtEnd;
		if (bIsUTurnCurve)
		{
			const FVector OwnerFwd3D = Owner->GetActorForwardVector();
			const FVector OwnerFwd2D = FVector(OwnerFwd3D.X, OwnerFwd3D.Y, 0.0f).GetSafeNormal();

			// U-turns always go to the LEFT of current travel direction, ignoring
			// CurrentTurnSignal. When incoming/outgoing tangents are nearly antiparallel,
			// CrossZ is numerical noise and left/right becomes random. Design choice:
			// U-turns always sweep left (into oncoming lane on the car's left side).
			const float SideSign = -1.0f;

			// Car's 2D right vector in UE left-handed Z-up:
			// right2D = (-Fwd.Y, Fwd.X). forward=+X → right=+Y (matches UE standard)
			const FVector Right2D = FVector(-OwnerFwd2D.Y, OwnerFwd2D.X, 0.0f);
			const FVector SideDir2D = Right2D * SideSign;

			//   P1 − P0 = −(CurOffset + NextOffset) × Right
			//            = (CurOffset + NextOffset) × SideDir   (SideDir = −Right)
			//    P1 = P0 + 2R × SideDir
			//    2R = CurOffset + NextOffset
			//
			// Auto-compute U-turn radius from lane positions:
			//   P0 = spline centre + CurOffset × Right
			//   P1 = spline centre − NextOffset × Right  (new seg reverses direction)
			//   Distance = CurOffset + NextOffset = 2R
			//   R = (CurOffset + NextOffset) / 2 + Padding
			// Works for inner-lane or outer-lane U-turns: each offset reflects
			// the car's actual lane position on its respective segment.
			float R;
			if (bAutoUTurnRadius)
			{
				const float CurOff = FMath::Abs(CurrentLateralOffset);
				const float NextOff = FMath::Abs(NextSegLaneOffset);
				R = FMath::Max((CurOff + NextOff) * 0.5f + UTurnRadiusPadding, 50.0f);

				if (CurrentLaneIndex > 0)
				{
					R += OuterOffset * CurrentLaneIndex;
				}
			}
			else
			{
				R = UTurnRadius;
			}

			// Circle center (flat, inherits P0.Z)
			UTurnCenter = FVector(
				JCurveP0.X + SideDir2D.X * R,
				JCurveP0.Y + SideDir2D.Y * R,
				JCurveP0.Z);

			// U axis points from center to P0
			UTurnAxisU = -SideDir2D;

			// V axis = car forward (flat) → tangent direction at t=0
			UTurnAxisV = OwnerFwd2D;

			UTurnArcRadius = R;
			UTurnArcZ0 = JCurveP0.Z;
			UTurnArcZ1 = JCurveP0.Z; 

			// End P(1) = P0 + 2R · SideDir
			JCurveP1 = FVector(
				JCurveP0.X + 2.0f * R * SideDir2D.X,
				JCurveP0.Y + 2.0f * R * SideDir2D.Y,
				JCurveP0.Z);

			JCurveT0 = OwnerFwd2D * R;
			JCurveT1 = -OwnerFwd2D * R;

			// Arc length = π·R, UTurnTangentScale tunes effective speed feel
			JCurveLength = PI * R * FMath::Max(UTurnTangentScale, 0.01f);

		}

		else
		{
			JCurveP1 = NextPos;

			// Tangent magnitude = distance × TangentScale, controls curve roundness.
			// For left turns from outer lanes, apply the same OffsetBoostRate factor used
			// for EffectiveBlendDist so the arc is proportionally wider.
			float EffTangentScale = JunctionCurveTangentScale;
			if (CurrentTurnSignal == ETurnSignal::Left)
			{
				const float OffsetBoost = (CurrentLateralOffset / 100.0f) * OffsetBoostRate;
				EffTangentScale *= (1.0f + FMath::Max(OffsetBoost, 0.0f));
			}
			const float TangentScale = (JCurveP1 - JCurveP0).Size() * EffTangentScale;
			JCurveT0 = Owner->GetActorForwardVector() * TangentScale;
			JCurveT1 = NextDir.GetSafeNormal() * TangentScale;

			// Approximate curve length (slightly longer than straight line)
			JCurveLength = (JCurveP1 - JCurveP0).Size() * 1.2f;
		}

		JCurveProgress = 0.0f;
		JCurveNextRefDist = NextSampleDist;

		bOnJunctionCurve = true;

		// Immediately cap CurrentSpeed to the curve speed limit.
		// ComputeDesiredSpeed() will enforce this every tick too, but the ramp-down
		// via BrakeDeceleration can lag one frame behind on curve entry — snapping here
		// ensures the car never overshoots the curve with approach speed.
		if (!bIsGapBridgeCurve)
		{
			const float CurveSpeedLimit = bIsUTurnCurve
				? FMath::Max(MaxSpeed * UTurnSpeedRatio, 400.0f)
				: MaxSpeed * JunctionMinSpeedRatio;
			CurrentSpeed = FMath::Min(CurrentSpeed, CurveSpeedLimit);
		}

		LogEvent(FString::Printf(TEXT("JUNCTION ENTER %s%s len=%.0f"),
			bIsUTurnCurve ? TEXT("U-TURN ") : TEXT(""),
			bIsGapBridgeCurve ? TEXT("GAP ") : TEXT(""),
			JCurveLength));


		// No "first-step SetActorLocationAndRotation" — P0 is already the car's current
		// pose and JCurveProgress starts at 0; the next Tick's curve update advances
		// smoothly at CurrentSpeed with neither a 1-frame stall nor a teleport.
		JCurveProgress = 0.0f;
		return;
		} // else (not straight → curve)
	}

	// ---- Check segment end (only reached when no next segment) ----
	const bool bSegDone =
		(Seg.Direction > 0.0f && ReferenceDistance >= Seg.EndDist)
		|| (Seg.Direction < 0.0f && ReferenceDistance <= Seg.EndDist);

	if (bSegDone)
	{
		if (NavState == ENavState::Parking && !bOnParkingCurve
			&& DestinationType == EDestinationType::ParkingLot)
		{
			// --- Parking Lot: full Hermite curve into spot ---
			// Stop any pending lane-force — if the outer lane was never reached
			// the curve simply starts from wherever the car actually is.
			bPendingFinalEdgeForce = false;
			bIsChangingLane        = false;
			ParkCurveP0 = Owner->GetActorLocation();
			// Lock Z to the car's current height; ignore the arrow's Z
			ParkCurveP1.Z = ParkCurveP0.Z;
			const float CurveDist = (ParkCurveP1 - ParkCurveP0).Size();
			ParkCurveT0 = Owner->GetActorForwardVector() * CurveDist * ParkingCurveTangentScale;
			ParkCurveLength = CurveDist * 1.3f;
			ParkCurveProgress = 0.0f;
			bOnParkingCurve = true;

		}
		else if (!bOnParkingCurve)
		{
			// No destination parking or not parking: end immediately
			bIsFollowing = false;
			CurrentSpeed = 0.0f;
			NavState = ENavState::Parked;
			CurrentTurnSignal = ETurnSignal::None;
			OnPathComplete.Broadcast(true);
			return;
		}
	}

	// ---- Hermite curve from road to spot ----
	if (bOnParkingCurve)
	{
		// Parking curve speed = MaxSpeed × ParkingCurveSpeedRatio
		const float ParkSpeed = MaxSpeed * ParkingCurveSpeedRatio;
		CurrentSpeed = FMath::FInterpTo(CurrentSpeed, ParkSpeed, DeltaTime, 3.0f);

		ParkCurveProgress += CurrentSpeed * DeltaTime;
		const float T = FMath::Clamp(ParkCurveProgress / FMath::Max(ParkCurveLength, 1.0f), 0.0f, 1.0f);

		const FVector CurvePos = FMath::CubicInterp(ParkCurveP0, ParkCurveT0, ParkCurveP1, ParkCurveT1, T);
		const FVector CurveTangent = FMath::CubicInterpDerivative(ParkCurveP0, ParkCurveT0, ParkCurveP1, ParkCurveT1, T);

		FRotator FinalRot = Owner->GetActorRotation();
		if (CurveTangent.SizeSquared() > 1.0f)
		{
			const FRotator TargetRot = CurveTangent.Rotation();
			FinalRot = FMath::RInterpTo(FinalRot, TargetRot, DeltaTime, 5.0f);
		}

		Owner->SetActorLocationAndRotation(CurvePos, FinalRot);

		if (T >= 1.0f)
		{
			// Curve done → parking complete
			bOnParkingCurve = false;
			bIsFollowing = false;
			CurrentSpeed = 0.0f;
			NavState = ENavState::Parked;
			CurrentTurnSignal = ETurnSignal::None;

			OnPathComplete.Broadcast(true);
		}
		return;
	}

	// ================================================================
	//     Normal driving — snap to spline position + rotation follows tangent
	// ================================================================
	const FRotator CurrentRot = Owner->GetActorRotation();
	const FVector CurrentPos = Owner->GetActorLocation();

	// Position: directly use spline sample (no VInterpTo)
	// Movement driven entirely by ReferenceDistance (Step 3), constant per frame → zero jitter
	//
	// Exception: just exited a junction curve (esp. U-turn) → car may be
	// a few hundred cm from RefPos. Hard-snapping teleports; use VInterpTo
	// for exponential-decay blending until the gap closes.
	FVector FinalPos;
	if (bJustExitedJunctionCurve)
	{
		const float GapDist = FVector::Dist(CurrentPos, RefPos);
		const float GapTolerance = FMath::Max(CurrentSpeed * DeltaTime * 2.0f + 10.0f, 30.0f);

		if (GapDist <= GapTolerance)
		{
			FinalPos = RefPos;
			bJustExitedJunctionCurve = false;
		}
		else
		{
			// Exponential-decay blend via VInterpTo (instead of constant-speed catchup).
			// Visually: smooth deceleration into the spline, no abrupt jump or
			// constant-speed "chasing" feel.
			FinalPos = FMath::VInterpTo(CurrentPos, RefPos, DeltaTime, PostUTurnBlendSpeed);
		}
	}
	else
	{
		FinalPos = RefPos;
	}
	/*
	// Skip during post-curve blend (VInterpTo convergence, not a real teleport)
	if (!bJustExitedJunctionCurve)
	{
		const float FrameMoveDist = FVector::Dist(CurrentPos, FinalPos);
		const float ExpectedMove = CurrentSpeed * DeltaTime;
		
		if (FrameMoveDist > ExpectedMove + 200.0f && CurrentSpeed > 1.0f)
		{
			UE_LOG(LogTemp, Error,
				TEXT("STEP6_TELEPORT! FrameMove=%.0f Expected=%.0f (Speed=%.0f dt=%.4f) | ")
				TEXT("Seg[%d] RefDist=%.0f LateralOff=%.0f | ")
				TEXT("CarPos=(%.0f,%.0f,%.0f) → FinalPos=(%.0f,%.0f,%.0f) | ")
				TEXT("SegAdvanced=%s JustExitedCurve=%d"),
				FrameMoveDist, ExpectedMove, CurrentSpeed, DeltaTime,
				CurrentSegmentIndex, ReferenceDistance, CurrentLateralOffset,
				CurrentPos.X, CurrentPos.Y, CurrentPos.Z,
				FinalPos.X, FinalPos.Y, FinalPos.Z,
				(CurrentSegmentIndex != PreAdvanceSegIdx) ? TEXT("YES") : TEXT("NO"),
				bJustExitedJunctionCurve ? 1 : 0);
		}
		
	}
	*/

	// Rotation: use actual movement direction (including lateral component)
	// so the car nose naturally turns when merging sideways
	FRotator FinalRot = CurrentRot;
	if (CurrentSpeed > 1.0f && RefDir.SizeSquared() > 0.1f)
	{
		FVector ActualMoveDir = RefDir.GetSafeNormal();

		// Heading tilt: ONLY applied during an active lane change (bIsChangingLane).
		// Any lateral drift from junction-zone tracking, post-curve correction, or
		// normal spline following must NOT rotate the car — those are position-only
		// adjustments. Zeroing SmoothedLateralVel immediately when not lane-changing
		// prevents any residual tilt from leaking into normal driving or red-light stops.
		const float FrameLateralDelta = CurrentLateralOffset - PrevLateralOffset;
		if (bIsChangingLane && FMath::Abs(FrameLateralDelta) > 0.01f && DeltaTime > SMALL_NUMBER)
		{
			const float RawLateralVel = FrameLateralDelta / DeltaTime;
			SmoothedLateralVel = FMath::FInterpTo(
				SmoothedLateralVel, RawLateralVel, DeltaTime, LaneChangeSteerBlendRate);
			ActualMoveDir = (RefDir.GetSafeNormal() * CurrentSpeed + RefRight * SmoothedLateralVel).GetSafeNormal();
		}
		else
		{
			// Not lane-changing (or no delta) — snap lateral steering to zero immediately.
			SmoothedLateralVel = 0.0f;
		}

		const FRotator TargetRot = ActualMoveDir.Rotation();
		// Use a lower rotation interp speed during lane change for a natural steering feel
		const float EffRotSpeed = bIsChangingLane ? LaneChangeRotInterpSpeed : RotationInterpSpeed;
		FinalRot = FMath::RInterpTo(
			CurrentRot, TargetRot, DeltaTime, EffRotSpeed);
	}
	else
	{
		// Car is stopped or nearly stopped (speed <= 1 cm/s).
		// The rotation block above doesn't run, so SmoothedLateralVel would otherwise
		// sit at whatever value it had when the car last decelerated — and then
		// re-accumulate on the next green-light departure, compounding each red-light
		// cycle until every car ends up permanently tilted.
		// Clearing it here ensures the slate is always clean before the next move.
		SmoothedLateralVel = 0.0f;
	}

	Owner->SetActorLocationAndRotation(FinalPos, FinalRot);

}

// ============================================================================
// SphereTrace forward obstacle detection (unified: vehicles, red light blockers, any obstacle)
// When vehicle is blocked and stopped, throttle detection rate to save performance
// ============================================================================
void URoadPathFollowerComponent::UpdateObstacleDetection()
{
	// ---- Throttle: when blocked and speed≈0, don't need per-frame detection ----
	if (ObstacleThrottleInterval > 0.0f
		&& CurrentSpeed < 5.0f
		&& ObstacleDistance >= 0.0f)  // had obstacle last frame
	{
		// If last obstacle actor is no longer valid → clear immediately, re-detect
		if (!ObstacleActor.IsValid())
		{
			ObstacleDistance = -1.0f;
			ObstacleThrottleTimer = 0.0f;
			// don't return, proceed to full detection
		}
		else
		{
			ObstacleThrottleTimer += GetWorld()->GetDeltaSeconds();
			if (ObstacleThrottleTimer < ObstacleThrottleInterval)
			{
				// Keep last frame's result, skip this frame's detection
				return;
			}
			ObstacleThrottleTimer = 0.0f;  // time's up → detect once
		}
	}
	else
	{
		ObstacleThrottleTimer = 0.0f;  //normal driving → detect every frame
	}

	ObstacleDistance = -1.0f;
	ObstacleActor = nullptr;

	UWorld* World = GetWorld();
	if (!World) return;

	AActor* Owner = GetOwner();
	if (!Owner) return;

	// Raise start by ObstacleTraceZOffset to avoid hitting terrain/bumps
	const FVector Start = Owner->GetActorLocation() + FVector(0.0f, 0.0f, ObstacleTraceZOffset);
	const FVector Forward = Owner->GetActorForwardVector();
	const FVector End = Start + Forward * ObstacleSlowdownDistance;

	FHitResult Hit;
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Owner);

	const bool bHit = World->SweepSingleByChannel(
		Hit,
		Start, End,
		FQuat::Identity,
		ECC_ObstacleDetect,
		FCollisionShape::MakeSphere(ObstacleTraceRadius),
		Params);

	if (bHit)
	{
		ObstacleDistance = Hit.Distance;
		ObstacleActor = Hit.GetActor();

		// Debug log: which vehicle is blocked by what actor
		if (bDebugObstacleTrace)
		{
			const FString OwnerName = Owner->GetName();
			const FString BlockerName = ObstacleActor.IsValid() ? ObstacleActor->GetName() : TEXT("Unknown");
			const FString BlockerClass = ObstacleActor.IsValid() ? ObstacleActor->GetClass()->GetName() : TEXT("?");
			const FString HitComp = Hit.GetComponent() ? Hit.GetComponent()->GetName() : TEXT("?");
		}
	}

	// During maneuvers (junction / parking / departure) be extra cautious about
	// OTHER VEHICLES only. Wider + longer sweep, then filter out non-PathFollower
	// hits so we don't falsely balloon the hitbox against red-light / signal colliders.
	if (IsPerformingManeuver())
	{
		const float VehRadius = ObstacleTraceRadius * ManeuverVehicleTraceRadiusMult;
		const float VehDist = ObstacleSlowdownDistance * ManeuverVehicleTraceDistanceMult;
		const FVector VehEnd = Start + Forward * VehDist;

		TArray<FHitResult> VehHits;
		World->SweepMultiByChannel(
			VehHits, Start, VehEnd, FQuat::Identity,
			ECC_ObstacleDetect,
			FCollisionShape::MakeSphere(VehRadius),
			Params);

		for (const FHitResult& VH : VehHits)
		{
			AActor* A = VH.GetActor();
			if (!A || A == Owner) continue;
			if (!A->FindComponentByClass<URoadPathFollowerComponent>()) continue;
			if (ObstacleDistance < 0.0f || VH.Distance < ObstacleDistance)
			{
				ObstacleDistance = VH.Distance;
				ObstacleActor = A;
			}
		}
	}
}

// ============================================================================
//  Obstacle speed limit — sqrt curve for more aggressive early braking
// ============================================================================
float URoadPathFollowerComponent::ComputeObstacleSpeedLimit() const
{
	if (ObstacleDistance < 0.0f)
	{
		// No obstacle detected
		return MaxSpeed;
	}

	if (ObstacleDistance <= ObstacleStopDistance)
	{
		// Too close → full stop
		return 0.0f;
	}

	if (ObstacleDistance >= ObstacleSlowdownDistance)
	{
		// Far enough → no limit
		return MaxSpeed;
	}

	// sqrt curve: speed ∝ √(distance) — brakes much harder at close range
	// compared to linear, this gives ~70% speed at 50% distance (linear would give 50%)
	// but ~45% speed at 20% distance (linear would give 20%), making close-range braking
	// much more aggressive while keeping far-range braking smooth
	const float Alpha = (ObstacleDistance - ObstacleStopDistance)
		/ (ObstacleSlowdownDistance - ObstacleStopDistance);
	return MaxSpeed * FMath::Sqrt(Alpha);
}

// ============================================================================
//  Front obstacle classification — uses ObstacleActor + TrafficSignal tag
// ============================================================================
EFrontObstacleType URoadPathFollowerComponent::ClassifyFrontObstacle() const
{
	if (!ObstacleActor.IsValid()) return EFrontObstacleType::None;
	AActor* Act = ObstacleActor.Get();
	if (!Act) return EFrontObstacleType::None;

	if (Act->FindComponentByClass<URoadPathFollowerComponent>())
	{
		return EFrontObstacleType::Vehicle;
	}
	if (Act->Tags.Contains(FName(TrafficSignalTagName)))
	{
		return EFrontObstacleType::TrafficSignal;
	}
	return EFrontObstacleType::Static;
}

// ============================================================================
//  Rear clearance — sphere-trace behind car for another vehicle.
//  Returns true if safe (no vehicle within RearClearDistance).
// ============================================================================
bool URoadPathFollowerComponent::IsRearClearOfVehicles(AActor*& OutRearVehicle) const
{
	OutRearVehicle = nullptr;
	UWorld* World = GetWorld();
	AActor* Owner = GetOwner();
	if (!World || !Owner) return true;

	const FVector Start = Owner->GetActorLocation() + FVector(0, 0, ObstacleTraceZOffset);
	const FVector Back = -Owner->GetActorForwardVector();
	const FVector End = Start + Back * RearClearDistance;

	FHitResult Hit;
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Owner);

	const bool bHit = World->SweepSingleByChannel(
		Hit, Start, End, FQuat::Identity, ECC_ObstacleDetect,
		FCollisionShape::MakeSphere(RearTraceRadius), Params);

	if (bHit)
	{
		AActor* HitActor = Hit.GetActor();
		if (HitActor && HitActor->FindComponentByClass<URoadPathFollowerComponent>())
		{
			OutRearVehicle = HitActor;
			return false;
		}
	}
	return true;
}

// ============================================================================
//  IsLaneChangeRearSafe — sphere-sweep BACKWARD in the target lane to check
//  whether a vehicle is approaching us from behind in that lane. Called
//  before every lane change (overtake start, overtake return, final-edge
//  force, departure-curve exit) so we don't cut in front of another car.
//
//  Implementation: sample the current segment's spline at
//  (ReferenceDistance - CheckDistance) using the TARGET lane's lateral
//  offset, then sweep a sphere FROM there TO our current spline point
//  (also in the target lane). If the sweep hits another PathFollower,
//  the rear is not safe. Self is ignored.
// ============================================================================
bool URoadPathFollowerComponent::IsLaneChangeRearSafe(
	int32 InTargetLaneIndex, float CheckDistance) const
{
	UWorld* World = GetWorld();
	AActor* Owner = GetOwner();
	if (!World || !Owner) return true;
	if (!PathSegments.IsValidIndex(CurrentSegmentIndex)) return true;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	if (!IsValid(Seg.Spline)) return true;

	const float TargetOffset = ComputeTargetLaneOffset(InTargetLaneIndex);

	// Spline distance "behind" me, respecting segment direction (+1 / -1).
	const float SegLo = FMath::Min(Seg.StartDist, Seg.EndDist);
	const float SegHi = FMath::Max(Seg.StartDist, Seg.EndDist);
	float BehindAbs = ReferenceDistance - Seg.Direction * CheckDistance;
	BehindAbs = FMath::Clamp(BehindAbs, SegLo, SegHi);

	FVector BehindPos, BehindDir, BehindRight;
	SampleSplineAtDist(Seg, BehindAbs, TargetOffset,
		BehindPos, BehindDir, BehindRight);

	FVector CurPos, CurDir, CurRight;
	SampleSplineAtDist(Seg, ReferenceDistance, TargetOffset,
		CurPos, CurDir, CurRight);

	const FVector Start = BehindPos + FVector(0.0f, 0.0f, ObstacleTraceZOffset);
	const FVector End   = CurPos   + FVector(0.0f, 0.0f, ObstacleTraceZOffset);

	// Degenerate sweep (segment too short to project CheckDistance back) →
	// not enough info, treat as safe so we don't block forever on a tiny
	// final edge.
	if (FVector::DistSquared(Start, End) < 1.0f) return true;

	TArray<FHitResult> Hits;
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Owner);

	World->SweepMultiByChannel(Hits, Start, End, FQuat::Identity,
		ECC_ObstacleDetect,
		FCollisionShape::MakeSphere(LaneChangeRearCheckRadius), Params);

	for (const FHitResult& H : Hits)
	{
		AActor* A = H.GetActor();
		if (!A || A == Owner) continue;
		// Only other vehicles block us. Static geometry, signals etc. are
		// irrelevant for rear-lane safety.
		if (A->FindComponentByClass<URoadPathFollowerComponent>())
		{
			return false;
		}
	}
	return true;
}

// ============================================================================
//  IsTargetLaneFrontClear — sphere-sweep FORWARD in the target lane to check
//  whether a PathFollower vehicle is already occupying the space we'd merge into.
//  Returns false (blocked) if any vehicle is found within CheckDistance ahead.
// ============================================================================
bool URoadPathFollowerComponent::IsTargetLaneFrontClear(
	int32 InTargetLaneIndex, float CheckDistance) const
{
	UWorld* World = GetWorld();
	AActor* Owner = GetOwner();
	if (!World || !Owner) return true;
	if (!PathSegments.IsValidIndex(CurrentSegmentIndex)) return true;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	if (!IsValid(Seg.Spline)) return true;

	const float TargetOffset = ComputeTargetLaneOffset(InTargetLaneIndex);

	// Current longitudinal reference
	FVector CurPos, CurDir, CurRight;
	SampleSplineAtDist(Seg, ReferenceDistance, TargetOffset, CurPos, CurDir, CurRight);

	// Point ahead respecting segment direction
	const float SegLo = FMath::Min(Seg.StartDist, Seg.EndDist);
	const float SegHi = FMath::Max(Seg.StartDist, Seg.EndDist);
	float AheadAbs = ReferenceDistance + Seg.Direction * CheckDistance;
	AheadAbs = FMath::Clamp(AheadAbs, SegLo, SegHi);

	FVector AheadPos, AheadDir2, AheadRight2;
	SampleSplineAtDist(Seg, AheadAbs, TargetOffset, AheadPos, AheadDir2, AheadRight2);

	const FVector Start = CurPos   + FVector(0.0f, 0.0f, ObstacleTraceZOffset);
	const FVector End   = AheadPos + FVector(0.0f, 0.0f, ObstacleTraceZOffset);

	if (FVector::DistSquared(Start, End) < 1.0f) return true;

	TArray<FHitResult> Hits;
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Owner);

	World->SweepMultiByChannel(Hits, Start, End, FQuat::Identity,
		ECC_ObstacleDetect,
		FCollisionShape::MakeSphere(LaneChangeRearCheckRadius), Params);

	for (const FHitResult& H : Hits)
	{
		AActor* A = H.GetActor();
		if (!A || A == Owner) continue;
		if (A->FindComponentByClass<URoadPathFollowerComponent>())
			return false;
	}
	return true;
}

// ============================================================================
//  Walk forward along the stuck chain via each vehicle's ObstacleActor link.
//  OutChain is filled tail→head (index 0 = me, last = leader).
// ============================================================================
bool URoadPathFollowerComponent::BuildStuckChainForward(
	TArray<URoadPathFollowerComponent*>& OutChain) const
{
	OutChain.Reset();
	TSet<const URoadPathFollowerComponent*> Visited;

	const URoadPathFollowerComponent* Cur = this;
	for (int32 Depth = 0; Depth < MaxChainWalkDepth; ++Depth)
	{
		if (!Cur || Visited.Contains(Cur)) return false;
		Visited.Add(Cur);
		OutChain.Add(const_cast<URoadPathFollowerComponent*>(Cur));

		if (!Cur->ObstacleActor.IsValid()) return true;
		AActor* FrontAct = Cur->ObstacleActor.Get();
		if (!FrontAct) return true;

		URoadPathFollowerComponent* FrontPF =
			FrontAct->FindComponentByClass<URoadPathFollowerComponent>();
		if (!FrontPF) return true;  // front is non-vehicle → current is leader

		// Front vehicle must also be stuck-stopped for the chain to continue.
		const bool bFrontStopped = (FrontPF->GetCurrentSpeed() < 5.0f);
		if (!bFrontStopped) return true;

		Cur = FrontPF;
	}
	return false;  // depth exhausted
}

// ============================================================================
//  IsChainLeader — walk forward along the stuck chain and decide.
//  Unlike the single-hop version, this handles cycles (crossing-deadlock at
//  intersections) by picking the FName-smallest member of the cycle as leader.
// ============================================================================
bool URoadPathFollowerComponent::IsChainLeader() const
{
	TSet<const URoadPathFollowerComponent*> Visited;
	TArray<const URoadPathFollowerComponent*> Chain;

	const URoadPathFollowerComponent* Cur = this;
	for (int32 Depth = 0; Depth < MaxChainWalkDepth; ++Depth)
	{
		if (!Cur) return (Depth == 0);

		// Cycle detected → leader = FName-smallest car in the cycle (self-decided,
		// every car reaches the same conclusion so exactly one reverses).
		if (Visited.Contains(Cur))
		{
			const AActor* MyOwner = GetOwner();
			if (!MyOwner) return false;
			const FString MyName = MyOwner->GetName();
			for (const URoadPathFollowerComponent* C : Chain)
			{
				if (!C || C == this) continue;
				const AActor* O = C->GetOwner();
				if (!O) continue;
				if (O->GetName().Compare(MyName) < 0) return false;  // someone smaller exists
			}
			return true;
		}
		Visited.Add(Cur);
		Chain.Add(Cur);

		// Terminal: no front obstacle → Cur is chain head. I'm leader iff Cur == me.
		if (!Cur->ObstacleActor.IsValid()) return (Cur == this);
		AActor* FrontAct = Cur->ObstacleActor.Get();
		if (!FrontAct) return (Cur == this);

		URoadPathFollowerComponent* FrontPF =
			FrontAct->FindComponentByClass<URoadPathFollowerComponent>();
		if (!FrontPF) return (Cur == this);                 // static / traffic → Cur is head
		if (FrontPF->GetCurrentSpeed() >= 5.0f) return (Cur == this);  // front moving → done

		Cur = FrontPF;
	}
	// Depth exhausted — conservatively say no (prevents runaway).
	return false;
}

// ============================================================================
//  FindChainReverser — walk forward along the stuck chain and decide who
//  should reverse to break the deadlock.
//
//  Why this exists (vs. IsChainLeader):
//    IsChainLeader returns bool "am I the leader". It gates each car's
//    decision to reverse, but it has two blind spots:
//      (a) 3+ car CYCLES at a junction are a bool-true for exactly one car
//          (the FName-smallest), but the OTHER two cars only yield. They
//          depend on the leader reversing + its rear-chain cascade covering
//          them. That mostly works, BUT cascade only kicks in AFTER the
//          leader crosses StuckRecoveryDelay — at junctions they can sit
//          longer than that because IsChainLeader's walk may terminate
//          early (e.g., a link breaks because one car briefly moved >5cm/s).
//      (b) "Zombie head" chains: the chain terminates at a stopped car
//          whose FORWARD has no vehicle (lost path, failed junction gap,
//          etc.). That head never enters Phase 1 because the Phase 1 gate
//          requires bFrontIsVehicle. Followers walk to it, see "head has
//          no obstacle", IsChainLeader returns false-for-them, and
//          EVERYONE yields forever.
//
//  This function returns the car that should actually reverse, OR nullptr
//  if the situation is self-resolving (front is moving, waiting for red
//  light, etc.). Callers are expected to trigger the returned reverser +
//  cascade its rear chain.
// ============================================================================
URoadPathFollowerComponent* URoadPathFollowerComponent::FindChainReverser(
	TArray<URoadPathFollowerComponent*>& OutChainGroup,
	FString& OutReason) const
{
	OutChainGroup.Reset();
	OutReason.Reset();

	TSet<URoadPathFollowerComponent*> Visited;
	URoadPathFollowerComponent* Cur = const_cast<URoadPathFollowerComponent*>(this);

	for (int32 Depth = 0; Depth < MaxChainWalkDepth; ++Depth)
	{
		if (!Cur)
		{
			OutReason = TEXT("null-cur");
			return nullptr;
		}

		// ---- Cycle detected ----
		if (Visited.Contains(Cur))
		{
			URoadPathFollowerComponent* Smallest = nullptr;
			FString SmallestName;
			for (URoadPathFollowerComponent* C : OutChainGroup)
			{
				if (!C || !C->GetOwner()) continue;
				const FString N = C->GetOwner()->GetName();
				if (!Smallest || N.Compare(SmallestName) < 0)
				{
					Smallest = C;
					SmallestName = N;
				}
			}
			OutReason = FString::Printf(TEXT("cycle-N=%d"), OutChainGroup.Num());
			return Smallest;
		}
		Visited.Add(Cur);
		OutChainGroup.Add(Cur);

		// ---- Terminal: no ObstacleActor ----
		// Head car has nothing in front. Either legitimately waiting
		// (red light ahead, approaching junction, etc.) or zombie-stuck.
		if (!Cur->ObstacleActor.IsValid())
		{
			const bool bCurStopped = (Cur->GetCurrentSpeed() < 5.0f);
			const bool bBusy = Cur->IsStuckReversing() || Cur->StuckCooldownTimer > 0.0f;
			if (bCurStopped && !bBusy)
			{
				// Guard: require the car to have been continuously obstacle-free AND
				// stopped for at least StuckRecoveryDelay seconds before we treat it
				// as a zombie. This prevents a car that was just blocked by a traffic
				// signal (and transiently lost its ObstacleActor due to a detection
				// throttle gap or signal flicker) from triggering a mass reversal of
				// the entire queued chain behind it.
				if (Cur->ZombieStuckTimer < Cur->StuckRecoveryDelay)
				{
					OutReason = TEXT("head-zombie-too-fresh");
					return nullptr;
				}

				// Zombie head — nothing visibly blocks it, but it's frozen.
				// If this is the ONLY car (Depth==0, self-loop), don't
				// trigger (I'd reverse myself for no reason). Only trigger
				// when there's at least one follower (me) behind it.
				if (Cur == this)
				{
					OutReason = TEXT("self-zombie");
					return nullptr;  // Phase 1 gate already excludes me
				}
				OutReason = TEXT("zombie-head-no-obstacle");
				return Cur;
			}
			OutReason = bBusy ? TEXT("head-reversing-or-cooldown") : TEXT("head-moving");
			return nullptr;
		}

		AActor* FrontAct = Cur->ObstacleActor.Get();
		URoadPathFollowerComponent* FrontPF = FrontAct
			? FrontAct->FindComponentByClass<URoadPathFollowerComponent>()
			: nullptr;

		// ---- Terminal: non-vehicle obstacle (static / signal) ----
		if (!FrontPF)
		{
			const EFrontObstacleType FT = Cur->ClassifyFrontObstacle();
			const bool bCurStopped = (Cur->GetCurrentSpeed() < 5.0f);
			const bool bBusy = Cur->IsStuckReversing() || Cur->StuckCooldownTimer > 0.0f;

			if (FT == EFrontObstacleType::TrafficSignal)
			{
				OutReason = TEXT("head-at-traffic-signal");
				return nullptr;  // legit wait
			}
			if (FT == EFrontObstacleType::Static && bCurStopped && !bBusy)
			{
				if (Cur == this)
				{
					OutReason = TEXT("self-static");
					return nullptr;  // let the static-escape path handle me
				}
				OutReason = TEXT("zombie-head-static");
				return Cur;
			}
			OutReason = TEXT("head-misc");
			return nullptr;
		}

		// ---- Front vehicle is moving → chain will clear on its own ----
		if (FrontPF->GetCurrentSpeed() >= 5.0f)
		{
			OutReason = TEXT("front-moving");
			return nullptr;
		}

		Cur = FrontPF;
	}

	OutReason = TEXT("depth-exhausted");
	return nullptr;
}

// ============================================================================
//  GetManeuverLabel —  mutual-stuck log
// ============================================================================
FString URoadPathFollowerComponent::GetManeuverLabel() const
{
	if (bOnDepartureCurve) return TEXT("DEPART");
	if (bOnParkingCurve)   return TEXT("PARKING");
	if (bOnJunctionCurve)
	{
		if (bIsUTurnCurve)                              return TEXT("U-TURN");
		if (CurrentTurnSignal == ETurnSignal::Left)     return TEXT("LEFT");
		if (CurrentTurnSignal == ETurnSignal::Right)    return TEXT("RIGHT");
		return TEXT("STRAIGHT-GAP");
	}
	if (CurrentTurnSignal == ETurnSignal::Left)         return TEXT("APPR-LEFT");
	if (CurrentTurnSignal == ETurnSignal::Right)        return TEXT("APPR-RIGHT");
	return TEXT("STRAIGHT");
}

// ============================================================================
//  CountStoppedRearChain — BFS walk backward: find every stopped PathFollower
//  that transitively points at me as its ObstacleActor.
// ============================================================================
int32 URoadPathFollowerComponent::CountStoppedRearChain(
	TArray<URoadPathFollowerComponent*>& OutChain) const
{
	OutChain.Reset();
	UWorld* W = GetWorld();
	if (!W) return 0;

	TSet<const URoadPathFollowerComponent*> Visited;
	Visited.Add(this);

	TArray<const URoadPathFollowerComponent*> Frontier;
	Frontier.Add(this);

	const int32 MaxRearDepth = FMath::Max(MaxChainWalkDepth, 10);
	int32 Depth = 0;

	while (Frontier.Num() > 0 && Depth < MaxRearDepth)
	{
		TArray<const URoadPathFollowerComponent*> NextFrontier;
		for (const URoadPathFollowerComponent* Cur : Frontier)
		{
			if (!Cur) continue;
			AActor* CurActor = Cur->GetOwner();
			if (!CurActor) continue;

			for (TObjectIterator<URoadPathFollowerComponent> It; It; ++It)
			{
				URoadPathFollowerComponent* Other = *It;
				if (!Other || Other == Cur) continue;
				if (Visited.Contains(Other)) continue;
				if (Other->GetWorld() != W) continue;
				if (!Other->ObstacleActor.IsValid()) continue;
				if (Other->ObstacleActor.Get() != CurActor) continue;
				if (Other->GetCurrentSpeed() >= 5.0f) continue;  // only stopped cars count

				Visited.Add(Other);
				OutChain.Add(Other);
				NextFrontier.Add(Other);
			}
		}
		Frontier = MoveTemp(NextFrontier);
		Depth++;
	}
	return OutChain.Num();
}

// ============================================================================
//  TriggerStuckReverse
// ============================================================================
void URoadPathFollowerComponent::TriggerStuckReverse(
	float OverrideReverseDistance,
	const TCHAR* Reason)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;
	if (bIsStuckReversing) return;   // already reversing

	if (TimeSinceLastStuck < ConsecutiveStuckWindow)
		ConsecutiveStuckCount++;
	else
		ConsecutiveStuckCount = 1;
	TimeSinceLastStuck = 0.0f;

	CurrentStuckReverseTarget = (OverrideReverseDistance > 0.0f)
		? OverrideReverseDistance
		: StuckReverseDistance * (1.0f + (ConsecutiveStuckCount - 1) * 0.5f);

	LogEvent(FString::Printf(TEXT("REVERSE %.0fcm reason=%s consec=#%d"),
		CurrentStuckReverseTarget,
		Reason ? Reason : TEXT(""),
		ConsecutiveStuckCount));

	bIsStuckReversing = true;
	StuckReversedSoFar = 0.0f;
	CurrentSpeed = 0.0f;
	StuckTimer = 0.0f;
	bMutualStuckResolved = false;
}

// ============================================================================
//  Detection flow:
//  1. Cooldown: right after reverse complete, blocked for cooldown period
//  2. Currently reversing: keep moving back until target distance
//  3. Detect stuck: my speed≈0 + vehicle/obstacle very close → accumulate StuckTimer
//  4. Trigger (checked in order):
//     a. Front already reversing → I yield, reset my timer
//     b. Front still moving (>5 cm/s) → just temporarily blocked, wait
//     c. Front also stuck → FName tiebreak (alphabetically smaller yields)
//     d. Static obstacle (red light, barrier) → wait 3× delay before trigger
//  5. After reverse:
//     a. Increment ConsecutiveStuckCount
//     b. If stuck N times in a row → switch to random parking destination
//     c. Otherwise re-route to same destination
//     d. Set StuckCooldownTimer
// ============================================================================
void URoadPathFollowerComponent::UpdateStuckRecovery(float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	// Update "time since last stuck"
	TimeSinceLastStuck += DeltaTime;

	// ================================================================
	// PRE-PHASE: ZERO-DIST TELEPORT SAFETY VALVE
	// Runs UNCONDITIONALLY before Phase 0/2 early-returns.
	//
	{
		// ---- Post-teleport grace period ----
		// After any ZERO-DIST teleport we suppress ZeroDistStuckTimer for
		// ZeroDistTeleportGraceDuration seconds.  Without this, if the
		// destination already has another car at 0 cm, the timer fires
		// again immediately → infinite teleport loop every 1 second.
		if (ZeroDistTeleportGraceTimer > 0.0f)
		{
			ZeroDistTeleportGraceTimer -= DeltaTime;
			ZeroDistStuckTimer = 0.0f;
			ZeroDistClearTimer = 0.0f;
			// PRE-PHASE done for this tick — fall through to Phase 0
		}
		else
		{
			const bool bAtZeroDist =
				(ObstacleDistance >= 0.0f && ObstacleDistance < EmergencyOverlapDistance)
				&& !IsChainHeadBlockedBySignal();

			if (bAtZeroDist)
			{
				ZeroDistStuckTimer += DeltaTime;
				ZeroDistClearTimer  = 0.0f;  // reset clear timer

				if (ZeroDistStuckTimer >= ZeroDistTeleportTimeout)
				{
					ZeroDistStuckTimer = 0.0f;
					ZeroDistClearTimer = 0.0f;

					const FVector TeleportDest = DestinationWorldLocation;

					LogEvent(FString::Printf(
						TEXT("ZERO-DIST-TELEPORT → dest=(%.0f,%.0f,%.0f)"),
						TeleportDest.X, TeleportDest.Y, TeleportDest.Z));

					// ── Teleport to the ACTUAL PARKING SPOT (not the road node) ──────
					// Placing the car at the spot itself guarantees:
					//   • Each car lands at its own unique, reserved spot → no two cars
					//     share the same destination world position.
					//   • Navigation is fully cleared, so there is no old spline path
					//     that could drag the car back.
					//   • After ZeroDistTeleportGraceDuration seconds the car picks a
					//     DIFFERENT random destination via SwitchToNewRandomDestination(),
					//     ensuring it never teleports to the same place twice.
					if (DestinationType == EDestinationType::ParkingLot
						&& TargetParkingLot.IsValid()
						&& TargetParkingSpotIndex != INDEX_NONE)
					{
						const FVector SpotPos = TargetParkingLot->GetSpotWorldPosition(TargetParkingSpotIndex);
						const FVector SpotFwd = TargetParkingLot->GetSpotWorldForward(TargetParkingSpotIndex);
						Owner->SetActorLocationAndRotation(
							SpotPos, SpotFwd.ToOrientationRotator(),
							false, nullptr, ETeleportType::TeleportPhysics);
					}
					else
					{
						// Fallback (no valid spot): just move to destination node position
						Owner->SetActorLocation(TeleportDest, false, nullptr, ETeleportType::TeleportPhysics);
					}

					// Park the car in place — wipe ALL navigation/curve/stuck state.
					// Spot occupancy (TargetParkingLot/TargetParkingSpotIndex) is kept so
					// SwitchToNewRandomDestination() can release it cleanly later.
					ResetFollowingState();
					NavState = ENavState::Parked;   // ResetFollowingState leaves NavState=Idle

					// After grace duration, switch to a brand-new random destination.
					// Using a timer handle so the countdown keeps running while parked
					// (bIsFollowing=false → UpdateStuckRecovery is skipped in Tick).
					if (UWorld* W = GetWorld())
					{
						W->GetTimerManager().ClearTimer(ZeroDistReNavigateTimerHandle);
						W->GetTimerManager().SetTimer(
							ZeroDistReNavigateTimerHandle,
							this, &URoadPathFollowerComponent::ZeroDistReNavigateCallback,
							ZeroDistTeleportGraceDuration, false);
					}

					return;
				}
			}
			else if (ZeroDistStuckTimer > 0.0f)
			{
				// Not at 0cm but was stuck before: accumulate clear time
				ZeroDistClearTimer += DeltaTime;
				if (ZeroDistClearTimer >= ZeroDistClearTimeout)
				{
					// Genuinely free (5s clear) → reset
					ZeroDistStuckTimer = 0.0f;
					ZeroDistClearTimer = 0.0f;
				}
			}
		}
	}

	// ================================================================
	// Phase 0： Cooldown period after reverse
	// ================================================================
	if (StuckCooldownTimer > 0.0f)
	{
		StuckCooldownTimer -= DeltaTime;
		StuckTimer = 0.0f;  //don't accumulate during cooldown
		return;
	}

	// ================================================================
	// Phase 2：Currently reversing
	// ================================================================
	if (bIsStuckReversing)
	{
		// Rear-clear gate — only reverse if no vehicle is right behind.
		// Cascades tail-first naturally: the car with no one behind moves first.
		// Safety valve: if we've been blocked too long (both chains stuck), reverse anyway.
		AActor* RearVehicle = nullptr;
		if (!IsRearClearOfVehicles(RearVehicle))
		{
			// ---- Classify rear vehicle relationship ----
			bool bRearShouldBlock = true;

			if (RearVehicle)
			{
				const FVector MyFwd2D   = Owner->GetActorForwardVector().GetSafeNormal2D();
				const FVector RearFwd2D = RearVehicle->GetActorForwardVector().GetSafeNormal2D();
				const float   RearDirDot = FVector::DotProduct(MyFwd2D, RearFwd2D);
				const float   RearDist   = FVector::Dist2D(
					Owner->GetActorLocation(), RearVehicle->GetActorLocation());

				const bool bRearSameDir = (RearDirDot > 0.5f);
				const bool bRearSameDirNormalDist = bRearSameDir
					&& (RearDist >= OverlapSeparationThreshold);

				if (bRearSameDirNormalDist)
				{
					// Same-dir normal distance: normal following scenario.
					// Rear car travels same direction — my reversing won't
					// cause a head-on collision. Don't let it block me.
					bRearShouldBlock = false;

					LogEvent(FString::Printf(
						TEXT("REAR-CLEAR SKIP: same-dir rear=%s dot=%.2f dist=%.0fcm → not blocking"),
						*RearVehicle->GetName(), RearDirDot, RearDist));
				}
				// Other cases (opposite-dir or same-dir overlap) → normal wait logic
			}

			if (bRearShouldBlock)
			{
				const float PrevAccum = RearWaitAccum;
				RearWaitAccum += DeltaTime;
				if (RearWaitAccum < RearWaitMaxSec)
				{
					CurrentSpeed = 0.0f;
					if (PrevAccum <= 0.05f)
					{
						LogEvent(FString::Printf(TEXT("REVERSE BLOCKED by rear=%s (waiting...)"),
							RearVehicle ? *RearVehicle->GetName() : TEXT("?")));
					}
					return;
				}
				if (PrevAccum < RearWaitMaxSec)
				{
					LogEvent(FString::Printf(TEXT("REVERSE FORCED (rear-wait %.1fs timeout, rear=%s)"),
						RearWaitAccum,
						RearVehicle ? *RearVehicle->GetName() : TEXT("?")));
				}
			}
			else
			{
				// Same-dir normal rear — no block, clear accum and proceed
				RearWaitAccum = 0.0f;
			}
		}
		else
		{
			RearWaitAccum = 0.0f;
		}

		const float ReverseDelta = StuckReverseSpeed * DeltaTime;
		StuckReversedSoFar += ReverseDelta;

		// Move backward along vehicle facing
		const FVector BackDir = -Owner->GetActorForwardVector();
		Owner->SetActorLocation(Owner->GetActorLocation() + BackDir * ReverseDelta);

		// Also reverse ReferenceDistance on spline
		if (PathSegments.IsValidIndex(CurrentSegmentIndex))
		{
			const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
			ReferenceDistance -= Seg.Direction * ReverseDelta;
			const float SegLo = FMath::Min(Seg.StartDist, Seg.EndDist);
			const float SegHi = FMath::Max(Seg.StartDist, Seg.EndDist);
			ReferenceDistance = FMath::Clamp(ReferenceDistance, SegLo, SegHi);
		}

		//  Reverse complete?
		if (StuckReversedSoFar >= CurrentStuckReverseTarget)
		{
			bIsStuckReversing = false;
			StuckReversedSoFar = 0.0f;
			StuckTimer = 0.0f;
			CurrentSpeed = 0.0f;

			//  Set cooldown
			StuckCooldownTimer = StuckCooldownDuration;

			UE_LOG(LogTemp, Warning, TEXT("[STUCK] %s: Reverse complete (%.0f cm), cooldown=%.1fs, consecutive=%d"),
				*Owner->GetName(), CurrentStuckReverseTarget, StuckCooldownDuration, ConsecutiveStuckCount);
			LogEvent(FString::Printf(TEXT("STUCK reverse COMPLETE (%.0fcm, consec=%d)"),
				CurrentStuckReverseTarget, ConsecutiveStuckCount));

			// Too many consecutive stucks → teleport to current target spot, park,
			// then pick a brand-new destination after the grace period.
			// Skip teleport if the entire chain is only waiting at a red light.
			if (ConsecutiveStuckCount >= MaxConsecutiveStucksBeforeSwitch
				&& !IsChainHeadBlockedBySignal())
			{
				ConsecutiveStuckCount = 0;
				CurrentStuckReverseTarget = StuckReverseDistance;

				bIsStuckReversing     = false;
				StuckReversedSoFar    = 0.0f;
				StuckTimer            = 0.0f;
				StuckCooldownTimer    = 0.0f;
				bMutualStuckResolved  = false;
				RearWaitAccum         = 0.0f;
				ZeroDistStuckTimer    = 0.0f;
				ZeroDistClearTimer    = 0.0f;

				// Teleport to the ACTUAL PARKING SPOT so there is no ambiguity
				// about where the car lands. Each car lands at its own reserved spot.
				if (DestinationType == EDestinationType::ParkingLot
					&& TargetParkingLot.IsValid()
					&& TargetParkingSpotIndex != INDEX_NONE)
				{
					const FVector SpotPos = TargetParkingLot->GetSpotWorldPosition(TargetParkingSpotIndex);
					const FVector SpotFwd = TargetParkingLot->GetSpotWorldForward(TargetParkingSpotIndex);
					Owner->SetActorLocationAndRotation(
						SpotPos, SpotFwd.ToOrientationRotator(),
						false, nullptr, ETeleportType::TeleportPhysics);

					LogEvent(FString::Printf(
						TEXT("CONSEC-TELEPORT-TO-SPOT → lot='%s' spot=%d"),
						*TargetParkingLot->ParkingLotName, TargetParkingSpotIndex));
				}
				else
				{
					// Fallback: no reserved spot — move to destination node position
					Owner->SetActorLocation(DestinationWorldLocation, false, nullptr, ETeleportType::TeleportPhysics);
					LogEvent(TEXT("CONSEC-TELEPORT → dest node (no spot)"));
				}

				// Park the car — clear all path/curve/stuck state.
				// Spot occupancy is preserved so SwitchToNewRandomDestination() can
				// release it cleanly when the timer fires.
				ResetFollowingState();
				NavState = ENavState::Parked;

				// After grace duration, switch to a new random destination.
				if (UWorld* W = GetWorld())
				{
					W->GetTimerManager().ClearTimer(ZeroDistReNavigateTimerHandle);
					W->GetTimerManager().SetTimer(
						ZeroDistReNavigateTimerHandle,
						this, &URoadPathFollowerComponent::ZeroDistReNavigateCallback,
						ZeroDistTeleportGraceDuration, false);
				}

				return;
			}

			// ---- Re-navigate to same destination ----
			// Car is still in Driving state, so NavigateToNode will take the RerouteToNode path.
			// For parking lot destinations, also re-append the BoundEdge final segment.
			if (bRerouteAfterStuckRecovery && DestinationNodeId != INDEX_NONE)
			{
				UE_LOG(LogTemp, Warning, TEXT("[STUCK] %s: Re-navigating to node %d (dest='%s')"),
					*Owner->GetName(), DestinationNodeId,
					TargetParkingLot.IsValid() ? *TargetParkingLot->ParkingLotName : TEXT("node"));

				NavigateToNode(DestinationNodeId);

				// Parking lot destination: re-append BoundEdge final segment
				if (TargetParkingLot.IsValid() && DestinationType == EDestinationType::ParkingLot)
				{
					UWorld* World = GetWorld();
					URoadNetworkSubsystem* Sub = World ? World->GetSubsystem<URoadNetworkSubsystem>() : nullptr;
					if (Sub)
					{
						const FRoadGraphEdge* BoundEdge = Sub->GetGraphEdgeById(TargetParkingLot->BoundEdgeId);
						if (BoundEdge && BoundEdge->InputSpline)
						{
							// Compute direction and projection distance (same logic as NavigateToParkingLot)
							const FVector AnchorFwd = TargetParkingLot->GetAnchorArrowForward();
							const FVector AnchorFwd2D = FVector(AnchorFwd.X, AnchorFwd.Y, 0.0f).GetSafeNormal();
							const FVector& ProjPt = TargetParkingLot->BoundProjectionPoint;
							const float ProjKey = BoundEdge->InputSpline->FindInputKeyClosestToWorldLocation(ProjPt);
							const float ProjDistAbs = BoundEdge->InputSpline->GetDistanceAlongSplineAtSplineInputKey(ProjKey);
							const FVector SplineDir = BoundEdge->InputSpline->GetDirectionAtDistanceAlongSpline(
								ProjDistAbs, ESplineCoordinateSpace::World).GetSafeNormal2D();
							const bool bForwardDrive = (FVector::DotProduct(AnchorFwd2D, SplineDir) >= 0.0f);

							AppendBoundEdgeFinalSegment(*BoundEdge, bForwardDrive, ProjDistAbs);
						}
					}
				}
			}
		}
		return;
	}

	// ================================================================
	// Phase 1：Detect stuck condition
	// ================================================================
	const bool bAmStopped = (CurrentSpeed < 5.0f);
	const float EffectiveStuckDist = FMath::Max(StuckDetectDistance, OverlapDetectionDistance);
	const bool bObstacleInStuckRange = (ObstacleDistance >= 0.0f
		&& ObstacleDistance < EffectiveStuckDist);

	// Pre-filter: front must be a VEHICLE. Static/mesh/None blockers don't go
	// through the stuck chain/mutual logic at all (they get the dedicated
	// "fall-through reverse" path far below, after StuckRecoveryDelay*3).
	const EFrontObstacleType FrontTypeEarly = bObstacleInStuckRange
		? ClassifyFrontObstacle() : EFrontObstacleType::None;
	const bool bFrontIsVehicle = (FrontTypeEarly == EFrontObstacleType::Vehicle);

	if (bAmStopped && bObstacleInStuckRange && ObstacleActor.IsValid() && bFrontIsVehicle)
	{
		const float PrevTimer = StuckTimer;
		StuckTimer += DeltaTime;

		// STUCK-ACCUM start event (vehicle-only — no static spam)
		if (PrevTimer < 0.1f && StuckTimer >= 0.1f)
		{
			LogEvent(FString::Printf(
				TEXT("STUCK-ACCUM start: blocked by %s @ %.0fcm"),
				*ObstacleActor->GetName(), ObstacleDistance));
		}

		// Zero-dist teleport handled by PRE-PHASE block above.

		// ================================================================
		// TOP PRIORITY — EMERGENCY OVERLAP (Obstacle: ~0 cm against Vehicle).
		// Physically touching another car is the most severe stuck state.
		// Fires IMMEDIATELY, bypassing mutual / cycle / chain-leader / delay.
		//
		// Three sub-cases:
		//   (A)  (dot < -0.5) — mutual
		//       Opposite-facing overlap — mutual collision, both need to separate.
		//       → Use longitudinal projection: RC in front → RC retreats
		//   (B) (dot > 0.5, ActualDist < OverlapSeparationThreshold)
		//       Same-dir overlap at extremely close range — teleport / spawn coincidence.
		//       → Rearmost car reverses, front car stays
		//   (C) (dot > 0.5, ActualDist >= OverlapSeparationThreshold)
		//       Same-dir normal following — front car blocks, NOT an overlap.
		//       → Skip this path; let normal chain/mutual logic handle it
		// ================================================================
		if (!bIsStuckReversing && StuckCooldownTimer <= 0.0f
			&& ObstacleDistance < EmergencyOverlapDistance)
		{
			AActor* FA_Emerg = ObstacleActor.Get();
			URoadPathFollowerComponent* FPF_Emerg = FA_Emerg
				? FA_Emerg->FindComponentByClass<URoadPathFollowerComponent>() : nullptr;

			const bool bFrontAlreadyHandling = FPF_Emerg
				&& (FPF_Emerg->IsStuckReversing() || FPF_Emerg->StuckCooldownTimer > 0.0f);

			if (!bFrontAlreadyHandling && FPF_Emerg)
			{
				// ---- Compute directional relationship and actual distance ----
				const FVector MyFwd  = Owner->GetActorForwardVector().GetSafeNormal2D();
				const FVector HisFwd = FA_Emerg->GetActorForwardVector().GetSafeNormal2D();
				const float   DirDot = FVector::DotProduct(MyFwd, HisFwd);

				const float ActualDist = FVector::Dist2D(
					Owner->GetActorLocation(), FA_Emerg->GetActorLocation());

				// ---- Longitudinal projection: Delta·MyFwd > 0 means he's ahead ----
				const FVector Delta2D = (FA_Emerg->GetActorLocation()
					- Owner->GetActorLocation()).GetSafeNormal2D();
				const float LongDot = FVector::DotProduct(Delta2D, MyFwd);

				// ----  Sub-case classification ----
				//   DirDot < -0.5  →  (opposite-facing)
				//   DirDot >  0.5  →  (same-facing)
				//   otherwise      
				const bool bOpposite = (DirDot < -0.5f);
				const bool bSameDir  = (DirDot >  0.5f);
				const bool bSameDirOverlap = bSameDir
					&& (ActualDist < OverlapSeparationThreshold);
				const bool bSameDirNormal  = bSameDir
					&& (ActualDist >= OverlapSeparationThreshold);

				if (bSameDirNormal)
				{
					// Case (C): same-dir normal following — not an overlap, skip
					goto AfterEmergencyOverlap;
				}

				// ---- Case (A) opposite or (B) same-dir overlap ----

				// Decide who reverses:
				//   Opposite (A): LongDot > 0 = he's ahead → he reverses (clears forward space)
				//                 LongDot < 0 = he's behind (abnormal) → I reverse
				//   Same-dir overlap (B): rearmost car reverses
				//                 LongDot > 0 = he's ahead → I'm behind → I reverse
				//                 LongDot < 0 = he's behind → he reverses
				bool bIShouldReverse;
				if (bOpposite)
				{
					// Opposite: he's ahead → he reverses; he's behind → I reverse
					bIShouldReverse = (LongDot <= 0.0f);
				}
				else
				{
					// Same-dir overlap: I'm behind (LongDot > 0) → I reverse
					bIShouldReverse = (LongDot > 0.0f);
				}

				if (!bIShouldReverse)
				{
					bMutualStuckResolved = true;

					LogEvent(FString::Printf(
						TEXT("OVERLAP-EMERG: me[%s] yield, %s[dot=%.2f,long=%.2f] reverses"),
						*GetManeuverLabel(), *FA_Emerg->GetName(), DirDot, LongDot));

					goto AfterEmergencyOverlap;
				}

				// ---- I should reverse ----

				// Rear chain cascade:
				//   Opposite (A): cascade all rear cars (same logic, unpack the whole queue)
				//   Same-dir overlap (B): only cascade same-direction rear cars;
				//                         skip opposite-direction rear cars
				TArray<URoadPathFollowerComponent*> MyRear;
				const int32 MyRearN = CountStoppedRearChain(MyRear);

				LogEvent(FString::Printf(
					TEXT("OVERLAP-EMERG[%s]: me[%s] @ %.0fcm vs %s (rearN=%d) → I reverse"),
					bOpposite ? TEXT("OPP") : TEXT("SAME"),
					*GetManeuverLabel(), ObstacleDistance,
					*FA_Emerg->GetName(), MyRearN));

				bMutualStuckResolved = true;
				TriggerStuckReverse(-1.0f, TEXT("overlap-emergency"));

				// Same-dir overlap: nudge the front car forward so both cars
				// separate immediately (one forward, one backward).
				if (bSameDirOverlap && FPF_Emerg)
				{
					const FVector FrontFwd = FA_Emerg->GetActorForwardVector().GetSafeNormal();
					FA_Emerg->SetActorLocation(
						FA_Emerg->GetActorLocation() + FrontFwd * OverlapSeparationThreshold,
						false, nullptr, ETeleportType::TeleportPhysics);
					FPF_Emerg->ZeroDistStuckTimer = 0.0f;
					FPF_Emerg->StuckTimer = 0.0f;

				}

				for (URoadPathFollowerComponent* RC : MyRear)
				{
					if (!RC || RC->IsStuckReversing()) continue;

					if (bSameDirOverlap)
					{
						// Same-dir overlap cascade: only bring same-direction rear cars
						const AActor* RCOwner = RC->GetOwner();
						if (!RCOwner) continue;
						const FVector RCFwd = RCOwner->GetActorForwardVector().GetSafeNormal2D();
						const float RCDirDot = FVector::DotProduct(MyFwd, RCFwd);
						if (RCDirDot <= 0.5f)
						{
							// Non-same-dir rear car — skip
							continue;
						}
					}
					RC->TriggerStuckReverse(-1.0f, TEXT("overlap-emergency-cascade"));
				}
				return;
			}
		}
		AfterEmergencyOverlap:;

		// ============================================================
		// MUTUAL-STUCK fast-path — fires IMMEDIATELY (no delay wait)
		// when A.front = B AND B.front = A. This is the only case that
		// *must* be resolved this update. Chooses reverser by rear-chain
		// length — shorter side reverses (dragging its whole chain).
		// ============================================================
		AActor* FA_Early = ObstacleActor.Get();
		URoadPathFollowerComponent* FPF_Early = FA_Early
			? FA_Early->FindComponentByClass<URoadPathFollowerComponent>() : nullptr;

		const bool bFrontAlsoStopped_Early = FPF_Early && (FPF_Early->GetCurrentSpeed() < 5.0f);
		const bool bMutualPair =
			FPF_Early
			&& bFrontAlsoStopped_Early
			&& FPF_Early->ObstacleActor.IsValid()
			&& FPF_Early->ObstacleActor.Get() == Owner;

		if (bMutualPair && !bMutualStuckResolved && !FPF_Early->bMutualStuckResolved
			&& !bIsStuckReversing && !FPF_Early->IsStuckReversing())
		{
			// Both sides mark themselves resolved so the resolver only runs once.
			bMutualStuckResolved = true;
			FPF_Early->bMutualStuckResolved = true;

			// Rear chain counts on each side
			TArray<URoadPathFollowerComponent*> MyRear, FrontRear;
			const int32 MyRearN   = CountStoppedRearChain(MyRear);
			const int32 FrontRearN = FPF_Early->CountStoppedRearChain(FrontRear);

			// Labels for log
			const FString MyMan    = GetManeuverLabel();
			const FString FrontMan = FPF_Early->GetManeuverLabel();

			// Decide reverser: shorter rear wins. Tie → FName for stability.
			const bool bIReverse = (MyRearN < FrontRearN)
				|| (MyRearN == FrontRearN
					&& Owner->GetName().Compare(FA_Early->GetName()) < 0);

			URoadPathFollowerComponent* Reverser = bIReverse ? this : FPF_Early;
			TArray<URoadPathFollowerComponent*>& ReverserChain = bIReverse ? MyRear : FrontRear;
			const int32 ReverserRear = bIReverse ? MyRearN : FrontRearN;

			// One giant log with every piece of info you asked for

			LogEvent(FString::Printf(
				TEXT("MUTUAL: me=%s[%s,r=%d] vs %s[%s,r=%d] → %s reverses"),
				*Owner->GetName(), *MyMan, MyRearN,
				*FA_Early->GetName(), *FrontMan, FrontRearN,
				*Reverser->GetOwner()->GetName()));

			// Kick the winner and its whole rear chain.
			Reverser->TriggerStuckReverse(-1.0f, TEXT("mutual-shorter-rear"));
			for (URoadPathFollowerComponent* RC : ReverserChain)
			{
				if (RC && !RC->IsStuckReversing())
				{
					RC->TriggerStuckReverse(-1.0f, TEXT("mutual-rear-cascade"));
				}
			}
			return;
		}

		// ============================================================
		// N-CYCLE fast-path — 3+ cars going A→B→C→A round-and-round at
		// a junction. MUTUAL above only catches the 2-car case. We detect
		// the cycle here and fire the same single-resolver pattern.
		// Condition: my forward walk returns to me (cycle), every member
		// is stopped, and no one in the cycle is already reversing.
		// ============================================================
		if (!bIsStuckReversing && !bMutualStuckResolved)
		{
			TArray<URoadPathFollowerComponent*> CycleGroup;
			FString CycleReason;
			URoadPathFollowerComponent* CycleRev = FindChainReverser(CycleGroup, CycleReason);

			const bool bIsCycle = CycleReason.StartsWith(TEXT("cycle-N="))
				&& CycleGroup.Num() >= 3 && CycleRev;

			if (bIsCycle)
			{
				// Verify entire cycle is inert (stopped, not reversing, not cooling).
				bool bAllInert = true;
				for (URoadPathFollowerComponent* C : CycleGroup)
				{
					if (!C) { bAllInert = false; break; }
					if (C->GetCurrentSpeed() >= 5.0f) { bAllInert = false; break; }
					if (C->IsStuckReversing())       { bAllInert = false; break; }
					if (C->StuckCooldownTimer > 0.0f){ bAllInert = false; break; }
					if (C->bMutualStuckResolved)     { bAllInert = false; break; }
				}

				if (bAllInert)
				{
					// Mark every cycle member resolved so the resolver runs once.
					for (URoadPathFollowerComponent* C : CycleGroup)
					{
						if (C) C->bMutualStuckResolved = true;
					}

					// Rear chain for the chosen reverser (drag followers with it).
					TArray<URoadPathFollowerComponent*> RevRear;
					const int32 RevRearN = CycleRev->CountStoppedRearChain(RevRear);

					// Build a compact member-list for the log (maneuver labels).
					FString Members;
					for (int32 i = 0; i < CycleGroup.Num(); ++i)
					{
						if (i > 0) Members += TEXT(" → ");
						Members += FString::Printf(TEXT("%s[%s]"),
							*CycleGroup[i]->GetOwner()->GetName(),
							*CycleGroup[i]->GetManeuverLabel());
					}
					LogEvent(FString::Printf(
						TEXT("N-CYCLE(%d): %s → %s reverses"),
						CycleGroup.Num(), *Members,
						*CycleRev->GetOwner()->GetName()));

					CycleRev->TriggerStuckReverse(-1.0f, TEXT("n-cycle-smallest-name"));
					for (URoadPathFollowerComponent* RC : RevRear)
					{
						if (RC && !RC->IsStuckReversing())
						{
							RC->TriggerStuckReverse(-1.0f, TEXT("n-cycle-rear-cascade"));
						}
					}
					return;
				}
			}
		}

		if (StuckTimer >= StuckRecoveryDelay)
		{
			// Chain-aware classification:
			// Only the chain leader decides whether reversing is the right move.
			// Non-leaders wait (they'll reverse later via the rear-clear cascade
			// once the leader reverses and the cars behind him do too).
			auto ThrottledYieldLog = [&](const TCHAR* Reason)
			{
				const float NowT = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;
				const FString RStr(Reason);
				if (RStr != LastYieldReason || (NowT - LastYieldLogTime) >= 3.0f)
				{
					LastYieldReason = RStr;
					LastYieldLogTime = NowT;
					LogEvent(FString::Printf(TEXT("YIELD: %s (front=%s)"),
						Reason,
						ObstacleActor.IsValid() ? *ObstacleActor->GetName() : TEXT("?")));
				}
			};

			// ============================================================
			// Chain resolver: walk forward, find the car that should reverse.
			// FindChainReverser handles:
			//   * Pure cycles (N≥3) — but those are typically caught earlier
			//     by the N-CYCLE fast-path. If they reach here it's because
			//     the cycle wasn't inert (someone moved briefly) — re-check.
			//   * Zombie-head chains — head has no ObstacleActor itself (so
			//     Phase 1 never runs for IT), but is stopped. Followers must
			//     trigger it externally.
			//   * Moving front / red-light — returns nullptr (we yield).
			// ============================================================
			TArray<URoadPathFollowerComponent*> ChainGroup;
			FString ChainReason;
			URoadPathFollowerComponent* Reverser = FindChainReverser(ChainGroup, ChainReason);

			if (!Reverser)
			{
				// Legitimate wait — front moving, red light, someone already
				// reversing, etc. Keep the timer primed so we react quickly
				// once the situation resolves.
				ThrottledYieldLog(*FString::Printf(TEXT("no-reverser (%s)"), *ChainReason));
				StuckTimer = FMath::Min(StuckTimer, StuckRecoveryDelay);
				return;
			}

			if (Reverser != this)
			{
				// Someone else in the chain is the designated reverser. Two
				// subcases:
				//   (A) Reverser is in Phase 1 itself → it will trigger on
				//       its own timer. I just yield and wait for the cascade.
				//   (B) Reverser is a "zombie head" that can't self-trigger
				//       (no ObstacleActor, or front is static). Its own
				//       Phase 1 gate rejects it, so I must trigger it from
				//       HERE and cascade its rear chain (which includes me).
				// ChainReason tells us which branch.
				const bool bZombie = ChainReason.StartsWith(TEXT("zombie-head-"));

				if (bZombie && !Reverser->IsStuckReversing()
					&& Reverser->StuckCooldownTimer <= 0.0f)
				{
					TArray<URoadPathFollowerComponent*> RevRear;
					const int32 RevRearN = Reverser->CountStoppedRearChain(RevRear);

					LogEvent(FString::Printf(
						TEXT("ZOMBIE-HEAD: %s → %s [%s,r=%d] (%s)"),
						*Owner->GetName(),
						*Reverser->GetOwner()->GetName(),
						*Reverser->GetManeuverLabel(),
						RevRearN,
						*ChainReason));

					Reverser->TriggerStuckReverse(-1.0f, TEXT("zombie-head-ext-trigger"));
					for (URoadPathFollowerComponent* RC : RevRear)
					{
						if (RC && !RC->IsStuckReversing())
						{
							RC->TriggerStuckReverse(-1.0f, TEXT("zombie-head-cascade"));
						}
					}
					return;
				}

				// (A) Reverser will handle itself — just yield.
				ThrottledYieldLog(*FString::Printf(
					TEXT("yield to reverser=%s (%s)"),
					*Reverser->GetOwner()->GetName(), *ChainReason));
				StuckTimer = FMath::Min(StuckTimer, StuckRecoveryDelay);
				return;
			}

			// Reverser == this → I commit to reversing (logic below).

			const EFrontObstacleType FrontType = ClassifyFrontObstacle();

			// Traffic signal (red light) → NEVER reverse, just wait it out.
			if (FrontType == EFrontObstacleType::TrafficSignal)
			{
				ThrottledYieldLog(TEXT("traffic signal"));
				StuckTimer = 0.0f;
				return;
			}

			URoadPathFollowerComponent* FrontPF =
				(FrontType == EFrontObstacleType::Vehicle && ObstacleActor.IsValid())
				? ObstacleActor->FindComponentByClass<URoadPathFollowerComponent>()
				: nullptr;

			if (FrontType == EFrontObstacleType::Static || FrontType == EFrontObstacleType::None)
			{
				// Static obstacle (wall, barrier) → leader reverses itself to escape.
				// Extra delay before committing so we don't react to transient hits.
				if (StuckTimer < StuckRecoveryDelay * 3.0f)
				{
					ThrottledYieldLog(TEXT("static obstacle (waiting 3× delay)"));
					return;
				}
				// Fall through to reverse-trigger.
			}
			else if (FrontPF)
			{
				// ========================================================
				// Front obstacle is a vehicle — 4-tier priority check
				// ========================================================

				// --- Tier 1: front car already reversing → I absolutely yield ---
				// (Critical fix: prevent both cars triggering reverse in same frame)
				if (FrontPF->IsStuckReversing())
				{
					ThrottledYieldLog(TEXT("front car reversing"));
					StuckTimer = 0.0f;  // reset so I don't re-accumulate
					return;
				}

				// --- Tier 2: front car still moving → just temporarily blocked ---
				const bool bFrontAlsoStopped = (FrontPF->GetCurrentSpeed() < 5.0f);
				if (!bFrontAlsoStopped)
				{
					ThrottledYieldLog(TEXT("front moving"));
					return;
				}

				// --- Tier 3: front in cooldown → it just reversed, I yield ---
				if (FrontPF->StuckCooldownTimer > 0.0f)
				{
					ThrottledYieldLog(TEXT("front in cooldown"));
					StuckTimer = 0.0f;
					return;
				}

				// --- Tier 4 removed: mutual-stuck (A↔B) is handled by the
				// fast-path resolver above (rear-chain length picks reverser).
				// For longer chains, IsChainLeader already decided I'm the
				// head; just commit to reversing. No more FName tiebreak
				// contradiction with the chain-leader walk.
			}

			// ============================================================
			// Trigger reverse
			// ============================================================

			// Chain leader committing to reverse (3+ car chains — 2-car mutual
			// is already handled by the fast-path resolver above). Delegate to
			// the shared TriggerStuckReverse helper; also attempt to drag our
			// rear chain along so the whole stuck queue clears together.
			TArray<URoadPathFollowerComponent*> MyRear;
			const int32 MyRearN = CountStoppedRearChain(MyRear);
			const FString ReasonStr = FString::Printf(
				TEXT("chain-leader [%s] rearN=%d"), *GetManeuverLabel(), MyRearN);
			TriggerStuckReverse(-1.0f, *ReasonStr);
			for (URoadPathFollowerComponent* RC : MyRear)
			{
				if (RC && !RC->IsStuckReversing())
				{
					RC->TriggerStuckReverse(-1.0f, TEXT("leader-cascade"));
				}
			}
		}
	}
	else
	{
		// Not stuck → reset timer
		if (StuckTimer > 0.0f)
		{
			StuckTimer = FMath::Max(StuckTimer - DeltaTime * 2.0f, 0.0f);  // decay
		}
		// Car is moving again (or no vehicle front) — clear mutual-stuck flag
		// so a future A↔B lock can trigger the resolver again.
		if (CurrentSpeed >= 5.0f)
		{
			bMutualStuckResolved = false;
		}
	}

	// ZombieStuckTimer: accumulates only when stopped with NO obstacle at all.
	// Resets immediately the moment any ObstacleActor appears (red light, car, wall).
	// This prevents a car that just lost its TrafficSignal actor for one throttle
	// interval from being misclassified as a zombie head by the chain walker.
	if (ObstacleActor.IsValid())
	{
		ZombieStuckTimer = 0.0f;
	}
	else if (bAmStopped)
	{
		ZombieStuckTimer += DeltaTime;
	}
	else
	{
		ZombieStuckTimer = FMath::Max(ZombieStuckTimer - DeltaTime * 2.0f, 0.0f);
	}

	// ================================================================
	// Static-obstacle silent escape: stopped against a wall/mesh for a
	// long time → reverse. No Recent-Events spam for this path.
	// ================================================================
	if (bAmStopped && bObstacleInStuckRange && ObstacleActor.IsValid()
		&& FrontTypeEarly == EFrontObstacleType::Static
		&& !bIsStuckReversing)
	{
		StaticStuckTimer += DeltaTime;
		if (StaticStuckTimer >= StuckRecoveryDelay * 3.0f)
		{
			TriggerStuckReverse(-1.0f, TEXT("static-escape"));
			StaticStuckTimer = 0.0f;
		}
	}
	else if (StaticStuckTimer > 0.0f)
	{
		StaticStuckTimer = FMath::Max(StaticStuckTimer - DeltaTime * 2.0f, 0.0f);
	}
}

// ============================================================================
//  Check if passing lane is clear using SphereTrace.
// ============================================================================
bool URoadPathFollowerComponent::IsPassingLaneClear(int32 PassingLaneIndex) const
{
	UWorld* World = GetWorld();
	if (!World) return false;

	AActor* Owner = GetOwner();
	if (!Owner) return false;

	if (CurrentSegmentIndex >= PathSegments.Num()) return false;

	//  Compute passing lane offset
	const float PassingOffset = ComputeTargetLaneOffset(PassingLaneIndex);
	const float OffsetDelta = PassingOffset - CurrentLateralOffset;

	// Start = car pos + offset direction (simulate being in passing lane)
	FVector RefPos, RefDir, RefRight;
	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	SampleSplineAtDist(Seg, ReferenceDistance, PassingOffset, RefPos, RefDir, RefRight);

	const FVector Forward = Owner->GetActorForwardVector();
	const FVector End = RefPos + Forward * OvertakeLateralCheckDistance;

	FHitResult Hit;
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Owner);

	const bool bHit = World->SweepSingleByChannel(
		Hit,
		RefPos, End,
		FQuat::Identity,
		ECC_ObstacleDetect,
		FCollisionShape::MakeSphere(ObstacleTraceRadius),
		Params);

	return !bHit;
}

// ============================================================================
//  Sphere-traces forward *in the original lane* (at its lateral offset) to
//  verify no other vehicle is there before committing to return.
// ============================================================================
bool URoadPathFollowerComponent::IsOriginalLaneClear(int32 OriginalLaneIndex, float CheckDistance) const
{
	UWorld* World = GetWorld();
	if (!World) return false;

	AActor* Owner = GetOwner();
	if (!Owner) return false;

	if (CurrentSegmentIndex >= PathSegments.Num()) return false;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	const float OrigOffset = ComputeTargetLaneOffset(OriginalLaneIndex);

	// Sample point in the original lane at the car's current longitudinal distance
	FVector RefPos, RefDir, RefRight;
	SampleSplineAtDist(Seg, ReferenceDistance, OrigOffset, RefPos, RefDir, RefRight);

	// Forward direction = spline direction at that point (so trace follows curvature)
	FVector Fwd = RefDir.GetSafeNormal2D();
	if (Seg.Direction < 0.0f) Fwd = -Fwd;
	if (Fwd.IsNearlyZero()) Fwd = Owner->GetActorForwardVector();

	const FVector End = RefPos + Fwd * CheckDistance;

	FHitResult Hit;
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Owner);

	const bool bHit = World->SweepSingleByChannel(
		Hit,
		RefPos, End,
		FQuat::Identity,
		ECC_ObstacleDetect,
		FCollisionShape::MakeSphere(ObstacleTraceRadius),
		Params);

	return !bHit;
}

// ============================================================================
//  Uses world-space longitudinal projection on the car's forward axis.
// ============================================================================
bool URoadPathFollowerComponent::IsTargetVehiclePast(float MinPastDistance) const
{
	if (!OvertakeTargetActor.IsValid()) return false;

	const AActor* Owner = GetOwner();
	if (!Owner) return false;

	const FVector MyPos = Owner->GetActorLocation();
	const FVector MyFwd = Owner->GetActorForwardVector().GetSafeNormal2D();
	const FVector TgtPos = OvertakeTargetActor->GetActorLocation();

	FVector Delta = TgtPos - MyPos;
	Delta.Z = 0.0f;

	// Positive = target is ahead; Negative = target is behind
	const float LongitudinalDot = FVector::DotProduct(Delta, MyFwd);

	// Target is behind by at least MinPastDistance
	return (LongitudinalDot <= -MinPastDistance);
}

// ============================================================================
//  Overtake state machine: None → Passing → Returning → None
// ============================================================================
void URoadPathFollowerComponent::UpdateOvertakeLogic()
{
	if (CurrentSegmentIndex >= PathSegments.Num()) return;

	// Never attempt overtake on the final (bound-edge) segment — we must
	// be lined up on the outer lane to pull into the parking lot.
	if (bIsFinalEdge)
	{
		if (OvertakeState != EOvertakeState::None)
		{
			OvertakeState = EOvertakeState::None;
			OvertakeTargetActor = nullptr;
			OvertakeReturnConfirmTimer = 0.0f;
			LogEvent(TEXT("OVERTAKE CANCELLED (final edge)"));
		}
		return;
	}

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	const float DistToJunction = GetDistanceToNextJunction();

	switch (OvertakeState)
	{
	case EOvertakeState::None:
	{
		// Obstacle ahead and it's a vehicle (has PathFollowerComponent)
		if (!ObstacleActor.IsValid()) break;

		URoadPathFollowerComponent* FrontPF =
			ObstacleActor->FindComponentByClass<URoadPathFollowerComponent>();
		if (!FrontPF) break; //  Not a vehicle → don't overtake

		// Front vehicle is slower
		if (FrontPF->GetCurrentSpeed() >= MaxSpeed * OvertakeSpeedThreshold) break;

		// Current road allows overtaking
		if (!Seg.DrivingRule.bAllowOvertaking) break;

		// Not enough lanes for overtaking
		if (Seg.DrivingRule.ForwardLaneCount < OvertakeMinLaneCount) break;

		// Not on junction curve && far from junction
		if (bOnJunctionCurve) break;
		if (DistToJunction < AutoLaneChangeDistance) break;

		// Already changing lane → don't trigger again
		if (bIsChangingLane) break;

		// Determine passing lane
		int32 PassingLane;
		if (Seg.DrivingRule.bPreferLeftLaneForPassing)
		{
			// Left lane (closer to center) = lower index
			PassingLane = FMath::Max(0, CurrentLaneIndex - 1);
		}
		else
		{
			PassingLane = FMath::Min(CurrentLaneIndex + 1, Seg.DrivingRule.ForwardLaneCount - 1);
		}

		// Already in passing lane
		if (PassingLane == CurrentLaneIndex) break;

		// Passing lane is clear (forward)
		if (!IsPassingLaneClear(PassingLane)) break;

		// Passing lane rear is clear (don't cut in front of a faster car)
		if (!IsLaneChangeRearSafe(PassingLane, LaneChangeRearCheckDistance)) break;

		// Initiate overtake
		PreOvertakeLaneIndex = CurrentLaneIndex;
		OvertakePassedDistAccum = 0.0f;
		OvertakeReturnConfirmTimer = 0.0f;
		OvertakeTargetActor = ObstacleActor;  // remember WHICH car we're passing
		RequestLaneChange(PassingLane);
		OvertakeState = EOvertakeState::Passing;

		// Left turn signal
		CurrentTurnSignal = ETurnSignal::Left;

		LogEvent(FString::Printf(TEXT("OVERTAKE START lane %d→%d (frontSpd=%.0f)"),
			PreOvertakeLaneIndex, PassingLane, FrontPF->GetCurrentSpeed()));
		break;
	}

	case EOvertakeState::Passing:
	{
		// Near junction → cancel overtake, but only if original lane is clear.
		// If original lane has vehicles ahead, stay in the passing lane rather
		// than forcing a cut into blocked traffic.
		if (DistToJunction < AutoLaneChangeDistance)
		{
			const bool bJunctionOrigClear =
				IsOriginalLaneClear(PreOvertakeLaneIndex, OvertakeSafeDistance);
			if (bJunctionOrigClear)
			{
				RequestLaneChange(PreOvertakeLaneIndex);
				OvertakeState = EOvertakeState::Returning;
				CurrentTurnSignal = ETurnSignal::Right;
			}
			break;
		}

		// ---------------------------------------------------------------
		// When is it safe to return
		//   (1) Target vehicle is actually behind us by > OvertakeMinPassDistance
		//   (2) Original lane ahead is clear for OvertakeSafeDistance
		//   (3) Both conditions hold for OvertakeReturnConfirmTime seconds (debounce)
		// ---------------------------------------------------------------
		const float DT = GetWorld()->GetDeltaSeconds();

		const bool bTargetPast = IsTargetVehiclePast(OvertakeMinPassDistance);
		const bool bOrigLaneClear = IsOriginalLaneClear(PreOvertakeLaneIndex, OvertakeSafeDistance);
		const bool bOrigLaneRearSafe = IsLaneChangeRearSafe(PreOvertakeLaneIndex, LaneChangeRearCheckDistance);

		// （destroyed / too far / gone）— fallback：
		//  OvertakeSafeDistance 。
		// Fallback when target is gone: accumulate pure distance.
		const bool bTargetLost = !OvertakeTargetActor.IsValid();

		if (bTargetLost)
		{
			OvertakePassedDistAccum += CurrentSpeed * DT;
		}
		else
		{
			OvertakePassedDistAccum = 0.0f;  // only counts when target lost
		}

		const bool bReadyByTarget = (bTargetPast && bOrigLaneClear && bOrigLaneRearSafe);
		const bool bReadyByFallback = (bTargetLost && OvertakePassedDistAccum >= OvertakeSafeDistance
			&& bOrigLaneClear && bOrigLaneRearSafe);

		if (bReadyByTarget || bReadyByFallback)
		{
			OvertakeReturnConfirmTimer += DT;
		}
		else
		{
			OvertakeReturnConfirmTimer = 0.0f;  // reset on any failure (debounce)
		}

		if (OvertakeReturnConfirmTimer >= OvertakeReturnConfirmTime)
		{
			// Confirmed safe → return to original lane
			RequestLaneChange(PreOvertakeLaneIndex);
			OvertakeState = EOvertakeState::Returning;
			CurrentTurnSignal = ETurnSignal::Right;

			LogEvent(FString::Printf(TEXT("OVERTAKE PASSED → return lane=%d"), PreOvertakeLaneIndex));
		}
		break;
	}

	case EOvertakeState::Returning:
	{
		// Lane change complete → overtake done
		if (!bIsChangingLane)
		{
			OvertakeState = EOvertakeState::None;
			OvertakeTargetActor = nullptr;
			OvertakeReturnConfirmTimer = 0.0f;
			OvertakePassedDistAccum = 0.0f;
			// Turn signal handled by Step 0
			CurrentTurnSignal = ETurnSignal::None;
			LogEvent(FString::Printf(TEXT("OVERTAKE COMPLETE lane=%d"), CurrentLaneIndex));
		}
		break;
	}
	}
}
