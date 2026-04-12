#include "RoadPathFollowerComponent.h"
#include "RoadNetworkSubsystem.h"
#include "RoadRuleLibrary.h"
#include "RoadWorldSettings.h"
#include "CollisionChannels.h"
#include "ParkingLotActor.h"
#include "Components/SplineComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"

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
void URoadPathFollowerComponent::ResetFollowingState()
{
	bIsFollowing = false;
	bOnJunctionCurve = false;
	bIsUTurnCurve = false;
	bJustExitedJunctionCurve = false;
	bIsChangingLane = false;
	bOnParkingCurve = false;
	ParkCurveProgress = 0.0f;
	bOnDepartureCurve = false;
	DepCurveProgress = 0.0f;
	JCurveProgress = 0.0f;
	ParkingTargetOffset = 0.0f;
	CurrentSpeed = 0.0f;
	CurrentTurnSignal = ETurnSignal::None;
	NavState = ENavState::Idle;
	PathSegments.Empty();
	CurrentSegmentIndex = 0;
	ObstacleDistance = -1.0f;
	ObstacleActor = nullptr;
	OvertakeState = EOvertakeState::None;
	OvertakePassedDistAccum = 0.0f;

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
		UE_LOG(LogTemp, Warning,
			TEXT("[PARK-NAV] Release previous ParkingLot '%s' spot=%d"),
			*TargetParkingLot->ParkingLotName, TargetParkingSpotIndex);
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

	// 找不到前方 node → fallback 到絕對最近 node（避免完全失敗）
	// No forward node → fallback to absolute nearest (rare, e.g. parked in dead-end)
	if (BestId == INDEX_NONE)
	{
		BestId = Sub->FindNearestGraphNode(CarPos);
		UE_LOG(LogTemp, Warning,
			TEXT("[NAV-START] No forward node found in front of car — fallback to nearest = %d"),
			BestId);
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
		UE_LOG(LogTemp, Warning, TEXT("[NAV-START] FindStartBoundEdge: no edges with splines"));
		return false;
	}

	// 用車朝向 vs spline 切線決定行進方向：
	//   dot >= 0 → 車沿 spline 正向走（StartNode → EndNode），Exit = EndNode
	//   dot  < 0 → 反向走，Exit = StartNode
	// Car forward vs spline direction determines travel direction along the edge.
	const float DirDot = FVector::DotProduct(
		CarFwd2D, BestSplineDir.GetSafeNormal2D());
	OutForward = (DirDot >= 0.0f);
	OutExitNodeId = OutForward ? BestEdge->EndNodeId : BestEdge->StartNodeId;
	OutEdge = BestEdge;
	OutProjDistAbs = BestProjDistAbs;

	UE_LOG(LogTemp, Warning,
		TEXT("[NAV-START] FindStartBoundEdge: Edge=%d ProjDist=%.0f (range=[%.0f,%.0f]) ProjPt=(%.0f,%.0f,%.0f) DistToRoad=%.0f DirDot=%.2f Fwd=%d ExitNode=%d"),
		BestEdge->EdgeId, BestProjDistAbs,
		BestEdge->StartDistanceOnSpline, BestEdge->EndDistanceOnSpline,
		BestProjPt.X, BestProjPt.Y, BestProjPt.Z,
		FMath::Sqrt(BestDistSq), DirDot,
		OutForward ? 1 : 0, OutExitNodeId);

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

	// 建立新段：從 ProjDistAbs 到 Exit 端（依行進方向）
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

	// 如果 A* 建出的第一段第一個 edge 等於 BoundEdge（可能方向相同或反向），
	// 會造成重複 — 在這種情況把 A* 原本的第一段移除，避免車「沿 BoundEdge 再 A* 又走一次」
	// If A*'s first segment is the same edge as BoundEdge, drop A*'s first segment to avoid
	// the car traversing the same edge twice.
	if (PathSegments.Num() > 0
		&& PathSegments[0].EdgeId == BoundEdge.EdgeId)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[PREPEND-BOUND] A* first seg == BoundEdge (EdgeId=%d) — dropping A* first seg to avoid duplicate traversal"),
			BoundEdge.EdgeId);
		PathSegments.RemoveAt(0);
	}

	// 連續性檢查：新段終點 EndNodeId 應該等於原本 PathSegments[0] 的起始 node
	// （由於 PathSegments 沒存 StartNodeId，我們只能看 FirstSeg.EdgeId 對應的 node）
	// Continuity check omitted (PathSegments doesn't store StartNodeId); rely on caller to
	// pass consistent A* target.

	PathSegments.Insert(Seg, 0);

	// 重新計算新段與後面段的 TurnAtEnd（若有後面段）
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

	UE_LOG(LogTemp, Warning,
		TEXT("[PREPEND-BOUND] Prepended EdgeId=%d Dir=%.0f StartDist=%.0f EndDist=%.0f | PathSegments now=%d"),
		Seg.EdgeId, Seg.Direction, Seg.StartDist, Seg.EndDist, PathSegments.Num());
}

// ============================================================================
//  StartNavigationInternal — 共用的 A* + BuildPath + 初始化
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

	// 如果正在跟隨，先重置 / If already following, reset first
	if (bIsFollowing)
	{
		ResetFollowingState();
	}

	// ---- 找車最近的 edge，用它當起始段 ----
	// ---- Find nearest edge; use it as the start bound edge ----
	AActor* OwnerActor = GetOwner();
	const FVector CarPos = OwnerActor ? OwnerActor->GetActorLocation() : FVector::ZeroVector;
	const FVector CarFwd = OwnerActor ? OwnerActor->GetActorForwardVector() : FVector::ForwardVector;

	const FRoadGraphEdge* StartBoundEdge = nullptr;
	float StartBoundProjDistAbs = 0.0f;
	bool bStartBoundForward = true;
	int32 StartBoundExitNode = INDEX_NONE;
	const bool bHaveStartBoundEdge = OwnerActor && FindStartBoundEdge(
		CarPos, CarFwd,
		StartBoundEdge, StartBoundProjDistAbs, bStartBoundForward, StartBoundExitNode);

	// 如果找到 start bound edge，A* 的起點用 edge 的 exit node，而不是 caller 傳進來的 FromNodeId
	// If start bound edge found, A* starts from its exit node (caller's FromNodeId becomes fallback)
	const int32 AStarFromNode = (bHaveStartBoundEdge && StartBoundExitNode != INDEX_NONE)
		? StartBoundExitNode : FromNodeId;

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: StartNav — CallerFrom=%d StartBoundExit=%d → AStarFrom=%d To=%d"),
		FromNodeId, StartBoundExitNode, AStarFromNode, ToNodeId);

	// Edge case: AStarFrom == ToNodeId 就不需要 A*（bound edge 本身就覆蓋全部路徑）
	FRoadGraphPath AStarPath;
	if (AStarFromNode == ToNodeId)
	{
		AStarPath.bPathFound = true;
		AStarPath.NodePath.Add(ToNodeId);
		AStarPath.TotalCost = 0.0f;
		UE_LOG(LogTemp, Warning,
			TEXT("PathFollower: AStarFrom == To (%d) — skipping A*, relying on start bound edge only"),
			ToNodeId);
	}
	else
	{
		AStarPath = Sub->FindPathAStar(AStarFromNode, ToNodeId);
	}

	if (!AStarPath.bPathFound)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: A* failed %d → %d"), AStarFromNode, ToNodeId);
		OnPathComplete.Broadcast(false);
		return;
	}

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: A* path %d → %d | %d nodes | Cost=%.1f"),
		AStarFromNode, ToNodeId, AStarPath.NodePath.Num(), AStarPath.TotalCost);

	BuildPathSegments(AStarPath);

	// ---- 把 start bound edge 的部分段插到最前面 ----
	// ---- Prepend the partial start bound edge as the first segment ----
	// 判斷是否「從停車場車位出發」— 若有 TargetParkingLot 且車離其 AnchorPos 很近，
	// 在 PrependStartBoundEdgeSegment 時把 StartDist 往前推 n（產生等腰三角形斜邊 2）
	// Detect parked-exit: if TargetParkingLot set and car is near its anchor, ask
	// PrependStartBoundEdgeSegment to shift StartDist forward by ParkingTriangleBaseLength,
	// so the spline follow resumes at (ProjPt + n along edge) — giving us exit hypotenuse 2.
	bool bTriangleExit = false;
	{
		const float TriggerSq = FMath::Square(ParkingTriangleBaseLength * 1.5f);
		for (TActorIterator<AParkingLotActor> It(World); It; ++It)
		{
			AParkingLotActor* Lot = *It;
			if (!Lot) continue;
			const FVector Anchor = Lot->GetAnchorArrowPosition();
			if (FVector::DistSquared2D(CarPos, Anchor) < TriggerSq)
			{
				bTriangleExit = true;
				UE_LOG(LogTemp, Warning,
					TEXT("[PARK-NAV] Parked-exit detected near '%s' — using triangle hypotenuse 2 on start"),
					*Lot->ParkingLotName);
				break;
			}
		}
	}

	if (bHaveStartBoundEdge && StartBoundEdge)
	{
		PrependStartBoundEdgeSegment(*StartBoundEdge, bStartBoundForward, StartBoundProjDistAbs, bTriangleExit);
	}

	if (PathSegments.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: No path segments built"));
		OnPathComplete.Broadcast(false);
		return;
	}

	// 初始化所有狀態 / Initialize all state
	CurrentSegmentIndex = 0;
	ReferenceDistance = PathSegments[0].StartDist;
	CurrentSpeed = 0.0f;  // 從靜止開始 / start from standstill
	TargetLaneIndex = CurrentLaneIndex;
	CurrentLateralOffset = ComputeTargetLaneOffset(CurrentLaneIndex);
	bIsChangingLane = false;
	bOnJunctionCurve = false;
	bJustExitedJunctionCurve = false;
	JCurveProgress = 0.0f;
	CurrentTurnSignal = ETurnSignal::None;
	NavState = ENavState::Driving;
	bIsFollowing = true;

	// ---- 出發曲線：把車位置投影到第一段 spline 最近點，再 Hermite 匯入 ----
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

		// 把車位置投影到第一段 spline 最近的距離
		// Project car pos onto first segment's spline to find nearest distance
		float ProjDistAbs = FirstSeg.StartDist;
		if (FirstSeg.Spline)
		{
			const float InputKey = FirstSeg.Spline->FindInputKeyClosestToWorldLocation(CarPos);
			const float RawDist = FirstSeg.Spline->GetDistanceAlongSplineAtSplineInputKey(InputKey);

			// 夾在第一段 spline 距離範圍內（依行進方向）
			// Clamp within segment distance range (direction-aware)
			const float LoDist = FMath::Min(FirstSeg.StartDist, FirstSeg.EndDist);
			const float HiDist = FMath::Max(FirstSeg.StartDist, FirstSeg.EndDist);
			ProjDistAbs = FMath::Clamp(RawDist, LoDist, HiDist);
		}

		// 採樣投影點（在最外側車道橫向偏移上）— 這就是「邊上最近的點」
		// Sample projection point at outermost-lane offset — this is the nearest point on the edge
		FVector RoadStartPos, RoadStartDir, RoadStartRight;
		SampleSplineAtDist(FirstSeg, ProjDistAbs, OuterLaneOffset,
			RoadStartPos, RoadStartDir, RoadStartRight);

		const float DistToRoad = FVector::Dist2D(CarPos, RoadStartPos);

		// 把 ReferenceDistance 設為投影距離（不再從 StartDist 開始）
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

			// 出發後進入最外側車道 / After departure, merge into outermost lane
			TargetLaneIndex = OutermostLane;
			CurrentLaneIndex = OutermostLane;
			CurrentLateralOffset = OuterLaneOffset;

			UE_LOG(LogTemp, Warning,
				TEXT("[DEPART-CURVE] START: CarPos=(%.0f,%.0f,%.0f) → EdgeProj=(%.0f,%.0f,%.0f) Dist=%.0f CurveLen=%.0f ProjDist=%.0f (SegRange=[%.0f,%.0f]) OuterLane=%d"),
				CarPos.X, CarPos.Y, CarPos.Z,
				RoadStartPos.X, RoadStartPos.Y, RoadStartPos.Z,
				DistToRoad, DepCurveLength, ProjDistAbs,
				FirstSeg.StartDist, FirstSeg.EndDist, OutermostLane);
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[DEPART-CURVE] SKIP: Dist=%.0f <= Trigger=%.0f (ProjDist=%.0f) → direct spline follow"),
				DistToRoad, DepartureCurveTriggerDistance, ProjDistAbs);
		}
	}

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: Started | %d segs | MaxSpeed=%.0f | Lane=%d | LateralOffset=%.0f | DepartureCurve=%s"),
		PathSegments.Num(), MaxSpeed, CurrentLaneIndex, CurrentLateralOffset,
		bOnDepartureCurve ? TEXT("YES") : TEXT("NO"));
}

// ============================================================================
//  StartFollowing — 自動導航到隨機停車場
//  Auto-navigate to a random parking lot.
// ============================================================================
void URoadPathFollowerComponent::StartFollowing()
{
	UWorld* World = GetWorld();
	if (!World) return;

	// 找場景中有空位的停車場 / Find parking lots with free spots
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
	UE_LOG(LogTemp, Warning, TEXT("[PARK-NAV] [START] Auto-navigate to ParkingLot '%s'"), *Lot->ParkingLotName);
	NavigateToParkingLot(Lot);
}

// ============================================================================
//  NavigateToNode — 從車目前位置導航到指定 Node ID
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

	// 記錄目的地資訊 / Store destination info
	DestinationNodeId = TargetNodeId;
	const TArray<FRoadGraphNode>& Nodes = Sub->GetGraphNodes();
	if (TargetNodeId >= 0 && TargetNodeId < Nodes.Num())
	{
		DestinationWorldLocation = Nodes[TargetNodeId].WorldLocation;
	}

	// 如果正在行駛中 → 中途重導航（不掉頭）
	// If currently driving → mid-drive reroute (no U-turn)
	if (bIsFollowing && NavState == ENavState::Driving
		&& CurrentSegmentIndex < PathSegments.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("[NAV-NODE] Currently driving → RerouteToNode(%d)"), TargetNodeId);
		RerouteToNode(TargetNodeId);
		return;
	}

	// 靜止狀態 → 找「車前方」的 node 當作 A* 起點，避免從車後方節點開始導致倒退
	// Idle/Parked → use a node IN FRONT of the car as A* start, so we don't drive backward
	// to a node that happens to be the absolute nearest (which is common when parked at a spot
	// whose nearest node is behind the car).
	const FVector CarPos = Owner->GetActorLocation();
	const FVector CarFwd = Owner->GetActorForwardVector();
	UE_LOG(LogTemp, Warning,
		TEXT("[NAV-NODE] Idle/Parked → finding forward node from car pos (%.0f,%.0f,%.0f) fwd=(%.2f,%.2f,%.2f)"),
		CarPos.X, CarPos.Y, CarPos.Z, CarFwd.X, CarFwd.Y, CarFwd.Z);
	const int32 FromNode = FindForwardStartNode(CarPos, CarFwd);
	if (FromNode == INDEX_NONE)
	{
		UE_LOG(LogTemp, Error, TEXT("[NAV-NODE] Cannot find any start node for car position!"));
		OnPathComplete.Broadcast(false);
		return;
	}

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: NavigateToNode — from nearest Node %d to Node %d"),
		FromNode, TargetNodeId);

	StartNavigationInternal(FromNode, TargetNodeId);
}

// ============================================================================
//  NavigateToLocation — 導航到世界座標（自動 SnapToRoad）
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

	// 釋放上一個目的地佔的車格（如有）— 換目的地時不再佔用舊車位
	// Release previously-held spot — switching destination shouldn't keep the old reservation
	ReleasePreviousDestinationSpot();

	// 目的地 SnapToRoad → 最近的 graph node
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

	// 記錄目的地世界座標 / Store destination world location
	DestinationWorldLocation = Destination;

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: NavigateToLocation — dest (%.0f, %.0f, %.0f) → Node %d"),
		Destination.X, Destination.Y, Destination.Z, GoalNode);

	NavigateToNode(GoalNode);
}

// ============================================================================
//  NavigateToParkingLot — 導航到停車場並停入車格
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

	UE_LOG(LogTemp, Warning,
		TEXT("[PARK-NAV] === NavigateToParkingLot START!! === Lot='%s' BoundEdgeId=%d SpotCount=%d"),
		*ParkingLot->ParkingLotName, ParkingLot->BoundEdgeId, ParkingLot->SpotCount);

	// 釋放上一個目的地佔的車格（如有）— 必須在 OccupySpot 之前做，否則會釋掉新格
	// Release any previously-held spot BEFORE occupying a new one
	ReleasePreviousDestinationSpot();

	// 找空位 / Find available spot
	if (SpotIndex == INDEX_NONE)
	{
		SpotIndex = ParkingLot->FindAvailableSpot();
		UE_LOG(LogTemp, Warning, TEXT("[PARK-NAV] FindAvailableSpot → %d"), SpotIndex);
	}
	if (SpotIndex == INDEX_NONE)
	{
		UE_LOG(LogTemp, Error, TEXT("[PARK-NAV] ParkingLot '%s' has NO available spots!"),
			*ParkingLot->ParkingLotName);
		OnPathComplete.Broadcast(false);
		return;
	}

	// 佔位 / Occupy spot
	if (!ParkingLot->OccupySpot(SpotIndex, GetOwner()))
	{
		UE_LOG(LogTemp, Error, TEXT("[PARK-NAV] Failed to occupy spot %d in '%s'"),
			SpotIndex, *ParkingLot->ParkingLotName);
		OnPathComplete.Broadcast(false);
		return;
	}

	const FVector SpotWorldPos = ParkingLot->GetSpotWorldPosition(SpotIndex);
	UE_LOG(LogTemp, Warning, TEXT("[PARK-NAV] Occupied spot %d at (%.0f,%.0f,%.0f)"),
		SpotIndex, SpotWorldPos.X, SpotWorldPos.Y, SpotWorldPos.Z);

	// 儲存目的地狀態 / Store destination state
	DestinationType = EDestinationType::ParkingLot;
	TargetParkingLot = ParkingLot;
	TargetParkingSpotIndex = SpotIndex;

	// ============================================================================
	// [PARK-NAV] 流程 / Flow:
	//   1. 讀停車場綁定的 Edge（Arrow[0] 在 BuildRoadCache 決定）
	//   2. 用 Arrow[0] vs spline 切線 dot 決定進入邊的方向與「Entry Node」（非 Exit！）
	//   3. A* 從車位置 → Entry Node（停在綁定 edge 入口那一側的 node）
	//   4. 手動把 BoundEdge 附加為最後一段，EndDist 截斷到 Arrow[0] 在 spline 上的投影距離
	//      → 車會自然沿 BoundEdge 行駛到投影點（edge 中段），沿路不會在 GoalNode 停下
	//   5. 抵達投影點後，Parking pull-over 邏輯依 bIsLeftSide 往左/右靠邊停
	// Key: A* goal is the ENTRY node (not exit) of BoundEdge. The final segment is the
	// BoundEdge itself, truncated at the anchor projection — so the car drives the bound
	// edge from its entry to the projection point without stopping at the intermediate node.
	// ============================================================================
	const int32 NavEdgeId = ParkingLot->BoundEdgeId;
	if (NavEdgeId == INDEX_NONE)
	{
		UE_LOG(LogTemp, Error,
			TEXT("[PARK-NAV] '%s' has no BoundEdgeId! Check TargetRoadActorLabel='%s' — did BindParkingActorsToEdges run?"),
			*ParkingLot->ParkingLotName, *ParkingLot->TargetRoadActorLabel);
		ParkingLot->ReleaseSpot(SpotIndex);
		OnPathComplete.Broadcast(false);
		return;
	}

	const FRoadGraphEdge* BoundEdge = Sub->GetGraphEdgeById(NavEdgeId);
	if (!BoundEdge || !BoundEdge->InputSpline)
	{
		UE_LOG(LogTemp, Error, TEXT("[PARK-NAV] BoundEdgeId=%d invalid!"), NavEdgeId);
		ParkingLot->ReleaseSpot(SpotIndex);
		OnPathComplete.Broadcast(false);
		return;
	}

	// Arrow[0] = 導航錨點 / navigation anchor
	const FVector AnchorPos = ParkingLot->GetAnchorArrowPosition();
	const FVector AnchorFwd = ParkingLot->GetAnchorArrowForward();
	const FVector AnchorProjPt = ParkingLot->BoundProjectionPoint;

	// 用 Arrow[0] 朝向 vs spline 在錨點投影位置的切線，決定車要沿哪個方向開過 BoundEdge
	// Use Arrow[0] forward vs spline tangent at anchor projection to pick drive direction
	const float AnchorKey = BoundEdge->InputSpline->FindInputKeyClosestToWorldLocation(AnchorPos);
	const float AnchorSplineDist = BoundEdge->InputSpline->GetDistanceAlongSplineAtSplineInputKey(AnchorKey);
	const FVector AnchorSplineDir = BoundEdge->InputSpline->GetDirectionAtDistanceAlongSpline(
		AnchorSplineDist, ESplineCoordinateSpace::World);
	const float AnchorDot = FVector::DotProduct(
		AnchorFwd.GetSafeNormal2D(), AnchorSplineDir.GetSafeNormal2D());

	// Spline 正向 = StartNode → EndNode。
	// dot >= 0 → 車沿正向 (Start→End) 行駛 → Entry=StartNode；dot < 0 → 反向 → Entry=EndNode
	// A* 的目標是 ENTRY node，不是 Exit node，這樣車到達 Entry 後才沿著 BoundEdge 開到投影點
	// (Car reaches entry, then drives along BoundEdge to the anchor projection in the edge middle.)
	const bool bForwardDrive = (AnchorDot >= 0.0f);
	const int32 EntryNodeId = bForwardDrive ? BoundEdge->StartNodeId : BoundEdge->EndNodeId;

	// 把錨點投影 clamp 到 BoundEdge 的 spline 距離範圍內
	// Clamp anchor projection to bound edge's spline distance range
	const float BoundMinDist = FMath::Min(BoundEdge->StartDistanceOnSpline, BoundEdge->EndDistanceOnSpline);
	const float BoundMaxDist = FMath::Max(BoundEdge->StartDistanceOnSpline, BoundEdge->EndDistanceOnSpline);
	const float ProjDistAbs = FMath::Clamp(AnchorSplineDist, BoundMinDist, BoundMaxDist);

	UE_LOG(LogTemp, Warning,
		TEXT("[PARK-NAV] '%s' BoundEdge=%d Start=%d End=%d | AnchorDot=%.2f → Entry=%d (fwd=%d) ProjDist=%.0f | ProjPt=(%.0f,%.0f,%.0f)"),
		*ParkingLot->ParkingLotName, NavEdgeId, BoundEdge->StartNodeId, BoundEdge->EndNodeId,
		AnchorDot, EntryNodeId, bForwardDrive ? 1 : 0, ProjDistAbs,
		AnchorProjPt.X, AnchorProjPt.Y, AnchorProjPt.Z);

	if (EntryNodeId == INDEX_NONE)
	{
		UE_LOG(LogTemp, Error, TEXT("[PARK-NAV] BoundEdge has invalid node IDs!"));
		ParkingLot->ReleaseSpot(SpotIndex);
		OnPathComplete.Broadcast(false);
		return;
	}

	ParkingMode = EParkingMode::RoadsideStop;

	UE_LOG(LogTemp, Warning,
		TEXT("[PARK-NAV] '%s' Calling NavigateToNode(Entry=%d) bIsFollowing=%s NavState=%d"),
		*ParkingLot->ParkingLotName, EntryNodeId,
		bIsFollowing ? TEXT("true") : TEXT("false"), (int32)NavState);

	// ---- A* 到 Entry Node（不是 Exit）----
	// ---- A* to Entry Node (not Exit) ----
	NavigateToNode(EntryNodeId);

	// ---- 手動把 BoundEdge 附加為最後一段，EndDist 截到投影距離 ----
	// ---- Manually append BoundEdge as the final segment, truncated to projection dist ----
	AppendBoundEdgeFinalSegment(*BoundEdge, bForwardDrive, ProjDistAbs);

	// NavigateToNode 會覆蓋 DestinationWorldLocation，所以要在之後再設
	// NavigateToNode overwrites DestinationWorldLocation, so set it AFTER
	// 目的地 = Arrow[0] 投影到綁定 edge 上的點
	// Destination = Arrow[0] projection on bound edge
	DestinationWorldLocation = AnchorProjPt;

	UE_LOG(LogTemp, Warning,
		TEXT("[PARK-NAV] '%s' === NavigateToParkingLot END === PathSegments=%d bIsFollowing=%s bIsLeftSide=%s targetSpot=%d"),
		*ParkingLot->ParkingLotName, PathSegments.Num(),
		bIsFollowing ? TEXT("true") : TEXT("false"),
		ParkingLot->bIsLeftSide ? TEXT("L") : TEXT("R"), SpotIndex);
}

// ============================================================================
//  TruncatePathToWorldPosition — 截斷路徑到目標世界位置
//  Truncate path so it ends at the spline projection of TargetPos.
// ============================================================================
void URoadPathFollowerComponent::TruncatePathToWorldPosition(const FVector& TargetPos, int32 TargetEdgeId)
{
	if (PathSegments.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("[TRUNCATE] PathSegments is empty — nothing to truncate"));
		return;
	}

	// 找匹配 EdgeId 的段 / Find segment matching EdgeId
	int32 TargetSegIdx = INDEX_NONE;
	for (int32 i = PathSegments.Num() - 1; i >= 0; --i)
	{
		if (PathSegments[i].EdgeId == TargetEdgeId)
		{
			TargetSegIdx = i;
			break;
		}
	}

	// 如果找不到匹配 edge，用最後一段 / If no matching edge, use last segment
	if (TargetSegIdx == INDEX_NONE)
	{
		TargetSegIdx = PathSegments.Num() - 1;
		UE_LOG(LogTemp, Warning, TEXT("[TRUNCATE] No segment matches EdgeId=%d, using last segment (idx=%d EdgeId=%d)"),
			TargetEdgeId, TargetSegIdx, PathSegments[TargetSegIdx].EdgeId);
	}

	FPathSegmentInternal& Seg = PathSegments[TargetSegIdx];
	if (!Seg.Spline)
	{
		UE_LOG(LogTemp, Warning, TEXT("[TRUNCATE] Target segment has no spline!"));
		return;
	}

	// 投影目標位置到 spline / Project target position onto spline
	const float InputKey = Seg.Spline->FindInputKeyClosestToWorldLocation(TargetPos);
	const float ProjectedDist = Seg.Spline->GetDistanceAlongSplineAtSplineInputKey(InputKey);

	// 取得投影點的世界位置來驗證 / Get projected world pos for verification
	const FVector ProjectedWorldPos = Seg.Spline->GetLocationAtDistanceAlongSpline(
		ProjectedDist, ESplineCoordinateSpace::World);

	const float OldEndDist = Seg.EndDist;
	const float MinDist = FMath::Min(Seg.StartDist, OldEndDist);
	const float MaxDist = FMath::Max(Seg.StartDist, OldEndDist);

	// Clamp 到段的有效範圍 / Clamp to segment's valid range
	const float ClampedDist = FMath::Clamp(ProjectedDist, MinDist, MaxDist);
	Seg.EndDist = ClampedDist;

	UE_LOG(LogTemp, Warning,
		TEXT("[TRUNCATE] Seg[%d] EdgeId=%d | SplineDist: Start=%.0f OldEnd=%.0f → NewEnd=%.0f (projected=%.0f) | Dir=%.1f"),
		TargetSegIdx, Seg.EdgeId, Seg.StartDist, OldEndDist, ClampedDist, ProjectedDist, Seg.Direction);
	UE_LOG(LogTemp, Warning,
		TEXT("[TRUNCATE] TargetPos=(%.0f,%.0f,%.0f) ProjectedPos=(%.0f,%.0f,%.0f) dist=%.0f cm"),
		TargetPos.X, TargetPos.Y, TargetPos.Z,
		ProjectedWorldPos.X, ProjectedWorldPos.Y, ProjectedWorldPos.Z,
		FVector::Dist(TargetPos, ProjectedWorldPos));

	// 移除此段之後的所有段 / Remove all segments after this one
	const int32 RemoveCount = PathSegments.Num() - TargetSegIdx - 1;
	if (RemoveCount > 0)
	{
		PathSegments.RemoveAt(TargetSegIdx + 1, RemoveCount);
		UE_LOG(LogTemp, Warning, TEXT("[TRUNCATE] Removed %d segments after target, %d remaining"),
			RemoveCount, PathSegments.Num());
	}
}

// ============================================================================
//  AppendBoundEdgeFinalSegment — 將綁定 edge 附加為路徑最後一段，EndDist 截到投影距離
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

	// 如果 NavigateToNode 因為失敗導致 PathSegments 為空（例如 A* 失敗或起點==終點），
	// 不把 bound edge 作為「唯一一段」附加：這需要 StartNavigationInternal 的完整初始化
	// (DepartureCurve, ReferenceDistance, NavState 等)，留給未來處理。
	// If NavigateToNode produced no segments (A* failed or from==to), bail out —
	// we'd need full StartNavigationInternal init (departure curve, state…) to handle it.
	if (PathSegments.Num() == 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[APPEND-BOUND] PathSegments empty after NavigateToNode — skipping bound edge append (car likely already at entry node)"));
		return;
	}

	// 建立新段
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

	// 截到投影距離 − n（等腰三角形斜邊 1 的起點：ProjPt 前 n 處）
	// Truncate to (projection − n along travel direction) — this is the entry hypotenuse's base point.
	// The remaining Hermite "parking curve" then runs from here (edge direction) to AnchorPos (arrow direction),
	// forming hypotenuse 1 of the isosceles triangle.
	const float BoundMinDist = FMath::Min(BoundEdge.StartDistanceOnSpline, BoundEdge.EndDistanceOnSpline);
	const float BoundMaxDist = FMath::Max(BoundEdge.StartDistanceOnSpline, BoundEdge.EndDistanceOnSpline);
	const float ClampedProj = FMath::Clamp(ProjDistAbs, BoundMinDist, BoundMaxDist);
	const float PulledBack = ClampedProj - Seg.Direction * ParkingTriangleBaseLength;
	Seg.EndDist = FMath::Clamp(PulledBack, BoundMinDist, BoundMaxDist);

	// 最後一段沒有下一個路口，TurnAtEnd = None
	Seg.TurnAtEnd = ETurnSignal::None;
	Seg.bUTurnAtEnd = false;

	// ---- 重算原本最後一段的 TurnAtEnd：它現在要接到 BoundEdge 入口 ----
	// ---- Recompute previously-last segment's TurnAtEnd — it now feeds into BoundEdge ----
	FPathSegmentInternal& PrevLast = PathSegments.Last();

	const int32 EntryNodeId = bForward ? BoundEdge.StartNodeId : BoundEdge.EndNodeId;

	// 連續性檢查：PrevLast 的終點應該 == BoundEdge 的 Entry
	if (PrevLast.EndNodeId != EntryNodeId)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[APPEND-BOUND] Discontinuity! PrevLast EdgeId=%d EndNode=%d but BoundEdge entry=%d — car may take a weird turn"),
			PrevLast.EdgeId, PrevLast.EndNodeId, EntryNodeId);
	}

	// 用實際 spline 切線比對 PrevLast 出口 vs 新 Seg 入口
	// Use actual spline tangents: PrevLast end tangent vs new Seg start tangent
	ComputeTurnAtJunction(PrevLast, Seg, PrevLast.TurnAtEnd, PrevLast.bUTurnAtEnd);

	PathSegments.Add(Seg);

	UE_LOG(LogTemp, Warning,
		TEXT("[APPEND-BOUND] Appended EdgeId=%d Dir=%.0f StartDist=%.0f EndDist=%.0f (proj=%.0f) | PathSegments now=%d"),
		Seg.EdgeId, Seg.Direction, Seg.StartDist, Seg.EndDist, ProjDistAbs, PathSegments.Num());
}

// ============================================================================
//  ComputeTurnAtJunction — 用實際 spline 切線判斷轉彎
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

	// 入口方向：InSeg 在 EndDist 的行進方向 / Incoming dir: InSeg travel dir at EndDist
	FVector InPos, InDir, InRight;
	SampleSplineAtDist(InSeg, InSeg.EndDist, 0.0f, InPos, InDir, InRight);

	// 出口方向：OutSeg 在 StartDist 的行進方向 / Outgoing dir: OutSeg travel dir at StartDist
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
		// U-turn：方向燈還是照 CrossZ 分左右，但標記 U-turn
		OutTurn = (CrossZ > 0.0f) ? ETurnSignal::Right : ETurnSignal::Left;
		bOutUTurn = true;
	}
	else
	{
		// UE 左手座標：CrossZ > 0 = 右轉 / CrossZ > 0 = right
		OutTurn = (CrossZ > 0.0f) ? ETurnSignal::Right : ETurnSignal::Left;
		bOutUTurn = false;
	}
}

// ============================================================================
//  BuildPathSegments（跟之前一樣，不變）
//  Build path segments from A* result (unchanged).
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

			UE_LOG(LogTemp, Warning,
				TEXT("  Seg[%d]: %s | Type=%d %s | TwoRoads=%s | Lanes=%d | AutoLnW=%.0f AutoMed=%.0f"),
				PathSegments.Num() - 1,
				bFwd ? TEXT("Fwd") : TEXT("Rev"),
				Seg.RoadType, *Seg.DrivingRule.RuleName,
				Seg.bTwoRoads ? TEXT("Y") : TEXT("N"),
				Seg.DrivingRule.ForwardLaneCount,
				Seg.AutoLaneWidthCm, Seg.AutoMedianCm);

			break;
		}
	}

	// ---- 用實際 spline 切線預計算每個路口的轉彎方向 ----
	// ---- Precompute turn direction at each junction using actual spline tangents ----
	for (int32 i = 0; i + 1 < PathSegments.Num(); ++i)
	{
		float DbgDot = 0.0f, DbgCross = 0.0f;
		ComputeTurnAtJunction(
			PathSegments[i], PathSegments[i + 1],
			PathSegments[i].TurnAtEnd, PathSegments[i].bUTurnAtEnd,
			&DbgDot, &DbgCross);

		UE_LOG(LogTemp, Warning,
			TEXT("  Seg[%d] TurnAtEnd=%s%s  Dot=%.3f CrossZ=%.3f"),
			i,
			(PathSegments[i].TurnAtEnd == ETurnSignal::Left) ? TEXT("LEFT") :
			(PathSegments[i].TurnAtEnd == ETurnSignal::Right) ? TEXT("RIGHT") : TEXT("NONE"),
			PathSegments[i].bUTurnAtEnd ? TEXT(" [U-TURN]") : TEXT(""),
			DbgDot, DbgCross);
	}
}

// ============================================================================
//  RerouteToNode — 中途重導航：保留當前段，從段終點 A* 到新目的地
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
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: RerouteToNode — current segment has no EndNodeId"));
		return;
	}

	// 如果剛好目的地就是當前段終點 → 不用重算
	// If destination is already the end of current segment → no reroute needed
	if (FromNodeId == TargetNodeId)
	{
		PathSegments.RemoveAt(CurrentSegmentIndex + 1, PathSegments.Num() - CurrentSegmentIndex - 1);
		return;
	}

	// 找當前段的起始 Node（車身後方的 node）
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
		UE_LOG(LogTemp, Warning,
			TEXT("PathFollower: RerouteToNode A* failed %d → %d"), FromNodeId, TargetNodeId);
		return;
	}

	// ---- 迴轉偵測：A* 第一步是否走回頭路 ----
	// ---- U-turn detection: does A* first step go backwards ----
	// 如果 A* 路徑第二個 node == CurSegStartNode → 走回頭路 → junction curve 會飛出路面
	// 解法：從 FromNode 找一個「前方」鄰居（不是 CurSegStartNode），先走一段再 A*
	// If A* path[1] == CurSegStartNode → going backwards → junction curve will overshoot
	// Fix: find a "forward" neighbor of FromNode (not CurSegStartNode), go there first, then A*
	if (AStarPath.NodePath.Num() >= 2 && CurSegStartNode != INDEX_NONE
		&& AStarPath.NodePath[1] == CurSegStartNode)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("PathFollower: RerouteToNode — U-turn detected, finding forward detour"));

		// 從 FromNode 的鄰居中找一個不是 CurSegStartNode 的前方 node
		// Find a neighbor of FromNode that's not CurSegStartNode
		const TArray<FRoadGraphNeighbor> Neighbors = Sub->GetNeighborNodes(FromNodeId);
		int32 ForwardNodeId = INDEX_NONE;
		for (const FRoadGraphNeighbor& Nb : Neighbors)
		{
			if (Nb.NeighborNodeId != CurSegStartNode)
			{
				ForwardNodeId = Nb.NeighborNodeId;
				break;
			}
		}

		if (ForwardNodeId != INDEX_NONE)
		{
			// 從前方 node 重新 A* 到目的地
			// Re-route from the forward node to destination
			FRoadGraphPath ForwardPath = Sub->FindPathAStar(ForwardNodeId, TargetNodeId);
			if (ForwardPath.bPathFound)
			{
				// 把 FromNodeId 插到路徑前面（FromNode → ForwardNode → ... → dest）
				// Prepend FromNodeId to path (FromNode → ForwardNode → ... → dest)
				ForwardPath.NodePath.Insert(FromNodeId, 0);
				AStarPath = ForwardPath;

				UE_LOG(LogTemp, Warning,
					TEXT("PathFollower: RerouteToNode — detour via Node %d → %d"),
					FromNodeId, ForwardNodeId);
			}
			// 如果前方路也找不到 → 用原始 A* 結果（容忍迴轉）
			// If forward route fails → use original A* (tolerate U-turn)
		}
		// 如果只有一個鄰居（死路）→ 用原始 A* 結果
		// If only one neighbor (dead end) → use original A*
	}

	// 清除當前段之後的所有段 / Clear all segments after current
	PathSegments.RemoveAt(CurrentSegmentIndex + 1, PathSegments.Num() - CurrentSegmentIndex - 1);

	// 用新 A* 路徑建段並接在後面 / Build new segments and append
	AppendPathSegments(AStarPath);

	// 取消超車狀態 / Cancel overtake
	if (OvertakeState != EOvertakeState::None)
	{
		OvertakeState = EOvertakeState::None;
	}

	// ---- 重新計算 TurnAtEnd（用實際 spline 切線）----
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

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: RerouteToNode — from EndNode %d to Node %d | %d total segs"),
		FromNodeId, TargetNodeId, PathSegments.Num());
}

// ============================================================================
//  AppendPathSegments — 用 A* 結果建段，接在 PathSegments 後面
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

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: Lane change requested → Lane %d"), TargetLaneIndex);
}

// ============================================================================
//  SampleSplineAtDist — 在指定段+距離處取樣位置（含偏移）
//  Sample position at given segment distance with lateral offset.
// ============================================================================
void URoadPathFollowerComponent::SampleSplineAtDist(
	const FPathSegmentInternal& Seg, float Dist, float LateralOffset,
	FVector& OutPosition, FVector& OutTravelDir, FVector& OutTravelRight) const
{
	if (!Seg.Spline)
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

	// 加上橫向偏移
	// Apply lateral offset
	OutPosition += OutTravelRight * LateralOffset;
}

// ============================================================================
//  GetPursuitPoint — 追蹤點：沿路徑往前看，可跨段
//  Pursuit point: look ahead along path, crossing segment boundaries.
// ============================================================================
FVector URoadPathFollowerComponent::GetPursuitPoint(float LateralOffset) const
{
	// 前瞻距離 = max(速度 × 時間, 最小值)
	// Look-ahead distance = max(speed × time, minimum)
	const float AheadDist = FMath::Max(CurrentSpeed * LookAheadTime, MinLookAheadDist);

	float Remaining = AheadDist;
	int32 SegIdx = CurrentSegmentIndex;
	float Dist = ReferenceDistance;

	while (Remaining > 0.0f && SegIdx < PathSegments.Num())
	{
		const FPathSegmentInternal& Seg = PathSegments[SegIdx];

		// 目前位置到該段終點的距離
		// Distance from current position to segment end
		const float DistToEnd = FMath::Abs(Seg.EndDist - Dist);

		if (Remaining <= DistToEnd)
		{
			// 追蹤點在這一段內
			// Pursuit point is within this segment
			const float FinalDist = Dist + Remaining * Seg.Direction;

			FVector Pos, Dir, Right;
			SampleSplineAtDist(Seg, FinalDist, LateralOffset, Pos, Dir, Right);
			return Pos;
		}

		// 跨到下一段
		// Cross to next segment
		Remaining -= DistToEnd;
		SegIdx++;

		if (SegIdx < PathSegments.Num())
		{
			Dist = PathSegments[SegIdx].StartDist;
		}
	}

	// 如果超出路徑末端，回傳最後一段的終點
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
//  GetRemainingDistance — 到路徑終點的剩餘距離
//  Remaining distance to end of path.
// ============================================================================
float URoadPathFollowerComponent::GetRemainingDistance() const
{
	if (!bIsFollowing || PathSegments.Num() == 0) return 0.0f;

	// 目前段的剩餘
	// Remaining in current segment
	float Total = FMath::Abs(PathSegments[CurrentSegmentIndex].EndDist - ReferenceDistance);

	// 加上後續所有段
	// Add all subsequent segments
	for (int32 i = CurrentSegmentIndex + 1; i < PathSegments.Num(); ++i)
	{
		Total += PathSegments[i].GetTravelLength();
	}

	return Total;
}

// ============================================================================
//  GetCurrentCurvature — 當前 spline 曲率（1/cm）
//  Current spline curvature (1/cm). Higher = sharper turn.
// ============================================================================
float URoadPathFollowerComponent::GetCurrentCurvature() const
{
	if (CurrentSegmentIndex >= PathSegments.Num()) return 0.0f;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	if (!Seg.Spline) return 0.0f;

	const float SplineLen = Seg.Spline->GetSplineLength();
	const float Clamped = FMath::Clamp(ReferenceDistance, 0.0f, SplineLen);

	// 用前後兩點的切線方向差來估算曲率
	// Estimate curvature from tangent direction change between two nearby points
	const float SampleDelta = 100.0f; // 1 公尺 / 1 meter
	const float DistA = FMath::Clamp(Clamped - SampleDelta, 0.0f, SplineLen);
	const float DistB = FMath::Clamp(Clamped + SampleDelta, 0.0f, SplineLen);

	if (FMath::IsNearlyEqual(DistA, DistB)) return 0.0f;

	const FVector TangentA = Seg.Spline->GetDirectionAtDistanceAlongSpline(
		DistA, ESplineCoordinateSpace::World);
	const FVector TangentB = Seg.Spline->GetDirectionAtDistanceAlongSpline(
		DistB, ESplineCoordinateSpace::World);

	// 角度差 / 弧長 = 曲率
	// Angle difference / arc length = curvature
	const float AngleRad = FMath::Acos(FMath::Clamp(
		FVector::DotProduct(TangentA, TangentB), -1.0f, 1.0f));
	const float ArcLength = DistB - DistA;

	return (ArcLength > 1.0f) ? (AngleRad / ArcLength) : 0.0f;
}

// ============================================================================
//  ComputeTargetLaneOffset — 指定車道的目標偏移量
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
//  GetDistanceToNextJunction — 到下一個段結束的距離
//  Distance to next segment boundary (junction).
// ============================================================================
float URoadPathFollowerComponent::GetDistanceToNextJunction() const
{
	if (CurrentSegmentIndex >= PathSegments.Num()) return FLT_MAX;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	return FMath::Abs(Seg.EndDist - ReferenceDistance);
}

// ============================================================================
//  ComputeDesiredSpeed — 綜合計算目標速度
//  Compute desired speed considering acceleration, curves, junctions, braking.
// ============================================================================
float URoadPathFollowerComponent::ComputeDesiredSpeed() const
{
	float Desired = MaxSpeed;

	// ---- 彎道減速（spline 曲率）/ Curve speed reduction ----
	const float Curvature = GetCurrentCurvature();
	if (Curvature > 0.0001f)
	{
		const float CurveFactor = 1.0f / (1.0f + CurveSensitivity * Curvature * 10000.0f);
		const float CurveSpeed = MaxSpeed * FMath::Max(CurveFactor, CurveMinSpeedRatio);
		Desired = FMath::Min(Desired, CurveSpeed);
	}

	// ---- 路口減速 / Junction approach slowdown ----
	// 在 JunctionBlendDistance 處就要到最低速（不是路口中心點）
	// 確保車以慢速進入 Hermite 曲線，速度無斷層
	// Reach min speed at JunctionBlendDistance (not junction center).
	// Ensures car enters Hermite curve already at low speed — no speed discontinuity.
	const float DistToJunction = GetDistanceToNextJunction();
	const bool bNextSegExists = (CurrentSegmentIndex + 1 < PathSegments.Num());

	if (bNextSegExists && DistToJunction < JunctionSlowdownDistance)
	{
		// 用 CurrentTurnSignal（Step 0 每幀更新）決定是否減速
		// Use CurrentTurnSignal (updated in Step 0 each tick) for slowdown decision
		if (CurrentTurnSignal != ETurnSignal::None)
		{
			// U-turn 預減速目標更高，保持自然速度不停頓
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
				// 已在曲線觸發區 → 保持最低速
				// Inside curve trigger zone → hold minimum
				Alpha = 1.0f;
			}
			else
			{
				// SlowdownDistance→BlendDistance 之間 SmoothStep 減速
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

	// 曲線全程維持低速，出曲線後才由 FInterpTo 慢慢加速
	// Hold low speed for entire curve. Post-curve FInterpTo ramps up naturally.
	if (bOnJunctionCurve)
	{
		if (bIsUTurnCurve)
		{
			// U 型掉頭：保持 UTurnSpeedRatio 作為目標速度，但不低於 MinUTurnSpeed
			// 正常汽車掉頭大約 10~20 km/h (278~556 cm/s)，不會停下來
			// U-turn: use UTurnSpeedRatio as target but enforce minimum speed
			// Normal cars U-turn at ~10-20 km/h (278-556 cm/s), never stopping
			const float UTurnTargetSpeed = MaxSpeed * UTurnSpeedRatio;
			const float MinUTurnSpeed = 400.0f;  // ~14.4 km/h 最低掉頭速度 / min U-turn speed
			Desired = FMath::Min(Desired, FMath::Max(UTurnTargetSpeed, MinUTurnSpeed));
		}
		else
		{
			Desired = FMath::Min(Desired, MaxSpeed * JunctionMinSpeedRatio);
		}
	}

	// ---- 障礙物減速 / Obstacle braking ----
	{
		const float ObstacleLimit = ComputeObstacleSpeedLimit();
		Desired = FMath::Min(Desired, ObstacleLimit);
	}

	// ---- 終點煞車 / End-of-path braking ----
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
//  GetRouteWorldPoints — 從目前位置到路徑終點取樣世界座標
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

void URoadPathFollowerComponent::GetRouteWorldPoints(TArray<FVector>& OutPoints, int32 SamplesPerSegment) const
{
	OutPoints.Empty();
	if (!bIsFollowing || PathSegments.Num() == 0) return;

	for (int32 SegIdx = CurrentSegmentIndex; SegIdx < PathSegments.Num(); ++SegIdx)
	{
		const FPathSegmentInternal& Seg = PathSegments[SegIdx];
		if (!Seg.Spline) continue;

		// 此段的取樣起點和終點
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
//  TickComponent — 主迴圈
//  Main update loop: speed control → advance reference → pursuit → interpolate.
// ============================================================================
void URoadPathFollowerComponent::TickComponent(
	float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// 第一次 Tick 時啟動（road graph 此時已建好）
	// 需要同時檢查 bAutoStart — TrafficManager 會在 spawn 後設 bAutoStart=false
	// 防止寫死的 StartNodeId/GoalNodeId 覆蓋 TrafficManager 指定的隨機目的地
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

	FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	if (!Seg.Spline)
	{
		bIsFollowing = false;
		return;
	}

	// ================================================================
	//  0. 方向燈 + 自動換道（預計算，不依賴距離門檻）
	//     Turn signal + auto lane change (precomputed, no distance threshold)
	// ================================================================
	{
		// 方向燈：只在接近下一個路口時才打（距離由 AutoLaneChangeDistance 控制）
		// 避免剛進入新段就誤報遠方的下一個轉彎
		// Turn signal: only when approaching the next junction (gated by AutoLaneChangeDistance).
		// Prevents fire-on-entry where a far-away upcoming turn wrongly lights the blinker.
		const ETurnSignal UpcomingTurn = Seg.TurnAtEnd;
		const float DistToJunction = GetDistanceToNextJunction();
		const bool bWithinSignalDist = (DistToJunction < AutoLaneChangeDistance);

		if (UpcomingTurn != ETurnSignal::None && bWithinSignalDist
			&& CurrentTurnSignal != UpcomingTurn)
		{
			CurrentTurnSignal = UpcomingTurn;
			UE_LOG(LogTemp, Warning,
				TEXT("PathFollower: Turn signal → %s (Seg[%d] DistToJct=%.0f)"),
				(CurrentTurnSignal == ETurnSignal::Left) ? TEXT("LEFT") : TEXT("RIGHT"),
				CurrentSegmentIndex, DistToJunction);
		}
		else if ((UpcomingTurn == ETurnSignal::None || !bWithinSignalDist)
			&& !bOnJunctionCurve)
		{
			CurrentTurnSignal = ETurnSignal::None;
		}

		// 自動換道：用 AutoLaneChangeDistance（比減速更早），讓車有時間完成換道
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
				RequestLaneChange(DesiredLane);
				UE_LOG(LogTemp, Warning,
					TEXT("PathFollower: Auto lane change → Lane %d for %s turn (junction in %.0f cm)"),
					DesiredLane,
					(UpcomingTurn == ETurnSignal::Left) ? TEXT("LEFT") : TEXT("RIGHT"),
					DistToJunction);
			}
		}
	}

	// ================================================================
	//  0.5 停車：最後一段 → 根據目的地類型執行對應停車策略
	//      Parking: last segment → execute parking based on destination type
	//
	//  核心邏輯 / Core logic:
	//    - ParkingLot / Roadside: 用目標世界座標投影到 spline 的 EndDist 處，
	//      計算橫向偏移，讓車平滑偏移到目標停車位然後停下
	//    - Default: 靠右路肩（原邏輯）
	// ================================================================
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
				// --- [PARK-NAV] 停車場：以 Arrow[0] 為停車目標，bIsLeftSide 決定靠哪一側路肩 ---
				// --- [PARK-NAV] ParkingLot: target = Arrow[0], bIsLeftSide picks shoulder ---
				const FVector AnchorPos = TargetParkingLot->GetAnchorArrowPosition();
				const FVector AnchorFwd = TargetParkingLot->GetAnchorArrowForward();
				ParkCurveP1 = AnchorPos;

				FVector SplineEndPos, EndTravelDir, EndTravelRight;
				SampleSplineAtDist(Seg, Seg.EndDist, 0.0f, SplineEndPos, EndTravelDir, EndTravelRight);

				// 轉向燈：直接看 bIsLeftSide（使用者設定）
				// Turn signal: directly from bIsLeftSide (user-configured)
				CurrentTurnSignal = TargetParkingLot->bIsLeftSide ? ETurnSignal::Left : ETurnSignal::Right;

				const float CurveDist = (ParkCurveP1 - SplineEndPos).Size();
				ParkCurveT1 = AnchorFwd * CurveDist * ParkingCurveTangentScale;

				UE_LOG(LogTemp, Warning,
					TEXT("[PARK-NAV] [0.5] LOT '%s' AnchorPos=(%.0f,%.0f,%.0f) AnchorFwd=(%.2f,%.2f,%.2f) | CurveDist=%.0f signal=%s bIsLeftSide=%s remain=%.0f"),
					*TargetParkingLot->ParkingLotName,
					AnchorPos.X, AnchorPos.Y, AnchorPos.Z,
					AnchorFwd.X, AnchorFwd.Y, AnchorFwd.Z,
					CurveDist,
					(CurrentTurnSignal == ETurnSignal::Left) ? TEXT("LEFT") : TEXT("RIGHT"),
					TargetParkingLot->bIsLeftSide ? TEXT("true") : TEXT("false"),
					RemainDist);
			}
			else
			{
				// --- 預設：右側路肩（原邏輯）---
				// --- Default: right shoulder (original logic) ---
				CurrentTurnSignal = ETurnSignal::Right;
				const int32 ShoulderLane = Seg.DrivingRule.ForwardLaneCount;
				ParkingTargetOffset = ComputeTargetLaneOffset(ShoulderLane);

				UE_LOG(LogTemp, Warning,
					TEXT("[PARK-0.5] DEFAULT | ShoulderLane=%d offset=%.0f remain=%.0f"),
					ShoulderLane, ParkingTargetOffset, RemainDist);
			}
		}
	}

	// ================================================================
	//  0.7 障礙物偵測（統一：前車、紅燈碰撞體、任何障礙物）
	//      Obstacle detection (unified: vehicles, red light blockers, any obstacle)
	// ================================================================
	UpdateObstacleDetection();

	// ================================================================
	//  0.8 超車邏輯（偵測到慢車 → 換道超車 → 超過後切回）
	//      Overtaking logic (slow car → pass → return)
	// ================================================================
	UpdateOvertakeLogic();

	// ================================================================
	//  1. 速度控制：加速到 DesiredSpeed 或煞車
	//     Speed control: accelerate toward desired or brake
	// ================================================================
	const float DesiredSpeed = ComputeDesiredSpeed();

	if (CurrentSpeed < DesiredSpeed)
	{
		// 加速：用 Acceleration 參數，不超過 DesiredSpeed
		// Accelerate using Acceleration param, capped at DesiredSpeed
		CurrentSpeed = FMath::Min(CurrentSpeed + Acceleration * DeltaTime, DesiredSpeed);
	}
	else
	{
		// 減速：用 BrakeDeceleration，不低於 DesiredSpeed
		// Brake with BrakeDeceleration, floored at DesiredSpeed
		CurrentSpeed = FMath::Max(CurrentSpeed - BrakeDeceleration * DeltaTime, DesiredSpeed);
	}

	// 確保不為負 / Ensure non-negative
	CurrentSpeed = FMath::Max(CurrentSpeed, 0.0f);

	// ================================================================
	//  1.5 出發曲線：從停車格自然匯入道路
	//      Departure curve: smoothly merge from parking spot onto road
	// ================================================================
	if (bOnDepartureCurve)
	{
		// 出發曲線用 DepartureCurveSpeedRatio 控制速度
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
			// 出發曲線結束 → 繼續正常 spline 跟隨
			// Departure curve done → continue normal spline following
			bOnDepartureCurve = false;

			UE_LOG(LogTemp, Warning,
				TEXT("[DEPART-CURVE] DONE: pos=(%.0f,%.0f,%.0f) → normal spline following"),
				Owner->GetActorLocation().X, Owner->GetActorLocation().Y, Owner->GetActorLocation().Z);
		}
		return; // 出發曲線進行中 → 跳過正常邏輯 / Departure curve in progress → skip normal logic
	}

	// ================================================================
	//  2. 路口曲線：如果正在走 Hermite 曲線，走完再回到正常 spline
	//     Junction curve: if on Hermite curve, follow it to completion
	// ================================================================
	if (bOnJunctionCurve)
	{
		JCurveProgress += CurrentSpeed * DeltaTime;
		const float T = FMath::Clamp(JCurveProgress / FMath::Max(JCurveLength, 1.0f), 0.0f, 1.0f);

		if (T < 1.0f)
		{
			FVector CurvePos;
			FVector CurveTangent;

			if (bIsUTurnCurve)
			{
				// 半圓弧：P(t) = Center + R*(cos(πt)*U + sin(πt)*V)
				// Semicircle arc: P(t) = Center + R*(cos(πt)*U + sin(πt)*V)
				const float Angle = PI * T;
				const float CosA = FMath::Cos(Angle);
				const float SinA = FMath::Sin(Angle);
				const FVector Flat = UTurnCenter + UTurnArcRadius * (CosA * UTurnAxisU + SinA * UTurnAxisV);
				CurvePos = FVector(Flat.X, Flat.Y, FMath::Lerp(UTurnArcZ0, UTurnArcZ1, T));

				// 切線 dP/dθ = R*(-sin(θ)*U + cos(θ)*V)
				// Tangent dP/dθ = R*(-sin(θ)*U + cos(θ)*V)
				CurveTangent = UTurnArcRadius * (-SinA * UTurnAxisU + CosA * UTurnAxisV);
			}
			else
			{
				// 一般 Hermite 曲線 / Normal Hermite curve
				CurvePos = FMath::CubicInterp(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T);
				CurveTangent = FMath::CubicInterpDerivative(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T);
			}

			// 旋轉用 RInterpTo 平滑過渡，不直接 snap → 轉彎時自然地邊移動邊轉
			// Rotation via RInterpTo for smooth transition, not snap → natural turning
			FRotator FinalRot = Owner->GetActorRotation();
			if (CurveTangent.SizeSquared() > 1.0f)
			{
				const FRotator TargetRot = CurveTangent.Rotation();
				FinalRot = FMath::RInterpTo(FinalRot, TargetRot, DeltaTime, RotationInterpSpeed);
			}
			Owner->SetActorLocationAndRotation(CurvePos, FinalRot);
			return; // 還在曲線上 → 跳過正常 spline 邏輯 / Still on curve → skip normal spline
		}

		// ---- T >= 1.0：曲線結束，直接銜接新段的 Spline 邏輯（同一幀！）----
		// ---- T >= 1.0: Curve done, fall through to new segment's spline logic (same frame!) ----
		bOnJunctionCurve = false;
		bIsUTurnCurve = false;
		CurrentSegmentIndex++;

		if (CurrentSegmentIndex >= PathSegments.Num())
		{
			bIsFollowing = false;
			CurrentSpeed = 0.0f;
			NavState = ENavState::Parked;
			CurrentTurnSignal = ETurnSignal::None;
			UE_LOG(LogTemp, Warning, TEXT("PathFollower: Path completed (curve end) — NavState=Parked"));
			OnPathComplete.Broadcast(true);
			return;
		}

		Seg = PathSegments[CurrentSegmentIndex];
		ReferenceDistance = JCurveNextRefDist;

		// 重置方向燈（新段會重新判斷）/ Reset turn signal
		CurrentTurnSignal = ETurnSignal::None;

		// 更新車道和橫向偏移到新段的正確值
		// Update lane and lateral offset for new segment
		TargetLaneIndex = JCurveNextLaneIndex;
		CurrentLaneIndex = JCurveNextLaneIndex;
		bIsChangingLane = false;
		CurrentLateralOffset = ComputeTargetLaneOffset(TargetLaneIndex);

		// 標記剛出曲線，讓 Step 6 用高 interp speed 消除銜接停頓
		// Flag just-exited so Step 6 uses high interp speed for seamless transition
		bJustExitedJunctionCurve = true;

		UE_LOG(LogTemp, Warning,
			TEXT("JUNCTION_CURVE_END: Entered Seg[%d] Type=%d RefDist=%.0f"),
			CurrentSegmentIndex, Seg.RoadType, ReferenceDistance);

		// 不 return — 直接 fall through 到下面的 Step 3~6
		// DON'T return — fall through to Step 3~6 below
	}

	// ================================================================
	//  3. 推進參考點：沿 spline 前進
	//     Advance reference point along spline
	// ================================================================
	ReferenceDistance += CurrentSpeed * DeltaTime * Seg.Direction;

	// 3a. 自然段切換：若參考點通過 EndDist 且下一段存在，把 overshoot 帶到新段。
	//     這是直線連接的唯一切換點 — 不做瞬間移動，因為兩段在 junction node 相接。
	// 3a. Natural segment advance: when ReferenceDistance passes EndDist and a next
	//     segment exists, carry overshoot forward. This is the ONE place straight
	//     junctions hand off — no teleport, because the two segments meet at the
	//     junction node (world positions are continuous).
	while (CurrentSegmentIndex + 1 < PathSegments.Num())
	{
		const FPathSegmentInternal& CurSeg = PathSegments[CurrentSegmentIndex];
		const bool bPassedEnd =
			(CurSeg.Direction > 0.0f && ReferenceDistance >= CurSeg.EndDist)
			|| (CurSeg.Direction < 0.0f && ReferenceDistance <= CurSeg.EndDist);
		if (!bPassedEnd) break;

		// 只有直線銜接才自動切段；轉彎 / U-turn 要靠 Step 5 建立 Hermite / 半圓弧
		// Only auto-advance for straight junctions; turns / U-turns need Step 5 curves
		if (CurSeg.TurnAtEnd != ETurnSignal::None || CurSeg.bUTurnAtEnd)
		{
			// 把 ReferenceDistance 夾在 EndDist，等 Step 5 的 junction curve 接手
			// Clamp to EndDist, let Step 5's junction curve take over
			ReferenceDistance = CurSeg.EndDist;
			break;
		}

		const float Overshoot = (CurSeg.Direction > 0.0f)
			? (ReferenceDistance - CurSeg.EndDist)
			: (CurSeg.EndDist - ReferenceDistance);

		CurrentSegmentIndex++;
		Seg = PathSegments[CurrentSegmentIndex];
		ReferenceDistance = Seg.StartDist + Overshoot * Seg.Direction;

		// 新段車道數可能不同 → clamp
		// New segment may have different lane count → clamp
		const int32 NewLaneCount = FMath::Max(1, Seg.DrivingRule.ForwardLaneCount);
		TargetLaneIndex = FMath::Clamp(TargetLaneIndex, 0, NewLaneCount - 1);
		CurrentLaneIndex = TargetLaneIndex;
		CurrentTurnSignal = ETurnSignal::None;

		UE_LOG(LogTemp, Warning,
			TEXT("SEG_ADVANCE(straight): → Seg[%d] RefDist=%.0f Overshoot=%.0f LateralOffset=%.0f"),
			CurrentSegmentIndex, ReferenceDistance, Overshoot, CurrentLateralOffset);
	}

	// ================================================================
	//  4. 車道切換插值
	//     Lane change interpolation
	// ================================================================
	const float PrevLateralOffset = CurrentLateralOffset;  // 記錄本幀前的偏移，用於計算車頭朝向 / Save for heading calc
	const float TargetOffset = ComputeTargetLaneOffset(TargetLaneIndex);

	if (bIsChangingLane)
	{
		CurrentLateralOffset = FMath::FInterpConstantTo(
			CurrentLateralOffset, TargetOffset, DeltaTime, LaneChangeSpeed);

		if (FMath::IsNearlyEqual(CurrentLateralOffset, TargetOffset, 1.0f))
		{
			CurrentLateralOffset = TargetOffset;
			CurrentLaneIndex = TargetLaneIndex;
			bIsChangingLane = false;
		}
	}
	else if (NavState == ENavState::Parking
		&& DestinationType == EDestinationType::None)
	{
		// 預設停車：用 lateral offset 漸進靠邊
		// Default parking: gradual lateral offset to target
		CurrentLateralOffset = FMath::FInterpConstantTo(
			CurrentLateralOffset, ParkingTargetOffset, DeltaTime, LaneChangeSpeed);
	}
	else
	{
		CurrentLateralOffset = FMath::FInterpTo(
			CurrentLateralOffset, TargetOffset, DeltaTime, PositionInterpSpeed);
	}

	// ================================================================
	//  5. 取樣位置 + 偵測路口 → 生成 Hermite 曲線
	//     Sample position + detect junction → generate Hermite curve
	// ================================================================
	FVector RefPos, RefDir, RefRight;
	SampleSplineAtDist(Seg, ReferenceDistance, CurrentLateralOffset,
		RefPos, RefDir, RefRight);

	const float DistToEnd = FMath::Abs(Seg.EndDist - ReferenceDistance);
	const bool bNextSegExists = (CurrentSegmentIndex + 1 < PathSegments.Num());

	// 接近路口且下一段存在 → 生成 Hermite 曲線並開始走
	// Approaching junction with next segment → create Hermite curve
	//
	// 多車道轉彎時動態加大 BlendDistance：車在外側車道時離轉角更近，
	// 需要更大的弧線空間。用 CurrentLateralOffset 按比例增加。
	// Dynamic blend for multi-lane turns: outer lanes are closer to the corner,
	// need a bigger arc. Scale up by CurrentLateralOffset.
	// 左轉用完整 BlendDistance，右轉弧度小用 × RightTurnBlendRatio
	// Left turn uses full BlendDistance, right turn uses × RightTurnBlendRatio (smaller arc)
	float EffectiveBlendDist = JunctionBlendDistance;
	if (CurrentTurnSignal == ETurnSignal::Right)
	{
		EffectiveBlendDist *= RightTurnBlendRatio;
	}
	if (CurrentTurnSignal != ETurnSignal::None)
	{
		// 每 100cm 的車道偏移，額外增加 BlendDistance（多車道動態調整）
		// Per 100cm lane offset, boost BlendDistance (multi-lane dynamic adjustment)
		const float OffsetBoost = (CurrentLateralOffset / 100.0f) * OffsetBoostRate;
		EffectiveBlendDist *= (1.0f + FMath::Max(OffsetBoost, 0.0f));
	}

	if (bNextSegExists && DistToEnd < EffectiveBlendDist)
	{
		const FPathSegmentInternal& NextSeg = PathSegments[CurrentSegmentIndex + 1];

		// ---- Dot 檢查：用 Seg 出口切線 vs NextSeg 入口切線，過濾直線 ----
		// ---- Dot gate: InSeg end tangent vs OutSeg start tangent, filter straight segments ----
		// 注意：U-turn 時還是走 junction curve（需要那條曲線去掉頭），
		// 只有「幾乎直線」才跳過 curve。
		// Note: U-turns still use the curve (needed to actually turn around);
		// only near-straight junctions skip the curve.
		float JDot = 1.0f, JCross = 0.0f;
		ETurnSignal TmpTurn;
		bool bTmpU;
		ComputeTurnAtJunction(Seg, NextSeg, TmpTurn, bTmpU, &JDot, &JCross);

		const bool bSkipStraight = (JDot > JunctionStraightDot);

		if (bSkipStraight)
		{
			// 直線：不建立 Hermite 曲線，也不在這裡切段。
			// 讓車子繼續沿 OldSeg 以當前速度行駛，Step 3a 會在
			// ReferenceDistance 真正到達 EndDist 時自然切到 NextSeg，
			// 世界座標完全連續 → 零瞬移。
			// Straight: don't create curve, don't advance segment here.
			// Car keeps driving on OldSeg; Step 3a naturally advances when
			// ReferenceDistance reaches EndDist → zero teleport.
		}
		else
		{
		UE_LOG(LogTemp, Warning,
			TEXT("JUNCTION_DOT_CHECK: Dot=%.3f CrossZ=%.3f StraightDot=%.3f UTurnDot=%.3f Turn=%s%s TurnSignal=%d"),
			JDot, JCross, JunctionStraightDot, JunctionUTurnDot,
			(TmpTurn == ETurnSignal::Left) ? TEXT("LEFT") :
			(TmpTurn == ETurnSignal::Right) ? TEXT("RIGHT") : TEXT("NONE"),
			bTmpU ? TEXT(" [U-TURN]") : TEXT(""),
			(int32)CurrentTurnSignal);

		// 曲線終點：深入下一段 EffectiveBlendDist 的位置
		// Curve end: EffectiveBlendDist into next segment
		const float EntryDist = EffectiveBlendDist;
		const float NextSampleDist = NextSeg.StartDist + EntryDist * NextSeg.Direction;

		// 根據轉彎方向決定新段的目標車道
		// Determine target lane in next segment based on turn direction
		const int32 NextLaneCount = NextSeg.DrivingRule.ForwardLaneCount;
		if (CurrentTurnSignal == ETurnSignal::Right)
		{
			// 右轉 → 新段最外側車道 / Right turn → outermost lane
			JCurveNextLaneIndex = NextLaneCount - 1;
		}
		else if (CurrentTurnSignal == ETurnSignal::Left)
		{
			// 左轉 → 對應車道，超出則取最大 / Left turn → same lane, clamped
			JCurveNextLaneIndex = FMath::Min(TargetLaneIndex, NextLaneCount - 1);
		}
		else
		{
			// 直行 → 對應車道，超出則取最大 / Straight → same lane, clamped
			JCurveNextLaneIndex = FMath::Min(TargetLaneIndex, NextLaneCount - 1);
		}

		// 用新段目標車道的 offset 取樣曲線終點
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

		// 建立 Hermite 曲線
		// P0 = 車的實際位置（不是 spline 參考點），消除 VInterpTo 延遲造成的跳躍
		// P0 = car's ACTUAL position (not spline ref), eliminates VInterpTo lag jump
		// T0 = 車的實際朝向，確保曲線從車目前的行進方向開始
		// T0 = car's ACTUAL forward, ensures curve starts from current heading
		JCurveP0 = Owner->GetActorLocation();

		// U 型掉頭：用 UTurnRadius 把終點偏移到旁邊，產生大 U 曲線
		// U-turn: offset endpoint sideways by UTurnRadius to create a wide U-shape
		bIsUTurnCurve = Seg.bUTurnAtEnd;
		if (bIsUTurnCurve)
		{
			/*
			// U 型掉頭：以 UTurnRadius 為半徑建立一個寬大的 U 曲線
// U-turn: build a wide U-shape curve of radius UTurnRadius.
//
// 策略：把切線同時往前 + 往側邊拉，讓 Hermite 曲線自然地弧出去再繞回來，
// 而不是用反向切線硬折出尖角。側邊方向由 TurnSignal 決定。
// Strategy: pull both tangents forward + sideways so the Hermite naturally
// bulges outward and loops back, instead of cusping with antiparallel tangents.
// Sideways direction depends on the TurnSignal.
			const FVector OwnerFwd = Owner->GetActorForwardVector();
			const FVector OwnerRight = Owner->GetActorRightVector();
			const FVector NextFwd = NextDir.GetSafeNormal();
			const FVector NextRightDir = FVector::CrossProduct(NextFwd, FVector::UpVector).GetSafeNormal();

			// 根據轉彎方向決定 U 型偏向（左手或右手）
			// Sideways bias direction for the U bulge (left-hand or right-hand U).
			const float SideSign = (CurrentTurnSignal == ETurnSignal::Left) ? -1.0f : 1.0f;

			JCurveP1 = NextPos;

			const float Dist = (JCurveP1 - JCurveP0).Size();
			// 切線長度 = UTurnRadius × UTurnTangentScale，用 UTurnRadius 主宰曲線寬度
			// Tangent length = UTurnRadius × UTurnTangentScale — UTurnRadius dominates width.
			const float UTurnTangentLen = UTurnRadius * UTurnTangentScale;

			// T0：沿當前前方 + 側向偏移 → 讓曲線一開始就往外弧
			// T0: forward + sideways bias → curve bulges outward at entry
			const FVector T0Dir = (OwnerFwd + OwnerRight * SideSign * 0.6f).GetSafeNormal();
			JCurveT0 = T0Dir * UTurnTangentLen;

			// T1：沿下一段前方 - 側向偏移 → 讓曲線尾端從外側順順收回來
			// T1: next-segment forward - sideways bias → curve sweeps back inward at exit
			const FVector T1Dir = (NextFwd + NextRightDir * SideSign * 0.6f).GetSafeNormal();
			JCurveT1 = T1Dir * UTurnTangentLen;

			// U 型曲線長度 ~ π × UTurnRadius（半圓周長）
			// U-turn curve length ~ π × UTurnRadius (semicircle perimeter)
			JCurveLength = FMath::Max(Dist * 1.5f, UTurnRadius * PI);
			*/
			// U 型掉頭：用參數化半圓弧 (parametric semicircle)，不用 Hermite。
			// Hermite 在反向切線下容易產生 8 字形；半圓弧保證自然平滑。
			// U-turn: parametric semicircle instead of Hermite.
			// Hermite with antiparallel tangents creates figure-8; semicircle is natural.
			//
			// P0 = 車目前位置, P1 = 下一段取樣位置
			// Center = (P0+P1)/2, 半徑 R = max(|P0-P1|/2, UTurnRadius)
			// U 軸 = Center→P0 方向, V 軸 = 車子前方（掃掠方向）
			// 角度 0..π 沿 U/V 畫半圓
			JCurveP1 = NextPos;

			const FVector MidPt = (JCurveP0 + JCurveP1) * 0.5f;
			const float HalfDist = (JCurveP1 - JCurveP0).Size2D() * 0.5f;
			// 半徑 = max(半距離, UTurnRadius)，UTurnRadius 控制最小 U 寬度
			// Radius = max(half-distance, UTurnRadius) — UTurnRadius sets minimum U width
			UTurnArcRadius = FMath::Max(HalfDist, UTurnRadius);

			// U 軸 = 圓心→P0（2D），歸一化
			// U axis = Center→P0 (2D), normalized
			const FVector ToP0 = JCurveP0 - MidPt;
			UTurnAxisU = FVector(ToP0.X, ToP0.Y, 0.0f).GetSafeNormal();
			if (UTurnAxisU.IsNearlyZero())
			{
				// P0≈P1 → fallback 用右方向 / fallback to right vector
				UTurnAxisU = Owner->GetActorRightVector();
			}

			// V 軸 = 車子前方投影到水平面（掃掠方向）
			// V axis = car's forward projected to horizontal (sweep direction)
			const FVector OwnerFwd2D = FVector(Owner->GetActorForwardVector().X,
				Owner->GetActorForwardVector().Y, 0.0f).GetSafeNormal();
			UTurnAxisV = OwnerFwd2D;

			// 如果 UTurnRadius > HalfDist，圓心需要偏移讓弧經過 P0 和 P1
			// If UTurnRadius > HalfDist, offset center so arc passes through P0 & P1
			if (UTurnArcRadius > HalfDist + 1.0f)
			{
				// 圓心從 MidPt 沿 -V 方向後退，使半徑 R 的圓通過 P0 和 P1
				// Center shifts backward along -V so a circle of radius R passes through P0 & P1
				const float BackDist = FMath::Sqrt(FMath::Max(UTurnArcRadius * UTurnArcRadius - HalfDist * HalfDist, 0.0f));
				UTurnCenter = MidPt - UTurnAxisV * BackDist;
			}
			else
			{
				UTurnCenter = MidPt;
			}

			// 重新計算 U 軸（從實際圓心到 P0）
			// Recalculate U axis from actual center to P0
			const FVector CToP0 = FVector(JCurveP0.X - UTurnCenter.X, JCurveP0.Y - UTurnCenter.Y, 0.0f);
			if (CToP0.SizeSquared() > 1.0f)
			{
				UTurnAxisU = CToP0.GetSafeNormal();
			}
			// V 軸 = U 軸旋轉 90° 朝車前方（確保掃掠方向正確）
			// V axis = U rotated 90° toward car forward (ensure correct sweep direction)
			const FVector CandidateV = FVector(-UTurnAxisU.Y, UTurnAxisU.X, 0.0f);
			UTurnAxisV = (FVector::DotProduct(CandidateV, OwnerFwd2D) >= 0.0f) ? CandidateV : -CandidateV;

			// 記錄 Z 高度，線性插值 / Record Z heights for linear lerp
			UTurnArcZ0 = JCurveP0.Z;
			UTurnArcZ1 = JCurveP1.Z;

			// 曲線長度 = π × R × UTurnTangentScale（可調速度感）
			// Arc length = π × R × UTurnTangentScale (tunable speed feel)
			JCurveLength = PI * UTurnArcRadius * UTurnTangentScale;

			UE_LOG(LogTemp, Warning,
				TEXT("UTURN_ARC_INIT: P0=(%.0f,%.0f) P1=(%.0f,%.0f) Center=(%.0f,%.0f) R=%.0f ArcLen=%.0f"),
				JCurveP0.X, JCurveP0.Y, JCurveP1.X, JCurveP1.Y,
				UTurnCenter.X, UTurnCenter.Y, UTurnArcRadius, JCurveLength);

			
		}
		else
		{
			JCurveP1 = NextPos;

			// 切線強度 = 距離 × JunctionCurveTangentScale，控制曲線弧度
			// Tangent magnitude = distance × TangentScale, controls curve roundness
			const float TangentScale = (JCurveP1 - JCurveP0).Size() * JunctionCurveTangentScale;
			JCurveT0 = Owner->GetActorForwardVector() * TangentScale;
			JCurveT1 = NextDir.GetSafeNormal() * TangentScale;

			// 近似曲線長度（比直線長一點）
			// Approximate curve length (slightly longer than straight line)
			JCurveLength = (JCurveP1 - JCurveP0).Size() * 1.2f;
		}

		JCurveProgress = 0.0f;
		JCurveNextRefDist = NextSampleDist;

		bOnJunctionCurve = true;

		UE_LOG(LogTemp, Warning,
			TEXT("JUNCTION_CURVE_START: P0→P1 dist=%.0f CurveLen=%.0f NextRefDist=%.0f CurOffset=%.0f NextOffset=%.0f TurnSignal=%d NextLanes=%d NextLaneIdx=%d"),
			(JCurveP1 - JCurveP0).Size(), JCurveLength, JCurveNextRefDist,
			CurrentLateralOffset, NextSegLaneOffset,
			(int32)CurrentTurnSignal, NextLaneCount, JCurveNextLaneIndex);

		// 不做「第一步 SetActorLocationAndRotation」— P0 已設為車子當前位置，
		// JCurveProgress 從 0 開始，下一幀 Tick 的 JCurve 更新會以 CurrentSpeed 沿曲線
		// 平滑推進，沒有 1 幀停頓也沒有瞬間移動。
		// No "first-step SetActorLocationAndRotation" — P0 is already the car's current
		// pose and JCurveProgress starts at 0; the next Tick's curve update advances
		// smoothly at CurrentSpeed with neither a 1-frame stall nor a teleport.
		JCurveProgress = 0.0f;
		return;
		} // else (not straight → curve)
	}

	// ---- 檢查段結束（沒有下一段時才會到這裡）----
	// ---- Check segment end (only reached when no next segment) ----
	const bool bSegDone =
		(Seg.Direction > 0.0f && ReferenceDistance >= Seg.EndDist)
		|| (Seg.Direction < 0.0f && ReferenceDistance <= Seg.EndDist);

	if (bSegDone)
	{
		if (NavState == ENavState::Parking && !bOnParkingCurve
			&& DestinationType == EDestinationType::ParkingLot)
		{
			// --- 停車場：啟動停車 Hermite 曲線（完整弧線入庫）---
			// --- Parking Lot: full Hermite curve into spot ---
			ParkCurveP0 = Owner->GetActorLocation();
			const float CurveDist = (ParkCurveP1 - ParkCurveP0).Size();
			ParkCurveT0 = Owner->GetActorForwardVector() * CurveDist * ParkingCurveTangentScale;
			ParkCurveLength = CurveDist * 1.3f;
			ParkCurveProgress = 0.0f;
			bOnParkingCurve = true;

			UE_LOG(LogTemp, Warning,
				TEXT("[PARK-NAV] [CURVE] LOT START: P0=(%.0f,%.0f,%.0f) P1=(%.0f,%.0f,%.0f) CurveLen=%.0f"),
				ParkCurveP0.X, ParkCurveP0.Y, ParkCurveP0.Z,
				ParkCurveP1.X, ParkCurveP1.Y, ParkCurveP1.Z,
				ParkCurveLength);
		}
		else if (!bOnParkingCurve)
		{
			// 非目的地停車或非 Parking 模式：直接結束
			// No destination parking or not parking: end immediately
			bIsFollowing = false;
			CurrentSpeed = 0.0f;
			NavState = ENavState::Parked;
			CurrentTurnSignal = ETurnSignal::None;
			UE_LOG(LogTemp, Warning, TEXT("PathFollower: Path completed (last seg) — NavState=Parked"));
			OnPathComplete.Broadcast(true);
			return;
		}
	}

	// ---- 停車曲線跟隨：Hermite curve from road to spot ----
	if (bOnParkingCurve)
	{
		// 停車曲線速度 = MaxSpeed × ParkingCurveSpeedRatio
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
			// 曲線結束 → 停車完成
			// Curve done → parking complete
			bOnParkingCurve = false;
			bIsFollowing = false;
			CurrentSpeed = 0.0f;
			NavState = ENavState::Parked;
			CurrentTurnSignal = ETurnSignal::None;

			UE_LOG(LogTemp, Warning,
				TEXT("[PARK-CURVE] DONE: Final pos=(%.0f,%.0f,%.0f) Yaw=%.1f"),
				Owner->GetActorLocation().X, Owner->GetActorLocation().Y, Owner->GetActorLocation().Z,
				Owner->GetActorRotation().Yaw);
			OnPathComplete.Broadcast(true);
		}
		return;
	}

	// ================================================================
	//  6. 正常行駛 — 直接定位到 Spline 位置 + 旋轉跟隨切線
	//     Normal driving — snap to spline position + rotation follows tangent
	// ================================================================
	const FRotator CurrentRot = Owner->GetActorRotation();

	// 位置：直接用 Spline 取樣位置（不做 VInterpTo）
	// 車的位移完全由 ReferenceDistance 驅動（Step 3），每幀等速 → 零抖動
	// Position: directly use spline sample (no VInterpTo)
	// Movement driven entirely by ReferenceDistance (Step 3), constant per frame → zero jitter
	const FVector FinalPos = RefPos;

	// 旋轉：用「實際移動方向」（含橫向偏移分量），車斜切時車頭會自然轉向
	// Rotation: use ACTUAL movement direction (including lateral component)
	// so the car nose naturally turns when merging sideways
	FRotator FinalRot = CurrentRot;
	if (CurrentSpeed > 1.0f && RefDir.SizeSquared() > 0.1f)
	{
		FVector ActualMoveDir = RefDir.GetSafeNormal();

		// 如果這一幀橫向偏移有變化，把橫向速度混入行進方向
		// If lateral offset changed this frame, blend lateral velocity into heading
		const float FrameLateralDelta = CurrentLateralOffset - PrevLateralOffset;
		if (FMath::Abs(FrameLateralDelta) > 0.01f && DeltaTime > SMALL_NUMBER)
		{
			const float LateralVel = FrameLateralDelta / DeltaTime;
			// 實際移動 = 前進方向 × 前進速度 + 右方向 × 橫向速度
			// Actual movement = forward × speed + right × lateral velocity
			ActualMoveDir = (RefDir.GetSafeNormal() * CurrentSpeed + RefRight * LateralVel).GetSafeNormal();
		}

		const FRotator TargetRot = ActualMoveDir.Rotation();
		FinalRot = FMath::RInterpTo(
			CurrentRot, TargetRot, DeltaTime, RotationInterpSpeed);
	}

	Owner->SetActorLocationAndRotation(FinalPos, FinalRot);

}

// ============================================================================
//  UpdateObstacleDetection — SphereTrace 偵測前方障礙物
//  SphereTrace forward obstacle detection (unified: vehicles, red light blockers, any obstacle)
// ============================================================================
void URoadPathFollowerComponent::UpdateObstacleDetection()
{
	ObstacleDistance = -1.0f;
	ObstacleActor = nullptr;

	UWorld* World = GetWorld();
	if (!World) return;

	AActor* Owner = GetOwner();
	if (!Owner) return;

	// 起點抬高 ObstacleTraceZOffset（避免打到地形、路面凸起）
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

		// Debug log：記錄哪台車被什麼物體擋到
		// Debug log: which vehicle is blocked by what actor
		if (bDebugObstacleTrace)
		{
			const FString OwnerName = Owner->GetName();
			const FString BlockerName = ObstacleActor.IsValid() ? ObstacleActor->GetName() : TEXT("Unknown");
			const FString BlockerClass = ObstacleActor.IsValid() ? ObstacleActor->GetClass()->GetName() : TEXT("?");
			const FString HitComp = Hit.GetComponent() ? Hit.GetComponent()->GetName() : TEXT("?");
			UE_LOG(LogTemp, Warning,
				TEXT("[ObstacleTrace] %s blocked by %s (%s) comp=%s dist=%.0f cm"),
				*OwnerName, *BlockerName, *BlockerClass, *HitComp, ObstacleDistance);
		}
	}
}

// ============================================================================
//  ComputeObstacleSpeedLimit — 障礙物兩段式限速（N 減速，M 停車）
//  Two-stage obstacle speed limit (N slowdown, M full stop)
// ============================================================================
float URoadPathFollowerComponent::ComputeObstacleSpeedLimit() const
{
	if (ObstacleDistance < 0.0f)
	{
		// 沒偵測到障礙物 / No obstacle detected
		return MaxSpeed;
	}

	if (ObstacleDistance <= ObstacleStopDistance)
	{
		// 太近 → 完全停車 / Too close → full stop
		return 0.0f;
	}

	if (ObstacleDistance >= ObstacleSlowdownDistance)
	{
		// 夠遠 → 不限速 / Far enough → no limit
		return MaxSpeed;
	}

	// N~M 之間線性減速 / Linear deceleration between N and M
	const float Alpha = (ObstacleDistance - ObstacleStopDistance)
		/ (ObstacleSlowdownDistance - ObstacleStopDistance);
	return MaxSpeed * Alpha;
}

// ============================================================================
//  IsPassingLaneClear — 超車道安全偵測
//  Check if passing lane is clear using SphereTrace.
// ============================================================================
bool URoadPathFollowerComponent::IsPassingLaneClear(int32 PassingLaneIndex) const
{
	UWorld* World = GetWorld();
	if (!World) return false;

	AActor* Owner = GetOwner();
	if (!Owner) return false;

	if (CurrentSegmentIndex >= PathSegments.Num()) return false;

	// 計算超車道偏移量 / Compute passing lane offset
	const float PassingOffset = ComputeTargetLaneOffset(PassingLaneIndex);
	const float OffsetDelta = PassingOffset - CurrentLateralOffset;

	// SphereTrace 起點 = 車位 + 偏移方向（模擬在超車道的位置���
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
//  UpdateOvertakeLogic — 超車狀態機
//  Overtake state machine: None → Passing → Returning → None
// ============================================================================
void URoadPathFollowerComponent::UpdateOvertakeLogic()
{
	if (CurrentSegmentIndex >= PathSegments.Num()) return;

	const FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];
	const float DistToJunction = GetDistanceToNextJunction();

	switch (OvertakeState)
	{
	case EOvertakeState::None:
	{
		// ① 前方有障礙物且是車輛（有 PathFollowerComponent）
		// ① Obstacle ahead and it's a vehicle (has PathFollowerComponent)
		if (!ObstacleActor.IsValid()) break;

		URoadPathFollowerComponent* FrontPF =
			ObstacleActor->FindComponentByClass<URoadPathFollowerComponent>();
		if (!FrontPF) break; // 不是車（紅燈碰撞體等）→ 不超車 / Not a vehicle → don't overtake

		// ② 前車速度比我慢 / Front vehicle is slower
		if (FrontPF->GetCurrentSpeed() >= MaxSpeed * OvertakeSpeedThreshold) break;

		// ③ 當前路段允許超車 / Current road allows overtaking
		if (!Seg.DrivingRule.bAllowOvertaking) break;

		// ④ 車道數不足則不超車 / Not enough lanes for overtaking
		if (Seg.DrivingRule.ForwardLaneCount < OvertakeMinLaneCount) break;

		// ⑤ 不在路口曲線 && 離路口夠遠 / Not on junction curve && far from junction
		if (bOnJunctionCurve) break;
		if (DistToJunction < AutoLaneChangeDistance) break;

		// ⑥ 已在換道中 → 不重複觸發 / Already changing lane → don't trigger again
		if (bIsChangingLane) break;

		// 決定超車道 / Determine passing lane
		int32 PassingLane;
		if (Seg.DrivingRule.bPreferLeftLaneForPassing)
		{
			// 左車道（靠中心）= lane index 更小 / Left lane (closer to center) = lower index
			PassingLane = FMath::Max(0, CurrentLaneIndex - 1);
		}
		else
		{
			PassingLane = FMath::Min(CurrentLaneIndex + 1, Seg.DrivingRule.ForwardLaneCount - 1);
		}

		// 已在超車道 → 無法再超 / Already in passing lane
		if (PassingLane == CurrentLaneIndex) break;

		// ⑦ 超車道安全 / Passing lane is clear
		if (!IsPassingLaneClear(PassingLane)) break;

		// 觸發超車 / Initiate overtake
		PreOvertakeLaneIndex = CurrentLaneIndex;
		OvertakePassedDistAccum = 0.0f;
		RequestLaneChange(PassingLane);
		OvertakeState = EOvertakeState::Passing;

		// 亮左方向燈 / Left turn signal
		CurrentTurnSignal = ETurnSignal::Left;

		UE_LOG(LogTemp, Warning,
			TEXT("PathFollower: OVERTAKE START — Lane %d → %d | FrontSpeed=%.0f MyMax=%.0f"),
			PreOvertakeLaneIndex, PassingLane, FrontPF->GetCurrentSpeed(), MaxSpeed);
		break;
	}

	case EOvertakeState::Passing:
	{
		// 路口附近 → 強制取消超車，切回原車道
		// Near junction → force cancel, return to original lane
		if (DistToJunction < AutoLaneChangeDistance)
		{
			RequestLaneChange(PreOvertakeLaneIndex);
			OvertakeState = EOvertakeState::Returning;
			CurrentTurnSignal = ETurnSignal::Right;
			UE_LOG(LogTemp, Warning,
				TEXT("PathFollower: OVERTAKE CANCEL (near junction) — returning to Lane %d"),
				PreOvertakeLaneIndex);
			break;
		}

		// 檢查是否已超過前車 / Check if passed the front vehicle
		// 前方 SphereTrace 不再偵測到原障礙物 → 已超過
		// Front SphereTrace no longer detects original obstacle → passed
		bool bPassed = !ObstacleActor.IsValid() || (ObstacleDistance < 0.0f);

		// 也可能偵測到不同的車 → 也算超過了原車
		// May detect a different vehicle → also counts as passed original
		if (ObstacleActor.IsValid() && ObstacleActor->FindComponentByClass<URoadPathFollowerComponent>())
		{
			// 如果偵測到的是不同車，也算超過了
			// 但如果是同一台，還沒超過
		}

		if (bPassed)
		{
			// 累計安全距離 / Accumulate safe distance
			OvertakePassedDistAccum += CurrentSpeed * GetWorld()->GetDeltaSeconds();

			if (OvertakePassedDistAccum >= OvertakeSafeDistance)
			{
				// 切回原車道 / Return to original lane
				RequestLaneChange(PreOvertakeLaneIndex);
				OvertakeState = EOvertakeState::Returning;
				CurrentTurnSignal = ETurnSignal::Right;

				UE_LOG(LogTemp, Warning,
					TEXT("PathFollower: OVERTAKE PASSED — returning to Lane %d after %.0f cm"),
					PreOvertakeLaneIndex, OvertakePassedDistAccum);
			}
		}
		break;
	}

	case EOvertakeState::Returning:
	{
		// 換道完成 → 超車結束 / Lane change complete → overtake done
		if (!bIsChangingLane)
		{
			OvertakeState = EOvertakeState::None;
			// 方向燈交給 Step 0 的 TurnAtEnd 邏輯接管 / Turn signal handled by Step 0
			CurrentTurnSignal = ETurnSignal::None;

			UE_LOG(LogTemp, Warning,
				TEXT("PathFollower: OVERTAKE COMPLETE — back in Lane %d"), CurrentLaneIndex);
		}
		break;
	}
	}
}
