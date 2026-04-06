#include "RoadPathFollowerComponent.h"
#include "RoadNetworkSubsystem.h"
#include "RoadRuleLibrary.h"
#include "RoadWorldSettings.h"
#include "CollisionChannels.h"
#include "Components/SplineComponent.h"
#include "Engine/World.h"

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
	bJustExitedJunctionCurve = false;
	bIsChangingLane = false;
	bParkingStraightening = false;
	JCurveProgress = 0.0f;
	ParkingStraightenRemain = 0.0f;
	ParkingTargetOffset = 0.0f;
	CurrentSpeed = 0.0f;
	CurrentTurnSignal = ETurnSignal::None;
	NavState = ENavState::Idle;
	PathSegments.Empty();
	CurrentSegmentIndex = 0;
	ObstacleDistance = -1.0f;
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

	const FRoadGraphPath AStarPath = Sub->FindPathAStar(FromNodeId, ToNodeId);
	if (!AStarPath.bPathFound)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: A* failed %d → %d"), FromNodeId, ToNodeId);
		OnPathComplete.Broadcast(false);
		return;
	}

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: A* path %d → %d | %d nodes | Cost=%.1f"),
		FromNodeId, ToNodeId, AStarPath.NodePath.Num(), AStarPath.TotalCost);

	BuildPathSegments(AStarPath);

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

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: Started | %d segs | MaxSpeed=%.0f | Lane=%d | LateralOffset=%.0f"),
		PathSegments.Num(), MaxSpeed, CurrentLaneIndex, CurrentLateralOffset);
}

// ============================================================================
//  StartFollowing — 用寫死的 StartNodeId / GoalNodeId（測試用）
//  Start following using hardcoded node IDs (for testing).
// ============================================================================
void URoadPathFollowerComponent::StartFollowing()
{
	DestinationNodeId = GoalNodeId;
	StartNavigationInternal(StartNodeId, GoalNodeId);
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

	// 用車目前位置找最近 node 作為起點
	// Snap car's current position to nearest graph node as start
	const int32 FromNode = Sub->FindNearestGraphNode(Owner->GetActorLocation());
	if (FromNode == INDEX_NONE)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: Cannot find nearest node for car position"));
		OnPathComplete.Broadcast(false);
		return;
	}

	// 記錄目的地資訊 / Store destination info
	DestinationNodeId = TargetNodeId;
	const TArray<FRoadGraphNode>& Nodes = Sub->GetGraphNodes();
	if (TargetNodeId >= 0 && TargetNodeId < Nodes.Num())
	{
		DestinationWorldLocation = Nodes[TargetNodeId].WorldLocation;
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

	// ---- 用 Graph Node 世界座標預計算每個路口的轉彎方向 ----
	// ---- Precompute turn direction at each junction using graph node world positions ----
	// 三點法：NodeA(段起點) → NodeB(路口) → NodeC(下一段終點)
	// Three-point method: NodeA(seg start) → NodeB(junction) → NodeC(next seg end)
	const TArray<FRoadGraphNode>& GraphNodeList = Sub->GetGraphNodes();
	for (int32 i = 0; i + 1 < PathSegments.Num(); ++i)
	{
		// Nodes: [0]─Seg[0]─[1]─Seg[1]─[2]─Seg[2]─[3]...
		// 段 i 的路口 = Nodes[i+1]
		const int32 IdA = Nodes[i];
		const int32 IdB = Nodes[i + 1];
		const int32 IdC = Nodes[i + 2];

		// 安全檢查：確保 NodeId 在範圍內
		if (IdA >= GraphNodeList.Num() || IdB >= GraphNodeList.Num() || IdC >= GraphNodeList.Num())
		{
			PathSegments[i].TurnAtEnd = ETurnSignal::None;
			continue;
		}

		const FVector PosA = GraphNodeList[IdA].WorldLocation;
		const FVector PosB = GraphNodeList[IdB].WorldLocation;
		const FVector PosC = GraphNodeList[IdC].WorldLocation;

		const FVector DirAB = (PosB - PosA).GetSafeNormal2D();
		const FVector DirBC = (PosC - PosB).GetSafeNormal2D();
		const float Dot = FVector::DotProduct(DirAB, DirBC);
		const float CrossZ = FVector::CrossProduct(DirAB, DirBC).Z;

		if (Dot > 0.985f)
		{
			PathSegments[i].TurnAtEnd = ETurnSignal::None;
		}
		else
		{
			// UE 左手座標系：+Y = 右，CrossZ > 0 = 右轉，< 0 = 左轉
			// UE left-handed coords: +Y = right, CrossZ > 0 = right turn, < 0 = left
			PathSegments[i].TurnAtEnd = (CrossZ > 0.0f) ? ETurnSignal::Right : ETurnSignal::Left;
		}

		UE_LOG(LogTemp, Warning,
			TEXT("  Seg[%d] TurnAtEnd=%s"),
			i,
			(PathSegments[i].TurnAtEnd == ETurnSignal::Left) ? TEXT("LEFT") :
			(PathSegments[i].TurnAtEnd == ETurnSignal::Right) ? TEXT("RIGHT") : TEXT("NONE"));
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
				MaxSpeed, MaxSpeed * JunctionMinSpeedRatio, Alpha);
			Desired = FMath::Min(Desired, JunctionSpeed);
		}
	}

	// 曲線全程維持最低速，出曲線後才由 FInterpTo 慢慢加速
	// Hold min speed for entire curve. Post-curve FInterpTo ramps up naturally.
	if (bOnJunctionCurve)
	{
		Desired = FMath::Min(Desired, MaxSpeed * JunctionMinSpeedRatio);
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
//  TickComponent — 主迴圈
//  Main update loop: speed control → advance reference → pursuit → interpolate.
// ============================================================================
void URoadPathFollowerComponent::TickComponent(
	float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// 第一次 Tick 時啟動（road graph 此時已建好）
	// First Tick: road graph is ready, start now
	if (bPendingStart)
	{
		bPendingStart = false;
		StartFollowing();
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
		// 方向燈：直接讀 BuildPathSegments 預計算的 TurnAtEnd
		// 進入段的第一幀就知道轉彎方向，不需要等接近路口
		// Turn signal: read precomputed TurnAtEnd immediately on segment entry
		const ETurnSignal UpcomingTurn = Seg.TurnAtEnd;
		if (UpcomingTurn != ETurnSignal::None && CurrentTurnSignal != UpcomingTurn)
		{
			CurrentTurnSignal = UpcomingTurn;
			UE_LOG(LogTemp, Warning,
				TEXT("PathFollower: Turn signal → %s (Seg[%d])"),
				(CurrentTurnSignal == ETurnSignal::Left) ? TEXT("LEFT") : TEXT("RIGHT"),
				CurrentSegmentIndex);
		}
		else if (UpcomingTurn == ETurnSignal::None && !bOnJunctionCurve)
		{
			CurrentTurnSignal = ETurnSignal::None;
		}

		// 自動換道：用 AutoLaneChangeDistance（比減速更早），讓車有時間完成換道
		// Auto lane change: use AutoLaneChangeDistance (earlier than slowdown)
		const float DistToJunction = GetDistanceToNextJunction();
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
	//  0.5 路邊停車：最後一段 → 直接偏移到路肩
	//      Roadside parking: last segment → offset directly to shoulder
	// ================================================================
	if (NavState == ENavState::Driving
		&& CurrentSegmentIndex == PathSegments.Num() - 1
		&& ParkingMode == EParkingMode::RoadsideStop)
	{
		const float RemainDist = GetRemainingDistance();
		if (RemainDist < ParkingPullOverDistance)
		{
			NavState = ENavState::Parking;

			// 亮右方向燈 / Right turn signal
			CurrentTurnSignal = ETurnSignal::Right;

			// 停車目標 = 虛擬 lane N（超出最外車道一格，就是路肩位置）
			// Parking target = virtual lane N (one beyond outermost = shoulder)
			const int32 ShoulderLane = Seg.DrivingRule.ForwardLaneCount;  // 2-lane → index 2
			ParkingTargetOffset = ComputeTargetLaneOffset(ShoulderLane);

			UE_LOG(LogTemp, Warning,
				TEXT("PathFollower: Parking — ShoulderLane=%d TargetOffset=%.0f CurOffset=%.0f remain=%.0f cm"),
				ShoulderLane, ParkingTargetOffset, CurrentLateralOffset, RemainDist);
		}
	}

	// ================================================================
	//  0.7 障礙物偵測（統一：前車、紅燈碰撞體、任何障礙物）
	//      Obstacle detection (unified: vehicles, red light blockers, any obstacle)
	// ================================================================
	UpdateObstacleDetection();

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
	//  2. 路口曲線：如果正在走 Hermite 曲線，走完再回到正常 spline
	//     Junction curve: if on Hermite curve, follow it to completion
	// ================================================================
	if (bOnJunctionCurve)
	{
		JCurveProgress += CurrentSpeed * DeltaTime;
		const float T = FMath::Clamp(JCurveProgress / FMath::Max(JCurveLength, 1.0f), 0.0f, 1.0f);

		if (T < 1.0f)
		{
			// 曲線進行中 — 直接定位，不經 VInterpTo
			// Curve in progress — direct positioning, no VInterpTo
			const FVector CurvePos = FMath::CubicInterp(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T);
			const FVector CurveTangent = FMath::CubicInterpDerivative(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T);
			FRotator FinalRot = Owner->GetActorRotation();
			if (CurveTangent.SizeSquared() > 1.0f)
			{
				FinalRot = CurveTangent.Rotation();
			}
			Owner->SetActorLocationAndRotation(CurvePos, FinalRot);
			return; // 還在曲線上 → 跳過正常 spline 邏輯 / Still on curve → skip normal spline
		}

		// ---- T >= 1.0：曲線結束，直接銜接新段的 Spline 邏輯（同一幀！）----
		// ---- T >= 1.0: Curve done, fall through to new segment's spline logic (same frame!) ----
		bOnJunctionCurve = false;
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

	// ================================================================
	//  4. 車道切換插值
	//     Lane change interpolation
	// ================================================================
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
	else if (NavState == ENavState::Parking)
	{
		// 停車中：直接平滑移動到路肩目標位置
		// Parking: smoothly move to shoulder target (computed in Step 0.5)
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
		JCurveP1 = NextPos;

		// 切線強度 = 距離 × JunctionCurveTangentScale，控制曲線弧度
		// Tangent magnitude = distance × TangentScale, controls curve roundness
		const float TangentScale = (JCurveP1 - JCurveP0).Size() * JunctionCurveTangentScale;
		JCurveT0 = Owner->GetActorForwardVector() * TangentScale;
		JCurveT1 = NextDir.GetSafeNormal() * TangentScale;

		// 近似曲線長度（比直線長一點）
		// Approximate curve length (slightly longer than straight line)
		JCurveLength = (JCurveP1 - JCurveP0).Size() * 1.2f;
		JCurveProgress = 0.0f;
		JCurveNextRefDist = NextSampleDist;

		bOnJunctionCurve = true;

		UE_LOG(LogTemp, Warning,
			TEXT("JUNCTION_CURVE_START: P0→P1 dist=%.0f CurveLen=%.0f NextRefDist=%.0f CurOffset=%.0f NextOffset=%.0f TurnSignal=%d NextLanes=%d NextLaneIdx=%d"),
			(JCurveP1 - JCurveP0).Size(), JCurveLength, JCurveNextRefDist,
			CurrentLateralOffset, NextSegLaneOffset,
			(int32)CurrentTurnSignal, NextLaneCount, JCurveNextLaneIndex);

		// 立刻走第一步，不浪費一幀（消除入彎停頓）
		// Execute first curve step immediately (eliminates 1-frame stall at entry)
		{
			JCurveProgress = CurrentSpeed * DeltaTime;
			const float T0Frame = FMath::Clamp(JCurveProgress / FMath::Max(JCurveLength, 1.0f), 0.0f, 1.0f);
			const FVector FirstPos = FMath::CubicInterp(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T0Frame);
			const FVector FirstTan = FMath::CubicInterpDerivative(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T0Frame);
			FRotator FirstRot = Owner->GetActorRotation();
			if (FirstTan.SizeSquared() > 1.0f)
			{
				FirstRot = FirstTan.Rotation();
			}
			Owner->SetActorLocationAndRotation(FirstPos, FirstRot);
		}
		return;
	}

	// ---- 檢查段結束（沒有下一段時才會到這裡）----
	// ---- Check segment end (only reached when no next segment) ----
	const bool bSegDone =
		(Seg.Direction > 0.0f && ReferenceDistance >= Seg.EndDist)
		|| (Seg.Direction < 0.0f && ReferenceDistance <= Seg.EndDist);

	if (bSegDone)
	{
		if (NavState == ENavState::Parking && !bParkingStraightening)
		{
			// 開始回正階段：慢慢前移 + 漸漸轉正（模擬真實停車）
			// Start straightening phase: creep forward + gradually align (realistic parking)
			bParkingStraightening = true;
			ParkingStraightenRemain = ParkingStraightenDistance;

			FVector TmpPos, TmpDir, TmpRight;
			SampleSplineAtDist(Seg, Seg.EndDist, CurrentLateralOffset, TmpPos, TmpDir, TmpRight);
			ParkingStraightenYaw = TmpDir.Rotation().Yaw;

			UE_LOG(LogTemp, Warning,
				TEXT("PathFollower: Parking straighten start — TargetYaw=%.1f RemainDist=%.0f"),
				ParkingStraightenYaw, ParkingStraightenRemain);
		}
		else if (!bParkingStraightening)
		{
			// 非停車模式：直接結束
			// Not parking: end immediately
			bIsFollowing = false;
			CurrentSpeed = 0.0f;
			NavState = ENavState::Parked;
			CurrentTurnSignal = ETurnSignal::None;
			UE_LOG(LogTemp, Warning, TEXT("PathFollower: Path completed (last seg) — NavState=Parked"));
			OnPathComplete.Broadcast(true);
			return;
		}
	}

	// ---- 停車回正階段：前移 + 橫移到路肩 + 轉正 ----
	// ---- Parking straighten: creep forward + slide to shoulder + align ----
	if (bParkingStraightening)
	{
		const float CreepSpeed = MaxSpeed * 0.1f;  // 10% 最大速度慢慢前移
		const FVector FwdDir = Owner->GetActorForwardVector();
		FVector NewPos = Owner->GetActorLocation() + FwdDir * CreepSpeed * DeltaTime;

		// 繼續橫向移動到路肩目標（bSegDone 前可能來不及到位）
		// Continue lateral slide to shoulder (may not have reached before bSegDone)
		CurrentLateralOffset = FMath::FInterpConstantTo(
			CurrentLateralOffset, ParkingTargetOffset, DeltaTime, LaneChangeSpeed);

		// 用最新 offset 在段終點取樣位置，用 VInterpTo 拉過去
		// Sample end-of-segment with updated offset, pull car toward it
		FVector TmpPos, TmpDir, TmpRight;
		SampleSplineAtDist(Seg, Seg.EndDist, CurrentLateralOffset, TmpPos, TmpDir, TmpRight);
		NewPos = FMath::VInterpTo(NewPos, TmpPos, DeltaTime, 10.0f);

		// 轉正 / Align rotation
		const FRotator CurRot = Owner->GetActorRotation();
		const FRotator TargetRot = FRotator(0.0f, ParkingStraightenYaw, 0.0f);
		const FRotator NewRot = FMath::RInterpTo(CurRot, TargetRot, DeltaTime, 3.0f);

		Owner->SetActorLocationAndRotation(NewPos, NewRot);

		ParkingStraightenRemain -= CreepSpeed * DeltaTime;
		const float YawDiff = FMath::Abs(FRotator::NormalizeAxis(CurRot.Yaw - ParkingStraightenYaw));
		const float OffsetDiff = FMath::Abs(CurrentLateralOffset - ParkingTargetOffset);

		// 轉正 + 到位 才停 / Must be both aligned AND at target offset to finish
		if (ParkingStraightenRemain <= 0.0f || (YawDiff < 1.0f && OffsetDiff < 5.0f))
		{
			bParkingStraightening = false;
			bIsFollowing = false;
			CurrentSpeed = 0.0f;
			NavState = ENavState::Parked;
			CurrentTurnSignal = ETurnSignal::None;

			UE_LOG(LogTemp, Warning,
				TEXT("PathFollower: Parked — Offset=%.0f TargetOffset=%.0f Yaw=%.1f"),
				CurrentLateralOffset, ParkingTargetOffset, Owner->GetActorRotation().Yaw);
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

	// 旋轉：用 Spline 切線方向，天生平滑穩定
	// Rotation: use spline tangent direction, inherently smooth and stable
	FRotator FinalRot = CurrentRot;
	if (CurrentSpeed > 1.0f && RefDir.SizeSquared() > 0.1f)
	{
		const FRotator TargetRot = RefDir.Rotation();
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

	UWorld* World = GetWorld();
	if (!World) return;

	AActor* Owner = GetOwner();
	if (!Owner) return;

	const FVector Start = Owner->GetActorLocation();
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
