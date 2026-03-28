#include "RoadPathFollowerComponent.h"
#include "RoadNetworkSubsystem.h"
#include "RoadRuleLibrary.h"
#include "RoadWorldSettings.h"
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
//  StartFollowing
// ============================================================================
void URoadPathFollowerComponent::StartFollowing()
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* Sub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!Sub)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: RoadNetworkSubsystem not found"));
		return;
	}

	const FRoadGraphPath AStarPath = Sub->FindPathAStar(StartNodeId, GoalNodeId);
	if (!AStarPath.bPathFound)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: A* failed %d → %d"), StartNodeId, GoalNodeId);
		OnPathComplete.Broadcast(false);
		return;
	}

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: A* path %d → %d | %d nodes | Cost=%.1f"),
		StartNodeId, GoalNodeId, AStarPath.NodePath.Num(), AStarPath.TotalCost);

	BuildPathSegments(AStarPath);

	if (PathSegments.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: No path segments built"));
		OnPathComplete.Broadcast(false);
		return;
	}

	// 初始化所有狀態
	// Initialize all state
	CurrentSegmentIndex = 0;
	ReferenceDistance = PathSegments[0].StartDist;
	CurrentSpeed = 0.0f;  // 從靜止開始 / start from standstill
	TargetLaneIndex = CurrentLaneIndex;
	CurrentLateralOffset = ComputeTargetLaneOffset(CurrentLaneIndex);
	bIsChangingLane = false;
	bOnJunctionCurve = false;
	JCurveProgress = 0.0f;
	bIsFollowing = true;

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: Started | %d segs | MaxSpeed=%.0f | Lane=%d | LateralOffset=%.0f"),
		PathSegments.Num(), MaxSpeed, CurrentLaneIndex, CurrentLateralOffset);
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
//  ComputeUpcomingTurnDirection — 偵測即將到來的轉彎方向
//  Detect upcoming turn direction by comparing current and next segment.
// ============================================================================
ETurnSignal URoadPathFollowerComponent::ComputeUpcomingTurnDirection() const
{
	if (CurrentSegmentIndex + 1 >= PathSegments.Num()) return ETurnSignal::None;

	const FPathSegmentInternal& CurSeg = PathSegments[CurrentSegmentIndex];
	const FPathSegmentInternal& NextSeg = PathSegments[CurrentSegmentIndex + 1];

	if (!CurSeg.Spline || !NextSeg.Spline) return ETurnSignal::None;

	// 取得當前段末端的行進方向
	// Get travel direction at end of current segment
	const FRotator CurRot = CurSeg.Spline->GetRotationAtDistanceAlongSpline(
		CurSeg.EndDist, ESplineCoordinateSpace::World);
	const FVector CurDir = CurRot.Vector() * CurSeg.Direction;

	// 取得下一段起點的行進方向
	// Get travel direction at start of next segment
	const FRotator NextRot = NextSeg.Spline->GetRotationAtDistanceAlongSpline(
		NextSeg.StartDist, ESplineCoordinateSpace::World);
	const FVector NextDir = NextRot.Vector() * NextSeg.Direction;

	// 用叉積判斷左右：CrossZ > 0 = 左轉，< 0 = 右轉
	// Cross product Z: > 0 = left turn, < 0 = right turn
	const float CrossZ = FVector::CrossProduct(CurDir, NextDir).Z;

	// 角度太小（< ~10°）視為直行，不打方向燈
	// If angle is small (< ~10°), treat as going straight
	const float DotProduct = FVector::DotProduct(CurDir.GetSafeNormal(), NextDir.GetSafeNormal());
	if (DotProduct > 0.985f) return ETurnSignal::None;  // cos(10°) ≈ 0.985

	return (CrossZ > 0.0f) ? ETurnSignal::Left : ETurnSignal::Right;
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
	// 在路口前 JunctionSlowdownDistance 開始減速，到路口時降到 JunctionMinSpeedRatio
	// Slow down within JunctionSlowdownDistance of junction,
	// reaching JunctionMinSpeedRatio at the junction point
	const float DistToJunction = GetDistanceToNextJunction();
	const bool bNextSegExists = (CurrentSegmentIndex + 1 < PathSegments.Num());

	if (bNextSegExists && DistToJunction < JunctionSlowdownDistance)
	{
		// 偵測轉彎角度 — 直行的路口不需要大幅減速
		// Detect turn angle — straight-through junctions need less slowdown
		const ETurnSignal TurnDir = ComputeUpcomingTurnDirection();

		if (TurnDir != ETurnSignal::None)
		{
			// 有轉彎：速度隨距離線性插值
			// Turning: linearly interpolate speed by distance
			// 離路口越近 → Alpha 越大 → 速度越低
			// Closer to junction → higher Alpha → lower speed
			const float Alpha = 1.0f - FMath::Clamp(
				DistToJunction / JunctionSlowdownDistance, 0.0f, 1.0f);
			const float JunctionSpeed = FMath::Lerp(
				MaxSpeed, MaxSpeed * JunctionMinSpeedRatio, Alpha);
			Desired = FMath::Min(Desired, JunctionSpeed);
		}
	}

	// 正在走路口 Hermite 曲線時也要限速
	// 前 60% 維持最低速，後 40% 才開始恢復
	// Also limit speed during active junction Hermite curve.
	// Hold min speed for first 60%, gradually recover in last 40%.
	if (bOnJunctionCurve)
	{
		const float CurveRatio = FMath::Clamp(
			JCurveProgress / FMath::Max(JCurveLength, 1.0f), 0.0f, 1.0f);

		// 前 60% 維持最低速，後 40% 才恢復
		// Hold min speed for first 60% of curve, recover in last 40%
		float RecoverAlpha = 0.0f;
		if (CurveRatio > 0.6f)
		{
			RecoverAlpha = (CurveRatio - 0.6f) / 0.4f; // 0→1 in last 40%
			RecoverAlpha = FMath::SmoothStep(0.0f, 1.0f, RecoverAlpha);
		}

		const float RecoverSpeed = FMath::Lerp(
			MaxSpeed * JunctionMinSpeedRatio, MaxSpeed, RecoverAlpha);
		Desired = FMath::Min(Desired, RecoverSpeed);
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
	//  0. 方向燈：偵測即將轉彎方向
	//     Turn signal: detect upcoming turn direction
	// ================================================================
	{
		const float DistToJunction = GetDistanceToNextJunction();
		const bool bNextSegExists = (CurrentSegmentIndex + 1 < PathSegments.Num());

		if (bNextSegExists && DistToJunction < JunctionSlowdownDistance)
		{
			// 接近路口，偵測轉彎方向並打方向燈
			// Approaching junction — detect turn and signal
			const ETurnSignal UpcomingTurn = ComputeUpcomingTurnDirection();
			if (UpcomingTurn != ETurnSignal::None && CurrentTurnSignal != UpcomingTurn)
			{
				CurrentTurnSignal = UpcomingTurn;
				UE_LOG(LogTemp, Log,
					TEXT("PathFollower: Turn signal → %s (junction in %.0f cm)"),
					(CurrentTurnSignal == ETurnSignal::Left) ? TEXT("LEFT") : TEXT("RIGHT"),
					DistToJunction);
			}
		}
		else if (!bOnJunctionCurve)
		{
			// 不在路口附近且不在走曲線，關閉方向燈
			// Not near junction and not on curve — signal off
			CurrentTurnSignal = ETurnSignal::None;
		}
	}

	// ================================================================
	//  1. 速度控制：加速到 DesiredSpeed 或煞車
	//     Speed control: accelerate toward desired or brake
	// ================================================================
	const float DesiredSpeed = ComputeDesiredSpeed();

	if (CurrentSpeed < DesiredSpeed)
	{
		// 加速 / Accelerate
		CurrentSpeed = FMath::Min(CurrentSpeed + Acceleration * DeltaTime, DesiredSpeed);
	}
	else
	{
		// 減速（用煞車減速度）/ Decelerate (use brake)
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

		// Hermite 曲線目標位置
		// Hermite curve target position
		const FVector CurvePos = FMath::CubicInterp(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T);

		// 入彎混合：前 20% 用漸增速度的 VInterpTo 銜接
		// 曲線前 20% 幾乎是直線（沿 T0 方向），VInterpTo 不會壓平弧度
		// 到 T=0.2 時 VInterpTo 速度已經很高，切到直接定位幾乎無感
		//
		// Entry blend: first 20% uses VInterpTo with ramping speed.
		// Curve barely turns in first 20% (follows T0 direction), so no arc flattening.
		// By T=0.2, interp speed is high enough that switch to direct is imperceptible.
		FVector FinalPos;
		constexpr float BlendInEnd = 0.20f;
		if (T < BlendInEnd)
		{
			const float BlendIn = T / BlendInEnd; // 0→1
			// 速度從正常(8)漸增到極高(100≈直接) / Ramp from normal(8) to very high(100≈direct)
			const float RampSpeed = FMath::Lerp(PositionInterpSpeed, 100.0f, BlendIn);
			const FVector CurrentPos = Owner->GetActorLocation();
			FinalPos = FMath::VInterpTo(CurrentPos, CurvePos, DeltaTime, RampSpeed);
		}
		else
		{
			FinalPos = CurvePos;
		}

		// 切線方向做旋轉（RInterpTo 平滑）
		// Tangent-based rotation (RInterpTo smoothed)
		const FVector CurveTangent = FMath::CubicInterpDerivative(JCurveP0, JCurveT0, JCurveP1, JCurveT1, T);
		const FRotator CurrentRot = Owner->GetActorRotation();
		FRotator FinalRot = CurrentRot;
		if (CurveTangent.SizeSquared() > 1.0f)
		{
			FinalRot = FMath::RInterpTo(
				CurrentRot, CurveTangent.Rotation(), DeltaTime, RotationInterpSpeed);
		}

		Owner->SetActorLocationAndRotation(FinalPos, FinalRot);

		UE_LOG(LogTemp, Verbose,
			TEXT("JUNCTION_CURVE: T=%.2f Progress=%.0f/%.0f Speed=%.0f"),
			T, JCurveProgress, JCurveLength, CurrentSpeed);

		if (T >= 1.0f)
		{
			// 曲線走完 → 進入新段
			// Curve complete → enter new segment
			bOnJunctionCurve = false;
			bJustExitedCurve = true;  // 下一幀跳過 VInterpTo / skip VInterpTo next frame
			CurrentSegmentIndex++;

			if (CurrentSegmentIndex >= PathSegments.Num())
			{
				bIsFollowing = false;
				CurrentSpeed = 0.0f;
				UE_LOG(LogTemp, Warning, TEXT("PathFollower: Path completed!"));
				OnPathComplete.Broadcast(true);
				return;
			}

			Seg = PathSegments[CurrentSegmentIndex];
			ReferenceDistance = JCurveNextRefDist;

			// 更新橫向偏移到新段的正確值
			// Update lateral offset to new segment's correct value
			CurrentLateralOffset = ComputeTargetLaneOffset(TargetLaneIndex);

			UE_LOG(LogTemp, Warning,
				TEXT("JUNCTION_CURVE_END: Entered Seg[%d] Type=%d RefDist=%.0f"),
				CurrentSegmentIndex, Seg.RoadType, ReferenceDistance);
		}

		// 每秒一次狀態 log
		static int32 CurveFrameCounter = 0;
		if (++CurveFrameCounter % 60 == 0)
		{
			UE_LOG(LogTemp, Log,
				TEXT("TICK(curve): Seg[%d/%d] Spd=%.0f Yaw=%.1f T=%.2f"),
				CurrentSegmentIndex, PathSegments.Num(), CurrentSpeed,
				Owner->GetActorRotation().Yaw, T);
		}

		return; // 在曲線上時跳過正常 spline 邏輯
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
	if (bNextSegExists && DistToEnd < JunctionBlendDistance)
	{
		const FPathSegmentInternal& NextSeg = PathSegments[CurrentSegmentIndex + 1];

		// 曲線終點：深入下一段 JunctionBlendDistance 的位置
		// Curve end: JunctionBlendDistance into next segment
		const float EntryDist = JunctionBlendDistance;
		const float NextSampleDist = NextSeg.StartDist + EntryDist * NextSeg.Direction;

		// 用下一段的正確 lane offset
		// Use next segment's correct lane offset
		const float NextSegLaneOffset = ComputeLaneOffsetCm(
			NextSeg.bTwoRoads, NextSeg.TwoRoadsGapM, NextSeg.RoadType,
			TargetLaneIndex,
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

		// 切線長度 ≈ 兩端點距離，讓曲線形狀合理
		// Tangent magnitude ≈ distance between endpoints for good curve shape
		// 切線強度 = 距離 × 0.5，控制曲線弧度
		// Tangent magnitude = distance × 0.5, controls curve roundness
		const float TangentScale = (JCurveP1 - JCurveP0).Size() * 0.5f;
		JCurveT0 = Owner->GetActorForwardVector() * TangentScale;
		JCurveT1 = NextDir.GetSafeNormal() * TangentScale;

		// 近似曲線長度（比直線長一點）
		// Approximate curve length (slightly longer than straight line)
		JCurveLength = (JCurveP1 - JCurveP0).Size() * 1.2f;
		JCurveProgress = 0.0f;
		JCurveNextRefDist = NextSampleDist;

		bOnJunctionCurve = true;

		UE_LOG(LogTemp, Warning,
			TEXT("JUNCTION_CURVE_START: P0→P1 dist=%.0f CurveLen=%.0f NextRefDist=%.0f CurOffset=%.0f NextOffset=%.0f"),
			(JCurveP1 - JCurveP0).Size(), JCurveLength, JCurveNextRefDist,
			CurrentLateralOffset, NextSegLaneOffset);

		return; // 下一幀開始走曲線
	}

	// ---- 檢查段結束（沒有下一段時才會到這裡）----
	// ---- Check segment end (only reached when no next segment) ----
	const bool bSegDone =
		(Seg.Direction > 0.0f && ReferenceDistance >= Seg.EndDist)
		|| (Seg.Direction < 0.0f && ReferenceDistance <= Seg.EndDist);

	if (bSegDone)
	{
		bIsFollowing = false;
		CurrentSpeed = 0.0f;
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: Path completed (last seg)!"));
		OnPathComplete.Broadcast(true);
		return;
	}

	// ================================================================
	//  6. 正常行駛 — 位置追蹤 + 旋轉跟隨移動方向
	//     Normal driving — position tracking + rotation follows velocity
	// ================================================================
	const FVector CurrentPos = Owner->GetActorLocation();
	const FRotator CurrentRot = Owner->GetActorRotation();

	// 接近路口時逐漸提高插值速度，讓車追上 spline 位置
	// 進入 Hermite 曲線時 VInterpTo 延遲幾乎為零 → 銜接自然
	// Ramp up interp speed approaching junction so VInterpTo lag → ~0
	// → seamless transition when Hermite curve starts
	float EffectiveInterpSpeed = PositionInterpSpeed;
	if (bNextSegExists && DistToEnd < JunctionBlendDistance * 2.0f)
	{
		const float Ratio = 1.0f - (DistToEnd / (JunctionBlendDistance * 2.0f));
		EffectiveInterpSpeed = FMath::Lerp(PositionInterpSpeed, 50.0f,
			FMath::Clamp(Ratio, 0.0f, 1.0f));
	}

	FVector FinalPos;
	if (bJustExitedCurve)
	{
		// 剛離開曲線的首幀：直接設位置，避免延遲跳躍
		// First frame after curve: direct position, skip lag jump
		FinalPos = RefPos;
		bJustExitedCurve = false;
	}
	else
	{
		FinalPos = FMath::VInterpTo(CurrentPos, RefPos, DeltaTime, EffectiveInterpSpeed);
	}

	const FVector Velocity = FinalPos - CurrentPos;
	FRotator FinalRot = CurrentRot;
	if (Velocity.SizeSquared() > 1.0f)
	{
		FinalRot = FMath::RInterpTo(
			CurrentRot, Velocity.Rotation(), DeltaTime, RotationInterpSpeed);
	}

	Owner->SetActorLocationAndRotation(FinalPos, FinalRot);

	// 每秒一次狀態 log
	static int32 FrameCounter = 0;
	if (++FrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Log,
			TEXT("TICK: Seg[%d/%d] Spd=%.0f/%.0f Yaw=%.1f Lane=%d Dist=%.0f→%.0f"),
			CurrentSegmentIndex, PathSegments.Num(),
			CurrentSpeed, ComputeDesiredSpeed(),
			Owner->GetActorRotation().Yaw, CurrentLaneIndex,
			ReferenceDistance, Seg.EndDist);
	}
}
