#include "RoadPathFollowerComponent.h"
#include "RoadNetworkSubsystem.h"
#include "RoadRuleLibrary.h"
#include "RoadWorldSettings.h"
#include "Components/SplineComponent.h"
#include "Engine/World.h"

/// <summary>
/// 根據 BP 的 GuardrailSideOffset 計算車道偏移量（cm）
/// Calculate lane offset using the actual road half-width from BP.
///
/// GuardrailSideOffsetCm = BP Road Settings 裡護欄到 spline 中心的距離
///                       = 路面半寬（未乘 Multiplier、未加 AdditionalWidth）
///
/// 路面結構圖 / Road cross-section:
///
///   bTwoRoads=false（共用路面，如 2+2-Lane Wide Road）:
///     ┃←──────────── ActualHalfWidth ────────────→┃
///     中心 ──[Median][ Lane0 ][ Lane1 ][ Shoulder ]── 護欄
///     DrivableWidth = ActualHalfWidth - MedianCm - ShoulderCm
///     LaneWidth = DrivableWidth / ForwardLaneCount
///     Offset = MedianCm + (LaneIndex + 0.5) × LaneWidth
///
///   bTwoRoads=true（分隔道路）:
///     ┃←──────────── ActualHalfWidth ────────────→┃
///     spline ──[ GAP/2 ][Median][ Lane0 ][ Lane1 ][ Shoulder ]── 護欄
///     DrivableWidth = ActualHalfWidth - GapHalf - MedianCm - ShoulderCm
///     LaneWidth = DrivableWidth / ForwardLaneCount
///     Offset = GapHalf + MedianCm + (LaneIndex + 0.5) × LaneWidth
/// </summary>
static float ComputeLaneOffsetCm(
	double GuardrailSideOffsetCm,
	double RoadWidthMultiplier,
	double AdditionalWidthM,
	int32 ForwardLaneCount,
	bool bTwoRoads,
	double TwoRoadsGapM,
	int32 LaneIndex,
	float TwoRoadsMedianCm = 0.0f,
	float SharedMedianCm = 0.0f,
	float TwoRoadsShoulderCm = 0.0f,
	float SharedShoulderCm = 0.0f)
{
	const int32 LaneCount = FMath::Max(1, ForwardLaneCount);

	// 計算實際路面半寬（spline 中心到護欄）
	// Calculate actual half-width (spline center to guardrail)
	const float BaseHalfWidth = static_cast<float>(GuardrailSideOffsetCm);
	const float Multiplier = static_cast<float>(FMath::Max(RoadWidthMultiplier, 0.1));
	const float AdditionalHalfCm = static_cast<float>(AdditionalWidthM) * 50.0f;
	const float ActualHalfWidth = BaseHalfWidth * Multiplier + AdditionalHalfCm;

	if (bTwoRoads)
	{
		// 分隔道路：spline 在兩條路中間的間隙
		// Separated roads: spline is in the gap between two road surfaces
		//
		//   ┃←──────────────── ActualHalfWidth ──────────────────→┃
		//   spline ──[ GAP/2 ][ Median ][ Lane0 ][ Lane1 ][ Shoulder ]── 護欄
		//            ↑ GapHalf ↑ MedianCm                  ↑ TwoRoadsShoulderCm
		//
		const float GapHalfCm = static_cast<float>(FMath::Max(TwoRoadsGapM, 0.0)) * 50.0f;
		const float InnerOffset = GapHalfCm + TwoRoadsMedianCm;
		const float DrivableWidth = FMath::Max(ActualHalfWidth - InnerOffset - TwoRoadsShoulderCm, 1.0f);
		const float LaneWidth = DrivableWidth / LaneCount;

		UE_LOG(LogTemp, Verbose,
			TEXT("ComputeOffset TwoRoads: HalfW=%.0f Gap=%.0f Med=%.0f Shldr=%.0f Drv=%.0f LnW=%.0f [%d]→%.0f"),
			ActualHalfWidth, GapHalfCm, TwoRoadsMedianCm, TwoRoadsShoulderCm,
			DrivableWidth, LaneWidth, LaneIndex,
			InnerOffset + (LaneIndex + 0.5f) * LaneWidth);

		return InnerOffset + (LaneIndex + 0.5f) * LaneWidth;
	}

	// 共用路面：從中心線往右偏移（如 2+2-Lane Wide Road）
	// Shared road surface: offset from center line to the right
	//
	//   ┃←──────────────── ActualHalfWidth ──────────────────→┃
	//   中心 ──[ Median ][ Lane0 ][ Lane1 ][ Shoulder ]── 護欄
	//          ↑ MedianCm                   ↑ SharedShoulderCm
	//
	const float DrivableWidth = FMath::Max(ActualHalfWidth - SharedMedianCm - SharedShoulderCm, 1.0f);
	const float LaneWidth = DrivableWidth / LaneCount;

	UE_LOG(LogTemp, Verbose,
		TEXT("ComputeOffset Shared: HalfW=%.0f Med=%.0f Shldr=%.0f Drv=%.0f LnW=%.0f [%d]→%.0f"),
		ActualHalfWidth, SharedMedianCm, SharedShoulderCm,
		DrivableWidth, LaneWidth, LaneIndex,
		SharedMedianCm + (LaneIndex + 0.5f) * LaneWidth);

	return SharedMedianCm + (LaneIndex + 0.5f) * LaneWidth;
}

URoadPathFollowerComponent::URoadPathFollowerComponent()
{
	// 每幀 Tick 才能移動車輛
	// Need Tick to move the vehicle each frame
	PrimaryComponentTick.bCanEverTick = true;
}

void URoadPathFollowerComponent::BeginPlay()
{
	Super::BeginPlay();

	if (bAutoStart)
	{
		// 不在 BeginPlay 直接啟動，因為此時 RoadNetworkSubsystem 還沒跑 BuildRoadCache
		// 改成設 flag，讓第一次 Tick 時才啟動（那時 graph 已經建好了）
		// Don't start here — road graph isn't built yet during BeginPlay.
		// Set flag so first Tick will start (graph is ready by then).
		bPendingStart = true;
	}
}

/// <summary>
/// 從 Road Network Subsystem 取得 A* 路徑，建立片段，開始跟隨
/// Query the road network for an A* path and start following it.
/// </summary>
void URoadPathFollowerComponent::StartFollowing()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	URoadNetworkSubsystem* RoadSubsystem = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSubsystem)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: RoadNetworkSubsystem not found"));
		return;
	}

	// 用寫死的 node ID 跑 A*
	// Run A* with hardcoded node IDs
	const FRoadGraphPath AStarPath = RoadSubsystem->FindPathAStar(StartNodeId, GoalNodeId);

	if (!AStarPath.bPathFound)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("PathFollower: A* failed from Node %d to Node %d"),
			StartNodeId, GoalNodeId);
		return;
	}

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: A* found path from Node %d to Node %d | %d nodes | Cost=%.1f"),
		StartNodeId, GoalNodeId, AStarPath.NodePath.Num(), AStarPath.TotalCost);

	BuildPathSegments(AStarPath);

	if (PathSegments.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("PathFollower: No path segments built"));
		return;
	}

	// 初始化：從第一段的起始距離開始
	// Initialize: start at the first segment's start distance
	CurrentSegmentIndex = 0;
	CurrentDistance = PathSegments[0].StartDist;
	bIsFollowing = true;

	UE_LOG(LogTemp, Warning,
		TEXT("PathFollower: Started | %d segments | Speed=%.0f cm/s | LaneIndex=%d"),
		PathSegments.Num(), Speed, CurrentLaneIndex);
}

/// <summary>
/// 從 A* 的 NodePath 建立內部路徑片段
/// 對每對相鄰 node，找到連接的 edge，記錄 spline 和方向
/// Build path segments from A* NodePath.
/// For each adjacent node pair, find the connecting edge and record spline + direction.
/// </summary>
void URoadPathFollowerComponent::BuildPathSegments(const FRoadGraphPath& AStarPath)
{
	PathSegments.Empty();

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	URoadNetworkSubsystem* RoadSubsystem = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSubsystem)
	{
		return;
	}

	const TArray<FRoadGraphEdge>& GraphEdges = RoadSubsystem->GetGraphEdges();
	const TArray<int32>& NodePath = AStarPath.NodePath;

	for (int32 i = 0; i < NodePath.Num() - 1; ++i)
	{
		const int32 FromNode = NodePath[i];
		const int32 ToNode = NodePath[i + 1];

		// 找到連接這兩個 node 的 edge
		// Find the edge connecting these two nodes
		for (const FRoadGraphEdge& Edge : GraphEdges)
		{
			const bool bForward =
				(Edge.StartNodeId == FromNode && Edge.EndNodeId == ToNode);
			const bool bReverse =
				(Edge.StartNodeId == ToNode && Edge.EndNodeId == FromNode);

			if (!bForward && !bReverse)
			{
				continue;
			}

			if (!Edge.InputSpline)
			{
				break;
			}

			FPathSegmentInternal Seg;
			Seg.Spline = Edge.InputSpline;
			Seg.bTwoRoads = Edge.bTwoRoads;
			Seg.TwoRoadsGapM = Edge.TwoRoadsGapM;
			Seg.RoadType = Edge.RoadType;
			Seg.DrivingRule = URoadRuleLibrary::GetDrivingRuleFromRoadType(Edge.RoadType);
			Seg.RoadWidthMultiplier = Edge.RoadWidthMultiplier;
			Seg.AdditionalWidthM = Edge.AdditionalWidthM;
			Seg.GuardrailSideOffsetCm = Edge.GuardrailSideOffsetCm;

			if (bForward)
			{
				// 正向：沿 spline 的 dist 增加方向走
				// Forward: travel in increasing distance direction
				Seg.StartDist = Edge.StartDistanceOnSpline;
				Seg.EndDist = Edge.EndDistanceOnSpline;
				Seg.Direction = 1.0f;
			}
			else
			{
				// 反向：沿 spline 的 dist 減少方向走
				// Reverse: travel in decreasing distance direction
				Seg.StartDist = Edge.EndDistanceOnSpline;
				Seg.EndDist = Edge.StartDistanceOnSpline;
				Seg.Direction = -1.0f;
			}

			PathSegments.Add(Seg);

			// 計算這一段的偏移量用於 debug 確認
			// Compute offset for this segment for debug verification
			const float DebugOffset = ComputeLaneOffsetCm(
				Seg.GuardrailSideOffsetCm, Seg.RoadWidthMultiplier, Seg.AdditionalWidthM,
				Seg.DrivingRule.ForwardLaneCount, Seg.bTwoRoads, Seg.TwoRoadsGapM, 0);

			UE_LOG(LogTemp, Warning,
				TEXT("  Seg[%d]: %s | Type=%d %s | HalfW=%.0f Mult=%.2f | TwoRoads=%s | Lanes=%d | Offset=%.0fcm"),
				PathSegments.Num() - 1,
				bForward ? TEXT("Fwd") : TEXT("Rev"),
				Seg.RoadType, *Seg.DrivingRule.RuleName,
				Seg.GuardrailSideOffsetCm, Seg.RoadWidthMultiplier,
				Seg.bTwoRoads ? TEXT("Y") : TEXT("N"),
				Seg.DrivingRule.ForwardLaneCount,
				DebugOffset);

			break;
		}
	}
}

/// <summary>
/// 每幀更新車輛位置：沿 spline 前進，偏移到右側車道
/// Each frame: advance along spline, offset to right lane, update actor transform.
///
/// 右側偏移原理 / Right-side offset principle:
///
///   行進方向（tangent）
///       ↑
///   ────┼────→ 右側 = CrossProduct(Forward, Up)
///       │
///   車子在右側 ✅（美國規則）
///
///   如果 spline 反向走（Direction=-1），tangent 要翻轉
///   才能讓「右側」始終是行進方向的右邊
/// </summary>
void URoadPathFollowerComponent::TickComponent(
	float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// 第一次 Tick：road graph 已建好，現在才啟動路徑跟隨
	// First Tick: road graph is ready, start path following now
	if (bPendingStart)
	{
		bPendingStart = false;
		StartFollowing();
	}

	if (!bIsFollowing || PathSegments.Num() == 0)
	{
		return;
	}

	FPathSegmentInternal& Seg = PathSegments[CurrentSegmentIndex];

	if (!Seg.Spline)
	{
		bIsFollowing = false;
		return;
	}

	// ---- 前進：沿 spline 移動 ----
	// Advance along the spline
	CurrentDistance += Speed * DeltaTime * Seg.Direction;

	// ---- 檢查是否走完這一段 ----
	// Check if we've reached the end of this segment
	const bool bSegmentFinished =
		(Seg.Direction > 0.0f && CurrentDistance >= Seg.EndDist)
		|| (Seg.Direction < 0.0f && CurrentDistance <= Seg.EndDist);

	if (bSegmentFinished)
	{
		// 切換到下一段
		// Move to next segment
		CurrentSegmentIndex++;

		if (CurrentSegmentIndex >= PathSegments.Num())
		{
			// 走完了！
			// Path completed!
			bIsFollowing = false;
			UE_LOG(LogTemp, Warning, TEXT("PathFollower: Path completed!"));
			return;
		}

		// 從下一段的起始位置開始
		// Start from the next segment's start position
		CurrentDistance = PathSegments[CurrentSegmentIndex].StartDist;
		Seg = PathSegments[CurrentSegmentIndex];
	}

	// ---- 取樣 spline 位置和方向 ----
	// Sample spline position and direction

	// Clamp distance 避免超出 spline 範圍
	// Clamp distance to stay within spline bounds
	const float SplineLength = Seg.Spline->GetSplineLength();
	const float ClampedDist = FMath::Clamp(CurrentDistance, 0.0f, SplineLength);

	const FVector SplinePosition = Seg.Spline->GetLocationAtDistanceAlongSpline(
		ClampedDist, ESplineCoordinateSpace::World);

	// ---- 用 spline 的旋轉來取得正確的方向 ----
	// Use spline's rotation to get correct orientation (handles slopes, banks, etc.)
	//
	// GetRotationAtDistanceAlongSpline 會回傳 spline 在該點的完整旋轉
	// 這包含了坡度、彎道傾斜等資訊，比手動 Cross 更準確
	//
	// GetRotationAtDistanceAlongSpline returns the full rotation at that point
	// This includes slope, banking, etc. — more accurate than manual Cross product
	const FRotator SplineRotation = Seg.Spline->GetRotationAtDistanceAlongSpline(
		ClampedDist, ESplineCoordinateSpace::World);

	// 從旋轉矩陣取出前方和右方向量
	// Extract forward and right vectors from the rotation matrix
	const FVector SplineForward = SplineRotation.Vector();  // spline 正向 / spline forward
	const FVector SplineRight = FRotationMatrix(SplineRotation).GetUnitAxis(EAxis::Y);

	// ---- 計算行進方向 ----
	// Calculate travel direction
	// 如果反向走 spline（Direction=-1），翻轉方向
	// If traveling in reverse (Direction=-1), flip directions
	const FVector TravelDirection = SplineForward * Seg.Direction;
	const FVector TravelRight = SplineRight * Seg.Direction;

	// ---- 右側偏移（靠右行駛）----
	// Right-side offset (drive on right side)
	//
	//   行進方向（TravelDirection）
	//       ↑
	//       │
	//   ────┼────→ TravelRight（行進方向的右側）
	//       │       車子偏移到這裡 ✅
	//
	//   Direction=+1 時，TravelRight = SplineRight（原方向）
	//   Direction=-1 時，TravelRight = -SplineRight（翻轉）
	//   所以不管 spline 怎麼畫，「右邊」永遠是行進方向的右邊
	//
	const float Offset = ComputeLaneOffsetCm(
		Seg.GuardrailSideOffsetCm, Seg.RoadWidthMultiplier, Seg.AdditionalWidthM,
		Seg.DrivingRule.ForwardLaneCount, Seg.bTwoRoads, Seg.TwoRoadsGapM,
		CurrentLaneIndex, TwoRoadsMedianOffsetCm, SharedRoadMedianOffsetCm,
		TwoRoadsShoulderWidthCm, SharedRoadShoulderWidthCm);

	const FVector FinalPosition = SplinePosition + TravelRight * Offset;

	// ---- 計算旋轉（車頭朝向行進方向）----
	// Calculate rotation (vehicle faces travel direction)
	const FRotator FinalRotation = TravelDirection.Rotation();

	// ---- 更新 Actor 位置和旋轉 ----
	// Update owner actor's transform
	AActor* Owner = GetOwner();
	if (Owner)
	{
		Owner->SetActorLocationAndRotation(FinalPosition, FinalRotation);
	}
}
