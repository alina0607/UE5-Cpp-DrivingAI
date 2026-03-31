#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RoadTypes.h"
#include "RoadRuleTypes.h"
#include "RoadPathFollowerComponent.generated.h"

class URoadNetworkSubsystem;
class USplineComponent;

/// <summary>
/// 方向燈狀態 / Turn signal state
/// </summary>
UENUM(BlueprintType)
enum class ETurnSignal : uint8
{
	None   UMETA(DisplayName = "None"),
	Left   UMETA(DisplayName = "Left"),
	Right  UMETA(DisplayName = "Right"),
};

/// <summary>
/// 路徑跟隨內部用的片段資料：一段 spline 上從 A 開到 B
/// Internal path segment: drive along a spline from StartDist to EndDist.
/// </summary>
USTRUCT()
struct FPathSegmentInternal
{
	GENERATED_BODY()

	UPROPERTY()
	TObjectPtr<USplineComponent> Spline = nullptr;

	float StartDist = 0.0f;
	float EndDist = 0.0f;

	/// +1.0 = 沿 spline 正向，-1.0 = 反向 / +1 forward, -1 reverse
	float Direction = 1.0f;

	bool bTwoRoads = false;
	double TwoRoadsGapM = 0.0;
	uint8 RoadType = 0;
	FRoadDrivingRule DrivingRule;
	double RoadWidthMultiplier = 1.0;
	double AdditionalWidthM = 0.0;
	double GuardrailSideOffsetCm = 350.0;
	float AutoLaneWidthCm = 0.0f;
	float AutoMedianCm = 0.0f;

	/// 此段結束時（路口處）的轉彎方向，BuildPathSegments 時用 Node 世界座標預計算
	/// Turn direction at end of this segment (precomputed from graph node world locations)
	ETurnSignal TurnAtEnd = ETurnSignal::None;

	/// 該段 spline 的長度（cm，用於剩餘距離計算）
	/// Segment travel length in cm (for remaining distance calculation)
	float GetTravelLength() const { return FMath::Abs(EndDist - StartDist); }
};

/// <summary>
/// 路徑完成事件 / Path completion event delegate
/// </summary>
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPathEvent, bool, bSuccess);

/// <summary>
/// 車輛路徑跟隨元件 — Layer 3 Motion Control
///
/// 核心機制 / Core mechanisms:
///   1. 追蹤點模型（Pursuit Point）— 車追向前方目標點，自然產生平滑轉彎
///   2. 加速/煞車 — 起步加速、終點前減速、彎道自動降速
///   3. 車道切換插值 — RequestLaneChange() 平滑橫向移動
///   4. 路口過渡混合 — 段與段銜接處 blend 避免突變
///
/// Vehicle path follower with pursuit point model, speed control,
/// lane change interpolation, and junction blending.
/// </summary>
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class CARDRIVINGPROJECT_API URoadPathFollowerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URoadPathFollowerComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	// ================================================================
	//  導航 / Navigation
	// ================================================================

	/// A* 起始 Node ID（寫死測試用）/ Start node for A* test
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	int32 StartNodeId = 9;

	/// A* 目標 Node ID（寫死測試用）/ Goal node for A* test
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	int32 GoalNodeId = 2;

	/// 是否在 BeginPlay 時自動開始 / Auto-start on BeginPlay
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	bool bAutoStart = true;

	// ================================================================
	//  速度控制 / Speed Control
	// ================================================================

	/// 最大速度（cm/s）1500 = 15 m/s ≈ 54 km/h
	/// Max speed (cm/s). 1500 = 15 m/s ~ 54 km/h
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0"))
	float MaxSpeed = 1500.0f;

	/// 加速度（cm/s²）/ Acceleration (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0"))
	float Acceleration = 400.0f;

	/// 煞車減速度（cm/s²）/ Brake deceleration (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0"))
	float BrakeDeceleration = 800.0f;

	/// 終點前多遠開始煞車（cm）/ Distance before end to start braking (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0"))
	float BrakeDistance = 1500.0f;

	/// 彎道減速靈敏度：值越大，越小的彎就會減速
	/// Curve slowdown sensitivity: higher = slows for gentler curves
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0.1"))
	float CurveSensitivity = 2.0f;

	/// 彎道最低速度比例（0.3 = 最低降到 MaxSpeed × 30%）
	/// Min speed ratio in curves (0.3 = floor at MaxSpeed × 30%)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float CurveMinSpeedRatio = 0.3f;

	// ================================================================
	//  平滑移動 / Smooth Movement
	// ================================================================

	/// 追蹤點前瞻時間（秒）— 車追向 CurrentSpeed × LookAheadTime 前方的位置
	/// 值越大轉彎越圓滑但延遲越高；值越小越貼合路線但可能抖動
	///
	/// Look-ahead time (sec) — vehicle steers toward position this far ahead.
	/// Higher = smoother turns but more latency; lower = tighter but jittery.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.05", ClampMax = "2.0"))
	float LookAheadTime = 0.5f;

	/// 追蹤點最小前瞻距離（cm）— 低速時保底不低於此值
	/// Min look-ahead distance (cm) — floor when speed is very low
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "50"))
	float MinLookAheadDist = 200.0f;

	/// 位置插值速度 — 越大越貼合路線，越小越平滑
	/// Position interp speed — higher = tighter path adherence
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "1.0"))
	float PositionInterpSpeed = 8.0f;

	/// 直行時旋轉插值速度 — 車頭微調的平滑程度
	/// Rotation interp speed on straight roads — heading fine-tuning
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "1.0"))
	float RotationInterpSpeed = 5.0f;

	/// 最大轉向角速度（度/秒）— 車頭每秒最多轉多少度
	/// 越低轉彎越慢越自然；90° 轉彎在 45°/s 下需要 2 秒
	///
	/// Max steering rate (deg/sec) — limits heading change per second.
	/// Lower = slower, more natural turns. A 90° turn at 45°/s takes 2 sec.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "5.0", ClampMax = "180.0"))
	float MaxTurnRateDegPerSec = 40.0f;

	/// 路口過渡混合距離（cm）— 越大轉彎越圓滑
	/// Junction blend distance (cm) — larger = smoother turns
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "10"))
	float JunctionBlendDistance = 1500.0f;

	/// 路口曲線切線強度 — 控制轉彎弧度
	/// 0.5=較直 0.7=圓弧 0.9=僵硬（兩直線接合感）1.2+=S型過衝
	/// Junction curve tangent scale — controls turn arc roundness
	/// 0.5=flatter 0.7=round arc 0.9=stiff (two straight lines) 1.2+=S-overshoot
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.2", ClampMax = "1.5"))
	float JunctionCurveTangentScale = 0.7f;

	/// 路口減速開始距離（cm）— 距離路口多遠開始減速
	/// Junction slowdown start distance (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "100"))
	float JunctionSlowdownDistance = 4000.0f;

	/// 路口最低速度比例（0.25 = 轉彎時最低降到 MaxSpeed × 25% ≈ 13.5 km/h）
	/// Junction min speed ratio (0.25 = floor at MaxSpeed × 25% during turn)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float JunctionMinSpeedRatio = 0.25f;

	// ================================================================
	//  車道 / Lane Control
	// ================================================================

	/// 目前車道索引（0 = 最內側）/ Current lane index (0 = innermost)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane")
	int32 CurrentLaneIndex = 0;

	/// 車道切換橫向速度（cm/s）/ Lane change lateral speed (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane", meta = (ClampMin = "50"))
	float LaneChangeSpeed = 250.0f;

	// ---- 車道偏移微調 / Lane Offset Adjustments ----

	/// bTwoRoads=true 中間偏移微調 / Two Roads median adjust (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Offset")
	float TwoRoadsMedianAdjustCm = 0.0f;

	/// bTwoRoads=true 車道寬度微調 / Two Roads lane width adjust (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Offset")
	float TwoRoadsLaneWidthAdjustCm = 0.0f;

	/// bTwoRoads=false 中間偏移微調（僅 RoadType=4）/ Shared median adjust (RoadType=4 only)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Offset")
	float SharedRoadMedianAdjustCm = 0.0f;

	/// bTwoRoads=false 車道寬度微調 / Shared lane width adjust (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Offset")
	float SharedRoadLaneWidthAdjustCm = 0.0f;

	// ================================================================
	//  API
	// ================================================================

	/// 手動開始跟隨 / Manually start path following
	UFUNCTION(BlueprintCallable, Category = "Road Path")
	void StartFollowing();

	/// <summary>
	/// 請求切換到指定車道（平滑插值過渡）
	/// Request smooth lane change to target lane index.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Path")
	void RequestLaneChange(int32 TargetLane);

	/// 取得目前實際速度（cm/s）/ Get current actual speed
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	float GetCurrentSpeed() const { return CurrentSpeed; }

	/// 是否正在換道中 / Is a lane change in progress?
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	bool IsChangingLane() const { return bIsChangingLane; }

	/// 取得目前方向燈狀態 / Get current turn signal state
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	ETurnSignal GetTurnSignal() const { return CurrentTurnSignal; }

	/// 路徑完成或失敗時廣播 / Broadcast on path complete/fail
	UPROPERTY(BlueprintAssignable, Category = "Road Path")
	FOnPathEvent OnPathComplete;

private:
	void BuildPathSegments(const FRoadGraphPath& AStarPath);

	/// <summary>
	/// 計算指定段+距離處的世界位置（含橫向偏移）
	/// Compute world position at segment/distance with lateral offset.
	/// 同時輸出行進方向和右方向量
	/// Also outputs travel direction and right vector.
	/// </summary>
	void SampleSplineAtDist(
		const FPathSegmentInternal& Seg, float Dist, float LateralOffset,
		FVector& OutPosition, FVector& OutTravelDir, FVector& OutTravelRight) const;

	/// <summary>
	/// 取得追蹤點（沿路徑往前看 AheadDist 距離的位置）
	/// 可以跨段，如果前方在下一段就自動切過去
	///
	/// Get pursuit point: look ahead by AheadDist along path.
	/// Crosses segment boundaries if needed.
	/// </summary>
	FVector GetPursuitPoint(float LateralOffset) const;

	/// <summary>
	/// 從目前位置到路徑終點的剩餘距離（cm）
	/// Remaining distance from current position to end of path (cm).
	/// </summary>
	float GetRemainingDistance() const;

	/// <summary>
	/// 取得目前 spline 位置的曲率（1/cm，越大 = 越急彎）
	/// Get curvature at current spline position (1/cm, higher = sharper).
	/// </summary>
	float GetCurrentCurvature() const;

	/// <summary>
	/// 計算指定車道的橫向偏移量（cm）
	/// Compute lateral offset for given lane index on current segment.
	/// </summary>
	float ComputeTargetLaneOffset(int32 LaneIdx) const;

	/// <summary>
	/// 計算當前幀的目標速度（考慮加速、彎道減速、終點煞車）
	/// Compute target speed this frame (acceleration, curve, braking).
	/// </summary>
	float ComputeDesiredSpeed() const;

	// ---- 內部狀態 / Internal State ----

	TArray<FPathSegmentInternal> PathSegments;
	int32 CurrentSegmentIndex = 0;

	/// 在 spline 上的參考點距離（cm）— 邏輯位置，車的實際位置會平滑追上
	/// Reference point distance on spline (cm) — vehicle smoothly pursues this
	float ReferenceDistance = 0.0f;

	/// 目前實際速度（cm/s）
	/// Current actual speed (cm/s)
	float CurrentSpeed = 0.0f;

	/// 目前橫向偏移（cm）— 平滑插值中，可能跟目標車道偏移不同
	/// Current lateral offset (cm) — interpolating, may differ from target lane
	float CurrentLateralOffset = 0.0f;

	/// 車道切換目標索引
	/// Lane change target index
	int32 TargetLaneIndex = 0;

	/// 是否正在車道切換中
	/// Whether a lane change is in progress
	bool bIsChangingLane = false;

	bool bIsFollowing = false;
	bool bPendingStart = false;

	// ---- 路口曲線 / Junction Curve ----
	// 用 Hermite 曲線穿過路口，取代 blend+offset patch
	// Hermite curve through junction, replacing blend+offset patches

	/// 是否正在走路口曲線
	/// Whether car is following a junction curve
	bool bOnJunctionCurve = false;

	FVector JCurveP0 = FVector::ZeroVector;  // 起點 / start pos
	FVector JCurveT0 = FVector::ZeroVector;  // 起點切線 / start tangent
	FVector JCurveP1 = FVector::ZeroVector;  // 終點 / end pos
	FVector JCurveT1 = FVector::ZeroVector;  // 終點切線 / end tangent
	float JCurveLength = 0.0f;               // 曲線近似長度
	float JCurveProgress = 0.0f;             // 目前走了多遠
	float JCurveNextRefDist = 0.0f;          // 曲線結束後新段的 RefDist
	int32 JCurveNextLaneIndex = 0;           // 曲線結束後新段的車道 / lane for next seg

	/// 方向燈 / Turn signal
	ETurnSignal CurrentTurnSignal = ETurnSignal::None;

	/// 到下一個路口的距離（用於減速判斷）
	/// Distance to next junction (for slowdown calculation)
	float GetDistanceToNextJunction() const;
};
