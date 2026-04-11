#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RoadTypes.h"
#include "RoadRuleTypes.h"
#include "RoadPathFollowerComponent.generated.h"

class AParkingLotActor;
class ARoadsideParkingActor;
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
/// 導航狀態 / Navigation state
/// </summary>
UENUM(BlueprintType)
enum class ENavState : uint8
{
	Idle     UMETA(DisplayName = "Idle"),       // 沒有路線 / No route
	Driving  UMETA(DisplayName = "Driving"),    // 正常行駛 / Normal driving
	Parking  UMETA(DisplayName = "Parking"),    // 靠邊停車中 / Pulling over
	Parked   UMETA(DisplayName = "Parked"),     // 已停好 / Parked
};

/// <summary>
/// 停車模式 / Parking mode at path end
/// </summary>
UENUM(BlueprintType)
enum class EParkingMode : uint8
{
	RoadsideStop  UMETA(DisplayName = "Roadside Stop"),  // 路邊靠肩停車 / Pull over to shoulder
	StopInPlace   UMETA(DisplayName = "Stop In Place"),  // 原地停車 / Stop where path ends
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

	/// 此段對應的 Graph Edge ID（用於紅綠燈查詢）/ Edge ID for this segment (for traffic light queries)
	int32 EdgeId = INDEX_NONE;

	/// 此段終點的 Graph Node ID / End node ID of this segment
	int32 EndNodeId = INDEX_NONE;

	/// 此段結束時（路口處）的轉彎方向，BuildPathSegments 時用 Node 世界座標預計算
	/// Turn direction at end of this segment (precomputed from graph node world locations)
	ETurnSignal TurnAtEnd = ETurnSignal::None;

	/// 此段結束是否為 U 型掉頭（Dot < -0.5）
	/// Whether this segment ends with a U-turn (Dot < -0.5)
	bool bUTurnAtEnd = false;

	/// 該段 spline 的長度（cm，用於剩餘距離計算）
	/// Segment travel length in cm (for remaining distance calculation)
	float GetTravelLength() const { return FMath::Abs(EndDist - StartDist); }
};

/// <summary>
/// 超車狀態 / Overtaking state
/// </summary>
UENUM()
enum class EOvertakeState : uint8
{
	None       UMETA(DisplayName = "None"),       // 正常行駛 / Normal driving
	Passing    UMETA(DisplayName = "Passing"),     // 在超車道超車中 / In passing lane, overtaking
	Returning  UMETA(DisplayName = "Returning"),   // 切回原車道中 / Returning to original lane
};

/// <summary>
/// 目的地類型 / Destination type
/// </summary>
UENUM(BlueprintType)
enum class EDestinationType : uint8
{
	None           UMETA(DisplayName = "None"),
	ParkingLot     UMETA(DisplayName = "Parking Lot"),
	RoadsideParking UMETA(DisplayName = "Roadside Parking"),
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

	/// A* 起始 Node ID（舊版測試用，不再使用）/ Legacy test node IDs, no longer used
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	int32 StartNodeId = 9;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	int32 GoalNodeId = 2;

	/// 是否在 BeginPlay 時自動開始（預設 false，由 TrafficManager 控制）
	/// Auto-start on BeginPlay (default false, controlled by TrafficManager)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	bool bAutoStart = false;

	/// 路徑完成時的停車模式 / Parking mode when path ends
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	EParkingMode ParkingMode = EParkingMode::RoadsideStop;

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


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.01"))
	float OffsetBoostRate = 0.5f;

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

	/// 路口過渡混合距離（cm）— 左轉用這個值，越大轉彎越圓滑
	/// Junction blend distance (cm) — used for LEFT turns, larger = smoother
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "10"))
	float JunctionBlendDistance = 1500.0f;

	/// 右轉混合距離比例 — 右轉弧度小，用較短的 BlendDistance
	/// 實際右轉 BlendDistance = JunctionBlendDistance × 此比例
	/// Right turn blend ratio — right turns have smaller arcs, use shorter blend
	/// Actual right turn BlendDistance = JunctionBlendDistance × this ratio
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.1", ClampMax = "1.0"))
	float RightTurnBlendRatio = 0.4f;


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
	//  掉頭 / U-Turn
	// ================================================================

	/// U 型掉頭曲線的寬度（cm）— 越大 U 型越寬越自然
	/// U-turn curve width (cm) — larger = wider, smoother U-shape
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn", meta = (ClampMin = "500"))
	float UTurnRadius = 5000.0f;

	/// U 型掉頭時的速度比例（相對 MaxSpeed）— 與 JunctionMinSpeedRatio 類似
	/// U-turn speed ratio (relative to MaxSpeed) — similar to JunctionMinSpeedRatio
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn", meta = (ClampMin = "0.05", ClampMax = "1"))
	float UTurnSpeedRatio = 0.2f;

	/// U 型掉頭曲線的切線強度（與 JunctionCurveTangentScale 類似）
	/// U-turn curve tangent scale (similar to JunctionCurveTangentScale)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn", meta = (ClampMin = "0.3", ClampMax = "5.0"))
	float UTurnTangentScale = 1.0f;

	// ================================================================
	//  車道 / Lane Control
	// ================================================================

	/// 目前車道索引（0 = 最內側）/ Current lane index (0 = innermost)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane")
	int32 CurrentLaneIndex = 0;

	/// 車道切換橫向速度（cm/s）/ Lane change lateral speed (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane", meta = (ClampMin = "50"))
	float LaneChangeSpeed = 250.0f;

	/// 轉彎前自動換道觸發距離（cm）— 比減速距離更早，讓車有足夠時間換道
	/// Auto lane change trigger distance before junction (cm) — earlier than slowdown
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane", meta = (ClampMin = "500"))
	float AutoLaneChangeDistance = 8000.0f;

	// ================================================================
	//  停車 / Parking
	// ================================================================

	/// 進入最後一段後，剩餘距離低於此值時開始靠邊（cm）
	/// Start pulling over when remaining distance drops below this (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0"))
	float ParkingPullOverDistance = 8000.0f;

	/// 停車曲線速度比例（相對 MaxSpeed）— 停車入庫/路邊停車時的速度
	/// Parking curve speed ratio (vs MaxSpeed) — speed when entering parking spot
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.05", ClampMax = "0.5"))
	float ParkingCurveSpeedRatio = 0.15f;

	/// 停車曲線切線強度 — 控制入庫弧度（越大越圓滑）
	/// Parking curve tangent scale — controls entry arc roundness
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.3", ClampMax = "3.0"))
	float ParkingCurveTangentScale = 1.0f;

	// ---- 出發曲線 / Departure Curve ----

	/// 出發曲線切線強度 — 控制出庫弧度（越大越圓滑、越繞）
	/// Departure curve tangent scale — controls exit arc roundness
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.1", ClampMax = "3.0"))
	float DepartureCurveTangentScale = 1.0f;

	/// 出發曲線起點切線強度倍率（相對 DistToRoad）
	/// 越大車頭方向保持越久，越小越快轉向道路
	/// Departure curve start-tangent strength ratio (× DistToRoad).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.1", ClampMax = "3.0"))
	float DepartureCurveStartTangentScale = 1.0f;

	/// 出發曲線長度倍率（相對車→匯入點距離）
	/// Departure curve length multiplier (× distance to merge point)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "1.0", ClampMax = "3.0"))
	float DepartureCurveLengthMultiplier = 1.3f;

	/// 出發曲線最小長度（cm）
	/// Departure curve minimum length (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "50"))
	float DepartureCurveMinLength = 200.0f;

	/// 距離道路多遠才觸發出發曲線（cm），小於此距離直接正常跟隨 spline
	/// Trigger distance for departure curve (cm); below this, just follow spline directly
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "10"))
	float DepartureCurveTriggerDistance = 200.0f;

	/// 出發曲線速度比例（相對 MaxSpeed）— 出庫時的速度
	/// Departure curve speed ratio (vs MaxSpeed) — speed while exiting
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float DepartureCurveSpeedRatio = 0.25f;

	// ================================================================
	//  障礙物偵測 / Obstacle Detection
	// ================================================================

	/// 開始減速的偵測距離（cm）/ Distance to start slowing down (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "500"))
	float ObstacleSlowdownDistance = 3000.0f;

	/// 完全停車的距離（cm）/ Distance to fully stop (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "50"))
	float ObstacleStopDistance = 300.0f;

	/// SphereTrace 偵測半徑（cm）/ SphereTrace radius (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "10"))
	float ObstacleTraceRadius = 120.0f;

	/// SphereTrace 起點 Z 軸抬高量（cm）— 避免打到地形
	/// SphereTrace start Z offset (cm) — raise to avoid hitting terrain
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle")
	float ObstacleTraceZOffset = 50.0f;

	/// 開啟障礙物偵測 debug log（Log 哪台車被什麼物體擋到）
	/// Enable obstacle detection debug logging (which vehicle blocked by what)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle")
	bool bDebugObstacleTrace = false;

	// ================================================================
	//  超車 / Overtaking
	// ================================================================

	/// 超車後安全距離：超過前車後再走多遠才切回（cm）
	/// Safe distance after passing before returning to original lane (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Overtaking", meta = (ClampMin = "500"))
	float OvertakeSafeDistance = 1500.0f;

	/// 前車速度低於我的 MaxSpeed × 此值才觸發超車
	/// Trigger overtake only when front car speed < MaxSpeed × this threshold
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Overtaking", meta = (ClampMin = "0.3", ClampMax = "1.0"))
	float OvertakeSpeedThreshold = 0.8f;

	/// 超車所需最低同向車道數（預設 2 = 至少雙車道才超車）
	/// Minimum forward lane count required to attempt overtaking
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Overtaking", meta = (ClampMin = "2"))
	int32 OvertakeMinLaneCount = 2;

	/// 超車道安全偵測距離（cm）/ Safety check distance for passing lane (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Overtaking", meta = (ClampMin = "500"))
	float OvertakeLateralCheckDistance = 2000.0f;

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

	/// 手動開始跟隨（用 StartNodeId / GoalNodeId，測試用）
	/// Manually start path following using hardcoded node IDs (for testing)
	UFUNCTION(BlueprintCallable, Category = "Road Path")
	void StartFollowing();

	/// 導航到世界座標位置（自動 SnapToRoad 找最近 node）
	/// Navigate to a world position (auto snap to nearest graph node)
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void NavigateToLocation(const FVector& Destination);

	/// 導航到指定 Graph Node ID（從車目前位置出發）— 內部用，外部請用 NavigateToParkingLot / NavigateToRoadside
	/// Navigate to a specific graph node — internal use, prefer NavigateToParkingLot/NavigateToRoadside
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void NavigateToNode(int32 TargetNodeId);

	/// 導航到停車場並停入指定車格（SpotIndex=-1 自動找空位）
	/// Navigate to parking lot and park in specified spot (-1 = auto-find)
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void NavigateToParkingLot(AParkingLotActor* ParkingLot, int32 SpotIndex = -1);

	/// 導航到路邊停車位置（SpotIndex = 佔用哪個停車格，-1 = 不佔用）
	/// Navigate to roadside parking position (SpotIndex = which spot to occupy, -1 = none)
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void NavigateToRoadside(ARoadsideParkingActor* RoadsideActor, const FVector& Position, bool bLeftSide, int32 SpotIndex = -1);

	/// 取得目的地類型 / Get destination type
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	EDestinationType GetDestinationType() const { return DestinationType; }

	/// 取得目前導航狀態 / Get current navigation state
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	ENavState GetNavState() const { return NavState; }

	/// 取得目的地世界座標 / Get destination world location
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	FVector GetDestinationLocation() const { return DestinationWorldLocation; }

	/// 取得目的地名稱（停車場名 / 路邊停車名）/ Get destination name
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	FString GetDestinationName() const;

	/// 取得目標停車格索引 / Get target spot index
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	int32 GetTargetSpotIndex() const;

	/// 取得目的地 Node ID / Get destination graph node ID
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	int32 GetDestinationNodeId() const { return DestinationNodeId; }

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

	/// 取得超車狀態 / Get overtaking state
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	EOvertakeState GetOvertakeState() const { return OvertakeState; }

	/// 取得前方障礙物距離（-1 = 無）/ Get obstacle distance ahead (-1 = none)
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	float GetObstacleDistance() const { return ObstacleDistance; }

	/// 取得目前路線的世界座標點序列（從車目前位置到終點，用於地圖繪製）
	/// Get current route world positions (from current pos to end, for map rendering)
	void GetRouteWorldPoints(TArray<FVector>& OutPoints, int32 SamplesPerSegment = 10) const;

	/// 路徑完成或失敗時廣播 / Broadcast on path complete/fail
	UPROPERTY(BlueprintAssignable, Category = "Road Path")
	FOnPathEvent OnPathComplete;

private:
	void BuildPathSegments(const FRoadGraphPath& AStarPath);

	/// <summary>
	/// 用實際 spline 切線判斷從 InSeg 出口轉到 OutSeg 入口的轉彎類型。
	/// Compute turn type from actual spline tangents at junction between InSeg end and OutSeg start.
	/// 直行：dot > JunctionStraightDot ; U-turn: dot < JunctionUTurnDot ; 否則看 CrossZ 分左右。
	/// </summary>
	void ComputeTurnAtJunction(
		const FPathSegmentInternal& InSeg,
		const FPathSegmentInternal& OutSeg,
		ETurnSignal& OutTurn,
		bool& bOutUTurn,
		float* OutDot = nullptr,
		float* OutCrossZ = nullptr) const;

public:
	/// 路口轉彎判定：dot > 此值 → 直行（不打方向燈、不減速）
	/// Junction turn detection: dot > this → straight (no signal, no slowdown)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.5", ClampMax = "1.0"))
	float JunctionStraightDot = 0.866f;

	/// 路口轉彎判定：dot < 此值 → U-turn
	/// Junction turn detection: dot < this → U-turn
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "-1.0", ClampMax = "-0.5"))
	float JunctionUTurnDot = -0.866f;

private:

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

	/// 是否正在走路口曲線 / Whether car is following a junction curve
	bool bOnJunctionCurve = false;

	/// 是否正在走 U 型掉頭曲線（使用 UTurn 參數而非 Junction 參數）
	/// Whether following a U-turn curve (uses UTurn params instead of Junction params)
	bool bIsUTurnCurve = false;

	/// 剛離開路口曲線（同一幀用高 interp speed 消除銜接停頓）
	/// Just exited junction curve (use high interp speed same frame for seamless transition)
	bool bJustExitedJunctionCurve = false;

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

	// ---- 導航狀態 / Navigation State ----

	/// 目前導航狀態 / Current navigation state
	ENavState NavState = ENavState::Idle;

	/// 目的地 Node ID / Destination graph node ID
	int32 DestinationNodeId = INDEX_NONE;

	/// 目的地世界座標 / Destination world location
	FVector DestinationWorldLocation = FVector::ZeroVector;

	/// 目的地類型 / Destination type
	EDestinationType DestinationType = EDestinationType::None;

	/// 目標停車場 / Target parking lot
	UPROPERTY()
	TWeakObjectPtr<AParkingLotActor> TargetParkingLot;
	int32 TargetParkingSpotIndex = INDEX_NONE;

	/// 目標路邊停車 / Target roadside parking
	UPROPERTY()
	TWeakObjectPtr<ARoadsideParkingActor> TargetRoadsideParking;
	int32 TargetRoadsideSpotIndex = INDEX_NONE;
	FVector TargetRoadsidePosition = FVector::ZeroVector;
	bool bTargetRoadsideLeftSide = false;

	/// 停車時的目標橫向偏移（cm）— Step 0.5 計算，Step 4 使用
	/// Parking target lateral offset (cm) — computed in Step 0.5, used in Step 4
	float ParkingTargetOffset = 0.0f;

	/// 是否正在走停車曲線（Hermite curve from road to spot）
	/// Whether car is following a parking Hermite curve
	bool bOnParkingCurve = false;

	FVector ParkCurveP0 = FVector::ZeroVector;  // 起點 / start pos
	FVector ParkCurveT0 = FVector::ZeroVector;  // 起點切線 / start tangent
	FVector ParkCurveP1 = FVector::ZeroVector;  // 終點（停車格位置）/ end pos (spot location)
	FVector ParkCurveT1 = FVector::ZeroVector;  // 終點切線（Arrow 前方）/ end tangent (arrow forward)
	float ParkCurveLength = 0.0f;               // 曲線近似長度
	float ParkCurveProgress = 0.0f;             // 目前走了多遠

	/// 是否正在走出發曲線（從停車格自然匯入道路）
	/// Whether car is following a departure curve (from parking spot to road)
	bool bOnDepartureCurve = false;

	FVector DepCurveP0 = FVector::ZeroVector;
	FVector DepCurveT0 = FVector::ZeroVector;
	FVector DepCurveP1 = FVector::ZeroVector;
	FVector DepCurveT1 = FVector::ZeroVector;
	float DepCurveLength = 0.0f;
	float DepCurveProgress = 0.0f;

	/// 路邊停車的 Arrow 前方方向（用於最終對齊）
	/// Roadside spot Arrow forward (for final alignment at path end)
	FVector RoadsideSpotForward = FVector::ZeroVector;

	/// 內部共用的導航啟動邏輯（A* + BuildPath + 初始化）
	/// Internal shared navigation start logic (A* + BuildPath + init)
	void StartNavigationInternal(int32 FromNodeId, int32 ToNodeId);

	/// 重置所有跟隨狀態（用於重導航）
	/// Reset all following state (for re-routing)
	void ResetFollowingState();

	// ---- 障礙物偵測 / Obstacle Detection ----

	/// 前方障礙物距離（cm，-1 = 無）/ Distance to obstacle ahead (-1 = none)
	float ObstacleDistance = -1.0f;

	/// 前方障礙物 Actor（用於超車判斷是車還是紅燈）
	/// Front obstacle actor (for overtaking: distinguish vehicle vs traffic light)
	UPROPERTY()
	TWeakObjectPtr<AActor> ObstacleActor;

	/// SphereTrace 偵測前方障礙物（統一：前車、紅燈碰撞體、任何障礙物）
	/// SphereTrace forward obstacle detection (unified: vehicles, red light blockers, any obstacle)
	void UpdateObstacleDetection();

	/// 根據障礙物距離計算限速 / Compute speed limit from obstacle distance
	float ComputeObstacleSpeedLimit() const;

	/// 到下一個路口的距離（用於減速判斷）
	/// Distance to next junction (for slowdown calculation)
	float GetDistanceToNextJunction() const;

	// ---- 超車 / Overtaking ----

	/// 超車狀態 / Overtaking state machine
	EOvertakeState OvertakeState = EOvertakeState::None;

	/// 超車前的車道索引（超車完要切回）/ Lane index before overtake (return target)
	int32 PreOvertakeLaneIndex = 0;

	/// 超車後的安全距離計數器（cm）/ Distance counter after passing (cm)
	float OvertakePassedDistAccum = 0.0f;

	/// ���車邏輯更新（Tick 中呼叫）/ Overtake logic update (called in Tick)
	void UpdateOvertakeLogic();

	/// 超車道安全偵測：目標車道方向是否有障礙
	/// Check if passing lane is clear (SphereTrace in target lane direction)
	bool IsPassingLaneClear(int32 PassingLaneIndex) const;

	// ---- 中途重導航 / Mid-drive re-routing ----

	/// 中途重導航：保留當前段，A* 從段終點到新目的地
	/// Mid-drive re-route: keep current segment, A* from segment end to new destination
	void RerouteToNode(int32 TargetNodeId);

	/// 用新 A* 路徑建段並接在當前段之後
	/// Build path segments from A* result and append after current segment
	void AppendPathSegments(const FRoadGraphPath& AStarPath);

	/// 截斷路徑到目標世界位置：找到匹配 EdgeId 的段，將 EndDist 設為目標投影距離，移除之後的段
	/// Truncate path at target world position: find segment matching EdgeId, set EndDist to projected dist, remove later segments
	void TruncatePathToWorldPosition(const FVector& TargetPos, int32 TargetEdgeId);

	/// 將綁定 Edge 作為路徑最後一段附加，EndDist 截斷到給定的 spline 絕對距離（投影點）
	/// 並重算原本最後一段的 TurnAtEnd，因為它現在接續到這條 bound edge
	/// Append the bound edge as the final path segment, with EndDist truncated to the given
	/// absolute spline distance (the projection point), and recompute the previously-last
	/// segment's TurnAtEnd so it connects cleanly into the bound edge.
	void AppendBoundEdgeFinalSegment(const struct FRoadGraphEdge& BoundEdge, bool bForward, float ProjDistAbs);

	/// 釋放上一個目的地佔用的停車格（停車場 / 路邊），並清空 Target* 狀態
	/// 在開始新的導航前呼叫，避免洩漏佔位
	/// Release any parking spot held by the previous destination (lot or roadside) and clear
	/// Target* state. Call before starting a new navigation to avoid leaking occupancy.
	void ReleasePreviousDestinationSpot();

	/// 找一個位於車輛前方的最近 graph node 作為 A* 起始點。
	/// 在前方錐外（後方）的 node 不會被選；若完全找不到 → fallback 到最近的任何 node。
	/// Find nearest graph node that lies in front of the car (forward cone) for use as A* start.
	/// Falls back to the absolute nearest node if no forward node is found.
	int32 FindForwardStartNode(const FVector& CarPos, const FVector& CarFwd) const;

	/// <summary>
	/// 找車最近的 edge 並投影 — 用於「從邊上最近的點」起始導航。
	/// Find the road edge nearest to the car and compute its projection. Used to start
	/// navigation from the nearest point ON an edge (not the nearest graph node).
	///
	/// 回傳成功時 OutEdge、OutProjDistAbs（絕對 spline 距離）、OutForward（車朝向
	/// 是否與 spline 正向同向）、OutExitNodeId（車行進方向會到達的 edge 終點 node）
	/// 均被設定。
	///
	/// On success, sets OutEdge (non-null), OutProjDistAbs (absolute spline distance of
	/// the car projection on the edge), OutForward (whether the car heading aligns with
	/// the edge's spline direction), and OutExitNodeId (the graph node the car would
	/// reach by driving forward along the edge from the projection).
	/// </summary>
	bool FindStartBoundEdge(
		const FVector& CarPos,
		const FVector& CarFwd,
		const struct FRoadGraphEdge*& OutEdge,
		float& OutProjDistAbs,
		bool& OutForward,
		int32& OutExitNodeId) const;

	/// <summary>
	/// 把 BoundEdge 從投影點到 ExitNode 這一段當作「第一段」插在 PathSegments 最前面。
	/// 作用與 AppendBoundEdgeFinalSegment 對稱 — 出發時從邊上最近的投影點開始行駛。
	///
	/// Prepend a partial segment (from ProjDistAbs to the forward end of BoundEdge) at
	/// the front of PathSegments. Symmetric to AppendBoundEdgeFinalSegment — used so the
	/// car starts driving from the projection point on the bound edge rather than from
	/// the far end node.
	/// </summary>
	void PrependStartBoundEdgeSegment(
		const struct FRoadGraphEdge& BoundEdge, bool bForward, float ProjDistAbs);
};
