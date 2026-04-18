#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RoadTypes.h"
#include "RoadRuleTypes.h"
#include "RoadPathFollowerComponent.generated.h"

class AParkingLotActor;
class URoadNetworkSubsystem;
class USplineComponent;

/// <summary>
/// Turn signal state
/// </summary>
UENUM(BlueprintType)
enum class ETurnSignal : uint8
{
	None   UMETA(DisplayName = "None"),
	Left   UMETA(DisplayName = "Left"),
	Right  UMETA(DisplayName = "Right"),
};

/// <summary>
/// Navigation state
/// </summary>
UENUM(BlueprintType)
enum class ENavState : uint8
{
	Idle     UMETA(DisplayName = "Idle"),       // No route
	Driving  UMETA(DisplayName = "Driving"),    // Normal driving
	Parking  UMETA(DisplayName = "Parking"),    // Pulling over
	Parked   UMETA(DisplayName = "Parked"),     // Parked
};

/// <summary>
/// Parking mode at path end
/// </summary>
UENUM(BlueprintType)
enum class EParkingMode : uint8
{
	RoadsideStop  UMETA(DisplayName = "Roadside Stop"),  // Pull over to shoulder
	StopInPlace   UMETA(DisplayName = "Stop In Place"),  // Stop where path ends
};

/// <summary>
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

	/// +1 forward, -1 reverse
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

	/// Edge ID for this segment (for traffic light queries)
	int32 EdgeId = INDEX_NONE;

	/// End node ID of this segment
	int32 EndNodeId = INDEX_NONE;

	/// Turn direction at end of this segment (precomputed from graph node world locations)
	ETurnSignal TurnAtEnd = ETurnSignal::None;

	/// Whether this segment ends with a U-turn (Dot < -0.5)
	bool bUTurnAtEnd = false;

	/// Segment travel length in cm (for remaining distance calculation)
	float GetTravelLength() const { return FMath::Abs(EndDist - StartDist); }
};

/// <summary>
/// Overtaking state
/// </summary>
UENUM()
enum class EOvertakeState : uint8
{
	None       UMETA(DisplayName = "None"),       // Normal driving
	Passing    UMETA(DisplayName = "Passing"),     // In passing lane, overtaking
	Returning  UMETA(DisplayName = "Returning"),   // Returning to original lane
};

/// 前方障礙物分類 / Classification of what's blocking us in front
UENUM(BlueprintType)
enum class EFrontObstacleType : uint8
{
	None          UMETA(DisplayName = "None"),           // No obstacle
	Vehicle       UMETA(DisplayName = "Vehicle"),         // Another PathFollower car
	TrafficSignal UMETA(DisplayName = "Traffic Signal"),  // Red-light collider (tagged)
	Static        UMETA(DisplayName = "Static"),          // Wall / barrier / unknown
};

/// <summary>
/// Destination type
/// </summary>
UENUM(BlueprintType)
enum class EDestinationType : uint8
{
	None           UMETA(DisplayName = "None"),
	ParkingLot     UMETA(DisplayName = "Parking Lot"),
};

/// <summary>
/// Path completion event delegate
/// </summary>
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPathEvent, bool, bSuccess);

/// <summary>
/// Motion Control
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
	//  Navigation
	// ================================================================


	/// Auto-start on BeginPlay (default false, controlled by TrafficManager)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	bool bAutoStart = false;

	/// Parking mode when path ends
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Navigation")
	EParkingMode ParkingMode = EParkingMode::RoadsideStop;

	// ================================================================
	//  Speed Control
	// ================================================================
	/// Max speed (cm/s). 1500 = 15 m/s ~ 54 km/h
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0"))
	float MaxSpeed = 1500.0f;

	/// Acceleration (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0"))
	float Acceleration = 400.0f;

	/// Brake deceleration (cm/s²)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0"))
	float BrakeDeceleration = 800.0f;

	/// How quickly the target speed is allowed to RISE (FInterpTo rate).
	/// Braking (target dropping) is always instant; this only smooths acceleration.
	/// Lower = gentler start from stop (1.0–2.0); higher = snappier (5.0+).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0.5", ClampMax = "20.0"))
	float AccelerationSmoothRate = 2.5f;

	/// Distance before end to start braking (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0"))
	float BrakeDistance = 1500.0f;

	/// Curve slowdown sensitivity: higher = slows for gentler curves
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0.1"))
	float CurveSensitivity = 2.0f;

	/// Min speed ratio in curves (0.3 = floor at MaxSpeed × 30%)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float CurveMinSpeedRatio = 0.3f;

	// ================================================================
	//  Smooth Movement
	// ================================================================


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.01"))
	float OffsetBoostRate = 0.5f;

	/// Look-ahead time (sec) — vehicle steers toward position this far ahead.
	/// Higher = smoother turns but more latency; lower = tighter but jittery.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.05", ClampMax = "2.0"))
	float LookAheadTime = 0.5f;

	/// Min look-ahead distance (cm) — floor when speed is very low
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "50"))
	float MinLookAheadDist = 200.0f;

	/// Position interp speed — higher = tighter path adherence
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "1.0"))
	float PositionInterpSpeed = 8.0f;

	/// Rotation interp speed on straight roads — heading fine-tuning
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "1.0"))
	float RotationInterpSpeed = 5.0f;

	/// Max steering rate (deg/sec) — limits heading change per second.
	/// Lower = slower, more natural turns. A 90° turn at 45°/s takes 2 sec.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "5.0", ClampMax = "180.0"))
	float MaxTurnRateDegPerSec = 40.0f;

	/// Junction blend distance (cm) — used for LEFT turns, larger = smoother
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "10"))
	float JunctionBlendDistance = 1500.0f;

	/// Right turn blend ratio — right turns have smaller arcs, use shorter blend
	/// Actual right turn BlendDistance = JunctionBlendDistance × this ratio
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.1", ClampMax = "1.0"))
	float RightTurnBlendRatio = 0.4f;


	/// Junction curve tangent scale — controls turn arc roundness
	/// 0.5=flatter 0.7=round arc 0.9=stiff (two straight lines) 1.2+=S-overshoot
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.2", ClampMax = "1.5"))
	float JunctionCurveTangentScale = 0.7f;

	/// Junction slowdown start distance (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "100"))
	float JunctionSlowdownDistance = 4000.0f;

	/// Junction min speed ratio (0.25 = floor at MaxSpeed × 25% during turn)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Speed", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float JunctionMinSpeedRatio = 0.25f;

	// ================================================================
	//  U-Turn
	// ================================================================

	/// Whether to auto-compute U-turn radius from lane offsets.
	/// Formula: R = (CurrentLateralOffset + NextLateralOffset) / 2 + Padding
	/// The car flips from current lane to the target lane on the new segment;
	/// radius = half the distance between them. Works for inner or outer lane.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn")
	bool bAutoUTurnRadius = true;

	/// Extra padding for auto radius (cm) — avoids scraping the opposite lane edge
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn", meta = (ClampMin = "0", EditCondition = "bAutoUTurnRadius"))
	float UTurnRadiusPadding = 50.0f;

	/// Manual U-turn radius (used when bAutoUTurnRadius is false)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn", meta = (ClampMin = "10", EditCondition = "!bAutoUTurnRadius"))
	float UTurnRadius = 100.0f;

	/// U-turn speed ratio (relative to MaxSpeed) — similar to JunctionMinSpeedRatio
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn", meta = (ClampMin = "0.05", ClampMax = "1"))
	float UTurnSpeedRatio = 0.2f;

	/// U-turn curve tangent scale (similar to JunctionCurveTangentScale)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn", meta = (ClampMin = "0.3", ClampMax = "5.0"))
	float UTurnTangentScale = 1.0f;


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn")
	float UTurnRotationSpeed = 5.0f;

	/// Post-U-turn position blend speed — higher = faster convergence to spline
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|U-Turn", meta = (ClampMin = "1.0", ClampMax = "30.0"))
	float PostUTurnBlendSpeed = 6.0f;

	// ================================================================
	//  Lane Control
	// ================================================================

	/// Current lane index (0 = innermost)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane")
	int32 CurrentLaneIndex = 0;

	/// Lane change lateral speed (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane", meta = (ClampMin = "50"))
	float LaneChangeSpeed = 250.0f;

	/// Auto lane change trigger distance before junction (cm) — earlier than slowdown
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane", meta = (ClampMin = "500"))
	float AutoLaneChangeDistance = 8000.0f;

	// ================================================================
	//  Parking
	// ================================================================

	/// Start pulling over when remaining distance drops below this (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0"))
	float ParkingPullOverDistance = 8000.0f;

	/// Parking curve speed ratio (vs MaxSpeed) — speed when entering parking spot
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.05", ClampMax = "0.5"))
	float ParkingCurveSpeedRatio = 0.15f;

	/// Parking curve tangent scale — controls entry arc roundness
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.3", ClampMax = "3.0"))
	float ParkingCurveTangentScale = 1.0f;

	/// Isosceles-triangle base length n (cm) used for parking lot entry/exit curves.
	/// Right-angle vertex = arrow projection on edge; height = projection → arrow pos.
	/// Entry hypotenuse1 runs from (ProjPt - n along edge) → AnchorPos.
	/// Exit  hypotenuse2 runs from AnchorPos → (ProjPt + n along edge).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "50"))
	float ParkingTriangleBaseLength = 1000.0f;

	// ---- Departure Curve ----

	/// Departure curve tangent scale — controls exit arc roundness
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.1", ClampMax = "3.0"))
	float DepartureCurveTangentScale = 3.0f;

	/// Departure curve start-tangent strength ratio (× DistToRoad).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.1", ClampMax = "3.0"))
	float DepartureCurveStartTangentScale = 1.0f;

	/// Departure curve length multiplier (× distance to merge point)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "1.0", ClampMax = "3.0"))
	float DepartureCurveLengthMultiplier = 1.3f;

	/// Departure curve minimum length (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "50"))
	float DepartureCurveMinLength = 200.0f;

	/// Trigger distance for departure curve (cm); below this, just follow spline directly
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "10"))
	float DepartureCurveTriggerDistance = 200.0f;

	/// Departure curve speed ratio (vs MaxSpeed) — speed while exiting
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Parking", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float DepartureCurveSpeedRatio = 0.25f;

	// ================================================================
	//  Obstacle Detection
	// ================================================================

	/// Distance to start slowing down (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "500"))
	float ObstacleSlowdownDistance = 3000.0f;

	/// Distance to fully stop (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "50"))
	float ObstacleStopDistance = 300.0f;

	/// Emergency brake multiplier — when obstacle closer than this fraction of SlowdownDistance,
	/// apply BrakeDeceleration × this multiplier for harder braking (default 3×)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "1.0", ClampMax = "10.0"))
	float EmergencyBrakeMultiplier = 3.0f;

	/// Distance threshold for emergency brake zone, as fraction of ObstacleSlowdownDistance
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "0.1", ClampMax = "0.8"))
	float EmergencyBrakeZoneFraction = 0.35f;

	/// SphereTrace radius (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "10"))
	float ObstacleTraceRadius = 120.0f;

	/// SphereTrace start Z offset (cm) — raise to avoid hitting terrain
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle")
	float ObstacleTraceZOffset = 50.0f;

	/// Enable obstacle detection debug logging (which vehicle blocked by what)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle")
	bool bDebugObstacleTrace = false;

	/// When blocked (speed≈0 with obstacle ahead), throttle detection to this interval (seconds).
	/// Saves performance when many cars are stopped. Set 0 to always detect every frame.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Obstacle", meta = (ClampMin = "0", ClampMax = "2.0"))
	float ObstacleThrottleInterval = 0.5f;

	// ================================================================
	//  Stuck / Overlap Recovery
	//  卡住 / 重疊恢復：偵測完全停止 → 倒退分離 → 重新導航
	// ================================================================

	/// If obstacle distance < this, consider vehicles overlapping (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "10"))
	float OverlapDetectionDistance = 150.0f;

	/// Looser distance for "stopped behind someone" stuck detection (cm).
	/// A car stopped with any obstacle within this range for StuckRecoveryDelay
	/// seconds counts as stuck. Must be >= OverlapDetectionDistance.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "50"))
	float StuckDetectDistance = 600.0f;

	/// How long vehicles must be stuck (speed≈0, obstacle ahead) before triggering recovery (seconds)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "0.5"))
	float StuckRecoveryDelay = 1.5f;

	/// Speed at which the vehicle reverses to create separation (cm/s)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "50"))
	float StuckReverseSpeed = 300.0f;

	/// Base distance to reverse when unsticking (cm) — escalates on consecutive stucks
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "100"))
	float StuckReverseDistance = 600.0f;

	/// After reversing, should reroute to current destination instead of just resuming forward
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery")
	bool bRerouteAfterStuckRecovery = true;

	/// Cooldown after a reverse completes — stuck detection disabled for this long (seconds)
	/// 倒退完成後的冷卻時間，避免立刻又觸發
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "0.5"))
	float StuckCooldownDuration = 3.0f;

	/// Window to count consecutive stucks (seconds) — resets if no stuck within this time
	/// 判斷「連續卡住」的時窗
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "5.0"))
	float ConsecutiveStuckWindow = 15.0f;

	/// After N consecutive stucks, switch to a new random destination (parking lot)
	/// 連續卡住幾次後就換個隨機目的地
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "2"))
	int32 MaxConsecutiveStucksBeforeSwitch = 3;

	/// Backward check distance before each reverse step (cm) — if a vehicle is within
	/// this distance behind us, the reverse step is skipped this frame (cascade gate).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "100"))
	float RearClearDistance = 400.0f;

	/// Sphere radius for backward vehicle trace
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "20"))
	float RearTraceRadius = 80.0f;

	/// Max chain length to walk forward before giving up (loop / cycle guard)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "2"))
	int32 MaxChainWalkDepth = 10;

	/// Max seconds to wait for rear to clear before reversing anyway (safety valve
	/// against two queues stuck bumper-to-bumper). Set high enough that normal
	/// tail-first cascade usually wins; low enough to break full gridlock.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "1.0"))
	float RearWaitMaxSec = 4.0f;

	/// Distance threshold (cm) below which two touching vehicles are treated as a
	/// hard overlap that must be resolved immediately (bypasses all delay gates).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Stuck Recovery", meta = (ClampMin = "0.0"))
	float EmergencyOverlapDistance = 20.0f;

	/// Distance threshold (cm) below which same-direction cars are treated as
	/// overlapping rather than normal following (used in overlap-emergency logic).
	UPROPERTY(EditAnywhere, Category = "StuckRecovery")
	float OverlapSeparationThreshold = 200.0f;

	/// Name of the actor tag ColliderSwitcher stamps on red-light blockers
	static constexpr const TCHAR* TrafficSignalTagName = TEXT("TrafficSignal");

	// ================================================================
	//  Sequential Departure (源停車場出場順序)
	// ================================================================

	/// Poll interval when waiting for the previous spot's car to clear (seconds)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Departure", meta = (ClampMin = "0.1"))
	float DepartureCheckInterval = 0.5f;

	/// Distance (cm) the previous-spot car must have moved before this car may depart
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Departure", meta = (ClampMin = "100"))
	float DepartureClearDistance = 800.0f;

	/// Max time to wait for a lower-indexed spot to clear — depart anyway after this (seconds)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Departure", meta = (ClampMin = "1.0"))
	float DepartureMaxWaitTime = 30.0f;

	// ================================================================
	//  Overtaking
	// ================================================================

	/// Safe distance after passing before returning to original lane (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Overtaking", meta = (ClampMin = "500"))
	float OvertakeSafeDistance = 1500.0f;

	/// Trigger overtake only when front car speed < MaxSpeed × this threshold
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Overtaking", meta = (ClampMin = "0.3", ClampMax = "1.0"))
	float OvertakeSpeedThreshold = 0.8f;

	/// Minimum forward lane count required to attempt overtaking
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Overtaking", meta = (ClampMin = "2"))
	int32 OvertakeMinLaneCount = 2;

	/// Safety check distance for passing lane (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Overtaking", meta = (ClampMin = "500"))
	float OvertakeLateralCheckDistance = 2000.0f;

	// ================================================================
	//  Lane Change — Rear Safety
	// ================================================================

	/// How far behind (cm) to sweep for vehicles before any lane change
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Change", meta = (ClampMin = "200"))
	float LaneChangeRearCheckDistance = 800.0f;

	/// Sphere radius (cm) used in the rear-safety sweep
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Change", meta = (ClampMin = "20"))
	float LaneChangeRearCheckRadius = 120.0f;

	/// How far ahead (cm) to sweep in the TARGET lane before committing to a lane
	/// change. If a vehicle is found within this distance, the change is blocked.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Change", meta = (ClampMin = "100"))
	float LaneChangeFrontCheckDistance = 1500.0f;

	/// FInterpTo rate for lateral offset during a lane change.
	/// Lower = slower, more natural car-like weave (1.5–3.0 recommended).
	/// Replaces the old constant-rate FInterpConstantTo.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Change", meta = (ClampMin = "0.1", ClampMax = "20.0"))
	float LaneChangeSmoothRate = 2.0f;

	/// Rotation interp speed while a lane change is active.
	/// Lower than RotationInterpSpeed → heading turns more gradually, like a real car.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Change", meta = (ClampMin = "0.5"))
	float LaneChangeRotInterpSpeed = 3.0f;

	/// How fast the lateral-velocity steering component blends in at the START of a lane
	/// change (FInterpTo rate).  Lower = softer initial turn-in; higher = snappier.
	/// This controls the "first steer" feel — the moment the car nose first starts to turn.
	/// Recommended range: 1.0 (very gentle) – 8.0 (nearly instant).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Change", meta = (ClampMin = "0.5", ClampMax = "20.0"))
	float LaneChangeSteerBlendRate = 3.0f;

	/// Max seconds to wait for the rear lane to clear when forced to the outer
	/// edge on the final segment. After this time the switch is committed anyway
	/// so the car doesn't hang indefinitely.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Change", meta = (ClampMin = "1.0"))
	float FinalEdgeLaneForceMaxWait = 4.0f;

	// ================================================================
	//  Maneuver Caution (U-turn / L-turn / R-turn / parking exit)
	//  During these maneuvers, vehicle detection uses a wider / longer
	//  sweep than normal. Traffic-signal colliders are NOT affected —
	//  their normal trace size is preserved to avoid hitting signal
	//  signs on the oncoming side.
	// ================================================================

	/// Extra radius multiplier for vehicle-only trace during maneuvers
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Maneuver", meta = (ClampMin = "1.0", ClampMax = "3.0"))
	float ManeuverVehicleTraceRadiusMult = 1.5f;

	/// Extra distance multiplier for vehicle-only trace during maneuvers
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Maneuver", meta = (ClampMin = "1.0", ClampMax = "3.0"))
	float ManeuverVehicleTraceDistanceMult = 1.3f;

	/// Spline distance behind parking-lot merge point to scan for oncoming cars (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Maneuver", meta = (ClampMin = "100"))
	float DepartureLaneCheckDistance = 1000.0f;

	/// Sphere radius used for the departure lane rear scan (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Maneuver", meta = (ClampMin = "20"))
	float DepartureLaneTraceRadius = 100.0f;

	/// Before starting a LEFT-turn junction curve, sphere-trace this far along
	/// the oncoming lane of NextSeg looking for straight-through traffic.
	/// 0 disables the check.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Maneuver", meta = (ClampMin = "0"))
	float LeftTurnOncomingScanDistance = 1500.0f;

	/// Sphere radius for the left-turn oncoming scan (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Maneuver", meta = (ClampMin = "20"))
	float LeftTurnOncomingScanRadius = 150.0f;

	/// Max seconds to yield to oncoming traffic before forcing commit
	/// (prevents forever-wait when both sides are stopped)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Maneuver", meta = (ClampMin = "0"))
	float LeftTurnYieldMaxSec = 5.0f;

	// ---- Lane Offset Adjustments ----

	/// bTwoRoads=true  / Two Roads median adjust (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Offset")
	float TwoRoadsMedianAdjustCm = 0.0f;

	/// bTwoRoads=true  / Two Roads lane width adjust (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Offset")
	float TwoRoadsLaneWidthAdjustCm = 0.0f;

	/// bTwoRoads=false （ RoadType=4）/ Shared median adjust (RoadType=4 only)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Offset")
	float SharedRoadMedianAdjustCm = 0.0f;

	/// bTwoRoads=false  / Shared lane width adjust (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Lane Offset")
	float SharedRoadLaneWidthAdjustCm = 0.0f;

	// ================================================================
	//  API
	// ================================================================

	/// Manually start path following using hardcoded node IDs (for testing)
	UFUNCTION(BlueprintCallable, Category = "Road Path")
	void StartFollowing();

	/// Navigate to a world position (auto snap to nearest graph node)
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void NavigateToLocation(const FVector& Destination);

	/// Navigate to a specific graph node — internal use, prefer NavigateToParkingLot
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void NavigateToNode(int32 TargetNodeId);

	/// Navigate to parking lot and park in specified spot (-1 = auto-find)
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void NavigateToParkingLot(AParkingLotActor* ParkingLot, int32 SpotIndex = -1);

	/// Register where this vehicle originally spawned — enables spot-order gated departure.
	/// Call at spawn BEFORE the first NavigateTo*. SourceLot->OccupySpot(SourceSpotIndex, Owner)
	/// is invoked automatically.
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void SetDepartureContext(AParkingLotActor* SourceLot, int32 SourceSpotIndex);

	/// Gated departure: wait until all lower-indexed spots at SourceLot have cleared
	/// (their occupants have driven at least DepartureClearDistance away), then call
	/// NavigateToParkingLot(DestLot, DestSpotIndex). Safe to call without SetDepartureContext
	/// — it falls through to NavigateToParkingLot immediately if no source context set.
	UFUNCTION(BlueprintCallable, Category = "Road Path|Navigation")
	void RequestDepartToParkingLot(AParkingLotActor* DestLot, int32 DestSpotIndex = -1);

	/// True once this vehicle has physically cleared its spawn spot (moved >= DepartureClearDistance).
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	bool HasDepartedFromSource() const { return bHasDepartedFromSource; }

	/// Get destination type
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	EDestinationType GetDestinationType() const { return DestinationType; }

	/// Get current navigation state
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	ENavState GetNavState() const { return NavState; }

	/// Get destination world location
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	FVector GetDestinationLocation() const { return DestinationWorldLocation; }

	/// Get destination name
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	FString GetDestinationName() const;

	/// Get target spot index
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	int32 GetTargetSpotIndex() const;

	/// Get destination graph node ID
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path|Navigation")
	int32 GetDestinationNodeId() const { return DestinationNodeId; }

	/// <summary>
	/// Request smooth lane change to target lane index.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Path")
	void RequestLaneChange(int32 TargetLane);

	/// Get current actual speed
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	float GetCurrentSpeed() const { return CurrentSpeed; }

	/// Get how long this vehicle has been stuck (for chain detection by other vehicles)
	float GetStuckTimer() const { return StuckTimer; }

	/// Is this vehicle currently reversing from a stuck situation?
	bool IsStuckReversing() const { return bIsStuckReversing; }

	/// Is a lane change in progress?
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	bool IsChangingLane() const { return bIsChangingLane; }

	/// Get current turn signal state
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	ETurnSignal GetTurnSignal() const { return CurrentTurnSignal; }

	/// Get overtaking state
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	EOvertakeState GetOvertakeState() const { return OvertakeState; }

	/// Get obstacle distance ahead (-1 = none)
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	float GetObstacleDistance() const { return ObstacleDistance; }

	/// Get obstacle actor name (empty if none)
	FString GetObstacleActorName() const;

	/// Get obstacle type string: "Vehicle", "Traffic", or "Static"
	FString GetObstacleTypeString() const;

	/// Get destination display location — for map marker, uses parking lot position
	/// instead of road projection point
	FVector GetDestinationDisplayLocation() const;

	/// Get current route world positions (from current pos to end, for map rendering)
	void GetRouteWorldPoints(TArray<FVector>& OutPoints, int32 SamplesPerSegment = 10) const;

	/// Broadcast on path complete/fail
	UPROPERTY(BlueprintAssignable, Category = "Road Path")
	FOnPathEvent OnPathComplete;
	 
private:
	void BuildPathSegments(const FRoadGraphPath& AStarPath);

	/// <summary>
	/// Compute turn type from actual spline tangents at junction between InSeg end and OutSeg start.
	/// straight：dot > JunctionStraightDot ; U-turn: dot < JunctionUTurnDot ;
	/// </summary>
	void ComputeTurnAtJunction(
		const FPathSegmentInternal& InSeg,
		const FPathSegmentInternal& OutSeg,
		ETurnSignal& OutTurn,
		bool& bOutUTurn,
		float* OutDot = nullptr,
		float* OutCrossZ = nullptr) const;

public:
	/// Junction turn detection: dot > this → straight (no signal, no slowdown)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "0.5", ClampMax = "1.0"))
	float JunctionStraightDot = 0.866f;

	/// Junction turn detection: dot < this → U-turn
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Smoothing", meta = (ClampMin = "-1.0", ClampMax = "-0.5"))
	float JunctionUTurnDot = -0.866f;

private:

	/// <summary>
	/// Compute world position at segment/distance with lateral offset.
	/// Also outputs travel direction and right vector.
	/// </summary>
	void SampleSplineAtDist(
		const FPathSegmentInternal& Seg, float Dist, float LateralOffset,
		FVector& OutPosition, FVector& OutTravelDir, FVector& OutTravelRight) const;

	/// <summary>
	/// Get pursuit point: look ahead by AheadDist along path.
	/// Crosses segment boundaries if needed.
	/// </summary>
	FVector GetPursuitPoint(float LateralOffset) const;

	/// <summary>
	/// Remaining distance from current position to end of path (cm).
	/// </summary>
	float GetRemainingDistance() const;

	/// <summary>
	/// Get curvature at current spline position (1/cm, higher = sharper).
	/// </summary>
	float GetCurrentCurvature() const;

	/// <summary>
	/// Compute lateral offset for given lane index on current segment.
	/// </summary>
	float ComputeTargetLaneOffset(int32 LaneIdx) const;

	/// <summary>
	/// Compute target speed this frame (acceleration, curve, braking).
	/// </summary>
	float ComputeDesiredSpeed() const;

	// ---- Internal State ----

	TArray<FPathSegmentInternal> PathSegments;
	int32 CurrentSegmentIndex = 0;

	/// Reference point distance on spline (cm) — vehicle smoothly pursues this
	float ReferenceDistance = 0.0f;

	/// Current actual speed (cm/s)
	float CurrentSpeed = 0.0f;

	/// Low-pass filtered desired speed — rises slowly (AccelerationSmoothRate),
	/// drops instantly so braking response is never delayed.
	float SmoothedDesiredSpeed = 0.0f;

	/// Current lateral offset (cm) — interpolating, may differ from target lane
	float CurrentLateralOffset = 0.0f;

	/// Lane change target index
	int32 TargetLaneIndex = 0;

	/// Whether a lane change is in progress
	bool bIsChangingLane = false;

	/// Smoothed lateral velocity used for heading blend-in (avoids first-frame snap)
	float SmoothedLateralVel = 0.0f;

	bool bIsFollowing = false;
	bool bPendingStart = false;

	// ---- Junction Curve ----
	// Hermite curve through junction, replacing blend+offset patches

	/// Whether car is following a junction curve
	bool bOnJunctionCurve = false;

	/// Whether following a U-turn curve (uses UTurn params instead of Junction params)
	bool bIsUTurnCurve = false;

	/// Whether this is a gap-bridge curve (straight but non-contiguous → no slowdown)
	bool bIsGapBridgeCurve = false;

	// Semicircle arc params (used instead of Hermite when bIsUTurnCurve=true)
	FVector UTurnCenter = FVector::ZeroVector;   
	float  UTurnArcRadius = 0.0f;                
	FVector UTurnAxisU = FVector::ZeroVector;     
	FVector UTurnAxisV = FVector::ZeroVector;     
	float  UTurnArcZ0 = 0.0f;                    
	float  UTurnArcZ1 = 0.0f;                    

	/// Just exited junction curve (use high interp speed same frame for seamless transition)
	bool bJustExitedJunctionCurve = false;

	FVector JCurveP0 = FVector::ZeroVector;  // start pos
	FVector JCurveT0 = FVector::ZeroVector;  // start tangent
	FVector JCurveP1 = FVector::ZeroVector;  // end pos
	FVector JCurveT1 = FVector::ZeroVector;  // end tangent
	float JCurveLength = 0.0f;               
	float JCurveProgress = 0.0f;             
	float JCurveNextRefDist = 0.0f;          
	int32 JCurveNextLaneIndex = 0;           // lane for next seg

	/// Turn signal
	ETurnSignal CurrentTurnSignal = ETurnSignal::None;

	// ---- Navigation State ----

	/// Current navigation state
	ENavState NavState = ENavState::Idle;

	/// Destination graph node ID
	int32 DestinationNodeId = INDEX_NONE;

	/// Destination world location
	FVector DestinationWorldLocation = FVector::ZeroVector;

	/// Destination type
	EDestinationType DestinationType = EDestinationType::None;

	/// Target parking lot
	UPROPERTY()
	TWeakObjectPtr<AParkingLotActor> TargetParkingLot;
	int32 TargetParkingSpotIndex = INDEX_NONE;

	/// Parking target lateral offset (cm) — computed in Step 0.5, used in Step 4
	float ParkingTargetOffset = 0.0f;

	/// Whether car is following a parking Hermite curve
	bool bOnParkingCurve = false;

	FVector ParkCurveP0 = FVector::ZeroVector;  // start pos
	FVector ParkCurveT0 = FVector::ZeroVector;  // start tangent
	FVector ParkCurveP1 = FVector::ZeroVector;  // end pos (spot location)
	FVector ParkCurveT1 = FVector::ZeroVector;  // end tangent (arrow forward)
	float ParkCurveLength = 0.0f;               
	float ParkCurveProgress = 0.0f;             

	/// Whether car is following a departure curve (from parking spot to road)
	bool bOnDepartureCurve = false;

	FVector DepCurveP0 = FVector::ZeroVector;
	FVector DepCurveT0 = FVector::ZeroVector;
	FVector DepCurveP1 = FVector::ZeroVector;
	FVector DepCurveT1 = FVector::ZeroVector;
	float DepCurveLength = 0.0f;
	float DepCurveProgress = 0.0f;

	/// Internal shared navigation start logic (A* + BuildPath + init)
	void StartNavigationInternal(int32 FromNodeId, int32 ToNodeId);

	/// Reset all following state (for re-routing)
	void ResetFollowingState();

	// ---- Obstacle Detection ----

	/// Distance to obstacle ahead (-1 = none)
	float ObstacleDistance = -1.0f;

	/// Timer for throttled obstacle detection when blocked
	float ObstacleThrottleTimer = 0.0f;

	/// Accumulates seconds while ObstacleDistance < EmergencyOverlapDistance.
	/// When it exceeds ZeroDistTeleportTimeout the car is teleported to its
	/// destination to break an unresolvable deadlock.
	float ZeroDistStuckTimer = 0.0f;

	/// Accumulated 0-cm time (seconds) before the car teleports.
	/// Non-continuous: timer only resets after ZeroDistClearTimeout of no 0cm.
	UPROPERTY(EditAnywhere, Category = "StuckRecovery")
	float ZeroDistTeleportTimeout = 1.0f;

	/// Seconds without a 0-cm event required to reset ZeroDistStuckTimer.
	/// Prevents timer from carrying over between genuinely-resolved incidents.
	UPROPERTY(EditAnywhere, Category = "StuckRecovery")
	float ZeroDistClearTimeout = 5.0f;

	/// Tracks how long the car has been continuously clear of 0-cm obstacles.
	float ZeroDistClearTimer = 0.0f;

	/// Post-teleport grace timer: blocks ZeroDistStuckTimer accumulation for
	/// ZeroDistTeleportGraceDuration seconds after every ZERO-DIST teleport.
	/// Prevents the infinite re-teleport loop when the destination already has
	/// another car parked at 0 cm.
	float ZeroDistTeleportGraceTimer = 0.0f;

	/// How long (seconds) after a ZERO-DIST teleport to suppress ZeroDistStuckTimer.
	/// Must be long enough for the car to start moving away from the destination.
	UPROPERTY(EditAnywhere, Category = "StuckRecovery")
	float ZeroDistTeleportGraceDuration = 6.0f;

	/// Timer handle for deferred re-navigation after a ZERO-DIST teleport-to-spot.
	/// Fires after ZeroDistTeleportGraceDuration and calls SwitchToNewRandomDestination.
	FTimerHandle ZeroDistReNavigateTimerHandle;

	/// Called by ZeroDistReNavigateTimerHandle after the grace period.
	/// Switches the car to a new random parking lot so it is never repeatedly
	/// stuck at the same location.
	void ZeroDistReNavigateCallback();

	/// Front obstacle actor (for overtaking: distinguish vehicle vs traffic light)
	UPROPERTY()
	TWeakObjectPtr<AActor> ObstacleActor;

	/// SphereTrace forward obstacle detection (unified: vehicles, red light blockers, any obstacle)
	void UpdateObstacleDetection();

	/// True when the car is in a maneuver state (junction curve, parking curve,
	/// departure curve). Vehicle-only obstacle detection widens in these cases.
	bool IsPerformingManeuver() const;

	/// Sphere-trace along the source lot's bound edge spline (backward from the
	/// merge point by DepartureLaneCheckDistance) for vehicles. Returns true if
	/// no PathFollower-bearing actor is within the scan window.
	bool IsDepartureLaneClear() const;

	/// Walk forward along ObstacleActor links (up to MaxChainWalkDepth steps).
	/// Returns true if any car in the chain (including this one) is blocked by
	/// a TrafficSignal — meaning the entire chain is just waiting at a red light
	/// and must NOT be teleported.
	bool IsChainHeadBlockedBySignal() const;

	/// Before committing to a LEFT-turn junction curve, scan the oncoming lane
	/// of NextSeg for straight-through traffic. Returns true if clear (safe to turn).
	/// @param NextSeg next path segment (the one we're turning INTO)
	/// @param OutBlockingActor receives the blocking vehicle (nullptr if clear)
	/// @param OutDistance receives distance to the blocker (>=0 if found)
	bool IsLeftTurnOncomingClear(
		const struct FPathSegmentInternal& NextSeg,
		AActor*& OutBlockingActor,
		float& OutDistance) const;

	/// Compute speed limit from obstacle distance
	float ComputeObstacleSpeedLimit() const;

	// ---- Stuck / Overlap Recovery State ----

	/// Accumulated time this vehicle has been stuck (speed≈0 with obstacle ahead)
	float StuckTimer = 0.0f;

	/// Whether the vehicle is currently reversing to unstick
	bool bIsStuckReversing = false;

	/// Distance already reversed during stuck recovery
	float StuckReversedSoFar = 0.0f;

	/// Current reverse distance target (may escalate on repeated stuck)
	float CurrentStuckReverseTarget = 600.0f;

	/// Cooldown timer: stuck detection disabled while > 0
	float StuckCooldownTimer = 0.0f;

	/// How many consecutive stucks (within ConsecutiveStuckWindow) happened recently
	int32 ConsecutiveStuckCount = 0;

	/// Time since last stuck recovery triggered (for consecutive-count window)
	float TimeSinceLastStuck = 999.0f;

	/// Accumulated wait-for-rear-clear time during Phase 2 reversing
	float RearWaitAccum = 0.0f;

	/// Last-logged yield reason (to avoid spamming the event ring buffer)
	FString LastYieldReason;
	float LastYieldLogTime = -999.0f;

	/// Accumulated time yielding to oncoming traffic at a left-turn junction;
	/// forces commit once it exceeds LeftTurnYieldMaxSec so two head-on stopped
	/// queues can't deadlock each other forever.
	float LeftTurnYieldAccum = 0.0f;

	/// Update stuck/overlap recovery logic (called in Tick)
	void UpdateStuckRecovery(float DeltaTime);

	/// Classify what's blocking us in front (uses ObstacleActor + tags)
	EFrontObstacleType ClassifyFrontObstacle() const;

	/// Sphere-trace directly behind the car looking for another vehicle.
	/// @param OutRearVehicle receives the found vehicle (nullptr if clear)
	/// @return true if no vehicle within RearClearDistance (safe to reverse)
	bool IsRearClearOfVehicles(AActor*& OutRearVehicle) const;

	/// Sweep backward in the target lane to check for an approaching vehicle.
	/// Returns false (NOT safe) if a PathFollower is found within CheckDistance.
	/// Called before every lane change to avoid cutting in front of rear traffic.
	bool IsLaneChangeRearSafe(int32 InTargetLaneIndex, float CheckDistance) const;

	/// Forward sweep in the target lane: returns false if a PathFollower vehicle
	/// is found ahead within CheckDistance. Blocks lane changes into occupied lanes.
	bool IsTargetLaneFrontClear(int32 InTargetLaneIndex, float CheckDistance) const;

	/// Walk forward along the stuck chain and decide who should reverse to
	/// break the deadlock (cycle, zombie-head, etc.).  Returns the reverser,
	/// or nullptr if the situation is self-resolving.  OutChainGroup is filled
	/// with every member of the detected group; OutReason is a short tag.
	URoadPathFollowerComponent* FindChainReverser(
		TArray<URoadPathFollowerComponent*>& OutChainGroup,
		FString& OutReason) const;

	/// Walk forward along (ObstacleActor→PF) links until the chain terminates
	/// (no front, front is not stuck, or depth exceeded). Fills OutChain from
	/// tail (me) to head (leader). Returns true if the walk terminated cleanly.
	bool BuildStuckChainForward(TArray<URoadPathFollowerComponent*>& OutChain) const;

	/// Am I at the head of the stuck chain? i.e. my front obstacle is not a
	/// vehicle, OR the front vehicle is not itself stuck-stopped.
	bool IsChainLeader() const;

	/// Short label for the current maneuver — "STRAIGHT", "LEFT", "RIGHT",
	/// "U-TURN", "DEPART", "PARKING", or "APPROACH-L/R" (pre-curve).
	FString GetManeuverLabel() const;

	/// Walk BACKWARDS: find every stopped PathFollower whose ObstacleActor
	/// (transitively) points at me. OutChain is filled newest-to-oldest but
	/// order doesn't matter for triggering. Returns count.
	int32 CountStoppedRearChain(TArray<URoadPathFollowerComponent*>& OutChain) const;

	/// Trigger a reverse on this car — same bookkeeping as Phase 1 normally does.
	/// Called by the mutual-stuck resolver to cascade reverse down a chain.
	void TriggerStuckReverse(float OverrideReverseDistance = -1.0f, const TCHAR* Reason = TEXT(""));

	/// Mutual-stuck resolver one-shot flag. Cleared when we move again.
	bool bMutualStuckResolved = false;

	/// Second throttle for mutual-stuck logging
	float LastMutualStuckLogTime = -999.0f;

	/// Separate stuck timer for static-obstacle (wall/mesh) escape path.
	/// Kept out of the main StuckTimer so its long 3× delay doesn't
	/// interfere with vehicle-vs-vehicle logic.
	float StaticStuckTimer = 0.0f;

	/// Pick a new random parking lot destination (different from current) — called on repeated stuck
	void SwitchToNewRandomDestination();

	// ---- Departure gating (source parking lot) ----

	/// Lot this car originally spawned at (set via SetDepartureContext)
	TWeakObjectPtr<AParkingLotActor> SourceParkingLot;

	/// Spot index at SourceParkingLot (INDEX_NONE = no gating)
	int32 SourceSpotIndex = INDEX_NONE;

	/// World location this car spawned at — used to detect "departed" via distance
	FVector SourceSpawnLocation = FVector::ZeroVector;

	/// Set true once this car has moved DepartureClearDistance from SourceSpawnLocation
	bool bHasDepartedFromSource = false;

	/// Accumulated seconds spent waiting for lower-indexed spot to clear (fallback safety)
	float DepartureWaitAccum = 0.0f;

	/// Timer handle for polling departure readiness
	FTimerHandle DepartureCheckTimerHandle;

	/// Cached target for the deferred departure (set by RequestDepartToParkingLot)
	TWeakObjectPtr<AParkingLotActor> PendingDestLot;
	int32 PendingDestSpotIndex = INDEX_NONE;

	/// Returns true if every spot with index < SourceSpotIndex at SourceParkingLot is
	/// either unoccupied or whose occupant has cleared (bHasDepartedFromSource / moved away).
	bool IsSourceLotReadyForDeparture() const;

	/// Poll handler invoked on DepartureCheckTimerHandle
	void TryDepartPoll();

	/// Tick helper: detect when this car has moved far enough to be considered departed,
	/// flip bHasDepartedFromSource and release the source spot.
	void UpdateDepartureProgress();

	/// Distance to next junction (for slowdown calculation)
	float GetDistanceToNextJunction() const;

	// ----  Overtaking ----

	/// Overtaking state machine
	EOvertakeState OvertakeState = EOvertakeState::None;

	/// Lane index before overtake (return target)
	int32 PreOvertakeLaneIndex = 0;

	/// Distance counter after passing (cm)
	float OvertakePassedDistAccum = 0.0f;

	/// The specific slow vehicle that triggered this overtake (tracked until actually past)
	TWeakObjectPtr<AActor> OvertakeTargetActor;

	/// Distance the car must be past the target vehicle before attempting to return (cm)
	UPROPERTY(EditAnywhere, Category = "Overtaking", meta = (ClampMin = "100.0"))
	float OvertakeMinPassDistance = 600.0f;

	/// Extra speed boost during overtake (multiplier on MaxSpeed)
	UPROPERTY(EditAnywhere, Category = "Overtaking", meta = (ClampMin = "1.0", ClampMax = "2.0"))
	float OvertakeSpeedBoost = 1.25f;

	/// How long the original lane must read clear before we commit to returning (seconds)
	UPROPERTY(EditAnywhere, Category = "Overtaking", meta = (ClampMin = "0.0"))
	float OvertakeReturnConfirmTime = 0.5f;

	/// Rolling timer — counts up while original lane is confirmed clear
	float OvertakeReturnConfirmTimer = 0.0f;

	/// Overtake logic update (called in Tick)
	void UpdateOvertakeLogic();

public:
	/// Max recent events retained per vehicle (ring buffer size)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path|Debug", meta = (ClampMin = "4", ClampMax = "60"))
	int32 MaxRecentEvents = 24;

	/// True once the car enters the last (bound-edge) segment — overtake disabled,
	/// forced lane change to outermost lane for smooth pull-over.
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Road Path")
	bool IsOnFinalEdge() const { return bIsFinalEdge; }

	/// Read-only access to the recent-events ring buffer (for map debug panel).
	/// Each entry is a pre-formatted "[t=123.4s] event description" string.
	const TArray<FString>& GetRecentEvents() const { return RecentEvents; }

private:
	/// True once we've entered the last segment of the path (bound edge).
	bool bIsFinalEdge = false;

	// ---- Final-edge deferred lane force (rear-safe) ----

	/// True while a final-edge outer-lane force is pending rear-safety confirmation.
	bool bPendingFinalEdgeForce = false;

	/// Target lane for the pending final-edge force.
	int32 PendingFinalEdgeTargetLane = 0;

	/// Accumulated wait time for the pending final-edge lane force (seconds).
	float FinalEdgeLaneForceWait = 0.0f;

	/// Last time we logged "auto-lane change blocked by rear traffic" (throttle).
	float LastAutoLaneRearBlockLogTime = -999.0f;

	/// Called right after CurrentSegmentIndex is advanced — handles final-edge prep.
	void HandleEnteredNewSegment();

	/// Push a tagged event into the ring buffer (also UE_LOGs with [TRACE] prefix).
	void LogEvent(const FString& EventText);

	/// Debug ring buffer of recent navigation / stuck events (pre-formatted strings).
	TArray<FString> RecentEvents;

	/// Check if passing lane is clear (SphereTrace in target lane direction)
	bool IsPassingLaneClear(int32 PassingLaneIndex) const;

	/// Check if the original lane ahead is clear — used before returning from overtake
	bool IsOriginalLaneClear(int32 OriginalLaneIndex, float CheckDistance) const;

	/// Is the target vehicle now behind us by at least MinPastDistance on our forward axis?
	bool IsTargetVehiclePast(float MinPastDistance) const;

	// ---- Mid-drive re-routing ----

	/// Mid-drive re-route: keep current segment, A* from segment end to new destination
	void RerouteToNode(int32 TargetNodeId);

	/// Build path segments from A* result and append after current segment
	void AppendPathSegments(const FRoadGraphPath& AStarPath);

	/// Truncate path at target world position: find segment matching EdgeId, set EndDist to projected dist, remove later segments
	void TruncatePathToWorldPosition(const FVector& TargetPos, int32 TargetEdgeId);

	/// Append the bound edge as the final path segment, with EndDist truncated to the given
	/// absolute spline distance (the projection point), and recompute the previously-last
	/// segment's TurnAtEnd so it connects cleanly into the bound edge.
	void AppendBoundEdgeFinalSegment(const struct FRoadGraphEdge& BoundEdge, bool bForward, float ProjDistAbs);

	/// Release any parking spot held by the previous destination and clear
	/// Target* state. Call before starting a new navigation to avoid leaking occupancy.
	void ReleasePreviousDestinationSpot();

	/// Find nearest graph node that lies in front of the car (forward cone) for use as A* start.
	/// Falls back to the absolute nearest node if no forward node is found.
	int32 FindForwardStartNode(const FVector& CarPos, const FVector& CarFwd) const;

	/// <summary>
	/// Find the road edge nearest to the car and compute its projection. Used to start
	/// navigation from the nearest point ON an edge (not the nearest graph node).
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
	/// Prepend a partial segment (from ProjDistAbs to the forward end of BoundEdge) at
	/// the front of PathSegments. Symmetric to AppendBoundEdgeFinalSegment — used so the
	/// car starts driving from the projection point on the bound edge rather than from
	/// the far end node.
	/// </summary>
	void PrependStartBoundEdgeSegment(
		const struct FRoadGraphEdge& BoundEdge, bool bForward, float ProjDistAbs,
		bool bTriangleExitOffset = false);
};
