#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RoadTypes.h"
#include "RoadRuleTypes.h"
#include "RoadPathFollowerComponent.generated.h"

class URoadNetworkSubsystem;
class USplineComponent;

/// <summary>
/// 路徑跟隨內部用的片段資料：一段 spline 上從 A 開到 B
/// Internal path segment: drive along a spline from StartDist to EndDist.
/// </summary>
USTRUCT()
struct FPathSegmentInternal
{
	GENERATED_BODY()

	/// <summary>
	/// 要走的 spline
	/// Spline to follow
	/// </summary>
	UPROPERTY()
	TObjectPtr<USplineComponent> Spline = nullptr;

	/// <summary>
	/// 起始 spline 距離（cm）
	/// Start distance along spline (cm)
	/// </summary>
	float StartDist = 0.0f;

	/// <summary>
	/// 結束 spline 距離（cm）
	/// End distance along spline (cm)
	/// </summary>
	float EndDist = 0.0f;

	/// <summary>
	/// 行進方向：+1.0 = 沿 spline 正向，-1.0 = 反向
	/// Travel direction: +1 = forward along spline, -1 = reverse
	/// </summary>
	float Direction = 1.0f;

	/// <summary>
	/// 該邊是否為 Two Roads 模式
	/// Whether this edge uses Two Roads mode
	/// </summary>
	bool bTwoRoads = false;

	/// <summary>
	/// Two Roads 的間距（公尺）
	/// Two Roads gap in meters
	/// </summary>
	double TwoRoadsGapM = 0.0;

	/// <summary>
	/// 該邊的道路類型（從 FRoadGraphEdge 取得）
	/// Road type for this segment (from FRoadGraphEdge)
	/// </summary>
	uint8 RoadType = 0;

	/// <summary>
	/// 快取的行駛規則（避免每幀查表）
	/// Cached driving rule (avoids per-tick lookup)
	/// </summary>
	FRoadDrivingRule DrivingRule;

	/// <summary>
	/// 路面寬度乘數（從 BP 讀取）
	/// Road width multiplier from BP.
	/// </summary>
	double RoadWidthMultiplier = 1.0;

	/// <summary>
	/// 額外路面寬度（公尺，從 BP 讀取）
	/// Additional road width in meters from BP.
	/// </summary>
	double AdditionalWidthM = 0.0;

	/// <summary>
	/// 路面半寬（cm），從 BP Road Settings 讀取
	/// Road half-width (cm), directly from BP's GuardrailSideOffset.
	/// </summary>
	double GuardrailSideOffsetCm = 350.0;
};

/// <summary>
/// 車輛路徑跟隨元件：掛在任何 Actor 上，讓它沿著 A* 路徑行駛
/// Vehicle path follower component: attach to any Actor to make it
/// drive along an A* path on the road network, offset to the right lane.
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

	// ---- 可在 Editor 裡設定的參數 / Editor-configurable parameters ----

	/// <summary>
	/// A* 起始 Node ID（寫死測試用）
	/// Start node ID for hardcoded A* test
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	int32 StartNodeId = 9;

	/// <summary>
	/// A* 目標 Node ID（寫死測試用）
	/// Goal node ID for hardcoded A* test
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	int32 GoalNodeId = 2;

	/// <summary>
	/// 行駛速度（cm/s）
	/// Travel speed in cm/s (500 = 5 m/s ≈ 18 km/h)
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	float Speed = 500.0f;

	/// <summary>
	/// 目前車道索引（0 = 最右邊的預設車道）
	/// Current lane index (0 = rightmost default lane)
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	int32 CurrentLaneIndex = 0;

	/// <summary>
	/// bTwoRoads=true 時的額外中間偏移（cm）
	/// 補償分隔道路中間的路燈、分隔島等空間
	/// 會從路面寬度中扣除，確保車道不會超出路面
	///
	/// Extra median offset for Two Roads mode (cm).
	/// Compensates for median lights/dividers between separated roads.
	/// Subtracted from road surface width so lanes stay within bounds.
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	float TwoRoadsMedianOffsetCm = 0.0f;

	/// <summary>
	/// bTwoRoads=false 時的額外中間偏移（cm）
	/// 補償共用路面中間的分隔帶（如 2+2-Lane Wide Road 的中央分隔島）
	/// 會從路面半寬中扣除，確保車道不會超出路面
	///
	/// Extra median offset for shared road surface (cm).
	/// Compensates for center median on roads like 2+2-Lane Wide Road.
	/// Subtracted from half-width so lanes stay within bounds.
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	float SharedRoadMedianOffsetCm = 0.0f;

	/// <summary>
	/// bTwoRoads=true 時的路肩寬度（cm）
	/// 最外側車道邊緣到護欄之間的距離
	///
	/// Shoulder width for Two Roads mode (cm).
	/// Distance from outer lane edge to guardrail.
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	float TwoRoadsShoulderWidthCm = 0.0f;

	/// <summary>
	/// bTwoRoads=false 時的路肩寬度（cm）
	/// 最外側車道邊緣到護欄之間的距離（如 2+2-Lane Wide Road）
	///
	/// Shoulder width for shared road surface (cm).
	/// Distance from outer lane edge to guardrail (e.g. 2+2-Lane Wide Road).
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	float SharedRoadShoulderWidthCm = 0.0f;

	/// <summary>
	/// 是否在 BeginPlay 時自動開始跟隨路徑
	/// Whether to automatically start following on BeginPlay
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Road Path")
	bool bAutoStart = true;

	/// <summary>
	/// 手動觸發開始跟隨路徑
	/// Manually start following the path
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Path")
	void StartFollowing();

private:

	/// <summary>
	/// 從 A* 結果建立內部路徑片段
	/// Build internal path segments from A* result
	/// </summary>
	void BuildPathSegments(const FRoadGraphPath& AStarPath);

	/// <summary>
	/// 內部路徑片段陣列
	/// Internal path segments to follow
	/// </summary>
	TArray<FPathSegmentInternal> PathSegments;

	/// <summary>
	/// 目前正在走第幾段
	/// Current segment index
	/// </summary>
	int32 CurrentSegmentIndex = 0;

	/// <summary>
	/// 目前在該段 spline 上的距離（cm）
	/// Current distance along the current segment's spline (cm)
	/// </summary>
	float CurrentDistance = 0.0f;

	/// <summary>
	/// 是否正在跟隨路徑
	/// Whether currently following a path
	/// </summary>
	bool bIsFollowing = false;

	/// <summary>
	/// 等待第一次 Tick 才啟動（因為 BeginPlay 時 road graph 還沒建好）
	/// Wait for first Tick to start (road graph not ready during BeginPlay)
	/// </summary>
	bool bPendingStart = false;
};
