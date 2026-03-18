#pragma once

#include "CoreMinimal.h"
#include "RoadRuleTypes.generated.h"

/// <summary>
/// Defines high-level driving rules associated with one road type.
/// </summary>
USTRUCT(BlueprintType)
struct CARDRIVINGPROJECT_API FRoadDrivingRule
{
    GENERATED_BODY()

    /// <summary>
    /// Debug name of this rule.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    FString RuleName;

    /// <summary>
    /// Whether this road is one-way only.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bIsOneWay = false;

    /// <summary>
    /// Number of lanes in the forward direction.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 ForwardLaneCount = 1;

    /// <summary>
    /// Number of lanes in the reverse direction.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    int32 ReverseLaneCount = 1;

    /// <summary>
    /// Whether overtaking is allowed on this road type.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bAllowOvertaking = false;

    /// <summary>
    /// Whether overtaking requires entering the opposing lane.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bUseOpposingLaneForPassing = false;

    /// <summary>
    /// Whether vehicles should return to the default travel lane after overtaking.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bReturnToDefaultLaneAfterPass = true;

    /// <summary>
    /// Whether the left lane should be treated as a passing / faster lane.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bPreferLeftLaneForPassing = false;

    /// <summary>
    /// Whether the rightmost lane should be treated as the slower / exit-preparation lane.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bRightmostLaneIsSlowOrExitLane = false;

    /// <summary>
    /// Whether this road type is currently implemented.
    /// </summary>
    UPROPERTY(BlueprintReadOnly)
    bool bImplemented = false;
};
