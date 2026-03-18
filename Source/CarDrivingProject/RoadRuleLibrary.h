// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RoadTypes.h"
#include "RoadRuleTypes.h"
#include "RoadRuleLibrary.generated.h"

/// <summary>
/// Static function library that converts road type values
/// into driving rule configurations.
/// </summary>
UCLASS()
class CARDRIVINGPROJECT_API URoadRuleLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:

    /// <summary>
    /// Builds a driving rule from the RoadType stored in FRoadRuntimeData.
    /// </summary>
    UFUNCTION(BlueprintCallable, Category = "Road Rules")
    static FRoadDrivingRule GetDrivingRuleFromRoadData(const FRoadRuntimeData& RoadData);

};
