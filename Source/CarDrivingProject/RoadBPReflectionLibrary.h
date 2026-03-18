// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RoadTypes.h"
#include "RoadBPReflectionLibrary.generated.h"

/// <summary>
/// Blueprint function library that exposes reflection utilities
/// for extracting data from Blueprint road actors into C++ structs.
/// </summary>
UCLASS()
class CARDRIVINGPROJECT_API URoadBPReflectionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:

	/// <summary>
	/// Test function for verifying the library is loaded and callable.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road BP Reflection")
	static void TestFunction();

	/// <summary>
	/// DumpAllProperties = debug function that prints all reflected properties
	/// of the given actor to the output log.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "BP Reflection")
	static void DumpAllProperties(AActor* TargetActor);

	/// <summary>
	/// Extract the core road data from one Blueprint road actor
	/// and store it into FRoadRuntimeData.
	/// </summary>
    UFUNCTION(BlueprintCallable, Category = "BP Reflection")
    static bool ExtractRoadCoreData(AActor* RoadActor, FRoadRuntimeData& OutData);

};
