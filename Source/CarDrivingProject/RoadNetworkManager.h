// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RoadTypes.h"
#include "RoadNetworkManager.generated.h"

/// <summary>
/// Legacy road network manager actor.
/// Road network logic has been moved to URoadNetworkSubsystem.
/// </summary>
UCLASS()
class CARDRIVINGPROJECT_API ARoadNetworkManager : public AActor
{
	GENERATED_BODY()

public:

	ARoadNetworkManager();

protected:

	virtual void BeginPlay() override;

public:

	/// <summary>
	/// Blueprint actor class used to identify road actors in this map.
	/// </summary>
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Network")
	TSubclassOf<AActor> RoadActorClass;

	/// <summary>
	/// Stores all extracted road runtime data for the whole level.
	/// </summary>
	UPROPERTY(BlueprintReadOnly, Category = "Road Network")
	TArray<FRoadRuntimeData> AllRoads;

	/// <summary>
	/// Scan the level and rebuild the road cache.
	/// </summary>
	UFUNCTION(BlueprintCallable, Category = "Road Network")
	void BuildRoadCache();
};
