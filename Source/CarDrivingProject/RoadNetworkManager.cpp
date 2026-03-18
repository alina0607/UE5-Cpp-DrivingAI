// Fill out your copyright notice in the Description page of Project Settings.


#include "RoadNetworkManager.h"
#include "Kismet/GameplayStatics.h"
#include "RoadBPReflectionLibrary.h"
#include "RoadNetworkSubsystem.h"

ARoadNetworkManager::ARoadNetworkManager()
{
	PrimaryActorTick.bCanEverTick = false;
}

void ARoadNetworkManager::BeginPlay()
{
	Super::BeginPlay();
	BuildRoadCache();
}

/// <summary>
/// Scan the level for road actors and extract their runtime data.
/// </summary>
void ARoadNetworkManager::BuildRoadCache()
{
	AllRoads.Empty();

	// Find all actors in the current world matching the assigned road Blueprint class.
    TArray<AActor*> FoundRoadActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), RoadActorClass, FoundRoadActors);

    UE_LOG(LogTemp, Warning, TEXT("BuildRoadCache: Found %d road actors"), FoundRoadActors.Num());

	// Extract runtime road data from each Blueprint road actor.
    for (AActor* RoadActor : FoundRoadActors)
    {
        if (!RoadActor)
        {
            continue;
        }

        FRoadRuntimeData RoadData;

        // Convert one Blueprint road actor into one FRoadRuntimeData entry.
        const bool bSuccess = URoadBPReflectionLibrary::ExtractRoadCoreData(RoadActor, RoadData);

        // Only store valid road data.
        if (bSuccess)
        {
            AllRoads.Add(RoadData);
        }
		UE_LOG(LogTemp, Warning, TEXT("BuildRoadCache: Cached %d roads"), AllRoads.Num());

    }
}
