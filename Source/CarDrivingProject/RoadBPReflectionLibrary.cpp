// Fill out your copyright notice in the Description page of Project Settings.


#include "RoadBPReflectionLibrary.h"
#include "UObject/UnrealType.h"
#include "Engine/Engine.h"
#include "Components/SplineComponent.h"

/// <summary>
/// Test function for verifying the library is loaded and callable.
/// </summary>
void URoadBPReflectionLibrary::TestFunction()
{
    UE_LOG(LogTemp, Warning, TEXT("TestFunction called from URoadBPReflectionLibrary"));

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            -1,
            3.0f,
            FColor::Green,
            TEXT("RoadBPReflectionLibrary::TestFunction works!")
        );
    }
}

/// <summary>
/// Debug function that iterates through all reflected properties
/// of the given actor and prints them to the output log.
/// </summary>
void URoadBPReflectionLibrary::DumpAllProperties(AActor* TargetActor)
{
    if (!TargetActor)
    {
        UE_LOG(LogTemp, Warning, TEXT("DumpAllProperties: TargetActor is NULL"));
        return;
    }

    UClass* ActorClass = TargetActor->GetClass();

    UE_LOG(LogTemp, Warning, TEXT("===== Dump Properties for Actor Class: %s ====="), *ActorClass->GetName());

    // TFieldIterator walks through the class metadata
    for (TFieldIterator<FProperty> PropertyIterator(ActorClass); PropertyIterator; ++PropertyIterator)
    {
        FProperty* Property = *PropertyIterator;
        FString PropertyName = Property->GetName();
        FString PropertyType = Property->GetClass()->GetName();

        UE_LOG(LogTemp, Warning, TEXT("Property Name: %s | Type: %s"),
            *PropertyName,
            *PropertyType);
    }
    UE_LOG(LogTemp, Warning, TEXT("===== End Dump ====="));
}

/// <summary>
/// Extract the core road data from one Blueprint road actor
/// and store it into FRoadRuntimeData using UE reflection.
/// </summary>
bool URoadBPReflectionLibrary::ExtractRoadCoreData(AActor* RoadActor, FRoadRuntimeData& OutData)
{
    if (!RoadActor)
    {
        UE_LOG(LogTemp, Warning, TEXT("ExtractRoadCoreData: RoadActor is null"));
        return false;
    }

    // Reset output data before filling it.
    OutData = FRoadRuntimeData();
    OutData.RoadActor = RoadActor;

    UClass* ActorClass = RoadActor->GetClass();

    // Read "Use Spline" as a bool property.
    if (FBoolProperty* BoolProp = FindFProperty<FBoolProperty>(ActorClass, TEXT("Use Spline")))
    {
        OutData.bUseSpline = BoolProp->GetPropertyValue_InContainer(RoadActor);
    }

    // Read "Closed Loop" as a bool property.
    if (FBoolProperty* BoolProp = FindFProperty<FBoolProperty>(ActorClass, TEXT("Closed Loop")))
    {
        OutData.bClosedLoop = BoolProp->GetPropertyValue_InContainer(RoadActor);
    }

    // Read "Length, m" as a double property.
    if (FDoubleProperty* DoubleProp = FindFProperty<FDoubleProperty>(ActorClass, TEXT("Length, m")))
    {
        OutData.LengthMeters = DoubleProp->GetPropertyValue_InContainer(RoadActor);
    }

    // Read "Road Type" as a byte property.
    if (FByteProperty* ByteProp = FindFProperty<FByteProperty>(ActorClass, TEXT("Road Type")))
    {
        OutData.RoadType = ByteProp->GetPropertyValue_InContainer(RoadActor);
    }

    // Read "Input Spline" as an object property and cast it to USplineComponent.
    if (FObjectProperty* ObjProp = FindFProperty<FObjectProperty>(ActorClass, TEXT("Input Spline")))
    {
        UObject* ObjectValue = ObjProp->GetObjectPropertyValue_InContainer(RoadActor);
        OutData.InputSpline = Cast<USplineComponent>(ObjectValue);
    }

    // Read "Start World Location" as a struct property and extract FVector.
    if (FStructProperty* StructProp = FindFProperty<FStructProperty>(ActorClass, TEXT("Start World Location")))
    {
        if (StructProp->Struct == TBaseStructure<FVector>::Get())
        {
            if (FVector* VecPtr = StructProp->ContainerPtrToValuePtr<FVector>(RoadActor))
            {
                OutData.StartWorldLocation = *VecPtr;
            }
        }
    }

    // Read "End World Location" as a struct property and extract FVector.
    if (FStructProperty* StructProp = FindFProperty<FStructProperty>(ActorClass, TEXT("End World Location")))
    {
        if (StructProp->Struct == TBaseStructure<FVector>::Get())
        {
            if (FVector* VecPtr = StructProp->ContainerPtrToValuePtr<FVector>(RoadActor))
            {
                OutData.EndWorldLocation = *VecPtr;
            }
        }
    }

    /*
    UE_LOG(LogTemp, Warning, TEXT("=== ExtractRoadCoreData Result ==="));
    UE_LOG(LogTemp, Warning, TEXT("Actor: %s"), *RoadActor->GetName());
    UE_LOG(LogTemp, Warning, TEXT("UseSpline: %s"), OutData.bUseSpline ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("ClosedLoop: %s"), OutData.bClosedLoop ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("LengthMeters: %f"), OutData.LengthMeters);
    UE_LOG(LogTemp, Warning, TEXT("RoadType: %d"), (int32)OutData.RoadType);
    UE_LOG(LogTemp, Warning, TEXT("Start: %s"), *OutData.StartWorldLocation.ToString());
    UE_LOG(LogTemp, Warning, TEXT("End: %s"), *OutData.EndWorldLocation.ToString());
    UE_LOG(LogTemp, Warning, TEXT("Spline: %s"), OutData.InputSpline ? *OutData.InputSpline->GetName() : TEXT("NULL"));
    */
    return OutData.InputSpline != nullptr;
}
