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

    // Read "Two Roads" as a bool property.
    if (FBoolProperty* BoolProp = FindFProperty<FBoolProperty>(ActorClass, TEXT("Two Roads")))
    {
        OutData.bTwoRoads = BoolProp->GetPropertyValue_InContainer(RoadActor);
    }

    // Read "Two Roads Gap, M" as a double property.
    // 兩條平行路面之間的間距（公尺）
    if (FDoubleProperty* DoubleProp = FindFProperty<FDoubleProperty>(ActorClass, TEXT("Two Roads Gap, M")))
    {
        OutData.TwoRoadsGapM = DoubleProp->GetPropertyValue_InContainer(RoadActor);
    }

    // Read "Road Width Multiplier" as a double property.
    // 路面寬度乘數（預設 1.0）
    if (FDoubleProperty* DoubleProp = FindFProperty<FDoubleProperty>(ActorClass, TEXT("Road Width Multiplier")))
    {
        OutData.RoadWidthMultiplier = DoubleProp->GetPropertyValue_InContainer(RoadActor);
    }

    // Read "Additional Width, m" as a double property.
    // 額外路面寬度（公尺）
    if (FDoubleProperty* DoubleProp = FindFProperty<FDoubleProperty>(ActorClass, TEXT("Additional Width, m")))
    {
        OutData.AdditionalWidthM = DoubleProp->GetPropertyValue_InContainer(RoadActor);
    }

    // 從 "Road Settings" struct 讀取 GuardrailSideOffset（路面半寬）
    // Read GuardrailSideOffset (road half-width) from "Road Settings" struct
    if (FStructProperty* StructProp = FindFProperty<FStructProperty>(ActorClass, TEXT("Road Settings")))
    {
        UScriptStruct* InnerStruct = StructProp->Struct;
        const void* StructData = StructProp->ContainerPtrToValuePtr<void>(RoadActor);

        // BP struct 欄位名稱帶有 GUID 後綴，無法用 FindFProperty 直接找
        // 改用遍歷找名稱開頭為 "GuardrailSideOffset" 的欄位
        // BP struct field names have GUID suffixes, so we iterate and match by prefix
        for (TFieldIterator<FProperty> It(InnerStruct); It; ++It)
        {
            FProperty* InnerProp = *It;
            if (InnerProp->GetName().StartsWith(TEXT("GuardrailSideOffset")))
            {
                if (FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(InnerProp))
                {
                    OutData.GuardrailSideOffsetCm = DoubleProp->GetPropertyValue_InContainer(StructData);
                }
                break;
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
