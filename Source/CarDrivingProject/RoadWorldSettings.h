#pragma once

#include "CoreMinimal.h"
#include "GameFramework/WorldSettings.h"
#include "RoadWorldSettings.generated.h"

/// <summary>
/// Custom WorldSettings that stores road network configuration
/// for the current map.
/// </summary>
UCLASS()
class CARDRIVINGPROJECT_API ARoadWorldSettings : public AWorldSettings
{
	GENERATED_BODY()

public:

    /// <summary>
    /// Blueprint actor class used to identify road actors in this map.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Network")
    TSubclassOf<AActor> RoadActorClass;

    /// <summary>
    /// Distance threshold used later for merging nearby road endpoints into graph nodes.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Network")
    float NodeMergeDistance = 100.0f;

    /// <summary>
    /// Whether to draw graph nodes for debugging.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    bool bDrawDebugNodes = true;

    /// <summary>
    /// Whether to draw graph edges for debugging.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    bool bDrawDebugEdges = true;

    /// <summary>
    /// Whether to draw node id labels.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    bool bDrawDebugNodeLabels = true;

    /// <summary>
    /// Whether to draw edge labels.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    bool bDrawDebugEdgeLabels = true;

    /// <summary>
    /// Radius of debug node spheres.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    float DebugNodeRadius = 200.0f;

    /// <summary>
    /// Thickness of debug edge lines.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    float DebugEdgeThickness = 20.0f;

    /// <summary>
    /// Debug draw duration in seconds.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    float DebugDrawDuration = 30.0f;

    // ---- 叉路偵測設定 / Junction Detection Settings ----

    /// <summary>
    /// 兩條路的取樣點距離小於此值就視為交會（公分）
    /// Two spline sample points closer than this are considered a junction (cm)
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Network")
    float JunctionDetectionRadius = 150.0f;

    /// <summary>
    /// 沿 spline 每隔多少公分取一個樣本點
    /// Sample one point every this many cm along each spline
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Network")
    float SplineSampleStep = 200.0f;
};
