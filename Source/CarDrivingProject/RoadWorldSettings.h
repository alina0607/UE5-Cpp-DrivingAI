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
    float NodeMergeDistance = 250.0f;

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
    float DebugNodeRadius = 50.0f;

    /// <summary>
    /// Thickness of debug edge lines.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    float DebugEdgeThickness = 20.0f;

    /// <summary>
    /// Debug 線條的 Z 軸抬高量（cm）— 避免被路面遮住
    /// Z offset for debug lines (cm) — raise above road surface.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    float DebugDrawZOffset = 500.0f;

    /// <summary>
    /// Debug draw duration in seconds.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Debug")
    float DebugDrawDuration = 300.0f;

    // ---- Junction Detection Settings ----

    /// <summary>
    /// Two spline sample points closer than this are considered a junction (cm)
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Network")
    float JunctionDetectionRadius = 500.0f;

    /// <summary>
    /// Sample one point every this many cm along each spline
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Network")
    float SplineSampleStep = 200.0f;

    /// <summary>
    /// 標準車道寬度（cm）。3.5m 為國際通用標準
    /// Standard lane width in cm. 3.5m is the international standard.
    /// </summary>
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Road Network")
    float LaneWidthCm = 350.0f;
};
