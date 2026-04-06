// 碰撞體切換器 — 配合 BP 紅綠燈時序，兩組 Collider 交替啟用
// Collider switcher — synced with BP traffic light timing, two groups alternate

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ColliderSwitcher.generated.h"

UCLASS()
class CARDRIVINGPROJECT_API AColliderSwitcher : public AActor
{
	GENERATED_BODY()

public:
	AColliderSwitcher();

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	// ================================================================
	//  時間配置（跟 BP 紅綠燈一致）/ Timing (match BP traffic light)
	// ================================================================

	/// 綠燈時間（秒）/ Green light duration (seconds)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Timing")
	float GreenTime = 30.0f;

	/// 黃燈 + 紅燈過渡時間（秒）/ Yellow + transition time (seconds)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Timing")
	float YellowRedTime = 3.0f;

	/// 全紅時間（秒）— 兩方向都紅的安全間隔 / All-red time (seconds) — safety gap
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Timing")
	float AllRedTime = 2.0f;

	// ================================================================
	//  碰撞體群組 / Collider Groups
	// ================================================================

	/// 第一組碰撞體（A+B 方向）/ Group 1 colliders (A+B direction)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Groups")
	TArray<AActor*> FirstGroup;

	/// 第二組碰撞體（C+D 方向）/ Group 2 colliders (C+D direction)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Groups")
	TArray<AActor*> SecondGroup;

private:
	/// 目前時間在循環中的位置（秒）/ Current time within the cycle (seconds)
	float CycleTimer = 0.0f;

	/// 一個完整循環的總時長 / Total duration of one full cycle
	float FullCycleTime = 0.0f;

	/// 上一幀的狀態（避免每幀重複設定）/ Previous frame state (avoid redundant sets)
	int32 LastState = -1;

	/// 設定碰撞啟用/關閉 / Set collision on/off for a group
	void SetGroupEnabled(const TArray<AActor*>& Group, bool bEnabled);

	/// 設定 Actor 對 ObstacleDetect 通道回應 Block
	/// Configure actor to respond Block on ObstacleDetect channel
	void ConfigureObstacleChannel(AActor* Actor);
};
