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
	//  Timing (match BP traffic light)
	// ================================================================

	///Green light duration (seconds)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Timing")
	float GreenTime = 30.0f;

	/// Yellow + transition time (seconds)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Timing")
	float YellowRedTime = 3.0f;

	/// All-red time (seconds) — safety gap
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Timing")
	float AllRedTime = 2.0f;

	// ================================================================
	//  Collider Groups
	// ================================================================

	/// Group 1 colliders (A+B direction)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Groups")
	TArray<AActor*> FirstGroup;

	/// Group 2 colliders (C+D direction)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Groups")
	TArray<AActor*> SecondGroup;

private:
	/// Current time within the cycle (seconds)
	float CycleTimer = 0.0f;

	/// Total duration of one full cycle
	float FullCycleTime = 0.0f;

	/// Previous frame state (avoid redundant sets)
	int32 LastState = -1;

	/// Set collision on/off for a group
	void SetGroupEnabled(const TArray<AActor*>& Group, bool bEnabled);

	/// Configure actor to respond Block on ObstacleDetect channel
	void ConfigureObstacleChannel(AActor* Actor);
};
