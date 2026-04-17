// Collider switcher — synced with BP traffic light timing, two groups alternate

#include "ColliderSwitcher.h"
#include "CollisionChannels.h"
#include "Components/PrimitiveComponent.h"

AColliderSwitcher::AColliderSwitcher()
{
	PrimaryActorTick.bCanEverTick = true;
}

void AColliderSwitcher::BeginPlay()
{
	Super::BeginPlay();

	// Auto-configure all cubes to respond Block on ObstacleDetect channel.
	// Also stamp tag "TrafficSignal" so PathFollowers can distinguish a red-light
	// block from a real static obstacle during stuck classification.
	const FName TrafficSignalTag(TEXT("TrafficSignal"));
	auto TagAndConfigure = [&](AActor* Actor)
	{
		if (!Actor) return;
		ConfigureObstacleChannel(Actor);
		if (!Actor->Tags.Contains(TrafficSignalTag))
		{
			Actor->Tags.Add(TrafficSignalTag);
		}
	};
	for (AActor* Actor : FirstGroup)  { TagAndConfigure(Actor); }
	for (AActor* Actor : SecondGroup) { TagAndConfigure(Actor); }

	// Calculate full cycle duration
	// Cycle: G1 green → yellow → all-red → G2 green → yellow → all-red
	const float HalfCycle = GreenTime + YellowRedTime + AllRedTime;
	FullCycleTime = HalfCycle * 2.0f;

	CycleTimer = 0.0f;
	LastState = -1;

}

void AColliderSwitcher::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (FullCycleTime <= 0.0f) return;

	CycleTimer += DeltaTime;
	if (CycleTimer >= FullCycleTime)
	{
		CycleTimer -= FullCycleTime;
	}

	// Determine current phase within the cycle
	//
	//Timeline:
	//   0 ─── GreenTime ─── +YellowRedTime ─── +AllRedTime ─── GreenTime ─── +YellowRedTime ─── +AllRedTime

	const float HalfCycle = GreenTime + YellowRedTime + AllRedTime;

	bool bG1Enabled; //true = blocker on (stop cars)
	bool bG2Enabled;

	if (CycleTimer < GreenTime)
	{
		// Phase 1: G1 green → G1 off (pass), G2 on (stop)
		bG1Enabled = false;
		bG2Enabled = true;
	}
	else if (CycleTimer < GreenTime + YellowRedTime)
	{
		// Phase 2: G1 yellow → both on (both stop)
		bG1Enabled = true;
		bG2Enabled = true;
	}
	else if (CycleTimer < HalfCycle)
	{
		// Phase 3: All red → both on
		bG1Enabled = true;
		bG2Enabled = true;
	}
	else if (CycleTimer < HalfCycle + GreenTime)
	{
		// Phase 4: G2 green → G2 off (pass), G1 on (stop)
		bG1Enabled = true;
		bG2Enabled = false;
	}
	else if (CycleTimer < HalfCycle + GreenTime + YellowRedTime)
	{
		// Phase 5: G2 yellow → both on
		bG1Enabled = true;
		bG2Enabled = true;
	}
	else
	{
		// Phase 6: All red → both on
		bG1Enabled = true;
		bG2Enabled = true;
	}

	// Use state code to avoid redundant sets per frame
	const int32 CurrentState = (bG1Enabled ? 1 : 0) | (bG2Enabled ? 2 : 0);
	if (CurrentState != LastState)
	{
		SetGroupEnabled(FirstGroup, bG1Enabled);
		SetGroupEnabled(SecondGroup, bG2Enabled);
		LastState = CurrentState;

	}
}

void AColliderSwitcher::SetGroupEnabled(const TArray<AActor*>& Group, bool bEnabled)
{
	for (AActor* Actor : Group)
	{
		if (!Actor) continue;
		Actor->SetActorHiddenInGame(!bEnabled);
		Actor->SetActorEnableCollision(bEnabled);
	}
}

void AColliderSwitcher::ConfigureObstacleChannel(AActor* Actor)
{
	if (!Actor) return;

	TArray<UPrimitiveComponent*> Primitives;
	Actor->GetComponents<UPrimitiveComponent>(Primitives);
	for (UPrimitiveComponent* Prim : Primitives)
	{
		Prim->SetCollisionResponseToChannel(ECC_ObstacleDetect, ECR_Block);
	}
}
