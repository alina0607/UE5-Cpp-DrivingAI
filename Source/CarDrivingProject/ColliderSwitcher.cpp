// 碰撞體切換器 — 配合 BP 紅綠燈時序，兩組 Collider 交替啟用
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

	// 自動設定所有 Cube 的 ObstacleDetect = Block
	// Auto-configure all cubes to respond Block on ObstacleDetect channel
	for (AActor* Actor : FirstGroup)
	{
		ConfigureObstacleChannel(Actor);
	}
	for (AActor* Actor : SecondGroup)
	{
		ConfigureObstacleChannel(Actor);
	}

	// 計算完整循環時長 / Calculate full cycle duration
	// 循環：G1綠 → 黃 → 全紅 → G2綠 → 黃 → 全紅
	// Cycle: G1 green → yellow → all-red → G2 green → yellow → all-red
	const float HalfCycle = GreenTime + YellowRedTime + AllRedTime;
	FullCycleTime = HalfCycle * 2.0f;

	CycleTimer = 0.0f;
	LastState = -1;

	UE_LOG(LogTemp, Log,
		TEXT("[ColliderSwitcher] 啟動: G1=%d個 G2=%d個 循環=%.1fs (綠%.1f+黃%.1f+全紅%.1f)×2 / Started: G1=%d G2=%d Cycle=%.1fs"),
		FirstGroup.Num(), SecondGroup.Num(), FullCycleTime,
		GreenTime, YellowRedTime, AllRedTime,
		FirstGroup.Num(), SecondGroup.Num(), FullCycleTime);
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

	// 判斷目前在循環中的哪個階段
	// Determine current phase within the cycle
	//
	// 時間軸 / Timeline:
	//   0 ─── GreenTime ─── +YellowRedTime ─── +AllRedTime ─── GreenTime ─── +YellowRedTime ─── +AllRedTime
	//   |  G1 綠（G1 OFF）  |   G1 黃（G1 ON）  |  全紅（都ON） | G2 綠（G2 OFF） |  G2 黃（G2 ON）   | 全紅（都ON）|
	//
	// G1 OFF = 第一組碰撞體關閉（車可以通過）
	// G1 ON  = 第一組碰撞體開啟（車要停）

	const float HalfCycle = GreenTime + YellowRedTime + AllRedTime;

	bool bG1Enabled; // true = 碰撞體開（擋車）/ true = blocker on (stop cars)
	bool bG2Enabled;

	if (CycleTimer < GreenTime)
	{
		// 階段 1：第一組綠燈 → G1 碰撞體關（車通行），G2 碰撞體開（車停）
		// Phase 1: G1 green → G1 off (pass), G2 on (stop)
		bG1Enabled = false;
		bG2Enabled = true;
	}
	else if (CycleTimer < GreenTime + YellowRedTime)
	{
		// 階段 2：第一組黃燈 → 兩組都開（都要停）
		// Phase 2: G1 yellow → both on (both stop)
		bG1Enabled = true;
		bG2Enabled = true;
	}
	else if (CycleTimer < HalfCycle)
	{
		// 階段 3：全紅 → 兩組都開
		// Phase 3: All red → both on
		bG1Enabled = true;
		bG2Enabled = true;
	}
	else if (CycleTimer < HalfCycle + GreenTime)
	{
		// 階段 4：第二組綠燈 → G2 碰撞體關，G1 碰撞體開
		// Phase 4: G2 green → G2 off (pass), G1 on (stop)
		bG1Enabled = true;
		bG2Enabled = false;
	}
	else if (CycleTimer < HalfCycle + GreenTime + YellowRedTime)
	{
		// 階段 5：第二組黃燈 → 兩組都開
		// Phase 5: G2 yellow → both on
		bG1Enabled = true;
		bG2Enabled = true;
	}
	else
	{
		// 階段 6：全紅 → 兩組都開
		// Phase 6: All red → both on
		bG1Enabled = true;
		bG2Enabled = true;
	}

	// 用狀態碼避免每幀重複設定 / Use state code to avoid redundant sets per frame
	const int32 CurrentState = (bG1Enabled ? 1 : 0) | (bG2Enabled ? 2 : 0);
	if (CurrentState != LastState)
	{
		SetGroupEnabled(FirstGroup, bG1Enabled);
		SetGroupEnabled(SecondGroup, bG2Enabled);
		LastState = CurrentState;

		UE_LOG(LogTemp, Log,
			TEXT("[ColliderSwitcher] G1=%s G2=%s (T=%.1f/%.1f)"),
			bG1Enabled ? TEXT("STOP") : TEXT("PASS"),
			bG2Enabled ? TEXT("STOP") : TEXT("PASS"),
			CycleTimer, FullCycleTime);
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
