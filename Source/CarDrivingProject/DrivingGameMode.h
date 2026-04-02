#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "DrivingGameMode.generated.h"

/// 駕駛專案的 GameMode 基底類別
/// 自動指定 HUD、PlayerController 等預設類別
///
/// Base game mode for driving project.
/// Auto-assigns default HUD, PlayerController, etc.
UCLASS()
class CARDRIVINGPROJECT_API ADrivingGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ADrivingGameMode();
};
