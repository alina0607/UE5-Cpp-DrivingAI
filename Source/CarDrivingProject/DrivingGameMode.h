#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "DrivingGameMode.generated.h"

/// Base game mode for driving project.
/// Auto-assigns default HUD, PlayerController, etc.
UCLASS()
class CARDRIVINGPROJECT_API ADrivingGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ADrivingGameMode();
};
