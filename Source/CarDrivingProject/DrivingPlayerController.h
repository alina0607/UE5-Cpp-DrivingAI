#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "DrivingPlayerController.generated.h"

/// Base player controller for driving project.
/// Empty for now — extend later for input handling, UI control, etc.
UCLASS()
class CARDRIVINGPROJECT_API ADrivingPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	ADrivingPlayerController();
};
