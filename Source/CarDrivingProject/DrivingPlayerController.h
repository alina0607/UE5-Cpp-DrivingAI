#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "DrivingPlayerController.generated.h"

/// 駕駛專案的 PlayerController 基底類別
/// 目前留空，未來可擴充輸入處理、UI 控制等
///
/// Base player controller for driving project.
/// Empty for now — extend later for input handling, UI control, etc.
UCLASS()
class CARDRIVINGPROJECT_API ADrivingPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	ADrivingPlayerController();
};
