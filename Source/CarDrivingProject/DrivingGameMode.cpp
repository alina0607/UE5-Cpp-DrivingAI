#include "DrivingGameMode.h"
#include "DrivingPlayerController.h"
#include "DrivingMapHUD.h"
#include "DrivingVehiclePawn.h"

ADrivingGameMode::ADrivingGameMode()
{
	// 指定預設類別 — BP 子類可以覆寫這些
	// Set default classes — BP subclasses can override these

	PlayerControllerClass = ADrivingPlayerController::StaticClass();
	HUDClass = ADrivingMapHUD::StaticClass();
	DefaultPawnClass = ADrivingVehiclePawn::StaticClass();
}
