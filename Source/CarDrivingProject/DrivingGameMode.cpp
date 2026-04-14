#include "DrivingGameMode.h"
#include "DrivingPlayerController.h"
#include "DrivingMapHUD.h"
#include "GameFramework/SpectatorPawn.h"

ADrivingGameMode::ADrivingGameMode()
{
	// Set default classes — BP subclasses can override these

	PlayerControllerClass = ADrivingPlayerController::StaticClass();
	HUDClass = ADrivingMapHUD::StaticClass();

	// Don't auto-spawn vehicle pawn — all vehicles managed by TrafficManager.
	// Player starts with free camera, clicks map to follow vehicles.
	DefaultPawnClass = ASpectatorPawn::StaticClass();
}
