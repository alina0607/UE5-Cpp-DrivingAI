#include "DrivingGameMode.h"
#include "DrivingPlayerController.h"
#include "DrivingMapHUD.h"
#include "GameFramework/SpectatorPawn.h"

ADrivingGameMode::ADrivingGameMode()
{
	// 指定預設類別 — BP 子類可以覆寫這些
	// Set default classes — BP subclasses can override these

	PlayerControllerClass = ADrivingPlayerController::StaticClass();
	HUDClass = ADrivingMapHUD::StaticClass();

	// 不自動生成車輛 Pawn — 所有車由 TrafficManager 生成
	// 玩家以自由相機開始，在地圖上點選車輛跟隨
	// Don't auto-spawn vehicle pawn — all vehicles managed by TrafficManager.
	// Player starts with free camera, clicks map to follow vehicles.
	DefaultPawnClass = ASpectatorPawn::StaticClass();
}
