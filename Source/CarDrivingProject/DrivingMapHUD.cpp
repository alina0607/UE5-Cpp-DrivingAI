#include "DrivingMapHUD.h"
#include "DrivingMapWidget.h"
#include "GameFramework/PlayerController.h"

void ADrivingMapHUD::BeginPlay()
{
	Super::BeginPlay();

	APlayerController* PC = GetOwningPlayerController();
	if (!PC) return;

	// Determine widget class
	TSubclassOf<UDrivingMapWidget> WidgetClass = MapWidgetClass;
	if (!WidgetClass)
	{
		WidgetClass = UDrivingMapWidget::StaticClass();
	}

	// Create map widget
	MapWidget = CreateWidget<UDrivingMapWidget>(PC, WidgetClass);
	if (MapWidget)
	{
		MapWidget->AddToViewport(100);
	}

	// Bind hotkeys
	EnableInput(PC);
	if (InputComponent)
	{
		InputComponent->BindKey(EKeys::M, IE_Pressed, this, &ADrivingMapHUD::HandleToggleMap);
		InputComponent->BindKey(EKeys::Escape, IE_Pressed, this, &ADrivingMapHUD::HandleEscapeKey);
	}
}

void ADrivingMapHUD::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (MapWidget)
	{
		MapWidget->RemoveFromParent();
		MapWidget = nullptr;
	}
	Super::EndPlay(EndPlayReason);
}

void ADrivingMapHUD::HandleToggleMap()
{
	if (!MapWidget) return;

	MapWidget->ToggleMapSize();

	APlayerController* PC = GetOwningPlayerController();
	if (!PC) return;

	if (MapWidget->bIsFullscreen)
	{
		// Fullscreen: show cursor, enable UI interaction
		PC->bShowMouseCursor = true;
		FInputModeGameAndUI InputMode;
		InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
		InputMode.SetWidgetToFocus(MapWidget->TakeWidget());
		PC->SetInputMode(InputMode);
	}
	else
	{
		// Minimap: hide cursor, return to game input
		PC->bShowMouseCursor = false;
		PC->SetInputMode(FInputModeGameOnly());
	}
}

void ADrivingMapHUD::HandleEscapeKey()
{
	if (!MapWidget) return;

	if (MapWidget->bIsFullscreen)
	{
		// Close fullscreen first
		HandleToggleMap();
	}
	else if (MapWidget->GetSelectedVehicle())
	{
		// Deselect vehicle
		MapWidget->DeselectVehicle();
	}
}
