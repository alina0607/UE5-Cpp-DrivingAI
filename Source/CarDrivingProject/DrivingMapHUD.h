#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "DrivingMapHUD.generated.h"

class UDrivingMapWidget;

/// Driving map HUD — creates/manages map widget and handles hotkeys
UCLASS()
class CARDRIVINGPROJECT_API ADrivingMapHUD : public AHUD
{
	GENERATED_BODY()

public:

	/// Map widget class (override with BP subclass for custom look)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map")
	TSubclassOf<UDrivingMapWidget> MapWidgetClass;

	/// Get map widget instance
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Driving Map")
	UDrivingMapWidget* GetMapWidget() const { return MapWidget; }

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	UPROPERTY()
	TObjectPtr<UDrivingMapWidget> MapWidget;

	/// Toggle map on M key
	void HandleToggleMap();

	/// Escape: close fullscreen or deselect vehicle
	void HandleEscapeKey();
};
