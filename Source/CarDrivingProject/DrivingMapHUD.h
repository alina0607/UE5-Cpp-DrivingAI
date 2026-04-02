#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "DrivingMapHUD.generated.h"

class UDrivingMapWidget;

/// 駕駛地圖 HUD — 建立並管理地圖 Widget，處理快捷鍵
/// Driving map HUD — creates/manages map widget and handles hotkeys
UCLASS()
class CARDRIVINGPROJECT_API ADrivingMapHUD : public AHUD
{
	GENERATED_BODY()

public:

	/// 地圖 Widget 類別（可用 BP 子類覆寫外觀）
	/// Map widget class (override with BP subclass for custom look)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map")
	TSubclassOf<UDrivingMapWidget> MapWidgetClass;

	/// 取得地圖 Widget / Get map widget instance
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Driving Map")
	UDrivingMapWidget* GetMapWidget() const { return MapWidget; }

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	UPROPERTY()
	TObjectPtr<UDrivingMapWidget> MapWidget;

	/// M 鍵切換地圖 / Toggle map on M key
	void HandleToggleMap();

	/// Escape 鍵：全螢幕時關閉地圖，否則取消選取
	/// Escape: close fullscreen or deselect vehicle
	void HandleEscapeKey();
};
