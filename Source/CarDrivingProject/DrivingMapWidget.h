#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styling/SlateBrush.h"
#include "DrivingMapWidget.generated.h"

class ACameraActor;
class URoadNetworkSubsystem;
class URoadPathFollowerComponent;
class USpringArmComponent;
class UTexture2D;

/// 地圖上的車輛快取 / Cached vehicle info for map rendering
struct FMapVehicleCache
{
	TWeakObjectPtr<AActor> Actor;
	TWeakObjectPtr<URoadPathFollowerComponent> PathFollower;
	FVector WorldLocation = FVector::ZeroVector;
	FVector ForwardVector = FVector::ForwardVector;
	float Speed = 0.0f;
};

/// 地圖上的節點快取（點擊用）/ Cached node for click detection
struct FMapNodeCache
{
	int32 NodeId = INDEX_NONE;
	FVector WorldLocation = FVector::ZeroVector;
};

/// 駕駛地圖 Widget — 精簡版
///
/// 功能：背景圖 + 車輛箭頭 + 點擊跟隨/導航
/// Features: background texture + vehicle arrows + click-to-follow/navigate
UCLASS()
class CARDRIVINGPROJECT_API UDrivingMapWidget : public UUserWidget
{
	GENERATED_BODY()

public:

	// ================================================================
	//  地圖模式 / Map Mode
	// ================================================================

	UPROPERTY(BlueprintReadWrite, Category = "Driving Map")
	bool bIsFullscreen = false;

	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void ToggleMapSize();

	// ================================================================
	//  車輛選取與攝影機 / Vehicle Selection & Camera
	// ================================================================

	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void SelectVehicle(AActor* Vehicle);

	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void DeselectVehicle();

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Driving Map")
	AActor* GetSelectedVehicle() const;

	// ================================================================
	//  地圖背景 / Map Background
	// ================================================================

	/// 背景圖（匯入 UE 的 Texture2D，在 BP 裡設定）
	/// Background image — import your screenshot as Texture2D, set in BP
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Background")
	TObjectPtr<UTexture2D> MapTexture;

	/// 背景圖對應的世界座標範圍 — 左下角
	/// World coordinate of the bottom-left corner of your map image
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Background")
	FVector2D MapTextureWorldMin = FVector2D(-50000.0, -50000.0);

	/// 背景圖對應的世界座標範圍 — 右上角
	/// World coordinate of the top-right corner of your map image
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Background")
	FVector2D MapTextureWorldMax = FVector2D(50000.0, 50000.0);

	// ================================================================
	//  外觀設定 / Visual Settings
	// ================================================================

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Minimap")
	FVector2D MinimapSize = FVector2D(350.0, 350.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Minimap")
	FVector2D MinimapMargin = FVector2D(20.0, 20.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Fullscreen")
	float FullscreenCoverage = 0.85f;

	/// 背景色（無背景圖時）/ Fallback background color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor BackgroundColor = FLinearColor(0.02f, 0.03f, 0.08f, 0.85f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor BorderColor = FLinearColor(0.2f, 0.3f, 0.5f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor VehicleColor = FLinearColor(0.1f, 1.0f, 0.3f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor SelectedColor = FLinearColor(1.0f, 0.85f, 0.0f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor DestinationColor = FLinearColor(1.0f, 0.2f, 0.2f, 1.0f);

	// ================================================================
	//  攝影機 / Camera Settings
	// ================================================================

	/// 右鍵拖曳旋轉靈敏度 / Right-click drag orbit sensitivity
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera")
	float OrbitSensitivity = 1.0f;

	/// 滾輪縮放速度 / Scroll zoom speed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera", meta = (ClampMin = "10"))
	float ZoomSpeed = 80.0f;

	/// SpringArm 最短距離 / Min arm length
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera", meta = (ClampMin = "50"))
	float MinFollowDistance = 200.0f;

	/// SpringArm 最長距離 / Max arm length
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera")
	float MaxFollowDistance = 3000.0f;

	// ================================================================
	//  自由相機 / Free Camera
	// ================================================================

	/// 自由相機移動速度（cm/s）/ Free camera movement speed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Free Camera")
	float FreeCameraMoveSpeed = 1500.0f;

	/// 自由相機旋轉速度 / Free camera rotation speed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Free Camera")
	float FreeCameraRotateSpeed = 2.5f;

	/// 按住 Shift 的加速倍率 / Speed multiplier when holding Shift
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Free Camera")
	float FreeCameraFastMultiplier = 3.0f;

	/// 目前是否為自由相機模式 / Whether currently in free camera mode
	UPROPERTY(BlueprintReadOnly, Category = "Driving Map|Free Camera")
	bool bFreeCameraMode = true;

protected:
	virtual void NativeConstruct() override;
	virtual void NativeDestruct() override;
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;

	virtual int32 NativePaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry,
		const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements,
		int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;

	virtual FReply NativeOnMouseButtonDown(const FGeometry& InGeometry,
		const FPointerEvent& InMouseEvent) override;

	virtual FReply NativeOnMouseWheel(const FGeometry& InGeometry,
		const FPointerEvent& InMouseEvent) override;

private:

	// ---- 快取 / Cache ----

	TArray<FMapNodeCache> CachedNodes;
	TArray<FMapVehicleCache> CachedVehicles;
	bool bCacheBuilt = false;

	FVector2D WorldBoundsMin = FVector2D::ZeroVector;
	FVector2D WorldBoundsMax = FVector2D(1.0, 1.0);

	UPROPERTY()
	TWeakObjectPtr<AActor> SelectedVehiclePtr;
	int32 SelectedDestinationNodeId = INDEX_NONE;

	/// 自由相機 Actor（遊戲開始時建立一次，永不銷毀）
	/// Persistent free camera actor — spawned once at startup
	UPROPERTY()
	TObjectPtr<ACameraActor> FreeCameraActor;

	// ---- 背景 Brush ----

	mutable FSlateBrush MapTextureBrush;
	bool bBrushReady = false;

	void SetupBrush();

	// ---- 快取建立 ----

	void BuildNodeCache();
	void UpdateVehicleCache();

	// ---- 座標轉換 ----

	FVector2D WorldToMap(const FVector& WorldPos, const FVector2D& MapOrigin, const FVector2D& MapSize) const;
	FVector MapToWorld(const FVector2D& MapPos, const FVector2D& MapOrigin, const FVector2D& MapSize) const;
	void GetMapDrawRect(const FGeometry& Geometry, FVector2D& OutOrigin, FVector2D& OutSize) const;

	// ---- 繪製 ----

	void DrawBackground(FSlateWindowElementList& OutDrawElements, const FGeometry& Geometry,
		const FVector2D& MapOrigin, const FVector2D& MapSize, int32 LayerId) const;

	void DrawArrow(FPaintContext& Context, const FVector2D& Position, const FVector2D& Direction,
		float Size, const FLinearColor& Color, float Thickness = 2.0f) const;

	void DrawCircle(FPaintContext& Context, const FVector2D& Center, float Radius,
		const FLinearColor& Color, float Thickness = 1.5f, int32 NumSegments = 20) const;

	// ---- 點擊偵測 ----

	AActor* FindVehicleNearMapPos(const FVector2D& LocalPos, const FVector2D& MapOrigin,
		const FVector2D& MapSize, float Radius) const;

	int32 FindNodeNearMapPos(const FVector2D& LocalPos, const FVector2D& MapOrigin,
		const FVector2D& MapSize, float Radius) const;

	// ---- 攝影機 ----

	/// 儲存選車前的 SpringArm 旋轉，取消時還原
	/// Store original boom rotation before orbit, restore on deselect
	FRotator OriginalBoomRotation = FRotator::ZeroRotator;

	void InitFreeCamera();
	void UpdateFreeCamera(float DeltaTime);
};
