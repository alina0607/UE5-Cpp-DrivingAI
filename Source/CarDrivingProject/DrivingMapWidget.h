#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styling/SlateBrush.h"
#include "DrivingMapWidget.generated.h"

class ACameraActor;
class ASceneCapture2D;
class URoadNetworkSubsystem;
class URoadPathFollowerComponent;
class USplineComponent;
class UTextureRenderTarget2D;

/// 地圖上的車輛快取 / Cached vehicle info for map rendering
struct FMapVehicleCache
{
	TWeakObjectPtr<AActor> Actor;
	TWeakObjectPtr<URoadPathFollowerComponent> PathFollower;
	FVector WorldLocation = FVector::ZeroVector;
	FVector ForwardVector = FVector::ForwardVector;
	float Speed = 0.0f;
};

/// 地圖上的道路快取（取樣折線）/ Cached road polyline sampled from spline
struct FMapRoadCache
{
	TArray<FVector> WorldPoints;
};

/// 地圖上的節點快取 / Cached node info
struct FMapNodeCache
{
	int32 NodeId = INDEX_NONE;
	FVector WorldLocation = FVector::ZeroVector;
};

/// 駕駛地圖 Widget
///
/// 功能 / Features:
///   - 小地圖（右下角）/ 全螢幕切換
///   - 空拍照片背景（SceneCapture2D 正射投影）
///   - 顯示所有車輛位置與行進方向
///   - 點擊車輛 → 攝影機跟隨
///   - 點擊節點 → 被選取車輛重新導航
///
/// Minimap/fullscreen toggle, aerial photo background via orthographic
/// scene capture, vehicle tracking, click-to-follow, click-to-navigate.
UCLASS()
class CARDRIVINGPROJECT_API UDrivingMapWidget : public UUserWidget
{
	GENERATED_BODY()

public:

	// ================================================================
	//  地圖模式 / Map Mode
	// ================================================================

	/// 是否全螢幕 / Whether map is in fullscreen mode
	UPROPERTY(BlueprintReadWrite, Category = "Driving Map")
	bool bIsFullscreen = false;

	/// 切換大小地圖 / Toggle between minimap and fullscreen
	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void ToggleMapSize();

	// ================================================================
	//  車輛選取與攝影機 / Vehicle Selection & Camera
	// ================================================================

	/// 選取車輛（攝影機跟隨）/ Select vehicle and follow with camera
	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void SelectVehicle(AActor* Vehicle);

	/// 取消選取 / Deselect vehicle
	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void DeselectVehicle();

	/// 取得被選取的車輛 / Get selected vehicle
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Driving Map")
	AActor* GetSelectedVehicle() const;

	/// 重新擷取空拍照片 / Recapture aerial photo
	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void RefreshCapture();

	// ================================================================
	//  空拍設定 / Scene Capture Settings
	// ================================================================

	/// 使用空拍照片作為地圖背景（false = 純線段繪製）
	/// Use aerial photo as map background (false = line-drawn fallback)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Capture")
	bool bUseSceneCapture = true;

	/// 擷取解析度 / Capture resolution (square, pixels)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Capture", meta = (ClampMin = "256", ClampMax = "8192"))
	int32 CaptureResolution = 2048;

	/// 擷取攝影機高度（cm）/ Capture camera height above ground
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Capture", meta = (ClampMin = "1000"))
	float CaptureHeight = 50000.0f;

	/// 空拍照片色調（可加冷色調讓地圖有衛星圖感覺）
	/// Aerial photo tint (blue tint for satellite map look)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Capture")
	FLinearColor CaptureTint = FLinearColor(0.85f, 0.88f, 0.95f, 1.0f);

	/// 擷取前等待秒數（讓場景載入完成）
	/// Delay before capture (wait for scene to finish loading)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Capture", meta = (ClampMin = "0"))
	float CaptureDelaySeconds = 1.0f;

	// ================================================================
	//  外觀設定 / Visual Settings
	// ================================================================

	/// 小地圖尺寸（像素）/ Minimap size in pixels
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Minimap")
	FVector2D MinimapSize = FVector2D(350.0, 350.0);

	/// 小地圖邊距 / Minimap margin from screen edge
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Minimap")
	FVector2D MinimapMargin = FVector2D(20.0, 20.0);

	/// 全螢幕地圖佔螢幕比例 / Fullscreen map screen coverage ratio
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Fullscreen")
	float FullscreenCoverage = 0.85f;

	/// 地圖邊界留白比例 / Map padding ratio around road network
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|General")
	float MapPaddingRatio = 0.1f;

	/// 道路取樣間距（cm，僅 bUseSceneCapture=false 時用）
	/// Road spline sample interval (only used when bUseSceneCapture=false)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|General")
	float RoadSampleInterval = 500.0f;

	/// 是否在空拍照片上疊加道路線 / Draw road line overlay on top of capture
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|General")
	bool bDrawRoadOverlay = false;

	/// 背景色（無空拍時的底色）/ Background color (fallback when no capture)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor BackgroundColor = FLinearColor(0.02f, 0.03f, 0.08f, 0.85f);

	/// 邊框色 / Border color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor BorderColor = FLinearColor(0.2f, 0.3f, 0.5f, 1.0f);

	/// 道路色 / Road color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor RoadColor = FLinearColor(0.35f, 0.4f, 0.5f, 1.0f);

	/// 節點色 / Node color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor NodeColor = FLinearColor(0.3f, 0.6f, 1.0f, 0.9f);

	/// 節點標籤色 / Node label color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor NodeLabelColor = FLinearColor(0.5f, 0.7f, 1.0f, 0.8f);

	/// 車輛色 / Vehicle color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor VehicleColor = FLinearColor(0.1f, 1.0f, 0.3f, 1.0f);

	/// 被選取車輛色 / Selected vehicle color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor SelectedColor = FLinearColor(1.0f, 0.85f, 0.0f, 1.0f);

	/// 目的地節點色 / Destination node highlight color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor DestinationColor = FLinearColor(1.0f, 0.2f, 0.2f, 1.0f);

	/// 跟隨攝影機距離（cm）/ Follow camera orbit distance
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera", meta = (ClampMin = "100"))
	float FollowCameraDistance = 600.0f;

	/// 攝影機平滑速度 / Camera interpolation speed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera")
	float CameraInterpSpeed = 5.0f;

	/// 預設軌道俯仰角（負=上方）/ Default orbit pitch (negative = above)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera")
	float DefaultOrbitPitch = -20.0f;

	/// 右鍵拖曳旋轉靈敏度 / Right-click drag orbit sensitivity
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera")
	float OrbitSensitivity = 0.3f;

	/// 滾輪縮放速度 / Scroll wheel zoom speed (cm per wheel click)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera", meta = (ClampMin = "10"))
	float ZoomSpeed = 80.0f;

	/// 最近距離 / Minimum zoom distance
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera", meta = (ClampMin = "50"))
	float MinFollowDistance = 200.0f;

	/// 最遠距離 / Maximum zoom distance
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera")
	float MaxFollowDistance = 3000.0f;

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

	// ---- 快取資料 / Cached Data ----

	TArray<FMapRoadCache> CachedRoads;
	TArray<FMapNodeCache> CachedNodes;
	TArray<FMapVehicleCache> CachedVehicles;

	bool bRoadCacheBuilt = false;

	/// 地圖世界邊界 / World bounds for projection
	FVector2D WorldBoundsMin = FVector2D::ZeroVector;
	FVector2D WorldBoundsMax = FVector2D(1.0, 1.0);

	/// 被選取的車 / Selected vehicle
	UPROPERTY()
	TWeakObjectPtr<AActor> SelectedVehiclePtr;

	/// 被選取車的目的地 Node / Destination node of selected vehicle
	int32 SelectedDestinationNodeId = INDEX_NONE;

	/// 跟隨攝影機 / Follow camera actor
	UPROPERTY()
	TObjectPtr<ACameraActor> FollowCamera;

	/// 選取前的原始 ViewTarget / Original view target before selection
	UPROPERTY()
	TWeakObjectPtr<AActor> OriginalViewTarget;

	// ---- 空拍擷取 / Scene Capture ----

	UPROPERTY()
	TObjectPtr<UTextureRenderTarget2D> MapRenderTarget;

	/// 用於 NativePaint 的 Brush（指向 RenderTarget）
	/// Brush for painting the render target in NativePaint
	mutable FSlateBrush MapTextureBrush;

	bool bCaptureReady = false;
	float CaptureTimer = 0.0f;

	void CaptureMapTexture();

	// ---- 快取建立 / Cache Building ----

	void BuildRoadCache();
	void UpdateVehicleCache();
	void CalculateWorldBounds();

	// ---- 座標轉換 / Coordinate Transform ----
	// 座標對應：攝影機 FRotator(-90,0,0) 俯瞰
	// World Y → 地圖水平，World X → 地圖垂直（X+ 在上方 = 前方朝上）
	// Mapping: Camera FRotator(-90,0,0) top-down
	// World Y → map horizontal, World X → map vertical (X+ = top = forward up)

	FVector2D WorldToMap(const FVector& WorldPos, const FVector2D& MapOrigin, const FVector2D& MapSize) const;
	FVector MapToWorld(const FVector2D& MapPos, const FVector2D& MapOrigin, const FVector2D& MapSize) const;
	void GetMapDrawRect(const FGeometry& Geometry, FVector2D& OutOrigin, FVector2D& OutSize) const;

	// ---- 繪製輔助 / Drawing Helpers ----

	void DrawMapBackground(FSlateWindowElementList& OutDrawElements, const FGeometry& Geometry,
		const FVector2D& MapOrigin, const FVector2D& MapSize, int32 LayerId) const;

	void DrawCircle(FPaintContext& Context, const FVector2D& Center, float Radius,
		const FLinearColor& Color, float Thickness = 1.5f, int32 NumSegments = 20) const;

	void DrawArrow(FPaintContext& Context, const FVector2D& Position, const FVector2D& Direction,
		float Size, const FLinearColor& Color, float Thickness = 2.0f) const;

	// ---- 點擊偵測 / Hit Detection ----

	AActor* FindVehicleNearMapPos(const FVector2D& LocalPos, const FVector2D& MapOrigin,
		const FVector2D& MapSize, float Radius) const;

	int32 FindNodeNearMapPos(const FVector2D& LocalPos, const FVector2D& MapOrigin,
		const FVector2D& MapSize, float Radius) const;

	// ---- 攝影機 / Camera ----

	void UpdateFollowCamera(float DeltaTime);
	void CreateFollowCamera();
	void DestroyFollowCamera();

	/// 平滑後的車輛位置（減少追蹤點模型微震動傳到攝影機）
	/// Smoothed vehicle position to dampen pursuit point micro-jitter
	FVector SmoothedFollowTarget = FVector::ZeroVector;
	bool bFollowTargetInitialized = false;

	/// 軌道偏航角（度，0=車後方）/ Orbit yaw offset (degrees, 0 = behind vehicle)
	float FollowOrbitYaw = 0.0f;

	/// 軌道俯仰角（度，負=上方）/ Orbit pitch (degrees, negative = above)
	float FollowOrbitPitch = -20.0f;
};
