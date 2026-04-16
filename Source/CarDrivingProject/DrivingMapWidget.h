#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Styling/SlateBrush.h"
#include "DrivingMapWidget.generated.h"

class ACameraActor;
class AParkingLotActor;
class URoadNetworkSubsystem;
class URoadPathFollowerComponent;
class USpringArmComponent;
class UTexture2D;

/// Cached vehicle info for map rendering
struct FMapVehicleCache
{
	TWeakObjectPtr<AActor> Actor;
	TWeakObjectPtr<URoadPathFollowerComponent> PathFollower;
	FVector WorldLocation = FVector::ZeroVector;
	FVector ForwardVector = FVector::ForwardVector;
	float Speed = 0.0f;
};

/// background texture + vehicle arrows + click-to-follow/navigate
UCLASS()
class CARDRIVINGPROJECT_API UDrivingMapWidget : public UUserWidget
{
	GENERATED_BODY()

public:

	// ================================================================
	//  Map Mode
	// ================================================================

	UPROPERTY(BlueprintReadWrite, Category = "Driving Map")
	bool bIsFullscreen = false;

	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void ToggleMapSize();

	// ================================================================
	//  Vehicle Selection & Camera
	// ================================================================

	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void SelectVehicle(AActor* Vehicle);

	UFUNCTION(BlueprintCallable, Category = "Driving Map")
	void DeselectVehicle();

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Driving Map")
	AActor* GetSelectedVehicle() const;

	// ================================================================
	//  Map Background
	// ================================================================

	/// Background image — import your screenshot as Texture2D, set in BP
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Background")
	TObjectPtr<UTexture2D> MapTexture;

	/// World coordinate of the bottom-left corner of your map image
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Background")
	FVector2D MapTextureWorldMin = FVector2D(-50000.0, -50000.0);

	/// World coordinate of the top-right corner of your map image
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Background")
	FVector2D MapTextureWorldMax = FVector2D(50000.0, 50000.0);

	// ================================================================
	// Visual Settings
	// ================================================================

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Minimap")
	FVector2D MinimapSize = FVector2D(350.0, 350.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Minimap")
	FVector2D MinimapMargin = FVector2D(20.0, 20.0);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Fullscreen")
	float FullscreenCoverage = 0.85f;

	/// Fallback background color
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

	/// Parking lot marker color on map
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor ParkingLotColor = FLinearColor(0.2f, 0.9f, 0.9f, 1.0f);

	/// Parking lot name font size
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Parking Text", meta = (ClampMin = "6", ClampMax = "30"))
	int32 ParkingLotFontSize = 12;

	/// Parking lot name text color
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Parking Text")
	FLinearColor ParkingLotTextColor = FLinearColor(0.2f, 0.9f, 0.9f, 1.0f);

	/// Show parking names on map?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Parking Text")
	bool bShowParkingNames = true;

	/// Parking lot name text offset from marker (px)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Parking Text")
	FVector2D ParkingLotNameOffset = FVector2D(8.0, -6.0);


	// ---- Navigation Route ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor RouteColor = FLinearColor(1.0f, 0.6f, 0.1f, 0.85f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Route")
	float RouteThicknessFullscreen = 3.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Route")
	float RouteThicknessMinimap = 2.0f;

	// ---- Road Edge ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor EdgeColor = FLinearColor(0.3f, 0.6f, 1.0f, 0.6f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Route")
	float EdgeThicknessFullscreen = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Route")
	float EdgeThicknessMinimap = 1.5f;

	// ---- Parking Lot Circle Outline ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor ParkingLotOutlineColor = FLinearColor(1.0f, 1.0f, 1.0f, 0.8f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Parking Text")
	float ParkingLotOutlineThickness = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Parking Text")
	float ParkingLotCircleRadius = 6.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Colors")
	FLinearColor ParkingLotHoverColor = FLinearColor(1.0f, 1.0f, 0.2f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Parking Text")
	int32 ParkingLotHoverFontSize = 20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Parking Text")
	FLinearColor ParkingLotHoverTextColor = FLinearColor(1.0f, 1.0f, 0.2f, 1.0f);


	// ================================================================
	//  Right Info Panel
	// ================================================================

	/// Info panel font size multiplier
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Info Panel", meta = (ClampMin = "0.5", ClampMax = "4.0"))
	float InfoPanelFontScale = 1.5f;

	/// Info panel background opacity
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Info Panel", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float InfoPanelBackgroundAlpha = 0.6f;

	/// Map background extra margin multiplier (larger = more margin, route stays inside)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Minimap", meta = (ClampMin = "0.1", ClampMax = "1.0"))
	float MapBoundsMarginRatio = 0.25f;

	// ================================================================
	//  Camera Settings
	// ================================================================

	/// Right-click drag orbit sensitivity
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera")
	float OrbitSensitivity = 1.0f;

	/// Scroll zoom speed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera", meta = (ClampMin = "10"))
	float ZoomSpeed = 80.0f;

	/// Min arm length
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera", meta = (ClampMin = "50"))
	float MinFollowDistance = 200.0f;

	/// Max arm length
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Camera")
	float MaxFollowDistance = 3000.0f;

	// ================================================================
	//  Free Camera
	// ================================================================

	/// Free camera movement speed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Free Camera")
	float FreeCameraMoveSpeed = 1500.0f;

	/// Free camera rotation speed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Free Camera")
	float FreeCameraRotateSpeed = 2.5f;

	/// Speed multiplier when holding Shift
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driving Map|Free Camera")
	float FreeCameraFastMultiplier = 3.0f;

	/// Whether currently in free camera mode
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

	virtual FReply NativeOnMouseMove(const FGeometry& InGeometry,
		const FPointerEvent& InMouseEvent) override;

private:



	// ---- Cache ----

// Hover 
	TWeakObjectPtr<AParkingLotActor> HoveredParkingLot;

	TArray<FMapVehicleCache> CachedVehicles;
	bool bCacheBuilt = false;

	FVector2D WorldBoundsMin = FVector2D::ZeroVector;
	FVector2D WorldBoundsMax = FVector2D(1.0, 1.0);

	UPROPERTY()
	TWeakObjectPtr<AActor> SelectedVehiclePtr;

	/// Destination actor (parking lot)
	UPROPERTY()
	TWeakObjectPtr<AActor> SelectedDestinationActor;

	/// Destination world position for marker
	FVector SelectedDestinationWorldPos = FVector::ZeroVector;

	/// Persistent free camera actor — spawned once at startup
	UPROPERTY()
	TObjectPtr<ACameraActor> FreeCameraActor;

	// ---- Brush ----

	mutable FSlateBrush MapTextureBrush;
	bool bBrushReady = false;

	void SetupBrush();


	void BuildNodeCache();
	void UpdateVehicleCache();

	FVector2D WorldToMap(const FVector& WorldPos, const FVector2D& MapOrigin, const FVector2D& MapSize) const;
	FVector MapToWorld(const FVector2D& MapPos, const FVector2D& MapOrigin, const FVector2D& MapSize) const;
	void GetMapDrawRect(const FGeometry& Geometry, FVector2D& OutOrigin, FVector2D& OutSize) const;

	void DrawBackground(FSlateWindowElementList& OutDrawElements, const FGeometry& Geometry,
		const FVector2D& MapOrigin, const FVector2D& MapSize, int32 LayerId) const;

	void DrawArrow(FPaintContext& Context, const FVector2D& Position, const FVector2D& Direction,
		float Size, const FLinearColor& Color, float Thickness = 2.0f) const;

	void DrawCircle(FPaintContext& Context, const FVector2D& Center, float Radius,
		const FLinearColor& Color, float Thickness = 1.5f, int32 NumSegments = 20) const;

	AActor* FindVehicleNearMapPos(const FVector2D& LocalPos, const FVector2D& MapOrigin,
		const FVector2D& MapSize, float Radius) const;

	/// Find parking lot near click
	AParkingLotActor* FindParkingLotNearMapPos(const FVector2D& LocalPos, const FVector2D& MapOrigin,
		const FVector2D& MapSize, float Radius) const;

	/// Store original boom rotation before orbit, restore on deselect
	FRotator OriginalBoomRotation = FRotator::ZeroRotator;

	void InitFreeCamera();
	void UpdateFreeCamera(float DeltaTime);
};
