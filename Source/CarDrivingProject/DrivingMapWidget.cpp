#include "DrivingMapWidget.h"
#include "RoadNetworkSubsystem.h"
#include "RoadPathFollowerComponent.h"
#include "RoadTypes.h"

#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Engine/Texture2D.h"
#include "EngineUtils.h"
#include "Blueprint/WidgetBlueprintLibrary.h"
#include "Rendering/DrawElements.h"

// ================================================================
//  生命週期 / Lifecycle
// ================================================================

void UDrivingMapWidget::NativeConstruct()
{
	Super::NativeConstruct();
	SetVisibility(ESlateVisibility::HitTestInvisible);
	BuildNodeCache();
}

void UDrivingMapWidget::NativeDestruct()
{
	DestroyFollowCamera();
	Super::NativeDestruct();
}

// ================================================================
//  Tick
// ================================================================

void UDrivingMapWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);

	if (!bCacheBuilt)
	{
		BuildNodeCache();
	}

	// 背景圖 Brush 設定（一次）
	if (!bBrushReady && MapTexture)
	{
		SetupBrush();
	}

	UpdateVehicleCache();

	// 軌道攝影機旋轉
	if (SelectedVehiclePtr.IsValid())
	{
		if (APlayerController* PC = GetOwningPlayer())
		{
			if (PC->IsInputKeyDown(EKeys::RightMouseButton) && !bIsFullscreen)
			{
				float DeltaX, DeltaY;
				PC->GetInputMouseDelta(DeltaX, DeltaY);
				FollowOrbitYaw += DeltaX * OrbitSensitivity;
				FollowOrbitPitch = FMath::Clamp(
					FollowOrbitPitch + DeltaY * OrbitSensitivity, -80.0f, -5.0f);
			}
		}
	}

	UpdateFollowCamera(InDeltaTime);
}

// ================================================================
//  繪製 / Paint
// ================================================================

int32 UDrivingMapWidget::NativePaint(
	const FPaintArgs& Args,
	const FGeometry& AllottedGeometry,
	const FSlateRect& MyCullingRect,
	FSlateWindowElementList& OutDrawElements,
	int32 LayerId,
	const FWidgetStyle& InWidgetStyle,
	bool bParentEnabled) const
{
	LayerId = Super::NativePaint(Args, AllottedGeometry, MyCullingRect,
		OutDrawElements, LayerId, InWidgetStyle, bParentEnabled);

	FVector2D MapOrigin, MapSize;
	GetMapDrawRect(AllottedGeometry, MapOrigin, MapSize);
	if (MapSize.X < 1.0 || MapSize.Y < 1.0) return LayerId;

	// ---- 1. 背景 ----
	DrawBackground(OutDrawElements, AllottedGeometry, MapOrigin, MapSize, LayerId);

	// ---- 2. 車輛箭頭 ----
	FPaintContext Context(AllottedGeometry, MyCullingRect, OutDrawElements, LayerId + 1,
		InWidgetStyle, bParentEnabled);

	for (const FMapVehicleCache& Vehicle : CachedVehicles)
	{
		FVector2D MapPos = WorldToMap(Vehicle.WorldLocation, MapOrigin, MapSize);

		FVector2D MapPosFwd = WorldToMap(
			Vehicle.WorldLocation + Vehicle.ForwardVector * 100.0f, MapOrigin, MapSize);
		FVector2D MapDir = MapPosFwd - MapPos;
		if (!MapDir.IsNearlyZero()) MapDir.Normalize();
		else MapDir = FVector2D(0.0, -1.0);

		bool bSelected = Vehicle.Actor.IsValid() && (Vehicle.Actor == SelectedVehiclePtr);
		FLinearColor Color = bSelected ? SelectedColor : VehicleColor;
		float ArrowSize = bIsFullscreen ? 10.0f : 7.0f;

		if (bSelected)
		{
			ArrowSize *= 1.4f;
			DrawCircle(Context, MapPos, ArrowSize * 2.2f, SelectedColor, 2.0f);
		}

		DrawArrow(Context, MapPos, MapDir, ArrowSize, Color, 2.5f);
	}

	// ---- 3. 邊框 ----
	{
		FVector2D TL = MapOrigin;
		FVector2D TR(MapOrigin.X + MapSize.X, MapOrigin.Y);
		FVector2D BR(MapOrigin.X + MapSize.X, MapOrigin.Y + MapSize.Y);
		FVector2D BL(MapOrigin.X, MapOrigin.Y + MapSize.Y);
		const float BorderThick = bIsFullscreen ? 2.5f : 1.5f;
		UWidgetBlueprintLibrary::DrawLine(Context, TL, TR, BorderColor, true, BorderThick);
		UWidgetBlueprintLibrary::DrawLine(Context, TR, BR, BorderColor, true, BorderThick);
		UWidgetBlueprintLibrary::DrawLine(Context, BR, BL, BorderColor, true, BorderThick);
		UWidgetBlueprintLibrary::DrawLine(Context, BL, TL, BorderColor, true, BorderThick);
	}

	// ---- 4. 提示文字 ----
	if (!bIsFullscreen)
	{
		FVector2D HintPos(MapOrigin.X, MapOrigin.Y - 16.0);
		UWidgetBlueprintLibrary::DrawText(Context, TEXT("[M] Map"), HintPos,
			FLinearColor(0.5f, 0.5f, 0.6f, 0.7f));
	}
	else
	{
		FVector2D HintPos(MapOrigin.X, MapOrigin.Y - 18.0);
		FString Hint = SelectedVehiclePtr.IsValid()
			? TEXT("[LMB] Click node = set destination  |  [RMB/Esc] Close")
			: TEXT("[LMB] Click vehicle = follow  |  [RMB/Esc] Close");
		UWidgetBlueprintLibrary::DrawText(Context, Hint, HintPos,
			FLinearColor(0.6f, 0.65f, 0.7f, 0.8f));
	}

	// ---- 5. 資訊面板 ----
	if (bIsFullscreen && SelectedVehiclePtr.IsValid())
	{
		URoadPathFollowerComponent* PF =
			SelectedVehiclePtr->FindComponentByClass<URoadPathFollowerComponent>();
		if (PF)
		{
			float SpeedKmh = PF->GetCurrentSpeed() * 0.036f;
			FString Info = FString::Printf(TEXT("Speed: %.1f km/h  |  State: %s"),
				SpeedKmh,
				*UEnum::GetValueAsString(PF->GetNavState()));
			FVector2D InfoPos(MapOrigin.X + 10.0, MapOrigin.Y + MapSize.Y + 8.0);
			UWidgetBlueprintLibrary::DrawText(Context, Info, InfoPos,
				FLinearColor(0.8f, 0.9f, 1.0f, 1.0f));
		}
	}

	return Context.MaxLayer;
}

// ================================================================
//  滑鼠 / Mouse
// ================================================================

FReply UDrivingMapWidget::NativeOnMouseButtonDown(
	const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	if (!bIsFullscreen) return FReply::Unhandled();

	FVector2D LocalPos = InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition());
	FVector2D MapOrigin, MapSize;
	GetMapDrawRect(InGeometry, MapOrigin, MapSize);

	bool bInMap = (LocalPos.X >= MapOrigin.X && LocalPos.X <= MapOrigin.X + MapSize.X
		&& LocalPos.Y >= MapOrigin.Y && LocalPos.Y <= MapOrigin.Y + MapSize.Y);
	if (!bInMap) return FReply::Unhandled();

	if (InMouseEvent.GetEffectingButton() == EKeys::LeftMouseButton)
	{
		// 點車
		AActor* ClickedVehicle = FindVehicleNearMapPos(LocalPos, MapOrigin, MapSize, 20.0f);
		if (ClickedVehicle)
		{
			SelectVehicle(ClickedVehicle);
			return FReply::Handled();
		}

		// 已選車 → 點節點設目的地
		if (SelectedVehiclePtr.IsValid())
		{
			int32 ClickedNodeId = FindNodeNearMapPos(LocalPos, MapOrigin, MapSize, 18.0f);
			if (ClickedNodeId != INDEX_NONE)
			{
				URoadPathFollowerComponent* PF =
					SelectedVehiclePtr->FindComponentByClass<URoadPathFollowerComponent>();
				if (PF)
				{
					PF->NavigateToNode(ClickedNodeId);
					SelectedDestinationNodeId = ClickedNodeId;
				}
				return FReply::Handled();
			}
		}

		DeselectVehicle();
		return FReply::Handled();
	}

	if (InMouseEvent.GetEffectingButton() == EKeys::RightMouseButton)
	{
		if (!SelectedVehiclePtr.IsValid())
		{
			ToggleMapSize();
		}
		return FReply::Handled();
	}

	return FReply::Unhandled();
}

FReply UDrivingMapWidget::NativeOnMouseWheel(
	const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	if (SelectedVehiclePtr.IsValid())
	{
		float Delta = InMouseEvent.GetWheelDelta();
		FollowCameraDistance = FMath::Clamp(
			FollowCameraDistance - Delta * ZoomSpeed,
			MinFollowDistance, MaxFollowDistance);
		return FReply::Handled();
	}
	return FReply::Unhandled();
}

// ================================================================
//  公開 API / Public API
// ================================================================

void UDrivingMapWidget::ToggleMapSize()
{
	bIsFullscreen = !bIsFullscreen;
	SetVisibility(bIsFullscreen ? ESlateVisibility::Visible : ESlateVisibility::HitTestInvisible);
}

void UDrivingMapWidget::SelectVehicle(AActor* Vehicle)
{
	if (!Vehicle) { DeselectVehicle(); return; }

	SelectedVehiclePtr = Vehicle;
	FollowOrbitYaw = 0.0f;
	FollowOrbitPitch = DefaultOrbitPitch;

	URoadPathFollowerComponent* PF = Vehicle->FindComponentByClass<URoadPathFollowerComponent>();
	if (PF)
	{
		FVector DestLoc = PF->GetDestinationLocation();
		if (!DestLoc.IsNearlyZero())
		{
			if (UWorld* World = GetWorld())
			{
				if (URoadNetworkSubsystem* RS = World->GetSubsystem<URoadNetworkSubsystem>())
				{
					SelectedDestinationNodeId = RS->FindNearestGraphNode(DestLoc);
				}
			}
		}
	}

	CreateFollowCamera();
	if (APlayerController* PC = GetOwningPlayer())
	{
		if (!OriginalViewTarget.IsValid())
			OriginalViewTarget = PC->GetViewTarget();
		if (FollowCamera)
			PC->SetViewTargetWithBlend(FollowCamera, 0.5f);
	}
}

void UDrivingMapWidget::DeselectVehicle()
{
	SelectedVehiclePtr = nullptr;
	SelectedDestinationNodeId = INDEX_NONE;

	if (APlayerController* PC = GetOwningPlayer())
	{
		if (OriginalViewTarget.IsValid())
			PC->SetViewTargetWithBlend(OriginalViewTarget.Get(), 0.5f);
		else if (APawn* Pawn = PC->GetPawn())
			PC->SetViewTargetWithBlend(Pawn, 0.5f);
	}

	OriginalViewTarget = nullptr;
	bFollowTargetInitialized = false;
	FollowOrbitYaw = 0.0f;
	FollowOrbitPitch = DefaultOrbitPitch;
	DestroyFollowCamera();
}

AActor* UDrivingMapWidget::GetSelectedVehicle() const
{
	return SelectedVehiclePtr.Get();
}

// ================================================================
//  背景 Brush
// ================================================================

void UDrivingMapWidget::SetupBrush()
{
	if (!MapTexture) return;

	MapTextureBrush.SetResourceObject(MapTexture);
	MapTextureBrush.ImageSize = FVector2D(MapTexture->GetSizeX(), MapTexture->GetSizeY());
	MapTextureBrush.DrawAs = ESlateBrushDrawType::Image;
	MapTextureBrush.Tiling = ESlateBrushTileType::NoTile;
	MapTextureBrush.ImageType = ESlateBrushImageType::FullColor;

	bBrushReady = true;
}

// ================================================================
//  快取 / Cache
// ================================================================

void UDrivingMapWidget::BuildNodeCache()
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* RoadSub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSub) return;

	const TArray<FRoadGraphNode>& GraphNodes = RoadSub->GetGraphNodes();
	if (GraphNodes.Num() == 0) return;

	CachedNodes.Reset();
	for (const FRoadGraphNode& Node : GraphNodes)
	{
		FMapNodeCache& Cache = CachedNodes.AddDefaulted_GetRef();
		Cache.NodeId = Node.NodeId;
		Cache.WorldLocation = Node.WorldLocation;
	}

	// 用節點算世界邊界（永遠算，確保車輛箭頭位置正確）
	{
		FVector2D Min(TNumericLimits<double>::Max(), TNumericLimits<double>::Max());
		FVector2D Max(TNumericLimits<double>::Lowest(), TNumericLimits<double>::Lowest());
		for (const FMapNodeCache& Node : CachedNodes)
		{
			Min.X = FMath::Min(Min.X, Node.WorldLocation.X);
			Min.Y = FMath::Min(Min.Y, Node.WorldLocation.Y);
			Max.X = FMath::Max(Max.X, Node.WorldLocation.X);
			Max.Y = FMath::Max(Max.Y, Node.WorldLocation.Y);
		}
		FVector2D Range = Max - Min;
		FVector2D Margin = Range * 0.1;
		WorldBoundsMin = Min - Margin;
		WorldBoundsMax = Max + Margin;
		if (WorldBoundsMax.X - WorldBoundsMin.X < 100.0) WorldBoundsMax.X = WorldBoundsMin.X + 100.0;
		if (WorldBoundsMax.Y - WorldBoundsMin.Y < 100.0) WorldBoundsMax.Y = WorldBoundsMin.Y + 100.0;
	}

	bCacheBuilt = true;
}

void UDrivingMapWidget::UpdateVehicleCache()
{
	CachedVehicles.Reset();
	UWorld* World = GetWorld();
	if (!World) return;

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		URoadPathFollowerComponent* PathComp =
			(*It)->FindComponentByClass<URoadPathFollowerComponent>();
		if (PathComp)
		{
			FMapVehicleCache& Cache = CachedVehicles.AddDefaulted_GetRef();
			Cache.Actor = *It;
			Cache.PathFollower = PathComp;
			Cache.WorldLocation = (*It)->GetActorLocation();
			Cache.ForwardVector = (*It)->GetActorForwardVector();
			Cache.Speed = PathComp->GetCurrentSpeed();
		}
	}
}

// ================================================================
//  座標轉換 / Coordinate Transform
// ================================================================

FVector2D UDrivingMapWidget::WorldToMap(
	const FVector& WorldPos,
	const FVector2D& MapOrigin,
	const FVector2D& MapSize) const
{
	double RangeX = WorldBoundsMax.X - WorldBoundsMin.X;
	double RangeY = WorldBoundsMax.Y - WorldBoundsMin.Y;
	if (RangeX < 1.0) RangeX = 1.0;
	if (RangeY < 1.0) RangeY = 1.0;

	double NormX = (WorldPos.X - WorldBoundsMin.X) / RangeX;
	double NormY = (WorldPos.Y - WorldBoundsMin.Y) / RangeY;

	return FVector2D(
		MapOrigin.X + NormY * MapSize.X,
		MapOrigin.Y + (1.0 - NormX) * MapSize.Y
	);
}

FVector UDrivingMapWidget::MapToWorld(
	const FVector2D& MapPos,
	const FVector2D& MapOrigin,
	const FVector2D& MapSize) const
{
	double RangeX = WorldBoundsMax.X - WorldBoundsMin.X;
	double RangeY = WorldBoundsMax.Y - WorldBoundsMin.Y;

	double NormY = (MapPos.X - MapOrigin.X) / MapSize.X;
	double NormX = 1.0 - (MapPos.Y - MapOrigin.Y) / MapSize.Y;

	return FVector(
		WorldBoundsMin.X + NormX * RangeX,
		WorldBoundsMin.Y + NormY * RangeY,
		0.0
	);
}

void UDrivingMapWidget::GetMapDrawRect(
	const FGeometry& Geometry,
	FVector2D& OutOrigin,
	FVector2D& OutSize) const
{
	FVector2D WidgetSize = Geometry.GetLocalSize();

	if (bIsFullscreen)
	{
		FVector2D FullSize = WidgetSize * FullscreenCoverage;
		OutOrigin = (WidgetSize - FullSize) * 0.5;
		OutSize = FullSize;
	}
	else
	{
		OutSize = MinimapSize;
		OutOrigin = WidgetSize - MinimapSize - MinimapMargin;
	}

	// 保持世界比例
	double WorldWidth = WorldBoundsMax.Y - WorldBoundsMin.Y;
	double WorldHeight = WorldBoundsMax.X - WorldBoundsMin.X;
	if (WorldWidth < 1.0) WorldWidth = 1.0;
	if (WorldHeight < 1.0) WorldHeight = 1.0;

	double WorldAspect = WorldWidth / WorldHeight;
	double MapAspect = OutSize.X / OutSize.Y;

	FVector2D AdjustedSize = OutSize;
	if (WorldAspect > MapAspect)
		AdjustedSize.Y = OutSize.X / WorldAspect;
	else
		AdjustedSize.X = OutSize.Y * WorldAspect;

	OutOrigin += (OutSize - AdjustedSize) * 0.5;
	OutSize = AdjustedSize;
}

// ================================================================
//  繪製 / Drawing
// ================================================================

void UDrivingMapWidget::DrawBackground(
	FSlateWindowElementList& OutDrawElements,
	const FGeometry& Geometry,
	const FVector2D& MapOrigin,
	const FVector2D& MapSize,
	int32 LayerId) const
{
	FPaintGeometry PaintGeo = Geometry.ToPaintGeometry(
		FVector2f(static_cast<float>(MapSize.X), static_cast<float>(MapSize.Y)),
		FSlateLayoutTransform(FVector2f(static_cast<float>(MapOrigin.X), static_cast<float>(MapOrigin.Y)))
	);

	// 有背景圖就直接畫圖
	if (bBrushReady && MapTextureBrush.GetResourceObject())
	{
		FSlateDrawElement::MakeBox(
			OutDrawElements, LayerId, PaintGeo,
			&MapTextureBrush, ESlateDrawEffect::None, FLinearColor::White);
	}
	else
	{
		// 純色底
		TArray<FSlateGradientStop> Stops;
		Stops.Emplace(FVector2D::ZeroVector, BackgroundColor);
		Stops.Emplace(FVector2D(MapSize.X, 0.0), BackgroundColor);
		FSlateDrawElement::MakeGradient(OutDrawElements, LayerId, PaintGeo,
			Stops, Orient_Horizontal, ESlateDrawEffect::None);
	}
}

void UDrivingMapWidget::DrawCircle(
	FPaintContext& Context, const FVector2D& Center, float Radius,
	const FLinearColor& Color, float Thickness, int32 NumSegments) const
{
	FVector2D Prev(Center.X + Radius, Center.Y);
	for (int32 i = 1; i <= NumSegments; ++i)
	{
		float Angle = 2.0f * PI * static_cast<float>(i) / NumSegments;
		FVector2D Next(Center.X + FMath::Cos(Angle) * Radius,
			Center.Y + FMath::Sin(Angle) * Radius);
		UWidgetBlueprintLibrary::DrawLine(Context, Prev, Next, Color, true, Thickness);
		Prev = Next;
	}
}

void UDrivingMapWidget::DrawArrow(
	FPaintContext& Context, const FVector2D& Position, const FVector2D& Direction,
	float Size, const FLinearColor& Color, float Thickness) const
{
	FVector2D Right(-Direction.Y, Direction.X);
	FVector2D Tip = Position + Direction * Size * 2.0;
	FVector2D BackLeft = Position - Direction * Size + Right * Size;
	FVector2D BackRight = Position - Direction * Size - Right * Size;

	UWidgetBlueprintLibrary::DrawLine(Context, Tip, BackLeft, Color, true, Thickness);
	UWidgetBlueprintLibrary::DrawLine(Context, BackLeft, BackRight, Color, true, Thickness);
	UWidgetBlueprintLibrary::DrawLine(Context, BackRight, Tip, Color, true, Thickness);
}

// ================================================================
//  點擊偵測 / Hit Detection
// ================================================================

AActor* UDrivingMapWidget::FindVehicleNearMapPos(
	const FVector2D& LocalPos, const FVector2D& MapOrigin,
	const FVector2D& MapSize, float Radius) const
{
	float BestDistSq = Radius * Radius;
	AActor* BestActor = nullptr;

	for (const FMapVehicleCache& Vehicle : CachedVehicles)
	{
		if (!Vehicle.Actor.IsValid()) continue;
		FVector2D MapPos = WorldToMap(Vehicle.WorldLocation, MapOrigin, MapSize);
		float DistSq = static_cast<float>(FVector2D::DistSquared(LocalPos, MapPos));
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestActor = Vehicle.Actor.Get();
		}
	}
	return BestActor;
}

int32 UDrivingMapWidget::FindNodeNearMapPos(
	const FVector2D& LocalPos, const FVector2D& MapOrigin,
	const FVector2D& MapSize, float Radius) const
{
	float BestDistSq = Radius * Radius;
	int32 BestNodeId = INDEX_NONE;

	for (const FMapNodeCache& Node : CachedNodes)
	{
		FVector2D MapPos = WorldToMap(Node.WorldLocation, MapOrigin, MapSize);
		float DistSq = static_cast<float>(FVector2D::DistSquared(LocalPos, MapPos));
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestNodeId = Node.NodeId;
		}
	}
	return BestNodeId;
}

// ================================================================
//  跟隨攝影機 / Follow Camera
// ================================================================

void UDrivingMapWidget::CreateFollowCamera()
{
	if (FollowCamera) return;
	UWorld* World = GetWorld();
	if (!World) return;

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	FollowCamera = World->SpawnActor<ACameraActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
	if (FollowCamera)
	{
		if (UCameraComponent* CamComp = FollowCamera->GetCameraComponent())
			CamComp->SetFieldOfView(75.0f);
	}
}

void UDrivingMapWidget::DestroyFollowCamera()
{
	if (FollowCamera)
	{
		FollowCamera->Destroy();
		FollowCamera = nullptr;
	}
}

void UDrivingMapWidget::UpdateFollowCamera(float DeltaTime)
{
	if (!FollowCamera || !SelectedVehiclePtr.IsValid()) return;

	FVector VehiclePos = SelectedVehiclePtr->GetActorLocation();
	FVector VehicleFwd = SelectedVehiclePtr->GetActorForwardVector();

	if (!bFollowTargetInitialized)
	{
		SmoothedFollowTarget = VehiclePos;
		bFollowTargetInitialized = true;
	}
	else
	{
		SmoothedFollowTarget = FMath::VInterpTo(
			SmoothedFollowTarget, VehiclePos, DeltaTime, 3.0f);
	}

	float VehicleYaw = VehicleFwd.Rotation().Yaw;
	float TotalYawRad = FMath::DegreesToRadians(VehicleYaw + FollowOrbitYaw + 180.0f);
	float PitchRad = FMath::DegreesToRadians(FollowOrbitPitch);

	FVector Offset;
	Offset.X = FMath::Cos(TotalYawRad) * FMath::Cos(PitchRad) * FollowCameraDistance;
	Offset.Y = FMath::Sin(TotalYawRad) * FMath::Cos(PitchRad) * FollowCameraDistance;
	Offset.Z = -FMath::Sin(PitchRad) * FollowCameraDistance;

	FVector TargetPos = SmoothedFollowTarget + Offset;
	FRotator TargetRot = (SmoothedFollowTarget - TargetPos).Rotation();

	FVector NewPos = FMath::VInterpTo(
		FollowCamera->GetActorLocation(), TargetPos, DeltaTime, CameraInterpSpeed);
	FRotator NewRot = FMath::RInterpTo(
		FollowCamera->GetActorRotation(), TargetRot, DeltaTime, CameraInterpSpeed);

	FollowCamera->SetActorLocation(NewPos);
	FollowCamera->SetActorRotation(NewRot);
}
