#include "DrivingMapWidget.h"
#include "RoadNetworkSubsystem.h"
#include "RoadPathFollowerComponent.h"
#include "RoadTypes.h"

#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/SplineComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/SceneCapture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "EngineUtils.h"
#include "Blueprint/WidgetBlueprintLibrary.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"

// ================================================================
//  生命週期 / Lifecycle
// ================================================================

void UDrivingMapWidget::NativeConstruct()
{
	Super::NativeConstruct();

	// 預設小地圖模式：不攔截點擊
	SetVisibility(ESlateVisibility::HitTestInvisible);

	BuildRoadCache();
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

	// 道路快取尚未建好就重試
	if (!bRoadCacheBuilt)
	{
		BuildRoadCache();
	}

	// 空拍延遲計時 → 場景載入完再擷取
	// Scene capture delay — wait for scene to finish loading
	if (bRoadCacheBuilt && bUseSceneCapture && !bCaptureReady)
	{
		CaptureTimer += InDeltaTime;
		if (CaptureTimer >= CaptureDelaySeconds)
		{
			CaptureMapTexture();
		}
	}

	UpdateVehicleCache();

	// 軌道攝影機：右鍵拖曳旋轉 / Orbit camera: right-click drag to rotate
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
	int32 MaxLayer = Super::NativePaint(Args, AllottedGeometry, MyCullingRect,
		OutDrawElements, LayerId, InWidgetStyle, bParentEnabled);

	if (!bRoadCacheBuilt) return MaxLayer;

	FVector2D MapOrigin, MapSize;
	GetMapDrawRect(AllottedGeometry, MapOrigin, MapSize);
	if (MapSize.X < 1.0 || MapSize.Y < 1.0) return MaxLayer;

	// ---- 1. 背景：空拍照片或純色底 / Background: aerial photo or solid fill ----
	if (bUseSceneCapture && bCaptureReady && MapRenderTarget)
	{
		FSlateDrawElement::MakeBox(
			OutDrawElements, MaxLayer,
			AllottedGeometry.ToPaintGeometry(
				FVector2f(static_cast<float>(MapSize.X), static_cast<float>(MapSize.Y)),
				FSlateLayoutTransform(FVector2f(static_cast<float>(MapOrigin.X), static_cast<float>(MapOrigin.Y)))
			),
			&MapTextureBrush,
			ESlateDrawEffect::None,
			CaptureTint
		);
	}
	else
	{
		DrawMapBackground(OutDrawElements, AllottedGeometry, MapOrigin, MapSize, MaxLayer);
	}
	MaxLayer++;

	// ---- 2. PaintContext 用於線段/文字 ----
	FPaintContext Context(AllottedGeometry, MyCullingRect, OutDrawElements, MaxLayer,
		InWidgetStyle, bParentEnabled);

	// ---- 3. 道路線段（無空拍或手動開啟 overlay 時）/ Road lines ----
	if (!bUseSceneCapture || !bCaptureReady || bDrawRoadOverlay)
	{
		const float RoadThickness = bIsFullscreen ? 2.5f : 1.5f;
		FLinearColor DrawRoadColor = (bCaptureReady && bDrawRoadOverlay)
			? FLinearColor(RoadColor.R, RoadColor.G, RoadColor.B, 0.4f)
			: RoadColor;
		for (const FMapRoadCache& Road : CachedRoads)
		{
			for (int32 i = 0; i < Road.WorldPoints.Num() - 1; ++i)
			{
				FVector2D A = WorldToMap(Road.WorldPoints[i], MapOrigin, MapSize);
				FVector2D B = WorldToMap(Road.WorldPoints[i + 1], MapOrigin, MapSize);
				UWidgetBlueprintLibrary::DrawLine(Context, A, B, DrawRoadColor, true, RoadThickness);
			}
		}
	}

	// ---- 4. 節點 / Nodes ----
	const float NodeRadius = bIsFullscreen ? 6.0f : 3.5f;
	for (const FMapNodeCache& Node : CachedNodes)
	{
		FVector2D MapPos = WorldToMap(Node.WorldLocation, MapOrigin, MapSize);

		bool bIsDestination = (Node.NodeId == SelectedDestinationNodeId);
		FLinearColor Color = bIsDestination ? DestinationColor : NodeColor;
		float Radius = bIsDestination ? NodeRadius * 1.5f : NodeRadius;

		DrawCircle(Context, MapPos, Radius, Color, 2.0f);

		// 全螢幕才顯示 Node ID
		if (bIsFullscreen)
		{
			FString Label = FString::FromInt(Node.NodeId);
			FVector2D LabelPos(MapPos.X + Radius + 3.0, MapPos.Y - 6.0);
			UWidgetBlueprintLibrary::DrawText(Context, Label, LabelPos, NodeLabelColor);
		}
	}

	// ---- 5. 車輛 / Vehicles ----
	for (const FMapVehicleCache& Vehicle : CachedVehicles)
	{
		FVector2D MapPos = WorldToMap(Vehicle.WorldLocation, MapOrigin, MapSize);

		// 計算地圖上的前方向量
		FVector2D MapPosFwd = WorldToMap(
			Vehicle.WorldLocation + Vehicle.ForwardVector * 100.0f, MapOrigin, MapSize);
		FVector2D MapDir = MapPosFwd - MapPos;
		if (!MapDir.IsNearlyZero())
		{
			MapDir.Normalize();
		}
		else
		{
			MapDir = FVector2D(0.0, -1.0);
		}

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

	// ---- 6. 邊框 / Border ----
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

	// ---- 7. 資訊面板（全螢幕 + 已選車時）----
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

	// ---- 8. 操作提示 ----
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

	return Context.MaxLayer;
}

// ================================================================
//  滑鼠點擊 / Mouse Click
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
		// 先看有沒有點到車
		const float VehicleClickRadius = 20.0f;
		AActor* ClickedVehicle = FindVehicleNearMapPos(
			LocalPos, MapOrigin, MapSize, VehicleClickRadius);
		if (ClickedVehicle)
		{
			SelectVehicle(ClickedVehicle);
			return FReply::Handled();
		}

		// 已選車 → 檢查節點（設定目的地）
		if (SelectedVehiclePtr.IsValid())
		{
			const float NodeClickRadius = 18.0f;
			int32 ClickedNodeId = FindNodeNearMapPos(
				LocalPos, MapOrigin, MapSize, NodeClickRadius);
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

		// 點空白 → 取消選取
		DeselectVehicle();
		return FReply::Handled();
	}

	if (InMouseEvent.GetEffectingButton() == EKeys::RightMouseButton)
	{
		// 已選車時右鍵不關閉（用於軌道旋轉），未選車才關閉
		// With vehicle selected: don't close (orbit uses right-drag). Without: close.
		if (!SelectedVehiclePtr.IsValid())
		{
			ToggleMapSize();
		}
		return FReply::Handled();
	}

	return FReply::Unhandled();
}

// ================================================================
//  滾輪縮放 / Mouse Wheel Zoom
// ================================================================

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

	// 同步目的地節點
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

	// 建立跟隨攝影機
	CreateFollowCamera();

	if (APlayerController* PC = GetOwningPlayer())
	{
		if (!OriginalViewTarget.IsValid())
		{
			OriginalViewTarget = PC->GetViewTarget();
		}
		if (FollowCamera)
		{
			PC->SetViewTargetWithBlend(FollowCamera, 0.5f);
		}
	}
}

void UDrivingMapWidget::DeselectVehicle()
{
	SelectedVehiclePtr = nullptr;
	SelectedDestinationNodeId = INDEX_NONE;

	if (APlayerController* PC = GetOwningPlayer())
	{
		if (OriginalViewTarget.IsValid())
		{
			PC->SetViewTargetWithBlend(OriginalViewTarget.Get(), 0.5f);
		}
		else if (APawn* Pawn = PC->GetPawn())
		{
			PC->SetViewTargetWithBlend(Pawn, 0.5f);
		}
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

void UDrivingMapWidget::RefreshCapture()
{
	bCaptureReady = false;
	CaptureTimer = 0.0f;
}

// ================================================================
//  空拍擷取 / Scene Capture
// ================================================================

void UDrivingMapWidget::CaptureMapTexture()
{
	UWorld* World = GetWorld();
	if (!World) return;

	// 建立 RenderTarget / Create render target
	if (!MapRenderTarget)
	{
		MapRenderTarget = NewObject<UTextureRenderTarget2D>(this);
	}
	MapRenderTarget->InitAutoFormat(CaptureResolution, CaptureResolution);
	MapRenderTarget->UpdateResourceImmediate(true);

	// 計算擷取位置（世界中心上空）/ Camera position: center of world, high above
	FVector Center(
		(WorldBoundsMin.X + WorldBoundsMax.X) * 0.5,
		(WorldBoundsMin.Y + WorldBoundsMax.Y) * 0.5,
		CaptureHeight
	);
	FRotator LookDown(-90.0, 0.0, 0.0);

	// Spawn 臨時擷取 Actor / Spawn temporary capture actor
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	ASceneCapture2D* CaptureActor = World->SpawnActor<ASceneCapture2D>(
		Center, LookDown, SpawnParams);
	if (!CaptureActor) return;

	USceneCaptureComponent2D* CaptureComp = CaptureActor->GetCaptureComponent2D();
	CaptureComp->ProjectionType = ECameraProjectionMode::Orthographic;

	// OrthoWidth = 水平方向（World Y）的範圍
	// OrthoWidth covers the horizontal axis (World Y range)
	float OrthoSize = static_cast<float>(WorldBoundsMax.Y - WorldBoundsMin.Y);
	CaptureComp->OrthoWidth = OrthoSize;

	CaptureComp->TextureTarget = MapRenderTarget;
	CaptureComp->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	CaptureComp->bCaptureEveryFrame = false;
	CaptureComp->bCaptureOnMovement = false;

	// 隱藏跟隨攝影機（如果存在）
	if (FollowCamera)
	{
		CaptureComp->HiddenActors.Add(FollowCamera);
	}

	// 拍照！/ Capture!
	CaptureComp->CaptureScene();

	// 等 GPU 完成擷取，確保 RenderTarget 可用
	// Flush GPU so the render target is ready for Slate
	FlushRenderingCommands();

	// 用完即銷毀 / Destroy after capture
	CaptureActor->Destroy();

	// 設定 Slate Brush 指向 RenderTarget
	MapTextureBrush = FSlateBrush();
	MapTextureBrush.SetResourceObject(MapRenderTarget);
	MapTextureBrush.ImageSize = FVector2D(CaptureResolution, CaptureResolution);
	MapTextureBrush.DrawAs = ESlateBrushDrawType::Image;
	MapTextureBrush.Tiling = ESlateBrushTileType::NoTile;

	bCaptureReady = true;
	UE_LOG(LogTemp, Log, TEXT("[DrivingMap] Aerial capture complete (%dx%d, OrthoWidth=%.0f cm)"),
		CaptureResolution, CaptureResolution, OrthoSize);
}

// ================================================================
//  快取建立 / Cache Building
// ================================================================

void UDrivingMapWidget::BuildRoadCache()
{
	UWorld* World = GetWorld();
	if (!World) return;

	URoadNetworkSubsystem* RoadSub = World->GetSubsystem<URoadNetworkSubsystem>();
	if (!RoadSub) return;

	const TArray<FRoadGraphEdge>& GraphEdges = RoadSub->GetGraphEdges();
	const TArray<FRoadGraphNode>& GraphNodes = RoadSub->GetGraphNodes();
	if (GraphEdges.Num() == 0 && GraphNodes.Num() == 0) return;

	// 取樣道路 Spline
	CachedRoads.Reset();
	for (const FRoadGraphEdge& Edge : GraphEdges)
	{
		FMapRoadCache& Road = CachedRoads.AddDefaulted_GetRef();

		USplineComponent* Spline = Edge.InputSpline;
		if (Spline)
		{
			float Start = Edge.StartDistanceOnSpline;
			float End = Edge.EndDistanceOnSpline;
			float Dir = (End >= Start) ? 1.0f : -1.0f;
			float Length = FMath::Abs(End - Start);
			int32 NumSamples = FMath::Max(2, FMath::CeilToInt(Length / RoadSampleInterval));

			Road.WorldPoints.Reserve(NumSamples + 1);
			for (int32 i = 0; i <= NumSamples; ++i)
			{
				float Dist = Start + Dir * (Length * static_cast<float>(i) / NumSamples);
				FVector Pos = Spline->GetLocationAtDistanceAlongSpline(
					Dist, ESplineCoordinateSpace::World);
				Road.WorldPoints.Add(Pos);
			}
		}
		else
		{
			Road.WorldPoints.Add(Edge.StartWorldLocation);
			Road.WorldPoints.Add(Edge.EndWorldLocation);
		}
	}

	// 快取節點
	CachedNodes.Reset();
	for (const FRoadGraphNode& Node : GraphNodes)
	{
		FMapNodeCache& Cache = CachedNodes.AddDefaulted_GetRef();
		Cache.NodeId = Node.NodeId;
		Cache.WorldLocation = Node.WorldLocation;
	}

	CalculateWorldBounds();
	bRoadCacheBuilt = true;
}

void UDrivingMapWidget::CalculateWorldBounds()
{
	if (CachedNodes.Num() == 0 && CachedRoads.Num() == 0) return;

	FVector2D Min(TNumericLimits<double>::Max(), TNumericLimits<double>::Max());
	FVector2D Max(TNumericLimits<double>::Lowest(), TNumericLimits<double>::Lowest());

	for (const FMapNodeCache& Node : CachedNodes)
	{
		Min.X = FMath::Min(Min.X, Node.WorldLocation.X);
		Min.Y = FMath::Min(Min.Y, Node.WorldLocation.Y);
		Max.X = FMath::Max(Max.X, Node.WorldLocation.X);
		Max.Y = FMath::Max(Max.Y, Node.WorldLocation.Y);
	}

	for (const FMapRoadCache& Road : CachedRoads)
	{
		for (const FVector& Pt : Road.WorldPoints)
		{
			Min.X = FMath::Min(Min.X, Pt.X);
			Min.Y = FMath::Min(Min.Y, Pt.Y);
			Max.X = FMath::Max(Max.X, Pt.X);
			Max.Y = FMath::Max(Max.Y, Pt.Y);
		}
	}

	// 留白 / Add margin
	FVector2D Range = Max - Min;
	FVector2D Margin = Range * MapPaddingRatio;
	WorldBoundsMin = Min - Margin;
	WorldBoundsMax = Max + Margin;

	// 確保不退化
	if (WorldBoundsMax.X - WorldBoundsMin.X < 100.0) WorldBoundsMax.X = WorldBoundsMin.X + 100.0;
	if (WorldBoundsMax.Y - WorldBoundsMin.Y < 100.0) WorldBoundsMax.Y = WorldBoundsMin.Y + 100.0;

	// 空拍模式：將邊界擴成正方形以匹配正方形 RenderTarget
	// Scene capture mode: make bounds square to match square render target
	if (bUseSceneCapture)
	{
		double RangeX = WorldBoundsMax.X - WorldBoundsMin.X;
		double RangeY = WorldBoundsMax.Y - WorldBoundsMin.Y;
		double MaxRange = FMath::Max(RangeX, RangeY);

		double CenterX = (WorldBoundsMin.X + WorldBoundsMax.X) * 0.5;
		double CenterY = (WorldBoundsMin.Y + WorldBoundsMax.Y) * 0.5;

		WorldBoundsMin.X = CenterX - MaxRange * 0.5;
		WorldBoundsMax.X = CenterX + MaxRange * 0.5;
		WorldBoundsMin.Y = CenterY - MaxRange * 0.5;
		WorldBoundsMax.Y = CenterY + MaxRange * 0.5;
	}
}

void UDrivingMapWidget::UpdateVehicleCache()
{
	CachedVehicles.Reset();

	UWorld* World = GetWorld();
	if (!World) return;

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		URoadPathFollowerComponent* PathComp =
			Actor->FindComponentByClass<URoadPathFollowerComponent>();
		if (PathComp)
		{
			FMapVehicleCache& Cache = CachedVehicles.AddDefaulted_GetRef();
			Cache.Actor = Actor;
			Cache.PathFollower = PathComp;
			Cache.WorldLocation = Actor->GetActorLocation();
			Cache.ForwardVector = Actor->GetActorForwardVector();
			Cache.Speed = PathComp->GetCurrentSpeed();
		}
	}
}

// ================================================================
//  座標轉換 / Coordinate Transform
//
//  攝影機 FRotator(-90, 0, 0) 俯瞰時：
//    圖片水平 = World Y+（左→右 = minY→maxY）
//    圖片垂直 = World X-（上→下 = maxX→minX）
//
//  Camera FRotator(-90,0,0) top-down:
//    Image horizontal = World Y+ (left→right = minY→maxY)
//    Image vertical   = World X- (top→bottom = maxX→minX)
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

	// World Y → 水平（左=minY, 右=maxY）
	// World X → 垂直反向（上=maxX, 下=minX，前方朝上）
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

	// 保持世界比例 / Preserve world aspect ratio
	// 水平 = World Y range，垂直 = World X range
	double WorldWidth = WorldBoundsMax.Y - WorldBoundsMin.Y;   // 水平方向
	double WorldHeight = WorldBoundsMax.X - WorldBoundsMin.X;  // 垂直方向
	if (WorldWidth < 1.0) WorldWidth = 1.0;
	if (WorldHeight < 1.0) WorldHeight = 1.0;

	double WorldAspect = WorldWidth / WorldHeight;
	double MapAspect = OutSize.X / OutSize.Y;

	FVector2D AdjustedSize = OutSize;
	if (WorldAspect > MapAspect)
	{
		AdjustedSize.Y = OutSize.X / WorldAspect;
	}
	else
	{
		AdjustedSize.X = OutSize.Y * WorldAspect;
	}

	OutOrigin += (OutSize - AdjustedSize) * 0.5;
	OutSize = AdjustedSize;
}

// ================================================================
//  繪製輔助 / Drawing Helpers
// ================================================================

void UDrivingMapWidget::DrawMapBackground(
	FSlateWindowElementList& OutDrawElements,
	const FGeometry& Geometry,
	const FVector2D& MapOrigin,
	const FVector2D& MapSize,
	int32 LayerId) const
{
	// MakeGradient 完全不需要 Brush，兩端同色 = 實心填充
	// MakeGradient needs no brush at all — same color at both ends = solid fill
	TArray<FSlateGradientStop> Stops;
	Stops.Emplace(FVector2D::ZeroVector, BackgroundColor);
	Stops.Emplace(FVector2D(MapSize.X, 0.0), BackgroundColor);

	FSlateDrawElement::MakeGradient(
		OutDrawElements,
		LayerId,
		Geometry.ToPaintGeometry(
			FVector2f(static_cast<float>(MapSize.X), static_cast<float>(MapSize.Y)),
			FSlateLayoutTransform(FVector2f(static_cast<float>(MapOrigin.X), static_cast<float>(MapOrigin.Y)))
		),
		Stops,
		Orient_Horizontal,
		ESlateDrawEffect::None
	);
}

void UDrivingMapWidget::DrawCircle(
	FPaintContext& Context,
	const FVector2D& Center,
	float Radius,
	const FLinearColor& Color,
	float Thickness,
	int32 NumSegments) const
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
	FPaintContext& Context,
	const FVector2D& Position,
	const FVector2D& Direction,
	float Size,
	const FLinearColor& Color,
	float Thickness) const
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
	const FVector2D& LocalPos,
	const FVector2D& MapOrigin,
	const FVector2D& MapSize,
	float Radius) const
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
	const FVector2D& LocalPos,
	const FVector2D& MapOrigin,
	const FVector2D& MapSize,
	float Radius) const
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
		{
			CamComp->SetFieldOfView(75.0f);
		}
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

	// 第一層：平滑車輛位置（吸收追蹤點模型的微震動）
	// Layer 1: smooth the vehicle position (absorb pursuit point micro-jitter)
	if (!bFollowTargetInitialized)
	{
		SmoothedFollowTarget = VehiclePos;
		bFollowTargetInitialized = true;
	}
	else
	{
		SmoothedFollowTarget = FMath::VInterpTo(
			SmoothedFollowTarget, VehiclePos, DeltaTime, 8.0f);
	}

	// 第二層：球面軌道攝影機（右鍵拖曳旋轉、滾輪縮放）
	// Layer 2: spherical orbit camera (right-drag to rotate, scroll to zoom)
	//
	// 偏航 = 車頭方向 + 使用者偏移 + 180°（站在車後方）
	// Yaw = vehicle heading + user offset + 180° (behind vehicle)
	float VehicleYaw = VehicleFwd.Rotation().Yaw;
	float TotalYawRad = FMath::DegreesToRadians(VehicleYaw + FollowOrbitYaw + 180.0f);
	float PitchRad = FMath::DegreesToRadians(FollowOrbitPitch);

	// 球面座標 → 相對偏移 / Spherical to offset
	FVector Offset;
	Offset.X = FMath::Cos(TotalYawRad) * FMath::Cos(PitchRad) * FollowCameraDistance;
	Offset.Y = FMath::Sin(TotalYawRad) * FMath::Cos(PitchRad) * FollowCameraDistance;
	Offset.Z = -FMath::Sin(PitchRad) * FollowCameraDistance; // 負俯仰 = 上方

	FVector TargetPos = SmoothedFollowTarget + Offset;
	FRotator TargetRot = (SmoothedFollowTarget - TargetPos).Rotation();

	// 平滑攝影機移動 / Smooth camera movement
	FVector NewPos = FMath::VInterpTo(
		FollowCamera->GetActorLocation(), TargetPos, DeltaTime, CameraInterpSpeed);
	FRotator NewRot = FMath::RInterpTo(
		FollowCamera->GetActorRotation(), TargetRot, DeltaTime, CameraInterpSpeed);

	FollowCamera->SetActorLocation(NewPos);
	FollowCamera->SetActorRotation(NewRot);
}
