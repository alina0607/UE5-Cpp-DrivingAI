#include "DrivingMapWidget.h"
#include "RoadNetworkSubsystem.h"
#include "RoadPathFollowerComponent.h"
#include "RoadTypes.h"
#include "ParkingLotActor.h"
#include "RoadsideParkingActor.h"

#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Engine/Texture2D.h"
#include "Engine/Font.h"
#include "Styling/AppStyle.h"
#include "EngineUtils.h"
#include "Blueprint/WidgetBlueprintLibrary.h"
#include "Components/SplineComponent.h"
#include "Rendering/DrawElements.h"

// ================================================================
//  生命週期 / Lifecycle
// ================================================================

void UDrivingMapWidget::NativeConstruct()
{
	Super::NativeConstruct();
	SetVisibility(ESlateVisibility::HitTestInvisible);
	BuildNodeCache();
	InitFreeCamera();
}

void UDrivingMapWidget::NativeDestruct()
{
	DeselectVehicle();
	if (FreeCameraActor)
	{
		FreeCameraActor->Destroy();
		FreeCameraActor = nullptr;
	}
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

	// 背景圖 Brush 設定（一次）/ Setup brush once
	if (!bBrushReady && MapTexture)
	{
		SetupBrush();
	}

	UpdateVehicleCache();

	// ---- ESC：跟隨車輛時按 ESC 退回自由相機 ----
	// ---- ESC: return to free camera while following vehicle ----
	if (!bFreeCameraMode && SelectedVehiclePtr.IsValid())
	{
		if (APlayerController* PC = GetOwningPlayer())
		{
			if (PC->WasInputKeyJustPressed(EKeys::SpaceBar))
			{
				DeselectVehicle();
				if (bIsFullscreen) ToggleMapSize();
				return; // 這幀不再處理其他操控 / Skip rest of input this frame
			}
		}
	}

	// ---- 自由相機模式：WASD + 右鍵旋轉 ----
	// ---- Free camera mode: WASD move + RMB rotate ----
	if (bFreeCameraMode && !bIsFullscreen)
	{
		UpdateFreeCamera(InDeltaTime);
	}
	// ---- 跟隨模式：右鍵拖曳旋轉 CameraBoom ----
	// ---- Follow mode: RMB drag to orbit CameraBoom ----
	else if (!bFreeCameraMode && SelectedVehiclePtr.IsValid() && !bIsFullscreen)
	{
		if (APlayerController* PC = GetOwningPlayer())
		{
			if (PC->IsInputKeyDown(EKeys::RightMouseButton))
			{
				float DeltaX, DeltaY;
				PC->GetInputMouseDelta(DeltaX, DeltaY);

				USpringArmComponent* Boom = SelectedVehiclePtr->FindComponentByClass<USpringArmComponent>();
				if (Boom)
				{
					FRotator BoomRot = Boom->GetRelativeRotation();
					BoomRot.Yaw += DeltaX * OrbitSensitivity;
					BoomRot.Pitch = FMath::Clamp(BoomRot.Pitch + DeltaY * OrbitSensitivity, -80.0f, -5.0f);
					Boom->SetRelativeRotation(BoomRot);
				}
			}
		}
	}
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

	// ---- 2. 道路 Spline 曲線 / Road spline curves ----
	{
		FPaintContext EdgeCtx(AllottedGeometry, MyCullingRect, OutDrawElements, LayerId + 1,
			InWidgetStyle, bParentEnabled);

		if (UWorld* World = GetWorld())
		{
			if (URoadNetworkSubsystem* RS = World->GetSubsystem<URoadNetworkSubsystem>())
			{
				const FLinearColor EdgeColor(0.3f, 0.6f, 1.0f, 0.6f);
				// 每條 Edge 沿 Spline 取樣多個點畫曲線
				// Sample multiple points along each edge's spline to draw curves
				constexpr int32 SplineSampleCount = 20;

				for (const FRoadGraphEdge& Edge : RS->GetGraphEdges())
				{
					if (!Edge.InputSpline) continue;

					const float StartDist = Edge.StartDistanceOnSpline;
					const float EndDist   = Edge.EndDistanceOnSpline;
					const float TotalDist = EndDist - StartDist;
					if (FMath::Abs(TotalDist) < 1.0f) continue;

					FVector2D PrevMapPt = FVector2D::ZeroVector;
					for (int32 s = 0; s <= SplineSampleCount; ++s)
					{
						const float Alpha = static_cast<float>(s) / SplineSampleCount;
						const float Dist  = StartDist + Alpha * TotalDist;
						const FVector WorldPt = Edge.InputSpline->GetLocationAtDistanceAlongSpline(
							Dist, ESplineCoordinateSpace::World);
						const FVector2D MapPt = WorldToMap(WorldPt, MapOrigin, MapSize);

						if (s > 0)
						{
							UWidgetBlueprintLibrary::DrawLine(
								EdgeCtx, PrevMapPt, MapPt, EdgeColor, true, 1.0f);
						}
						PrevMapPt = MapPt;
					}
				}
			}
		}
	}

	// ---- 3. 車輛箭頭 ----
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

	// ---- 3.5 選中車輛的路線粗線 / Selected vehicle route thick line ----
	if (SelectedVehiclePtr.IsValid())
	{
		URoadPathFollowerComponent* PF =
			SelectedVehiclePtr->FindComponentByClass<URoadPathFollowerComponent>();
		if (PF)
		{
			TArray<FVector> RoutePoints;
			PF->GetRouteWorldPoints(RoutePoints, 15);

			if (RoutePoints.Num() > 1)
			{
				const FLinearColor RouteColor(1.0f, 0.6f, 0.1f, 0.85f);
				const float RouteThickness = bIsFullscreen ? 3.5f : 2.0f;

				FVector2D PrevPt = WorldToMap(RoutePoints[0], MapOrigin, MapSize);
				for (int32 r = 1; r < RoutePoints.Num(); ++r)
				{
					FVector2D CurPt = WorldToMap(RoutePoints[r], MapOrigin, MapSize);
					UWidgetBlueprintLibrary::DrawLine(Context, PrevPt, CurPt,
						RouteColor, true, RouteThickness);
					PrevPt = CurPt;
				}
			}

			// 終點標記 / Destination marker
			if (SelectedDestinationNodeId != INDEX_NONE && CachedNodes.Num() > 0)
			{
				for (const FMapNodeCache& NC : CachedNodes)
				{
					if (NC.NodeId == SelectedDestinationNodeId)
					{
						FVector2D DestMapPos = WorldToMap(NC.WorldLocation, MapOrigin, MapSize);
						DrawCircle(Context, DestMapPos, bIsFullscreen ? 8.0f : 5.0f,
							DestinationColor, 2.5f);
						break;
					}
				}
			}
		}
	}

	// ---- 4. 邊框 / Border ----
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

	// ---- 5. 停車場 & 路邊停車標示 / Parking lot & roadside parking markers ----
	if (bIsFullscreen)
	{
		UWorld* ParkWorld = GetWorld();
		if (ParkWorld)
		{
			// 停車場 / Parking lots
			for (TActorIterator<AParkingLotActor> It(ParkWorld); It; ++It)
			{
				AParkingLotActor* Lot = *It;
				if (!Lot) continue;

				FVector2D LotMapPos = WorldToMap(Lot->GetActorLocation(), MapOrigin, MapSize);
				DrawCircle(Context, LotMapPos, 6.0f, ParkingLotColor, 2.0f);
				UWidgetBlueprintLibrary::DrawText(Context,
					FString::Printf(TEXT("P %s"), *Lot->ParkingLotName),
					FVector2D(LotMapPos.X + 8.0f, LotMapPos.Y - 6.0f), ParkingLotColor);

				for (int32 s = 0; s < Lot->SpotCount; ++s)
				{
					FVector2D SpotPos = WorldToMap(Lot->GetSpotWorldPosition(s), MapOrigin, MapSize);
					const bool bOcc = Lot->IsSpotOccupied(s);
					const float Rad = bOcc ? 2.5f : 3.5f;
					FLinearColor SpotColor = bOcc
						? FLinearColor(0.5f, 0.5f, 0.5f, 0.5f) : ParkingLotColor;
					DrawCircle(Context, SpotPos, Rad, SpotColor, 1.5f);
				}
			}

			// 路邊停車範圍 / Roadside parking zones
			for (TActorIterator<ARoadsideParkingActor> It(ParkWorld); It; ++It)
			{
				ARoadsideParkingActor* RSP = *It;
				if (!RSP) continue;

				for (int32 z = 0; z < RSP->GetZoneCount(); ++z)
				{
					FVector2D ZoneStart = WorldToMap(RSP->GetZoneWorldStart(z), MapOrigin, MapSize);
					FVector2D ZoneEnd = WorldToMap(RSP->GetZoneWorldEnd(z), MapOrigin, MapSize);
					UWidgetBlueprintLibrary::DrawLine(Context, ZoneStart, ZoneEnd,
						RoadsideParkingColor, true, 3.0f);
				}

				if (RSP->GetZoneCount() > 0)
				{
					FVector2D NamePos = WorldToMap(RSP->GetZoneWorldStart(0), MapOrigin, MapSize);
					UWidgetBlueprintLibrary::DrawText(Context, RSP->ZoneName,
						FVector2D(NamePos.X + 5.0f, NamePos.Y - 12.0f), RoadsideParkingColor);
				}
			}
		}
	}

	// ---- 5.5 提示文字 / Hint text ----
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
			? TEXT("[LMB] Click node = set destination  |  [Space] Unfollow  |  [M] Close")
			: TEXT("[LMB] Click vehicle = follow  |  [M] Close");
		UWidgetBlueprintLibrary::DrawText(Context, Hint, HintPos,
			FLinearColor(0.6f, 0.65f, 0.7f, 0.8f));
	}

	// ---- 6. 右側資訊面板（全螢幕 + 選中車輛時）----
	//      Right info panel — far right, semi-transparent bg, scaled font
	if (bIsFullscreen && SelectedVehiclePtr.IsValid())
	{
		URoadPathFollowerComponent* PF =
			SelectedVehiclePtr->FindComponentByClass<URoadPathFollowerComponent>();
		if (PF)
		{
			FVector2D WidgetSize = AllottedGeometry.GetLocalSize();

			// 面板定位：螢幕最右邊 / Panel: far right of screen
			const float PanelWidth = WidgetSize.X * 0.22f;
			const float PanelMarginRight = WidgetSize.X * 0.01f;
			const float PanelX = WidgetSize.X - PanelWidth - PanelMarginRight;
			const float PanelTopY = WidgetSize.Y * 0.05f;
			const float PanelHeight = WidgetSize.Y * 0.75f;

			// ---- 半透明背景 / Semi-transparent background ----
			{
				FPaintGeometry BgGeo = AllottedGeometry.ToPaintGeometry(
					FVector2f(static_cast<float>(PanelWidth), static_cast<float>(PanelHeight)),
					FSlateLayoutTransform(FVector2f(static_cast<float>(PanelX), static_cast<float>(PanelTopY)))
				);
				TArray<FSlateGradientStop> BgStops;
				const FLinearColor BgColor(0.02f, 0.03f, 0.06f, InfoPanelBackgroundAlpha);
				BgStops.Emplace(FVector2D::ZeroVector, BgColor);
				BgStops.Emplace(FVector2D(PanelWidth, 0.0), BgColor);
				FSlateDrawElement::MakeGradient(OutDrawElements, LayerId + 1, BgGeo,
					BgStops, Orient_Horizontal, ESlateDrawEffect::None);
			}

			const float TextX = PanelX + 15.0f;
			float TextY = PanelTopY + 15.0f;
			const float LineH = 22.0f * InfoPanelFontScale;
			const FLinearColor TitleColor(1.0f, 0.85f, 0.3f, 1.0f);
			const FLinearColor InfoColor(0.85f, 0.9f, 1.0f, 1.0f);
			const FLinearColor DimColor(0.5f, 0.55f, 0.6f, 0.8f);

			FSlateFontInfo ScaledFont = FAppStyle::GetFontStyle("NormalFont");
			ScaledFont.Size = static_cast<int32>(10.0f * InfoPanelFontScale);

			// 輔助 lambda：在面板畫一行文字 / Helper: draw one line of scaled text
			auto DrawPanelLine = [&](const FString& Text, const FLinearColor& Color)
			{
				FSlateDrawElement::MakeText(OutDrawElements, LayerId + 2,
					AllottedGeometry.ToPaintGeometry(
						FVector2f(PanelWidth - 30.0f, LineH),
						FSlateLayoutTransform(FVector2f(TextX, TextY))),
					Text, ScaledFont, ESlateDrawEffect::None, Color);
				TextY += LineH;
			};

			DrawPanelLine(TEXT("--- Vehicle Info ---"), TitleColor);
			TextY += LineH * 0.3f;

			// 速度 / Speed
			const float SpeedKmh = PF->GetCurrentSpeed() * 0.036f;
			const float MaxKmh = PF->MaxSpeed * 0.036f;
			DrawPanelLine(FString::Printf(TEXT("Speed: %.1f / %.0f km/h"), SpeedKmh, MaxKmh), InfoColor);

			// 狀態 / State
			FString StateStr;
			switch (PF->GetNavState())
			{
			case ENavState::Idle:    StateStr = TEXT("Idle"); break;
			case ENavState::Driving: StateStr = TEXT("Driving"); break;
			case ENavState::Parking: StateStr = TEXT("Parking"); break;
			case ENavState::Parked:  StateStr = TEXT("Parked"); break;
			}
			DrawPanelLine(FString::Printf(TEXT("State: %s"), *StateStr), InfoColor);

			// 方向燈 / Turn signal
			FString SignalStr = TEXT("None");
			if (PF->GetTurnSignal() == ETurnSignal::Left) SignalStr = TEXT("LEFT");
			else if (PF->GetTurnSignal() == ETurnSignal::Right) SignalStr = TEXT("RIGHT");
			DrawPanelLine(FString::Printf(TEXT("Signal: %s"), *SignalStr), InfoColor);

			// 車道 / Lane
			DrawPanelLine(FString::Printf(TEXT("Lane: %d"), PF->CurrentLaneIndex), InfoColor);

			// 障礙物 / Obstacle
			const float ObDist = PF->GetObstacleDistance();
			FString ObStr = (ObDist < 0.0f) ? TEXT("None") : FString::Printf(TEXT("%.0f cm"), ObDist);
			DrawPanelLine(FString::Printf(TEXT("Obstacle: %s"), *ObStr), InfoColor);

			// 超車 / Overtake
			FString OvtStr;
			FLinearColor OvtColor = InfoColor;
			switch (PF->GetOvertakeState())
			{
			case EOvertakeState::None:      OvtStr = TEXT("None"); break;
			case EOvertakeState::Passing:    OvtStr = TEXT("PASSING"); OvtColor = FLinearColor(1.0f, 0.4f, 0.1f, 1.0f); break;
			case EOvertakeState::Returning:  OvtStr = TEXT("Returning"); OvtColor = FLinearColor(0.3f, 0.8f, 1.0f, 1.0f); break;
			}
			DrawPanelLine(FString::Printf(TEXT("Overtake: %s"), *OvtStr), OvtColor);

			// 目的地 / Destination
			FString DestStr = (SelectedDestinationNodeId != INDEX_NONE)
				? FString::Printf(TEXT("Dest: Node %d"), SelectedDestinationNodeId)
				: TEXT("Dest: None");
			DrawPanelLine(DestStr, (SelectedDestinationNodeId != INDEX_NONE) ? InfoColor : DimColor);

			TextY += LineH * 0.8f;
			DrawPanelLine(TEXT("--- Controls ---"), TitleColor);
			TextY += LineH * 0.3f;
			DrawPanelLine(TEXT("Click node = Set dest"), DimColor);
			DrawPanelLine(TEXT("[Space] = Unfollow"), DimColor);
			DrawPanelLine(TEXT("[Scroll] = Zoom"), DimColor);
			DrawPanelLine(TEXT("[M] = Close map"), DimColor);
		}
	}

	return LayerId + 3;
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

		// 已選車 → 點停車場或節點設目的地
		// Selected vehicle → click parking lot or node to set destination
		if (SelectedVehiclePtr.IsValid())
		{
			URoadPathFollowerComponent* PF =
				SelectedVehiclePtr->FindComponentByClass<URoadPathFollowerComponent>();

			// 優先偵測停車場 / Parking lot has priority
			AActor* ClickedLot = FindParkingLotNearMapPos(LocalPos, MapOrigin, MapSize, 22.0f);
			if (ClickedLot && PF)
			{
				// 找停車場最近的 Graph Node 作為導航目的地
				// Find nearest graph node to parking lot as navigation destination
				if (UWorld* World = GetWorld())
				{
					if (URoadNetworkSubsystem* RS = World->GetSubsystem<URoadNetworkSubsystem>())
					{
						int32 NearestNode = RS->FindNearestGraphNode(ClickedLot->GetActorLocation());
						if (NearestNode != INDEX_NONE)
						{
							PF->NavigateToNode(NearestNode);
							SelectedDestinationNodeId = NearestNode;
							UE_LOG(LogTemp, Log, TEXT("MapWidget: Navigate to ParkingLot '%s' → Node %d"),
								*Cast<AParkingLotActor>(ClickedLot)->ParkingLotName, NearestNode);
						}
					}
				}
				return FReply::Handled();
			}

			// 點節點 / Click node
			int32 ClickedNodeId = FindNodeNearMapPos(LocalPos, MapOrigin, MapSize, 18.0f);
			if (ClickedNodeId != INDEX_NONE && PF)
			{
				PF->NavigateToNode(ClickedNodeId);
				SelectedDestinationNodeId = ClickedNodeId;
				return FReply::Handled();
			}
		}

		// 點到地圖空白處不做任何事（不取消選取、不關地圖）
		// Clicking empty area on map → do nothing (don't deselect, don't close)
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
		USpringArmComponent* Boom = SelectedVehiclePtr->FindComponentByClass<USpringArmComponent>();
		if (Boom)
		{
			float Delta = InMouseEvent.GetWheelDelta();
			Boom->TargetArmLength = FMath::Clamp(
				Boom->TargetArmLength - Delta * ZoomSpeed,
				MinFollowDistance, MaxFollowDistance);
		}
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
	bFreeCameraMode = false;

	// 記錄目的地節點 / Record destination node
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

	// 儲存 SpringArm 原始旋轉，方便取消時還原
	// Save original boom rotation so we can restore on deselect
	USpringArmComponent* Boom = Vehicle->FindComponentByClass<USpringArmComponent>();
	if (Boom)
	{
		OriginalBoomRotation = Boom->GetRelativeRotation();
	}

	// 直接把 ViewTarget 設成車輛本身 → 用車輛的 CameraBoom + FollowCamera
	// Set view target to the vehicle itself → uses its own CameraBoom + FollowCamera
	if (APlayerController* PC = GetOwningPlayer())
	{
		PC->SetViewTargetWithBlend(Vehicle, 0.5f);
	}
}

void UDrivingMapWidget::DeselectVehicle()
{
	// 還原 SpringArm 旋轉 / Restore original boom rotation
	if (SelectedVehiclePtr.IsValid())
	{
		USpringArmComponent* Boom = SelectedVehiclePtr->FindComponentByClass<USpringArmComponent>();
		if (Boom)
		{
			Boom->SetRelativeRotation(OriginalBoomRotation);
		}
	}

	// 把自由相機移到目前視角位置，避免跳回舊位置
	// Move free camera to current view so it doesn't snap back
	if (FreeCameraActor)
	{
		if (APlayerController* PC = GetOwningPlayer())
		{
			FVector ViewLoc;
			FRotator ViewRot;
			PC->GetPlayerViewPoint(ViewLoc, ViewRot);
			FreeCameraActor->SetActorLocation(ViewLoc);
			FreeCameraActor->SetActorRotation(ViewRot);
		}
	}

	SelectedVehiclePtr = nullptr;
	SelectedDestinationNodeId = INDEX_NONE;
	bFreeCameraMode = true;

	// 切回自由相機 / Switch back to free camera
	if (APlayerController* PC = GetOwningPlayer())
	{
		if (FreeCameraActor)
			PC->SetViewTargetWithBlend(FreeCameraActor, 0.3f);
	}
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
		FVector2D Margin = Range * static_cast<double>(MapBoundsMarginRatio);
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
		// 全螢幕地圖放左側（佔螢幕 55% 寬），右側留給路徑面板
		// Fullscreen map on left side (55% of screen), right side for route panel
		FVector2D FullSize;
		FullSize.X = WidgetSize.X * 0.55;
		FullSize.Y = WidgetSize.Y * FullscreenCoverage;
		const float MarginY = (WidgetSize.Y - FullSize.Y) * 0.5;
		const float MarginX = WidgetSize.X * 0.03;
		OutOrigin = FVector2D(MarginX, MarginY);
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

AActor* UDrivingMapWidget::FindParkingLotNearMapPos(
	const FVector2D& LocalPos, const FVector2D& MapOrigin,
	const FVector2D& MapSize, float Radius) const
{
	float BestDistSq = Radius * Radius;
	AActor* BestLot = nullptr;

	UWorld* World = GetWorld();
	if (!World) return nullptr;

	for (TActorIterator<AParkingLotActor> It(World); It; ++It)
	{
		AParkingLotActor* Lot = *It;
		if (!Lot) continue;

		FVector2D MapPos = WorldToMap(Lot->GetActorLocation(), MapOrigin, MapSize);
		float DistSq = static_cast<float>(FVector2D::DistSquared(LocalPos, MapPos));
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestLot = Lot;
		}
	}
	return BestLot;
}

// ================================================================
//  自由相機 / Free Camera
// ================================================================

void UDrivingMapWidget::InitFreeCamera()
{
	if (FreeCameraActor) return;

	UWorld* World = GetWorld();
	if (!World) return;

	// 從目前玩家視角位置開始 / Start at current player view
	FVector StartLoc = FVector(0.0, 0.0, 500.0);
	FRotator StartRot = FRotator(-30.0, 0.0, 0.0);
	if (APlayerController* PC = GetOwningPlayer())
	{
		PC->GetPlayerViewPoint(StartLoc, StartRot);
	}

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	FreeCameraActor = World->SpawnActor<ACameraActor>(StartLoc, StartRot, Params);

	if (FreeCameraActor)
	{
		if (UCameraComponent* CamComp = FreeCameraActor->GetCameraComponent())
		{
			CamComp->SetFieldOfView(90.0f);
		}

		// 立即設為 ViewTarget / Set as active view target immediately
		if (APlayerController* PC = GetOwningPlayer())
		{
			PC->SetViewTarget(FreeCameraActor);
		}
	}

	bFreeCameraMode = true;
}

void UDrivingMapWidget::UpdateFreeCamera(float DeltaTime)
{
	if (!FreeCameraActor) return;

	APlayerController* PC = GetOwningPlayer();
	if (!PC) return;

	// ---- 右鍵旋轉 / RMB + mouse to rotate ----
	if (PC->IsInputKeyDown(EKeys::RightMouseButton))
	{
		float DeltaX, DeltaY;
		PC->GetInputMouseDelta(DeltaX, DeltaY);

		FRotator CamRot = FreeCameraActor->GetActorRotation();
		CamRot.Yaw   += DeltaX * FreeCameraRotateSpeed;
		CamRot.Pitch  = FMath::Clamp(CamRot.Pitch + DeltaY * FreeCameraRotateSpeed, -89.0f, 89.0f);
		CamRot.Roll   = 0.0f;
		FreeCameraActor->SetActorRotation(CamRot);
	}

	// ---- WASD + QE 移動 / WASD + QE movement ----
	FVector MoveInput = FVector::ZeroVector;
	if (PC->IsInputKeyDown(EKeys::W)) MoveInput.X += 1.0f;
	if (PC->IsInputKeyDown(EKeys::S)) MoveInput.X -= 1.0f;
	if (PC->IsInputKeyDown(EKeys::D)) MoveInput.Y += 1.0f;
	if (PC->IsInputKeyDown(EKeys::A)) MoveInput.Y -= 1.0f;
	if (PC->IsInputKeyDown(EKeys::E)) MoveInput.Z += 1.0f;
	if (PC->IsInputKeyDown(EKeys::Q)) MoveInput.Z -= 1.0f;

	if (!MoveInput.IsNearlyZero())
	{
		MoveInput.Normalize();

		float Speed = FreeCameraMoveSpeed;
		if (PC->IsInputKeyDown(EKeys::LeftShift))
		{
			Speed *= FreeCameraFastMultiplier;
		}

		FRotator CamRot = FreeCameraActor->GetActorRotation();
		FVector Forward = FRotationMatrix(CamRot).GetUnitAxis(EAxis::X);
		FVector Right   = FRotationMatrix(CamRot).GetUnitAxis(EAxis::Y);
		FVector Up      = FVector::UpVector;

		FVector WorldMove = Forward * MoveInput.X + Right * MoveInput.Y + Up * MoveInput.Z;
		FreeCameraActor->SetActorLocation(
			FreeCameraActor->GetActorLocation() + WorldMove * Speed * DeltaTime);
	}
}

