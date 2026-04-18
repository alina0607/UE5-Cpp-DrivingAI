#include "DrivingMapWidget.h"
#include "RoadNetworkSubsystem.h"
#include "RoadPathFollowerComponent.h"
#include "RoadTypes.h"
#include "ParkingLotActor.h"

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
#include "DrawDebugHelpers.h"

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

	// Setup brush once
	if (!bBrushReady && MapTexture)
	{
		SetupBrush();
	}

	UpdateVehicleCache();

	// ---- ESC: return to free camera while following vehicle ----
	if (!bFreeCameraMode && SelectedVehiclePtr.IsValid())
	{
		if (APlayerController* PC = GetOwningPlayer())
		{
			if (PC->WasInputKeyJustPressed(EKeys::SpaceBar))
			{
				DeselectVehicle();
				if (bIsFullscreen) ToggleMapSize();
				return; // Skip rest of input this frame
			}
		}
	}

	// ---- Free camera mode: WASD move + RMB rotate ----
	if (bFreeCameraMode && !bIsFullscreen)
	{
		UpdateFreeCamera(InDeltaTime);
	}
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
	/*
	// ================================================================
	//  In-viewport vehicle arrows + click-to-select (minimap mode only)
	//  Draws a 3D arrow above every vehicle and lets the user click any
	//  vehicle to lock camera + show info panel — WITHOUT opening the
	//  fullscreen map.
	// ================================================================
#if ENABLE_DRAW_DEBUG
	if (!bIsFullscreen)
	{
		UWorld* World = GetWorld();
		if (World)
		{
			for (const FMapVehicleCache& V : CachedVehicles)
			{
				AActor* A = V.Actor.Get();
				if (!A) continue;

				const bool bSel = (SelectedVehiclePtr.Get() == A);
				const FColor ArrowColor = bSel
					? FColor(255, 220, 0)       // yellow for selected
					: FColor(40, 255, 90);      // green for unselected

				const FVector Base = V.WorldLocation + FVector(0, 0, 380.0f);
				const FVector Tip  = Base + V.ForwardVector * 250.0f;

				DrawDebugDirectionalArrow(
					World, Base, Tip,
					120.0f,                     // arrow-head size
					ArrowColor,
					false,                      // bPersistent
					-1.0f,                      // lifetime (single frame)
					0,                          // depth priority
					bSel ? 6.0f : 4.0f);        // thickness
			}
		}
	}
#endif

	*/
	// Click-to-select without opening the big map.
	// The widget is HitTestInvisible in minimap mode, so we poll the PC directly.
	if (!bIsFullscreen)
	{
		if (APlayerController* PC = GetOwningPlayer())
		{
			if (PC->WasInputKeyJustPressed(EKeys::LeftMouseButton))
			{
				FHitResult ClickHit;
				if (PC->GetHitResultUnderCursor(ECC_Visibility, true, ClickHit))
				{
					if (AActor* HitActor = ClickHit.GetActor())
					{
						// Walk up to find an actor that carries a PathFollower
						AActor* VehicleActor = HitActor;
						while (VehicleActor && !VehicleActor->FindComponentByClass<URoadPathFollowerComponent>())
						{
							VehicleActor = VehicleActor->GetOwner();
						}
						if (VehicleActor)
						{
							UE_LOG(LogTemp, Warning,
								TEXT("[VIEWPORT-CLICK] Selected vehicle %s"),
								*VehicleActor->GetName());
							SelectVehicle(VehicleActor);
							// NOTE: do NOT ToggleMapSize here — stay in minimap mode
						}
					}
				}
			}
		}
	}
}

// ================================================================
//  Paint
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

	DrawBackground(OutDrawElements, AllottedGeometry, MapOrigin, MapSize, LayerId);

	/*
	// ---- Road spline curves ----
	{
		FPaintContext EdgeCtx(AllottedGeometry, MyCullingRect, OutDrawElements, LayerId + 1,
			InWidgetStyle, bParentEnabled);

		if (UWorld* World = GetWorld())
		{
			if (URoadNetworkSubsystem* RS = World->GetSubsystem<URoadNetworkSubsystem>())
			{
				const float EdgeThickness = bIsFullscreen ? EdgeThicknessFullscreen : EdgeThicknessMinimap; // 新增
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
								EdgeCtx, PrevMapPt, MapPt, EdgeColor, true, EdgeThickness);
						}
						PrevMapPt = MapPt;
					}
				}
			}
		}
	}
	*/
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

	// ---- Selected vehicle route thick line ----
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
				const float RouteThickness = bIsFullscreen ? RouteThicknessFullscreen : RouteThicknessMinimap;

				FVector2D PrevPt = WorldToMap(RoutePoints[0], MapOrigin, MapSize);
				for (int32 r = 1; r < RoutePoints.Num(); ++r)
				{
					FVector2D CurPt = WorldToMap(RoutePoints[r], MapOrigin, MapSize);
					UWidgetBlueprintLibrary::DrawLine(Context, PrevPt, CurPt,
						RouteColor, true, RouteThickness);
					PrevPt = CurPt;
				}
			}

			// Dynamically read from PF so auto-nav destination changes are shown immediately
			// NativePaint is const so we only READ here; SelectVehicle() caches the value for non-const use
			{
				FVector DestDisplayPos = SelectedDestinationWorldPos;
				// If selected vehicle has PF, prefer live value (auto-nav may have changed dest)
				AActor* SelVeh = SelectedVehiclePtr.Get();
				if (SelVeh)
				{
					URoadPathFollowerComponent* SelPF = SelVeh->FindComponentByClass<URoadPathFollowerComponent>();
					if (SelPF)
					{
						const FVector LiveDest = SelPF->GetDestinationDisplayLocation();
						if (!LiveDest.IsNearlyZero())
						{
							DestDisplayPos = LiveDest;
						}
					}
				}

				if (!DestDisplayPos.IsNearlyZero())
				{
					FVector2D DestMapPos = WorldToMap(DestDisplayPos, MapOrigin, MapSize);
					DrawCircle(Context, DestMapPos, bIsFullscreen ? 8.0f : 5.0f,
						DestinationColor, 2.5f);
				}
			}
		}
	}

	// ---- Border ----
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

	// ---- Parking lot markers ----
	if (bIsFullscreen)
	{
		UWorld* ParkWorld = GetWorld();
		if (ParkWorld)
		{
			// Parking lot font
			FSlateFontInfo LotFont = FAppStyle::GetFontStyle("NormalFont");
			LotFont.Size = ParkingLotFontSize;

			// Parking lots
			for (TActorIterator<AParkingLotActor> It(ParkWorld); It; ++It)
			{
				AParkingLotActor* Lot = *It;
				if (!Lot) continue;

				FVector2D LotMapPos = WorldToMap(Lot->GetActorLocation(), MapOrigin, MapSize);

				const bool bHovered = (HoveredParkingLot.IsValid() && HoveredParkingLot.Get() == Lot);
				const FLinearColor CurrentLotColor = bHovered ? ParkingLotHoverColor : ParkingLotColor;

				DrawCircle(Context, LotMapPos, ParkingLotCircleRadius, CurrentLotColor, 2.0f);
				DrawCircle(Context, LotMapPos,
					ParkingLotCircleRadius + ParkingLotOutlineThickness + 1.0f,
					bHovered ? ParkingLotHoverColor : ParkingLotOutlineColor,
					ParkingLotOutlineThickness);
			}

		}
	}

	// ---- Hint text ----
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
			? TEXT("[LMB] Click parking = set destination  |  [Space] Unfollow  |  [M] Close")
			: TEXT("[LMB] Click vehicle = follow  |  [M] Close");
		UWidgetBlueprintLibrary::DrawText(Context, Hint, HintPos,
			FLinearColor(0.6f, 0.65f, 0.7f, 0.8f));
	}

	//      Right info panel — far right, semi-transparent bg, scaled font
	//  Show whenever a vehicle is selected (fullscreen OR minimap mode), so
	//  picking a car in the viewport immediately reveals its detail panel.
	if (SelectedVehiclePtr.IsValid())
	{
		URoadPathFollowerComponent* PF =
			SelectedVehiclePtr->FindComponentByClass<URoadPathFollowerComponent>();
		if (PF)
		{
			FVector2D WidgetSize = AllottedGeometry.GetLocalSize();

			// Panel: far right of screen
			const float PanelWidth = WidgetSize.X * 0.22f;
			const float PanelMarginRight = WidgetSize.X * 0.01f;
			const float PanelX = WidgetSize.X - PanelWidth - PanelMarginRight;
			const float PanelTopY = WidgetSize.Y * 0.05f;
			const float PanelHeight = WidgetSize.Y * 0.75f;

			// ---- Semi-transparent background ----
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

			// Helper: draw one line of scaled text
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

			// Speed
			const float SpeedKmh = PF->GetCurrentSpeed() * 0.036f;
			const float MaxKmh = PF->MaxSpeed * 0.036f;
			DrawPanelLine(FString::Printf(TEXT("Speed: %.1f / %.0f km/h"), SpeedKmh, MaxKmh), InfoColor);

			// State
			FString StateStr;
			switch (PF->GetNavState())
			{
			case ENavState::Idle:    StateStr = TEXT("Idle"); break;
			case ENavState::Driving: StateStr = TEXT("Driving"); break;
			case ENavState::Parking: StateStr = TEXT("Parking"); break;
			case ENavState::Parked:  StateStr = TEXT("Parked"); break;
			}
			DrawPanelLine(FString::Printf(TEXT("State: %s"), *StateStr), InfoColor);

			// Turn signal
			FString SignalStr = TEXT("None");
			if (PF->GetTurnSignal() == ETurnSignal::Left) SignalStr = TEXT("LEFT");
			else if (PF->GetTurnSignal() == ETurnSignal::Right) SignalStr = TEXT("RIGHT");
			DrawPanelLine(FString::Printf(TEXT("Signal: %s"), *SignalStr), InfoColor);

			// Lane
			DrawPanelLine(FString::Printf(TEXT("Lane: %d"), PF->CurrentLaneIndex), InfoColor);

			// Obstacle — show distance, type, and name
			{
				const float ObDist = PF->GetObstacleDistance();
				if (ObDist < 0.0f)
				{
					DrawPanelLine(TEXT("Obstacle: None"), DimColor);
				}
				else
				{
					const FString ObType = PF->GetObstacleTypeString();
					const FString ObName = PF->GetObstacleActorName();
					FLinearColor ObColor = InfoColor;
					if (ObType == TEXT("Vehicle"))       ObColor = FLinearColor(1.0f, 0.6f, 0.2f, 1.0f); // orange
					else if (ObType == TEXT("Traffic"))   ObColor = FLinearColor(1.0f, 0.3f, 0.3f, 1.0f); // red
					else                                  ObColor = FLinearColor(0.7f, 0.7f, 0.7f, 1.0f); // grey

					DrawPanelLine(FString::Printf(TEXT("Obstacle: %.0f cm [%s]"), ObDist, *ObType), ObColor);
					if (!ObName.IsEmpty())
					{
						DrawPanelLine(FString::Printf(TEXT("  → %s"), *ObName), DimColor);
					}
				}
			}

			// Overtake
			FString OvtStr;
			FLinearColor OvtColor = InfoColor;
			switch (PF->GetOvertakeState())
			{
			case EOvertakeState::None:      OvtStr = TEXT("None"); break;
			case EOvertakeState::Passing:    OvtStr = TEXT("PASSING"); OvtColor = FLinearColor(1.0f, 0.4f, 0.1f, 1.0f); break;
			case EOvertakeState::Returning:  OvtStr = TEXT("Returning"); OvtColor = FLinearColor(0.3f, 0.8f, 1.0f, 1.0f); break;
			}
			DrawPanelLine(FString::Printf(TEXT("Overtake: %s"), *OvtStr), OvtColor);

			// Destination
			{
				FString DestStr;
				FLinearColor DestColor = DimColor;
				const FString DestName = PF->GetDestinationName();
				const int32 SpotIdx = PF->GetTargetSpotIndex();

				switch (PF->GetDestinationType())
				{
				case EDestinationType::ParkingLot:
					DestStr = TEXT("Dest: Parking Lot");
					DestColor = ParkingLotColor;
					break;
				default:
					DestStr = TEXT("Dest: None");
					break;
				}
				DrawPanelLine(DestStr, DestColor);

				// Destination name + spot index
				if (!DestName.IsEmpty())
				{
					DrawPanelLine(FString::Printf(TEXT("  Name: %s"), *DestName), DestColor);
					if (SpotIdx != INDEX_NONE)
					{
						DrawPanelLine(FString::Printf(TEXT("  Spot: #%d"), SpotIdx), DestColor);
					}
				}
			}

			// ---- Recent events (stuck / overtake / depart / segment) ----
			TextY += LineH * 0.8f;
			DrawPanelLine(TEXT("--- Recent Events ---"), TitleColor);
			TextY += LineH * 0.2f;
			{
				const TArray<FString>& Events = PF->GetRecentEvents();
				// Smaller font for log entries so more history fits.
				FSlateFontInfo LogFont = FAppStyle::GetFontStyle("NormalFont");
				LogFont.Size = FMath::Max(8, static_cast<int32>(8.0f * InfoPanelFontScale));
				const float LogLineH = 16.0f * InfoPanelFontScale;
				const FLinearColor LogColor(0.85f, 0.9f, 1.0f, 0.95f);
				// Draw newest first — iterate backwards.
				for (int32 i = Events.Num() - 1; i >= 0; --i)
				{
					FSlateDrawElement::MakeText(OutDrawElements, LayerId + 2,
						AllottedGeometry.ToPaintGeometry(
							FVector2f(PanelWidth - 30.0f, LogLineH),
							FSlateLayoutTransform(FVector2f(TextX, TextY))),
						Events[i], LogFont, ESlateDrawEffect::None, LogColor);
					TextY += LogLineH;
					if (TextY > PanelTopY + PanelHeight - LogLineH) break;
				}
			}

			TextY += LineH * 0.8f;
			DrawPanelLine(TEXT("--- Controls ---"), TitleColor);
			TextY += LineH * 0.3f;
			DrawPanelLine(TEXT("Click parking = Set dest"), DimColor);
			DrawPanelLine(TEXT("[Space] = Unfollow"), DimColor);
			DrawPanelLine(TEXT("[Scroll] = Zoom"), DimColor);
			DrawPanelLine(TEXT("[M] = Close map"), DimColor);
		}
	}


	// ---- Hover 名稱顯示（視窗中上）----
	if (bIsFullscreen && HoveredParkingLot.IsValid())
	{
		FSlateFontInfo HoverFont = FAppStyle::GetFontStyle("NormalFont");
		HoverFont.Size = ParkingLotHoverFontSize;

		FVector2D WidgetSize = AllottedGeometry.GetLocalSize();
		const FString HoverText = HoveredParkingLot->ParkingLotName;

		// 置中上方
		const float TextWidth = 300.0f;
		const float TextX = (WidgetSize.X - TextWidth) * 0.5f;
		const float TextY = WidgetSize.Y * 0.05f;

		FSlateDrawElement::MakeText(
			OutDrawElements, LayerId + 3,
			AllottedGeometry.ToPaintGeometry(
				FVector2f(TextWidth, 40.0f),
				FSlateLayoutTransform(FVector2f(TextX, TextY))),
			HoverText,
			HoverFont,
			ESlateDrawEffect::None,
			ParkingLotHoverTextColor);


		HoverFont.Size = ParkingLotFontSize;

		// 用停車場世界座標轉地圖座標
		FVector2D LotMapPos = WorldToMap(
			HoveredParkingLot->GetActorLocation(), MapOrigin, MapSize);

		const FString HoverText2 = HoveredParkingLot->ParkingLotName;

		FSlateDrawElement::MakeText(
			OutDrawElements, LayerId + 3,
			AllottedGeometry.ToPaintGeometry(
				FVector2f(TextWidth, 40.0f),
				FSlateLayoutTransform(FVector2f(
					LotMapPos.X + ParkingLotNameOffset.X,
					LotMapPos.Y + ParkingLotNameOffset.Y))),
			HoverText2,
			HoverFont,
			ESlateDrawEffect::None,
			ParkingLotTextColor);

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

		// 已選車 → 點停車場設目的地
		// Selected vehicle → click parking lot to set destination
		if (SelectedVehiclePtr.IsValid())
		{
			URoadPathFollowerComponent* PF =
				SelectedVehiclePtr->FindComponentByClass<URoadPathFollowerComponent>();

			UE_LOG(LogTemp, Warning, TEXT("[MAP-CLICK] Vehicle selected, PF=%s, NavState=%d"),
				PF ? TEXT("valid") : TEXT("null"),
				PF ? (int32)PF->GetNavState() : -1);

			// 優先偵測停車場 / Parking lot has priority
			AParkingLotActor* ClickedLot = FindParkingLotNearMapPos(LocalPos, MapOrigin, MapSize, 22.0f);
			if (ClickedLot)
			{
				UE_LOG(LogTemp, Warning, TEXT("[MAP-CLICK] Clicked ParkingLot '%s' at (%.0f,%.0f,%.0f) NearestEdge=%d"),
					*ClickedLot->ParkingLotName,
					ClickedLot->GetActorLocation().X, ClickedLot->GetActorLocation().Y, ClickedLot->GetActorLocation().Z,
					ClickedLot->NearestEdgeId);

				if (PF)
				{
					PF->NavigateToParkingLot(ClickedLot);
					SelectedDestinationActor = ClickedLot;
					SelectedDestinationWorldPos = ClickedLot->GetActorLocation();
				}
				return FReply::Handled();
			}

			// 沒點到任何停車設施 / Didn't hit any parking facility
			UE_LOG(LogTemp, Warning, TEXT("[MAP-CLICK] No parking lot at click pos (%.0f,%.0f)"),
				LocalPos.X, LocalPos.Y);
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

FReply UDrivingMapWidget::NativeOnMouseMove(const FGeometry& InGeometry, const FPointerEvent& InMouseEvent)
{
	if (!bIsFullscreen)
	{
		HoveredParkingLot = nullptr;
		return FReply::Unhandled();
	}

	FVector2D LocalPos = InGeometry.AbsoluteToLocal(InMouseEvent.GetScreenSpacePosition());
	FVector2D MapOrigin, MapSize;
	GetMapDrawRect(InGeometry, MapOrigin, MapSize);

	HoveredParkingLot = FindParkingLotNearMapPos(LocalPos, MapOrigin, MapSize, 22.0f);

	return FReply::Unhandled();
}

// ================================================================
//  公開 API / Public API
// ================================================================

void UDrivingMapWidget::ToggleMapSize()
{
	bIsFullscreen = !bIsFullscreen;
	if (!bIsFullscreen) HoveredParkingLot = nullptr;

	SetVisibility(bIsFullscreen ? ESlateVisibility::Visible : ESlateVisibility::HitTestInvisible);
}

void UDrivingMapWidget::SelectVehicle(AActor* Vehicle)
{
	if (!Vehicle) { DeselectVehicle(); return; }

	SelectedVehiclePtr = Vehicle;
	bFreeCameraMode = false;

	// 記錄目的地 / Record destination
	URoadPathFollowerComponent* PF = Vehicle->FindComponentByClass<URoadPathFollowerComponent>();
	if (PF)
	{
		// 用 DisplayLocation — 停車場時回傳停車場位置（不是路上投影點）
		// Use DisplayLocation — for parking lots returns the lot position (not road projection)
		SelectedDestinationWorldPos = PF->GetDestinationDisplayLocation();
		if (!SelectedDestinationWorldPos.IsNearlyZero())
		{
			SelectedDestinationActor = nullptr; // 已有位置就足夠畫標記 / Position is enough for marker
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
	SelectedDestinationActor = nullptr;
	SelectedDestinationWorldPos = FVector::ZeroVector;
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

	// 用 graph nodes 計算世界邊界（確保車輛箭頭、停車標示位置正確）
	// Compute world bounds from graph nodes (for map coordinate transform)
	{
		FVector2D Min(TNumericLimits<double>::Max(), TNumericLimits<double>::Max());
		FVector2D Max(TNumericLimits<double>::Lowest(), TNumericLimits<double>::Lowest());
		for (const FRoadGraphNode& Node : GraphNodes)
		{
			Min.X = FMath::Min(Min.X, Node.WorldLocation.X);
			Min.Y = FMath::Min(Min.Y, Node.WorldLocation.Y);
			Max.X = FMath::Max(Max.X, Node.WorldLocation.X);
			Max.Y = FMath::Max(Max.Y, Node.WorldLocation.Y);
		}

		// 也把停車場和路邊停車位置納入 bounds 計算
		// Also include parking actors in bounds calculation
		for (TActorIterator<AParkingLotActor> It(World); It; ++It)
		{
			if (!*It) continue;
			const FVector& Loc = (*It)->GetActorLocation();
			Min.X = FMath::Min(Min.X, Loc.X);
			Min.Y = FMath::Min(Min.Y, Loc.Y);
			Max.X = FMath::Max(Max.X, Loc.X);
			Max.Y = FMath::Max(Max.Y, Loc.Y);
		}
		FVector2D Range = Max - Min;
		FVector2D Margin = Range * static_cast<double>(MapBoundsMarginRatio);
		WorldBoundsMin = Min - Margin;
		WorldBoundsMax = Max + Margin;
		if (WorldBoundsMax.X - WorldBoundsMin.X < 100.0) WorldBoundsMax.X = WorldBoundsMin.X + 100.0;
		if (WorldBoundsMax.Y - WorldBoundsMin.Y < 100.0) WorldBoundsMax.Y = WorldBoundsMin.Y + 100.0;
	}

	bCacheBuilt = true;

	UE_LOG(LogTemp, Warning, TEXT("[MAP] WorldBoundsMin=(%.0f, %.0f) WorldBoundsMax=(%.0f, %.0f)"),
		WorldBoundsMin.X, WorldBoundsMin.Y,
		WorldBoundsMax.X, WorldBoundsMax.Y);
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
	// 改用背景圖對應的固定世界範圍
	const FVector2D& BoundsMin = MapTextureWorldMin;
	const FVector2D& BoundsMax = MapTextureWorldMax;

	double RangeX = BoundsMax.X - BoundsMin.X;
	double RangeY = BoundsMax.Y - BoundsMin.Y;
	if (RangeX < 1.0) RangeX = 1.0;
	if (RangeY < 1.0) RangeY = 1.0;

	double NormX = (WorldPos.X - BoundsMin.X) / RangeX;
	double NormY = (WorldPos.Y - BoundsMin.Y) / RangeY;

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

AParkingLotActor* UDrivingMapWidget::FindParkingLotNearMapPos(
	const FVector2D& LocalPos, const FVector2D& MapOrigin,
	const FVector2D& MapSize, float Radius) const
{
	float BestDistSq = Radius * Radius;
	AParkingLotActor* BestLot = nullptr;

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

