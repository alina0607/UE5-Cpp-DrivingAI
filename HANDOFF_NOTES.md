# CarDrivingProject - Handoff Notes for Next Claude Session

## Project Overview
UE5.7 C++ autonomous driving simulation. Vehicles follow road graph paths (A* pathfinding).
User prefers Chinese responses, bilingual code comments (Chinese + English).

## Build System
- **Build.cs** dependencies: `Core, CoreUObject, Engine, InputCore, EnhancedInput` (public), `Slate, SlateCore, UMG` (private)
- Project module: `CARDRIVINGPROJECT_API`

## Architecture - Key Classes

### Road System (DONE, stable)
- `URoadNetworkSubsystem` — World subsystem, holds road graph (`TArray<FRoadGraphNode>`), A* pathfinding
- `URoadPathFollowerComponent` — AI path following component on each vehicle, has `NavigateToNode()`, `GetCurrentSpeed()`, `GetNavState()`, `GetDestinationLocation()`
- `FRoadGraphNode` — Node struct with `NodeId`, `WorldLocation`, edges

### Vehicle (DONE, stable)
- `ADrivingVehiclePawn` (APawn subclass)
  - Hierarchy: `RootScene` (root) -> `VehicleMesh` (child, rotate in BP) + `CameraBoom` (SpringArm) -> `FollowCamera`
  - `PathFollower` component for AI navigation
  - CameraBoom defaults: TargetArmLength=600, Pitch=-20, CameraLagSpeed=5

### Map Widget (recently reworked)
- `UDrivingMapWidget` (UUserWidget subclass) — All rendering via NativePaint override
- `DrivingMapHUD` — HUD that creates the widget, binds M key (toggle map) and Escape key

### Camera State Machine (just implemented)
Two modes controlled by `bFreeCameraMode`:

1. **Free Camera Mode** (`bFreeCameraMode = true`, default at startup)
   - Single persistent `ACameraActor` (`FreeCameraActor`) spawned once in `InitFreeCamera()`
   - WASD = move, Q/E = down/up, RMB+mouse = rotate, Shift = speed boost
   - Adjustable via BP: `FreeCameraMoveSpeed`, `FreeCameraRotateSpeed`, `FreeCameraFastMultiplier`

2. **Vehicle Follow Mode** (`bFreeCameraMode = false`)
   - `SelectVehicle()` calls `PC->SetViewTargetWithBlend(Vehicle)` — uses vehicle's OWN CameraBoom + FollowCamera
   - RMB+mouse = orbit vehicle's SpringArm, scroll = adjust SpringArm TargetArmLength
   - Adjustable: `OrbitSensitivity`, `ZoomSpeed`, `MinFollowDistance`, `MaxFollowDistance`
   - ESC = `DeselectVehicle()` -> returns to free camera (positioned at current view, no snap)
   - `OriginalBoomRotation` saved on select, restored on deselect

### Map Features
- **Minimap**: always visible (bottom-right), shows vehicle arrows in real-time
- **Fullscreen map**: press M to toggle, click vehicle to follow, click node to set destination
- **Background**: user's own Texture2D screenshot set via `MapTexture` UPROPERTY in BP
- **WorldBounds**: auto-computed from road graph nodes (not from MapTextureWorldMin/Max)
- **Coordinate transform**: World Y -> map horizontal, World X -> map vertical (inverted)
- NO road lines, NO node circles drawn (user's background image has these)
- NO SceneCapture2D (was removed after many failed attempts)

## Key Files
```
Source/CarDrivingProject/
  CarDrivingProject.Build.cs
  DrivingMapWidget.h / .cpp      <-- Map + camera system (recently reworked)
  DrivingMapHUD.h / .cpp          <-- Creates widget, key bindings
  DrivingVehiclePawn.h / .cpp     <-- Vehicle with CameraBoom
  RoadNetworkSubsystem.h / .cpp   <-- Road graph + A* pathfinding
  RoadPathFollowerComponent.h / .cpp <-- AI path following
  RoadTypes.h                     <-- FRoadGraphNode, FRoadGraphEdge, etc.
```

## Known Issues / Things to Watch
1. `MapTextureWorldMin/Max` UPROPERTYs exist but are UNUSED — WorldBounds always computed from nodes
2. When the fullscreen map opens, free camera movement is disabled (`!bIsFullscreen` check)
3. The HUD's Escape binding might conflict with the widget's ESC handling for vehicle deselect — test this
4. Vehicle arrows may not align perfectly with user's background screenshot — depends on image coverage vs node bounds
5. `Context.MaxLayer` is returned from NativePaint — do NOT use `MaxLayer++` (caused crashes before)

## Coding Conventions
- Bilingual comments: Chinese first, English after `/ `
- Use `TObjectPtr<>` for UPROPERTY object pointers, `TWeakObjectPtr<>` for non-owning references
- No SceneCapture, no dynamic camera creation/destruction per selection
- All map rendering in NativePaint via FSlateDrawElement + UWidgetBlueprintLibrary
- Only ONE camera actor exists (FreeCameraActor) — vehicle cameras are built-in components

## What Was NOT Done Yet (Potential Next Tasks)
- Traffic light system
- Lane change logic improvements
- Vehicle spawning UI
- Mini-map zoom/pan
- Sound system
- Any BP setup guidance the user might need
