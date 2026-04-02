#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "DrivingVehiclePawn.generated.h"

class URoadPathFollowerComponent;
class USpringArmComponent;
class UCameraComponent;

/// 駕駛車輛 Pawn 基底類別
///
/// 元件階層 / Component hierarchy:
///   RootScene          ← PathFollower 控制這個的 Transform
///     └─ VehicleMesh   ← BP 裡自由旋轉來對齊模型朝向
///     └─ CameraBoom    ← BP 裡調高度 / 距離 / 角度
///         └─ FollowCamera
///
/// RootScene is moved by PathFollower. VehicleMesh is a child,
/// so you can rotate it freely in BP to fix model orientation.
UCLASS()
class CARDRIVINGPROJECT_API ADrivingVehiclePawn : public APawn
{
	GENERATED_BODY()

public:
	ADrivingVehiclePawn();

	// ================================================================
	//  元件 / Components
	// ================================================================

	/// 根節點（PathFollower 控制這個的位置/旋轉）
	/// Root scene component — PathFollower moves this
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	TObjectPtr<USceneComponent> RootScene;

	/// 車體 Mesh — 在 BP 裡：
	///   1. 設定 Static Mesh（選你的車模型）
	///   2. 調整 Relative Rotation 讓車頭朝 X+ 方向
	///
	/// Vehicle mesh — in BP Details:
	///   1. Set Static Mesh (your car model)
	///   2. Adjust Relative Rotation so front faces X+
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	TObjectPtr<UStaticMeshComponent> VehicleMesh;

	/// AI 路徑跟隨元件 / AI path following component
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	TObjectPtr<URoadPathFollowerComponent> PathFollower;

	/// 攝影機臂 — 在 BP 裡可調：
	///   - Target Arm Length（距離）
	///   - Relative Rotation 的 Pitch（越負越高，例如 -30 = 更高視角）
	///   - Camera Lag Speed（平滑速度）
	///
	/// Camera boom — adjust in BP:
	///   - Target Arm Length (distance)
	///   - Relative Rotation Pitch (more negative = higher, e.g. -30)
	///   - Camera Lag Speed (smoothing)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle|Camera")
	TObjectPtr<USpringArmComponent> CameraBoom;

	/// 跟隨攝影機 / Follow camera
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle|Camera")
	TObjectPtr<UCameraComponent> FollowCamera;
};
