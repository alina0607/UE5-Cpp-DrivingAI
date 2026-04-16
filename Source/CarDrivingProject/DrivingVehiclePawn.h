#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "DrivingVehiclePawn.generated.h"

class URoadPathFollowerComponent;
class USpringArmComponent;
class UCameraComponent;

/// RootScene is moved by PathFollower. VehicleMesh is a child,
/// so you can rotate it freely in BP to fix model orientation.
UCLASS()
class CARDRIVINGPROJECT_API ADrivingVehiclePawn : public APawn
{
	GENERATED_BODY()

public:
	ADrivingVehiclePawn();

	// ================================================================
	//  Components
	// ================================================================

	/// Root scene component — PathFollower moves this
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	TObjectPtr<USceneComponent> RootScene;

	/// Vehicle mesh in BP :
	///   1. Set Static Mesh (your car model)
	///   2. Adjust Relative Rotation so front faces X+
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	TObjectPtr<UStaticMeshComponent> VehicleMesh;

	/// AI path following component
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	TObjectPtr<URoadPathFollowerComponent> PathFollower;

	/// Camera boom — adjust in BP:
	///   - Target Arm Length (distance)
	///   - Relative Rotation Pitch (more negative = higher, e.g. -30)
	///   - Camera Lag Speed (smoothing)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle|Camera")
	TObjectPtr<USpringArmComponent> CameraBoom;

	/// Follow camera
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle|Camera")
	TObjectPtr<UCameraComponent> FollowCamera;
};
