#include "DrivingVehiclePawn.h"
#include "RoadPathFollowerComponent.h"
#include "CollisionChannels.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"

ADrivingVehiclePawn::ADrivingVehiclePawn()
{
	PrimaryActorTick.bCanEverTick = true;

	// Root scene — PathFollower moves this
	RootScene = CreateDefaultSubobject<USceneComponent>(TEXT("RootScene"));
	SetRootComponent(RootScene);

	// Vehicle mesh — child of root, rotate in BP to fix model orientation
	VehicleMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VehicleMesh"));
	VehicleMesh->SetupAttachment(RootScene);
	VehicleMesh->SetCollisionProfileName(TEXT("Vehicle"));
	VehicleMesh->SetSimulatePhysics(false); 
	// Allow other vehicles' SphereTrace (ObstacleDetect) to detect this vehicle
	VehicleMesh->SetCollisionResponseToChannel(ECC_ObstacleDetect, ECR_Block);

	PathFollower = CreateDefaultSubobject<URoadPathFollowerComponent>(TEXT("PathFollower"));

	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(RootScene);
	CameraBoom->TargetArmLength = 600.0f;
	CameraBoom->SetRelativeRotation(FRotator(-20.0f, 0.0f, 0.0f));
	CameraBoom->bUsePawnControlRotation = false;  
	CameraBoom->bDoCollisionTest = true;
	CameraBoom->bEnableCameraLag = true;
	CameraBoom->CameraLagSpeed = 15.0f;            // Higher to reduce position jitter
	CameraBoom->CameraLagMaxDistance = 50.0f;       // Cap max lag distance
	CameraBoom->bEnableCameraRotationLag = true;
	CameraBoom->CameraRotationLagSpeed = 12.0f;     // Higher to reduce rotation jitter

	FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
	FollowCamera->SetupAttachment(CameraBoom);
	FollowCamera->SetFieldOfView(75.0f);

	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;
	bUseControllerRotationRoll = false;
}
