#include "DrivingVehiclePawn.h"
#include "RoadPathFollowerComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"

ADrivingVehiclePawn::ADrivingVehiclePawn()
{
	PrimaryActorTick.bCanEverTick = true;

	// ---- 根節點（PathFollower 控制位置/旋轉）----
	// Root scene — PathFollower moves this
	RootScene = CreateDefaultSubobject<USceneComponent>(TEXT("RootScene"));
	SetRootComponent(RootScene);

	// ---- 車體 Mesh（掛在根節點下，BP 裡自由旋轉對齊模型朝向）----
	// Vehicle mesh — child of root, rotate in BP to fix model orientation
	VehicleMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VehicleMesh"));
	VehicleMesh->SetupAttachment(RootScene);
	VehicleMesh->SetCollisionProfileName(TEXT("Vehicle"));
	VehicleMesh->SetSimulatePhysics(false); // AI 直接控制位置，不用物理

	// ---- 路徑跟隨 ----
	PathFollower = CreateDefaultSubobject<URoadPathFollowerComponent>(TEXT("PathFollower"));

	// ---- 攝影機臂（也掛在 RootScene 下）----
	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(RootScene);
	CameraBoom->TargetArmLength = 600.0f;
	CameraBoom->SetRelativeRotation(FRotator(-20.0f, 0.0f, 0.0f));
	CameraBoom->bUsePawnControlRotation = false;  // 跟車頭方向，不跟控制器
	CameraBoom->bDoCollisionTest = true;
	CameraBoom->bEnableCameraLag = true;
	CameraBoom->CameraLagSpeed = 5.0f;
	CameraBoom->bEnableCameraRotationLag = true;
	CameraBoom->CameraRotationLagSpeed = 5.0f;

	// ---- 攝影機 ----
	FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
	FollowCamera->SetupAttachment(CameraBoom);
	FollowCamera->SetFieldOfView(75.0f);

	// 不需要 Controller 旋轉（AI 車不需要）
	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;
	bUseControllerRotationRoll = false;
}
