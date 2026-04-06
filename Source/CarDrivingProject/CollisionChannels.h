// 自訂碰撞通道定義 / Custom collision channel definitions
#pragma once

#include "Engine/EngineTypes.h"

/// 障礙物偵測專用通道 — SphereTrace 和紅燈 Blocker 都用這個
/// Dedicated obstacle detection channel — used by SphereTrace and red light blockers
/// 在 DefaultEngine.ini 中註冊為 ECC_GameTraceChannel1
/// Registered as ECC_GameTraceChannel1 in DefaultEngine.ini
constexpr ECollisionChannel ECC_ObstacleDetect = ECC_GameTraceChannel1;
