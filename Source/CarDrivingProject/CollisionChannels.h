// Custom collision channel definitions
#pragma once

#include "Engine/EngineTypes.h"

/// Dedicated obstacle detection channel — used by SphereTrace and red light blockers
/// Registered as ECC_GameTraceChannel1 in DefaultEngine.ini
constexpr ECollisionChannel ECC_ObstacleDetect = ECC_GameTraceChannel1;
