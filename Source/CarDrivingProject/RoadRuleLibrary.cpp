// Fill out your copyright notice in the Description page of Project Settings.


#include "RoadRuleLibrary.h"

/// <summary>
/// RoadType comes from Blueprint and is stored in FRoadRuntimeData.
/// Interpret that value here and convert it into a driving rule.
/// </summary>
FRoadDrivingRule URoadRuleLibrary::GetDrivingRuleFromRoadData(const FRoadRuntimeData& RoadData)
{
    return GetDrivingRuleFromRoadType(RoadData.RoadType);
}

FRoadDrivingRule URoadRuleLibrary::GetDrivingRuleFromRoadType(uint8 InRoadType)
{
	FRoadDrivingRule Rule;

    switch (InRoadType)
    {
    case 0:
    {
        // 單車道路：只有一條車道，雙向共用（需要會車讓行）
        // Single-lane road: one lane shared by both directions (yield on encounter)
        Rule.RuleName = TEXT("1-Lane Road");
        Rule.bIsOneWay = false;
        Rule.ForwardLaneCount = 1;
        Rule.ReverseLaneCount = 0;
        Rule.bAllowOvertaking = false;
        Rule.bUseOpposingLaneForPassing = false;
        Rule.bReturnToDefaultLaneAfterPass = true;
        Rule.bPreferLeftLaneForPassing = false;
        Rule.bRightmostLaneIsSlowOrExitLane = false;
        Rule.bImplemented = true;
        break;
    }

    case 1:
    {
        // Two-way road with one lane per direction.
        // Overtaking is allowed by temporarily entering the opposing lane.
        Rule.RuleName = TEXT("2-Lane Road");
        Rule.bIsOneWay = false;
        Rule.ForwardLaneCount = 1;
        Rule.ReverseLaneCount = 1;
        Rule.bAllowOvertaking = true;
        Rule.bUseOpposingLaneForPassing = true;
        Rule.bReturnToDefaultLaneAfterPass = true;
        Rule.bPreferLeftLaneForPassing = true;
        Rule.bRightmostLaneIsSlowOrExitLane = false;
        Rule.bImplemented = true;
        break;
    }

    case 2:
    {
        // 三車道路：正向 2 車道 + 反向 1 車道
        // 中間車道可用於超車（進入對向車道超車）
        // 3-Lane Road: 2 forward lanes + 1 reverse lane
        // Middle lane can be used for overtaking (entering opposing lane)
        //
        // 路面佈局 / Road layout (bTwoRoads=false):
        //   ← 反向 1 車道 ← | → 正向 2 車道 →
        //                    ↑ spline 中心
        //   LaneIndex 0 = 靠中心的快車道（超車道）
        //   LaneIndex 1 = 靠外側的慢車道（預設車道）
        Rule.RuleName = TEXT("3-Lane Road");
        Rule.bIsOneWay = false;
        Rule.ForwardLaneCount = 2;
        Rule.ReverseLaneCount = 1;
        Rule.bAllowOvertaking = true;
        Rule.bUseOpposingLaneForPassing = false;
        Rule.bReturnToDefaultLaneAfterPass = true;
        Rule.bPreferLeftLaneForPassing = true;
        Rule.bRightmostLaneIsSlowOrExitLane = false;
        Rule.bImplemented = true;
        break;
    }

    case 3:
    {
        // 四車道路：正向 2 車道 + 反向 2 車道
        // 在自己方向的車道內超車（不跨越中線）
        // 4-Lane Road: 2 forward lanes + 2 reverse lanes
        // Overtake within your own direction lanes (don't cross center line)
        //
        // 路面佈局 / Road layout (bTwoRoads=false):
        //   ← 反向 2 車道 ← | → 正向 2 車道 →
        //                    ↑ spline 中心
        //   LaneIndex 0 = 靠中心的快車道（超車道）
        //   LaneIndex 1 = 靠外側的慢車道（預設車道）
        Rule.RuleName = TEXT("4-Lane Road");
        Rule.bIsOneWay = false;
        Rule.ForwardLaneCount = 2;
        Rule.ReverseLaneCount = 2;
        Rule.bAllowOvertaking = true;
        Rule.bUseOpposingLaneForPassing = false;
        Rule.bReturnToDefaultLaneAfterPass = true;
        Rule.bPreferLeftLaneForPassing = true;
        Rule.bRightmostLaneIsSlowOrExitLane = true;
        Rule.bImplemented = true;
        break;
    }

    case 4:
    {
        // Divided two-way road with two lanes per direction.
        // Overtaking happens within the same direction using the left lane.
        Rule.RuleName = TEXT("2+2-Lane Wide Road");
        Rule.bIsOneWay = false;
        Rule.ForwardLaneCount = 2;
        Rule.ReverseLaneCount = 2;
        Rule.bAllowOvertaking = true;
        Rule.bUseOpposingLaneForPassing = false;
        Rule.bReturnToDefaultLaneAfterPass = true;
        Rule.bPreferLeftLaneForPassing = true;
        Rule.bRightmostLaneIsSlowOrExitLane = true;
        Rule.bImplemented = true;
        break;
    }

    case 5:
    {
        // Overtaking is disabled by default.
        Rule.RuleName = TEXT("Tunnel");
        Rule.bIsOneWay = false;
        Rule.ForwardLaneCount = 1;
        Rule.ReverseLaneCount = 1;
        Rule.bAllowOvertaking = false;
        Rule.bUseOpposingLaneForPassing = false;
        Rule.bReturnToDefaultLaneAfterPass = true;
        Rule.bPreferLeftLaneForPassing = false;
        Rule.bRightmostLaneIsSlowOrExitLane = false;
        Rule.bImplemented = true;
        break;
    }

    case 6:
    {
        // 單車道橋：只有一條車道，不可超車
        // Single-lane bridge: one lane, no overtaking
        Rule.RuleName = TEXT("Bridge 1-Lane");
        Rule.bIsOneWay = false;
        Rule.ForwardLaneCount = 1;
        Rule.ReverseLaneCount = 0;
        Rule.bAllowOvertaking = false;
        Rule.bUseOpposingLaneForPassing = false;
        Rule.bReturnToDefaultLaneAfterPass = true;
        Rule.bPreferLeftLaneForPassing = false;
        Rule.bRightmostLaneIsSlowOrExitLane = false;
        Rule.bImplemented = true;
        break;
    }

    case 7:
    {
        // Overtaking is disabled by default.
        Rule.RuleName = TEXT("Bridge 2-Lane");
        Rule.bIsOneWay = false;
        Rule.ForwardLaneCount = 1;
        Rule.ReverseLaneCount = 1;
        Rule.bAllowOvertaking = false;
        Rule.bUseOpposingLaneForPassing = false;
        Rule.bReturnToDefaultLaneAfterPass = true;
        Rule.bPreferLeftLaneForPassing = false;
        Rule.bRightmostLaneIsSlowOrExitLane = false;
        Rule.bImplemented = true;
        break;
    }

    default:
    {
        // Unknown road type from Blueprint.
        Rule.RuleName = TEXT("Unknown Road Type");
        Rule.bImplemented = false;
        break;
    }
    }


	return Rule;
}
