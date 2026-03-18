// Fill out your copyright notice in the Description page of Project Settings.


#include "RoadRuleLibrary.h"

/// <summary>
/// RoadType comes from Blueprint and is stored in FRoadRuntimeData.
/// Interpret that value here and convert it into a driving rule.
/// </summary>
FRoadDrivingRule URoadRuleLibrary::GetDrivingRuleFromRoadData(const FRoadRuntimeData& RoadData)
{
	FRoadDrivingRule Rule;

    switch (RoadData.RoadType)
    {
    case 0:
    {
        // Reserved for future implementation.
        Rule.RuleName = TEXT("1-Lane Road");
        Rule.bImplemented = false;
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
        // Reserved for future implementation.
        Rule.RuleName = TEXT("3-Lane Road");
        Rule.bImplemented = false;
        break;
    }

    case 3:
    {
        // Reserved for future implementation.
        Rule.RuleName = TEXT("4-Lane Road");
        Rule.bImplemented = false;
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
        // Reserved for future implementation.
        Rule.RuleName = TEXT("Bridge 1-Lane");
        Rule.bImplemented = false;
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
