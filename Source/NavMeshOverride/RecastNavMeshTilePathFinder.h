// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine.h"
#include "AI/Navigation/RecastNavMesh.h"
#include "RecastNavMeshTilePathFinder.generated.h"

/**
*
*/

const static int n = 32; // horizontal size of the map
const static int m = 32; // vertical size size of the map
const static int dir = 4;
const static int dx[dir] = { 1, 0, -1, 0 };
const static int dy[dir] = { 0, 1, 0, -1 };

UCLASS()
class NAVMESHOVERRIDE_API ARecastNavMeshTilePathFinder : public ARecastNavMesh
{
	GENERATED_BODY()

		ARecastNavMeshTilePathFinder(const FObjectInitializer& ObjectInitializer);

	static FPathFindingResult FindPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query);

};
