// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#include "NavMeshOverrideGameMode.h"
#include "NavMeshOverridePlayerController.h"
#include "NavMeshOverrideCharacter.h"
#include "UObject/ConstructorHelpers.h"

ANavMeshOverrideGameMode::ANavMeshOverrideGameMode()
{
	// use our custom PlayerController class
	PlayerControllerClass = ANavMeshOverridePlayerController::StaticClass();

	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/TopDownCPP/Blueprints/TopDownCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}