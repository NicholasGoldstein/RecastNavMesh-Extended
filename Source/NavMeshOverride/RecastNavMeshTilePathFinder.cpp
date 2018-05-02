// Fill out your copyright notice in the Description page of Project Settings.

#include "RecastNavMeshTilePathFinder.h"

DECLARE_CYCLE_STAT(TEXT("Custom Pathfinding"), STAT_Navigation_CustomPathfinding, STATGROUP_Navigation)

class node
{
	// current position
	int xPos;
	int yPos;
	// total distance already traveled to reach the node
	int level;
	// priority=level+remaining distance estimate
	int priority;  // smaller: higher priority

public:
	node(int xp, int yp, int d, int p)
	{
		xPos = xp; yPos = yp; level = d; priority = p;
	}

	int getxPos() const { return xPos; }
	int getyPos() const { return yPos; }
	int getLevel() const { return level; }
	int getPriority() const { return priority; }

	void updatePriority(const int & xDest, const int & yDest)
	{
		priority = level + estimate(xDest, yDest) * 10; //A*
	}

	// give better priority to going strait instead of diagonally
	void nextLevel(const int & i) // i: direction
	{
		level += (dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
	}

	// Estimation function for the remaining distance to the goal.
	const int & estimate(const int & xDest, const int & yDest) const
	{
		static int xd, yd, d;
		xd = xDest - xPos;
		yd = yDest - yPos;

		// Euclidean Distance
		//d = static_cast<int>(sqrt(xd*xd + yd * yd));

		// Manhattan distance
		d = abs(xd) + abs(yd);

		// Chebyshev distance
		//d=max(abs(xd), abs(yd));

		return(d);
	}
};

// Determine priority (in the priority queue)
struct nodePredicate
{
	bool operator()(const node & a, const node & b) const
	{
		return a.getPriority() < b.getPriority();
	}
};

ARecastNavMeshTilePathFinder::ARecastNavMeshTilePathFinder(const FObjectInitializer& ObjectInitializer)
	:ARecastNavMesh(ObjectInitializer)
{
	FindPathImplementation = FindPath;
	
}

FPathFindingResult ARecastNavMeshTilePathFinder::FindPath(const FNavAgentProperties& AgentProperties, const FPathFindingQuery& Query)
{
	SCOPE_CYCLE_COUNTER(STAT_Navigation_CustomPathfinding);
	
	const ANavigationData* Self = Query.NavData.Get();
	check(Cast<const ARecastNavMesh>(Self));

	const ARecastNavMesh* RecastNavMesh = (const ARecastNavMesh*)Self;
	if (Self == NULL)
	{
		return ENavigationQueryResult::Error;
	}

	FPathFindingResult Result(ENavigationQueryResult::Error);

	FNavigationPath* NavPath = Query.PathInstanceToFill.Get();
	FNavMeshPath* NavMeshPath = NavPath ? NavPath->CastPath<FNavMeshPath>() : nullptr;

	if (NavMeshPath)
	{
		Result.Path = Query.PathInstanceToFill;
		NavMeshPath->ResetForRepath();
	}
	else
	{
		Result.Path = Self->CreatePathInstance<FNavMeshPath>(Query);
		NavPath = Result.Path.Get();
		NavMeshPath = NavPath ? NavPath->CastPath<FNavMeshPath>() : nullptr;
	}

	const FNavigationQueryFilter* NavFilter = Query.QueryFilter.Get();
	if (NavMeshPath && NavFilter)
	{
		NavMeshPath->ApplyFlags(Query.NavDataFlags);
		FNavLocation OutLocation;
		const FVector AdjustedEndLocation = NavFilter->GetAdjustedEndLocation(Query.EndLocation);
		if ((Query.StartLocation - AdjustedEndLocation).IsNearlyZero() == true)
		{
			Result.Path->GetPathPoints().Reset();
			Result.Path->GetPathPoints().Add(FNavPathPoint(AdjustedEndLocation));
			Result.Result = ENavigationQueryResult::Success;
		}
		else
		{
			static int map[n][m];
			static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
			static int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
			static int dir_map[n][m]; // map of directions

									  // A-star algorithm.
			FVector PathLocation = Query.EndLocation.GridSnap(200);
			FVector EndTile = (Query.EndLocation.GridSnap(200) - Query.StartLocation.GridSnap(200)) / 200 + FVector(16.f, 16.f, 0.f);

			TArray<node> pq[2]; // list of open (not-yet-tried) nodes
			static int pqi; // pq index
			static node* n0;
			static node* m0;
			static int i, j, x, y, xdx, ydy;
			static char c;
			pqi = 0;

			// reset the node maps
			for (y = 0; y < m; y++)
			{
				for (x = 0; x < n; x++)
				{
					closed_nodes_map[x][y] = 0;
					open_nodes_map[x][y] = 0;
				}
			}

			// create the start node and push into list of open nodes

			n0 = new node(16, 16, 0, 0);
			n0->updatePriority(EndTile.X, EndTile.Y);
			pq[pqi].HeapPush(*n0, nodePredicate());
			open_nodes_map[x][y] = n0->getPriority(); // mark it on the open nodes map

			// A* search
			while (!(pq[pqi].Num() == 0))
			{
				// get the current node w/ the highest priority
				// from the list of open nodes
				n0 = new node(pq[pqi].HeapTop().getxPos(), pq[pqi].HeapTop().getyPos(),
					pq[pqi].HeapTop().getLevel(), pq[pqi].HeapTop().getPriority());

				x = n0->getxPos(); y = n0->getyPos();

				pq[pqi].HeapPopDiscard(nodePredicate(), true); // remove the node from the open list
				open_nodes_map[x][y] = 0;
				// mark it on the closed nodes map
				closed_nodes_map[x][y] = 1;

				// quit searching when the goal state is reached
				if (x == EndTile.X && y == EndTile.Y)
				{
					Result.Path->GetPathPoints().Add(FNavPathPoint(PathLocation));
					// generate the path from finish to start
					// by following the directions
					while (!(x == 16 && y == 16))
					{
						j = dir_map[x][y];
						switch (j) {
						case 0:
							PathLocation += FVector(200, 0, 0);
							break;
						case 1:
							PathLocation += FVector(0, 200, 0);
							break;
						case 2:
							PathLocation += FVector(-200, 0, 0);
							break;
						case 3:
							PathLocation += FVector(0, -200, 0);
							break;
						}
						Result.Path->GetPathPoints().Add(FNavPathPoint(PathLocation));
						x += dx[j];
						y += dy[j];
					}

					// garbage collection
					delete n0;
					// empty the leftover nodes
					while (!(pq[pqi].Num() == 0)) pq[pqi].HeapPopDiscard(nodePredicate(), true);
					Algo::Reverse(Result.Path->GetPathPoints());
					Result.Path->MarkReady();
					Result.Result = ENavigationQueryResult::Success;
					return Result;
				}


				FVector TileStartLocation = FVector(Query.StartLocation.X, Query.StartLocation.Y, 0.f).GridSnap(200.f) + FVector(0.f, 0.f, Query.StartLocation.Z) + FVector(n0->getxPos()-16, n0->getyPos()-16, 0) * 200;
				// generate moves (child nodes) in all possible directions
				for (i = 0; i < dir; i++)
				{
					xdx = x + dx[i]; ydy = y + dy[i];

					// check if tile is within navmesh bounds
					const FVector RelativeTileOffset = FVector(xdx, ydy, 0) - FVector(16, 16, 0);
					const FVector TileLocation = RelativeTileOffset.X * FVector(200.f, 0.f, 0.f) + RelativeTileOffset.Y * FVector(0.f, 200.f, 0.f) + FVector(Query.StartLocation.X, Query.StartLocation.Y, 0.f).GridSnap(200.f) + FVector(0.f, 0.f, Query.StartLocation.Z);
					FVector HitLocation;
					bool Reachable = Cast<ARecastNavMesh>(Query.NavData.Get())->NavMeshRaycast(Query.NavData.Get(), TileStartLocation, TileLocation, HitLocation, Query.QueryFilter, Query.Owner.Get());
					// Debugging: view tiles traversed (Warning: Lags terribly when trying to path to unreachable location)
					/* 
					if (Reachable)
					{
						DrawDebugBox(Query.NavData->GetWorld(), TileLocation, FVector(50.f,50.f, 250.f), FColor::Red, false, 0.5f);
					}
					else
					{
						DrawDebugBox(Query.NavData->GetWorld(), TileLocation, FVector(50.f, 50.f, 250.f) / 2.f, FColor::Green, false, 0.5f);
					}
					*/
					if (!(xdx<0 || xdx>n - 1 || ydy<0 || ydy>m - 1 || map[xdx][ydy] == 1
						|| closed_nodes_map[xdx][ydy] == 1 ||Reachable))
					{
						// generate a child node
						m0 = new node(xdx, ydy, n0->getLevel(),
							n0->getPriority());
						m0->nextLevel(i);
						m0->updatePriority(EndTile.X, EndTile.Y);

						// if it is not in the open list then add into that
						if (open_nodes_map[xdx][ydy] == 0)
						{
							open_nodes_map[xdx][ydy] = m0->getPriority();
							pq[pqi].HeapPush(*m0, nodePredicate());
							// mark its parent node direction
							dir_map[xdx][ydy] = (i + dir / 2) % dir;
						}
						else if (open_nodes_map[xdx][ydy] > m0->getPriority())
						{
							// update the priority info
							open_nodes_map[xdx][ydy] = m0->getPriority();
							// update the parent direction info
							dir_map[xdx][ydy] = (i + dir / 2) % dir;

							// replace the node
							// by emptying one pq to the other one
							// except the node to be replaced will be ignored
							// and the new node will be pushed in instead
							while (!(pq[pqi].HeapTop().getxPos() == xdx &&
								pq[pqi].HeapTop().getyPos() == ydy))
							{
								pq[1 - pqi].HeapPush(pq[pqi].HeapTop(), nodePredicate());
								pq[pqi].HeapPopDiscard(nodePredicate(), true);
							}
							pq[pqi].HeapPopDiscard(nodePredicate(), true); // remove the wanted node

																		   // empty the larger size pq to the smaller one
							if (pq[pqi].Num() > pq[1 - pqi].Num()) pqi = 1 - pqi;
							while (!(pq[pqi].Num() == 0))
							{
								pq[1 - pqi].HeapPush(pq[pqi].HeapTop(), nodePredicate());
								pq[pqi].HeapPopDiscard(nodePredicate(), true);
							}
							pqi = 1 - pqi;
							pq[pqi].HeapPush(*m0, nodePredicate()); // add the better node instead
						}
						else delete m0; // garbage collection
					}
				}
				delete n0; // garbage collection
			}
			return Result; // no route found
		}

		const bool bPartialPath = Result.IsPartial();
		if (bPartialPath)
		{
			Result.Result = Query.bAllowPartialPaths ? ENavigationQueryResult::Success : ENavigationQueryResult::Fail;
		}
	}

	return Result;
}


