#include <DetourNavMeshQuery.h>
#include <DetourCommon.h>
#include <math.h>
#include <cstring>
#include <new>
#include "Agent.h"

rcdtPath* rcdtAllocPath()
{
	void* mem = dtAlloc(sizeof(rcdtPath), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) rcdtPath;
}

void rcdtFreePath(rcdtPath* path)
{
	if (!path) return;
	path->~rcdtPath();
	dtFree(path);
}

void rcdtFreeBuildResults(rcdtBuildResults* results) {
	if (!results) {
		return; // Safe guard against null pointer
	}

	// Free the internal resources
	if (results->navMesh) {
		dtFreeNavMesh(results->navMesh);
	}

	if (results->navMeshQuery) {
		dtFreeNavMeshQuery(results->navMeshQuery);
	}

	// Finally, free the results structure itself
	delete results;
}

void rcdtGetNextPosition(rcdtBuildResults* buildResults, float* startPos, float* endPos, float* nextPosition)
{
	dtNavMeshQuery* navQuery = buildResults->navMeshQuery;
	dtQueryFilter filter;
	float polyPickExt[3] = { 10, 10, 10 }; // Example extents

	dtPolyRef startRef, endRef;
	float nearestPtDiscard[3]; // Store the nearest point on navmesh
	navQuery->findNearestPoly(startPos, polyPickExt, &filter, &startRef, nearestPtDiscard);
	navQuery->findNearestPoly(endPos, polyPickExt, &filter, &endRef, nearestPtDiscard);

	dtPolyRef path[128]; // Path result
	int pathCount;
	navQuery->findPath(startRef, endRef, startPos, endPos, &filter, path, &pathCount, 128);

	float straightPath[128 * 3];
	unsigned char straightPathFlags[128];
	dtPolyRef straightPathPolys[128];
	int straightPathCount;
	navQuery->findStraightPath(startPos, endPos, path, pathCount, straightPath, straightPathFlags, straightPathPolys, &straightPathCount, 128);

	if (straightPathCount > 1) {
		nextPosition[0] = straightPath[3];
		nextPosition[1] = straightPath[4];
		nextPosition[2] = straightPath[5];
	}
	else if (straightPathCount == 1) {
		nextPosition[0] = endPos[0];
		nextPosition[1] = endPos[1];
		nextPosition[2] = endPos[2];
	}
	else {
		nextPosition[0] = startPos[0];
		nextPosition[1] = startPos[1];
		nextPosition[2] = startPos[2];
	}
}

void rcdtFindStraightPath(rcdtBuildResults* buildResults, const float* startPos, const float* endPos, rcdtPath* path)
{
	dtNavMeshQuery* navQuery = buildResults->navMeshQuery;
	dtQueryFilter filter;
	float polyPickExt[3] = { 10, 10, 10 }; // Example extents

	dtPolyRef startRef, endRef;
	float nearestPtDiscard[3]; // Store the nearest point on navmesh
	navQuery->findNearestPoly(startPos, polyPickExt, &filter, &startRef, nearestPtDiscard);
	navQuery->findNearestPoly(endPos, polyPickExt, &filter, &endRef, nearestPtDiscard);
	
	int pathPolyCount = 0;
	dtPolyRef pathPolys[MAX_PATH_POLYS];
	navQuery->findPath(startRef, endRef, startPos, endPos, &filter, pathPolys, &pathPolyCount, MAX_PATH_NODES);

	unsigned char straightPathFlags[MAX_PATH_POLYS];
	navQuery->findStraightPath(startPos, endPos, pathPolys, pathPolyCount, path->nodes, straightPathFlags, path->polys, &path->nnodes, MAX_PATH_NODES);
}

void rcdtFindSmoothPath(rcdtBuildResults* buildResults, const float* startPos, const float* endPos, rcdtPath* path) {
	if (!buildResults->navMeshQuery) return;

	dtNavMeshQuery* navQuery = buildResults->navMeshQuery;

	dtQueryFilter filter;
	dtPolyRef startRef, endRef;
	float nearestStartPos[3], nearestEndPos[3];
	float polyPickExt[3] = { 10, 10, 10 };

	navQuery->findNearestPoly(startPos, polyPickExt, &filter, &startRef, nearestStartPos);
	navQuery->findNearestPoly(endPos, polyPickExt, &filter, &endRef, nearestEndPos);

	navQuery->findPath(startRef, endRef, nearestStartPos, nearestEndPos, &filter, path->polys, &path->npolys, MAX_PATH_POLYS);

	if (path->npolys == 0) return;

	float iterPos[3], targetPos[3];
	bool posOverPoly_dummy;
	navQuery->closestPointOnPoly(startRef, nearestStartPos, iterPos, &posOverPoly_dummy);
	navQuery->closestPointOnPoly(path->polys[path->npolys - 1], nearestEndPos, targetPos, &posOverPoly_dummy);

	const float STEP_SIZE = 0.5f;
	const float SLOP = 0.01f;

	path->nnodes = 0;

	// Insert the start position
	dtVcopy(&path->nodes[path->nnodes * 3], iterPos);
	path->nnodes++;

	while (path->npolys && path->nnodes < MAX_PATH_NODES) {
		float steerPos[3];
		unsigned char steerPosFlag;
		dtPolyRef steerPosRef;

		if (!getSteerTarget(navQuery, iterPos, targetPos, SLOP, path->polys, path->npolys, steerPos, &steerPosFlag, &steerPosRef)) {
			break;
		}

		bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) != 0;
		bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;

		float delta[3];
		dtVsub(delta, steerPos, iterPos);
		float len = rcSqrt(dtVdot(delta, delta));
		if (endOfPath || offMeshConnection) {
			if (len < STEP_SIZE) len = 1;
		}
		else {
			len = STEP_SIZE / len;
		}

		float moveTgt[3];
		dtVmad(moveTgt, iterPos, delta, len);

		float result[3];
		dtPolyRef visited[16];
		int nvisited = 0;
		navQuery->moveAlongSurface(path->polys[0], iterPos, moveTgt, &filter, result, visited, &nvisited, 16);

		path->npolys = fixupCorridor(path->polys, path->npolys, MAX_PATH_POLYS, visited, nvisited);
		path->npolys = fixupShortcuts(path->polys, path->npolys, navQuery);

		float h = 0.0f;
		navQuery->getPolyHeight(path->polys[0], result, &h);
		result[1] = h;

		dtVcopy(iterPos, result);

		if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f)) {
			dtVcopy(iterPos, targetPos);
			if (path->nnodes < MAX_PATH_NODES) {
				dtVcopy(&path->nodes[path->nnodes * 3], iterPos);
				path->nnodes++;
			}
			break;
		}
		else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f)) {
			// Handle off-mesh connection
			// [Handle the off-mesh connection here]
		}

		if (path->nnodes < MAX_PATH_NODES) {
			dtVcopy(&path->nodes[path->nnodes * 3], iterPos);
			path->nnodes++;
		}
	}
}


bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
	float minTargetDist, const dtPolyRef* path, int pathSize,
	float* steerPos, unsigned char* steerPosFlag, dtPolyRef* steerPosRef) {
	// Find steer target
	const int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS * 3];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = 0;
	navQuery->findStraightPath(startPos, endPos, path, pathSize,
		steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);

	if (nsteerPath == 0)
		return false;

	// Find vertex far enough to steer to
	int ns = 0;
	while (ns < nsteerPath) {
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRange(&steerPath[ns * 3], startPos, minTargetDist, 1000.0f))
			break;
		ns++;
	}

	if (ns >= nsteerPath)
		return false;

	dtVcopy(steerPos, &steerPath[ns * 3]);
	*steerPosFlag = steerPathFlags[ns];
	*steerPosRef = steerPathPolys[ns];

	return true;
}

bool inRange(const float* v1, const float* v2, float r, float h) {
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return (dx * dx + dz * dz) < r * r && fabs(dy) < h;
}

int fixupCorridor(dtPolyRef* path, int npath, int maxPath, const dtPolyRef* visited, int nvisited) {
	int furthestPath = -1;
	int furthestVisited = -1;

	// Find furthest common polygon
	for (int i = npath - 1; i >= 0; --i) {
		bool found = false;
		for (int j = nvisited - 1; j >= 0; --j) {
			if (path[i] == visited[j]) {
				furthestPath = i;
				furthestVisited = j;
				found = true;
				break;
			}
		}
		if (found)
			break;
	}

	// If no intersection found, return current path
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;

	// Concatenate paths
	int req = nvisited - furthestVisited;
	int orig = rcMin(furthestPath + 1, npath);
	int size = rcMax(0, npath - orig);
	if (req + size > maxPath)
		size = maxPath - req;

	// Adjust the path
	for (int i = size - 1; i >= 0; --i)
		path[req + i] = path[orig + i];

	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited - 1) - i];

	return req + size;
}

int fixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery) {
	if (npath < 3)
		return npath;

	// Get connected polygons
	const int MAX_NEIS = 16;
	dtPolyRef neis[MAX_NEIS];
	int nneis = 0;

	const dtMeshTile* tile;
	const dtPoly* poly;
	if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
		return npath;

	for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next) {
		const dtLink* link = &tile->links[k];
		if (link->ref != 0) {
			if (nneis < MAX_NEIS)
				neis[nneis++] = link->ref;
		}
	}

	// Check for shortcuts
	const int MAX_LOOK_AHEAD = 6;
	int cut = 0;
	for (int i = rcMin(MAX_LOOK_AHEAD, npath) - 1; i > 1 && cut == 0; --i) {
		for (int j = 0; j < nneis; ++j) {
			if (path[i] == neis[j]) {
				cut = i;
				break;
			}
		}
	}

	if (cut > 1) {
		int offset = cut - 1;
		npath -= offset;
		for (int i = 1; i < npath; ++i)
			path[i] = path[i + offset];
	}

	return npath;
}


rcdtBuildResults* rcdtBuildSoloMesh(rcdtNavMeshConfig* config)
{
	rcdtBuildResults* result = new rcdtBuildResults { nullptr, nullptr, false };

	if (!config) {
		return result; // Invalid input
	}

	rcContext* ctx = new rcContext(false);

	// Prepare the configuration for heightfield
	float bmin[3];
	float bmax[3];
	const float cs = config->cs;
	const float ch = config->ch;
	const float walkableSlopeAngle = config->agentMaxSlope;
	const int walkableHeight = (int)ceilf(config->agentHeight / config->ch);
	const int walkableClimb = (int)floorf(config->agentMaxClimb / config->ch);
	const int walkableRadius = (int)ceilf(config->agentRadius / config->cs);
	const int maxEdgeLen = (int)(config->edgeMaxLen / config->cs);
	const float maxSimplificationError = config->edgeMaxError;
	const int minRegionArea = (int)rcSqr(config->regionMinSize); // Note: area = size*size
	const int mergeRegionArea = (int)rcSqr(config->regionMergeSize);
	const int maxVertsPerPoly = 6;
	const float detailSampleDist = config->detailSampleDist < 0.9f ? 0 : config->cs * config->detailSampleDist;
	const float detailSampleMaxError = config->ch * config->detailSampleMaxError;
	int tileSize = config->tileSize;
	int borderSize = 0;
	int width;
	int height;

	// Step 0: Calc inputs
	rcCalcBounds(config->verts, config->nverts, bmin, bmax);
	rcCalcGridSize(bmin, bmax, cs, &width, &height);


	// Step 1: Initialize build configuration and heightfield
	rcHeightfield* solid = rcAllocHeightfield();
	if (!solid || !rcCreateHeightfield(ctx, *solid, width, height,
		bmin, bmax, cs, ch)) {
		return result; // Failed to create heightfield
	}

	// Step 2: Rasterize input geometry into the heightfield
	const float* verts = config->verts;
	const int* tris = config->tris;
	const int ntris = config->ntris;

	// Allocate an array to hold walkable triangle flags
	unsigned char* triAreas = new unsigned char[ntris];
	memset(triAreas, 0, ntris * sizeof(unsigned char));

	// Mark walkable triangles
	rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, config->nverts, tris, ntris, triAreas);

	// Use the triangle areas as a mask to rasterize the triangles
	rcRasterizeTriangles(ctx, verts, config->nverts, tris, triAreas, ntris, *solid, walkableClimb);
	delete[] triAreas;

	// Step 3: Filter walkable surfaces
	rcFilterLowHangingWalkableObstacles(ctx, walkableClimb, *solid);
	rcFilterLedgeSpans(ctx, walkableHeight, walkableClimb, *solid);
	rcFilterWalkableLowHeightSpans(ctx, walkableHeight, *solid);

	// Step 4: Partition heightfield to create simple layers (using watershed partitioning)
	rcCompactHeightfield* chf = rcAllocCompactHeightfield();
	if (!chf || !rcBuildCompactHeightfield(ctx, walkableHeight, walkableClimb, *solid, *chf)) {
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		return result;
	}
	rcFreeHeightField(solid);

	if (!rcErodeWalkableArea(ctx, walkableRadius, *chf))
	{
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		return result;
	}

	if (!rcBuildDistanceField(ctx, *chf))
	{
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		return result;
	}

	if (!rcBuildRegions(ctx, *chf, borderSize, minRegionArea, mergeRegionArea))
	{
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		return result;
	}

	// Step 5: Create contours from the compact heightfield
	rcContourSet* cset = rcAllocContourSet();
	if (!cset || !rcBuildContours(ctx, *chf, maxSimplificationError, maxEdgeLen, *cset)) {
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		return result;
	}

	// Step 6: Build polygons from contours
	rcPolyMesh* pmesh = rcAllocPolyMesh();
	if (!pmesh || !rcBuildPolyMesh(ctx, *cset, maxVertsPerPoly, *pmesh)) {
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		return result;
	}
	rcFreeContourSet(cset);

	// Update poly flags from areas.
	for (int i = 0; i < pmesh->npolys; ++i)
	{
		if (pmesh->areas[i] == RC_WALKABLE_AREA) {
			pmesh->areas[i] = 0;
			pmesh->flags[i] = 1;
		}
	}

	// Step 7 (Optional): Create the detail mesh
	rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
	if (!dmesh || !rcBuildPolyMeshDetail(ctx, *pmesh, *chf, detailSampleDist, detailSampleMaxError, *dmesh)) {
		rcFreeCompactHeightfield(chf);
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		return result;
	}
	rcFreeCompactHeightfield(chf);

	dtNavMeshCreateParams params = { 0 };
	params.verts = pmesh->verts;
	params.vertCount = pmesh->nverts;
	params.polys = pmesh->polys;
	params.polyAreas = pmesh->areas;
	params.polyFlags = pmesh->flags;
	params.polyCount = pmesh->npolys;
	params.nvp = pmesh->nvp;
	params.detailMeshes = dmesh->meshes;
	params.detailVerts = dmesh->verts;
	params.detailVertsCount = dmesh->nverts;
	params.detailTris = dmesh->tris;
	params.detailTriCount = dmesh->ntris;
	params.walkableHeight = config->agentHeight;
	params.walkableRadius = config->agentRadius;
	params.walkableClimb = config->agentMaxClimb;
	rcVcopy(params.bmin, pmesh->bmin);
	rcVcopy(params.bmax, pmesh->bmax);
	params.cs = cs;
	params.ch = ch;
	params.buildBvTree = true;
	params.tileX = 0;
	params.tileY = 0;

	unsigned char* navData = nullptr;
	int navDataSize;

	if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		dtFree(navData);
		return result;
	}
	rcFreePolyMesh(pmesh);
	rcFreePolyMeshDetail(dmesh);

	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh) {
		return result;
	}

	if (dtStatusFailed(navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA))) {
		dtFreeNavMesh(navMesh);
		return result;
	}
	result->navMesh = navMesh;

	// Set the result
	dtNavMeshQuery* query = dtAllocNavMeshQuery();
	if (dtStatusFailed(query->init(navMesh, 2048))) {
		dtFreeNavMesh(navMesh);
		dtFreeNavMeshQuery(query);
		return result;
	}

	result->navMeshQuery = query;
	result->succeeded = true;
	return result; // Successful completion
}
