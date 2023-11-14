#include "DetourNavMeshQuery.h"

dtStatus dtNavMeshQuery_init(dtNavMeshQuery* query, const dtNavMesh* nav, int maxNodes) 
{
	return query->init(nav, maxNodes);
}

dtStatus dtNavMeshQuery_findPath(dtNavMeshQuery* query, dtPolyRef startRef, dtPolyRef endRef,
	const float* startPos, const float* endPos, const dtQueryFilter* filter,
	dtPolyRef* path, int* pathCount, int maxPath) 
{
	return query->findPath(startRef, endRef, startPos, endPos, filter, path, pathCount, maxPath);
}

dtStatus dtNavMeshQuery_findStraightPath(dtNavMeshQuery* query, const float* startPos, const float* endPos,
	const dtPolyRef* path, int pathSize, float* straightPath,
	unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
	int* straightPathCount, int maxStraightPath, int options) 
{
	return query->findStraightPath(startPos, endPos, path, pathSize, straightPath, straightPathFlags,
		straightPathRefs, straightPathCount, maxStraightPath, options);
}

dtStatus dtNavMeshQuery_initSlicedFindPath(dtNavMeshQuery* query, dtPolyRef startRef, dtPolyRef endRef,
	const float* startPos, const float* endPos,
	const dtQueryFilter* filter, unsigned int options) 
{
	return query->initSlicedFindPath(startRef, endRef, startPos, endPos, filter, options);
}

dtStatus dtNavMeshQuery_updateSlicedFindPath(dtNavMeshQuery* query, int maxIter, int* doneIters) 
{
	return query->updateSlicedFindPath(maxIter, doneIters);
}

dtStatus dtNavMeshQuery_finalizeSlicedFindPath(dtNavMeshQuery* query, dtPolyRef* path, int* pathCount, int maxPath) 
{
	return query->finalizeSlicedFindPath(path, pathCount, maxPath);
}

dtStatus dtNavMeshQuery_finalizeSlicedFindPathPartial(dtNavMeshQuery* query, const dtPolyRef* existing, int existingSize,
	dtPolyRef* path, int* pathCount, int maxPath) 
{
	return query->finalizeSlicedFindPathPartial(existing, existingSize, path, pathCount, maxPath);
}

dtStatus dtNavMeshQuery_findPolysAroundCircle(dtNavMeshQuery* query, dtPolyRef startRef, const float* centerPos, float radius,
	const dtQueryFilter* filter, dtPolyRef* resultRef, dtPolyRef* resultParent,
	float* resultCost, int* resultCount, int maxResult) 
{
	return query->findPolysAroundCircle(startRef, centerPos, radius, filter, resultRef, resultParent, resultCost,
		resultCount, maxResult);
}

dtStatus dtNavMeshQuery_findPolysAroundShape(dtNavMeshQuery* query, dtPolyRef startRef, const float* verts, int nverts,
	const dtQueryFilter* filter, dtPolyRef* resultRef, dtPolyRef* resultParent,
	float* resultCost, int* resultCount, int maxResult) 
{
	return query->findPolysAroundShape(startRef, verts, nverts, filter, resultRef, resultParent, resultCost,
		resultCount, maxResult);
}

dtStatus dtNavMeshQuery_getPathFromDijkstraSearch(dtNavMeshQuery* query, dtPolyRef endRef, dtPolyRef* path,
	int* pathCount, int maxPath) 
{
	return query->getPathFromDijkstraSearch(endRef, path, pathCount, maxPath);
}

dtStatus dtNavMeshQuery_findNearestPoly(dtNavMeshQuery* query, const float* center, const float* halfExtents,
	const dtQueryFilter* filter, dtPolyRef* nearestRef, float* nearestPt) 
{
	return query->findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt);
}

dtStatus dtNavMeshQuery_findNearestPoly2(dtNavMeshQuery* query, const float* center, const float* halfExtents,
	const dtQueryFilter* filter, dtPolyRef* nearestRef, float* nearestPt, bool* isOverPoly) 
{
	return query->findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt, isOverPoly);
}

dtStatus dtNavMeshQuery_queryPolygons(dtNavMeshQuery* query, const float* center, const float* halfExtents,
	const dtQueryFilter* filter, dtPolyRef* polys, int* polyCount, const int maxPolys) 
{
	return query->queryPolygons(center, halfExtents, filter, polys, polyCount, maxPolys);
}

dtStatus dtNavMeshQuery_queryPolygons2(dtNavMeshQuery* navMeshQuery, const float* center, const float* halfExtents,
	const dtQueryFilter* filter, dtPolyQuery* query) 
{
	return navMeshQuery->queryPolygons(center, halfExtents, filter, query);
}


dtStatus dtNavMeshQuery_findLocalNeighbourhood(dtNavMeshQuery* query, dtPolyRef startRef, const float* centerPos, float radius,
	const dtQueryFilter* filter, dtPolyRef* resultRef, dtPolyRef* resultParent,
	int* resultCount, const int maxResult) 
{
	return query->findLocalNeighbourhood(startRef, centerPos, radius, filter, resultRef, resultParent, resultCount, maxResult);
}

dtStatus dtNavMeshQuery_moveAlongSurface(dtNavMeshQuery* query, dtPolyRef startRef, const float* startPos, const float* endPos,
	const dtQueryFilter* filter, float* resultPos, dtPolyRef* visited,
	int* visitedCount, const int maxVisitedSize) 
{
	return query->moveAlongSurface(startRef, startPos, endPos, filter, resultPos, visited, visitedCount, maxVisitedSize);
}

dtStatus dtNavMeshQuery_raycast(dtNavMeshQuery* query, dtPolyRef startRef, const float* startPos, const float* endPos,
	const dtQueryFilter* filter, float* t, float* hitNormal, dtPolyRef* path,
	int* pathCount, const int maxPath) 
{
	return query->raycast(startRef, startPos, endPos, filter, t, hitNormal, path, pathCount, maxPath);
}

dtStatus dtNavMeshQuery_raycast2(dtNavMeshQuery* query, dtPolyRef startRef, const float* startPos, const float* endPos,
	const dtQueryFilter* filter, const unsigned int options, dtRaycastHit* hit,
	dtPolyRef prevRef) 
{
	return query->raycast(startRef, startPos, endPos, filter, options, hit, prevRef);
}

dtStatus dtNavMeshQuery_findDistanceToWall(dtNavMeshQuery* query, dtPolyRef startRef, const float* centerPos, const float maxRadius,
	const dtQueryFilter* filter, float* hitDist, float* hitPos, float* hitNormal) 
{
	return query->findDistanceToWall(startRef, centerPos, maxRadius, filter, hitDist, hitPos, hitNormal);
}

dtStatus dtNavMeshQuery_getPolyWallSegments(dtNavMeshQuery* query, dtPolyRef ref, const dtQueryFilter* filter,
	float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
	const int maxSegments) 
{
	return query->getPolyWallSegments(ref, filter, segmentVerts, segmentRefs, segmentCount, maxSegments);
}

dtStatus dtNavMeshQuery_findRandomPoint(dtNavMeshQuery* query, const dtQueryFilter* filter, float (*frand)(),
	dtPolyRef* randomRef, float* randomPt) 
{
	return query->findRandomPoint(filter, frand, randomRef, randomPt);
}

dtStatus dtNavMeshQuery_findRandomPointAroundCircle(dtNavMeshQuery* query, dtPolyRef startRef, const float* centerPos,
	const float maxRadius, const dtQueryFilter* filter, float (*frand)(),
	dtPolyRef* randomRef, float* randomPt) 
{
	return query->findRandomPointAroundCircle(startRef, centerPos, maxRadius, filter, frand, randomRef, randomPt);
}

dtStatus dtNavMeshQuery_closestPointOnPoly(dtNavMeshQuery* query, dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) 
{
	return query->closestPointOnPoly(ref, pos, closest, posOverPoly);
}

dtStatus dtNavMeshQuery_closestPointOnPolyBoundary(dtNavMeshQuery* query, dtPolyRef ref, const float* pos, float* closest) 
{
	return query->closestPointOnPolyBoundary(ref, pos, closest);
}

dtStatus dtNavMeshQuery_getPolyHeight(dtNavMeshQuery* query, dtPolyRef ref, const float* pos, float* height) 
{
	return query->getPolyHeight(ref, pos, height);
}

bool dtNavMeshQuery_isValidPolyRef(dtNavMeshQuery* query, dtPolyRef ref, const dtQueryFilter* filter) 
{
	return query->isValidPolyRef(ref, filter);
}

bool dtNavMeshQuery_isInClosedList(dtNavMeshQuery* query, dtPolyRef ref) 
{
	return query->isInClosedList(ref);
}

const dtNavMesh* dtNavMeshQuery_getAttachedNavMesh(dtNavMeshQuery* query) 
{
	return query->getAttachedNavMesh();
}