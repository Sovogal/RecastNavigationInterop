#pragma once
#include "DetourNavMeshQuery.h"

#ifdef DetourExport
#define DETOUR_INT __declspec(dllexport)
#else
#define DETOUR_INT __declspec(dllimport)
#endif

extern "C" {
	/// Initializes the query object.
///  @param[in]		nav			Pointer to the dtNavMesh object to use for all queries.
///  @param[in]		maxNodes	Maximum number of search nodes. [Limits: 0 < value <= 65535]
/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_init(dtNavMeshQuery* query, const dtNavMesh* nav, const int maxNodes);

	/// @name Standard Pathfinding Functions
	/// @{

	/// Finds a path from the start polygon to the end polygon.
	///  @param[in]		startRef	The reference id of the start polygon.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 1]
	DETOUR_INT dtStatus dtNavMeshQuery_findPath(dtNavMeshQuery* query, dtPolyRef startRef, dtPolyRef endRef,
		const float* startPos, const float* endPos,
		const dtQueryFilter* filter,
		dtPolyRef* path, int* pathCount, const int maxPath);

	/// Finds the straight path from the start to the end position within the polygon corridor.
	///  @param[in]		startPos			Path start position. [(x, y, z)]
	///  @param[in]		endPos				Path end position. [(x, y, z)]
	///  @param[in]		path				An array of polygon references that represent the path corridor.
	///  @param[in]		pathSize			The number of polygons in the @p path array.
	///  @param[out]	straightPath		Points describing the straight path. [(x, y, z) * @p straightPathCount].
	///  @param[out]	straightPathFlags	Flags describing each point. (See: #dtStraightPathFlags) [opt]
	///  @param[out]	straightPathRefs	The reference id of the polygon that is being entered at each point. [opt]
	///  @param[out]	straightPathCount	The number of points in the straight path.
	///  @param[in]		maxStraightPath		The maximum number of points the straight path arrays can hold.  [Limit: > 0]
	///  @param[in]		options				Query options. (see: #dtStraightPathOptions)
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findStraightPath(dtNavMeshQuery* query, const float* startPos, const float* endPos,
		const dtPolyRef* path, const int pathSize,
		float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
		int* straightPathCount, const int maxStraightPath, const int options = 0);

	///@}
	/// @name Sliced Pathfinding Functions
	/// Common use case:
	///	-# Call initSlicedFindPath() to initialize the sliced path query.
	///	-# Call updateSlicedFindPath() until it returns complete.
	///	-# Call finalizeSlicedFindPath() to get the path.
	///@{ 

	/// Initializes a sliced path query.
	///  @param[in]		startRef	The reference id of the start polygon.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		options		query options (see: #dtFindPathOptions)
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_initSlicedFindPath(dtNavMeshQuery* query, dtPolyRef startRef, dtPolyRef endRef,
		const float* startPos, const float* endPos,
		const dtQueryFilter* filter, const unsigned int options = 0);

	/// Updates an in-progress sliced path query.
	///  @param[in]		maxIter		The maximum number of iterations to perform.
	///  @param[out]	doneIters	The actual number of iterations completed. [opt]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_updateSlicedFindPath(dtNavMeshQuery* query, const int maxIter, int* doneIters);

	/// Finalizes and returns the results of a sliced path query.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The max number of polygons the path array can hold. [Limit: >= 1]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_finalizeSlicedFindPath(dtNavMeshQuery* query, dtPolyRef* path, int* pathCount, const int maxPath);

	/// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
	/// polygon on the existing path that was visited during the search.
	///  @param[in]		existing		An array of polygon references for the existing path.
	///  @param[in]		existingSize	The number of polygon in the @p existing array.
	///  @param[out]	path			An ordered list of polygon references representing the path. (Start to end.) 
	///  								[(polyRef) * @p pathCount]
	///  @param[out]	pathCount		The number of polygons returned in the @p path array.
	///  @param[in]		maxPath			The max number of polygons the @p path array can hold. [Limit: >= 1]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_finalizeSlicedFindPathPartial(dtNavMeshQuery* query, const dtPolyRef* existing, const int existingSize,
		dtPolyRef* path, int* pathCount, const int maxPath);

	///@}
	/// @name Dijkstra Search Functions
	/// @{ 

	/// Finds the polygons along the navigation graph that touch the specified circle.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		radius			The radius of the search circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the circle. [opt]
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. 
	///  								Zero if a result polygon has no parent. [opt]
	///  @param[out]	resultCost		The search cost from @p centerPos to the polygon. [opt]
	///  @param[out]	resultCount		The number of polygons found. [opt]
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findPolysAroundCircle(dtNavMeshQuery* query, dtPolyRef startRef, const float* centerPos, const float radius,
		const dtQueryFilter* filter,
		dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
		int* resultCount, const int maxResult);

	/// Finds the polygons along the naviation graph that touch the specified convex polygon.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		verts			The vertices describing the convex polygon. (CCW) 
	///  								[(x, y, z) * @p nverts]
	///  @param[in]		nverts			The number of vertices in the polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the search polygon. [opt]
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. Zero if a 
	///  								result polygon has no parent. [opt]
	///  @param[out]	resultCost		The search cost from the centroid point to the polygon. [opt]
	///  @param[out]	resultCount		The number of polygons found.
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findPolysAroundShape(dtNavMeshQuery* query, dtPolyRef startRef, const float* verts, const int nverts,
		const dtQueryFilter* filter,
		dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
		int* resultCount, const int maxResult);

	/// Gets a path from the explored nodes in the previous search.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 0]
	///  @returns		The status flags. Returns DT_FAILURE | DT_INVALID_PARAM if any parameter is wrong, or if
	///  				@p endRef was not explored in the previous search. Returns DT_SUCCESS | DT_BUFFER_TOO_SMALL
	///  				if @p path cannot contain the entire path. In this case it is filled to capacity with a partial path.
	///  				Otherwise returns DT_SUCCESS.
	///  @remarks		The result of this function depends on the state of the query object. For that reason it should only
	///  				be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
	DETOUR_INT dtStatus dtNavMeshQuery_getPathFromDijkstraSearch(dtNavMeshQuery* query, dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath);

	/// @}
	/// @name Local Query Functions
	///@{

	/// Finds the polygon nearest to the specified center point.
	/// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
	///
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents	The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
	///  @param[out]	nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findNearestPoly(dtNavMeshQuery* query, const float* center, const float* halfExtents,
		const dtQueryFilter* filter,
		dtPolyRef* nearestRef, float* nearestPt);

	/// Finds the polygon nearest to the specified center point.
	/// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
	/// 
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents	The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
	///  @param[out]	nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
	///  @param[out]	isOverPoly 	Set to true if the point's X/Z coordinate lies inside the polygon, false otherwise. Unchanged if no polygon is found. [opt]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findNearestPoly2(dtNavMeshQuery* query, const float* center, const float* halfExtents,
		const dtQueryFilter* filter,
		dtPolyRef* nearestRef, float* nearestPt, bool* isOverPoly);

	/// Finds polygons that overlap the search box.
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents		The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	polys		The reference ids of the polygons that overlap the query box.
	///  @param[out]	polyCount	The number of polygons in the search result.
	///  @param[in]		maxPolys	The maximum number of polygons the search result can hold.
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_queryPolygons(dtNavMeshQuery* query, const float* center, const float* halfExtents,
		const dtQueryFilter* filter,
		dtPolyRef* polys, int* polyCount, const int maxPolys);

	/// Finds polygons that overlap the search box.
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents		The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		query		The query. Polygons found will be batched together and passed to this query.
	DETOUR_INT dtStatus dtNavMeshQuery_queryPolygons2(dtNavMeshQuery* navMeshQuery, const float* center, const float* halfExtents,
		const dtQueryFilter* filter, dtPolyQuery* query);

	/// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the query circle. [(x, y, z)]
	///  @param[in]		radius			The radius of the query circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the circle.
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. 
	///  								Zero if a result polygon has no parent. [opt]
	///  @param[out]	resultCount		The number of polygons found.
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findLocalNeighbourhood(dtNavMeshQuery* query, dtPolyRef startRef, const float* centerPos, const float radius,
		const dtQueryFilter* filter,
		dtPolyRef* resultRef, dtPolyRef* resultParent,
		int* resultCount, const int maxResult);

	/// Moves from the start to the end position constrained to the navigation mesh.
	///  @param[in]		startRef		The reference id of the start polygon.
	///  @param[in]		startPos		A position of the mover within the start polygon. [(x, y, x)]
	///  @param[in]		endPos			The desired end position of the mover. [(x, y, z)]
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultPos		The result position of the mover. [(x, y, z)]
	///  @param[out]	visited			The reference ids of the polygons visited during the move.
	///  @param[out]	visitedCount	The number of polygons visited during the move.
	///  @param[in]		maxVisitedSize	The maximum number of polygons the @p visited array can hold.
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_moveAlongSurface(dtNavMeshQuery* query, dtPolyRef startRef, const float* startPos, const float* endPos,
		const dtQueryFilter* filter,
		float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize);

	/// Casts a 'walkability' ray along the surface of the navigation mesh from 
	/// the start position toward the end position.
	/// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
	///  @param[in]		startRef	The reference id of the start polygon.
	///  @param[in]		startPos	A position within the start polygon representing 
	///  							the start of the ray. [(x, y, z)]
	///  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	t			The hit parameter. (FLT_MAX if no wall hit.)
	///  @param[out]	hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	///  @param[out]	path		The reference ids of the visited polygons. [opt]
	///  @param[out]	pathCount	The number of visited polygons. [opt]
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold.
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_raycast(dtNavMeshQuery* query, dtPolyRef startRef, const float* startPos, const float* endPos,
		const dtQueryFilter* filter,
		float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath);

	/// Casts a 'walkability' ray along the surface of the navigation mesh from 
	/// the start position toward the end position.
	///  @param[in]		startRef	The reference id of the start polygon.
	///  @param[in]		startPos	A position within the start polygon representing 
	///  							the start of the ray. [(x, y, z)]
	///  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		options		govern how the raycast behaves. See dtRaycastOptions
	///  @param[out]	hit			Pointer to a raycast hit structure which will be filled by the results.
	///  @param[in]		prevRef		parent of start ref. Used during for cost calculation [opt]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_raycast2(dtNavMeshQuery* query, dtPolyRef startRef, const float* startPos, const float* endPos,
		const dtQueryFilter* filter, const unsigned int options,
		dtRaycastHit* hit, dtPolyRef prevRef = 0);


	/// Finds the distance from the specified position to the nearest polygon wall.
	///  @param[in]		startRef		The reference id of the polygon containing @p centerPos.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		maxRadius		The radius of the search circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	hitDist			The distance to the nearest wall from @p centerPos.
	///  @param[out]	hitPos			The nearest position on the wall that was hit. [(x, y, z)]
	///  @param[out]	hitNormal		The normalized ray formed from the wall point to the 
	///  								source point. [(x, y, z)]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findDistanceToWall(dtNavMeshQuery* query, dtPolyRef startRef, const float* centerPos, const float maxRadius,
		const dtQueryFilter* filter,
		float* hitDist, float* hitPos, float* hitNormal);

	/// Returns the segments for the specified polygon, optionally including portals.
	///  @param[in]		ref				The reference id of the polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	segmentVerts	The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
	///  @param[out]	segmentRefs		The reference ids of each segment's neighbor polygon. 
	///  								Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount] 
	///  @param[out]	segmentCount	The number of segments returned.
	///  @param[in]		maxSegments		The maximum number of segments the result arrays can hold.
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_getPolyWallSegments(dtNavMeshQuery* query, dtPolyRef ref, const dtQueryFilter* filter,
		float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
		const int maxSegments);

	/// Returns random location on navmesh.
	/// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[in]		frand			Function returning a random number [0..1).
	///  @param[out]	randomRef		The reference id of the random location.
	///  @param[out]	randomPt		The random location. 
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findRandomPoint(dtNavMeshQuery* query, const dtQueryFilter* filter, float (*frand)(),
		dtPolyRef* randomRef, float* randomPt);

	/// Returns random location on navmesh within the reach of specified location.
	/// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	/// The location is not exactly constrained by the circle, but it limits the visited polygons.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		maxRadius		The radius of the search circle. [Units: wu]
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[in]		frand			Function returning a random number [0..1).
	///  @param[out]	randomRef		The reference id of the random location.
	///  @param[out]	randomPt		The random location. [(x, y, z)]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_findRandomPointAroundCircle(dtNavMeshQuery* query, dtPolyRef startRef, const float* centerPos, const float maxRadius,
		const dtQueryFilter* filter, float (*frand)(),
		dtPolyRef* randomRef, float* randomPt);

	/// Finds the closest point on the specified polygon.
	///  @param[in]		ref			The reference id of the polygon.
	///  @param[in]		pos			The position to check. [(x, y, z)]
	///  @param[out]	closest		The closest point on the polygon. [(x, y, z)]
	///  @param[out]	posOverPoly	True of the position is over the polygon.
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_closestPointOnPoly(dtNavMeshQuery* query, dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly);

	/// Returns a point on the boundary closest to the source point if the source point is outside the 
	/// polygon's xz-bounds.
	///  @param[in]		ref			The reference id to the polygon.
	///  @param[in]		pos			The position to check. [(x, y, z)]
	///  @param[out]	closest		The closest point. [(x, y, z)]
	/// @returns The status flags for the query.
	DETOUR_INT dtStatus dtNavMeshQuery_closestPointOnPolyBoundary(dtNavMeshQuery* query, dtPolyRef ref, const float* pos, float* closest);

	/// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
	///  @param[in]		ref			The reference id of the polygon.
	///  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
	///  @param[out]	height		The height at the surface of the polygon.
	/// @returns The status flags for the query.
	DETOUR_INT DETOUR_INT dtStatus dtNavMeshQuery_getPolyHeight(dtNavMeshQuery* query, dtPolyRef ref, const float* pos, float* height);

	/// @}
	/// @name Miscellaneous Functions
	/// @{

	/// Returns true if the polygon reference is valid and passes the filter restrictions.
	///  @param[in]		ref			The polygon reference to check.
	///  @param[in]		filter		The filter to apply.
	DETOUR_INT bool dtNavMeshQuery_isValidPolyRef(dtNavMeshQuery* query, dtPolyRef ref, const dtQueryFilter* filter);

	/// Returns true if the polygon reference is in the closed list. 
	///  @param[in]		ref		The reference id of the polygon to check.
	/// @returns True if the polygon is in closed list.
	DETOUR_INT bool dtNavMeshQuery_isInClosedList(dtNavMeshQuery* query, dtPolyRef ref);

	/// Gets the navigation mesh the query object is using.
	/// @return The navigation mesh the query object is using.
	DETOUR_INT const dtNavMesh* dtNavMeshQuery_getAttachedNavMesh(dtNavMeshQuery* query);
}