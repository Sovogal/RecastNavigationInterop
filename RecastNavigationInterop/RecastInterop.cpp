#include "RecastInterop.h"
#include "Recast.h"

/// @name Allocation Functions
/// Functions used to allocate and de-allocate Recast objects.
/// @see rcAllocSetCustom
/// @{

/// Allocates a heightfield object using the Recast allocator.
///  @return A heightfield that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcCreateHeightfield, rcFreeHeightField
rcHeightfield* rcAllocHeightfieldWrapper() { return rcAllocHeightfield(); }

/// Frees the specified heightfield object using the Recast allocator.
///  @param[in]		hf	A heightfield allocated using #rcAllocHeightfield
///  @ingroup recast
///  @see rcAllocHeightfield
void rcFreeHeightFieldWrapper(rcHeightfield* hf) { rcFreeHeightField(hf); }

/// Allocates a compact heightfield object using the Recast allocator.
///  @return A compact heightfield that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildCompactHeightfield, rcFreeCompactHeightfield
rcCompactHeightfield* rcAllocCompactHeightfieldWrapper() { return rcAllocCompactHeightfield(); }

/// Frees the specified compact heightfield object using the Recast allocator.
///  @param[in]		chf		A compact heightfield allocated using #rcAllocCompactHeightfield
///  @ingroup recast
///  @see rcAllocCompactHeightfield
void rcFreeCompactHeightfieldWrapper(rcCompactHeightfield* chf) { rcFreeCompactHeightfield(chf); }

/// Allocates a heightfield layer set using the Recast allocator.
///  @return A heightfield layer set that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildHeightfieldLayers, rcFreeHeightfieldLayerSet
rcHeightfieldLayerSet* rcAllocHeightfieldLayerSetWrapper() { return rcAllocHeightfieldLayerSet(); }

/// Frees the specified heightfield layer set using the Recast allocator.
///  @param[in]		lset	A heightfield layer set allocated using #rcAllocHeightfieldLayerSet
///  @ingroup recast
///  @see rcAllocHeightfieldLayerSet
void rcFreeHeightfieldLayerSetWrapper(rcHeightfieldLayerSet* lset) { rcFreeHeightfieldLayerSet(lset); }

/// Allocates a contour set object using the Recast allocator.
///  @return A contour set that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildContours, rcFreeContourSet
rcContourSet* rcAllocContourSetWrapper() { return rcAllocContourSet(); }

/// Frees the specified contour set using the Recast allocator.
///  @param[in]		cset	A contour set allocated using #rcAllocContourSet
///  @ingroup recast
///  @see rcAllocContourSet
void rcFreeContourSetWrapper(rcContourSet* cset) { rcFreeContourSet(cset); }

/// Allocates a polygon mesh object using the Recast allocator.
///  @return A polygon mesh that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildPolyMesh, rcFreePolyMesh
rcPolyMesh* rcAllocPolyMeshWrapper() { return rcAllocPolyMesh(); }

/// Frees the specified polygon mesh using the Recast allocator.
///  @param[in]		pmesh	A polygon mesh allocated using #rcAllocPolyMesh
///  @ingroup recast
///  @see rcAllocPolyMesh
void rcFreePolyMeshWrapper(rcPolyMesh* pmesh) { rcFreePolyMesh(pmesh); }

/// Allocates a detail mesh object using the Recast allocator.
///  @return A detail mesh that is ready for initialization, or null on failure.
///  @ingroup recast
///  @see rcBuildPolyMeshDetail, rcFreePolyMeshDetail
rcPolyMeshDetail* rcAllocPolyMeshDetailWrapper() { return rcAllocPolyMeshDetail(); }

/// Frees the specified detail mesh using the Recast allocator.
///  @param[in]		dmesh	A detail mesh allocated using #rcAllocPolyMeshDetail
///  @ingroup recast
///  @see rcAllocPolyMeshDetail
void rcFreePolyMeshDetailWrapper(rcPolyMeshDetail* dmesh) { rcFreePolyMeshDetail(dmesh); }

/// Calculates the bounding box of an array of vertices.
///  @ingroup recast
///  @param[in]		verts	An array of vertices. [(x, y, z) * @p nv]
///  @param[in]		nv		The number of vertices in the @p verts array.
///  @param[out]	bmin	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[out]	bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
void rcCalcBoundsWrapper(const float* verts, int nv, float* bmin, float* bmax) { rcCalcBounds(verts, nv, bmin, bmax); }

/// Calculates the grid size based on the bounding box and grid cell size.
///  @ingroup recast
///  @param[in]		bmin	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[in]		bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
///  @param[in]		cs		The xz-plane cell size. [Limit: > 0] [Units: wu]
///  @param[out]	w		The width along the x-axis. [Limit: >= 0] [Units: vx]
///  @param[out]	h		The height along the z-axis. [Limit: >= 0] [Units: vx]
void rcCalcGridSizeWrapper(const float* bmin, const float* bmax, float cs, int* w, int* h) { rcCalcGridSize(bmin, bmax, cs, w, h); }

/// Initializes a new heightfield.
///  @ingroup recast
///  @param[in,out]	hf		The allocated heightfield to initialize.
///  @param[in]		width	The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
///  @param[in]		height	The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
///  @param[in]		bmin	The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
///  @param[in]		bmax	The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
///  @param[in]		cs		The xz-plane cell size to use for the field. [Limit: > 0] [Units: wu]
///  @param[in]		ch		The y-axis cell size to use for field. [Limit: > 0] [Units: wu]
///  @returns True if the operation completed successfully.
bool rcCreateHeightfieldWrapper(rcHeightfield& hf, int width, int height,
	const float* bmin, const float* bmax,
	float cs, float ch)
{
	rcContext* ctx = new rcContext(false);
	return rcCreateHeightfield(ctx, hf, width, height, bmin, bmax, cs, ch);
}

/// Sets the area id of all triangles with a slope below the specified value
/// to #RC_WALKABLE_AREA.
///  @ingroup recast
///  @param[in,out]	ctx					The build context to use during the operation.
///  @param[in]		walkableSlopeAngle	The maximum slope that is considered walkable.
///  									[Limits: 0 <= value < 90] [Units: Degrees]
///  @param[in]		verts				The vertices. [(x, y, z) * @p nv]
///  @param[in]		nv					The number of vertices.
///  @param[in]		tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]		nt					The number of triangles.
///  @param[out]	areas				The triangle area ids. [Length: >= @p nt]
void rcMarkWalkableTrianglesWrapper(const float walkableSlopeAngle, const float* verts, int nv,
	const int* tris, int nt, unsigned char* areas)
{
	rcContext* ctx = new rcContext(false);
	rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, tris, nt, areas);
}

/// Sets the area id of all triangles with a slope greater than or equal to the specified value to #RC_NULL_AREA.
///  @ingroup recast
///  @param[in,out]	ctx					The build context to use during the operation.
///  @param[in]		walkableSlopeAngle	The maximum slope that is considered walkable.
///  									[Limits: 0 <= value < 90] [Units: Degrees]
///  @param[in]		verts				The vertices. [(x, y, z) * @p nv]
///  @param[in]		nv					The number of vertices.
///  @param[in]		tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]		nt					The number of triangles.
///  @param[out]	areas				The triangle area ids. [Length: >= @p nt]
void rcClearUnwalkableTrianglesWrapper(const float walkableSlopeAngle, const float* verts, int nv,
	const int* tris, int nt, unsigned char* areas)
{
	rcContext* ctx = new rcContext(false);
	rcClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, tris, nt, areas);
}

/// Adds a span to the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in,out]	hf				An initialized heightfield.
///  @param[in]		x				The width index where the span is to be added.
///  								[Limits: 0 <= value < rcHeightfield::width]
///  @param[in]		y				The height index where the span is to be added.
///  								[Limits: 0 <= value < rcHeightfield::height]
///  @param[in]		smin			The minimum height of the span. [Limit: < @p smax] [Units: vx]
///  @param[in]		smax			The maximum height of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT] [Units: vx]
///  @param[in]		area			The area id of the span. [Limit: <= #RC_WALKABLE_AREA)
///  @param[in]		flagMergeThr	The merge theshold. [Limit: >= 0] [Units: vx]
///  @returns True if the operation completed successfully.
bool rcAddSpanWrapper(rcHeightfield& hf, const int x, const int y,
	const unsigned short smin, const unsigned short smax,
	const unsigned char area, const int flagMergeThr)
{
	rcContext* ctx = new rcContext(false);
	return rcAddSpan(ctx, hf, x, y, smin, smax, area, flagMergeThr);
}

/// Rasterizes a triangle into the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		v0				Triangle vertex 0 [(x, y, z)]
///  @param[in]		v1				Triangle vertex 1 [(x, y, z)]
///  @param[in]		v2				Triangle vertex 2 [(x, y, z)]
///  @param[in]		area			The area id of the triangle. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]	solid			An initialized heightfield.
///  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
///  								[Limit: >= 0] [Units: vx]
///  @returns True if the operation completed successfully.
bool rcRasterizeTriangleWrapper(const float* v0, const float* v1, const float* v2,
	const unsigned char area, rcHeightfield& solid,
	const int flagMergeThr)
{
	rcContext* ctx = new rcContext(false);
	return rcRasterizeTriangle(ctx, v0, v1, v2, area, solid, flagMergeThr);
}

/// Rasterizes an indexed triangle mesh into the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		verts			The vertices. [(x, y, z) * @p nv]
///  @param[in]		nv				The number of vertices.
///  @param[in]		tris			The triangle indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]		areas			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]		nt				The number of triangles.
///  @param[in,out]	solid			An initialized heightfield.
///  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag. 
///  								[Limit: >= 0] [Units: vx]
///  @returns True if the operation completed successfully.
bool rcRasterizeTrianglesWrapper(const float* verts, const int nv,
	const int* tris, const unsigned char* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr)
{
	rcContext* ctx = new rcContext(false);
	return rcRasterizeTriangles(ctx, verts, nv, tris, areas, nt, solid, flagMergeThr);
}

/// Rasterizes an indexed triangle mesh into the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx			The build context to use during the operation.
///  @param[in]		verts		The vertices. [(x, y, z) * @p nv]
///  @param[in]		nv			The number of vertices.
///  @param[in]		tris		The triangle indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]		areas		The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]		nt			The number of triangles.
///  @param[in,out]	solid		An initialized heightfield.
///  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag. 
///  							[Limit: >= 0] [Units: vx]
///  @returns True if the operation completed successfully.
bool rcRasterizeTriangles2Wrapper(const float* verts, const int nv,
	const unsigned short* tris, const unsigned char* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr)
{
	rcContext* ctx = new rcContext(false);
	return rcRasterizeTriangles(ctx, verts, nv, tris, areas, nt, solid, flagMergeThr);
}


/// Rasterizes triangles into the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		verts			The triangle vertices. [(ax, ay, az, bx, by, bz, cx, by, cx) * @p nt]
///  @param[in]		areas			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]		nt				The number of triangles.
///  @param[in,out]	solid			An initialized heightfield.
///  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag. 
///  								[Limit: >= 0] [Units: vx]
///  @returns True if the operation completed successfully.
bool rcRasterizeTriangles3Wrapper(const float* verts, const unsigned char* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr)
{
	rcContext* ctx = new rcContext(false);
	return rcRasterizeTriangles(ctx, verts, areas, nt, solid, flagMergeThr);
}

/// Marks non-walkable spans as walkable if their maximum is within @p walkableClimp of a walkable neighbor. 
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		walkableClimb	Maximum ledge height that is considered to still be traversable. 
///  								[Limit: >=0] [Units: vx]
///  @param[in,out]	solid			A fully built heightfield.  (All spans have been added.)
void rcFilterLowHangingWalkableObstaclesWrapper(const int walkableClimb, rcHeightfield& solid)
{
	rcContext* ctx = new rcContext(false);
	rcFilterLowHangingWalkableObstacles(ctx, walkableClimb, solid);
}

/// Marks spans that are ledges as not-walkable. 
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area to 
///  								be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[in]		walkableClimb	Maximum ledge height that is considered to still be traversable. 
///  								[Limit: >=0] [Units: vx]
///  @param[in,out]	solid			A fully built heightfield.  (All spans have been added.)
void rcFilterLedgeSpansWrapper(const int walkableHeight,
	const int walkableClimb, rcHeightfield& solid)
{
	rcContext* ctx = new rcContext(false);
	rcFilterLedgeSpans(ctx, walkableHeight, walkableClimb, solid);
}

/// Marks walkable spans as not walkable if the clearence above the span is less than the specified height. 
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area to 
///  								be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[in,out]	solid			A fully built heightfield.  (All spans have been added.)
void rcFilterWalkableLowHeightSpansWrapper(int walkableHeight, rcHeightfield& solid)
{
	rcContext* ctx = new rcContext(false);
	rcFilterWalkableLowHeightSpans(ctx, walkableHeight, solid);
}

/// Returns the number of spans contained in the specified heightfield.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		hf		An initialized heightfield.
///  @returns The number of spans in the heightfield.
int rcGetHeightFieldSpanCountWrapper(rcHeightfield& hf)
{
	rcContext* ctx = new rcContext(false);
	return rcGetHeightFieldSpanCount(ctx, hf);
}

/// @}
/// @name Compact Heightfield Functions
/// @see rcCompactHeightfield
/// @{

/// Builds a compact heightfield representing open space, from a heightfield representing solid space.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area 
///  								to be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[in]		walkableClimb	Maximum ledge height that is considered to still be traversable. 
///  								[Limit: >=0] [Units: vx]
///  @param[in]		hf				The heightfield to be compacted.
///  @param[out]	chf				The resulting compact heightfield. (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
bool rcBuildCompactHeightfieldWrapper(const int walkableHeight, const int walkableClimb,
	rcHeightfield& hf, rcCompactHeightfield& chf)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildCompactHeightfield(ctx, walkableHeight, walkableClimb, hf, chf);
}

/// Erodes the walkable area within the heightfield by the specified radius. 
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		radius	The radius of erosion. [Limits: 0 < value < 255] [Units: vx]
///  @param[in,out]	chf		The populated compact heightfield to erode.
///  @returns True if the operation completed successfully.
bool rcErodeWalkableAreaWrapper(int radius, rcCompactHeightfield& chf)
{
	rcContext* ctx = new rcContext(false);
	return rcErodeWalkableArea(ctx, radius, chf);
}

/// Applies a median filter to walkable area types (based on area id), removing noise.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in,out]	chf		A populated compact heightfield.
///  @returns True if the operation completed successfully.
bool rcMedianFilterWalkableAreaWrapper(rcCompactHeightfield& chf)
{
	rcContext* ctx = new rcContext(false);
	return rcMedianFilterWalkableArea(ctx, chf);
}

/// Applies an area id to all spans within the specified bounding box. (AABB) 
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		bmin	The minimum of the bounding box. [(x, y, z)]
///  @param[in]		bmax	The maximum of the bounding box. [(x, y, z)]
///  @param[in]		areaId	The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]	chf		A populated compact heightfield.
void rcMarkBoxAreaWrapper(const float* bmin, const float* bmax, unsigned char areaId,
	rcCompactHeightfield& chf)
{
	rcContext* ctx = new rcContext(false);
	rcMarkBoxArea(ctx, bmin, bmax, areaId, chf);
}

/// Applies the area id to the all spans within the specified convex polygon. 
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		verts	The vertices of the polygon [Fomr: (x, y, z) * @p nverts]
///  @param[in]		nverts	The number of vertices in the polygon.
///  @param[in]		hmin	The height of the base of the polygon.
///  @param[in]		hmax	The height of the top of the polygon.
///  @param[in]		areaId	The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]	chf		A populated compact heightfield.
void rcMarkConvexPolyAreaWrapper(const float* verts, const int nverts,
	const float hmin, const float hmax, unsigned char areaId,
	rcCompactHeightfield& chf)
{
	rcContext* ctx = new rcContext(false);
	rcMarkConvexPolyArea(ctx, verts, nverts, hmin, hmax, areaId, chf);
}

/// Helper function to offset voncex polygons for rcMarkConvexPolyArea.
///  @ingroup recast
///  @param[in]		verts		The vertices of the polygon [Form: (x, y, z) * @p nverts]
///  @param[in]		nverts		The number of vertices in the polygon.
///  @param[out]	outVerts	The offset vertices (should hold up to 2 * @p nverts) [Form: (x, y, z) * return value]
///  @param[in]		maxOutVerts	The max number of vertices that can be stored to @p outVerts.
///  @returns Number of vertices in the offset polygon or 0 if too few vertices in @p outVerts.
int rcOffsetPolyWrapper(const float* verts, const int nverts, const float offset,
	float* outVerts, const int maxOutVerts)
{
	return rcOffsetPoly(verts, nverts, offset, outVerts, maxOutVerts);
}

/// Applies the area id to all spans within the specified cylinder.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		pos		The center of the base of the cylinder. [Form: (x, y, z)] 
///  @param[in]		r		The radius of the cylinder.
///  @param[in]		h		The height of the cylinder.
///  @param[in]		areaId	The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]	chf	A populated compact heightfield.
void rcMarkCylinderAreaWrapper(const float* pos, const float r, const float h, unsigned char areaId,
	rcCompactHeightfield& chf)
{
	rcContext* ctx = new rcContext(false);
	rcMarkCylinderArea(ctx, pos, r, h, areaId, chf);
}

/// Builds the distance field for the specified compact heightfield. 
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in,out]	chf		A populated compact heightfield.
///  @returns True if the operation completed successfully.
bool rcBuildDistanceFieldWrapper(rcCompactHeightfield& chf)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildDistanceField(ctx, chf);
}

/// Builds region data for the heightfield using watershed partitioning.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in,out]	chf				A populated compact heightfield.
///  @param[in]		borderSize		The size of the non-navigable border around the heightfield.
///  								[Limit: >=0] [Units: vx]
///  @param[in]		minRegionArea	The minimum number of cells allowed to form isolated island areas.
///  								[Limit: >=0] [Units: vx].
///  @param[in]		mergeRegionArea		Any regions with a span count smaller than this value will, if possible,
///  								be merged with larger regions. [Limit: >=0] [Units: vx] 
///  @returns True if the operation completed successfully.
bool rcBuildRegionsWrapper(rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildRegions(ctx, chf, borderSize, minRegionArea, mergeRegionArea);
}

/// Builds region data for the heightfield by partitioning the heightfield in non-overlapping layers.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in,out]	chf				A populated compact heightfield.
///  @param[in]		borderSize		The size of the non-navigable border around the heightfield.
///  								[Limit: >=0] [Units: vx]
///  @param[in]		minRegionArea	The minimum number of cells allowed to form isolated island areas.
///  								[Limit: >=0] [Units: vx].
///  @returns True if the operation completed successfully.
bool rcBuildLayerRegionsWrapper(rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildLayerRegions(ctx, chf, borderSize, minRegionArea);
}

/// Builds region data for the heightfield using simple monotone partitioning.
///  @ingroup recast 
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in,out]	chf				A populated compact heightfield.
///  @param[in]		borderSize		The size of the non-navigable border around the heightfield.
///  								[Limit: >=0] [Units: vx]
///  @param[in]		minRegionArea	The minimum number of cells allowed to form isolated island areas.
///  								[Limit: >=0] [Units: vx].
///  @param[in]		mergeRegionArea	Any regions with a span count smaller than this value will, if possible, 
///  								be merged with larger regions. [Limit: >=0] [Units: vx] 
///  @returns True if the operation completed successfully.
bool rcBuildRegionsMonotoneWrapper(rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildRegionsMonotone(ctx, chf, borderSize, minRegionArea, mergeRegionArea);
}

/// Builds a layer set from the specified compact heightfield.
///  @ingroup recast
///  @param[in,out]	ctx			The build context to use during the operation.
///  @param[in]		chf			A fully built compact heightfield.
///  @param[in]		borderSize	The size of the non-navigable border around the heightfield. [Limit: >=0] 
///  							[Units: vx]
///  @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area 
///  							to be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[out]	lset		The resulting layer set. (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
bool rcBuildHeightfieldLayersWrapper(rcCompactHeightfield& chf,
	const int borderSize, const int walkableHeight,
	rcHeightfieldLayerSet& lset)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildHeightfieldLayers(ctx, chf, borderSize, walkableHeight, lset);
}

/// Builds a contour set from the region outlines in the provided compact heightfield.
///  @ingroup recast
///  @param[in,out]	ctx			The build context to use during the operation.
///  @param[in]		chf			A fully built compact heightfield.
///  @param[in]		maxError	The maximum distance a simplfied contour's border edges should deviate 
///  							the original raw contour. [Limit: >=0] [Units: wu]
///  @param[in]		maxEdgeLen	The maximum allowed length for contour edges along the border of the mesh. 
///  							[Limit: >=0] [Units: vx]
///  @param[out]	cset		The resulting contour set. (Must be pre-allocated.)
///  @param[in]		buildFlags	The build flags. (See: #rcBuildContoursFlags)
///  @returns True if the operation completed successfully.
bool rcBuildContoursWrapper(rcCompactHeightfield& chf,
	const float maxError, const int maxEdgeLen,
	rcContourSet& cset, const int buildFlags)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildContours(ctx, chf, maxError, maxEdgeLen, cset, buildFlags);
}

/// Builds a polygon mesh from the provided contours.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		cset	A fully built contour set.
///  @param[in]		nvp		The maximum number of vertices allowed for polygons generated during the 
///  						contour to polygon conversion process. [Limit: >= 3] 
///  @param[out]	mesh	The resulting polygon mesh. (Must be re-allocated.)
///  @returns True if the operation completed successfully.
bool rcBuildPolyMeshWrapper(rcContourSet& cset, const int nvp, rcPolyMesh& mesh)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildPolyMesh(ctx, cset, nvp, mesh);
}

/// Merges multiple polygon meshes into a single mesh.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		meshes	An array of polygon meshes to merge. [Size: @p nmeshes]
///  @param[in]		nmeshes	The number of polygon meshes in the meshes array.
///  @param[in]		mesh	The resulting polygon mesh. (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
bool rcMergePolyMeshesWrapper(rcPolyMesh** meshes, const int nmeshes, rcPolyMesh& mesh)
{
	rcContext* ctx = new rcContext(false);
	return rcMergePolyMeshes(ctx, meshes, nmeshes, mesh);
}

/// Builds a detail mesh from the provided polygon mesh.
///  @ingroup recast
///  @param[in,out]	ctx				The build context to use during the operation.
///  @param[in]		mesh			A fully built polygon mesh.
///  @param[in]		chf				The compact heightfield used to build the polygon mesh.
///  @param[in]		sampleDist		Sets the distance to use when samping the heightfield. [Limit: >=0] [Units: wu]
///  @param[in]		sampleMaxError	The maximum distance the detail mesh surface should deviate from 
///  								heightfield data. [Limit: >=0] [Units: wu]
///  @param[out]	dmesh			The resulting detail mesh.  (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
bool rcBuildPolyMeshDetailWrapper(const rcPolyMesh& mesh, const rcCompactHeightfield& chf,
	const float sampleDist, const float sampleMaxError,
	rcPolyMeshDetail& dmesh)
{
	rcContext* ctx = new rcContext(false);
	return rcBuildPolyMeshDetail(ctx, mesh, chf, sampleDist, sampleMaxError, dmesh);
}

/// Copies the poly mesh data from src to dst.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		src		The source mesh to copy from.
///  @param[out]	dst		The resulting detail mesh. (Must be pre-allocated, must be empty mesh.)
///  @returns True if the operation completed successfully.
bool rcCopyPolyMeshWrapper(const rcPolyMesh& src, rcPolyMesh& dst)
{
	rcContext* ctx = new rcContext(false);
	return rcCopyPolyMesh(ctx, src, dst);
}

/// Merges multiple detail meshes into a single detail mesh.
///  @ingroup recast
///  @param[in,out]	ctx		The build context to use during the operation.
///  @param[in]		meshes	An array of detail meshes to merge. [Size: @p nmeshes]
///  @param[in]		nmeshes	The number of detail meshes in the meshes array.
///  @param[out]	mesh	The resulting detail mesh. (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
bool rcMergePolyMeshDetailsWrapper(rcPolyMeshDetail** meshes, const int nmeshes, rcPolyMeshDetail& mesh)
{
	rcContext* ctx = new rcContext(false);
	return rcMergePolyMeshDetails(ctx, meshes, nmeshes, mesh);
}