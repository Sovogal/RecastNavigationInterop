#include "DetourInterop.h"
#include "DetourCommon.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"


/* DetourCommon */
/// Derives the closest point on a triangle from the specified reference point.
///  @param[out]	closest	The closest point on the triangle.	
///  @param[in]		p		The reference point from which to test. [(x, y, z)]
///  @param[in]		a		Vertex A of triangle ABC. [(x, y, z)]
///  @param[in]		b		Vertex B of triangle ABC. [(x, y, z)]
///  @param[in]		c		Vertex C of triangle ABC. [(x, y, z)]
void dtClosestPtPointTriangleWrapper(float* closest, const float* p,
	const float* a, const float* b, const float* c)
{
	dtClosestPtPointTriangle(closest, p, a, b, c);
}

/// Derives the y-axis height of the closest point on the triangle from the specified reference point.
///  @param[in]		p		The reference point from which to test. [(x, y, z)]
///  @param[in]		a		Vertex A of triangle ABC. [(x, y, z)]
///  @param[in]		b		Vertex B of triangle ABC. [(x, y, z)]
///  @param[in]		c		Vertex C of triangle ABC. [(x, y, z)]
///  @param[out]	h		The resulting height.
bool dtClosestHeightPointTriangleWrapper(const float* p, const float* a, const float* b, const float* c, float& h)
{
	return dtClosestHeightPointTriangle(p, a, b, c, h);
}

bool dtIntersectSegmentPoly2DWrapper(const float* p0, const float* p1,
	const float* verts, int nverts,
	float& tmin, float& tmax,
	int& segMin, int& segMax)
{
	return dtIntersectSegmentPoly2D(p0, p1, verts, nverts, tmin, tmax, segMin, segMax);
}

bool dtIntersectSegSeg2DWrapper(const float* ap, const float* aq,
	const float* bp, const float* bq,
	float& s, float& t)
{
	return dtIntersectSegSeg2D(ap, aq, bp, bq, s, t);
}

/// Determines if the specified point is inside the convex polygon on the xz-plane.
///  @param[in]		pt		The point to check. [(x, y, z)]
///  @param[in]		verts	The polygon vertices. [(x, y, z) * @p nverts]
///  @param[in]		nverts	The number of vertices. [Limit: >= 3]
/// @return True if the point is inside the polygon.
bool dtPointInPolygonWrapper(const float* pt, const float* verts, const int nverts)
{
	return dtPointInPolygon(pt, verts, nverts);
}

bool dtDistancePtPolyEdgesSqrWrapper(const float* pt, const float* verts, const int nverts,
	float* ed, float* et)
{
	return dtDistancePtPolyEdgesSqr(pt, verts, nverts, ed, et);
}

float dtDistancePtSegSqr2DWrapper(const float* pt, const float* p, const float* q, float& t)
{
	return dtDistancePtSegSqr2D(pt, p, q, t);
}

/// Derives the centroid of a convex polygon.
///  @param[out]	tc		The centroid of the polgyon. [(x, y, z)]
///  @param[in]		idx		The polygon indices. [(vertIndex) * @p nidx]
///  @param[in]		nidx	The number of indices in the polygon. [Limit: >= 3]
///  @param[in]		verts	The polygon vertices. [(x, y, z) * vertCount]
void dtCalcPolyCenterWrapper(float* tc, const unsigned short* idx, int nidx, const float* verts)
{
	dtCalcPolyCenter(tc, idx, nidx, verts);
}

/// Determines if the two convex polygons overlap on the xz-plane.
///  @param[in]		polya		Polygon A vertices.	[(x, y, z) * @p npolya]
///  @param[in]		npolya		The number of vertices in polygon A.
///  @param[in]		polyb		Polygon B vertices.	[(x, y, z) * @p npolyb]
///  @param[in]		npolyb		The number of vertices in polygon B.
/// @return True if the two polygons overlap.
bool dtOverlapPolyPoly2DWrapper(const float* polya, const int npolya,
	const float* polyb, const int npolyb)
{
	return dtOverlapPolyPoly2D(polya, npolya, polyb, npolyb);
}

void dtRandomPointInConvexPolyWrapper(const float* pts, const int npts, float* areas,
	const float s, const float t, float* out)
{
	dtRandomPointInConvexPoly(pts, npts, areas, s, t, out);
}


/* DetourNavMeshBuilder */
/// Builds navigation mesh tile data from the provided tile creation data.
/// @ingroup detour
///  @param[in]		params		Tile creation data.
///  @param[out]	outData		The resulting tile data.
///  @param[out]	outDataSize	The size of the tile data array.
/// @return True if the tile data was successfully created.
bool dtCreateNavMeshDataWrapper(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize)
{
	return dtCreateNavMeshData(params, outData, outDataSize);
}

/// Swaps the endianess of the tile data's header (#dtMeshHeader).
///  @param[in,out]	data		The tile data array.
///  @param[in]		dataSize	The size of the data array.
bool dtNavMeshHeaderSwapEndianWrapper(unsigned char* data, const int dataSize)
{
	return dtNavMeshHeaderSwapEndian(data, dataSize);
}

/// Swaps endianess of the tile data.
///  @param[in,out]	data		The tile data array.
///  @param[in]		dataSize	The size of the data array.
bool dtNavMeshDataSwapEndianWrapper(unsigned char* data, const int dataSize)
{
	return dtNavMeshDataSwapEndian(data, dataSize);
}

/// Allocates a navigation mesh object using the Detour allocator.
/// @return A navigation mesh that is ready for initialization, or null on failure.
///  @ingroup detour
dtNavMesh* dtAllocNavMeshWrapper()
{
	return dtAllocNavMesh();
}

/// Frees the specified navigation mesh object using the Detour allocator.
///  @param[in]	navmesh		A navigation mesh allocated using #dtAllocNavMesh
///  @ingroup detour
void dtFreeNavMeshWrapper(dtNavMesh* navmesh)
{
	dtFreeNavMesh(navmesh);
}

/* DetourNavMeshQuery */
/// Allocates a query object using the Detour allocator.
/// @return An allocated query object, or null on failure.
/// @ingroup detour
dtNavMeshQuery* dtAllocNavMeshQueryWrapper()
{
	return dtAllocNavMeshQuery();
}

/// Frees the specified query object using the Detour allocator.
///  @param[in]		query		A query object allocated using #dtAllocNavMeshQuery
/// @ingroup detour
void dtFreeNavMeshQueryWrapper(dtNavMeshQuery* query)
{
	dtFreeNavMeshQuery(query);
}

