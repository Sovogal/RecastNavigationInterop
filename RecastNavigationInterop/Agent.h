#pragma once
#include "DetourNavMeshQuery.h"
#include "Recast.h"
#include "DetourNavMeshBuilder.h"

#define RCDT_AGENT_EXPORT __declspec(dllexport)

static const int MAX_PATH_POLYS = 256;
static const int MAX_PATH_NODES = 2048;

struct rcdtNavMeshConfig {
	// Agent parameters
	const float agentHeight;
	const float agentRadius;
	const float agentMaxClimb;
	const float agentMaxSlope;

	// Mesh partitioning parameters
	const float cs;
	const float ch;
	const int regionMinSize;
	const int regionMergeSize;
	const float edgeMaxLen;
	const float edgeMaxError;
	const float detailSampleDist;
	const float detailSampleMaxError;

	// Tile parameters (for tiled navmesh)
	const int tileSize;

	// Geometry data
	const float* verts;
	const int nverts;
	const int* tris;
	const int ntris;
};

struct rcdtBuildResults
{
	dtNavMesh* navMesh;
	dtNavMeshQuery* navMeshQuery;
	bool succeeded;
};

class rcdtPath {
public:
	float nodes[MAX_PATH_NODES * 3];
	int nnodes;
	dtPolyRef polys[MAX_PATH_POLYS];
	int npolys;

};

int fixupCorridor(dtPolyRef* path, int npath, int maxPath, const dtPolyRef* visited, int nvisited);
int fixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery);
bool inRange(const float* v1, const float* v2, float r, float h);
bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
	float minTargetDist, const dtPolyRef* path, int pathSize,
	float* steerPos, unsigned char* steerPosFlag, dtPolyRef* steerPosRef);

extern "C" {
	RCDT_AGENT_EXPORT rcdtPath* rcdtAllocPath();
	RCDT_AGENT_EXPORT void rcdtFreePath(rcdtPath* path);
	RCDT_AGENT_EXPORT rcdtBuildResults* rcdtBuildSoloMesh(rcdtNavMeshConfig* config);
	RCDT_AGENT_EXPORT void rcdtFreeBuildResults(rcdtBuildResults* results);

	RCDT_AGENT_EXPORT void rcdtFindStraightPath(rcdtBuildResults* buildResults, const float* startPos, const float* endPos, rcdtPath* path);
	RCDT_AGENT_EXPORT void rcdtFindSmoothPath(rcdtBuildResults* builderResults, const float* startPos, const float* endPos, rcdtPath* path);

	RCDT_AGENT_EXPORT void rcdtGetNextPosition(rcdtBuildResults* builderResults, float* startPos, float* endPos, float* firstDestination);
}