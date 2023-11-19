#include "DetourNavMesh.h"
#include "dtNavMeshInterop.h"

dtStatus dtNavMesh_init(dtNavMesh* mesh, const dtNavMeshParams* params)
{
	return mesh->init(params);
}

dtStatus dtNavMesh_init2(dtNavMesh* mesh, unsigned char* data, const int dataSize, const int flags)
{
	return mesh->init(data, dataSize, flags);
}

const dtNavMeshParams* dtNavMesh_getParams(dtNavMesh* mesh)
{
	return mesh->getParams();
}

dtStatus dtNavMesh_addTile(dtNavMesh* mesh, unsigned char* data, int dataSize, int flags, dtTileRef lastRef, dtTileRef* result)
{
	return mesh->addTile(data, dataSize, flags, lastRef, result);
}

dtStatus dtNavMesh_removeTile(dtNavMesh* mesh, dtTileRef ref, unsigned char** data, int* dataSize) 
{
	return mesh->removeTile(ref, data, dataSize);
}

void dtNavMesh_calcTileLoc(dtNavMesh* mesh, const float* pos, int* tx, int* ty) 
{
	mesh->calcTileLoc(pos, tx, ty);
}

const dtMeshTile* dtNavMesh_getTileAt(dtNavMesh* mesh, int x, int y, int layer) 
{
	return mesh->getTileAt(x, y, layer);
}

int dtNavMesh_getTilesAt(dtNavMesh* mesh, int x, int y, dtMeshTile const** tiles, int maxTiles) 
{
	return mesh->getTilesAt(x, y, tiles, maxTiles);
}

dtTileRef dtNavMesh_getTileRefAt(dtNavMesh* mesh, int x, int y, int layer) 
{
	return mesh->getTileRefAt(x, y, layer);
}

dtTileRef dtNavMesh_getTileRef(dtNavMesh* mesh, const dtMeshTile* tile) 
{
	return mesh->getTileRef(tile);
}

const dtMeshTile* dtNavMesh_getTileByRef(dtNavMesh* mesh, dtTileRef ref) 
{
	return mesh->getTileByRef(ref);
}

int dtNavMesh_getMaxTiles(dtNavMesh* mesh)
{
	return mesh->getMaxTiles();
}

dtStatus dtNavMesh_getTileAndPolyByRef(dtNavMesh* mesh, dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) 
{
	return mesh->getTileAndPolyByRef(ref, tile, poly);
}

void dtNavMesh_getTileAndPolyByRefUnsafe(dtNavMesh* mesh, dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) 
{
	mesh->getTileAndPolyByRefUnsafe(ref, tile, poly);
}

bool dtNavMesh_isValidPolyRef(dtNavMesh* mesh, dtPolyRef ref) 
{
	return mesh->isValidPolyRef(ref);
}

dtPolyRef dtNavMesh_getPolyRefBase(dtNavMesh* mesh, const dtMeshTile* tile) 
{
	return mesh->getPolyRefBase(tile);
}

dtStatus dtNavMesh_getOffMeshConnectionPolyEndPoints(dtNavMesh* mesh, dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) 
{
	return mesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
}

const dtOffMeshConnection* dtNavMesh_getOffMeshConnectionByRef(dtNavMesh* mesh, dtPolyRef ref) 
{
	return mesh->getOffMeshConnectionByRef(ref);
}

dtStatus dtNavMesh_setPolyFlags(dtNavMesh* mesh, dtPolyRef ref, unsigned short flags) 
{
	return mesh->setPolyFlags(ref, flags);
}

dtStatus dtNavMesh_getPolyFlags(dtNavMesh* mesh, dtPolyRef ref, unsigned short* resultFlags) 
{
	return mesh->getPolyFlags(ref, resultFlags);
}

dtStatus dtNavMesh_setPolyArea(dtNavMesh* mesh, dtPolyRef ref, unsigned char area) 
{
	return mesh->setPolyArea(ref, area);
}

dtStatus dtNavMesh_getPolyArea(dtNavMesh* mesh, dtPolyRef ref, unsigned char* resultArea) 
{
	return mesh->getPolyArea(ref, resultArea);
}

int dtNavMesh_getTileStateSize(dtNavMesh* mesh, const dtMeshTile* tile) 
{
	return mesh->getTileStateSize(tile);
}

dtStatus dtNavMesh_storeTileState(dtNavMesh* mesh, const dtMeshTile* tile, unsigned char* data, int maxDataSize) 
{
	return mesh->storeTileState(tile, data, maxDataSize);
}

dtStatus dtNavMesh_restoreTileState(dtNavMesh* mesh, dtMeshTile* tile, const unsigned char* data, int maxDataSize) 
{
	return mesh->restoreTileState(tile, data, maxDataSize);
}
