#pragma once
#include "DetourNavMesh.h"

#ifdef DetourExport
#define DETOUR_INT __declspec(dllexport)
#else
#define DETOUR_INT __declspec(dllimport)
#endif

extern "C" {
	/// @name Initialization and Tile Management

	/// Initializes the navigation mesh for tiled use.
	///  @param[in]	params		Initialization parameters.
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_init(dtNavMesh* mesh, const dtNavMeshParams* params);

	/// Initializes the navigation mesh for single tile use.
	///  @param[in]	data		Data of the new tile. (See: #dtCreateNavMeshData)
	///  @param[in]	dataSize	The data size of the new tile.
	///  @param[in]	flags		The tile flags. (See: #dtTileFlags)
	/// @return The status flags for the operation.
	///  @see dtCreateNavMeshData
	DETOUR_INT dtStatus dtNavMesh_init2(dtNavMesh* mesh, unsigned char* data, const int dataSize, const int flags);

	/// The navigation mesh initialization params.
	DETOUR_INT const dtNavMeshParams* dtNavMesh_getParams(dtNavMesh* mesh);

	/// Adds a tile to the navigation mesh.
	///  @param[in]		data		Data for the new tile mesh. (See: #dtCreateNavMeshData)
	///  @param[in]		dataSize	Data size of the new tile mesh.
	///  @param[in]		flags		Tile flags. (See: #dtTileFlags)
	///  @param[in]		lastRef		The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
	///  @param[out]	result		The tile reference. (If the tile was succesfully added.) [opt]
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_addTile(dtNavMesh* mesh, unsigned char* data, int dataSize, int flags, dtTileRef lastRef, dtTileRef* result);

	/// Removes the specified tile from the navigation mesh.
	///  @param[in]		ref			The reference of the tile to remove.
	///  @param[out]	data		Data associated with deleted tile.
	///  @param[out]	dataSize	Size of the data associated with deleted tile.
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_removeTile(dtNavMesh* mesh, dtTileRef ref, unsigned char** data, int* dataSize);

	/// @}

	/// @{
	/// @name Query Functions

	/// Calculates the tile grid location for the specified world position.
	///  @param[in]	pos  The world position for the query. [(x, y, z)]
	///  @param[out]	tx		The tile's x-location. (x, y)
	///  @param[out]	ty		The tile's y-location. (x, y)
	DETOUR_INT void dtNavMesh_calcTileLoc(dtNavMesh* mesh, const float* pos, int* tx, int* ty);

	/// Gets the tile at the specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile, or null if the tile does not exist.
	DETOUR_INT const dtMeshTile* dtNavMesh_getTileAt(dtNavMesh* mesh, const int x, const int y, const int layer);

	/// Gets all tiles at the specified grid location. (All layers.)
	///  @param[in]		x			The tile's x-location. (x, y)
	///  @param[in]		y			The tile's y-location. (x, y)
	///  @param[out]	tiles		A pointer to an array of tiles that will hold the result.
	///  @param[in]		maxTiles	The maximum tiles the tiles parameter can hold.
	/// @return The number of tiles returned in the tiles array.
	DETOUR_INT int dtNavMesh_getTilesAt(dtNavMesh* mesh, const int x, const int y,
		dtMeshTile const** tiles, const int maxTiles);

	/// Gets the tile reference for the tile at specified grid location.
	///  @param[in]	x		The tile's x-location. (x, y, layer)
	///  @param[in]	y		The tile's y-location. (x, y, layer)
	///  @param[in]	layer	The tile's layer. (x, y, layer)
	/// @return The tile reference of the tile, or 0 if there is none.
	DETOUR_INT dtTileRef dtNavMesh_getTileRefAt(dtNavMesh* mesh, int x, int y, int layer);

	/// Gets the tile reference for the specified tile.
	///  @param[in]	tile	The tile.
	/// @return The tile reference of the tile.
	DETOUR_INT dtTileRef dtNavMesh_getTileRef(dtNavMesh* mesh, const dtMeshTile* tile);

	/// Gets the tile for the specified tile reference.
	///  @param[in]	ref		The tile reference of the tile to retrieve.
	/// @return The tile for the specified reference, or null if the 
	///		reference is invalid.
	DETOUR_INT const dtMeshTile* dtNavMesh_getTileByRef(dtNavMesh* mesh, dtTileRef ref);

	/// The maximum number of tiles supported by the navigation mesh.
	/// @return The maximum number of tiles supported by the navigation mesh.
	DETOUR_INT int dtNavMesh_getMaxTiles(dtNavMesh* mesh);

	/// Gets the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		The reference for the a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_getTileAndPolyByRef(dtNavMesh* mesh, const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly);

	/// Returns the tile and polygon for the specified polygon reference.
	///  @param[in]		ref		A known valid reference for a polygon.
	///  @param[out]	tile	The tile containing the polygon.
	///  @param[out]	poly	The polygon.
	DETOUR_INT void dtNavMesh_getTileAndPolyByRefUnsafe(dtNavMesh* mesh, const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly);

	/// Checks the validity of a polygon reference.
	///  @param[in]	ref		The polygon reference to check.
	/// @return True if polygon reference is valid for the navigation mesh.
	DETOUR_INT bool dtNavMesh_isValidPolyRef(dtNavMesh* mesh, dtPolyRef ref);

	/// Gets the polygon reference for the tile's base polygon.
	///  @param[in]	tile		The tile.
	/// @return The polygon reference for the base polygon in the specified tile.
	DETOUR_INT dtPolyRef dtNavMesh_getPolyRefBase(dtNavMesh* mesh, const dtMeshTile* tile);

	/// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
	///  @param[in]		prevRef		The reference of the polygon before the connection.
	///  @param[in]		polyRef		The reference of the off-mesh connection polygon.
	///  @param[out]	startPos	The start position of the off-mesh connection. [(x, y, z)]
	///  @param[out]	endPos		The end position of the off-mesh connection. [(x, y, z)]
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_getOffMeshConnectionPolyEndPoints(dtNavMesh* mesh, dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos);

	/// Gets the specified off-mesh connection.
	///  @param[in]	ref		The polygon reference of the off-mesh connection.
	/// @return The specified off-mesh connection, or null if the polygon reference is not valid.
	DETOUR_INT const dtOffMeshConnection* dtNavMesh_getOffMeshConnectionByRef(dtNavMesh* mesh, dtPolyRef ref);

	/// @}

	/// @{
	/// @name State Management
	/// These functions do not effect #dtTileRef or #dtPolyRef's. 

	/// Sets the user defined flags for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	flags	The new flags for the polygon.
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_setPolyFlags(dtNavMesh* mesh, dtPolyRef ref, unsigned short flags);

	/// Gets the user defined flags for the specified polygon.
	///  @param[in]		ref				The polygon reference.
	///  @param[out]	resultFlags		The polygon flags.
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_getPolyFlags(dtNavMesh* mesh, dtPolyRef ref, unsigned short* resultFlags);

	/// Sets the user defined area for the specified polygon.
	///  @param[in]	ref		The polygon reference.
	///  @param[in]	area	The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_setPolyArea(dtNavMesh* mesh, dtPolyRef ref, unsigned char area);

	/// Gets the user defined area for the specified polygon.
	///  @param[in]		ref			The polygon reference.
	///  @param[out]	resultArea	The area id for the polygon.
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_getPolyArea(dtNavMesh* mesh, dtPolyRef ref, unsigned char* resultArea);

	/// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
	///  @param[in]	tile	The tile.
	/// @return The size of the buffer required to store the state.
	DETOUR_INT int dtNavMesh_getTileStateSize(dtNavMesh* mesh, const dtMeshTile* tile);

	/// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
	///  @param[in]		tile			The tile.
	///  @param[out]	data			The buffer to store the tile's state in.
	///  @param[in]		maxDataSize		The size of the data buffer. [Limit: >= #getTileStateSize]
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_storeTileState(dtNavMesh* mesh, const dtMeshTile* tile, unsigned char* data, const int maxDataSize);

	/// Restores the state of the tile.
	///  @param[in]	tile			The tile.
	///  @param[in]	data			The new state. (Obtained from #storeTileState.)
	///  @param[in]	maxDataSize		The size of the state within the data buffer.
	/// @return The status flags for the operation.
	DETOUR_INT dtStatus dtNavMesh_restoreTileState(dtNavMesh* mesh, dtMeshTile* tile, const unsigned char* data, const int maxDataSize);

}