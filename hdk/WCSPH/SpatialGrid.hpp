/*******************************************************************************
 *
 * BasicSPH particle-based fluid solver
 * Copyright (C) 2015 François Dagenais
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Description: Spatial grid structure used to retrieve neighbor points
 *
 ******************************************************************************/

#include <cmath>

//------------------------------------------------------------------------------
// Constructor / Destructor
//------------------------------------------------------------------------------
template<class T>
SpatialGrid<T>::SpatialGrid(double cellSize, const UT_Vector3& volumeMin, const UT_Vector3& volumeMax)
: _resX(0),
  _resY(0),
  _resZ(0),
  _cellSize(0.0)
{
	initGrid(cellSize, volumeMin, volumeMax);
}

template<class T>
SpatialGrid<T>::~SpatialGrid()
{
	clear();
}

//------------------------------------------------------------------------------
// Public functions
//------------------------------------------------------------------------------
template<class T>
void SpatialGrid<T>::initGrid(double cellSize, const UT_Vector3& volumeMin, const UT_Vector3& volumeMax)
{
	// Clear previous grid
	if (!_grid.empty())
	{
		clear();
	}

	// Compute number of cell on each axis
	_resX = static_cast<int>(ceil((volumeMax.x()-volumeMin.x())/cellSize));
	_resY = static_cast<int>(ceil((volumeMax.y()-volumeMin.y())/cellSize));
	_resZ = static_cast<int>(ceil((volumeMax.z()-volumeMin.z())/cellSize));

	// Compute grid dimensions
	_cellSize = cellSize;
	_volMin = volumeMin;

	// Allocate memory
	_grid.resize(_resX*_resY*_resZ);
}

template<class T>
void SpatialGrid<T>::clear()
{
	// Swap trick! This will effectively unallocate _grid's memory
	_grid.clear();
	std::vector<std::vector<T> >().swap(_grid);
}

template<class T>
inline
void SpatialGrid<T>::insert(const T& element, const UT_Vector3& position)
{
	// Get Index
	int xIndex = getXIndex(position);
	int yIndex = getYIndex(position);
	int zIndex = getZIndex(position);
	int cellIndex = getGridIndex(xIndex, yIndex, zIndex);

	// Make sure we are not out of the grid
	if ((xIndex>=0) && (xIndex<_resX) && (yIndex>=0) && (yIndex<_resY) &&
			(zIndex>=0) && (zIndex<_resZ))
	{
		// Add into grid
		_grid[cellIndex].push_back(element);
	}
}

template<class T>
inline
void SpatialGrid<T>::insert(const T& element, const UT_Vector3& AABBmin, const UT_Vector3& AABBmax)
{
	// Get min/max indices
	int xMin = getXIndex(AABBmin.x());	
	int yMin = getYIndex(AABBmin.y());	
	int zMin = getZIndex(AABBmin.z());	
	int xMax = getXIndex(AABBmax.x());
	int yMax = getYIndex(AABBmax.y());
	int zMax = getZIndex(AABBmax.z());

	// clamp indices
	if (xMin<0) xMin = 0;
	if (xMax>=_resX) xMax = _resX-1;

	if (yMin<0) yMin = 0;
	if (yMax>=_resY) yMax = _resY-1;

	if (zMin<0) zMin = 0;
	if (zMax>=_resZ) zMax = _resZ-1;

	// Insert element into cells
	for (int ix=xMin; ix<=xMax; ++ix)
	{
		for (int iy=yMin; iy<=yMax; ++iy)
		{
			for (int iz=zMin; iz<=zMax; ++iz)
			{
				int cellIndex = getGridIndex(ix, iy, iz);
				_grid[cellIndex].push_back(element);
			}
		}
	}
}

template<class T>
inline
void SpatialGrid<T>::getElements(const UT_Vector3&		position,
								 double 			radius,
								 std::vector<T*>&	elements)
{

	// Get min/max indices
	int xMin = getXIndex(position.x() - radius);
	int yMin = getYIndex(position.y() - radius);
	int zMin = getZIndex(position.z() - radius);
	int xMax = getXIndex(position.x() + radius);
	int yMax = getYIndex(position.y() + radius);
	int zMax = getZIndex(position.z() + radius);

	// Clamp indices
	if (xMin<0) xMin = 0;
	if (xMax>=_resX) xMax = _resX-1;

	if (yMin<0) yMin = 0;
	if (yMax>=_resY) yMax = _resY-1;

	if (zMin<0) zMin = 0;
	if (zMax>=_resZ) zMax = _resZ-1;

	// Traverse cells
	int e = 0;
	for (int ix=xMin; ix<=xMax; ++ix)
	{
		for (int iy=yMin; iy<=yMax; ++iy)
		{
			for (int iz=zMin; iz<=zMax; ++iz)
			{
				int cellIndex = getGridIndex(ix, iy, iz);
				
				// Traverse elements in cell
				std::vector<T>& cell = _grid[cellIndex];
				for (int i=0; i<cell.size(); ++i, ++e)
				{
					if (e < elements.size())
					{
						// This is faster than using push_back,
						// therefore, if we can we'll use it instead
						elements[e] = &cell[i];	
					}
					else
					{
						elements.push_back(&cell[i]);
					}
				}
			}
		}
	}

	if (e < elements.size())
	{
		elements.resize(e);
	}
}

template<class T>
inline
void SpatialGrid<T>::getElements(int ix, int iy, int iz, std::vector<T*>& elements)
{
	int cellIndex = getGridIndex(ix, iy, iz);
	std::vector<T>& cell = _grid[cellIndex];
	int e=0;
	for (int i=0; i<cell.size(); ++i, ++e)
	{
		if (e < elements.size())
		{
			// This is faster than using push_back,
			// therefore, if we can we'll use it instead
			elements[e] = &cell[i];	
		}
		else
		{
			elements.push_back(&cell[i]);
		}
	}

	if (e < elements.size())
	{
		elements.resize(e);
	}
}

//------------------------------------------------------------------------------
// Private functions
//------------------------------------------------------------------------------
template<class T>
inline
int SpatialGrid<T>::getGridIndex(int ix, int iy, int iz) const
{
	return ix + (iy*_resX) + (iz*_resX*_resY);
}

template<class T>
inline
int SpatialGrid<T>::getXIndex(const UT_Vector3& position) const
{
	return static_cast<int>(floor( (position.x()-_volMin.x())/_cellSize ));
}

template<class T>
inline
int SpatialGrid<T>::getYIndex(const UT_Vector3& position) const
{
	return static_cast<int>(floor( (position.y()-_volMin.y())/_cellSize ));
}

template<class T>
inline
int SpatialGrid<T>::getZIndex(const UT_Vector3& position) const
{
	return static_cast<int>(floor( (position.z()-_volMin.z())/_cellSize ));
}

template<class T>
inline
int SpatialGrid<T>::getXIndex(double xPos) const
{
	return static_cast<int>(floor( (xPos-_volMin.x())/_cellSize ));
}

template<class T>
inline
int SpatialGrid<T>::getYIndex(double yPos) const
{
	return static_cast<int>(floor( (yPos-_volMin.y())/_cellSize ));
}

template<class T>
inline
int SpatialGrid<T>::getZIndex(double zPos) const
{
	return static_cast<int>(floor( (zPos-_volMin.z())/_cellSize ));
}

