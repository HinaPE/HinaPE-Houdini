/**
 * Copyright 2022 Xiaowei He
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "Node.h"

#include "Topology/SignedDistanceField.h"
#include "Topology/TriangleSet.h"

namespace dyno
{
	template<typename TDataType>
	class VolumeClipper : public Node
	{
		DECLARE_TCLASS(VolumeClipper, TDataType)
	public:
		typedef typename TDataType::Real Real;
		typedef typename TDataType::Coord Coord;

		VolumeClipper();
		~VolumeClipper() override;

	public:
		DEF_VAR(Coord, Translation, Coord(0), "");
		DEF_VAR(Coord, Rotation, Coord(0), "");

		DEF_ARRAY_STATE(Real, Field, DeviceType::GPU, "Signed distance field defined on trianglular vertices");

		DEF_INSTANCE_STATE(TriangleSet<TDataType>, Plane, "");

		DEF_INSTANCE_STATE(TriangleSet<TDataType>, TriangleSet, "An iso surface");

		DEF_INSTANCE_IN(SignedDistanceField<TDataType>, LevelSet, "A 3D signed distance field");

	protected:
		void resetStates() override;
	};

	IMPLEMENT_TCLASS(VolumeClipper, TDataType)
}