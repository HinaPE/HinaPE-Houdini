/**
 * Copyright 2017-2021 Xiaowei He
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
#include "Module/ComputeModule.h"

namespace dyno 
{
	template<typename TDataType>
	class CalculateNorm : public ComputeModule
	{
		DECLARE_TCLASS(ColorMapping, TDataType)
	public:
		typedef typename TDataType::Real Real;
		typedef typename TDataType::Coord Coord;

		CalculateNorm() {};
		~CalculateNorm() override {};

		void compute() override;

	public:
		DEF_ARRAY_IN(Coord, Vec, DeviceType::GPU, "");
		DEF_ARRAY_OUT(Real, Norm, DeviceType::GPU, "");
	};
}
