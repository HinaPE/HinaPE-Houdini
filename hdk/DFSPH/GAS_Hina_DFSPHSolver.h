#ifndef HINAPE_GAS_HINA_DFSPHSOLVER_H
#define HINAPE_GAS_HINA_DFSPHSOLVER_H

#include <Common/SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		DFSPHSolver,

		bool _compute_alpha();
		THREADED_METHOD(GAS_Hina_DFSPHSolver, true, _compute_alpha_mt)
		void _compute_alpha_mtPartial(const UT_JobInfo &info);
)

#endif //HINAPE_GAS_HINA_DFSPHSOLVER_H
