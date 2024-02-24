#include "GAS_Hina_UpdateDensityAkinci.h"
#include "GAS_Hina_UpdateDensity.h"
#include <Particles/SIM_Hina_Particles.h>
#include <Particles/SIM_Hina_DFSPHParticles.h>
#include <Particles/SIM_Hina_Akinci2012BoundaryParticles.h>
#include <CUDA_HinaPE/kernels.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		UpdateDensityAkinci,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
		static std::array<PRM_Name, 4> Kernels = {\
            PRM_Name("0", "Poly64"), \
            PRM_Name("1", "Spiky"), \
            PRM_Name("2", "CubicSpline"), \
            PRM_Name(nullptr), \
		}; \
        static PRM_Name KernelName("Kernel", "Kernel"); \
        static PRM_Default KernelNameDefault(2, "CubicSpline"); \
        static PRM_ChoiceList CL(PRM_CHOICELIST_SINGLE, Kernels.data()); \
        PRMS.emplace_back(PRM_ORD, 1, &KernelName, &KernelNameDefault, &CL); \
)

void GAS_Hina_UpdateDensityAkinci::_init() {}
void GAS_Hina_UpdateDensityAkinci::_makeEqual(const GAS_Hina_UpdateDensityAkinci *src) {}
bool GAS_Hina_UpdateDensityAkinci::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> boundary_particles;
	SIM_ObjectArray affectors;
	obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(obj->getName()))
			continue;
		UT_String boundary_obj_name = obj_collider->getName();
		SIM_Hina_Akinci2012BoundaryParticles *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Akinci2012BoundaryParticles::DATANAME, SIM_Hina_Akinci2012BoundaryParticles);
		if (boundary_akinci)
			boundary_particles[boundary_obj_name] = boundary_akinci;
	}

	HinaPE::CubicSplineKernel<false> kernel(particles->getTargetSpacing() * particles->getKernelRadiusOverTargetSpacing());
	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp_fluid = lock.getGdp();
	GA_ROHandleF mass_handle = gdp_fluid.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_RWHandleF density_handle = gdp_fluid.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);

	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp_fluid, pt_off)
		{
			fpreal rho = 0.;

			fpreal m_i = mass_handle.get(pt_off);
			rho += m_i * kernel.kernel(0.); // important: self is also a neighbor
			UT_Vector3 p_i = gdp_fluid.getPos3(pt_off);
			particles->for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
			{
				UT_Vector3 p_j = n_pos;
				const UT_Vector3 r = p_i - p_j;
				rho += m_i * kernel.kernel(r.length());
			});
			for (const auto &pair: boundary_particles)
			{
				SIM_GeometryAutoReadLock read_lock(pair.second);
				const GU_Detail *gdp_boundary = read_lock.getGdp();
				GA_ROHandleF volume_handle = gdp_boundary->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
				particles->for_each_neighbor_others(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
				{
					fpreal volume = volume_handle.get(n_off);
					UT_Vector3 p_j = n_pos;
					const UT_Vector3 r = p_i - p_j;
					rho += 1000. * volume * kernel.kernel(r.length());
				}, pair.first);
			}
			density_handle.set(pt_off, rho);
		}
	return true;
}
