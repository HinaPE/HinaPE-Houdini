#include "SIM_Hina_Particles.h"

SIM_HINA_GEOMETRY_IMPLEMENT(
		Particles,
		false,
		HINA_FLOAT_VECTOR_PARAMETER(FluidDomain, 3, 2, 2, 2) \
        HINA_FLOAT_PARAMETER(TargetSpacing, .02) \
        HINA_FLOAT_PARAMETER(KernelRadiusOverTargetSpacing, 1.8) \
        HINA_FLOAT_PARAMETER(TargetDensity, 1000.) \
        HINA_FLOAT_VECTOR_PARAMETER(Gravity, 3, 0, -9.8, 0) \
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
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
)
void SIM_Hina_Particles::_init_Particles()
{
	this->offset2index.clear();
	this->index2offset.clear();
	this->x = nullptr;
	this->v = nullptr;
	this->f = nullptr;
	this->m = nullptr;
	this->V = nullptr;
	this->rho = nullptr;
}
void SIM_Hina_Particles::_makeEqual_Particles(const SIM_Hina_Particles *src)
{
	this->offset2index = src->offset2index;
	this->index2offset = src->index2offset;
	this->x = src->x;
	this->v = src->v;
	this->f = src->f;
	this->m = src->m;
	this->V = src->V;
	this->rho = src->rho;
}
void SIM_Hina_Particles::_setup_gdp(GU_Detail *gdp) const
{
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_VELOCITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_FORCE, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_FORCE_NORM, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_MASS, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_VOLUME, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DENSITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_PRESSURE, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_SELF, HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_OTHERS, HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
}
void SIM_Hina_Particles::load()
{
	if (x == nullptr || v == nullptr || m == nullptr || f == nullptr || V == nullptr || rho == nullptr)
	{
		std::cout << "SIM_Hina_Particles::load() called with nullptr" << std::endl;
		return;
	}

	SIM_GeometryAutoReadLock lock(this);
	const GU_Detail *gdp = lock.getGdp();
	GA_ROHandleV3 pos_handle = gdp->getP();
	GA_ROHandleV3 vel_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_ROHandleF mass_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_Offset pt_off;
	(*x).resize(gdp->getNumPoints());
	(*v).resize(gdp->getNumPoints());
	(*m).resize(gdp->getNumPoints());
	GA_FOR_ALL_PTOFF(gdp, pt_off)
		{
			GA_Size index = gdp->pointIndex(pt_off);
			UT_Vector3 pos = pos_handle.get(pt_off);
			UT_Vector3 vel = vel_handle.get(pt_off);
			fpreal mass = mass_handle.get(pt_off);
			offset2index[pt_off] = index;
			index2offset[index] = pt_off;
			(*x)[index] = pos;
			(*v)[index] = vel;
			(*m)[index] = mass;
		}
}
void SIM_Hina_Particles::commit()
{
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 pos_handle = gdp.getP();
	GA_RWHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_RWHandleV3 force_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
	GA_RWHandleF force_n_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE_NORM);
	GA_RWHandleF mass_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_RWHandleF volume_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
	GA_RWHandleF density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
	GA_RWHandleI self_n_sum_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_SELF);
	GA_RWHandleI other_n_sum_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_OTHERS);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			UT_Vector3 pos = (*x)[offset2index[pt_off]];
			UT_Vector3 vel = (*v)[offset2index[pt_off]];
			UT_Vector3 force = (*f)[offset2index[pt_off]];
			fpreal force_n = force.length();
			fpreal mass = (*m)[offset2index[pt_off]];
			fpreal volume = (*V)[offset2index[pt_off]];
			fpreal density = (*rho)[offset2index[pt_off]];
			pos_handle.set(pt_off, pos);
			vel_handle.set(pt_off, vel);
			force_handle.set(pt_off, force);
			force_n_handle.set(pt_off, force_n);
			mass_handle.set(pt_off, mass);
			volume_handle.set(pt_off, volume);
			density_handle.set(pt_off, density);

//			int fn_sum = neighbor_lists_cache[pt_off].size();
//			int bn_sum = 0;
//			for (auto &pair: other_neighbor_lists_cache)
//				bn_sum += pair.second[pt_off].size();
//			self_n_sum_handle.set(pt_off, fn_sum);
//			other_n_sum_handle.set(pt_off, bn_sum);
		}
}
