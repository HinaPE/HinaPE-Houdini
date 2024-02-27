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
	this->gdp_dirty = true;
	this->offset2index.clear();
	this->index2offset.clear();
	this->x = nullptr;
	this->v = nullptr;
	this->f = nullptr;
	this->m = nullptr;
	this->V = nullptr;
	this->rho = nullptr;
	this->neighbor_this = nullptr;
	this->neighbor_others = nullptr;
}
void SIM_Hina_Particles::_makeEqual_Particles(const SIM_Hina_Particles *src)
{
	this->gdp_dirty = src->gdp_dirty;
	this->offset2index = src->offset2index;
	this->index2offset = src->index2offset;
	this->x = src->x;
	this->v = src->v;
	this->f = src->f;
	this->m = src->m;
	this->V = src->V;
	this->rho = src->rho;
	this->neighbor_this = src->neighbor_this;
	this->neighbor_others = src->neighbor_others;
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
	if (!gdp_dirty)
		return;

	if (x == nullptr || v == nullptr || m == nullptr || f == nullptr || V == nullptr || rho == nullptr || neighbor_this == nullptr || neighbor_others == nullptr)
	{
		std::cout << "SIM_Hina_Particles::load() called with nullptr" << std::endl;
		return;
	}

	SIM_GeometryAutoReadLock lock(this);
	const GU_Detail *gdp = lock.getGdp();
	GA_ROHandleV3 pos_handle = gdp->getP();
	GA_ROHandleV3 vel_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_ROHandleV3 force_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
	GA_ROHandleF mass_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_ROHandleF volume_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
	GA_ROHandleF density_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
	GA_ROHandleI self_n_sum_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_SELF);
	GA_ROHandleI other_n_sum_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_OTHERS);
	GA_Offset pt_off;
	size_t sz = gdp->getNumPoints();
	(*x).resize(sz);
	(*v).resize(sz);
	(*f).resize(sz);
	(*m).resize(sz);
	(*V).resize(sz);
	(*rho).resize(sz);
	(*neighbor_this).resize(sz);
	(*neighbor_others).resize(sz);
	GA_FOR_ALL_PTOFF(gdp, pt_off)
		{
			GA_Size pt_idx = gdp->pointIndex(pt_off);
			UT_Vector3 pos = pos_handle.get(pt_off);
			UT_Vector3 vel = vel_handle.get(pt_off);
			UT_Vector3 force = force_handle.get(pt_off);
			fpreal mass = mass_handle.get(pt_off);
			fpreal volume = volume_handle.get(pt_off);
			fpreal density = density_handle.get(pt_off);
			int fn_sum = self_n_sum_handle.get(pt_off);
			int bn_sum = other_n_sum_handle.get(pt_off);
			offset2index[pt_off] = pt_idx;
			index2offset[pt_idx] = pt_off;
			(*x)[pt_idx] = pos;
			(*v)[pt_idx] = vel;
			(*f)[pt_idx] = force;
			(*m)[pt_idx] = mass;
			(*V)[pt_idx] = volume;
			(*rho)[pt_idx] = density;
			(*neighbor_this)[pt_idx] = fn_sum;
			(*neighbor_others)[pt_idx] = bn_sum;
		}
	gdp_dirty = false;
}
void SIM_Hina_Particles::commit()
{
	if (x == nullptr || v == nullptr || m == nullptr || f == nullptr || V == nullptr || rho == nullptr || neighbor_this == nullptr || neighbor_others == nullptr)
	{
		std::cout << "SIM_Hina_Particles::load() called with nullptr" << std::endl;
		return;
	}

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
			size_t pt_idx = offset2index[pt_off];
			UT_Vector3 pos = (*x)[pt_idx];
			UT_Vector3 vel = (*v)[pt_idx];
			UT_Vector3 force = (*f)[pt_idx];
			fpreal force_n = force.length();
			fpreal mass = (*m)[pt_idx];
			fpreal volume = (*V)[pt_idx];
			fpreal density = (*rho)[pt_idx];
			pos_handle.set(pt_off, pos);
			vel_handle.set(pt_off, vel);
			force_handle.set(pt_off, force);
			force_n_handle.set(pt_off, force_n);
			mass_handle.set(pt_off, mass);
			volume_handle.set(pt_off, volume);
			density_handle.set(pt_off, density);

			int fn_sum = (*neighbor_this)[pt_idx];
			int bn_sum = (*neighbor_others)[pt_idx];
			self_n_sum_handle.set(pt_off, fn_sum);
			other_n_sum_handle.set(pt_off, bn_sum);
		}
}
