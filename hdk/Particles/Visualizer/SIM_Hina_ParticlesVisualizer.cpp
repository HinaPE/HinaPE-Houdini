#include "SIM_Hina_ParticlesVisualizer.h"

#include <Particles/SIM_Hina_Akinci2012BoundaryParticles.h>
#include <SOP/SOP_Node.h>

const SIM_DopDescription *SIM_Hina_ParticlesVisualizer::getDopDescription()
{
	static std::array<PRM_Template, 1> PRMS{
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "hina_particlesvisualizer",
								   "Hina Particles Visualizer",
								   "Hina_ParticlesVisualizer",
								   classname(),
								   PRMS.data());

	static PRM_Name ShowGuideGeometry(SIM_NAME_SHOWGUIDE, "ShowGuideGeometry");
	static PRM_Name InspectOffset(SIM_NAME_OFFSET, "InspectOffset");
	static std::array<PRM_Template, 3> PRMS_GUIDE{
			PRM_Template(PRM_TOGGLE, 1, &ShowGuideGeometry, PRMoneDefaults),
			PRM_Template(PRM_INT, 1, &InspectOffset, PRMzeroDefaults),
			PRM_Template()
	};
	DESC.setGuideTemplates(PRMS_GUIDE.data());
	return &DESC;
}
void SIM_Hina_ParticlesVisualizer::buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const
{
	if (gdh.isNull())
		return;

	if (!getShowGuideGeometry(options))
		return;

	const SIM_Object *obj = (const SIM_Object *) &root;
	const SIM_Hina_Particles *fluid_particles = (const SIM_Hina_Particles *) obj->getConstNamedSubData(SIM_Hina_Particles::DATANAME);

	if (fluid_particles->neighbor_lists_cache.empty())
		return;

	GU_DetailHandleAutoWriteLock gdl(gdh);
	GU_Detail *gdp = gdl.getGdp();
	gdp->clearAndDestroy();

	GA_Offset inspect = getInspectOffset(options);
	{
		SIM_GeometryAutoReadLock lock(fluid_particles);
		const GU_Detail *f_gdp = lock.getGdp();
		for (GA_Offset n_off: fluid_particles->neighbor_lists_cache.at(inspect))
		{
			GA_Offset new_pt_off = gdp->appendPoint();
			gdp->setPos3(new_pt_off, f_gdp->getPos3(n_off));

			GU_PrimSphereParms params;
			params.gdp = gdp;
			params.ptoff = new_pt_off;
			params.xform.scale(.005, .005, .005);
			params.xform.translate(UT_Vector3(0.));
			GU_PrimSphere::build(params);
		}
	}

	if (fluid_particles->other_neighbor_lists_cache.empty())
		return;

	SIM_ConstObjectArray affectors;
	obj->getConstAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		const SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(obj->getName()))
			continue;

		UT_String name = obj_collider->getName();
		if (fluid_particles->other_neighbor_lists_cache.find(name) ==
			fluid_particles->other_neighbor_lists_cache.end())
			continue;

		const SIM_Hina_Akinci2012BoundaryParticles *boundary_particles = (const SIM_Hina_Akinci2012BoundaryParticles *) obj_collider->getConstNamedSubData(SIM_Hina_Akinci2012BoundaryParticles::DATANAME);
		SIM_GeometryAutoReadLock lock(boundary_particles);
		const GU_Detail *b_gdp = lock.getGdp();
		for (GA_Offset n_off: fluid_particles->other_neighbor_lists_cache.at(name).at(inspect))
		{
			GA_Offset new_pt_off = gdp->appendPoint();
			gdp->setPos3(new_pt_off, b_gdp->getPos3(n_off));

			GU_PrimSphereParms params;
			params.gdp = gdp;
			params.ptoff = new_pt_off;
			params.xform.scale(.005, .005, .005);
			params.xform.translate(UT_Vector3(0.));
			GU_PrimSphere::build(params);
		}
	}
}
SIM_Guide *SIM_Hina_ParticlesVisualizer::createGuideObjectSubclass() const { return new SIM_GuideShared(this, false); }
