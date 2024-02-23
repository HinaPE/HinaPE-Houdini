#include "SIM_Hina_ParticlesVisualizer.h"

#include <Particles/SIM_Hina_Akinci2012BoundaryParticles.h>

const SIM_DopDescription *SIM_Hina_ParticlesNeighborsVisualizer::getDopDescription()
{
	static std::array<PRM_Template, 1> PRMS{
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "Hina_ParticlesNeighborsVisualizer",
								   "Hina_ParticlesNeighborsVisualizer",
								   "Hina_ParticlesNeighborsVisualizer",
								   classname(),
								   PRMS.data());

	static PRM_Name ShowGuideGeometry(SIM_NAME_SHOWGUIDE, "ShowGuideGeometry");
	static PRM_Name InspectOffset(SIM_NAME_OFFSET, "InspectOffset");
	static PRM_Name DomainColor("DomainColor", "DomainColor");
	static std::array<PRM_Default, 3> DomainColorDefault{.0156356, 0, .5};
	static std::array<PRM_Template, 4> PRMS_GUIDE{
			PRM_Template(PRM_TOGGLE, 1, &ShowGuideGeometry, PRMoneDefaults),
			PRM_Template(PRM_INT, 1, &InspectOffset, PRMzeroDefaults),
			PRM_Template(PRM_RGBA, 3, &DomainColor, DomainColorDefault.data()),
			PRM_Template()
	};
	DESC.setGuideTemplates(PRMS_GUIDE.data());
	return &DESC;
}
void SIM_Hina_ParticlesNeighborsVisualizer::buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const
{
	if (gdh.isNull())
		return;

	if (!getShowGuideGeometry(options))
		return;

	const SIM_Object *obj = (const SIM_Object *) &root;
	const SIM_Hina_Particles *fluid_particles = (const SIM_Hina_Particles *) obj->getConstNamedSubData(SIM_Hina_Particles::DATANAME);

	GU_DetailHandleAutoWriteLock gdl(gdh);
	GU_Detail *gdp = gdl.getGdp();
	gdp->clearAndDestroy();
	GA_RWHandleV3 cd_handle(gdp->findPointAttribute("Cd"));
	if (!cd_handle.isValid())
		cd_handle = GA_RWHandleV3(gdp->addFloatTuple(GA_ATTRIB_POINT, "Cd", 3, GA_Defaults(0.0)));
	GA_RWHandleF alpha_handle(gdp->findPointAttribute("Alpha"));
	if (!alpha_handle.isValid())
		alpha_handle = GA_RWHandleF(gdp->addFloatTuple(GA_ATTRIB_POINT, "Alpha", 1, GA_Defaults(0.0)));


	// ========== Visualize the fluid neighbors ==========
	if (fluid_particles->neighbor_lists_cache.empty())
		return;

	fpreal spacing = fluid_particles->getTargetSpacing();
	fpreal kernel_radius = fluid_particles->getKernelRadiusOverTargetSpacing() * spacing;
	fpreal p_radius = spacing /= 2;

	GA_Offset inspect = getInspectOffset(options);
	{
		SIM_GeometryAutoReadLock lock(fluid_particles);
		const GU_Detail *f_gdp = lock.getGdp();
		{
			GA_Offset new_pt_off = gdp->appendPoint();
			gdp->setPos3(new_pt_off, f_gdp->getPos3(inspect));
			GU_PrimSphereParms params;
			params.gdp = gdp;
			params.ptoff = new_pt_off;
			params.xform.scale(kernel_radius, kernel_radius, kernel_radius);
			params.xform.translate(UT_Vector3(0.));
			GEO_PrimSphere *sphere_prim = (GEO_PrimSphere *) GU_PrimSphere::build(params);

			UT_Vector3 _color = {1, 1, 1};
			GA_Offset start, end;
			for (GA_Iterator it(sphere_prim->getPointRange()); it.blockAdvance(start, end);)
				for (GA_Offset offset = start; offset < end; ++offset)
				{
					cd_handle.set(offset, _color);
					alpha_handle.set(offset, .2);
				}
		}
		for (const auto& neighbor: fluid_particles->neighbor_lists_cache.at(inspect))
		{
			GA_Offset n_off = neighbor.first;
			GA_Offset new_pt_off = gdp->appendPoint();
			gdp->setPos3(new_pt_off, f_gdp->getPos3(n_off));

			GU_PrimSphereParms params;
			params.gdp = gdp;
			params.ptoff = new_pt_off;
			params.xform.scale(p_radius, p_radius, p_radius);
			params.xform.translate(UT_Vector3(0.));
			GEO_PrimSphere *sphere_prim = (GEO_PrimSphere *) GU_PrimSphere::build(params);

			UT_Vector3 _color = {1, 0, 0};
			GA_Offset start, end;
			for (GA_Iterator it(sphere_prim->getPointRange()); it.blockAdvance(start, end);)
				for (GA_Offset offset = start; offset < end; ++offset)
				{
					cd_handle.set(offset, _color);
					alpha_handle.set(offset, 0.1);
				}
		}
	}


	// ========== Visualize the boundary neighbors ==========
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
		for (const auto& neighbor: fluid_particles->other_neighbor_lists_cache.at(name).at(inspect))
		{
			GA_Offset n_off = neighbor.first;
			GA_Offset new_pt_off = gdp->appendPoint();
			gdp->setPos3(new_pt_off, b_gdp->getPos3(n_off));

			GU_PrimSphereParms params;
			params.gdp = gdp;
			params.ptoff = new_pt_off;
			params.xform.scale(p_radius, p_radius, p_radius);
			params.xform.translate(UT_Vector3(0.));
			GEO_PrimSphere *sphere_prim = (GEO_PrimSphere *) GU_PrimSphere::build(params);

			UT_Vector3 _color = {1, 1, 1};
			GA_Offset start, end;
			for (GA_Iterator it(sphere_prim->getPointRange()); it.blockAdvance(start, end);)
				for (GA_Offset offset = start; offset < end; ++offset)
				{
					cd_handle.set(offset, _color);
					alpha_handle.set(offset, 1);
				}
		}
	}
}
SIM_Guide *SIM_Hina_ParticlesNeighborsVisualizer::createGuideObjectSubclass() const { return new SIM_GuideShared(this, true); }
