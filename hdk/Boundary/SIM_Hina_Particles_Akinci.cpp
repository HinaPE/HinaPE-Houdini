#include "SIM_Hina_Particles_Akinci.h"
#include <SIM/SIM_Position.h>
#include <Rigid/SIM_Hina_RigidBody.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Particles_Akinci,
		Particles,
		true,
		HINA_STRING_PARAMETER(TargetGeometryDATANAME, SIM_GEOMETRY_DATANAME) \
		HINA_FLOAT_PARAMETER(SolidDensity, 1000.) \
        HINA_FLOAT_PARAMETER(Buoyancy, 1.) \
        HINA_FLOAT_PARAMETER(RotationStiffness,1.) \
        HINA_BOOL_PARAMETER(IsDynamic, false) \
)
void SIM_Hina_Particles_Akinci::_init_Particles_Akinci()
{
	this->x_init = nullptr;
	this->xform = nullptr;
	this->center_of_mass = UT_Vector3(0, 0, 0);
	this->b_set_index = -1;
    this->SP = nullptr;
    this->normals = nullptr;
    this->u_diff = nullptr;
}
void SIM_Hina_Particles_Akinci::_makeEqual_Particles_Akinci(const SIM_Hina_Particles_Akinci *src)
{
	this->x_init = src->x_init;
	this->xform = src->xform;
	this->center_of_mass = src->center_of_mass;
	this->b_set_index = src->b_set_index;
    this->SP = src->SP;
    this->normals = src->normals;
    this->u_diff = src->u_diff;
}
void SIM_Hina_Particles_Akinci::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
    HINA_GEOMETRY_POINT_ATTRIBUTE("SP", HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
    HINA_GEOMETRY_POINT_ATTRIBUTE("Normal", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
    HINA_GEOMETRY_POINT_ATTRIBUTE("U_diff", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
}

void SIM_Hina_Particles_Akinci::commit()
{
    SIM_Hina_Particles::commit();

    if (x_init == nullptr)
    {
        std::cout << "SIM_Hina_Particles_Akinci::load() called with nullptr" << std::endl;
        return;
    }

    size_t size = x_init->size();
    if (size == 0)
        return;

    SIM_GeometryAutoWriteLock lock(this);
    GU_Detail &gdp = lock.getGdp();
    GA_RWHandleI sp_handle = gdp.findPointAttribute("SP");
    GA_RWHandleV3 normal_handle = gdp.addFloatTuple(GA_ATTRIB_POINT, "Normal", 3);
    GA_RWHandleV3 u_diff_handle = gdp.addFloatTuple(GA_ATTRIB_POINT, "U_diff", 3);
    GA_Offset pt_off;
    GA_FOR_ALL_PTOFF(&gdp, pt_off)
        {
            int sp = (*SP)[offset2index[pt_off]];
            sp_handle.set(pt_off, sp);
            Vector normal = (*normals)[offset2index[pt_off]];
            normal_handle.set(pt_off, normal);
            Vector uDiff = (*u_diff)[offset2index[pt_off]];
            u_diff_handle.set(pt_off, uDiff);
        }
}

/// Fetch all akinci boundaries from [fluid_obj]
auto FetchAllAkinciBoundaries(SIM_Object *fluid_obj) -> std::vector<SIM_Hina_Particles_Akinci *>
{
	std::vector<SIM_Hina_Particles_Akinci *> res;
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		SIM_Hina_Particles_Akinci *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Particles_Akinci::DATANAME, SIM_Hina_Particles_Akinci);
		if (boundary_akinci)
			res.emplace_back(boundary_akinci);
	}
	return res;
}

/// Load sop geometry into [x_init] ONLY (ensure [x_init] is already mapped to the fluid solver, aka, [x_init] is not nullptr) (Fluid solver would deal with the `size` problems)
void InitAllAkinciBoundaries(SIM_Object *fluid_obj)
{
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		SIM_Hina_Particles_Akinci *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Particles_Akinci::DATANAME, SIM_Hina_Particles_Akinci);
		if (boundary_akinci)
		{
			(*boundary_akinci->x_init).clear();
			{
				SIM_Geometry *boundary_sop = SIM_DATA_GET(*obj_collider, boundary_akinci->getTargetGeometryDATANAME().c_str(), SIM_Geometry);
				if (!boundary_sop)
					return;
				SIM_GeometryAutoReadLock lock(boundary_sop);
				const GU_Detail *gdp = lock.getGdp();

				UT_Vector3 center_of_mass{0, 0, 0};
				{
					GA_Offset pt_off;
					GA_FOR_ALL_PTOFF(gdp, pt_off)
						{
							UT_Vector3 pos = gdp->getPos3(pt_off);
							center_of_mass += pos;
						}
					center_of_mass /= gdp->getNumPoints();
				}
				boundary_akinci->center_of_mass = center_of_mass;

                //std::cout << center_of_mass << std::endl;

				(*boundary_akinci->x_init).reserve(gdp->getNumPoints());
				GA_Offset pt_off;
				GA_FOR_ALL_PTOFF(gdp, pt_off)
					{
						UT_Vector3 pos = gdp->getPos3(pt_off);
						(*boundary_akinci->x_init).emplace_back(pos - center_of_mass);
                        (*boundary_akinci->SP).emplace_back(0);
                        (*boundary_akinci->normals).emplace_back(0, 0, 0);
                        (*boundary_akinci->u_diff).emplace_back(0, 0, 0);
					}
			}
		}
	}
}

/// Update Transforms of all Akinci boundaries, and update to [xform] (ensure [xform] is already mapped to the fluid solver, aka, [xform] is not nullptr)
void UpdateAllAkinciBoundaries(SIM_Object *fluid_obj)
{
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		SIM_Hina_Particles_Akinci *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Particles_Akinci::DATANAME, SIM_Hina_Particles_Akinci);
		if (boundary_akinci)
		{
			SIM_Hina_RigidBody *rb = SIM_DATA_GET(*obj_collider, SIM_Hina_RigidBody::DATANAME, SIM_Hina_RigidBody);
			UT_DMatrix4 xform; // this is row major matrix
			xform.identity();
			if (rb && rb->rb)
			{
				rb->b_set_index = boundary_akinci->b_set_index;

				auto _t = rb->rb->getTransform();
				auto _p = _t.getPosition();
//                std::cout << _p.x << " " << _p.y << " " << _p.z << std::endl;
//                std::cout << std::endl;
				auto _q = _t.getOrientation();

				const reactphysics3d::Matrix3x3 &rq = _q.getMatrix(); // this is also a col major matrix
                const reactphysics3d::Matrix3x3 matrix = rq.getTranspose();

                /*////Test
                for(int i = 0; i < 3; i++)
                {
                    for(int j = 0; j < 3; j++)
                    {
                        std::cout << matrix[i][j] << " ";
                    }
                    std::cout << std::endl;
                }
                std::cout << std::endl;*/

				UT_DMatrix4 final;

				xform[0][0] = matrix[0][0];
				xform[0][1] = matrix[0][1];
				xform[0][2] = matrix[0][2];
				xform[0][3] = 0;

				xform[1][0] = matrix[1][0];
				xform[1][1] = matrix[1][1];
				xform[1][2] = matrix[1][2];
				xform[1][3] = 0;

				xform[2][0] = matrix[2][0];
				xform[2][1] = matrix[2][1];
				xform[2][2] = matrix[2][2];
				xform[2][3] = 0;

				xform[3][0] = _p.x;
				xform[3][1] = _p.y;
				xform[3][2] = _p.z;
				xform[3][3] = 1;
			} else
			{
				UT_Vector3 center_of_mass = boundary_akinci->center_of_mass;
//                std::cout << "1:" << center_of_mass << std::endl;
                center_of_mass = UT_Vector3(0, 0, 0);
//                std::cout << "2:" << center_of_mass << std::endl;
				xform[3][0] = center_of_mass.x();
				xform[3][1] = center_of_mass.y();
				xform[3][2] = center_of_mass.z();
			}
			(*boundary_akinci->xform) = xform;
            //std::cout << xform << std::endl;
		}
	}
}
