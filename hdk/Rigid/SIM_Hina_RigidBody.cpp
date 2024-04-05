#include "SIM_Hina_RigidBody.h"

SIM_HINA_DATA_IMPLEMENT(
		RigidBody,
		true,
		HINA_STRING_PARAMETER(TargetGeometryDATANAME, "GeometryConvex") \
		HINA_BOOL_PARAMETER(IsDynamic, true) \
		HINA_FLOAT_PARAMETER(Mass, 1.) \
		HINA_FLOAT_PARAMETER(Bounciness, .4) \
        HINA_FLOAT_PARAMETER(Friction, .02) \
)

void SIM_Hina_RigidBody::_init_RigidBody()
{
	this->rb = nullptr;
	this->b_set_index = -1;

	this->V = nullptr;
	this->I = nullptr;
	this->F = nullptr;
	this->v_size = 0;
	this->i_size = 0;
	this->F_size = 0;
}
void SIM_Hina_RigidBody::_makeEqual_RigidBody(const SIM_Hina_RigidBody *src)
{
	this->rb = src->rb;
	this->b_set_index = src->b_set_index;

	this->V = src->V;
	this->I = src->I;
	this->F = src->F;
	this->v_size = src->v_size;
	this->i_size = src->i_size;
	this->F_size = src->F_size;
}

void SIM_Hina_RigidBody::buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const
{
	if (gdh.isNull()) return;
	GU_DetailHandleAutoWriteLock gdl(gdh);
	GU_Detail *gdp = gdl.getGdp();
	gdp->clearAndDestroy();

	if (rb)
	{
		for (int i = 0; i < v_size; ++i)
		{
			reactphysics3d::Vector3 pos_final = WorldTransformCache * reactphysics3d::Vector3{V[i * 3 + 0], V[i * 3 + 1], V[i * 3 + 2]};
			GA_Offset pt_off = gdp->appendPoint();
			gdp->setPos3(pt_off, UT_Vector3F{pos_final.x, pos_final.y, pos_final.z});
		}

		for (int i = 0; i < F_size; ++i)
		{
			reactphysics3d::PolygonVertexArray::PolygonFace &face = F[i];
			GEO_PrimPoly *prim = GEO_PrimPoly::build(gdp, face.nbVertices, GU_POLY_CLOSED, 0);
			for (int j = 0; j < face.nbVertices; ++j)
			{
				int pt_idx = I[face.indexBase + j];
				prim->setVertexPoint(j, gdp->pointOffset(pt_idx));
			}
		}
	}
}
SIM_Guide *SIM_Hina_RigidBody::createGuideObjectSubclass() const
{
	return new SIM_GuideShared(this, true);
}

auto FetchAllRigidBodies(SIM_Object *obj) -> std::vector<SIM_Hina_RigidBody *>
{
	std::vector<SIM_Hina_RigidBody *> res;
	SIM_ObjectArray affectors;
	obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		SIM_Hina_RigidBody *rigidbody = SIM_DATA_GET(*obj_collider, SIM_Hina_RigidBody::DATANAME, SIM_Hina_RigidBody);
		if (rigidbody)
			res.emplace_back(rigidbody);
	}
	return res;
}
void InitAllRigidBodies(SIM_Object *obj, reactphysics3d::PhysicsCommon &physicsCommon, reactphysics3d::PhysicsWorld *world)
{
	SIM_ObjectArray affectors;
	obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		// include self
		SIM_Object *obj_collider = affectors(i);
		SIM_Hina_RigidBody *rigidbody = SIM_DATA_GET(*obj_collider, SIM_Hina_RigidBody::DATANAME, SIM_Hina_RigidBody);
		if (rigidbody)
		{
			SIM_Geometry *SOPGeometry = SIM_DATA_GET(*obj_collider, rigidbody->getTargetGeometryDATANAME().c_str(), SIM_Geometry);
			SIM_GeometryAutoReadLock lock(SOPGeometry);
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

            /*std::cout << center_of_mass << std::endl;
            std::cout << std::endl;*/

			float *vertices = new float[gdp->getNumPoints() * 3];
			{
				GA_Offset pt_off;
				GA_FOR_ALL_PTOFF(gdp, pt_off)
					{
						GA_Index pt_idx = gdp->pointIndex(pt_off);
						UT_Vector3 pos = gdp->getPos3(pt_off);
						vertices[pt_idx * 3 + 0] = pos.x() - center_of_mass.x();
						vertices[pt_idx * 3 + 1] = pos.y() - center_of_mass.y();
						vertices[pt_idx * 3 + 2] = pos.z() - center_of_mass.z();
					}
			}
			rigidbody->V = vertices;
			rigidbody->v_size = gdp->getNumPoints();
			GA_Size prim_vertex_count = 0;
			{
				const GEO_Primitive *prim;
				GA_FOR_ALL_PRIMITIVES(gdp, prim)
				{
					prim_vertex_count += prim->getVertexCount();
				}
			}
			int *indices = new int[prim_vertex_count];
			reactphysics3d::PolygonVertexArray::PolygonFace *faces = new reactphysics3d::PolygonVertexArray::PolygonFace[gdp->getNumPrimitives()];
			{
				size_t temp_idx = 0;
				reactphysics3d::PolygonVertexArray::PolygonFace *face = faces;
				const GEO_Primitive *prim;
				GA_FOR_ALL_PRIMITIVES(gdp, prim)
				{
					face->indexBase = temp_idx;
					face->nbVertices = prim->getVertexCount();
					face++;
					for (int _ = prim->getVertexCount() - 1; _ >= 0; --_) // [VERY IMPORTANT!] reverse order
					{
						GA_Index pt_idx = prim->getPointIndex(_);
						indices[temp_idx++] = pt_idx;
					}
				}
			}
			rigidbody->I = indices;
			rigidbody->i_size = prim_vertex_count;
			rigidbody->F = faces;
			rigidbody->F_size = gdp->getNumPrimitives();
			reactphysics3d::PolygonVertexArray *PVA = new reactphysics3d::PolygonVertexArray(
					gdp->getNumPoints(),
					vertices,
					3 * sizeof(float),
					indices,
					sizeof(int),
					gdp->getNumPrimitives(),
					faces,
					reactphysics3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
					reactphysics3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
			reactphysics3d::PolyhedronMesh *PM = physicsCommon.createPolyhedronMesh(PVA);
			reactphysics3d::ConvexMeshShape *CMS = physicsCommon.createConvexMeshShape(PM);
			reactphysics3d::Transform transform;
			transform.setPosition({center_of_mass.x(), center_of_mass.y(), center_of_mass.z()});
			transform.setOrientation(reactphysics3d::Quaternion::identity());
			rigidbody->rb = world->createRigidBody(transform);
			rigidbody->rb->setMass(rigidbody->getMass());
			rigidbody->rb->setType(rigidbody->getIsDynamic() ? reactphysics3d::BodyType::DYNAMIC : reactphysics3d::BodyType::STATIC);

			reactphysics3d::Collider *collider = rigidbody->rb->addCollider(CMS, reactphysics3d::Transform::identity());
			collider->getMaterial().setBounciness(rigidbody->getBounciness());
			collider->getMaterial().setFrictionCoefficient(rigidbody->getFriction());

			rigidbody->rb->updateMassPropertiesFromColliders();
		}
	}
}
void UpdateAllRigidBodies(SIM_Object *obj)
{
	SIM_ObjectArray affectors;
	obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		// include self
		SIM_Object *obj_collider = affectors(i);
		SIM_Hina_RigidBody *rigidbody = SIM_DATA_GET(*obj_collider, SIM_Hina_RigidBody::DATANAME, SIM_Hina_RigidBody);
		if (rigidbody)
		{
			rigidbody->WorldTransformCache = rigidbody->rb->getTransform();
            /*std::cout << rigidbody->WorldTransformCache.getPosition().x << " " << rigidbody->WorldTransformCache.getPosition().y << " " << rigidbody->WorldTransformCache.getPosition().z << std::endl;
            std::cout << std::endl;*/
		}
	}
}
