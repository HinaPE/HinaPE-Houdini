#include "SIM_Hina_RigidBody.h"

SIM_HINA_DATA_IMPLEMENT(
		RigidBody,
		true,
		HINA_BOOL_PARAMETER(IsDynamic, true)
)

void SIM_Hina_RigidBody::_init_RigidBody()
{
	this->rb = nullptr;
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
		auto &pos = rb->getTransform().getPosition();
		for (int i = 0; i < v_size; ++i)
		{
			GA_Offset pt_off = gdp->appendPoint();
			gdp->setPos3(pt_off, UT_Vector3{V[i * 3 + 0] + pos.x, V[i * 3 + 1] + pos.y, V[i * 3 + 2] + pos.z});
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
			SIM_Geometry *SOPGeometry = SIM_DATA_GET(*obj_collider, "GeometryMesh", SIM_Geometry);
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

						std::cout << pt_idx << ": " << vertices[pt_idx * 3 + 0] << " " << vertices[pt_idx * 3 + 1] << " " << vertices[pt_idx * 3 + 2] << std::endl;
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
					for (int _ = 0; _ < prim->getVertexCount(); ++_)
					{
						GA_Index pt_idx = prim->getPointIndex(_);
						std::cout << pt_idx << " ";
						indices[temp_idx++] = pt_idx;
					}
					std::cout << std::endl;
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
			rigidbody->rb->setMass(5);
			rigidbody->rb->setType(rigidbody->getIsDynamic() ? reactphysics3d::BodyType::DYNAMIC : reactphysics3d::BodyType::STATIC);

			reactphysics3d::Collider *collider = rigidbody->rb->addCollider(CMS, reactphysics3d::Transform::identity());
			collider->getMaterial().setBounciness(0.9);
			collider->getMaterial().setFrictionCoefficient(0.01);

			rigidbody->rb->updateMassPropertiesFromColliders();
		}
	}
}
void InitAllRigidBodiesTest(SIM_Object *obj, reactphysics3d::PhysicsCommon &physicsCommon, reactphysics3d::PhysicsWorld *world)
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
			SIM_Geometry *SOPGeometry = SIM_DATA_GET(*obj_collider, "GeometryMesh", SIM_Geometry);
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

			{
				using namespace reactphysics3d;
				float *vertices = new float [24];
				float sz = 0.5;
				vertices [0] = -sz; vertices [1] = -sz; vertices [2] = sz;
				vertices [3] = sz; vertices [4] = -sz; vertices [5] = sz;
				vertices [6] = sz; vertices [7] = -sz; vertices [8] = -sz;
				vertices [9] = -sz; vertices [10] = -sz; vertices [11] = -sz;
				vertices [12] = -sz; vertices [13] = sz; vertices [14] = sz;
				vertices [15] = sz; vertices [16] = sz; vertices [17] = sz;
				vertices [18] = sz; vertices [19] = sz; vertices [20] = -sz;
				vertices [21] = -sz; vertices [22] = sz; vertices [23] = -sz;
				// Array with the vertices indices for each face of the mesh
				int *indices = new int [24];
				indices [0]=0; indices [1]=3; indices [2]=2; indices [3]=1;
				indices [4]=4; indices [5]=5; indices [6]=6; indices [7]=7;
				indices [8]=0; indices [9]=1; indices [10]=5; indices [11]=4;
				indices [12]=1; indices [13]=2; indices [14]=6; indices [15]=5;
				indices [16]=2; indices [17]=3; indices [18]=7; indices [19]=6;
				indices [20]=0; indices [21]=4; indices [22]=7; indices [23]=3;
				// Description of the six faces of the convex mesh
				PolygonVertexArray :: PolygonFace * polygonFaces = new
						PolygonVertexArray :: PolygonFace [6];
				PolygonVertexArray :: PolygonFace * face = polygonFaces ;
				for (int f = 0; f < 6; f++) {
					// First vertex of the face in the indices array
					face -> indexBase = f * 4;
					// Number of vertices in the face
					face -> nbVertices = 4;
					face ++;
				}
				// Create the polygon vertex array
				PolygonVertexArray * polygonVertexArray = new
						PolygonVertexArray (8 , vertices , 3 * sizeof ( float ) ,
											indices , sizeof (int) , 6, polygonFaces ,
											PolygonVertexArray :: VertexDataType :: VERTEX_FLOAT_TYPE ,
											PolygonVertexArray :: IndexDataType :: INDEX_INTEGER_TYPE );
				// Create the polyhedron mesh
				PolyhedronMesh * polyhedronMesh = physicsCommon.createPolyhedronMesh ( polygonVertexArray );
				ConvexMeshShape * convexMeshShape = physicsCommon.createConvexMeshShape ( polyhedronMesh );

				reactphysics3d::Transform transform;
				transform.setPosition({center_of_mass.x(), center_of_mass.y(), center_of_mass.z()});
				transform.setOrientation(reactphysics3d::Quaternion::identity());
				rigidbody->rb = world->createRigidBody(transform);
				rigidbody->rb->setMass(5);
				rigidbody->rb->setType(rigidbody->getIsDynamic() ? reactphysics3d::BodyType::DYNAMIC : reactphysics3d::BodyType::STATIC);

				reactphysics3d::Collider *collider = rigidbody->rb->addCollider(convexMeshShape, reactphysics3d::Transform::identity());
				collider->getMaterial().setBounciness(0.9);
				collider->getMaterial().setFrictionCoefficient(0.01);

				rigidbody->rb->updateMassPropertiesFromColliders();


				rigidbody->V = vertices;
				rigidbody->v_size = 8;
				rigidbody->I = indices;
				rigidbody->i_size = 24;
				rigidbody->F = polygonFaces;
				rigidbody->F_size = 6;
			}
		}
	}
}
