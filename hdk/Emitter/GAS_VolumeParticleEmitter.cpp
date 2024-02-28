//
// Created by LiYifan on 2024/2/27.
//

#include "GAS_VolumeParticleEmitter.h"
#include "Particles/SIM_FluidParticle.h"
#include "Geometry/TriangleMesh3.hpp"
#include "Geometry/ImplicitSurfaceSet.hpp"
#include "Utils/Logging.hpp"
#include "SIM_Hina_Generator.h"
#include "Searcher/PointParallelHashGridSearcher.hpp"
#include "PointGenerator/BccLatticePointGenerator.hpp"
#include "Utils/Samplers.hpp"
#include "Utils/Logging.hpp"

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_GuideShared.h>
#include <SIM/SIM_ColliderLabel.h>
#include <SIM/SIM_ForceGravity.h>
#include <SIM/SIM_Time.h>
#include <SIM/SIM_Utils.h>

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>
#include <PRM/PRM_Utils.h>
#include <PRM/PRM_SpareData.h>

#include <SOP/SOP_Node.h>

#include <GEO/GEO_PrimPoly.h>

#include <UT/UT_WorkBuffer.h>
#include <UT/UT_NetMessage.h>

bool
GAS_VolumeParticleEmitter::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    /*if (!InnerPtr)
    {
        if (!InitRuntime(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
        {
            SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
            return false;
        }
    }*/

    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_VolumeParticleEmitter::initializeSubclass() {
    SIM_Data::initializeSubclass();
    /*this->InnerPtr = nullptr;*/

    this->Emitted = false;
    this->EmittedParticles = 0;
}

void GAS_VolumeParticleEmitter::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
    const GAS_VolumeParticleEmitter *src = SIM_DATA_CASTCONST(source, GAS_VolumeParticleEmitter);
    /*this->InnerPtr = src->InnerPtr;*/
    this->EmittedParticles = src->EmittedParticles;
}

const char *GAS_VolumeParticleEmitter::DATANAME = "VolumeParticleEmitter";

const SIM_DopDescription *GAS_VolumeParticleEmitter::getDopDescription()
{
    static PRM_Name Jitter("Jitter", "Jitter");
    static PRM_Default JitterDefault(0.0);
    static PRM_Name MaxNumberOfParticles("MaxNumberOfParticles", "MaxNumberOfParticles");
    static PRM_Default MaxNumberOfParticlesDefault(1000);
    static PRM_Name IsOneShot("IsOneShot", "IsOneShot");
    static PRM_Default IsOneShotDefault(true);
    static PRM_Name IsEmitterMove("IsEmitterMove", "IsEmitterMove");
    static PRM_Default IsEmitterMoveDefault(false);
    static PRM_Name RandomSeed("RandomSeed", "RandomSeed");
    static PRM_Default RandomSeedDefault(0);


    static std::array<PRM_Template, 6> PRMS{
            PRM_Template(PRM_FLT, 1, &Jitter, &JitterDefault),
            PRM_Template(PRM_INT, 1, &MaxNumberOfParticles, &MaxNumberOfParticlesDefault),
            PRM_Template(PRM_TOGGLE, 1, &IsOneShot, &IsOneShotDefault),
            PRM_Template(PRM_TOGGLE, 1, &IsEmitterMove, &IsEmitterMoveDefault),
            PRM_Template(PRM_INT, 1, &RandomSeed, &RandomSeedDefault),
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "volume_particle_emitter",
                                   "Volume Particle Emitter",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_VolumeParticleEmitter::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
                                      UT_WorkBuffer &error_msg){
    if (!obj)
    {
        error_msg.appendSprintf("Object Is Null, From %s\n", DATANAME);
        return false;
    }

    SIM_GeometryCopy *geo = SIM_DATA_GET(*obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy);
    if (!geo)
    {
        error_msg.appendSprintf("Geometry Is Null, From %s\n", DATANAME);
        return false;
    }

    SIM_FluidParticle *particles = SIM_DATA_GET(*obj, SIM_FluidParticle::DATANAME, SIM_FluidParticle);
    if (!particles)
    {
        error_msg.appendSprintf("No Valid Target Data, From %s\n", DATANAME);
        return false;
    }

    if (!particles->Configured)
    {
        error_msg.appendSprintf("SIM_FluidParticle Not Configured Yet, From %s\n", DATANAME);
        return false;
    }

    CubbyFlow::Array1<CubbyFlow::Surface3Ptr> MultipleSurfaces;
    SIM_Geometry *src_geo = SIM_DATA_GET(*this, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (src_geo)
    {
        SIM_GeometryAutoReadLock lock(src_geo);
        const GU_Detail *gdp_source = lock.getGdp();
        if (!gdp_source)
        {
            error_msg.appendSprintf("Source Geometry GDP is nullptr, From %s\n", DATANAME);
            return false;
        }

        CubbyFlow::TriangleMesh3::PointArray points;
        CubbyFlow::TriangleMesh3::IndexArray point_indices;
        {
            GA_Offset pt_off;
            GA_FOR_ALL_PTOFF(gdp_source, pt_off)
                {
                    const UT_Vector3 pos = gdp_source->getPos3(pt_off);
                    points.Append({pos.x(), pos.y(), pos.z()});
                }

            const GEO_Primitive *prim;
            GA_FOR_ALL_PRIMITIVES(gdp_source, prim)
            {
                const auto *poly = dynamic_cast<const GEO_PrimPoly *>(prim);
                if (!poly)
                {
                    error_msg.appendSprintf("ERROR ON CONVERT PRIM TO POLY, From %s\n", DATANAME);
                    return false;
                }

                // Triangulate Polygon
                std::vector<size_t> polyIndices;
                for (int vi = 0; vi < poly->getVertexCount(); ++vi)
                    polyIndices.push_back(poly->getPointIndex(vi));
                for (size_t i = 1; i < polyIndices.size() - 1; ++i)
                    point_indices.Append({polyIndices[0], polyIndices[i + 1], polyIndices[i]}); // notice the normal
            }
        }
        CubbyFlow::TriangleMesh3Ptr mesh = CubbyFlow::TriangleMesh3::GetBuilder().WithPoints(points).WithPointIndices(point_indices).MakeShared();

        MultipleSurfaces.Append(mesh);
    }

    if (MultipleSurfaces.IsEmpty())
    {
        error_msg.appendSprintf("NO Source Geometry, From %s\n", DATANAME);
        return false;
    }

    CubbyFlow::ImplicitSurfaceSet3Ptr implicit = CubbyFlow::ImplicitSurfaceSet3::GetBuilder().WithExplicitSurfaces(MultipleSurfaces).MakeShared();

    bool IsOneShot = getIsOneShot();
    size_t MaxNumberOfParticles = getMaxNumberOfParticles();
    fpreal TargetSpacing = particles->getTargetSpacing();
    UT_Vector3 FluidDomain = particles->getFluidDomain();
    fpreal Jitter = getJitter();

    if(!implicit)
    {
        error_msg.appendSprintf("Implicit Surface Is Null, From %s\n", DATANAME);
        return false;
    }

    if (IsOneShot && Emitted)
        return true;

    // Fetch Existing Particles
    UT_Vector3Array exist_positions;
    {
        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail &gdp = lock.getGdp();
        GA_Offset pt_pff;
        GA_FOR_ALL_PTOFF(&gdp, pt_pff)
            {
                //std::cout << "pt_pff = " << pt_pff << std::endl;
                //std::cout << "gdp.getPos3(pt_pff) = " << gdp.getPos3(pt_pff) << std::endl;
                exist_positions.append(gdp.getPos3(pt_pff));
            }
    }

    if(exist_positions.size() == 0)
    {
        std::cout << "exist_positions.size() = " << exist_positions.size() << std::endl;
    }

    // Generate New Particles
    UT_Vector3Array new_positions;
    UT_Vector3Array new_velocities;
    GA_Size new_particles{};
    {
        using namespace CubbyFlow;
        Logging::Mute();

        /*CubbyFlow::ImplicitSurfaceSet3Ptr implicit = CubbyFlow::ImplicitSurfaceSet3::GetBuilder().WithExplicitSurfaces(MultipleSurfaces).MakeShared();*/

        const double maxJitterDist = 0.5 * Jitter * TargetSpacing;
        UT_Vector3 HalfFluidDomain = FluidDomain;
        HalfFluidDomain /= 2;
        BoundingBox3D region;
        region.lowerCorner = AS_CFVector3D(-HalfFluidDomain);
        region.upperCorner = AS_CFVector3D(HalfFluidDomain);
        if (implicit->IsBounded())
        {
            const BoundingBox3D surfaceBBox = implicit->GetBoundingBox();
            region.lowerCorner = Max(region.lowerCorner, surfaceBBox.lowerCorner);
            region.upperCorner = Min(region.upperCorner, surfaceBBox.upperCorner);
        }

        implicit->UpdateQueryEngine();

        int DEFAULT_HASH_GRID_RESOLUTION = 64;
        PointParallelHashGridSearcher3 neighbor_searcher(
                Vector3UZ(DEFAULT_HASH_GRID_RESOLUTION,
                          DEFAULT_HASH_GRID_RESOLUTION,
                          DEFAULT_HASH_GRID_RESOLUTION),
                2. * TargetSpacing);

        Array1<Vector3D> pos_array;
        for (auto & exist_position : exist_positions)
            pos_array.Append(AS_CFVector3D(exist_position));
        neighbor_searcher.Build(pos_array);

        // Point Generator
        auto points_gen = std::make_shared<BccLatticePointGenerator>();

        std::mt19937 m_rng(0);
        //std::cout << "111111" << std::endl;
        points_gen->ForEachPoint(region, TargetSpacing, [&](const Vector3D &point)
        {
            std::uniform_real_distribution<> d1{0.0, 1.0};
            std::uniform_real_distribution<> d2{0.0, 1.0};
            const Vector3D randomDir =
                    UniformSampleSphere(d1(m_rng), d2(m_rng));
            const Vector3D offset = maxJitterDist * randomDir;
            const Vector3D candidate = point + offset;

            //std::cout << "candidate = " << candidate.x << "," << candidate.y << "," << candidate.z << std::endl;

            if (implicit->IsInside(candidate) && !neighbor_searcher.HasNearbyPoint(candidate, TargetSpacing))
            {
                if (this->EmittedParticles >= MaxNumberOfParticles)
                    return false;
                new_positions.append(AS_UTVector3D(candidate));
                new_velocities.append(AS_UTVector3D(Vector3D()));
                this->Emitted = true;
                ++this->EmittedParticles;
            }
            return true;
        });
        //std::cout << "222222" << std::endl;
        new_particles = new_positions.size();
        std::cout << "new_particles = " << new_particles << std::endl;
    }

    SIM_GeometryAutoWriteLock lock(geo);
    GU_Detail &gdp = lock.getGdp();
    GA_RWHandleV3 pos_handle = gdp.getP();
    GA_RWHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
    for (GA_Size idx = 0; idx < new_particles; ++idx)
    {
        GA_Offset pt_off = gdp.appendPoint();
        UT_Vector3 pos = new_positions[idx];
        UT_Vector3 vel = new_velocities[idx];
        pos_handle.set(pt_off, pos);
        vel_handle.set(pt_off, vel);
    }
    particles->load(geo);

    return true;
}






