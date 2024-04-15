//
// Created by LiYifan on 2024/4/15.
//

#include "GAS_Hina_Solver_DFSPH.h"
#include "SpatialGrid.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_GeometryCopy.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>

#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ParallelPipeline.h>

#include <array>
#include <iostream>

UT_StringHolder AccelAttributeName("accel");
UT_StringHolder DensityAttributeName("density");
UT_StringHolder PressureAttributeName("pressure");
UT_StringHolder OneDensityAttributeName("oneOverDensity");

fpreal SPHSolver::_halfH = 0.0;
fpreal SPHSolver::_kernelValueCoeff = 0.0;
fpreal SPHSolver::_kernelGradientCoeffA = 0.0;
fpreal SPHSolver::_kernelGradientCoeffB = 0.0;

const SIM_DopDescription *SPHSolver::GetDescription() {
    static PRM_Name volume_min("volume_min", "Volume Min");
    static std::array<PRM_Default, 3> volume_min_default = {-3, 0, -1};

    static PRM_Name volume_max("volume_max", "Volume Max");
    static std::array<PRM_Default, 3> volume_max_default = {3, 3, 1};

    static PRM_Name mass("mass", "Mass");
    static PRM_Default mass_default(1.0);

    static PRM_Name rest_density("rest_density", "Rest Density");
    static PRM_Default rest_density_default(998.23);

    static PRM_Name pressure_constant("pressure_constant", "Pressure Constant");
    static PRM_Default pressure_constant_default(100.0);

    static PRM_Name time_step("time_step", "Time Step");
    static PRM_Default time_step_default(0.005);

    static PRM_Name kernel_radius("kernel_radius", "Kernel Radius");
    static PRM_Default kernel_radius_default(0.2);

    static PRM_Name bulk_viscosity("bulk_viscosity", "Bulk Viscosity");
    static PRM_Default bulk_viscosity_default(0.0);

    static PRM_Name shear_viscosity("shear_viscosity", "Shear Viscosity");
    static PRM_Default shear_viscosity_default(0.0);

    static std::array<PRM_Template, 10> PRMS{
            PRM_Template(PRM_XYZ_J, 3, &volume_min, volume_min_default.data()),
            PRM_Template(PRM_XYZ_J, 3, &volume_max, volume_max_default.data()),
            PRM_Template(PRM_FLT_J, 1, &mass, &mass_default),
            PRM_Template(PRM_FLT_J, 1, &rest_density, &rest_density_default),
            PRM_Template(PRM_FLT_J, 1, &pressure_constant, &pressure_constant_default),
            PRM_Template(PRM_FLT_J, 1, &time_step, &time_step_default),
            PRM_Template(PRM_FLT_J, 1, &kernel_radius, &kernel_radius_default),
            PRM_Template(PRM_FLT_J, 1, &bulk_viscosity, &bulk_viscosity_default),
            PRM_Template(PRM_FLT_J, 1, &shear_viscosity, &shear_viscosity_default),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "basicSph_solver",
                                   "Basic SPH Solver",
                                   "BasicSPHSolver",
                                   classname(),
                                   PRMS.data());
    return &DESC;
}

SIM_Solver::SIM_Result
SPHSolver::solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects,
                                     const SIM_Time &timestep, bool newobject) {
    Log.reset();
    static bool NeedReBuild = true;
    if (NeedReBuild || newobject) {
        init(object);
        NeedReBuild = false;
    }else{
        solve(object, timestep);
    }
    return Log.report();
}

void SPHSolver::init(SIM_Object &obj)
{
    std::cout << "Initializing simulator..." << std::endl;

    SIM_GeometryCopy *geo;
    geo = SIM_DATA_CREATE(obj, "Geometry", SIM_GeometryCopy,
                          SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
    if (!geo)
        Log.error_nullptr("INIT::SIM_GeometryCopy");

    {
        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail &gdp = lock.getGdp();

        {
            GA_RWAttributeRef vel_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, gdp.getStdAttributeName(GEO_ATTRIBUTE_VELOCITY), 3, GA_Defaults(0));
            GA_RWAttributeRef accel_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, AccelAttributeName, 3, GA_Defaults(0));
            GA_RWAttributeRef density_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, DensityAttributeName, 1, GA_Defaults(0));
            GA_RWAttributeRef pressure_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, PressureAttributeName, 1, GA_Defaults(0));
            GA_RWAttributeRef oneOverDensity_ref = gdp.addFloatTuple(GA_ATTRIB_POINT, OneDensityAttributeName, 1, GA_Defaults(0));
        }
    }
    precomputeKernelCoefficients();
}

namespace
{
    // This is much faster than calling pow(val, exponent)
    inline double pow2(double val) { return val*val; }
    inline double pow3(double val) { return val*val*val; }
    inline double pow7(double val) { return val*val*val*val*val*val*val; }
}

void SPHSolver::precomputeKernelCoefficients() const {
    const fpreal PI = 3.14159265359;
    const fpreal h = getKernelRadius();

    _halfH = h/2.0;	// In Monaghan2005, h=half of smoothing radius

    // Precompute value coefficient (Identical for part A and B)
    _kernelValueCoeff = 1.0 / (4.0*PI*pow(_halfH,3));

    // Precompute gradient coefficients
    _kernelGradientCoeffA = 3.0 / (4.0*PI*pow(_halfH,4));
    _kernelGradientCoeffB = -3.0 / (4.0*PI*pow(_halfH,4));
}

void SPHSolver::solve(SIM_Object &obj, const SIM_Time &dt){
    const double frameTime = 1.0/24.0;
    const double totalSimulationTime = 1.0;
    double time = 0.0;
    int currentFrame = 2;

/*    while (time < totalSimulationTime)
    {*/
    // Run simulation
    run(frameTime, obj);

    // Update simulation time
/*        time += frameTime;
        ++currentFrame;
    }*/
}

void SPHSolver::run(fpreal time, SIM_Object &obj) {

    const double dt = getTimeStep();
    double timeLeft = time;

    const fpreal _h = getKernelRadius();
    const UT_Vector3 _volumeMin = getVolumeMin();
    const UT_Vector3 _volumeMax = getVolumeMax();
    const fpreal _restDensity = getRestDensity();
    const fpreal _k = getPressureConstant();
    const fpreal _mass = getMass();

    const double alpha = getBulkViscosity();	// Bulk viscosity
    const double beta = getShearViscosity();	// Shear viscosity

    SIM_GeometryCopy *geo;
    geo = SIM_DATA_CREATE(obj, "Geometry", SIM_GeometryCopy,
                          SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
    if (!geo)
        Log.error_nullptr("INIT::SIM_GeometryCopy");

    {
        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail &gdp = lock.getGdp();

        GA_RWHandleV3 pos_handle = gdp.findPointAttribute("P");
        GA_RWHandleV3 vel_handle = gdp.findPointAttribute(gdp.getStdAttributeName(GEO_ATTRIBUTE_VELOCITY));
        GA_RWHandleV3 accel_handle = gdp.findPointAttribute(AccelAttributeName);
        GA_RWHandleF density_handle = gdp.findPointAttribute(DensityAttributeName);
        GA_RWHandleF pressure_handle = gdp.findPointAttribute(PressureAttributeName);
        GA_RWHandleF oneOverDensity_handle = gdp.findPointAttribute(OneDensityAttributeName);

        long long particle_size = gdp.getNumPoints();

/*        while (timeLeft > 0.0)
        {*/
        // Run simulation steps

        /// 1.buildNeighbors
        std::vector<std::vector<Neighbor> >	_neighbors;
        // Reserve space and initialize neighbors' data
        _neighbors.clear();
        _neighbors.resize(particle_size);
        // Init spatial grid
        UT_Vector3 borders(_h*2.0, _h*2.0, _h*2.0);
        UT_Vector3 gridMin = _volumeMin;
        gridMin -= borders;
        UT_Vector3 gridMax = _volumeMax;
        gridMax += borders;
        SpatialGrid<long> grid(_h, gridMin, gridMax);
        // Insert particles into grid
        for (long p = 0; p < particle_size; ++p)
        {
            GA_Offset offset = gdp.pointOffset(p);
            UT_Vector3 pos = pos_handle.get(offset);
            grid.insert(p, pos);
        }
        // Use grid to retrieve neighbor particles
        double h2 = _h*_h;
        std::vector<long*> nearbyParticles;
        for (long p=0; p<particle_size; ++p)
        {
            GA_Offset offset = gdp.pointOffset(p);
            UT_Vector3 pos = pos_handle.get(offset);
            // Get nearby particles
            grid.getElements(pos, _h, nearbyParticles);

            // Find particles that are within smoothing radius
            _neighbors[p].reserve(50);
            for (auto & nearbyParticle : nearbyParticles)
            {
                long nID = *nearbyParticle;
                GA_Offset nearbyOffset = gdp.pointOffset(nID);
                UT_Vector3 nearbyPos = pos_handle.get(nearbyOffset);

                // Skip current particle
                if (nID==p)
                    continue;

                UT_Vector3 xij = pos - nearbyPos;

                // Check if distance is lower than smoothing radius
                double dist2 = xij.dot(xij);
                if (dist2 < h2)
                {
                    // Yup! Add the particle to the neighbors list along with
                    // some precomputed informations
                    _neighbors[p].emplace_back(nID, xij, sqrt(dist2));
                }
            }
        }

        /// 2.computeDensityAndPressure
        // Precompute Taits coefficient
        const fpreal B = (_k * _restDensity) / 7.0;

        // Iterate particles
        for (long p=0; p<particle_size; ++p)
        {
            GA_Offset offset = gdp.pointOffset(p);

            // Reinitialize particle properties
            density_handle.set(offset, 0.0);
            oneOverDensity_handle.set(offset, 0.0);
            accel_handle.set(offset, UT_Vector3(0,0,0));
            pressure_handle.set(offset, 0.0);
            oneOverDensity_handle.set(offset, 0.0);

            // Add current particle's contribution

            fpreal particle_density = _mass * getKernelValue(0.0);

            // Compute density from neighbors contributions
            // rho_i = SUM_j (m_j * Wij)
            for (const auto & neighbor : _neighbors[p])
            {
                // Add contribution
                particle_density += _mass * getKernelValue(neighbor.dist);
            }

            // Precompute 1/rho and compute pressure using Tait's equation:
            // p_i = B * ((rho/rho_rest)^7-1)
            if (particle_density != 0.0)
            {
                oneOverDensity_handle.set(offset, 1.0/particle_density);
                density_handle.set(offset, particle_density);
                pressure_handle.set(offset, B * (pow7(particle_density/_restDensity) - 1.0));
            }
        }

        /// 3.addExternalForces
        for (long p = 0; p < particle_size; ++p)
        {
            GA_Offset offset = gdp.pointOffset(p);
            UT_Vector3 accel = accel_handle.get(offset);
            accel += UT_Vector3(0.0, -9.81, 0.0);
            accel_handle.set(offset, accel);
        }

        /// 4.computeArtificialViscosityForces
        // Precompute coefficients
        const double speedOfSound = sqrt(_k);	// c
        // Compute artificial viscosity forces
        for (long p = 0; p < particle_size; ++p)
        {
            GA_Offset offset = gdp.pointOffset(p);

            // No need to compute current particle's contribution, since its gradient is null!

            // Get neighbors contributions
            for (long n=0; n<_neighbors[p].size(); ++n)
            {
                const Neighbor &neighbor = _neighbors[p][n];
                GA_Offset neighborOffset = gdp.pointOffset(neighbor.id);

                // Compute contribution (based on the paper of Monaghan (1992))
                // fv_i/rho_i = SUM_j(m_j * IIij * gradient(Wij))
                //         | (alpha * c * uij + beta * uij^2) / avgRho,	when vij.xij < 0
                // IIij = -|
                //         | 0,											otherwise
                // uij = h * (vij.xij) / (|xij|^2 + 0.01*h^2)
                // vij = vi - vj
                // xij = xi - xj
                // avgRho = 0.5 * (rho_i + rho_j)
                UT_Vector3 vij = vel_handle.get(offset) - vel_handle.get(neighborOffset);
                double vijxij = vij.dot(neighbor.xij);
                double dij = neighbor.dist;
                double uij = _h*vijxij / (dij*dij + 0.01*h2);
                if (uij < 0)
                {
                    // Compute contribution
                    double avgDensity = 0.5 * (density_handle.get(offset) + density_handle.get(neighborOffset));
                    double IIij = (alpha*uij*speedOfSound + beta*uij*uij) / avgDensity;
                    UT_Vector3 contribution = getKernelGradient(neighbor.dist, neighbor.xij);
                    contribution *= IIij;
                    contribution *= _mass;

                    // Add contribution
                    UT_Vector3 accel = accel_handle.get(offset);
                    accel += contribution;
                    accel_handle.set(offset, accel);
                }
            }
        }

        /// 5.computePressureForces
        // Compute pressure forces
        for (long p = 0; p < particle_size; ++p)
        {
            GA_Offset offset = gdp.pointOffset(p);

            // No need to compute current particle's contribution, since its gradient is null!

            // Get neighbors contributions
            // fp_i/rho_i = -SUM_j (m_j * (pi/rho_i^2 + pj/rho_j^2) * gradient(Wij)
            for (long n = 0; n < _neighbors[p].size(); ++n)
            {
                const Neighbor &neighbor = _neighbors[p][n];
                GA_Offset neighborOffset = gdp.pointOffset(neighbor.id);

                // Compute contribution
                UT_Vector3 contribution = getKernelGradient(neighbor.dist, neighbor.xij);
                contribution *= (pressure_handle.get(offset) * oneOverDensity_handle.get(offset) * oneOverDensity_handle.get(offset)) +
                                (pressure_handle.get(neighborOffset) * oneOverDensity_handle.get(neighborOffset) * oneOverDensity_handle.get(neighborOffset));
                contribution *= -1.0;
                contribution *= _mass;

                // Add contribution
                UT_Vector3 accel = accel_handle.get(offset);
                accel += contribution;
                accel_handle.set(offset, accel);
            }
        }

        /// 6.Update particles
        // Update particles velocity and position
        for (long p = 0; p < particle_size; ++p)
        {
            GA_Offset offset = gdp.pointOffset(p);

            // Update velocity and position using the semi-implicit Euler method:
            // v(t+dt) = v(t) + a(t)*dt
            // x(t+dt) = x(t) + v(t+1)*dt
            UT_Vector3 vel = vel_handle.get(offset);
            UT_Vector3 pos = pos_handle.get(offset);
            UT_Vector3 accel = accel_handle.get(offset);
            vel += accel * dt;
            pos += vel * dt;

            // Apply boundary conditions
            if (pos.x() < _volumeMin.x())
            {
                pos.x() = _volumeMin.x();
                vel.x() = 0.0;
            }
            else if (pos.x() > _volumeMax.x())
            {
                pos.x() = _volumeMax.x();
                vel.x() = 0.0;
            }
            if (pos.y() < _volumeMin.y())
            {
                pos.y() = _volumeMin.y();
                vel.y() = 0.0;
            }
            else if (pos.y() > _volumeMax.y())
            {
                pos.y() = _volumeMax.y();
                vel.y() = 0.0;
            }
            if (pos.z() < _volumeMin.z())
            {
                pos.z() = _volumeMin.z();
                vel.z() = 0.0;
            }
            else if (pos.z() > _volumeMax.z())
            {
                pos.z() = _volumeMax.z();
                vel.z() = 0.0;
            }

            UT_Vector3 final_vel(vel.x(), vel.y(), vel.z());
            vel_handle.set(offset, final_vel);
            UT_Vector3 final_pos(pos.x(), pos.y(), pos.z());
            gdp.setPos3(offset, final_pos);
        }


/*            // Update time
            timeLeft -= dt;

        }*/
    }
}

fpreal SPHSolver::getKernelValue(fpreal dist) const {
    fpreal q = dist/_halfH;
    if (q<1.0)
    {
        return _kernelValueCoeff * ( pow3(2.0-q)-4*pow3(1.0-q) );
    }
    else
    {
        return _kernelValueCoeff * pow3(2.0-q);
    }
}

UT_Vector3 SPHSolver::getKernelGradient(fpreal dist, const UT_Vector3 &xij) const {
    fpreal q = dist/_halfH;
    UT_Vector3 gradient = xij;
    if (q<= 0.0)
    {
        gradient = UT_Vector3(0,0,0);
    }
    else if (q<1.0)
    {
        gradient *= _kernelGradientCoeffA * (4.0 * pow2(1.0-q) - pow2(2.0-q)) / dist;
    }
    else
    {
        gradient *= (_kernelGradientCoeffB * pow2(2.0 - q)) / dist;
    }

    return gradient;
}

SPHSolver::Neighbor::Neighbor(long i, const UT_Vector3& x, double d) {
    id = i;
    xij = x;
    dist = d;
}
