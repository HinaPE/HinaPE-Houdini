set(HINA_TARGET SIM_Celeste_Fluid)

file(GLOB_RECURSE HINAPE_EADER *.h)
file(GLOB_RECURSE HINAPE_SOURCE *.cpp)

add_library(
        ${HINA_TARGET}
        SHARED
        ${HINAPE_HEADER}
        ${HINAPE_SOURCE}

        Particles/Particles.h
        Particles/PbfParticle.h
        Particles/SIM_FluidParticle.cpp
        Particles/SIM_FluidParticle.h
        Particles/GAS_ConfigureFluidParticleData.cpp
        Particles/GAS_ConfigureFluidParticleData.h
        Emitter/GAS_VolumeParticleEmitter.cpp
        Emitter/GAS_VolumeParticleEmitter.h
        Solvers/GAS_ParticleIntegrator.cpp
        Solvers/GAS_ParticleIntegrator.h
        Neighbor/GAS_BuildNeighborLists.cpp
        Neighbor/GAS_BuildNeighborLists.h
        Density/GAS_ParticleDensity.cpp
        Density/GAS_ParticleDensity.h
        Viscosity/GAS_ParticleViscosity.cpp
        Viscosity/GAS_ParticleViscosity.h
        Boundary/SIM_SemiAnalyticalCollider.cpp
        Boundary/SIM_SemiAnalyticalCollider.h
        Boundary/SemiAnalyticalBoundary.h
        Boundary/AlignedBox.h
        Boundary/GAS_ConfigureSemiAnalyticalCollider.cpp
        Boundary/GAS_ConfigureSemiAnalyticalCollider.h
        Boundary/GAS_SetCollider.cpp
        Boundary/GAS_SetCollider.h
)

# Link Houdini Toolkit
target_link_libraries(
        ${HINA_TARGET}
        PUBLIC
        Houdini
        CUDA_CubbyFlow
        CUDA_HinaPE
        TreeNSearch
)
if (MSVC)
    target_link_libraries(
            ${HINA_TARGET}
            PUBLIC
            CUDA_NeighborSearch
    )
endif ()
target_include_directories(
        ${HINA_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(
        ${HINA_TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
houdini_configure_target(${HINA_TARGET})
