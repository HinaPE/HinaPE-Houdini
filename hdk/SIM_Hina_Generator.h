#ifndef HINAPE_SIM_HINA_GENERATOR_H
#define HINAPE_SIM_HINA_GENERATOR_H

#include <GAS/GAS_SubSolver.h>
#include <GAS/GAS_Utils.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_GuideShared.h>
#include <SIM/SIM_Collider.h>
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
#include <PRM/PRM_ChoiceList.h>

#include <GEO/GEO_PrimPoly.h>
#include <GEO/GEO_PrimSphere.h>
#include <GU/GU_PrimSphere.h>

#include <UT/UT_WorkBuffer.h>
#include <UT/UT_NetMessage.h>
#include <UT/UT_ThreadedAlgorithm.h>

#include <memory>

#define AS_UTVector3D(Vec3) UT_Vector3D(Vec3.x, Vec3.y, Vec3.z)
#define AS_CFVector3D(Vec3) CubbyFlow::Vector3D(Vec3.x(), Vec3.y(), Vec3.z())

#define CHECK_NULL_RETURN_BOOL(ptr) \
if (!ptr) \
{ \
error_msg.appendSprintf("%s::NULL POINTER Exception, From %s\n", ptr->getDataType().c_str(), DATANAME); \
return false; \
}

#define CHECK_NULL_RETURN_VOID(ptr) \
if (!ptr) \
{ \
error_msg.appendSprintf("%s::NULL POINTER Exception, From %s\n", ptr->getDataType().c_str(), DATANAME); \
}

#define SIM_HINA_GEOMETRY_CLASS(NAME, ...) \
class SIM_Hina_##NAME : public SIM_GeometryCopy \
{ \
public: \
    static const char *DATANAME; \
    bool Configured = false; \
    mutable GU_DetailHandle my_detail_handle; \
    mutable UT_WorkBuffer error_msg; \
    __VA_ARGS__ \
protected: \
    explicit SIM_Hina_##NAME(const SIM_DataFactory *factory) : BaseClass(factory) {} \
    ~SIM_Hina_##NAME() override = default; \
    void initializeSubclass() override; \
    void makeEqualSubclass(const SIM_Data *source) override; \
    GU_ConstDetailHandle getGeometrySubclass() const final; \
    static const SIM_DopDescription *getDopDescription(); \
    DECLARE_STANDARD_GETCASTTOTYPE(); \
    DECLARE_DATAFACTORY(SIM_Hina_##NAME, SIM_GeometryCopy, "Hina_"#NAME, getDopDescription()); \
private: \
    void _init_##NAME(); \
    void _makeEqual_##NAME(const SIM_Hina_##NAME *src); \
protected: \
	virtual void _setup_gdp(GU_Detail *gdp) const; \
};

#define SIM_HINA_GEOMETRY_IMPLEMENT(NAME, GEN_NODE, ...) \
void SIM_Hina_##NAME::initializeSubclass() \
{ \
    SIM_GeometryCopy::initializeSubclass(); \
    this->Configured = false; \
    this->error_msg.clear(); \
	my_detail_handle.clear(); \
    _init_##NAME(); \
} \
void SIM_Hina_##NAME::makeEqualSubclass(const SIM_Data *source) \
{ \
    SIM_GeometryCopy::makeEqualSubclass(source); \
    const SIM_Hina_##NAME *src = SIM_DATA_CASTCONST(source, SIM_Hina_##NAME); \
    this->Configured = src->Configured; \
    this->error_msg = src->error_msg; \
    this->my_detail_handle = src->my_detail_handle; \
    _makeEqual_##NAME(src); \
} \
GU_ConstDetailHandle SIM_Hina_##NAME::getGeometrySubclass() const \
{ \
	if (my_detail_handle.isNull()) \
	{ \
		GU_Detail *gdp = new GU_Detail(); \
		my_detail_handle.allocateAndSet(gdp); \
    	_setup_gdp(gdp); \
	} \
	return my_detail_handle; \
} \
const char *SIM_Hina_##NAME::DATANAME = "Hina_"#NAME; \
const SIM_DopDescription *SIM_Hina_##NAME::getDopDescription() \
{ \
static std::vector<PRM_Template> PRMS; \
PRMS.clear(); \
__VA_ARGS__ \
PRMS.emplace_back(); \
static SIM_DopDescription DESC(GEN_NODE, \
                               "Hina_"#NAME, \
                               "Hina "#NAME, \
                               DATANAME, \
                               classname(), \
                               PRMS.data()); \
DESC.setDefaultUniqueDataName(false); \
return &DESC; \
}

#define SIM_HINA_DERIVED_GEOMETRY_CLASS(DERIVED_NAME, PARENT_NAME, ...) \
class SIM_Hina_##DERIVED_NAME : public SIM_Hina_##PARENT_NAME \
{ \
public: \
    __VA_ARGS__ \
protected: \
    explicit SIM_Hina_##DERIVED_NAME(const SIM_DataFactory *factory) : SIM_Hina_##PARENT_NAME(factory) {} \
    ~SIM_Hina_##DERIVED_NAME() override = default; \
    void initializeSubclass() override; \
    void makeEqualSubclass(const SIM_Data *source) override; \
    static const SIM_DopDescription *getDopDescription(); \
DECLARE_STANDARD_GETCASTTOTYPE(); \
DECLARE_DATAFACTORY(SIM_Hina_##DERIVED_NAME, SIM_Hina_##PARENT_NAME, "Hina_"#DERIVED_NAME, SIM_Hina_##DERIVED_NAME::getDopDescription()); \
private: \
void _init_##DERIVED_NAME(); \
void _makeEqual_##DERIVED_NAME(const SIM_Hina_##DERIVED_NAME *src); \
protected: \
void _setup_gdp(GU_Detail *gdp) const override; \
};

#define SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(DERIVED_NAME, PARENT_NAME, GEN_NODE, ...) \
void SIM_Hina_##DERIVED_NAME::initializeSubclass() \
{ \
    SIM_Hina_##PARENT_NAME::initializeSubclass(); \
    _init_##DERIVED_NAME(); \
} \
void SIM_Hina_##DERIVED_NAME::makeEqualSubclass(const SIM_Data *source) \
{ \
    SIM_Hina_##PARENT_NAME::makeEqualSubclass(source); \
    const SIM_Hina_##DERIVED_NAME *src = SIM_DATA_CASTCONST(source, SIM_Hina_##DERIVED_NAME); \
    _makeEqual_##DERIVED_NAME(src); \
} \
const SIM_DopDescription *SIM_Hina_##DERIVED_NAME::getDopDescription() \
{ \
    static std::vector<PRM_Template> PRMS; \
    PRMS.clear(); \
    const PRM_Template *parent_templates = SIM_Hina_##PARENT_NAME::getDopDescription()->getTemplates(); \
    int iter = 0; \
    while (parent_templates[iter].getType() != PRM_LIST_TERMINATOR) \
    { \
        PRMS.push_back(parent_templates[iter]); \
        ++iter; \
    } \
    __VA_ARGS__ \
	PRMS.emplace_back(); \
    static SIM_DopDescription DESC(GEN_NODE, \
                               "Hina_"#DERIVED_NAME, \
                               "Hina "#DERIVED_NAME, \
                               DATANAME, \
                               classname(), \
                               PRMS.data()); \
    DESC.setDefaultUniqueDataName(false); \
    return &DESC; \
}

#define GAS_HINA_SUBSOLVER_CLASS(NAME, ...) \
class GAS_Hina_##NAME : public GAS_SubSolver \
{ \
public: \
static const char *DATANAME; \
mutable UT_WorkBuffer error_msg; \
__VA_ARGS__ \
protected: \
GAS_Hina_##NAME(const SIM_DataFactory *factory) : BaseClass(factory) {} \
~GAS_Hina_##NAME() override = default; \
bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override; \
void initializeSubclass() override; \
void makeEqualSubclass(const SIM_Data *source) override; \
static const SIM_DopDescription *getDopDescription(); \
DECLARE_STANDARD_GETCASTTOTYPE(); \
DECLARE_DATAFACTORY(GAS_Hina_##NAME, GAS_SubSolver, "Hina_"#NAME, getDopDescription());                    \
private: \
void _init(); \
void _makeEqual(const GAS_Hina_##NAME *src); \
bool _solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep); \
};

#define GAS_HINA_SUBSOLVER_IMPLEMENT(NAME, GEN_NODE, UNIQUE, ...) \
bool GAS_Hina_##NAME::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) \
{ \
    CHECK_NULL_RETURN_BOOL(obj) \
    if (!_solve(engine, obj, time, timestep) || UTisstring(this->error_msg.buffer())) \
    { \
        SIM_Data::addError(obj, SIM_MESSAGE, this->error_msg.buffer(), UT_ERROR_ABORT); \
        return false; \
    } \
    return true; \
} \
void GAS_Hina_##NAME::initializeSubclass() \
{ \
    SIM_Data::initializeSubclass(); \
    this->error_msg.clear(); \
    _init(); \
} \
void GAS_Hina_##NAME::makeEqualSubclass(const SIM_Data *source) \
{ \
    SIM_Data::makeEqualSubclass(source); \
    const GAS_Hina_##NAME *src = SIM_DATA_CASTCONST(source, GAS_Hina_##NAME); \
    this->error_msg = src->error_msg; \
    _makeEqual(src); \
} \
const char *GAS_Hina_##NAME::DATANAME = "Hina_"#NAME; \
const SIM_DopDescription *GAS_Hina_##NAME::getDopDescription() \
{ \
static std::vector<PRM_Template> PRMS; \
PRMS.clear(); \
__VA_ARGS__ \
PRMS.emplace_back(); \
static SIM_DopDescription DESC(GEN_NODE, \
                               "Hina_"#NAME, \
                               "Hina "#NAME, \
                               DATANAME, \
                               classname(), \
                               PRMS.data()); \
DESC.setDefaultUniqueDataName(UNIQUE); \
setGasDescription(DESC); \
return &DESC; \
}


// ==================================================================================================
// ==================================================================================================
// ==================================================================================================

#define HINA_GETSET_PARAMETER(NAME, GETSET_TYPE) \
GETSET_TYPE(#NAME, NAME)

#define HINA_INT_PARAMETER(NAME, DEFAULT_VALUE) \
static PRM_Name NAME(#NAME, #NAME); \
static PRM_Default Default##NAME(DEFAULT_VALUE);  \
PRMS.emplace_back(PRM_INT, 1, &NAME, &Default##NAME);

#define HINA_FLOAT_PARAMETER(NAME, DEFAULT_VALUE) \
static PRM_Name NAME(#NAME, #NAME); \
static PRM_Default Default##NAME(DEFAULT_VALUE);  \
PRMS.emplace_back(PRM_FLT, 1, &NAME, &Default##NAME);

#define HINA_STRING_PARAMETER(NAME, DEFAULT_VALUE) \
static PRM_Name NAME(#NAME, #NAME); \
static PRM_Default Default##NAME(0, DEFAULT_VALUE);  \
PRMS.emplace_back(PRM_STRING, 1, &NAME, &Default##NAME);

#define HINA_BOOL_PARAMETER(NAME, DEFAULT_VALUE) \
static PRM_Name NAME(#NAME, #NAME); \
static PRM_Default Default##NAME(DEFAULT_VALUE);  \
PRMS.emplace_back(PRM_TOGGLE, 1, &NAME, &Default##NAME);

#define HINA_INT_VECTOR_PARAMETER(NAME, SIZE, ...) \
static PRM_Name NAME(#NAME, #NAME); \
static std::array<PRM_Default, SIZE> Default##NAME{__VA_ARGS__};  \
PRMS.emplace_back(PRM_INT, SIZE, &NAME, Default##NAME.data());

#define HINA_FLOAT_VECTOR_PARAMETER(NAME, SIZE, ...) \
static PRM_Name NAME(#NAME, #NAME); \
static std::array<PRM_Default, SIZE> Default##NAME{__VA_ARGS__};  \
PRMS.emplace_back(PRM_FLT, SIZE, &NAME, Default##NAME.data());

#define HINA_GEOMETRY_ATTRIBUTE_COLOR "Cd"
#define HINA_GEOMETRY_ATTRIBUTE_VELOCITY "v"
#define HINA_GEOMETRY_ATTRIBUTE_FORCE "f"
#define HINA_GEOMETRY_ATTRIBUTE_ACCELERATION "a"
#define HINA_GEOMETRY_ATTRIBUTE_FORCE_NORM "f_n"
#define HINA_GEOMETRY_ATTRIBUTE_MASS "m"
#define HINA_GEOMETRY_ATTRIBUTE_DENSITY "den"
#define HINA_GEOMETRY_ATTRIBUTE_PRESSURE "pres"
#define HINA_GEOMETRY_ATTRIBUTE_VOLUME "vol"
#define HINA_GEOMETRY_ATTRIBUTE_NEIGHBORS "nbs"
#define HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_SELF "n_sum_s"
#define HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_OTHERS "n_sum_o"
#define HINA_GEOMETRY_ATTRIBUTE_DFSPH_FACTOR "df_factor"
#define HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA_DENSITY "df_kappa_density"
#define HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA_DIVERGENCE "df_kappa_divergence"
#define HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV "df_density_adv"
#define HINA_GEOMETRY_ATTRIBUTE_DFSPH_D_DENSITY "df_d_density"

#define HINA_GEOMETRY_ATTRIBUTE_TYPE_STRING "HINA_STRING"
#define HINA_GEOMETRY_ATTRIBUTE_TYPE_INT "HINA_INT"
#define HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT "HINA_FLOAT"
#define HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3 "HINA_VECTOR3"

#define HINA_GEOMETRY_POINT_ATTRIBUTE(ATTRIBUTE_NAME, ATTRIBUTE_TYPE) \
{ \
if (std::string(ATTRIBUTE_TYPE) == HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3) \
{ \
GA_RWAttributeRef ref = gdp->addFloatTuple(GA_ATTRIB_POINT, ATTRIBUTE_NAME, 3, GA_Defaults(0)); \
ref.setTypeInfo(GA_TYPE_VECTOR); \
} \
if (std::string(ATTRIBUTE_TYPE) == HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT) \
{ \
GA_RWAttributeRef ref = gdp->addFloatTuple(GA_ATTRIB_POINT, ATTRIBUTE_NAME, 1, GA_Defaults(0)); \
ref.setTypeInfo(GA_TYPE_VOID); \
} \
if (std::string(ATTRIBUTE_TYPE) == HINA_GEOMETRY_ATTRIBUTE_TYPE_INT) \
{ \
GA_RWAttributeRef ref = gdp->addIntTuple(GA_ATTRIB_POINT, ATTRIBUTE_NAME, 1, GA_Defaults(0)); \
ref.setTypeInfo(GA_TYPE_VOID); \
} \
if (std::string(ATTRIBUTE_TYPE) == HINA_GEOMETRY_ATTRIBUTE_TYPE_STRING) \
{ \
GA_RWAttributeRef ref = gdp->addStringTuple(GA_ATTRIB_POINT, ATTRIBUTE_NAME, 1); \
ref.setTypeInfo(GA_TYPE_VOID); \
} \
}

// ==================================================================================================
// ==================================================================================================
// ==================================================================================================

#define TARGET_PARTICLE_GEOMETRY(PARTICLE_CLASS) \
static PRM_Name    theGeometryName(GAS_NAME_GEOMETRY, PARTICLE_CLASS::DATANAME); \
static PRM_Default    theGeometryNameDefault(0, PARTICLE_CLASS::DATANAME); \
PRMS.emplace_back(PRM_STRING, 1, &theGeometryName, &theGeometryNameDefault);

#define ACTIVATE_GAS_VELOCITY \
static PRM_Name    theVelocityName(GAS_NAME_VELOCITY, "Velocity"); \
static PRM_Default    theVelocityNameDefault(0, "Velocity"); \
PRMS.emplace_back(PRM_STRING, 1, &theVelocityName, &theVelocityNameDefault);

#define ACTIVATE_GAS_DENSITY \
static PRM_Name    theDensityName(GAS_NAME_DENSITY, "Density"); \
static PRM_Default    theDensityNameDefault(0, "Density"); \
PRMS.emplace_back(PRM_STRING, 1, &theDensityName, &theDensityNameDefault);

#define ACTIVATE_GAS_TEMPERATURE \
static PRM_Name    theTemperatureName(GAS_NAME_TEMPERATURE, "Temperature"); \
static PRM_Default    theTemperatureNameDefault(0, "Temperature"); \
PRMS.emplace_back(PRM_STRING, 1, &theTemperatureName, &theTemperatureNameDefault);

#define ACTIVATE_GAS_PRESSURE \
static PRM_Name    thePressureName(GAS_NAME_PRESSURE, "Pressure"); \
static PRM_Default    thePressureNameDefault(0, "Pressure"); \
PRMS.emplace_back(PRM_STRING, 1, &thePressureName, &thePressureNameDefault);


// ==================================================================================================
// ==================================================================================================
// ==================================================================================================

#define SIM_HINA_COLLIDER_CLASS(NAME, ...) \
class SIM_Hina_##NAME : public SIM_Collider \
{ \
public: \
static const char *DATANAME; \
bool Configured = false; \
mutable UT_WorkBuffer error_msg; \
__VA_ARGS__ \
protected: \
SIM_Hina_##NAME(const SIM_DataFactory *factory) : SIM_Collider(factory) {} \
~SIM_Hina_##NAME() override = default; \
void initializeSubclass() override; \
void makeEqualSubclass(const SIM_Data *source) override; \
static const SIM_DopDescription *getDopDescription(); \
DECLARE_STANDARD_GETCASTTOTYPE(); \
DECLARE_DATAFACTORY(SIM_Hina_##NAME, SIM_Collider, "Hina_"#NAME, getDopDescription());                    \
private: \
void _init(); \
void _makeEqual(const SIM_Hina_##NAME *src); \
};

#define SIM_HINA_COLLIDER_IMPLEMENT(NAME, ...) \
void SIM_Hina_##NAME::initializeSubclass() \
{ \
    SIM_Collider::initializeSubclass(); \
    this->Configured = false; \
    this->error_msg.clear(); \
    _init(); \
} \
void SIM_Hina_##NAME::makeEqualSubclass(const SIM_Data *source) \
{ \
    SIM_Collider::makeEqualSubclass(source); \
    const SIM_Hina_##NAME *src = SIM_DATA_CASTCONST(source, SIM_Hina_##NAME); \
    this->Configured = src->Configured; \
    this->error_msg = src->error_msg; \
    _makeEqual(src); \
} \
const char *SIM_Hina_##NAME::DATANAME = "Hina_"#NAME; \
const SIM_DopDescription *SIM_Hina_##NAME::getDopDescription() \
{ \
static std::vector<PRM_Template> PRMS;             \
PRMS.clear(); \
__VA_ARGS__ \
PRMS.emplace_back(); \
static SIM_DopDescription DESC(true, \
                               "Hina_"#NAME, \
                               "Hina "#NAME, \
                               DATANAME, \
                               classname(), \
                               PRMS.data()); \
DESC.setDefaultUniqueDataName(false); \
return &DESC; \
}

#define SIM_HINA_DATA_CLASS(NAME, ...) \
class SIM_Hina_##NAME : public SIM_Data \
{ \
public: \
    static const char *DATANAME; \
    bool Configured = false; \
    mutable GU_DetailHandle my_detail_handle; \
    mutable UT_WorkBuffer error_msg; \
    __VA_ARGS__ \
protected: \
    explicit SIM_Hina_##NAME(const SIM_DataFactory *factory) : BaseClass(factory) {} \
    ~SIM_Hina_##NAME() override = default; \
    void initializeSubclass() override; \
    void makeEqualSubclass(const SIM_Data *source) override; \
    static const SIM_DopDescription *getDopDescription(); \
    DECLARE_STANDARD_GETCASTTOTYPE(); \
    DECLARE_DATAFACTORY(SIM_Hina_##NAME, SIM_Data, "Hina_"#NAME, getDopDescription()); \
private: \
    void _init_##NAME(); \
    void _makeEqual_##NAME(const SIM_Hina_##NAME *src); \
};

#define SIM_HINA_DATA_IMPLEMENT(NAME, GEN_NODE, ...) \
void SIM_Hina_##NAME::initializeSubclass() \
{ \
    SIM_Data::initializeSubclass(); \
    this->Configured = false; \
    this->error_msg.clear(); \
    _init_##NAME(); \
} \
void SIM_Hina_##NAME::makeEqualSubclass(const SIM_Data *source) \
{ \
    SIM_Data::makeEqualSubclass(source); \
    const SIM_Hina_##NAME *src = SIM_DATA_CASTCONST(source, SIM_Hina_##NAME); \
    this->Configured = src->Configured; \
    this->error_msg = src->error_msg; \
    _makeEqual_##NAME(src); \
} \
const char *SIM_Hina_##NAME::DATANAME = "Hina_"#NAME; \
const SIM_DopDescription *SIM_Hina_##NAME::getDopDescription() \
{ \
static std::vector<PRM_Template> PRMS; \
PRMS.clear(); \
__VA_ARGS__ \
PRMS.emplace_back(); \
static SIM_DopDescription DESC(GEN_NODE, \
                               "Hina_"#NAME, \
                               "Hina "#NAME, \
                               DATANAME, \
                               classname(), \
                               PRMS.data()); \
DESC.setDefaultUniqueDataName(false); \
return &DESC; \
}

#endif //HINAPE_SIM_HINA_GENERATOR_H
