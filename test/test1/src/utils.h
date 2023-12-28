#ifndef SIM_PBF_UTILS_H
#define SIM_PBF_UTILS_H

#include "data.h"
#include <GU/GU_Detail.h>

namespace HinaPE_PBF
{
template<typename T, typename Prim>
struct ArrayHandle
{
	ArrayHandle(T *_internal) : internal(_internal) {}
	~ArrayHandle() = default;

	virtual Prim get(int off) const = 0;
	virtual void set(int off, Prim p) = 0;

	T *internal;
};

template<typename T, typename Prim>
struct ConstArrayHandle
{
	ConstArrayHandle(ArrayHandle<T, Prim> *_internal) : internal(_internal) {}
	~ConstArrayHandle() = default;

	Prim get(int off) const { return internal->get(off); }

	ArrayHandle<T, Prim> *internal;
};

struct NativeVector3Handle : public ArrayHandle<std::vector<real>, UT_Vector3T<real>>
{
	NativeVector3Handle(std::vector<real> *_internal) : ArrayHandle(_internal) {}
	~NativeVector3Handle() = default;

	UT_Vector3T<real> get(int off) const override { return UT_Vector3T<real>(internal->data() + off * 3); }
	void set(int off, UT_Vector3T<real> p) override { memcpy(internal->data() + off * 3, p.data(), sizeof(real) * 3); }
};

struct GDPVector3Handle : public ArrayHandle<GU_Detail, UT_Vector3T<real>>
{
	GDPVector3Handle(GU_Detail *gdp) : ArrayHandle(gdp) {}
	~GDPVector3Handle() = default;

	UT_Vector3T<real> get(int off) const override { return internal->getPos3(off); }
	void set(int off, UT_Vector3T<real> p) override { internal->setPos3(off, p); }
};
}

void SyncParticlePositionFromGDP(HinaPE_PBF::PBF_DATA &data, const GU_Detail &gdp);
void SyncGDPFromParticlePosition(GU_Detail &gdp, const HinaPE_PBF::PBF_DATA &data);

#endif //SIM_PBF_UTILS_H
