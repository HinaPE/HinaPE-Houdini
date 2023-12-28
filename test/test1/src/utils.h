#ifndef SIM_PBF_UTILS_H
#define SIM_PBF_UTILS_H

#include <GU/GU_Detail.h>

#include <numeric>

using real = float;
using sz = int32;

template<typename SRC>
struct UT_Vector3TArrayHandle
{
	explicit UT_Vector3TArrayHandle(SRC *_internal, std::function<sz(sz)> &&ito = [](sz i) { return i; })
			: internal(_internal), index_to_offset(std::move(ito)) {}
	virtual ~UT_Vector3TArrayHandle() = default;

	virtual UT_Vector3T<real> geti(int idx) const { return geto(index_to_offset(idx)); }
	virtual void seti(int idx, UT_Vector3T<real> p) { seto(index_to_offset(idx), p); }

	virtual UT_Vector3T<real> geto(int off) const = 0;
	virtual void seto(int off, UT_Vector3T<real> p) = 0;

	std::function<sz(sz)> index_to_offset;

protected:
	SRC *internal;
};

template<typename SRC>
struct MultipleUT_Vector3TArrayHandle
{
	explicit MultipleUT_Vector3TArrayHandle(std::vector<std::pair<UT_Vector3TArrayHandle<SRC> *, sz>> *InArrays)
			: Arrays(InArrays) {}
	~MultipleUT_Vector3TArrayHandle() = default;

	virtual UT_Vector3T<real> geti(int idx) const
	{
		int sum = 0;
		int last_sum = 0;
		int iter = -1;
		while (sum <= idx)
		{
			iter++;
			last_sum = sum;
			sum += (*Arrays)[iter].second;
		}
		if (iter >= Arrays->size() || iter < 0)
			return UT_Vector3T<real>();
		int real_idx = idx - last_sum;
		return (*Arrays)[iter].first->geti(real_idx);
	}
	virtual void seti(int idx, UT_Vector3T<real> p)
	{
		int sum = 0;
		int last_sum = 0;
		int iter = -1;
		while (sum <= idx)
		{
			iter++;
			last_sum = sum;
			sum += (*Arrays)[iter].second;
		}
		if (iter >= Arrays->size() || iter < 0)
			return;
		int real_idx = idx - last_sum;
		(*Arrays)[iter].first->seti(real_idx, p);
	}
	virtual sz value_size()
	{
		return std::accumulate(Arrays->begin(), Arrays->end(), 0, [](sz acc, const std::pair<UT_Vector3TArrayHandle<SRC> *, sz> &p)
		{
			return acc + p.second;
		});
	}
	sz size()
	{
		return Arrays->size();
	}

protected:
	std::vector<std::pair<UT_Vector3TArrayHandle<SRC> *, sz>> *Arrays;
};

struct NativeArrayHandle : public UT_Vector3TArrayHandle<std::vector<real>>
{
	NativeArrayHandle(std::vector<real> *_internal) : UT_Vector3TArrayHandle(_internal) {}
	~NativeArrayHandle() override = default;

	UT_Vector3T<real> geto(int off) const override { return UT_Vector3T<real>(internal->data() + off * 3); }
	void seto(int off, UT_Vector3T<real> p) override { memcpy(internal->data() + off * 3, p.data(), sizeof(real) * 3); }
};

struct GA_ArrayHandle : public UT_Vector3TArrayHandle<GA_RWHandleT<UT_Vector3T<real>>>
{
	GA_ArrayHandle(GA_RWHandleT<UT_Vector3T<real>> *handle) : UT_Vector3TArrayHandle(handle) {}
	~GA_ArrayHandle() override = default;

	UT_Vector3T<real> geto(int off) const override { return internal->get(off); }
	void seto(int off, UT_Vector3T<real> p) override { internal->set(off, p); }
};

#endif //SIM_PBF_UTILS_H
