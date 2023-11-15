#ifndef HINAPE_HOUDINI_DATA_H
#define HINAPE_HOUDINI_DATA_H

#include <SIM/SIM_Data.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DataUtils.h>

namespace HinaPE
{
using real = double;
using size = size_t;

//@formatter:off
template <typename BaseType, size Dimension>
class ArrayHandle
{
public:
	ArrayHandle(BaseType *array_pointer, size array_size) : array(array_pointer), array_size(array_size) {}
	auto Size() const -> size { return array_size / Dimension; }

	struct Iter
	{
		Iter(BaseType *start_pointer) : start_ptr(start_pointer) {}
		auto operator[](size i) -> BaseType & { if (i >= Dimension) throw std::out_of_range("ArrayHandle::Iter::operator[]: Index out of range."); return start_ptr[i]; }
		BaseType *start_ptr;
	};
	auto operator[](size i) -> Iter { return Iter(&array[i * Dimension]); }

private:
	BaseType * const array{nullptr};
	size array_size{0};
};

template <typename BaseType, size Dimension>
class ConstArrayHandle
{
public:
	ConstArrayHandle(const BaseType *array_pointer, size array_size) : array(array_pointer), array_size(array_size) {}
	auto Size() const -> size { return array_size / Dimension; }

	struct Iter
	{
		Iter(const BaseType *start_pointer) : start_ptr(start_pointer) {}
		auto operator[](size i) const -> const BaseType & { if (i >= Dimension) throw std::out_of_range("ConstArrayHandle::Iter::operator[]: Index out of range."); return start_ptr[i]; }
		const BaseType *start_ptr;
	};
	auto operator[](size i) const -> Iter { return Iter(&array[i * Dimension]); }

private:
	const BaseType * const array{nullptr};
	size array_size{0};
};

struct MeshData
{
	using Point = ArrayHandle<real, 3>::Iter;
	using Triangle = ArrayHandle<size, 3>::Iter;
	using ConstPoint = ConstArrayHandle<real, 3>::Iter;
	using ConstTriangle = ConstArrayHandle<size, 3>::Iter;

	Point P(size i) { return ArrayHandle<real, 3>(V, V_N)[i];};
	Triangle T(size i) { return ArrayHandle<size, 3>(I, I_N)[i]; };
	ConstPoint P(size i) const { return ConstArrayHandle<real, 3>(V, V_N)[i]; };
	ConstTriangle T(size i) const { return ConstArrayHandle<size, 3>(I, I_N)[i]; };

	explicit MeshData(real *vertices_array, size vertices_num, size *indices_array, size indices_num)
			: V(vertices_array), V_N(vertices_num), I(indices_array), I_N(indices_num) {}
private:
	real *V; size V_N;
	size *I; size I_N;
};

struct ClothData
{
	ArrayHandle<real, 3> P;
	ArrayHandle<real, 3> V;
	ArrayHandle<real, 3> F;
};

struct ClothFactory
{
	static void Init(size VertexPoolSize, size IndexPoolSize)
	{
		VERTEX_POOL = new real[VertexPoolSize];
		VERTEX_POOL_SIZE = VertexPoolSize;
		VERTEX_POOL_USED = 0;
		INDEX_POOL = new size[IndexPoolSize];
		INDEX_POOL_SIZE = IndexPoolSize;
		INDEX_POOL_USED = 0;
	}
	static void Destroy()
	{
		delete[] VERTEX_POOL;
		VERTEX_POOL_SIZE = 0;
		VERTEX_POOL_USED = 0;
		delete[] INDEX_POOL;
		INDEX_POOL_SIZE = 0;
		INDEX_POOL_USED = 0;
	}
	static bool VALID()
	{
		return
			VERTEX_POOL != nullptr && INDEX_POOL != nullptr &&
			VERTEX_POOL_SIZE != 0 && INDEX_POOL_SIZE != 0 &&
			VERTEX_POOL_USED <= VERTEX_POOL_SIZE && INDEX_POOL_USED <= INDEX_POOL_SIZE;
	}

private:
	static real* VERTEX_POOL;
	static size VERTEX_POOL_SIZE;
	static size VERTEX_POOL_USED;
	static size* INDEX_POOL;
	static size INDEX_POOL_SIZE;
	static size INDEX_POOL_USED;
};
//@formatter:on

class ClothSolver
{
public:
	struct Args
	{
		real dt;
	};
	void Init(const Args &args);
	void Solve(const Args &args);

private:

	// Constraints
	size *Constraints;
	size Constraints_Num;
};
}

class HinaClothData final : public SIM_Data, public SIM_OptionsUser
{
public:
	explicit HinaClothData(const SIM_DataFactory *factory);
	~HinaClothData() override;

protected:
	static auto GetDescription() -> const SIM_DopDescription *;
DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(HinaClothData, SIM_Data, "Hina Cloth Data", GetDescription());
};

#endif //HINAPE_HOUDINI_DATA_H
