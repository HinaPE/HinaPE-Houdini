#ifndef HINAPE_INTERFACE_H
#define HINAPE_INTERFACE_H

#include <UT/UT_Vector3.h>

#include <utility>
#include <memory>

using real = fpreal64;
using integer = int32;
using size = size_t;

namespace HinaPE
{
#if defined(GCC4) || defined(__clang__)
#define HINA_FORCE_INLINE    __attribute__ ((always_inline)) inline
#elif defined(_MSC_VER)
#define HINA_FORCE_INLINE	__forceinline
#else
#define HINA_FORCE_INLINE	inline
#endif

template<typename T>
struct HVectorHDK
{
	static constexpr int tuple_size = 3;

	HINA_FORCE_INLINE HVectorHDK() : Inner() {}
	constexpr HINA_FORCE_INLINE HVectorHDK(const HVectorHDK<T> &that) : Inner(that.Inner) {}
	constexpr HINA_FORCE_INLINE HVectorHDK(HVectorHDK<T> &&that) : Inner(std::move(that.Inner)) {}
	HINA_FORCE_INLINE HVectorHDK(const UT_Vector3T<T> &that) : Inner(that) {}
	HINA_FORCE_INLINE HVectorHDK(UT_Vector3T<T> &&that) : Inner(std::move(that)) {}

	constexpr HINA_FORCE_INLINE HVectorHDK(const T vx, const T vy, const T vz) noexcept:
			Inner{vx, vy, vz} {}
	constexpr explicit HINA_FORCE_INLINE HVectorHDK(const T v) noexcept:
			HVectorHDK(v, v, v) {}
	constexpr HINA_FORCE_INLINE HVectorHDK(const real v[tuple_size]) noexcept:
			HVectorHDK(v[0], v[1], v[2]) {}
	constexpr HINA_FORCE_INLINE HVectorHDK(const integer v[tuple_size]) noexcept:
			HVectorHDK(v[0], v[1], v[2]) {}

	template<typename S>
	constexpr HINA_FORCE_INLINE HVectorHDK(const HVectorHDK<S> &v) noexcept :
			HVectorHDK(v[0], v[1], v[2]) {}

	constexpr HINA_FORCE_INLINE HVectorHDK<T> &operator=(const HVectorHDK<T> &that)
	{
		Inner.operator=(that.Inner);
		return *this;
	}
	constexpr HINA_FORCE_INLINE HVectorHDK<T> &operator=(HVectorHDK<T> &&that)
	{
		return Inner.operator=(std::move(that.Inner));
		return *this;
	}

	template<typename S>
	HINA_FORCE_INLINE HVectorHDK<T> &operator=(const HVectorHDK<S> &v)
	{
		Inner[0] = v[0];
		Inner[1] = v[1];
		Inner[2] = v[2];
		return *this;
	}

	constexpr HINA_FORCE_INLINE const T &operator[](exint i) const noexcept
	{
		UT_ASSERT_P((0 <= i) && (i < tuple_size));
		return Inner.operator()[i];
	}

	constexpr HINA_FORCE_INLINE T &operator[](exint i) noexcept
	{
		UT_ASSERT_P((0 <= i) && (i < tuple_size));
		return Inner.operator()[i];
	}

	constexpr HINA_FORCE_INLINE T &operator()(unsigned i) noexcept
	{
		UT_ASSERT_P(i < tuple_size);
		return Inner.operator()(i);
	}

	constexpr HINA_FORCE_INLINE T operator()(unsigned i) const noexcept
	{
		UT_ASSERT_P(i < tuple_size);
		return Inner.operator()(i);
	}

	constexpr HINA_FORCE_INLINE const T *data() const noexcept { return Inner.data(); }
	constexpr HINA_FORCE_INLINE T *data() noexcept { return Inner.data(); }

	constexpr HINA_FORCE_INLINE HVectorHDK &operator+=(const HVectorHDK &a) noexcept
	{
		Inner.operator+=(a.Inner);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorHDK &operator-=(const HVectorHDK &a) noexcept
	{
		Inner.operator-=(a.Inner);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorHDK &operator+=(const T &a) noexcept
	{
		Inner.operator+=(a);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorHDK &operator-=(const T &a) noexcept
	{
		Inner.operator-=(a);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorHDK &operator*=(const T &a) noexcept
	{
		Inner.operator*=(a);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorHDK &operator/=(const T &a) noexcept
	{
		Inner.operator/=(a);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorHDK &operator*=(const HVectorHDK &a) noexcept
	{
		Inner.operator*=(a.Inner);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorHDK &operator/=(const HVectorHDK &a) noexcept
	{
		Inner.operator/=(a.Inner);
		return *this;
	}

	constexpr HINA_FORCE_INLINE T length() const noexcept { return Inner.length(); }
	constexpr HINA_FORCE_INLINE T length2() const noexcept { return Inner.length2(); }
	constexpr HINA_FORCE_INLINE T distance(const HVectorHDK &b) const noexcept { return Inner.distance(b); }
	constexpr HINA_FORCE_INLINE T distance2(const HVectorHDK &b) const noexcept { return Inner.distance2(b); }
	T distance(const HVectorHDK<T> &p1, const HVectorHDK<T> &v1) const { return distance(p1.Inner, v1.Inner); }
	T distance(const HVectorHDK<T> &p1, const HVectorHDK<T> &v1, const HVectorHDK<T> &p2, const HVectorHDK<T> &v2) const { return distance(p1.Inner, v1.Inner, p2.Inner, v2.Inner); }

	HINA_FORCE_INLINE void normalize() noexcept { Inner.normalize(); }
	HINA_FORCE_INLINE HVectorHDK normalized() noexcept
	{
		HVectorHDK tmp(Inner.x(), Inner.y(), Inner.z());
		tmp.normalize();
		return tmp;
	}

	constexpr HINA_FORCE_INLINE T dot(const HVectorHDK &b) const noexcept { return Inner.dot(b.Inner); }
	constexpr HINA_FORCE_INLINE void cross(const HVectorHDK<T> &v) noexcept { Inner.cross(v.Inner); }
	HINA_FORCE_INLINE void normal(const HVectorHDK<T> &va, const HVectorHDK<T> &vb) { Inner.normal(va.Inner, vb.Inner); }
	HVectorHDK<T> getBary(const HVectorHDK<T> &t0, const HVectorHDK<T> &t1, const HVectorHDK<T> &t2, bool *degen = NULL) const { Inner.getBary(t0.Inner, t1.Inner, t2.Inner, degen); }

	constexpr HINA_FORCE_INLINE T &x() noexcept { return Inner.x(); }
	constexpr HINA_FORCE_INLINE T x() const noexcept { return Inner.x(); }
	constexpr HINA_FORCE_INLINE T &y() noexcept { return Inner.y(); }
	constexpr HINA_FORCE_INLINE T y() const noexcept { return Inner.y(); }
	constexpr HINA_FORCE_INLINE T &z() noexcept { return Inner.z(); }
	constexpr HINA_FORCE_INLINE T z() const noexcept { return Inner.z(); }
	constexpr HINA_FORCE_INLINE T &r() noexcept { return Inner.x(); }
	constexpr HINA_FORCE_INLINE T r() const noexcept { return Inner.x(); }
	constexpr HINA_FORCE_INLINE T &g() noexcept { return Inner.y(); }
	constexpr HINA_FORCE_INLINE T g() const noexcept { return Inner.y(); }
	constexpr HINA_FORCE_INLINE T &b() noexcept { return Inner.z(); }
	constexpr HINA_FORCE_INLINE T b() const noexcept { return Inner.z(); }

	unsigned hash() const { return Inner.hash(); }

	// TODO: Matrix Operations

private:
	friend constexpr bool operator==(const HVectorHDK &a, const HVectorHDK &b) noexcept { return a.Inner == b.Inner; }
	friend constexpr bool operator!=(const HVectorHDK &a, const HVectorHDK &b) noexcept { return a.Inner != b.Inner; }
	friend constexpr bool operator<(const HVectorHDK &a, const HVectorHDK &b) noexcept { return a.Inner < b.Inner; }
	friend constexpr bool operator<=(const HVectorHDK &a, const HVectorHDK &b) noexcept { return a.Inner <= b.Inner; }
	friend constexpr bool operator>(const HVectorHDK &a, const HVectorHDK &b) noexcept { return a.Inner > b.Inner; }
	friend constexpr bool operator>=(const HVectorHDK &a, const HVectorHDK &b) noexcept { return a.Inner >= b.Inner; }
	friend std::ostream &operator<<(std::ostream &os, const HVectorHDK<T> &v) { return operator<<(os, v.Inner); }

	UT_Vector3T<T> Inner;
};

// Free floating functions:
template<typename T>
constexpr
HVectorHDK<T> operator+(const HVectorHDK<T> &a, const HVectorHDK<T> &b) noexcept
{
	return HVectorHDK<T>(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
}

template<typename T>
constexpr
HVectorHDK<T> operator-(const HVectorHDK<T> &a, const HVectorHDK<T> &b) noexcept
{
	return HVectorHDK<T>(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
}

template<typename T>
constexpr
HVectorHDK<T> operator*(const HVectorHDK<T> &v1, const HVectorHDK<T> &v2) noexcept
{
	return HVectorHDK<T>(v1.x() * v2.x(), v1.y() * v2.y(), v1.z() * v2.z());
}

template<typename T>
constexpr
HVectorHDK<T> operator/(const HVectorHDK<T> &v1, const HVectorHDK<T> &v2) noexcept
{
	return HVectorHDK<T>(v1.x() / v2.x(), v1.y() / v2.y(), v1.z() / v2.z());
}

template<typename T, typename S>
constexpr
HVectorHDK<T> operator+(const HVectorHDK<T> &v, S scalar) noexcept
{
	return HVectorHDK<T>(v.x() + scalar, v.y() + scalar, v.z() + scalar);
}

template<typename T, typename S>
constexpr
HVectorHDK<T> operator+(S scalar, const HVectorHDK<T> &v) noexcept
{
	return HVectorHDK<T>(scalar + v.x(), scalar + v.y(), scalar + v.z());
}

template<typename T, typename S>
constexpr
HVectorHDK<T> operator-(const HVectorHDK<T> &v, S scalar) noexcept
{
	return HVectorHDK<T>(v.x() - scalar, v.y() - scalar, v.z() - scalar);
}

template<typename T, typename S>
constexpr
HVectorHDK<T> operator-(S scalar, const HVectorHDK<T> &v) noexcept
{
	return HVectorHDK<T>(scalar - v.x(), scalar - v.y(), scalar - v.z());
}

template<typename T, typename S>
constexpr
HVectorHDK<T> operator*(const HVectorHDK<T> &v, S scalar) noexcept
{
	return HVectorHDK<T>(v.x() * scalar, v.y() * scalar, v.z() * scalar);
}

template<typename T, typename S>
constexpr
HVectorHDK<T> operator*(S scalar, const HVectorHDK<T> &v) noexcept
{
	return HVectorHDK<T>(scalar * v.x(), scalar * v.y(), scalar * v.z());
}

template<typename T, typename S>
constexpr
HVectorHDK<T> operator/(const HVectorHDK<T> &v, S scalar) noexcept
{
	//TODO: in C++17, this can be "if constexpr"
	if (SYS_IsFloatingPoint_v<T>)
	{
		// This has to be T because S may be int for "v = v/2" code
		// For the same reason we must cast the 1
		T inv = T(1) / scalar;
		return HVectorHDK<T>(v.vec[0] * inv, v.vec[1] * inv, v.vec[2] * inv);
	}
	return HVectorHDK<T>(v.vec[0] / scalar, v.vec[1] / scalar, v.vec[2] / scalar);
}

template<typename T, typename S>
constexpr
HVectorHDK<T> operator/(S scalar, const HVectorHDK<T> &v) noexcept
{
	return HVectorHDK<T>(scalar / v.vec[0], scalar / v.vec[1], scalar / v.vec[2]);
}
}

#ifdef USE_EIGEN
#include <Eigen/Eigen>
namespace HinaPE
{
template<typename T>
struct HVectorEigen
{

	static constexpr int tuple_size = 3;

	HINA_FORCE_INLINE HVectorEigen() : Inner() {}
	constexpr HINA_FORCE_INLINE HVectorEigen(const HVectorEigen<T> &that) : Inner(that.Inner) {}
	constexpr HINA_FORCE_INLINE HVectorEigen(HVectorEigen<T> &&that) : Inner(std::move(that.Inner)) {}
	HINA_FORCE_INLINE HVectorEigen(const Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &that) : Inner(that) {}
	HINA_FORCE_INLINE HVectorEigen(Eigen::Matrix<T, 3, 1, Eigen::DontAlign> &&that) : Inner(std::move(that)) {}

	constexpr HINA_FORCE_INLINE HVectorEigen(const T vx, const T vy, const T vz) noexcept:
			Inner{vx, vy, vz} {}
	constexpr explicit HINA_FORCE_INLINE HVectorEigen(const T v) noexcept:
			HVectorEigen(v, v, v) {}
	constexpr HINA_FORCE_INLINE HVectorEigen(const real v[tuple_size]) noexcept:
			HVectorEigen(v[0], v[1], v[2]) {}
	constexpr HINA_FORCE_INLINE HVectorEigen(const integer v[tuple_size]) noexcept:
			HVectorEigen(v[0], v[1], v[2]) {}

	template<typename S>
	constexpr HINA_FORCE_INLINE HVectorEigen(const HVectorEigen<S> &v) noexcept :
			HVectorEigen(v[0], v[1], v[2]) {}

	constexpr HINA_FORCE_INLINE HVectorEigen<T> &operator=(const HVectorEigen<T> &that)
	{
		Inner.operator=(that.Inner);
		return *this;
	}
	constexpr HINA_FORCE_INLINE HVectorEigen<T> &operator=(HVectorEigen<T> &&that)
	{
		return Inner.operator=(std::move(that.Inner));
		return *this;
	}

	template<typename S>
	HINA_FORCE_INLINE HVectorEigen<T> &operator=(const HVectorEigen<S> &v)
	{
		Inner[0] = v[0];
		Inner[1] = v[1];
		Inner[2] = v[2];
		return *this;
	}

	constexpr HINA_FORCE_INLINE const T &operator[](exint i) const noexcept
	{
		UT_ASSERT_P((0 <= i) && (i < tuple_size));
		return Inner.operator()(i);
	}

	constexpr HINA_FORCE_INLINE T &operator[](exint i) noexcept
	{
		UT_ASSERT_P((0 <= i) && (i < tuple_size));
		return Inner.operator()(i);
	}

	constexpr HINA_FORCE_INLINE T &operator()(unsigned i) noexcept
	{
		UT_ASSERT_P(i < tuple_size);
		return Inner.operator()(i);
	}

	constexpr HINA_FORCE_INLINE T operator()(unsigned i) const noexcept
	{
		UT_ASSERT_P(i < tuple_size);
		return Inner.operator()(i);
	}

	constexpr HINA_FORCE_INLINE const T *data() const noexcept { return Inner.data(); }
	constexpr HINA_FORCE_INLINE T *data() noexcept { return Inner.data(); }

	constexpr HINA_FORCE_INLINE HVectorEigen &operator+=(const HVectorEigen &a) noexcept
	{
		Inner.operator+=(a.Inner);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorEigen &operator-=(const HVectorEigen &a) noexcept
	{
		Inner.operator-=(a.Inner);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorEigen &operator+=(const T &a) noexcept
	{
		Inner.operator+=(a);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorEigen &operator-=(const T &a) noexcept
	{
		Inner.operator-=(a);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorEigen &operator*=(const T &a) noexcept
	{
		Inner.operator*=(a);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorEigen &operator/=(const T &a) noexcept
	{
		Inner.operator/=(a);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorEigen &operator*=(const HVectorEigen &a) noexcept
	{
		Inner.operator*=(a.Inner);
		return *this;
	}

	constexpr HINA_FORCE_INLINE HVectorEigen &operator/=(const HVectorEigen &a) noexcept
	{
		Inner.operator/=(a.Inner);
		return *this;
	}

	constexpr HINA_FORCE_INLINE T length() const noexcept { return Inner.norm(); }
	constexpr HINA_FORCE_INLINE T length2() const noexcept { return Inner.squaredNorm(); }
	constexpr HINA_FORCE_INLINE T distance(const HVectorEigen &b) const noexcept { return (Inner - b.Innder).norm(); }
	constexpr HINA_FORCE_INLINE T distance2(const HVectorEigen &b) const noexcept
	{
		return (Inner - b.Innder).squaredNorm();
	}
	T distance(const HVectorEigen<T> &p1, const HVectorEigen<T> &v1) const { return distance(p1.Inner, v1.Inner); }
	T distance(const HVectorEigen<T> &p1, const HVectorEigen<T> &v1, const HVectorEigen<T> &p2, const HVectorEigen<T> &v2) const { return distance(p1.Inner, v1.Inner, p2.Inner, v2.Inner); }

	HINA_FORCE_INLINE void normalize() noexcept { Inner.normalize(); }
	HINA_FORCE_INLINE HVectorEigen normalized() noexcept
	{
		HVectorEigen tmp(Inner.x(), Inner.y(), Inner.z());
		tmp.normalize();
		return tmp;
	}

	constexpr HINA_FORCE_INLINE T dot(const HVectorEigen &b) const noexcept { return Inner.dot(b.Inner); }
	constexpr HINA_FORCE_INLINE void cross(const HVectorEigen<T> &v) noexcept { Inner.cross(v.Inner); }
//	HINA_FORCE_INLINE void normal(const HVectorEigen<T> &va, const HVectorEigen<T> &vb) { Inner.normal(va.Inner, vb.Inner); }
//	HVectorEigen<T> getBary(const HVectorEigen<T> &t0, const HVectorEigen<T> &t1, const HVectorEigen<T> &t2, bool *degen = NULL) const { Inner.getBary(t0.Inner, t1.Inner, t2.Inner, degen); }

	constexpr HINA_FORCE_INLINE T &x() noexcept { return Inner.x(); }
	constexpr HINA_FORCE_INLINE T x() const noexcept { return Inner.x(); }
	constexpr HINA_FORCE_INLINE T &y() noexcept { return Inner.y(); }
	constexpr HINA_FORCE_INLINE T y() const noexcept { return Inner.y(); }
	constexpr HINA_FORCE_INLINE T &z() noexcept { return Inner.z(); }
	constexpr HINA_FORCE_INLINE T z() const noexcept { return Inner.z(); }
	constexpr HINA_FORCE_INLINE T &r() noexcept { return Inner.x(); }
	constexpr HINA_FORCE_INLINE T r() const noexcept { return Inner.x(); }
	constexpr HINA_FORCE_INLINE T &g() noexcept { return Inner.y(); }
	constexpr HINA_FORCE_INLINE T g() const noexcept { return Inner.y(); }
	constexpr HINA_FORCE_INLINE T &b() noexcept { return Inner.z(); }
	constexpr HINA_FORCE_INLINE T b() const noexcept { return Inner.z(); }

//	unsigned hash() const { return Inner.hash(); }

	// TODO: Matrix Operations

private:
	Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Inner;
};
}
#endif

using HVector = HinaPE::HVectorHDK<real>;
#ifdef USE_EIGEN
//using HVector = HinaPE::HVectorEigen<real>;
#endif

#endif //HINAPE_INTERFACE_H
