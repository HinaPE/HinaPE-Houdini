#ifndef HINAPE_KERNELS_H
#define HINAPE_KERNELS_H

/**
 * now it is only valid for Houdini UT_Vector3T<real>
 * TODO: make it more general
 */
template<typename real, typename Vector3>
struct ICubic
{
	inline static void set_radius(real r)
	{
		_r = r;
		const real pi = static_cast<real>(3.14159265358979323846);

		const real h3 = _r * _r * _r;
		_k = static_cast<real>(8.0) / (pi * h3);
		_l = static_cast<real>(48.0) / (pi * h3);
		_W_0 = W(Vector3{0, 0, 0});
	}
	inline static real W(const real r)
	{
		real res = 0.0;
		const real q = r / _r;
		if (q <= 1.0)
		{
			if (q <= 0.5)
			{
				const real q2 = q * q;
				const real q3 = q2 * q;
				res = _k * (static_cast<real>(6.0) * q3 - static_cast<real>(6.0) * q2 + static_cast<real>(1.0));
			} else
			{
				res = _k * (static_cast<real>(2.0) * pow(static_cast<real>(1.0) - q, static_cast<real>(3.0)));
			}
		}
		return res;
	}
	inline static real W(const Vector3 &r)
	{
		return W(r.length());
	}

	inline static Vector3 gradW(const Vector3 &r)
	{
		Vector3 res;
		const real rl = r.length();
		const real q = rl / _r;
		if ((rl > 1.0e-9) && (q <= 1.0))
		{
			Vector3 gradq = r / rl;
			gradq /= _r;
			if (q <= 0.5)
			{
				res = _l * q * ((real) 3.0 * q - static_cast<real>(2.0)) * gradq;
			} else
			{
				const real factor = static_cast<real>(1.0) - q;
				res = _l * (-factor * factor) * gradq;
			}
		} else
			res = Vector3{0, 0, 0};

		return res;
	}

	inline static real W_zero()
	{
		return _W_0;
	}

	inline static real _r;
	inline static real _k;
	inline static real _l;
	inline static real _W_0;
};

template<typename real, typename Vector3>
struct Poly6
{
    inline static void set_radius(real r){
        _r = r;
        const real pi = static_cast<real>(3.14159265358979323846);
        const real h3 = _r * _r * _r;
        const real h9 = h3 * h3 * h3;
        _k = static_cast<real>(315.0) / (static_cast<real>(64.0) * pi * h9);
        _l = -static_cast<real>(945.0) / (static_cast<real>(32.0) * pi * h9);
        _m = _l;
        _W_0 = W(Vector3{0, 0, 0});
    }

    /**
    * W(r,h) = (315/(64 pi h^9))(h^2-|r|^2)^3
    *        = (315/(64 pi h^9))(h^2-r*r)^3
    */
    inline static real W(const real r)
    {
        real res = 0.0;
        const real r2 = r * r;
        const real h2 = _r * _r;
        if (r2 <= h2)
        {
            res = _k * (h2 - r2) * (h2 - r2) * (h2 - r2);
        }
        return res;
    }

    inline static real W(const Vector3 &r)
    {
        return W(r.length());
    }

    /**
		* grad(W(r,h)) = r(-945/(32 pi h^9))(h^2-|r|^2)^2
		*              = r(-945/(32 pi h^9))(h^2-r*r)^2
		*/
    inline static Vector3 gradW(const Vector3 &r)
    {
        Vector3 res;
        const real rl = r.length();
        const real r2 = rl * rl;
        const real h2 = _r * _r;
        if (r2 <= h2)
        {
            res = _l * (h2 - r2) * (h2 - r2) * r;
        } else
            res = Vector3{0, 0, 0};
        return res;
    }

    inline static real laplacianW(const Vector3 &r)
    {
        real res = 0.0;
        const real rl = r.length();
        const real r2 = rl * rl;
        const real h2 = _r * _r;
        if (r2 <= h2)
        {
            res = _m * (h2 - r2) * (static_cast<real>(3.0) * h2 - static_cast<real>(7.0) * r2);
        }
        return res;
    }

    inline static real W_zero()
    {
        return _W_0;
    }

    inline static real _r;
    inline static real _k;
    inline static real _l;
    inline static real _m;
    inline static real _W_0;
};

template<typename real, typename Vector3>
struct Spiky
{
    inline static void set_radius(real r){
        _r = r;
        const real pi = static_cast<real>(3.14159265358979323846);
        const real h3 = _r * _r * _r;
        const real h6 = h3 * h3;
        _k = static_cast<real>(15.0) / (pi * h6);
        _l = -static_cast<real>(45.0) / (pi * h6);
        _W_0 = W(Vector3{0, 0, 0});
    }

    /**
		* W(r,h) = 15/(pi*h^6) * (h-r)^3
		*/
    inline static real W(const real r)
    {
        real res = 0.0;
        const real r2 = r * r;
        const real h2 = _r * _r;
        if (r <= _r)
        {
            res = _k * (_r - r) * (_r - r) * (_r - r);
        }
        return res;
    }

    inline static real W(const Vector3 &r)
    {
        return W(r.length());
    }

    /**
		* grad(W(r,h)) = -r(45/(pi*h^6) * (h-r)^2)
		*/
    inline static Vector3 gradW(const Vector3 &r)
    {
        Vector3 res;
        const real rl = r.length();
        const real r2 = rl * rl;
        const real h2 = _r * _r;
        if(r2 < h2)
        {
            res = _l * (_r - rl) * (_r - rl) * r * (static_cast<real>(1.0) / rl);
        } else
            res = Vector3{0, 0, 0};
        return res;
    }

    inline static real W_zero()
    {
        return _W_0;
    }

    inline static real _r;
    inline static real _k;
    inline static real _l;
    inline static real _W_0;
};

#endif //HINAPE_KERNELS_H
