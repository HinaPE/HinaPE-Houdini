#include "BoundarySolver.h"

void HinaPE::BoundarySolver::solve(SIM_RawField *IO_V_X, SIM_RawField *IO_V_Y, SIM_RawField *IO_V_Z, const SIM_RawField *CollisionSDF, const size_t iter)
{
	SIM_RawField MarkerX0;
	SIM_RawField MarkerY0;
	SIM_RawField MarkerZ0;
	_build_marker(&MarkerX0, IO_V_X, CollisionSDF, UT_Axis3::XAXIS);
	_build_marker(&MarkerY0, IO_V_Y, CollisionSDF, UT_Axis3::YAXIS);
	_build_marker(&MarkerZ0, IO_V_Z, CollisionSDF, UT_Axis3::ZAXIS);
	SIM_RawField MarkerX1 = MarkerX0;
	SIM_RawField MarkerY1 = MarkerY0;
	SIM_RawField MarkerZ1 = MarkerZ0;
	for (int i = 0; i < iter; ++i)
	{
		if (i % 2)
		{
			_extrapolate(IO_V_X, &MarkerX1, &MarkerX0);
			_extrapolate(IO_V_Y, &MarkerY1, &MarkerY0);
			_extrapolate(IO_V_Z, &MarkerZ1, &MarkerZ0);
		} else
		{
			_extrapolate(IO_V_X, &MarkerX0, &MarkerX1);
			_extrapolate(IO_V_Y, &MarkerY0, &MarkerY1);
			_extrapolate(IO_V_Z, &MarkerZ0, &MarkerZ1);
		}
	}
	SIM_RawField VEL_X0 = *IO_V_X;
	SIM_RawField VEL_Y0 = *IO_V_Y;
	SIM_RawField VEL_Z0 = *IO_V_Z;
	_fractional(IO_V_X, &VEL_X0, &VEL_Y0, &VEL_Z0, CollisionSDF, UT_Axis3::XAXIS);
	_fractional(IO_V_Y, &VEL_Y0, &VEL_X0, &VEL_Z0, CollisionSDF, UT_Axis3::YAXIS);
	_fractional(IO_V_Z, &VEL_Z0, &VEL_X0, &VEL_Y0, CollisionSDF, UT_Axis3::ZAXIS);
	_enforce_boundary(IO_V_X, IO_V_Y, IO_V_Z);
}
void HinaPE::BoundarySolver::_build_markerPartial(SIM_RawField *OUT_Marker, SIM_RawField *OUT_V, const SIM_RawField *CollisionSDF, const UT_Axis3::axis &AXIS, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	OUT_Marker->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		const UT_Vector3 pt = OUT_V->indexToPos({vit.x(), vit.y(), vit.z()});
		fpreal phi0, phi1;
		switch (AXIS)
		{
			case UT_Axis3::XAXIS:
			{
				phi0 = CollisionSDF->getValue(pt - 0.5 * OUT_V->getVoxelSize().x());
				phi1 = CollisionSDF->getValue(pt + 0.5 * OUT_V->getVoxelSize().x());
			}
				break;
			case UT_Axis3::YAXIS:
			{
				phi0 = CollisionSDF->getValue(pt - 0.5 * OUT_V->getVoxelSize().y());
				phi1 = CollisionSDF->getValue(pt + 0.5 * OUT_V->getVoxelSize().y());
			}
				break;
			case UT_Axis3::ZAXIS:
			{
				phi0 = CollisionSDF->getValue(pt - 0.5 * OUT_V->getVoxelSize().z());
				phi1 = CollisionSDF->getValue(pt + 0.5 * OUT_V->getVoxelSize().z());
			}
				break;
		}
		fpreal frac;
		if (phi0 < 0 && phi1 < 0)
			frac = 1;
		else if (phi0 < 0 && phi1 >= 0)
			frac = phi0 / (phi0 - phi1);
		else if (phi0 >= 0 && phi1 < 0)
			frac = phi1 / (phi1 - phi0);
		else
			frac = 0;
		frac = 1 - std::clamp(frac, (fpreal) 0, (fpreal) 1);

		if (frac > 0)
			vit.setValue(1);
		else
		{
			OUT_V->fieldNC()->setValue(vit.x(), vit.y(), vit.z(), 0);
			vit.setValue(0);
		}
	}
}
void HinaPE::BoundarySolver::_extrapolatePartial(SIM_RawField *OUT_V, SIM_RawField *OUT_Marker, const SIM_RawField *IN_Marker, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	OUT_Marker->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		if (!IN_Marker->field()->getValue(vit.x(), vit.y(), vit.z()))
		{
			fpreal sum = 0;
			size_t count = 0;

			if (vit.x() + 1 < OUT_V->field()->getXRes() && IN_Marker->field()->getValue(vit.x() + 1, vit.y(), vit.z()))
			{
				sum += OUT_V->field()->getValue(vit.x() + 1, vit.y(), vit.z());
				++count;
			}

			if (vit.x() > 0 && IN_Marker->field()->getValue(vit.x() - 1, vit.y(), vit.z()))
			{
				sum += OUT_V->field()->getValue(vit.x() - 1, vit.y(), vit.z());
				++count;
			}

			if (vit.y() + 1 < OUT_V->field()->getYRes() && IN_Marker->field()->getValue(vit.x(), vit.y() + 1, vit.z()))
			{
				sum += OUT_V->field()->getValue(vit.x(), vit.y() + 1, vit.z());
				++count;
			}

			if (vit.y() > 0 && IN_Marker->field()->getValue(vit.x(), vit.y() - 1, vit.z()))
			{
				sum += OUT_V->field()->getValue(vit.x(), vit.y() - 1, vit.z());
				++count;
			}

			if (vit.z() + 1 < OUT_V->field()->getZRes() && IN_Marker->field()->getValue(vit.x(), vit.y(), vit.z() + 1))
			{
				sum += OUT_V->field()->getValue(vit.x(), vit.y(), vit.z() + 1);
				++count;
			}

			if (vit.z() > 0 && IN_Marker->field()->getValue(vit.x(), vit.y(), vit.z() - 1))
			{
				sum += OUT_V->field()->getValue(vit.x(), vit.y(), vit.z() - 1);
				++count;
			}

			if (count > 0)
			{
				OUT_V->fieldNC()->setValue(vit.x(), vit.y(), vit.z(), (sum / count));
				OUT_Marker->fieldNC()->setValue(vit.x(), vit.y(), vit.z(), 1);
			}
		} else
			OUT_Marker->fieldNC()->setValue(vit.x(), vit.y(), vit.z(), 1);
	}
}
void HinaPE::BoundarySolver::_fractionalPartial(SIM_RawField *OUT_V, const SIM_RawField *IN_V_X, const SIM_RawField *IN_V_Y, const SIM_RawField *IN_V_Z, const SIM_RawField *CollisionSDF, const UT_Axis3::axis &AXIS, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	OUT_V->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		const UT_Vector3 pt = OUT_V->indexToPos({vit.x(), vit.y(), vit.z()});

		if (CollisionSDF->getValue(pt) < 0)
		{
			const UT_Vector3 vel_collider = {0, 0, 0};
			const UT_Vector3 vel = UT_Vector3D{
					IN_V_X->getValue(pt),
					IN_V_Y->getValue(pt),
					IN_V_Z->getValue(pt)
			};
			const UT_Vector3 g = CollisionSDF->getGradient(pt);
			if (g.length2() > std::numeric_limits<fpreal>::epsilon())
			{
				UT_Vector3 n = g;
				n.normalize();
				const UT_Vector3 vel_r = vel - vel_collider;
				const UT_Vector3 vel_t = _project_and_apply_friction(vel_r, n);
				const UT_Vector3 vel_p = vel_t + vel_collider;

				switch (AXIS)
				{
					case UT_Axis3::XAXIS:
						vit.setValue(vel_p.x());
						break;
					case UT_Axis3::YAXIS:
						vit.setValue(vel_p.y());
						break;
					case UT_Axis3::ZAXIS:
						vit.setValue(vel_p.z());
						break;
				}
			} else
			{
				switch (AXIS)
				{
					case UT_Axis3::XAXIS:
						vit.setValue(vel_collider.x());
						break;
					case UT_Axis3::YAXIS:
						vit.setValue(vel_collider.y());
						break;
					case UT_Axis3::ZAXIS:
						vit.setValue(vel_collider.z());
						break;
				}
			}
		}
	}
}
UT_Vector3 HinaPE::BoundarySolver::_project_and_apply_friction(const UT_Vector3 &vel, const UT_Vector3 normal)
{
	UT_Vector3 vel_t = vel.project(normal);
	if (vel_t.length2() > std::numeric_limits<fpreal>::epsilon())
	{
		const fpreal veln = std::max((fpreal) -vel.dot(normal), (fpreal) 0);
		vel_t *= std::max((fpreal) 1 - FRICTION * veln / vel_t.length(), (fpreal) 0);
	}
	return vel_t;
}
void HinaPE::BoundarySolver::_enforce_boundary(SIM_RawField *OUT_V_X, SIM_RawField *OUT_V_Y, SIM_RawField *OUT_V_Z)
{
	for (int i = 0; i < OUT_V_Z->getXRes(); ++i)
	{
		for (int j = 0; j < OUT_V_Z->getYRes(); ++j)
		{
			if (CLOSE_FRONT)
				OUT_V_Z->fieldNC()->setValue(i, j, 0, 0);
			if (CLOSE_BACK)
				OUT_V_Z->fieldNC()->setValue(i, j, OUT_V_Z->getZRes() - 1, 0);
		}
	}

	for (int j = 0; j < OUT_V_X->getYRes(); ++j)
	{
		for (int k = 0; k < OUT_V_X->getZRes(); ++k)
		{
			if (CLOSE_LEFT)
				OUT_V_X->fieldNC()->setValue(0, j, k, 0);
			if (CLOSE_RIGHT)
				OUT_V_X->fieldNC()->setValue(OUT_V_X->getXRes() - 1, j, k, 0);
		}
	}

	for (int i = 0; i < OUT_V_Y->getXRes(); ++i)
	{
		for (int k = 0; k < OUT_V_Y->getZRes(); ++k)
		{
			if (CLOSE_BOTTOM)
				OUT_V_Y->fieldNC()->setValue(i, 0, k, 0);
			if (CLOSE_TOP)
				OUT_V_Y->fieldNC()->setValue(i, OUT_V_Y->getYRes() - 1, k, 0);
		}
	}
}
