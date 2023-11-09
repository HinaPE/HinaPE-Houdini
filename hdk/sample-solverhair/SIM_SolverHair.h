/*
 * Copyright (c) 2023
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 */

/*
 * The simplest network to see something working with this...
 *
 *   RBD Object    EmptyObject   
 *       \             | 
 *        \         HairSolver
 *         \           |
 *          \__    Gravity
 *             \    /
 *              Merge
 *
 * The gravity node should be set to have a non-zero X velocity.
 * The HairSolver node should have the name of the rbd object
 * (default rbdobject1) as its Source Object.
 */


#ifndef __SIM_SolverHair_h__
#define __SIM_SolverHair_h__

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Utils.h>

#define SIM_NAME_SOURCEOBJECTS	"sourceobjects"

class SIM_ObjectArray;
class SIM_GeometryCopy;

namespace HDK_Sample {

// This class implemented a computational fluid dynamics solver.
class SIM_SolverHair : public SIM_SingleSolver,
		       public SIM_OptionsUser
{
protected:
    explicit		 SIM_SolverHair(const SIM_DataFactory *factory);
                        ~SIM_SolverHair() override;

    SIM_Result           solveSingleObjectSubclass(SIM_Engine &engine,
					SIM_Object &object,
					SIM_ObjectArray &feedbacktoobjects,
					const SIM_Time &timestep,
					bool newobject) override;

    // These virtual functions can be overridden to create new hair
    // solvers that are based on this solver.
    virtual void	 createHairFromSource(SIM_GeometryCopy &hairgeo,
			    const SIM_ObjectArray &srcobjs) const;
    virtual void	 solveHair(SIM_GeometryCopy &hairgeo,
			    const SIM_ObjectArray &srcobjs,
			    const SIM_Object &object,
			    const SIM_Time &timestep) const;

private:
    static const SIM_DopDescription	*getSolverHairDopDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_SolverHair,
			SIM_SingleSolver,
			"Hair Solver",
			getSolverHairDopDescription());
};

} // End HDK_Sample namespace

#endif

