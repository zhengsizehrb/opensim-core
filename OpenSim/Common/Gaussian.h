#ifndef OPENSIM_GAUSSIAN_H_
#define OPENSIM_GAUSSIAN_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Gaussian.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// INCLUDES
#include "Function.h"
#include "FunctionAdapter.h"
#include <string>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a Gaussian function.
 *
 * This class inherits from Function and can be used as input to
 * any Component requiring a Function as input. Implements:
 *  f(x) = a*exp(-(x-b)^2 / 2*c^2);
 *
 * @author Nicholas Bianco
 * @version 1.0
 */
class OSIMCOMMON_API Gaussian : public Function {
    OpenSim_DECLARE_CONCRETE_OBJECT(Gaussian, Function);

protected:
    //==============================================================================
    // PROPERTIES
    //==============================================================================

    OpenSim_DECLARE_PROPERTY(
            a, double, "TODO.");

    OpenSim_DECLARE_PROPERTY(
            b, double, "TODO.");

    OpenSim_DECLARE_PROPERTY(
            c, double, "TODO.");


    //=============================================================================
    // METHODS
    //=============================================================================
public:
    // Default construct, copy and assignment
    Gaussian() { constructProperties(); }

    // Convenience Constructor
    Gaussian(double a, double b, double c) : Gaussian() {
        set_a(a);
        set_b(b);
        set_c(c);
    }

    virtual ~Gaussian(){};

    //--------------------------------------------------------------------------
    // EVALUATION
    //--------------------------------------------------------------------------
    double calcValue(const SimTK::Vector& x) const override {
        return get_a() * exp(-(x[0]-get_b())*(x[0]-get_b()) / 
                            (2*get_c()*get_c()));
    }

    double calcDerivative(const std::vector<int>& derivComponents,
            const SimTK::Vector& x) const override {
        int n = (int)derivComponents.size();

        // TODO hard code derivatives
        return 1;
    }

    SimTK::Function* createSimTKFunction() const override {
        return new FunctionAdapter(*this);
    }

    int getArgumentSize() const override { return 1; }
    int getMaxDerivativeOrder() const override { return 1; }

private:
    void constructProperties() {
        constructProperty_a(1.0);
        constructProperty_b(0.0);
        constructProperty_c(1.0);
    }
//=============================================================================
}; // END class Gaussian
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SINE_H_
