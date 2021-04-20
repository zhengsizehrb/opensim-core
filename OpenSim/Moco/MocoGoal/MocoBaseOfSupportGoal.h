#ifndef OPENSIM_MOCOBASEOFSUPPORTGOAL_H
#define OPENSIM_MOCOBASEOFSUPPORTGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoBaseOfSupportGoal.h                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2021 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MocoGoal.h"

namespace OpenSim {

using SimTK::Vec3;

class OSIMMOCO_API MocoBaseOfSupportGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoBaseOfSupportGoal, MocoGoal);

public:
    MocoBaseOfSupportGoal() { constructProperties(); }
    MocoBaseOfSupportGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoBaseOfSupportGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    void setLeftFootToePoint(std::string point) {
        set_left_toe_point(std::move(point));
    }
    void setLeftFootHeelPoint(std::string point) {
        set_left_heel_point(std::move(point));
    }
    void setRightFootToePoint(std::string point) {
        set_right_toe_point(std::move(point));
    }
    void setRightFootHeelPoint(std::string point) {
        set_right_heel_point(std::move(point));
    }

    void setSmoothing(double smoothing) { set_smoothing(smoothing); }
    double getSmoothing() { return get_smoothing(); }

    /// Set the vector to use for projecting the COM onto the ground.
    void setProjectionVector(SimTK::Vec3 normal) {
        set_projection_vector(std::move(normal));
    }
    /// Unset the projection vector.
    void clearProjectionVector() { updProperty_projection_vector().clear(); }
    SimTK::Vec3 getProjectionVector() const { return get_projection_vector(); }

    /** Set if the goal should be divided by the displacement of the system's
    center of mass over the phase. */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
        if (get_divide_by_displacement()) {
            cost[0] /= calcSystemDisplacement(
                    input.initial_state, input.final_state);
        }
    }

private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(left_toe_point, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(left_heel_point, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(right_toe_point, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(right_heel_point, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(smoothing, double, "TODO");
    OpenSim_DECLARE_PROPERTY(projection_vector, Vec3, "TODO");
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase (default: "
            "false)");

    mutable SimTK::ReferencePtr<const Station> m_left_toe_point;
    mutable SimTK::ReferencePtr<const Station> m_left_heel_point;
    mutable SimTK::ReferencePtr<const Station> m_right_toe_point;
    mutable SimTK::ReferencePtr<const Station> m_right_heel_point;

    mutable double m_smoothing;
    mutable SimTK::UnitVec3 m_projectionVector;

    using ConditionalFunction = double(const double&, const double&);
    mutable std::function<ConditionalFunction> m_conditional;

    using MidpointFunction = Vec3(const Vec3&, const Vec3&);
    mutable std::function<MidpointFunction> m_midpoint;

    using ProjectFunction = Vec3(const Vec3&);
    mutable std::function<ProjectFunction> m_project;

    using DistanceFunction = double(const Vec3&, const Vec3&);
    mutable std::function<DistanceFunction> m_distance;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOBASEOFSUPPORTGOAL_H