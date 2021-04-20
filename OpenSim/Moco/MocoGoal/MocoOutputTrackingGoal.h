#ifndef OPENSIM_MOCOOUTPUTTRACKINGGOAL_H
#define OPENSIM_MOCOOUTPUTTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoOutputTrackingGoal.h                                          *
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
#include "OpenSim/Simulation/TableProcessor.h"

namespace OpenSim {

enum DataType {
    Type_double,
    Type_Vec3,
};

class OSIMMOCO_API MocoOutputTrackingGoalGroup : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputTrackingGoalGroup, Object);

public:
    OpenSim_DECLARE_PROPERTY(output_path, std::string,
            "The absolute path to the output in the model to use as the "
            "integrand for this goal.");
    OpenSim_DECLARE_PROPERTY(weight, double, "");
    OpenSim_DECLARE_PROPERTY(index, int, "");

    MocoOutputTrackingGoalGroup();
    MocoOutputTrackingGoalGroup(const std::string& outputPath, double weight);
    MocoOutputTrackingGoalGroup(
            const std::string& outputPath, double weight, int index);
private:
    void constructProperties();
};

// TODO
class OSIMMOCO_API MocoOutputTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputTrackingGoal, MocoGoal);

public:
    MocoOutputTrackingGoal() { constructProperties(); }
    MocoOutputTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoOutputTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    void addOutputGroup(const std::string& outputPath, double weight) {
        append_output_groups(MocoOutputTrackingGoalGroup(outputPath, weight));
    }
    void addOutputGroup(
            const std::string& outputPath, double weight, int index) {
        append_output_groups(
                MocoOutputTrackingGoalGroup(outputPath, weight, index));
    }

    void setStatesReference(const TableProcessor& ref) {
        set_states_reference(std::move(ref));
    }

    void setMinimizeEndpointDeviation(bool tf) {
        set_minimize_endpoint_deviation(tf);
    }
    bool getMinimizeEndpointDeviation() const {
        return get_minimize_endpoint_deviation();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(
            output_groups, MocoOutputTrackingGoalGroup, "TODO.");
    OpenSim_DECLARE_PROPERTY(states_reference, TableProcessor,
            "Trajectories of model state "
            "variables from which tracked rotation data is computed. Column "
            "labels should be model state paths, "
            "e.g., '/jointset/ankle_angle_r/value'");
    OpenSim_DECLARE_PROPERTY(minimize_endpoint_deviation, bool, "TODO");
    void constructProperties();

    mutable GCVSplineSet m_refs_double;
    mutable GCVSplineSet m_refs_Vec3;
    mutable std::vector<DataType> m_data_types;
    mutable std::vector<SimTK::ReferencePtr<const AbstractOutput>> m_outputs;
    mutable std::vector<double> m_weights;
    mutable std::vector<int> m_indices;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOOUTPUTTRACKINGGOAL_H
