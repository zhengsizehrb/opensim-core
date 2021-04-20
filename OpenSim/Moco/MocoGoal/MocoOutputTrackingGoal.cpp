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

#include "MocoOutputTrackingGoal.h"
#include "OpenSim/Common/STOFileAdapter.h"

using namespace OpenSim;

OpenSim::MocoOutputTrackingGoalGroup::MocoOutputTrackingGoalGroup() {
    constructProperties();
}

MocoOutputTrackingGoalGroup::MocoOutputTrackingGoalGroup(
        const std::string& outputPath, double weight) {
    constructProperties();
    set_output_path(outputPath);
    set_weight(weight);
}

MocoOutputTrackingGoalGroup::MocoOutputTrackingGoalGroup(
        const std::string& outputPath, double weight, int index) : 
        MocoOutputTrackingGoalGroup(outputPath, weight){
    set_index(index);
}

void MocoOutputTrackingGoalGroup::constructProperties() {
    constructProperty_output_path("");
    constructProperty_weight(1.0);
    constructProperty_index(0);
}

void MocoOutputTrackingGoal::constructProperties() {
    constructProperty_output_groups();
    constructProperty_states_reference(TableProcessor());
    constructProperty_minimize_endpoint_deviation(false);
}

void MocoOutputTrackingGoal::initializeOnModelImpl(
        const Model& model) const {
    
    std::string outputPath;
    std::string componentPath;
    std::string outputName;
    std::string channelName;
    std::string alias;
    const int numGroups = getProperty_output_groups().size();
    m_outputs.resize(numGroups);
    std::vector<std::string> doublePaths;
    std::vector<std::string> vec3Paths;
    for (int ig = 0; ig < numGroups; ++ig) {
        const auto& group = get_output_groups(ig);

        outputPath = group.get_output_path();
        OPENSIM_THROW_IF_FRMOBJ(outputPath.empty(), Exception,
                "No output_path provided.");

        AbstractInput::parseConnecteePath(outputPath,
                componentPath, outputName, channelName, alias);
        const auto& component = model.getComponent(componentPath);
        const auto* abstractOutput = &component.getOutput(outputName);

        if (dynamic_cast<const Output<double>*>(abstractOutput)) {
            m_data_types.push_back(Type_double);
            doublePaths.push_back(outputPath);
        } else if (dynamic_cast<const Output<SimTK::Vec3>*>(abstractOutput)) {
            m_data_types.push_back(Type_Vec3);
            vec3Paths.push_back(outputPath);
        } else {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Data type of specified model output not supported.");
        }

        m_outputs[ig].reset(abstractOutput);
        m_indices.push_back(group.get_index());
        m_weights.push_back(group.get_weight());
    }

    TimeSeriesTable statesTable =
            get_states_reference().processAndConvertToRadians(model);

    // Check that the reference state names match the model state names.
    checkLabelsMatchModelStates(model, statesTable.getColumnLabels());

    // Create the StatesTrajectory.
    auto statesTraj =
            StatesTrajectory::createFromStatesTable(model, statesTable);

    TimeSeriesTable doubleTable;
    TimeSeriesTableVec3 vec3Table;
    
    for (const auto& state : statesTraj) {
        // This realization ignores any SimTK::Motions prescribed in the
        // model.
        model.realizeAcceleration(state);
        std::vector<double> doubles;
        std::vector<SimTK::Vec3> vec3s;
        for (int ig = 0; ig < numGroups; ++ig) {
            const auto& group = get_output_groups(ig);
            if (m_data_types[ig] == Type_double) {
                doubles.push_back(
                    static_cast<const Output<double>*>(m_outputs[ig].get())
                                ->getValue(state));
            } else if (m_data_types[ig] == Type_Vec3) {
                vec3s.push_back(static_cast<const Output<SimTK::Vec3>*>(
                        m_outputs[ig].get())
                                ->getValue(state));
            }
        }
        doubleTable.appendRow(state.getTime(), doubles);
        vec3Table.appendRow(state.getTime(), vec3s);
    }
    doubleTable.setColumnLabels(doublePaths);
    vec3Table.setColumnLabels(vec3Paths);

    m_refs_double = GCVSplineSet(doubleTable);
    m_refs_Vec3 = GCVSplineSet(vec3Table.flatten());

    const int numEndpoints = get_minimize_endpoint_deviation() ? numGroups : 0;
    setRequirements(1, 1 + numEndpoints, SimTK::Stage::Acceleration);
}

void MocoOutputTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    getModel().realizeAcceleration(input.state);
    SimTK::Vector timeVec(1, input.state.getTime());

    double model_val = 0;
    double ref_val = 0;
    int idouble = 0;
    int ivec3 = 0;
    for (int i = 0; i < (int)m_outputs.size(); ++i) {
        if (m_data_types[i] == Type_double) {
            model_val = static_cast<const Output<double>*>(m_outputs[i].get())
                            ->getValue(input.state);
            ref_val = m_refs_double[idouble].calcValue(timeVec);
            ++idouble;
        } else if (m_data_types[i] == Type_Vec3) {
            model_val = static_cast<const Output<SimTK::Vec3>*>(m_outputs[i].get())
                            ->getValue(input.state)[m_indices[i]];
            ref_val = m_refs_Vec3[ivec3 + m_indices[i]].calcValue(timeVec);
            ivec3 += 3;
        }
        double error = model_val - ref_val;
        integrand += m_weights[i] * error * error;
    }
}

void MocoOutputTrackingGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;

    if (get_minimize_endpoint_deviation()) {
        getModel().realizeAcceleration(input.final_state);
        SimTK::Vector timeVec(1, input.final_state.getTime());

        double model_val = 0;
        double ref_val = 0;
        int idouble = 0;
        int ivec3 = 0;
        for (int i = 0; i < (int)m_outputs.size(); ++i) {
            if (m_data_types[i] == Type_double) {
                model_val =
                        static_cast<const Output<double>*>(m_outputs[i].get())
                                ->getValue(input.final_state);
                ref_val = m_refs_double[idouble].calcValue(timeVec);
                ++idouble;
            } else if (m_data_types[i] == Type_Vec3) {
                model_val = static_cast<const Output<SimTK::Vec3>*>(
                        m_outputs[i].get())
                                    ->getValue(input.final_state)[m_indices[i]];
                ref_val = m_refs_Vec3[ivec3 + m_indices[i]].calcValue(timeVec);
                ivec3 += 3;
            }
            double error = model_val - ref_val;
            cost[i + 1] = m_weights[i] * error * error;
        }
    }
    
}