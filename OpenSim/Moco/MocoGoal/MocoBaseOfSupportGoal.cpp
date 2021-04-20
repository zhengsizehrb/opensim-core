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

#include "MocoBaseOfSupportGoal.h"

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::square;

void MocoBaseOfSupportGoal::constructProperties() {
    constructProperty_left_toe_point("");
    constructProperty_left_heel_point("");
    constructProperty_right_toe_point("");
    constructProperty_right_heel_point("");
    constructProperty_smoothing(250.0);
    constructProperty_projection_vector(Vec3(0,1,0));
    constructProperty_divide_by_displacement(false);
}

void MocoBaseOfSupportGoal::initializeOnModelImpl(const Model& model) const {

    m_left_toe_point.reset(
        &model.getComponent<Station>(get_left_toe_point()));
    m_left_heel_point.reset(
        &model.getComponent<Station>(get_left_heel_point()));
    m_right_toe_point.reset(
        &model.getComponent<Station>(get_right_toe_point()));
    m_right_heel_point.reset(
        &model.getComponent<Station>(get_right_heel_point()));

    checkPropertyValueIsPositive(getProperty_smoothing());
    m_smoothing = get_smoothing();
    m_projectionVector = SimTK::UnitVec3(get_projection_vector());

    // Define the smoothing function.
    m_conditional = [](const double& cond, const double& smoothing) {
        return 0.5 + 0.5 * tanh(smoothing * cond);
    };

    m_midpoint = [](const Vec3& vec1, const Vec3& vec2) {
        return 0.5 * (vec1 + vec2);
    };

    Vec3 projectionVector(0, 1, 0);
    m_project = [projectionVector](const Vec3& vec) {
       return vec - SimTK::dot(vec, projectionVector) * projectionVector;
    };

    m_distance = [](const Vec3& vec1, const Vec3& vec2) {
        const Vec3 diff = vec2 - vec1;
        return diff.norm();
    };

    setRequirements(1, 1, SimTK::Stage::Position);
}

void MocoBaseOfSupportGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    getModel().realizePosition(state);

    const Vec3& leftToePoint = m_left_toe_point->getLocationInGround(state);
    const Vec3& leftHeelPoint = m_left_heel_point->getLocationInGround(state);
    const Vec3& rightToePoint = m_right_toe_point->getLocationInGround(state);
    const Vec3& rightHeelPoint = m_right_heel_point->getLocationInGround(state);

    const Vec3 toePoint = m_project(m_midpoint(leftToePoint, rightToePoint));
    const Vec3 heelPoint = m_project(m_midpoint(leftHeelPoint, rightHeelPoint));
    const Vec3 leftPoint = m_project(m_midpoint(leftToePoint, leftHeelPoint));
    const Vec3 rightPoint = m_project(m_midpoint(rightToePoint, rightHeelPoint));
    const Vec3 COM = m_project(getModel().calcMassCenterPosition(state));

    const double leftRightDistance = m_distance(leftPoint, rightPoint);
    const double heelToeDistance = m_distance(heelPoint, toePoint);
    const double leftCOMDistance = m_distance(leftPoint, COM);
    const double rightCOMDistance = m_distance(rightPoint, COM);
    const double toeCOMDistance = m_distance(toePoint, COM);
    const double heelCOMDistance = m_distance(heelPoint, COM);

    const double toeCOMError = toeCOMDistance - heelToeDistance;
    const double heelCOMError = heelCOMDistance - heelToeDistance;
    const double leftCOMError = leftCOMDistance - leftRightDistance;
    const double rightCOMError = rightCOMDistance - leftRightDistance;

    const double toeCOMFlag = m_conditional(toeCOMError, m_smoothing);
    const double heelCOMFlag = m_conditional(heelCOMError, m_smoothing);
    const double leftCOMFlag = m_conditional(leftCOMError, m_smoothing);
    const double rightCOMFlag = m_conditional(rightCOMError, m_smoothing);

    integrand = toeCOMFlag * square(toeCOMError) +
                heelCOMFlag * square(heelCOMError) +
                leftCOMFlag * square(leftCOMError) +
                rightCOMFlag * square(rightCOMError);
}
