/**
(C) Copyright 2022 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
Contributors:
- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
*/

#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <Eigen/StdVector>

namespace DQ_robotics
{

///****************************************************************************************
///                        PRIVATE FUNCTIONS
/// ***************************************************************************************
/**
 * @brief _get_mdh_matrix returns a matrix related to the modified D-H parameters of the
 *                        Franka Emika Panda robot, which is
 *                        defined as
 *
 *                        Matrix<double, 5, 6> raw_franka_mdh;
 *                        raw_franka_mdh << theta,
 *                                              d,
 *                                              a,
 *                                           alpha,
 *                                            type_of_joints;
 * Source: https://frankaemika.github.io/docs/control_parameters.html
 *
 * @return MatrixXd raw_franka_mdh a matrix related to the modified D-H parameters
 */
MatrixXd _get_mdh_matrix()
{
    const double pi2 = pi/2.0;
    Matrix<double,5,7> raw_franka_mdh(5,7);
    raw_franka_mdh <<  0,    0,       0,         0,         0,      0,      0,
                     0.333,  0, 3.16e-1,         0,   3.84e-1,      0,      0,
                      0,     0,       0,   8.25e-2,  -8.25e-2,      0, 8.8e-2,
                      0,  -pi2,     pi2,       pi2,      -pi2,    pi2,    pi2,
                      0,     0,       0,         0,         0,      0,      0;

    return raw_franka_mdh;
}


/**
 * @brief _get_offset_base returns the base offset of the Franka Emika Panda robot.
 * @return a unit dual quatenion representing the base offset of the robot.
 */
DQ _get_offset_base()
{
    return 1 + E_ * 0.5 * DQ(0, 0.0413, 0, 0);
}

/**
 * @brief _get_offset_flange returns the end-effector offset of the Franka Emika Panda robot,
 *                           which is called "Flange" by the manufacturer.
 *                           This offset does not correspond to the end-effector tool but is part
 *                           of the modeling based on Craig's convention.
 * Source: https://frankaemika.github.io/docs/control_parameters.html
 * @return
 */
DQ _get_offset_flange()
{
    return 1+E_*0.5*k_*1.07e-1;
}

/**
 * @brief _get_q_limits returns the joint limits of the Franka Emika Panda robot
 *                      as suggested by the manufacturer.
 *
 * source: https://frankaemika.github.io/docs/control_parameters.html
 *
 * @return a std::make_tuple(q_min_, q_max_) containing the joint limits.
 */
std::tuple<const VectorXd, const VectorXd> _get_q_limits()
{
    const VectorXd q_max_ = ((VectorXd(7) <<  2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895).finished());
    const VectorXd q_min_ = ((VectorXd(7) << -2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895).finished());
    return std::make_tuple(q_min_, q_max_);
}


/**
 * @brief _get_q_dot_limits returns the joint velocity limits of the Franka Emika Panda robot
 *                          as suggested by the manufacturer.
 *
 * source: https://frankaemika.github.io/docs/control_parameters.html
 *
 * @return a std::make_tuple(q_min_dot_, q_max_dot_) containing the joint velocity limits.
 */
std::tuple<const VectorXd, const VectorXd> _get_q_dot_limits()
{
    const VectorXd q_min_dot_ = ((VectorXd(7) << -2, -1, -1.5, -1.25, -3, -1.5, -3).finished());
    const VectorXd q_max_dot_ = ((VectorXd(7) <<  2,  1,  1.5,  1.25,  3,  1.5,  3).finished());
    return std::make_tuple(q_min_dot_, q_max_dot_);
}




///****************************************************************************************
///                        PUBLIC FUNCTIONS
/// ***************************************************************************************


/**
 * @brief FrankaEmikaPandaRobot::kinematics returns an object of the class DQ_SerialManipulatorMDH()
 *                                          that contain the kinematic model of the Franka Emika Panda robot.
 *           Example of use:
 *
 *           #include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
 *           DQ_SerialManipulatorMDH franka = FrankaEmikaPandaRobot::kinematics();
 *           DQ x = franka.fkm(VectorXd::Zero(franka.get_dim_configuration_space()));
 *
 * @return A DQ_SerialManipulatorMDH() object of the desire robot
 */
DQ_SerialManipulatorMDH FrankaEmikaPandaRobot::kinematics()
{
    DQ_SerialManipulatorMDH franka(_get_mdh_matrix());
    franka.set_base_frame(_get_offset_base());
    franka.set_reference_frame(_get_offset_base());
    franka.set_effector(_get_offset_flange());
    VectorXd q_min;
    VectorXd q_max;
    VectorXd q_dot_min;
    VectorXd q_dot_max;
    std::tie(q_min, q_max) = _get_q_limits();
    std::tie(q_dot_min, q_dot_max) = _get_q_dot_limits();
    franka.set_lower_q_limit(q_min);
    franka.set_upper_q_limit(q_max);
    franka.set_lower_q_dot_limit(q_dot_min);
    franka.set_upper_q_dot_limit(q_dot_max);
    return franka;
}

}

