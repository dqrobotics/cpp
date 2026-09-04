/**
(C) Copyright 2019 DQ Robotics Developers

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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#ifndef DQ_ROBOT_CONTROL_DQ_KINEMATICCONSTRAINEDCONTROLLER_H
#define DQ_ROBOT_CONTROL_DQ_KINEMATICCONSTRAINEDCONTROLLER_H

#include <memory>

#include <dqrobotics/robot_control/DQ_KinematicController.h>

using namespace Eigen;

namespace DQ_robotics
{

/**
 * @brief Abstract superclass used to define concrete kinematic controllers with algebraic constraints.
 *
 * This class extends DQ_KinematicController with equality and inequality constraints
 * on the control input. Constrained controllers can store matrices and vectors that
 * are later supplied to optimization-based control laws.
 *
 * @see DQ_KinematicController, DQ_QuadraticProgrammingController
 */
class DQ_KinematicConstrainedController: public DQ_KinematicController
{
protected:
    /** @brief Matrix used in equality constraints of the form Aeq*u = beq. */
    MatrixXd equality_constraint_matrix_;
    /** @brief Vector used in equality constraints of the form Aeq*u = beq. */
    VectorXd equality_constraint_vector_;
    /** @brief Matrix used in inequality constraints of the form A*u <= b. */
    MatrixXd inequality_constraint_matrix_;
    /** @brief Vector used in inequality constraints of the form A*u <= b. */
    VectorXd inequality_constraint_vector_;

    /**
     * @brief Constructs a constrained controller from a legacy raw robot pointer.
     *
     * @param robot Non-owning pointer to the robot kinematic model.
     */
    [[deprecated("Use the smart pointer version instead")]]
    DQ_KinematicConstrainedController(DQ_Kinematics* robot);
    /**
     * @brief Constructs a constrained controller from a shared robot pointer.
     *
     * @param robot Shared pointer to the robot kinematic model.
     */
    DQ_KinematicConstrainedController(const std::shared_ptr<DQ_Kinematics>& robot);
public:
    //Remove default constructor
    DQ_KinematicConstrainedController()=delete;

    /**
     * @brief Sets the equality constraint passed to constrained control laws.
     *
     * @param B Equality-constraint matrix.
     * @param b Equality-constraint vector.
     */
    virtual void set_equality_constraint(const MatrixXd& B, const VectorXd& b);
    /**
     * @brief Sets the inequality constraint passed to constrained control laws.
     *
     * @param B Inequality-constraint matrix.
     * @param b Inequality-constraint vector.
     */
    virtual void set_inequality_constraint(const MatrixXd& B, const VectorXd& b);

};


}


#endif
