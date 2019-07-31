#include <dqrobotics/robot_control/DQ_KinematicConstrainedController.h>

namespace DQ_robotics
{

DQ_KinematicConstrainedController::DQ_KinematicConstrainedController(DQ_Kinematics *robot):DQ_KinematicController(robot)
{
    //Nothing to do
}

void DQ_KinematicConstrainedController::set_equality_constraint(const MatrixXd &B, const VectorXd &b)
{
    equality_constraint_matrix_ = B;
    equality_constraint_vector_ = b;
}

void DQ_KinematicConstrainedController::set_inequality_constraint(const MatrixXd &B, const VectorXd &b)
{
    inequality_constraint_matrix_ = B;
    inequality_constraint_vector_ = b;
}

}
