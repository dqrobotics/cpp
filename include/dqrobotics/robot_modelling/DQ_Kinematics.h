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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#ifndef DQ_ROBOT_MODELLING_DQ_KINEMATICS_H
#define DQ_ROBOT_MODELLING_DQ_KINEMATICS_H

#include<dqrobotics/DQ.h>

namespace DQ_robotics
{
    class DQ_Kinematics
    {
    private:
        DQ reference_frame_;
        DQ base_frame_;
        std::string name_;

    protected:
        VectorXd q;

    public:
        //Constructor
        DQ_Kinematics();

        //Virtual destructor
        virtual ~DQ_Kinematics()=0;

        //Concrete methods
        void set_reference_frame(const DQ& reference_frame);
        DQ   reference_frame() const;
        void set_base_frame(const DQ& base_frame);
        DQ   base_frame() const;
        void set_name(const std::string& name);
        std::string name() const;

        //Abstract methods
        virtual int get_dim_configuration_space()=0;
        virtual DQ  fkm(const VectorXd& joint_configurations)=0;
        virtual MatrixXd pose_jacobian(const VectorXd& joint_configurations,const int& to_link)=0;

        //Static methods
        static MatrixXd distance_jacobian(const MatrixXd& pose_jacobian, const DQ& pose);
        static MatrixXd translation_jacobian(const MatrixXd& pose_jacobian, const DQ& pose);
        static MatrixXd rotation_jacobian(const MatrixXd& pose_jacobian);
    };
}

#endif
