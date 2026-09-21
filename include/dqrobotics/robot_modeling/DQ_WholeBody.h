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

#ifndef DQ_ROBOT_MODELLING_DQ_WHOLE_BODY_H
#define DQ_ROBOT_MODELLING_DQ_WHOLE_BODY_H

#include<vector>
#include<memory>
#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>

namespace DQ_robotics
{

/**
 * @brief Robot model composed of multiple kinematic chains connected in series.
 *
 * DQ_WholeBody concatenates several DQ_Kinematics objects and treats each one
 * as a whole subchain. The methods that take an index operate at the subchain
 * level, that is, they stop at the specified chain instead of at an individual link.
 *
 * @see DQ_Kinematics, DQ_SerialWholeBody
 */
class DQ_WholeBody : public DQ_Kinematics
{
protected:
    /** @brief Ordered list of kinematic chains that form the whole-body model. */
    std::vector<std::shared_ptr<DQ_Kinematics>> chain_;
    /**
     * @brief Checks whether a chain index is valid.
     *
     * @param to_ith_chain Chain index.
     * @throws std::runtime_error If @p to_ith_chain is outside the valid range.
     */
    void _check_to_ith_chain(const int& to_ith_chain) const;
public:
    /** @brief Deleted default constructor. */
    DQ_WholeBody()=delete;
    /**
     * @brief Constructs a whole-body model from its first chain.
     *
     * @param robot Shared pointer to the first chain.
     */
    DQ_WholeBody(std::shared_ptr<DQ_Kinematics> robot);

    /**
     * @brief Appends a new chain to the end of the whole-body model.
     *
     * @param robot Shared pointer to the chain to be appended.
     */
    void add(std::shared_ptr<DQ_Kinematics> robot);
    /**
     * @brief Computes the raw forward kinematics up to a given chain.
     *
     * The returned pose does not include the reference frame.
     *
     * @param q Combined configuration vector of the whole-body model.
     * @param to_ith_chain Index of the last chain to be accounted for.
     * @return Pose obtained by composing the forward kinematics of the chains up to @p to_ith_chain.
     * @throws std::runtime_error If @p to_ith_chain is outside the valid range.
     */
    DQ raw_fkm(const VectorXd& q, const int& to_ith_chain) const;
    /**
     * @brief Computes the raw forward kinematics of the complete whole-body model.
     *
     * @param q Combined configuration vector of the whole-body model.
     * @return Pose of the last chain without the reference frame.
     */
    DQ raw_fkm(const VectorXd& q) const;
    /**
     * @brief Sets an end-effector rigid transformation on the last chain.
     *
     * This method is intended for whole-body chains whose last element is a
     * serial manipulator.
     *
     * @param effector Constant rigid transformation from the last link to the tool frame.
     */
    void set_effector(const DQ& effector);

    /**
     * @brief Returns a raw pointer to one of the stored chains.
     *
     * @param to_ith_chain Chain index.
     * @return Raw pointer to the selected chain.
     * @throws std::runtime_error If @p to_ith_chain is outside the valid range.
     */
    DQ_Kinematics* get_chain(const int& to_ith_chain);
    /**
     * @brief Returns a copy of the selected chain as a DQ_SerialManipulatorDH.
     *
     * This method expects the selected chain to have dynamic type DQ_SerialManipulatorDH.
     *
     * @param to_ith_chain Chain index.
     * @return Copy of the selected chain as a DQ_SerialManipulatorDH.
     */
    DQ_SerialManipulatorDH get_chain_as_serial_manipulator_dh(const int& to_ith_chain) const;
    /**
     * @brief Returns a copy of the selected chain as a DQ_HolonomicBase.
     *
     * This method expects the selected chain to have dynamic type DQ_HolonomicBase.
     *
     * @param to_ith_chain Chain index.
     * @return Copy of the selected chain as a DQ_HolonomicBase.
     */
    DQ_HolonomicBase get_chain_as_holonomic_base(const int& to_ith_chain) const;

    //Abstract methods' implementation
    /**
     * @brief Computes the forward kinematics of the complete whole-body model.
     *
     * @param q Combined configuration vector of the whole-body model.
     * @return Whole-body pose including the reference frame.
     */
    DQ fkm(const VectorXd& q) const override;
    /**
     * @brief Computes the forward kinematics up to a given chain.
     *
     * This overload receives the combined configuration vector of the whole-body model
     * and stops the computation at the chain indexed by @p to_chain.
     *
     * @param to_chain Index of the last chain to be accounted for.
     * @return Whole-body pose up to the requested chain, including the reference frame.
     */
    DQ fkm(const VectorXd&, const int& to_chain) const override;
    /**
     * @brief Computes the pose Jacobian up to a given chain.
     *
     * @param q Combined configuration vector of the whole-body model.
     * @param to_ith_chain Index of the last chain to be accounted for.
     * @return Whole-body pose Jacobian up to the requested chain.
     */
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_ith_chain) const override;
    /**
     * @brief Computes the pose Jacobian of the complete whole-body model.
     *
     * @param q Combined configuration vector of the whole-body model.
     * @return Whole-body pose Jacobian.
     */
    MatrixXd pose_jacobian(const VectorXd& q) const override;
    /**
     * @brief Computes the time derivative of the pose Jacobian.
     *
     * @param q Combined configuration vector of the whole-body model.
     * @param q_dot Combined configuration-velocity vector.
     * @param to_ith_link Index parameter forwarded by the current interface.
     * @return The pose-Jacobian derivative.
     * @throws std::runtime_error Always, because this method is not implemented.
     */
    MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                      const VectorXd& q_dot,
                                      const int& to_ith_link) const override; //To be implemented.
    /**
     * @brief Computes the time derivative of the pose Jacobian of the complete whole-body model.
     *
     * @param q Combined configuration vector of the whole-body model.
     * @param q_dot Combined configuration-velocity vector.
     * @return The pose-Jacobian derivative.
     * @throws std::runtime_error Always, because this method is not implemented.
     */
    MatrixXd pose_jacobian_derivative (const VectorXd& q,
                                       const VectorXd& q_dot) const override; //To be implemented.
};

}

#endif
