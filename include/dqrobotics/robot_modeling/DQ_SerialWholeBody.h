/**
(C) Copyright 2020-2022 DQ Robotics Developers

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

#ifndef DQ_ROBOT_MODELLING_DQ_SERIAL_WHOLE_BODY_H
#define DQ_ROBOT_MODELLING_DQ_SERIAL_WHOLE_BODY_H

#include<vector>
#include<memory>
#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>

namespace DQ_robotics
{

/**
 * @brief Robot model composed of multiple serially coupled kinematic chains.
 *
 * DQ_SerialWholeBody concatenates several DQ_Kinematics objects and exposes a
 * single combined link index across the whole serial composition. Methods with
 * explicit chain and link indices are provided to address a particular subchain.
 *
 * @note The constructor currently accepts the string "standard". The string
 * "reversed" is recognized by the implementation but is not implemented.
 *
 * @see DQ_Kinematics, DQ_WholeBody
 */
class DQ_SerialWholeBody : public DQ_Kinematics
{
protected:
    /** @brief Ordered list of kinematic chains that form the serial whole-body model. */
    std::vector<std::shared_ptr<DQ_Kinematics>> chain_;
    /**
     * @brief Checks whether a chain index is valid.
     *
     * @param to_ith_chain Chain index.
     * @throws std::runtime_error If @p to_ith_chain is outside the valid range.
     */
    void _check_to_ith_chain(const int& to_ith_chain) const;
    /**
     * @brief Checks whether a link index is valid inside a given chain.
     *
     * @param to_ith_chain Chain index.
     * @param to_jth_link Link index inside the selected chain.
     * @throws std::runtime_error If either index is outside the valid range.
     */
    void _check_to_jth_link_of_ith_chain(const int& to_ith_chain, const int& to_jth_link) const;
public:
    /** @brief Deleted default constructor. */
    DQ_SerialWholeBody()=delete;
    /**
     * @brief Constructs a serial whole-body model from its first chain.
     *
     * @param robot Shared pointer to the first chain.
     * @param type Composition type. The current implementation accepts "standard".
     * @throws std::runtime_error If @p type is invalid or corresponds to an unimplemented mode.
     */
    DQ_SerialWholeBody(std::shared_ptr<DQ_Kinematics> robot, const std::string type=std::string("standard"));

    /**
     * @brief Appends a new chain to the end of the serial whole-body model.
     *
     * @param robot Shared pointer to the chain to be appended.
     */
    void add(std::shared_ptr<DQ_Kinematics> robot);

    /**
     * @brief Computes the raw forward kinematics up to a specific link inside a specific chain.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @param to_ith_chain Index of the last chain to be accounted for.
     * @param to_jth_link Link index inside the last chain.
     * @return Pose obtained by composing the chains up to the requested chain and link, without the reference frame.
     */
    DQ raw_fkm_by_chain(const VectorXd& q, const int& to_ith_chain, const int& to_jth_link) const;
    /**
     * @brief Computes the raw forward kinematics up to the end of a specific chain.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @param to_ith_chain Index of the last chain to be accounted for.
     * @return Pose obtained by composing the chains up to the end of @p to_ith_chain, without the reference frame.
     */
    DQ raw_fkm_by_chain(const VectorXd& q, const int& to_ith_chain) const;
    /**
     * @brief Maps a combined link index to a pair of chain and local-link indices.
     *
     * @param to_ith_link Link index in the combined serial whole-body model.
     * @return Tuple containing the chain index and the link index inside that chain.
     * @throws std::runtime_error If the mapping cannot be determined.
     */
    std::tuple<int, int> get_chain_and_link_from_index(const int& to_ith_link) const;

    /**
     * @brief Computes the raw forward kinematics of the complete serial whole-body model.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @return Pose of the last link without the reference frame.
     */
    DQ raw_fkm(const VectorXd& q) const;
    /**
     * @brief Computes the raw forward kinematics up to a combined link index.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @param to_ith_link Link index in the combined serial whole-body model.
     * @return Pose of the requested link without the reference frame.
     */
    DQ raw_fkm(const VectorXd& q, const int& to_ith_link) const;

    /**
     * @brief Sets an end-effector rigid transformation on the last chain.
     *
     * This method is intended for serial whole-body chains whose last element is a
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
    /**
     * @brief Computes the raw pose Jacobian up to a specific link inside a specific chain.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @param to_ith_chain Index of the last chain to be accounted for.
     * @param to_jth_link Link index inside the last chain.
     * @return Raw pose Jacobian up to the requested chain and link.
     */
    MatrixXd raw_pose_jacobian_by_chain(const VectorXd& q, const int& to_ith_chain, const int& to_jth_link) const;
    /**
     * @brief Computes the raw time derivative of the pose Jacobian up to a specific link inside a specific chain.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @param q_dot Combined configuration-velocity vector.
     * @param to_ith_chain Index of the last chain to be accounted for.
     * @param to_jth_link Link index inside the last chain.
     * @return Raw pose-Jacobian derivative up to the requested chain and link.
     * @throws std::runtime_error Always, because this method is not implemented.
     */
    MatrixXd raw_pose_jacobian_derivative_by_chain(const VectorXd& q,
                                                   const VectorXd& q_dot,
                                                   const int& to_ith_chain,
                                                   const int& to_jth_link) const; //To be implemented.

    //Abstract methods' implementation
    /**
     * @brief Computes the forward kinematics of the complete serial whole-body model.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @return Serial whole-body pose including the reference frame.
     */
    DQ fkm(const VectorXd& q) const override;
    /**
     * @brief Computes the forward kinematics up to a combined link index.
     *
     * This overload receives the combined configuration vector of the serial whole-body model
     * and stops the computation at the link indexed by @p to_ith_link.
     *
     * @param to_ith_link Link index in the combined serial whole-body model.
     * @return Pose of the requested link including the reference frame.
     */
    DQ fkm(const VectorXd&, const int& to_ith_link) const override;
    /**
     * @brief Computes the pose Jacobian up to a combined link index.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @param to_ith_link Link index in the combined serial whole-body model.
     * @return Pose Jacobian up to the requested link.
     */
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_ith_link) const override;
    /**
     * @brief Computes the pose Jacobian of the complete serial whole-body model.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @return Pose Jacobian of the complete model.
     */
    MatrixXd pose_jacobian(const VectorXd& q) const override;
    /**
     * @brief Computes the time derivative of the pose Jacobian up to a combined link index.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @param q_dot Combined configuration-velocity vector.
     * @param to_ith_link Link index in the combined serial whole-body model.
     * @return Pose-Jacobian derivative up to the requested link.
     * @throws std::runtime_error Always, because the underlying derivative computation is not implemented.
     */
    MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                      const VectorXd& q_dot,
                                      const int& to_ith_link) const override; //To be implemented.
    /**
     * @brief Computes the time derivative of the pose Jacobian of the complete serial whole-body model.
     *
     * @param q Combined configuration vector of the serial whole-body model.
     * @param q_dot Combined configuration-velocity vector.
     * @return Pose-Jacobian derivative of the complete model.
     * @throws std::runtime_error Always, because the underlying derivative computation is not implemented.
     */
    MatrixXd pose_jacobian_derivative (const VectorXd& q,
                                       const VectorXd& q_dot) const override; //To be implemented.
};

}

#endif
