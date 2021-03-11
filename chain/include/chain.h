#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <vector>

namespace chain {

template <class Scalar>
class Joint {
 public:
    using angle_type = Scalar;
    using position_type = Eigen::Matrix<Scalar, 3, 1>;
    using orientation_type = Eigen::Quaternion<Scalar>;

    Joint()
            : position_(0, 0, 0),
              orientation_(1, 0, 0, 0),  // w, x, y, z
              angle_(0),
              child_(nullptr) {}

    /// Construct a joint from position and orientation arrays.
    Joint(const Scalar* position, const Scalar* orientation)
            : position_(position),
              orientation_(orientation),
              angle_(0),
              child_(nullptr) {}

    const angle_type& get_angle() { return angle_; }

    const position_type& get_position() { return position_; }

    const orientation_type& get_orientation() { return orientation_; }

    /// Set the angle of the joint in radians.
    void set_angle(angle_type angle) {
        std::shared_ptr<Joint<Scalar>> next = child_;
        while (next != NULL) {
            orientation_type xrot;
            xrot = Eigen::AngleAxis<Scalar>(angle - angle_,
                                            position_type::UnitX());

            auto rotation = xrot * orientation_;
            rotation.normalize();

            next->position_ = rotation * (next->position_ - position_);
            next->orientation_ = rotation * next->orientation_;

            next->orientation_.normalize();

            next = next->child_;
        }

        angle_ = angle;
    }

    template <class T>
    friend class JointChain;

 private:
    position_type position_;
    orientation_type orientation_;
    angle_type angle_;
    std::shared_ptr<Joint<Scalar>> child_;
};

template <class Scalar>
class JointChain {
 public:
    using joint_type = Joint<Scalar>;

    /// Construct a new joint chain with a single base joint.
    JointChain() {
        // put one joint (the base) in the chain
        chain_.push_back(std::make_shared<joint_type>());
    }

    joint_type& add_joint(const Scalar* position, const Scalar* orientation) {
        chain_.push_back(std::make_shared<joint_type>(position, orientation));

        // also store a reference in the parent joint
        chain_.rbegin()[1]->child_ = chain_.back();

        return *chain_.back();
    }

    /// Return a reference to the first joint in the chain - the base.
    joint_type& base() { return *chain_.front(); }

    // Get the ith joint in the chain.
    joint_type& get_joint(size_t i) { return *chain_[i]; }

    /// Return a reference to the last joint in the chain - the gripper.
    joint_type& gripper() { return *chain_.back(); }

    /// The number of joints in the chain.
    size_t num_joints() { return chain_.size(); }

 private:
    std::vector<std::shared_ptr<joint_type>> chain_;
};

}  // namespace chain
