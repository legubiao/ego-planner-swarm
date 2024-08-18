//
// Created by tlab-uav on 8/18/24.
//

#ifndef SO3CONTROL_H
#define SO3CONTROL_H

#include <Eigen/Geometry>

class SO3Control {
public:
    SO3Control();

    void setMass(double mass);

    void setGravity(double g);

    void setPosition(const Eigen::Vector3d &position);

    void setVelocity(const Eigen::Vector3d &velocity);

    void setAcc(const Eigen::Vector3d &acc);

    void calculateControl(const Eigen::Vector3d &des_pos,
                          const Eigen::Vector3d &des_vel,
                          const Eigen::Vector3d &des_acc, double des_yaw,
                          const Eigen::Vector3d &kx,
                          const Eigen::Vector3d &kv);

    const Eigen::Vector3d &getComputedForce();

    const Eigen::Quaterniond &getComputedOrientation();


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // Inputs for the controller
    double mass_;
    double g_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d acc_;

    // Outputs of the controller
    Eigen::Vector3d force_;
    Eigen::Quaterniond orientation_;
};


#endif //SO3CONTROL_H
