#ifndef EXCAVATOR_IK_HPP
#define EXCAVATOR_IK_HPP

#include <cmath>
#include <vector>
#include <iostream>

#include <urdf/model.h>

class ExcavatorIK
{
public:
  ExcavatorIK();
  ExcavatorIK(const std::string& urdf_file);
  ~ExcavatorIK();

  int loadURDF(const std::string& urdf_file);

  int inverseKinematics3Dof(const double x, const double y, const double z, std::vector<double>& joint_values);

  /* input: position of bucket_edge, angle of bucket_joint */
  int inverseKinematics4Dof(const double x, const double y, const double z, const double bucket_angle,
                            std::vector<double>& joint_values);

private:
  urdf::Model model_;

  // std::vector<double> offset_base_to_body_;
  // std::vector<double> offset_body_to_boom_;
  // std::vector<double> offset_boom_to_arm_;
  // std::vector<double> offset_arm_to_bucket_;
  // std::vector<double> offset_bucket_to_bucket_end_;
  urdf::Pose origin_base_to_body_;
  urdf::Pose origin_body_to_boom_;
  urdf::Pose origin_boom_to_arm_;
  urdf::Pose origin_arm_to_bucket_;
  urdf::Pose origin_bucket_to_bucket_end_;
  urdf::Pose origin_body_to_bucket_;
  std::vector<double> offset_body_to_bucket_;

  std::vector<double> joint_offset_;               // offset for ik
  std::vector<std::vector<double>> joint_limits_;  // limit for ik
};

#endif  // EXCAVATOR_IK_HPP