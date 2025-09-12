#include "../include/tms_if_for_opera/excavator_ik.hpp"

ExcavatorIK::ExcavatorIK()
{
}

ExcavatorIK::ExcavatorIK(const std::string& urdf_file)
{
  loadURDF(urdf_file);
}

ExcavatorIK::~ExcavatorIK()
{
}

int ExcavatorIK::loadURDF(const std::string& urdf_file)
{
  if (!model_.initString(urdf_file))
  {
    std::cout << "Failed to parse urdf file" << std::endl;
    return -1;
  }

  std::cout << "Successfully parsed urdf file" << std::endl;

  // Get origin
  origin_base_to_body_ = model_.getJoint("swing_joint")->parent_to_joint_origin_transform;
  origin_body_to_boom_ = model_.getJoint("boom_joint")->parent_to_joint_origin_transform;
  origin_boom_to_arm_ = model_.getJoint("arm_joint")->parent_to_joint_origin_transform;
  origin_arm_to_bucket_ = model_.getJoint("bucket_joint")->parent_to_joint_origin_transform;
  origin_bucket_to_bucket_end_ = model_.getJoint("bucket_end_joint")->parent_to_joint_origin_transform;
  offset_body_to_bucket_ = {
    origin_body_to_boom_.position.x + origin_boom_to_arm_.position.x + origin_arm_to_bucket_.position.x,
    origin_body_to_boom_.position.y + origin_boom_to_arm_.position.y + origin_arm_to_bucket_.position.y,
    origin_body_to_boom_.position.z + origin_boom_to_arm_.position.z + origin_arm_to_bucket_.position.z
  };

  // Calculate joint offset
  // TODO: pitch回転以外への対応
  // TODO: position.z==0のときの例外処理
  std::vector<double> rpy = { 0.0, 0.0, 0.0 };
  joint_offset_.resize(5, 0.0);
  origin_body_to_boom_.rotation.getRPY(rpy[0], rpy[1], rpy[2]);
  joint_offset_[1] -= rpy[1];
  joint_offset_[2] -= rpy[1];
  joint_offset_[3] -= rpy[1];

  joint_offset_[1] += atan2(origin_boom_to_arm_.position.z, origin_boom_to_arm_.position.x);  // boom_joint

  origin_boom_to_arm_.rotation.getRPY(rpy[0], rpy[1], rpy[2]);
  joint_offset_[2] -= rpy[1];
  joint_offset_[3] -= rpy[1];
  joint_offset_[2] -= joint_offset_[1];

  joint_offset_[2] += atan2(origin_arm_to_bucket_.position.z, origin_arm_to_bucket_.position.x);  // arm_joint

  origin_arm_to_bucket_.rotation.getRPY(rpy[0], rpy[1], rpy[2]);
  joint_offset_[3] -= rpy[1];
  joint_offset_[3] -= joint_offset_[1];
  joint_offset_[3] -= joint_offset_[2];
  joint_offset_[3] +=
      atan2(origin_bucket_to_bucket_end_.position.z, origin_bucket_to_bucket_end_.position.x);  // bucket_end_joint

  // std::cout << "joint_ofs:" << joint_offset_[0] * 180.0 / M_PI << ", " << joint_offset_[1] * 180.0 / M_PI << ", "
  //           << joint_offset_[2] * 180.0 / M_PI << ", " << joint_offset_[3] * 180.0 / M_PI << std::endl;

  // Get joint limits
  // TODO: Load joint limits.yaml if exists
  joint_limits_.resize(5, std::vector<double>(2, 0.0));

  if (model_.getJoint("swing_joint")->limits != NULL)
  {
    joint_limits_[0][0] = model_.getJoint("swing_joint")->limits->lower;
    joint_limits_[0][1] = model_.getJoint("swing_joint")->limits->upper;
  }
  else
  {
    joint_limits_[0][0] = -M_PI;
    joint_limits_[0][1] = M_PI;
  }

  if (model_.getJoint("boom_joint")->limits != NULL)
  {
    joint_limits_[1][0] = model_.getJoint("boom_joint")->limits->lower;
    joint_limits_[1][1] = model_.getJoint("boom_joint")->limits->upper;
  }
  else
  {
    joint_limits_[0][0] = -M_PI;
    joint_limits_[0][1] = M_PI;
  }

  if (model_.getJoint("arm_joint")->limits != NULL)
  {
    joint_limits_[2][0] = model_.getJoint("arm_joint")->limits->lower;
    joint_limits_[2][1] = model_.getJoint("arm_joint")->limits->upper;
  }
  else
  {
    joint_limits_[0][0] = -M_PI;
    joint_limits_[0][1] = M_PI;
  }

  if (model_.getJoint("bucket_joint")->limits != NULL)
  {
    joint_limits_[3][0] = model_.getJoint("bucket_joint")->limits->lower;
    joint_limits_[3][1] = model_.getJoint("bucket_joint")->limits->upper;
  }
  else
  {
    joint_limits_[0][0] = -M_PI;
    joint_limits_[0][1] = M_PI;
  }

  if (model_.getJoint("bucket_end_joint")->limits != NULL)
  {
    joint_limits_[4][0] = model_.getJoint("bucket_end_joint")->limits->lower;
    joint_limits_[4][1] = model_.getJoint("bucket_end_joint")->limits->upper;
  }
  else
  {
    joint_limits_[0][0] = -M_PI;
    joint_limits_[0][1] = M_PI;
  }

  return 0;
}

/* input: position of bucket_joint */
int ExcavatorIK::inverseKinematics3Dof(const double x, const double y, const double z,
                                       std::vector<double>& joint_values)
{
  if (!model_.getRoot())
  {
    std::cerr << "Error: model_ is not loaded." << std::endl;
    return -1;
  }

  std::vector<double> res_joint_values = joint_values;

  // Check if res_joint_values vector is empty
  if (res_joint_values.empty())
  {
    std::cerr << "Error: res_joint_values vector is empty." << std::endl;
    return -1;
  }

  // Check if res_joint_values vector has enough space for all joint values
  if (res_joint_values.size() < 3)
  {
    std::cerr << "Error: res_joint_values vector does not have enough space for all joint values." << std::endl;
    return -1;
  }

  std::vector<double> position_from_body = { x - origin_base_to_body_.position.x, y - origin_base_to_body_.position.y,
                                             z - origin_base_to_body_.position.z };

  double swing_angle_offset =
      atan2(offset_body_to_bucket_[1],
            sqrt(position_from_body[0] * position_from_body[0] + position_from_body[1] * position_from_body[1] -
                 offset_body_to_bucket_[1] * offset_body_to_bucket_[1]));

  res_joint_values[0] = atan2(position_from_body[1], position_from_body[0]) - swing_angle_offset;  // swing_joint

  std::vector<double> position_from_boom_tmp = {
    position_from_body[0] - (origin_body_to_boom_.position.x * cos(-res_joint_values[0]) +
                             origin_body_to_boom_.position.y * sin(-res_joint_values[0])),
    position_from_body[1] - (-origin_body_to_boom_.position.x * sin(-res_joint_values[0]) +
                             origin_body_to_boom_.position.y * cos(-res_joint_values[0])),
    position_from_body[2] - origin_body_to_boom_.position.z
  };
  std::vector<double> position_from_boom = {
    cos(res_joint_values[0]) * position_from_boom_tmp[0] + sin(res_joint_values[0]) * position_from_boom_tmp[1],
    -sin(res_joint_values[0]) * position_from_boom_tmp[0] + cos(res_joint_values[0]) * position_from_boom_tmp[1],
    position_from_boom_tmp[2]
  };

  std::vector<double> a(5, 0.0);
  a[1] = sqrt(origin_boom_to_arm_.position.x * origin_boom_to_arm_.position.x +
              origin_boom_to_arm_.position.z * origin_boom_to_arm_.position.z);
  a[2] = sqrt(origin_arm_to_bucket_.position.x * origin_arm_to_bucket_.position.x +
              origin_arm_to_bucket_.position.z * origin_arm_to_bucket_.position.z);

  a[4] = sqrt(position_from_boom[0] * position_from_boom[0] + position_from_boom[2] * position_from_boom[2]);

  res_joint_values[1] = -atan2(sqrt(1 - pow((a[1] * a[1] + a[4] * a[4] - a[2] * a[2]) / (2.0 * a[1] * a[4]), 2)),
                               (a[1] * a[1] + a[4] * a[4] - a[2] * a[2]) / (2.0 * a[1] * a[4])) -
                        atan2(position_from_boom[2], position_from_boom[0]);  // boom_joint
  res_joint_values[2] = -atan2(sqrt(1 - pow((a[1] * a[1] + a[2] * a[2] - a[4] * a[4]) / (2.0 * a[1] * a[2]), 2)),
                               (a[1] * a[1] + a[2] * a[2] - a[4] * a[4]) / (2.0 * a[1] * a[2])) +
                        M_PI;  // arm_joint

  for (int i = 0; i < res_joint_values.size(); i++)
  {
    if (res_joint_values[i] == NAN)
    {
      std::cerr << "Error: solution is not found." << std::endl;
      return -1;
    }

    // std::cout << joint_offset_[i] << " " << std::endl;
    res_joint_values[i] += joint_offset_[i];

    if (res_joint_values[i] < joint_limits_[i][0] || res_joint_values[i] > joint_limits_[i][1])
    {
      std::cerr << "Error: joint [" << i << "]==" << res_joint_values[i] << " is out of limit." << std::endl;
      return -1;
    }
  }

  joint_values = res_joint_values;

  return 0;
}

/* input: position of bucket_edge, バケットが水平面に対して成す角[rad] */
int ExcavatorIK::inverseKinematics4Dof(const double x, const double y, const double z, const double theta_w,
                                       std::vector<double>& joint_values)
{
  if (!model_.getRoot())
  {
    std::cerr << "Error: model_ is not loaded." << std::endl;
    return -1;
  }

  std::vector<double> res_joint_values = joint_values;

  // Check if res_joint_values vector is empty
  if (res_joint_values.empty())
  {
    std::cerr << "Error: res_joint_values vector is empty." << std::endl;
    return -1;
  }

  // Check if res_joint_values vector has enough space for all joint values
  if (res_joint_values.size() < 4)
  {
    std::cerr << "Error: res_joint_values vector does not have enough space for all joint values." << std::endl;
    return -1;
  }

  std::vector<double> position_from_body = { x - origin_base_to_body_.position.x, y - origin_base_to_body_.position.y,
                                             z - origin_base_to_body_.position.z };

  double swing_angle_offset =
      atan2(offset_body_to_bucket_[1],
            sqrt(position_from_body[0] * position_from_body[0] + position_from_body[1] * position_from_body[1] -
                 offset_body_to_bucket_[1] * offset_body_to_bucket_[1]));

  res_joint_values[0] = atan2(position_from_body[1], position_from_body[0]) - swing_angle_offset;  // swing_joint

  std::vector<double> position_from_boom_tmp = {
    position_from_body[0] - (origin_body_to_boom_.position.x * cos(-res_joint_values[0]) +
                             origin_body_to_boom_.position.y * sin(-res_joint_values[0])),
    position_from_body[1] - (-origin_body_to_boom_.position.x * sin(-res_joint_values[0]) +
                             origin_body_to_boom_.position.y * cos(-res_joint_values[0])),
    position_from_body[2] - origin_body_to_boom_.position.z
  };
  // 回転．y成分は0
  std::vector<double> position_from_boom = {
    cos(res_joint_values[0]) * position_from_boom_tmp[0] + sin(res_joint_values[0]) * position_from_boom_tmp[1],
    -sin(res_joint_values[0]) * position_from_boom_tmp[0] + cos(res_joint_values[0]) * position_from_boom_tmp[1],
    position_from_boom_tmp[2]
  };
  // std::cout << "position_from_boom: " << position_from_boom[0] << ", " << position_from_boom[1] << ", "
  //           << position_from_boom[2] << std::endl;

  std::vector<double> a(5, 0.0);
  a[1] = sqrt(origin_boom_to_arm_.position.x * origin_boom_to_arm_.position.x +
              origin_boom_to_arm_.position.z * origin_boom_to_arm_.position.z);
  a[2] = sqrt(origin_arm_to_bucket_.position.x * origin_arm_to_bucket_.position.x +
              origin_arm_to_bucket_.position.z * origin_arm_to_bucket_.position.z);
  a[3] = sqrt(origin_bucket_to_bucket_end_.position.x * origin_bucket_to_bucket_end_.position.x +
              origin_bucket_to_bucket_end_.position.z * origin_bucket_to_bucket_end_.position.z);

  std::vector<double> bucket_joint_position = { position_from_boom[0] - a[3] * cos(-theta_w), 0.0,
                                                position_from_boom[2] - a[3] * sin(-theta_w) };
  // std::cout << "bucket_joint_position: " << bucket_joint_position[0] << ", " << bucket_joint_position[1] << ", "
  //           << bucket_joint_position[2] << std::endl;
  a[4] =
      sqrt(bucket_joint_position[0] * bucket_joint_position[0] + bucket_joint_position[2] * bucket_joint_position[2]);

  res_joint_values[1] = -atan2(sqrt(1 - pow((a[1] * a[1] + a[4] * a[4] - a[2] * a[2]) / (2.0 * a[1] * a[4]), 2)),
                               (a[1] * a[1] + a[4] * a[4] - a[2] * a[2]) / (2.0 * a[1] * a[4])) -
                        atan2(bucket_joint_position[2], bucket_joint_position[0]);  // boom_joint

  res_joint_values[2] = -atan2(sqrt(1 - pow((a[1] * a[1] + a[2] * a[2] - a[4] * a[4]) / (2.0 * a[1] * a[2]), 2)),
                               (a[1] * a[1] + a[2] * a[2] - a[4] * a[4]) / (2.0 * a[1] * a[2])) +
                        M_PI;  // arm_joint

  res_joint_values[3] = theta_w - res_joint_values[1] - res_joint_values[2];  // bucket_joint

  for (int i = 0; i < res_joint_values.size(); i++)
  {
    // if (res_joint_values[i] == NAN)
    if(std::isnan(res_joint_values[i]))
    {
      std::cerr << "Error: solution is not found." << std::endl;
      return -1;
    }

    res_joint_values[i] += joint_offset_[i];

    if (res_joint_values[i] < joint_limits_[i][0] || res_joint_values[i] > joint_limits_[i][1])
    {
      std::cerr << "Error: joint [" << i << "]==" << res_joint_values[i] << " is out of limit." << std::endl;
      return -1;
    }
  }

  joint_values = res_joint_values;

  return 0;
}