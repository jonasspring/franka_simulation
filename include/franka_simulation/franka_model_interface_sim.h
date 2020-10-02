#ifndef FRANKA_MODEL_INTERFACE_SIM_H
#define FRANKA_MODEL_INTERFACE_SIM_H

#include <franka_hw/franka_model_interface.h>
#include "franka_model.h"

#include <eigen3/Eigen/Dense>

struct DH_Parameters
{
  DH_Parameters() {}
  std::array<double, 8> dh_a = {0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0};
  std::array<double, 8> dh_d = {0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107};
  std::array<double, 8> dh_alpha = {0.0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2, 0.0};
};

class FrankaModelHandle_Sim
{
public:

  //franka::Model* model = nullptr;
  //franka::RobotState robot_state;
  FrankaModelHandle_Sim(const std::string& name, franka::RobotState& robot_state)
    :name_(name), robot_state_(&robot_state)
    //: FrankaModelHandle("test",  model,  robot_state), name_1(name)
  {
  }

  /**
   * Gets the name of the model handle.
   *
   * @return Name of the model handle.
   */
  std::string getName() const noexcept { return name_; }


  /**
   * Calculates the 7x7 mass matrix from the current robot state. Unit: \f$[kg \times m^2]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   *
   * @see franka::Model::mass
   */
  std::array<double, 49> getMass() const {
    Eigen::Vector7d q_vector;
    for(int i = 0; i < robot_state_->q.size(); i++){
      q_vector(i) = robot_state_->q[i];
    }

    Eigen::Matrix7d mass_matrix = MassMatrix(q_vector);
    std::array<double, 49> mass_array;
    //mass_array = mass_matrix.data();
    for(int i = 0; i<mass_matrix.cols();i++){
      for(int j = 0; j<mass_matrix.rows(); j++){
        mass_array[j+(i*mass_matrix.rows())] = mass_matrix(j,i);
      }
    }
    return mass_array;
  }

  /**
   * Calculates the 7x7 mass matrix from a given robot state. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] total_inertia Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] total_mass Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load
   * including end effector.
   * Unit: \f$[m]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   *
   * @see franka::Model::mass
   */
  std::array<double, 49> getMass(
      const std::array<double, 7>& q,
      const std::array<double, 9>& total_inertia,
      double total_mass,
      const std::array<double, 3>& F_x_Ctotal) const
  {
    Eigen::Vector7d q_vector;
    for(int i = 0; i < q.size(); i++){
      q_vector(i) = q[i];
    }

    Eigen::Matrix7d mass_matrix = MassMatrix(q_vector);
    std::array<double, 49> mass_array;
    //mass_array = mass_matrix.data();
    for(int i = 0; i<mass_matrix.cols();i++){
      for(int j = 0; j<mass_matrix.rows(); j++){
        mass_array[j+(i*mass_matrix.rows())] = mass_matrix(j,i);
      }
    }
    return mass_array;
  }

  /**
   * Calculates the Coriolis force vector (state-space equation) from the current robot state:
   * \f$ c= C \times dq\f$, in \f$[Nm]\f$.
   *
   * @return Coriolis force vector.
   *
   * @see franka::Model::coriolis
   */
  std::array<double, 7> getCoriolis() const { std::array<double, 7> position;
                                              std::fill(std::begin(position), std::end(position), 0);
                                              return position; }

  /**
   * Calculates the Coriolis force vector (state-space equation) from the given robot state:
   * \f$ c= C \times dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] dq Joint velocity. Unit: \f$[\frac{rad}{s}]\f$.
   * @param[in] total_inertia Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] total_mass Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load
   * including end effector.
   * Unit: \f$[m]\f$.
   *
   * @return Coriolis force vector.
   *
   * @see franka::Model::coriolis
   */
  std::array<double, 7> getCoriolis(
      const std::array<double, 7>& q,
      const std::array<double, 7>& dq,
      const std::array<double, 9>& total_inertia,
      double total_mass,
      const std::array<double, 3>& F_x_Ctotal) const {  std::array<double, 7> position;
                                                        std::fill(std::begin(position), std::end(position), 0);
                                                        return position;
  }

  /**
   * Calculates the gravity vector from the current robot state. Unit: \f$[Nm]\f$.
   *
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * Default to {0.0, 0.0, -9.81}.
   *
   * @return Gravity vector.
   *
   * @see franka::Model::gravity
   */
  std::array<double, 7> getGravity(const std::array<double, 3>& gravity_earth = {
                                       {0., 0., -9.81}}) const {
    Eigen::Vector7d q_vector;
    for(int i = 0; i < robot_state_->q.size(); i++){
      q_vector(i) = robot_state_->q[i];
    }

    Eigen::Vector7d gravity_vector_eigen = GravityVector(q_vector);
    std::array<double, 7> gravity_vector;
    //mass_array = mass_matrix.data();
    for(int i = 0; i<gravity_vector_eigen.rows();i++){
      gravity_vector[i] = gravity_vector_eigen(i);
    }

    return gravity_vector;
  }

  /**
   * Calculates the gravity vector from the given robot state. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] total_mass Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load
   * including end effector.
   * Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   * Default to {0.0, 0.0, -9.81}.
   *
   * @return Gravity vector.
   *
   * @see franka::Model::gravity
   */
  std::array<double, 7> getGravity(
      const std::array<double, 7>& q,
      double total_mass,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT (readability-identifier-naming)
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const
  {
    Eigen::Vector7d q_vector;
    for(int i = 0; i < q.size(); i++){
      q_vector(i) = q[i];
    }

    Eigen::Vector7d gravity_vector_eigen = GravityVector(q_vector);
    std::array<double, 7> gravity_vector;
    //mass_array = mass_matrix.data();
    for(int i = 0; i<gravity_vector_eigen.rows();i++){
      gravity_vector[i] = gravity_vector_eigen(i);
    }
    return gravity_vector;
  }

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame, calculated from the current
   * robot state.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   *
   * @see franka::Model::pose
   */
  std::array<double, 16> getPose(const franka::Frame& frame) const {
    Eigen::Matrix4f trans_matrix;
    trans_matrix.setZero(4, 4);
    trans_matrix(0,0) = 1.0;
    trans_matrix(1,1) = 1.0;
    trans_matrix(2,2) = 1.0;
    trans_matrix(3,3) = 1.0;

    int frame_number = static_cast<int>(frame);

    for(int i=0; i<=frame_number; i++){
      Eigen::Matrix4f local_trans_matrix;
      local_trans_matrix.setZero(4, 4);

      //calculate beforehand to reduce calculations
      double cos_q = std::cos(robot_state_->q[i]);
      double sin_q = std::sin(robot_state_->q[i]);
      double cos_alpha = std::cos(dh_parameters.dh_alpha[i]);
      double sin_alpha = std::sin(dh_parameters.dh_alpha[i]);

      local_trans_matrix(0,0) = cos_q;
      local_trans_matrix(1,0) = sin_q;

      local_trans_matrix(0,1) = - sin_q * cos_alpha;
      local_trans_matrix(1,1) = cos_q * cos_alpha;
      local_trans_matrix(2,1) =  sin_alpha;

      local_trans_matrix(0,2) =  sin_q * sin_alpha;
      local_trans_matrix(1,2) = - cos_q * sin_alpha;
      local_trans_matrix(2,2) =  cos_alpha;

      local_trans_matrix(0,3) =  dh_parameters.dh_a[i] * cos_q;
      local_trans_matrix(1,3) =  dh_parameters.dh_a[i] * sin_q;
      local_trans_matrix(2,3) =  dh_parameters.dh_d[i];
      local_trans_matrix(3,3) = 1.0;

      trans_matrix = trans_matrix * local_trans_matrix;
    }

    std::array<double, 16> pose_vector;
    for(int i = 0; i<trans_matrix.cols();i++){
      for(int j = 0; j<trans_matrix.rows(); j++){
        pose_vector[j+(i*trans_matrix.rows())] = trans_matrix(j,i);
      }
    }
    return pose_vector;
  }

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame, calculated from the current
   * robot state.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   *
   * @see franka::Model::pose
   */
  std::array<double, 16> getPose(const int frame) const {
    Eigen::Matrix4f trans_matrix;
    trans_matrix.setZero(4, 4);
    trans_matrix(0,0) = 1.0;
    trans_matrix(1,1) = 1.0;
    trans_matrix(2,2) = 1.0;
    trans_matrix(3,3) = 1.0;

    int frame_number = frame;

    for(int i=0; i<=frame_number; i++){
      Eigen::Matrix4f local_trans_matrix;
      local_trans_matrix.setZero(4, 4);

      //calculate beforehand to reduce calculations
      double cos_q = std::cos(robot_state_->q[i]);
      double sin_q = std::sin(robot_state_->q[i]);
      double cos_alpha = std::cos(dh_parameters.dh_alpha[i]);
      double sin_alpha = std::sin(dh_parameters.dh_alpha[i]);

      local_trans_matrix(0,0) = cos_q;
      local_trans_matrix(1,0) = sin_q;

      local_trans_matrix(0,1) = - sin_q * cos_alpha;
      local_trans_matrix(1,1) = cos_q * cos_alpha;
      local_trans_matrix(2,1) =  sin_alpha;

      local_trans_matrix(0,2) =  sin_q * sin_alpha;
      local_trans_matrix(1,2) = - cos_q * sin_alpha;
      local_trans_matrix(2,2) =  cos_alpha;

      local_trans_matrix(0,3) =  dh_parameters.dh_a[i] * cos_q;
      local_trans_matrix(1,3) =  dh_parameters.dh_a[i] * sin_q;
      local_trans_matrix(2,3) =  dh_parameters.dh_d[i];
      local_trans_matrix(3,3) = 1.0;

      trans_matrix = trans_matrix * local_trans_matrix;
    }

    std::array<double, 16> pose_vector;
    for(int i = 0; i<trans_matrix.cols();i++){
      for(int j = 0; j<trans_matrix.rows(); j++){
        pose_vector[j+(i*trans_matrix.rows())] = trans_matrix(j,i);
      }
    }
    return pose_vector;
  }

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame, calculated from the given
   * robot state.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   *
   * @see franka::Model::pose
   */
  std::array<double, 16> getPose(
      const franka::Frame& frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT (readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT (readability-identifier-naming)
      const {
    std::array<double, 16> position;
    std::fill(std::begin(position), std::end(position), 0);
    return position;
  }

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
   * the current robot state.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   *
   * @see franka::Model::bodyJacobian
   */
  std::array<double, 42> getBodyJacobian(const franka::Frame& frame) const {
    std::array<double, 42> position;
    std::fill(std::begin(position), std::end(position), 0);
    return position;
  }

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
   * the given robot state.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   *
   * @see franka::Model::bodyJacobian
   */
  std::array<double, 42> getBodyJacobian(
      const franka::Frame& frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT (readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT (readability-identifier-naming)
      const {
    std::array<double, 42> position;
    std::fill(std::begin(position), std::end(position), 0);
    return position;
  }

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
   * the current robot state.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   *
   * @see franka::Model::zeroJacobian
   */
  std::array<double, 42> getZeroJacobian(const franka::Frame& frame) const {
    Eigen::Matrix<double, 6, 7> jacobian;
    jacobian.setZero(6,7);

    int frame_number = static_cast<int>(frame);

    Eigen::Vector3d vec_z_0(0,0,1);

    std::array<double, 16> pose_frame = getPose(frame);
    Eigen::Vector3d vec_p_frame(pose_frame[12],pose_frame[13],pose_frame[14]);
    Eigen::Vector3d vec;

    vec = vec_z_0.cross(vec_p_frame);

    jacobian.block<3,1>(0,0) = vec;

    for(int i = 1; i<=frame_number; i++){
      pose_frame = getPose(i);
      Eigen::Vector3d vec_i_frame(pose_frame[13],pose_frame[14],pose_frame[15]);
      Eigen::Vector3d vec_z_i(pose_frame[8],pose_frame[9],pose_frame[10]);

      vec = vec_z_i.cross(vec_p_frame - vec_i_frame);
      jacobian.block<3,1>(0,i) = vec;

    }

    std::array<double, 42> jacobian_vector;
    for(int i = 0; i<jacobian.cols();i++){
      for(int j = 0; j<jacobian.rows(); j++){
        jacobian_vector[j+(i*jacobian.rows())] = jacobian(j,i);
      }
    }
    return jacobian_vector;
  }

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format and calculated from
   * the given robot state.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position. Unit: \f$[rad]\f$.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   *
   * @see franka::Model::zeroJacobian
   */
  std::array<double, 42> getZeroJacobian(
      const franka::Frame& frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT (readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT (readability-identifier-naming)
      const {
    std::array<double, 42> position;
    std::fill(std::begin(position), std::end(position), 0);
    return position;
  }

private:
  std::string name_;
  const franka::RobotState* robot_state_;
  DH_Parameters dh_parameters;

};

class FrankaModelInterface_Sim : public hardware_interface::HardwareResourceManager<FrankaModelHandle_Sim> {
};

#endif // FRANKA_MODEL_INTERFACE_SIM_H
