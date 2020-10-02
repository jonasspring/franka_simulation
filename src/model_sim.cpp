#include <franka_simulation/model_sim.h>

franka::Model* null_model = nullptr;

Model_Sim::Model_Sim() : franka::Model(std::move(*null_model))
{

}

std::array<double, 16> Model_Sim::pose(Frame frame, const franka::RobotState& robot_state) const{
  std::array<double, 16> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 16> Model_Sim::pose(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
const
{
  std::array<double, 16> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 42> Model_Sim::bodyJacobian(Frame frame, const franka::RobotState& robot_state)const{
  std::array<double, 42> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 42> Model_Sim::bodyJacobian(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
const
{
  std::array<double, 42> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 42> Model_Sim::zeroJacobian(Frame frame, const franka::RobotState& robot_state)const{
  std::array<double, 42> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 42> Model_Sim::zeroJacobian(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
const
{
  std::array<double, 42> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 49> Model_Sim::mass(const franka::RobotState& robot_state)const noexcept{
  std::array<double, 49> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 49> Model_Sim::mass(
    const std::array<double, 7>& q,
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
const noexcept
{
  std::array<double, 49> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 7> Model_Sim::coriolis(const franka::RobotState& robot_state)const noexcept{
  std::array<double, 7> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 7> Model_Sim::coriolis(
    const std::array<double, 7>& q,
    const std::array<double, 7>& dq,
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
const noexcept
{
  std::array<double, 7> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 7> Model_Sim::gravity(
    const std::array<double, 7>& q,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
    const std::array<double, 3>& gravity_earth)
const noexcept
{
  std::array<double, 7> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}

std::array<double, 7> Model_Sim::gravity(const franka::RobotState& robot_state,
                              const std::array<double, 3>& gravity_earth)
const noexcept
{
  std::array<double, 7> position;
  std::fill(std::begin(position), std::end(position), 0);
  return position;
}
