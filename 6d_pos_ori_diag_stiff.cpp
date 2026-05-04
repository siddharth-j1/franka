#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

struct TrajectoryStep {
  double x, y, z;
  double qw, qx, qy, qz;
  double vx, vy, vz;
};

struct LogEntry {
  double x, y, z, qw, qx, qy, qz;
};

int main(int argc, char** argv) {
  const std::string csv_file = (argc > 1) ? argv[1] : "exp3b_trajectory_10sec.csv";
  const std::string robot_ip = "192.168.1.12";

  // Fixed diagonal impedance. Optional command line override:
  // ./6d_pos_ori_diag_stiff csv_file Kx Ky Kz Krot
  const double kx = (argc > 2) ? std::stod(argv[2]) : 100.0;
  const double ky = (argc > 3) ? std::stod(argv[3]) : 100.0;
  const double kz = (argc > 4) ? std::stod(argv[4]) : 100.0;
  const double k_rot = (argc > 5) ? std::stod(argv[5]) : 50.0;

  const double dx = 2.0 * std::sqrt(kx);
  const double dy = 2.0 * std::sqrt(ky);
  const double dz = 2.0 * std::sqrt(kz);
  const double d_rot = 2.0 * std::sqrt(k_rot);

  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 6> damping;
  stiffness.setZero();
  damping.setZero();
  stiffness.diagonal() << kx, ky, kz, k_rot, k_rot, k_rot;
  damping.diagonal() << dx, dy, dz, d_rot, d_rot, d_rot;

  std::cout << "1. Loading trajectory poses from: " << csv_file << std::endl;
  std::cout << "   Using fixed diagonal translational stiffness only." << std::endl;
  std::cout << "   K diag = [" << kx << ", " << ky << ", " << kz << ", " << k_rot
            << ", " << k_rot << ", " << k_rot << "]" << std::endl;
  std::cout << "   D diag = [" << dx << ", " << dy << ", " << dz << ", " << d_rot
            << ", " << d_rot << ", " << d_rot << "]" << std::endl;

  std::ifstream file(csv_file);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open " << csv_file << std::endl;
    return -1;
  }

  std::vector<TrajectoryStep> plan;
  std::string line;
  std::getline(file, line);  // header

  bool warned_extra_columns = false;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream ss(line);
    std::string val;
    std::vector<double> row;
    while (std::getline(ss, val, ',')) {
      row.push_back(std::stod(val));
    }

    if (row.size() < 7) {
      std::cerr << "Error: CSV row " << (plan.size() + 2)
                << " has fewer than 7 pose columns." << std::endl;
      return -1;
    }

    if (row.size() > 7 && !warned_extra_columns) {
      std::cout << "   Note: CSV has stiffness/damping columns, but this controller ignores them."
                << std::endl;
      warned_extra_columns = true;
    }

    TrajectoryStep step{row[0], row[1], row[2], row[3], row[4], row[5], row[6], 0.0, 0.0, 0.0};
    const double q_norm =
        std::sqrt(step.qw * step.qw + step.qx * step.qx + step.qy * step.qy + step.qz * step.qz);
    if (q_norm < 0.9 || q_norm > 1.1) {
      std::cerr << "Error: Quaternion norm is " << q_norm << " on CSV row "
                << (plan.size() + 2) << std::endl;
      return -1;
    }
    step.qw /= q_norm;
    step.qx /= q_norm;
    step.qy /= q_norm;
    step.qz /= q_norm;

    plan.push_back(step);
  }

  if (plan.empty()) {
    std::cerr << "Error: No trajectory rows loaded." << std::endl;
    return -1;
  }

  std::cout << "   Loaded " << plan.size() << " control steps." << std::endl;

  constexpr double kControlDt = 0.001;
  for (size_t i = 0; i < plan.size(); i++) {
    const size_t prev = (i == 0) ? i : i - 1;
    const size_t next = (i + 1 >= plan.size()) ? i : i + 1;
    const double dt = static_cast<double>(next - prev) * kControlDt;
    if (dt > 0.0) {
      plan[i].vx = (plan[next].x - plan[prev].x) / dt;
      plan[i].vy = (plan[next].y - plan[prev].y) / dt;
      plan[i].vz = (plan[next].z - plan[prev].z) / dt;
    }
  }

  const int hold_steps = 2000;
  std::vector<LogEntry> actual_log(plan.size() + hold_steps);

  try {
    std::cout << "2. Connecting to Franka hardware..." << std::endl;
    franka::Robot robot(robot_ip);
    robot.automaticErrorRecovery();
    franka::Model model = robot.loadModel();

    std::cout << "PRESS ENTER TO EXECUTE 1000HZ DIAGONAL IMPEDANCE LOOP";
    std::cin.ignore();

    franka::RobotState initial_state = robot.readOnce();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d initial_pos = initial_transform.translation();
    Eigen::Quaterniond initial_ori(initial_transform.rotation());

    int current_step = 0;
    const int max_step = static_cast<int>(plan.size()) - 1;
    const int finish_step = max_step + hold_steps;

    auto impedance_control_callback =
        [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
      (void)period;

      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position = transform.translation();
      Eigen::Quaterniond orientation(transform.rotation());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

      if (current_step < static_cast<int>(actual_log.size())) {
        actual_log[current_step] = {position.x(), position.y(), position.z(),
                                    orientation.w(), orientation.x(), orientation.y(),
                                    orientation.z()};
      }

      const int plan_step = std::min(current_step, max_step);
      const TrajectoryStep target = plan[plan_step];
      Eigen::Vector3d target_pos_csv(target.x, target.y, target.z);
      Eigen::Vector3d target_vel_csv(target.vx, target.vy, target.vz);
      Eigen::Quaterniond target_ori_csv(target.qw, target.qx, target.qy, target.qz);

      if (initial_ori.coeffs().dot(target_ori_csv.coeffs()) < 0.0) {
        target_ori_csv.coeffs() = -target_ori_csv.coeffs();
      }

      const double alpha = std::min(1.0, static_cast<double>(current_step) / 500.0);
      Eigen::Vector3d target_pos = (1.0 - alpha) * initial_pos + alpha * target_pos_csv;
      Eigen::Vector3d target_vel = target_vel_csv;
      if (current_step < 500) {
        target_vel = target_vel_csv * alpha + (target_pos_csv - initial_pos) / 0.5;
      }
      if (current_step > max_step) {
        target_vel.setZero();
      }
      Eigen::Quaterniond target_ori = initial_ori.slerp(alpha, target_ori_csv);

      Eigen::Vector3d error_pos = target_pos - position;

      if (target_ori.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() = -orientation.coeffs();
      }

      Eigen::Quaterniond error_quaternion(orientation.inverse() * target_ori);
      Eigen::Vector3d error_ori_vec(error_quaternion.x(), error_quaternion.y(),
                                    error_quaternion.z());
      error_ori_vec = transform.rotation() * error_ori_vec;

      Eigen::Matrix<double, 6, 1> error;
      error.head(3) = error_pos;
      error.tail(3) = error_ori_vec;

      Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;
      Eigen::Matrix<double, 6, 1> desired_velocity;
      desired_velocity.setZero();
      desired_velocity.head(3) = target_vel;
      Eigen::Matrix<double, 6, 1> force_ext = stiffness * error - damping * (velocity - desired_velocity);

      Eigen::Matrix<double, 7, 1> tau_d = jacobian.transpose() * force_ext + coriolis;

      const double kDeltaTauMax = 0.99;
      std::array<double, 7> tau_d_rate_limited;
      for (size_t i = 0; i < 7; i++) {
        const double difference = tau_d[i] - robot_state.tau_J_d[i];
        tau_d_rate_limited[i] =
            robot_state.tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
      }

      if (current_step >= finish_step) {
        return franka::MotionFinished(franka::Torques(tau_d_rate_limited));
      }
      current_step++;
      return tau_d_rate_limited;
    };

    robot.control(impedance_control_callback);
    std::cout << "Hardware execution complete." << std::endl;

    std::cout << "3. Saving hardware log to execution_log_diag.csv..." << std::endl;
    std::ofstream log_file("execution_log_diag.csv");
    log_file << "x,y,z,qw,qx,qy,qz\n";
    for (const auto& entry : actual_log) {
      log_file << entry.x << "," << entry.y << "," << entry.z << "," << entry.qw << ","
               << entry.qx << "," << entry.qy << "," << entry.qz << "\n";
    }
    log_file.close();
    std::cout << "Log saved successfully." << std::endl;

  } catch (const franka::Exception& e) {
    std::cout << "Hardware Error: " << e.what() << std::endl;
  }

  return 0;
}
