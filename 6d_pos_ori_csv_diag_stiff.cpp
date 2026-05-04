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
  double kx, ky, kz;
  double vx, vy, vz;
};

struct LogEntry {
  double target_x, target_y, target_z;
  double actual_x, actual_y, actual_z;
  double error_x, error_y, error_z;
  double force_x, force_y, force_z;
  double qw, qx, qy, qz;
};

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

int main(int argc, char** argv) {
  const std::string csv_file =
      (argc > 1) ? argv[1] : "exp3b_trajectory_10sec_smooth.csv";
  const std::string robot_ip = "192.168.1.12";

  // Optional command line:
  // ./6d_pos_ori_csv_diag_stiff csv_file stiffness_scale k_rot max_trans_k damping_scale
  //                                min_kx min_ky min_kz
  const double stiffness_scale = (argc > 2) ? std::stod(argv[2]) : 1.0;
  const double k_rot = (argc > 3) ? std::stod(argv[3]) : 60.0;
  const double max_trans_k = (argc > 4) ? std::stod(argv[4]) : 250.0;
  const double damping_scale = (argc > 5) ? std::stod(argv[5]) : 1.0;
  const double min_kx = (argc > 6) ? std::stod(argv[6]) : 30.0;
  const double min_ky = (argc > 7) ? std::stod(argv[7]) : min_kx;
  const double min_kz = (argc > 8) ? std::stod(argv[8]) : min_kx;
  const double d_rot = 2.0 * std::sqrt(k_rot);

  std::cout << "1. Loading trajectory + diagonal CSV stiffness from: " << csv_file
            << std::endl;
  std::cout << "   Using Kx=k11, Ky=k22, Kz=k33 only. Off-diagonal terms ignored."
            << std::endl;
  std::cout << "   Translational Kx = clamp(csv_k11 * " << stiffness_scale << ", "
            << min_kx << ", " << max_trans_k << ")" << std::endl;
  std::cout << "   Translational Ky = clamp(csv_k22 * " << stiffness_scale << ", "
            << min_ky << ", " << max_trans_k << ")" << std::endl;
  std::cout << "   Translational Kz = clamp(csv_k33 * " << stiffness_scale << ", "
            << min_kz << ", " << max_trans_k << ")" << std::endl;
  std::cout << "   Translational D = " << damping_scale << " * 2 * sqrt(K)"
            << std::endl;
  std::cout << "   Rotational K = " << k_rot << ", D = " << d_rot << std::endl;

  std::ifstream file(csv_file);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open " << csv_file << std::endl;
    return -1;
  }

  std::vector<TrajectoryStep> plan;
  std::string line;
  std::getline(file, line);

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

    if (row.size() < 16) {
      std::cerr << "Error: CSV row " << (plan.size() + 2)
                << " must contain pose plus 3x3 stiffness columns." << std::endl;
      return -1;
    }

    TrajectoryStep step;
    step.x = row[0];
    step.y = row[1];
    step.z = row[2];
    step.qw = row[3];
    step.qx = row[4];
    step.qy = row[5];
    step.qz = row[6];

    const double q_norm =
        std::sqrt(step.qw * step.qw + step.qx * step.qx + step.qy * step.qy +
                  step.qz * step.qz);
    if (q_norm < 0.9 || q_norm > 1.1) {
      std::cerr << "Error: Quaternion norm is " << q_norm << " on CSV row "
                << (plan.size() + 2) << std::endl;
      return -1;
    }
    step.qw /= q_norm;
    step.qx /= q_norm;
    step.qy /= q_norm;
    step.qz /= q_norm;

    step.kx = clamp(row[7] * stiffness_scale, min_kx, max_trans_k);
    step.ky = clamp(row[11] * stiffness_scale, min_ky, max_trans_k);
    step.kz = clamp(row[15] * stiffness_scale, min_kz, max_trans_k);
    step.vx = 0.0;
    step.vy = 0.0;
    step.vz = 0.0;

    plan.push_back(step);
  }

  if (plan.empty()) {
    std::cerr << "Error: No trajectory rows loaded." << std::endl;
    return -1;
  }

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

  double observed_min_kx = plan[0].kx, observed_max_kx = plan[0].kx;
  double observed_min_ky = plan[0].ky, observed_max_ky = plan[0].ky;
  double observed_min_kz = plan[0].kz, observed_max_kz = plan[0].kz;
  for (const auto& step : plan) {
    observed_min_kx = std::min(observed_min_kx, step.kx);
    observed_max_kx = std::max(observed_max_kx, step.kx);
    observed_min_ky = std::min(observed_min_ky, step.ky);
    observed_max_ky = std::max(observed_max_ky, step.ky);
    observed_min_kz = std::min(observed_min_kz, step.kz);
    observed_max_kz = std::max(observed_max_kz, step.kz);
  }

  std::cout << "   Loaded " << plan.size() << " control steps." << std::endl;
  std::cout << "   Kx range: " << observed_min_kx << " to " << observed_max_kx
            << std::endl;
  std::cout << "   Ky range: " << observed_min_ky << " to " << observed_max_ky
            << std::endl;
  std::cout << "   Kz range: " << observed_min_kz << " to " << observed_max_kz
            << std::endl;

  const int hold_steps = 2000;
  std::vector<LogEntry> actual_log(plan.size() + hold_steps);

  try {
    std::cout << "2. Connecting to Franka hardware..." << std::endl;
    franka::Robot robot(robot_ip);
    robot.automaticErrorRecovery();
    franka::Model model = robot.loadModel();

    std::cout << "PRESS ENTER TO EXECUTE 1000HZ CSV-DIAGONAL IMPEDANCE LOOP";
    std::cin.ignore();

    franka::RobotState initial_state = robot.readOnce();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d initial_pos = initial_transform.translation();
    Eigen::Quaterniond initial_ori(initial_transform.rotation());

    int current_step = 0;
    const int max_step = static_cast<int>(plan.size()) - 1;
    const int finish_step = max_step + hold_steps;

    auto impedance_control_callback =
        [&](const franka::RobotState& robot_state, franka::Duration period)
        -> franka::Torques {
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
        target_vel = alpha * target_vel_csv + (target_pos_csv - initial_pos) / 0.5;
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

      const double ramp = std::min(1.0, static_cast<double>(current_step) / 2000.0);
      const double ramped_scale = 0.4 + 0.6 * ramp;

      const double kx = target.kx * ramped_scale;
      const double ky = target.ky * ramped_scale;
      const double kz = target.kz * ramped_scale;
      const double dx = damping_scale * 2.0 * std::sqrt(kx);
      const double dy = damping_scale * 2.0 * std::sqrt(ky);
      const double dz = damping_scale * 2.0 * std::sqrt(kz);

      Eigen::Matrix<double, 6, 6> stiffness;
      Eigen::Matrix<double, 6, 6> damping;
      stiffness.setZero();
      damping.setZero();
      stiffness.diagonal() << kx, ky, kz, k_rot, k_rot, k_rot;
      damping.diagonal() << dx, dy, dz, d_rot, d_rot, d_rot;

      Eigen::Matrix<double, 6, 1> error;
      error.head(3) = error_pos;
      error.tail(3) = error_ori_vec;

      Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;
      Eigen::Matrix<double, 6, 1> desired_velocity;
      desired_velocity.setZero();
      desired_velocity.head(3) = target_vel;

      Eigen::Matrix<double, 6, 1> force_ext =
          stiffness * error - damping * (velocity - desired_velocity);
      Eigen::Matrix<double, 7, 1> tau_d = jacobian.transpose() * force_ext + coriolis;

      const double kDeltaTauMax = 0.99;
      std::array<double, 7> tau_d_rate_limited;
      for (size_t i = 0; i < 7; i++) {
        const double difference = tau_d[i] - robot_state.tau_J_d[i];
        tau_d_rate_limited[i] =
            robot_state.tau_J_d[i] + clamp(difference, -kDeltaTauMax, kDeltaTauMax);
      }

      if (current_step < static_cast<int>(actual_log.size())) {
        actual_log[current_step] = {target_pos.x(), target_pos.y(), target_pos.z(),
                                    position.x(), position.y(), position.z(),
                                    error_pos.x(), error_pos.y(), error_pos.z(),
                                    force_ext[0], force_ext[1], force_ext[2],
                                    orientation.w(), orientation.x(), orientation.y(),
                                    orientation.z()};
      }

      if (current_step >= finish_step) {
        return franka::MotionFinished(franka::Torques(tau_d_rate_limited));
      }
      current_step++;
      return tau_d_rate_limited;
    };

    robot.control(impedance_control_callback);
    std::cout << "Hardware execution complete." << std::endl;

    std::cout << "3. Saving detailed log to execution_log_csv_diag.csv..." << std::endl;
    std::ofstream log_file("execution_log_csv_diag.csv");
    log_file << "target_x,target_y,target_z,actual_x,actual_y,actual_z,"
             << "error_x,error_y,error_z,force_x,force_y,force_z,qw,qx,qy,qz\n";
    for (const auto& entry : actual_log) {
      log_file << entry.target_x << "," << entry.target_y << "," << entry.target_z
               << "," << entry.actual_x << "," << entry.actual_y << ","
               << entry.actual_z << "," << entry.error_x << "," << entry.error_y
               << "," << entry.error_z << "," << entry.force_x << ","
               << entry.force_y << "," << entry.force_z << "," << entry.qw << ","
               << entry.qx << "," << entry.qy << "," << entry.qz << "\n";
    }
    log_file.close();
    std::cout << "Log saved successfully." << std::endl;

  } catch (const franka::Exception& e) {
    std::cout << "Hardware Error: " << e.what() << std::endl;
  }

  return 0;
}
