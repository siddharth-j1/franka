#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <Eigen/Dense>

// Struct for the planned trajectory from Python
struct TrajectoryStep {
    double x, y, z;
    double qw, qx, qy, qz;
    Eigen::Matrix<double, 6, 6> K;
    Eigen::Matrix<double, 6, 6> D;
};

// Struct to save the actual hardware data
struct LogEntry {
    double x, y, z, qw, qx, qy, qz;
};

int main(int argc, char** argv) {
    // Dynamically load whichever CSV you pass in the terminal
    std::string csv_file = (argc > 1) ? argv[1] : "quintic_trajectory.csv";
    std::string robot_ip = "192.168.1.12";
    std::vector<TrajectoryStep> plan;

    std::cout << "1. Loading 6D Trajectory: " << csv_file << std::endl;
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << " Error: Could not open " << csv_file << std::endl;
        return -1;
    }

    std::string line;
    std::getline(file, line); // Skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        TrajectoryStep step;
        std::vector<double> row;
        while (std::getline(ss, val, ',')) { row.push_back(std::stod(val)); }
        
        step.x = row[0]; step.y = row[1]; step.z = row[2];
        step.qw = row[3]; step.qx = row[4]; step.qy = row[5]; step.qz = row[6];
        
        step.K.setZero();
        step.K.diagonal() << row[7], row[8], row[9], row[10], row[11], row[12];
        
        step.D.setZero();
        step.D.diagonal() << row[13], row[14], row[15], row[16], row[17], row[18];
        
        plan.push_back(step);
    }
    std::cout << "   -> Loaded " << plan.size() << " milliseconds of 6D data." << std::endl;

    std::vector<LogEntry> actual_log(plan.size());

    try {
        std::cout << "2. Connecting to Franka hardware..." << std::endl;
        franka::Robot robot(robot_ip);
        robot.automaticErrorRecovery();
        franka::Model model = robot.loadModel();

        std::cout << " PRESS ENTER TO EXECUTE 1000HZ LOOP ";
        std::cin.ignore();

        // CAPTURE INITIAL STATE RIGHT AT START for the Soft-Start
        franka::RobotState initial_state = robot.readOnce();
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d initial_pos = initial_transform.translation();
        Eigen::Quaterniond initial_ori(initial_transform.rotation());

        int current_step = 0;
        int max_step = plan.size() - 1;

        auto impedance_control_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
            
            // 1. Get current physical state
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position = transform.translation();
            Eigen::Quaterniond orientation(transform.rotation());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
            
            std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

            // 2. Save hardware data to log
            actual_log[current_step] = {position.x(), position.y(), position.z(), 
                                        orientation.w(), orientation.x(), orientation.y(), orientation.z()};

            // 3. Get CSV Target and apply 500ms Soft-Start blend
            TrajectoryStep target = plan[current_step];
            Eigen::Vector3d target_pos_csv(target.x, target.y, target.z);
            Eigen::Quaterniond target_ori_csv(target.qw, target.qx, target.qy, target.qz);

            double alpha = std::min(1.0, (double)current_step / 500.0);
            Eigen::Vector3d target_pos = (1.0 - alpha) * initial_pos + alpha * target_pos_csv;
            Eigen::Quaterniond target_ori = initial_ori.slerp(alpha, target_ori_csv);

            // 4. Calculate 6D Error
            Eigen::Vector3d error_pos = target_pos - position;

            if (target_ori.coeffs().dot(orientation.coeffs()) < 0.0) { 
                orientation.coeffs() << -orientation.coeffs(); 
            }
            Eigen::Quaterniond error_quaternion(orientation.inverse() * target_ori);
            Eigen::Vector3d error_ori_vec;
            error_ori_vec << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            error_ori_vec = transform.rotation() * error_ori_vec;

            Eigen::Matrix<double, 6, 1> error;
            error.head(3) << error_pos;
            error.tail(3) << error_ori_vec;

            // 5. Apply Impedance Law
            Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;
            Eigen::Matrix<double, 6, 1> F_ext = target.K * error - target.D * velocity;

            Eigen::VectorXd tau_d = jacobian.transpose() * F_ext + coriolis;
            
            std::array<double, 7> tau_d_array;
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

            if (current_step >= max_step) { return franka::MotionFinished(franka::Torques(tau_d_array)); }
            current_step++;
            return tau_d_array;
        };

        robot.control(impedance_control_callback);
        std::cout << " Hardware Execution Complete!" << std::endl;

        // 6. Write Log to CSV
        std::cout << "3. Saving hardware log to execution_log.csv..." << std::endl;
        std::ofstream log_file("execution_log.csv");
        log_file << "x,y,z,qw,qx,qy,qz\n";
        for (const auto& entry : actual_log) {
            log_file << entry.x << "," << entry.y << "," << entry.z << "," 
                     << entry.qw << "," << entry.qx << "," << entry.qy << "," << entry.qz << "\n";
        }
        log_file.close();
        std::cout << "Log saved successfully." << std::endl;

    } catch (const franka::Exception& e) {
        std::cout << " Hardware Error: " << e.what() << std::endl;
    }
    return 0;
}