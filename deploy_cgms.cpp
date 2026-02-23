// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <string>
// #include <sstream>

// #include <franka/robot.h>
// #include <franka/model.h>
// #include <franka/exception.h>  // <--- ADD THIS EXACT LINE
// #include <Eigen/Dense>

// // Struct to hold exactly one millisecond of AI data
// struct TrajectoryStep {
//     double dx, dy, dz;
//     Eigen::Matrix3d K;
//     Eigen::Matrix3d D;
// };

// int main() {
//     std::string robot_ip = "192.168.1.15";
//     std::vector<TrajectoryStep> plan;

//     std::cout << "1. Loading AI Trajectory into RAM..." << std::endl;
//     std::ifstream file("real_trajectory.csv");
//     std::string line;
//     std::getline(file, line); // Skip header

//     while (std::getline(file, line)) {
//         std::stringstream ss(line);
//         std::string val;
//         TrajectoryStep step;
//         std::vector<double> row;
        
//         while (std::getline(ss, val, ',')) { row.push_back(std::stod(val)); }
        
//         step.dx = row[0]; step.dy = row[1]; step.dz = row[2];
        
//         // Load the 3x3 matrices from the CSV columns
//         step.K << row[3], row[4], row[5],
//                   row[6], row[7], row[8],
//                   row[9], row[10],row[11];
                  
//         step.D << row[12], row[13], row[14],
//                   row[15], row[16], row[17],
//                   row[18], row[19], row[20];
                  
//         plan.push_back(step);
//     }
//     std::cout << "   -> Loaded " << plan.size() << " milliseconds of dynamic impedance data." << std::endl;

//     try {
//         std::cout << "2. Connecting to Franka hardware..." << std::endl;
//         franka::Robot robot(robot_ip);
//         robot.automaticErrorRecovery();
//         franka::Model model = robot.loadModel();

//         // Lock in the starting position
//         franka::RobotState initial_state = robot.readOnce();
//         Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
//         Eigen::Vector3d initial_position = initial_transform.translation();
//         Eigen::Quaterniond initial_orientation(initial_transform.rotation());

//         std::cout << "   -> Target: Move 5cm in X using strictly C++ Real-Time torques." << std::endl;
//         std::cout << "PRESS ENTER TO EXECUTE 1000HZ LOOP ";
//         std::cin.ignore();

//         int current_step = 0;
//         int max_step = plan.size() - 1;

//         // ===================================================================
//         // THE 1000 HZ REAL-TIME LOOP
//         // ===================================================================
//         auto impedance_control_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
            
//             // 1. Get the AI data for this exact millisecond
//             if (current_step < max_step) { current_step++; }
//             TrajectoryStep target = plan[current_step];

//             // 2. Read Robot's Current State
//             std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
//             Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
//             Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
//             // Save the array first so it doesn't get deleted!
//             std::array<double, 7> coriolis_array = model.coriolis(robot_state);
//             Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

//             Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
//             Eigen::Vector3d position = transform.translation();
//             Eigen::Quaterniond orientation(transform.rotation());

//             // 3. Compute Positional Error
//             Eigen::Vector3d target_position = initial_position + Eigen::Vector3d(target.dx, target.dy, target.dz);
//             Eigen::Vector3d error_pos = target_position - position;

//             // 4. Compute Orientational Error (Keeps the gripper perfectly level)
//             Eigen::Vector3d error_ori;
//             if (initial_orientation.coeffs().dot(orientation.coeffs()) < 0.0) { orientation.coeffs() << -orientation.coeffs(); }
//             Eigen::Quaterniond error_quaternion(orientation.inverse() * initial_orientation);
//             error_ori << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
//             error_ori = transform.rotation() * error_ori;

//             // 5. Apply the AI's Dynamic Stiffness Matrix!
//             Eigen::Matrix<double, 6, 1> F_ext;
            
//             // Apply the 3x3 translational matrix directly from your .csv!
//             Eigen::Vector3d F_translation = target.K * error_pos - target.D * (jacobian.topRows(3) * dq);
            
//             // Apply a fixed rotational stiffness (50 Nm/rad) to keep it stable
//             Eigen::Vector3d F_rotation = 50.0 * error_ori - 5.0 * (jacobian.bottomRows(3) * dq);
            
//             F_ext.head(3) << F_translation;
//             F_ext.tail(3) << F_rotation;

//             // 6. Convert to Joint Torques
//             Eigen::VectorXd tau_d = jacobian.transpose() * F_ext + coriolis;

//             // 7. Send to motors safely
//             std::array<double, 7> tau_d_array;
//             Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
            
//             // Stop the loop if we hit the end of the trajectory
//             if (current_step >= max_step) { return franka::MotionFinished(franka::Torques(tau_d_array)); }
//             return tau_d_array;
//         };

//         robot.control(impedance_control_callback);
//         std::cout << " Hardware Execution Complete! Flawless 1000Hz stream." << std::endl;

//     } catch (const franka::Exception& e) {
//         std::cout << " Hardware Error: " << e.what() << std::endl;
//     }
//     return 0;
// }

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <Eigen/Dense>

struct TrajectoryStep {
    double dx, dy, dz;
    Eigen::Matrix3d K;
    Eigen::Matrix3d D;
};

int main() {
    std::string robot_ip = "192.168.1.15";
    std::vector<TrajectoryStep> plan;

    std::cout << "1. Loading AI Trajectory into RAM..." << std::endl;
    std::ifstream file("real_trajectory.csv");
    std::string line;
    std::getline(file, line); 

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        TrajectoryStep step;
        std::vector<double> row;
        while (std::getline(ss, val, ',')) { row.push_back(std::stod(val)); }
        
        step.dx = row[0]; step.dy = row[1]; step.dz = row[2];
        step.K << row[3], row[4], row[5], row[6], row[7], row[8], row[9], row[10],row[11];
        step.D << row[12], row[13], row[14], row[15], row[16], row[17], row[18], row[19], row[20];
        plan.push_back(step);
    }
    std::cout << "   -> Loaded " << plan.size() << " milliseconds." << std::endl;

    //  NEW: Allocate RAM to store the robot's actual physical position!
    std::vector<Eigen::Vector3d> actual_positions(plan.size());

    try {
        std::cout << "2. Connecting to Franka hardware..." << std::endl;
        franka::Robot robot(robot_ip);
        robot.automaticErrorRecovery();
        franka::Model model = robot.loadModel();

        franka::RobotState initial_state = robot.readOnce();
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d initial_position = initial_transform.translation();
        Eigen::Quaterniond initial_orientation(initial_transform.rotation());

        std::cout << "⚠️ PRESS ENTER TO EXECUTE 1000HZ LOOP ⚠️";
        std::cin.ignore();

        int current_step = 0;
        int max_step = plan.size() - 1;

        auto impedance_control_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
            if (current_step < max_step) { current_step++; }
            TrajectoryStep target = plan[current_step];

            std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
            
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position = transform.translation();
            Eigen::Quaterniond orientation(transform.rotation());

            //  NEW: Save the real physical position into RAM for this millisecond!
            actual_positions[current_step] = position;

            Eigen::Vector3d target_position = initial_position + Eigen::Vector3d(target.dx, target.dy, target.dz);
            Eigen::Vector3d error_pos = target_position - position;

            Eigen::Vector3d error_ori;
            if (initial_orientation.coeffs().dot(orientation.coeffs()) < 0.0) { orientation.coeffs() << -orientation.coeffs(); }
            Eigen::Quaterniond error_quaternion(orientation.inverse() * initial_orientation);
            error_ori << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            error_ori = transform.rotation() * error_ori;

            Eigen::Matrix<double, 6, 1> F_ext;
            Eigen::Vector3d F_translation = target.K * error_pos - target.D * (jacobian.topRows(3) * dq);
            Eigen::Vector3d F_rotation = 50.0 * error_ori - 5.0 * (jacobian.bottomRows(3) * dq);
            
            F_ext.head(3) << F_translation;
            F_ext.tail(3) << F_rotation;

            Eigen::VectorXd tau_d = jacobian.transpose() * F_ext + coriolis;
            std::array<double, 7> tau_d_array;
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
            
            if (current_step >= max_step) { return franka::MotionFinished(franka::Torques(tau_d_array)); }
            return tau_d_array;
        };

        robot.control(impedance_control_callback);
        std::cout << "Hardware Execution Complete!" << std::endl;

        //  NEW: The robot has stopped. Now it is safe to write the RAM data to a file!
        std::cout << "3. Saving physical execution data to 'execution_log.csv'..." << std::endl;
        std::ofstream log_file("execution_log.csv");
        log_file << "real_x,real_y,real_z\n";
        for (int i = 0; i <= max_step; ++i) {
            log_file << actual_positions[i].x() << "," << actual_positions[i].y() << "," << actual_positions[i].z() << "\n";
        }
        log_file.close();
        std::cout << "   -> Saved successfully!" << std::endl;

    } catch (const franka::Exception& e) {
        std::cout << " Hardware Error: " << e.what() << std::endl;
    }
    return 0;
}