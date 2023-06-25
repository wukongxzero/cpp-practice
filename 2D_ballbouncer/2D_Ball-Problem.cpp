#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

struct Joint {
    int type;     // 0 for revolute and 1 for end
    double theta; // angle between consecutive links
    double d;     // link offset
    double a;     // distance of the links
    double alpha; // twist of the links
};

// Function to read the configuration values from a file
bool readConfigFile(const std::string& filename, double& d1, double& d2, double& d3, double& d4, double& ball_x, double& ball_y, double& ball_theta) {
    std::ifstream configFile(filename);
    if (!configFile.is_open()) {
        std::cout << "Failed to open the config file!" << std::endl;
        return false;
    }

    configFile >> d1 >> d2 >> d3 >> d4 >> ball_x >> ball_y >> ball_theta;
    configFile.close();
    return true;
}

// Function to calculate the end effector positions given the configuration values
void solveForEndEffector(double d1, double d2, double d3, double ball_x, double ball_y, double d4, double ball_theta, double& e1_x, double& e1_y, double& e2_x, double& e2_y) {
    if (ball_theta >= 0 && ball_theta < 90) {
        // Calculate angles in radians
        double e1_angle = (90.0 - ball_theta) / 2.0;
        double e2_angle = (90.0 - ball_theta) / 2.0;
        double e1_theta = e1_angle * M_PI / 180.0;
        double e2_theta = e2_angle * M_PI / 180.0;

        // Calculate end effector positions
        e1_x = -d4 / 2.0 + d4 / 2.0 * std::sin(e1_theta);
        e1_y = sqrt(d1 * d1 + d2 * d2 - d1 * d2 * std::cos(M_PI * 2 / 3)) - d4 / 2.0 * std::sin(e1_theta);

        e2_x = d4 / 2.0 - d4 / 2.0 * std::sin(e1_theta);
        e2_y = sqrt(d1 * d1 + d2 * d2 - d1 * d2 * std::cos(M_PI * 4 / 3)) + d4 / 2.0 * std::sin(e2_theta);
    }

    if (ball_theta >= 90 && ball_theta <= 180) {
        // Calculate angles in radians
        double e1_angle = (ball_theta - 90) / 2.0;
        double e2_angle = (ball_theta - 90) / 2.0;
        double e1_theta = e1_angle * M_PI / 180.0;
        double e2_theta = e2_angle * M_PI / 180.0;

        // Calculate end effector positions
        e1_x = d4 / 2.0 - d4 / 2.0 * std::sin(e1_theta);
        e1_y = sqrt(d1 * d1 + d2 * d2 - d1 * d2 * std::cos(M_PI * 2 / 3)) + d4 / 2.0 * std::sin(e1_theta); // home position at e1_theta = 0

        e2_x = -d4 / 2.0 + d4 / 2.0 * std::sin(e1_theta);
        e2_y = sqrt(d1 * d1 + d2 * d2 - d1 * d2 * std::cos(M_PI * 4 / 3)) - d4 / 2.0 * std::sin(e2_theta); // home position at e1_theta = 0
    }
}

// Function to apply jerk motion to the ball
void createJerkMotion(double& ball_y, double& ball_theta) {
    // Check if the ball is within a specific range or meets the conditions for jerk motion
    if (ball_y <= 4.0 && ball_theta > 0) {
        // Apply jerk motion to increase ball_y and decrease ball_theta
        ball_y += 0.1;
        ball_theta -= -10;
    }
}


// Function to calculate inverse kinematics for a given end effector position
void inverseKinematics(double e_x, double e_y, const std::vector<Joint>& joints, std::vector<double>& joint_angles) {
    double a1 = joints[0].a;
    double a2 = joints[1].a;

    // Calculate theta1 and distance from origin to end effector
    double theta1 = atan2(e_y, e_x) * 180.0 / M_PI;
    double d = sqrt(e_x * e_x + e_y * e_y);

    // Calculate cos(theta2) using law of cosines
    double cosTheta2 = (d * d - a1 * a1 - a2 * a2) / (2 * a1 * a2);

    // Calculate sin(theta2) using the Pythagorean identity
    double sinTheta2 = sqrt(1 - cosTheta2 * cosTheta2);

    // Calculate theta2 in degrees
    double theta2 = atan2(sinTheta2, cosTheta2) * 180.0 / M_PI;

    // Store the joint angles in the provided vector
    joint_angles.clear();
    joint_angles.push_back(theta1);
    joint_angles.push_back(theta2);
}

int main() {
    // Read the configuration values from the config file
    double d1, d2, d3, d4, ball_x, ball_y, ball_theta;
    if (!readConfigFile("config.txt", d1, d2, d3, d4, ball_x, ball_y, ball_theta)) {
        return 1;
    }

    // Print the configuration values
    std::cout << "d1: " << d1 << std::endl; // lower arm
    std::cout << "d2: " << d2 << std::endl; // upper arm
    std::cout << "d3: " << d3 << std::endl; // distance between lower arms
    std::cout << "d4: " << d4 << std::endl; // platform length

    // Print the initial ball position and angle
    std::cout << "Ball position: (" << ball_x << ", " << ball_y << ")" << std::endl;
    std::cout << "Ball angle: " << ball_theta << std::endl;

    // Ball proximity condition
    while (ball_theta < 180) {
        // Calculate forward kinematics for end effector positions

        double angle = atan2((std::abs(d1 - d2) - sqrt(d1 * d1 + d2 * d2 - 2*d1 * d2 * std::cos(M_PI * 2 / 3))), d4/2)*180/M_PI;
        double angle1 = 2*abs(angle) - 90;
        double angle2 = 180 - angle1;
        
        double e1_x, e1_y, e2_x, e2_y;
        if (e1_y < d1 + d2 && e2_y < d1 + d2 && -d4 / 2 < ball_x && ball_x < d4 / 2 && std::abs(d1 - d2) <= ball_y && ball_y <= d1 + d2 && ball_theta >= angle1 && ball_theta< angle2) {
            solveForEndEffector(d1, d2, d3, ball_x, ball_y, d4, ball_theta, e1_x, e1_y, e2_x, e2_y);
        } else {
            std::cout << "Out of bounds!" << std::endl;
            break;
        }

        // Print the new end effector positions
        std::cout << "New End Effector E1 position: (" << e1_x << ", " << e1_y << ")" << std::endl;
        std::cout << "New End Effector E2 position: (" << e2_x << ", " << e2_y << ")" << std::endl;

        // Define the robot structure and parameters using DH parameters
        std::vector<Joint> joints = {
            {0, 77.837297, 0, d1, 0}, // joint 1
            {0, 71.9407695, 0, d2, 0}   // joint 2
        };

        std::vector<Joint> robot2_joints = {
            {0,102.162703, 0, d1, 0}, // joint 1
            {0, 71.9407695, 0, d2, 0}  // joint 2
        };

        std::vector<double> joint_angles;
        inverseKinematics(e1_x, e1_y, joints, joint_angles);
        
        if(ball_theta>=90){
        std::cout << "Inverse Kinematics for Robot1 /absolute value for change in angle - M11: " << joint_angles[0] - joints[0].theta << ", M12: " << joint_angles[1] - joints[1].theta << std::endl;
        }
        else {std::cout << "Inverse Kinematics for Robot1/absolute value for change in angle  - M11: " << joint_angles[0] - robot2_joints[0].theta << ", M12: " << joint_angles[1] - joints[1].theta << std::endl;
        }
    
        inverseKinematics(e2_x, e2_y, joints, joint_angles);
            if(ball_theta>=90){
        std::cout << "Inverse Kinematics for Robot2 /absolute value for change in angle - M21: " << joint_angles[0] - robot2_joints[0].theta << ", M22: " << joint_angles[1] - robot2_joints[1].theta << std::endl;
        }
        else{std::cout << "Inverse Kinematics for Robot2/absolute value for change in angle  - M21: " << joint_angles[0] - joints[0].theta << ", M22: " << +joint_angles[1] - robot2_joints[1].theta << std::endl;}
        // Apply jerk motion to the ball
        createJerkMotion(ball_y, ball_theta);

        // Print the updated ball position and angle after jerk motion
        std::cout << "\n\nCreated jerking motion: " << std::endl;
        std::cout << "Updated Ball position: (" << ball_x << ", " << ball_y << ")" << std::endl;
        std::cout << "Updated Ball angle: " << ball_theta << std::endl;
    }

    return 0;
}
