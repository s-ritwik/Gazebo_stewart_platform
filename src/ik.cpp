#include <cmath>
#include "eigen3/Eigen/Core"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

class IK
{
public:
    IK(int argc, char **argv)
    {
        ros::init(argc, argv, "ik");
        ros::NodeHandle nh;

        // Define leg length limits
        L_min = 0.1;      // Minimum leg length (from SDF)
        L_max = 1.4588;   // Maximum leg length (from SDF)
        L_neutral = (L_min + L_max) / 2.0;  // Desired leg length at neutral position

        // Attachment points position in the base platform (from SDF)
        b.resize(6, 4);
        b << 1.5588457268119897, 0.8999999999999999, 0.25, 1,
             1.1021821192326179e-16, 1.8, 0.25, 1,
             -1.5588457268119897, 0.8999999999999999, 0.25, 1,
             -1.5588457268119895, -0.9000000000000002, 0.25, 1,
             -3.3065463576978537e-16, -1.8, 0.25, 1,
             1.558845726811989, -0.9000000000000008, 0.25, 1;

        // Attachment points position in the moving platform (from SDF)
        p.resize(6, 4);
        p << 0.9545941546018393, 0.9545941546018392, -0.05, 1,
             0.34940571088840333, 1.303999865490242, -0.05, 1,
             -1.303999865490242, 0.3494057108884034, -0.05, 1,
             -1.3039998654902425, -0.3494057108884025, -0.05, 1,
             0.34940571088840355, -1.303999865490242, -0.05, 1,
             0.9545941546018399, -0.9545941546018385, -0.05, 1;

        // Compute the height offset
        height_offset = computeHeightOffset();

        // Initialize the message
        for (int i = 0; i < 6; i++)
        {
            f32ma_msg.data.push_back(0);
        }

        pub = nh.advertise<std_msgs::Float32MultiArray>("/stewart/legs_position_cmd", 100);
        sub = nh.subscribe("stewart/platform_pose", 100, &IK::callback, this);

        ROS_INFO("IK node initialized with height offset: %f", height_offset);
    }

    void run()
    {
        ros::spin();
    }

private:
    void callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        float x = msg->linear.x;
        float y = msg->linear.y;
        float z = msg->linear.z + height_offset;  // Apply height offset
        float roll = msg->angular.x;
        float pitch = msg->angular.y;
        float yaw = msg->angular.z;

        // Build the transformation matrix
        Eigen::Matrix<float, 4, 4> T = transformation_matrix(x, y, z, roll, pitch, yaw);

        for (size_t i = 0; i < 6; i++)
        {
            Eigen::Vector4f transformed_p = T * p.row(i).transpose();
            Eigen::Vector4f delta = transformed_p - b.row(i).transpose();
            float leg_length = delta.head<3>().norm();
            f32ma_msg.data[i] = leg_length;

            // Log the leg lengths
            // ROS_INFO("Leg %lu length: %f", i + 1, f32ma_msg.data[i]);

            // Check if leg length is within limits
            if (leg_length < L_min || leg_length > L_max)
            {
                ROS_WARN("Leg %lu length out of bounds: %f", i + 1, leg_length);
                // Optionally, clamp leg lengths to joint limits
                f32ma_msg.data[i] = std::min(std::max(leg_length, L_min), L_max);
            }
        }
        pub.publish(f32ma_msg);
    }

    Eigen::Matrix<float, 4, 4> transformation_matrix(float x, float y, float z, float roll, float pitch, float yaw)
    {
        float c_y = cos(yaw);
        float s_y = sin(yaw);
        float c_p = cos(pitch);
        float s_p = sin(pitch);
        float c_r = cos(roll);
        float s_r = sin(roll);

        Eigen::Matrix<float, 4, 4> T;
        T << c_y * c_p, c_y * s_p * s_r - s_y * c_r, c_y * s_p * c_r + s_y * s_r, x,
             s_y * c_p, s_y * s_p * s_r + c_y * c_r, s_y * s_p * c_r - c_y * s_r, y,
               -s_p,          c_p * s_r,              c_p * c_r,                   z,
                0,                 0,                      0,                      1;
        return T;
    }

    float computeHeightOffset()
    {
        // Compute average z positions of attachment points in base and platform
        float z_b_avg = b.col(2).mean();
        float z_p_avg = p.col(2).mean();

        // Initial vertical distance between base and platform attachment points
        float delta_z_initial = z_p_avg - z_b_avg;

        // Desired vertical distance to achieve L_neutral
        // For each leg, compute the required delta_z to achieve L_neutral
        // Assuming platform is at zero rotation and x = y = 0

        // Since the legs are not vertical, need to account for horizontal distances
        // Compute the average horizontal distance between corresponding attachment points
        float horizontal_distances[6];
        for (int i = 0; i < 6; i++)
        {
            float dx = p(i, 0) - b(i, 0);
            float dy = p(i, 1) - b(i, 1);
            horizontal_distances[i] = sqrt(dx * dx + dy * dy);
        }

        float avg_horizontal_distance = 0;
        for (int i = 0; i < 6; i++)
        {
            avg_horizontal_distance += horizontal_distances[i];
        }
        avg_horizontal_distance /= 6.0;

        // Compute the required vertical distance using the Pythagorean theorem
        float delta_z_required = sqrt(L_neutral * L_neutral - avg_horizontal_distance * avg_horizontal_distance);

        // Compute the height offset needed
        float height_offset = delta_z_required - delta_z_initial;

        ROS_INFO("Computed height offset: %f", height_offset);
        return height_offset;
    }

    float L_min, L_max, L_neutral;
    float height_offset;

    Eigen::Matrix<float, 6, 4> b, p;

    ros::Publisher pub;
    ros::Subscriber sub;
    std_msgs::Float32MultiArray f32ma_msg;
};

int main(int argc, char **argv)
{
    IK ik(argc, argv);
    ik.run();

    return 0;
}
