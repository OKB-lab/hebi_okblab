#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <Eigen/Dense>

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;

class Node_Class : public rclcpp::Node{
    public:
        // コンストラクタ
        Node_Class() : Node("mecanum_velocity")
        {
            // Publisherの作成
            pub_wheel_vel_1_ = this->create_publisher<std_msgs::msg::Float64>(
                "w1/cmd_velocity", 10
            );
            pub_wheel_vel_2_ = this->create_publisher<std_msgs::msg::Float64>(
                "w2/cmd_velocity", 10
            );
            pub_wheel_vel_3_ = this->create_publisher<std_msgs::msg::Float64>(
                "w3/cmd_velocity", 10
            );
            pub_wheel_vel_4_ = this->create_publisher<std_msgs::msg::Float64>(
                "w4/cmd_velocity", 10
            );

            // サブスクライバの作成
            sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10,
                std::bind(&Node_Class::sub_callback_cmd_vel,this, std::placeholders::_1)
            );

            // Publish timer
            timer_pub_wheel_vel_com_ = this->create_wall_timer(
                100ms, std::bind(&Node_Class::timer_callback_pub_wheel_vel_com, this)
            );

            // パラメータの宣言
            this->declare_parameter("mecanum_width", 0.3);
            this->declare_parameter("mecanum_length", 0.4);
            this->declare_parameter("wheel_radius", 0.05);
            this->declare_parameter("wheel_rotation_direction_1", 1);
            this->declare_parameter("wheel_rotation_direction_2", 1);
            this->declare_parameter("wheel_rotation_direction_3", 1);
            this->declare_parameter("wheel_rotation_direction_4", 1);

            // パラメータの取得
            this->get_parameter("mecanum_width", mecanum_width);
            this->get_parameter("mecanum_length", mecanum_length);
            this->get_parameter("wheel_radius", wheel_radius);
            this->get_parameter("wheel_rotation_direction_1", wheel_rotation_direction_1);
            this->get_parameter("wheel_rotation_direction_2", wheel_rotation_direction_2);
            this->get_parameter("wheel_rotation_direction_3", wheel_rotation_direction_3);
            this->get_parameter("wheel_rotation_direction_4", wheel_rotation_direction_4);
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wheel_vel_1_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wheel_vel_2_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wheel_vel_3_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wheel_vel_4_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
        rclcpp::TimerBase::SharedPtr timer_pub_wheel_vel_com_;
        std_msgs::msg::Float64 wheel_vel_1, wheel_vel_2, wheel_vel_3, wheel_vel_4;
        double mecanum_width, mecanum_length, wheel_radius;
        int wheel_rotation_direction_1, wheel_rotation_direction_2, wheel_rotation_direction_3, wheel_rotation_direction_4;
        Eigen::Vector4d wheel_vel_vector;
        Eigen::Vector3d robot_vel_vector;
        Eigen::MatrixXd Jwr;

        // Callback function of subscriber
        void sub_callback_cmd_vel(const geometry_msgs::msg::Twist msg){
            robot_vel_vector(0) = msg.linear.x;
            robot_vel_vector(1) = msg.linear.y;
            robot_vel_vector(2) = msg.angular.z;

            Jwr = Eigen::MatrixXd::Zero(4,3);
            Jwr(0,0) = 1;
            Jwr(0,1) = -1;
            Jwr(0,2) = (mecanum_width + mecanum_length)/2;

            Jwr(1,0) = 1;
            Jwr(1,1) = 1;
            Jwr(1,2) = -(mecanum_width + mecanum_length)/2;

            Jwr(2,0) = 1;
            Jwr(2,1) = 1;
            Jwr(2,2) = (mecanum_width + mecanum_length)/2;

            Jwr(3,0) = 1;
            Jwr(3,1) = -1;
            Jwr(3,2) = -(mecanum_width + mecanum_length)/2;

            wheel_vel_vector = Jwr * robot_vel_vector;

            wheel_vel_1.data = wheel_rotation_direction_1 / wheel_radius * wheel_vel_vector(0);
            wheel_vel_2.data = wheel_rotation_direction_2 / wheel_radius * wheel_vel_vector(1);
            wheel_vel_3.data = wheel_rotation_direction_3 / wheel_radius * wheel_vel_vector(2);
            wheel_vel_4.data = wheel_rotation_direction_4 / wheel_radius * wheel_vel_vector(3);
        }

        // Timer of publishe
        void timer_callback_pub_wheel_vel_com(){
            pub_wheel_vel_1_->publish(wheel_vel_1);
            pub_wheel_vel_2_->publish(wheel_vel_2);
            pub_wheel_vel_3_->publish(wheel_vel_3);
            pub_wheel_vel_4_->publish(wheel_vel_4);
        }
};

int main(int argc, char **argv){
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // Nodeを作成
    auto node = std::make_shared<Node_Class>();

    // Nodeをspinする
    rclcpp::spin(node);

    // 終了処理
    rclcpp::shutdown();
    return 0;
}