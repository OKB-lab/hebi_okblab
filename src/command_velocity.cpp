#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"

// 時間リテラルを使えるようにする
using namespace std::chrono_literals;
using namespace hebi;

class Node_Class : public rclcpp::Node{
    public:
        // コンストラクタ
        Node_Class() : Node("command_velocity"), cmd_velocity(1)
        {
            // サブスクライバの作成
            sub_cmd_ = this->create_subscription<std_msgs::msg::Float64>(
                "cmd_velocity", 10,
                std::bind(&Node_Class::subscribe_callback_cmd,this, std::placeholders::_1)
            );

            // Creage command timer
            timer_cmd_ = this->create_wall_timer(
                100ms, std::bind(&Node_Class::timer_cmd_callback, this)
            );

            // パラメータの宣言
            this->declare_parameter("name_family", "R8-16");
            this->declare_parameter("name_actuator", "R8-00552");
            // パラメータの取得
            this->get_parameter("name_family", family_name);
            this->get_parameter("name_actuator", actuator_name);

            // HEBIモジュールの検索
            group = lookup_.getGroupFromNames({family_name}, {actuator_name});
            if (!group) {
                std::cout
                << "Group not found! Check that the family and name of a module on the network" << std::endl
                << "matches what is given in the source file." << std::endl;
                // return 1;
            }
        }
    
    private:
        // メンバ変数の定義
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_cmd_;
        // std_msgs::msg::Float64 cmd_position;
        Lookup lookup_;
        std::shared_ptr<Group> group;
        rclcpp::TimerBase::SharedPtr timer_cmd_;
        Eigen::VectorXd cmd_velocity;
        std::string family_name, actuator_name;

        // Callback function of subscriber
        void subscribe_callback_cmd(const std_msgs::msg::Float64 msg){
            cmd_velocity[0] = msg.data;

        }

        // Timer of motion command
        void timer_cmd_callback(){
            GroupCommand group_command(group->size());
            group_command.setVelocity(cmd_velocity);
            group->sendCommand(group_command);
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