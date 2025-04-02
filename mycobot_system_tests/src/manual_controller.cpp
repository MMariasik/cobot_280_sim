#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <thread>

using namespace std::chrono_literals;

class ArmGripperController : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GripperCommand = control_msgs::action::GripperCommand;

    ArmGripperController() : Node("arm_gripper_controller"), t1{std::bind(&ArmGripperController::executeSequence, this)}
    {
        // Create action clients
        arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_controller/follow_joint_trajectory");
        gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "/gripper_action_controller/gripper_cmd");

        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ArmGripperController::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for action servers...");
        if (!arm_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Arm action server not available!");
            return;
        }
        if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Gripper action server not available!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Action servers connected!");
    }
    
    ~ArmGripperController()
    {
        stop_thread = true; // Signal the thread to stop
        pthread_cancel(t1.native_handle());

        if (t1.joinable()) {
            t1.join(); // Wait for the thread to finish
        }

        RCLCPP_INFO(this->get_logger(), "Thread stopped and joined.");
    }

    void moveArmToTarget(const std::vector<double>& target_positions, double duration)
    {
        if (joint_positions_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No joint states received yet!");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names.push_back(joint_names_[6]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[4]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[7]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[0]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[1]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[2]);

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_positions;
        point.time_from_start = rclcpp::Duration::from_seconds(duration);

        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        arm_client_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(this->get_logger(), "Moving arm to target position...");
        std::this_thread::sleep_for(std::chrono::seconds((int)duration)); // Wait for arm to move
    }

    void moveJointToTarget(const double pos, size_t joint, double duration)
    {
        if (joint_positions_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No joint states received yet!");
            return;
        }
        if(joint > 5)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid joint");
            return;
        }
        if((joint < 6) && (abs(pos) > 2.879793)){
            RCLCPP_WARN(this->get_logger(), "Invalid position for this joint correct: -2.879793 to 2.879793");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names.push_back(joint_names_[6]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[4]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[7]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[0]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[1]);
        goal_msg.trajectory.joint_names.push_back(joint_names_[2]);

        // set positions to current ones
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.push_back(joint_positions_[6]);
        point.positions.push_back(joint_positions_[4]);
        point.positions.push_back(joint_positions_[7]);
        point.positions.push_back(joint_positions_[0]);
        point.positions.push_back(joint_positions_[1]);
        point.positions.push_back(joint_positions_[2]);

        // update single joint
        point.positions.at(joint) = pos;

        point.time_from_start = rclcpp::Duration::from_seconds(duration);

        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        arm_client_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(this->get_logger(), "Moving arm to target position...");
        std::this_thread::sleep_for(std::chrono::seconds((int)duration)); // Wait for arm to move
    }

    void moveGripper(double position, double max_effort)
    {
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = max_effort;

        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        gripper_client_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(this->get_logger(), "Moving gripper to position %f", position);
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait for gripper action
    }

    void executeSequence()
    {
        while (!stop_thread)  // Keep running until stop_thread becomes true
        {
            if (joint_positions_.empty()) continue;
            
            std::cout << "choose option A/G/J/W or H for help" << std::endl;
            char c;
            std::cin >> c;

            switch (c)
            {
            case 'A':
            case 'a':
            {
                std::cout << "move Arm to desired position" << std::endl;

                std::vector<std::vector<double>> vectorOfPositions;
                vectorOfPositions.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); // Example position - home position
                vectorOfPositions.push_back({1.0, -1.2, 0.5, -0.3, 0.6, -1.0}); // Example position

                size_t whichPos;
                do{
                    std::cout << "Which position to move 0-" << vectorOfPositions.size()-1 << " ( 0=home )" << std::endl;
                    std::cin >> whichPos;
                }while(whichPos >= vectorOfPositions.size());
                // Step 1: Move the arm to a target position
                moveArmToTarget(vectorOfPositions.at(whichPos), 3.0);
                break;
            }

            case 'G':
            case 'g':
            {
                std::cout << "move Gripper to desired position" << std::endl;

                double pos;
                do{
                std::cout << "give position for gripper from -0.7 to 0.15" << std::endl;
                std::cin >> pos;
                }while(pos < -0.7 || pos > 0.15);
                moveGripper(pos, 5.0);
                break;
            }

            case 'J':
            case 'j':
            {
                std::cout << "move Joint to desired position" << std::endl;

                double pos;
                size_t joint;

                do{
                    std::cout << "Which joint to change 0-5" << std::endl;
                    std::cin >> joint;
                }while(joint > 5);

                std::cout << "give position for joint from -2.879793 to 2.879793 for 0-6 joints" << std::endl;
                std::cin >> pos;

                moveJointToTarget(pos, joint, 3.0);
                break;
            }

            case 'W':
            case 'w':
            {
                std::cout << "Write joints" << std::endl;
                for(size_t i =0; i < joint_names_.size(); i++)
                {
                    RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", 
                    joint_names_[i].c_str(), joint_positions_[i]);
                }
                break;
            }
            
            case 'H':
            case 'h':
            {
                std::cout << "A - move Arm to desired position" << std::endl;
                std::cout << "G - move Gripper to desired position" << std::endl;
                std::cout << "J - move Joint to desired position" << std::endl;
                std::cout << "W - Write joints" << std::endl;
                break;
            }

            default:
                break;
            }
            
        }
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_names_ = msg->name;
        joint_positions_ = msg->position;
    }

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::thread t1;
    std::atomic<bool> stop_thread;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmGripperController>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
