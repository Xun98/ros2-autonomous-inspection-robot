#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <queue>

class Nav2ActionClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
            this, "navigate_to_pose");
        load_goals();

        cur = 1;

        send_goal();
    }

    void send_goal()
    {

        if (!action_client_->wait_for_action_server(
                std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "NavigateToPose action server not available");

            return;
        }

        auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandle::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Goal rejected, retry after 1 second");

                retry_timer_ = this->create_wall_timer(
                    std::chrono::seconds(1),
                    [this]()
                    {
                        retry_timer_->cancel();
                        send_goal();
                    });

                return;
            }

            RCLCPP_INFO(this->get_logger(), "pub goal %d: { %.2f, %.2f }", cur, goals[cur].pose.pose.position.x, goals[cur].pose.pose.position.y);
    
        };

        send_goal_options.feedback_callback = [this](GoalHandle::SharedPtr,
                                                     const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
        {
            // RCLCPP_INFO(this->get_logger(), "Received feedback");
        };

        send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
            }
           
            cur = (cur + 1) % goals.size();
            send_goal();
        };

        goals[cur].pose.header.stamp = get_clock()->now();
        action_client_->async_send_goal(goals[cur], send_goal_options);
    }

private:

    void load_goals()
    {
        int goal_count = this->declare_parameter<int>("goal_count", 0);
        if(goal_count < 2)
        {
            throw std::runtime_error("goal only start");
        }

        for (int i = 0; i < goal_count; ++i)
        {
            std::string name = "goal_" + std::to_string(i);
            auto point = this->declare_parameter<std::vector<double>>(
                name, {});

            if (point.size() != 3)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Invalid parameter: %s", name.c_str());

                throw std::runtime_error("Invalid goal configuration");
            }

            auto goal_msg = NavigateToPoseAction::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = this->now();
            goal_msg.pose.pose.position.x = point[0];
            goal_msg.pose.pose.position.y = point[1];

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, point[3]);
            goal_msg.pose.pose.orientation.x = q.getX();
            goal_msg.pose.pose.orientation.y = q.getY();
            goal_msg.pose.pose.orientation.z = q.getZ();
            goal_msg.pose.pose.orientation.w = q.getW();

            goals.push_back(goal_msg);
        }
    }
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;
    std::vector<NavigateToPoseAction::Goal> goals;
    int cur = 0;
    rclcpp::TimerBase::SharedPtr retry_timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Nav2ActionClient>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}