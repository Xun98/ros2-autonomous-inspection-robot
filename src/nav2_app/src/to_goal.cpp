#include "to_goal.h"

Nav2ActionClient::Nav2ActionClient() : Node("nav2_action_client")
{
    action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
        this, "navigate_to_pose");
    load_goals();

    send_goal();
}

void Nav2ActionClient::send_goal()
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

        RCLCPP_INFO(this->get_logger(), "pub %s: { %.2f, %.2f }", goals[cur_goals_index].name.c_str(), goals[cur_goals_index].x, goals[cur_goals_index].y);
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

        cur_goals_index = (cur_goals_index + 1) % goals.size();
        send_goal();
    };


    auto goal_msg = NavigateToPoseAction::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = goals[cur_goals_index].x;
        goal_msg.pose.pose.position.y = goals[cur_goals_index].y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, goals[cur_goals_index].yaw);
        goal_msg.pose.pose.orientation.x = q.getX();
        goal_msg.pose.pose.orientation.y = q.getY();
        goal_msg.pose.pose.orientation.z = q.getZ();
        goal_msg.pose.pose.orientation.w = q.getW();

    goal_msg.pose.header.stamp = get_clock()->now();
    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void Nav2ActionClient::load_goals()
{

    int goal_count = this->declare_parameter<int>("goal_count", 0);
    if (goal_count < 2)
    {
        throw std::runtime_error("goal only start");
    }
    goals.reserve(goal_count);

    max_rounds = this->declare_parameter<int>("max_rounds", 0);
    if (max_rounds < 1)
    {
        throw std::runtime_error("max rounds: " + std::to_string(max_rounds));
    }

    for (int i = 1; i <= goal_count; ++i)
    {
        PatrolPoint point;
        point.name = "goal_" + std::to_string(i);
        
        auto pose = declare_parameter<std::vector<double>>(point.name + ".pose", {.0, .0, .0});
        if (pose.size() != 3)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid parameter: %s", point.name.c_str());

            throw std::runtime_error("Invalid goal configuration");
        }
        point.x = pose[0];
        point.y = pose[1];
        point.yaw = pose[2];

        point.need_capture = declare_parameter<bool>(point.name + ".need_capture", false);
        goals.push_back(std::move(point));
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Nav2ActionClient>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}