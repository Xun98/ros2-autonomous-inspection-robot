#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <queue>

class Nav2ActionClient : public rclcpp::Node
{
public:
    struct PatrolPoint
    {
        std::string name;
        double x;
        double y;
        double yaw;
        bool need_capture;
    };

    enum class State
    {
        IDLE,
        NAVIGATION,
        CAPTURIN,
        ERROR,
        FINISHED,
    };
    
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    Nav2ActionClient();

    void start_patrol();

private:

    void load_goals();
    
    void send_goal();

    void on_success();
    void on_failed();

    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;
    std::vector<PatrolPoint> goals;
    int cur_goals_index = 0;
    rclcpp::TimerBase::SharedPtr retry_timer_;

    int max_rounds;
    int cur_rounds;
    State state = State::IDLE;

    int retry_count;
    const int RETRY_COUNT = 3;
};
