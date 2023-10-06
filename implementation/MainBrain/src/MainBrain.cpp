class mainBrain : public rclpp::Node {
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped t;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    public:

    MainBrain() : Node("mainBrain"), count(_0) {

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        subscription_ = this->create_subscription<std_msgs::msg::String>("keyboard_input", 10, std::bind(&move_to_marker::executeCommand, this, _1));

        timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&move_to_marker::tfCallback, this));


    
    
    }


    private:

    void tfCallback() {
     // Check if the transformation is between "world" and "req_frame"
        std::string fromFrameRel = "world";
        std::string toFrameRel = req_frame;

        try {
            t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return;
        }
    }

    void executeCommand() {}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mainBrain>());
  rclcpp::shutdown();
  return 0;
}