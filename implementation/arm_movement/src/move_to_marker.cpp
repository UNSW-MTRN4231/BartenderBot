#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <functional>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
using std::placeholders::_1;

//Function to generate a collision object
auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0; 
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}


using std::placeholders::_1;

class move_to_marker : public rclcpp::Node
{
  public:
    move_to_marker() : Node("move_to_marker")
    {

      // Initalise the pose subscriber
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("move_to", 10, std::bind(&move_to_marker::messageRead, this, _1));
      // Initialise the arm ready publisher
      arm_ready_publisher_ = this->create_publisher<std_msgs::msg::String>("done_move", 10);

      // Generate the movegroup interface
      move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
      
      move_group_interface->setPlanningTime(10.0);
      move_group_interface->setNumPlanningAttempts(15);
      move_group_interface->setPlannerId("RRTstarkConfigDefault");

      std::string frame_id = move_group_interface->getPlanningFrame();

      //create walls for collision
      RCLCPP_INFO(this->get_logger(), "generate walls");

      // Generate the objects to avoid
      // Generate a table collision object based on the lab task
      auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
      auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
      auto col_object_table = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, -0.01, frame_id, "table");

      RCLCPP_INFO(this->get_logger(), "insert walls");

      // Generate path constraint obstacles

      auto col_object_backConst = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.10, 0.5, frame_id, "backConst");
      auto col_object_sideConst = generateCollisionObject( 0.04, 1.2, 1.0, -0.25, 0.25, 0.5, frame_id, "sideConst");

      RCLCPP_INFO(this->get_logger(), "insert constraints");


      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // Apply table as a collision object
      planning_scene_interface.applyCollisionObject(col_object_backWall);
      planning_scene_interface.applyCollisionObject(col_object_sideWall);
      planning_scene_interface.applyCollisionObject(col_object_table);
      // Apply constraints as a collision object
      //planning_scene_interface.applyCollisionObject(col_object_backConst);
      //planning_scene_interface.applyCollisionObject(col_object_sideConst);

      RCLCPP_INFO(this->get_logger(), "constructed");
    }
  private:

    // Plan Movement
    void executeFreeMove(const geometry_msgs::msg::Pose &msg) const {

      //TODO NEW CHECK FOR LEGAL POSE
      if (true) {
        auto success = false;

        RCLCPP_INFO(this->get_logger(), "Starting free move");
        
        moveit::planning_interface::MoveGroupInterface::Plan planMessage;

        //Plan movement to ball point
        move_group_interface->setPoseTarget(msg);
        success = static_cast<bool>(move_group_interface->plan(planMessage));
        RCLCPP_INFO(this->get_logger(), "Planning done");
        //Execute movement to point 1
        if (success) {
          move_group_interface->execute(planMessage);
          RCLCPP_INFO(this->get_logger(), "Done moving");
        } else {
          std::cout <<  "Planning failed!" << std::endl;
        }

      }
    }

    void executeLinearMove(const geometry_msgs::msg::Pose &msg) const {
      //TODO NEW CHECK FOR LEGAL POSE
      if (true) {

        RCLCPP_INFO(this->get_logger(), "Starting linear move");
        
        // Cartesian Paths
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose targetPose1 = msg;
        waypoints.push_back(targetPose1);
        moveit_msgs::msg::RobotTrajectory trajectory;
        //double fraction = move_group_interface->computeCartesianPath(waypoints, 0.01, 0.0, planMessage.trajectory_);
        double fraction = move_group_interface->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        RCLCPP_INFO(this->get_logger(),"Visualising cartesian path, (%.2f%% achieved)", fraction*100.0);

        //Execute movement to point 1
        if (fraction == 1) {
          move_group_interface->execute(trajectory);
          RCLCPP_INFO(this->get_logger(), "Done moving");
        } else {
          std::cout <<  "Planning failed! \nExecuting free move" << std::endl;
          executeFreeMove(msg);
        }

      } 
    }

    // Plan Movement
    void executeRotationMove(const geometry_msgs::msg::Pose &msg) const {

      //TODO NEW CHECK FOR LEGAL POSE
      if (true) {
        auto success = false;

        RCLCPP_INFO(this->get_logger(), "Starting rotation move");
        
        moveit::planning_interface::MoveGroupInterface::Plan planMessage;

        //Plan movement to ball point
        //move_group_interface->setOrientationTarget(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
        move_group_interface->setJointValueTarget("wrist_3_joint", msg.orientation.w);
        success = static_cast<bool>(move_group_interface->plan(planMessage));
        RCLCPP_INFO(this->get_logger(), "Planning done");
        //Execute movement to point 1
        if (success) {
          move_group_interface->execute(planMessage);
          RCLCPP_INFO(this->get_logger(), "Done moving");
        } else {
          std::cout <<  "Planning failed!" << std::endl;
        }

      }
    }

    void executeHomeMove() const {
      //TODO NEW CHECK FOR LEGAL POSE
      if (true) {
        auto success = false;

        RCLCPP_INFO(this->get_logger(), "Starting home move");
        
        moveit::planning_interface::MoveGroupInterface::Plan planMessage;

        //Plan movement to home
        std::vector<double> joints = {-150.0*M_PI/180, -90.0*M_PI/180, -90.0*M_PI/180, -180.0*M_PI/180, -90.0*M_PI/180, 90.0*M_PI/180};
        //std::vector<double> joints = {0.0*M_PI/180, -75.0*M_PI/180, 90.0*M_PI/180, -15.0*M_PI/180, 90.0*M_PI/180, 90.0*M_PI/180};
        move_group_interface->setJointValueTarget(joints);
        success = static_cast<bool>(move_group_interface->plan(planMessage));
        RCLCPP_INFO(this->get_logger(), "Planning done");
        //Execute movement to point 1
        if (success) {
          move_group_interface->execute(planMessage);
          RCLCPP_INFO(this->get_logger(), "Done moving");

        } else {
          std::cout <<  "Planning failed!" << std::endl;
        }

      }
    }

    void addPoint(const geometry_msgs::msg::Pose &msg) {
      if (true) {
        RCLCPP_INFO(this->get_logger(), "Adding point to path");
        posepoints.push_back(msg);
      }
    }

    void executePath() {
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_interface->computeCartesianPath(posepoints, 0.05, 5.0, trajectory);
        //double fraction = move_group_interface->computeCartesianPath(posepoints, 0.01, 0.0, trajectory);
        posepoints.clear();
        RCLCPP_INFO(this->get_logger(),"Visualising cartesian path, (%.2f%% achieved)", fraction*100.0);

        move_group_interface->execute(trajectory);
        RCLCPP_INFO(this->get_logger(), "Done moving");
    }

    void messageRead(const geometry_msgs::msg::PoseStamped &msg) {
      std_msgs::msg::String status;
      status.data = "waiting";
      arm_ready_publisher_->publish(status);
      if (msg.header.frame_id == "free") {
        executeFreeMove(msg.pose);
      } else if ((msg.header.frame_id == "linear")) {
        executeLinearMove(msg.pose);
      } else if ((msg.header.frame_id == "rotate")) {
        executeRotationMove(msg.pose);
      } else if ((msg.header.frame_id == "home")) {
        executeHomeMove();
      } else if ((msg.header.frame_id == "add")) {
        addPoint(msg.pose);
      } else if ((msg.header.frame_id == "go")) {
        executePath();
      }else {
        RCLCPP_INFO(this->get_logger(), "Cannot read move command, executing free move");
        executeFreeMove(msg.pose);
      }
      status.data = "done";
      arm_ready_publisher_->publish(status);
    }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_ready_publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
  std::vector<geometry_msgs::msg::Pose> posepoints;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<move_to_marker>());
  rclcpp::shutdown();
  return 0;
}
