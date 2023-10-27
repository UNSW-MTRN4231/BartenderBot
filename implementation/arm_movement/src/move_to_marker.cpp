#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("move_to", 10, std::bind(&move_to_marker::executeMove, this, _1));


      // Generate the movegroup interface
      move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
      
      move_group_interface->setPlanningTime(30.0);

      std::string frame_id = move_group_interface->getPlanningFrame();

      //create walls for collision
      RCLCPP_INFO(this->get_logger(), "generate walls");

      // Generate the objects to avoid
      // Generate a table collision object based on the lab task
      auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
      auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
      auto col_object_table = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, -0.03, frame_id, "table");

      RCLCPP_INFO(this->get_logger(), "insert walls");

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // Apply table as a collision object
      planning_scene_interface.applyCollisionObject(col_object_backWall);
      planning_scene_interface.applyCollisionObject(col_object_sideWall);
      planning_scene_interface.applyCollisionObject(col_object_table);

      RCLCPP_INFO(this->get_logger(), "constructed");
    }
  private:

  // Plan Movement
  void executeMove(const geometry_msgs::msg::Pose &msg) const {

    //TODO NEW CHECK FOR LEGAL POSE
    if (true) {
      auto success = false;

      RCLCPP_INFO(this->get_logger(), "Starting move");
      
      moveit::planning_interface::MoveGroupInterface::Plan planMessage;

      //Plan movement to ball point
      move_group_interface->setPoseTarget(msg);
      success = static_cast<bool>(move_group_interface->plan(planMessage));
      RCLCPP_INFO(this->get_logger(), "Planning done");
      //Execute movement to point 1
      if (success) {
        move_group_interface->execute(planMessage);
      } else {
        std::cout <<  "Planning failed!" << std::endl;
      }

      RCLCPP_INFO(this->get_logger(), "Done moving");
    }
  }


  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<move_to_marker>());
  rclcpp::shutdown();
  return 0;
}
