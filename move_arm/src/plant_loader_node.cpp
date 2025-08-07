#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
//previous two instead of #include <shapes/mesh_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>

class PlantLoaderNode : public rclcpp::Node
{
public:
  PlantLoaderNode() : Node("plant_loader_node")
  {
    using namespace std::chrono_literals;
    // Give MoveIt a moment to start up
    timer_ = this->create_wall_timer(1s, std::bind(&PlantLoaderNode::loadPlant, this));
  }

private:
  void loadPlant()
  {
    timer_->cancel();  // Prevent repeated loading

    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "base";  // Adjust to match your robot base frame
    object.id = "plant";

    std::string pkg_path = ament_index_cpp::get_package_share_directory("move_arm");
    std::string mesh_path = pkg_path + "/meshes/collisions/tomato-plant.stl";
    std::string mesh_resource = "file://" + mesh_path;

    double scale = 0.0003;  // Scale down to 1% of original size (adjust as needed)

    shapes::Mesh* m = shapes::createMeshFromResource(mesh_resource);
    if (!m)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from: %s", mesh_resource.c_str());
      return;
    }

    for (unsigned int i = 0; i < m->vertex_count; ++i)
    {
        m->vertices[3 * i + 0] *= scale;  // X
        m->vertices[3 * i + 1] *= scale;  // Y
        m->vertices[3 * i + 2] *= scale;  // Z
    }

    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg mesh_msg_tmp;
    shapes::constructMsgFromShape(m, mesh_msg_tmp);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);
    
    //shapes::Mesh* m = shapes::createMeshFromResource("file://" + mesh_path);
    //shape_msgs::msg::Mesh mesh_msg;
    //shapes::ShapeMsg mesh_msg_tmp;
    //shapes::constructMsgFromShape(m, mesh_msg_tmp);
    //mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);

    geometry_msgs::msg::Pose plant_pose;
    plant_pose.position.x = 0.6;
    plant_pose.position.y = 0.0;
    plant_pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(M_PI / 2, 0, 0);  // Roll (X), Pitch (Y), Yaw (Z rotation)

    plant_pose.orientation.x = q.x();
    plant_pose.orientation.y = q.y();
    plant_pose.orientation.z = q.z();
    plant_pose.orientation.w = q.w();

    object.meshes.push_back(mesh_msg);
    object.mesh_poses.push_back(plant_pose);
    object.operation = object.ADD;

    //auto quaternion = object.Pose.quaternion PASCAL EXAMPLE IDEA

    psi.applyCollisionObjects({object});

    RCLCPP_INFO(this->get_logger(), "Loaded plant into planning scene.");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlantLoaderNode>());
  rclcpp::shutdown();
  return 0;
}