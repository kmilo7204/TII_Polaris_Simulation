#include <gazebo/gazebo.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>

#include <ros/ros.h>
#include <nav_msgs/Path.h>


namespace gazebo
{
    class PathDrawerPlugin : public WorldPlugin
    {
    public:
        PathDrawerPlugin() : WorldPlugin() {}

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
        {
            // Subscribe to the /path topic
            ros::NodeHandle nh;
            path_sub_ = nh.subscribe("/path", 10, &PathDrawerPlugin::PathCallback, this);

            // Store the world pointer
            world_ = _world;
        }

        void PathCallback(const nav_msgs::Path::ConstPtr& path)
        {
          ROS_WARN("Drawing path...");
          int sphere_num = 0;
          for (int i = 0; i < path->poses.size(); ++i)
          {
            if (i % 25 != 0)
            {
              continue;
            }

            sdf::SDF sphereSDF;
            sphereSDF.SetFromString(
                  "<sdf version ='1.4'>\
                  <model name ='red_sphere'>\
                    <pose>" + std::to_string(path->poses[i].pose.position.x) + " " +
                      std::to_string(path->poses[i].pose.position.y) + "0.1s 0 0 0</pose>\
                    <static>true</static>\
                    <link name ='link'>\
                      <pose>0 0 0 0 0 0</pose>\
                      <gravity>false</gravity>\
                      <visual name ='visual'>\
                        <geometry>\
                          <sphere><radius>0.05</radius></sphere>\
                        </geometry>\
                        <material>\
                          <script>\
                            <uri>file://media/materials/scripts/gazebo.material</uri>\
                            <name>Gazebo/Red</name>\
                          </script>\
                        </material>\
                      </visual>\
                    </link>\
                  </model>\
                </sdf>");
            // Demonstrate using a custom model name.
            sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
            model->GetAttribute("name")->SetFromString("unique_sphere_" + std::to_string(sphere_num));
            world_->InsertModelSDF(sphereSDF);
            ++sphere_num;
          }
        }
    private:
      ros::Subscriber path_sub_;
      physics::WorldPtr world_;
    };

    GZ_REGISTER_WORLD_PLUGIN(PathDrawerPlugin)
} // namespace gazebo
