#include <gazebo/gazebo.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>

namespace gazebo
{
    class PathDrawerPlugin : public WorldPlugin
    {
    public:
        PathDrawerPlugin() : WorldPlugin() {}

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
        {

            // Option 2: Insert model from string via function call.
            // Insert a sphere model from string
            sdf::SDF sphereSDF;
            // sphereSDF.SetFromString(
              // "<sdf version ='1.4'>\
              //     <model name ='sphere'>\
              //       <pose>1 0 0 0 0 0</pose>\
              //       <static>true</static>\
              //       <link name ='link'>\
              //         <pose>0 2.5 .5 0 0 0</pose>\
              //         <gravity>false</gravity>\
              //         <collision name ='collision'>\
              //           <geometry>\
              //             <sphere><radius>0.5</radius></sphere>\
              //           </geometry>\
              //         </collision>\
              //         <visual name ='visual'>\
              //           <geometry>\
              //             <sphere><radius>0.1</radius></sphere>\
              //           </geometry>\
              //         </visual>\
              //       </link>\
              //     </model>\
              //   </sdf>");
            sphereSDF.SetFromString(
                  "<sdf version ='1.4'>\
                  <model name ='red_sphere'>\
                    <pose>1 0 0 0 0 0</pose>\
                    <static>true</static>\
                    <link name ='link'>\
                      <pose>0 2.5 .5 0 0 0</pose>\
                      <gravity>false</gravity>\
                      <collision name ='collision'>\
                        <geometry>\
                          <sphere><radius>0.5</radius></sphere>\
                        </geometry>\
                      </collision>\
                      <visual name ='visual'>\
                        <geometry>\
                          <sphere><radius>0.5</radius></sphere>\
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
            model->GetAttribute("name")->SetFromString("unique_sphere");
            _world->InsertModelSDF(sphereSDF);

            // Create a pose
            // ignition::math::Pose3d pose(1, 2, 0, 0, 0, 0);

            // // Create a new model
            // physics::ModelPtr markerModel = _world->CreateModel("marker_model");

            // // Create a link for the model
            // physics::LinkPtr markerLink = markerModel->CreateLink("marker_link");

            // // Create a visual for the link
            // rendering::VisualPtr markerVisual = markerLink->GetVisual("visual");
            // if (!markerVisual)
            // {
            //     markerVisual = markerLink->CreateVisual("visual");
            // }

            // // Set the visual's pose
            // markerVisual->SetPose(pose);

            // // Add the model to the world
            // _world->InsertModelFile("model://marker_model");
            // Create a new model as a string

            // Get the path to the plugin's shared library
            // std::string pluginPath = gazebo::common::SystemPaths::Instance()->FindPlugin("path_drawer_plugin")->String();

            // // Extract the directory containing the plugin
            // std::string pluginDir = gazebo::common::SystemPaths::Instance()->GetPath("path_drawer_plugin");

            // // Path to the URDF file (assuming it's named marker_model.urdf)
            // std::string urdfPath = pluginDir + "/marker_model.urdf";
            // _world->InsertModelFile(urdfPath);

            // std::string modelString = "<model name='marker_model'><pose>" + 
            //                           std::to_string(pose.Pos().X()) + " " +
            //                           std::to_string(pose.Pos().Y()) + " " +
            //                           std::to_string(pose.Pos().Z()) + " " +
            //                           std::to_string(pose.Rot().Roll()) + " " +
            //                           std::to_string(pose.Rot().Pitch()) + " " +
            //                           std::to_string(pose.Rot().Yaw()) + "</pose><static>true</static><link name='marker_link'><visual name='visual'><geometry><sphere><radius>0.1</radius></sphere></geometry></visual></link></model>";

            // // Insert the model into the world
            // _world->InsertModelString(modelString);
                  // Insert the model into the world
        }
    };

    GZ_REGISTER_WORLD_PLUGIN(PathDrawerPlugin)
} // namespace gazebo
