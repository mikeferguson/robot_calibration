
#include <robot_calibration/ceres/optimizer.h>


int main(int argc, char** argv)
{
  return 0;
}
//   std_msgs::string description_msg;
//   if (!nh.getParam("/robot_description", description_msg.data))
//   {
//     ROS_FATAL("robot_description not set!");
//     return -1;
//   }
  
//   // Create instance of optimizer
//   robot_calibration::OptimizationParams params;
//   robot_calibration::Optimizer opt(description_msg.data);








//   // Load calibration steps (if any)
//   XmlRpc::XmlRpcValue cal_steps;
//   if (nh.getParam("cal_steps", cal_steps))
//   {
//     // Should be a struct (mapping name -> config)
//     if (cal_steps.getType() != XmlRpc::XmlRpcValue::TypeStruct)
//     {
//       ROS_FATAL("Parameter 'cal_steps' should be a struct.");
//       return false;
//     }

//     XmlRpc::XmlRpcValue::iterator it;
//     size_t step;
//     size_t max_step = (cal_steps.size()>0)?cal_steps.size():1;
//     std::vector<std::string> prev_frame_names;
//     std::string prev_params_yaml;
//     for (step = 0, it = cal_steps.begin(); step < max_step; step++, it++)
//     {
//       std::string name = static_cast<std::string>(it->first);
//       ros::NodeHandle cal_steps_handle(nh, "cal_steps/"+name);
//       params.LoadFromROS(cal_steps_handle);
//       opt.optimize(params, data, verbose);
//       if (verbose)
//       {
//         std::cout << "Parameter Offsets:" << std::endl;
//         std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
//       }
//     }
//   }
//   else
//   {
//     // Single step calibration
//     params.LoadFromROS(nh);
//     opt.optimize(params, data, verbose);
//     if (verbose)
//     {
//       std::cout << "Parameter Offsets:" << std::endl;
//       std::cout << opt.getOffsets()->getOffsetYAML() << std::endl;
//     }
//   }

//   // Generate datecode
//   char datecode[80];
//   {
//     std::time_t t = std::time(NULL);
//     std::strftime(datecode, 80, "%Y_%m_%d_%H_%M_%S", std::localtime(&t));
//   }

//   // Save updated URDF
//   {
//     std::string s = opt.getOffsets()->updateURDF(description_msg.data);
//     std::stringstream urdf_name;
//     urdf_name << "/tmp/calibrated_" << datecode << ".urdf";
//     std::ofstream file;
//     file.open(urdf_name.str().c_str());
//     file << s;
//     file.close();
//   }

//   // Output camera calibration
//   {
//     std::stringstream depth_name;
//     depth_name << "/tmp/depth_" << datecode << ".yaml";
//     camera_calibration_parsers::writeCalibration(depth_name.str(), "",
//         robot_calibration::updateCameraInfo(
//                          opt.getOffsets()->get("camera_fx"),
//                          opt.getOffsets()->get("camera_fy"),
//                          opt.getOffsets()->get("camera_cx"),
//                          opt.getOffsets()->get("camera_cy"),
//                          data[0].observations[0].ext_camera_info.camera_info));  // TODO avoid hardcoding index

//     std::stringstream rgb_name;
//     rgb_name << "/tmp/rgb_" << datecode << ".yaml";
//     camera_calibration_parsers::writeCalibration(rgb_name.str(), "",
//         robot_calibration::updateCameraInfo(
//                          opt.getOffsets()->get("camera_fx"),
//                          opt.getOffsets()->get("camera_fy"),
//                          opt.getOffsets()->get("camera_cx"),
//                          opt.getOffsets()->get("camera_cy"),
//                          data[0].observations[0].ext_camera_info.camera_info));  // TODO avoid hardcoding index
//   }

//   // Output the calibration yaml
//   {
//     std::stringstream yaml_name;
//     yaml_name << "/tmp/calibration_" << datecode << ".yaml";
//     std::ofstream file;
//     file.open(yaml_name.str().c_str());
//     file << opt.getOffsets()->getOffsetYAML();
//     file << "depth_info: depth_" << datecode << ".yaml" << std::endl;
//     file << "rgb_info: rgb_" << datecode << ".yaml" << std::endl;
//     file << "urdf: calibrated_" << datecode << ".urdf" << std::endl;
//     file.close();
//   }

//   ROS_INFO("Done calibrating");

//   return 0;
// }