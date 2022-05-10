/*
 * Copyright (C) 2018-2019 Michael Ferguson
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#include <fstream>
#include <string>
#include <map>
#include <tinyxml.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/models/chain.h>  // for rotation functions

namespace robot_calibration
{

CalibrationOffsetParser::CalibrationOffsetParser()
{
  num_free_params_ = 0;
}

bool CalibrationOffsetParser::add(const std::string name)
{
  double value = 0.0;

  // Check against parameters
  for (size_t i = 0; i < parameter_names_.size(); ++i)
  {
    if (parameter_names_[i] == name)
    {
      if (i < num_free_params_)
      {
        // This is already a free param, don't re-add
        return false;
      }
      // Get the current value
      value = parameter_offsets_[i];
      // Remove the non-free-param version
      parameter_names_.erase(parameter_names_.begin() + i);
      parameter_offsets_.erase(parameter_offsets_.begin() + i);
    }
  }

  // Add the parameter at end of current free params
  parameter_names_.insert(parameter_names_.begin() + num_free_params_, name);
  parameter_offsets_.insert(parameter_offsets_.begin() + num_free_params_, value);
  ++num_free_params_;
  return true;
}

bool CalibrationOffsetParser::addFrame(
    const std::string name,
    bool calibrate_x, bool calibrate_y, bool calibrate_z,
    bool calibrate_roll, bool calibrate_pitch, bool calibrate_yaw)
{
  frame_names_.push_back(name);
  if (calibrate_x)
    add(std::string(name).append("_x"));
  if (calibrate_y)
    add(std::string(name).append("_y"));
  if (calibrate_z)
    add(std::string(name).append("_z"));

  // These don't really correspond to rpy unless only one is set 
  // TODO: check that we do either roll, pitch or yaw, or all 3 (never just 2)   
  if (calibrate_roll)
    add(std::string(name).append("_a"));
  if (calibrate_pitch)
    add(std::string(name).append("_b"));
  if (calibrate_yaw)
    add(std::string(name).append("_c"));

  return true;
}

bool CalibrationOffsetParser::set(const std::string name, double value)
{
  for (size_t i = 0; i < num_free_params_; ++i)
  {
    if (parameter_names_[i] == name)
    {
      parameter_offsets_[i] = value;
      return true;
    }
  }
  return false;
}

bool CalibrationOffsetParser::setFrame(
    const std::string name,
    double x, double y, double z,
    double roll, double pitch, double yaw)
{
  // Get axis-magnitude
  double a, b, c;
  KDL::Rotation r = KDL::Rotation::RPY(roll, pitch, yaw);
  axis_magnitude_from_rotation(r, a, b, c);

  // Set values
  set(std::string(name).append("_x"), x);
  set(std::string(name).append("_y"), y);
  set(std::string(name).append("_z"), z);
  set(std::string(name).append("_a"), a);
  set(std::string(name).append("_b"), b);
  set(std::string(name).append("_c"), c);

  return true;
}

bool CalibrationOffsetParser::initialize(double* free_params)
{
  for (size_t i = 0; i < num_free_params_; ++i)
    free_params[i] = parameter_offsets_[i];
  return true;
}

bool CalibrationOffsetParser::update(const double* const free_params)
{
  for (size_t i = 0; i < num_free_params_; ++i)
    parameter_offsets_[i] = free_params[i];
  return true;
}

double CalibrationOffsetParser::get(const std::string name) const
{
  for (size_t i = 0; i < parameter_names_.size(); ++i)
  {
    if (parameter_names_[i] == name)
      return parameter_offsets_[i];
  }
  // Not calibrating this
  return 0.0;
}

bool CalibrationOffsetParser::getFrame(const std::string name, KDL::Frame& offset) const
{
  // Don't bother with following computation if this isn't a calibrated frame.
  bool has_offset = false;
  for (size_t i = 0; i < frame_names_.size(); ++i)
  {
    if (frame_names_[i] == name)
    {
      has_offset = true;
      break;
    }
  }

  if (!has_offset)
    return false;

  offset.p.x(get(std::string(name).append("_x")));
  offset.p.y(get(std::string(name).append("_y")));
  offset.p.z(get(std::string(name).append("_z")));

  offset.M = rotation_from_axis_magnitude(
                 get(std::string(name).append("_a")),
                 get(std::string(name).append("_b")),
                 get(std::string(name).append("_c")));

  return true;
}

size_t CalibrationOffsetParser::size()
{
  return num_free_params_;
}

bool CalibrationOffsetParser::reset()
{
  num_free_params_ = 0;
  return true;
}

bool CalibrationOffsetParser::loadOffsetYAML(const std::string& filename)
{
  std::string line;
  std::ifstream f(filename.c_str());
  while (std::getline(f, line))
  {
    std::istringstream str(line.c_str());
    std::string param;
    double value;
    if (str >> param >> value)
    {
      // Remove the ":"
      param.erase(param.size() - 1);
      std::cout << "Loading '" << param << "' with value " << value << std::endl;
      set(param, value);
    }
  }
  f.close();
  return true;
}

std::string CalibrationOffsetParser::getOffsetYAML()
{
  std::stringstream ss;
  for (size_t i = 0; i < parameter_names_.size(); ++i)
  {
    ss << parameter_names_[i] << ": " << parameter_offsets_[i] << std::endl;
  }
  return ss.str();
}

std::string CalibrationOffsetParser::updateURDF(const std::string &urdf)
{
  const double precision = 8;

  TiXmlDocument xml_doc;
  xml_doc.Parse(urdf.c_str());

  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    // TODO: error notification? We should never get here since URDF parse
    //       at beginning of calibration will fail
    return urdf;
  }

  // Update each joint
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    const char * name = joint_xml->Attribute("name");

    // Is there a joint calibration needed?
    double offset = get(std::string(name));
    if (offset != 0.0)
    {
      TiXmlElement *calibration_xml = joint_xml->FirstChildElement("calibration");
      if (calibration_xml)
      {
        // Existing calibration, update rising attribute
        const char * rising_position_str = calibration_xml->Attribute("rising");
        if (rising_position_str != NULL)
        {
          try
          {
            offset += double(boost::lexical_cast<double>(rising_position_str));
            calibration_xml->SetDoubleAttribute("rising", offset);
          }
          catch (boost::bad_lexical_cast &e)
          {
            // TODO: error
          }
        }
        else
        {
          // TODO: error
        }
      }
      else
      {
        // No calibration previously, add an element + attribute
        calibration_xml = new TiXmlElement("calibration");
        calibration_xml->SetDoubleAttribute("rising", offset);
        TiXmlNode * calibration = calibration_xml->Clone();
        joint_xml->InsertEndChild(*calibration);
      }
    }

    KDL::Frame frame_offset;
    bool has_update = getFrame(name, frame_offset);
    if (has_update)
    {
      std::vector<double> xyz(3, 0.0);
      std::vector<double> rpy(3, 0.0);

      // String streams for output
      std::stringstream xyz_ss, rpy_ss;

      TiXmlElement *origin_xml = joint_xml->FirstChildElement("origin");
      if (origin_xml)
      {
        // Update existing origin
        const char * xyz_str = origin_xml->Attribute("xyz");
        const char * rpy_str = origin_xml->Attribute("rpy");

        // Split out xyz of origin, break into 3 strings
        std::vector<std::string> xyz_pieces;
        boost::split(xyz_pieces, xyz_str, boost::is_any_of(" "));

        // Split out rpy of origin, break into 3 strings
        std::vector<std::string> rpy_pieces;
        boost::split(rpy_pieces, rpy_str, boost::is_any_of(" "));

        KDL::Frame origin(KDL::Rotation::Identity(), KDL::Vector::Zero());
        if (xyz_pieces.size() == 3)
        {
          origin.p = KDL::Vector(boost::lexical_cast<double>(xyz_pieces[0]), boost::lexical_cast<double>(xyz_pieces[1]), boost::lexical_cast<double>(xyz_pieces[2]));
        }

        if (rpy_pieces.size() == 3)
        {
          origin.M = KDL::Rotation::RPY(boost::lexical_cast<double>(rpy_pieces[0]), boost::lexical_cast<double>(rpy_pieces[1]), boost::lexical_cast<double>(rpy_pieces[2]));
        }

        // Update
        origin = origin * frame_offset;

        xyz[0] = origin.p.x();
        xyz[1] = origin.p.y();
        xyz[2] = origin.p.z();

        // Get roll, pitch, yaw about fixed axis
        origin.M.GetRPY(rpy[0], rpy[1], rpy[2]);

        // Update xyz
        for (int i = 0; i < 3; ++i)
        {
          if (i > 0)
            xyz_ss << " ";
          xyz_ss << std::fixed << std::setprecision(precision) << xyz[i];
        }

        // Update rpy
        for (int i = 0; i < 3; ++i)
        {
          if (i > 0)
            rpy_ss << " ";
          rpy_ss << std::fixed << std::setprecision(precision) << rpy[i];
        }

        // Update xml
        origin_xml->SetAttribute("xyz", xyz_ss.str());
        origin_xml->SetAttribute("rpy", rpy_ss.str());
      }
      else
      {
        xyz[0] = frame_offset.p.x();
        xyz[1] = frame_offset.p.y();
        xyz[2] = frame_offset.p.z();

        // Get roll, pitch, yaw about fixed axis
        frame_offset.M.GetRPY(rpy[0], rpy[1], rpy[2]);

        // No existing origin, create an element with attributes
        origin_xml = new TiXmlElement("origin");

        // Create xyz
        for (int i = 0; i < 3; ++i)
        {
          if (i > 0)
            xyz_ss << " ";
          xyz_ss << std::fixed << std::setprecision(precision) << xyz[i];
        }
        origin_xml->SetAttribute("xyz", xyz_ss.str());

        // Create rpy
        for (int i = 0; i < 3; ++i)
        {
          if (i > 0)
            rpy_ss << " ";
          rpy_ss << std::fixed << std::setprecision(precision) << rpy[i];
        }
        origin_xml->SetAttribute("rpy", rpy_ss.str());

        TiXmlNode * origin = origin_xml->Clone();
        joint_xml->InsertEndChild(*origin);
      }
    }
  }

  // Print to a string
  TiXmlPrinter printer;
  printer.SetIndent("  ");
  xml_doc.Accept(&printer);
  std::string new_urdf = printer.CStr();

  return new_urdf;
}

}  // namespace robot_calibration
