/*
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

#include <string>
#include <map>
#include <tinyxml.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/models/chain.h>  // for rotation functions

namespace robot_calibration
{

CalibrationOffsetParser::CalibrationOffsetParser()
{
  // TODO?
}

bool CalibrationOffsetParser::add(const std::string name)
{
  parameter_names_.push_back(name);
  parameter_offsets_.push_back(0.0);
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

bool CalibrationOffsetParser::update(const double* const free_params)
{
  for (size_t i = 0; i < parameter_offsets_.size(); ++i)
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

int CalibrationOffsetParser::size()
{
  return parameter_names_.size();
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

      xyz[0] = frame_offset.p.x();
      xyz[1] = frame_offset.p.y();
      xyz[2] = frame_offset.p.z();

      // Get roll, pitch, yaw about fixed axis
      frame_offset.M.GetRPY(rpy[0], rpy[1], rpy[2]);

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

        if (xyz_pieces.size() == 3)
        {
          // Update xyz
          for (int i = 0; i < 3; ++i)
          {
            double x = double(boost::lexical_cast<double>(xyz_pieces[i]) + xyz[i]);
            if (i > 0)
              xyz_ss << " ";
            xyz_ss << std::fixed << std::setprecision(precision) << x;
          }
        }
        else
        {
          // Create xyz
          for (int i = 0; i < 3; ++i)
          {
            if (i > 0)
              xyz_ss << " ";
            xyz_ss << std::fixed << std::setprecision(precision) << xyz[i];
          }
        }

        // Split out rpy of origin, break into 3 strings
        std::vector<std::string> rpy_pieces;
        boost::split(rpy_pieces, rpy_str, boost::is_any_of(" "));

        if (rpy_pieces.size() == 3)
        {
          // Update rpy
          for (int i = 0; i < 3; ++i)
          {
            double x = double(boost::lexical_cast<double>(rpy_pieces[i]) + rpy[i]);
            if (i > 0)
              rpy_ss << " ";
            rpy_ss << std::fixed << std::setprecision(precision) << x;
          }
        }
        else
        {
          // Create rpy
          for (int i = 0; i < 3; ++i)
          {
            if (i > 0)
              rpy_ss << " ";
            rpy_ss << std::fixed << std::setprecision(precision) << rpy[i];
          }
        }

        // Update xml
        origin_xml->SetAttribute("xyz", xyz_ss.str());
        origin_xml->SetAttribute("rpy", rpy_ss.str());
      }
      else
      {
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
