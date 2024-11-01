/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the MiniBot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <minibot_control/minibot_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <minibot_control/interpolating_map.h>
#include <boost/asio/io_service.hpp>
#include <angles/angles.h>
using namespace::boost::asio;
using boost::asio::serial_port;

namespace minibot_control
{

  MiniBotJoint::MiniBotJoint(JointType t) {
    type = t;
  }

  MiniBotJoint::MiniBotJoint() {

  }

  // Motors

  MiniBotMotorJoint::MiniBotMotorJoint(uint8_t port, double velocity_mult, double velocity_feed_forward, bool inverted) : MiniBotJoint(motor) {
    this->type = motor;
    this->inverted = inverted;
    this->port = port;
    this->velocity_mult = velocity_mult;
    this->velocity_feed_forward = velocity_feed_forward;
  }

  std::string MiniBotMotorJoint::setVelocity(double cmd) {
    double output = (inverted ? -1.0 : 1.0) * ((cmd >= 0 ? 1 : -1) * velocity_feed_forward + velocity_mult * cmd);
    if (cmd == 0) {
      output = 0.0;
    }
    std::string toWrite = "m" + std::to_string(port) + ";" + std::to_string(output) + ";";
    return toWrite;
  }

  // Servos

  MiniBotServoJoint::MiniBotServoJoint(uint8_t port, double offset, bool inverted, double scale, double initial_position) : MiniBotJoint(motor), port(port), offset(offset), inverted(inverted), scale(scale) {
    this->initial_position = initial_position;
  }

  std::string MiniBotServoJoint::setPosition(double cmd) {
    std::string toWrite = "s" + std::to_string(port) + ";" + std::to_string(angles::to_degrees(((inverted ? -1.0 : 1.0) * cmd) + offset) / scale) + ";";
    return toWrite;
  }

  JointType getType(std::string s) {
    ROS_INFO_STREAM("MOTOR TYPE " << s);
    if (s == "motor") {
      return motor;
    } else if (s == "servo") {
      return servo;
    } else {
      ROS_WARN_STREAM("DEFAULTING TO MOTOR TYPE");
      return motor;
    }
  }

  MiniBotJoint* parseJoint(ros::NodeHandle nh, const std::string &n) {
    ros::NodeHandle rpnh(
        nh, n);
    std::size_t error = 0;
    std::string jointType;
    error += !rosparam_shortcuts::get(n, rpnh, "type", jointType);
    JointType t;
    t = getType(jointType);
    if (t == motor) {
      bool inverted;
      int port;
      double velocity_mult;
      double vyi;
      error += !rosparam_shortcuts::get(n, rpnh, "inverted", inverted);
      error += !rosparam_shortcuts::get(n, rpnh, "port", port);
      error += !rosparam_shortcuts::get(n, rpnh, "velocity_mult", velocity_mult);
      error += !rosparam_shortcuts::get(n, rpnh, "velocity_feed_forward", vyi);
      MiniBotMotorJoint* j = new MiniBotMotorJoint(port, velocity_mult, vyi, inverted);
      ROS_INFO_STREAM("Motor joint, name = " << n << ", port = " << port << ", inverted = " << inverted << ", vmult = " << velocity_mult);
      rosparam_shortcuts::shutdownIfError(n, error);
      return j;
    }
    else if (t == servo) {
      int port;
      double offset;
      double scale;
      bool inverted;
      double initial_position;
      error += !rosparam_shortcuts::get(n, rpnh, "inverted", inverted);
      error += !rosparam_shortcuts::get(n, rpnh, "port", port);
      error += !rosparam_shortcuts::get(n, rpnh, "offset", offset);
      error += !rosparam_shortcuts::get(n, rpnh, "scale", scale);
      error += !rosparam_shortcuts::get(n, rpnh, "initial_position", initial_position);
      MiniBotServoJoint* j = new MiniBotServoJoint(port, offset, inverted, scale, initial_position);
      j->setPosition(initial_position);
      ROS_INFO_STREAM("Servo joint, name = " << n << ", port = " << port << ", offset = " << offset << ", scale = " << scale << ", inverted = " << inverted << ", initial_position = " << initial_position);
      rosparam_shortcuts::shutdownIfError(n, error);
      return j;
    }
    return new MiniBotJoint(t);
  }


  MiniBotHWInterface::MiniBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    io_service ios;

    ros::NodeHandle hwnh(nh, "hardware_interface");

    std::size_t error = 0;
    std::string serial_port;
    error += !rosparam_shortcuts::get("hardware_interface", hwnh, "write_proto", write_proto);
    error += !rosparam_shortcuts::get("hardware_interface", hwnh, "websocket_logging_level", log_ws_);
    // ===== BT serial START =====
    if (write_proto == 0) {
      ROS_WARN_STREAM("Using bluetooth serial");
      error += !rosparam_shortcuts::get("hardware_interface", hwnh, "port", serial_port);
      p = new boost::asio::serial_port(ios, serial_port);

      try {
        p->set_option(boost::asio::serial_port_base::baud_rate(921600));
      } catch (boost::system::system_error::exception e) {
        ROS_ERROR_STREAM("error setting serial port baud rate");
      }
    } 
    // ===== BT serial END =====
    // ===== Websockets START ===== 
    else if (write_proto == 1) {
      ROS_WARN_STREAM("Using websockets");
      use_websocket_ = true;
      error += !rosparam_shortcuts::get("hardware_interface", hwnh, "websocket_uri", ws_uri_);
      ws_thread_ = std::thread(&MiniBotHWInterface::websocketConnect, this, ws_uri_);
    } 
    // ===== Websockets END =====
    else {
      ROS_ERROR_STREAM("Exiting: Invalid write_proto value " << write_proto);
      exit(1);
    }

    rosparam_shortcuts::shutdownIfError("MiniBotHWInterface", error);
    for (std::string n : joint_names_) {
      std::shared_ptr<MiniBotJoint> jp = std::shared_ptr<MiniBotJoint>(parseJoint(nh, n));
      joints_[n] = jp;
    }
    ROS_INFO_NAMED("minibot_hw_interface", "MiniBotHWInterface Ready.");

    // ROS_INFO_STREAM("Parsed " << joints_.size() << " joints");
    // for (std::size_t joint_id = 0; joint_id < joints_.size(); ++joint_id) {
    //   auto thisJoint = joints_[joint_names_[joint_id]];
    //   ROS_INFO_STREAM("Joint " << joint_names_[joint_id] << " has type " << thisJoint->type);
    //   if (thisJoint->type == motor) {
    //     joint_velocity_[joint_id] = 0.0;
    //   } else if (thisJoint->type == servo) {
    //     joint_position_[joint_id] = thisJoint->initial_position;
    //     ROS_INFO_STREAM("Initial joint position for " << joint_names_[joint_id] << " = " << joint_position_[joint_id]);
    //   }
    // }
  }

  void MiniBotHWInterface::read(ros::Duration& elapsed_time)
  {
    if (use_websocket_ && ws_connected_) {
      ws_client_.send(ws_hdl_, "voltage?", websocketpp::frame::opcode::text);
    }
  }

  void MiniBotHWInterface::setupWebsocketLogging() {
    using namespace websocketpp::log;
    
    if (!log_ws_) {
      ws_client_.clear_access_channels(alevel::all);
      ws_client_.set_access_channels(alevel::connect | alevel::disconnect);
    }

    ROS_INFO_STREAM("Set logging channels for WS");
  }

  void MiniBotHWInterface::websocketConnect(const std::string& uri) {
      setupWebsocketLogging();
      ws_client_.init_asio();
      ws_client_.set_open_handler(std::bind(&MiniBotHWInterface::on_open, this, std::placeholders::_1));
      ws_client_.set_message_handler(std::bind(&MiniBotHWInterface::on_message, this, std::placeholders::_1, std::placeholders::_2));
      ws_client_.set_close_handler(std::bind(&MiniBotHWInterface::on_close, this, std::placeholders::_1));
      
      websocketpp::lib::error_code ec;
      ROS_INFO_STREAM("URI IS " << uri);
      WebSocketClient::connection_ptr con = ws_client_.get_connection(uri, ec);
      if (ec) {
        ROS_ERROR_STREAM("WebSocket Error: " << ec.message());
        return;
      }
      
      ws_client_.connect(con);
      ws_client_.run();
  }

  void MiniBotHWInterface::on_open(websocketpp::connection_hdl hdl) {
      ws_hdl_ = hdl;
      ws_connected_ = true;
      ROS_INFO_STREAM("WebSocket connection opened.");
      const std::string inital_cmd = "ping"; 
      ws_client_.send(ws_hdl_, inital_cmd, websocketpp::frame::opcode::text);
  }

  void MiniBotHWInterface::on_message(websocketpp::connection_hdl hdl, WebSocketClient::message_ptr msg) {
      ROS_INFO_STREAM("Received message: " << msg->get_payload());
  }


  void MiniBotHWInterface::on_close(websocketpp::connection_hdl hdl) {
    ws_connected_ = false;
    ROS_ERROR_STREAM("WebSocket connection closed. Attempting to reconnect...");
  }

  void MiniBotHWInterface::write(ros::Duration& elapsed_time)
  {
    //ROS_INFO_STREAM("Ran write function"); 
    // Safety
    enforceLimits(elapsed_time);

    std::lock_guard<std::mutex> lock(command_mutex_);
    //ROS_INFO_STREAM("Mutex locked");
    // Write to serial for each joint
    std::string command;
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
      if (joint_velocity_[joint_id] != joint_velocity_command_[joint_id]) {
        // polymorphism FTW
        auto thisJoint = joints_[joint_names_[joint_id]];
        command += thisJoint->setVelocity(joint_velocity_command_[joint_id]); // + " "
        joint_velocity_[joint_id] = joint_velocity_command_[joint_id];
      }
      joint_position_[joint_id] += joint_velocity_command_[joint_id] * elapsed_time.toSec(); // if doing position control, velocity command MUST BE ZERO

      if (joint_position_[joint_id] != joint_position_command_[joint_id]) {
        // polymorphism FTW
        auto thisJoint = joints_[joint_names_[joint_id]];
        command += thisJoint->setPosition(joint_position_command_[joint_id]); // + " "
        joint_position_[joint_id] = joint_position_command_[joint_id];
      }
    }
    if(command.find_first_not_of(' ') != std::string::npos) {
      ROS_INFO_STREAM("Sending command " << command << ")"); // TODO replace with actual write
    }
    else {
      return;
    }

    if (use_websocket_) {
      if (!ws_connected_) {
        ROS_ERROR_STREAM_THROTTLE(2, "Websocket not connected");
      }
      // connected and using websockets
      else {
        ws_client_.send(ws_hdl_, command, websocketpp::frame::opcode::text);
      }
    }  
    else if (write_proto == 0) {
      ROS_WARN_STREAM_THROTTLE(2, "Using BT serial");
      p->write_some(boost::asio::buffer(command));
    }

  }

  void MiniBotHWInterface::enforceLimits(ros::Duration& period)
  {
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    // vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
  }

}  // namespace minibot_control
