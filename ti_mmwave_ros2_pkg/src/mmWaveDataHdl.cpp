/*
 * @file mmWaveDataHdl.cpp
 *
 * @brief
 * Creates the data handler node and sets parameters.
 *
 * \par
 * NOTE:
 * (C) Copyright 2020 Texas Instruments, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "rclcpp/exceptions.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "ti_mmwave_ros2_pkg/DataHandlerClass.hpp"
#include "ti_mmwave_ros2_pkg/mmWaveDataHdl.hpp"

// Default values
static const std::string DEFAULT_DATA_PORT = "/dev/ttyACM1";
static const int DEFAULT_DATA_RATE = 921600;
static const std::string DEFAULT_FRAME_ID = "ti_mmwave";

namespace ti_mmwave_ros2_pkg
{

    mmWaveDataHdl::mmWaveDataHdl(const rclcpp::NodeOptions &options)
        : Node("mmWaveDataHdl", options)
    {
        onInit();
    }

    rclcpp::QoS mmWaveDataHdl::createQoSProfile()
    {
        // Set the history depth to 1 to keep only the latest message
        size_t depth = 1;

        // Set the reliability to reliable to ensure message delivery
        rmw_qos_reliability_policy_t reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

        // Set the durability to transient local to allow late-joining subscribers to receive the last message
        rmw_qos_durability_policy_t durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

        // Set the liveliness to automatic with a lease duration of 1 second (adjust if needed)
        rmw_qos_liveliness_policy_t liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
        rmw_time_t liveliness_lease_duration = {1, 0}; // 1 second

        // Avoid ROS namespace conventions
        bool avoid_ros_namespace_conventions = false;

        // Create the QoS profile with the desired settings
        rclcpp::QoS qos_profile = rclcpp::QoS(depth);
        qos_profile.reliability(reliability);
        qos_profile.durability(durability);
        qos_profile.liveliness(liveliness);
        qos_profile.liveliness_lease_duration(liveliness_lease_duration);
        qos_profile.avoid_ros_namespace_conventions(avoid_ros_namespace_conventions);

        return qos_profile;
    }

    void mmWaveDataHdl::onInit()
    {
        std::string serial_port;
        std::string frame_id;
        std::string ns;

        int baudrate;
        int max_allowed_elevation_angle_deg;
        int max_allowed_azimuth_angle_deg;

        // Parameter declaration
        serial_port = this->declare_parameter("data_port", DEFAULT_DATA_PORT);
        baudrate = this->declare_parameter("data_rate", DEFAULT_DATA_RATE);
        frame_id = this->declare_parameter("frame_id", DEFAULT_FRAME_ID);

        max_allowed_elevation_angle_deg = this->declare_parameter("max_allowed_elevation_angle_deg", 90);
        max_allowed_azimuth_angle_deg = this->declare_parameter("max_allowed_azimuth_angle_deg", 90);

        RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: data_port = %s",
                    serial_port.c_str());

        RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: data_rate = %d", baudrate);
        RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: max_allowed_elevation_angle_deg = %d",
                    max_allowed_elevation_angle_deg);
        RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d",
                    max_allowed_azimuth_angle_deg);

        // Calculate expanded topic names
        auto pcl_topic = rclcpp::expand_topic_or_service_name("ti_mmwave/radar_scan_pcl",
                                                              this->get_name(), this->get_namespace());

        auto radar_scan_topic = rclcpp::expand_topic_or_service_name("ti_mmwave/radar_scan", this->get_name(),
                                                                     this->get_namespace());

        auto marker_topic = rclcpp::expand_topic_or_service_name("ti_mmwave/radar_scan_markers",
                                                                 this->get_name(), this->get_namespace());

        RCLCPP_INFO(this->get_logger(), "Radar PCL topic: " + pcl_topic);
        RCLCPP_INFO(this->get_logger(), "Radarscan topic: " + radar_scan_topic);
        RCLCPP_INFO(this->get_logger(), "Marker topic: " + marker_topic);

        // Create a QoS profile using the sensor message preset
        // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

        auto sensor_qos_profile = createQoSProfile(); //rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile));
        // sensor_qos_profile.get_rmw_qos_profile() = qos_profile;

        // Setup publishers
        auto DataUARTHandler_pub = create_publisher<PointCloud2>(pcl_topic, sensor_qos_profile);
        auto radar_scan_pub = create_publisher<RadarScan>(radar_scan_topic, sensor_qos_profile);
        auto marker_pub = create_publisher<Marker>(marker_topic, sensor_qos_profile);

        // Initialize the data handler class
        DataHandler = std::make_shared<DataUARTHandler>();

        DataHandler->onInit();
        DataHandler->setPublishers(DataUARTHandler_pub, radar_scan_pub, marker_pub);

        DataHandler->setFrameID((char *)frame_id.c_str());
        DataHandler->setUARTPort((char *)serial_port.c_str());
        DataHandler->setBaudRate(baudrate);
        DataHandler->setMaxAllowedElevationAngleDeg(max_allowed_elevation_angle_deg);
        DataHandler->setMaxAllowedAzimuthAngleDeg(max_allowed_azimuth_angle_deg);

        // Start the node
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        rclcpp::spin_some(DataHandler);

        // Start data handler
        DataHandler->start();
        RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: Finished onInit function");
    }
}

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ti_mmwave_ros2_pkg::mmWaveDataHdl)
