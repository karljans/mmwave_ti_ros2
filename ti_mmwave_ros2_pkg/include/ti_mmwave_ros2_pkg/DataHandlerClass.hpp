#ifndef _DATA_HANDLER_CLASS_
#define _DATA_HANDLER_CLASS_

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <pthread.h>

#include <boost/shared_ptr.hpp>

#include "point_types.hpp"

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <visualization_msgs/msg/marker.hpp>

#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "ti_mmwave_ros2_pkg/mmWave.hpp"
#include "ti_mmwave_ros2_pkg/visibility_control.hpp"
#include "ti_mmwave_ros2_interfaces/msg/radar_scan.hpp"

#define COUNT_SYNC_MAX 2

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using RadarScan = ti_mmwave_ros2_interfaces::msg::RadarScan;
using Marker = visualization_msgs::msg::Marker;

class DataUARTHandler : public rclcpp::Node
{

public:
  COMPOSITION_PUBLIC
  DataUARTHandler();

  COMPOSITION_PUBLIC
  void setPublishers(
      const rclcpp::Publisher<PointCloud2>::SharedPtr DataUARTHandler_pub_in,
      const rclcpp::Publisher<RadarScan>::SharedPtr radar_scan_pub_in,
      const rclcpp::Publisher<Marker>::SharedPtr marker_pub_in);

  void onInit();

  void setNamespace(const std::string &ns);

  void setFrameID(char *myFrameID);

  /*User callable function to set the UARTPort*/
  void setUARTPort(char *serial_port);

  /*User callable function to set the BaudRate*/
  void setBaudRate(int baudrate);

  /*User callable function to set maxAllowedElevationAngleDeg*/
  void setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg);

  /*User callable function to set maxAllowedElevationAngleDeg*/
  void setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg);

  /*User callable function to start the handler's internal threads*/
  void start(void);

  /*Helper functions to allow pthread compatability*/
  static void *readIncomingData_helper(void *context);

  static void *sortIncomingData_helper(void *context);

  static void *syncedBufferSwap_helper(void *context);

  void callbackGlobalParam(
      std::shared_future<std::vector<rclcpp::Parameter>> future);

  /*Sorted mmwDemo Data structure*/
  mmwDataPacket mmwData;

private:
  int nr;
  int nd;
  int ntx;
  float fs;
  float fc;
  float BW;
  float PRI;
  float tfr;
  float max_range;
  float vrange;
  float max_vel;
  float vvel;

  char *frameID;
  /*Contains the name of the serial port*/
  char *dataSerialPort;

  /*Contains the baud Rate*/
  int dataBaudRate;

  /*Contains the max_allowed_elevation_angle_deg (points with elevation angles
    outside +/- max_allowed_elevation_angle_deg will be removed)*/
  int maxAllowedElevationAngleDeg;

  /*Contains the max_allowed_azimuth_angle_deg (points with azimuth angles
    outside +/- max_allowed_azimuth_angle_deg will be removed)*/
  int maxAllowedAzimuthAngleDeg;

  /*Mutex protected variable which synchronizes threads*/
  int countSync;

  /*Read/Write Buffers*/
  std::vector<uint8_t> pingPongBuffers[2];

  /*Pointer to current data (sort)*/
  std::vector<uint8_t> *currentBufp;

  /*Pointer to new data (read)*/
  std::vector<uint8_t> *nextBufp;

  /*Mutex protecting the countSync variable */
  pthread_mutex_t countSync_mutex;

  /*Mutex protecting the nextBufp pointer*/
  pthread_mutex_t nextBufp_mutex;

  /*Mutex protecting the currentBufp pointer*/
  pthread_mutex_t currentBufp_mutex;

  /*Condition variable which blocks the Swap Thread until signaled*/
  pthread_cond_t countSync_max_cv;

  /*Condition variable which blocks the Read Thread until signaled*/
  pthread_cond_t read_go_cv;

  /*Condition variable which blocks the Sort Thread until signaled*/
  pthread_cond_t sort_go_cv;

  /*Swap Buffer Pointers Thread*/
  void *syncedBufferSwap(void);

  /*Checks if the magic word was found*/
  int isMagicWord(uint8_t last8Bytes[8]);

  /*Read incoming UART Data Thread*/
  void *readIncomingData(void);

  /*Sort incoming UART Data Thread*/
  void *sortIncomingData(void);

  void visualize(const RadarScan &msg);

  rclcpp::Publisher<PointCloud2>::SharedPtr DataUARTHandler_pub;
  rclcpp::Publisher<RadarScan>::SharedPtr radar_scan_pub;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub;

  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
};

#endif
