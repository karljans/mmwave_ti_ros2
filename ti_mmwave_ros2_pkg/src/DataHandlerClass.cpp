/*
 * @file DataHandlerClass.cpp
 *
 * @brief
 * Handles and publishes incoming data from the sensor and .
 *
 * \par
 * NOTE:
 * (C) Copyright 2020 Texas Instruments, Inc.
 * ROS 2 Copyright 2022 Swimming Kim, Inc.
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

#include <iostream>
#include <iomanip>  // for std::fixed and std::setprecision
#include <sstream>  // for std::ostringstream
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"

#include "ti_mmwave_ros2_pkg/DataHandlerClass.hpp"

/* Default values */
static const std::string SRV_NODE_NAME = "mmWaveCommSrvNode";

DataUARTHandler::DataUARTHandler(): rclcpp::Node("DataUARTHandler"), currentBufp(&pingPongBuffers[0]),
      nextBufp(&pingPongBuffers[1]) {}

void DataUARTHandler::onInit()
{

  // Radar Service Client
  auto client_name = rclcpp::expand_topic_or_service_name(SRV_NODE_NAME, this->get_name(), this->get_namespace());
  RCLCPP_INFO(this->get_logger(), "Using parameter client: " + client_name);

  parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, client_name);

  // Wait for parameter service to be up
  while (!parameters_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }

  // Get parameters
  auto parameters_future = parameters_client->get_parameters(
      {
        "numAdcSamples", 
        "numLoops", 
        "num_TX", 
        "f_s", 
        "f_c", 
        "BW", 
        "PRI", 
        "t_fr",
        "max_range", 
        "range_resolution", 
        "max_doppler_vel",
        "doppler_vel_resolution"
      },
      std::bind(&DataUARTHandler::callbackGlobalParam, this, std::placeholders::_1));

  maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
  maxAllowedAzimuthAngleDeg = 90;   // Use max angle if none specified
}

// Setup topci publishers
void DataUARTHandler::setPublishers(
    const rclcpp::Publisher<PointCloud2>::SharedPtr DataUARTHandler_pub_in,
    const rclcpp::Publisher<RadarScan>::SharedPtr radar_scan_pub_in,
    const rclcpp::Publisher<Marker>::SharedPtr marker_pub_in)
{
  this->DataUARTHandler_pub = DataUARTHandler_pub_in;
  this->radar_scan_pub = radar_scan_pub_in;
  this->marker_pub = marker_pub_in;
}

// Read configuraiton
void DataUARTHandler::callbackGlobalParam(
    std::shared_future<std::vector<rclcpp::Parameter>> future)
{

  auto result = future.get();

  nr = result.at(0).as_int();
  nd = result.at(1).as_int();
  ntx = result.at(2).as_int();

  fs = static_cast<float>(result.at(3).as_double());
  fc = static_cast<float>(result.at(4).as_double());
  BW = static_cast<float>(result.at(5).as_double());
  PRI = static_cast<float>(result.at(6).as_double());
  tfr = static_cast<float>(result.at(7).as_double());
  max_range = static_cast<float>(result.at(8).as_double());
  vrange = static_cast<float>(result.at(9).as_double());
  max_vel = static_cast<float>(result.at(10).as_double());
  vvel = static_cast<float>(result.at(11).as_double());

  std::ostringstream oss;

  oss << "\n\n==============================\n"
        << "List of parameters\n"
        << "==============================\n"
        << "Number of range samples: " << nr << "\n"
        << "Number of chirps: " << nd << "\n"
        << "f_s: " << std::fixed << std::setprecision(3) << fs / 1e6 << " MHz\n"
        << "f_c: " << std::fixed << std::setprecision(3) << fc / 1e9 << " GHz\n"
        << "Bandwidth: " << std::fixed << std::setprecision(3) << BW / 1e6 << " MHz\n"
        << "PRI: " << std::fixed << std::setprecision(3) << PRI * 1e6 << " us\n"
        << "Frame time: " << std::fixed << std::setprecision(3) << tfr * 1e3 << " ms\n"
        << "Max range: " << std::fixed << std::setprecision(3) << max_range << " m\n"
        << "Range resolution: " << std::fixed << std::setprecision(3) << vrange << " m\n"
        << "Max Doppler: +-" << std::fixed << std::setprecision(3) << max_vel / 2 << " m/s\n"
        << "Doppler resolution: " << std::fixed << std::setprecision(3) << vvel << " m/s\n"
        << "==============================\n";

  std::string output = oss.str();
  RCLCPP_INFO(this->get_logger(), output);
}

void DataUARTHandler::setFrameID(char *myFrameID) { frameID = myFrameID; }

/*Implementation of setUARTPort*/
void DataUARTHandler::setUARTPort(char *serial_port)
{
  dataSerialPort = serial_port;
}

/*Implementation of setBaudRate*/
void DataUARTHandler::setBaudRate(int baudrate)
{
  dataBaudRate = baudrate;
}

/*Implementation of setMaxAllowedElevationAngleDeg*/
void DataUARTHandler::setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg)
{
  maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/*Implementation of setMaxAllowedAzimuthAngleDeg*/
void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg)
{
  maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}

/* Implementation of readIncomingData */
void *DataUARTHandler::readIncomingData(void)
{
  int firstPacketReady = 0;
  uint8_t last8Bytes[8] = {0};

  /* Open UART Port and error checking */
  serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(100));
  mySerialObject.setPort(dataSerialPort);

  try
  {
    mySerialObject.open();
  }
  
  catch (std::exception &e1)
  {
    printf("DataUARTHandler Read Thread: Failed to open Data serial port "
           "with error: %s", e1.what());

    printf("DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");

    try
    {
      // Wait 20 seconds and try to open serial port again
      rclcpp::sleep_for(std::chrono::seconds(20));
      mySerialObject.open();
    }

    catch (std::exception &e2)
    {
      //   TODO
        RCLCPP_FATAL(this->get_logger(), "DataUARTHandler Read Thread: Failed second time to open" 
                                        "Data serial port, error: %s", 
                                        e1.what());

        RCLCPP_FATAL(this->get_logger(), "DataUARTHandler Read Thread: Port could not be opened. "
                                         "Port is \"%s\" and baud rate is %d", 
                                         dataSerialPort, dataBaudRate);

      pthread_exit(NULL);
    }
  }

  if (mySerialObject.isOpen())
    RCLCPP_INFO(this->get_logger(), "DataUARTHandler Read Thread: Serial port is open");

  else
    RCLCPP_ERROR(this->get_logger(), "DataUARTHandler Read Thread: Serial port cannot be opened!");

  /* Quick magicWord check to synchronize program with data Stream */
  while (!isMagicWord(last8Bytes))
  {
    last8Bytes[0] = last8Bytes[1];
    last8Bytes[1] = last8Bytes[2];
    last8Bytes[2] = last8Bytes[3];
    last8Bytes[3] = last8Bytes[4];
    last8Bytes[4] = last8Bytes[5];
    last8Bytes[5] = last8Bytes[6];
    last8Bytes[6] = last8Bytes[7];
    mySerialObject.read(&last8Bytes[7], 1);
  }

  /* Lock nextBufp before entering main loop */
  pthread_mutex_lock(&nextBufp_mutex);

  while (rclcpp::ok())
  {
    /*Start reading UART data and writing to buffer while also checking for
     * magicWord*/
    last8Bytes[0] = last8Bytes[1];
    last8Bytes[1] = last8Bytes[2];
    last8Bytes[2] = last8Bytes[3];
    last8Bytes[3] = last8Bytes[4];
    last8Bytes[4] = last8Bytes[5];
    last8Bytes[5] = last8Bytes[6];
    last8Bytes[6] = last8Bytes[7];
    mySerialObject.read(&last8Bytes[7], 1);

    nextBufp->push_back(last8Bytes[7]); // push byte onto buffer

    /*If a magicWord is found wait for sorting to finish and switch buffers*/
    if (isMagicWord(last8Bytes))
    {

      /*Lock countSync Mutex while unlocking nextBufp so that the swap thread
       * can use it*/
      pthread_mutex_lock(&countSync_mutex);
      pthread_mutex_unlock(&nextBufp_mutex);

      /*increment countSync*/
      countSync++;

      /*If this is the first packet to be found, increment countSync again since
       * Sort thread is not reading data yet*/
      if (firstPacketReady == 0)
      {
        countSync++;
        firstPacketReady = 1;
      }

      /*Signal Swap Thread to run if countSync has reached its max value*/
      if (countSync == COUNT_SYNC_MAX)
      {
        pthread_cond_signal(&countSync_max_cv);
      }

      /*Wait for the Swap thread to finish swapping pointers and signal us to
       * continue*/
      pthread_cond_wait(&read_go_cv, &countSync_mutex);

      /*Unlock countSync so that Swap Thread can use it*/
      pthread_mutex_unlock(&countSync_mutex);
      pthread_mutex_lock(&nextBufp_mutex);

      nextBufp->clear();
      memset(last8Bytes, 0, sizeof(last8Bytes));
    }
  }

  mySerialObject.close();
  pthread_exit(NULL);
}

/* Check if magic word is valid */
int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8])
{
  int val = 0, i = 0, j = 0;
  for (i = 0; i < 8; i++)
  {
    if (last8Bytes[i] == magicWord[i])
    {
      j++;
    }
  }
  if (j == 8)
  {
    val = 1;
  }

  return val;
}

/* Thread-safe communication */
void *DataUARTHandler::syncedBufferSwap(void)
{
  while (rclcpp::ok())
  {
    pthread_mutex_lock(&countSync_mutex);

    while (countSync < COUNT_SYNC_MAX)
    {
      pthread_cond_wait(&countSync_max_cv, &countSync_mutex);

      pthread_mutex_lock(&currentBufp_mutex);
      pthread_mutex_lock(&nextBufp_mutex);

      std::vector<uint8_t> *tempBufp = currentBufp;

      this->currentBufp = this->nextBufp;

      this->nextBufp = tempBufp;

      pthread_mutex_unlock(&currentBufp_mutex);
      pthread_mutex_unlock(&nextBufp_mutex);

      countSync = 0;

      pthread_cond_signal(&sort_go_cv);
      pthread_cond_signal(&read_go_cv);
    }

    pthread_mutex_unlock(&countSync_mutex);
  }

  pthread_exit(NULL);
}

/* Processes incoming data */
void *DataUARTHandler::sortIncomingData(void)
{
  int j = 0;
  uint i = 0;
  uint tlvCount = 0;
  uint currentDatap = 0;

  uint32_t tlvLen = 0;
  uint32_t headerSize;
    
  float maxElevationAngleRatioSquared;
  float maxAzimuthAngleRatio;
  
  SorterState sorterState = READ_HEADER;
  MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;

  boost::shared_ptr<pcl::PointCloud<radar_pcl::PointXYZIVRB>> RScan(new pcl::PointCloud<radar_pcl::PointXYZIVRB>);

  sensor_msgs::msg::PointCloud2 output_pointcloud;
  ti_mmwave_ros2_interfaces::msg::RadarScan radarscan;

  // wait for first packet to arrive
  pthread_mutex_lock(&countSync_mutex);
  pthread_cond_wait(&sort_go_cv, &countSync_mutex);
  pthread_mutex_unlock(&countSync_mutex);

  pthread_mutex_lock(&currentBufp_mutex);

  /* Main loop */
  while (rclcpp::ok())
  {
    switch (sorterState)
    {

    case READ_HEADER:

      // init variables
      mmwData.numObjOut = 0;

      // make sure packet has at least first three fields (12 bytes) before we
      // read them (does not include magicWord since it was already removed)
      if (currentBufp->size() < 12)
      {
        sorterState = SWAP_BUFFERS;
        break;
      }

      // get version (4 bytes)
      memcpy(&mmwData.header.version, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.version));
      currentDatap += (sizeof(mmwData.header.version));

      // get totalPacketLen (4 bytes)
      memcpy(&mmwData.header.totalPacketLen, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.totalPacketLen));
      currentDatap += (sizeof(mmwData.header.totalPacketLen));

      // get platform (4 bytes)
      memcpy(&mmwData.header.platform, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.platform));
      currentDatap += (sizeof(mmwData.header.platform));

      // if packet doesn't have correct header size (which is based on
      // platform), throw it away
      //  (does not include magicWord since it was already removed)
      if ((mmwData.header.platform & 0xFFFF) == 0x1443) // platform is xWR1443)
      {
        headerSize = 7 * 4; // xWR1443 SDK demo header does not have subFrameNumber field
      }
      else
      {
        headerSize = 8 * 4; // header includes subFrameNumber field
      }
      if (currentBufp->size() < headerSize)
      {
        sorterState = SWAP_BUFFERS;
        break;
      }

      // get frameNumber (4 bytes)
      memcpy(&mmwData.header.frameNumber, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.frameNumber));
      currentDatap += (sizeof(mmwData.header.frameNumber));

      // get timeCpuCycles (4 bytes)
      memcpy(&mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.timeCpuCycles));
      currentDatap += (sizeof(mmwData.header.timeCpuCycles));

      // get numDetectedObj (4 bytes)
      memcpy(&mmwData.header.numDetectedObj, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.numDetectedObj));
      currentDatap += (sizeof(mmwData.header.numDetectedObj));

      // get numTLVs (4 bytes)
      memcpy(&mmwData.header.numTLVs, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.numTLVs));
      currentDatap += (sizeof(mmwData.header.numTLVs));

      // get subFrameNumber (4 bytes) (not used for XWR1443)
      if ((mmwData.header.platform & 0xFFFF) != 0x1443)
      {
        memcpy(&mmwData.header.subFrameNumber, &currentBufp->at(currentDatap),
               sizeof(mmwData.header.subFrameNumber));
        currentDatap += (sizeof(mmwData.header.subFrameNumber));
      }

      // if packet lengths do not match, throw it away
      if (mmwData.header.totalPacketLen == currentBufp->size())
      {
        sorterState = CHECK_TLV_TYPE;
      }
      else
        sorterState = SWAP_BUFFERS;
      break;

    case READ_OBJ_STRUCT:

      // CHECK_TLV_TYPE code has already read tlvType and tlvLen

      i = 0;
      if (((mmwData.header.version >> 24) & 0xFF) <
          3) // SDK version is older than 3.x
      {
        // get number of objects
        memcpy(&mmwData.numObjOut, &currentBufp->at(currentDatap),
               sizeof(mmwData.numObjOut));
        currentDatap += (sizeof(mmwData.numObjOut));

        // get xyzQFormat
        memcpy(&mmwData.xyzQFormat, &currentBufp->at(currentDatap),
               sizeof(mmwData.xyzQFormat));
        currentDatap += (sizeof(mmwData.xyzQFormat));
      }
      else // SDK version is at least 3.x
      {
        mmwData.numObjOut = mmwData.header.numDetectedObj;
      }

      // PCL pointcloud
      RScan->header.frame_id = frameID;
      RScan->height = 1;
      RScan->width = mmwData.numObjOut;
      RScan->is_dense = 1;
      RScan->points.resize(RScan->width * RScan->height);

      // Calculate ratios for max desired elevation and azimuth angles
      if ((maxAllowedElevationAngleDeg >= 0) &&
          (maxAllowedElevationAngleDeg < 90))
      {
        maxElevationAngleRatioSquared =
            tan(maxAllowedElevationAngleDeg * M_PI / 180.0);

        maxElevationAngleRatioSquared =
            maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
      }
      else
        maxElevationAngleRatioSquared = -1;

      if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90))
        maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);

      else
        maxAzimuthAngleRatio = -1;

      // Populate pointcloud
      while (i < mmwData.numObjOut)
      {
        if (((mmwData.header.version >> 24) & 0xFF) <
            3)
        { // SDK version is older than 3.x
          // get object range index
          memcpy(&mmwData.objOut.rangeIdx, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.rangeIdx));
          currentDatap += (sizeof(mmwData.objOut.rangeIdx));

          // get object doppler index
          memcpy(&mmwData.objOut.dopplerIdx, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.dopplerIdx));
          currentDatap += (sizeof(mmwData.objOut.dopplerIdx));

          // get object peak intensity value
          memcpy(&mmwData.objOut.peakVal, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.peakVal));
          currentDatap += (sizeof(mmwData.objOut.peakVal));

          // get object x-coordinate
          memcpy(&mmwData.objOut.x, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.x));
          currentDatap += (sizeof(mmwData.objOut.x));

          // get object y-coordinate
          memcpy(&mmwData.objOut.y, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.y));
          currentDatap += (sizeof(mmwData.objOut.y));

          // get object z-coordinate
          memcpy(&mmwData.objOut.z, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.z));

          currentDatap += (sizeof(mmwData.objOut.z));

          float temp[8];

          temp[0] = (float)mmwData.objOut.x;
          temp[1] = (float)mmwData.objOut.y;
          temp[2] = (float)mmwData.objOut.z;
          temp[3] = (float)mmwData.objOut.dopplerIdx;

          for (int j = 0; j < 4; j++)
          {
            if (temp[j] > 32767)
            {
              temp[j] -= 65536;
            }
            if (j < 3)
            {
              temp[j] = temp[j] / pow(2, mmwData.xyzQFormat);
            }
          }

          temp[7] = temp[3] * vvel;

          temp[4] = (float)mmwData.objOut.rangeIdx * vrange;
          temp[5] = 10 * log10(mmwData.objOut.peakVal + 1); // intensity
          temp[6] = std::atan2(-temp[0], temp[1]) / M_PI * 180;

          uint16_t tmp = (uint16_t)(temp[3] + nd / 2);

          // Map mmWave sensor coordinates to ROS coordinate system PCL Pointcloud
          // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
          RScan->points[i].x = temp[1];
          // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
          RScan->points[i].y = -temp[0];
          // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
          RScan->points[i].z = temp[2];

          RScan->points[i].intensity = temp[5];
          RScan->points[i].velocity = temp[7];
          RScan->points[i].range = temp[4];
          RScan->points[i].bearing = temp[6];
          
          // Custom radarscan message
          radarscan.header.frame_id = frameID;
          radarscan.header.stamp = rclcpp::Clock().now();

          radarscan.point_id = i;
          radarscan.x = temp[1];
          radarscan.y = -temp[0];
          radarscan.z = temp[2];
          radarscan.range = temp[4];
          radarscan.velocity = temp[7];
          radarscan.doppler_bin = tmp;
          radarscan.bearing = temp[6];
          radarscan.intensity = temp[5];
        }

        else
        { // SDK version is 3.x+
          // get object x-coordinate (meters)
          memcpy(&mmwData.objOut_cartes.x, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut_cartes.x));
          currentDatap += (sizeof(mmwData.objOut_cartes.x));

          // get object y-coordinate (meters)
          memcpy(&mmwData.objOut_cartes.y, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut_cartes.y));
          currentDatap += (sizeof(mmwData.objOut_cartes.y));

          // get object z-coordinate (meters)
          memcpy(&mmwData.objOut_cartes.z, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut_cartes.z));
          currentDatap += (sizeof(mmwData.objOut_cartes.z));

          // get object velocity (m/s)
          memcpy(&mmwData.objOut_cartes.velocity, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut_cartes.velocity));
          currentDatap += (sizeof(mmwData.objOut_cartes.velocity));

          // Calculate range (distance) using the pythagorean theorem
          float point_range = sqrt(radarscan.x * radarscan.x + radarscan.y * radarscan.y + radarscan.z * radarscan.z);

          // Calculate bearing
          float point_bearing = std::atan2(-mmwData.objOut_cartes.x, mmwData.objOut_cartes.y) / M_PI * 180;

          // Map mmWave sensor coordinates to ROS coordinate system
          // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
          RScan->points[i].x = mmwData.objOut_cartes.y;

          // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
          RScan->points[i].y = -mmwData.objOut_cartes.x;

          // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
          RScan->points[i].z = mmwData.objOut_cartes.z;

          // Velocity in m/s
          RScan->points[i].velocity = mmwData.objOut_cartes.velocity;

          // Range in m
          RScan->points[i].range = point_range;
          
          // Bearing on degrees
          RScan->points[i].bearing = point_bearing;

          // Set frame ID
          radarscan.header.frame_id = frameID;
          radarscan.header.stamp = rclcpp::Clock().now();

          radarscan.point_id = i;
          radarscan.x = mmwData.objOut_cartes.y;
          radarscan.y = -mmwData.objOut_cartes.x;
          radarscan.z = mmwData.objOut_cartes.z;
          radarscan.velocity = mmwData.objOut_cartes.velocity;
          radarscan.range = point_range;

          radarscan.doppler_bin = (uint16_t)(mmwData.detList.dopplerIdx + nd / 2);

          radarscan.bearing = point_bearing;

          radarscan.intensity = (float)mmwData.sideInfo.snr / 10.0;

          // For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed
          // in the READ_SIDE_INFO code
        }

        if (((maxElevationAngleRatioSquared == -1) || (((RScan->points[i].z * RScan->points[i].z) /
                                                        (RScan->points[i].x * RScan->points[i].x + RScan->points[i].y * RScan->points[i].y)) <
                                                       maxElevationAngleRatioSquared)) &&
            ((maxAzimuthAngleRatio == -1) ||
             (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
            (RScan->points[i].x != 0))
        {
          radar_scan_pub->publish(radarscan);
          this->visualize(radarscan);
        }
        i++;
      }

      sorterState = CHECK_TLV_TYPE;

      break;

    case READ_SIDE_INFO:

      // Make sure we already received and parsed detected obj list
      // (READ_OBJ_STRUCT)
      if (mmwData.numObjOut > 0)
      {
        for (i = 0; i < mmwData.numObjOut; i++)
        {
          // get snr (unit is 0.1 steps of dB)
          memcpy(&mmwData.sideInfo.snr, &currentBufp->at(currentDatap),
                 sizeof(mmwData.sideInfo.snr));
          currentDatap += (sizeof(mmwData.sideInfo.snr));

          // get noise (unit is 0.1 steps of dB)
          memcpy(&mmwData.sideInfo.noise, &currentBufp->at(currentDatap),
                 sizeof(mmwData.sideInfo.noise));
          currentDatap += (sizeof(mmwData.sideInfo.noise));

          // Use snr for "intensity" field (divide by 10 since unit of snr is 0.1dB)
          RScan->points[i].intensity = (float)mmwData.sideInfo.snr / 10.0;
        }
      }

      else // else just skip side info section if we have not already received
           // and parsed detected obj list
      {
        i = 0;

        while (i++ < tlvLen - 1)
        {
        }
        currentDatap += tlvLen;
      }
      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_LOG_MAG_RANGE:
      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_NOISE:

      i = 0;

      // Do nothing until the condition is met
      while (i++ < tlvLen - 1){}

      currentDatap += tlvLen;
      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_AZIMUTH:

      i = 0;

      // Do nothing until the condition is met
      while (i++ < tlvLen - 1){}

      currentDatap += tlvLen;
      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_DOPPLER:

      i = 0;

      // Do nothing until the condition is met
      while (i++ < tlvLen - 1){}

      currentDatap += tlvLen;
      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_STATS:

      i = 0;

      // Do nothing until the condition is met
      while (i++ < tlvLen - 1){}

      currentDatap += tlvLen;
      sorterState = CHECK_TLV_TYPE;
      break;

    case CHECK_TLV_TYPE:
      if (tlvCount++ >=
          mmwData.header.numTLVs) // Done parsing all received TLV sections
      {
        // Publish detected object pointcloud
        if (mmwData.numObjOut > 0)
        {
          j = 0;
          for (i = 0; i < mmwData.numObjOut; i++)
          {
            // Keep point if elevation and azimuth angles are less than
            // specified max values (NOTE: The following calculations are done
            // using ROS standard coordinate system axis definitions where X is
            // forward and Y is left)
            if (((maxElevationAngleRatioSquared == -1) || (((RScan->points[i].z * RScan->points[i].z) /
                                                            (RScan->points[i].x * RScan->points[i].x + RScan->points[i].y * RScan->points[i].y)) <
                                                           maxElevationAngleRatioSquared)) &&
                ((maxAzimuthAngleRatio == -1) ||
                 (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                (RScan->points[i].x != 0))
            {
              memcpy(&RScan->points[j], &RScan->points[i], sizeof(RScan->points[i]));
              j++;
            }
          }
          mmwData.numObjOut = j; // update number of objects as some points may
                                 // have been removed

          // Resize point cloud since some points may have been removed
          RScan->width = mmwData.numObjOut;
          RScan->points.resize(RScan->width * RScan->height);

          pcl::PCLPointCloud2 cloud_ROI;
          pcl::toPCLPointCloud2(*RScan, cloud_ROI);
          pcl_conversions::fromPCL(cloud_ROI, output_pointcloud);
          DataUARTHandler_pub->publish(output_pointcloud);
        }
        sorterState = SWAP_BUFFERS;
      }

      else // More TLV sections to parse
      {
        // get tlvType (32 bits) & remove from queue
        memcpy(&tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
        currentDatap += (sizeof(tlvType));

        // get tlvLen (32 bits) & remove from queue
        memcpy(&tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
        currentDatap += (sizeof(tlvLen));

        switch (tlvType)
        {
        case MMWDEMO_OUTPUT_MSG_NULL:
          break;

        case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
          sorterState = READ_OBJ_STRUCT;
          break;

        case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
          sorterState = READ_LOG_MAG_RANGE;
          break;

        case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
          sorterState = READ_NOISE;
          break;

        case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
          sorterState = READ_AZIMUTH;
          break;

        case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
          sorterState = READ_DOPPLER;
          break;

        case MMWDEMO_OUTPUT_MSG_STATS:
          sorterState = READ_STATS;
          break;

        case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
          sorterState = READ_SIDE_INFO;
          break;

        case MMWDEMO_OUTPUT_MSG_MAX:
          sorterState = READ_HEADER;
          break;

        default:
          break;
        }
      }

      break;

    case SWAP_BUFFERS:

      pthread_mutex_lock(&countSync_mutex);
      pthread_mutex_unlock(&currentBufp_mutex);

      countSync++;

      if (countSync == COUNT_SYNC_MAX)
      {
        pthread_cond_signal(&countSync_max_cv);
      }

      pthread_cond_wait(&sort_go_cv, &countSync_mutex);

      pthread_mutex_unlock(&countSync_mutex);
      pthread_mutex_lock(&currentBufp_mutex);

      currentDatap = 0;
      tlvCount = 0;

      sorterState = READ_HEADER;

      break;

    default:
      break;
    }
  }

  pthread_exit(NULL);
}

void DataUARTHandler::start(void)
{

  pthread_t uartThread, sorterThread, swapThread;

  int iret1, iret2, iret3;

  pthread_mutex_init(&countSync_mutex, NULL);
  pthread_mutex_init(&nextBufp_mutex, NULL);
  pthread_mutex_init(&currentBufp_mutex, NULL);
  pthread_cond_init(&countSync_max_cv, NULL);
  pthread_cond_init(&read_go_cv, NULL);
  pthread_cond_init(&sort_go_cv, NULL);

  countSync = 0;

  /* Create independent threads each of which will execute function */
  iret1 = pthread_create(&uartThread, NULL, this->readIncomingData_helper, this);
  if (iret1)
  {
    RCLCPP_FATAL(this->get_logger(), "Error - pthread_create() return code:" + iret1);
    rclcpp::shutdown();
  }

  iret2 = pthread_create(&sorterThread, NULL, this->sortIncomingData_helper, this);
  if (iret2)
  {
    RCLCPP_FATAL(this->get_logger(), "Error - pthread_create() return code:" + iret2);
    rclcpp::shutdown();
  }

  iret3 = pthread_create(&swapThread, NULL, this->syncedBufferSwap_helper, this);
  if (iret3)
  {
    RCLCPP_FATAL(this->get_logger(), "Error - pthread_create() return code:" + iret3);
    rclcpp::shutdown();
  }

  // Wait until we should shut down
  while (rclcpp::ok())
    continue;

  // Wait for threads to finish
  pthread_join(iret1, NULL);
  RCLCPP_INFO(this->get_logger(), "DataUARTHandler Read Thread joined");

  pthread_join(iret2, NULL);
  RCLCPP_INFO(this->get_logger(), "DataUARTHandler Sort Thread joined");

  pthread_join(iret3, NULL);
  RCLCPP_INFO(this->get_logger(), "DataUARTHandler Swap Thread joined");

  pthread_mutex_destroy(&countSync_mutex);
  pthread_mutex_destroy(&nextBufp_mutex);
  pthread_mutex_destroy(&currentBufp_mutex);
  pthread_cond_destroy(&countSync_max_cv);
  pthread_cond_destroy(&read_go_cv);
  pthread_cond_destroy(&sort_go_cv);

  rclcpp::shutdown();
}


void *DataUARTHandler::readIncomingData_helper(void *context)
{
  return (static_cast<DataUARTHandler *>(context)->readIncomingData());
}

void *DataUARTHandler::sortIncomingData_helper(void *context)
{
  return (static_cast<DataUARTHandler *>(context)->sortIncomingData());
}

void *DataUARTHandler::syncedBufferSwap_helper(void *context)
{
  return (static_cast<DataUARTHandler *>(context)->syncedBufferSwap());
}

/* Radar point visualization markers */
void DataUARTHandler::visualize(const ti_mmwave_ros2_interfaces::msg::RadarScan &msg)
{
  auto marker = visualization_msgs::msg::Marker();

  marker.header.frame_id = frameID;
  marker.header.stamp = rclcpp::Clock().now();
  marker.id = msg.point_id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.lifetime = rclcpp::Duration(tfr, 0);
  marker.action = marker.ADD;

  marker.pose.position.x = msg.x;
  marker.pose.position.y = msg.y;
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;

  marker.scale.x = .03;
  marker.scale.y = .03;
  marker.scale.z = .03;

  marker.color.a = 1;
  marker.color.r = (int)255 * msg.intensity;
  marker.color.g = (int)255 * msg.intensity;
  marker.color.b = 1;

  this->marker_pub->publish(marker);
}
