/*
* Copyright 2022 LeoDrive.ai, Inc. All rights reserved.
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


#include "lucid_vision_driver/arena_camera.h"
#include "lucid_vision_driver/arena_camera_node.h"
#include <rclcpp/rclcpp.hpp>

ArenaCamera::ArenaCamera(Arena::IDevice * device, CameraSetting & camera_setting)
: m_device(device),
  m_camera_name(camera_setting.get_camera_name()),
  m_frame_id(camera_setting.get_frame_id()),
  m_pixel_format(camera_setting.get_pixel_format()),
  m_serial_no(camera_setting.get_serial_no()),
  m_fps(camera_setting.get_fps()),
  m_horizontal_binning(camera_setting.get_horizontal_binning()),
  m_vertical_binning(camera_setting.get_vertical_binning())
{
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

ArenaCamera::ArenaCamera(
  Arena::IDevice * device, std::string & camera_name, std::string & frame_id,
  std::string & pixel_format, uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning,
  uint32_t vertical_binning)
: m_device(device),
  m_camera_name(camera_name),
  m_frame_id(frame_id),
  m_pixel_format(pixel_format),
  m_serial_no(serial_no),
  m_fps(fps),
  m_horizontal_binning(horizontal_binning),
  m_vertical_binning(vertical_binning)
{
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

std::thread ArenaCamera::start_stream()
{
  return std::thread([=] { this->acquisition(); });
}

void ArenaCamera::acquisition()
{
  auto node_map = m_device->GetNodeMap();
  std::cout << "Camera idx:" << m_cam_idx << " acquisition thread." << std::endl;

  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "AcquisitionMode", "Continuous");
  
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");

  // Enable stream auto negotiate packet size
  Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

  // Enable stream packet resend
  Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

  /*GenApi::CIntegerPtr pBinningHorizontal = m_device->GetNodeMap()->GetNode("BinningHorizontal");
  if (GenApi::IsWritable(pBinningHorizontal)) {
    size_t binning_x_to_set = m_horizontal_binning;
    if (binning_x_to_set < pBinningHorizontal->GetMin()) {
      binning_x_to_set = pBinningHorizontal->GetMin();
    } else if (binning_x_to_set > pBinningHorizontal->GetMax()) {
      binning_x_to_set = pBinningHorizontal->GetMax();
    }
    pBinningHorizontal->SetValue(binning_x_to_set);
    m_reached_horizontal_binning = pBinningHorizontal->GetValue();
    if (m_reached_horizontal_binning != m_horizontal_binning) {
      RCLCPP_INFO(
        rclcpp::get_logger("ARENA_CAMERA"),
        "Not possible to use hardware based binning for horizontal binning:%d , software binning "
        "will be used.",
        m_horizontal_binning);
    }
  } else {
    std::cout << "Binning horizantal value not readable." << std::endl;
  }*/

  /*GenApi::CIntegerPtr pBinningVertical = m_device->GetNodeMap()->GetNode("BinningVertical");
  if (GenApi::IsWritable(pBinningVertical)) {
    size_t binning_y_to_set = m_vertical_binning;
    if (binning_y_to_set < pBinningVertical->GetMin()) {
      binning_y_to_set = pBinningVertical->GetMin();
    } else if (binning_y_to_set > pBinningVertical->GetMax()) {
      binning_y_to_set = pBinningVertical->GetMax();
    }
    pBinningHorizontal->SetValue(binning_y_to_set);
    m_reached_vertical_binning = pBinningVertical->GetValue();
    if (m_reached_vertical_binning != m_vertical_binning) {
      RCLCPP_INFO(
        rclcpp::get_logger("ARENA_CAMERA"),
        "Not possible to use hardware based binning for vertical binning:%d , software binning "
        "will be used.",
        m_horizontal_binning);
    }
  } else {
    std::cout << "Binning vertical value not readable." << std::endl;
  }*/

  m_device->StartStream();
}

void ArenaCamera::stop_stream() { 

//std::cout<<(m_device==nullptr)<<std::endl;

//if(m_device!=nullptr)
//std::cout<<"NO SOY NULL"<<std::endl;
m_signal_publish_image = nullptr; // AQUI ESTA EL PROBLEMA y LA SOLUCION!!
//std::cout<<"Antes"<<std::endl;
m_device->StopStream(); 
//std::cout<<"Despues"<<std::endl;

}

void ArenaCamera::destroy_device(Arena::ISystem * system)
{
  if (m_device != nullptr) {
    system->DestroyDevice(m_device);
  }
}

void ArenaCamera::set_on_image_callback(ImageCallbackFunction callback)
{
  m_signal_publish_image = std::move(callback);
}

cv::Mat ArenaCamera::convert_to_image(Arena::IImage * pImage, const std::string & frame_id)
{
  if(dim_received==false) {
    img_height = pImage->GetHeight();
    img_width = pImage->GetWidth();
    dim_received=true;
  }
  
  //cv::Mat img(img_height, img_width, CV_8UC1, (uint8_t *)pImage->GetData());
  
  //m_device->RequeueBuffer(pImage); ERROR!

  //if(ArenaCameraNode::continueGrabbingImgs)
  //if(pImage!=nullptr)
  return cv::Mat(img_height, img_width, CV_8UC1, (uint8_t *)pImage->GetData());
  //else
  //return cv::Mat::zeros(img_height, img_width, CV_8UC1);
  //return img;
}

ArenaCamera::~ArenaCamera()
{
  std::cout << "Camera stopping:" << m_cam_idx << " ~ArenaCamera()" << std::endl;
  //stop_stream();
  //std::cout << "Camera stopped:" << std::endl;
  //destroy_device(Arena::ISystem * system) // It cant be called from here
}
