// Copyright 2023 Evan Flynn
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Evan Flynn nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef USB_CAM__FORMATS__PIXEL_FORMAT_BASE_HPP_
#define USB_CAM__FORMATS__PIXEL_FORMAT_BASE_HPP_

#include <string>

#include "linux/videodev2.h"

#include "usb_cam/constants.hpp"


namespace usb_cam
{
namespace formats
{


/// @brief Base pixel format class. Provide all necessary information for converting between
/// V4L2 and ROS formats. Meant to be overridden if conversion function is required.
class pixel_format_base
{
public:
  pixel_format_base(
    std::string name, uint32_t v4l2, std::string ros,
    uint8_t channels, uint8_t bit_depth, bool requires_conversion)
  : m_name(name),
    m_v4l2(v4l2),
    m_ros(ros),
    m_channels(channels),
    m_bit_depth(bit_depth),
    m_requires_conversion(requires_conversion)
  {}

  inline std::string name() {return m_name;}
  inline uint32_t v4l2() {return m_v4l2;}
  inline std::string ros() {return m_ros;}
  inline uint8_t channels() {return m_channels;}
  inline uint8_t bit_depth() {return m_bit_depth;}
  inline bool requires_conversion() {return m_requires_conversion;}

  /// @brief Conversion method. Meant to be overridden if pixel format requires it.
  virtual void convert(const char * & src, char * & dest, const int & bytes_used)
  {
    // provide default implementation so derived classes do not have to implement
    // this method if not required
    (void)src;
    (void)dest;
    (void)bytes_used;
  }

  inline bool is_color()
  {
    return
      m_ros == usb_cam::constants::RGB8 ||
      m_ros == usb_cam::constants::BGR8 ||
      m_ros == usb_cam::constants::RGBA8 ||
      m_ros == usb_cam::constants::BGRA8 ||
      m_ros == usb_cam::constants::RGB16 ||
      m_ros == usb_cam::constants::BGR16 ||
      m_ros == usb_cam::constants::RGBA16 ||
      m_ros == usb_cam::constants::BGRA16 ||
      m_ros == usb_cam::constants::NV21 ||
      m_ros == usb_cam::constants::NV24;
  }

  inline bool is_mono()
  {
    return
      m_ros == usb_cam::constants::MONO8 ||
      m_ros == usb_cam::constants::MONO16;
  }

  inline bool is_bayer()
  {
    return
      m_ros == usb_cam::constants::BAYER_RGGB8 ||
      m_ros == usb_cam::constants::BAYER_BGGR8 ||
      m_ros == usb_cam::constants::BAYER_GBRG8 ||
      m_ros == usb_cam::constants::BAYER_GRBG8 ||
      m_ros == usb_cam::constants::BAYER_RGGB16 ||
      m_ros == usb_cam::constants::BAYER_BGGR16 ||
      m_ros == usb_cam::constants::BAYER_GBRG16 ||
      m_ros == usb_cam::constants::BAYER_GRBG16;
  }

  inline bool has_alpha()
  {
    return
      m_ros == usb_cam::constants::RGBA8 ||
      m_ros == usb_cam::constants::BGRA8 ||
      m_ros == usb_cam::constants::RGBA16 ||
      m_ros == usb_cam::constants::BGRA16;
  }

protected:
  /// @brief Unique name for this pixel format
  std::string m_name;
  /// @brief Integer correspoding to a specific V4L2_PIX_FMT_* constant
  /// See `linux/videodev2.h` for a list of all possible values for here
  uint32_t m_v4l2;
  // TODO(flynneva): make this a vector of supported conversions for the specified V4L2 format
  /// @brief This should match ROS encoding string
  /// See `sensor_msgs/image_encodings.hpp` for corresponding possible values. Copy of
  /// those values are stored in `usb_cam/constants.hpp`
  std::string m_ros;
  /// @brief Number of channels (aka bytes per pixel) of output (ROS format above)
  uint8_t m_channels;
  /// @brief Bitdepth of output (ROS format above)
  uint8_t m_bit_depth;
  /// @brief boolean whether or not the current format requires a call to `convert`.
  /// Setting this to true requires that the virtual `convert` method is implemented.
  bool m_requires_conversion;
};


class default_pixel_format : public pixel_format_base
{
public:
  default_pixel_format()
  : pixel_format_base(
      "yuyv",
      V4L2_PIX_FMT_YUYV,
      usb_cam::constants::YUV422_YUY2,
      2,
      8,
      false)
  {}
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__PIXEL_FORMAT_BASE_HPP_
