/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "ffmpeg_image_transport_msgs/FFMPEGPacket.h"
#include "ffmpeg_image_transport/tdiff.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <unordered_map>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avio.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
}


namespace ffmpeg_image_transport {
  using Image = sensor_msgs::Image;
  using ImagePtr = sensor_msgs::ImagePtr;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  using FFMPEGPacket = ffmpeg_image_transport_msgs::FFMPEGPacket;
  typedef std::unordered_map<int64_t, ros::Time> PTSMap;

  class FFMPEGDecoder {
  public:
    FFMPEGDecoder();
    ~FFMPEGDecoder();
    bool isInitialized() const { return (codecContext_ != NULL); }
    // Initialize decoder upon first packet received,
    // providing callback to be called when frame is complete.
    // You must still call decodePacket(msg) afterward!
    bool initialize(const FFMPEGPacket::ConstPtr& msg,
                    boost::function<void(const ImageConstPtr &img)> callback);
    // clears all state, but leaves config intact
    void reset();
    // decode packet (may result in frame callback!)
    bool decodePacket(const FFMPEGPacket::ConstPtr &msg);
  private:
    bool initDecoder(int width, int height, const std::string &codecName);
    // --------- variables
    boost::function<void(const ImageConstPtr &img)> callback_;
    // libav stuff
    std::string       encoding_;
    AVCodecContext   *codecContext_{NULL};
    AVFrame          *decodedFrame_{NULL};
    AVFrame          *colorFrame_{NULL};
    SwsContext       *swsContext_{NULL};
    std::unordered_map<std::string, std::string> codecMap_;
    AVPacket          packet_;
    // mapping of header
    PTSMap            ptsToStamp_;
    // performance analysis
    unsigned int      frameCnt_{0};
    TDiff             tdiffTotal_;
  };
}
