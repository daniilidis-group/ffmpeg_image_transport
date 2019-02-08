/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "ffmpeg_image_transport_msgs/FFMPEGPacket.h"
#include "ffmpeg_image_transport/tdiff.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>

#include <memory>
#include <list>
#include <iostream>
#include <unordered_map>
#include <mutex>

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
  typedef std::unordered_map<int64_t, ros::Time> PTSMap;

  class FFMPEGEncoder {
    using FFMPEGPacket = ffmpeg_image_transport_msgs::FFMPEGPacket;
    using FFMPEGPacketConstPtr =
      ffmpeg_image_transport_msgs::FFMPEGPacketConstPtr;
    typedef std::unique_lock<std::recursive_mutex> Lock;
    typedef boost::function<void(const FFMPEGPacketConstPtr &pkt)> Callback;
  public:
    FFMPEGEncoder();
    ~FFMPEGEncoder();
    // ------- various encoding settings
    void setCodec(const std::string &n) {
      Lock lock(mutex_);
      codecName_ = n;
    }
    void setProfile(const std::string &p) {
      Lock lock(mutex_);
      profile_ = p;
    }
    void setPreset(const std::string &p) {
      Lock lock(mutex_);
      preset_ = p;
    }
    void setQMax(int q)    {
      Lock lock(mutex_);
      qmax_  = q;
    }
    void setBitRate(int r) {
      Lock lock(mutex_);
      bitRate_ = r;
    }
    void setGOPSize(int g) {
      Lock lock(mutex_);
      GOPSize_ = g;
    }
    void setFrameRate(int frames, int second) {
      Lock lock(mutex_);
      frameRate_.num = frames;
      frameRate_.den = second;
      timeBase_.num  = second;
      timeBase_.den  = frames;
    }
    void setMeasurePerformance(bool p) {
      Lock lock(mutex_);
      measurePerformance_ = p;
    }
    // ------- teardown and startup
    bool isInitialized() const {
      Lock lock(mutex_);
      return (codecContext_ != NULL);
    }
    bool initialize(int width, int height, Callback callback);

    void reset();
    // encode image
    void encodeImage(const cv::Mat &img,
                     const std_msgs::Header &header, const ros::WallTime &t0);
    void encodeImage(const sensor_msgs::Image &msg);
    // ------- performance statistics
    void printTimers(const std::string &prefix) const;
    void resetTimers();
  private:
    bool openCodec(int width, int height);
    void closeCodec();
    int  drainPacket(const std_msgs::Header &hdr, int width, int height);
    // --------- variables
    mutable std::recursive_mutex mutex_;
    boost::function<void(const FFMPEGPacketConstPtr &pkt)> callback_;
    // config
    std::string       codecName_;
    std::string       preset_;
    std::string       profile_;
    AVPixelFormat     pixFormat_{AV_PIX_FMT_YUV420P};
    AVRational        timeBase_{1, 40};
    AVRational        frameRate_{40, 1};
    int               GOPSize_{15};
    int64_t           bitRate_{1000000};
    int               qmax_{0};
    // libav state
    AVCodecContext    *codecContext_{NULL};
    AVFrame           *frame_{NULL};
    AVPacket          packet_;
    int64_t           pts_{0};
    PTSMap            ptsToStamp_;
    // performance analysis
    bool              measurePerformance_{true};
    int64_t           totalInBytes_{0};
    int64_t           totalOutBytes_{0};
    unsigned int      frameCnt_{0};
    TDiff             tdiffUncompress_;
    TDiff             tdiffEncode_;
    TDiff             tdiffDebayer_;
    TDiff             tdiffFrameCopy_;
    TDiff             tdiffSendFrame_;
    TDiff             tdiffReceivePacket_;
    TDiff             tdiffCopyOut_;
    TDiff             tdiffPublish_;
    TDiff             tdiffTotal_;
  };
}
