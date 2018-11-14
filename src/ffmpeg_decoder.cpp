/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_decoder.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>

namespace ffmpeg_image_transport {

  FFMPEGDecoder::FFMPEGDecoder() {
    codecMap_["h264_nvenc"] = "h264";
    codecMap_["libx264"]    = "h264";
  }

  FFMPEGDecoder::~FFMPEGDecoder() {
    reset();
  }

  void FFMPEGDecoder::reset() {
    if (codecContext_) {
      avcodec_close(codecContext_);
      av_free(codecContext_);
      codecContext_ = NULL;
    }
    if (swsContext_) {
      sws_freeContext(swsContext_);
      swsContext_ = NULL;
    }
    av_free(decodedFrame_);
    decodedFrame_ = NULL;
    av_free(colorFrame_);
    colorFrame_ = NULL;
  }

  bool FFMPEGDecoder::initialize(const FFMPEGPacket::ConstPtr& msg,
                                 boost::function<void(const ImageConstPtr &img)> callback) {
    callback_ = callback;
    const auto it = codecMap_.find(msg->encoding);
    if (it == codecMap_.end()) {
      ROS_ERROR_STREAM("unknown encoding: " << msg->encoding);
      return (false);
    }
    encoding_ = msg->encoding;
    return (initDecoder(msg->img_width, msg->img_height, it->second));
  }

	bool FFMPEGDecoder::initDecoder(int width, int height, const std::string &codecName) {
    try {
      const AVCodec *codec = avcodec_find_decoder_by_name(codecName.c_str());
      if (!codec) {
        throw (std::runtime_error("unknown codec: " + codecName));
      }
      codecContext_ = avcodec_alloc_context3(codec);
      if (!codecContext_) {
        throw (std::runtime_error("alloc context failed for " + codecName));
      }
      codecContext_->width  = width;
      codecContext_->height = height;
      if (avcodec_open2(codecContext_, codec, NULL) < 0) {
        throw (std::runtime_error("cannot open context for " + codecName));
      }

      decodedFrame_       = av_frame_alloc();
      colorFrame_         = av_frame_alloc();
      colorFrame_->width  = width;
      colorFrame_->height = height;
      colorFrame_->format = AV_PIX_FMT_BGR24;


    } catch (const std::runtime_error &e) {
      ROS_ERROR_STREAM(e.what());
      reset();
      return (false);
    }
    ROS_INFO_STREAM("initialized decoder for " << codecName << " size: " << width << "x" << height);
    return (true);
	}

  bool FFMPEGDecoder::decodePacket(const FFMPEGPacket::ConstPtr &msg) {
    if (msg->encoding != encoding_) {
      ROS_ERROR_STREAM("cannot change encoding on the fly!!!");
      return (false);
    }
    AVCodecContext *ctx = codecContext_;
    AVPacket packet;
    av_init_packet(&packet);
    av_new_packet(&packet, msg->data.size());
    memcpy(packet.data, &msg->data[0], msg->data.size());
    packet.pts = pts_;
    packet.dts = pts_;
    ptsToStamp_[pts_] = msg->header.stamp;
    pts_++;
    
    int gotFrame(0);
    int ret = avcodec_decode_video2(ctx, decodedFrame_, &gotFrame, &packet);
    if (ret > 0 && gotFrame) {
      // convert image to something palatable
      if (!swsContext_) {
        swsContext_ = sws_getContext(ctx->width, ctx->height, (AVPixelFormat)decodedFrame_->format, // src
                                     ctx->width, ctx->height, (AVPixelFormat)colorFrame_->format, // dest
                                     SWS_FAST_BILINEAR, NULL, NULL, NULL);
        if (!swsContext_) {
          ROS_ERROR("cannot allocate sws context!!!!");
          ros::shutdown();
          return (false);
        }
      }
      // prepare the decoded message
      ImagePtr image(new sensor_msgs::Image());
      image->height = decodedFrame_->height;
      image->width  = decodedFrame_->width;
      image->step   = image->width * 3; // 3 bytes per pixel
      image->encoding = sensor_msgs::image_encodings::BGR8;
      image->data.resize(image->step * image->height);

      // bend the memory pointers in colorFrame to the right locations 
      av_image_fill_arrays(colorFrame_->data,  colorFrame_->linesize,
                           &(image->data[0]), (AVPixelFormat)colorFrame_->format,
                           colorFrame_->width, colorFrame_->height, 1);
      sws_scale(swsContext_,
                decodedFrame_->data,  decodedFrame_->linesize, 0, // src
                ctx->height, colorFrame_->data, colorFrame_->linesize); // dest
      auto it = ptsToStamp_.find(decodedFrame_->pts);
      if (it == ptsToStamp_.end()) {
        ROS_ERROR_STREAM("cannot find pts that matches " << decodedFrame_->pts);
      } else {
        image->header = msg->header;
        image->header.stamp = it->second;
        ptsToStamp_.erase(it);
        callback_(image); // deliver callback
      }
    }
    av_free_packet(&packet);
    return (true);
  }
 
}  // namespace
