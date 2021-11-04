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
#include <unordered_map>

namespace ffmpeg_image_transport {

  FFMPEGDecoder::FFMPEGDecoder() {
    codecMap_["h264_nvenc"] = {"h264"};
    codecMap_["libx264"]    = {"h264"};
    codecMap_["hevc_nvenc"] = {"hevc_cuvid", "hevc"};
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
    if (hwDeviceContext_) {
      av_buffer_unref(&hwDeviceContext_);
    }
    av_free(decodedFrame_);
    decodedFrame_ = NULL;
    av_free(cpuFrame_);
    cpuFrame_ = NULL;
    av_free(colorFrame_);
    colorFrame_ = NULL;
  }

  bool FFMPEGDecoder::initialize(const FFMPEGPacket::ConstPtr& msg,
                                 Callback callback,
                                 const std::string &codecName) {
    callback_ = callback;
    std::string cname = codecName;
    std::vector<std::string> codecs;
    if (cname.empty()) {
      // try and find the right codec from the map
      const auto it = codecMap_.find(msg->encoding);
      if (it == codecMap_.end()) {
        ROS_ERROR_STREAM("message has unknown encoding: " << msg->encoding);
        return (false);
      }
      cname  = msg->encoding;
      codecs = it->second;
    } else {
      codecs.push_back(codecName);
    }
    encoding_ = msg->encoding;
    return (initDecoder(msg->img_width, msg->img_height, cname, codecs));
  }

  static enum AVHWDeviceType get_hw_type(const std::string &name) {
    enum AVHWDeviceType type = av_hwdevice_find_type_by_name(name.c_str());
    if (type == AV_HWDEVICE_TYPE_NONE) {
      ROS_WARN_STREAM("hw accel device is not supported: " << name);
      ROS_INFO_STREAM("available devices:");
      while((type = av_hwdevice_iterate_types(type)) != AV_HWDEVICE_TYPE_NONE)
        ROS_INFO_STREAM(av_hwdevice_get_type_name(type));
      return (type);
    }
    return (type);
  }

  static AVBufferRef *hw_decoder_init(AVBufferRef **hwDeviceContext,
                                      const enum AVHWDeviceType hwType) {
    int rc = av_hwdevice_ctx_create(hwDeviceContext, hwType, NULL, NULL, 0);
    if (rc < 0) {
      ROS_ERROR_STREAM("failed to create context for HW device: " << hwType);
      return (NULL);
    }
    return (av_buffer_ref(*hwDeviceContext));
  }

  static std::unordered_map<AVCodecContext *, AVPixelFormat> pix_format_map;
  
  static enum AVPixelFormat
  get_hw_format(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts) {
    enum AVPixelFormat pf = pix_format_map[ctx];
    const enum AVPixelFormat *p;
    for (p = pix_fmts; *p != -1; p++) {
      if (*p == pf) {
        //ROS_INFO_STREAM("found hw pix format: " << pf);
        return *p;
      }
    }
    ROS_ERROR_STREAM("Failed to get HW surface format.");
    return AV_PIX_FMT_NONE;
  }

  static enum AVPixelFormat find_pix_format(
    const std::string &codecName, enum AVHWDeviceType hwDevType,
    const AVCodec *codec, const std::string &hwAcc) {

    for (int i = 0; ; i++) {
      const AVCodecHWConfig *config = avcodec_get_hw_config(codec, i);
      if (!config) {
        ROS_WARN_STREAM("decoder " << codecName <<
                        " does not support hw accel: " << hwAcc);
        return (AV_PIX_FMT_NONE);
      }
      if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
          config->device_type == hwDevType) {
        return (config->pix_fmt);
      }
    }
    return (AV_PIX_FMT_NONE);
  }
                                                  
  bool FFMPEGDecoder::initDecoder(int width, int height,
                                  const std::string &codecName,
                                  const std::vector<std::string> &codecs) {
    std::string codecUsed = "NO_CODEC_FOUND";
    try {
      const AVCodec *codec = NULL;
      for (const auto &c: codecs) {
        codec = avcodec_find_decoder_by_name(c.c_str());
        if (!codec) {
          ROS_WARN_STREAM("no codec " << c << " found!");
          continue;
        }
        codecContext_ = avcodec_alloc_context3(codec);
        if (!codecContext_) {
          ROS_WARN_STREAM("alloc context failed for " + codecName);
          codec = NULL;
          continue;
        }
        av_opt_set_int(codecContext_, "refcounted_frames", 1, 0);  
        const std::string hwAcc("cuda");
        enum AVHWDeviceType hwDevType = get_hw_type(hwAcc);

        if (hwDevType != AV_HWDEVICE_TYPE_NONE) {
          codecContext_->hw_device_ctx = hw_decoder_init(&hwDeviceContext_,
                                                         hwDevType);
          hwPixFormat_ = find_pix_format(codecName, hwDevType, codec, hwAcc);
          // must put in global hash for the callback function
          pix_format_map[codecContext_] = hwPixFormat_;
          codecContext_->get_format = get_hw_format;
        } else {
          hwPixFormat_ = AV_PIX_FMT_NONE;
        }
        codecContext_->width  = width;
        codecContext_->height = height;

        if (avcodec_open2(codecContext_, codec, NULL) < 0) {
          ROS_WARN_STREAM("open context failed for " + codecName);
          av_free(codecContext_);
          codecContext_ = NULL;
          codec = NULL;
          continue;
        }
        codecUsed = c;
        break;
      }
      if (!codec)
        throw (std::runtime_error("cannot open codec " + codecName));
      
      decodedFrame_       = av_frame_alloc();
      cpuFrame_  = (hwPixFormat_ == AV_PIX_FMT_NONE) ? NULL : av_frame_alloc();
      colorFrame_         = av_frame_alloc();
      colorFrame_->width  = width;
      colorFrame_->height = height;
      colorFrame_->format = AV_PIX_FMT_BGR24;


    } catch (const std::runtime_error &e) {
      ROS_ERROR_STREAM(e.what());
      reset();
      return (false);
    }
    if (codecName != codecUsed) {
      ROS_INFO_STREAM("message encoded with " <<
                      codecName << " decoded with " << codecUsed);
    } else {
      ROS_INFO_STREAM("decoding with " << codecUsed);
    }
    return (true);
	}

  bool FFMPEGDecoder::decodePacket(const FFMPEGPacket::ConstPtr &msg) {
    ros::WallTime t0;
    if (measurePerformance_) {
      t0 = ros::WallTime::now();
    }
    if (msg->encoding != encoding_) {
      ROS_ERROR_STREAM("cannot change encoding on the fly!!!");
      return (false);
    }
    AVCodecContext *ctx = codecContext_;
    AVPacket packet;
    av_init_packet(&packet);
    av_new_packet(&packet, msg->data.size()); // will add some padding!
    memcpy(packet.data, &msg->data[0], msg->data.size());
    packet.pts = msg->pts;
    packet.dts = packet.pts;
    ptsToStamp_[packet.pts] = msg->header.stamp;
    int ret = avcodec_send_packet(ctx, &packet);
    if (ret != 0) {
      ROS_WARN_STREAM("send_packet failed for pts: " <<  msg->pts);
      av_packet_unref(&packet);
      return (false);
    }
    ret = avcodec_receive_frame(ctx, decodedFrame_);
    const bool isAcc = (ret == 0) && (decodedFrame_->format == hwPixFormat_);
    if (isAcc) {
      ret = av_hwframe_transfer_data(cpuFrame_, decodedFrame_, 0);
      if (ret < 0) {
        ROS_WARN_STREAM("failed to transfer data from GPU->CPU");
        av_packet_unref(&packet);
        return  (false);
      }
    }
    AVFrame *frame = isAcc ? cpuFrame_ : decodedFrame_;
        
    if (ret == 0 && frame->width != 0) {
      // convert image to something palatable
      if (!swsContext_) {
        swsContext_ = sws_getContext(
          ctx->width, ctx->height, (AVPixelFormat)frame->format, //src
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
      image->height = frame->height;
      image->width  = frame->width;
      image->step   = image->width * 3; // 3 bytes per pixel
      image->encoding = sensor_msgs::image_encodings::BGR8;
      image->data.resize(image->step * image->height);

      // bend the memory pointers in colorFrame to the right locations 
      av_image_fill_arrays(colorFrame_->data,  colorFrame_->linesize,
                           &(image->data[0]),
                           (AVPixelFormat)colorFrame_->format,
                           colorFrame_->width, colorFrame_->height, 1);
      sws_scale(swsContext_,
                frame->data,  frame->linesize, 0, // src
                ctx->height, colorFrame_->data, colorFrame_->linesize); // dest
      auto it = ptsToStamp_.find(decodedFrame_->pts);
      if (it == ptsToStamp_.end()) {
        ROS_ERROR_STREAM("cannot find pts that matches "
                         << decodedFrame_->pts);
      } else {
        image->header = msg->header;
        image->header.stamp = it->second;
        ptsToStamp_.erase(it);
        callback_(image, decodedFrame_->key_frame == 1); // deliver callback
      }
    }
    av_packet_unref(&packet);
    if (measurePerformance_) {
      ros::WallTime t1 = ros::WallTime::now();
      double dt = (t1-t0).toSec();
      tdiffTotal_.update(dt);
    }
    return (true);
  }

  void FFMPEGDecoder::resetTimers() {
    tdiffTotal_.reset();
  }

  void FFMPEGDecoder::printTimers(const std::string &prefix) const {
    ROS_INFO_STREAM(prefix << " total decode: " << tdiffTotal_);
  }
 
}  // namespace
