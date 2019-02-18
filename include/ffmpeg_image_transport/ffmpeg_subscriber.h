/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#pragma once

#include "ffmpeg_image_transport_msgs/FFMPEGPacket.h"
#include "ffmpeg_image_transport/ffmpeg_decoder.h"
#include <image_transport/simple_subscriber_plugin.h>
#include <sensor_msgs/Image.h>
#include <string>

namespace ffmpeg_image_transport {
  using Image         = sensor_msgs::Image;
  using ImagePtr      = sensor_msgs::ImagePtr;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  
  typedef image_transport::SimpleSubscriberPlugin<
    ffmpeg_image_transport_msgs::FFMPEGPacket>  FFMPEGSubscriberPlugin;
  class FFMPEGSubscriber: public FFMPEGSubscriberPlugin  {
  public:
    virtual ~FFMPEGSubscriber() {}

    virtual std::string getTransportName() const {
      return "ffmpeg";
    }

  protected:
    virtual void
    internalCallback(const typename FFMPEGPacket::ConstPtr& message,
                     const Callback& user_cb) override;
    virtual void
    subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic,
                  uint32_t queue_size, const Callback &callback,
                  const ros::VoidPtr &tracked_object,
                  const image_transport::TransportHints &transport_hints)
      override;
  private:
    void frameReady(const ImageConstPtr &img, bool isKeyFrame) const;
    FFMPEGDecoder decoder_;
    std::string   decoderType_;
    const Callback *userCallback_;
  };
}
