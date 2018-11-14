/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#pragma once

#include "ffmpeg_image_transport_msgs/FFMPEGPacket.h"
#include "ffmpeg_image_transport/ffmpeg_decoder.h"
#include <image_transport/simple_subscriber_plugin.h>
#include <sensor_msgs/Image.h>


namespace ffmpeg_image_transport {
  using Image = sensor_msgs::Image;
  using ImagePtr = sensor_msgs::ImagePtr;
  using ImageConstPtr = sensor_msgs::ImageConstPtr;
  
  class FFMPEGSubscriber : public image_transport::SimpleSubscriberPlugin<FFMPEGPacket>
  {
  public:
    virtual ~FFMPEGSubscriber() {}

    virtual std::string getTransportName() const
    {
      return "ffmpeg";
    }

  protected:
    virtual void internalCallback(const typename FFMPEGPacket::ConstPtr& message,
                                  const Callback& user_cb) override;
  private:
    void frameReady(const ImageConstPtr &img) const;
    FFMPEGDecoder decoder_;
    const Callback *userCallback_;
  };
}
