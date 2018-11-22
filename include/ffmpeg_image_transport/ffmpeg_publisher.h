/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "ffmpeg_image_transport_msgs/FFMPEGPacket.h"
#include "ffmpeg_image_transport/ffmpeg_encoder.h"
#include "ffmpeg_image_transport/EncoderDynConfig.h"

#include <image_transport/simple_publisher_plugin.h>
#include <dynamic_reconfigure/server.h>

#include <mutex>
#include <memory>

namespace ffmpeg_image_transport {
  typedef image_transport::SimplePublisherPlugin<
    ffmpeg_image_transport_msgs::FFMPEGPacket>  FFMPEGPublisherPlugin;
  class FFMPEGPublisher : public FFMPEGPublisherPlugin {
    typedef std::unique_lock<std::recursive_mutex> Lock;
    using FFMPEGPacketConstPtr = ffmpeg_image_transport_msgs::FFMPEGPacketConstPtr;
  public:
    virtual std::string getTransportName() const override {
      return "ffmpeg";
    }
    void configure(EncoderDynConfig& config, int level);

  protected:
    // override to set up reconfigure server
    virtual void advertiseImpl(ros::NodeHandle &nh,
                               const std::string &base_topic,
                               uint32_t queue_size,
                               const image_transport::SubscriberStatusCallback
                               &user_connect_cb,
                               const image_transport::SubscriberStatusCallback
                               &user_disconnect_cb,
                               const ros::VoidPtr &tracked_object,
                               bool latch) override;
    void publish(const sensor_msgs::Image& message,
                 const PublishFn& publish_fn) const override;
    void connectCallback(const ros::SingleSubscriberPublisher &pub) override;
    void disconnectCallback(const ros::SingleSubscriberPublisher &pub) override;

  private:
    void packetReady(const FFMPEGPacketConstPtr &pkt);
    void setCodecFromConfig(const EncoderDynConfig &cfg);
    void initConfigServer();
    // variables ---------
    typedef dynamic_reconfigure::Server<EncoderDynConfig> ConfigServer;
    std::shared_ptr<ros::NodeHandle>   nh_;
    std::shared_ptr<ConfigServer> configServer_;
    const PublishFn              *publishFunction_{NULL};
    FFMPEGEncoder                 encoder_;
    unsigned int                  frameCounter_{0};
    EncoderDynConfig              config_;
    std::recursive_mutex          configMutex_;
  };
}
