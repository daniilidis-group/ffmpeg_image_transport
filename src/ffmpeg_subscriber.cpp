/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_subscriber.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace ffmpeg_image_transport {

  void FFMPEGSubscriber::frameReady(const ImageConstPtr &img,
                                    bool isKeyFrame) const {
    (*userCallback_)(img);
  }

  void FFMPEGSubscriber::subscribeImpl(
    ros::NodeHandle &nh, const std::string &base_topic,
    uint32_t queue_size, const Callback &callback,
    const ros::VoidPtr &tracked_object,
    const image_transport::TransportHints &transport_hints) {
    // bump queue size a bit to avoid lost packets
    nh.param<std::string>("decoder_type", decoderType_, "");
    queue_size = std::max((int)queue_size, 20);
    FFMPEGSubscriberPlugin::subscribeImpl(nh, base_topic,
                                          queue_size, callback,
                                          tracked_object,
                                          transport_hints);
  }

  void FFMPEGSubscriber::internalCallback(const FFMPEGPacket::ConstPtr& msg,
                                        const Callback& user_cb) {
    if (!decoder_.isInitialized()) {
      if (msg->flags == 0) {
        return; // wait for key frame!
      }
      userCallback_ = &user_cb;
      if (!decoder_.initialize(
            msg, boost::bind(&FFMPEGSubscriber::frameReady, this, ::_1, ::_2),
            decoderType_)) {
        ROS_ERROR_STREAM("cannot initialize decoder!");
        return;
      }
    }
    decoder_.decodePacket(msg);
  }
}
