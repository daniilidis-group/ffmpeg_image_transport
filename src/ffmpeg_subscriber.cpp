/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_subscriber.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace ffmpeg_image_transport {

  void FFMPEGSubscriber::frameReady(const ImageConstPtr &img) const {
    (*userCallback_)(img);
  }

  void FFMPEGSubscriber::internalCallback(const FFMPEGPacket::ConstPtr& msg,
                                        const Callback& user_cb) {
    if (!decoder_.isInitialized()) {
      userCallback_ = &user_cb;
      if (!decoder_.initialize(msg, boost::bind(&FFMPEGSubscriber::frameReady, this, ::_1))) {
        ROS_ERROR_STREAM("cannot initialize decoder!");
        return;
      }
    }
    decoder_.decodePacket(msg);
  }
}
