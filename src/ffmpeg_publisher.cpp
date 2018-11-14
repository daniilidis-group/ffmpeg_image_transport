/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "ffmpeg_image_transport/ffmpeg_publisher.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace ffmpeg_image_transport {


  void FFMPEGPublisher::packetReady(const FFMPEGPacketConstPtr &pkt) {
    (*publishFunction_)(*pkt);
  }

  static bool is_equal(const EncoderDynConfig &a,
                       const EncoderDynConfig &b) {
    return (a.encoder  == b.encoder &&
            a.profile  == b.profile &&
            a.qmax     == b.qmax &&
            a.bit_rate == b.bit_rate &&
            a.gop_size == b.gop_size &&
            a.measure_performance == b.measure_performance);
  }
  
  void
  FFMPEGPublisher::configure(EncoderDynConfig& config, int level) {
    if (!is_equal(config_, config)) {
      config_ = config;
      setCodecFromConfig(config);
      encoder_.reset(); // will be opened on next image
    }
  }

  void FFMPEGPublisher::setCodecFromConfig(const EncoderDynConfig &config) {
    encoder_.setCodec(config.encoder);
    encoder_.setProfile(config.profile);
    encoder_.setPreset(config.preset);
    encoder_.setQMax(config.qmax);
    encoder_.setBitRate(config.bit_rate);
    encoder_.setGOPSize(config.gop_size);
    encoder_.setMeasurePerformance(config.measure_performance);
    ROS_INFO_STREAM(" setting codec: " << config.encoder <<
                    ", profile: " << config.profile <<
                    ", preset: " << config.preset <<
                    ", bit rate: " << config.bit_rate <<
                    ", qmax: " << config.qmax);
  }


  void
  FFMPEGPublisher::publish(const sensor_msgs::Image& message,
                           const PublishFn &publish_fn) const {
    FFMPEGPublisher *me = const_cast<FFMPEGPublisher *>(this);
    if (!me->encoder_.isInitialized()) {
      me->initConfigServer();
      me->publishFunction_ = &publish_fn;
      if (!me->encoder_.initialize(message, boost::bind(&FFMPEGPublisher::packetReady, me, ::_1))) {
        ROS_ERROR_STREAM("cannot initialize encoder!");
        return;
      }
    }
    me->encoder_.encodeImage(message); // may trigger packetReady() callback(s) from encoder!
    Lock lock(me->configMutex_);
    if (me->config_.measure_performance) {
      if (++me->frameCounter_ > (unsigned int)me->config_.performance_interval) {
        me->encoder_.printTimers();
        me->encoder_.resetTimers();
        me->frameCounter_ = 0;
      }
    }
  }

  void FFMPEGPublisher::initConfigServer() {
    Lock lock(configMutex_);
    if (!configServer_) {
      ROS_INFO_STREAM("init name server: " << this->nh().getNamespace());
      configServer_.reset(new ConfigServer(this->nh()));
      // this will trigger an immediate callback!
      configServer_->setCallback(boost::bind(&FFMPEGPublisher::configure, this, _1, _2));
    }
  }
  
  void FFMPEGPublisher::connectCallback(const ros::SingleSubscriberPublisher &pub) {
    ROS_INFO_STREAM("connect: got number of subscribers: " << getNumSubscribers());
    initConfigServer();
  }

  void FFMPEGPublisher::disconnectCallback(const ros::SingleSubscriberPublisher &pub) {
    ROS_INFO_STREAM("disconnect: got number of subscribers: " << getNumSubscribers());
  }
}
