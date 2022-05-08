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

  void FFMPEGPublisher::advertiseImpl(
    ros::NodeHandle &nh,
    const std::string &base_topic,
    uint32_t queue_size,
    const image_transport::SubscriberStatusCallback  &conn_cb,
    const image_transport::SubscriberStatusCallback  &disconn_cb,
    const ros::VoidPtr &tracked_object, bool latch) {
    const std::string transportTopic = getTopicToAdvertise(base_topic);
    nh_.reset(new ros::NodeHandle(transportTopic));
    initConfigServer();
    // make the queue twice the size between keyframes.
    queue_size = std::max((int)queue_size, 2 * config_.gop_size);
    FFMPEGPublisherPlugin::advertiseImpl(nh, base_topic, queue_size,
                                         conn_cb, disconn_cb, tracked_object, latch);
  }

  void FFMPEGPublisher::setCodecFromConfig(const EncoderDynConfig &config) {
    encoder_.setCodec(config.encoder);
    encoder_.setProfile(config.profile);
    encoder_.setPreset(config.preset);
    encoder_.setQMax(config.qmax);
    encoder_.setBitRate(config.bit_rate);
    encoder_.setGOPSize(config.gop_size);
    encoder_.setMeasurePerformance(config.measure_performance);
    ROS_DEBUG_STREAM("FFMPEGPublisher codec: " << config.encoder <<
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
      if (!me->encoder_.initialize(message.width, message.height,
              boost::bind(&FFMPEGPublisher::packetReady, me, boost::placeholders::_1))) {
        ROS_ERROR_STREAM("cannot initialize encoder!");
        return;
      }
    }
    me->encoder_.encodeImage(message); // may trigger packetReady() callback(s) from encoder!
    Lock lock(me->configMutex_);
    if (me->config_.measure_performance) {
      if (++me->frameCounter_ > (unsigned int)me->config_.performance_interval) {
        me->encoder_.printTimers(nh_->getNamespace());
        me->encoder_.resetTimers();
        me->frameCounter_ = 0;
      }
    }
  }

  void FFMPEGPublisher::initConfigServer() {
    Lock lock(configMutex_);
    if (!configServer_) {
      configServer_.reset(new ConfigServer(*nh_));
      // this will trigger an immediate callback!
      configServer_->setCallback(boost::bind(&FFMPEGPublisher::configure, this,
            boost::placeholders::_1, boost::placeholders::_2));
    }
  }
  
  void FFMPEGPublisher::connectCallback(
    const ros::SingleSubscriberPublisher &pub) {
    ROS_DEBUG_STREAM("FFMPEGPublisher: connect() now has subscribers: "
                     << getNumSubscribers());
    initConfigServer();
  }

  void FFMPEGPublisher::disconnectCallback(
    const ros::SingleSubscriberPublisher &pub) {
    ROS_DEBUG_STREAM("FFMPEGPublisher: disconnect() subscribers left: "
                     << getNumSubscribers());
  }
}
