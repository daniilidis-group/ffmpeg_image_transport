#include <pluginlib/class_list_macros.h>
#include "ffmpeg_image_transport/ffmpeg_publisher.h"
#include "ffmpeg_image_transport/ffmpeg_subscriber.h"

PLUGINLIB_EXPORT_CLASS(ffmpeg_image_transport::FFMPEGPublisher,  image_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(ffmpeg_image_transport::FFMPEGSubscriber, image_transport::SubscriberPlugin)
