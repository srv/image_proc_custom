#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <opencv/cv.h>
#include "image_proc_custom/ScaleConfig.h"


namespace image_proc_custom {

class ScaleNodelet : public nodelet::Nodelet
{
  // Input/Output types
  enum ImageType
  {
    MONO=0, COLOR, RECT, RECT_COLOR
  };
  static const int NUM_IMAGE_TYPES = 4;

  // ROS communication
  typedef message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image> SynchronizerImage1;
  typedef message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image> SynchronizerImage2;
  typedef message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image > SynchronizerImage3;
  typedef message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image > SynchronizerImage4;

  ros::NodeHandle node_cam_in_;
  ros::NodeHandle node_cam_out_;
  boost::shared_ptr<image_transport::ImageTransport> it_in_;
  boost::shared_ptr<image_transport::ImageTransport> it_out_;
  ros::Publisher publ_info_;
  image_transport::Publisher publ_image_[NUM_IMAGE_TYPES];
  message_filters::Subscriber<sensor_msgs::CameraInfo> subs_info_;
  image_transport::SubscriberFilter subs_image_[NUM_IMAGE_TYPES];

  boost::shared_ptr<SynchronizerImage1> sync_image_1_;
  boost::shared_ptr<SynchronizerImage2> sync_image_2_;
  boost::shared_ptr<SynchronizerImage3> sync_image_3_;
  boost::shared_ptr<SynchronizerImage4> sync_image_4_;

  message_filters::Connection cb_connection_;

  int queue_size_;

  // Current inputs
  std::vector<ImageType> current_inputs_;

  // Dynamic reconfigure
  typedef image_proc_custom::ScaleConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;
  int interpolation_flag_;

  virtual void onInit();

  int updateCvInterpolationFlag(std::string& method);

  void configCb(Config &config, uint32_t level);

  void connectCb();

  void imagesCb(const sensor_msgs::CameraInfoConstPtr& info_msg,
                const sensor_msgs::ImageConstPtr& img_msg_1,
                const sensor_msgs::ImageConstPtr& img_msg_2,
                const sensor_msgs::ImageConstPtr& img_msg_3,
                const sensor_msgs::ImageConstPtr& img_msg_4);
};


void ScaleNodelet::onInit()
{
  ros::NodeHandle& node = getNodeHandle();
  ros::NodeHandle& priv = getPrivateNodeHandle();
  node_cam_in_ = ros::NodeHandle(node, "camera");
  node_cam_out_ = ros::NodeHandle(node, "camera_out");

  it_in_ = boost::make_shared<image_transport::ImageTransport>(node_cam_in_);
  it_out_ = boost::make_shared<image_transport::ImageTransport>(node_cam_out_);

  image_transport::SubscriberStatusCallback image_connect_cb = boost::bind(&ScaleNodelet::connectCb, this);
  ros::SubscriberStatusCallback info_connect_cb = boost::bind(&ScaleNodelet::connectCb, this);
  publ_info_ = node_cam_out_.advertise<sensor_msgs::CameraInfo>("camera_info", 1, info_connect_cb, info_connect_cb);
  publ_image_[MONO]  = it_out_->advertise("image_mono", 1, image_connect_cb, image_connect_cb);
  publ_image_[COLOR] = it_out_->advertise("image_color", 1, image_connect_cb, image_connect_cb);
  publ_image_[RECT]  = it_out_->advertise("image_rect", 1, image_connect_cb, image_connect_cb);
  publ_image_[RECT_COLOR] = it_out_->advertise("image_rect_color", 1, image_connect_cb, image_connect_cb);

  interpolation_flag_ = cv::INTER_LINEAR;

  reconfigure_server_ = boost::make_shared<ReconfigureServer>(priv);
  reconfigure_server_->setCallback(boost::bind(&ScaleNodelet::configCb, this, _1, _2));

  priv.param("queue_size", queue_size_, 5);

  sync_image_1_ = boost::make_shared<SynchronizerImage1>(queue_size_+1);
  sync_image_2_ = boost::make_shared<SynchronizerImage2>(queue_size_+2);
  sync_image_3_ = boost::make_shared<SynchronizerImage3>(queue_size_+3);
  sync_image_4_ = boost::make_shared<SynchronizerImage4>(queue_size_+4);
}


int ScaleNodelet::updateCvInterpolationFlag(std::string& method)
{
  const int NUM_METHODS = 5;
  const std::string methods[NUM_METHODS] = {"nearest","linear","area","cubic","lanczos4"};
  const int flags[NUM_METHODS] = {cv::INTER_NEAREST, cv::INTER_LINEAR, cv::INTER_AREA, cv::INTER_CUBIC, cv::INTER_LANCZOS4};
  for (int i=0; i<NUM_METHODS; i++)
    if (method == methods[i])
      return flags[i];
  ROS_WARN_STREAM("Unknown interpolation method " << method << " , "
                  "defaulting to linear.");
  method = "linear";
  return cv::INTER_LINEAR;
}


void ScaleNodelet::configCb(Config &config, uint32_t level)
{
  if (config.interpolation != config_.interpolation)
    interpolation_flag_ = updateCvInterpolationFlag(config.interpolation);
  config_ = config;
}


// Handles (un)subscribing when clients (un)subscribe
void ScaleNodelet::connectCb()
{
  std::string image_topic[NUM_IMAGE_TYPES];
  image_topic[MONO]  = "image_mono";
  image_topic[COLOR] = "image_color";
  image_topic[RECT]  = "image_rect";
  image_topic[RECT_COLOR] = "image_rect_color";

  std::vector<ImageType> inputs;

  for (int i=0; i<NUM_IMAGE_TYPES; i++)
  {
    if ( publ_image_[i].getNumSubscribers() > 0 )
    {
      inputs.push_back(ImageType(i));
      if ( ! (subs_image_[i].getSubscriber()) )
        subs_image_[i].subscribe(*it_in_,image_topic[i],queue_size_);
    }
    else if ( subs_image_[i].getSubscriber() )
        subs_image_[i].unsubscribe();
  }

  if ( inputs != current_inputs_ )
  {
    if ( current_inputs_.size()>0 )
      cb_connection_.disconnect();
    const int num_inputs = inputs.size();
    if ( num_inputs > 0 )
    {
      if (!subs_info_.getSubscriber())
        subs_info_.subscribe(node_cam_in_,"camera_info", queue_size_);
      sensor_msgs::ImageConstPtr null_ptr;
      switch (num_inputs)
      {
        case 1 :
          sync_image_1_->connectInput(subs_info_, subs_image_[inputs[0]]);
          cb_connection_ = sync_image_1_->registerCallback(
                  boost::bind(&ScaleNodelet::imagesCb, this, _1, _2, null_ptr, null_ptr, null_ptr) );
          break;
        case 2 :
          sync_image_2_->connectInput(subs_info_, subs_image_[inputs[0]], subs_image_[inputs[1]]);
          cb_connection_ = sync_image_2_->registerCallback(
                  boost::bind(&ScaleNodelet::imagesCb, this, _1, _2, _3, null_ptr, null_ptr) );
          break;
        case 3 :
          sync_image_3_->connectInput(subs_info_, subs_image_[inputs[0]], subs_image_[inputs[1]], subs_image_[inputs[2]]);
          cb_connection_ = sync_image_3_->registerCallback(
                  boost::bind(&ScaleNodelet::imagesCb, this, _1, _2, _3, _4, null_ptr) );
          break;
        case 4 :
          sync_image_4_->connectInput(subs_info_, subs_image_[inputs[0]], subs_image_[inputs[1]], subs_image_[inputs[2]], subs_image_[inputs[3]]);
          cb_connection_ = sync_image_4_->registerCallback(
                  boost::bind(&ScaleNodelet::imagesCb, this, _1, _2, _3, _4, _5) );
          break;
      }
    }
    else if (subs_info_.getSubscriber())
        subs_info_.unsubscribe();
    current_inputs_ = inputs;
  }
}


void ScaleNodelet::imagesCb(const sensor_msgs::CameraInfoConstPtr& info_msg,
                            const sensor_msgs::ImageConstPtr& img_msg_1,
                            const sensor_msgs::ImageConstPtr& img_msg_2,
                            const sensor_msgs::ImageConstPtr& img_msg_3,
                            const sensor_msgs::ImageConstPtr& img_msg_4)
{
  const sensor_msgs::ImageConstPtr images[NUM_IMAGE_TYPES] = {img_msg_1, img_msg_2, img_msg_3, img_msg_4};
  const int num_images = current_inputs_.size();
  for (int i=0; i<num_images; i++)
  {
    const sensor_msgs::ImageConstPtr& image_msg = images[i];
    const int max_width = image_msg->width - config_.roi_offset_x;
    const int max_height = image_msg->height - config_.roi_offset_y;
    if (max_width <= 0 || max_height <= 0)
    {
      NODELET_ERROR_STREAM("ROI offset " << config_.roi_offset_x << "," << config_.roi_offset_y
                           << " out of image size " << image_msg->width << "x" << image_msg->height << "." );
      continue;
    }
    const int roi_width = (config_.roi_width == 0 || config_.roi_width > max_width) ? max_width : config_.roi_width;
    const int roi_height = (config_.roi_height == 0 || config_.roi_height > max_height) ? max_height : config_.roi_height;
    const double scale_x = 1.0/config_.binning_x;
    const double scale_y = 1.0/config_.binning_y;
    cv::Rect roi(config_.roi_offset_x, config_.roi_offset_y, roi_width, roi_height);
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image_msg);
    cv_bridge::CvImagePtr cv_image_scaled = boost::make_shared<cv_bridge::CvImage>();
    cv_image_scaled->header = cv_image->header;
    cv_image_scaled->encoding = cv_image->encoding;
    cv::resize((cv_image->image)(roi), cv_image_scaled->image, cv::Size(), scale_x, scale_y, interpolation_flag_);
    const sensor_msgs::ImageConstPtr image_roi_scaled_msg = cv_image_scaled->toImageMsg();
    publ_image_[current_inputs_[i]].publish(image_roi_scaled_msg);
  }

  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
  const int binning_x = std::max<unsigned int>(info_msg->binning_x, 1);
  const int binning_y = std::max<unsigned int>(info_msg->binning_y, 1);
  out_info->binning_x = config_.binning_x * binning_x ;
  out_info->binning_y = config_.binning_y * binning_y ;
  out_info->roi.x_offset += config_.roi_offset_x * binning_x;
  out_info->roi.y_offset += config_.roi_offset_y * binning_y;
  out_info->roi.height = config_.roi_height * binning_y;
  out_info->roi.width = config_.roi_width * binning_x;
  publ_info_.publish(out_info);
}


} // namespace image_proc_scale


// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_proc_custom, scale, image_proc_custom::ScaleNodelet, nodelet::Nodelet)
