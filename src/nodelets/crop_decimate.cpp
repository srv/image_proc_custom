#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <image_proc/CropDecimateConfig.h>

namespace image_proc_custom {

class CropDecimateNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
  image_transport::CameraSubscriber sub_;
  image_transport::CameraPublisher pub_;
  int queue_size_;

  // Dynamic reconfigure
  typedef image_proc::CropDecimateConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config, uint32_t level);
};

void CropDecimateNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh, "camera");
  ros::NodeHandle nh_out(nh, "camera_out");
  it_in_ .reset(new image_transport::ImageTransport(nh_in));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&CropDecimateNodelet::connectCb, this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&CropDecimateNodelet::connectCb, this);
  pub_ = it_out_->advertiseCamera("image_raw",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&CropDecimateNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

// Handles (un)subscribing when clients (un)subscribe
void CropDecimateNodelet::connectCb()
{
  if (pub_.getNumSubscribers() == 0)
    sub_.shutdown();
  else if (!sub_)
    sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &CropDecimateNodelet::imageCb, this);
}

void CropDecimateNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  /// @todo Check image dimensions match info_msg
  /// @todo Publish tweaks to config_ so they appear in reconfigure_gui
  
  /// @todo Check if offsets are larger than image size.
  const int max_width = image_msg->width - config_.x_offset;
  const int max_height = image_msg->height - config_.y_offset;
  if (max_width <= 0 || max_height <= 0)
  {
    NODELET_ERROR_STREAM("ROI offset " << config_.x_offset << "," << config_.y_offset
                         << " out of image size " << image_msg->width << "x" << image_msg->height << "." );
    return;
  }
  const int roi_width = (config_.width == 0 || config_.width > max_width) ? max_width : config_.width;
  const int roi_height = (config_.height == 0 || config_.height > max_height) ? max_height : config_.height;

  // On no-op, just pass the messages along
  if (config_.decimation_x == 1  &&
      config_.decimation_y == 1  &&
      config_.x_offset == 0      &&
      config_.y_offset == 0      &&
      roi_width  == (int) image_msg->width &&
      roi_height == (int) image_msg->height)
  {
    pub_.publish(image_msg, info_msg);
    return;
  }

  const unsigned int num_channels = sensor_msgs::image_encodings::numChannels(image_msg->encoding);
  const unsigned int bit_depth = sensor_msgs::image_encodings::bitDepth(image_msg->encoding);
  const unsigned int bytes_per_pixel = num_channels*(bit_depth/8);
  const bool is_bayer = sensor_msgs::image_encodings::isBayer(image_msg->encoding);

  // Create new image message
  sensor_msgs::ImagePtr out_image = boost::make_shared<sensor_msgs::Image>();
  out_image->header = image_msg->header;
  out_image->encoding = image_msg->encoding;
  if (is_bayer)
  {
    // Subsample Bayer squares of 2x2 pixels instead if single pixels
    // Offsets and binned ROI dimensions should be even to fit Bayer squares
    out_image->width = ( (roi_width / 2) / config_.decimation_x ) * 2;
    out_image->height = ( (roi_height / 2) / config_.decimation_y ) * 2;
    config_.x_offset = (config_.x_offset / 2) * 2;
    config_.y_offset = (config_.y_offset / 2) * 2;
  }
  else
  {
    out_image->width = roi_width / config_.decimation_x;
    out_image->height = roi_height / config_.decimation_y;
  }
  out_image->step = out_image->width*bytes_per_pixel;
  out_image->data.resize(out_image->height * out_image->step);

  if (config_.decimation_x == 1 && config_.decimation_y == 1)
  {
    // Copy ROI without subsampling
    const uint8_t* input_buffer = &image_msg->data[config_.y_offset*image_msg->step + config_.x_offset*bytes_per_pixel];
    uint8_t* output_buffer = &out_image->data[0];
    for (unsigned int y = 0; y < out_image->height; y++, input_buffer += image_msg->step, output_buffer += out_image->step)
    {
      memcpy(output_buffer, input_buffer, out_image->step);
    }
  }
  else if (is_bayer)
  {
    // Subsample Bayer squares
    const unsigned int input_step = config_.decimation_y * image_msg->step * 2;
    const unsigned int input_skip = config_.decimation_x * bytes_per_pixel * 2;
    const unsigned int chunk_size = bytes_per_pixel * 2;
    const uint8_t* input_row = &image_msg->data[config_.y_offset*image_msg->step + config_.x_offset*bytes_per_pixel];
    uint8_t* output_buffer = &out_image->data[0];
    for (unsigned int y = 0; y < out_image->height; y += 2, input_row += input_step)
    {
      const uint8_t* input_buffer = input_row;
      for (unsigned int x = 0; x < out_image->width; x += 2, input_buffer += input_skip, output_buffer += chunk_size)
      {
        memcpy(output_buffer, input_buffer, chunk_size);
        memcpy(output_buffer+out_image->step, input_buffer+image_msg->step, chunk_size);
      }
      output_buffer += out_image->step;
    }
  }
  else
  {
    // Subsample single pixels
    const unsigned int input_step = config_.decimation_y * image_msg->step;
    const unsigned int input_skip = config_.decimation_x * bytes_per_pixel;
    const uint8_t* input_row = &image_msg->data[config_.y_offset*image_msg->step + config_.x_offset*bytes_per_pixel];
    uint8_t* output_buffer = &out_image->data[0];
    for (unsigned int y = 0; y < out_image->height; y++, input_row += input_step)
    {
      const uint8_t* input_buffer = input_row;
      for (unsigned int x = 0; x < out_image->width; x++, input_buffer += input_skip, output_buffer += bytes_per_pixel)
      {
        memcpy(output_buffer, input_buffer, bytes_per_pixel);
      }
    }
  }

  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
  const int binning_x = std::max((int)info_msg->binning_x, 1);
  const int binning_y = std::max((int)info_msg->binning_y, 1);
  out_info->binning_x = binning_x * config_.decimation_x;
  out_info->binning_y = binning_y * config_.decimation_y;
  out_info->roi.x_offset += config_.x_offset * binning_x;
  out_info->roi.y_offset += config_.y_offset * binning_y;
  out_info->roi.height = roi_height * binning_y;
  out_info->roi.width = roi_width * binning_x;

  // Publish new image and new camera info
  pub_.publish(out_image, out_info);
}

void CropDecimateNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_proc_custom, crop_decimate, image_proc_custom::CropDecimateNodelet, nodelet::Nodelet)
