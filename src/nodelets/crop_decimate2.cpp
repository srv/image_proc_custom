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

  void subsample(const uint8_t* const in, const int32_t& in_step,
                 const int& in_width, const int& in_height,
                 const int& bytes_per_pixel,
                 const int& cell_width, const int& cell_height,
                 const int& roi_offset_x, const int& roi_offset_y,
                 const int& roi_width, const int& roi_height,
                 const int& binning_x, const int& binning_y,
                 uint8_t* const out, const int& out_step,
                 const int& out_width, const int& out_height);
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
  const unsigned int config_width  = (config_.width == 0) ? image_msg->width : config_.width;
  const unsigned int config_height = (config_.height == 0) ? image_msg->height : config_.height;
  // On no-op, just pass the messages along
  if (config_.decimation_x == 1         &&
      config_.decimation_y == 1         &&
      config_.x_offset == 0             &&
      config_.y_offset == 0             &&
      config_width  == image_msg->width &&
      config_height == image_msg->height  )
  {
    pub_.publish(image_msg, info_msg);
    return;
  }
  

  const unsigned int num_channels = sensor_msgs::image_encodings::numChannels(image_msg->encoding);
  const unsigned int bit_depth = sensor_msgs::image_encodings::bitDepth(image_msg->encoding);
  const unsigned int bytes_per_pixel = num_channels*(bit_depth/8);
  const bool is_bayer = sensor_msgs::image_encodings::isBayer(image_msg->encoding);
  const unsigned int cell_size = (is_bayer) ? 2 : 1;
  const unsigned int width  = ( (config_width  / cell_size) / config_.decimation_x ) * cell_size;
  const unsigned int height = ( (config_height / cell_size) / config_.decimation_y ) * cell_size;
  const unsigned int roi_width = width * config_.decimation_x;
  const unsigned int roi_height = height * config_.decimation_y;
  const unsigned int offset_x = (config_.x_offset / cell_size) * cell_size;
  const unsigned int offset_y = (config_.y_offset / cell_size) * cell_size;

  // Create new image message
  sensor_msgs::ImagePtr out_image = boost::make_shared<sensor_msgs::Image>();
  out_image->header = image_msg->header;
  out_image->encoding = image_msg->encoding;
  out_image->width  = width;
  out_image->height = height;
  out_image->step = width*bytes_per_pixel;
  out_image->data.resize(out_image->height * out_image->step, 0);
  subsample(image_msg->data.data(), image_msg->step, image_msg->width, image_msg->height,
            bytes_per_pixel, cell_size, cell_size,
            offset_x, offset_y, roi_width, roi_height,
            config_.decimation_x, config_.decimation_y,
            out_image->data.data(), out_image->step, out_image->width, out_image->height);


  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
  const unsigned int binning_x = std::max<unsigned int>(info_msg->binning_x, 1);
  const unsigned int binning_y = std::max<unsigned int>(info_msg->binning_y, 1);
  out_info->binning_x = binning_x * config_.decimation_x;
  out_info->binning_y = binning_y * config_.decimation_y;
  out_info->roi.x_offset += offset_x * binning_x;
  out_info->roi.y_offset += offset_y * binning_y;
  out_info->roi.height = roi_height * binning_y;
  out_info->roi.width = roi_width * binning_x;

  pub_.publish(out_image, out_info);
}

void CropDecimateNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

void CropDecimateNodelet::subsample(const uint8_t* const in, const int& in_step,
                                    const int& in_width, const int& in_height,
                                    const int& bytes_per_pixel,
                                    const int& cell_width, const int& cell_height,
                                    const int& roi_offset_x, const int& roi_offset_y,
                                    const int& roi_width, const int& roi_height,
                                    const int& binning_x, const int& binning_y,
                                    uint8_t* const out, const int& out_step,
                                    const int& out_width, const int& out_height)
{
  const int cell_step = bytes_per_pixel * cell_width;
  const int in_skip_y = binning_y * in_step * cell_height;
  const int in_skip_x = binning_x * cell_step;
  const int max_width = std::min(roi_width, in_width - roi_offset_x);
  const int max_height = std::min(roi_height, in_height - roi_offset_y);
  const int binned_width  = ( (max_width  / cell_width ) / binning_x ) * cell_width;
  const int binned_height = ( (max_height / cell_height) / binning_y ) * cell_height;
  const int width  = std::min(binned_width , out_width );
  const int height = std::min(binned_height, out_height);
  const uint8_t* in_row = in + roi_offset_y*in_step + roi_offset_x*bytes_per_pixel;
  uint8_t* out_row = out;
  for (int i = 0; i < height; i+=cell_height, in_row += in_skip_y)
  {
    const uint8_t* in_cell_row = in_row;
    for (int k = 0; k < cell_height; k++, in_cell_row+=in_step, out_row+=out_step)
    {
      const uint8_t* in_buffer = in_cell_row;
      uint8_t* out_buffer = out_row;
      for (int j = 0; j < width; j+=cell_width, in_buffer += in_skip_x, out_buffer += cell_step)
      {
        memcpy(out_buffer, in_buffer, cell_step);
      }
    }
  }
}

} // namespace image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_proc_custom, crop_decimate, image_proc_custom::CropDecimateNodelet, nodelet::Nodelet)
