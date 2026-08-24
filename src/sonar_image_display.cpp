#include <rviz_sonar_image/sonar_image_display.h>
#include "rviz_common/logging.hpp"

#include <cmath>
#include <memory>
#include <vector>

namespace rviz_sonar_image
{

SonarImageDisplay::SonarImageDisplay()
  :color_map_(std::make_shared<ColorMap>())
{

  alpha_property_ =
      new rviz_common::properties::FloatProperty("Alpha", 1.0f, "The amount of transparency to apply to the curtain.", this,
                        SLOT(updateAlpha()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  colormap_minimum_property_ = new rviz_common::properties::FloatProperty("Colormap minimum value", -80.0f, "The value representing the bottom of the color map", this, SLOT(updateColormapRange()));
  colormap_minimum_property_->setMin(-200.0f);
  colormap_minimum_property_->setMax(4096.0f);

  colormap_maximum_property_ = new rviz_common::properties::FloatProperty("Colormap maximum value", -20.0f, "The value representing the top of the color map", this, SLOT(updateColormapRange()));
  colormap_maximum_property_->setMin(-200.0f);
  colormap_maximum_property_->setMax(4096.0f);
  updateColormapRange();

  minimum_data_value_property_ = new rviz_common::properties::FloatProperty("Minimum data value", 0.0f, "The minimum data value received", this);
  minimum_data_value_property_->setReadOnly(true);
  minimum_data_value_property_->setShouldBeSaved(false);

  maximum_data_value_property_ = new rviz_common::properties::FloatProperty("Maximum data value", 0.0f, "The maximum data value received", this);
  maximum_data_value_property_->setReadOnly(true);
  maximum_data_value_property_->setShouldBeSaved(false);
}

SonarImageDisplay::~SonarImageDisplay()
{

}

void SonarImageDisplay::onInitialize()
{
  MFDClass::onInitialize();

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  auto node = ros_node_abstraction->get_raw_node();
  image_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
      "watercolumn_image_polar", rclcpp::QoS(1).transient_local());
  cartesian_image_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
      "watercolumn_image_cartesian", rclcpp::QoS(1).transient_local());
}

void SonarImageDisplay::reset()
{
  MFDClass::reset();
  fans_.clear();
}

void SonarImageDisplay::updateAlpha()
{
  for(auto cv: curtains_)
    for(auto c: cv)
      c->updateAlpha(alpha_property_->getFloat());
}

void SonarImageDisplay::updateColormapRange()
{
  color_map_->setRange(colormap_minimum_property_->getFloat(), colormap_maximum_property_->getFloat());
}

void SonarImageDisplay::processMessage(marine_acoustic_msgs::msg::RawSonarImage::ConstSharedPtr msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error transforming from frame '" << msg->header.frame_id << "' to frame '" << qPrintable( fixed_frame_ ) << "'");
    return;
  }

  // if(msg->image.dtype == marine_acoustic_msgs::SonarImageData::DTYPE_UINT16)
  // {
  //   color_map_->setRange(0, 1000);
  // }
  // else
  //   color_map_->setRange(-80, -20);

  if(image_pub_ && msg->image.beam_count > 0 && msg->samples_per_beam > 0)
  {
    const size_t beam_count = msg->image.beam_count;
    const size_t samples_per_beam = msg->samples_per_beam;
    const size_t total_samples = beam_count * samples_per_beam;

    std::vector<float> polar_values(total_samples, 0.0f);
    bool dtype_ok = false;

    switch(msg->image.dtype)
    {
      case marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT16:
      {
        const uint16_t* sonar_data = reinterpret_cast<const uint16_t*>(msg->image.data.data());
        for(size_t i = 0; i < total_samples; ++i)
          polar_values[i] = static_cast<float>(sonar_data[i]);
        dtype_ok = true;
        break;
      }
      case marine_acoustic_msgs::msg::SonarImageData::DTYPE_INT16:
      {
        const int16_t* sonar_data = reinterpret_cast<const int16_t*>(msg->image.data.data());
        for(size_t i = 0; i < total_samples; ++i)
          polar_values[i] = static_cast<float>(sonar_data[i]);
        dtype_ok = true;
        break;
      }
      case marine_acoustic_msgs::msg::SonarImageData::DTYPE_FLOAT32:
      {
        const float* sonar_data = reinterpret_cast<const float*>(msg->image.data.data());
        for(size_t i = 0; i < total_samples; ++i)
          polar_values[i] = sonar_data[i];
        dtype_ok = true;
        break;
      }
      default:
        RVIZ_COMMON_LOG_WARNING_STREAM("Unimplemented dtype for Image publish: " << msg->image.dtype);
        break;
    }

    if(dtype_ok)
    {
      for(float v : polar_values)
      {
        minimum_data_value_ = std::min(minimum_data_value_, v);
        maximum_data_value_ = std::max(maximum_data_value_, v);
      }
      minimum_data_value_property_->setFloat(minimum_data_value_);
      maximum_data_value_property_->setFloat(maximum_data_value_);
      colormap_minimum_property_->setMin(minimum_data_value_);
      colormap_maximum_property_->setMin(minimum_data_value_);
      colormap_minimum_property_->setMax(maximum_data_value_);
      colormap_maximum_property_->setMax(maximum_data_value_);

      sensor_msgs::msg::Image polar_image;
      polar_image.header = msg->header;
      polar_image.width = beam_count;
      polar_image.height = samples_per_beam;
      polar_image.encoding = "rgba8";
      polar_image.step = polar_image.width * 4;
      polar_image.data.resize(polar_image.step * polar_image.height);
      for(size_t i = 0; i < total_samples; ++i)
      {
        auto c = color_map_->lookup(polar_values[i]);
        polar_image.data[i * 4 + 0] = static_cast<uint8_t>(c.r * 255);
        polar_image.data[i * 4 + 1] = static_cast<uint8_t>(c.g * 255);
        polar_image.data[i * 4 + 2] = static_cast<uint8_t>(c.b * 255);
        polar_image.data[i * 4 + 3] = static_cast<uint8_t>(c.a * 255);
      }
      image_pub_->publish(polar_image);

      if(cartesian_image_pub_ && !msg->rx_angles.empty() && msg->rx_angles.size() >= beam_count)
      {
        double sound_speed = 1500.0;
        if(msg->ping_info.sound_speed > 0.0)
          sound_speed = msg->ping_info.sound_speed;
        double sample_length = (sound_speed / msg->sample_rate) / 2.0;

        double max_range = (msg->sample0 + samples_per_beam) * sample_length;
        double resolution = sample_length;
        int image_size = static_cast<int>(std::ceil(max_range / resolution));
        if(image_size < 1)
          image_size = 1;

        const float empty_value = -std::numeric_limits<float>::infinity();
        std::vector<float> cart_values(static_cast<size_t>(image_size) * image_size, empty_value);

        const int center = image_size / 2;
        for(size_t s = 0; s < samples_per_beam; ++s)
        {
          double r = (msg->sample0 + s) * sample_length;
          for(size_t b = 0; b < beam_count; ++b)
          {
            double theta = msg->rx_angles[b];
            double x_cart = r * std::sin(theta);
            double y_cart = r * std::cos(theta);

            int px = center + static_cast<int>(std::lround(x_cart / resolution));
            int py = static_cast<int>(std::lround(y_cart / resolution));

            if(px < 0 || px >= image_size || py < 0 || py >= image_size)
              continue;

            size_t idx = static_cast<size_t>(py) * image_size + px;
            float v = polar_values[b * samples_per_beam + s];
            if(v > cart_values[idx])
              cart_values[idx] = v;
          }
        }

        sensor_msgs::msg::Image cart_image;
        cart_image.header = msg->header;
        cart_image.width = image_size;
        cart_image.height = image_size;
        cart_image.encoding = "rgba8";
        cart_image.step = cart_image.width * 4;
        cart_image.data.resize(cart_image.step * cart_image.height);
        for(int i = 0; i < image_size * image_size; ++i)
        {
          if(std::isinf(cart_values[i]) && cart_values[i] < 0)
          {
            cart_image.data[i * 4 + 0] = 0;
            cart_image.data[i * 4 + 1] = 0;
            cart_image.data[i * 4 + 2] = 0;
            cart_image.data[i * 4 + 3] = 0;
            continue;
          }
          auto c = color_map_->lookup(cart_values[i]);
          cart_image.data[i * 4 + 0] = static_cast<uint8_t>(c.r * 255);
          cart_image.data[i * 4 + 1] = static_cast<uint8_t>(c.g * 255);
          cart_image.data[i * 4 + 2] = static_cast<uint8_t>(c.b * 255);
          cart_image.data[i * 4 + 3] = static_cast<uint8_t>(c.a * 255);
        }
        cartesian_image_pub_->publish(cart_image);
      }
    }
  }

  uint32_t sector_size = 4096;

  if(curtain_beam_ >= 0 && curtains_.empty()) // first time?
    if(msg->image.beam_count > 1) // only default to showing curtain for single beam
      curtain_beam_ = -1;

  if(curtain_beam_ >= 0 && (curtains_.empty() ||  (!curtains_.back().empty() && curtains_.back().front()->full())))
  {
    curtains_.push_back(std::vector<std::shared_ptr<SonarImageCurtain> >());
    while (curtains_.size() > curtain_length_ && !curtains_.empty())
    {
      curtains_.pop_front();
    }
  }

  int i = 0;
  uint32_t start_row = 0;
  while (start_row < msg->samples_per_beam)
  {
    uint32_t end_row = std::min(start_row+sector_size, msg->samples_per_beam);
    if(i >= fans_.size())
      fans_.push_back(std::make_shared<SonarImageFan>(context_->getSceneManager(), scene_node_, color_map_));
    fans_[i]->setMessage(msg, start_row, end_row);
    fans_[i]->setFramePosition( position );
    fans_[i]->setFrameOrientation( orientation );

    // if(msg->image.beam_count > 0)
    //   curtain_beam_ = msg->image.beam_count/2;

    if(curtain_beam_ >= 0 && curtain_length_ > 0)
    {
      if(i >= curtains_.back().size())
      {
        curtains_.back().push_back(std::make_shared<SonarImageCurtain>(context_->getSceneManager(), scene_node_, color_map_));
        updateAlpha();
      }
      curtains_.back()[i]->addMessage(msg, start_row, end_row, curtain_beam_, position, orientation);
      auto range = curtains_.back()[i]->getDataValueRange();
      minimum_data_value_ = std::min(minimum_data_value_, range.first);
      maximum_data_value_ = std::max(maximum_data_value_, range.second);

      minimum_data_value_property_->setFloat(minimum_data_value_);
      maximum_data_value_property_->setFloat(maximum_data_value_);

      colormap_minimum_property_->setMin(minimum_data_value_);
      colormap_maximum_property_->setMin(minimum_data_value_);
      colormap_minimum_property_->setMax(maximum_data_value_);
      colormap_maximum_property_->setMax(maximum_data_value_);
    }
    i++;
    start_row += sector_size-1;
  }
}

} // namespace rviz_sonar_image

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_sonar_image::SonarImageDisplay, rviz_common::Display)
