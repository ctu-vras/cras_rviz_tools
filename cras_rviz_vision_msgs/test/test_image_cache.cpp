// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Tests of the cache of the camera images.
 * \author Czech Technical University in Prague
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "cras_rviz_vision_msgs/image_cache.hpp"

namespace {

/**
 * \brief Image message with a constant image of the given size and encoding.
 * \param[in] nanoseconds Timestamp of the image.
 * \param[in] width Width of the image.
 * \param[in] height Height of the image.
 * \param[in] encoding Encoding of the image.
 */
sensor_msgs::msg::Image::SharedPtr makeImage(int64_t nanoseconds, uint32_t width = 64, uint32_t height = 32,
    const std::string& encoding = sensor_msgs::image_encodings::BGR8) {
  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->header.stamp = builtin_interfaces::msg::Time();
  message->header.stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  message->header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  message->height = height;
  message->width = width;
  message->encoding = encoding;
  const auto channels = static_cast<uint32_t>(encoding == sensor_msgs::image_encodings::MONO8 ||
                                              encoding == sensor_msgs::image_encodings::MONO16 ? 1 :
                                                (encoding == sensor_msgs::image_encodings::RGBA8 ||
                                                 encoding == sensor_msgs::image_encodings::BGRA8 ? 4 : 3));
  message->step = width * channels;
  message->data = std::vector<uint8_t>(static_cast<size_t>(message->step) * height, 73);
  return message;
}

/**
 * \brief Compressed image message with a constant image of the given size.
 * \param[in] nanoseconds Timestamp of the image.
 * \param[in] width Width of the image.
 * \param[in] height Height of the image.
 * \param[in] format Format the image is reported as (the part before the first semicolon is what it was compressed
 *   from).
 */
sensor_msgs::msg::CompressedImage::SharedPtr makeCompressed(int64_t nanoseconds, uint32_t width = 64,
    uint32_t height = 32, const std::string& format = "bgr8; png compressed rgb8") {
  auto message = std::make_shared<sensor_msgs::msg::CompressedImage>();
  message->header.stamp = builtin_interfaces::msg::Time();
  message->header.stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  message->header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  message->format = format;
  const cv::Mat image(static_cast<int>(height), static_cast<int>(width), CV_8UC3, cv::Scalar(73, 73, 73));
  cv::imencode(".png", image, message->data);  // PNG is lossless, so the pixel values can be checked
  return message;
}

}  // namespace

TEST(ImageCache, FindsTheClosestImage)
{
  cras_rviz_vision_msgs::ImageCache cache;

  cache.add(makeImage(1000000000), 0);
  cache.add(makeImage(2000000000), 0);

  ASSERT_EQ(cache.size(), 2);
  EXPECT_TRUE(static_cast<bool>(cache.get(rclcpp::Time(1000000000), 0.01)));
  EXPECT_TRUE(static_cast<bool>(cache.get(rclcpp::Time(1990000000), 0.02)));
  EXPECT_FALSE(static_cast<bool>(cache.get(rclcpp::Time(1400000000), 0.1)));  // 0.4 s away from both
  EXPECT_TRUE(static_cast<bool>(cache.get(rclcpp::Time(1400000000), 0.5)));  // the closest one is 0.4 s old
  EXPECT_FALSE(static_cast<bool>(cache.get(rclcpp::Time(3000000000), 0.5)));
  EXPECT_FALSE(static_cast<bool>(cache.get(rclcpp::Time(1000000000), -1.)));  // negative tolerance is disabled
  EXPECT_TRUE(static_cast<bool>(cache.get(rclcpp::Time(1000000000), 0.)));  // zero tolerance matches exactly
  EXPECT_FALSE(static_cast<bool>(cache.get(rclcpp::Time(1000000001), 0.)));

  EXPECT_TRUE(cache.has(rclcpp::Time(2000000000), 0.01));
  EXPECT_FALSE(cache.has(rclcpp::Time(3000000000), 0.01));
}

TEST(ImageRegion, PaddingShowsTheSameCutoutAsTheVisualizationNodeOfTheDetections)
{
  const auto box = cras_rviz_vision_msgs::ImageRegion {20., 10., 60., 50.};
  // The picture of an object is its bounding box made the given factor bigger, i.e. here a tenth of its size of the
  // surroundings on every side. The visualization node of the detections crops the same part of the same picture out
  // (its bbox_scale parameter, with the same default) and the user click in that picture, so the two have to understand
  // the number in the same way - otherwise the preview of a clicked detection shows something else than what was
  // clicked, which is what the operator notices as a wrong cutout.
  const auto shown = box.padded(1.2);
  EXPECT_DOUBLE_EQ(shown.x, 14.);
  EXPECT_DOUBLE_EQ(shown.y, 5.);
  EXPECT_DOUBLE_EQ(shown.width, 72.);
  EXPECT_DOUBLE_EQ(shown.height, 60.);
  EXPECT_DOUBLE_EQ(box.padded(1.).width, 60.);  // without any surroundings, the bounding box alone
}

TEST(TextureCache, ShownPartIsMovedInsideOfTheImageInsteadOfBeingCutThere)
{
  cras_rviz_vision_msgs::ImageCache cache;
  cache.add(makeImage(1000000000, 200, 100), 0);
  const auto image = cache.get(rclcpp::Time(1000000000), 1.);
  ASSERT_TRUE(static_cast<bool>(image));

  // A detection at the border of the image and the surroundings around it: what does not fit beside the box has to be
  // taken on its other side, so that the shown part keeps the size the padding asks for and the object stays in the
  // middle of it. Cutting the region at the border instead would show a different cutout than the tile of the same
  // detection in the visualization strip, which is what the user clicks on.
  const auto shown = cras_rviz_vision_msgs::TextureCache::sourceRect(image,
      cras_rviz_vision_msgs::ImageRegion{-10., -10., 72., 60.});
  EXPECT_EQ(shown.x, 0);
  EXPECT_EQ(shown.y, 0);
  EXPECT_EQ(shown.width, 72);
  EXPECT_EQ(shown.height, 60);
}

TEST(ImageCache, NearestImageIsFoundWhateverItsAge)
{
  cras_rviz_vision_msgs::ImageCache cache;

  auto left = makeImage(1000000000);
  left->header.frame_id = "left_camera";
  cache.add(left, 0);
  auto right = makeImage(2000000000);
  right->header.frame_id = "right_camera";
  cache.add(right, 0);

  // No tolerance to give here - the picture of the detection the user highlighted is the closest one there is, however
  // old it is, and of the camera the detection was seen by.
  EXPECT_EQ(cache.nearest(rclcpp::Time(900000000)).stamp.nanoseconds(), 1000000000);
  EXPECT_EQ(cache.nearest(rclcpp::Time(1100000000)).stamp.nanoseconds(), 1000000000);
  EXPECT_EQ(cache.nearest(rclcpp::Time(3000000000), "left_camera").stamp.nanoseconds(), 1000000000);
  EXPECT_EQ(cache.nearest(rclcpp::Time(1500000000), "right_camera").stamp.nanoseconds(), 2000000000);
  EXPECT_FALSE(static_cast<bool>(cache.nearest(rclcpp::Time(1500000000), "back_camera")));
}

TEST(ImageCache, KeepsOnlyTheNewestImages)
{
  cras_rviz_vision_msgs::ImageCache cache(2);

  cache.add(makeImage(1000000000), 0);
  cache.add(makeImage(2000000000), 0);
  ASSERT_EQ(cache.size(), 2);
  cache.add(makeImage(3000000000), 0);

  ASSERT_EQ(cache.size(), 2);
  EXPECT_FALSE(static_cast<bool>(cache.get(rclcpp::Time(1000000000), 0.01)));
  EXPECT_TRUE(static_cast<bool>(cache.get(rclcpp::Time(2000000000), 0.01)));
  EXPECT_TRUE(static_cast<bool>(cache.get(rclcpp::Time(3000000000), 0.01)));

  cache.setMaxImages(1);
  EXPECT_EQ(cache.size(), 1);
  cache.clear();
  EXPECT_EQ(cache.size(), 0);
}

TEST(ImageCache, ScalesDownLargeImages)
{
  cras_rviz_vision_msgs::ImageCache cache;

  const auto stamp = cache.add(makeImage(1000000000, 1280, 720), 640);

  ASSERT_NE(stamp.nanoseconds(), 0);
  const auto image = cache.get(stamp, 0.1);
  ASSERT_TRUE(static_cast<bool>(image));
  EXPECT_EQ(image.image->image.cols, 640);
  EXPECT_EQ(image.image->image.rows, 360);
  EXPECT_DOUBLE_EQ(image.scale, .5);
}

TEST(ImageCache, TellsWhichPartOfTheImageAPreviewShows)
{
  cras_rviz_vision_msgs::ImageCache cache;
  const auto stamp = cache.add(makeImage(1000000000, 1280, 720), 640);  // the image is kept in half the resolution
  const auto image = cache.get(stamp, 0.1);
  ASSERT_TRUE(static_cast<bool>(image));
  ASSERT_DOUBLE_EQ(image.scale, .5);

  // The whole image when no part of it is asked for.
  EXPECT_EQ(cras_rviz_vision_msgs::TextureCache::sourceRect(image, {}), cv::Rect(0, 0, 640, 360));
  // A part of it, in the resolution in which the image is really kept.
  EXPECT_EQ(cras_rviz_vision_msgs::TextureCache::sourceRect(image, {100., 200., 40., 80.}), cv::Rect(50, 100, 20, 40));
  // What reaches outside of the image is not cut away - the shown part is moved inside of the image, keeping the shape
  // the region asks for, so that a detection at the border is shown in the same cutout as the same detection in the
  // middle of the image (and as its picture in the visualization strip, which moves its crops the same way).
  EXPECT_EQ(cras_rviz_vision_msgs::TextureCache::sourceRect(image, {1200., 200., 200., 200.}),
      cv::Rect(540, 100, 100, 100));
  // A region bigger than the image itself has to become as big as the image is.
  EXPECT_EQ(cras_rviz_vision_msgs::TextureCache::sourceRect(image, {-1000., -1000., 4000., 4000.}),
      cv::Rect(0, 0, 640, 360));
  EXPECT_EQ(cras_rviz_vision_msgs::TextureCache::sourceRect(cras_rviz_vision_msgs::CameraImage(),
      {1., 2., 3., 4.}), cv::Rect());
}

TEST(ImageCache, KeepsSmallImagesUnscaled)
{
  cras_rviz_vision_msgs::ImageCache cache;

  const auto stamp = cache.add(makeImage(1000000000, 320, 200), 640);

  const auto image = cache.get(stamp, 0.1);
  ASSERT_TRUE(static_cast<bool>(image));
  EXPECT_EQ(image.image->image.cols, 320);
  EXPECT_EQ(image.image->image.rows, 200);
  EXPECT_DOUBLE_EQ(image.scale, 1.);
}

TEST(ImageCache, ConvertsEncodings)
{
  cras_rviz_vision_msgs::ImageCache cache;

  const auto mono = cache.add(makeImage(1000000000, 64, 32, sensor_msgs::image_encodings::MONO8), 0);
  ASSERT_NE(mono.nanoseconds(), 0);
  EXPECT_EQ(cache.get(mono, .1).image->encoding, sensor_msgs::image_encodings::MONO8);

  const auto rgba = cache.add(makeImage(2000000000, 64, 32, sensor_msgs::image_encodings::RGBA8), 0);
  ASSERT_NE(rgba.nanoseconds(), 0);
  EXPECT_EQ(cache.get(rgba, .1).image->encoding, sensor_msgs::image_encodings::BGRA8);

  const auto bgra = cache.add(makeImage(3000000000, 64, 32, sensor_msgs::image_encodings::BGRA8), 0);
  ASSERT_NE(bgra.nanoseconds(), 0);
  EXPECT_EQ(cache.get(bgra, .1).image->encoding, sensor_msgs::image_encodings::BGRA8);
}

TEST(ImageCache, ReportsUnusableImages)
{
  cras_rviz_vision_msgs::ImageCache cache;

  EXPECT_EQ(cache.add(sensor_msgs::msg::Image::ConstSharedPtr(), 0).nanoseconds(), 0);

  const auto stamp = cache.add(makeImage(1000000000, 64, 32, "some_nonsense_encoding"), 0);
  EXPECT_EQ(stamp.nanoseconds(), 0);
  EXPECT_EQ(cache.size(), 0);
  EXPECT_FALSE(cache.lastError().empty());
  EXPECT_EQ(cache.lastEncoding(), "some_nonsense_encoding");

  // An empty image is not stored either.
  auto empty = makeImage(2000000000, 0, 0);
  empty->data.clear();
  EXPECT_EQ(cache.add(empty, 0).nanoseconds(), 0);
  EXPECT_FALSE(cache.lastError().empty());
}

TEST(ImageCache, AddsCompressedImages)
{
  cras_rviz_vision_msgs::ImageCache cache;

  const auto stamp = cache.add(makeCompressed(1000000000, 128, 64), 0);

  ASSERT_NE(stamp.nanoseconds(), 0);
  const auto image = cache.get(stamp, 0.01);
  ASSERT_TRUE(static_cast<bool>(image));
  EXPECT_EQ(image.image->image.cols, 128);
  EXPECT_EQ(image.image->image.rows, 64);
  EXPECT_EQ(image.image->encoding, sensor_msgs::image_encodings::BGR8);
  EXPECT_EQ(image.image->image.at<cv::Vec3b>(10, 10), (cv::Vec3b(73, 73, 73)));
  EXPECT_TRUE(cache.lastError().empty());
}

TEST(ImageCache, ScalesAndMixesCompressedAndRawImages)
{
  cras_rviz_vision_msgs::ImageCache cache;

  const auto stamp = cache.add(makeCompressed(1000000000, 1280, 720), 640);
  ASSERT_NE(stamp.nanoseconds(), 0);
  const auto compressed = cache.get(stamp, 0.01);
  ASSERT_TRUE(static_cast<bool>(compressed));
  EXPECT_EQ(compressed.image->image.cols, 640);
  EXPECT_EQ(compressed.image->image.rows, 360);
  EXPECT_DOUBLE_EQ(compressed.scale, .5);

  // Both kinds of images are kept together and matched by their timestamp.
  const auto rawStamp = cache.add(makeImage(1100000000, 64, 32), 0);
  ASSERT_NE(rawStamp.nanoseconds(), 0);
  ASSERT_EQ(cache.size(), 2);
  EXPECT_TRUE(static_cast<bool>(cache.get(rclcpp::Time(1050000000), 0.06)));
  EXPECT_TRUE(static_cast<bool>(cache.get(rclcpp::Time(1050000000), 0.2)));
}

TEST(ImageCache, ReportsBrokenCompressedImages)
{
  cras_rviz_vision_msgs::ImageCache cache;

  EXPECT_EQ(cache.add(sensor_msgs::msg::CompressedImage::ConstSharedPtr(), 0).nanoseconds(), 0);

  auto broken = makeCompressed(1000000000);
  broken->data = {1, 2, 3, 4, 5};  // not a PNG stream at all
  EXPECT_EQ(cache.add(broken, 0).nanoseconds(), 0);
  EXPECT_EQ(cache.size(), 0);
  EXPECT_FALSE(cache.lastError().empty());
  EXPECT_EQ(cache.lastEncoding(), broken->format);
}
