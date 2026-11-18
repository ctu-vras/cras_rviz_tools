// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the camera image and texture caches.
 * \author Czech Technical University in Prague
 */

#include "cras_rviz_vision_msgs/image_cache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <OgreDataStream.h>
#include <OgreImage.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgproc.hpp>

namespace cras_rviz_vision_msgs {

namespace {

//! Uploads the given image as a texture of the given name. Must be called in the rendering thread.
void uploadTexture(const std::string& name, const cv::Mat& image, unsigned width, unsigned height) {
  const auto channels = static_cast<size_t>(image.channels());
  const auto format = channels == 1 ? Ogre::PF_L8 : (channels == 4 ? Ogre::PF_A8R8G8B8 : Ogre::PF_R8G8B8);
  // Ogre expects the pixels to be stored one after another, without any padding.
  const cv::Mat packed = image.isContinuous() ? image : image.clone();
  auto stream = Ogre::DataStreamPtr(new Ogre::MemoryDataStream(packed.data,
      packed.total() * packed.elemSize()));

  Ogre::Image ogreImage;
  ogreImage.loadRawData(stream, width, height, 1, format);

  auto& textureManager = Ogre::TextureManager::getSingleton();
  const auto texture = textureManager.createManual(name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, width, height, 1, 0, format,
      Ogre::TU_STATIC);
  texture->loadImage(ogreImage);
}

}  // namespace

ImageCache::ImageCache(size_t maxImages)
    : maxImages_(std::max(size_t{1}, maxImages)) {
}

rclcpp::Time ImageCache::add(const sensor_msgs::msg::Image::ConstSharedPtr& image, size_t maxImageSize) {
  if (!image) {
    return {0, 0};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  lastEncoding_ = image->encoding;
  lastError_.clear();

  cv_bridge::CvImageConstPtr cvImage;
  try {
    // Store the data in a format that can be uploaded to a texture without further conversions.
    cvImage = cv_bridge::toCvCopy(image, targetEncoding(image->encoding));
  } catch (const std::exception& e) {
    failed(image->encoding, e.what());
    return {0, 0};
  }

  return store(cvImage, rclcpp::Time(image->header.stamp), maxImageSize);
}

rclcpp::Time ImageCache::add(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image, size_t maxImageSize) {
  if (!image) {
    return {0, 0};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  lastEncoding_ = image->format;
  lastError_.clear();

  // The format of a compressed image says what it was compressed from (e.g. "rgb8; jpeg compressed bgr8"), so the
  // encoding before the first semicolon is the one the blob is decoded to.
  const auto separator = image->format.find(';');
  const auto decoded = separator == std::string::npos ? image->format : image->format.substr(0, separator);

  cv_bridge::CvImageConstPtr cvImage;
  try {
    cvImage = cv_bridge::toCvCopy(image, targetEncoding(decoded));
  } catch (const std::exception& e) {
    failed(image->format, e.what());
    return {0, 0};
  }

  return store(cvImage, rclcpp::Time(image->header.stamp), maxImageSize);
}

rclcpp::Time ImageCache::store(const cv_bridge::CvImageConstPtr& image, const rclcpp::Time& stamp,
    size_t maxImageSize) {
  if (!image || image->image.empty()) {
    lastError_ = "Received an empty image.";
    return {0, 0};
  }

  // The previews are never shown in the original resolution, so do not waste memory with the full images.
  cv_bridge::CvImage stored = *image;
  const auto width = static_cast<size_t>(stored.image.cols);
  const auto height = static_cast<size_t>(stored.image.rows);
  CameraImage entry;
  entry.stamp = stamp;
  if (maxImageSize > 0 && std::max(width, height) > maxImageSize) {
    entry.scale = static_cast<double>(maxImageSize) / static_cast<double>(std::max(width, height));
    cv::resize(stored.image, stored.image, cv::Size(), entry.scale, entry.scale, cv::INTER_AREA);
  }
  entry.image = std::make_shared<const cv_bridge::CvImage>(std::move(stored));

  images_.push_back(std::move(entry));
  while (images_.size() > maxImages_) {
    images_.pop_front();
  }

  return images_.back().stamp;
}

void ImageCache::failed(const std::string& encoding, const std::string& what) {
  lastError_ = "Could not convert image with format \"" + encoding + "\": " + what;
  if (failedEncodings_.insert(encoding).second) {
    // Log this only once per encoding, the image topic may be republished at high rates.
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("cras_rviz_vision_msgs.ImageCache"),
      "Could not convert incoming images with format \"" << encoding << "\": " << what);
  }
}

CameraImage ImageCache::get(const rclcpp::Time& stamp, double tolerance) const{
  if (tolerance < 0.) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto index = findIndex(stamp, static_cast<int64_t>(tolerance * 1e9));
  return index < 0 ? CameraImage{} : images_[static_cast<size_t>(index)];
}

bool ImageCache::has(const rclcpp::Time& stamp, double tolerance) const{
  if (tolerance < 0.) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return findIndex(stamp, static_cast<int64_t>(tolerance * 1e9)) >= 0;
}

std::string ImageCache::lastEncoding() const{
  std::lock_guard<std::mutex> lock(mutex_);
  return lastEncoding_;
}

std::string ImageCache::lastError() const{
  std::lock_guard<std::mutex> lock(mutex_);
  return lastError_;
}

void ImageCache::setMaxImages(size_t maxImages) {
  std::lock_guard<std::mutex> lock(mutex_);
  maxImages_ = std::max(size_t{1}, maxImages);
  while (images_.size() > maxImages_) {
    images_.pop_front();
  }
}

void ImageCache::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  images_.clear();
}

size_t ImageCache::size() const{
  std::lock_guard<std::mutex> lock(mutex_);
  return images_.size();
}

std::string ImageCache::targetEncoding(const std::string& encoding) {
  if (encoding == sensor_msgs::image_encodings::MONO8 || encoding == sensor_msgs::image_encodings::MONO16) {
    return sensor_msgs::image_encodings::MONO8;
  }
  if (encoding == sensor_msgs::image_encodings::BGRA8 || encoding == sensor_msgs::image_encodings::RGBA8) {
    return sensor_msgs::image_encodings::BGRA8;
  }
  return sensor_msgs::image_encodings::BGR8;
}

int ImageCache::findIndex(const rclcpp::Time& stamp, int64_t toleranceNs) const{
  int best = -1;
  int64_t bestDiff = std::numeric_limits<int64_t>::max();
  const int64_t wanted = stamp.nanoseconds();
  for (size_t i = 0; i < images_.size(); ++i) {
    const int64_t diff = std::abs(images_[i].stamp.nanoseconds() - wanted);
    if (diff <= toleranceNs && diff < bestDiff) {
      bestDiff = diff;
      best = static_cast<int>(i);
    }
  }
  return best;
}

TextureCache::TextureCache(size_t maxTextures)
    : maxTextures_(std::max(size_t{1}, maxTextures)) {
}

TextureCache::~TextureCache() {
  // The resources are destroyed in enforceLimits(), which must be called from the rendering thread. The display is
  // (and hence also this class) destroyed from the rendering thread, so this is fine.
  enforceLimits();
}

std::string TextureCache::getMaterial(const CameraImage& image, const ImageRegion& region, unsigned targetWidth,
    const std::string& key, unsigned& width, unsigned& height) {
  width = 0;
  height = 0;
  if (!image || targetWidth == 0) {
    return {};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  ++usageCounter_;
  enforceLimitsLocked();

  auto& entry = entries_[key];
  entry.lastUse = usageCounter_;

  // The previews are shown in a much smaller resolution than the original image, so work with the stored (scaled)
  // image and scale the requested region accordingly. An empty region means to show the whole image.
  const auto scale = image.scale;
  const auto& whole = image.image->image;
  auto source = region.width > 0. && region.height > 0. ? cv::Rect(
      std::max(0, static_cast<int>(std::round(region.x * scale))),
      std::max(0, static_cast<int>(std::round(region.y * scale))),
      std::max(1, static_cast<int>(std::round(region.width * scale))),
      std::max(1, static_cast<int>(std::round(region.height * scale)))) :
                  cv::Rect(0, 0, whole.cols, whole.rows);
  source &= cv::Rect(0, 0, whole.cols, whole.rows);
  if (source.width < 1 || source.height < 1) {
    retiredEntries_.push_back(entry);
    entries_.erase(key);
    return {};
  }

  // Do not create textures with a needlessly high resolution (twice the resolution of the shown part of the image is
  // more than enough).
  const auto textureWidth = static_cast<unsigned>(
    std::max(1., std::min(static_cast<double>(targetWidth), 2. * static_cast<double>(source.width))));
  const auto textureHeight = static_cast<unsigned>(std::max(1., static_cast<double>(textureWidth) *
    static_cast<double>(source.height) / static_cast<double>(source.width)));

  const bool sameContent = entry.width == textureWidth && entry.height == textureHeight &&
                           entry.stamp.nanoseconds() == image.stamp.nanoseconds() && entry.imageScale == scale &&
                           std::abs(entry.region.x - region.x) < 0.5 && std::abs(entry.region.y - region.y) < 0.5 &&
                           std::abs(entry.region.width - region.width) < 0.5 &&
                           std::abs(entry.region.height - region.height) < 0.5;
  if (sameContent && !entry.materialName.empty()) {
    width = entry.width;
    height = entry.height;
    return entry.materialName;  // the material is up-to-date
  }

  const cv::Mat cropped = whole(source);
  cv::Mat resized;
  const auto interpolation = static_cast<int>(textureWidth) < source.width ? cv::INTER_AREA : cv::INTER_LINEAR;
  cv::resize(cropped, resized, cv::Size(static_cast<int>(textureWidth), static_cast<int>(textureHeight)), 0, 0,
    interpolation);

  if (!entry.name.empty()) {
    // The rendering resources cannot be resized, so create new ones and keep the old ones until they can be safely
    // destroyed.
    retiredEntries_.push_back(entry);
  }
  const auto id = ++textureCounter_;
  entry.name = "cras_rviz_vision_msgs_texture_" + std::to_string(id);
  entry.stamp = image.stamp;
  entry.region = region;
  entry.imageScale = scale;
  entry.width = textureWidth;
  entry.height = textureHeight;

  uploadTexture(entry.name, resized, textureWidth, textureHeight);
  entry.materialName = createMaterial(entry.name, id);

  width = textureWidth;
  height = textureHeight;
  return entry.materialName;
}

void TextureCache::release(const std::string& key) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = entries_.find(key);
  if (entry != entries_.end()) {
    retiredEntries_.push_back(entry->second);
    entries_.erase(entry);
  }
}

void TextureCache::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& entry : entries_) {
    retiredEntries_.push_back(entry.second);
  }
  entries_.clear();
}

void TextureCache::enforceLimits() {
  // The resource managers are not thread-safe, hence the resources may only be destroyed here, which is expected to
  // be called from the rendering thread.
  std::lock_guard<std::mutex> lock(mutex_);
  enforceLimitsLocked();
  for (const auto& entry : retiredEntries_) {
    destroyNow(entry);
  }
  retiredEntries_.clear();
}

void TextureCache::enforceLimitsLocked() {
  while (entries_.size() > maxTextures_) {
    auto worst = entries_.begin();
    for (auto entry = entries_.begin(); entry != entries_.end(); ++entry) {
      if (entry->second.lastUse < worst->second.lastUse) {
        worst = entry;
      }
    }
    retiredEntries_.push_back(worst->second);
    entries_.erase(worst);
  }
}

std::string TextureCache::createMaterial(const std::string& textureName, unsigned id) {
  const auto name = "cras_rviz_vision_msgs_material_" + std::to_string(id);
  const auto material = Ogre::MaterialManager::getSingleton().create(name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  material->setDepthCheckEnabled(false);
  material->setDepthWriteEnabled(false);
  material->setCullingMode(Ogre::CULL_NONE);
  auto* pass = material->getTechnique(0)->getPass(0);
  auto* textureUnit = pass->createTextureUnitState(textureName);
  textureUnit->setTextureFiltering(Ogre::TFO_BILINEAR);
  return name;
}

void TextureCache::destroyNow(const Entry& entry) {
  auto* materialManager = Ogre::MaterialManager::getSingletonPtr();
  if (materialManager && !entry.materialName.empty() && materialManager->resourceExists(entry.materialName)) {
    materialManager->remove(entry.materialName);
  }
  auto* textureManager = Ogre::TextureManager::getSingletonPtr();
  if (textureManager && !entry.name.empty() && textureManager->resourceExists(entry.name)) {
    textureManager->remove(entry.name);
  }
}

size_t TextureCache::size() const{
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.size();
}

}  // namespace cras_rviz_vision_msgs
