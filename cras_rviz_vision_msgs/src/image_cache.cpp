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

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
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
  const auto frameId = image->header.frame_id;
  auto& source = sources_[frameId];
  source.encoding = image->encoding;
  ++source.received;
  lastEncoding_ = image->encoding;
  lastError_.clear();

  cv_bridge::CvImageConstPtr cvImage;
  try {
    // Store the data in a format that can be uploaded to a texture without further conversions.
    cvImage = cv_bridge::toCvCopy(image, targetEncoding(image->encoding));
  } catch (const std::exception& e) {
    failed(frameId, image->encoding, e.what());
    return {0, 0};
  }

  return store(cvImage, rclcpp::Time(image->header.stamp), maxImageSize);
}

rclcpp::Time ImageCache::add(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image, size_t maxImageSize) {
  if (!image) {
    return {0, 0};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto frameId = image->header.frame_id;
  auto& source = sources_[frameId];
  source.encoding = image->format;
  ++source.received;
  lastEncoding_ = image->format;
  lastError_.clear();

  // The format of a compressed image says what it was compressed from (e.g. "rgb8; jpeg compressed bgr8"), so the
  // encoding before the first semicolon is the one the blob is decoded to.
  const auto separator = image->format.find(';');
  const auto decoded = separator == std::string::npos ? image->format : image->format.substr(0, separator);

  // JPEG can decode the image directly in a reduced size, which is much cheaper than decoding it in full and scaling
  // it down afterwards - and the previews are never shown in the original resolution. The reduction factor is picked
  // from the size of the previous image of the same camera (a camera does not change its resolution), so that the first
  // image is still decoded in full and the following ones are decoded about as big as they are needed.
  // Each camera has its own reduction factor, because each of them has its own resolution.
  cv_bridge::CvImageConstPtr cvImage;
  auto factor = 1;
  const auto sourceWidth = sourceWidths_.count(frameId) ? sourceWidths_.at(frameId) : 0;
  if (maxImageSize > 0 && sourceWidth > 0) {
    while (factor < 8 && sourceWidth > int64_t {2 * factor} *static_cast<int64_t>(maxImageSize)) {
      factor *= 2;
    }
  }
  // The previews are cut out of the images in the resolution the camera published (that is the resolution the
  // detections' bounding boxes are given in), so the cache has to know how big the decoded image is w.r.t. it.
  auto decodedScale = 1.;
  try {
    const cv::Mat blob(1, static_cast<int>(image->data.size()), CV_8UC1,
        const_cast<uint8_t*>(image->data.data()));  // NOLINT: imdecode only reads the data
    // The flags of the reduced decoding differ for every factor, so look the right one up in the table.
    static constexpr int REDUCED_FLAGS[] = {cv::IMREAD_COLOR, cv::IMREAD_REDUCED_COLOR_2,
      cv::IMREAD_REDUCED_COLOR_4, cv::IMREAD_REDUCED_COLOR_8};
    auto pixels = cv::imdecode(blob, REDUCED_FLAGS[factor == 1 ? 0 : (factor == 2 ? 1 : (factor == 4 ? 2 : 3))]);
    if (pixels.empty()) {
      throw std::runtime_error("the image could not be decoded");
    }
    // The reduced decoding is only a request the decoder does not have to honour (and the resolution of a camera is
    // known only from the images which came before), so check what really came out of the decoder before the reduction
    // is believed. A different size means the camera reports another resolution than the one assumed; that is the
    // resolution to use for the following images.
    const auto fullWidth = static_cast<double>(pixels.cols) * static_cast<double>(factor);
    if (sourceWidth > 0 && std::abs(fullWidth - static_cast<double>(sourceWidth)) <= factor) {
      decodedScale = 1. / static_cast<double>(factor);
    } else {
      sourceWidths_[frameId] = static_cast<int64_t>(fullWidth);
    }
    cvImage = cv_bridge::cvtColor(std::make_shared<const cv_bridge::CvImage>(
      cv_bridge::CvImage(image->header, "bgr8", std::move(pixels))), targetEncoding(decoded));
  } catch (const std::exception& e) {
    failed(frameId, image->format, e.what());
    return {0, 0};
  }

  return store(cvImage, rclcpp::Time(image->header.stamp), maxImageSize, decodedScale);
}

rclcpp::Time ImageCache::store(const cv_bridge::CvImageConstPtr& image, const rclcpp::Time& stamp,
    size_t maxImageSize, double decodedScale) {
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
  entry.frameId = image->header.frame_id;
  sourceWidths_.insert({entry.frameId, static_cast<int64_t>(width / decodedScale)});
  entry.scale = decodedScale;
  if (maxImageSize > 0 && std::max(width, height) > maxImageSize) {
    const auto scale = static_cast<double>(maxImageSize) / static_cast<double>(std::max(width, height));
    cv::resize(stored.image, stored.image, cv::Size(), scale, scale, cv::INTER_AREA);
    entry.scale *= scale;
  }
  entry.image = std::make_shared<const cv_bridge::CvImage>(std::move(stored));

  images_.push_back(std::move(entry));
  while (images_.size() > maxImages_) {
    images_.pop_front();
  }
  // The detections are usually stamped by the moment the camera took the image, while the image itself arrives later.
  // The cache therefore has to keep the images for a while, not just a few of them.
  if (keepDurationNs_ > 0) {
    const auto oldest = images_.back().stamp.nanoseconds() - keepDurationNs_;
    while (images_.size() > 4 && images_.front().stamp.nanoseconds() < oldest) {
      images_.pop_front();
    }
  }

  return images_.back().stamp;
}

void ImageCache::failed(const std::string& frameId, const std::string& encoding, const std::string& what) {
  lastError_ = "Could not convert image with format \"" + encoding + "\": " + what;
  // Remember it for the camera, so that the status can say which of them is broken while the others work.
  sources_[frameId].error = what;
  if (failedEncodings_.insert(encoding).second) {
    // Log this only once per encoding, the image topic may be republished at high rates.
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("cras_rviz_vision_msgs.ImageCache"),
      "Could not convert incoming images with format \"" << encoding << "\": " << what);
  }
}

CameraImage ImageCache::get(const rclcpp::Time& stamp, double tolerance, const std::string& frameId) const{
  if (tolerance < 0.) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto index = findIndex(stamp, static_cast<int64_t>(tolerance * 1e9), frameId);
  return index < 0 ? CameraImage{} : images_[static_cast<size_t>(index)];
}

CameraImage ImageCache::nearest(const rclcpp::Time& stamp, const std::string& frameId) const{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto index = findIndex(stamp, std::numeric_limits<int64_t>::max(), frameId);
  return index < 0 ? CameraImage{} : images_[static_cast<size_t>(index)];
}

int64_t ImageCache::nearestGapNs(const rclcpp::Time& stamp, const std::string& frameId) const {
  std::lock_guard<std::mutex> lock(mutex_);
  int64_t best = -1;
  for (const auto& image : images_) {
    if (!frameId.empty() && image.frameId != frameId) {
      continue;
    }
    const auto gap = std::abs(image.stamp.nanoseconds() - stamp.nanoseconds());
    if (best < 0 || gap < best) {
      best = gap;
    }
  }
  return best;
}

bool ImageCache::has(const rclcpp::Time& stamp, double tolerance, const std::string& frameId) const{
  if (tolerance < 0.) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return findIndex(stamp, static_cast<int64_t>(tolerance * 1e9), frameId) >= 0;
}

std::vector<ImageCache::Source> ImageCache::sources() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = sources_;
  for (const auto& image : images_) {
    ++result[image.frameId].images;
  }
  std::vector<Source> list;
  list.reserve(result.size());
  for (auto& entry : result) {
    entry.second.frameId = entry.first;
    list.push_back(entry.second);
  }
  return list;
}

size_t ImageCache::count(const std::string& frameId) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frameId.empty()) {
    return images_.size();
  }
  size_t num = 0;
  for (const auto& image : images_) {
    if (image.frameId == frameId) {
      ++num;
    }
  }
  return num;
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

void ImageCache::setKeepDuration(double seconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  keepDurationNs_ = seconds > 0. ? static_cast<int64_t>(seconds * 1e9) : 0;
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

int ImageCache::findIndex(const rclcpp::Time& stamp, int64_t toleranceNs, const std::string& frameId) const{
  int best = -1;
  int64_t bestDiff = std::numeric_limits<int64_t>::max();
  const int64_t wanted = stamp.nanoseconds();
  for (size_t i = 0; i < images_.size(); ++i) {
    // The images of a different camera are never the image of the detection, no matter how well their stamps match.
    if (!frameId.empty() && images_[i].frameId != frameId) {
      continue;
    }
    const int64_t diff = std::abs(images_[i].stamp.nanoseconds() - wanted);
    if (diff <= toleranceNs && diff < bestDiff) {
      bestDiff = diff;
      best = static_cast<int>(i);
    }
  }
  return best;
}

TextureCache::TextureCache(size_t maxTextures, std::string namePrefix)
    : namePrefix_(namePrefix.empty() ? "cras_rviz_vision_msgs" : std::move(namePrefix)),
      maxTextures_(std::max(size_t{1}, maxTextures)) {
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

  const auto scale = image.scale;
  const auto& whole = image.image->image;
  const auto source = sourceRect(image, region);
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
  entry.name = namePrefix_ + "_texture_" + std::to_string(id);
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

cv::Rect TextureCache::sourceRect(const CameraImage& image, const ImageRegion& region) {
  if (!image) {
    return {};
  }
  // The previews are shown in a much smaller resolution than the original image, so the image is kept reduced and the
  // requested region has to be scaled accordingly. An empty region means to show the whole image.
  const auto& whole = image.image->image;
  if (!(region.width > 0.) || !(region.height > 0.)) {
    return cv::Rect {0, 0, whole.cols, whole.rows};
  }
  auto wanted = cv::Rect {static_cast<int>(std::round(region.x * image.scale)),
    static_cast<int>(std::round(region.y * image.scale)),
    std::max(1, static_cast<int>(std::round(region.width * image.scale))),
    std::max(1, static_cast<int>(std::round(region.height * image.scale)))};
  // What reaches outside of the image is not cut away - the shown part is moved inside of the image instead, so that
  // the detection keeps the place it has in the middle of its picture and the preview shows the same cutout as the
  // visualization strip of the same detections, which moves its crops the same way. Only a part which is bigger than
  // the image itself has to become smaller.
  wanted.width = std::min(wanted.width, whole.cols);
  wanted.height = std::min(wanted.height, whole.rows);
  wanted.x = std::max(0, std::min(wanted.x, whole.cols - wanted.width));
  wanted.y = std::max(0, std::min(wanted.y, whole.rows - wanted.height));
  return wanted;
}

std::string TextureCache::peekMaterial(const std::string& key, unsigned* width, unsigned* height) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = entries_.find(key);
  if (entry == entries_.end() || entry->second.materialName.empty()) {
    return {};
  }
  entry->second.lastUse = usageCounter_;
  if (width) {
    *width = entry->second.width;
  }
  if (height) {
    *height = entry->second.height;
  }
  return entry->second.materialName;
}

void TextureCache::setLimit(size_t maxTextures) {
  std::lock_guard<std::mutex> lock(mutex_);
  maxTextures_ = std::max(size_t{1}, maxTextures);
  enforceLimitsLocked();
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

std::string TextureCache::createMaterial(const std::string& textureName, unsigned id) const {
  // The names of the rendering resources are shared by the whole process, so they have to tell the RViz instance and
  // the display instance they belong to.
  const auto name = namePrefix_ + "_material_" + std::to_string(id);
  const auto material = Ogre::MaterialManager::getSingleton().create(name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  material->setDepthCheckEnabled(false);
  material->setDepthWriteEnabled(false);
  material->setCullingMode(Ogre::CULL_NONE);
  auto* pass = material->getTechnique(0)->getPass(0);
  auto* textureUnit = pass->createTextureUnitState(textureName);
  textureUnit->setTextureFiltering(Ogre::TFO_BILINEAR);
  // The alpha of a preview comes from the colour of its vertices (the images themselves have no alpha channel), so that
  // the previews can be semi-transparent and let the scene behind them be seen. The picture is multiplied by the colour
  // of the vertices by default, so only the blending has to be set up here.
  pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
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
