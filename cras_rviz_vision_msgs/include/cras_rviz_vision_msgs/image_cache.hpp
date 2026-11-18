// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Cache of camera images used as previews of the visualized detections.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <OgreTextureManager.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.hpp>

namespace cras_rviz_vision_msgs {

/**
 * \brief One stored camera image.
 */
struct CameraImage {
  cv_bridge::CvImageConstPtr image;  //!< The (possibly downscaled) image.
  rclcpp::Time stamp {0, 0};  //!< Stamp of the original image.
  double scale {1.};  //!< Size of the stored image w.r.t. the original image.

  explicit operator bool() const {
    return static_cast<bool>(image);
  }
};

/**
 * \brief Holds the last few images received on the camera image topic.
 *
 * The images are never shown in their original resolution (they only serve as previews of the detections), so only a
 * downscaled version of each image is kept in memory.
 *
 * The class is thread-safe. The images may be arriving in a ROS callback thread, while they are used by the rendering
 * thread.
 */
class ImageCache {
public:
  /**
   * \param[in] maxImages Maximal number of images to keep in the cache.
   */
  explicit ImageCache(size_t maxImages = 4);

  ImageCache(const ImageCache&) = delete;
  ImageCache& operator=(const ImageCache&) = delete;

  /**
   * \brief Adds a new image to the cache (downscaling it if needed).
   * \param[in] image The image to add.
   * \param[in] maxImageSize Maximal width/height of the stored (scaled) image in pixels. 0 disables scaling.
   * \return The stamp of the added image, or zero time if the image could not be used.
   */
  rclcpp::Time add(const sensor_msgs::msg::Image::ConstSharedPtr& image, size_t maxImageSize);

  /**
   * \brief Adds a compressed camera image to the cache (decoding and downscaling it if needed).
   * \param[in] image The image to add.
   * \param[in] maxImageSize Maximal width/height of the stored (scaled) image in pixels. 0 disables scaling.
   * \return The stamp of the added image, or zero time if the image could not be used.
   */
  rclcpp::Time add(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image, size_t maxImageSize);

  /**
   * \brief Returns the image closest to the given stamp (within the given tolerance).
   * \param[in] stamp Stamp of the image.
   * \param[in] tolerance Maximal allowed time difference in seconds. Negative value returns no image.
   * \return The image, or an empty CameraImage if no matching image is in the cache.
   */
  CameraImage get(const rclcpp::Time& stamp, double tolerance) const;

  /**
   * \brief Checks whether an image with the given stamp is in the cache.
   */
  bool has(const rclcpp::Time& stamp, double tolerance) const;

  /**
   * \brief Encoding of the most recently added image (for status reporting).
   */
  std::string lastEncoding() const;
  /**
   * \brief Reason why the most recently added image could not be used (empty if it is usable).
   */
  std::string lastError() const;

  /**
   * \brief Sets the maximal number of kept images. Older images are dropped.
   */
  void setMaxImages(size_t maxImages);

  /**
   * \brief Drops all images.
   */
  void clear();

  /**
   * \brief Number of images currently in the cache.
   */
  size_t size() const;

protected:
  //! Encoding in which the given input encoding should be stored so that it can be uploaded to a texture.
  static std::string targetEncoding(const std::string& encoding);

  /**
   * \brief Stores an already decoded image (the downscaling and the cache size limit are applied here).
   * \note The mutex has to be held by the caller.
   */
  rclcpp::Time store(const cv_bridge::CvImageConstPtr& image, const rclcpp::Time& stamp, size_t maxImageSize);

  //! Remembers why the last image could not be used and logs it (only once per encoding, to not spam the log).
  void failed(const std::string& encoding, const std::string& what);

  //! Index of the image closest to the given stamp (within the given tolerance), or -1 if there is no such image.
  int findIndex(const rclcpp::Time& stamp, int64_t toleranceNs) const;

  mutable std::mutex mutex_;  //!< Guards everything below this line.
  size_t maxImages_;  //!< Maximal number of kept images.
  std::deque<CameraImage> images_;  //!< Cached images, newest at the back.
  std::set<std::string> failedEncodings_;  //!< Encodings that failed to convert (to avoid log spamming).
  std::string lastEncoding_;  //!< Encoding of the last added image.
  std::string lastError_;  //!< Error of the last added image.
};

/**
 * \brief A rectangle in the coordinates of the original (non-scaled) camera image.
 */
struct ImageRegion {
  double x {0};  //!< Horizontal position of the left edge.
  double y {0};  //!< Vertical position of the top edge.
  double width {0};  //!< Width.
  double height {0};  //!< Height.

  //! Region with the given fraction added around the original region.
  ImageRegion padded(double fraction) const{
    const auto addX = width * fraction;
    const auto addY = height * fraction;
    return {x - addX, y - addY, width + 2. * addX, height + 2. * addY};
  }
};

/**
 * \brief Materials showing (a part of) the camera images.
 *
 * The materials (and the textures they use) are created lazily and reused as long as possible. Each material has its
 * key (typically the ID of the detection it belongs to); when the same key is requested again, the material is
 * recreated only if the shown image, region or resolution changed.
 *
 * The rendering resources must not be destroyed outside of the rendering thread. Therefore the resources that are not
 * needed anymore are only scheduled for destruction and destroyed later in enforceLimits(), which must be called from
 * the rendering thread.
 */
class TextureCache {
public:
  /**
   * \param[in] maxTextures Maximal number of textures to keep. The least recently used ones are dropped.
   */
  explicit TextureCache(size_t maxTextures = 32);
  ~TextureCache();

  TextureCache(const TextureCache&) = delete;
  TextureCache& operator=(const TextureCache&) = delete;

  /**
   * \brief Returns a material showing the given region of the given image.
   * \note Must be called from the rendering thread.
   * \param[in] image The image to show.
   * \param[in] region Which part of the image to show (in the original image coordinates). If the width or height is
   *   zero, the whole image is shown.
   * \param[in] targetWidth Requested width of the texture in pixels.
   * \param[in] key Identifies the material. The same key always reuses the same material.
   * \param[out] width Width of the returned texture.
   * \param[out] height Height of the returned texture.
   * \return Name of the material, or an empty string if nothing can be shown.
   */
  std::string getMaterial(const CameraImage& image, const ImageRegion& region, unsigned targetWidth,
      const std::string& key, unsigned& width, unsigned& height);

  //! Forgets all textures belonging to the given key.
  void release(const std::string& key);

  //! Forgets all textures.
  void clear();

  /**
   * \brief Destroys the textures that are not needed anymore.
   * \note Must be called from the rendering thread.
   */
  void enforceLimits();

  //! Number of kept textures.
  size_t size() const;

protected:
  //! One cached material and the texture it shows.
  struct Entry {
    std::string name;  //!< Name of the texture.
    std::string materialName;  //!< Name of the material showing the texture.
    rclcpp::Time stamp;  //!< Stamp of the shown image.
    ImageRegion region;  //!< Shown region of the image.
    double imageScale {1.};  //!< Scale of the stored image w.r.t. the original image.
    unsigned width {0};  //!< Width of the texture.
    unsigned height {0};  //!< Height of the texture.
    uint64_t lastUse {0};  //!< When the texture was used the last time.
  };

  //! Drops the least recently used entries until the number of entries fits the limit. Must hold mutex_.
  void enforceLimitsLocked();

  //! Creates the material showing the given texture.
  static std::string createMaterial(const std::string& textureName, unsigned id);

  //! Destroys the given texture and material right now. Must hold mutex_ and must be called in the rendering thread.
  static void destroyNow(const Entry& entry);

  mutable std::mutex mutex_;  //!< Guards everything below this line.
  std::map<std::string, Entry> entries_;  //!< Key to Entry.
  std::vector<Entry> retiredEntries_;  //!< Entries to destroy in the rendering thread.
  size_t maxTextures_;  //!< Maximal number of kept textures.
  uint64_t usageCounter_ {0};  //!< Increments with every texture request.
  unsigned textureCounter_ {0};  //!< Makes texture names unique.
};

}  // namespace cras_rviz_vision_msgs
