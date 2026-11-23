// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Cache of camera images used as previews of the visualized detections.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <algorithm>
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
  std::string frameId;  //!< Frame of the camera the image comes from.

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
 * Several cameras can share one topic, and images of two cameras with the same stamp would otherwise be taken for each
 * other. The images therefore keep the frame they arrived in and can be looked up in it.
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
   * \param[in] frameId Frame of the camera to look in. Empty to look in the images of any camera.
   * \return The image, or an empty CameraImage if no matching image is in the cache.
   */
  CameraImage get(const rclcpp::Time& stamp, double tolerance, const std::string& frameId = {}) const;

  /**
   * \brief Returns the image closest to the given stamp, however old it is.
   *
   * The previews of the detections the user highlighted are shown whatever it costs them (see \ref get), so they take
   * the closest image the camera has offered instead of none at all.
   *
   * \param[in] stamp Stamp of the image.
   * \param[in] frameId Frame of the camera to look in. Empty to look in the images of any camera.
   * \return The image, or an empty CameraImage if the cache holds no image of that camera.
   */
  CameraImage nearest(const rclcpp::Time& stamp, const std::string& frameId = {}) const;

  /**
   * \brief Checks whether an image with the given stamp is in the cache.
   */
  bool has(const rclcpp::Time& stamp, double tolerance, const std::string& frameId = {}) const;

  /**
   * \brief Distance of the given stamp to the closest image in the cache, whatever it is.
   *
   * Lets the display tell the user how far the camera images are from the detections when they do not match.
   *
   * \param[in] stamp Stamp of the image.
   * \param[in] frameId Frame of the camera to look in. Empty to look in the images of any camera.
   * \return The distance in nanoseconds, or -1 when the cache is empty.
   */
  int64_t nearestGapNs(const rclcpp::Time& stamp, const std::string& frameId = {}) const;

  /**
   * \brief What the cache knows about the images of one camera.
   *
   * The cameras are told apart by the frame of their images, which is the only thing a topic carrying several cameras
   * offers.
   */
  struct Source {
    std::string frameId;  //!< Frame of the images of this camera.
    size_t images {0};  //!< Number of the images of this camera which are currently kept.
    size_t received {0};  //!< Number of the images of this camera which arrived since the cache was created.
    std::string encoding;  //!< Encoding of the last image of this camera.
    std::string error;  //!< Why the last image of this camera could not be used (empty if it could).
  };

  /**
   * \brief The cameras whose images arrived (or at least tried to) since the cache was created.
   *
   * Lets the display say which cameras it sees when the detections do not match any image, instead of blaming the
   * time tolerance for a camera which is not there at all.
   */
  std::vector<Source> sources() const;

  /**
   * \brief Number of kept images of the given camera. Empty frameId gives the number of all of them.
   */
  size_t count(const std::string& frameId) const;

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
   * \brief Forgets the images which are older than the newest one by more than the given time.
   *
   * The camera images are usually some time behind the detections which were computed from them, so the cache has to
   * bridge that delay - a cache sized only by the number of images can easily be too short for it.
   *
   * \param[in] seconds How much older an image may be to the newest one to be kept. 0 turns this off.
   */
  void setKeepDuration(double seconds);

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
  void failed(const std::string& frameId, const std::string& encoding, const std::string& what);

  /**
   * \brief Index of the image closest to the given stamp (within the given tolerance), or -1 if there is no such image.
   * \param[in] frameId Frame of the camera to look in. Empty to look in the images of any camera.
   */
  int findIndex(const rclcpp::Time& stamp, int64_t toleranceNs, const std::string& frameId) const;

  mutable std::mutex mutex_;  //!< Guards everything below this line.
  size_t maxImages_;  //!< Maximal number of kept images.
  int64_t keepDurationNs_ {0};  //!< How much older an image may be to the newest one to be kept.
  std::map<std::string, int64_t> sourceWidths_;  //!< Width of the last image of each camera, before any reduction.
  std::map<std::string, Source> sources_;  //!< What arrived from each camera, by its frame.
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

  /**
   * \brief Region whose size is the given factor of the original one, with the same part added all around.
   *
   * A factor of 1 returns the region itself, a factor of 1.2 shows the object with a tenth of its size of the
   * surroundings on every side. That is the meaning of the \c Preview padding property of the detection display and of
   * the \c bbox_scale parameter of the visualization node of the detections - the two have to agree, because the
   * picture of an object in the visualization strip is the cutout the user clicks on, and the preview of the clicked
   * detection is supposed to show the very same cutout.
   */
  ImageRegion padded(double factor) const{
    const auto addX = width * (factor - 1.) * .5;
    const auto addY = height * (factor - 1.) * .5;
    return {x - addX, y - addY, width + 2. * addX, height + 2. * addY};
  }

  /**
   * \brief Region limited to an image of the given size.
   *
   * Only the part of the region which is really inside the image can be shown, so this is also the region whose aspect
   * ratio the preview has to use - otherwise the image is stretched.
   *
   * \param[in] imageWidth Width of the image the region is taken from.
   * \param[in] imageHeight Height of the image the region is taken from.
   */
  ImageRegion clamped(double imageWidth, double imageHeight) const {
    if (width <= 0. || height <= 0.) {
      return *this;  // an empty region means the whole image, and that one is never outside of itself
    }
    const auto left = std::max(0., x);
    const auto top = std::max(0., y);
    const auto right = std::min(imageWidth, x + width);
    const auto bottom = std::min(imageHeight, y + height);
    return {left, top, std::max(0., right - left), std::max(0., bottom - top)};
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
   * \param[in] namePrefix Prefix of the names of the created textures and materials. It has to identify the RViz
   *   instance the cache belongs to, because several RViz instances (e.g. the one of \c rqt_rviz) can share one
   *   process, and the names of the rendering resources are shared between them.
   */
  explicit TextureCache(size_t maxTextures = 32, std::string namePrefix = "cras_rviz_vision_msgs");
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

  /**
   * \brief Name of the material the given key already shows, without looking at any image.
   *
   * The image a preview was made from can be long gone from the image cache while the detection is still displayed.
   * The preview of such a detection stays what it was, so that it does not disappear only because the camera image
   * arrived earlier than RViz.
   *
   * The size of the texture is returned with it because that is the shape of the picture which is going to be drawn -
   * the place kept for a preview has to have it too, otherwise the picture looks squeezed.
   *
   * \param[in] key Identifies the material, as in \ref getMaterial.
   * \param[out] width Width of the shown texture; only written when there is one and the pointer is not null.
   * \param[out] height Height of the shown texture; only written when there is one and the pointer is not null.
   * \return The name of the material, or an empty string if no material for that key exists.
   */
  std::string peekMaterial(const std::string& key, unsigned* width = nullptr, unsigned* height = nullptr);

  /**
   * \brief The part of a stored image that \ref getMaterial really shows for the given region.
   *
   * The region of a detection can reach outside of the image (and the image is kept in a reduced resolution), so the
   * picture which ends up in a preview is this rectangle, not the region itself. The shape of a preview has to follow
   * it, otherwise the picture is stretched.
   *
   * \param[in] image The stored image the preview is made of.
   * \param[in] region The requested part of the image, in the coordinates of the original image.
   * \return The part of the stored image to show; empty when there is nothing to show.
   */
  static cv::Rect sourceRect(const CameraImage& image, const ImageRegion& region);

  //! Sets the maximal number of textures to keep and drops the least recently used ones above it.
  void setLimit(size_t maxTextures);

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
  std::string createMaterial(const std::string& textureName, unsigned id) const;

  //! Destroys the given texture and material right now. Must hold mutex_ and must be called in the rendering thread.
  static void destroyNow(const Entry& entry);

  mutable std::mutex mutex_;  //!< Guards everything below this line.
  std::map<std::string, Entry> entries_;  //!< Key to Entry.
  std::vector<Entry> retiredEntries_;  //!< Entries to destroy in the rendering thread.
  std::string namePrefix_;  //!< Makes the names of the resources unique between the RViz instances in one process.
  size_t maxTextures_;  //!< Maximal number of kept textures.
  uint64_t usageCounter_ {0};  //!< Increments with every texture request.
  unsigned textureCounter_ {0};  //!< Makes texture names unique.
};

}  // namespace cras_rviz_vision_msgs
