// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief RViz display for vision_msgs/Detection2DArray messages.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cras_cpp_common/image_transport.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <image_transport/subscriber.hpp>
#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "cras_rviz_vision_msgs/detection_buffer.hpp"
#include "cras_rviz_vision_msgs/detection_store.hpp"
#include "cras_rviz_vision_msgs/image_cache.hpp"
#include "cras_rviz_vision_msgs/image_topic_property.hpp"
#include "cras_rviz_vision_msgs/keepout_zones.hpp"
#include "cras_rviz_vision_msgs/layout_planner.hpp"

namespace cras_rviz_vision_msgs {

class MouseWatcher;  //!< Watches the mouse in the view (defined in the source file).

/**
 * \brief Visualization of detections in the 2D image plane (vision_msgs/Detection2DArray).
 *
 * Every detection is shown at the position of its most probable class hypothesis. Because the hypotheses come with a
 * 3D pose estimate, the detections can be shown in the 3D scene and projected into the currently active view, which
 * works both with the TopDownOrtho camera (the typical use case) and with any other camera. The position of a
 * detection is marked by a small solid sphere, drawn the same way the PointStamped display draws a point.
 *
 * The detections are kept also after the message which brought them is gone - a detector typically publishes the same
 * objects repeatedly and publishes an empty array when it sees nothing, so neither showing only the contents of the
 * newest message nor hiding the detections on an empty message would be useful. A detection which is (almost) at the
 * same place, in the same part of the image and at the same time as an already remembered one replaces it, so that the
 * remembered detections stay sparse, and the oldest ones are dropped when the number of displayed markers exceeds
 * \c Max. markers.
 *
 * Only a limited number of detections (up to \c Max. images) can be shown with a preview of the source camera image;
 * which ones get a preview decides ScreenLayout, so that the previews never overlap each other and the most important
 * detections win. The previews have a fixed size in the view, so zooming in always makes room for more of them. The
 * preview of a detection stays shown as long as the detection is remembered, even when the camera image it was taken
 * from is long gone.
 *
 * Everything is drawn only when something changed (a detection arrived, the view moved, the user hovered a marker or
 * a property was set), so an idle view costs a few comparisons per frame.
 */
class Detection2DArrayDisplay
    : public rviz_common::RosTopicDisplay<vision_msgs::msg::Detection2DArray> {
  Q_OBJECT

public:
  //! How the colour of the detection marker is chosen.
  enum class ColorMode{
    PROBABILITY = 0,  //!< From the detection probability (green for high, red for low).
    CLASS_NAME = 1,  //!< Hashed from the class name (the same class always has the same colour).
    NONE = 2,  //!< Constant colour of all detections.
    ZONE = 3,  //!< From the membership in a keep-out zone (red inside one, green outside of all of them).
  };

  Detection2DArrayDisplay();
  ~Detection2DArrayDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  //! Number of visualized detections. Only meant for testing and debugging.
  size_t getVisualizationSize() const;
  //! Number of detections that are currently shown with an image preview. Only meant for testing and debugging.
  size_t getPreviewCount() const;
  //! Number of detections loaded from the persistent storage. Only meant for testing and debugging.
  size_t getHistoryCount() const;
  //! Serial number of the detection which is currently highlighted, 0 when none is. Only meant for testing.
  uint64_t getSelectedSerial() const;

protected:
  void processMessage(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) override;

  //! Callback for the incoming camera images.
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  /**
   * \brief Remembers the keep-out zones the detections are tested against.
   * \param[in] msg The received collection of the zone polygons.
   */
  void zonesCallback(const polygon_msgs::msg::Polygon2DCollection::ConstSharedPtr& msg);

  //! Callback for the incoming compressed camera images.
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);

  //! Callback for the point of the camera image the user clicked.
  void selectedPointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg);

  //! Callback for the header of the camera image the user clicked.
  void selectedHeaderCallback(const std_msgs::msg::Header::ConstSharedPtr& msg);

  /**
   * \brief Highlights the detection the user pointed at.
   *
   * The detection is kept highlighted (its marker enlarged and drawn above the other markers, its preview opaque, with
   * a frame of its own and the highest priority in the layout) for \c Highlight. \c s, and asking again prolongs it.
   * A serial number of a detection which is not displayed anymore simply stops being highlighted.
   */
  void select(uint64_t serial);

  /**
   * \brief Looks for the detection which was seen at the given place of a camera image and highlights it.
   * \param[in] selection Where and when the user pointed (see \ref DetectionBuffer::select).
   */
  void select(const Selection& selection);

  /**
   * \brief Takes hold of the camera image the user clicked in, for the preview of the detection it highlighted.
   *
   * The highlighted detection has the absolute priority in showing its image preview, and the picture it should show is
   * the image the click came from. That image may be older than what the image cache keeps (the visualization strip the
   * user clicks in only refreshes once in a while), and the layout which is going to show the preview runs up to a
   * second later, so the image is taken out of the cache the moment the click arrives and held until the highlight
   * ends.
   *
   * \param[in] serial Detection the click led to.
   * \param[in] stamp When the clicked image was taken.
   * \param[in] cameraFrame Frame of the camera the detection was seen in (empty when it is not known).
   */
  void pinClickedImage(uint64_t serial, const rclcpp::Time& stamp, const std::string& cameraFrame);

  /**
   * \brief Looks for the detection the user pointed at and highlights it, and finishes the expired highlights.
   *
   * Handles both the requests which arrived on the selection topic and the clicks in the view of the display, and
   * finishes the ones whose time is over. Called in every frame.
   */
  void updateSelection();

  /**
   * \brief The marker of the detection closest to the given place of the view, if there is one to point at.
   *
   * Uses the projections of the last layout, so it costs a few distance comparisons and can be called every frame.
   *
   * \param[in] x Horizontal place in the view [px].
   * \param[in] y Vertical place in the view [px].
   * \param[out] nearestGap How far the closest marker is from the place, whatever it is; tells a click which missed by
   *   a few pixels from a click in an empty part of the view.
   * \return Serial number of the detection, 0 when no marker is close enough to the place.
   */
  uint64_t nearestMarker(double x, double y, double* nearestGap = nullptr) const;

  //! Remembers the place (in the pixels of the view) the user pressed the mouse button at.
  void mousePressed(double x, double y);

  //! Forgets the detections which are highlighted (used when the drawn detections are destroyed, see reset()).
  void clearSelection();

  /**
   * \brief The user released the mouse button at the given place of the view [px].
   *
   * A press and a release at the same place is a click, while the mouse moved in between is the tool of the user doing
   * its work (rotating the view, moving an object) and must not highlight anything.
   */
  void mouseReleased(double x, double y);

  /**
   * \brief Highlights the detection drawn at the given place of the view, if there is one.
   * \param[in] x Horizontal position in the view [px].
   * \param[in] y Vertical position in the view [px].
   * \return Whether something was highlighted.
   */
  bool selectAt(double x, double y);

  /**
   * \brief The current time, in the time base of the detections.
   *
   * It is the time reported by the ROS clock, so that the age of the detections (and the time range of the loaded
   * history) is relative to the simulation time when the clock is driven by \c /clock. When the clock is not driven
   * yet, the wall clock is used instead and \c rosClock is set to false.
   *
   * \param[out] rosClock Can be null. Set to true if the returned time comes from the ROS clock.
   */
  int64_t nowNs(bool* rosClock = nullptr) const;

  /**
   * \brief Tries to transform the detections whose frame could not be transformed when they arrived.
   *
   * The detections which could not be remembered in the global frame are drawn (if at all) by the transform of their
   * own time, and that transform leaves the TF buffer some time after the detection arrived. This method keeps looking
   * for a chance to move them to the global frame, where they are safe.
   */
  void resolvePendingDetections();
  /**
   * \brief Tells which of the remembered detections lie in one of the keep-out zones known now.
   *
   * The membership is evaluated in the frame of the zones, which is the only place where the polygons and the
   * detections meet; a detection whose position in that frame is not known (yet) keeps the membership it arrived (or
   * was recorded) with. It is called before the age of the detections is looked at, because the zones decide which of
   * the two lifetimes a detection runs out of.
   *
   * Must be called with \c detectionsMutex_ locked.
   *
   * \return Whether the membership of any detection changed, so that the marks and their labels can be drawn again.
   */
  bool markZones();

  //! Returns the colour of a detection according to the selected colour mode.
  Ogre::ColourValue getColor(const StoredDetection& detection) const;

  //! \brief How many of something there are, written the way the status of the display says it, e.g. \c "2 labels".
  static std::string plural(size_t count, const std::string& word);

  //! \brief Writes a time in seconds the way the status of the display does it, e.g. \c "6.0" s.
  static std::string secondsText(double seconds);

  /**
   * \brief How long the display keeps the detections which stop being reported, in the words used in its status.
   * \return Text such as \c "10.0" s out of a keep-out zone and \c 30.0" s inside one"; the lifetime which is not set
   *   is left out.
   */
  std::string agingText() const;

  //! Returns the text describing the given detection (empty if labels are disabled).
  std::string getLabel(const StoredDetection& detection) const;

  //! The query describing what should be loaded from the persistent storage.
  StorageQuery historyQuery() const;

  /**
   * \brief Frames of the cameras whose images are currently in the cache (refreshed at the beginning of every layout).
   *
   * The detections of several cameras can come on one topic, and their images can come on one topic too; the frames are
   * the only thing to tell them apart.
   */
  const std::set<std::string>& cameraFrames();

  /**
   * \brief Frame of the camera images to look the preview of a detection up in.
   * \param[in] cameraFrame Frame the detection was reported in (the frame of the message it arrived in).
   * \return Frame to look in, or an empty string to take the image of any camera.
   */
  std::string wantedFrame(const std::string& cameraFrame);

  //! Sets the level and message of one aspect of the display status; does nothing when nothing changed.
  void report(
      const std::string& name, rviz_common::properties::StatusProperty::Level level,
      const std::string& message);

  //! A text label attached to a detection; the labels are reused so that they do not have to be re-created.
  struct Label{
    Ogre::SceneNode* node {nullptr};  //!< Node the text is attached to.
    std::unique_ptr<rviz_rendering::MovableText> text;  //!< The drawn text (the node only draws it).
    uint64_t serial {0};  //!< Detection the label belongs to, 0 for an unused label.
    std::string caption;  //!< Text which is drawn by the label right now.
    float height {0.};  //!< Height of the text in the world, as tall as the font is in the view [m].
  };

  //! The sphere marking one detection; it exists as long as the detection is displayed.
  struct Marker{
    Ogre::SceneNode* node {nullptr};  //!< Places the sphere in the scene.
    Ogre::Entity* entity {nullptr};  //!< The drawn sphere.
    std::string material;  //!< Name of the material the sphere is drawn with.
    std::string previewKey;  //!< Key of the image preview of the detection, to release it when it is gone.
    double size {0.};  //!< Size the sphere is scaled to.
    uint8_t queue {0};  //!< Rendering queue the sphere is drawn in (above the others when highlighted).
  };

  //! Everything needed to draw one detection.
  struct Item{
    const StoredDetection* detection {nullptr};  //!< The detection this item shows.
    uint64_t serial {0};  //!< Serial number of the detection.
    Ogre::Vector3 position {};  //!< Position of the detection in the frame of the view.
    Ogre::Vector2 screen {0, 0};  //!< Projection of the detection to the view.
    Ogre::Vector3 unitX {};  //!< World vector of one pixel in the horizontal direction.
    Ogre::Vector3 unitY {};  //!< World vector of one pixel in the vertical direction.
    double pixelsPerUnit {1.};  //!< How many pixels corresponds to one world unit.
    double radius {0.};  //!< Radius of the drawn sphere [px].
    Ogre::ColourValue colour {};  //!< Colour of the marker.
    std::string label;  //!< Text describing the detection.
    double aspectRatio {1.};  //!< Width / height of the image preview.
    std::string material;  //!< Material of the image preview, empty when it is not created yet.
    unsigned textureWidth {0};  //!< Size of the picture of the preview [px]; it decides the shape of the preview.
    unsigned textureHeight {0};
    CameraImage previewImage;  //!< Camera image the preview of this detection is taken from.
    ImageRegion previewRegion;  //!< Which part of that image to show.
    std::string textureKey;  //!< Key of the preview in the texture cache.
    bool hasPreview {false};  //!< Whether there is anything to show as the image preview.
    bool inView {false};  //!< Whether the detection is inside the view (so that it can show anything at all).
    bool selected {false};  //!< Whether the user asked to highlight this detection.
    double priority {0.};  //!< Ranking of the detection among its neighbours.
    LayoutItem layout;  //!< The annotations of the detection and their placement.
  };

  //! State of the (batched) loading of the detections from the persistent storage.
  struct HistoryLoad{
    std::vector<StoredRecord> records;  //!< The records read from the storage, not converted yet.
    size_t index {0};  //!< Index of the next record to add to the displayed detections.
    size_t skipped {0};  //!< Number of rows of the storage which could not be read.
    size_t loaded {0};  //!< Number of stored detections currently displayed.
    std::string error;  //!< What is wrong with the storage (empty if it is readable).
    double age {0.};  //!< Time since the loading was asked for [s].
    double pause {0.};  //!< Time until the next batch of detections is added [s].
    bool running {false};  //!< Whether the records above are being added to the display.
  };

  //! Prefix of the names of the rendering resources of this display instance.
  std::string resourcePrefix() const;
  /**
   * \brief Name of the (shared) material the detection markers are drawn with.
   * \param[in] colour Colour of the marker.
   * \param[in] highlighted Whether the marker is drawn brighter and without depth testing (the highlighted detection
   *   has to be seen even when it is behind another one).
   */
  const std::string& markerMaterial(const Ogre::ColourValue& colour, bool highlighted);

  /**
   * \brief Projects the remembered detections, places their annotations and draws everything.
   * \param[in] camera Camera of the currently active view.
   * \param[in] viewport Viewport the view is rendered to.
   */
  void layout(Ogre::Camera& camera, Ogre::Viewport& viewport);

  //! Destroys the spheres, previews and labels of the detections which are not displayed anymore.
  void clearDrawn();

  /**
   * \brief Destroys all the label texts.
   *
   * The text objects can only be destroyed while their scene nodes exist, which is not the case after the display was
   * disabled - they therefore have to be destroyed before the scene of the display is torn down, see onDisable().
   */
  void clearLabels();

  //! Adds the detections which were loaded from the persistent storage to the displayed detections.
  void takeHistoryBatch();

  //! Drops the detections which were loaded from the persistent storage.
  void clearHistory();

  //! Drives the delayed and batched loading of the detections from the persistent storage.
  void updateHistory(float wall_dt);

  //! The image of a detection loaded from the persistent storage (the images are cached, see ImageCache).
  CameraImage historyImage(const std::string& path);

  //! Which detection the mouse points at; keeps it until the user moves the mouse away from it.
  void updateHover();

  //! Key of the image preview of the given detection in the texture cache.
  std::string previewKey(const StoredDetection& detection) const;

  //! Turns off the depth checking of the text, so that the labels are not hidden by the annotations at the same depth.
  static void ignoreDepth(rviz_rendering::MovableText& text);

  /**
   * \brief Creates the text object of a label.
   * \return False if the text cannot be drawn (then no other label is attempted).
   */
  bool createLabelText(Label& label);

  /**
   * \brief Material of the frames around the previews and of the lines connecting them to their detections.
   *
   * There is only one, and its colour comes from the vertex colours, so that the hovered preview can be highlighted
   * without another material.
   */
  const std::string& annotationMaterial();

  /**
   * \brief Point of the plane of the screen of the detection which is at the given place of the view [px].
   */
  Ogre::Vector3 screenPoint(const Item& item, double x, double y);

  //! Adds a four-corner shape (a frame or a line) to the annotations which are currently being collected.
  void addAnnotationQuad(
      const Ogre::Vector3& a, const Ogre::Vector3& b, const Ogre::Vector3& c,
      const Ogre::Vector3& d, const Ogre::ColourValue& colour, unsigned& vertex);

  /**
   * \brief Draws the frame of a preview and the line which leads from it to its marker.
   *
   * Both have the colour of the detection, so that the previews can be told apart, and both are brighter and thicker
   * for the preview the user points at.
   *
   * \param[in] item The detection whose preview is drawn.
   * \param[in] hovered Whether the user points at it.
   * \param[in] selected Whether the user asked to highlight it (its frame then has a colour of its own).
   * \param[in] alpha Opacity of the previews which are neither hovered nor highlighted.
   * \param[in] thickness Width of the frame and of the line [px].
   * \param[in,out] vertex Number of the vertices already handed to the annotation object.
   */
  void annotatePreview(
      Item& item, bool hovered, bool selected, double alpha, double thickness,
      unsigned& vertex);

  //! Which of the image messages the user can click in to highlight a detection (see the Selection properties).
  enum class SelectionMode : int {NONE, HEADER, POINT, };

  friend class MouseWatcher;  //!< Needs to report the mouse position in the view.

  DetectionBuffer detections_ {DetectionBuffer::DEFAULT_LIMIT};  //!< The detections to display.
  mutable std::mutex detectionsMutex_;  //!< Guards detections_ and the members below it.
  std::string frameId;  //!< Frame of the last message.
  std::string detectionsTopicName_;  //!< The topic the accumulated detections come from.
  HistoryLoad history_;  //!< The detections being loaded from the persistent storage.
  bool historyDirty_ {true};  //!< Whether the storage should be read again.
  bool historyWaitingForClock_ {false};  //!< Whether the history waits for the clock of ROS to know what to load.

  std::unique_ptr<ImageCache> images_;  //!< Cache of the camera images.
  std::unique_ptr<TextureCache> textures_;  //!< Cache of the image previews.
  KeepoutZones zones_;  //!< Keep-out zones the detections are tested against (guarded by detectionsMutex_).
  rclcpp::Subscription<polygon_msgs::msg::Polygon2DCollection>::SharedPtr zonesSub_;  //!< Zones subscriber.
  std::unique_ptr<cras::ImageTransport> imageTransport_;  //!< Camera image transport.
  image_transport::Subscriber imageSubscriber_;  //!< Camera image subscriber.
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressedSubscriber_;  //!< Compressed images.
  std::string zonesTopicName_;  //!< The topic of the keep-out zones the display is subscribed to.
  std::string imageTopicName_;  //!< The image topic the display is currently subscribed to.
  bool imageTopicCompressed_ {false};  //!< Whether that topic carries compressed images.
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pointSubscriber_;  //!< Clicks in a camera image.
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr headerSubscriber_;  //!< Camera images the user clicked in.
  std::string selectionTopicName_;  //!< The topic the clicks arrive on, and the type it currently uses.
  std::map<std::string, CameraImage> historyImages_;  //!< Cache of the images loaded for the history detections.
  std::vector<std::string> historyImageOrder_;  //!< Order in which the cached history images were used.

  //! Camera image topic, offering both the raw and the compressed image topics.
  ImageTopicProperty* imageTopicProperty_ {nullptr};
  rviz_common::properties::FloatProperty* imageToleranceProperty_ {nullptr};
  rviz_common::properties::BoolProperty* statusDebugProperty_ {nullptr};
  rviz_common::properties::IntProperty* maxMarkersProperty_ {nullptr};
  rviz_common::properties::BoolProperty* accumulateProperty_ {nullptr};
  rviz_common::properties::FloatProperty* lifetimeProperty_ {nullptr};
  rviz_common::properties::FloatProperty* markerSizeProperty_ {nullptr};
  rviz_common::properties::FloatProperty* minProbabilityProperty_ {nullptr};
  rviz_common::properties::EnumProperty* colorModeProperty_ {nullptr};
  rviz_common::properties::FloatProperty* merge3dProperty_ {nullptr};
  rviz_common::properties::FloatProperty* merge2dProperty_ {nullptr};
  rviz_common::properties::FloatProperty* mergeTimeProperty_ {nullptr};
  rviz_common::properties::BoolProperty* showLabelsProperty_ {nullptr};
  rviz_common::properties::BoolProperty* showClassProperty_ {nullptr};
  rviz_common::properties::BoolProperty* showProbabilityProperty_ {nullptr};
  rviz_common::properties::BoolProperty* showIdProperty_ {nullptr};
  rviz_common::properties::BoolProperty* showTimeProperty_ {nullptr};
  rviz_common::properties::BoolProperty* showZoneProperty_ {nullptr};
  rviz_common::properties::IntProperty* maxLabelsProperty_ {nullptr};
  rviz_common::properties::FloatProperty* fontSizeProperty_ {nullptr};
  rviz_common::properties::StringProperty* fontProperty_ {nullptr};
  rviz_common::properties::BoolProperty* showImagesProperty_ {nullptr};
  rviz_common::properties::IntProperty* maxPreviewsProperty_ {nullptr};
  rviz_common::properties::IntProperty* previewWidthProperty_ {nullptr};
  rviz_common::properties::FloatProperty* previewPaddingProperty_ {nullptr};
  rviz_common::properties::FloatProperty* spacingProperty_ {nullptr};
  rviz_common::properties::StringProperty* cameraFrameProperty_ {nullptr};
  rviz_common::properties::FloatProperty* previewAlphaProperty_ {nullptr};
  rviz_common::properties::BoolProperty* previewFrameProperty_ {nullptr};
  rviz_common::properties::FloatProperty* stemProperty_ {nullptr};
  rviz_common::properties::FloatProperty* hoverDelayProperty_ {nullptr};
  rviz_common::properties::FloatProperty* updatePeriodProperty_ {nullptr};
  rviz_common::properties::EnumProperty* selectionTopicProperty_ {nullptr};
  rviz_common::properties::StringProperty* selectionTopicNameProperty_ {nullptr};
  rviz_common::properties::FloatProperty* selectionToleranceProperty_ {nullptr};
  rviz_common::properties::FloatProperty* highlightProperty_ {nullptr};
  rviz_common::properties::FloatProperty* highlightSizeProperty_ {nullptr};
  rviz_common::properties::ColorProperty* highlightColorProperty_ {nullptr};
  rviz_common::properties::RosTopicProperty* zonesTopicProperty_ {nullptr};
  rviz_common::properties::FloatProperty* inZoneLifetimeProperty_ {nullptr};
  rviz_common::properties::BoolProperty* historyEnabledProperty_ {nullptr};
  rviz_common::properties::StringProperty* storageDirProperty_ {nullptr};
  rviz_common::properties::StringProperty* globalFrameProperty_ {nullptr};
  rviz_common::properties::IntProperty* historyFromProperty_ {nullptr};
  rviz_common::properties::IntProperty* historyToProperty_ {nullptr};
  rviz_common::properties::StringProperty* historyClassesProperty_ {nullptr};
  rviz_common::properties::StringProperty* historyCamerasProperty_ {nullptr};
  rviz_common::properties::EnumProperty* historyZoneProperty_ {nullptr};
  rviz_common::properties::BoolProperty* historyImagesProperty_ {nullptr};
  rviz_common::properties::FloatProperty* historyReloadProperty_ {nullptr};
  rviz_common::properties::FloatProperty* historyDelayProperty_ {nullptr};
  rviz_common::properties::IntProperty* historyBatchProperty_ {nullptr};
  rviz_common::properties::FloatProperty* historyPauseProperty_ {nullptr};

  Ogre::SceneNode* overlayNode_ {nullptr};  //!< Node with all the drawn annotations.
  Ogre::ManualObject* previewObject_ {nullptr};  //!< The drawn image previews.
  std::unique_ptr<MouseWatcher> mouseWatcher_;  //!< Watches the mouse in the view of the display.
  std::map<uint64_t, std::unique_ptr<Marker>> markers_;  //!< The drawn detection spheres, by detection serial.
  std::vector<std::unique_ptr<Label>> labels_;  //!< The drawn text labels, reused between the detections.
  std::map<uint64_t, size_t> labelOf_;  //!< Index of the label of each detection in labels_.
  Ogre::ManualObject* annotationObject_ {nullptr};  //!< Frames of the previews and the lines to their detections.
  std::string annotationMaterial_;  //!< Name of the material of the preview frames and their lines.
  std::map<int, std::string> markerMaterials_;  //!< Quantised colour to the material the markers are drawn with.
  std::string prefix_;  //!< Prefix of the names of the rendering resources of this display instance.
  bool textFailed_ {false};  //!< Whether creating the text labels failed.

  std::vector<StoredDetection> snapshot_;  //!< Copy of the displayed detections, used by layout().
  std::vector<Item> items_;  //!< The detections projected to the view, built by layout().
  std::vector<uint64_t> shownPreviews_;  //!< Serials of the detections whose preview was shown by the last layout.
  std::unordered_set<uint64_t> knownSerials_;  //!< Serials of the detections which were displayed before.
  std::set<std::string> cameraFrames_;  //!< Frames of the cameras whose images are in the cache.
  uint64_t layoutRevision_ {0};  //!< Buffer revision the last layout was made for.
  std::atomic<bool> layoutDirty_ {true};  //!< Whether the layout has to be recomputed.
  std::string lastViewKey_;  //!< Identification of the camera and view size the last layout was made for.
  double refreshAge_ {0.};  //!< Time since the layout was last recomputed [s].
  // Starts big so that the first layout is drawn at once and not only after the update period passes.
  double layoutAge_ {1e6};  //!< Time since the layout was last recomputed [s], for the update period.
  bool layoutUrgent_ {false};  //!< Whether the user did something the layout has to react to immediately.
  double transformRefresh_ {1.};  //!< How often to draw the detections again because of the transforms [s].
  double mouseScaleX_ {1.};  //!< Converts the mouse position from the window to the view pixels.
  double mouseScaleY_ {1.};
  double mouseX_ {0.};  //!< Position of the mouse in the view [px].
  double mouseY_ {0.};
  bool mouseInside_ {false};  //!< Whether the mouse is in the view at all.
  uint64_t hoverCandidate_ {0};  //!< Detection the mouse points at since the last check.
  int64_t hoverSinceNs_ {0};  //!< When the mouse started pointing at that detection (wall clock) [ns].
  uint64_t hoveredSerial_ {0};  //!< Detection whose preview should be shown with the highest priority.
  std::vector<uint64_t> requested_;  //!< Detections the user clicked on, the oldest request first.
  std::unordered_map<uint64_t, int64_t> highlightedUntilNs_;  //!< Until when each detection stays highlighted.
  uint64_t selectedSerial_ {0};  //!< Detection which is highlighted now (drawn, and first in the layout).
  /**
   * \brief The camera image held for the preview of the detection the user clicked in.
   */
  struct PinnedImage {
    uint64_t serial {0};  //!< The detection the image was clicked in, 0 when there is none.
    CameraImage image;  //!< The image itself, empty when the click did not come from one.
  };
  PinnedImage pinned_;  //!< Guarded by \c detectionsMutex_, because the click arrives in another thread.
  int64_t nextSelectionCheckNs_ {0};  //!< When to render again to notice that a highlight ends [ns].
  bool selectionMissed_ {false};  //!< Whether the last click did not match any displayed detection.
  bool selectedPreviewShown_ {false};  //!< Whether the highlighted detection is showing its picture (see the layout).
  std::string selectedPreviewNote_;  //!< Why the highlighted detection has no picture to show (empty if it has one).
  double pressedX_ {0.};  //!< Place in the view where the user pressed the mouse button [px].
  double pressedY_ {0.};
  bool mousePressed_ {false};  //!< Whether the mouse button is held down.
  double selectionGap_ {1e9};  //!< How far the closest detection was from a click which led to nothing [px].
  size_t previewCount_ {0};  //!< Number of drawn image previews.
  size_t visualizedCount_ {0};  //!< Number of drawn detections.
  size_t labelCount_ {0};  //!< Number of drawn labels.
  size_t historyCount_ {0};  //!< Number of drawn detections from the persistent storage.
  bool statusDebug_ {false};  //!< Reports every change of the status to the log (only for diagnosing a running system).
  std::map<std::string, int64_t> lastStatusLogNs_;  //!< When each informational status was last repeated in the log.
  uint64_t messagesReceived_ {0};  //!< Number of the received detection messages.
  int64_t lastMessageNs_ {0};  //!< When the last message arrived (monotonic time).
  double messageRate_ {0.};  //!< Smoothed rate of the incoming messages [Hz].
  size_t poselessCount_ {0};  //!< Number of detections that have no 3D pose.
  size_t filteredCount_ {0};  //!< Number of detections hidden by the minimal probability.
  size_t untransformableCount_ {0};  //!< Number of detections that cannot be transformed to the view.
  std::string untransformableFrame_;  //!< One of the frames which could not be transformed (for the status).
  int64_t lastResolveNs_ {0};  //!< When the missing transforms were last looked for again.
  int64_t closestImageGapNs_ {0};  //!< Smallest distance of a detection to any cached camera image [ns].
  size_t imageCount_ {0};  //!< Number of camera images available for the detections.
  size_t zoneCount_ {0};  //!< Number of the known keep-out zones.
  size_t inZoneCount_ {0};  //!< Number of the remembered detections which lie in one of them.
  size_t unknownZoneCount_ {0};  //!< Number of detections whose position in the frame of the zones is not known.
  std::map<std::string, std::pair<int, std::string>> lastStatus_;  //!< The last status reported for each aspect.

protected Q_SLOTS:
  //! (Re)subscribes to the image topic.
  void updateImageTopic();
  //! (Re)subscribes to the topic of the places the user clicks in a camera image.
  void updateSelectionTopic();
  //! (Re)subscribes to the topic the keep-out zones are published on.
  void updateZonesTopic();
  //! Draws the detections again (called when any of the visualization properties changes).
  void redraw();
  //! Asks for the historical detections to be loaded again (called when the storage settings change).
  void reloadHistory();
  //! Re-evaluates the whole status from the current state of the display.
  void updateStatus();

  //! \brief Turns the logging of the status messages on and off (see the \c Log status property).
  void updateStatusDebug();
};

}  // namespace cras_rviz_vision_msgs
