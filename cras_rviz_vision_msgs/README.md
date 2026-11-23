<!--
SPDX-License-Identifier: Apache-2.0
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# RViz visualisation of vision_msgs detections

`cras_rviz_vision_msgs/Detection2DArray` displays `vision_msgs/msg/Detection2DArray` messages in RViz.

Each detection is shown at the position of the pose belonging to its most probable class hypothesis (the hypothesis
with the highest `score`). The position is marked by a small solid sphere, the same way the `PointStamped` display
shows a point, and a text label with the class name and its probability, and optionally a preview of the source camera
image are attached to it. Everything is projected into the currently active view, which makes the visualisation work
with any camera – the view `TopDownOrtho` being the typical use case.

## More detections than a message contains

A detector typically publishes the objects it sees repeatedly and publishes an empty array when it sees nothing, so the
display does not show only the contents of the newest message:

- The detections of the older messages are kept, which is also why an empty message does not hide anything. How many of
  them are kept is limited by *Markers/Max. markers* (default 2000) and optionally by their age (see
  [Keep-out zones](#keep-out-zones) for the two lifetimes); the oldest detections are dropped first and the number of
  the dropped ones is reported in the status of the display.
- The picture follows the detections with a delay of at most *Layout update period* (1 s by default). A dense detector
  publishes many times per second, and rearranging the labels and previews with every message would only make the view
  flicker. What the display says about the detections in its status is up to date immediately.
- A detection which is published again with the same ID (in the same frame) is updated in place, so a tracked object
  does not pile up.
- Detections *without* an ID are merged too: a new detection replaces an existing one when it is close to it in all the
  three worlds at once - within *Merge similar detections/Distance* in 3D (default 1 m), within *Image distance* in the
  image (default 50 px) and within *Time distance* in time (default 5 s). A person standing in front of the camera is
  then one marker rather than hundreds of them, which is what makes a long session with a busy scene usable. Set any of
  the three limits to 0 to stop merging (or to ignore that world).
- With hundreds of detections, only the most important ones (the most probable ones, and those in the centre of the
  view) get a text label and an image preview; the rest is drawn as a plain sphere, so the scene stays readable and
  fast to render.

![Preview](doc/preview.png)

## Image previews

The images the detections come from are subscribed on the topic set in the *Image topic* property, which offers both
`sensor_msgs/msg/Image` topics (subscribed with `cras::ImageTransport`, so a transport plugin such as `theora` can be
used if it is available) and `sensor_msgs/msg/CompressedImage` topics (decoded by OpenCV). The image whose timestamp is
the closest to the timestamp of the detection message (within *Image time tolerance*) is used as the source of the
previews of that message, and the part of it around the detection bounding box is shown. The preview is drawn in the
plane of the screen, so it always faces the camera.

The previews all have the same size in the view (*Image previews/Preview width*), so the space they take does not
depend on how far the detections are, and showing a preview of every detection would quickly cover the whole view. The
display therefore adapts to the space available in the view:

- The previews and the labels are placed by a layout which looks for the closest free place around each detection and
  never lets two of them overlap. The marker of a detection is drawn in any case and never blocks a label, so a dense
  crowd still gets its labels.
- The order in which the detections get a place is: the detection the user clicked on, then the detection the user
  points at, then the detections which just arrived, then the detections whose preview was shown before, and only then
  the importance of the detection (its probability and its distance from the centre of the view). Zooming or a new
  detection therefore does not make the already visible previews jump around.
- When there is not enough room around a detection, its preview is moved to the closest free place near it, and if there
  is none even there, the detection is only plotted as a plain sphere with its label - except for the detection the user
  clicked on, whose picture is shown over the previews in its way rather than not shown at all (see
  [Selecting a detection](#selecting-a-detection)).
- When the user zooms in, the detections spread out in the view and space for further previews appears.
- Pointing at a detection with the mouse shows its preview with the highest priority (after *Image previews/Hover
  delay*, so that moving the mouse across the view does not make the previews flicker).

The previews are semi-transparent (*Image previews/Preview alpha*) so that the scene behind them stays visible; the
preview the user points at, and the one of the selected detection, are drawn fully opaque.

## Selecting a detection

Clicking a detection - either a marker or an image preview in the view of the display, or a place in a camera image
shown by another part of the interface - highlights it:

- its sphere is drawn bigger (*Selection/Marker scale*), in *Selection/Color*, and over all the other markers,
- its image preview has the absolute priority in showing a picture for *Selection/Highlight* seconds (30 by default):
  it is the first detection to be offered a place in the layout, it is given one even when the view has no room free
  for it (its picture is then drawn over the previews which are in its way), it is drawn last so that nothing can cover
  it, and it is opaque and framed in *Selection/Color*,
- its picture never waits for the next layout period: the click makes the layout run again in the current frame,
- the status of the display says what the click led to, including the case that nothing matched the clicked place, and
  the reason why the picture of the clicked detection cannot be shown when it cannot be.

Clicking in the view is a press and a release of the left mouse button at the same place, so dragging to rotate or move
the view never selects anything. The other way round, the display listens for the places the user clicks in a camera
image: *rqt_image_view* publishes the clicked place (rescaled to the native resolution of the image it shows) as a
`geometry_msgs/msg/Point`, and the `detection_visualizer` node of the `g4s_mission` package publishes the place of the
click in a displayed crop, translated back to the pixels of the camera image it was cropped from, as a
`geometry_msgs/msg/PointStamped` on `clicked_image` and as a `std_msgs/msg/Header` (the camera image the click belongs
to) on `selected_image`. The display subscribes to one of them, whichever *Selection/Clicked image type* says (the two
message types cannot be told apart on a topic without subscribing to it), and matches the place against the bounding
boxes, the camera frames and the timestamps of the detections it shows:

- A detection is a candidate when its timestamp is within *Selection/Image time tolerance* from the clicked image (the
  detections are usually a little older than the image they are shown in) and when it was reported by the camera whose
  image was clicked. A detection which does not know its camera still is a candidate, only ranked lower - the
  detections are often reported in a frame other than the camera's.
- With the clicked place known, the detection has to have it inside its bounding box, or within 40 px of it (a person is
  rarely exactly inside its box). When several detections overlap under the click, the smallest one wins - the user can
  only have clicked on what they could see.
- The preview of the selected detection is taken from the clicked image, so the operator sees exactly the moment they
  clicked on, even when the detection has been reported from a much older image since.
- The picture of a detection is its bounding box made *Image previews/Preview padding* times bigger, which is the same
  rule (and the same default) as the `bbox_scale` parameter of the `detection_visualizer` node. The cutout the operator
  clicks in the visualization strip is therefore the cutout the preview shows, and a detection which has been reported
  from a newer image since is still shown with its bounding box in the middle of that cutout. Only a click beside the
  box moves the shown part of the image, because the clicked place is then at another object of the picture.
- A highlighted detection is never left without a picture only because of the time it would take to match one: when its
  own image is gone from the cache (the visualization strip the operator clicks in only refreshes once in a while, and
  the click may be about a second old when the preview is made), it shows the image the click came from, held for the
  whole time the detection is highlighted, or the closest image of its camera. Only a camera which never delivered an
  image leaves the detection without a picture, and the status says so.

## Keep-out zones

The display can listen to the keep-out zones drawn by the *Zone of interest* tool of this package (a
`polygon_msgs/msg/Polygon2DCollection` on the topic set by *Markers/Keep-out zones topic*, `polygons` by default, latched
so that a display which starts later gets them too). Knowing which detections are inside a zone and which are out of all
of them is what makes two things work:

- *Markers/Color by: Keep-out zone* colours a detection inside a zone red and one outside of every zone green, and
  *Labels/Show zone* adds `[zone 2]` or `[out]` to its label,
- a detection which stops being reported is forgotten after *Markers/Max. age in a zone* (default 0) if it is inside a
  zone, and after *Markers/Max. age out of a zone* (default 0) if it is not - an object which got into a zone is
  usually the one the operator wants to keep an eye on, so it can be allowed to stay longer than the ones walking
  around outside. A lifetime of 0 means the detections of that kind are kept until *Max. markers* makes room for a
  newer one. The detections read from the persistent history are a record of the past and are not aged out by this.

Forgetting a detection means it really disappears: its sphere, its label and its image preview are all dropped, and the
space the preview took becomes available for another detection. What the display knows about the zones - how many of
them it has, in which frame, and how many of the remembered detections lie in one of them - is in its status, so a
missing or unreachable zone topic is visible rather than silently making every detection count as being out of them.

The membership is evaluated in the frame of the zones, so the pose of a detection is transformed there (once per frame
and camera, not once per detection, and only for the detections whose zones are not known yet). A historical detection
is judged by where it stands in the zones *now*, which is what the operator looks at; the zone the recorder noted when
the detection arrived stays in its label and in the history query. A detection whose position in the frame of the zones
cannot be found at its own time uses the newest available transform instead - the zones do not move, so it still tells
where the place of the detection is in them. Only a detection for which no transform to the frame of the zones exists at
all is treated as being out of them, and the status of the display says how many of them that is.

## Trying it out

```
$(ros2 pkg prefix cras_rviz_vision_msgs)/share/cras_rviz_vision_msgs/demo/fake_detector.py 12 cluster &
rviz2 -d $(ros2 pkg prefix cras_rviz_vision_msgs)/share/cras_rviz_vision_msgs/rviz/detections.rviz
```

The demo publisher publishes a grid of detections (in one dense circle with `cluster`) together with synthetic camera
images on `/image`, so zooming in and out shows how the number of visible previews changes. With the `sparse` option,
every message contains other detections with new IDs (so the display has to keep them all), `wide` spreads the detections
tens of metres apart, `extframe` publishes the detections in a sensor frame instead of `map` (a static transform from
`map` to `camera` has to be published for them to show up), `noimage` publishes no images at all, `empty` makes the
detector regularly see nothing, `compressed` sends the images on `/image/compressed`, `zones` publishes a keep-out zone
over a corner of the scene on `polygons` (set *Markers/Max. age out of a zone* to a few seconds and *Markers/Max. age in
a zone* to a longer one to see the zone decide which detections stay), and `sim` publishes the `/clock` topic::

    # a lot of detections arriving in separate messages
    $(ros2 pkg prefix cras_rviz_vision_msgs)/share/cras_rviz_vision_msgs/demo/fake_detector.py 300 sparse empty &
    # or a small dense crowd, to show how the previews make room for each other
    $(ros2 pkg prefix cras_rviz_vision_msgs)/share/cras_rviz_vision_msgs/demo/fake_detector.py 12 cluster &

## Time and coordinate frames

The age of the detections (*Markers/Max. age out of a zone*, *Markers/Max. age in a zone*) and the time period of the
loaded history are resolved by the ROS clock of the RViz node, so both follow the simulation time when RViz runs with
`use_sim_time` and the `/clock` topic is published (e.g. when playing a bag). Until the clock is driven, the wall clock
is used.

The detections are remembered in the *Persistent history/Global frame* (default `FP_ECEF`) whenever the transform of
their own frame to it is available for their timestamp. That is the frame in which a place on the Earth means the same
place forever, so the transform from it to the frame of the view is static and stays valid however old the detections
are - the markers of a busy scene therefore stay exactly where they were seen, and their previews stay available long
after the camera image they were taken from is gone.

A detection whose transform is not available yet (the system has just started, the TFs come later) stays in the frame it
arrived in, and the display keeps looking for the transform until it appears. Until then, it is drawn by the transform
of its own timestamp - never by the newest one, because that would move the detection around as the sensor moves. The
number of such detections and the frame they are stuck in is reported in the status of the display.

## Properties

| Property | Meaning |
| --- | --- |
| *Topic* | The `vision_msgs/msg/Detection2DArray` topic to visualise. |
| *Image topic* | Camera images the detections come from; offers both `Image` and `CompressedImage` topics. Empty to only draw the markers. |
| *Image time tolerance* | Maximal time difference between a detection message and an image to use it [s]. |
| *Layout update period* | How often the markers, labels and image previews can be rearranged [s]. The detections arrive much faster, and following every message would make the view flicker. |
| *Markers/Size* | Diameter of the drawn sphere of a detection [m]. |
| *Markers/Keep older detections* | Whether the detections from the older messages stay visible. |
| *Markers/Max. markers* | Maximal number of detections kept and drawn at once; the oldest ones are dropped when it is exceeded. |
| *Markers/Max. age out of a zone* | How long a detection which is not reported again stays shown when it is out of every keep-out zone [s]. 0 keeps it until the limit above is exceeded. |
| *Markers/Max. age in a zone* | The same for a detection which is inside a keep-out zone [s]. 0 keeps it until the limit above is exceeded. |
| *Markers/Keep-out zones topic* | Topic with the `polygon_msgs/msg/Polygon2DCollection` zones to tell the two lifetimes apart (and to colour the detections by them). |
| *Markers/Min. probability* | Detections with a lower probability than this are not visualized. |
| *Markers/Color by* | Whether the colour of the marker expresses the probability, the class, the keep-out zone, or nothing. |
| *Markers/Merge similar detections/…* | The 3D, image and time distances below which a new detection replaces an existing one. 0 in any of them turns the merging off. |
| *Labels/Show labels* | Whether the text labels next to the detections are shown. |
| *Labels/…* | Which information to show in the label (the class, its probability, the ID, the time, the keep-out zone), and in which font. |
| *Labels/Max. labels* | Maximal number of labels shown at once; the most important detections are labelled first. |
| *Image previews/Show image previews* | Whether the detections can be shown with an image preview. |
| *Image previews/Max. images* | Maximal number of detections shown with a preview; each of them needs a texture. |
| *Image previews/Preview width* | Width of a preview in the view [px]; the previews always have this size, so zooming changes how many of them fit. |
| *Image previews/Preview padding* | The picture of a detection shows its bounding box made this many times bigger. The `bbox_scale` parameter of the `detection_visualizer` node means the same and has the same default, so that a preview shows the very cutout the operator clicked on in the visualization strip. |
| *Image previews/Spacing* | Minimal spacing between two annotations [px]. |
| *Image previews/Hover delay* | How long the mouse has to point at a detection to raise its preview to the top [s]. |
| *Selection/Clicked image topic* | Topic to listen on for the places the user clicks in a camera image. |
| *Selection/Clicked image type* | Whether that topic carries `std_msgs/msg/Header` (the clicked image) or `geometry_msgs/msg/Point` (the clicked place in it). |
| *Selection/Image time tolerance* | Maximal time difference between a detection and the clicked image for it to be selected [s]. |
| *Selection/Highlight* | How long a clicked detection stays highlighted [s]. Clicking it again starts the time again. |
| *Selection/Marker scale* | How much bigger the marker of a highlighted detection is. |
| *Selection/Color* | Colour of the frame of the highlighted preview and of its marker. |
| *Persistent history/Show history* | Whether the detections read from the persistent storage are shown together with the received ones. |
| *Persistent history/Storage folder* | Folder with the stored detections. Defaults to `$ROS_HOME/g4s_mission`, a leading `~` is expanded. |
| *Persistent history/Global frame* | Frame the global (Earth-centred) positions are expressed in, typically `FP_ECEF`. It is also the frame the received detections are remembered in. Empty to keep every detection in the frame it arrived in. |
| *Persistent history/Load from (s ago)* | Age of the oldest detection to load [s]. 0 loads the whole storage whatever its age is. |
| *Persistent history/Load to (s ago)* | Age of the newest detection to load [s]. 0 loads detections of any age, even ones stamped in the future. |
| *Persistent history/Classes* | Comma-separated list of the classes to load, empty for all of them. |
| *Persistent history/Keep-out zones* | Whether to show only the detections recorded inside a keep-out zone, or outside of all of them. |
| *Persistent history/Use stored images* | Whether the previews of the stored detections can be taken from the images saved with them. |
| *Persistent history/Reload period* | How often to read the storage again [s]. 0 loads the history only when the settings change. |
| *Persistent history/Start delay* | How long to wait with loading the history after RViz starts [s], so that the startup is not blocked. |
| *Persistent history/Batch size* and *Batch pause* | How many detections to load at once and how long to wait between the batches [s]. The progress is reported in the status of the display. |

## Persistent history

Besides the detections received on a topic, the display can also show the detections read from the persistent
detection storage written by the `detection_recorder` node of the `g4s_mission` package. Both sources are shown at
once, so the operator can see where the objects were seen before and what the detector reports right now. The
differences between the two sources are deliberate:

- The recorder stores the global (Earth-centred) position of every localized detection, so the stored detections are
  placed in the *Global frame* (default `FP_ECEF`) when it is part of the current TF tree. That is the only position
  which means the same place in the next mission, when the local frame of the robot starts elsewhere. Detections
  recorded without it, i.e. by an older version of the recorder, are placed by the newest known transform of the frame
  they were recorded in, because the transform of their (long past) timestamp is not available anymore. Their preferred
  position is then the one in the frame of the keep-out zones, which the recorder resolved when the detection arrived.
  Set the *Global frame* property to an empty string to always use the local frame of the recorded mission.
- Their preview comes from the image cropped around the detection bounding box and saved with it, so no live camera
  images are needed to look at the history.
- They are loaded only after *Persistent history/Start delay*, and then in batches with *Batch pause* between them, so
  that reading thousands of stored detections does not freeze the user interface. The loading progress and the number of
  the detections which were loaded, filtered out, and which are really shown are all reported in the status.
- Their label carries the time of the detection and the number of the keep-out zone it was recorded in, and the
  *Color by* property offers a *Keep-out zone* mode which colours the detections inside a zone red and the ones
  outside of every zone green. Reading a longer period this way shows where the intrusions happened.

The storage layout (one `detections_YYYY-MM-DD.csv` per day and the crops in `images/YYYY-MM-DD/<class>/`) and the
numbering of the zones are documented in the `g4s_mission` package, which also provides a command-line tool to query
the same data.

## Keeping a busy scene responsive

A scene with thousands of remembered detections is drawn without recomputing it every frame:

- The scene is rebuilt only when something changed - a detection arrived or was forgotten, the camera moved, the mouse
  moved onto another detection, or a property changed. An idle frame costs a few comparisons.
- The transform of the detections to the frame of the view is looked up once per the frame and timestamp of the
  detections, not once per detection, and the detections remembered in the global frame share one static transform.
- The images of the stored detections are only read from the disk for the detections which can realistically win one of
  the preview slots, and a preview which is already shown is kept without touching any image again.
- The markers, labels and preview quads are reused between the frames and their rendering resources are only destroyed
  in the rendering thread. The names of the textures and materials contain the name of the RViz node, so several RViz
  instances in one process (such as the one of `rqt_rviz`) cannot clash over them.

## Limitations

- A detection without a pose in `results[].pose.pose` cannot be placed in the scene, so it is not visualised (the
  display reports it in its status). Use an image overlay tool if your detections have no 3D poses. This applies to
  the stored detections with an unknown pose as well, they are only listed in the status of the history.
- The sphere marking a detection has the same size for all of them (the *Markers/Size* property), because
  `vision_msgs/msg/Detection2D` does not carry the 3D dimensions of the detected object.
