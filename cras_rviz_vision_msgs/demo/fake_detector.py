#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague
"""
Publishes fake vision_msgs/Detection2DArray messages together with the camera images they come from.

It is meant for trying out the cras_rviz_vision_msgs/Detection2DArray RViz display::

    demo/fake_detector.py 12 cluster &
    rviz2 -d $(ros2 pkg prefix cras_rviz_vision_msgs)/share/cras_rviz_vision_msgs/rviz/detections.rviz

The detections of the following options are published repeatedly with the same IDs (the display is supposed to update
them instead of showing each message as new ones). With the `sparse` option, every message contains a different set of
detections with unique IDs, which shows how the display accumulates them (and how it copes with a lot of them at once),
and the `empty` option makes the detector regularly see nothing, which must not make the detections disappear.

With the `sim` option, the detector also publishes the `/clock` topic and stamps everything with a time shifted by
`SIM_SHIFT` seconds, which shows that the display works with the simulation time (run RViz with
`--ros-args -p use_sim_time:=true` to try it).

The `wide` option spreads the detections tens of metres apart instead of a few metres, which is the case when a lot
of image previews fit into the view at once.

The `extframe` option publishes the detections in the `camera` frame instead of `map`, so that the display has to
transform them (a static transform from `map` to `camera` has to be published for them to show up), and `noimage`
prevents the images from being published at all.

The `mask` option publishes a segmentation of the scene on `/mask`, where the pixels of the i-th detection of a message
carry the value `i + 1`. The `detection_visualizer` node of the g4s_mission package then replaces only the pixels of the
object when the object is detected again (`mask_topic` parameter), and keeps the older picture around it.

The `zones` option publishes a keep-out zone over a corner of the scene on the `polygons` topic, so that the detections
which fall into it can be seen coloured (and let expire after a different time) than the ones outside.

Usage: fake_detector.py [number_of_detections] [cluster] [sparse] [wide] [extframe] [noimage] [compressed]
       [empty] [sim] [mask] [zones] [duration_s]
"""
import math
import sys
import time

import numpy as np
import rclpy

from builtin_interfaces.msg import Time
from polygon_msgs.msg import Point2D, Polygon2D, Polygon2DCollection
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

CLASSES = ['car', 'person', 'bicycle']
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
# How many messages are published between the ones that carry nothing.
EMPTY_EVERY = 25
# How many detections a single message carries when the `sparse` option is used.
SPARSE_BATCH = 40
# How much is the simulated time behind the wall clock [s], to make a difference visible.
SIM_SHIFT = 100000.


class FakeDetector(Node):

    def __init__(self, count, cluster, sparse, wide, extframe, noimage, compressed, empty, sim,
                 mask, zones):
        super().__init__('fake_detector')
        self.count = count
        self.cluster = cluster
        self.sparse = sparse
        self.wide = wide
        self.extframe = extframe
        self.noimage = noimage
        self.compressed = compressed
        self.empty = empty
        self.sim = sim
        self.mask = mask
        self.round = 0
        if zones:
            # The zones are drawn once and stay, so they are published with the latched quality of service.
            self.zones = self.create_publisher(Polygon2DCollection, 'polygons', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.detections = self.create_publisher(Detection2DArray, '/detections', 10)
        self.image = self.create_publisher(Image, '/image', 10)
        self.mask_image = self.create_publisher(Image, '/mask', 10)
        self.compressed_image = self.create_publisher(CompressedImage, '/image/compressed', 10)
        if not self.noimage:
            self.image_timer = self.create_timer(0.05, self.publish_image)
        self.detection_timer = self.create_timer(0.2, self.publish_detections)
        if self.sim:
            self.clock = self.create_publisher(Clock, '/clock', 10)
        if zones:
            self.publish_zones()
        self.image_stamp = None
        if self.compressed:
            import cv2  # noqa: F401  N813  The compressed images are only made when this option is used.
            self.cv2 = cv2

    def positions(self, index):
        """Where the detection with the given index is, in the frame of the detections message."""
        # The detections are spread over a bounded grid of places, so that the ones published with new IDs (and
        # therefore kept in addition to the older ones) stay in the same area of the scene and can be seen.
        place = index % max(1, self.count)
        step = 20. if self.wide else .6
        if self.cluster:
            angle = 2.0 * math.pi * place / max(1, self.count)
            return .6 * math.cos(angle), .6 * math.sin(angle)
        rows = max(1, int(math.ceil(math.sqrt(max(1, self.count)))))
        return step * (place % rows - .5 * rows), step * (place // rows - .5 * rows)

    def pixel_positions(self, i):
        """Position of the i-th detection in the image, in pixels."""
        return (60.0 + 100.0 * (i % 6), 160.0 + 40.0 * (i % 3))

    def now(self):
        """Time of the detections and of the images.

        In the `sim` mode it is not the time the node sees, so that the shift of the simulated clock is visible also
        when RViz does not use it.
        """
        if not self.sim:
            return self.get_clock().now().to_msg()
        shifted = Time(sec=int(time.time() - SIM_SHIFT), nanosec=0)
        self.clock.publish(Clock(clock=shifted))
        return shifted

    def publish_image(self):
        now = self.now()
        self.image_stamp = now
        canvas = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
        canvas[:, :, 1] = 40
        # Draw a coloured block at the position of every detection bounding box, so that the image previews
        # attached to the detections can be told apart.
        segmentation = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH), dtype=np.uint8)
        for i in range(min(self.count, 18)):
            x, y = self.pixel_positions(i)
            color = ((30 + 20 * i) % 256, (220 - 15 * i) % 256, (90 + 25 * i) % 256)
            canvas[int(y - 40):int(y + 40), int(x - 45):int(x + 45)] = color
            # The segmentation of the same scene marks the pixels of the i-th detection of a message by i + 1.
            segmentation[int(y - 40):int(y + 40), int(x - 45):int(x + 45)] = i + 1
        if self.mask:
            message = Image()
            message.header.stamp = now
            message.header.frame_id = 'camera'
            message.height = IMAGE_HEIGHT
            message.width = IMAGE_WIDTH
            message.encoding = 'mono8'
            message.step = IMAGE_WIDTH
            message.data = segmentation.tobytes()
            self.mask_image.publish(message)
        if self.compressed:
            message = CompressedImage()
            message.header.stamp = now
            message.header.frame_id = 'camera'
            message.format = 'bgr8; png compressed rgb8'
            message.data = self.cv2.imencode('.png', canvas)[1].tobytes()
            self.compressed_image.publish(message)
        else:
            message = Image()
            message.header.stamp = now
            message.header.frame_id = 'camera'
            message.height = IMAGE_HEIGHT
            message.width = IMAGE_WIDTH
            message.encoding = 'bgr8'
            message.step = IMAGE_WIDTH * 3
            message.data = canvas.tobytes()
            self.image.publish(message)

    def publish_zones(self):
        """Publish a keep-out zone over a corner of the scene, once and for all."""
        message = Polygon2DCollection()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        zone = Polygon2D()
        # The detections of the grid stand .6 m apart around the origin, so a zone of this size covers a part of them
        # and leaves the others out of it - which is what the two lifetimes of the display are good for.
        for x, y in ((-1.3, -1.3), (-.5, -1.3), (-.5, -.5), (-1.3, -.5)):
            zone.points.append(Point2D(x=x, y=y))
        message.polygons.append(zone)
        self.zones.publish(message)

    def published(self):
        """Indices of the detections the next message contains."""
        self.round += 1
        if self.empty and self.round % EMPTY_EVERY == 0:
            return []  # the detector saw nothing, which must not hide the detections it saw before
        if not self.sparse:
            return range(self.count)
        # Each message brings other objects, so that the display has to keep them all.
        first = (self.round - 1) * SPARSE_BATCH
        return (first + i for i in range(SPARSE_BATCH))

    def publish_detections(self):
        message = Detection2DArray()
        message.header.stamp = self.image_stamp or self.now()
        # The detections of a real detector come in a sensor or odometry frame, which the display has to transform.
        message.header.frame_id = 'camera' if self.extframe else 'map'
        for index in self.published():
            x, y = self.positions(index)
            detection = Detection2D()
            detection.id = str(index)
            for class_index, class_name in enumerate(CLASSES):
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                # Rotate which class wins, so that the visualized classes and their colours vary.
                rank = (class_index + index) % len(CLASSES)
                hypothesis.hypothesis.score = .9 / (rank + 1) * (.3 + .7 * ((index * 7) % 10) / 10.)
                hypothesis.pose.pose.position.x = x
                hypothesis.pose.pose.position.y = y
                hypothesis.pose.pose.position.z = .5
                hypothesis.pose.pose.orientation.w = 1.
                detection.results.append(hypothesis)
            px, py = self.pixel_positions(index)
            detection.bbox.center.position.x = px
            detection.bbox.center.position.y = py
            detection.bbox.size_x = 90.
            detection.bbox.size_y = 80.
            message.detections.append(detection)
        self.detections.publish(message)


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    options = sys.argv[2:]
    sparse = 'sparse' in options
    wide = 'wide' in options
    extframe = 'extframe' in options
    noimage = 'noimage' in options
    compressed = 'compressed' in options
    empty = 'empty' in options
    sim = 'sim' in options
    mask = 'mask' in options
    zones = 'zones' in options
    numbers = [arg for arg in options if arg.isdigit()]
    deadline = time.time() + float(numbers[0]) if numbers else None
    rclpy.init()
    node = FakeDetector(count, 'cluster' in options, sparse, wide, extframe, noimage, compressed,
                        empty, sim, mask, zones)
    try:
        while rclpy.ok() and (deadline is None or time.time() < deadline):
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
