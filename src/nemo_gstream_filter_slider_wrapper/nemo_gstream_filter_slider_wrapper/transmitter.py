#!/usr/bin/env python3
"""
transmitter.py  —  TAC 2026  |  Camera 0  Groov-e  (/dev/video0  →  port 5000)
================================================================================
Captures from Groov-e camera, applies lightweight GStreamer-side pre-processing
(videobalance for brightness/contrast, videoconvert for colour space), then
encodes H.264 and streams over UDP to the laptop.

The heavy per-frame computer-vision enhancement (all 8 fixes) is done on the
RECEIVER side in receiver.py / underwater_enhance.py because:
  - Jetson CPU is busy with thruster control, IMU, etc.
  - The enhance pipeline uses numpy + OpenCV which are not GStreamer elements.
  - receiver.py can toggle TAC mode live without restarting the Jetson stream.

What IS done here (GStreamer-native, near-zero CPU cost):
  • videobalance brightness=+0.05 contrast=1.10   → mild lift before encode
  • queue leaky=downstream max-size-buffers=1      → always send latest frame
  • x264enc tune=zerolatency bframes=0             → minimum encode latency

KEYS:   none — this is a background ROS 2 node.

USAGE:
  ros2 run nemo_logitech_wrapper transmitter
  ros2 run nemo_logitech_wrapper transmitter --ros-args \\
      -p device:=/dev/video0 -p host:=192.168.2.1 -p port:=5000
"""

import threading
import rclpy
from rclpy.node import Node

try:
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GObject", "2.0")
    from gi.repository import Gst, GLib
except Exception as e:
    raise RuntimeError(
        "Missing GStreamer Python bindings.\n"
        "Install: sudo apt install python3-gi gir1.2-gstreamer-1.0"
    ) from e


class GstTxNode(Node):
    def __init__(self):
        super().__init__("gst_tx_node_cam0")

        # ── ROS parameters
        self.declare_parameter("device",          "/dev/video0")   # Groov-e
        self.declare_parameter("host",            "192.168.2.1")
        self.declare_parameter("port",            5000)
        self.declare_parameter("width",           640)
        self.declare_parameter("height",          480)
        self.declare_parameter("fps",             30)
        self.declare_parameter("bitrate_kbps",    5000)
        self.declare_parameter("speed_preset",    "veryfast")
        self.declare_parameter("key_int_max",     60)
        self.declare_parameter("use_videoconvert", True)
        # GStreamer-side brightness lift (0.0 = no change, range -1..1)
        self.declare_parameter("gst_brightness",  0.05)
        # GStreamer-side contrast boost (1.0 = no change, range 0..2)
        self.declare_parameter("gst_contrast",    1.10)

        Gst.init(None)

        pipe_str = self._build_pipeline()
        self.get_logger().info(f"[Cam0 Groov-e] TX pipeline:\n{pipe_str}")

        self.pipeline = Gst.parse_launch(pipe_str)
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        self.loop = GLib.MainLoop()
        self._gst_thread = threading.Thread(target=self._run, daemon=True)
        self._gst_thread.start()

        rclpy.get_default_context().on_shutdown(self._shutdown_hook)

    # ──────────────────────────────────────────
    def _build_pipeline(self) -> str:
        device    = self.get_parameter("device").value
        host      = self.get_parameter("host").value
        port      = self.get_parameter("port").value
        width     = self.get_parameter("width").value
        height    = self.get_parameter("height").value
        fps       = self.get_parameter("fps").value
        bitrate   = self.get_parameter("bitrate_kbps").value
        speed     = self.get_parameter("speed_preset").value
        keyint    = self.get_parameter("key_int_max").value
        use_vc    = self.get_parameter("use_videoconvert").value
        brightness = self.get_parameter("gst_brightness").value
        contrast   = self.get_parameter("gst_contrast").value

        # videobalance gives a mild brightness + contrast lift at zero CPU cost
        # (hardware-accelerated in GStreamer).  This is not a replacement for the
        # full enhance pipeline on the receiver but it stops the encoder wasting
        # bits on very dark frames.
        videobalance = (
            f"videobalance brightness={brightness:.3f} contrast={contrast:.3f} ! "
        )
        convert = "videoconvert ! " if use_vc else ""

        return (
            f"v4l2src device={device} do-timestamp=true ! "
            f"video/x-raw,width={width},height={height},framerate={fps}/1 ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"{convert}"
            f"{videobalance}"
            f"x264enc tune=zerolatency speed-preset={speed} bitrate={bitrate} "
            f"key-int-max={keyint} bframes=0 ! "
            f"rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={host} port={port} sync=false async=false"
        )

    # ──────────────────────────────────────────
    def _run(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("[Cam0] Failed to set TX pipeline to PLAYING")
            return
        try:
            self.loop.run()
        except Exception as e:
            self.get_logger().error(f"[Cam0] GLib loop: {e}")

    def _on_bus_message(self, bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            self.get_logger().error(f"[Cam0] Gst ERROR: {err} | {dbg}")
        elif t == Gst.MessageType.EOS:
            self.get_logger().warn("[Cam0] Gst EOS")
        elif t == Gst.MessageType.WARNING:
            w, dbg = msg.parse_warning()
            self.get_logger().warn(f"[Cam0] Gst WARNING: {w} | {dbg}")

    def _shutdown_hook(self):
        self.get_logger().info("[Cam0] Stopping TX pipeline…")
        try:
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            if self.loop and self.loop.is_running():
                self.loop.quit()
        except Exception:
            pass


def main():
    rclpy.init()
    node = GstTxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
