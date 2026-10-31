#!/usr/bin/env python3
"""
transmitter-1.py  —  TAC 2026  |  Camera 1  Logitech C270  (/dev/video2  →  port 5001)
========================================================================================
Captures from Logitech C270 HD WEBCAM on /dev/video2, applies GStreamer-native
brightness/contrast lift (videobalance), encodes H.264, streams UDP to port 5001.

The heavy per-frame enhancement (all 8 fixes: undistort, WB, CLAHE, dehaze, etc.)
runs on the RECEIVER side in receiver-1.py / underwater_enhance.py (cam_id=1).

What IS done here (GStreamer-native):
  • videobalance brightness=+0.08 contrast=1.15   → slightly stronger lift for C270
    (C270 auto-exposure tends to underexpose more than Groov-e)
  • queue leaky=downstream max-size-buffers=1      → always latest frame
  • x264enc tune=zerolatency bframes=0             → minimum encode latency

USAGE:
  ros2 run nemo_logitech_wrapper transmitter_1
  ros2 run nemo_logitech_wrapper transmitter_1 --ros-args \\
      -p device:=/dev/video2 -p host:=192.168.2.1 -p port:=5001
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
        super().__init__("gst_tx_node_cam1")

        # ── ROS parameters
        self.declare_parameter("device",           "/dev/video4")  # Logitech C270
        self.declare_parameter("host",             "192.168.2.1")
        self.declare_parameter("port",             5001)           # separate from Groov-e (5000)
        self.declare_parameter("width",            640)
        self.declare_parameter("height",           480)
        self.declare_parameter("fps",              30)
        self.declare_parameter("bitrate_kbps",     5000)
        self.declare_parameter("speed_preset",     "veryfast")
        self.declare_parameter("key_int_max",      60)
        self.declare_parameter("use_videoconvert", True)
        # Slightly stronger lift for C270 (darker auto-exposure than Groov-e)
        self.declare_parameter("gst_brightness",   0.08)
        self.declare_parameter("gst_contrast",     1.15)

        Gst.init(None)

        pipe_str = self._build_pipeline()
        self.get_logger().info(f"[Cam1 C270] TX pipeline:\n{pipe_str}")

        self.pipeline = Gst.parse_launch(pipe_str)
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        self.loop = GLib.MainLoop()
        self._gst_thread = threading.Thread(target=self._run, daemon=True)
        self._gst_thread.start()

        # NOTE: use rclpy.get_default_context().on_shutdown() — NOT add_on_shutdown()
        # (add_on_shutdown does not exist in rclpy; the original transmitter-1 had this bug)
        rclpy.get_default_context().on_shutdown(self._shutdown_hook)

    # ──────────────────────────────────────────
    def _build_pipeline(self) -> str:
        device     = self.get_parameter("device").value
        host       = self.get_parameter("host").value
        port       = self.get_parameter("port").value
        width      = self.get_parameter("width").value
        height     = self.get_parameter("height").value
        fps        = self.get_parameter("fps").value
        bitrate    = self.get_parameter("bitrate_kbps").value
        speed      = self.get_parameter("speed_preset").value
        keyint     = self.get_parameter("key_int_max").value
        use_vc     = self.get_parameter("use_videoconvert").value
        brightness = self.get_parameter("gst_brightness").value
        contrast   = self.get_parameter("gst_contrast").value

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
            self.get_logger().error("[Cam1] Failed to set TX pipeline to PLAYING")
            return
        try:
            self.loop.run()
        except Exception as e:
            self.get_logger().error(f"[Cam1] GLib loop: {e}")

    def _on_bus_message(self, bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            self.get_logger().error(f"[Cam1] Gst ERROR: {err} | {dbg}")
        elif t == Gst.MessageType.EOS:
            self.get_logger().warn("[Cam1] Gst EOS")
        elif t == Gst.MessageType.WARNING:
            w, dbg = msg.parse_warning()
            self.get_logger().warn(f"[Cam1] Gst WARNING: {w} | {dbg}")

    def _shutdown_hook(self):
        self.get_logger().info("[Cam1] Stopping TX pipeline…")
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
