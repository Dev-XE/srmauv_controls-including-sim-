#!/usr/bin/env python3
"""
nemo_perception.py - low-load dual camera streamer + logger
"""

import argparse
import csv
import os
import re
import threading
import time
from datetime import datetime

import cv2

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
CAP_W, CAP_H = 320, 240
CAP_FPS = 15
CAP_FPS_HIGH = 30
PIN_CORE = 3
NICE_VALUE = 10
REPORT_PERIOD_S = 5.0
TEMP_WARN_C = 75.0
REOPEN_AFTER_S = 2.0


class PrintLogger:
    def info(self, m):
        print(f'[INFO] {m}', flush=True)

    def warn(self, m):
        print(f'[WARN] {m}', flush=True)

    def error(self, m):
        print(f'[ERROR] {m}', flush=True)


def limit_resources():
    try:
        os.nice(NICE_VALUE)
    except OSError:
        pass
    try:
        cores = os.sched_getaffinity(0)
        if PIN_CORE in cores and len(cores) > 1:
            os.sched_setaffinity(0, {PIN_CORE})
    except (AttributeError, OSError):
        pass


def read_temp_c():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return float('nan')


def gstreamer_available():
    try:
        return re.search(r'GStreamer:\s+YES', cv2.getBuildInformation()) is not None
    except Exception:
        return False


# ---------------------------------------------------------------------------
# GStreamer live streamer
# ---------------------------------------------------------------------------
class Streamer:
    def __init__(self, name, host, port, fps, kbps, logger):
        self.name, self.host, self.port = name, host, port
        self.fps, self.kbps, self.log = fps, kbps, logger
        self.writer = None

    def _pipelines(self):
        tail = ('h264parse config-interval=1 ! rtph264pay pt=96 config-interval=1 ! '
                f'udpsink host={self.host} port={self.port} sync=false async=false')
        gop = max(1, int(self.fps))
        hw = ('appsrc ! videoconvert ! video/x-raw,format=I420 ! '
              f'v4l2h264enc extra-controls="controls,video_bitrate={self.kbps * 1000},'
              f'h264_i_frame_period={gop}" ! video/x-h264,level=(string)4 ! ' + tail)
        sw = ('appsrc ! videoconvert ! video/x-raw,format=I420 ! '
              f'x264enc tune=zerolatency speed-preset=ultrafast bitrate={self.kbps} '
              f'key-int-max={gop} ! ' + tail)
        return [('hardware H.264 (v4l2h264enc)', hw), ('software H.264 (x264enc)', sw)]

    def open(self):
        if not gstreamer_available():
            self.log.warn(f'[{self.name}] OpenCV build has no GStreamer support.')
            return False
        for label, pipe in self._pipelines():
            w = cv2.VideoWriter(pipe, cv2.CAP_GSTREAMER, 0, float(self.fps), (CAP_W, CAP_H), True)
            if w.isOpened():
                self.writer = w
                self.log.info(f'[{self.name}] streaming with {label} -> '
                              f'{self.host}:{self.port} @ {self.fps} fps, {self.kbps} kbps')
                return True
        self.log.warn(f'[{self.name}] no GStreamer pipeline could start.')
        return False

    def push(self, frame):
        if self.writer is not None:
            self.writer.write(frame)

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None


# ---------------------------------------------------------------------------
# Camera Worker
# ---------------------------------------------------------------------------
class CameraWorker(threading.Thread):
    def __init__(self, name, dev, fourcc, out_dir, rec_fps, record, stop_evt, logger,
                 stream_host='', stream_port=5000, stream_fps=10, stream_kbps=800):
        super().__init__(daemon=True)
        self.name = name
        self.dev = dev
        self.fourcc = fourcc
        self.out_dir = out_dir
        want = max(rec_fps, stream_fps if stream_host else 0)
        self.cap_fps = CAP_FPS if want <= CAP_FPS else CAP_FPS_HIGH
        self.rec_fps = min(rec_fps, self.cap_fps)
        self.record = record
        self.stop_evt = stop_evt
        self.log = logger
        self.grabbed = 0
        self.saved = 0
        self.streamed = 0
        self.stream_on = False
        self.streamer = None
        if stream_host:
            self.streamer = Streamer(name, stream_host, stream_port,
                                     min(stream_fps, self.cap_fps), stream_kbps, logger)

    def _open(self):
        cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.log.warn(f'[{self.name}] cannot open {self.dev}, retrying')
            return None
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_H)
        cap.set(cv2.CAP_PROP_FPS, self.cap_fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.log.info(
            f'[{self.name}] opened {self.dev}: '
            f'{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} '
            f'@ {cap.get(cv2.CAP_PROP_FPS):.0f} fps')
        return cap

    def run(self):
        period = 1.0 / self.rec_fps
        tol = 0.4 / self.cap_fps
        next_save = 0.0
        next_stream = 0.0
        cap, writer, csv_file, csv_w = None, None, None, None
        last_ok = time.time()
        try:
            self.stream_on = self.streamer is not None and self.streamer.open()
            s_period = (1.0 / self.streamer.fps) if self.stream_on else 0.0

            if self.record:
                csv_file = open(os.path.join(self.out_dir, f'{self.name}.csv'), 'w', newline='')
                csv_w = csv.writer(csv_file)
                csv_w.writerow(['frame', 'epoch_s'])

            while not self.stop_evt.is_set():
                if cap is None:
                    cap = self._open()
                    if cap is None:
                        time.sleep(1.0)
                        continue
                    last_ok = time.time()

                if not cap.grab():
                    if time.time() - last_ok > REOPEN_AFTER_S:
                        self.log.warn(f'[{self.name}] no frames, reopening {self.dev}')
                        cap.release()
                        cap = None
                    else:
                        time.sleep(0.01)
                    continue

                last_ok = time.time()
                self.grabbed += 1

                due_save = last_ok + tol >= next_save
                due_stream = self.stream_on and (last_ok + tol >= next_stream)
                if not (due_save or due_stream):
                    continue
                if due_save:
                    next_save += period
                    if next_save < last_ok:
                        next_save = last_ok + period
                if due_stream:
                    next_stream += s_period
                    if next_stream < last_ok:
                        next_stream = last_ok + s_period

                ok, frame = cap.retrieve()
                if not ok or frame is None:
                    continue

                # Ensure 3-channel BGR format for color streaming
                if len(frame.shape) == 2 or frame.shape[2] == 1:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

                if frame.shape[1] != CAP_W or frame.shape[0] != CAP_H:
                    frame = cv2.resize(frame, (CAP_W, CAP_H))

                ts = time.time()

                if due_stream:
                    self.streamer.push(frame)
                    self.streamed += 1

                if not due_save:
                    continue
                if self.record:
                    if writer is None:
                        path = os.path.join(self.out_dir, f'{self.name}.avi')
                        writer = cv2.VideoWriter(
                            path, cv2.VideoWriter_fourcc(*'MJPG'), self.rec_fps, (CAP_W, CAP_H))
                        if not writer.isOpened():
                            self.log.error(f'[{self.name}] cannot open writer, recording off')
                            self.record = False
                            continue
                    writer.write(frame)
                    csv_w.writerow([self.saved, f'{ts:.3f}'])
                    if self.saved % 25 == 0:
                        csv_file.flush()
                self.saved += 1
        finally:
            if cap is not None:
                cap.release()
            if writer is not None:
                writer.release()
            if csv_file is not None:
                csv_file.close()
            if self.streamer is not None:
                self.streamer.close()


# ---------------------------------------------------------------------------
# Core
# ---------------------------------------------------------------------------
class Perception:
    def __init__(self, log, cam0, cam1, record, rec_fps, out_root,
                 stream_host='', stream_port=5000, stream_fps=10, stream_kbps=800):
        run_dir = os.path.join(out_root, datetime.now().strftime('%Y%m%d_%H%M%S'))
        os.makedirs(run_dir, exist_ok=True)
        self.stop_evt = threading.Event()
        self.workers = [
            CameraWorker('forward', cam0, 'YUYV', run_dir, rec_fps, record, self.stop_evt, log,
                         stream_host, stream_port, stream_fps, stream_kbps),
            CameraWorker('down', cam1, 'MJPG', run_dir, rec_fps, record, self.stop_evt, log,
                         stream_host, stream_port + 1, stream_fps, stream_kbps),
        ]
        for w in self.workers:
            w.start()
        self.last_t = time.time()
        self.last_cpu = time.process_time()
        self.last_counts = {w.name: (0, 0, 0) for w in self.workers}
        log.info(f'Perception started (record={record}, rec_fps={rec_fps}, '
                 f'stream={stream_host or "off"}, dir={run_dir})')

    def sample(self):
        now, cpu_now = time.time(), time.process_time()
        dt = max(now - self.last_t, 1e-3)
        cpu_pct = (cpu_now - self.last_cpu) / dt * 100.0
        data, parts = [], []
        for w in self.workers:
            g0, s0, t0 = self.last_counts[w.name]
            gfps, sfps, tfps = (w.grabbed - g0) / dt, (w.saved - s0) / dt, (w.streamed - t0) / dt
            self.last_counts[w.name] = (w.grabbed, w.saved, w.streamed)
            data += [gfps, sfps]
            txt = f'{w.name}: grab {gfps:.1f} / save {sfps:.1f}'
            if w.stream_on:
                txt += f' / stream {tfps:.1f}'
            parts.append(txt + ' fps')
        temp = read_temp_c()
        data += [cpu_pct, temp]
        self.last_t, self.last_cpu = now, cpu_now
        text = ' | '.join(parts) + f' | cpu {cpu_pct:.0f}% of one core | {temp:.0f} C'
        return [float(x) for x in data], text, temp

    def shutdown(self):
        self.stop_evt.set()
        for w in self.workers:
            w.join(timeout=2.0)


# ---------------------------------------------------------------------------
# ROS 2 wrapper
# ---------------------------------------------------------------------------
if HAVE_ROS:
    class NemoPerceptionNode(Node):
        def __init__(self):
            super().__init__('nemo_perception')
            self.declare_parameter('cam0_dev', '/dev/video6')   # RealSense RGB
            self.declare_parameter('cam1_dev', '/dev/video0')   # Logitech C270
            self.declare_parameter('record', False)
            self.declare_parameter('rec_fps', 5)
            self.declare_parameter('out_root', os.path.expanduser('~/nemo_runs'))
            self.declare_parameter('stream_host', '192.168.2.1')
            self.declare_parameter('stream_port', 5000)
            self.declare_parameter('stream_fps', 15)
            self.declare_parameter('stream_kbps', 1000)
            g = lambda k: self.get_parameter(k).value
            self.core = Perception(
                self.get_logger(), g('cam0_dev'), g('cam1_dev'), g('record'),
                int(g('rec_fps')), g('out_root'),
                g('stream_host'), int(g('stream_port')), int(g('stream_fps')),
                int(g('stream_kbps')))
            self.pub = self.create_publisher(Float32MultiArray, '/camera_status', 10)
            self.create_timer(REPORT_PERIOD_S, self.report)

        def report(self):
            data, text, temp = self.core.sample()
            self.pub.publish(Float32MultiArray(data=data))
            self.get_logger().info(text)
            if temp >= TEMP_WARN_C:
                self.get_logger().warn(f'Temperature {temp:.0f} C - risk of throttling')

        def shutdown(self):
            self.core.shutdown()


def run_ros():
    rclpy.init()
    node = NemoPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_standalone(args):
    log = PrintLogger()
    core = Perception(log, args.cam0, args.cam1, not args.no_record,
                      args.rec_fps, os.path.expanduser(args.out_root),
                      args.stream_host, args.stream_port, args.stream_fps, args.stream_kbps)
    try:
        while True:
            time.sleep(REPORT_PERIOD_S)
            _, text, temp = core.sample()
            log.info(text)
            if temp >= TEMP_WARN_C:
                log.warn(f'Temperature {temp:.0f} C - risk of throttling')
    except KeyboardInterrupt:
        log.info('Stopping...')
    finally:
        core.shutdown()
        log.info('Done. Files saved.')


def main():
    p = argparse.ArgumentParser(description='NEMO low-load camera logger + GStreamer stream')
    p.add_argument('--standalone', action='store_true', help='run without ROS even if installed')
    p.add_argument('--cam0', default='/dev/video2', help='forward camera (RealSense RGB)')
    p.add_argument('--cam1', default='/dev/video0', help='downward camera (Logitech C270)')
    p.add_argument('--no-record', action='store_true', default=True, help='do not write video files')
    p.add_argument('--rec-fps', type=int, default=5, help='frames per second saved to disk')
    p.add_argument('--out-root', default='~/nemo_runs')
    p.add_argument('--stream-host', default='192.168.2.1', help='IP of the surface laptop')
    p.add_argument('--stream-port', type=int, default=5000, help='forward cam port; down cam = port+1')
    p.add_argument('--stream-fps', type=int, default=15, help='frames per second sent live')
    p.add_argument('--stream-kbps', type=int, default=1000, help='H.264 bitrate per camera')
    args, _ = p.parse_known_args()

    limit_resources()
    if HAVE_ROS and not args.standalone:
        run_ros()
    else:
        run_standalone(args)


if __name__ == '__main__':
    main()