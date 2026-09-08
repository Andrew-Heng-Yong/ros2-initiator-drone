"""Local camera portal. Run: python3 -m tracking.server [--demo]."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import signal
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

import cv2
import numpy as np

from .sensor_stream import SensorStream
from .mapping import SceneMap
from .odometry import RGBDOdometry

ROOT = Path(__file__).resolve().parents[1]


class Tracker:
    def __init__(self, method='pnp', gyro=None, demo=False, processing='pi', alignment=None, thermal_range=(19., 28.)):
        # ponytail: one processing lock; separate snapshots if HTTP latency becomes limiting.
        self.lock = threading.RLock()
        if len(thermal_range) != 2 or not np.isfinite(thermal_range).all() or thermal_range[1] <= thermal_range[0]:
            raise ValueError('Thermal range must contain increasing finite limits')
        self.thermal_range = tuple(thermal_range)
        self.processing = processing
        self.stream = SensorStream(alignment, demo)
        self.temperatures = None
        self.thermal_stamp = None
        self.method, self.gyro, self.demo = method, gyro, demo
        self.mapper = SceneMap()
        self.odom = None
        self.pose = np.eye(4)
        self.path = deque(maxlen=2000)
        self.images = {}
        self.image_times = {}
        self.status, self.reason = 'waiting', 'Waiting for RGB and depth'
        self.last_frame = 0
        self.last_stamp = None
        self.sequence = self.map_version = 0
        self.metrics = {}
        self.reset_requested = False
        self.record_dir = None
        self.record_count = 0
        self.K = None
        self.map_pose = None

    def preview(self, name, image):
        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.lock:
                self.images[name] = jpeg.tobytes()
                self.image_times[name] = time.time()

    def thermal(self, temperatures, stamp=None):
        with self.lock:
            self.temperatures = np.asarray(temperatures, dtype='<f4').copy()
            self.thermal_stamp = time.time() if stamp is None else stamp
        valid = np.isfinite(temperatures)
        if not valid.any():
            return
        low, high = self.thermal_range
        normalized = np.nan_to_num((temperatures - low) * 255 / max(high-low, 1))
        self.preview('thermal', cv2.applyColorMap(normalized.clip(0, 255).astype('uint8'), cv2.COLORMAP_INFERNO))
        with self.lock:
            self.metrics['thermal_range'] = [round(float(low), 1), round(float(high), 1)]

    def process(self, rgb, depth, K, stamp, sync_error=0., depth_stamp=None):
        start = time.monotonic()
        self.preview('rgb', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        display = cv2.applyColorMap((np.nan_to_num(depth) * 255 / 6).clip(0, 255).astype('uint8'), cv2.COLORMAP_TURBO)
        display[~np.isfinite(depth) | (depth <= 0)] = 0
        self.preview('depth', display)
        with self.lock:
            self.last_frame = time.time()
            if K is None or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
                self.status, self.reason = 'uncalibrated', 'Camera intrinsics are missing or invalid'
                return
            if rgb.shape[:2] != depth.shape:
                self.status, self.reason = 'uncalibrated', 'Depth must be registered to RGB'
                return
            if self.reset_requested or (self.K is not None and not np.allclose(K, self.K)):
                self.stream.reset()
            gyro_payload = {}
            if self.gyro:
                snap = self.gyro.recording_snapshot()
                samples = np.asarray(snap['samples'])
                samples = samples[samples[:, 0] >= stamp-1.2] if len(samples) else samples
                status = self.gyro.status()
                gyro_payload = dict(samples=samples.tolist(),
                                    bias=np.nan_to_num(snap['bias_rad_s']).tolist(),
                                    ready=status.get('fusion_ready', False),
                                    rotation=status.get('rotation_camera_from_gyro', np.eye(3).tolist()))
            self.stream.publish(self.images['rgb'], depth, K, stamp,
                                stamp if depth_stamp is None else depth_stamp,
                                self.temperatures, self.thermal_stamp, gyro_payload)
            if self.processing == 'phone':
                self.K = K.copy()
                self.reset_requested = False
                self.sequence += 1
                self.status, self.reason = 'streaming', 'Phone processing · Pi captures and records'
                self.record_frame(rgb, depth, K, stamp, depth_stamp, sync_error)
                self.metrics['processing_ms'] = round((time.monotonic()-start)*1000, 1)
                return
            if self.odom is None or self.reset_requested or not np.allclose(K, self.K):
                self.odom = RGBDOdometry(K, method=self.method)
                self.K = K.copy()
                self.mapper.clear()
                self.path.clear()
                self.pose = np.eye(4)
                self.last_stamp = None
                self.reset_requested = False
                self.map_version += 1
                self.map_pose = None
            prior = None
            gyro_start = time.monotonic()
            # Anchor each short gyro increment to the last accepted visual pose.
            # A long-lived keyframe must not accumulate uncorrected gyro drift.
            if self.gyro and self.last_stamp is not None and 0 < stamp-self.last_stamp <= 1.:
                increment = self.gyro.relative_rotation(self.last_stamp, stamp)
                if increment is not None:
                    prior = self.odom.rotation_prior_from_increment(increment)
            gyro_ms = (time.monotonic()-gyro_start)*1000
            result = self.odom.update(rgb, depth, stamp, rotation_prior=prior)
            if result['status'] in ('tracking', 'initializing'):
                self.last_stamp = stamp
            self.status = result['status']
            self.reason = {'initializing': 'Establishing visual reference', 'tracking': 'RGB-D tracking', 'lost': 'Tracking lost · return to a previously seen view'}.get(self.status, self.status)
            self.pose = np.asarray(result['pose'])
            if self.status == 'tracking':
                moved = self.map_pose is None or np.linalg.norm(self.pose[:3,3]-self.map_pose[:3,3]) > .03
                turned = self.map_pose is None or np.linalg.norm(cv2.Rodrigues(self.map_pose[:3,:3].T@self.pose[:3,:3])[0]) > .04
                if moved or turned:
                    self.mapper.add(rgb, depth, K, self.pose)
                    self.map_pose = self.pose.copy()
                    self.map_version += 1
                self.path.append(self.pose[:3, 3].tolist())
            self.sequence += 1
            self.metrics.update(inliers=int(result.get('inliers', 0)), matches=int(result.get('matches', 0)),
                                rmse=result.get('rmse'), solver=result.get('solver'),
                                rmse_unit='pixels' if result.get('solver')=='pnp' else 'metres',
                                gyro_prior=prior is not None, gyro_ms=round(gyro_ms, 1),
                                sync_ms=round(sync_error*1000, 1),
                                valid_depth_percent=round(float(np.mean(np.isfinite(depth) & (depth>.2) & (depth<6)))*100, 1))
            self.record_frame(rgb, depth, K, stamp, depth_stamp, sync_error)
            self.metrics['processing_ms'] = round((time.monotonic()-start)*1000, 1)

    def record_frame(self, rgb, depth, K, stamp, depth_stamp, sync_error):
        if self.record_dir and self.record_count < 300:
            sensors = {'gyro_'+k: v for k,v in self.gyro.recording_snapshot().items()} if self.gyro else {}
            # Compression cost ~112 ms/frame on Pi versus ~15 ms for raw NPZ.
            target = self.record_dir / f'{self.record_count:05d}.npz'
            temporary = target.with_suffix('.npz.part')
            with temporary.open('wb') as stream:
                np.savez(stream, rgb=rgb, depth=depth, K=K, timestamp=stamp,
                         depth_timestamp=stamp if depth_stamp is None else depth_stamp,
                         sync_error=sync_error, pose=self.pose, tracking_status=self.status,
                         thermal=self.temperatures if self.temperatures is not None else np.empty((0,0)),
                         thermal_timestamp=self.thermal_stamp if self.thermal_stamp is not None else np.nan,
                         sensor_session=self.stream.session, **sensors)
            temporary.replace(target)
            self.record_count += 1
            if self.record_count == 300:
                self.record_dir = None

    def snapshot(self):
        with self.lock:
            age = time.time()-self.last_frame if self.last_frame else None
            stale = age is not None and age > 2
            return dict(processing=self.processing, sensor_session=self.stream.session, remote_poses=self.stream.pose_snapshot(), status='stale' if stale else self.status, reason='Camera frames stopped' if stale else self.reason,
                        mode='demo' if self.demo else 'live', method=self.method, pose=self.pose.tolist(),
                        trajectory=list(self.path), frame=self.sequence, map_version=self.map_version,
                        points=len(self.mapper), age=age, metrics=self.metrics.copy(),
                        images={k: round(time.time()-v, 2) for k,v in self.image_times.items()},
                        gyro=self.gyro.status() if self.gyro else {'state': 'unavailable'},
                        recording=self.record_dir is not None, recorded_frames=self.record_count)


def run_ros(tracker, stop):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('camera_and_gyro')
    bridge = CvBridge()
    queues = {'rgb': deque(maxlen=8), 'depth': deque(maxlen=8)}
    calibration = {}
    pending = deque(maxlen=1)  # Keep live latency bounded if processing takes longer than a frame.
    pending_thermal = deque(maxlen=1)
    condition = threading.Condition()

    def info(msg):
        calibration['K'] = np.array(msg.k).reshape(3, 3)
        calibration['D'] = np.array(msg.d)

    def frame(kind, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        queues[kind].append((stamp, msg))
        other = 'depth' if kind == 'rgb' else 'rgb'
        if not queues[other]:
            return
        match = min(queues[other], key=lambda pair: abs(pair[0]-stamp))
        error = abs(match[0]-stamp)
        # Unsynchronized 15 Hz streams drift through a 33 ms half-period.
        # A 20 ms cutoff caused measured 2.8 s stretches without a pair.
        if error > .035:
            return
        queues[other].remove(match)
        queues[kind].pop()
        rgb_msg, depth_msg = (msg, match[1]) if kind == 'rgb' else (match[1], msg)
        with condition:
            pending.append((rgb_msg, depth_msg, error, calibration.copy()))
            condition.notify()

    def worker():
        while not stop.is_set():
            with condition:
                condition.wait_for(lambda: pending or pending_thermal or stop.is_set(), timeout=.5)
                pair = pending.pop() if pending else None
                thermal_msg = pending_thermal.pop() if pending_thermal else None
            if thermal_msg is not None:
                try:
                    tracker.thermal(bridge.imgmsg_to_cv2(thermal_msg, 'passthrough'), thermal_msg.header.stamp.sec + thermal_msg.header.stamp.nanosec*1e-9)
                except Exception as error:
                    node.get_logger().error('Thermal preview: '+str(error))
            if pair is None:
                continue
            rgb_msg, depth_msg, error, cal = pair
            try:
                rgb = bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
                depth = bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype('float32')
                if depth_msg.encoding == '16UC1':
                    depth *= .001
                elif depth_msg.encoding != '32FC1':
                    raise ValueError('Unsupported depth encoding: '+depth_msg.encoding)
                if depth_msg.header.frame_id != rgb_msg.header.frame_id:
                    raise ValueError('Depth and RGB optical frames differ; enable registration')
                K, distortion = cal.get('K'), cal.get('D')
                if K is not None and K[0, 0] > 0 and distortion is not None and np.any(distortion):
                    h,w = depth.shape
                    mx,my = cv2.initUndistortRectifyMap(K,distortion,None,K,(w,h),cv2.CV_32FC1)
                    rgb = cv2.remap(rgb,mx,my,cv2.INTER_LINEAR)
                    depth = cv2.remap(depth,mx,my,cv2.INTER_NEAREST)
                stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec*1e-9
                depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec*1e-9
                tracker.process(rgb, depth, K, stamp, error, depth_stamp)
            except Exception as error:
                with tracker.lock:
                    tracker.status, tracker.reason = 'error', str(error)
                node.get_logger().error(str(error))

    def thermal_frame(msg):
        # ROS callbacks must not wait for the processing/map lock.
        with condition:
            pending_thermal.append(msg)
            condition.notify()

    node.create_subscription(CameraInfo, '/camera/color/camera_info', info, qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/color/image_raw', lambda m:frame('rgb',m), qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/depth/image_raw', lambda m:frame('depth',m), qos_profile_sensor_data)
    node.create_subscription(Image, '/thermal/image_raw', thermal_frame, qos_profile_sensor_data)
    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.2)
    finally:
        stop.set()
        with condition:
            condition.notify_all()
        thread.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


def handler_for(tracker):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, payload, content_type, status=200):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/sensors':
                with tracker.lock:
                    payload = tracker.stream.packet
                return self.reply(payload or b'Waiting for sensors', 'application/octet-stream', 200 if payload else 503)
            if path == '/api/clock':
                return self.reply(json.dumps(dict(timestamp=time.time())).encode(), 'application/json')
            if path == '/api/state':
                state = tracker.snapshot()
                # Invalid fits have no numerical residual; JSON never emits NaN/Infinity.
                rmse = state['metrics'].get('rmse')
                if rmse is not None and not np.isfinite(rmse):
                    state['metrics']['rmse'] = None
                return self.reply(json.dumps(state, allow_nan=False).encode(), 'application/json')
            if path == '/api/scene':
                with tracker.lock:
                    payload = np.asarray(tracker.mapper.points(), dtype='<f4').tobytes()
                return self.reply(payload, 'application/octet-stream')
            if path == '/api/map.ply':
                with tracker.lock:
                    payload = tracker.mapper.export_ply()
                return self.reply(payload, 'application/octet-stream')
            if path.startswith('/api/image/'):
                with tracker.lock:
                    payload = tracker.images.get(path.rsplit('/',1)[-1])
                return self.reply(payload or b'No camera image yet', 'image/jpeg' if payload else 'text/plain', 200 if payload else 503)
            assets = {'/':'index.html', '/app.js':'app.js', '/style.css':'style.css', '/three.module.js':'three.module.js', '/OrbitControls.js':'OrbitControls.js'}
            if path in assets:
                target = ROOT/'web'/assets[path]
                if target.is_file():
                    mime = 'text/html' if path=='/' else 'text/css' if path.endswith('.css') else 'text/javascript'
                    return self.reply(target.read_bytes(),mime)
            self.reply(b'Not found','text/plain',404)

        def do_POST(self):
            origin = self.headers.get('Origin')
            if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                return self.reply(b'Origin rejected','text/plain',403)
            if self.headers.get('Content-Type') != 'application/json':
                return self.reply(b'Expected application/json','text/plain',415)
            if self.path == '/api/poses':
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 4096:
                        raise ValueError('Invalid body length')
                    self.connection.settimeout(2)
                    value = json.loads(self.rfile.read(length))
                    if not isinstance(value, dict):
                        raise ValueError('Expected object')
                    with tracker.lock:
                        tracker.stream.accept_poses(value)
                    return self.reply(b'{}', 'application/json')
                except (ValueError, TypeError, OSError):
                    return self.reply(b'Invalid, stale or wrong-session pose', 'text/plain', 400)
            if self.path == '/api/reset':
                with tracker.lock:
                    tracker.reset_requested = True
                return self.reply(b'{}','application/json')
            if self.path == '/api/record':
                with tracker.lock:
                    if tracker.record_dir:
                        tracker.record_dir = None
                    else:
                        tracker.record_dir = ROOT/'recordings'/time.strftime('%Y%m%d-%H%M%S')
                        tracker.record_dir.mkdir(parents=True, exist_ok=True)
                        tracker.record_count = 0
                return self.reply(b'{}','application/json')
            self.reply(b'Not found','text/plain',404)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', default=8080, type=int)
    parser.add_argument('--method', choices=['svd','pnp'], default='pnp')
    parser.add_argument('--processing', choices=['pi', 'phone'], default='pi')
    parser.add_argument('--thermal-range', nargs=2, type=float, default=(19.,28.), metavar=('MIN_C','MAX_C'))
    parser.add_argument('--thermal-alignment', type=Path, default=ROOT/'config'/'thermal-alignment.json')
    parser.add_argument('--demo', action='store_true')
    default_gyro = ROOT/'config'/'gyro.json'
    parser.add_argument('--gyro-config', type=Path, default=default_gyro if default_gyro.is_file() else None)
    args = parser.parse_args()
    cv2.setNumThreads(2)
    gyro = None
    if not args.demo:
        from .gyro import Gyro
        config = json.loads(args.gyro_config.read_text()) if args.gyro_config else {}
        gyro = Gyro(**config)
        gyro.start()
    alignment = json.loads(args.thermal_alignment.read_text()) if args.thermal_alignment.is_file() else {}
    tracker = Tracker(args.method, gyro, args.demo, args.processing, alignment, args.thermal_range)
    stop = threading.Event()
    def capture():
        try:
            if args.demo:
                from .demo import run_demo
                run_demo(tracker, stop)
            else:
                run_ros(tracker, stop)
        except Exception as error:
            with tracker.lock:
                tracker.status, tracker.reason = 'error', str(error)
            print('Capture error:',error,flush=True)
    thread = threading.Thread(target=capture,daemon=True)
    thread.start()
    server = ThreadingHTTPServer((args.host,args.port),handler_for(tracker))
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    print(f'Camera + Gyro portal: http://{args.host}:{args.port}',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        thread.join(timeout=5)
        if gyro:
            gyro.close()


if __name__ == '__main__':
    main()
