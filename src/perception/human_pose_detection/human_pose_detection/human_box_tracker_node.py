import os
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        try:
            from tensorflow.lite.python.interpreter import Interpreter
        except ImportError:
            try:
                from tensorflow.lite import Interpreter
            except ImportError:
                Interpreter = None


DEFAULT_PERSON_CLASS_IDS = [0, 1]
MODEL_PATH_ENV = 'HUMAN_BOX_MODEL_PATH'
COMMON_MODEL_PATHS = (
    '/home/andrew/ros2-initiator-drone/models/efficientdet_lite0.tflite',
    '/home/andrew/ros2-initiator-drone/models/lite-model_efficientdet_lite0_detection_metadata_1.tflite',
    '/home/andrew/ros2-initiator-drone/models/ssd_mobilenet_v1_1_metadata_1.tflite',
    '/home/andrew/models/efficientdet_lite0.tflite',
    '/home/andrew/models/lite-model_efficientdet_lite0_detection_metadata_1.tflite',
    '/home/andrew/models/ssd_mobilenet_v1_1_metadata_1.tflite',
    '~/models/efficientdet_lite0.tflite',
    '~/models/lite-model_efficientdet_lite0_detection_metadata_1.tflite',
    '~/models/ssd_mobilenet_v1_1_metadata_1.tflite',
)


class HumanBoxTrackerNode(Node):
    def __init__(self):
        super().__init__('human_box_tracker_node')
        self.bridge = CvBridge()

        self.declare_parameter('model_path', '')
        self.declare_parameter('model_name', 'efficientdet_lite0_person_boxes')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('boxes_topic', '/human_pose/keypoints')
        self.declare_parameter('person_detected_topic', '/human_pose/person_detected')
        self.declare_parameter('debug_image_topic', '/human_pose/debug_image')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('max_detections', 8)
        self.declare_parameter('person_class_ids', DEFAULT_PERSON_CLASS_IDS)
        self.declare_parameter('log_top_detections', True)
        self.declare_parameter('draw_candidate_boxes', True)
        self.declare_parameter('crop_left_px', 29)
        self.declare_parameter('crop_right_px', 29)
        self.declare_parameter('crop_top_px', 0)
        self.declare_parameter('crop_bottom_px', 0)
        self.declare_parameter('track_iou_threshold', 0.3)
        self.declare_parameter('max_track_missed_frames', 5)
        self.declare_parameter('max_inference_fps', 5.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('allow_missing_model_debug_stream', True)

        self.model_path = os.path.expanduser(self.get_parameter('model_path').value)
        self.model_name = self.get_parameter('model_name').value
        self.image_topic = self.get_parameter('image_topic').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.max_detections = int(self.get_parameter('max_detections').value)
        self.person_class_ids = {
            int(class_id) for class_id in self.get_parameter('person_class_ids').value
        }
        self.log_top_detections = bool(self.get_parameter('log_top_detections').value)
        self.draw_candidate_boxes = bool(self.get_parameter('draw_candidate_boxes').value)
        self.crop_left_px = int(self.get_parameter('crop_left_px').value)
        self.crop_right_px = int(self.get_parameter('crop_right_px').value)
        self.crop_top_px = int(self.get_parameter('crop_top_px').value)
        self.crop_bottom_px = int(self.get_parameter('crop_bottom_px').value)
        self.track_iou_threshold = float(self.get_parameter('track_iou_threshold').value)
        self.max_track_missed_frames = int(self.get_parameter('max_track_missed_frames').value)
        self.max_inference_fps = float(self.get_parameter('max_inference_fps').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.allow_missing_model_debug_stream = bool(
            self.get_parameter('allow_missing_model_debug_stream').value
        )
        self.last_inference_time = None
        self.next_track_id = 1
        self.tracks: List[Dict[str, float]] = []
        self.model_error: Optional[str] = None
        self.interpreter = None
        self.logged_output_details = False
        self.logged_first_detections = False
        self.latest_candidates: List[Tuple[int, float, float, float, float, float]] = []

        self._load_model()

        self.boxes_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('boxes_topic').value,
            10,
        )
        self.detected_pub = self.create_publisher(
            Bool,
            self.get_parameter('person_detected_topic').value,
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            self.get_parameter('debug_image_topic').value,
            10,
        )
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(
            f'Running {self.model_name} on {self.image_topic}; '
            f'box threshold={self.confidence_threshold:.2f}; '
            f'person class ids={sorted(self.person_class_ids)}; '
            f'crop left/right/top/bottom='
            f'{self.crop_left_px}/{self.crop_right_px}/{self.crop_top_px}/{self.crop_bottom_px}; '
            f'max detections={self.max_detections}; '
            f'max inference FPS={self.max_inference_fps:.1f}'
        )

    def _load_model(self) -> None:
        if Interpreter is None:
            self.handle_model_error(
                'Install a TensorFlow Lite interpreter: tflite_runtime, ai-edge-litert, or tensorflow.'
            )
            return

        self.model_path = self.resolve_model_path(self.model_path)
        if not self.model_path:
            self.handle_model_error(
                f'Human box model file not found. Set model_path or {MODEL_PATH_ENV}.'
            )
            return

        self.interpreter = Interpreter(model_path=self.model_path, num_threads=2)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()[0]
        self.output_details = self.interpreter.get_output_details()
        self.log_output_details()

        input_shape = self.input_details['shape']
        if len(input_shape) != 4 or int(input_shape[3]) != 3:
            raise RuntimeError(f'Unsupported detector input shape: {input_shape}')
        self.input_height = int(input_shape[1])
        self.input_width = int(input_shape[2])
        self.input_dtype = self.input_details['dtype']
        self.get_logger().info(
            f'Loaded {self.model_path} with input {self.input_width}x{self.input_height} '
            f'{self.input_dtype}'
        )

    def log_output_details(self) -> None:
        if self.logged_output_details:
            return
        self.logged_output_details = True
        for index, detail in enumerate(self.output_details):
            self.get_logger().info(
                'Output tensor '
                f'{index}: name={detail.get("name", "")}, '
                f'shape={list(detail.get("shape", []))}, '
                f'dtype={detail.get("dtype")}'
            )

    def resolve_model_path(self, configured_path: str) -> str:
        candidates = []
        env_path = os.environ.get(MODEL_PATH_ENV, '')
        if env_path:
            candidates.append(env_path)
        if configured_path:
            candidates.append(configured_path)
        candidates.extend(COMMON_MODEL_PATHS)

        for candidate in candidates:
            expanded = os.path.expanduser(candidate)
            if os.path.isfile(expanded):
                if expanded != configured_path:
                    self.get_logger().info(f'Using human box model: {expanded}')
                return expanded
        return ''

    def handle_model_error(self, message: str) -> None:
        if not self.allow_missing_model_debug_stream:
            raise RuntimeError(message)
        self.model_error = message
        self.get_logger().error(message)
        self.get_logger().warn(
            'Continuing without detector so /human_pose/debug_image still shows the cropped RGB stream.'
        )

    def image_callback(self, msg: Image) -> None:
        if not self.should_process_frame():
            return

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().warn(f'Unsupported camera image format: {error}')
            return

        cropped_bgr = self.crop_image(bgr_image)
        crop_height, crop_width = cropped_bgr.shape[:2]

        if self.interpreter is None:
            self.publish_result([])
            self.detected_pub.publish(Bool(data=False))
            if self.publish_debug_image:
                debug_image = self.draw_missing_model_image(cropped_bgr)
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))
            return

        rgb_image = cv2.cvtColor(cropped_bgr, cv2.COLOR_BGR2RGB)
        try:
            boxes = self.run_inference(rgb_image, crop_width, crop_height)
            tracked_boxes = self.update_tracks(boxes)
        except Exception as error:
            self.get_logger().warn(f'Box inference failed for this frame: {error}')
            return

        detected = len(tracked_boxes) > 0
        self.publish_result(tracked_boxes)
        self.detected_pub.publish(Bool(data=detected))

        if self.publish_debug_image:
            debug_image = self.draw_debug_image(cropped_bgr, tracked_boxes)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))

    def crop_image(self, image: np.ndarray) -> np.ndarray:
        height, width = image.shape[:2]
        left = max(0, min(width - 1, self.crop_left_px))
        right = max(0, min(width - left - 1, self.crop_right_px))
        top = max(0, min(height - 1, self.crop_top_px))
        bottom = max(0, min(height - top - 1, self.crop_bottom_px))
        cropped = image[top:height - bottom, left:width - right]
        if cropped.size == 0:
            self.get_logger().warn('Configured RGB crop is empty; using full frame.')
            return image
        return cropped

    def draw_missing_model_image(self, image: np.ndarray) -> np.ndarray:
        output = image.copy()
        message = self.model_error or 'Human box model missing.'
        cv2.rectangle(output, (8, 8), (min(output.shape[1] - 1, 574), 58), (0, 0, 0), -1)
        cv2.putText(
            output,
            message[:80],
            (16, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (48, 145, 255),
            1,
            cv2.LINE_AA,
        )
        return output

    def should_process_frame(self) -> bool:
        if self.max_inference_fps <= 0:
            return True

        now = self.get_clock().now()
        if self.last_inference_time is not None:
            elapsed = (now - self.last_inference_time).nanoseconds / 1e9
            if elapsed < 1.0 / self.max_inference_fps:
                return False
        self.last_inference_time = now
        return True

    def run_inference(self, rgb_image: np.ndarray, width: int, height: int) -> List[Tuple[float, float, float, float, float]]:
        input_image = cv2.resize(rgb_image, (self.input_width, self.input_height), interpolation=cv2.INTER_LINEAR)
        input_tensor = np.expand_dims(input_image, axis=0)

        if np.issubdtype(self.input_dtype, np.floating):
            input_tensor = input_tensor.astype(self.input_dtype) / 255.0
        else:
            input_scale, input_zero_point = self.input_details.get('quantization', (0.0, 0))
            if input_scale and input_scale > 0:
                quantized = np.round(input_tensor.astype(np.float32) / input_scale + input_zero_point)
            else:
                quantized = input_tensor.astype(np.float32)
            limits = np.iinfo(self.input_dtype)
            input_tensor = np.clip(quantized, limits.min, limits.max).astype(self.input_dtype)

        self.interpreter.set_tensor(self.input_details['index'], input_tensor)
        self.interpreter.invoke()
        outputs = {
            detail.get('name', f'output_{index}'): self.dequantize(
                self.interpreter.get_tensor(detail['index']),
                detail,
            )
            for index, detail in enumerate(self.output_details)
        }
        return self.decode_outputs(outputs, width, height)

    def dequantize(self, tensor: np.ndarray, detail) -> np.ndarray:
        if np.issubdtype(tensor.dtype, np.floating):
            return tensor

        scale, zero_point = detail.get('quantization', (0.0, 0))
        if scale and scale > 0:
            return (tensor.astype(np.float32) - zero_point) * scale
        return tensor.astype(np.float32)

    def decode_outputs(self, outputs, width: int, height: int) -> List[Tuple[float, float, float, float, float]]:
        by_name = {name.lower(): np.squeeze(value) for name, value in outputs.items()}
        boxes = self.find_output(by_name, ('box', 'location'))
        scores = self.find_output(by_name, ('score',))
        classes = self.find_output(by_name, ('class', 'category'))

        if boxes is None or scores is None:
            boxes, scores, classes = self.find_positional_outputs(list(by_name.values()))
        if boxes is None or scores is None:
            raise RuntimeError('Could not identify detector boxes and scores outputs.')

        boxes = np.asarray(boxes)
        scores = np.asarray(scores).reshape(-1)
        classes = np.zeros_like(scores) if classes is None else np.asarray(classes).reshape(-1)

        if boxes.ndim == 1:
            boxes = boxes.reshape(-1, 4)
        boxes = boxes.reshape(-1, boxes.shape[-1])[:, :4]

        detections = []
        candidates = []
        for box, score, class_id in zip(boxes, scores, classes):
            class_value = int(round(float(class_id)))
            x, y, w, h = self.decode_box(box, width, height)
            if w <= 1.0 or h <= 1.0:
                continue
            candidates.append((class_value, x, y, w, h, float(score)))

            if len(detections) >= self.max_detections:
                break
            if float(score) < self.confidence_threshold:
                continue
            if class_value not in self.person_class_ids:
                continue

            detections.append((x, y, w, h, float(score)))
        self.latest_candidates = candidates[:self.max_detections]
        self.log_detection_sample(scores, classes, detections)
        return detections

    def find_output(self, outputs, hints: Sequence[str]) -> Optional[np.ndarray]:
        for name, value in outputs.items():
            if all(hint in name for hint in hints):
                return value
        return None

    def find_positional_outputs(self, values: List[np.ndarray]):
        box_candidates = [value for value in values if np.asarray(value).shape[-1:] == (4,)]
        vectors = [
            np.asarray(value).reshape(-1)
            for value in values
            if np.asarray(value).ndim <= 2
            and np.asarray(value).size > 1
            and np.asarray(value).shape[-1:] != (4,)
        ]
        boxes = box_candidates[0] if box_candidates else None
        classes = None
        scores = None

        for vector in vectors:
            if np.all(vector >= 0.0) and np.all(vector <= 1.0) and not np.allclose(vector, np.round(vector)):
                scores = vector
                break

        for vector in vectors:
            if scores is not None and vector is scores:
                continue
            if np.allclose(vector, np.round(vector), atol=0.01):
                classes = vector
                break

        if scores is None and vectors:
            scores = vectors[0]
        if classes is None and len(vectors) > 1:
            classes = vectors[1]
        return boxes, scores, classes

    def log_detection_sample(
        self,
        scores: np.ndarray,
        classes: np.ndarray,
        detections: List[Tuple[float, float, float, float, float]],
    ) -> None:
        if not self.log_top_detections or self.logged_first_detections:
            return
        self.logged_first_detections = True
        count = min(5, len(scores), len(classes))
        sample = [
            f'class={int(round(float(classes[index])))} score={float(scores[index]):.2f}'
            for index in range(count)
        ]
        self.get_logger().info(
            f'First detector candidates: {", ".join(sample) if sample else "none"}; '
            f'accepted person boxes={len(detections)}'
        )

    def decode_box(self, box: np.ndarray, width: int, height: int) -> Tuple[float, float, float, float]:
        values = [float(value) for value in box]
        if all(0.0 <= value <= 1.5 for value in values):
            y_min, x_min, y_max, x_max = values
            x = x_min * width
            y = y_min * height
            w = (x_max - x_min) * width
            h = (y_max - y_min) * height
        else:
            y_min, x_min, y_max, x_max = values
            x = x_min
            y = y_min
            w = x_max - x_min
            h = y_max - y_min

        x = max(0.0, min(float(width - 1), x))
        y = max(0.0, min(float(height - 1), y))
        w = max(0.0, min(float(width) - x, w))
        h = max(0.0, min(float(height) - y, h))
        return x, y, w, h

    def update_tracks(
        self,
        boxes: List[Tuple[float, float, float, float, float]],
    ) -> List[Tuple[int, float, float, float, float, float]]:
        assignments = {}
        used_tracks = set()

        for detection_index, box in enumerate(boxes):
            best_track_index = None
            best_iou = 0.0
            for track_index, track in enumerate(self.tracks):
                if track_index in used_tracks:
                    continue
                overlap = self.iou(box[:4], (track['x'], track['y'], track['w'], track['h']))
                if overlap > best_iou:
                    best_iou = overlap
                    best_track_index = track_index

            if best_track_index is not None and best_iou >= self.track_iou_threshold:
                assignments[detection_index] = best_track_index
                used_tracks.add(best_track_index)

        for track_index, track in enumerate(self.tracks):
            if track_index not in used_tracks:
                track['missed'] += 1

        tracked = []
        for detection_index, (x, y, w, h, score) in enumerate(boxes):
            track_index = assignments.get(detection_index)
            if track_index is None:
                track = {
                    'id': self.next_track_id,
                    'x': x,
                    'y': y,
                    'w': w,
                    'h': h,
                    'score': score,
                    'missed': 0,
                }
                self.next_track_id += 1
                self.tracks.append(track)
            else:
                track = self.tracks[track_index]
                track.update({'x': x, 'y': y, 'w': w, 'h': h, 'score': score, 'missed': 0})
            tracked.append((int(track['id']), x, y, w, h, score))

        self.tracks = [
            track for track in self.tracks
            if track['missed'] <= self.max_track_missed_frames
        ]
        return tracked

    def iou(self, first: Sequence[float], second: Sequence[float]) -> float:
        ax, ay, aw, ah = first
        bx, by, bw, bh = second
        intersection_x1 = max(ax, bx)
        intersection_y1 = max(ay, by)
        intersection_x2 = min(ax + aw, bx + bw)
        intersection_y2 = min(ay + ah, by + bh)
        intersection_w = max(0.0, intersection_x2 - intersection_x1)
        intersection_h = max(0.0, intersection_y2 - intersection_y1)
        intersection_area = intersection_w * intersection_h
        union_area = aw * ah + bw * bh - intersection_area
        if union_area <= 0.0:
            return 0.0
        return intersection_area / union_area

    def publish_result(self, boxes: List[Tuple[int, float, float, float, float, float]]) -> None:
        payload = [float(len(boxes))]
        for track_id, x, y, w, h, score in boxes:
            payload.extend([float(track_id), float(x), float(y), float(w), float(h), float(score)])
        self.boxes_pub.publish(Float32MultiArray(data=payload))

    def draw_debug_image(self, image: np.ndarray, boxes: List[Tuple[int, float, float, float, float, float]]) -> np.ndarray:
        output = image.copy()
        if self.draw_candidate_boxes:
            for class_id, x, y, w, h, score in self.latest_candidates:
                if class_id in self.person_class_ids and score >= self.confidence_threshold:
                    continue
                cv2.rectangle(
                    output,
                    (int(x), int(y)),
                    (int(x + w), int(y + h)),
                    (255, 180, 40),
                    1,
                )
                cv2.putText(
                    output,
                    f'c{class_id} {score:.2f}',
                    (int(x), max(14, int(y) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 180, 40),
                    1,
                    cv2.LINE_AA,
                )
        for track_id, x, y, w, h, score in boxes:
            start = (int(x), int(y))
            end = (int(x + w), int(y + h))
            cv2.rectangle(output, start, end, (48, 145, 255), 2)
            label = f'human #{track_id}: {score:.2f}'
            cv2.putText(
                output,
                label,
                (int(x), max(16, int(y) - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (48, 145, 255),
                2,
                cv2.LINE_AA,
            )
        return output


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HumanBoxTrackerNode()
        rclpy.spin(node)
    except Exception as error:
        logger = rclpy.logging.get_logger('human_box_tracker_node')
        logger.fatal(str(error))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
