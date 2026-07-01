import os
from typing import List, Optional, Tuple

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


KEYPOINT_NAMES = [
    'nose',
    'left_eye',
    'right_eye',
    'left_ear',
    'right_ear',
    'left_shoulder',
    'right_shoulder',
    'left_elbow',
    'right_elbow',
    'left_wrist',
    'right_wrist',
    'left_hip',
    'right_hip',
    'left_knee',
    'right_knee',
    'left_ankle',
    'right_ankle',
]

SKELETON_EDGES = [
    (0, 1), (0, 2), (1, 3), (2, 4),
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
]


class MovenetPoseNode(Node):
    def __init__(self):
        super().__init__('movenet_pose_node')
        self.bridge = CvBridge()

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('keypoints_topic', '/human_pose/keypoints')
        self.declare_parameter('person_detected_topic', '/human_pose/person_detected')
        self.declare_parameter('debug_image_topic', '/human_pose/debug_image')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('min_confident_keypoints', 5)
        self.declare_parameter('max_inference_fps', 5.0)
        self.declare_parameter('publish_debug_image', True)

        self.model_path = os.path.expanduser(self.get_parameter('model_path').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.min_confident_keypoints = int(self.get_parameter('min_confident_keypoints').value)
        self.max_inference_fps = float(self.get_parameter('max_inference_fps').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.last_inference_time = None

        self._load_model()

        self.keypoints_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('keypoints_topic').value,
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
            f'Running MoveNet Lightning INT8 on {self.image_topic}; '
            f'keypoint threshold={self.confidence_threshold:.2f}; '
            f'max inference FPS={self.max_inference_fps:.1f}'
        )

    def _load_model(self) -> None:
        if Interpreter is None:
            raise RuntimeError(
                'Install a TensorFlow Lite interpreter: tflite_runtime, ai-edge-litert, or tensorflow.'
            )
        if not self.model_path:
            raise RuntimeError('model_path parameter is required.')
        if not os.path.isfile(self.model_path):
            raise RuntimeError(f'MoveNet model file not found: {self.model_path}')

        self.interpreter = Interpreter(model_path=self.model_path, num_threads=2)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()[0]
        self.output_details = self.interpreter.get_output_details()[0]

        input_shape = self.input_details['shape']
        if len(input_shape) != 4 or int(input_shape[3]) != 3:
            raise RuntimeError(f'Unsupported MoveNet input shape: {input_shape}')
        self.input_height = int(input_shape[1])
        self.input_width = int(input_shape[2])
        self.input_dtype = self.input_details['dtype']
        self.get_logger().info(
            f'Loaded {self.model_path} with input {self.input_width}x{self.input_height} '
            f'{self.input_dtype}'
        )

    def image_callback(self, msg: Image) -> None:
        if not self.should_process_frame():
            return

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().warn(f'Unsupported camera image format: {error}')
            return

        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        try:
            keypoints, scale, pad_x, pad_y = self.run_inference(rgb_image)
            decoded = self.decode_keypoints(keypoints, msg.width, msg.height, scale, pad_x, pad_y)
            bbox = self.bounding_box(decoded)
            detected = bbox is not None
        except Exception as error:
            self.get_logger().warn(f'Pose inference failed for this frame: {error}')
            return

        self.publish_result(decoded, bbox, detected)
        self.detected_pub.publish(Bool(data=detected))

        if self.publish_debug_image:
            debug_image = self.draw_debug_image(bgr_image, decoded, bbox)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))

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

    def run_inference(self, rgb_image: np.ndarray) -> Tuple[np.ndarray, float, int, int]:
        # MoveNet Lightning expects a square RGB tensor. Letterboxing preserves
        # the camera aspect ratio, which usually gives steadier keypoints than
        # stretching the whole frame into a square.
        input_image, image_scale, pad_x, pad_y = self.letterbox(rgb_image)
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
        output_tensor = self.interpreter.get_tensor(self.output_details['index'])
        if not np.issubdtype(output_tensor.dtype, np.floating):
            output_scale, output_zero_point = self.output_details.get('quantization', (0.0, 0))
            if output_scale and output_scale > 0:
                output_tensor = (output_tensor.astype(np.float32) - output_zero_point) * output_scale
        return np.squeeze(output_tensor), image_scale, pad_x, pad_y

    def letterbox(self, rgb_image: np.ndarray) -> Tuple[np.ndarray, float, int, int]:
        height, width = rgb_image.shape[:2]
        scale = min(self.input_width / float(width), self.input_height / float(height))
        resized_width = max(1, int(round(width * scale)))
        resized_height = max(1, int(round(height * scale)))
        pad_x = (self.input_width - resized_width) // 2
        pad_y = (self.input_height - resized_height) // 2

        resized = cv2.resize(rgb_image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
        canvas[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized
        return canvas, scale, pad_x, pad_y

    def decode_keypoints(
        self,
        raw_keypoints: np.ndarray,
        width: int,
        height: int,
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> List[Tuple[float, float, float]]:
        # MoveNet returns 17 keypoints as normalized [y, x, score].
        if scale <= 0:
            self.get_logger().warn('Invalid letterbox scale; skipping keypoint decode for this frame.')
            return [(-1.0, -1.0, 0.0)] * 17

        keypoints = raw_keypoints.reshape(17, 3)
        decoded = []
        for y_norm, x_norm, score in keypoints:
            model_x = float(x_norm) * float(self.input_width)
            model_y = float(y_norm) * float(self.input_height)
            x = (model_x - pad_x) / scale
            y = (model_y - pad_y) / scale
            x = max(0.0, min(float(width - 1), x))
            y = max(0.0, min(float(height - 1), y))
            decoded.append((x, y, float(score)))
        return decoded

    def bounding_box(self, keypoints: List[Tuple[float, float, float]]) -> Optional[Tuple[float, float, float, float]]:
        confident = [(x, y) for x, y, score in keypoints if score >= self.confidence_threshold]
        if len(confident) < self.min_confident_keypoints:
            return None

        xs = [point[0] for point in confident]
        ys = [point[1] for point in confident]
        x_min = max(0.0, min(xs))
        y_min = max(0.0, min(ys))
        x_max = max(xs)
        y_max = max(ys)
        return (x_min, y_min, x_max - x_min, y_max - y_min)

    def publish_result(
        self,
        keypoints: List[Tuple[float, float, float]],
        bbox: Optional[Tuple[float, float, float, float]],
        detected: bool,
    ) -> None:
        if bbox is None:
            bbox_values = [-1.0, -1.0, -1.0, -1.0]
        else:
            bbox_values = [float(value) for value in bbox]

        payload = [1.0 if detected else 0.0, *bbox_values]
        for x, y, score in keypoints:
            if score < self.confidence_threshold:
                payload.extend([-1.0, -1.0, float(score)])
            else:
                payload.extend([float(x), float(y), float(score)])
        self.keypoints_pub.publish(Float32MultiArray(data=payload))

    def draw_debug_image(
        self,
        image: np.ndarray,
        keypoints: List[Tuple[float, float, float]],
        bbox: Optional[Tuple[float, float, float, float]],
    ) -> np.ndarray:
        output = image.copy()

        for start, end in SKELETON_EDGES:
            x1, y1, s1 = keypoints[start]
            x2, y2, s2 = keypoints[end]
            if s1 >= self.confidence_threshold and s2 >= self.confidence_threshold:
                cv2.line(output, (int(x1), int(y1)), (int(x2), int(y2)), (64, 220, 255), 2)

        for index, (x, y, score) in enumerate(keypoints):
            if score >= self.confidence_threshold:
                cv2.circle(output, (int(x), int(y)), 4, (44, 255, 120), -1)
                cv2.putText(
                    output,
                    KEYPOINT_NAMES[index],
                    (int(x) + 5, int(y) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(output, (int(x), int(y)), (int(x + w), int(y + h)), (48, 145, 255), 2)
        return output


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MovenetPoseNode()
        rclpy.spin(node)
    except Exception as error:
        logger = rclpy.logging.get_logger('movenet_pose_node')
        logger.fatal(str(error))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
