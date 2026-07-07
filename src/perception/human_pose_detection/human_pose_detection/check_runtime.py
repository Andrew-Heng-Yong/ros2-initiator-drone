import argparse
import importlib
import os
import sys


INTERPRETER_MODULES = (
    'tflite_runtime.interpreter',
    'ai_edge_litert.interpreter',
    'tensorflow',
)


def has_module(name):
    try:
        importlib.import_module(name)
        return True
    except ImportError:
        return False


def main():
    parser = argparse.ArgumentParser(description='Check human tracking runtime dependencies.')
    parser.add_argument('--model', required=True, help='Path to the TensorFlow Lite person-box .tflite file.')
    args = parser.parse_args()

    if not args.model or not os.path.isfile(os.path.expanduser(args.model)):
        print(
            'Missing human box model file. Put the EfficientDet Lite0 .tflite at '
            '~/ros2-initiator-drone/models/efficientdet_lite0.tflite, set '
            'HUMAN_BOX_MODEL_PATH, or update human_box_tracker.yaml to the real file path.'
        )
        return 1

    missing = []
    for module in ('cv2', 'numpy', 'cv_bridge'):
        if not has_module(module):
            missing.append(module)

    if not any(has_module(module) for module in INTERPRETER_MODULES):
        missing.append('tflite_runtime or ai-edge-litert or tensorflow')

    if missing:
        print('Missing human tracking runtime dependency: ' + ', '.join(missing))
        return 1

    print('Human tracking runtime dependencies look OK.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
