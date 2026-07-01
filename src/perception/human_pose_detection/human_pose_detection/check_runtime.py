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
    parser = argparse.ArgumentParser(description='Check human pose runtime dependencies.')
    parser.add_argument('--model', required=True, help='Path to the MoveNet Lightning INT8 .tflite file.')
    args = parser.parse_args()

    if not args.model or not os.path.isfile(os.path.expanduser(args.model)):
        print(
            'Missing MoveNet model file. Put the MoveNet Lightning INT8 .tflite at '
            '~/models/movenet_lightning_int8.tflite, or set MOVENET_MODEL_PATH / '
            'pose_model_path to the real file path.'
        )
        return 1

    missing = []
    for module in ('cv2', 'numpy', 'cv_bridge'):
        if not has_module(module):
            missing.append(module)

    if not any(has_module(module) for module in INTERPRETER_MODULES):
        missing.append('tflite_runtime or ai-edge-litert or tensorflow')

    if missing:
        print('Missing human pose runtime dependency: ' + ', '.join(missing))
        return 1

    print('Human pose runtime dependencies look OK.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
