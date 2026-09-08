"""Start only RGB, depth, thermal, gyro and the scene portal."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    logs=ROOT/'log'
    logs.mkdir(exist_ok=True)
    # Direct parameters avoid the legacy launch file silently dropping options.
    camera_parameters=(
        'camera_name:=camera','enable_color:=true','color_width:=640','color_height:=360',
        'color_fps:=15','color_format:=MJPG','enable_depth:=true','depth_width:=640',
        'depth_height:=360','depth_fps:=15','depth_format:=Y11','enable_ir:=false',
        'enable_accel:=false','enable_gyro:=false','depth_registration:=true',
        'align_mode:=HW','enable_frame_sync:=false','enable_point_cloud:=false')
    commands=[
        ('camera',['ros2','run','orbbec_camera','orbbec_camera_node','--ros-args',
                   '-r','__node:=camera','-r','__ns:=/camera']
                  +[arg for value in camera_parameters for arg in ('-p',value)]),
        ('thermal',['ros2','run','mi0802_senxor_driver','mi0802_senxor_node',
                    '--ros-args','-p','flip_vertical:=true']),
        ('portal',[sys.executable,'-m','tracking.server',*sys.argv[1:]])]
    processes=[]
    def stopped(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stopped)
    try:
        for name,command in commands:
            stream=open(logs/f'{name}.log','a',buffering=1)
            process=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append((name,process,stream))
            print(f'{name}: pid {process.pid}',flush=True)
        print('Portal address is in log/portal.log · Ctrl+C stops this stack',flush=True)
        while True:
            for name,process,_ in processes:
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited ({process.returncode}); see log/{name}.log')
            time.sleep(.5)
    except KeyboardInterrupt:
        pass
    finally:
        for _,process,_ in processes:
            try:os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        for _,process,stream in processes:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                process.wait()
            stream.close()


if __name__=='__main__':main()
