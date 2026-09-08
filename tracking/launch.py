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
    commands=[
        ('camera',['ros2','launch','orbbec_camera','gemini_e.launch.py',
                   'enable_color:=true','color_width:=640','color_height:=360','color_fps:=5',
                   'enable_depth:=true','depth_width:=640','depth_height:=360','depth_fps:=5',
                   'enable_ir:=false','depth_registration:=true','align_mode:=HW','enable_point_cloud:=false']),
        ('thermal',['ros2','run','mi0802_senxor_driver','mi0802_senxor_node']),
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
