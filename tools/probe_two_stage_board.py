"""Read-only target probe; password from temporary environment or stdin, never saved."""
import json,os,sys,paramiko

def main():
    secret=os.environ.get('ADAS_BOARD_PASSWORD') or sys.stdin.readline().rstrip('\r\n')
    client=paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect('192.168.137.17',username='cat',password=secret,timeout=8,
                       auth_timeout=8,banner_timeout=8,look_for_keys=False,allow_agent=False)
        secret=None
        command="uname -m; cat /proc/device-tree/compatible; echo; python3 --version; ls -l /usr/lib/librknnrt.so /usr/lib/aarch64-linux-gnu/librknnrt.so 2>/dev/null; python3 -c 'import importlib.util; print({m:importlib.util.find_spec(m) is not None for m in [\"rknn\",\"rknnlite\",\"cv2\"]})'; pgrep -a -x adas_four_view"
        _,stdout,stderr=client.exec_command(command,timeout=15)
        print(stdout.read().decode(errors='replace'))
        print(stderr.read().decode(errors='replace'))
    except Exception as error:
        print(json.dumps(dict(error_type=type(error).__name__,message=str(error))))
        raise SystemExit(1)
    finally:client.close()

if __name__=='__main__':main()
