"""Windows FPGA 540p video sender. Run with Python 3.10+."""
import ipaddress
import json
import math
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from protocol import BOARD_IP, BOARD_MAC, WIDTH, HEIGHT, Sender, rgb565, wait_until
from media import parse_size, VideoFrames

COMMON_RESOLUTIONS = (
    '320x180', '320x240', '640x360', '640x480', '720x480',
    '720x576', '800x600', '854x480', '960x540', '1024x768',
    '1280x720', '1280x800', '1366x768', '1440x900',
    '1600x900', '1920x1080', '1920x1200', '2560x1440',
    '2560x1600', '3840x2160',
)


def check_network(local_ip):
    ip = ipaddress.IPv4Address(local_ip)
    if ip not in ipaddress.ip_network('192.168.1.0/24') or str(ip) in (
            '192.168.1.0', BOARD_IP, '192.168.1.255'):
        raise ValueError('本机需使用 192.168.1.x/24，不能与 FPGA 的 .10 冲突')
    # local_ip has been parsed as IPv4 before inclusion in this command.
    script = f"""
$ErrorActionPreference='Stop'
[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new()
$a=@(Get-NetIPAddress -AddressFamily IPv4 -IPAddress '{ip}')
if($a.Count -ne 1){{throw '找不到唯一的本机 IPv4 网卡'}}
$n=@(Get-NetNeighbor -InterfaceIndex $a[0].InterfaceIndex -IPAddress '{BOARD_IP}' -ErrorAction SilentlyContinue)
$link=Get-NetAdapter | Where-Object {{ $_.ifIndex -eq $a[0].InterfaceIndex }}
$mtu=Get-NetIPInterface -InterfaceIndex $a[0].InterfaceIndex -AddressFamily IPv4
[pscustomobject]@{{Prefix=$a[0].PrefixLength; Status=[string]$link.Status; Speed=$link.LinkSpeed; SpeedBps=$link.ReceiveLinkSpeed; Mtu=$mtu.NlMtu; Neighbors=@($n | Select-Object LinkLayerAddress,@{{n='State';e={{[string]$_.State}}}})}} | ConvertTo-Json -Depth 4 -Compress
"""
    result = subprocess.run(['powershell.exe', '-NoProfile', '-Command', script],
                            capture_output=True, timeout=15,
                            creationflags=subprocess.CREATE_NO_WINDOW)
    if result.returncode:
        raise RuntimeError('无法读取网卡配置。请检查本机 IP，必要时以管理员运行。\n' +
                           result.stderr.decode(errors='replace')[-600:])
    data = json.loads(result.stdout)
    if data['Prefix'] != 24 or data['Status'] != 'Up' or int(data['Mtu']) < 1228:
        raise ValueError('需要已连接的 /24 网卡，IPv4 MTU 至少 1228')
    if int(data.get('SpeedBps') or 0) < 1_000_000_000:
        raise ValueError('需要千兆链路；提供的 RGMII 接收代码未见百兆适配，请检查网线和 PHY 配置')
    if not any(n['LinkLayerAddress'].upper().replace(':', '-') == BOARD_MAC and
               n['State'] == 'Permanent' for n in data['Neighbors']):
        raise ValueError('未找到正确的静态 ARP。请先按 README 运行 configure_network.ps1')
    return f"网卡 {data['Speed']}，静态 ARP 正确（建议千兆直连）"


def fit_frame(frame, cv2, np, size=(WIDTH, HEIGHT), mode='等比留黑'):
    width, height = size
    h, w = frame.shape[:2]
    if (w, h) == size:
        return frame
    if mode == '拉伸填满':
        return cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
    scale = (max if mode == '居中裁剪' else min)(width / w, height / h)
    nw, nh = max(1, round(w * scale)), max(1, round(h * scale))
    if mode == '居中裁剪':
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
        x, y = (nw - width) // 2, (nh - height) // 2
        return resized[y:y + height, x:x + width].copy()
    out = np.zeros((height, width, 3), dtype=np.uint8)
    x, y = (width - nw) // 2, (height - nh) // 2
    out[y:y + nh, x:x + nw] = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
    return out


def color_bars(np):
    colors = [(255,255,255), (0,255,255), (255,255,0), (0,255,0),
              (255,0,255), (0,0,255), (255,0,0), (0,0,0)]
    out = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    for i, color in enumerate(colors):
        out[:, i * 120:(i + 1) * 120] = color
    return out


def wait_for_frame(started, fps, stop, clock=time.perf_counter, precise_wait=wait_until):
    """Meet the output frame deadline without a Windows timer tick overshoot."""
    deadline = started + 1 / fps
    while not stop.is_set():
        remaining = deadline - clock()
        if remaining <= 0:
            return
        if remaining > 0.05:
            # Long delays remain interruptible. Leave room for a precise finish.
            stop.wait(remaining - 0.02)
        else:
            precise_wait(deadline)
            return


class App:
    def __init__(self, root):
        self.root = root
        root.title('FPGA 视频 / 图片发送器')
        root.geometry('940x860')
        self.events = queue.Queue()
        self.preview = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.worker = None
        self.closing = False
        self.path = tk.StringVar()
        self.local = tk.StringVar(value='192.168.1.102')
        self.fps = tk.StringVar(value='10')
        self.rate = tk.StringVar(value='120')
        self.source = tk.StringVar(value='彩条测试')
        self.loop = tk.BooleanVar(value=True)
        self.input_size = tk.StringVar(value='原始尺寸')
        self.input_fps = tk.StringVar(value='30')
        self.output_size = tk.StringVar(value='960x540')
        self.mode = tk.StringVar(value='等比留黑')
        self.info = tk.StringVar(value='选择文件后显示原始分辨率和帧率；图片按设定发送帧率重复。')
        self.status = tk.StringVar(value='未发送。请先配置静态 ARP，并确认 FPGA 已复位、DDR 已初始化。')
        body = ttk.Frame(root, padding=16)
        body.pack(fill='both', expand=True)
        ttk.Label(body, text='视频 / 图片 → RGB565 → FPGA', font=('', 18, 'bold')).pack(anchor='w')
        ttk.Label(body, text=f'目标固定：{BOARD_IP}:1234  /  {BOARD_MAC}').pack(anchor='w', pady=8)
        row = ttk.Frame(body); row.pack(fill='x')
        for title, var, width in [('本机 IPv4', self.local, 18), ('限速 Mbps', self.rate, 6)]:
            ttk.Label(row, text=title).pack(side='left', padx=(0, 6))
            ttk.Entry(row, textvariable=var, width=width).pack(side='left', padx=(0, 14))
        row = ttk.Frame(body); row.pack(fill='x', pady=12)
        ttk.Combobox(row, textvariable=self.source, values=['彩条测试', '视频文件', '图片文件'], state='readonly', width=12).pack(side='left')
        ttk.Entry(row, textvariable=self.path).pack(side='left', fill='x', expand=True, padx=8)
        ttk.Button(row, text='选择视频 / 图片', command=self.choose).pack(side='left')
        ttk.Label(body, textvariable=self.info, wraplength=890).pack(anchor='w')
        box = ttk.LabelFrame(body, text='输入处理（不会修改原文件）', padding=8); box.pack(fill='x', pady=8)
        ttk.Label(box, text='输入分辨率').pack(side='left')
        ttk.Combobox(box, textvariable=self.input_size,
                     values=('原始尺寸',) + COMMON_RESOLUTIONS, width=13).pack(side='left', padx=(8, 2))
        ttk.Button(box, text='自定义…', command=lambda: self.custom_resolution(self.input_size)).pack(side='left', padx=(0, 8))
        ttk.Label(box, text='输入采样 fps').pack(side='left')
        ttk.Combobox(box, textvariable=self.input_fps, values=['5','10','15','24','25','30','60'], width=7).pack(side='left', padx=8)
        ttk.Combobox(box, textvariable=self.mode, values=['等比留黑','居中裁剪','拉伸填满'], state='readonly', width=12).pack(side='left')
        box = ttk.LabelFrame(body, text='发送设置', padding=8); box.pack(fill='x', pady=4)
        ttk.Label(box, text='输出分辨率（实际发送）').pack(side='left')
        ttk.Combobox(box, textvariable=self.output_size,
                     values=COMMON_RESOLUTIONS, width=13).pack(side='left', padx=(8, 2))
        ttk.Button(box, text='自定义…', command=lambda: self.custom_resolution(self.output_size)).pack(side='left', padx=(0, 8))
        ttk.Label(box, text='发送 fps').pack(side='left')
        ttk.Combobox(box, textvariable=self.fps, values=['1','5','10','15','24','25','30','60'], width=7).pack(side='left', padx=8)
        ttk.Label(body, text='宽×高、输入和输出 fps 均可手动填写；输出尺寸会改变帧头与像素数量，FPGA 必须匹配。').pack(anchor='w')
        ttk.Checkbutton(body, text='循环视频 / 连续发送图片（取消后图片只发一帧）', variable=self.loop).pack(anchor='w')
        row = ttk.Frame(body); row.pack(fill='x', pady=12)
        self.start_button = ttk.Button(row, text='开始发送', command=self.start)
        self.start_button.pack(side='left')
        self.stop_button = ttk.Button(row, text='发完当前帧后停止', command=self.request_stop, state='disabled')
        self.stop_button.pack(side='left', padx=10)
        ttk.Label(body, textvariable=self.status, wraplength=750).pack(anchor='w', pady=8)
        self.canvas = ttk.Label(body, text='发送帧预览（不是 FPGA 回传）', anchor='center')
        self.canvas.pack(fill='both', expand=True)
        ttk.Label(body, text='限速或解码跟不上时自动降低播放速度，不突发补发。网络丢包无法由现有 FPGA 协议恢复。', wraplength=750).pack(anchor='w', pady=8)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(100, self.poll)

    def custom_resolution(self, variable):
        current = variable.get()
        answer = simpledialog.askstring(
            '自定义分辨率', '请输入宽x高，例如 1024x600：',
            initialvalue=current if current != '原始尺寸' else '', parent=self.root)
        if answer is None:
            return
        try:
            size = parse_size(answer.strip())
            if size is None:
                raise ValueError('请填写具体的宽和高')
        except ValueError as e:
            messagebox.showerror('分辨率错误', str(e), parent=self.root)
            return
        variable.set(f'{size[0]}x{size[1]}')

    def choose(self):
        path = filedialog.askopenfilename(filetypes=[('视频和图片', '*.mp4 *.avi *.mkv *.mov *.wmv *.png *.jpg *.jpeg *.bmp *.tif *.tiff *.webp'), ('所有文件', '*.*')])
        if path:
            self.path.set(path)
            is_image = Path(path).suffix.lower() in ('.png','.jpg','.jpeg','.bmp','.tif','.tiff','.webp')
            self.source.set('图片文件' if is_image else '视频文件')
            try:
                import cv2
                import numpy as np
                if is_image:
                    frame = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None: raise ValueError('无法解码图片')
                    self.info.set(f'原始图片：{frame.shape[1]}×{frame.shape[0]}；静态图片没有原生帧率。')
                else:
                    cap = cv2.VideoCapture(path)
                    try:
                        if not cap.isOpened(): raise ValueError('无法打开视频')
                        native = cap.get(cv2.CAP_PROP_FPS)
                        self.info.set(f'原始视频：{int(cap.get(3))}×{int(cap.get(4))}，{native:.3f} fps')
                        if math.isfinite(native) and native > 0: self.input_fps.set(f'{native:.3f}')
                    finally: cap.release()
            except Exception as e:
                self.info.set(f'读取媒体信息失败：{e}')

    def start(self):
        if self.worker and self.worker.is_alive():
            return
        try:
            fps, rate = float(self.fps.get()), float(self.rate.get())
            infps = float(self.input_fps.get())
            if not all(math.isfinite(v) and v > 0 for v in (fps, infps)):
                raise ValueError('输入和输出帧率必须是大于 0 的有限数字，可填写 29.97 等小数')
            if not math.isfinite(rate) or not 1 <= rate <= 950:
                raise ValueError('限速范围 1–950 Mbps')
            insize, outsize = map(parse_size, (self.input_size.get(), self.output_size.get()))
            if outsize is None: raise ValueError('输出需填写具体分辨率')
            config = (self.local.get().strip(), fps, rate, self.source.get(), self.path.get(), self.loop.get())
            if config[3] != '彩条测试' and not Path(config[4]).is_file():
                raise ValueError('请先选择有效的视频或图片文件')
            config += (insize, infps, outsize, self.mode.get())
        except ValueError as e:
            messagebox.showerror('参数错误', str(e)); return
        self.stop.clear()
        self.start_button.configure(state='disabled')
        self.stop_button.configure(state='normal')
        self.status.set('正在检查依赖和网络配置…')
        self.worker = threading.Thread(target=self.run, args=(config,), daemon=False)
        self.worker.start()

    def request_stop(self):
        self.stop.set()
        self.status.set('正在发完当前帧，请勿断开网线或关闭进程…')

    def run(self, config):
        sender = cap = executor = None
        transmission_started = False
        try:
            try:
                import cv2
                import numpy as np
            except ImportError as e:
                self.events.put(('setup_error',
                    f'视频依赖加载失败：{e}\n当前 Python：{sys.executable}\n'
                    '请关闭程序，双击同目录的 start.cmd，使用已安装依赖的项目环境。\n'
                    '尚未发送任何数据，无需因此复位 FPGA。'))
                return
            local, fps, rate, source, path, loop = config[:6]
            insize, infps, outsize, mode = config[6:] if len(config) > 6 else (None,30,(960,540),'等比留黑')
            info = check_network(local)
            self.events.put(('status', info))
            if source == '视频文件':
                cap = VideoFrames(path, cv2, infps, fps, loop)
            still = None
            if source == '图片文件':
                still = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
                if still is None: raise ValueError('无法解码图片')
            sender = Sender(local, mbps=rate, size=outsize)
            def prepare_next():
                prep_started = time.perf_counter()
                if cap is None:
                    frame = still if still is not None else color_bars(np)
                else:
                    frame = cap.read(self.stop)
                    if frame is None:
                        return None
                if insize:
                    frame = fit_frame(frame, cv2, np, insize, mode)
                frame = fit_frame(frame, cv2, np, outsize, mode)
                payload = rgb565(frame, outsize)
                thumb = cv2.cvtColor(fit_frame(frame, cv2, np, (640,360)), cv2.COLOR_BGR2RGB)
                preview = b'P6\n640 360\n255\n' + thumb.tobytes()
                return payload, preview, (time.perf_counter() - prep_started) * 1000

            # One decoder and one sender. Buffer at most one prepared frame;
            # frames still go to the UDP socket in their original order.
            executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='video_decode')
            future = executor.submit(prepare_next)
            count, began = 0, time.perf_counter()
            while not self.stop.is_set():
                ready_started = time.perf_counter()
                prepared_frame = future.result()
                if prepared_frame is None:
                    break
                payload, preview, prep_ms = prepared_frame
                wait_ms = (time.perf_counter() - ready_started) * 1000
                if self.stop.is_set():
                    break
                next_future = None
                if source != '图片文件' or loop:
                    next_future = executor.submit(prepare_next)
                started = time.perf_counter()
                transmission_started = True
                sender.send_frame(payload)
                sent_at = time.perf_counter()
                count += 1
                try:
                    self.preview.put_nowait(preview)
                except queue.Full:
                    pass
                elapsed = time.perf_counter() - began
                packet_count = 1 + math.ceil(outsize[0]*outsize[1]*2/1200)
                send_ms = (sent_at - started) * 1000
                self.events.put(('status',
                    f'{outsize[0]}×{outsize[1]}，目标 {fps:g} fps / 实际平均 {count / elapsed:.2f} fps；'
                    f'本帧解码/转换 {prep_ms:.1f} ms、等待画面 {wait_ms:.1f} ms、发包 {send_ms:.1f} ms；'
                    f'已提交 {count} 帧 / {count * packet_count} 包。'))
                if next_future is None:
                    break
                wait_for_frame(started, fps, self.stop)
                future = next_future
            self.events.put(('status', f'已在完整帧边界停止，共提交 {count} 帧。'))
        except Exception as e:
            if transmission_started:
                self.events.put(('error', f'{e}\n发送已开始，请复位 FPGA 再重试；程序不会自动补发或重连。'))
            else:
                self.events.put(('setup_error', f'{e}\n尚未发送任何数据，无需因此复位 FPGA。'))
        finally:
            if executor is not None:
                executor.shutdown(wait=True, cancel_futures=True)
            if cap is not None: cap.close()
            if sender is not None: sender.close()
            self.events.put(('done', None))

    def poll(self):
        try:
            while True:
                kind, text = self.events.get_nowait()
                if kind in ('status', 'error', 'setup_error'):
                    self.status.set(text)
                if kind == 'error' and not self.closing:
                    messagebox.showerror('发送失败', text)
                if kind == 'setup_error' and not self.closing:
                    messagebox.showerror('启动检查未通过', text)
                if kind == 'done':
                    self.start_button.configure(state='normal')
                    self.stop_button.configure(state='disabled')
        except queue.Empty:
            pass
        try:
            self.photo = tk.PhotoImage(data=self.preview.get_nowait(), format='PPM')
            self.canvas.configure(image=self.photo, text='')
        except queue.Empty:
            pass
        if self.closing and (not self.worker or not self.worker.is_alive()):
            self.root.destroy(); return
        self.root.after(100, self.poll)

    def close(self):
        self.closing = True
        self.request_stop()


if __name__ == '__main__':
    root = tk.Tk()
    App(root)
    root.mainloop()
