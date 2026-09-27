"""Windows FPGA 540p video sender. Run with Python 3.10+."""
import ipaddress
import json
import math
import re
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from protocol import BOARD_IP, BOARD_MAC, WIDTH, HEIGHT, Sender, rgb565, wait_until
from media import parse_size, VideoFrames


def network_parameters(local_ip, destination, mac):
    addresses = []
    for label, value in [('本机', local_ip), ('目标', destination)]:
        try:
            ip = ipaddress.IPv4Address(value.strip())
        except ValueError:
            raise ValueError(f'{label} IPv4 格式不正确') from None
        if ip.is_unspecified or ip.is_multicast or ip.is_loopback or ip.is_reserved:
            raise ValueError(f'{label} IPv4 必须是网卡可使用的单播地址')
        addresses.append(str(ip))
    if addresses[0] == addresses[1]:
        raise ValueError('本机 IP 和目标 IP 不能相同')
    mac = mac.strip().upper()
    if not re.fullmatch(r'(?:[0-9A-F]{12}|(?:[0-9A-F]{2}:){5}[0-9A-F]{2}|(?:[0-9A-F]{2}-){5}[0-9A-F]{2})', mac):
        raise ValueError('目标 MAC 格式应为 A0-B1-C2-D3-E4-E4 或 A0:B1:C2:D3:E4:E4')
    compact = mac.replace(':', '').replace('-', '')
    if int(compact[:2], 16) & 1 or int(compact, 16) == 0:
        raise ValueError('目标 MAC 必须是非零单播 MAC 地址')
    return (*addresses, '-'.join(compact[i:i+2] for i in range(0, 12, 2)))


def list_local_ips():
    script = """
$ErrorActionPreference='Stop'
ConvertTo-Json -InputObject @(Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.AddressState -eq 'Preferred' -and $_.IPAddress -ne '127.0.0.1' } |
    Select-Object -ExpandProperty IPAddress -Unique) -Compress
"""
    result = subprocess.run(['powershell.exe', '-NoProfile', '-Command', script],
                            capture_output=True, timeout=15,
                            creationflags=subprocess.CREATE_NO_WINDOW)
    if result.returncode:
        raise RuntimeError('读取本机 IP 失败，仍可手动填写网卡 IP')
    return sorted(json.loads(result.stdout), key=ipaddress.IPv4Address)


def check_network(local_ip, destination=BOARD_IP, mac=BOARD_MAC, chunk_bytes=1200):
    local_ip, destination, mac = network_parameters(local_ip, destination, mac)
    ip = ipaddress.IPv4Address(local_ip)
    # Only parsed IP literals enter the PowerShell command.
    script = f"""
$ErrorActionPreference='Stop'
[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new()
$a=@(Get-NetIPAddress -AddressFamily IPv4 -IPAddress '{ip}')
if($a.Count -ne 1){{throw '找不到唯一的本机 IPv4 网卡'}}
$n=@(Get-NetNeighbor -InterfaceIndex $a[0].InterfaceIndex -IPAddress '{destination}' -ErrorAction SilentlyContinue)
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
    subnet = ipaddress.IPv4Network(f'{ip}/{int(data["Prefix"])}', strict=False)
    target = ipaddress.IPv4Address(destination)
    if target not in subnet or (subnet.prefixlen < 31 and
            any(addr in (subnet.network_address, subnet.broadcast_address) for addr in (ip, target))):
        raise ValueError(f'本机与目标必须是网卡实际子网 {subnet} 内的主机地址；当前仅支持同网段直连')
    if data['Status'] != 'Up' or int(data['Mtu']) < chunk_bytes + 28:
        raise ValueError(f'需要已连接的网卡，当前负载要求 IPv4 MTU 至少 {chunk_bytes + 28}')
    if int(data.get('SpeedBps') or 0) < 1_000_000_000:
        raise ValueError('需要千兆链路；提供的 RGMII 接收代码未见百兆适配，请检查网线和 PHY 配置')
    if not any(n['LinkLayerAddress'].upper().replace(':', '-') == mac and
               n['State'] == 'Permanent' for n in data['Neighbors']):
        raise ValueError('未找到匹配目标 IP/MAC 的静态 ARP。请在程序目录的管理员 PowerShell 执行：\n'
                         f'.\\configure_network.ps1 -LocalIP {local_ip} -TargetIP {destination} -TargetMAC {mac}')
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
        self.destination = tk.StringVar(value=BOARD_IP)
        self.target_mac = tk.StringVar(value=BOARD_MAC)
        self.fps = tk.StringVar(value='10')
        self.rate = tk.StringVar(value='120')
        self.source = tk.StringVar(value='彩条测试')
        self.loop = tk.BooleanVar(value=True)
        self.input_size = tk.StringVar(value='原始尺寸')
        self.input_fps = tk.StringVar(value='30')
        self.output_size = tk.StringVar(value='960x540')
        self.vs_blank_ms = tk.StringVar(value='2.0')
        self.chunk_bytes = tk.StringVar(value='256')
        self.packet_gap_us = tk.StringVar(value='20')
        self.mode = tk.StringVar(value='等比留黑')
        self.info = tk.StringVar(value='选择文件后显示原始分辨率和帧率；图片按设定发送帧率重复。')
        self.status = tk.StringVar(value='未发送。请先配置静态 ARP，并确认 FPGA 已复位、DDR 已初始化。')
        body = ttk.Frame(root, padding=16)
        body.pack(fill='both', expand=True)
        ttk.Label(body, text='视频 / 图片 → RGB565 → FPGA', font=('', 18, 'bold')).pack(anchor='w')
        network = ttk.LabelFrame(body, text='网络设置（目标 UDP 端口 1234）', padding=8)
        network.pack(fill='x', pady=8)
        row = ttk.Frame(network); row.pack(fill='x')
        ttk.Label(row, text='本机 IPv4').pack(side='left')
        self.local_combo = ttk.Combobox(row, textvariable=self.local, width=18)
        self.local_combo.pack(side='left', padx=8)
        self.refresh_button = ttk.Button(row, text='刷新本机 IP', command=self.refresh_ips)
        self.refresh_button.pack(side='left', padx=(0, 14))
        ttk.Label(row, text='目标 IPv4').pack(side='left')
        ttk.Entry(row, textvariable=self.destination, width=18).pack(side='left', padx=8)
        row = ttk.Frame(network); row.pack(fill='x', pady=(8, 0))
        ttk.Label(row, text='目标 MAC').pack(side='left')
        ttk.Entry(row, textvariable=self.target_mac, width=23).pack(side='left', padx=8)
        ttk.Label(row, text='IP 可选择或手填；目标 IP/MAC 必须匹配 FPGA。').pack(side='left')
        row = ttk.Frame(body); row.pack(fill='x')
        for title, var, width in [('限速 Mbps', self.rate, 6)]:
            ttk.Label(row, text=title).pack(side='left', padx=(0, 6))
            ttk.Entry(row, textvariable=var, width=width).pack(side='left', padx=(0, 14))
        row = ttk.Frame(body); row.pack(fill='x', pady=12)
        ttk.Combobox(row, textvariable=self.source, values=['彩条测试', '视频文件', '图片文件'], state='readonly', width=12).pack(side='left')
        ttk.Entry(row, textvariable=self.path).pack(side='left', fill='x', expand=True, padx=8)
        ttk.Button(row, text='选择视频 / 图片', command=self.choose).pack(side='left')
        ttk.Label(body, textvariable=self.info, wraplength=890).pack(anchor='w')
        sizes = ['320x180', '640x360', '640x480', '960x540', '1280x720', '1920x1080']
        box = ttk.LabelFrame(body, text='输入处理（不会修改原文件）', padding=8); box.pack(fill='x', pady=8)
        ttk.Label(box, text='输入分辨率').pack(side='left')
        ttk.Combobox(box, textvariable=self.input_size, values=['原始尺寸'] + sizes, width=16).pack(side='left', padx=8)
        ttk.Label(box, text='输入采样 fps').pack(side='left')
        ttk.Combobox(box, textvariable=self.input_fps, values=['5','10','15','24','25','30','60'], width=7).pack(side='left', padx=8)
        ttk.Combobox(box, textvariable=self.mode, values=['等比留黑','居中裁剪','拉伸填满'], state='readonly', width=12).pack(side='left')
        box = ttk.LabelFrame(body, text='发送设置', padding=8); box.pack(fill='x', pady=4)
        ttk.Label(box, text='输出分辨率（实际发送）').pack(side='left')
        ttk.Combobox(box, textvariable=self.output_size, values=sizes, width=13).pack(side='left', padx=8)
        ttk.Label(box, text='发送 fps').pack(side='left')
        ttk.Combobox(box, textvariable=self.fps, values=['1','5','10','15','24','25','30','60'], width=7).pack(side='left', padx=8)
        ttk.Label(box, text='VS 消隐 ms').pack(side='left')
        ttk.Combobox(box, textvariable=self.vs_blank_ms,
                     values=['0.5','0.67','1.0','1.33','1.67','2.0','3.33'], width=7).pack(side='left', padx=8)
        box = ttk.LabelFrame(body, text='UDP 平滑发送（降低 FPGA 写 FIFO 瞬时压力）', padding=8); box.pack(fill='x', pady=4)
        ttk.Label(box, text='像素负载 bytes').pack(side='left')
        ttk.Combobox(box, textvariable=self.chunk_bytes,
                     values=['64','128','256','480','800','1200'], width=7).pack(side='left', padx=8)
        ttk.Label(box, text='最小包间隔 us').pack(side='left')
        ttk.Combobox(box, textvariable=self.packet_gap_us,
                     values=['0','10','20','50','100','200'], width=7).pack(side='left', padx=8)
        ttk.Label(box, text='小包和大间隔更平滑，但最高帧率更低。').pack(side='left', padx=8)
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
        self.refresh_ips()

    def refresh_ips(self):
        self.refresh_button.configure(state='disabled')
        def discover():
            try:
                self.events.put(('local_ips', list_local_ips()))
            except Exception as e:
                self.events.put(('local_ips_error', str(e)))
        threading.Thread(target=discover, daemon=True).start()

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
            local, destination, mac = network_parameters(
                self.local.get(), self.destination.get(), self.target_mac.get())
            fps, rate = float(self.fps.get()), float(self.rate.get())
            infps = float(self.input_fps.get())
            vs_blank_ms = float(self.vs_blank_ms.get())
            chunk_bytes = int(self.chunk_bytes.get())
            packet_gap_us = float(self.packet_gap_us.get())
            if not all(math.isfinite(v) and v > 0 for v in (fps, infps)):
                raise ValueError('输入和输出帧率必须是大于 0 的有限数字，可填写 29.97 等小数')
            if not math.isfinite(rate) or not 1 <= rate <= 950:
                raise ValueError('限速范围 1–950 Mbps')
            if not math.isfinite(vs_blank_ms) or not 0.01 <= vs_blank_ms <= 1000:
                raise ValueError('VS 消隐时间范围为 0.01–1000 ms')
            if not 4 <= chunk_bytes <= 1472 or chunk_bytes % 4:
                raise ValueError('UDP 像素负载必须是 4–1472 范围内的 4 字节整数倍')
            if not math.isfinite(packet_gap_us) or not 0 <= packet_gap_us <= 100000:
                raise ValueError('最小包间隔范围为 0–100000 us')
            insize, outsize = map(parse_size, (self.input_size.get(), self.output_size.get()))
            if outsize is None: raise ValueError('输出需填写具体分辨率')
            config = (local, fps, rate, self.source.get(), self.path.get(), self.loop.get())
            if config[3] != '彩条测试' and not Path(config[4]).is_file():
                raise ValueError('请先选择有效的视频或图片文件')
            config += (insize, infps, outsize, self.mode.get(), vs_blank_ms,
                       chunk_bytes, packet_gap_us, destination, mac)
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
            if len(config) >= 13:
                insize, infps, outsize, mode, vs_blank_ms, chunk_bytes, packet_gap_us = config[6:13]
            elif len(config) >= 11:
                insize, infps, outsize, mode, vs_blank_ms = config[6:11]
                chunk_bytes, packet_gap_us = 256, 20.0
            elif len(config) >= 10:
                # Keep compatibility with callers saved before VS blanking was configurable.
                insize, infps, outsize, mode = config[6:10]
                vs_blank_ms = 2.0
                chunk_bytes, packet_gap_us = 256, 20.0
            else:
                insize, infps, outsize, mode, vs_blank_ms, chunk_bytes, packet_gap_us = (
                    None, 30, (960,540), '等比留黑', 2.0, 256, 20.0)
            destination, mac = config[13:15] if len(config) >= 15 else (BOARD_IP, BOARD_MAC)
            info = check_network(local, destination, mac, chunk_bytes)
            self.events.put(('status', info))
            if source == '视频文件':
                cap = VideoFrames(path, cv2, infps, fps, loop)
            still = None
            if source == '图片文件':
                still = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
                if still is None: raise ValueError('无法解码图片')
            sender = Sender(local, mbps=rate, size=outsize, vs_blank_ms=vs_blank_ms,
                            chunk_bytes=chunk_bytes, min_packet_gap_us=packet_gap_us,
                            destination=destination)
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
                packet_count = 2 + math.ceil(outsize[0]*outsize[1]*2/chunk_bytes)
                send_ms = (sent_at - started) * 1000
                self.events.put(('status',
                    f'{outsize[0]}×{outsize[1]}，VS消隐 {vs_blank_ms:g} ms，'
                    f'UDP负载 {chunk_bytes} B / 最小间隔 {packet_gap_us:g} us，'
                    f'目标 {fps:g} fps / 实际平均 {count / elapsed:.2f} fps；'
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
                if kind in ('local_ips', 'local_ips_error'):
                    self.refresh_button.configure(state='normal')
                    if kind == 'local_ips':
                        self.local_combo.configure(values=text)
                    elif not self.closing:
                        messagebox.showwarning('读取网卡', text)
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
