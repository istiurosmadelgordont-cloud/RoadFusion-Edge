#!/usr/bin/env python3
"""Linux PCIe HDMI 采集上位机（Tkinter 图形界面 + 无头命令行两种模式）。

线程模型（很重要）：
  Tk 及其所有控件只能在**主线程**创建和更新；
  采集在 FramePipeline 自己的后台线程里跑，通过 LatestFrame 投递最新一帧；
  主线程用 root.after() 周期性轮询（不是阻塞等待），避免界面卡死。

三种数据来源严格区分，界面和日志里都会标注，不要混淆：
  hardware —— 从 /dev/pcie_hdmi_host 读，才是真正的 PCIe 采集
  demo     —— 本机合成彩条，用来验证界面和处理流程
  file     —— 读磁盘上已有的原始帧
只有 hardware 模式的结果能作为 PCIe 链路通不通的证据。
"""
import argparse
import queue
import threading
import time
import subprocess
import json
from pathlib import Path
from frames import acquire, rgb, check, save, preview_rgb
from preview_queue import LatestFrame
from capture_pipeline import FramePipeline
from fpga_control import send_enhancement


def diagnostics():
    """收集 lspci 与 dmesg 的最近输出，供界面上的"设备/内核诊断"按钮显示。

    两个命令都做了超时保护：dmesg 在某些内核上可能很慢或需要权限，
    失败时把错误文本本身返回，让用户看到原因而不是空白。
    """
    sections = []
    for cmd in (["lspci", "-nnk"], ["dmesg", "--color=never"]):
        try:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    timeout=8, universal_newlines=True)
            lines = result.stdout.splitlines()
            # 只取末尾 100 行，避免把整个内核日志灌进文本框
            sections.append("$ " + " ".join(cmd) + "\n" + "\n".join(lines[-100:]))
        except (OSError, subprocess.TimeoutExpired) as e:
            sections.append(str(e))
    return "\n\n".join(sections)


def gui(args):
    """构建并运行图形界面（必须在主线程调用）。"""
    # 延迟导入：headless 模式不需要 Tk/PIL，缺这些包时也能跑命令行
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    from tkinter.scrolledtext import ScrolledText
    from PIL import Image, ImageTk

    root = tk.Tk()
    root.title("PCIe HDMI alloc")
    root.geometry("1120x820")

    # 跨线程通信只用这两个：
    #   events —— 后台线程发往主线程的事件（错误/完成/诊断文本）
    #   latest —— 只保留最新一帧的预览队列（覆盖式，不排队）
    events = queue.Queue()
    latest = LatestFrame()
    stop = threading.Event()

    # 界面状态。count/received 用于算显示帧率与接收帧率，两者可能不同：
    # 接收帧率反映 PCIe 实际吞吐，显示帧率受转换和 Tk 渲染限制。
    state = dict(busy=False, closing=False, raw=None, meta=None, count=0, photo=None)
    source = tk.StringVar(value=args.source)
    device = tk.StringVar(value=args.device)
    path = tk.StringVar(value=args.input or "")
    verify = tk.BooleanVar(value=args.check_colorbars and not args.no_check)
    dark_amount = tk.StringVar(value="32")
    highlight_amount = tk.StringVar(value="32")
    status = tk.StringVar(value="就绪 | 单路 1920×1080 RGB565 | 四帧 DMA 缓冲")

    # ---------- 顶部工具条 ----------
    tools = ttk.Frame(root, padding=10); tools.pack(fill="x")
    ttk.Label(tools, text="数据来源").grid(row=0, column=0)
    selection = ttk.Combobox(tools, textvariable=source, values=["hardware", "demo", "file"], state="readonly", width=12)
    selection.grid(row=0, column=1, padx=8)
    ttk.Label(tools, text="设备").grid(row=0, column=2)
    ttk.Entry(tools, textvariable=device, width=30).grid(row=0, column=3, padx=8)
    ttk.Checkbutton(tools, text="校验标准彩条", variable=verify).grid(row=0, column=4)
    ttk.Entry(tools, textvariable=path, width=65).grid(row=1, column=0, columnspan=4, sticky="ew", pady=8)

    def browse():
        chosen = filedialog.askopenfilename(filetypes=[("RGB565 原始帧", "*.rgb565"), ("所有文件", "*")])
        if chosen:
            path.set(chosen); source.set("file")
    ttk.Button(tools, text="选择原始帧", command=browse).grid(row=1, column=4)

    # FPGA 侧图像增强：这两个按钮只下发参数，硬件没有回读手段，
    # 因此无法确认是否生效（详见 FPGA增强控制说明.md）。
    ttk.Label(tools, text="FPGA 暗光参数 0–255").grid(row=2, column=0, columnspan=2)
    ttk.Entry(tools, textvariable=dark_amount, width=8).grid(row=2, column=2)
    ttk.Button(tools, text="发送暗光增强", command=lambda: control(0x150, dark_amount.get())).grid(row=2, column=3)
    ttk.Label(tools, text="FPGA 高光参数 0–255").grid(row=3, column=0, columnspan=2)
    ttk.Entry(tools, textvariable=highlight_amount, width=8).grid(row=3, column=2)
    ttk.Button(tools, text="发送高光压制", command=lambda: control(0x140, highlight_amount.get())).grid(row=3, column=3)
    ttk.Button(tools, text="关闭两项增强", command=lambda: control(0x160, "0")).grid(row=3, column=4)

    # 切换来源时更新提示语——反复强调 demo/file 不代表 PCIe 测试通过
    source_note = tk.StringVar()

    def update_source_note(*unused):
        source_note.set({
            "demo": "演示模式：彩条由本机软件生成，不读取 FPGA；PASS 仅表示合成图像校验通过。",
            "file": "文件模式：读取已有图像，不验证当前 PCIe 传输。",
            "hardware": "硬件模式：从 PCIe 驱动读取图像；需要连接 FPGA 并加载驱动。"
        }[source.get()])
    source.trace_add("write", update_source_note)
    update_source_note()
    ttk.Label(root, textvariable=source_note, foreground="#b45309").pack(anchor="w", padx=10)

    # ---------- 预览区 ----------
    bar = ttk.Frame(root, padding=(10, 0)); bar.pack(fill="x")
    preview = tk.Label(root, text="尚未采集\nhardware：PCIe 设备  |  demo：合成彩条  |  file：已有原始帧",
                       bg="#15202b", fg="white")
    preview.pack(fill="both", expand=True, padx=10, pady=10)
    ttk.Label(root, textvariable=status).pack(anchor="w", padx=10)
    log = ScrolledText(root, height=9, state="disabled"); log.pack(fill="x", padx=10, pady=10)

    def write(text):
        """向日志框追加一行（带时间戳），并限制总行数避免无限增长。"""
        log.configure(state="normal")
        log.insert("end", time.strftime("[%H:%M:%S] ") + text + "\n")
        if int(log.index("end-1c").split(".")[0]) > 400:
            log.delete("1.0", "100.0")       # 超过 400 行就砍掉最旧的 99 行
        log.see("end"); log.configure(state="disabled")

    def control(offset, text):
        """校验并下发一次增强参数（在后台线程里做，避免阻塞界面）。"""
        if source.get() != "hardware":
            messagebox.showinfo("仅硬件模式", "请先选择 hardware 数据来源")
            return
        try:
            value = int(text)
            if not 0 <= value <= 255: raise ValueError()
        except ValueError:
            messagebox.showerror("参数错误", "请输入 0～255 的整数")
            return

        target = device.get()

        def task():
            try:
                send_enhancement(target, offset, value)
                # 说明用词要准确：只是"提交"了写事务，硬件没有回读确认
                events.put(("diag", "控制写入已提交：BAR1+0x{:03x}，值={}；无 FPGA 生效回读".format(offset, value)))
            except OSError as exc:
                events.put(("diag", "增强命令发送失败：{}；确认已更新驱动".format(exc)))

        threading.Thread(target=task, daemon=True).start()

    def worker(continuous, settings):
        """采集线程：从流水线取帧，转成预览图，交给主线程显示。

        注意这里**不做任何 Tk 操作**——只往 latest 里放数据，
        由主线程的 poll() 去渲染。
        """
        try:
            with FramePipeline(settings[0], settings[1], settings[2], stop, continuous) as pipeline:
                while not stop.is_set():
                    item = pipeline.take()
                    if item is None: break
                    raw, elapsed, timestamp, sequence = item

                    meta = dict(source=settings[0], device=settings[1], width=1920, height=1080,
                                format="RGB565_LE", capture_seconds=elapsed, timestamp=timestamp,
                                received_count=sequence, queue_dropped=pipeline.dropped)
                    # 彩条校验很费 CPU，默认只在单帧模式下开启（GUI 里的勾选框控制）
                    if settings[3]: meta["colorbar_check"] = check(raw)

                    # 记录"转换耗时"，和下面的 Tk 渲染耗时一起用来判断瓶颈在哪
                    conversion_start = time.monotonic()
                    pixels_rgb = preview_rgb(raw, args.preview_step)
                    im = Image.fromarray(pixels_rgb)
                    meta["conversion_ms"] = (time.monotonic() - conversion_start) * 1000
                    meta["preview_ready"] = time.monotonic()

                    latest.put(("frame", raw, meta, im))
        except Exception as e:
            events.put(("error", str(e)))
        finally:
            events.put(("done",))

    def start(continuous):
        """开始采集（continuous=True 为连续预览，False 为单帧）。"""
        if state["busy"]: return
        settings = (source.get(), device.get(), path.get(), verify.get())
        if settings[0] == "file" and not settings[2]:
            messagebox.showerror("缺少文件", "请选择 RGB565 原始帧"); return

        state["busy"] = True
        state.update(count=0, received=0, fps_start=time.monotonic(),
                     fps_displayed=0, fps_received=0, last_log=0)
        latest.take()                      # 丢掉上一轮遗留的帧
        for button in (single, repeat): button.configure(state="disabled")
        stop.clear()
        status.set("采集中… 来源 " + settings[0])
        # 采集线程用非 daemon：关闭窗口时也会等它把当前 read 走完
        threading.Thread(target=worker, args=(continuous, settings), daemon=False).start()

    def halt():
        """请求停止。只是置位事件，等采集线程自己退出（可能等约 5 秒）。"""
        stop.set()
        status.set("正在停止；等待当前驱动读取返回（无新帧时约 5 秒）")

    def save_current():
        if state["raw"] is None:
            messagebox.showinfo("提示", "请先采集一帧"); return
        folder = filedialog.askdirectory(initialdir=str(Path(args.output).resolve()))
        if folder:
            try: write("保存：" + save(state["raw"], folder, state["meta"]))
            except Exception as e: messagebox.showerror("保存失败", str(e))

    def diag():
        def task(): events.put(("diag", diagnostics()))
        threading.Thread(target=task, daemon=True).start()

    single = ttk.Button(bar, text="采集一帧", command=lambda: start(False)); single.pack(side="left")
    repeat = ttk.Button(bar, text="连续预览", command=lambda: start(True)); repeat.pack(side="left", padx=8)
    ttk.Button(bar, text="停止", command=halt).pack(side="left")
    ttk.Button(bar, text="保存 PNG / RAW", command=save_current).pack(side="left", padx=8)
    ttk.Button(bar, text="设备 / 内核诊断", command=diag).pack(side="left")
    ttk.Label(bar, text="停止仅停止预览；demo/file 不验证 PCIe").pack(side="right")

    def poll():
        """主线程轮询：消费事件与最新帧，更新界面。每 4ms 跑一次。

        用轮询而不是阻塞等待，是为了让 Tk 的事件循环始终保持响应。
        """
        try:
            pending = []
            # 一次最多取 8 个事件，防止事件风暴把这一轮拖太久
            for _control in range(8):
                try: pending.append(events.get_nowait())
                except queue.Empty: break
            frame = latest.take()
            if frame is not None: pending.insert(0, frame)   # 先渲染最新帧

            for item in pending:
                if item[0] == "frame":
                    _, raw, meta, im = item
                    state.update(raw=raw, meta=meta, count=state["count"]+1)

                    # 测量"预览就绪 → 真正画到屏幕上"的等待时间，
                    # 过长说明主线程被别的事占住了
                    display_start = time.monotonic()
                    ready_age_ms = (display_start - meta["preview_ready"]) * 1000
                    state["photo"] = ImageTk.PhotoImage(im)   # 必须持引用，否则会被 GC 掉
                    preview.configure(image=state["photo"], text="")
                    tk_ms = (time.monotonic() - display_start) * 1000

                    result = meta.get("colorbar_check")
                    result_text = ("PASS" if result["passed"] else "FAIL 错误像素={}".format(result["mismatched_pixels"])) if result else "未校验"

                    now = time.monotonic()
                    state["received"] = meta["received_count"]
                    duration = now - state["fps_start"]
                    if duration >= 1.0 or state["count"] == 1:
                        # 两个帧率含义不同：read = PCIe 实际到手，shown = 真正渲染出来的
                        shown_fps = (state["count"] - state["fps_displayed"]) / max(duration, 0.001)
                        read_fps = (state["received"] - state["fps_received"]) / max(duration, 0.001)
                        rates = "统计中" if duration < 1.0 else "读取 {:.1f} FPS | 显示 {:.1f} FPS".format(read_fps, shown_fps)
                        text = "来源={} | {} | 接收 {} / 显示 {} | {}".format(meta["source"], rates, state["received"], state["count"], result_text)
                        text += " | 处理队列丢旧帧 {}".format(meta.get("queue_dropped", 0))
                        text += " | 读 {:.0f}ms 转 {:.0f}ms Tk {:.0f}ms 待显 {:.0f}ms".format(meta["capture_seconds"]*1000, meta["conversion_ms"], tk_ms, ready_age_ms)
                        status.set(text)

                        # 日志每秒最多一条，避免刷屏
                        if now - state["last_log"] >= 1.0:
                            write(text)
                            if result and result["first"]: write("首个错误：" + str(result["first"]))
                            state["last_log"] = now
                        if duration >= 1.0:
                            state.update(fps_start=now, fps_displayed=state["count"], fps_received=state["received"])

                elif item[0] == "error":
                    status.set("采集失败"); write(item[1] + "；请查看设备 / 内核诊断")
                elif item[0] == "done":
                    state["busy"] = False
                    if stop.is_set(): status.set("已停止")
                    for button in (single, repeat): button.configure(state="normal")
                elif item[0] == "diag": write(item[1])
        except queue.Empty:
            pass
        except Exception as e:
            # 显示环节出错就主动停下，避免错误反复刷屏
            stop.set()
            status.set('显示处理失败，已请求停止')
            write(str(e))

        # 关闭窗口时，等采集线程收尾完成再真正销毁窗口
        if state["closing"] and not state["busy"]:
            root.destroy(); return
        root.after(4, poll)

    def close():
        """点关闭按钮：先请求停止，等 poll() 确认空闲后再销毁窗口。"""
        state["closing"] = True
        halt()

    root.protocol("WM_DELETE_WINDOW", close)
    root.after(4, poll)
    root.mainloop()


def main():
    """命令行入口：无 --headless 则进 GUI，否则按 --count 采集若干帧。"""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", choices=["hardware", "demo", "file"], default="hardware")
    p.add_argument("--device", default="/dev/pcie_hdmi_host")
    p.add_argument("--input")
    p.add_argument("--output", default="captures")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--count", type=int, default=1)
    p.add_argument("--no-check", action="store_true", help="不检查标准彩条，适用于其他图像")
    p.add_argument("--preview-step", type=int, choices=[1, 2, 3], default=2,
                   help="预览降采样：1=原尺寸，2=960x540，3=640x360；保存仍为原尺寸")
    p.add_argument("--check-colorbars", action="store_true", help="GUI 启用逐帧彩条校验（影响性能）")
    p.add_argument("--diagnostics", action="store_true")
    args = p.parse_args()

    # 参数早校验，避免跑起来才发现用不了
    if args.count < 1: p.error("--count 必须大于 0")
    if args.source == "file" and not args.input: p.error("file 模式需要 --input")
    if args.diagnostics: print(diagnostics()); return 0
    if not args.headless:
        gui(args); return 0

    # ---- 无头模式：逐帧采集、打印元数据、落盘 ----
    failed = False
    for i in range(args.count):
        raw, elapsed = acquire(args.source, args.device, args.input)
        meta = dict(source=args.source, capture_seconds=elapsed, timestamp=time.time(),
                    width=1920, height=1080, format="RGB565_LE")
        if not args.no_check:
            meta["colorbar_check"] = check(raw)
            failed |= not meta["colorbar_check"]["passed"]
        print(json.dumps(meta, ensure_ascii=False), flush=True)
        print("保存：" + save(raw, args.output, meta), flush=True)
    # 退出码 1 表示有帧没通过彩条校验，便于脚本判断
    return 1 if failed else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, ImportError) as e:
        # 统一把启动/采集类失败转成友好提示 + 退出码 1
        print("启动/采集失败：{}".format(e))
        raise SystemExit(1)
