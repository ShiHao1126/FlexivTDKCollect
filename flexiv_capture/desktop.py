"""Manual episode lifecycle and Tk operator view; all SDK work stays off the UI thread."""
from __future__ import annotations

from pathlib import Path
import threading
import traceback

from .recording import Recorder, finalize


class ManualSession:
    def __init__(self, config: dict, output: Path, *, reset_home=False, recorder_factory=Recorder):
        self.config, self.output = config, output
        self.reset_home, self.recorder_factory = reset_home, recorder_factory
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.status = {"state": "idle", "progress": {}, "path": "", "error": ""}

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def start(self):
        with self.lock:
            if (self.thread is not None and self.thread.is_alive()) or self.status["state"] == "fault":
                return False
            self.stop_event.clear()
            self.status = {"state": "starting", "progress": {}, "path": "", "error": ""}
            self.thread = threading.Thread(target=self._run, name="manual-episode", daemon=False)
            self.thread.start()
            return True

    def stop(self):
        with self.lock:
            if self.status["state"] in {"starting", "recording", "stopping"}:
                self.stop_event.set()
                self.status["state"] = "stopping"

    def _progress(self, value):
        with self.lock:
            self.status["progress"] = value
            if self.status["state"] != "stopping":
                self.status["state"] = "recording"

    def _run(self):
        try:
            recorder = self.recorder_factory(self.config, self.output)
            try:
                if self.stop_event.is_set():
                    with self.lock:
                        self.status["state"] = "idle"
                    return
                reset = recorder.reset_paired_home() if self.reset_home else None
                partial, final, stats = recorder.record_episode(
                    None, stop_event=self.stop_event, start_reset=reset, progress=self._progress)
            finally:
                recorder.close()
            # Only publish after recording AND device cleanup both succeeded.
            finalize(partial, final)
            with self.lock:
                self.status.update(state="saved", path=str(final), progress={
                    "leader": stats["leader"], "robot": stats["robot"],
                    "elapsed_s": stats["wall_duration_s"]})
        except BaseException as exc:
            details = traceback.format_exc()
            try:
                self.output.mkdir(parents=True, exist_ok=True)
                with (self.output / "session_errors.log").open("a", encoding="utf-8") as log:
                    log.write(details + "\n")
            except OSError:
                pass
            with self.lock:
                self.status.update(state="fault", error=f"{type(exc).__name__}: {exc}")


class CollectionWindow:
    def __init__(self, root, session: ManualSession, *, execute: bool):
        from tkinter import ttk, messagebox, StringVar
        self.root, self.session, self.execute = root, session, execute
        self.messagebox = messagebox
        self.closing = False
        root.title("Flexiv TDK · 双侧力采集")
        root.geometry("780x430")
        root.minsize(680, 400)
        panel = ttk.Frame(root, padding=24)
        panel.pack(fill="both", expand=True)
        ttk.Label(panel, text="双臂遥操作 · 双侧力记录", font=("Microsoft YaHei UI", 18)).pack(anchor="w")
        mode = "真机模式：开始后将初始化、归零并启用遥操作" if execute else "离线预览：不连接设备，采集按钮禁用（execute=False）"
        ttk.Label(panel, text=mode).pack(anchor="w", pady=(12, 6))
        ttk.Label(panel, text=f"Leader: {session.config['teleop']['leader_sn']}    Follower: {session.config['robot']['robot_sn']}").pack(anchor="w")
        ttk.Label(panel, text=f"保存目录：{session.output}", wraplength=720).pack(anchor="w", pady=6)
        self.state_text, self.stats_text, self.result_text = StringVar(), StringVar(), StringVar()
        ttk.Label(panel, textvariable=self.state_text, font=("Microsoft YaHei UI", 14)).pack(anchor="w", pady=(16, 8))
        ttk.Label(panel, textvariable=self.stats_text).pack(anchor="w")
        buttons = ttk.Frame(panel)
        buttons.pack(fill="x", pady=18)
        self.start_button = ttk.Button(buttons, text="开始采集", command=self.start)
        self.start_button.pack(side="left", padx=(0, 12))
        self.stop_button = ttk.Button(buttons, text="停止并保存", command=session.stop)
        self.stop_button.pack(side="left")
        ttk.Label(panel, text="无时长上限；手动停止后结束遥操作、校验并保存。异常会自动停止。\n关闭窗口也会请求停止并保存；软件停止按钮不能代替物理急停。", wraplength=720).pack(anchor="w")
        ttk.Label(panel, textvariable=self.result_text, wraplength=720).pack(anchor="w", pady=12)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda event: session.stop())
        self.refresh()

    def start(self):
        if not self.execute:
            return
        if self.messagebox.askokcancel("开始双臂遥操作", "确认设备身份、工具负载、传感器空载归零条件、工作区及急停。\n开始后将初始化并启用双臂遥操作。", parent=self.root):
            self.session.start()

    def close(self):
        self.closing = True
        self.session.stop()

    def refresh(self):
        snapshot = self.session.snapshot()
        state = snapshot["state"]
        busy = self.session.thread is not None and self.session.thread.is_alive()
        names = {"idle": "等待开始", "starting": "初始化中", "recording": "正在采集 · 等待手动停止",
                 "stopping": "正在停止并保存，请稍候", "saved": "已保存，可开始下一段", "fault": "异常停止 · 请检查日志并重启程序"}
        self.state_text.set(names[state])
        stats = snapshot["progress"]
        self.stats_text.set(f"已采集 {stats.get('elapsed_s', 0):.1f} 秒    Leader {stats.get('leader', {}).get('samples', 0)} 条    Follower {stats.get('robot', {}).get('samples', 0)} 条")
        self.result_text.set(snapshot["error"] or snapshot["path"])
        self.start_button.configure(state="normal" if self.execute and not busy and state != "fault" and not self.closing else "disabled")
        self.stop_button.configure(state="normal" if busy and state in {"starting", "recording"} else "disabled")
        if self.closing and not busy:
            if state == "fault":
                self.messagebox.showerror("采集未成功保存", snapshot["error"], parent=self.root)
            self.root.destroy()
            return
        self.root.after(100, self.refresh)


def run_window(config, output, *, execute=False, reset_home=False):
    import tkinter as tk
    root = tk.Tk()
    session = ManualSession(config, output, reset_home=reset_home)
    CollectionWindow(root, session, execute=execute)
    try:
        root.mainloop()
    finally:
        session.stop()
        if session.thread is not None:
            session.thread.join(timeout=5)
            if session.thread.is_alive():
                raise RuntimeError("Device cleanup has not completed; use physical emergency stop if necessary")
