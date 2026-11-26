"""Tk operator view; SDK work is owned by the shared manual session."""
from .manual_session import ManualSession


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
        self.home_button = ttk.Button(buttons, text="双臂 Home (h)", command=self.home)
        self.home_button.pack(side="left", padx=(0, 12))
        self.engage_button = ttk.Button(buttons, text="开启遥操作 (r)", command=self.engage)
        self.engage_button.pack(side="left", padx=(0, 12))
        self.start_button = ttk.Button(buttons, text="开始采集 (c)", command=self.start)
        self.start_button.pack(side="left", padx=(0, 12))
        self.stop_button = ttk.Button(buttons, text="停止采集并保存 (s)", command=session.stop)
        self.stop_button.pack(side="left")
        ttk.Label(panel, text="s 只停止采集并保存，遥操作保持开启；h 将双臂回 Home。异常会自动停止。\n关闭窗口也会请求停止并保存；软件停止按钮不能代替物理急停。", wraplength=720).pack(anchor="w")
        ttk.Label(panel, textvariable=self.result_text, wraplength=720).pack(anchor="w", pady=12)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda event: session.stop())
        self.refresh()

    def home(self):
        if self.execute:
            self.session.home()

    def engage(self):
        if self.execute:
            self.session.engage()

    def start(self):
        if self.execute:
            self.session.start()

    def close(self):
        self.closing = True
        self.session.shutdown()

    def refresh(self):
        snapshot = self.session.snapshot()
        state = snapshot["state"]
        busy = self.session.thread is not None and self.session.thread.is_alive()
        names = {"idle": "等待双臂 Home / 开启遥操作", "homing": "双臂正在回 Home",
                 "ready": "双臂 Home 完成，遥操作关闭", "starting": "正在开启遥操作",
                 "teleop": "遥操作开启 · 未采集", "recording": "遥操作开启 · 正在采集",
                 "saving": "停止采集并保存 · 遥操作保持开启", "closing": "正在停止全部并退出",
                 "closed": "已停止", "fault": "异常停止 · 请检查日志并重启程序"}
        self.state_text.set(names[state])
        stats = snapshot["progress"]
        self.stats_text.set(f"已采集 {stats.get('elapsed_s', 0):.1f} 秒    Leader {stats.get('leader', {}).get('samples', 0)} 条    Follower {stats.get('robot', {}).get('samples', 0)} 条")
        self.result_text.set(snapshot["error"] or snapshot["path"])
        enabled = self.execute and not self.closing
        self.home_button.configure(state="normal" if enabled and state in {"idle", "ready", "teleop"} else "disabled")
        self.engage_button.configure(state="normal" if enabled and state in {"idle", "ready"} else "disabled")
        self.start_button.configure(state="normal" if enabled and state == "teleop" else "disabled")
        self.stop_button.configure(state="normal" if state == "recording" else "disabled")
        if self.closing and not busy:
            if state == "fault":
                self.messagebox.showerror("采集未成功保存", snapshot["error"], parent=self.root)
            self.root.destroy()
            return
        self.root.after(100, self.refresh)


def run_window(config, output, *, execute=False):
    import tkinter as tk
    root = tk.Tk()
    session = ManualSession(config, output)
    CollectionWindow(root, session, execute=execute)
    try:
        root.mainloop()
    finally:
        session.shutdown()
        if session.thread is not None:
            session.thread.join(timeout=5)
            if session.thread.is_alive():
                raise RuntimeError("Device cleanup has not completed; use physical emergency stop if necessary")
