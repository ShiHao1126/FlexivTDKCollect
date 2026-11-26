"""Line-based SSH interface: no Tk, display server or hardware imports."""
import queue
import signal
import sys
import threading


def run_terminal(session, *, execute=False, stdin=None, stdout=None):
    stdin = sys.stdin if stdin is None else stdin
    stdout = sys.stdout if stdout is None else stdout
    if execute and not stdin.isatty():
        raise RuntimeError("Hardware mode requires an interactive terminal (SSH with a PTY)")
    commands = queue.Queue(maxsize=16)
    finished = threading.Event()
    interrupted = threading.Event()
    previous_handlers = {}
    def handle_signal(signum, frame):
        interrupted.set()
    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        sig = getattr(signal, name, None)
        if sig is not None:
            previous_handlers[sig] = signal.signal(sig, handle_signal)

    def reader():
        try:
            while not finished.is_set():
                line = stdin.readline()
                item = None if line == "" else line.strip()
                while not finished.is_set():
                    try:
                        commands.put(item, timeout=.1)
                        break
                    except queue.Full:
                        continue
                if item is None:
                    break
        except (OSError, ValueError):
            interrupted.set()

    def write(text):
        try:
            stdout.write(text + "\n")
            stdout.flush()
        except (OSError, ValueError):
            interrupted.set()

    write("TDK force collection | " + ("HARDWARE" if execute else "OFFLINE PREVIEW (no devices)"))
    write(f"Leader: {session.config['teleop']['leader_sn']}  Follower: {session.config['robot']['robot_sn']}")
    write(f"Output: {session.output}")
    write("h: BOTH arms Home | r: teleop ON | c: record | s: stop RECORDING and save | q: stop ALL and quit (Enter after command)")
    write("No duration limit. Fault protection remains active. Software stop is not an emergency stop.")
    threading.Thread(target=reader, name="terminal-input", daemon=True).start()
    quitting = False
    last_status = None
    try:
        while True:
            if interrupted.is_set():
                session.abort("Terminal disconnected or interrupted")
                quitting = True
            if not quitting:
                try:
                    command = commands.get(timeout=.2)
                except queue.Empty:
                    command = ""
                if command is None:
                    session.abort("Terminal input closed")
                    quitting = True
                elif command == "q":
                    session.shutdown()
                    quitting = True
                elif command == "s":
                    session.stop()
                elif command in {"h", "r"}:
                    if not execute:
                        write("Offline preview: hardware commands disabled (execute=False).")
                    else:
                        action = session.home if command == 'h' else session.engage
                        if not action():
                            write("Command rejected: stop recording first; check session state.")
                elif command == "c":
                    if not execute or not session.start():
                        write("Recording rejected: enable teleoperation with r first (execute=True required).")
                elif command:
                    write("Commands: h, r, c, s, q (then Enter)")
            else:
                finished.wait(.1)
            snapshot = session.snapshot()
            progress = snapshot['progress']
            status = (f"[{snapshot['state']}] {progress.get('elapsed_s', 0):.1f}s | "
                      f"leader={progress.get('leader', {}).get('samples', 0)} "
                      f"follower={progress.get('robot', {}).get('samples', 0)} | "
                      f"{snapshot['error'] or snapshot['path']}")
            if status != last_status:
                write(status)
                last_status = status
            busy = session.thread is not None and session.thread.is_alive()
            if quitting and not busy:
                return 1 if interrupted.is_set() or snapshot['state'] == 'fault' else 0
    finally:
        finished.set()
        session.shutdown()
        if session.thread is not None:
            session.thread.join(timeout=5)
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        if session.thread is not None and session.thread.is_alive():
            raise RuntimeError("Device cleanup has not completed; use physical emergency stop if necessary")
