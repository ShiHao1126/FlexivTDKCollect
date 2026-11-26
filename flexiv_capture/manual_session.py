"""Own separate bilateral Home, teleoperation and recording lifecycles."""
from pathlib import Path
import threading
import queue
import traceback
import json
import h5py
from .recording import Recorder, finalize


class ManualSession:
    def __init__(self, config: dict, output: Path, *, recorder_factory=Recorder):
        self.config, self.output = config, output
        self.recorder_factory = recorder_factory
        self.stop_event = threading.Event()
        self.exit_event = threading.Event()
        self.commands = queue.Queue()
        self.lock = threading.Lock()
        self.thread = None
        self.abort_reason = ""
        self.status = {"state": "idle", "progress": {}, "path": "", "error": ""}

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def _request(self, command, allowed, state):
        with self.lock:
            if self.exit_event.is_set() or self.status["state"] not in allowed:
                return False
            self.status["state"] = state
            if command == "record":
                self.stop_event.clear()
                self.status.update(progress={}, path="")
            self.commands.put(command)
            if self.thread is None:
                self.thread = threading.Thread(target=self._run, name="manual-session", daemon=False)
                self.thread.start()
            return True

    def home(self):
        return self._request("home", {"idle", "ready", "teleop"}, "homing")

    def engage(self):
        return self._request("engage", {"idle", "ready"}, "starting")

    def start(self):
        return self._request("record", {"teleop"}, "recording")

    def stop(self):
        """Stop only recording; leave teleoperation active on normal completion."""
        with self.lock:
            if self.status["state"] in {"recording", "saving"}:
                self.stop_event.set()
                self.status["state"] = "saving"

    def shutdown(self):
        with self.lock:
            self.exit_event.set()
            self.stop_event.set()
            if self.status["state"] not in {"fault", "closed"}:
                self.status["state"] = "closing" if self.thread else "closed"

    def abort(self, reason):
        with self.lock:
            self.abort_reason = reason
        self.shutdown()

    def _check_cancelled(self):
        if self.exit_event.is_set():
            raise RuntimeError("Session closing; Home cancelled")

    def _progress(self, value):
        with self.lock:
            self.status["progress"] = value

    def _set_state(self, state):
        with self.lock:
            self.status["state"] = "closing" if self.exit_event.is_set() else state

    def _run(self):
        recorder = None
        failure = None
        last_home = None
        try:
            recorder = self.recorder_factory(self.config, self.output)
            while not self.exit_event.is_set():
                try:
                    command = self.commands.get(timeout=.05)
                except queue.Empty:
                    if recorder.robot.engaged:
                        # Keep fault and state-staleness supervision active without recording.
                        recorder.robot.read_pair()
                    else:
                        recorder.check_teleop_health()
                    continue
                if self.exit_event.is_set():
                    break
                if command == "home":
                    last_home = recorder.reset_paired_home(check_cancelled=self._check_cancelled)
                    self._set_state("ready")
                elif command == "engage":
                    recorder.start_teleop(cancel_event=self.exit_event)
                    self._set_state("teleop")
                elif command == "record":
                    partial, final, stats = recorder.record_episode(
                        None, stop_event=self.stop_event, start_reset=last_home,
                        progress=self._progress, manage_teleop=False)
                    if self.exit_event.is_set():
                        recorder.stop_teleop()
                    if self.abort_reason:
                        with h5py.File(partial, "r+") as failed:
                            failed.attrs["failure_json"] = json.dumps({"type": "SessionInterrupted", "message": self.abort_reason})
                        raise RuntimeError(self.abort_reason + "; partial retained, not published")
                    finalize(partial, final)
                    with self.lock:
                        self.status.update(path=str(final), progress={
                            "leader": stats["leader"], "robot": stats["robot"],
                            "elapsed_s": stats["wall_duration_s"]})
                    self._set_state("teleop")
        except BaseException as exc:
            failure = f"{type(exc).__name__}: {exc}"
            details = traceback.format_exc()
            try:
                self.output.mkdir(parents=True, exist_ok=True)
                with (self.output / "session_errors.log").open("a", encoding="utf-8") as log:
                    log.write(details + "\n")
            except OSError:
                pass
        finally:
            if recorder is not None:
                try:
                    recorder.close()
                except BaseException as exc:
                    failure = (failure + "; " if failure else "") + f"Cleanup failed: {exc}"
            with self.lock:
                self.status.update(state="fault" if failure else "closed", error=failure or "")
