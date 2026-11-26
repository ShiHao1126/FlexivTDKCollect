"""Shared manual episode lifecycle for desktop and SSH terminal interfaces."""
from pathlib import Path
import threading
import traceback
import json
import h5py
from .recording import Recorder, finalize


class ManualSession:
    def __init__(self, config: dict, output: Path, *, reset_home=False, recorder_factory=Recorder):
        self.config, self.output = config, output
        self.reset_home, self.recorder_factory = reset_home, recorder_factory
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.abort_reason = ""
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

    def abort(self, reason):
        with self.lock:
            self.abort_reason = reason
        self.stop()

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
            if self.abort_reason:
                with h5py.File(partial, "r+") as failed:
                    failed.attrs["failure_json"] = json.dumps({"type": "SessionInterrupted", "message": self.abort_reason})
                raise RuntimeError(self.abort_reason + "; partial retained, not published")
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
