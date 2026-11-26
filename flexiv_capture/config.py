from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml


def validate_force_only_config(config: dict[str, Any]) -> None:
    """Scope guard for the rebuilt subset; run before importing any hardware SDK."""
    validate_record_config(config)
    if config["robot"].get("control_mode") != "tdk":
        raise ConfigError("This rebuild supports only robot.control_mode=tdk")
    streams = {"camera": config["camera"], "wrist_camera": config.get("wrist_camera", {}),
               "gripper": config.get("gripper", {}), "gripper_teleop": config.get("gripper_teleop", {})}
    streams.update({f"tactile/{k}": v for k, v in config.get("tactile", {}).items()})
    for name, settings in streams.items():
        if settings.get("enabled", name == "camera") is not False:
            raise ConfigError(f"{name} must be disabled in the force-only rebuild")
    if "follower_cartesian_home" in config["episode_reset"]:
        raise ConfigError("This rebuild supports joint Home only")
    size = config["recording"]["robot_queue_size"]
    if type(size) is not int or size < 1:
        raise ConfigError("recording.robot_queue_size must be a positive integer")
    _number(config["recording"], "flush_interval_s", 0.001, 1.0)


class ConfigError(ValueError):
    pass


def load_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ConfigError(f"Config file does not exist: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ConfigError("Config root must be a mapping")
    for section in (
        "robot",
        "camera",
        "recording",
        "episode_reset",
    ):
        if not isinstance(config.get(section), dict):
            raise ConfigError(f"Missing config section: {section}")
    return config


def validate_record_config(config: dict[str, Any]) -> None:
    validate_auxiliary_config(config)
    robot = config["robot"]
    camera = config["camera"]
    mode = robot.get("control_mode", "floating")
    if mode not in {"floating", "tdk"}:
        raise ConfigError("robot.control_mode must be floating or tdk")
    if not str(robot.get("robot_sn", "")).strip():
        raise ConfigError(
            "robot.robot_sn is empty. Set it to the robot serial number, "
            "for example Rizon4s-123456."
        )
    robot_rate_hz = float(robot.get("rate_hz", 0))
    if not math.isfinite(robot_rate_hz) or robot_rate_hz != 100.0:
        raise ConfigError(
            "robot.rate_hz must be 100 for robot state sampling / floating commands"
        )
    camera_enabled = camera.get("enabled", True)
    if not isinstance(camera_enabled, bool):
        raise ConfigError("camera.enabled must be a boolean")
    if camera_enabled:
        if int(camera.get("width", 0)) != 640 or int(camera.get("height", 0)) != 480:
            raise ConfigError("This dataset format requires camera resolution 640x480")
        if int(camera.get("color_fps", 0)) != 60:
            raise ConfigError("This dataset format requires camera.color_fps = 60")
        if int(camera.get("depth_fps", 0)) != 30:
            raise ConfigError("This dataset format requires camera.depth_fps = 30")
        color_options = camera.get("color_options")
        if not isinstance(color_options, dict) or not color_options:
            raise ConfigError("camera.color_options must be a non-empty mapping")
        if float(color_options.get("auto_exposure_priority", -1)) != 0:
            raise ConfigError(
                "camera.color_options.auto_exposure_priority must be 0 to enforce "
                "a constant RGB frame rate"
            )
    recording = config["recording"]
    if recording.get("compression") not in (None, "none", "null", "", "lzf", "gzip"):
        raise ConfigError("recording.compression must be gzip, lzf, or null")
    level = recording.get("compression_level", 4)
    if type(level) is not int or not 0 <= level <= 9:
        raise ConfigError("recording.compression_level must be an integer from 0 to 9")
    workers = recording.get("compression_workers", 4)
    if type(workers) is not int or workers < 1:
        raise ConfigError("recording.compression_workers must be a positive integer")
    for name in ("robot_queue_size", "color_queue_size", "depth_queue_size"):
        if int(recording.get(name, 0)) <= 0:
            raise ConfigError(f"recording.{name} must be positive")
    if camera_enabled:
        depth_range = camera.get("depth_range_m")
        if not isinstance(depth_range, dict):
            raise ConfigError("camera.depth_range_m must be a mapping with min and max")
        min_depth_m = float(depth_range.get("min", 0))
        max_depth_m = float(depth_range.get("max", 0))
        if min_depth_m <= 0 or max_depth_m <= min_depth_m:
            raise ConfigError(
                "camera.depth_range_m must satisfy 0 < min < max (values are metres)"
            )
        depth_options = camera.get("depth_options")
        if not isinstance(depth_options, dict) or not depth_options:
            raise ConfigError("camera.depth_options must be a non-empty mapping")

    if mode == "tdk":
        validate_teleop_config(config)
        return
    if config.get("gripper_teleop", {}).get("enabled", False):
        raise ConfigError("gripper_teleop requires robot.control_mode = tdk")

    floating_control = config.get("floating_control")
    if not isinstance(floating_control, dict):
        raise ConfigError("floating_control must be a mapping")
    locked_axes = floating_control.get("locked_axes")
    if not isinstance(locked_axes, list) or any(
        not isinstance(axis, str) for axis in locked_axes
    ):
        raise ConfigError("floating_control.locked_axes must be a list of axis names")
    normalized_axes = [axis.strip().lower() for axis in locked_axes]
    available_axes = {"x", "y", "z", "rx", "ry", "rz"}
    unknown_axes = sorted(set(normalized_axes) - available_axes)
    if unknown_axes:
        raise ConfigError(
            "Unknown floating_control.locked_axes: " + ", ".join(unknown_axes)
        )
    if len(normalized_axes) != len(set(normalized_axes)):
        raise ConfigError("floating_control.locked_axes must not contain duplicates")
    try:
        zero_timeout_s = float(
            floating_control.get("zero_ft_sensor_timeout_s", 30.0)
        )
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            "floating_control.zero_ft_sensor_timeout_s must be numeric"
        ) from exc
    if not math.isfinite(zero_timeout_s) or zero_timeout_s <= 0:
        raise ConfigError(
            "floating_control.zero_ft_sensor_timeout_s must be positive"
        )

    reset = config.get("episode_reset")
    if not isinstance(reset, dict):
        raise ConfigError("episode_reset must be a mapping")
    reference = reset.get("reference")
    if not isinstance(reference, dict):
        raise ConfigError("episode_reset.reference must be a mapping")
    try:
        q = [float(value) for value in reference["q_rad"]]
        pose = [float(value) for value in reference["tcp_pose_wxyz"]]
        position_jitter = [
            float(value) for value in reset["position_jitter_world_m"]
        ]
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigError(
            "episode_reset reference and position jitter must be numeric sequences"
        ) from exc
    if len(q) != 7:
        raise ConfigError("episode_reset.reference.q_rad must contain 7 values")
    if len(pose) != 7:
        raise ConfigError(
            "episode_reset.reference.tcp_pose_wxyz must contain 7 values"
        )
    if len(position_jitter) != 3 or any(value < 0 for value in position_jitter):
        raise ConfigError(
            "episode_reset.position_jitter_world_m must contain 3 non-negative values"
        )
    if not all(math.isfinite(value) for value in (*q, *pose, *position_jitter)):
        raise ConfigError("episode_reset numeric values must be finite")
    quaternion_norm = sum(value * value for value in pose[3:]) ** 0.5
    if abs(quaternion_norm - 1.0) > 1e-3:
        raise ConfigError(
            "episode_reset.reference TCP quaternion must have unit norm"
        )
    try:
        rotation_jitter = float(reset.get("rotation_jitter_tcp_y_deg", -1))
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            "episode_reset.rotation_jitter_tcp_y_deg must be numeric"
        ) from exc
    if not math.isfinite(rotation_jitter) or rotation_jitter < 0:
        raise ConfigError(
            "episode_reset.rotation_jitter_tcp_y_deg must be non-negative"
        )
    velocity_raw = reset.get("movej_velocity_scale", 0)
    try:
        velocity_numeric = float(velocity_raw)
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            "episode_reset.movej_velocity_scale must be an integer"
        ) from exc
    if (
        isinstance(velocity_raw, bool)
        or not math.isfinite(velocity_numeric)
        or not velocity_numeric.is_integer()
        or not 1 <= velocity_numeric <= 100
    ):
        raise ConfigError(
            "episode_reset.movej_velocity_scale must be an integer between 1 and 100"
        )
    for name in ("enable_timeout_s", "motion_timeout_s"):
        try:
            value = float(reset.get(name, 0))
        except (TypeError, ValueError) as exc:
            raise ConfigError(f"episode_reset.{name} must be numeric") from exc
        if not math.isfinite(value) or value <= 0:
            raise ConfigError(f"episode_reset.{name} must be positive")
    attempts = reset.get("max_ik_attempts", 0)
    try:
        attempts_numeric = float(attempts)
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            "episode_reset.max_ik_attempts must be a positive integer"
        ) from exc
    if (
        isinstance(attempts, bool)
        or not math.isfinite(attempts_numeric)
        or not attempts_numeric.is_integer()
        or attempts_numeric <= 0
    ):
        raise ConfigError("episode_reset.max_ik_attempts must be a positive integer")


def _number(settings: dict, name: str, low: float, high: float = math.inf) -> float:
    try:
        value = float(settings[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigError(f"{name} must be numeric") from exc
    if isinstance(settings[name], bool) or not math.isfinite(value) or not low <= value <= high:
        raise ConfigError(f"{name} must be finite and in [{low}, {high}]")
    return value


def validate_teleop_config(config: dict[str, Any]) -> None:
    teleop = config.get("teleop")
    if not isinstance(teleop, dict):
        raise ConfigError("teleop must be a mapping")
    leader = teleop.get("leader_sn", "")
    if not isinstance(leader, str) or not leader.strip():
        raise ConfigError("teleop.leader_sn must be a non-empty string")
    if leader.replace(" ", "") == config["robot"]["robot_sn"].replace(" ", ""):
        raise ConfigError("Leader and follower serial numbers must differ")
    axes = teleop.get("locked_axes")
    if not isinstance(axes, list) or any(
        axis not in ("x", "y", "z", "rx", "ry", "rz") for axis in axes
    ) or len(set(axes)) != len(axes):
        raise ConfigError("teleop.locked_axes must contain unique Cartesian axis names")
    if teleop.get("lock_coordinate", "world") not in ("world", "tcp"):
        raise ConfigError("teleop.lock_coordinate must be world or tcp")
    if not isinstance(teleop.get("limit_wrist_singular"), bool):
        raise ConfigError("teleop.limit_wrist_singular must be boolean")
    _number(teleop, "feedback_scale", 0, 3)
    stiffness = teleop.get("follower_stiffness")
    if isinstance(stiffness, list):
        if len(stiffness) != 6:
            raise ConfigError("teleop.follower_stiffness must contain six ratios in WORLD X/Y/Z/Rx/Ry/Rz order")
        for index, ratio in enumerate(stiffness):
            name = f"follower_stiffness[{index}]"
            _number({name: ratio}, name, 0, 1)
    else:
        _number(teleop, "follower_stiffness", 0, 1)
    _number(teleop, "state_stale_timeout_s", 0.01)
    _number(teleop, "max_age_ms", 1)
    wrench = teleop.get("max_contact_wrench")
    if not isinstance(wrench, list) or len(wrench) != 6:
        raise ConfigError("teleop.max_contact_wrench must contain six values")
    for value in wrench:
        _number({"max_contact_wrench": value}, "max_contact_wrench", 0)
    reset = config["episode_reset"]
    for name in ("leader_home_q_deg", "follower_home_q_deg"):
        angles = reset.get(name)
        if not isinstance(angles, list) or len(angles) != 7:
            raise ConfigError(f"episode_reset.{name} must contain seven joint angles")
        for angle in angles:
            _number({name: angle}, name, -360, 360)
    speed = _number(reset, "movej_velocity_scale", 1, 100)
    if not speed.is_integer():
        raise ConfigError("movej_velocity_scale must be integral")
    for name in ("position_tolerance_deg", "enable_timeout_s", "motion_timeout_s"):
        _number(reset, name, 0.001)
    if "follower_cartesian_home" in reset:
        home = reset["follower_cartesian_home"]
        if not isinstance(home, dict):
            raise ConfigError("episode_reset.follower_cartesian_home must be a mapping")
        if home.get("robot_sn") != config["robot"]["robot_sn"]:
            raise ConfigError("follower_cartesian_home.robot_sn must match the follower")
        pose = home.get("tcp_pose_wxyz")
        if not isinstance(pose, list) or len(pose) != 7:
            raise ConfigError("follower_cartesian_home.tcp_pose_wxyz must contain seven values")
        for value in pose:
            _number({"tcp_pose_wxyz": value}, "tcp_pose_wxyz", -math.inf)
        if abs(sum(float(value) ** 2 for value in pose[3:]) - 1.0) > 0.001:
            raise ConfigError("follower_cartesian_home quaternion must have unit norm")
        _number(home, "approach_height_m", 0.001)
        _number(home, "linear_velocity_m_s", 0.001, 2.2)
        _number(home, "angular_velocity_deg_s", 1, 500)
        _number(home, "position_tolerance_m", 0.00001)
        _number(home, "rotation_tolerance_deg", 0.001)

    gripper = config.get("gripper_teleop", {})
    if not isinstance(gripper, dict) or not isinstance(gripper.get("enabled", False), bool):
        raise ConfigError("gripper_teleop must have a boolean enabled flag")
    if not gripper.get("enabled", False):
        return
    if not config.get("gripper", {}).get("enabled", False):
        raise ConfigError("gripper_teleop requires gripper.enabled = true")
    if not isinstance(gripper.get("port"), str) or not gripper["port"].strip():
        raise ConfigError("gripper_teleop.port must be a non-empty string")
    if not isinstance(gripper.get("strict"), bool):
        raise ConfigError("gripper_teleop.strict must be boolean")
    for name, low, high in (
        ("index", 1, 247), ("baudrate", 1, 4_000_000), ("max_attempts", 1, 10),
        ("queue_size", 1, 100_000),
    ):
        if not _number(gripper, name, low, high).is_integer():
            raise ConfigError(f"gripper_teleop.{name} must be integral")
    for name, low, high in (
        ("gap", -1, 0.1), ("read_timeout_s", 0.001, 0.5),
        ("open_angle_deg", 0, 360), ("close_angle_deg", 0, 360),
        ("open_width_m", 0, 0.085), ("close_width_m", 0, 0.085),
        ("deadband_m", 0, 0.085), ("velocity_m_s", 0.001, 0.35),
        ("force_n", 0.001, 60), ("rate_hz", 1, 200), ("max_age_ms", 1, 1000),
    ):
        _number(gripper, name, low, high)
    if gripper["open_angle_deg"] == gripper["close_angle_deg"]:
        raise ConfigError("Encoder open and close angles must differ")
    if gripper["open_width_m"] <= gripper["close_width_m"]:
        raise ConfigError("Gripper open width must exceed close width")


def validate_auxiliary_config(config: dict[str, Any]) -> None:
    tactile = config.get("tactile", {})
    if not isinstance(tactile, dict):
        raise ConfigError("tactile must be a mapping")
    streams = {
        "wrist_camera": config.get("wrist_camera", {}),
        "gripper": config.get("gripper", {}),
        **{f"tactile.{side}": tactile.get(side, {}) for side in ("left", "right")},
    }
    for name, settings in streams.items():
        if not isinstance(settings, dict) or not isinstance(
            settings.get("enabled", False), bool
        ):
            raise ConfigError(f"{name} must be a mapping with a boolean enabled flag")
        if not settings.get("enabled", False):
            continue
        identifiers = ("serial_number",) if name == "wrist_camera" else ("mac_addr",)
        if name.startswith("tactile."):
            identifiers += ("device_id",)
            if settings.get("color_order") not in {"rgb", "bgr"}:
                raise ConfigError(f"{name}.color_order must be rgb or bgr")
        for key in identifiers:
            if not isinstance(settings.get(key), str) or not settings[key].strip():
                raise ConfigError(f"{name}.{key} must be a non-empty string")
        for key in ("queue_size", "max_age_ms") + (
            () if name == "wrist_camera" else ("rate_hz",)
        ):
            try:
                value = float(settings[key])
            except (KeyError, ValueError, TypeError) as exc:
                raise ConfigError(f"{name}.{key} must be positive") from exc
            if (
                not math.isfinite(value)
                or value <= 0
                or (key == "queue_size" and not value.is_integer())
            ):
                raise ConfigError(
                    f"{name}.{key} must be positive (queue_size must be integral)"
                )
        if name == "wrist_camera":
            if (settings.get("width"), settings.get("height"), settings.get("fps")) != (
                640,
                480,
                60,
            ):
                raise ConfigError("wrist_camera requires 640x480 RGB at 60 FPS")
    left, right = tactile.get("left", {}), tactile.get("right", {})
    if (
        left.get("enabled")
        and right.get("enabled")
        and left["device_id"] == right["device_id"]
    ):
        raise ConfigError("Left and right tactile device_id must differ")
    wrist = streams["wrist_camera"]
    if wrist.get("enabled") and wrist["serial_number"] == config.get("camera", {}).get(
        "serial_number"
    ):
        raise ConfigError("Wrist and main camera serial numbers must differ")
