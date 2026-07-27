from dataclasses import dataclass, field
from pathlib import Path


ROBOT_FIELDS = (
    "q",
    "dq",
    "tau",
    "tau_ext",
    "tau_interact",
    "tcp_pose",
    "tcp_vel",
    "ext_wrench_in_tcp",
    "ext_wrench_in_world",
)


@dataclass(frozen=True)
class CollectConfig:
    output_dir: Path

    enable_robot: bool = True
    leader_sn: str = "Rizon4s-063652"
    follower_sn: str = "Rizon4s-063586"
    leader_tool: str = "gripper"
    follower_tool: str = "xense"
    lan_ips: tuple[str, ...] = ()
    robot_rate_hz: float = 200.0

    enable_gripper: bool = True
    gripper_macs: tuple[str, ...] = ("1659f0e0dde0",)
    enable_tactile: bool = True
    tactile_sns: tuple[str, ...] = ("OG001452", "OG001454")
    xense_rate_hz: float = 60.0

    enable_camera: bool = True
    camera_serials: tuple[str, ...] = ()
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30
    collect_depth: bool = True

    image_queue_size: int = 64
    metadata: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.robot_rate_hz <= 0.0 or self.xense_rate_hz <= 0.0:
            raise ValueError("collection rates must be positive")
        if any(value <= 0 for value in (self.camera_width, self.camera_height, self.camera_fps)):
            raise ValueError("camera dimensions and rate must be positive")
        if self.image_queue_size <= 0:
            raise ValueError("image_queue_size must be positive")
        self._require_unique("camera serials", self.camera_serials)
        self._require_unique("gripper MACs", self.gripper_macs)
        self._require_unique("tactile serials", self.tactile_sns)
        if self.enable_robot and (not self.leader_tool or not self.follower_tool):
            raise ValueError("enabled robots require both leader_tool and follower_tool")
        if self.enable_gripper and not self.gripper_macs:
            raise ValueError("enable_gripper requires at least one gripper MAC")
        if self.enable_tactile:
            if not self.tactile_sns:
                raise ValueError("enable_tactile requires at least one tactile serial")
            if len(self.gripper_macs) != 1:
                raise ValueError("tactile sensors require exactly one gripper MAC for connection")

    @property
    def enable_xense(self) -> bool:
        return self.enable_gripper or self.enable_tactile

    @property
    def camera_count(self) -> int:
        return len(self.camera_serials) if self.camera_serials else 1

    @staticmethod
    def _require_unique(name: str, values: tuple[str, ...]) -> None:
        if len(set(values)) != len(values):
            raise ValueError(f"{name} must be unique")
