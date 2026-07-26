from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class CollectConfig:
    output_dir: Path
    leader_sn: str
    follower_sn: str = "Rizon4s-063586"
    lan_ips: tuple[str, ...] = ()
    gripper_mac: str = "1659f0e0dde0"
    left_tactile_sn: str = "OG001452"
    right_tactile_sn: str = "OG001454"
    robot_rate_hz: float = 200.0
    xense_rate_hz: float = 60.0
    image_queue_size: int = 64

    enable_robot: bool = True
    enable_robot_joint_state: bool = True
    enable_robot_torque: bool = True
    enable_robot_tcp: bool = True
    enable_robot_wrench_tcp: bool = True
    enable_robot_wrench_world: bool = True

    camera_serials: tuple[str, ...] = ()
    enable_camera_color: bool = True
    camera_color_width: int = 640
    camera_color_height: int = 480
    camera_color_fps: int = 30
    enable_camera_depth: bool = True
    camera_depth_width: int = 640
    camera_depth_height: int = 480
    camera_depth_fps: int = 30
    align_depth_to_color: bool = True

    enable_gripper_width: bool = True
    enable_tactile_marker: bool = True
    enable_tactile_force: bool = True
    enable_tactile_rectify: bool = True
    enable_tactile_difference: bool = True
    enable_tactile_depth: bool = True
    metadata: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.robot_rate_hz <= 0.0 or self.xense_rate_hz <= 0.0:
            raise ValueError("collection rates must be positive")
        camera_values = (
            self.camera_color_width, self.camera_color_height, self.camera_color_fps,
            self.camera_depth_width, self.camera_depth_height, self.camera_depth_fps,
        )
        if any(value <= 0 for value in camera_values):
            raise ValueError("camera dimensions and rates must be positive")
        if self.image_queue_size <= 0:
            raise ValueError("image_queue_size must be positive")
        if len(set(self.camera_serials)) != len(self.camera_serials):
            raise ValueError("camera serials must be unique")

    @property
    def enable_camera(self) -> bool:
        return self.enable_camera_color or self.enable_camera_depth

    @property
    def camera_count(self) -> int:
        return len(self.camera_serials) if self.camera_serials else 1

    @property
    def enable_tactile(self) -> bool:
        return any((
            self.enable_tactile_marker,
            self.enable_tactile_force,
            self.enable_tactile_rectify,
            self.enable_tactile_difference,
            self.enable_tactile_depth,
        ))

    @property
    def enable_xense(self) -> bool:
        return self.enable_gripper_width or self.enable_tactile

    @property
    def robot_fields(self) -> tuple[str, ...]:
        fields: list[str] = []
        if self.enable_robot_joint_state:
            fields.extend(("q", "dq"))
        if self.enable_robot_torque:
            fields.extend(("tau", "tau_ext", "tau_interact"))
        if self.enable_robot_tcp:
            fields.extend(("tcp_pose", "tcp_vel"))
        if self.enable_robot_wrench_tcp:
            fields.append("ext_wrench_in_tcp")
        if self.enable_robot_wrench_world:
            fields.append("ext_wrench_in_world")
        return tuple(fields)
