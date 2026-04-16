#!/usr/bin/env python3
"""
carla_live_publisher.py — CARLA 0.9.15 to ROS2 Real-Time Sensor Bridge

Bridges CARLA synchronous simulation to a multi-sensor SLAM pipeline:
  - FAST-LIO2  (LiDAR + IMU odometry)
  - VINS-Fusion (Mono camera + IMU visual-inertial odometry)
  - robot_localization (GPS / multi-source EKF fusion)

This node is the /clock server for the entire ROS2 graph.
All downstream nodes MUST launch with  use_sim_time:=true.
This node itself does NOT use sim time — it IS the time source.

Usage:
    conda activate carla_env
    python3 carla_live_publisher.py

    # With parameters:
    python3 carla_live_publisher.py --ros-args \
        -p carla_host:=localhost -p carla_port:=2000 \
        -p town:=Town10HD -p vehicle_filter:=vehicle.tesla.model3
"""

import os
import sys

# Re-exec under carla_env python if the current interpreter lacks the `carla`
# module. Colcon installs console_scripts with the shebang of whichever python
# ran setup.py (often /usr/bin/python3), which does not have `carla`. This
# guard makes the node robust regardless of how it was launched.
_CARLA_ENV_PY = '/home/haohua/miniconda3/envs/carla_env/bin/python'
if os.path.exists(_CARLA_ENV_PY) and sys.executable != _CARLA_ENV_PY:
    try:
        import carla  # noqa: F401
    except ModuleNotFoundError:
        os.execv(_CARLA_ENV_PY, [_CARLA_ENV_PY] + sys.argv)

import copy
import math
import signal
import queue

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import (
    Imu, PointCloud2, PointField, Image, CameraInfo,
    NavSatFix, NavSatStatus,
)
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge

import carla


# ════════════════════════════════════════════════════════════════
# Quaternion helper  (avoids tf_transformations / transforms3d dep)
# ════════════════════════════════════════════════════════════════

def _euler_to_quaternion(roll, pitch, yaw):
    """Euler angles (rad, extrinsic XYZ / intrinsic ZYX) → (x, y, z, w).

    Equivalent to  tf_transformations.quaternion_from_euler(r, p, y, 'sxyz').
    Derivation: Q = Rz(yaw) · Ry(pitch) · Rx(roll), then extract components.
    """
    cr, sr = math.cos(roll  * 0.5), math.sin(roll  * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw   * 0.5), math.sin(yaw   * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


# ════════════════════════════════════════════════════════════════
# CarlaLVISamBridge
# ════════════════════════════════════════════════════════════════

class CarlaLVISamBridge(Node):
    """ROS2 node that publishes CARLA sensor data for
    FAST-LIO2 + VINS-Fusion (mono) + GPS fusion."""

    # ── Camera intrinsics (FOV 90°, 1280×720) ──────────────────
    # fx = fy = width / (2 · tan(fov / 2))
    #        = 1280  / (2 · tan(45°))  = 640.0
    CAM_WIDTH = 1280
    CAM_HEIGHT = 720
    CAM_FX = 640.0
    CAM_FY = 640.0
    CAM_CX = 640.0   # width  / 2
    CAM_CY = 360.0   # height / 2

    # ── PointCloud2 field layout ────────────────────────────────
    # Fields: x(f32) y(f32) z(f32) intensity(f32) ring(u16) [pad2] time(f32)
    # = 24 bytes per point.  ring is UINT16 to match FAST-LIO2's
    # velodyne_ros::Point struct (preprocess.h).
    PC2_FIELDS = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring',      offset=16, datatype=PointField.UINT16,  count=1),
        PointField(name='time',      offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    PC2_POINT_STEP = 24

    # Structured dtype for numpy packing — matches the field layout above.
    # 2-byte pad after ring keeps time at offset 20.
    PC2_DTYPE = np.dtype([
        ('x',         '<f4'),
        ('y',         '<f4'),
        ('z',         '<f4'),
        ('intensity', '<f4'),
        ('ring',      '<u2'),
        ('_pad',      '<u2'),
        ('time',      '<f4'),
    ])

    # ── Sensor definitions ──────────────────────────────────────
    SENSOR_DEFS = {
        'imu': {
            'bp': 'sensor.other.imu',
            'transform': carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0)),
            'attrs': {'sensor_tick': '0.0'},            # every physics tick → 200 Hz
            'frame': 'imu_link',
        },
        'lidar': {
            'bp': 'sensor.lidar.ray_cast',
            'transform': carla.Transform(carla.Location(x=0.0, y=0.0, z=1.8)),
            'attrs': {
                'channels':           '64',
                # 1.3M/s at 200 Hz rotation gives only ~6.5k pts per full
                # scan. Boost to ~130k pts (VLP-64-like density).
                'points_per_second':  '26000000',
                # CARLA sync-mode quirk: each physics tick emits
                # (rotation_frequency * fixed_delta_seconds) fraction of a
                # rotation. To get a full 360° scan per delivery, spin the
                # LiDAR at physics rate (1/dt = 200 Hz). sensor_tick still
                # throttles the callback rate to 10 Hz.
                'rotation_frequency': '200',
                'range':              '100',
                'upper_fov':          '15',
                'lower_fov':          '-25',
                'sensor_tick':        '0.1',            # callback every 0.1 s → 10 Hz
            },
            'frame': 'lidar_link',
        },
        'camera': {
            'bp': 'sensor.camera.rgb',
            'transform': carla.Transform(carla.Location(x=0.5, y=0.0, z=1.5)),
            'attrs': {
                'image_size_x': '1280',
                'image_size_y': '720',
                'fov':          '90',
                'sensor_tick':  '0.1',                  # 10 Hz
            },
            'frame': 'camera_link',
        },
        'gnss': {
            'bp': 'sensor.other.gnss',
            'transform': carla.Transform(carla.Location(x=0.0, y=0.0, z=1.8)),
            'attrs': {'sensor_tick': '0.2'},            # 5 Hz
            'frame': 'gps_link',
        },
    }

    # ────────────────────────────────────────────────────────────
    # Construction
    # ────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('carla_lvi_sam_bridge')

        # ROS2 parameters (overridable from CLI / launch file)
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('town', 'Town10HD')
        self.declare_parameter('vehicle_filter', 'vehicle.tesla.model3')
        self.declare_parameter('fixed_delta_seconds', 0.005)

        # Internal state
        self._cv_bridge = CvBridge()
        self._vehicle = None
        self._sensors = []
        self._spectator = None
        self._original_settings = None
        self._running = True

        # Thread-safe queues — one per sensor, filled by CARLA callbacks
        self._queues = {name: queue.Queue() for name in self.SENSOR_DEFS}

        # Pre-build the static CameraInfo (constant every frame)
        self._camera_info_template = self._build_camera_info_template()

        # Initialise ROS publishers & TF, then CARLA world
        self._create_publishers()
        self._publish_static_transforms()
        self._setup_carla()
        self._spawn_vehicle()
        self._attach_sensors()

        self.get_logger().info('CarlaLVISamBridge ready — entering main loop')

    # ────────────────────────────────────────────────────────────
    # ROS Publishers
    # ────────────────────────────────────────────────────────────

    def _create_publishers(self):
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        large_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        gnss_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._pub_clock       = self.create_publisher(Clock,      '/clock',              clock_qos)
        self._pub_imu         = self.create_publisher(Imu,        '/imu/data',           best_effort)
        self._pub_lidar       = self.create_publisher(PointCloud2,'/lidar/points',       large_qos)
        self._pub_image       = self.create_publisher(Image,      '/camera/image_color', large_qos)
        self._pub_camera_info = self.create_publisher(CameraInfo, '/camera/camera_info', large_qos)
        self._pub_gnss        = self.create_publisher(NavSatFix,  '/gps/fix',            gnss_qos)

    # ────────────────────────────────────────────────────────────
    # Static TF: base_link → sensor frames
    # ────────────────────────────────────────────────────────────

    def _publish_static_transforms(self):
        broadcaster = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()

        tfs = []
        for cfg in self.SENSOR_DEFS.values():
            rx, ry, rz = self._carla_location_to_ros(cfg['transform'].location)
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'base_link'
            t.child_frame_id = cfg['frame']
            t.transform.translation.x = rx
            t.transform.translation.y = ry
            t.transform.translation.z = rz
            t.transform.rotation.w = 1.0             # identity rotation
            tfs.append(t)

        broadcaster.sendTransform(tfs)
        self._static_tf_broadcaster = broadcaster     # prevent GC
        self.get_logger().info(f'Published {len(tfs)} static transforms')

    # ────────────────────────────────────────────────────────────
    # CARLA world, vehicle, sensors
    # ────────────────────────────────────────────────────────────

    def _setup_carla(self):
        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        town = self.get_parameter('town').value
        dt   = self.get_parameter('fixed_delta_seconds').value

        self._client = carla.Client(host, port)
        self._client.set_timeout(60.0)               # town load can be slow

        self._world = self._client.load_world(town)
        self.get_logger().info(f'Loaded map: {town}')

        # Save original settings for cleanup
        self._original_settings = self._world.get_settings()

        # Enable synchronous mode
        settings = self._world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        self._world.apply_settings(settings)

        # Traffic manager must also run in sync
        self._tm = self._client.get_trafficmanager()
        self._tm.set_synchronous_mode(True)

        self.get_logger().info(
            f'Sync mode ON: dt={dt}s ({1.0 / dt:.0f} Hz physics)')

    def _spawn_vehicle(self):
        bp_filter = self.get_parameter('vehicle_filter').value
        bp_lib = self._world.get_blueprint_library()
        vehicle_bp = bp_lib.find(bp_filter)

        spawns = self._world.get_map().get_spawn_points()
        if not spawns:
            raise RuntimeError('No spawn points on this map')

        self._vehicle = self._world.spawn_actor(vehicle_bp, spawns[0])
        self._vehicle.set_autopilot(True, self._tm.get_port())

        # Keep the car moving continuously for SLAM demos (no red-light
        # waits). Stick to the speed limit so inter-scan motion stays
        # manageable for 10 Hz FAST-LIO (high speed = ICP struggles).
        self._tm.ignore_lights_percentage(self._vehicle, 100.0)
        self._tm.ignore_signs_percentage(self._vehicle, 100.0)
        self._tm.vehicle_percentage_speed_difference(self._vehicle, 0.0)

        self.get_logger().info(
            f'Spawned {bp_filter} at {spawns[0].location}  '
            f'(autopilot ON, ignoring lights/signs, speed-limit)')

        self._spectator = self._world.get_spectator()
        self._update_spectator()
        self.get_logger().info('Spectator will chase the ego vehicle (~8 m back, 3 m up)')

    def _update_spectator(self):
        """Move the CARLA spectator camera to a chase view behind the ego vehicle."""
        if self._spectator is None or self._vehicle is None:
            return
        vt = self._vehicle.get_transform()
        yaw_rad = math.radians(vt.rotation.yaw)
        back = 8.0
        up = 3.0
        cam_loc = carla.Location(
            x=vt.location.x - back * math.cos(yaw_rad),
            y=vt.location.y - back * math.sin(yaw_rad),
            z=vt.location.z + up,
        )
        cam_rot = carla.Rotation(pitch=-15.0, yaw=vt.rotation.yaw, roll=0.0)
        self._spectator.set_transform(carla.Transform(cam_loc, cam_rot))

    def _attach_sensors(self):
        bp_lib = self._world.get_blueprint_library()

        for name, cfg in self.SENSOR_DEFS.items():
            bp = bp_lib.find(cfg['bp'])
            for attr, val in cfg['attrs'].items():
                bp.set_attribute(attr, val)

            sensor = self._world.spawn_actor(
                bp, cfg['transform'], attach_to=self._vehicle)

            q = self._queues[name]
            sensor.listen(lambda data, _q=q: _q.put(data))
            self._sensors.append(sensor)
            self.get_logger().info(f'  Sensor attached: {name} ({cfg["bp"]})')

    # ════════════════════════════════════════════════════════════
    # Coordinate transforms — CARLA (Left-Handed) → ROS (Right-Handed)
    # ════════════════════════════════════════════════════════════
    #
    # CARLA:  X-forward,  Y-right,  Z-up   (Left-Handed)
    # ROS:    X-forward,  Y-left,   Z-up   (Right-Handed)
    #
    # The two frames share the XZ-plane but the Y-axis points in
    # opposite directions.  Every Y-component must therefore be
    # negated when converting positions, velocities, and
    # accelerations.
    #
    # Angular velocity is a *pseudo-vector* (axial vector).
    # Under a single-axis reflection (negating Y), the transform
    # rule is  ω' = det(M) · M · ω  where M = diag(1,−1,1).
    # det(M) = −1, so ω' = diag(−1,1,−1) · ω.
    # In practice, for CARLA→ROS the community convention (and
    # the empirically validated mapping) is:
    #     gyro_x →  gyro_x
    #     gyro_y → −gyro_y
    #     gyro_z → −gyro_z
    #
    # Rotations (Euler):
    #     ROS roll  =  CARLA roll   (converted deg → rad)
    #     ROS pitch = −CARLA pitch  (converted deg → rad)
    #     ROS yaw   = −CARLA yaw   (converted deg → rad)
    # ════════════════════════════════════════════════════════════

    @staticmethod
    def _seconds_to_ros_time(sec_float):
        """float seconds → builtin_interfaces.msg.Time"""
        sec = int(sec_float)
        nanosec = int((sec_float - sec) * 1e9)
        return Time(sec=sec, nanosec=nanosec)

    def _make_header(self, frame_id, timestamp):
        """Create a stamped Header from a CARLA elapsed-seconds timestamp."""
        h = Header()
        h.stamp = self._seconds_to_ros_time(timestamp)
        h.frame_id = frame_id
        return h

    @staticmethod
    def _carla_location_to_ros(loc):
        """CARLA Location (LH) → ROS position tuple (x, -y, z)."""
        return (loc.x, -loc.y, loc.z)

    @staticmethod
    def _carla_rotation_to_quat(rot):
        """CARLA Rotation (degrees, LH) → ROS quaternion (x, y, z, w)."""
        roll  =  math.radians(rot.roll)
        pitch = -math.radians(rot.pitch)
        yaw   = -math.radians(rot.yaw)
        return _euler_to_quaternion(roll, pitch, yaw)

    # ────────────────────────────────────────────────────────────
    # Message builders
    # ────────────────────────────────────────────────────────────

    def _build_imu_msg(self, data, ts):
        """CARLA IMUMeasurement → sensor_msgs/Imu."""
        msg = Imu()
        msg.header = self._make_header('imu_link', ts)

        # Ground-truth orientation from the sensor's world transform
        q = self._carla_rotation_to_quat(data.transform.rotation)
        msg.orientation.x, msg.orientation.y = q[0], q[1]
        msg.orientation.z, msg.orientation.w = q[2], q[3]

        # Angular velocity — negate Y and Z (pseudo-vector LH→RH)
        msg.angular_velocity.x =  data.gyroscope.x
        msg.angular_velocity.y = -data.gyroscope.y
        msg.angular_velocity.z = -data.gyroscope.z

        # Linear acceleration — negate Y only
        msg.linear_acceleration.x =  data.accelerometer.x
        msg.linear_acceleration.y = -data.accelerometer.y
        msg.linear_acceleration.z =  data.accelerometer.z

        # Diagnostic: log every ~1 s worth of IMU (at 200 Hz → every 200 frames)
        self._imu_log_ctr = getattr(self, '_imu_log_ctr', 0) + 1
        if self._imu_log_ctr % 200 == 0:
            self.get_logger().info(
                f'[IMU] accel=({msg.linear_acceleration.x:+.3f}, '
                f'{msg.linear_acceleration.y:+.3f}, '
                f'{msg.linear_acceleration.z:+.3f}) m/s²  '
                f'gyro=({msg.angular_velocity.x:+.4f}, '
                f'{msg.angular_velocity.y:+.4f}, '
                f'{msg.angular_velocity.z:+.4f}) rad/s')

        return msg

    def _build_pointcloud_msg(self, data, ts):
        """CARLA LidarMeasurement → sensor_msgs/PointCloud2.

        Adds synthetic *ring* (channel index, uint16) and *time*
        (per-point offset within the scan, float32) fields for
        FAST-LIO2 compatibility.  Uses a structured numpy array so
        that the ring field is packed as native uint16 matching
        FAST-LIO2's velodyne_ros::Point layout.
        """
        header = self._make_header('lidar_link', ts)

        # Parse raw data: 4 × float32 per point (x, y, z, intensity)
        raw = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
        n = raw.shape[0]
        if n == 0:
            return self._empty_pc2(header)

        # Allocate structured array (24 bytes/point, ring is uint16)
        cloud = np.zeros(n, dtype=self.PC2_DTYPE)
        cloud['x']         = raw[:, 0]
        cloud['y']         = raw[:, 1] * -1.0            # CARLA LH → ROS RH
        cloud['z']         = raw[:, 2]
        cloud['intensity'] = raw[:, 3]

        # Ring: channel index from per-channel point counts
        idx = 0
        for ch in range(data.channels):
            cnt = data.get_point_count(ch)
            cloud['ring'][idx:idx + cnt] = ch
            idx += cnt

        # Time offset: linearly spaced 0 → scan_period within one scan
        scan_period = 1.0 / float(
            self.SENSOR_DEFS['lidar']['attrs']['rotation_frequency'])
        cloud['time'] = np.linspace(0.0, scan_period, n, dtype=np.float32)

        msg = PointCloud2()
        msg.header      = header
        msg.height      = 1
        msg.width       = n
        msg.fields      = self.PC2_FIELDS
        msg.is_bigendian = False
        msg.point_step  = self.PC2_POINT_STEP
        msg.row_step    = self.PC2_POINT_STEP * n
        msg.is_dense    = True
        msg.data        = cloud.tobytes()

        # Diagnostic: log cloud stats every 50 scans (~5 s at 10 Hz)
        self._pc_log_ctr = getattr(self, '_pc_log_ctr', 0) + 1
        if self._pc_log_ctr % 50 == 1:
            x = cloud['x']; y = cloud['y']; z = cloud['z']
            finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            self.get_logger().info(
                f'[LiDAR] n={n} finite={finite.sum()} '
                f'x[{x.min():+.1f},{x.max():+.1f}] '
                f'y[{y.min():+.1f},{y.max():+.1f}] '
                f'z[{z.min():+.1f},{z.max():+.1f}] '
                f'ring[{cloud["ring"].min()},{cloud["ring"].max()}] '
                f't[{cloud["time"].min():.4f},{cloud["time"].max():.4f}]')
        return msg

    def _empty_pc2(self, header):
        msg = PointCloud2()
        msg.header = header
        msg.fields = self.PC2_FIELDS
        msg.point_step = self.PC2_POINT_STEP
        return msg

    def _build_image_msg(self, data, ts):
        """CARLA Image (BGRA8) → sensor_msgs/Image (BGR8)."""
        header = self._make_header('camera_link', ts)
        arr = np.frombuffer(data.raw_data, dtype=np.uint8)
        arr = arr.reshape(data.height, data.width, 4)
        bgr = np.ascontiguousarray(arr[:, :, :3])      # drop alpha
        return self._cv_bridge.cv2_to_imgmsg(bgr, encoding='bgr8',
                                              header=header)

    def _build_camera_info_template(self):
        """Pre-build the CameraInfo (everything except the stamp)."""
        msg = CameraInfo()
        msg.width  = self.CAM_WIDTH
        msg.height = self.CAM_HEIGHT
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [self.CAM_FX, 0.0, self.CAM_CX,
                  0.0, self.CAM_FY, self.CAM_CY,
                  0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        msg.p = [self.CAM_FX, 0.0, self.CAM_CX, 0.0,
                  0.0, self.CAM_FY, self.CAM_CY, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return msg

    def _stamp_camera_info(self, ts):
        """Return a stamped copy of the CameraInfo template."""
        msg = copy.deepcopy(self._camera_info_template)
        msg.header = self._make_header('camera_link', ts)
        return msg

    def _build_navsatfix_msg(self, data, ts):
        """CARLA GnssMeasurement → sensor_msgs/NavSatFix."""
        msg = NavSatFix()
        msg.header = self._make_header('gps_link', ts)
        msg.status.status  = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude  = data.latitude
        msg.longitude = data.longitude
        msg.altitude  = data.altitude
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg

    # ────────────────────────────────────────────────────────────
    # Main loop
    # ────────────────────────────────────────────────────────────

    def _drain_and_publish(self, current_frame, ts):
        """Drain every sensor queue; publish data that matches *current_frame*."""

        # (sensor_name, builder, publisher)
        dispatch = [
            ('imu',    self._build_imu_msg,        self._pub_imu),
            ('lidar',  self._build_pointcloud_msg,  self._pub_lidar),
            ('camera', self._build_image_msg,       self._pub_image),
            ('gnss',   self._build_navsatfix_msg,   self._pub_gnss),
        ]

        for name, builder, pub in dispatch:
            q = self._queues[name]
            latest = None
            # Drain: keep only the item matching current_frame, discard stale
            while True:
                try:
                    item = q.get_nowait()
                    if item.frame == current_frame:
                        latest = item
                except queue.Empty:
                    break

            if latest is not None:
                pub.publish(builder(latest, ts))
                # Publish CameraInfo in lockstep with each image
                if name == 'camera':
                    self._pub_camera_info.publish(self._stamp_camera_info(ts))

    def run(self):
        """Synchronous main loop — one world.tick() per iteration.

        Order within each tick:
            1. world.tick()      — advance simulation one step
            2. publish /clock    — BEFORE sensor data (critical!)
            3. drain & publish   — sensor messages for this frame
            4. spin_once         — process any pending ROS2 callbacks
        """
        tick = 0

        while self._running and rclpy.ok():
            try:
                # 1 — advance the simulation
                self._world.tick(10.0)
                snap = self._world.get_snapshot()
                sim_time = snap.timestamp.elapsed_seconds
                frame    = snap.frame

                # 2 — publish clock FIRST so downstream sim-time users advance
                clock_msg = Clock()
                clock_msg.clock = self._seconds_to_ros_time(sim_time)
                self._pub_clock.publish(clock_msg)

                # 3 — publish sensor data for this frame
                self._drain_and_publish(frame, sim_time)

                # 3b — chase the ego vehicle with the spectator camera
                self._update_spectator()

                # 4 — service any pending ROS2 callbacks (param changes, etc.)
                rclpy.spin_once(self, timeout_sec=0)

                tick += 1
                if tick % 200 == 0:                     # ~1 s of sim time
                    self.get_logger().info(
                        f'tick {tick}  sim={sim_time:.3f}s')

            except RuntimeError as exc:
                self.get_logger().error(f'Tick failed: {exc}')
                break

    # ────────────────────────────────────────────────────────────
    # Cleanup
    # ────────────────────────────────────────────────────────────

    def cleanup(self):
        """Destroy all CARLA actors and restore the world to async mode."""
        self._running = False
        self.get_logger().info('Cleaning up CARLA resources …')

        for sensor in self._sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except Exception as exc:
                self.get_logger().warn(f'Sensor destroy: {exc}')
        self._sensors.clear()

        if self._vehicle is not None:
            try:
                self._vehicle.destroy()
            except Exception:
                pass
            self._vehicle = None

        if self._original_settings is not None:
            try:
                self._world.apply_settings(self._original_settings)
                self._tm.set_synchronous_mode(False)
            except Exception:
                pass

        self.get_logger().info('Cleanup complete')


# ════════════════════════════════════════════════════════════════
# Entry point
# ════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = CarlaLVISamBridge()

    def _shutdown(sig, _frame):
        node.get_logger().info(f'Signal {sig} received — shutting down')
        node.cleanup()
        rclpy.try_shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
