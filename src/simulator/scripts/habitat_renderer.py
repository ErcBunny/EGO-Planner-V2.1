import cv2
import habitat_sim
import math
import numpy as np
import open3d as o3d
import rospy
import std_msgs.msg
import threading
from cv_bridge import CvBridge
from habitat_sim.utils.common import quat_to_coeffs
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField, Image


def create_basic_camera_spec() -> habitat_sim.CameraSensorSpec:
    camera_spec = habitat_sim.CameraSensorSpec()
    camera_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    camera_spec.resolution = [
        rospy.get_param("~image_height"),
        rospy.get_param("~image_width"),
    ]
    camera_spec.hfov = rospy.get_param("~image_hfov")
    camera_spec.position = [
        -rospy.get_param("~camera_position_y"),
        +rospy.get_param("~camera_position_z"),
        -rospy.get_param("~camera_position_x"),
    ]
    camera_spec.orientation = [
        -rospy.get_param("~camera_orientation_y"),
        +rospy.get_param("~camera_orientation_z"),
        -rospy.get_param("~camera_orientation_x"),
    ]
    return camera_spec


def get_pcd_transform(sensor_pose_habitat: habitat_sim.agent.SixDOFPose) -> np.ndarray:
    rotation_world2cam_habitat = Rotation.from_quat(
        quat_to_coeffs(sensor_pose_habitat.rotation)
    )
    translation_world2cam_habitat = sensor_pose_habitat.position

    rotation_world_mesh2habitat = Rotation.from_matrix(
        np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    )
    rotation_cam_habitat2mesh = Rotation.from_matrix(
        np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    )

    # need to calculate
    # world2cam_mesh: world -> cam transform under mesh/o3d convention
    rotation_world2cam_mesh: Rotation = (
            rotation_world_mesh2habitat
            * rotation_world2cam_habitat
            * rotation_cam_habitat2mesh
    )

    translation_world2cam_mesh = rotation_world_mesh2habitat.apply(
        translation_world2cam_habitat
    )

    pcd_transform = np.zeros([4, 4])
    pcd_transform[-1, -1] = 1
    pcd_transform[:3, :3] = rotation_world2cam_mesh.as_matrix()
    pcd_transform[:3, -1] = translation_world2cam_mesh

    return pcd_transform


def pcd_msg_from_points_xyz(points: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = np.asarray(points, np.float32).tobytes()
    return msg


class HabitatRenderer:
    simulator: habitat_sim.Simulator
    agent: habitat_sim.Agent
    agent_type: str  # "aerial" or "terrestrial"
    terrestrial_operating_height: float
    crop_point_cloud_offset_gnd: float  # point cloud cropping only applies to a terrestrial agent
    crop_point_cloud_offset_top: float
    frame_id_mesh: str
    depth_img_trunc_meter: float
    fps: int
    o3d_camera_intrinsic: o3d.camera.PinholeCameraIntrinsic

    translation_world2body_mesh: np.ndarray
    rotation_world2body_mesh: Rotation
    rotation_world_habitat2mesh: Rotation
    rotation_body_mesh2habitat: Rotation
    mtx: threading.Lock

    color_img_raw_pub: rospy.Publisher
    depth_img_raw_pub: rospy.Publisher
    local_point_cloud_pub: rospy.Publisher
    local_point_cloud_aux_pub: rospy.Publisher

    def __init__(self) -> None:
        # sim config
        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.create_renderer = True
        sim_cfg.default_agent_id = 0
        sim_cfg.enable_physics = False
        sim_cfg.frustum_culling = True
        sim_cfg.gpu_device_id = rospy.get_param("~gpu_device_id")
        sim_cfg.scene_id = rospy.get_param("~scene_id")
        sim_cfg.scene_dataset_config_file = rospy.get_param(
            "~scene_dataset_config_file"
        )
        sim_cfg.use_semantic_textures = True

        # agent config
        agent_cfg = habitat_sim.AgentConfiguration()
        agent_cfg.height = rospy.get_param("~agent_height")
        agent_cfg.radius = rospy.get_param("~agent_radius")

        color_sensor_spec = create_basic_camera_spec()
        color_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
        color_sensor_spec.uuid = "color_sensor"

        depth_sensor_spec = create_basic_camera_spec()
        depth_sensor_spec.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor_spec.uuid = "depth_sensor"

        agent_cfg.sensor_specifications = [color_sensor_spec, depth_sensor_spec]

        self.agent_type = rospy.get_param("~agent_type")
        if self.agent_type == "aerial":
            depth_sensor_spec_top = create_basic_camera_spec()
            depth_sensor_spec_top.sensor_type = habitat_sim.SensorType.DEPTH
            depth_sensor_spec_top.uuid = "depth_sensor_top"
            depth_sensor_spec_top.orientation = [math.pi / 2, 0, 0]
            agent_cfg.sensor_specifications.append(depth_sensor_spec_top)

            depth_sensor_spec_bottom = create_basic_camera_spec()
            depth_sensor_spec_bottom.sensor_type = habitat_sim.SensorType.DEPTH
            depth_sensor_spec_bottom.uuid = "depth_sensor_bottom"
            depth_sensor_spec_bottom.orientation = [-math.pi / 2, 0, 0]
            agent_cfg.sensor_specifications.append(depth_sensor_spec_bottom)

        self.simulator = habitat_sim.Simulator(
            habitat_sim.Configuration(sim_cfg, [agent_cfg])
        )
        self.agent = self.simulator.initialize_agent(0)
        self.terrestrial_operating_height = rospy.get_param(
            "~terrestrial_operating_height"
        )
        self.crop_point_cloud_offset_gnd = rospy.get_param(
            "~crop_point_cloud_offset_gnd"
        )
        self.crop_point_cloud_offset_top = rospy.get_param(
            "~crop_point_cloud_offset_top"
        )
        self.frame_id_mesh = rospy.get_param("~frame_id_mesh")
        self.depth_img_trunc_meter = rospy.get_param("~depth_img_trunc_meter")
        self.fps = rospy.get_param("~fps")

        # camera intrinsic
        hfov_rad = math.radians(rospy.get_param("~image_hfov"))
        w = int(rospy.get_param("~image_width"))
        h = int(rospy.get_param("~image_height"))
        fx = w / (2 * math.tan(hfov_rad / 2))
        fy = fx
        cx = w / 2
        cy = h / 2
        self.o3d_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            w, h, fx, fy, cx, cy
        )

        # subscribe to odom topic and prepare transformation
        self.rotation_world_habitat2mesh = Rotation.from_matrix(
            np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        )
        self.rotation_body_mesh2habitat = Rotation.from_matrix(
            np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        )
        self.translation_world2body_mesh = np.array([0, 0, 0])
        self.rotation_world2body_mesh = Rotation.from_quat([0, 0, 0, 1])
        self.mtx = threading.Lock()
        rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry, self.odom_sub_cb)

        # publishers
        self.color_img_raw_pub = rospy.Publisher(
            rospy.get_param("~color_image_raw_topic"), Image, queue_size=1
        )
        self.depth_img_raw_pub = rospy.Publisher(
            rospy.get_param("~depth_image_raw_topic"), Image, queue_size=1
        )
        self.local_point_cloud_pub = rospy.Publisher(
            rospy.get_param("~local_pcd_topic"), PointCloud2, queue_size=1
        )
        self.local_point_cloud_aux_pub = rospy.Publisher(
            rospy.get_param("~local_pcd_aux_topic"), PointCloud2, queue_size=1
        )

    def odom_sub_cb(self, msg: Odometry) -> None:
        self.mtx.acquire()
        self.translation_world2body_mesh = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )
        self.rotation_world2body_mesh = Rotation.from_quat(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )
        self.mtx.release()

    def get_pose_wrt_habitat_frame(self) -> (np.array, np.array):
        self.mtx.acquire()
        translation_world2body_mesh = self.translation_world2body_mesh
        rotation_world2body_mesh = self.rotation_world2body_mesh
        self.mtx.release()

        translation_world2body_habitat = self.rotation_world_habitat2mesh.apply(
            translation_world2body_mesh
        )
        rotation_world2body_habitat = (
                self.rotation_world_habitat2mesh
                * rotation_world2body_mesh
                * self.rotation_body_mesh2habitat
        )
        return translation_world2body_habitat, rotation_world2body_habitat.as_quat(
            canonical=False
        )

    def render_and_publish(self) -> None:
        # set agent state
        agent_state = habitat_sim.AgentState()
        agent_state.position, agent_state.rotation = self.get_pose_wrt_habitat_frame()
        self.agent.set_state(agent_state)

        # query observations from habitat
        observations = self.simulator.get_sensor_observations(0)

        # process rgb-d
        rgba = observations["color_sensor"]
        rgb = cv2.cvtColor(rgba, cv2.COLOR_RGBA2RGB)
        o3d_color = o3d.geometry.Image(rgb)
        depth = observations["depth_sensor"]

        o3d_depth = o3d.geometry.Image(depth)
        o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color=o3d_color,
            depth=o3d_depth,
            depth_scale=1,
            depth_trunc=self.depth_img_trunc_meter,
            convert_rgb_to_intensity=False,
        )
        d = np.rint(np.asarray(o3d_rgbd.depth) * 1000)

        # process local point cloud
        o3d_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            image=o3d_rgbd, intrinsic=self.o3d_camera_intrinsic, extrinsic=np.eye(4)
        ).transform(
            get_pcd_transform(self.agent.get_state().sensor_states["color_sensor"])
        )

        # process additional depth images for aerial agents
        if self.agent_type == "aerial":
            depth_top = observations["depth_sensor_top"]
            depth_bottom = observations["depth_sensor_bottom"]

            o3d_depth_top = o3d.geometry.Image(depth_top)
            o3d_depth_bottom = o3d.geometry.Image(depth_bottom)

            o3d_pcd_top = o3d.geometry.PointCloud.create_from_depth_image(
                depth=o3d_depth_top, intrinsic=self.o3d_camera_intrinsic, extrinsic=np.eye(4)
            ).transform(
                get_pcd_transform(self.agent.get_state().sensor_states["depth_sensor_top"])
            )

            o3d_pcd_bottom = o3d.geometry.PointCloud.create_from_depth_image(
                depth=o3d_depth_bottom, intrinsic=self.o3d_camera_intrinsic, extrinsic=np.eye(4)
            ).transform(
                get_pcd_transform(self.agent.get_state().sensor_states["depth_sensor_bottom"])
            )

            o3d_pcd = o3d_pcd + o3d_pcd_top + o3d_pcd_bottom

        o3d_pcd_points = np.asarray(o3d_pcd.points)

        o3d_pcd_points_flat = None
        if self.agent_type == "terrestrial":
            pcd_crop_min = (
                    self.terrestrial_operating_height + self.crop_point_cloud_offset_gnd
            )
            pcd_crop_max = (
                    self.terrestrial_operating_height
                    + self.agent.agent_config.height
                    + self.crop_point_cloud_offset_top
            )
            o3d_pcd_points_flat = o3d_pcd_points[
                (o3d_pcd_points[:, 2] >= pcd_crop_min)
                & (o3d_pcd_points[:, 2] <= pcd_crop_max)
                ]
            o3d_pcd_points_flat[:, 2] = self.terrestrial_operating_height

        # create msg and publish
        timestamp = rospy.Time.now()
        common_header = std_msgs.msg.Header()
        common_header.stamp = timestamp

        rgb_msg = CvBridge().cv2_to_imgmsg(rgb.astype(np.uint8), encoding="rgb8")
        rgb_msg.header = common_header

        d_msg = CvBridge().cv2_to_imgmsg(d.astype(np.uint16), encoding="16UC1")
        d_msg.header = common_header

        pcd_msg = pcd_msg_from_points_xyz(o3d_pcd_points)
        pcd_msg.header = common_header
        pcd_msg.header.frame_id = self.frame_id_mesh

        self.color_img_raw_pub.publish(rgb_msg)
        self.depth_img_raw_pub.publish(d_msg)

        if self.agent_type == "terrestrial":
            pcd_flat_msg = pcd_msg_from_points_xyz(o3d_pcd_points_flat)
            pcd_flat_msg.header = common_header
            pcd_flat_msg.header.frame_id = self.frame_id_mesh
            self.local_point_cloud_pub.publish(pcd_flat_msg)

            self.local_point_cloud_aux_pub.publish(pcd_msg)
        else:
            self.local_point_cloud_pub.publish(pcd_msg)


if __name__ == "__main__":
    rospy.init_node("habitat_renderer")
    renderer = HabitatRenderer()
    rate = rospy.Rate(renderer.fps)
    cv2.setNumThreads(0)
    while not rospy.is_shutdown():
        renderer.render_and_publish()
        rate.sleep()
