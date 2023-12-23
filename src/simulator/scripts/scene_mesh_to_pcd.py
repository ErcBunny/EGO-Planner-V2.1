import numpy as np
import open3d as o3d
import rospy
import std_msgs
from sensor_msgs.msg import PointCloud2, PointField

if __name__ == "__main__":
    rospy.init_node("habitat_scene_mesh_to_pcd")

    # o3d read mesh and sample points from it as the global point cloud
    scene_mesh = o3d.io.read_triangle_mesh(rospy.get_param("~scene_id"))
    pcd = scene_mesh.sample_points_uniformly(
        int(rospy.get_param("~mesh_sampling_points"))
    )
    pcd_down = pcd.voxel_down_sample(
        voxel_size=rospy.get_param("~global_pcd_downsample_voxel_size")
    )
    global_point_cloud_points = np.asarray(pcd_down.points)
    rospy.loginfo(
        "Created a global point cloud with %d points.",
        global_point_cloud_points.shape[0],
    )

    # register ROS I/O
    global_point_cloud_pub = rospy.Publisher(
        rospy.get_param("~global_pcd_publish_topic"), PointCloud2, queue_size=1
    )

    # create pcd msg
    msg = PointCloud2()

    msg.header = std_msgs.msg.Header()
    msg.header.frame_id = rospy.get_param("~global_pcd_reference_frame")
    msg.header.stamp = rospy.Time.now()

    msg.height = 1
    msg.width = len(global_point_cloud_points)

    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]

    msg.is_bigendian = False
    msg.is_dense = True

    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width

    msg.data = np.asarray(global_point_cloud_points, np.float32).tobytes()

    # publish cloud at 1 hz
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        global_point_cloud_pub.publish(msg)
        rate.sleep()
