#!/usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def parse_link_filter(filter_string):
    """
    Convert comma-separated link names into a set.

    Example:
      "saw,handbase,pitch_joint_link"
    becomes:
      {"saw", "handbase", "pitch_joint_link"}
    """
    if filter_string is None:
        return set()

    filter_string = filter_string.strip()

    if filter_string == "":
        return set()

    return set([
        name.strip()
        for name in filter_string.split(",")
        if name.strip() != ""
    ])


def get_inertial_xyz_rpy(inertial):
    """
    Get inertial origin xyz/rpy.

    In URDF:
      <inertial>
        <origin xyz="..." rpy="..."/>
      </inertial>

    xyz = CoG position relative to link frame.
    rpy = inertial frame rotation relative to link frame.
    """
    if inertial.origin is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

    xyz = inertial.origin.xyz if inertial.origin.xyz is not None else [0.0, 0.0, 0.0]
    rpy = inertial.origin.rpy if inertial.origin.rpy is not None else [0.0, 0.0, 0.0]

    return xyz, rpy


def make_sphere_marker(marker_id, frame_id, xyz, scale=0.025):
    """
    Red sphere marker showing inertial origin / CoG.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "inertial_origin"
    marker.id = marker_id

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = xyz[0]
    marker.pose.position.y = xyz[1]
    marker.pose.position.z = xyz[2]
    marker.pose.orientation.w = 1.0

    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker


def make_text_marker(marker_id, frame_id, link_name, xyz, rpy, mass, inertia):
    """
    White text marker showing mass, CoG, and inertia values.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "inertia_text"
    marker.id = marker_id

    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.pose.position.x = xyz[0]
    marker.pose.position.y = xyz[1]
    marker.pose.position.z = xyz[2] + 0.06
    marker.pose.orientation.w = 1.0

    marker.scale.z = 0.025

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker.text = (
        f"{link_name}\n"
        f"mass = {mass:.6f} kg\n"
        f"CoG xyz = [{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}]\n"
        f"CoG rpy = [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]\n"
        f"Ixx = {inertia.ixx:.6e}\n"
        f"Ixy = {inertia.ixy:.6e}\n"
        f"Ixz = {inertia.ixz:.6e}\n"
        f"Iyy = {inertia.iyy:.6e}\n"
        f"Iyz = {inertia.iyz:.6e}\n"
        f"Izz = {inertia.izz:.6e}"
    )

    return marker


def make_axis_marker(marker_id, frame_id, xyz, axis_length=0.06):
    """
    Yellow small axis marker at inertial origin.

    This simple version shows axes aligned with the link frame.
    It does not rotate by inertial rpy.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "inertial_axes"
    marker.id = marker_id

    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.004

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    p0 = Point()
    p0.x = xyz[0]
    p0.y = xyz[1]
    p0.z = xyz[2]

    px = Point()
    px.x = xyz[0] + axis_length
    px.y = xyz[1]
    px.z = xyz[2]

    py = Point()
    py.x = xyz[0]
    py.y = xyz[1] + axis_length
    py.z = xyz[2]

    pz = Point()
    pz.x = xyz[0]
    pz.y = xyz[1]
    pz.z = xyz[2] + axis_length

    marker.points.append(p0)
    marker.points.append(px)

    marker.points.append(p0)
    marker.points.append(py)

    marker.points.append(p0)
    marker.points.append(pz)

    return marker


def main():
    rospy.init_node("urdf_inertia_markers")

    # Default robot description parameter.
    # If your robot description is namespaced, run:
    #   rosrun gimbalrotor urdf_inertia_markers.py _robot_description_param:=/gimbalrotor/robot_description
    robot_description_param = rospy.get_param("~robot_description_param", "/robot_description")

    # TF prefix.
    # In your RViz RobotModel, TF Prefix is gimbalrotor.
    # So run:
    #   rosrun gimbalrotor urdf_inertia_markers.py _tf_prefix:=gimbalrotor
    tf_prefix = rospy.get_param("~tf_prefix", "")
    if tf_prefix != "" and not tf_prefix.endswith("/"):
        tf_prefix += "/"

    # Choose links to visualize.
    #
    # Example:
    #   _only_links:=saw
    #
    # Example:
    #   _only_links:=saw,pitch_joint_link,hand_assem_link
    #
    # If empty, visualize all links with inertial tags.
    only_links_param = rospy.get_param("~only_links", "")
    only_links = parse_link_filter(only_links_param)

    if len(only_links) > 0:
        rospy.loginfo("Only visualizing links: %s", sorted(list(only_links)))
    else:
        rospy.loginfo("Visualizing all inertial links")

    if not rospy.has_param(robot_description_param):
        rospy.logerr("Param %s does not exist.", robot_description_param)
        rospy.logerr("Available params containing 'description':")

        for param_name in rospy.get_param_names():
            if "description" in param_name:
                rospy.logerr("  %s", param_name)

        return

    rospy.loginfo("Loading URDF from param: %s", robot_description_param)

    robot = URDF.from_parameter_server(robot_description_param)

    rospy.loginfo("Loaded robot: %s", robot.name)
    rospy.loginfo("Number of links: %d", len(robot.links))
    rospy.loginfo("Number of joints: %d", len(robot.joints))
    rospy.loginfo("Using TF prefix: '%s'", tf_prefix)

    pub = rospy.Publisher(
        "/urdf/inertia_markers",
        MarkerArray,
        queue_size=1,
        latch=True
    )

    marker_array = MarkerArray()
    marker_id = 0

    found_filtered_links = set()

    for link in robot.links:
        if link.inertial is None:
            continue

        # If _only_links is specified, skip everything not in that list.
        if len(only_links) > 0 and link.name not in only_links:
            continue

        found_filtered_links.add(link.name)

        inertial = link.inertial
        xyz, rpy = get_inertial_xyz_rpy(inertial)

        mass = inertial.mass
        inertia = inertial.inertia

        frame_name = tf_prefix + link.name

        sphere = make_sphere_marker(
            marker_id=marker_id,
            frame_id=frame_name,
            xyz=xyz,
            scale=0.025
        )
        marker_id += 1
        marker_array.markers.append(sphere)

        text = make_text_marker(
            marker_id=marker_id,
            frame_id=frame_name,
            link_name=link.name,
            xyz=xyz,
            rpy=rpy,
            mass=mass,
            inertia=inertia
        )
        marker_id += 1
        marker_array.markers.append(text)

        axis = make_axis_marker(
            marker_id=marker_id,
            frame_id=frame_name,
            xyz=xyz,
            axis_length=0.06
        )
        marker_id += 1
        marker_array.markers.append(axis)

        rospy.loginfo("Added markers for link: %s frame: %s", link.name, frame_name)

        if link.name == "saw":
            rospy.loginfo("========== SAW INERTIAL ==========")
            rospy.loginfo("frame_id = %s", frame_name)
            rospy.loginfo("mass = %.6f kg", mass)
            rospy.loginfo("CoG xyz = [%.6f, %.6f, %.6f]", xyz[0], xyz[1], xyz[2])
            rospy.loginfo("CoG rpy = [%.6f, %.6f, %.6f]", rpy[0], rpy[1], rpy[2])
            rospy.loginfo(
                "I = [[%.6e, %.6e, %.6e], [%.6e, %.6e, %.6e], [%.6e, %.6e, %.6e]]",
                inertia.ixx, inertia.ixy, inertia.ixz,
                inertia.ixy, inertia.iyy, inertia.iyz,
                inertia.ixz, inertia.iyz, inertia.izz
            )
            rospy.loginfo("==================================")

    # Warn if user requested a link name that was not found.
    if len(only_links) > 0:
        missing_links = only_links - found_filtered_links
        if len(missing_links) > 0:
            rospy.logwarn("Requested links not found or have no inertial tag: %s",
                          sorted(list(missing_links)))

    rospy.loginfo("Publishing %d inertia markers on /urdf/inertia_markers",
                  len(marker_array.markers))

    if len(marker_array.markers) == 0:
        rospy.logwarn("No markers were created. Check _only_links names or inertial tags.")

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        now = rospy.Time.now()

        for marker in marker_array.markers:
            marker.header.stamp = now

        pub.publish(marker_array)
        rate.sleep()


if __name__ == "__main__":
    main()
