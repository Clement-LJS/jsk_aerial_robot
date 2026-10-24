#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_base"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/lib/python3/dist-packages:/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base" \
    "/usr/bin/python3" \
    "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_base/setup.py" \
     \
    build --build-base "/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/base" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install" --install-scripts="/home/clement/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/.roll_yaw_verify/install/bin"
