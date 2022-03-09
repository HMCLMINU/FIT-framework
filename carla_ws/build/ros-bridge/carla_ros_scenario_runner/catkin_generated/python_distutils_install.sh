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

echo_and_run cd "/home/autoware/carla_ws/src/ros-bridge/carla_ros_scenario_runner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/autoware/carla_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/autoware/carla_ws/install/lib/python2.7/dist-packages:/home/autoware/carla_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/autoware/carla_ws/build" \
    "/usr/bin/python2" \
    "/home/autoware/carla_ws/src/ros-bridge/carla_ros_scenario_runner/setup.py" \
     \
    build --build-base "/home/autoware/carla_ws/build/ros-bridge/carla_ros_scenario_runner" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/autoware/carla_ws/install" --install-scripts="/home/autoware/carla_ws/install/bin"