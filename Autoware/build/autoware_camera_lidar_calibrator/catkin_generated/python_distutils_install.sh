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

echo_and_run cd "/home/autoware/Autoware/src/autoware/utilities/autoware_camera_lidar_calibrator"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/autoware/Autoware/install/autoware_camera_lidar_calibrator/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/autoware/Autoware/install/autoware_camera_lidar_calibrator/lib/python2.7/dist-packages:/home/autoware/Autoware/build/autoware_camera_lidar_calibrator/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/autoware/Autoware/build/autoware_camera_lidar_calibrator" \
    "/usr/bin/python2" \
    "/home/autoware/Autoware/src/autoware/utilities/autoware_camera_lidar_calibrator/setup.py" \
     \
    build --build-base "/home/autoware/Autoware/build/autoware_camera_lidar_calibrator" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/autoware/Autoware/install/autoware_camera_lidar_calibrator" --install-scripts="/home/autoware/Autoware/install/autoware_camera_lidar_calibrator/bin"
