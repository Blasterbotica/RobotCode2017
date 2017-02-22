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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/namwob44/turtlebot/src/turtlebot_create_desktop/create_dashboard"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/namwob44/turtlebot/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/namwob44/turtlebot/install/lib/python2.7/dist-packages:/home/namwob44/turtlebot/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/namwob44/turtlebot/build" \
    "/usr/bin/python" \
    "/home/namwob44/turtlebot/src/turtlebot_create_desktop/create_dashboard/setup.py" \
    build --build-base "/home/namwob44/turtlebot/build/turtlebot_create_desktop/create_dashboard" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/namwob44/turtlebot/install" --install-scripts="/home/namwob44/turtlebot/install/bin"
