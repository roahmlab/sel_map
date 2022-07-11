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

echo_and_run cd "/home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_mesh"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/usr/local/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/usr/local/lib/python3/dist-packages:/home/firefly/ROAHM/sel_map_ws/src/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/firefly/ROAHM/sel_map_ws/src" \
    "/usr/bin/python3" \
    "/home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_mesh/setup.py" \
     \
    build --build-base "/home/firefly/ROAHM/sel_map_ws/src/sel_map/sel_map_mesh" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/usr/local" --install-scripts="/usr/local/bin"
