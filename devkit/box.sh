#!/bin/bash
# The box's main process: the web desktop when this is the gui image, then wait forever.
# feb creates the container with this as the entrypoint; everything else is `docker exec`.
if [ -x /home/autodrive_devkit/desktop.sh ]; then
    source /home/autodrive_devkit/desktop.sh
fi
exec sleep infinity
