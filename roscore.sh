#!/bin/bash
docker exec -it $(docker ps -q) /ros_entrypoint.sh roscore
