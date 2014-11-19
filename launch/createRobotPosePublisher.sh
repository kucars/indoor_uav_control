#!/bin/bash
rosservice call /add_robot_tracker '{robot_name: 'uav', origin_marker_id: \"508\", x_marker_id: \"507\", y_marker_id: \"9\"}' &
exec "$@"
