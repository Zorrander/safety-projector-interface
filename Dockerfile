# Stage 1: Build Stage
FROM osrf/ros:noetic-desktop-full AS build

WORKDIR /catkin_ws

COPY . /catkin_ws/src

RUN rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Stage 2: Runtime Stage
FROM osrf/ros:noetic-desktop-full

WORKDIR /catkin_ws

COPY --from=build /catkin_ws/devel /catkin_ws/devel
COPY --from=build /catkin_ws/src /catkin_ws/src

RUN apt-get update && apt-get install -y \
    python3-opencv 

RUN chmod +x /catkin_ws/src/entrypoint.sh

ENTRYPOINT ["/catkin_ws/src/entrypoint.sh"]
CMD ["roslaunch", "tuni_whitegoods", "hmi.launch"]
