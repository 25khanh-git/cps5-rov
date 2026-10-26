# =============================================================================
# CPS 5 — BlueOS Extension: ROS 2 Humble + MAVROS + rov_bringup/rov_safety trên Raspberry Pi 4.
#
# Build trên laptop (ở thư mục gốc repo), cho Pi 4 (arm64):
#   docker buildx build --platform linux/arm64 -f docker/Dockerfile -t <dockerhub_user>/cps5-ros:0.1 --push .
# Cài vào BlueOS: xem docker/README.md.
# =============================================================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-mavros ros-humble-mavros-extras ros-humble-v4l2-camera \
        python3-colcon-common-extensions python3-yaml \
    && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# Chỉ các package chạy trên robot (không cần Gazebo / sim).
WORKDIR /ws
COPY src/rov_bringup src/rov_bringup
COPY src/rov_safety src/rov_safety
COPY src/rov_teleop src/rov_teleop
RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-select rov_bringup rov_safety rov_teleop \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Cùng ROS_DOMAIN_ID với laptop chạy topside.launch.py.
ENV ROS_DOMAIN_ID=0

# ---- Metadata cho BlueOS Extension Manager ----------------------------------
LABEL version="0.1.0"
LABEL type="device-integration"
LABEL tags='["ros2", "mavros", "rov"]'
LABEL authors='[{"name": "Nguyen Duy Khanh"}]'
LABEL company='{"about": "CPS 5 ROV", "name": "CPS5"}'
LABEL readme=""
LABEL links='{}'
LABEL requirements="core >= 1.1"
# network host: MAVROS nhận UDP 14555 từ BlueOS và ROS 2 DDS ra được tether.
LABEL permissions='{\
  "NetworkMode": "host",\
  "HostConfig": {\
    "NetworkMode": "host",\
    "Privileged": true,\
    "Binds": ["/dev:/dev"]\
  }\
}'

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rov_bringup", "real.launch.py"]
