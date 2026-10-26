# Chạy ROS 2 trên Raspberry Pi bằng BlueOS Extension

Hướng dẫn cho **cách B** trong README chính (mục 7.2): MAVROS và safety_node chạy ngay trên Pi của robot, laptop điều khiển chỉ chạy `topside.launch.py`.

BlueOS chạy mọi dịch vụ trong Docker. Vì vậy không cài ROS trực tiếp lên Pi; ROS được đóng gói thành một **Extension** (một container Docker do nhóm tự build) và BlueOS chạy nó bên cạnh các dịch vụ của BlueOS.

Nếu chỉ cần lái robot, dùng **cách A** (ROS trên laptop) đơn giản hơn và không cần thư mục này.

## 0. Kiểm tra Pi chạy 64-bit (bắt buộc)

ROS 2 Humble chỉ có bản cho **arm64**. Trong BlueOS → Tools → **Terminal**, chạy `uname -m`:

| Kết quả | Ý nghĩa |
|---|---|
| `aarch64` | Kernel 64-bit, tiếp tục được |
| `armv7l` | 32-bit. Cần flash BlueOS bản 64-bit (ARMv8). Bản 64-bit được làm cho Pi 5; trên Pi 4 Blue Robotics chưa hỗ trợ chính thức |

## 1. Tạo endpoint MAVLink cho MAVROS (một lần)

BlueOS → **MAVLink Endpoints** → **+**:

| Trường | Giá trị |
|---|---|
| Name | `cps5-mavros` |
| Type | `UDP Client` |
| IP | `127.0.0.1` |
| Port | `14555` |

`real.launch.py` mặc định dùng `fcu_url = udp://:14555@`, khớp với endpoint này.

## 2. Build image (trên máy có Docker)

Ở thư mục gốc của repo:

```bash
docker buildx create --use            # một lần
docker buildx build --platform linux/arm64 -f docker/Dockerfile \
    -t <dockerhub_user>/cps5-ros:0.1 --push .
```

`<dockerhub_user>` là tài khoản Docker Hub của nhóm. Mỗi lần đổi code chạy trên robot, tăng số phiên bản (tag) và cập nhật `LABEL version` trong `Dockerfile`.

## 3. Cài vào BlueOS

BlueOS → **Extensions** → tab **Installed** → **+** (cài thủ công):

| Trường | Giá trị |
|---|---|
| Extension Identifier | `cps5.ros` |
| Extension Name | `CPS5 ROS` |
| Docker image | `<dockerhub_user>/cps5-ros` |
| Docker tag | `0.1` |
| Original Settings | JSON trong `LABEL permissions` của `docker/Dockerfile` |

Mở log của Extension trong trang Extensions; MAVROS phải báo `CON: Got HEARTBEAT, connected`.

## 4. Lưu ý khi lái

- **Chỉ một nguồn lái:** không lái bằng QGroundControl (joystick) và ROS cùng lúc; cả hai đều là GCS sysid 255 nên lệnh sẽ đè nhau. Khi lái bằng ROS, tắt joystick trong QGroundControl (Vehicle Setup → Joystick → bỏ Enable).
- **Camera:** BlueOS giữ camera cho luồng video (Video Streams), xem video bằng QGroundControl. Chỉ bật `camera:=true` cho một camera mà BlueOS không dùng.
- **Laptop:** chạy `ros2 launch rov_bringup topside.launch.py`, cùng `ROS_DOMAIN_ID` với container (mặc định 0).
