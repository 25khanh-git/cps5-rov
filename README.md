# CPS5 — Phần mềm điều khiển ROV 8 thruster

Repo chứa toàn bộ phần mềm của ROV CPS5: mô phỏng (sim), code ROS 2 để lái, và tham số ArduSub dùng chung cho sim và robot thật.

- **Phần cứng:** khung một ống kín, 8 thruster Blue Robotics T200 (frame ArduSub **Vectored-6DOF**), Pixhawk 6C chạy ArduSub 4.7.2, Raspberry Pi 4 chạy BlueOS, pin 3S (~12 V).
- **Phần mềm:** ROS 2 Humble, MAVROS, Gazebo Harmonic, ArduSub SITL.
- **Nguồn thông số:** vị trí thruster lấy từ CAD Onshape *one tube 8 thruster layout assembly*. Các số chưa đo được ghi ở mục [Trạng thái số liệu](#trạng-thái-số-liệu).

Sim dùng **đúng phần mềm chạy trên robot thật**: firmware ArduSub thật (biên dịch cho PC, gọi là SITL) và MAVROS. Vì vậy code ROS 2 kiểm tra trong sim chạy được trên robot mà không phải sửa, chỉ đổi kết nối (`fcu_url`).

## Mục lục

1. [Kiến trúc](#kiến-trúc)
2. [Thuật ngữ](#thuật-ngữ)
3. [Trạng thái số liệu](#trạng-thái-số-liệu)
4. [Cài đặt môi trường](#1-cài-đặt-môi-trường-làm-một-lần)
5. [Làm việc hằng ngày với sim](#2-làm-việc-hằng-ngày-với-sim)
6. [Sửa thông số ở đâu](#3-sửa-thông-số-ở-đâu)
7. [Thứ tự motor và hệ trục](#4-thứ-tự-motor-và-hệ-trục)
8. [Topic ROS chính](#5-topic-ros-chính)
9. [Kiểm tra tự động](#6-kiểm-tra-tự-động)
10. [Chạy robot thật](#7-chạy-robot-thật)
11. [Xử lý sự cố](#8-xử-lý-sự-cố)
12. [Cấu trúc repo](#9-cấu-trúc-repo)

## Kiến trúc

**Trong sim** (tất cả chạy trên laptop, trong WSL):

```
 bàn phím / tay cầm ─► teleop_node ──/mavros/rc/override──► MAVROS ◄─MAVLink UDP 14550─► ArduSub SITL
                          │ /rov/topside_heartbeat              ▲                          ▲ JSON UDP 9002
                          ▼                                     │                          ▼
                     safety_node ───── disarm / SURFACE ────────┘          Gazebo: ArduPilotPlugin → 8 thruster
                                                                           Buoyancy + Hydrodynamics (nước)
 rqt_image_view ◄── /camera/image_raw ◄── ros_gz_bridge ◄──────────────── camera, ground truth
```

**Trên robot thật:** ArduSub SITL được thay bằng Pixhawk 6C, Gazebo được thay bằng robot. MAVROS nhận MAVLink từ BlueOS qua UDP cổng 14555 (xem [mục 7](#7-chạy-robot-thật)).

```
 Laptop (WSL: teleop_node, safety_node, MAVROS; QGroundControl)
   │ tether Ethernet (laptop 192.168.2.1 — Pi 192.168.2.2)
 Raspberry Pi 4 (BlueOS: MAVLink router, video)
   │ USB
 Pixhawk 6C (ArduSub 4.7.2) ──► 8 ESC ──► 8 thruster T200
```

## Thuật ngữ

| Từ dùng trong repo | Nghĩa |
|---|---|
| **sim** | Mô phỏng: Gazebo (vật lý, nước) + ArduSub SITL (firmware chạy trên PC) |
| **SITL** | Software In The Loop: firmware ArduSub biên dịch để chạy trên PC thay cho Pixhawk |
| **robot thật** | ROV vật lý, Pixhawk 6C + Raspberry Pi 4 |
| **laptop điều khiển** (topside) | Máy trên bờ: chạy QGroundControl và các node ROS 2 |
| **Pi** | Raspberry Pi 4 trên robot, chạy BlueOS |
| **M1 … M8** | Số thứ tự motor theo ArduSub, ứng với cổng MAIN 1 … MAIN 8 của Pixhawk ([mục 4](#4-thứ-tự-motor-và-hệ-trục)) |
| **T200 <n>** | Tên thruster trong CAD Onshape (số trong ngoặc là số instance của CAD) |
| **hệ trục robot** | x hướng mũi, y sang trái, z lên (chuẩn ROS REP-103, còn gọi là FLU) |
| **ARM / DISARM** | Bật / tắt cho phép thruster chạy |
| **MANUAL / STABILIZE / ALT_HOLD** | Chế độ lái của ArduSub: lái tay hoàn toàn / tự giữ thăng bằng / tự giữ thăng bằng và độ sâu |
| **failsafe** | Phản ứng tự động khi mất lệnh lái, pin yếu, rò nước… |

## Trạng thái số liệu

Một số thông số trong sim vẫn là ước lượng. Khi đo được số thật, cập nhật vào file trong cột cuối và sửa bảng này.

| Thông số | Giá trị hiện tại | Nguồn | Độ tin cậy | File cần sửa |
|---|---|---|---|---|
| Vị trí, hướng 8 thruster | xem `cps5_vehicle.yaml` | CAD Onshape | chính xác | `cps5_vehicle.yaml` |
| Khối lượng | 7 kg | tạm tính (8 × T200 ≈ 2.8 kg) | cần cân | `cps5_vehicle.yaml` |
| Lực đẩy mỗi thruster | 30 N | datasheet T200 ở 12 V | cần đo bằng cân treo | `cps5_vehicle.yaml` |
| Quán tính | xem file | xấp xỉ hình hộp | ước lượng | `cps5_vehicle.yaml` |
| Lực cản, added mass | xem file | lấy theo BlueROV2 Heavy | ước lượng | `cps5_vehicle.yaml` |
| Tâm nổi so với trọng tâm | 2 cm | ước lượng | cần đo trong nước | `cps5_vehicle.yaml` |
| Ngưỡng pin | 10.5 V / 9.9 V | pin 3S | đúng nếu dùng 3S | `cps5.parm`, `safety.yaml` |

---

## 1. Cài đặt môi trường (làm một lần)

**Yêu cầu:** Windows 10/11 có WSL2 (Ubuntu chạy bên trong Windows), khoảng 15 GB trống, RAM từ 8 GB.

1. **Cài Ubuntu 22.04 trong WSL.** Mở PowerShell bằng quyền Admin:
   ```powershell
   wsl --install -d Ubuntu-22.04
   wsl --set-default Ubuntu-22.04
   ```
2. **Cài VS Code** và extension **WSL** (`ms-vscode-remote.remote-wsl`).
3. **Clone repo và mở trong WSL.** Mở VS Code trong WSL là bắt buộc, để IntelliSense tìm thấy thư viện ROS 2. Trong terminal Ubuntu (thay đường dẫn bằng vị trí repo trên máy):
   ```bash
   cd /mnt/d/Projects/CPS5     # ví dụ: repo nằm ở D:\Projects\CPS5 trên Windows
   code .
   ```
   Hoặc trong VS Code: `Ctrl+Shift+P` → **WSL: Open Folder in WSL…** → chọn thư mục repo.
4. **Bật cấu hình VS Code dùng chung** (task, settings, extension gợi ý): đổi tên thư mục `vscode` thành `.vscode` (`mv vscode .vscode`). Chi tiết trong [`vscode/README.md`](vscode/README.md).
5. **Cài môi trường:** `Terminal → Run Task… → CPS5: 1. Cài môi trường`, hoặc:
   ```bash
   bash scripts/setup_wsl.sh
   ```
   Script cài ROS 2 Humble, MAVROS, Gazebo Harmonic, build ArduSub SITL (**Sub-4.7.2**, phải cùng phiên bản với firmware trên Pixhawk) và ArduPilotPlugin, rồi build workspace. Mất 30–90 phút. Chạy lại an toàn: bước nào đã xong sẽ được bỏ qua.

## 2. Làm việc hằng ngày với sim

| Việc | VS Code (`Ctrl+Shift+P` → Run Task) | Terminal |
|---|---|---|
| Build | **Ctrl+Shift+B** | `bash scripts/build.sh` |
| Chạy sim | CPS5: 3. Chạy simulation | `bash scripts/run_sim.sh` |
| Lái bằng bàn phím | CPS5: 4. Lái bằng bàn phím | `bash scripts/keyboard.sh` (terminal riêng) |
| Kiểm tra tự động 9 mục | CPS5: 5. Tự kiểm tra simulation | `bash scripts/check_sim.sh` |
| Unit test (không cần ROS) | CPS5: Unit test | `bash scripts/test.sh` |
| Xem số liệu vật lý suy ra | CPS5: Xem số liệu vật lý + sinh SDF | `bash scripts/generate_sdf.sh` |

**Lái thử trong sim:**

1. Chạy sim, chờ khoảng 20 giây để ArduSub khởi động xong.
2. Mở teleop bàn phím ở terminal khác. Giữ phím thì robot chạy, nhả phím thì dừng.
   - `1`: ARM, rồi `h`: ALT_HOLD (tự giữ độ sâu, dễ lái nhất)
   - giữ `f`: lặn, `w`: tiến, `e`: quay phải
   - `Space`: dừng ngay; `[` / `]`: giảm / tăng lực
   - `0`: DISARM

   Dòng trạng thái cuối terminal hiển thị: ARM, chế độ, độ sâu, hướng la bàn, lệnh đang gửi và PWM của 8 thruster (1500 = không đẩy). Nhấn phím di chuyển khi chưa ARM sẽ có cảnh báo.

**Camera bám đuôi (chỉ khi có cửa sổ Gazebo):** node `chase_camera` đặt camera của cửa sổ Gazebo phía sau và hơi cao hơn robot, chỉ theo vị trí và hướng đi (có làm mượt), không lắc theo robot. Không dùng chức năng "Follow" của Gazebo: Follow xoay theo cả độ nghiêng của robot nên hình rung liên tục.
- Chỉnh khoảng cách: `bash scripts/run_sim.sh chase_distance:=2.0 chase_height:=0.8`; tắt: `chase:=false`.
- Xoay góc nhìn (trong terminal bàn phím): `j` / `l` xoay trái / phải 20°, `i` / `k` nhìn cao hơn / thấp hơn, `c` về sau đuôi.
- Không kéo camera bằng chuột khi chase đang bật (node đặt lại camera 10 lần/giây).

**Xem camera của robot:** `ros2 run rqt_image_view rqt_image_view /camera/image_raw`.

**Khi nào cần build lại:** workspace build bằng `--symlink-install`, nên sửa file `.py`, `.launch.py`, `.yaml` có hiệu lực ngay lần chạy sau. Chỉ build lại khi **thêm file mới** hoặc sửa `setup.py` / `package.xml`. Thư mục build/install nằm ở `~/.cps5/ws` trong WSL (nhanh hơn ổ Windows); mã nguồn vẫn ở thư mục repo.

## 3. Sửa thông số ở đâu

Mỗi thông số chỉ nằm ở **một** file. Không sửa trùng ở nơi khác.

| Muốn đổi | File |
|---|---|
| Khối lượng, lực nổi, quán tính, lực đẩy và vị trí thruster, lực cản, camera (chỉ ảnh hưởng sim) | `src/cps5_description/config/cps5_vehicle.yaml` |
| Bể nước, vật thể trong bể (sim) | `src/cps5_gazebo/config/pool_world.yaml` |
| Tham số ArduSub: frame, failsafe, pin — **dùng chung cho sim và Pixhawk thật** | `src/rov_bringup/config/ardusub/cps5.parm` |
| Ánh xạ tay cầm, gain | `src/rov_bringup/config/teleop.yaml` |
| Ngưỡng an toàn của safety_node (heartbeat, pin, độ sâu) | `src/rov_bringup/config/safety.yaml` |

File SDF của Gazebo được **sinh tự động** từ YAML mỗi lần chạy sim (ghi vào `~/.cps5/generated/`). Không sửa file `.sdf` sinh ra; sửa YAML. Thể tích hộp nổi được tính từ `mass × (1 + buoyancy_margin) / density`, nên lực nổi luôn khớp với khối lượng.

## 4. Thứ tự motor và hệ trục

Frame **Vectored-6DOF** (`FRAME_CONFIG = 2`). Thứ tự và chiều lực lấy từ source ArduSub Sub-4.7.2 (`AP_Motors6DOF.cpp`) và được kiểm tra bằng unit test (`test_model.py`).

| Cổng Pixhawk | Motor | Vị trí trên robot | Thruster trong CAD | PWM > 1500 đẩy robot về |
|---|---|---|---|---|
| MAIN 1 | M1 | ngang xéo 45°, trước-phải | T200 <8> | sau-phải |
| MAIN 2 | M2 | ngang xéo 45°, trước-trái | T200 <5> | sau-trái |
| MAIN 3 | M3 | ngang xéo 45°, sau-phải | T200 <7> | trước-phải |
| MAIN 4 | M4 | ngang xéo 45°, sau-trái | T200 <6> | trước-trái |
| MAIN 5 | M5 | đứng, trước-phải | T200 <1> | xuống |
| MAIN 6 | M6 | đứng, trước-trái | T200 <4> | xuống |
| MAIN 7 | M7 | đứng, sau-phải | T200 <2> | xuống |
| MAIN 8 | M8 | đứng, sau-trái | T200 <3> | xuống |

- **Trước / sau / trái / phải** tính khi đứng sau robot nhìn về mũi. Mũi là phía camera, đuôi là phía tether.
- **Đổi trục từ CAD sang hệ trục robot:** mũi = +Z của CAD, trên = −Y của CAD, phải = +X của CAD.
- **Trên robot thật**, thruster nào đẩy ngược chiều so với bảng: đặt `MOT_n_DIRECTION = -1` (n = số motor), hoặc đảo 2 trong 3 dây motor.
- ESC phải ở chế độ **2 chiều** (bidirectional / 3D).

## 5. Topic ROS chính

| Topic | Kiểu | Ghi chú |
|---|---|---|
| `/mavros/rc/override` | mavros_msgs/OverrideRCIn | Lệnh lái. Kênh 3 throttle, 4 yaw, 5 forward, 6 lateral; 1500 = trung tính |
| `/mavros/state` | mavros_msgs/State | connected, armed, mode |
| `/mavros/imu/data` | sensor_msgs/Imu | Hướng của robot do EKF tính (ENU) |
| `/mavros/global_position/rel_alt` | std_msgs/Float64 | Độ cao tương đối, âm khi ở dưới nước |
| `/camera/image_raw` | sensor_msgs/Image | Hiện chỉ có trong sim. Trên robot thật, video do BlueOS gửi thẳng cho QGroundControl |
| `/cps5/ground_truth/odom` | nav_msgs/Odometry | **Chỉ có trong sim**: vị trí thật của robot trong Gazebo |
| `/rov/topside_heartbeat` | std_msgs/Header | teleop_node → safety_node, báo laptop điều khiển còn hoạt động |
| `/rov/safety/status` | std_msgs/String | Tóm tắt trạng thái an toàn |

## 6. Kiểm tra tự động

`bash scripts/check_sim.sh` bật sim (không cửa sổ), gửi lệnh lái và so sánh với chuyển động thật của robot trong Gazebo. Chạy trước khi commit thay đổi về thruster, frame hoặc tham số ArduSub.

| # | Kiểm tra | Phát hiện lỗi gì |
|---|---|---|
| 1 | MAVROS kết nối SITL | SITL không chạy, sai cổng |
| 2 | Hướng EKF khớp Gazebo | sai đổi hệ trục ENU/NED |
| 3 | ARM ở MANUAL | ArduSub từ chối ARM |
| 4 | Lặn, tiến, quay phải, sang phải | sai thứ tự motor, sai chiều thruster |
| 5 | ALT_HOLD giữ độ sâu | cảm biến độ sâu, gain điều khiển |
| 6 | Thả tay thì tự nổi và tự cân bằng | sai lực nổi, sai tâm nổi |
| 7 | Có ảnh camera | lỗi render, lỗi bridge |
| 8 | Ngừng gửi lệnh thì ArduSub tự DISARM | failsafe `FS_PILOT_INPUT` |
| 9 | Mất heartbeat thì safety_node DISARM | safety_node |

Nếu mục 5 dao động lên xuống: chỉnh `PSC_POSZ_P`, `PSC_VELZ_P`, `PSC_ACCZ_P/I` trong `cps5.parm`. Gain mặc định của ArduSub được chỉnh cho BlueROV2 (~11 kg); CPS5 (~7 kg, 8 thruster) có cấu hình gần giống.

## 7. Chạy robot thật

Các phần chỉ dùng cho sim, không cần khi chạy thật: `cps5_description`, `cps5_gazebo`, `sim.launch.py`, `check_sim`, `run_sitl.sh`, `scripts/run_sim.sh`.

**Lưu ý chung về phần cứng:**

- **Pixhawk 6C:** nạp ArduSub **cùng phiên bản với SITL** (4.7.2) bằng QGroundControl; nạp `cps5.parm` (Parameters → Tools → Load from file), rồi reboot.
- **Power module:** module trong BOM gốc của CPS 5 là loại cho Pixhawk 2.4.8. Với Pixhawk 6C nên dùng Holybro PM02/PM06, hoặc kiểm tra kỹ sơ đồ chân trước khi cấp điện.
- **Cảm biến độ sâu MS5837:** cắm vào cổng I2C. ArduSub tự nhận; kiểm tra trên QGroundControl rằng độ sâu thay đổi khi nhúng cảm biến vào nước.
- **Nguồn:** 8 × T200 ở 12 V chạy hết công suất kéo trên 100 A. Dây nguồn và cầu chì phải chịu được, hoặc giới hạn bằng `MOT_THST_MAX` / gain lái.
- **ESC:** đặt chế độ 2 chiều. Với ESC BLHeli, dùng BLHeliSuite (có trong gói tài liệu CPS 5 resources, thư mục `Chapter 8 - Electronics pipe/Software`, không nằm trong repo).
- **Raspberry Pi 4 (BlueOS):** BlueOS giữ cổng USB của Pixhawk, giữ camera và chuyển tiếp MAVLink cho QGroundControl. MAVROS luôn nhận MAVLink qua một **endpoint UDP cổng 14555** do BlueOS tạo (`real.launch.py` mặc định `fcu_url = udp://:14555@`).
- **Chỉ một nguồn lái:** không lái bằng QGroundControl và ROS cùng lúc (cả hai cùng là GCS sysid 255, lệnh sẽ đè nhau). Khi lái bằng ROS, tắt joystick trong QGroundControl (Vehicle Setup → Joystick).

### 7.1. Cách A — ROS chạy trên laptop (khuyến nghị)

Pi giữ nguyên BlueOS, không cài thêm gì. MAVROS, safety_node và teleop chạy trên laptop (WSL), nhận MAVLink qua tether. Cách này không phụ thuộc BlueOS là 32-bit hay 64-bit.

| Máy | File / phần mềm |
|---|---|
| Pixhawk 6C | ArduSub 4.7.2 + `src/rov_bringup/config/ardusub/cps5.parm` |
| Raspberry Pi 4 | BlueOS |
| Laptop | QGroundControl; `real.launch.py` (MAVROS + safety_node); `topside.launch.py` (tay cầm + teleop); `mavros.yaml`, `safety.yaml`, `teleop.yaml` |

**Làm một lần:**

1. **ESC:** đặt chế độ 2 chiều cho cả 8 ESC.
2. **Tham số:** QGroundControl → Parameters → Tools → Load from file → `cps5.parm`, rồi reboot Pixhawk.
3. **Motor Test** (QGroundControl hoặc BlueOS; robot trên cạn **đã tháo cánh**, hoặc đặt trong thùng nước): chạy lần lượt M1–M8, so với bảng ở [mục 4](#4-thứ-tự-motor-và-hệ-trục). Thruster nào đẩy ngược: đặt `MOT_n_DIRECTION = -1`.
4. **MAVLink endpoint:** BlueOS → MAVLink Endpoints → **+** → type `UDP Client`, IP `192.168.2.1`, port `14555`.
5. **Mạng cho WSL (Windows 11):** ở chế độ mặc định (NAT), WSL không nhận được gói UDP từ tether. Tạo file `%USERPROFILE%\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
   rồi chạy `wsl --shutdown` trong PowerShell. Nếu vẫn không nhận được, cho phép UDP cổng 14555 trong Windows Firewall.

**Mỗi lần thả robot:**

1. Cắm pin, nối tether vào laptop. Đặt IP tĩnh cho card mạng tether: `192.168.2.1` / `255.255.255.0`.
2. Mở QGroundControl để xem camera và trạng thái (joystick trong QGroundControl đã tắt).
3. Terminal WSL 1:
   ```bash
   ros2 launch rov_bringup real.launch.py
   ```
   Chờ dòng `CON: Got HEARTBEAT, connected`.
4. Terminal WSL 2:
   ```bash
   ros2 launch rov_bringup topside.launch.py              # tay cầm
   # không có tay cầm: topside.launch.py joy:=false, rồi chạy bash scripts/keyboard.sh
   ```
5. ARM rồi lái. Phím và nút giống hệt khi chạy sim.

Nếu tether bị đứt, ROS mất kết nối, nhưng ArduSub vẫn có failsafe riêng (`FS_PILOT_INPUT`, `FS_GCS_ENABLE`): robot tự DISARM và nổi lên vì được chỉnh nổi dương.

### 7.2. Cách B — ROS chạy trên Pi (BlueOS Extension)

Yêu cầu BlueOS **64-bit**: trong BlueOS → Terminal, lệnh `uname -m` phải trả về `aarch64`. ROS 2 chạy trong một container Docker bên cạnh BlueOS; hướng dẫn trong [`docker/README.md`](docker/README.md). Endpoint MAVLink khi đó là `UDP Client` → `127.0.0.1:14555`. Laptop chỉ chạy `topside.launch.py`, cùng `ROS_DOMAIN_ID` với Pi.

## 8. Xử lý sự cố

| Triệu chứng | Cách xử lý |
|---|---|
| Lỗi `$'\r': command not found` | File bị đổi sang xuống dòng kiểu Windows (CRLF). Trong VS Code: góc dưới phải bấm `CRLF` → chọn `LF`, rồi lưu. |
| Cửa sổ Gazebo đen hoặc crash | `bash scripts/run_sim.sh render_engine:=ogre`; nếu vẫn lỗi: `export LIBGL_ALWAYS_SOFTWARE=1`, hoặc chạy `gui:=false` |
| Không có ảnh camera khi `gui:=false` | Render không cửa sổ (EGL) thường lỗi trên WSL: dùng `gui:=true`, hoặc `camera:=false` |
| `/mavros/state` báo connected = false (sim) | Xem log SITL trong terminal. Nếu thiếu `~/ardupilot/build/sitl/bin/ardusub`: chạy `bash scripts/setup_wsl.sh ardupilot` |
| MAVROS không connected (robot thật) | Kiểm tra endpoint `UDP Client 192.168.2.1:14555` trong BlueOS, file `.wslconfig` (mirrored), firewall UDP 14555 |
| ARM bị từ chối | `ros2 topic echo /mavros/statustext/recv` để xem lý do (thường do EKF chưa sẵn sàng; chờ thêm 10–20 s) |
| Lái không có tác dụng | Kiểm tra MAVROS chạy với `system_id: 255` (`config/mavros.yaml`) và chỉ có **một** nguồn gửi lệnh lái |
| Robot trong sim chìm hoặc nổi quá nhanh | `bash scripts/generate_sdf.sh` để xem lực nổi dư; sửa `buoyancy_margin` |
| Tay cầm USB không nhận trong WSL | Dùng `usbipd` để chuyển thiết bị USB vào WSL, hoặc lái bằng bàn phím |

## 9. Cấu trúc repo

```
CPS5/
├── scripts/                setup_wsl.sh, build.sh, run_sim.sh, keyboard.sh, check_sim.sh, test.sh, generate_sdf.sh, env.sh
├── src/
│   ├── cps5_description/   cps5_vehicle.yaml (thông số vật lý) + bộ sinh SDF robot (model.py)
│   ├── cps5_gazebo/        pool_world.yaml + bộ sinh world (world.py), bridge.yaml, gazebo.launch.py
│   ├── rov_bringup/        launch sim / real / topside, cps5.parm, YAML cho mavros/teleop/safety, run_sitl.sh, check_sim
│   ├── rov_teleop/         teleop_node (tay cầm → lệnh lái), keyboard_teleop
│   └── rov_safety/         safety_node (heartbeat, pin, độ sâu)
├── docker/                 Dockerfile + hướng dẫn chạy ROS 2 trên Pi dưới dạng BlueOS Extension
└── vscode/                 cấu hình VS Code dùng chung — đổi tên thành .vscode để dùng
```

## Tham khảo

[ArduSub frames](https://ardupilot.org/sub/docs/sub-frames.html) ·
[ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) ·
[bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz) ·
[MAVROS](https://github.com/mavlink/mavros) ·
[BlueOS](https://blueos.cloud/docs/stable/usage/installation/) ·
CPS 5 Bill Of Materials 2.0.5.
