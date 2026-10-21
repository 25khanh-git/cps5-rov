# CPS 5 — Simulation (ROS 2 Humble · Gazebo Harmonic · ArduSub SITL)

Đây là simulation của ROV CPS 5 (frame ArduSub **SimpleROV-5**, Pixhawk 6C + Raspberry Pi 4).
Nó dùng **đúng stack phần mềm sẽ chạy trên robot thật**: firmware ArduSub thật (SITL) và MAVROS.
Code ROS 2 viết cho sim chạy lại nguyên vẹn trên robot. Khi chuyển sang robot, chỉ đổi `fcu_url`.

```
 bàn phím / tay cầm ─► teleop_node ──/mavros/rc/override──► MAVROS ◄─MAVLink UDP 14550─► ArduSub SITL
                          │ /rov/topside_heartbeat              ▲                          ▲ JSON UDP 9002
                          ▼                                     │                          ▼
                     safety_node ───── disarm / SURFACE ────────┘          Gazebo: ArduPilotPlugin → 5 thruster
                                                                           Buoyancy + Hydrodynamics (nước)
 rqt_image_view ◄── /camera/image_raw ◄── ros_gz_bridge ◄──────────────── camera, ground truth
```

---

## 1. Cài đặt (làm một lần)

**Cần:** Windows 10/11 có WSL2, khoảng 15 GB trống và RAM ≥ 8 GB.

1. **Ubuntu 22.04 trong WSL.** Mở PowerShell (Admin):
   ```powershell
   wsl --install -d Ubuntu-22.04
   wsl --set-default Ubuntu-22.04
   ```
2. **VS Code** và extension **WSL** (`ms-vscode-remote.remote-wsl`).
3. **Mở dự án trong WSL** (bắt buộc để IntelliSense thấy thư viện ROS 2). Trong terminal Ubuntu:
   ```bash
   cd /mnt/d/Projects/CPS5
   code .
   ```
   Hoặc trong VS Code: `Ctrl+Shift+P` → **WSL: Open Folder in WSL…** → chọn `D:\Projects\CPS5`.
   **Kích hoạt cấu hình VS Code (một lần):** đổi tên thư mục `vscode` thành `.vscode`
   (`mv vscode .vscode` trong terminal WSL). Sau đó VS Code sẽ gợi ý cài các extension được khuyên dùng, hãy chọn cài.
4. **Cài môi trường:** `Terminal → Run Task… → CPS5: 1. Cài môi trường`, hoặc chạy:
   ```bash
   bash scripts/setup_wsl.sh
   ```
   Script cài ROS 2 Humble, MAVROS, Gazebo Harmonic, build ArduSub SITL (**Sub-4.7.2**) và ArduPilotPlugin, rồi build workspace.
   Mất khoảng 30–90 phút. Có thể chạy lại an toàn, bước nào xong rồi sẽ được bỏ qua.

## 2. Dùng hằng ngày

| Việc | VS Code (`Ctrl+Shift+P` → Run Task) | Terminal |
|---|---|---|
| Build | **Ctrl+Shift+B** | `bash scripts/build.sh` |
| Chạy simulation | CPS5: 3. Chạy simulation | `bash scripts/run_sim.sh` |
| Lái bằng bàn phím | CPS5: 4. Lái bằng bàn phím | `bash scripts/keyboard.sh` (terminal riêng) |
| Tự kiểm tra 9 mốc | CPS5: 5. Tự kiểm tra simulation | `bash scripts/check_sim.sh` |
| Unit test (không cần ROS) | CPS5: Unit test | `bash scripts/test.sh` |
| Xem số liệu vật lý | CPS5: Xem số liệu vật lý + sinh SDF | `bash scripts/generate_sdf.sh` |

**Lái thử:**

1. Chạy simulation và chờ khoảng 20 giây cho ArduSub khởi động xong.
2. Mở bàn phím teleop. **Giữ phím thì robot chạy, nhả phím thì dừng.**
   - `1` để **ARM**, rồi `h` để vào ALT_HOLD (tự giữ độ sâu — dễ lái nhất);
   - giữ `f` để lặn, `w` để tiến, `e` để quay phải;
   - `Space` để dừng ngay; `[` / `]` giảm / tăng lực;
   - `0` để **DISARM**.

   Dòng trạng thái ở cuối terminal cho biết ARM, mode, độ sâu, hướng la bàn, lệnh đang gửi và
   **PWM thực tế của 5 thruster** (1500 = không đẩy). Nhấn phím di chuyển khi chưa ARM sẽ có cảnh báo.

**Camera bám đuôi:** khi chạy có cửa sổ Gazebo, node `chase_camera` tự đặt camera ở **sau và hơi cao
so với robot**, chỉ theo vị trí + hướng đi (có làm mượt), không lắc theo robot. **Đừng bật "Follow" của
Gazebo** — Follow xoay theo cả độ nghiêng của robot nên rất loạn.
Chỉnh: `bash scripts/run_sim.sh chase_distance:=2.0 chase_height:=0.8`; tắt: `chase:=false`.
**Xoay góc nhìn quanh robot** (trong terminal bàn phím, khoảng cách giữ nguyên): `j` / `l` xoay sang trái / phải
(mỗi lần 20°), `i` / `k` nhìn từ cao hơn / thấp hơn, `c` về lại sau đuôi. Không dùng chuột kéo camera khi
chase đang bật (node đặt lại camera 10 lần/giây).

Xem camera: `ros2 run rqt_image_view rqt_image_view /camera/image_raw`.

Workspace được build bằng `--symlink-install`. Vì vậy sửa file `.py`, `.launch.py` hay `.yaml` là có hiệu lực ngay ở lần chạy sau.
Chỉ cần build lại khi **thêm file mới** hoặc sửa `setup.py` / `package.xml`.
Thư mục build/install nằm ở `~/.cps5/ws` trong WSL (nhanh hơn ổ D:). Mã nguồn vẫn nằm ở `D:\Projects\CPS5`.

## 3. Sửa thông số ở đâu?

| Muốn đổi | File |
|---|---|
| Khối lượng, lực nổi, quán tính, lực đẩy thruster, vị trí thruster, lực cản, camera | `src/cps5_description/config/cps5_vehicle.yaml` |
| Bể nước, vật thể trong bể | `src/cps5_gazebo/config/pool_world.yaml` |
| Tham số ArduSub (frame, failsafe, pin) — **dùng chung cho Pixhawk thật** | `src/rov_bringup/config/ardusub/cps5.parm` |
| Ánh xạ tay cầm, gain | `src/rov_bringup/config/teleop.yaml` |
| Ngưỡng an toàn (heartbeat, pin, độ sâu) | `src/rov_bringup/config/safety.yaml` |

File SDF được **sinh tự động** từ YAML mỗi lần chạy (ghi vào `~/.cps5/generated/`). Đừng sửa file `.sdf` sinh ra.
Thể tích hộp nổi được tính từ `mass × (1 + buoyancy_margin) / density`, nên lực nổi luôn khớp khối lượng.

**Số liệu hiện tại là ước lượng.** Tổng khối lượng ≈ 2,1 kg lấy từ BOM và datasheet. Lực đẩy 5 N/thruster chưa đo. Hệ số cản tính từ kích thước.
Khi có robot thật: cân khối lượng, đo lực đẩy bằng cân treo trong thùng nước, rồi sửa YAML.

## 4. Thứ tự motor (SimpleROV-5, `FRAME_CONFIG = 6`)

Thứ tự này lấy từ source ArduSub (`AP_Motors6DOF.cpp`) và được unit test kiểm tra lại.

| Cổng Pixhawk | Motor | Vị trí | PWM > 1500 đẩy về |
|---|---|---|---|
| MAIN 1 | M1 | ngang, bên **phải** | phía trước |
| MAIN 2 | M2 | ngang, bên **trái** | phía trước |
| MAIN 3 | M3 | đứng, bên **phải** | **xuống** |
| MAIN 4 | M4 | đứng, bên **trái** | **xuống** |
| MAIN 5 | M5 | ngang hông, giữa | bên **phải** |

Trên robot thật, nếu một thruster quay ngược: đảo 2 trong 3 dây motor, hoặc đặt `MOT_n_DIRECTION = -1`.
ESC phải ở chế độ **2 chiều** (3D / bidirectional).

## 5. Topic chính

| Topic | Kiểu | Ghi chú |
|---|---|---|
| `/mavros/rc/override` | mavros_msgs/OverrideRCIn | kênh 3 throttle, 4 yaw, 5 forward, 6 lateral; 1500 = trung tính |
| `/mavros/state` | mavros_msgs/State | connected, armed, mode |
| `/mavros/imu/data` | sensor_msgs/Imu | hướng EKF (ENU) |
| `/mavros/global_position/rel_alt` | std_msgs/Float64 | âm khi ở dưới nước |
| `/camera/image_raw` | sensor_msgs/Image | sim: Gazebo; thật: v4l2_camera |
| `/cps5/ground_truth/odom` | nav_msgs/Odometry | **chỉ có trong sim**: vị trí thật trong Gazebo |
| `/rov/topside_heartbeat` | std_msgs/Header | teleop → safety_node |
| `/rov/safety/status` | std_msgs/String | tóm tắt trạng thái an toàn |

## 6. Kiểm tra tự động (`scripts/check_sim.sh`)

Script so sánh lệnh gửi cho ArduSub với chuyển động thật trong Gazebo:

1. MAVROS kết nối SITL.
2. Hướng EKF khớp Gazebo (đúng đổi hệ trục ENU/NED).
3. ARM ở MANUAL.
4. Lặn, tiến, quay phải, sang phải. Bước này bắt được lỗi sai thứ tự motor hoặc sai chiều thruster.
5. ALT_HOLD giữ độ sâu.
6. Thả tay thì tự nổi và tự cân bằng.
7. Có ảnh camera.
8. Ngừng gửi lệnh thì ArduSub tự DISARM.
9. Mất heartbeat thì safety_node DISARM.

Nếu bước 5 dao động: chỉnh `PSC_POSZ_P`, `PSC_VELZ_P`, `PSC_ACCZ_P/I` trong `cps5.parm`.
Các gain mặc định được chỉnh cho BlueROV2 (~11 kg), còn CPS 5 nhẹ hơn nhiều.

## 7. Chuyển sang robot thật

- **Pixhawk 6C:**
  - nạp ArduSub **cùng phiên bản** với SITL (4.7.2) bằng QGroundControl;
  - nạp `cps5.parm` (Parameters → Tools → Load from file), rồi reboot.
- **Power module:** module trong BOM gốc là loại cho Pixhawk 2.4.8. Với 6C nên dùng **Holybro PM02/PM06**, hoặc kiểm tra kỹ chân cắm trước khi cấp điện.
- **MS5837 (cảm biến độ sâu):** cắm vào cổng I2C. ArduSub tự dò; kiểm tra trong QGC rằng độ sâu thay đổi khi nhúng cảm biến xuống nước.
- **Raspberry Pi 4:**
  - cài Ubuntu 22.04 Server và ROS 2 Humble, rồi `ros-humble-mavros ros-humble-v4l2-camera`;
  - chép thư mục `src/` này và build;
  - chạy `ros2 launch rov_bringup real.launch.py` (Pixhawk qua USB `/dev/ttyACM0`).
- **Laptop:** `ros2 launch rov_bringup topside.launch.py` (tay cầm + teleop). Hai máy dùng cùng `ROS_DOMAIN_ID`, nối qua cáp Ethernet trong tether.

## 8. Xử lý sự cố

| Triệu chứng | Cách xử lý |
|---|---|
| Lỗi `$'\r': command not found` | File bị đổi sang CRLF. Trong VS Code: góc dưới phải `CRLF` → `LF`, rồi lưu. |
| Cửa sổ Gazebo đen / crash | `bash scripts/run_sim.sh render_engine:=ogre`; nếu vẫn lỗi: `export LIBGL_ALWAYS_SOFTWARE=1`, hoặc `gui:=false` |
| Không có ảnh camera khi `gui:=false` | Render headless (EGL) lỗi trên WSL: dùng `gui:=true`, hoặc `camera:=false` |
| `/mavros/state` connected = false | Xem log SITL trong terminal. Thiếu `~/ardupilot/build/sitl/bin/ardusub` → chạy `bash scripts/setup_wsl.sh ardupilot` |
| ARM bị từ chối | `ros2 topic echo /mavros/statustext/recv` để xem lý do (thường do EKF chưa sẵn sàng; chờ thêm 10–20 s) |
| Lái không có tác dụng | Kiểm tra MAVROS chạy với `system_id: 255` (`config/mavros.yaml`), và chỉ có **một** nguồn gửi RC override |
| Robot chìm hoặc nổi quá nhanh | `bash scripts/generate_sdf.sh` để xem lực nổi dư; sửa `buoyancy_margin` |
| Tay cầm USB không thấy trong WSL | Dùng `usbipd` để chuyển USB vào WSL, hoặc dùng bàn phím |

## 9. Cấu trúc

```
CPS5/
├── scripts/            setup_wsl.sh, build.sh, run_sim.sh, keyboard.sh, check_sim.sh, test.sh, generate_sdf.sh, env.sh
├── src/
│   ├── cps5_description/   cps5_vehicle.yaml + bộ sinh SDF robot (model.py)
│   ├── cps5_gazebo/        pool_world.yaml + bộ sinh world (world.py), bridge.yaml, gazebo.launch.py
│   ├── rov_bringup/        sim / real / topside launch, cps5.parm, mavros/teleop/safety YAML, run_sitl.sh, check_sim
│   ├── rov_teleop/         teleop_node (joy → RC override), keyboard_teleop
│   └── rov_safety/         safety_node (heartbeat, pin, độ sâu)
└── vscode/             cấu hình VS Code — đổi tên thành .vscode để kích hoạt (tasks, settings, extensions)
```

Nguồn tham khảo: [ArduSub frames](https://ardupilot.org/sub/docs/sub-frames.html) ·
[ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) ·
[bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz) · [MAVROS](https://github.com/mavlink/mavros) ·
CPS 5 Bill Of Materials 2.0.5.
