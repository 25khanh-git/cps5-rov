# Cấu hình VS Code dùng chung

Thư mục này chứa cấu hình VS Code cho cả nhóm. VS Code chỉ đọc cấu hình trong thư mục có tên `.vscode`, nên cần đổi tên thư mục này thành `.vscode` một lần sau khi clone repo:

- **Terminal WSL** (ở thư mục gốc của repo):
  ```bash
  mv vscode .vscode
  ```
- **Windows Explorer:** đổi tên thành `.vscode.` (thêm dấu chấm ở cuối; Windows tự bỏ dấu chấm đó).

Mở lại VS Code sau khi đổi tên. Cấu hình gồm:

| File | Nội dung |
|---|---|
| `tasks.json` | Các task `CPS5: …` (build, chạy sim, test). `Ctrl+Shift+B` = build. Chạy được khi VS Code mở ở chế độ Windows hoặc WSL. |
| `settings.json` | Trình thông dịch Python của ROS 2 Humble, đường dẫn import cho IntelliSense, xuống dòng LF, ẩn thư mục build. |
| `extensions.json` | Danh sách extension nên cài (VS Code tự gợi ý khi mở repo). |
| `python.env` | Biến môi trường cho Python extension (PYTHONPATH tới các package trong `src/`). |

Muốn đổi cấu hình cho cả nhóm: sửa file ở đây và commit. Thiết lập riêng của từng người đặt trong User Settings của VS Code, không đặt vào repo.
