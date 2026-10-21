# Cấu hình VS Code cho CPS 5

Công cụ của Claude không được phép ghi trực tiếp vào thư mục `.vscode`, nên cấu hình được để ở đây.
Bạn kích hoạt một lần bằng cách **đổi tên thư mục `vscode` thành `.vscode`**, chọn một trong hai cách:

- **Windows Explorer:** đổi tên `vscode` → `.vscode.` (có dấu chấm ở cuối, Windows sẽ tự bỏ dấu chấm đó).
- **Terminal WSL:**
  ```bash
  cd /mnt/d/Projects/CPS5 && mv vscode .vscode
  ```

Sau đó mở lại VS Code. Bạn sẽ có:

- `tasks.json`: các task CPS5 (Ctrl+Shift+B = build), chạy được cả khi VS Code mở ở chế độ Windows lẫn WSL;
- `settings.json`: Python của ROS 2 Humble, dùng LF, ẩn file build;
- `extensions.json`: các extension nên cài.
