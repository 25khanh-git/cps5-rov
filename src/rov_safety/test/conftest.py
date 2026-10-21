# Cho phép chạy pytest không cần build: thêm src/<pkg> vào sys.path.
import os
import sys

SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for pkg in ('rov_safety',):
    path = os.path.join(SRC, pkg)
    if path not in sys.path:
        sys.path.insert(0, path)
