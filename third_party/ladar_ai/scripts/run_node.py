#!/usr/bin/env python3
"""通用节点启动脚本。根据可执行文件名直接执行对应节点文件。"""
import sys
import os
import importlib

exe_name = os.path.basename(os.path.realpath(sys.argv[0]))
module_name = f"ladar_ai.{exe_name}"

mod = importlib.import_module(module_name)
if hasattr(mod, 'main'):
    mod.main()
else:
    print(f"No main() in {module_name}")
    sys.exit(1)
