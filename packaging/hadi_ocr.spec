# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec for any version: the name comes from APP_VERSION in
# hadi_ocr_app.py. Run via packaging/build.ps1 (or: pyinstaller packaging/hadi_ocr.spec).
import re
from pathlib import Path
from PyInstaller.utils.hooks import collect_all

ROOT = Path(SPECPATH).parent
VERSION = re.search(r'^APP_VERSION = "([^"]+)"', (ROOT / "hadi_ocr_app.py").read_text(encoding="utf-8"),
                    re.M).group(1)
NAME = f"HADI_OCR_v{VERSION}"

datas = [
    (str(ROOT / 'load_cell_icon.png'), '.'),
    (str(ROOT / 'ocr_stream.py'), '.'),
    (str(ROOT / 'gps_receiver.py'), '.'),
]
binaries = []
hiddenimports = ['serial', 'serial.tools', 'serial.tools.list_ports']

# These packages ship native DLLs / bundled data (ONNX models, DirectShow
# COM glue) that PyInstaller's static import scan won't pick up on its own.
# rapidocr downloads PP-OCRv6_rec_tiny.onnx into its models/ folder on first
# use; run the app once from source before building so it gets bundled.
for pkg in ('cv2', 'PIL', 'rapidocr', 'omegaconf', 'onnxruntime', 'pygrabber', 'comtypes'):
    pkg_datas, pkg_binaries, pkg_hiddenimports = collect_all(pkg)
    datas += pkg_datas
    binaries += pkg_binaries
    hiddenimports += pkg_hiddenimports

a = Analysis(
    [str(ROOT / 'hadi_ocr_app.py')],
    pathex=[str(ROOT)],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # rapidocr ships adapters for other inference engines; collect_all pulls
    # in whichever are installed on the build machine (PyTorch, Paddle, ...)
    # — ~550 MB the app never loads, since it only uses ONNX Runtime.
    excludes=['torch', 'torchvision', 'paddle', 'paddlex', 'scipy', 'pandas',
              'matplotlib', 'openvino', 'MNN', 'tensorflow'],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

# One-FOLDER build (no unpacking to %TEMP% on every launch), UPX off.
exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name=NAME,
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=[str(ROOT / 'app_icon.ico')],
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name=NAME,
)
