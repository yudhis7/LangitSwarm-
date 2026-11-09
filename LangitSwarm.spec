# PyInstaller spec for Langit Swarm (Windows).  Build with build_windows.bat
# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_submodules

hidden = (
    collect_submodules("langit")
    + ["pymavlink.dialects.v20.ardupilotmega", "pymavlink.dialects.v20.common", "pymavlink.mavutil",
       "ot.lp", "lap", "scipy.optimize", "scipy.spatial", "aiohttp", "webview"]
)

a = Analysis(
    ["app.py"],
    pathex=["."],
    binaries=[],
    datas=[("web", "web"), ("lua", "lua"), ("tools", "tools"), ("samples", "samples"),
           ("assets/brand", "assets/brand"), ("README.md", "."),
           ("CHANGELOG.md", ".")],
    hiddenimports=hidden,
    excludes=["torch", "tensorflow", "jax", "cupy", "matplotlib", "tkinter", "IPython", "pandas",
              "sklearn", "autograd", "pymanopt", "cvxopt"],
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz, a.scripts, [],
    exclude_binaries=True,
    name="LangitSwarm",
    console=True,          # keep the console: it shows server logs during a show
    icon="assets/brand/langitswarm.ico",
    version="assets/brand/version_info.txt",
)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name="LangitSwarm")
