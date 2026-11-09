@echo off
rem LangitSwarm - run from source on Windows (creates a local Python environment on first run)
setlocal
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (
  echo Creating Python environment...
  py -3 -m venv .venv || python -m venv .venv || (echo Python 3.11+ 64-bit is required: https://www.python.org/downloads/ & pause & exit /b 1)
  .venv\Scripts\python.exe -m pip install --upgrade pip
  .venv\Scripts\python.exe -m pip install -r requirements.txt || (echo Package install failed & pause & exit /b 1)
)
.venv\Scripts\python.exe app.py %*
