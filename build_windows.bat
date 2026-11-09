@echo off
rem Build LangitSwarm.exe (folder build in dist\LangitSwarm) with PyInstaller.
setlocal
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (
  py -3 -m venv .venv || python -m venv .venv || (echo Python 3.11+ 64-bit is required & pause & exit /b 1)
)
.venv\Scripts\python.exe -m pip install --upgrade pip
.venv\Scripts\python.exe -m pip install -r requirements.txt pyinstaller || (echo Package install failed & pause & exit /b 1)
.venv\Scripts\python.exe -m PyInstaller --noconfirm --clean LangitSwarm.spec || (echo Build failed & pause & exit /b 1)
rem files that must sit next to LangitSwarm.exe (.NET window settings, shortcuts, notes)
copy /y "assets\windows\LangitSwarm.exe.config" "dist\LangitSwarm\" >nul
copy /y "assets\windows\*.cmd" "dist\LangitSwarm\" >nul
copy /y "assets\windows\START HERE.txt" "dist\LangitSwarm\" >nul
xcopy /e /i /y samples "dist\LangitSwarm\samples" >nul
copy /y README.md "dist\LangitSwarm\" >nul
copy /y CHANGELOG.md "dist\LangitSwarm\" >nul
echo.
echo Done: dist\LangitSwarm\LangitSwarm.exe
echo Copy the whole dist\LangitSwarm folder to the show laptop.
pause
