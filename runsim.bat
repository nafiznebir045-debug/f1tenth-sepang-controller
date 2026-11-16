@echo off
REM ==============================================================================
REM F1TENTH Autonomous Racing Launcher - Windows CMD / PowerShell Shortcut
REM Group 3 | Track: Sepang | Driver: NEBIR NAFIZ MOLLAH
REM ==============================================================================
wsl.exe bash -c "if [ -d /home/admin1/Downloads/nebirAssaignment-main ]; then cd /home/admin1/Downloads/nebirAssaignment-main && ./runsim; else cd /mnt/d/nebirAssaignment && ./runsim; fi"
