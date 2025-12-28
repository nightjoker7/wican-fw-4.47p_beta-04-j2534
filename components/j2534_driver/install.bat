@echo off
REM WiCAN J2534 Driver Installation Script
REM Must be run as Administrator!

echo ============================================
echo WiCAN J2534 Driver Installer
echo ============================================
echo.

REM Check for admin privileges
net session >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: This script must be run as Administrator!
    echo Right-click on this file and select "Run as administrator"
    pause
    exit /b 1
)

set INSTALL_DIR=C:\Program Files\WiCAN J2534

echo Creating installation directory...
if not exist "%INSTALL_DIR%" mkdir "%INSTALL_DIR%"

echo Copying DLL files...

REM Look for built DLLs in various locations
set FOUND_32=0
set FOUND_64=0

if exist "build\Release\wican_j2534_32.dll" (
    copy /Y "build\Release\wican_j2534_32.dll" "%INSTALL_DIR%\"
    set FOUND_32=1
    echo   Installed: wican_j2534_32.dll from build\Release
)

if exist "build32\Release\wican_j2534_32.dll" (
    copy /Y "build32\Release\wican_j2534_32.dll" "%INSTALL_DIR%\"
    set FOUND_32=1
    echo   Installed: wican_j2534_32.dll
)

if exist "Release\wican_j2534_32.dll" (
    copy /Y "Release\wican_j2534_32.dll" "%INSTALL_DIR%\"
    set FOUND_32=1
    echo   Installed: wican_j2534_32.dll
)

if exist "wican_j2534_32.dll" (
    copy /Y "wican_j2534_32.dll" "%INSTALL_DIR%\"
    set FOUND_32=1
    echo   Installed: wican_j2534_32.dll
)

if exist "build64\Release\wican_j2534_64.dll" (
    copy /Y "build64\Release\wican_j2534_64.dll" "%INSTALL_DIR%\"
    set FOUND_64=1
    echo   Installed: wican_j2534_64.dll
)

if exist "Release\wican_j2534_64.dll" (
    copy /Y "Release\wican_j2534_64.dll" "%INSTALL_DIR%\"
    set FOUND_64=1
    echo   Installed: wican_j2534_64.dll
)

if exist "wican_j2534_64.dll" (
    copy /Y "wican_j2534_64.dll" "%INSTALL_DIR%\"
    set FOUND_64=1
    echo   Installed: wican_j2534_64.dll
)

if %FOUND_32%==0 if %FOUND_64%==0 (
    echo WARNING: No DLL files found! Build the driver first using build.bat
)

echo.
echo Registering driver in Windows Registry...
regedit /s register_driver.reg
if %ERRORLEVEL% neq 0 (
    echo ERROR: Registry registration failed!
    pause
    exit /b 1
)

echo.
echo ============================================
echo Installation Complete!
echo ============================================
echo.
echo Driver installed to: %INSTALL_DIR%
echo.
echo To use the driver:
echo   1. Power on your WiCAN Pro device
echo   2. Connect your PC to the WiCAN WiFi network
echo   3. Open your J2534-compatible software
echo   4. Select "WiCAN J2534" as the interface
echo.

pause
