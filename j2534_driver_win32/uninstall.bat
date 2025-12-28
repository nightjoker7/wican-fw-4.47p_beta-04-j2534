@echo off
REM WiCAN J2534 Driver Uninstallation Script
REM Must be run as Administrator!

echo ============================================
echo WiCAN J2534 Driver Uninstaller
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

echo Removing registry entries...

REM Remove 32-bit registry entries
reg delete "HKLM\SOFTWARE\WOW6432Node\PassThruSupport.04.04\WiCAN J2534" /f 2>nul
if %ERRORLEVEL% equ 0 (
    echo   Removed: WiCAN J2534 (32-bit registry)
) else (
    echo   Not found: WiCAN J2534 (32-bit registry)
)

reg delete "HKLM\SOFTWARE\WOW6432Node\PassThruSupport.04.04\WiCAN_J2534" /f 2>nul
if %ERRORLEVEL% equ 0 (
    echo   Removed: WiCAN_J2534 (32-bit registry)
) else (
    echo   Not found: WiCAN_J2534 (32-bit registry)
)

REM Remove 64-bit registry entries
reg delete "HKLM\SOFTWARE\PassThruSupport.04.04\WiCAN J2534" /f 2>nul
if %ERRORLEVEL% equ 0 (
    echo   Removed: WiCAN J2534 (64-bit registry)
) else (
    echo   Not found: WiCAN J2534 (64-bit registry)
)

reg delete "HKLM\SOFTWARE\PassThruSupport.04.04\WiCAN_J2534" /f 2>nul
if %ERRORLEVEL% equ 0 (
    echo   Removed: WiCAN_J2534 (64-bit registry)
) else (
    echo   Not found: WiCAN_J2534 (64-bit registry)
)

echo.
echo Removing DLL files...

set INSTALL_DIR1=C:\Program Files\WiCAN J2534
set INSTALL_DIR2=C:\Program Files\WiCAN

if exist "%INSTALL_DIR1%\wican_j2534_32.dll" (
    del /f "%INSTALL_DIR1%\wican_j2534_32.dll"
    echo   Removed: %INSTALL_DIR1%\wican_j2534_32.dll
)

if exist "%INSTALL_DIR1%\wican_j2534_64.dll" (
    del /f "%INSTALL_DIR1%\wican_j2534_64.dll"
    echo   Removed: %INSTALL_DIR1%\wican_j2534_64.dll
)

if exist "%INSTALL_DIR2%\wican_j2534_32.dll" (
    del /f "%INSTALL_DIR2%\wican_j2534_32.dll"
    echo   Removed: %INSTALL_DIR2%\wican_j2534_32.dll
)

if exist "%INSTALL_DIR2%\wican_j2534_64.dll" (
    del /f "%INSTALL_DIR2%\wican_j2534_64.dll"
    echo   Removed: %INSTALL_DIR2%\wican_j2534_64.dll
)

REM Remove directories if empty
if exist "%INSTALL_DIR1%" rmdir "%INSTALL_DIR1%" 2>nul
if exist "%INSTALL_DIR2%" rmdir "%INSTALL_DIR2%" 2>nul

echo.
echo ============================================
echo Uninstallation complete!
echo ============================================
pause
