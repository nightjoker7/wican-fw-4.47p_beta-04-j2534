@echo off
REM WiCAN J2534 Driver Build Script
REM Requires: Visual Studio Build Tools or MinGW
REM Usage: build.bat [32|64|all]

setlocal EnableDelayedExpansion

set BUILD_TYPE=%1
if "%BUILD_TYPE%"=="" set BUILD_TYPE=all

echo ============================================
echo WiCAN J2534 Driver Build Script
echo ============================================
echo.

REM Check for CMake
where cmake >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo ERROR: CMake not found. Please install CMake and add it to PATH.
    exit /b 1
)

REM Build 32-bit DLL
if "%BUILD_TYPE%"=="32" goto :build32
if "%BUILD_TYPE%"=="all" goto :build32
goto :check64

:build32
echo Building 32-bit driver...
if not exist build32 mkdir build32
cd build32
cmake -G "Visual Studio 17 2022" -A Win32 ..
if %ERRORLEVEL% neq 0 (
    echo ERROR: CMake configuration failed for 32-bit build
    cd ..
    exit /b 1
)
cmake --build . --config Release
if %ERRORLEVEL% neq 0 (
    echo ERROR: Build failed for 32-bit driver
    cd ..
    exit /b 1
)
echo 32-bit build successful!
cd ..

:check64
if "%BUILD_TYPE%"=="64" goto :build64
if "%BUILD_TYPE%"=="all" goto :build64
goto :done

:build64
echo.
echo Building 64-bit driver...
if not exist build64 mkdir build64
cd build64
cmake -G "Visual Studio 17 2022" -A x64 ..
if %ERRORLEVEL% neq 0 (
    echo ERROR: CMake configuration failed for 64-bit build
    cd ..
    exit /b 1
)
cmake --build . --config Release
if %ERRORLEVEL% neq 0 (
    echo ERROR: Build failed for 64-bit driver
    cd ..
    exit /b 1
)
echo 64-bit build successful!
cd ..

:done
echo.
echo ============================================
echo Build Complete!
echo ============================================
echo.
echo Output files:
if exist build32\Release\wican_j2534_32.dll (
    echo   - build32\Release\wican_j2534_32.dll
)
if exist build64\Release\wican_j2534_64.dll (
    echo   - build64\Release\wican_j2534_64.dll
)
echo.
echo To install the driver:
echo   1. Copy DLL files to C:\Program Files\WiCAN\
echo   2. Run register_driver.reg as Administrator
echo   3. Connect to WiCAN WiFi network (WiCAN_xxxx)
echo.

endlocal
