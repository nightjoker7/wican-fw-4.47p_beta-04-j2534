@echo off
REM WiCAN J2534 Driver MinGW Build Script
REM Requires: MinGW-w64 with GCC

setlocal EnableDelayedExpansion

echo ============================================
echo WiCAN J2534 Driver MinGW Build Script
echo ============================================
echo.

REM Check for GCC
where gcc >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo ERROR: GCC not found. Please install MinGW-w64 and add it to PATH.
    exit /b 1
)

REM Create output directory
if not exist bin mkdir bin

echo Building wican_j2534.dll...
gcc -shared -o bin\wican_j2534.dll ^
    wican_j2534.c ^
    wican_comm.c ^
    -I. ^
    -DWIN32 -D_WIN32_WINNT=0x0601 ^
    -lws2_32 -lsetupapi ^
    -Wl,--out-implib,bin\wican_j2534.lib ^
    -static-libgcc

if %ERRORLEVEL% neq 0 (
    echo ERROR: DLL build failed
    exit /b 1
)
echo DLL build successful!

echo.
echo Building wican_config.exe...
gcc -o bin\wican_config.exe ^
    wican_config.c ^
    -I. ^
    -DWIN32 -D_WIN32_WINNT=0x0601 ^
    -lcomctl32 -lws2_32 -lsetupapi ^
    -mwindows ^
    -static-libgcc

if %ERRORLEVEL% neq 0 (
    echo ERROR: Config utility build failed
    exit /b 1
)
echo Config utility build successful!

echo.
echo ============================================
echo Build Complete!
echo ============================================
echo.
echo Output files in bin\:
dir bin\*.dll bin\*.exe 2>nul
echo.

endlocal
