@echo off
REM WiCAN J2534 Driver MSVC Build Script
REM Uses Visual Studio 2022 Build Tools

setlocal EnableDelayedExpansion

echo ============================================
echo WiCAN J2534 Driver MSVC Build Script
echo ============================================
echo.

REM Setup Visual Studio environment
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
if %ERRORLEVEL% neq 0 (
    call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars32.bat" >nul 2>&1
)

REM Create output directory
if not exist bin mkdir bin

echo Building 64-bit wican_j2534.dll...
cl /LD /O2 /DWIN32 /D_WIN32_WINNT=0x0601 ^
   /Fe:bin\wican_j2534_64.dll ^
   wican_j2534.c wican_comm.c ^
   ws2_32.lib setupapi.lib advapi32.lib ^
   /link /DEF:wican_j2534.def

if %ERRORLEVEL% neq 0 (
    echo ERROR: 64-bit DLL build failed
    goto :builddll32
)
echo 64-bit DLL build successful!

:builddll32
REM Setup 32-bit environment
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars32.bat" >nul 2>&1

echo.
echo Building 32-bit wican_j2534.dll...
cl /LD /O2 /DWIN32 /D_WIN32_WINNT=0x0601 ^
   /Fe:bin\wican_j2534_32.dll ^
   wican_j2534.c wican_comm.c ^
   ws2_32.lib setupapi.lib advapi32.lib ^
   /link /DEF:wican_j2534.def

if %ERRORLEVEL% neq 0 (
    echo ERROR: 32-bit DLL build failed
)
echo 32-bit DLL build successful!

echo.
echo Building wican_config.exe...
cl /O2 /DWIN32 /D_WIN32_WINNT=0x0601 ^
   /Fe:bin\wican_config.exe ^
   wican_config.c ^
   comctl32.lib ws2_32.lib setupapi.lib advapi32.lib user32.lib gdi32.lib ^
   /link /SUBSYSTEM:WINDOWS

if %ERRORLEVEL% neq 0 (
    echo ERROR: Config utility build failed
) else (
    echo Config utility build successful!
)

REM Cleanup intermediate files
del *.obj 2>nul
del *.exp 2>nul
del *.lib 2>nul

echo.
echo ============================================
echo Build Complete!
echo ============================================
echo.
echo Output files:
dir /b bin\*.dll bin\*.exe 2>nul
echo.

endlocal
