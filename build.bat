@echo off
REM ============================================================================
REM  STM8S103F3P6 HC-SR04 + 2-Digit 7-Seg LED - Windows Build Script (SDCC)
REM ============================================================================
REM  Prerequisites:
REM    1. Install SDCC: https://sdcc.sourceforge.net/
REM    2. Ensure sdcc.exe is in your PATH
REM ============================================================================

setlocal enabledelayedexpansion

set PROJECT=project5
set SRC1=main.c
set SRC2=hcsr04.c
set SRC3=segled.c
set SRC4=uart.c
set SRC5=beep.c
set REL1=main.rel
set REL2=hcsr04.rel
set REL3=segled.rel
set REL4=uart.rel
set REL5=beep.rel
set OUTPUT=%PROJECT%.ihx
set HEX=%PROJECT%.hex

echo.
echo ========================================
echo   STM8S103 HC-SR04 + 7SEG - Build Script
echo ========================================
echo.

REM --- Check if SDCC is installed ---
where sdcc >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] SDCC not found in PATH.
    echo         Please install SDCC from https://sdcc.sourceforge.net/
    pause
    exit /b 1
)

echo [INFO] Compiler: SDCC
sdcc --version 2>&1 | findstr /C:"SDCC"
echo.

REM --- Check if source files exist ---
for %%F in ("%SRC1%" "%SRC2%" "%SRC3%" "%SRC4%" "%SRC5%") do (
    if not exist %%~F (
        echo [ERROR] Source file '%%~nxF' not found.
        pause
        exit /b 1
    )
)

echo [INFO] Sources: %SRC1%  %SRC2%  %SRC3%  %SRC4%  %SRC5%
echo.

REM ====== Compile ======
for %%S in (%SRC1% %SRC2% %SRC3% %SRC4% %SRC5%) do (
    echo [STEP] Compiling %%S...
    REM --std-c99 is REQUIRED: without it SDCC's va_arg handling for the
    REM variadic uart_printf() misreads promoted arguments -> garbled UART
    REM output (same pitfall recorded for project1).
    sdcc -mstm8 --std-c99 -c %%S
    if !ERRORLEVEL! NEQ 0 (
        echo [ERROR] Compilation of %%S failed!
        pause
        exit /b 1
    )
)

echo [STEP] Linking...
sdcc -mstm8 --std-c99 %REL1% %REL2% %REL3% %REL4% %REL5% -o "%OUTPUT%"

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Compilation failed! Check errors above.
    pause
    exit /b 1
)

echo [ OK ] Compilation successful.

REM ====== Generate Intel HEX ======
if exist "%OUTPUT%" (
    echo [STEP] Generating Intel HEX...
    packihx "%OUTPUT%" > "%HEX%"
    if exist "%HEX%" (
        echo [ OK ] HEX file generated: %HEX%
    ) else (
        echo [WARN] packihx may have failed. HEX not generated.
    )
)

REM ====== Summary ======
echo.
echo ========================================
echo   Build Summary
echo ========================================
if exist "%OUTPUT%" echo   %OUTPUT%  -  IHX output
if exist "%HEX%"    echo   %HEX%     -  Intel HEX file
if exist "%PROJECT%.lk"   echo   %PROJECT%.lk    -  linker script
if exist "%PROJECT%.map"  echo   %PROJECT%.map   -  memory map
if exist "%PROJECT%.lst"  echo   %PROJECT%.lst   -  assembly listing
echo ========================================
echo.

REM ====== Flash options ======
echo Flash commands:
echo   ST-LINK/V2:  stm8flash -c stlinkv2 -p stm8s103f3 -w %OUTPUT%
echo.

REM ====== Clean up intermediate files ======
echo [STEP] Cleaning intermediate files...
for %%x in (asm lst rst rel sym map lk mem) do (
    if exist "%PROJECT%.%%x"  del "%PROJECT%.%%x"
    for %%S in (main hcsr04 segled uart beep) do (
        if exist "%%S.%%x" del "%%S.%%x"
    )
)
echo [ OK ] Intermediate files removed.

echo.
pause
endlocal
