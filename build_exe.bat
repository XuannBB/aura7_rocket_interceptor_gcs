@echo off
REM ============================================================
REM  Aura 7" Ground Station - Automated Build Script
REM  Packages the simulation suite into a standalone Windows .exe
REM ============================================================

echo.
echo ========================================================
echo   Aura 7" Ground Station - PyInstaller Build
echo ========================================================
echo.

REM Step 1: Install / upgrade PyInstaller
echo [1/2] Installing PyInstaller...
pip install pyinstaller
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install PyInstaller. Check your Python environment.
    pause
    exit /b 1
)

echo.

REM Step 2: Run PyInstaller to compile the standalone executable
echo [2/2] Building Aura7_GroundStation.exe ...
pyinstaller --onefile --name=Aura7_GroundStation --add-data "rocket_simulation.csv;." --clean app.py
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: PyInstaller build failed. See output above.
    pause
    exit /b 1
)

echo.
echo ========================================================
echo   BUILD COMPLETE
echo   Output: dist\Aura7_GroundStation.exe
echo ========================================================
echo.
pause
