@echo off
echo ===============================================
echo AMR STL to Images Converter
echo For Fusion 360 and SolidWorks STL files
echo ===============================================
echo.

REM Check if Python is available
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7+ and try again
    pause
    exit /b 1
)

echo Installing required dependencies...
pip install -r requirements.txt

echo.
echo Starting STL conversion...
echo.

python stl_to_images.py

echo.
echo Conversion complete!
echo Check the 'images' folder for generated files.
echo.
pause