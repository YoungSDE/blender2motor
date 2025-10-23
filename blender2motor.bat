@echo off
chcp 65001 > nul
echo ====================================
echo Dynamixel 애니메이션 플레이어
echo ====================================
echo.

REM Python 설치 확인
python --version > nul 2>&1
if errorlevel 1 (
    echo [오류] Python이 설치되어 있지 않습니다.
    echo Python을 먼저 설치해주세요: https://www.python.org/downloads/
    echo.
    pause
    exit /b 1
)

echo [확인] Python이 설치되어 있습니다.
echo.

REM 필요한 패키지 설치
echo 필요한 패키지를 설치합니다...
echo.

python -m pip install --upgrade pip --quiet
if errorlevel 1 (
    echo [경고] pip 업그레이드에 실패했지만 계속 진행합니다.
)

echo [1/4] numpy 설치 중...
python -m pip install numpy --quiet
if errorlevel 1 (
    echo [오류] numpy 설치 실패
    pause
    exit /b 1
)

echo [2/4] matplotlib 설치 중...
python -m pip install matplotlib --quiet
if errorlevel 1 (
    echo [오류] matplotlib 설치 실패
    pause
    exit /b 1
)

echo [3/4] dynamixel_sdk 설치 중...
python -m pip install dynamixel_sdk --quiet
if errorlevel 1 (
    echo [오류] dynamixel_sdk 설치 실패
    pause
    exit /b 1
)

echo [4/4] 설치 완료!
echo.
echo ====================================
echo 프로그램을 실행합니다...
echo ====================================
echo.

REM 프로그램 실행
python dynamixel_control.py

echo.
echo 프로그램이 종료되었습니다.
pause