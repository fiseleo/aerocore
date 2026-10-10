@echo off
setlocal

rem Native CMD launcher: no PowerShell execution-policy changes are needed.
call "%~dp0scripts\java-env.cmd"
if errorlevel 1 exit /b 1

set "IDEA_EXE="
set "CHECK_ONLY="
if /i "%~1"=="--check" (
    set "CHECK_ONLY=1"
) else if not "%~1"=="" (
    set "IDEA_EXE=%~1"
    if not exist "%~1" (
        echo [ERROR] The specified IntelliJ executable does not exist.
        exit /b 2
    )
)

if not defined IDEA_EXE (
    for /d %%D in (
        "%LOCALAPPDATA%\Programs\IntelliJ IDEA*"
        "%ProgramFiles%\JetBrains\IntelliJ IDEA*"
        "%LOCALAPPDATA%\JetBrains\Toolbox\apps\IntelliJ*"
        "%LOCALAPPDATA%\JetBrains\Toolbox\apps\IDEA*"
        "%LOCALAPPDATA%\JetBrains\Toolbox\apps\idea*"
    ) do if exist "%%~fD\bin\idea64.exe" set "IDEA_EXE=%%~fD\bin\idea64.exe"
)

if not defined IDEA_EXE (
    echo [ERROR] IntelliJ IDEA is not installed in a standard location.
    echo Install it first using the verified installer:
    echo "%LOCALAPPDATA%\MCMod\downloads\idea-2026.2.3.exe"
    echo Then run open-ide.cmd again.
    echo For a custom installation, pass the full path to bin\idea64.exe as an argument.
    exit /b 2
)

echo Java: "%JAVA_HOME%"
echo IDE: "%IDEA_EXE%"
echo Project: "%~dp0."
if defined CHECK_ONLY exit /b 0

set "TEMP=%USERPROFILE%\.mcmod\tmp"
set "TMP=%TEMP%"
if not exist "%TEMP%" mkdir "%TEMP%"
if not exist "%TEMP%" (
    echo [ERROR] Cannot create the local Java temporary directory.
    exit /b 3
)

rem Open the interactive IDE requested by the user, with the project JDK in its environment.
start "" /D "%~dp0." "%IDEA_EXE%" "%~dp0."
exit /b %ERRORLEVEL%
