@echo off
setlocal
set "AEROCORE_JDK="

rem Only use complete JDK 21 installations. The user profile is visible outside packaged apps.
if defined JAVA_HOME call :try_jdk "%JAVA_HOME%"
for /d %%J in (
    "%USERPROFILE%\.mcmod\java\jdk-21*"
    "%USERPROFILE%\.jdks\*21*"
    "%ProgramFiles%\Microsoft\jdk-21*"
    "%ProgramFiles%\Java\jdk-21*"
    "%ProgramFiles%\Eclipse Adoptium\jdk-21*"
) do call :try_jdk "%%~fJ"

if not defined AEROCORE_JDK (
    echo [ERROR] No complete JDK 21 installation was found. 1>&2
    echo Expected location: "%USERPROFILE%\.mcmod\java" 1>&2
    echo You can also set JAVA_HOME to an existing JDK 21 installation. 1>&2
    exit /b 1
)

for %%J in ("%AEROCORE_JDK%") do (
    endlocal
    set "JAVA_HOME=%%~J"
)
if /i "%~1"=="--print" echo %JAVA_HOME%
exit /b 0

:try_jdk
if defined AEROCORE_JDK exit /b 0
if not exist "%~1\bin\java.exe" exit /b 0
if not exist "%~1\bin\javac.exe" exit /b 0
if not exist "%~1\release" exit /b 0
%SystemRoot%\System32\findstr.exe /b /r /c:"JAVA_VERSION=.21[.]" "%~1\release" >nul
if errorlevel 1 exit /b 0
set "AEROCORE_JDK=%~1"
exit /b 0
