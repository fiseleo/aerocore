$ErrorActionPreference = 'Stop'
$javaHome = & "$env:SystemRoot\System32\cmd.exe" /d /c ('call "' + (Join-Path $PSScriptRoot 'scripts\java-env.cmd') + '" --print')
if ($LASTEXITCODE -ne 0) { throw 'No JDK 21 found. Run .\scripts\setup-java.ps1 first.' }
$javaHome = ($javaHome | Select-Object -Last 1).Trim()
if (-not (Test-Path -LiteralPath (Join-Path $javaHome 'bin\javac.exe'))) {
    throw 'JDK 21 is missing. Run .\scripts\setup-java.ps1 first.'
}

$ideaCandidates = @()
$installRoots = @(
    (Join-Path $env:LOCALAPPDATA 'Programs'),
    'C:\Program Files\JetBrains',
    (Join-Path $env:LOCALAPPDATA 'JetBrains\Toolbox\apps')
)
foreach ($installRoot in $installRoots) {
    if (Test-Path -LiteralPath $installRoot) {
        $ideaCandidates += Get-ChildItem -LiteralPath $installRoot -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match 'IntelliJ|IDEA|^idea' } |
            ForEach-Object {
                $candidate = Join-Path $_.FullName 'bin\idea64.exe'
                if (Test-Path -LiteralPath $candidate) { Get-Item -LiteralPath $candidate }
            }
    }
}
$idea = $ideaCandidates | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $idea) {
    throw 'IntelliJ IDEA is not installed in a standard location. Install it first, then run this script again.'
}

$previousJavaHome = $env:JAVA_HOME
$previousTemp = $env:TEMP
$previousTmp = $env:TMP
try {
    $env:JAVA_HOME = $javaHome
    $javaTemp = Join-Path $env:USERPROFILE '.mcmod\tmp'
    New-Item -ItemType Directory -Force -Path $javaTemp | Out-Null
    $env:TEMP = $javaTemp
    $env:TMP = $javaTemp
    # This is the interactive IDE requested by the user, so show its main window.
    Start-Process -FilePath $idea.FullName -ArgumentList ('"' + $PSScriptRoot + '"') -WorkingDirectory $PSScriptRoot -WindowStyle Normal
} finally {
    $env:JAVA_HOME = $previousJavaHome
    $env:TEMP = $previousTemp
    $env:TMP = $previousTmp
}
