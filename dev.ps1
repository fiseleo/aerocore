param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $GradleArguments = @('build')
)
$ErrorActionPreference = 'Stop'
$projectJava = & "$env:SystemRoot\System32\cmd.exe" /d /c ('call "' + (Join-Path $PSScriptRoot 'scripts\java-env.cmd') + '" --print')
if ($LASTEXITCODE -ne 0) { throw 'No JDK 21 found. Run .\scripts\setup-java.ps1 first.' }
$projectJava = ($projectJava | Select-Object -Last 1).Trim()
$previousJavaHome = $env:JAVA_HOME
$previousTemp = $env:TEMP
$previousTmp = $env:TMP
try {
    if (Test-Path -LiteralPath (Join-Path $projectJava 'bin\javac.exe')) {
        $env:JAVA_HOME = $projectJava
    } elseif (-not $env:JAVA_HOME) {
        throw 'Run .\scripts\setup-java.ps1 first, or set JAVA_HOME to a JDK 21 installation.'
    }
    # A dedicated local temp directory avoids Windows packaged-app socket path redirection.
    $javaTemp = Join-Path $env:USERPROFILE '.mcmod\tmp'
    New-Item -ItemType Directory -Force -Path $javaTemp | Out-Null
    $env:TEMP = $javaTemp
    $env:TMP = $javaTemp
    Push-Location $PSScriptRoot
    try {
        # Launch the official wrapper directly; this also works from mapped network drives.
        & (Join-Path $env:JAVA_HOME 'bin\java.exe') "-Djdk.net.unixdomain.tmpdir=$javaTemp" -Xmx64m -Xms64m -jar (Join-Path $PSScriptRoot 'gradle\wrapper\gradle-wrapper.jar') @GradleArguments
        $result = $LASTEXITCODE
    } finally {
        Pop-Location
    }
} finally {
    $env:JAVA_HOME = $previousJavaHome
    $env:TEMP = $previousTemp
    $env:TMP = $previousTmp
}
exit $result
