$ErrorActionPreference = 'Stop'
$installRoot = Join-Path $env:USERPROFILE '.mcmod\java'
$javaHome = Join-Path $installRoot 'jdk-21.0.12.1+1'
if (-not (Test-Path -LiteralPath (Join-Path $javaHome 'bin\javac.exe'))) {
    New-Item -ItemType Directory -Force -Path $installRoot | Out-Null
    $archive = Join-Path $installRoot 'microsoft-jdk-21.0.12.1.zip'
    $uri = 'https://aka.ms/download-jdk/microsoft-jdk-21.0.12.1-windows-x64.zip'
    & curl.exe --fail --location --retry 3 --silent --show-error --output $archive $uri
    if ($LASTEXITCODE -ne 0) { throw 'JDK download failed.' }
    $expectedHash = '192441a9d27da813bada974bb88b4cf64d37a9589ed37f204374d411ca5ce07f'
    if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $expectedHash) {
        throw 'JDK checksum mismatch; archive was not extracted.'
    }
    Expand-Archive -LiteralPath $archive -DestinationPath $installRoot -Force
}
& (Join-Path $javaHome 'bin\java.exe') -version
Write-Output "Java is ready at: $javaHome"
