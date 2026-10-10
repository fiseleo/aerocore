param([string] $RuntimeId = 'aerocore')
$ErrorActionPreference = 'Stop'
if ($RuntimeId -notmatch '^[A-Za-z0-9_-]+$') { throw 'Invalid RuntimeId' }
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$sourcePath = Join-Path $projectRoot 'neoforge\run'
$targetRoot = [System.IO.Path]::GetFullPath((Join-Path $env:USERPROFILE ".mcmod\runs\$RuntimeId"))
$targetPath = Join-Path $targetRoot 'client'
if ($targetRoot.StartsWith('\\')) { throw 'The runtime target must be a local disk.' }
if (-not (Test-Path -LiteralPath $sourcePath -PathType Container)) { throw "Source runtime not found: $sourcePath" }
# Acquire every world's Minecraft lock while taking this offline snapshot. Keep the originals.
$heldLocks = [System.Collections.Generic.List[System.IO.FileStream]]::new()
try {
    foreach ($runtimePath in @($sourcePath, $targetPath)) {
        if (-not (Test-Path -LiteralPath $runtimePath -PathType Container)) { continue }
        foreach ($lock in (Get-ChildItem -LiteralPath $runtimePath -Filter session.lock -Recurse -File)) {
            $heldLocks.Add([System.IO.File]::Open($lock.FullName, 'Open', 'ReadWrite', 'None'))
        }
    }
    $files = @(Get-ChildItem -LiteralPath $sourcePath -Recurse -File | Where-Object Name -ne 'session.lock')
    $manifest = [System.Collections.Generic.List[object]]::new()
    # Preflight all collisions before copying anything. Re-running never overwrites newer local data.
    foreach ($file in $files) {
        $relative = $file.FullName.Substring($sourcePath.TrimEnd('\').Length + 1)
        $destination = [System.IO.Path]::GetFullPath((Join-Path $targetPath $relative))
        if (-not $destination.StartsWith($targetPath + '\', [System.StringComparison]::OrdinalIgnoreCase)) { throw 'Path escaped runtime directory' }
        $hash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash
        if ((Test-Path -LiteralPath $destination) -and (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash -ne $hash) {
            throw "Local data differs; refusing overwrite: $destination"
        }
        $manifest.Add([pscustomobject]@{Path=$relative; Bytes=$file.Length; SHA256=$hash})
    }
    foreach ($item in $manifest) {
        $sourceFile = Join-Path $sourcePath $item.Path
        $destination = Join-Path $targetPath $item.Path
        New-Item -ItemType Directory -Path ([System.IO.Path]::GetDirectoryName($destination)) -Force | Out-Null
        if (-not (Test-Path -LiteralPath $destination)) { Copy-Item -LiteralPath $sourceFile -Destination $destination }
        if ((Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash -ne $item.SHA256) { throw "Copy verification failed: $destination" }
        if ((Get-FileHash -LiteralPath $sourceFile -Algorithm SHA256).Hash -ne $item.SHA256) { throw "Source changed during migration: $sourceFile" }
    }
    $report = [pscustomobject]@{Source=$sourcePath; Target=$targetPath; VerifiedUtc=[DateTime]::UtcNow.ToString('o'); Files=$manifest}
    $report | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $targetRoot 'migration-manifest.json') -Encoding utf8
    Write-Output "Verified $($manifest.Count) files in $targetPath"
    Write-Output 'Original network files retained as a backup. Do not run the old and new world copies simultaneously.'
} finally {
    foreach ($heldLock in $heldLocks) { $heldLock.Dispose() }
}
