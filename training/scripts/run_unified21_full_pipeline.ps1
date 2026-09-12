param(
    [int]$Epochs = 24,
    [int]$Batch = 16,
    [switch]$Shutdown
)

$ErrorActionPreference = 'Stop'
$scripts = $PSScriptRoot

& (Join-Path $scripts 'acquire_prepare_atlas.ps1')
if ($LASTEXITCODE -ne 0) { throw 'ATLAS acquisition/preparation failed.' }

if ($Shutdown) {
    & (Join-Path $scripts 'continue_after_atlas_unified21.ps1') -Epochs $Epochs -Batch $Batch -Shutdown
} else {
    & (Join-Path $scripts 'continue_after_atlas_unified21.ps1') -Epochs $Epochs -Batch $Batch
}
if ($LASTEXITCODE -ne 0) { throw 'Unified21 continuation pipeline stopped before completion.' }
