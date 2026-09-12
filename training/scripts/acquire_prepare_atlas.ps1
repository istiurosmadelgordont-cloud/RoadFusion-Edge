$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$status = Join-Path $root 'reports\atlas_pipeline_status.json'
$archiveDir = Join-Path $root 'incoming\atlas\archives'
$extractDir = Join-Path $root 'incoming\atlas\full'
$sevenZip = Join-Path $root 'tools\7zip\extra\x64\7za.exe'
$python = Join-Path $root '.venv-train\Scripts\python.exe'

function Write-Status([string]$state, [string]$detail) {
    [ordered]@{
        state = $state
        detail = $detail
        updated_at = (Get-Date).ToString('o')
        pid = $PID
    } | ConvertTo-Json | Set-Content -LiteralPath $status -Encoding UTF8
}

try {
    Write-Status 'downloading' 'Downloading and validating six ATLAS split archives.'
    & (Join-Path $root 'scripts\download_atlas_full.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'ATLAS archive download failed.' }

    Write-Status 'extracting' 'Extracting the validated split ZIP on D drive.'
    New-Item -ItemType Directory -Force -Path $extractDir | Out-Null
    & $sevenZip x (Join-Path $archiveDir 'ATLAS.zip') "-o$extractDir" -y *>> (Join-Path $root 'reports\atlas_extract.log')
    if ($LASTEXITCODE -ne 0) { throw 'ATLAS extraction failed.' }

    $atlasRoot = Join-Path $extractDir 'ATLAS'
    if (-not (Test-Path -LiteralPath (Join-Path $atlasRoot 'ATLAS_classes.yaml'))) {
        throw "ATLAS root not found after extraction: $atlasRoot"
    }
    Write-Status 'preparing' 'Selecting balanced eight-light still-image crops, including straight arrows.'
    & $python (Join-Path $root 'scripts\prepare_atlas_eight_lights.py') `
        --source $atlasRoot `
        --output (Join-Path $root 'datasets\atlas_full_eight_lights') `
        --max-train-per-class 1500 `
        --max-val-per-class 300 *>> (Join-Path $root 'reports\atlas_prepare.log')
    if ($LASTEXITCODE -ne 0) { throw 'ATLAS eight-light preparation failed.' }

    Write-Status 'ready' 'ATLAS eight-light additions are prepared; inspect counts before Unified21 training.'
}
catch {
    Write-Status 'failed' $_.Exception.Message
    throw
}
