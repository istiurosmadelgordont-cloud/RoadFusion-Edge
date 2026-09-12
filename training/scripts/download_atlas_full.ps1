$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$python = Join-Path $root '.venv-train\Scripts\python.exe'
$downloader = Join-Path $root 'scripts\download_ranged.py'
$archiveDir = Join-Path $root 'incoming\atlas\archives'
$statusPath = Join-Path $root 'reports\atlas_download_status.json'
$logPath = Join-Path $root 'reports\atlas_download.log'

$files = @(
    [pscustomobject]@{ Name='ATLAS.z01'; Size=5368709120; Md5='0575c9140d3ecce9219cfd2b5ecdd2aa'; Parts=160 },
    [pscustomobject]@{ Name='ATLAS.z02'; Size=5368709120; Md5='2db9cbdd8c02d2981754bcea2a3e683e'; Parts=160 },
    [pscustomobject]@{ Name='ATLAS.z03'; Size=5368709120; Md5='671d93a5b78b843cb69c4715ea958f55'; Parts=160 },
    [pscustomobject]@{ Name='ATLAS.z04'; Size=5368709120; Md5='bb6d7f23c733e8c340c5d99e56d28e3d'; Parts=160 },
    [pscustomobject]@{ Name='ATLAS.z05'; Size=5368709120; Md5='625c2cb8862a66a4a025270001dda01b'; Parts=160 },
    [pscustomobject]@{ Name='ATLAS.zip'; Size=2646962590; Md5='2b84880001b21e1e0b344cef7ac4b0db'; Parts=79 }
)

New-Item -ItemType Directory -Force -Path $archiveDir | Out-Null
foreach ($file in $files) {
    [ordered]@{
        state = 'downloading'
        current = $file.Name
        updated_at = (Get-Date).ToString('o')
        completed = @($files | Where-Object { Test-Path -LiteralPath (Join-Path $archiveDir $_.Name) } | ForEach-Object Name)
    } | ConvertTo-Json | Set-Content -LiteralPath $statusPath -Encoding UTF8

    $url = "https://zenodo.org/api/records/14846709/files/$($file.Name)/content"
    $output = Join-Path $archiveDir $file.Name
    & $python $downloader $url $output --size $file.Size --md5 $file.Md5 --parts $file.Parts --workers 8 *>> $logPath
    if ($LASTEXITCODE -ne 0) {
        throw "ATLAS download failed: $($file.Name)"
    }
}

[ordered]@{
    state = 'completed'
    updated_at = (Get-Date).ToString('o')
    completed = @($files | ForEach-Object Name)
    archive_dir = $archiveDir
} | ConvertTo-Json | Set-Content -LiteralPath $statusPath -Encoding UTF8
