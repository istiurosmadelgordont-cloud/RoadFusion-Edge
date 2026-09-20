param(
    [Parameter(Mandatory=$true)]
    [string]$RepositoryRoot,
    [ValidateSet('Compare','Pull','Push')]
    [string]$Mode = 'Compare'
)

$ErrorActionPreference = 'Stop'
$local = Join-Path $PSScriptRoot 'source'
$tracked = Join-Path $RepositoryRoot 'fpga\ethernet_video\source_960x540'
if (-not (Test-Path -LiteralPath $local)) { throw "Local source directory not found: $local" }
if (-not (Test-Path -LiteralPath $tracked)) { throw "Repository source directory not found: $tracked" }

if ($Mode -eq 'Pull') {
    Copy-Item -Path (Join-Path $tracked '*') -Destination $local -Force
    Write-Host 'Repository 960x540 source copied to the local PDS project.'
    exit 0
}
if ($Mode -eq 'Push') {
    Copy-Item -Path (Join-Path $local '*') -Destination $tracked -Force
    Write-Host 'Local PDS source copied to the repository checkout. Review and commit there.'
    exit 0
}

$names = @(Get-ChildItem -LiteralPath $local -File -Filter '*.v' | Select-Object -ExpandProperty Name) +
         @(Get-ChildItem -LiteralPath $tracked -File -Filter '*.v' | Select-Object -ExpandProperty Name) |
         Sort-Object -Unique
$different = foreach ($name in $names) {
    $a = Join-Path $local $name
    $b = Join-Path $tracked $name
    if (-not (Test-Path -LiteralPath $a) -or -not (Test-Path -LiteralPath $b) -or
        (Get-FileHash -LiteralPath $a -Algorithm SHA256).Hash -ne (Get-FileHash -LiteralPath $b -Algorithm SHA256).Hash) {
        $name
    }
}
if ($different) {
    Write-Host 'Different files:'
    $different | ForEach-Object { Write-Host "  $_" }
    exit 1
}
Write-Host 'Local and repository source files are identical.'
