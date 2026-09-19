$ErrorActionPreference = 'Stop'

$packageRoot = Split-Path -Parent $PSScriptRoot
$partsDirectory = Join-Path $packageRoot 'project_parts'
$outputPath = Join-Path $packageRoot 'hdmi_ddr_pice_handshake_20260919.zip'
$expectedHash = '1766B7285FCAFEC8C9E39230B77E2E83CE771B5F0E5C773DFB33D1F14680C576'
$parts = @(Get-ChildItem -LiteralPath $partsDirectory -File -Filter '*.part*' | Sort-Object Name)

if ($parts.Count -ne 28) {
    throw "Expected 28 archive parts, found $($parts.Count)."
}

$output = [System.IO.File]::Create($outputPath)
try {
    foreach ($part in $parts) {
        $input = [System.IO.File]::OpenRead($part.FullName)
        try {
            $input.CopyTo($output)
        }
        finally {
            $input.Dispose()
        }
    }
}
finally {
    $output.Dispose()
}

$actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $outputPath).Hash
if ($actualHash -ne $expectedHash) {
    throw "SHA-256 mismatch: expected $expectedHash, got $actualHash."
}

Write-Host "Created: $outputPath"
Write-Host "SHA-256 verified: $actualHash"

