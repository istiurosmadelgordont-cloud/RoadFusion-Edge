param([string]$Gcc='gcc',[string]$OutputDirectory='')
$ErrorActionPreference='Stop'
if(!$OutputDirectory){$OutputDirectory=Join-Path $PSScriptRoot '.out'}
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$binary=Join-Path $OutputDirectory 'rc_reference_test.exe'
& $Gcc -std=c99 -Wall -Wextra -Werror (Join-Path $PSScriptRoot 'rc_reference_test.c') -o $binary
if($LASTEXITCODE -ne 0){throw 'RC reference build failed'}
& $binary
if($LASTEXITCODE -ne 0){throw 'RC reference tests failed'}
