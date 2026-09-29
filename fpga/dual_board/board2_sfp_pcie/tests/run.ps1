param([string]$Iverilog='iverilog',[string]$Vvp='vvp',[int]$ImagePackets=2,[string]$OutputDirectory='')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path $PSScriptRoot -Parent
if(!$OutputDirectory){$OutputDirectory=Join-Path $PSScriptRoot '.out'}
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$outputPath=Join-Path $OutputDirectory "handshake_$ImagePackets.vvp"
& $Iverilog -g2012 -Wall -s handshake_tb "-Phandshake_tb.IMAGE_PACKETS=$ImagePackets" -o $outputPath (Join-Path $projectRoot 'source/source/pcie_dma_ctrl.v') (Join-Path $PSScriptRoot 'fifo_model.v') (Join-Path $PSScriptRoot 'handshake_tb.sv')
if($LASTEXITCODE -ne 0){throw 'Verilog compilation failed'}
& $Vvp $outputPath
if($LASTEXITCODE -ne 0){throw 'Handshake simulation failed'}
