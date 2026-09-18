# Run in an elevated PowerShell. Does not change the adapter IP address.
param([Parameter(Mandatory=$true)][string]$LocalIP)
$ErrorActionPreference = 'Stop'
$parsed = [ipaddress]::Parse($LocalIP)
if ($parsed.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) { throw 'IPv4 required' }
$adapterAddress = @(Get-NetIPAddress -AddressFamily IPv4 -IPAddress $parsed.IPAddressToString)
if ($adapterAddress.Count -ne 1) { throw 'Choose a unique IPv4 address already assigned to the FPGA Ethernet adapter' }
$idx = $adapterAddress[0].InterfaceIndex
$existing = Get-NetNeighbor -InterfaceIndex $idx -IPAddress '192.168.1.10' -ErrorAction SilentlyContinue
if ($existing) {
    $existing | Remove-NetNeighbor -Confirm:$false
}
New-NetNeighbor -InterfaceIndex $idx -IPAddress '192.168.1.10' -LinkLayerAddress 'A0-B1-C2-D3-E4-E4' -State Permanent
Get-NetNeighbor -InterfaceIndex $idx -IPAddress '192.168.1.10'
