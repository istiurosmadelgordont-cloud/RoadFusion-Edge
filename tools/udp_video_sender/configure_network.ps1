# Run in an elevated PowerShell. Does not change the adapter IP address.
param(
    [Parameter(Mandatory=$true)][string]$LocalIP,
    [string]$TargetIP = '192.168.1.10',
    [string]$TargetMAC = 'A0-B1-C2-D3-E4-E4'
)
$ErrorActionPreference = 'Stop'
$parsed = [ipaddress]::Parse($LocalIP)
if ($parsed.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) { throw 'IPv4 required' }
$target = [ipaddress]::Parse($TargetIP)
if ($target.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) { throw 'Target IPv4 required' }
$TargetIP = $target.IPAddressToString
if ($parsed.Equals($target)) { throw 'Local and target IP must differ' }
$TargetMAC = $TargetMAC.Trim().ToUpperInvariant()
if ($TargetMAC -notmatch '^(?:[0-9A-F]{12}|(?:[0-9A-F]{2}:){5}[0-9A-F]{2}|(?:[0-9A-F]{2}-){5}[0-9A-F]{2})$') { throw 'Invalid target MAC' }
$macHex = $TargetMAC.Replace(':','').Replace('-','')
if (([Convert]::ToInt32($macHex.Substring(0,2),16) -band 1) -ne 0 -or $macHex -eq '000000000000') { throw 'Target MAC must be nonzero unicast' }
$TargetMAC = (0..5 | ForEach-Object { $macHex.Substring($_ * 2,2) }) -join '-'
$adapterAddress = @(Get-NetIPAddress -AddressFamily IPv4 -IPAddress $parsed.IPAddressToString)
if ($adapterAddress.Count -ne 1) { throw 'Choose a unique IPv4 address already assigned to the FPGA Ethernet adapter' }
$idx = $adapterAddress[0].InterfaceIndex
$prefix = [int]$adapterAddress[0].PrefixLength
$localBytes = $parsed.GetAddressBytes()
$targetBytes = $target.GetAddressBytes()
foreach ($bytes in @($localBytes, $targetBytes)) {
    if ($bytes[0] -eq 0 -or $bytes[0] -eq 127 -or $bytes[0] -ge 224) { throw 'Use unicast adapter IPv4 addresses' }
    $hostZero = $true
    $hostOnes = $true
    for ($i = 0; $i -lt 4; $i++) {
        $bits = [Math]::Min(8, [Math]::Max(0, $prefix - 8 * $i))
        $mask = (255 -shl (8 - $bits)) -band 255
        if (($bytes[$i] -band $mask) -ne ($localBytes[$i] -band $mask)) { throw 'Target must be in the local adapter subnet' }
        $hostMask = 255 -bxor $mask
        if (($bytes[$i] -band $hostMask) -ne 0) { $hostZero = $false }
        if (($bytes[$i] -band $hostMask) -ne $hostMask) { $hostOnes = $false }
    }
    if ($prefix -lt 31 -and ($hostZero -or $hostOnes)) { throw 'Network and broadcast IP addresses are not allowed' }
}
$existing = Get-NetNeighbor -InterfaceIndex $idx -IPAddress $TargetIP -ErrorAction SilentlyContinue
if ($existing) {
    $existing | Remove-NetNeighbor -Confirm:$false
}
New-NetNeighbor -InterfaceIndex $idx -IPAddress $TargetIP -LinkLayerAddress $TargetMAC -State Permanent
Get-NetNeighbor -InterfaceIndex $idx -IPAddress $TargetIP
