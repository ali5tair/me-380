param([string]$Port = "COM9", [int]$Seconds = 10, [int]$Baud = 115200)
$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud
$sp.DtrEnable = $true
$sp.Open()
$end = (Get-Date).AddSeconds($Seconds)
while ((Get-Date) -lt $end) {
  $s = $sp.ReadExisting()
  if ($s) { Write-Host -NoNewline $s }
  Start-Sleep -Milliseconds 50
}
$sp.Close()
