# Regenerates CO2_Monitor_Documentation.pdf from the HTML file next to this script.
# Usage (PowerShell):  .\build_pdf.ps1
$ErrorActionPreference = 'Stop'

$html = Join-Path $PSScriptRoot 'CO2_Monitor_Documentation.html'
$pdf  = Join-Path $PSScriptRoot 'CO2_Monitor_Documentation.pdf'

$candidates = @(
    "$env:LOCALAPPDATA\Google\Chrome\Application\chrome.exe",
    "$env:ProgramFiles\Google\Chrome\Application\chrome.exe",
    "${env:ProgramFiles(x86)}\Google\Chrome\Application\chrome.exe",
    "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe",
    "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe"
)
$candidates += Get-ChildItem "${env:ProgramFiles(x86)}\Microsoft\EdgeCore\*\msedge.exe" -ErrorAction SilentlyContinue |
    Sort-Object FullName -Descending | ForEach-Object FullName

$browser = $candidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
if (-not $browser) { throw 'Neither Chrome nor Edge was found.' }

$profileDir = Join-Path $env:TEMP 'co2-docs-pdf-profile'
$url = ([System.Uri]$html).AbsoluteUri
if (Test-Path $pdf) { Remove-Item $pdf }

$arguments = @(
    '--headless=new', '--disable-gpu', '--no-first-run', '--no-pdf-header-footer',
    "--user-data-dir=`"$profileDir`"", "--print-to-pdf=`"$pdf`"", $url
)
# Start-Process waits for the browser and ignores its harmless stderr chatter
Start-Process -FilePath $browser -ArgumentList $arguments -Wait -WindowStyle Hidden

if (-not (Test-Path $pdf)) { throw "PDF was not created ($browser)." }
Write-Host "PDF written: $pdf ($([math]::Round((Get-Item $pdf).Length / 1KB)) KB)"
