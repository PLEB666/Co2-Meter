# Packs index.html into ..\web_page.h, gzip-compressed, so the firmware can serve it.
# Run it after every change to index.html, then upload the firmware again.
# Usage (PowerShell):  .\make_web_page.ps1
$ErrorActionPreference = 'Stop'

$source = Join-Path $PSScriptRoot 'index.html'
$target = Join-Path $PSScriptRoot '..\web_page.h'

$bytes = [System.IO.File]::ReadAllBytes($source)
$memory = New-Object System.IO.MemoryStream
$gzip = New-Object System.IO.Compression.GZipStream($memory, [System.IO.Compression.CompressionLevel]::Optimal)
$gzip.Write($bytes, 0, $bytes.Length)
$gzip.Close()
$packed = $memory.ToArray()

$rows = for ($i = 0; $i -lt $packed.Length; $i += 16) {
    $last = [Math]::Min($i + 15, $packed.Length - 1)
    '  ' + (($packed[$i..$last] | ForEach-Object { '0x{0:x2}' -f $_ }) -join ', ') + ','
}
$text = @(
    '// Made by web/make_web_page.ps1 from web/index.html - change that file, not this one.'
    "// The web page, gzip-compressed: $($packed.Length) bytes ($($bytes.Length) bytes uncompressed)"
    '#pragma once'
    ''
    '#include <Arduino.h>'
    ''
    'const uint8_t WEB_PAGE[] PROGMEM = {'
    $rows
    '};'
    ''
) -join "`n"
[System.IO.File]::WriteAllText([System.IO.Path]::GetFullPath($target), $text, (New-Object System.Text.UTF8Encoding $false))
Write-Host "web_page.h written: $($packed.Length) bytes ($($bytes.Length) bytes uncompressed)"
