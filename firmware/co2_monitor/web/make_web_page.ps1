# Packs index.html into ..\web_page.h, gzip-compressed, so the firmware can serve it.
# It also writes the mockup docs\web_page_mockup.html: the same page with mockup.js, which
# makes up the values, so the page can be seen without a monitor.
# Run it after every change to index.html or mockup.js, then upload the firmware again.
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

# The mockup: mockup.js goes in front of the page's own script, so its fake /api answers
# are in place before the page asks for anything
$page = [System.IO.File]::ReadAllText($source)
$mock = [System.IO.File]::ReadAllText((Join-Path $PSScriptRoot 'mockup.js'))
$main = [regex]::Match($page, "<script>\r?\n'use strict';")
if (-not $main.Success) { throw "index.html: the page's main script (<script> + 'use strict';) not found" }
$page = $page.Substring(0, $main.Index) + "<script>`n" + $mock.TrimEnd() + "`n</script>`n" + $page.Substring($main.Index)
$page = $page.Replace('<title>CO2 monitor</title>', '<title>CO2 monitor mockup</title>')
$page = $page -replace '^<!doctype html>\r?\n', "<!doctype html>`n<!-- Made by firmware/co2_monitor/web/make_web_page.ps1 from index.html and mockup.js - change those, not this file. -->`n"
$mockup = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..\docs\web_page_mockup.html'))
[System.IO.File]::WriteAllText($mockup, $page, (New-Object System.Text.UTF8Encoding $false))
Write-Host "docs\web_page_mockup.html written: $($page.Length) characters"
