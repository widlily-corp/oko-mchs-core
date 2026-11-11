$PSScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $PSScriptRoot
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "   ОКО МЧС: ТАКТИЧЕСКИЙ АРМ ОПЕРАТОРА (WINDOWS GUI)    " -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Cyan
python "\\wsl.localhost\Ubuntu-22.04\root\oko-mchs-core\src\mchs_ui\app.py"
