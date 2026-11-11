Set-Location $PSScriptRoot
Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "   ОКО МЧС: ТАКТИЧЕСКИЙ АРМ ОПЕРАТОРА (WINDOWS GUI)    " -ForegroundColor Green
Write-Host "========================================================" -ForegroundColor Cyan
python "$PSScriptRoot\src\mchs_ui\app.py"
