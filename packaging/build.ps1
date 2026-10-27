# Build a release: the app (PyInstaller) and its installer (Inno Setup).
#
#   powershell -ExecutionPolicy Bypass -File packaging\build.ps1
#
# Version comes from APP_VERSION in hadi_ocr_app.py. Output:
#   dist\HADI_OCR_v<ver>\                  the app folder
#   dist\installer\HADI_OCR_Setup_v<ver>.exe   the one file to hand out
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$version = (Select-String -Path "hadi_ocr_app.py" -Pattern '^APP_VERSION = "([^"]+)"').Matches[0].Groups[1].Value
Write-Host "Building HADI OCR v$version" -ForegroundColor Cyan

$iscc = @("$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe",
          "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe",
          "$env:ProgramFiles\Inno Setup 6\ISCC.exe") | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $iscc) { throw "Inno Setup not found. Install it with: winget install JRSoftware.InnoSetup" }

Write-Host "[1/2] Building the app (PyInstaller)..." -ForegroundColor Cyan
python -m PyInstaller packaging\hadi_ocr.spec --noconfirm --distpath dist --workpath build
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed" }
if (-not (Test-Path "dist\HADI_OCR_v$version\_internal\rapidocr\models\PP-OCRv6_rec_tiny.onnx")) {
    throw "OCR model missing from the build - run the app once from source first so it downloads."
}

Write-Host "[2/2] Building the installer (Inno Setup)..." -ForegroundColor Cyan
& $iscc /Qp "/DAppVer=$version" packaging\installer.iss
if ($LASTEXITCODE -ne 0) { throw "Inno Setup failed" }

$setup = Get-Item "dist\installer\HADI_OCR_Setup_v$version.exe"
Write-Host ("Done: {0}  ({1:N0} MB)" -f $setup.FullName, ($setup.Length / 1MB)) -ForegroundColor Green
