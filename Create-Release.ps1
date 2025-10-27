# Create-Release.ps1
# This script builds the project in Release mode, bundles all necessary dependencies,
# and creates a distributable zip archive.

# Stop on errors
$ErrorActionPreference = "Stop"

# Get script's directory and set it as current location
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir

Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "  Creating Release Package" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""

# 1. Build the project using a temporary batch script to avoid pauses
Write-Host "[1/5] Building project in Release mode..." -ForegroundColor Yellow
try {
    $BuildScriptContent = Get-Content -Path "build.bat" -Raw
    $TempBuildScriptContent = $BuildScriptContent -replace "pause", ""
    Set-Content -Path "temp_build.bat" -Value $TempBuildScriptContent

    # Execute the temporary build script
    ./temp_build.bat

    # Clean up the temporary script
    Remove-Item -Path "temp_build.bat"
} catch {
    Write-Host "Error: Build failed. Please fix the build issues before creating a release." -ForegroundColor Red
    # Clean up the temporary script in case of failure
    if (Test-Path "temp_build.bat") {
        Remove-Item -Path "temp_build.bat"
    }
    exit 1
}
Write-Host "      ✓ Build successful" -ForegroundColor Green
Write-Host ""

# 2. Create a clean release directory
Write-Host "[2/5] Preparing release directory..." -ForegroundColor Yellow
$ReleaseDir = "release"
if (Test-Path $ReleaseDir) {
    Write-Host "      Cleaning up old release directory..." -ForegroundColor Gray
    Remove-Item -Recurse -Force $ReleaseDir
}
New-Item -ItemType Directory -Path $ReleaseDir
Write-Host "      ✓ Release directory created" -ForegroundColor Green
Write-Host ""

# 3. Copy executables
Write-Host "[3/5] Copying executables..." -ForegroundColor Yellow
$InstallBinDir = "install/bin"
Copy-Item -Path "$InstallBinDir/*.exe" -Destination $ReleaseDir
Write-Host "      ✓ Executables copied" -ForegroundColor Green
Write-Host ""

# 4. Find and copy dependencies
Write-Host "[4/5] Bundling dependencies..." -ForegroundColor Yellow

# Find MinGW bin directory from where objdump is located
try {
    $ObjdumpPath = (Get-Command objdump).Source
    $MinGWBinDir = Split-Path -Parent $ObjdumpPath
    Write-Host "      Found MinGW bin directory: $MinGWBinDir" -ForegroundColor Gray
} catch {
    Write-Host "      ✗ Error: objdump.exe not found." -ForegroundColor Red
    Write-Host "      Please ensure the MinGW toolchain (containing objdump) is in your system's PATH." -ForegroundColor White
    exit 1
}

# List of system DLLs to ignore
$SystemDlls = @(
    "ADVAPI32.dll", "COMCTL32.dll", "COMDLG32.dll", "GDI32.dll",
    "KERNEL32.dll", "msvcrt.dll", "NTDLL.DLL", "ole32.dll",
    "OLEAUT32.dll", "SHELL32.dll", "USER32.dll", "WINMM.dll",
    "WS2_32.dll", "WINSPOOL.DRV", "IMM32.DLL", "VERSION.dll",
    "SETUPAPI.dll", "WTSAPI32.dll", "WINTRUST.dll", "CRYPT32.dll",
    "bcrypt.dll", "dwmapi.dll", "uxtheme.dll"
)

$CopiedFiles = @{}
$Queue = [System.Collections.Queue]::new()

# Initial queue with executables
Get-ChildItem -Path $ReleaseDir -Filter "*.exe" | ForEach-Object {
    $Queue.Enqueue($_.Name)
    $CopiedFiles[$_.Name] = $true
}

# Recursively find and copy dependencies
while ($Queue.Count -gt 0) {
    $FileToCheck = $Queue.Dequeue()
    $FilePath = Join-Path $ReleaseDir $FileToCheck

    Write-Host "      Checking dependencies for: $FileToCheck" -ForegroundColor Gray

    # Use objdump to find DLL dependencies
    $Dependencies = & objdump -p $FilePath | Select-String -Pattern "DLL Name"

    foreach ($DependencyLine in $Dependencies) {
        $DllName = ($DependencyLine.ToString() -split ":")[1].Trim()

        if ($SystemDlls -notcontains $DllName -and !$CopiedFiles.ContainsKey($DllName)) {
            $SourceDllPath = Join-Path $MinGWBinDir $DllName
            
            if (Test-Path $SourceDllPath) {
                Write-Host "      -> Found and copying: $DllName" -ForegroundColor DarkGray
                Copy-Item -Path $SourceDllPath -Destination $ReleaseDir
                $CopiedFiles[$DllName] = $true
                $Queue.Enqueue($DllName) # Add the new DLL to the queue to check its dependencies
            } else {
                Write-Host "      -> ⚠ Warning: Could not find dependency '$DllName' in '$MinGWBinDir'" -ForegroundColor Yellow
            }
        }
    }
}
Write-Host "      ✓ Dependency bundling complete" -ForegroundColor Green
Write-Host ""

# 5. Create a zip archive
Write-Host "[5/5] Creating release archive..." -ForegroundColor Yellow
$ZipFile = "ImageProcessor-Release.zip"
if (Test-Path $ZipFile) {
    Remove-Item $ZipFile
}
Compress-Archive -Path "$ReleaseDir\*" -DestinationPath $ZipFile
Write-Host "      ✓ Release package created: $ZipFile" -ForegroundColor Green
Write-Host ""

# Final message
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "  Release process complete!" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "You can now distribute the '$ZipFile' file." -ForegroundColor White
Write-Host ""
