# ==============================================================================
# GET_HWID.ps1 - VLASER MACHINE CODE GENERATOR
# Tạo chuỗi mã máy duy nhất (đã mã hóa) để gửi cho Nhà Cung Cấp
# ==============================================================================

Clear-Host
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "       VLASER - MA DANG KY KICH HOAT HE THONG (MACHINE CODE)         " -ForegroundColor Yellow
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Dang khoi tao thong so he thong..." -ForegroundColor Gray

# 1. Thu thap thong so he thong ngam (khong hien thi ten thiet bi ra man hinh)
$f1 = ""
try { $f1 = (Get-ItemProperty -Path "HKLM:\SOFTWARE\Microsoft\Cryptography" -ErrorAction SilentlyContinue).MachineGuid } catch {}
if (-not $f1) {
    try { $f1 = (Get-CimInstance -ClassName Win32_ComputerSystemProduct -ErrorAction SilentlyContinue).UUID } catch {}
}
if (-not $f1) { $f1 = "UNKNOWN" }

$f2 = ""
try {
    $volOut = cmd.exe /c "vol c:" 2>$null
    if ($volOut -match "([0-9A-Fa-f]{4}-[0-9A-Fa-f]{4})") {
        $f2 = $matches[1].ToUpper()
    }
} catch {}
if (-not $f2) {
    try {
        $vsn = (Get-CimInstance -ClassName Win32_LogicalDisk -Filter "DeviceID='C:'" -ErrorAction SilentlyContinue).VolumeSerialNumber
        if ($vsn -and $vsn.Length -eq 8) {
            $f2 = ($vsn.Substring(0, 4) + "-" + $vsn.Substring(4, 4)).ToUpper()
        }
    } catch {}
}
if (-not $f2) { $f2 = "UNKNOWN" }

$f3 = ""
try {
    $nics = [System.Net.NetworkInformation.NetworkInterface]::GetAllNetworkInterfaces()
    $ethTypes = @(
        [System.Net.NetworkInformation.NetworkInterfaceType]::Ethernet,
        [System.Net.NetworkInformation.NetworkInterfaceType]::GigabitEthernet,
        [System.Net.NetworkInformation.NetworkInterfaceType]::FastEthernetT,
        [System.Net.NetworkInformation.NetworkInterfaceType]::FastEthernetFx
    )
    $fallbackMac = ""
    foreach ($nic in $nics) {
        if ($nic.NetworkInterfaceType -eq [System.Net.NetworkInformation.NetworkInterfaceType]::Loopback -or
            $nic.NetworkInterfaceType -eq [System.Net.NetworkInformation.NetworkInterfaceType]::Tunnel) {
            continue
        }
        $bytes = $nic.GetPhysicalAddress().GetAddressBytes()
        if ($bytes.Length -eq 6 -and ($bytes | Where-Object { $_ -ne 0 }).Count -gt 0) {
            $hexArr = @()
            foreach ($b in $bytes) { $hexArr += ('{0:X2}' -f $b) }
            $macStr = $hexArr -join ":"
            if ($ethTypes -contains $nic.NetworkInterfaceType) {
                $f3 = $macStr
                break
            }
            if (-not $fallbackMac) {
                $fallbackMac = $macStr
            }
        }
    }
    if (-not $f3 -and $fallbackMac) { $f3 = $fallbackMac }
} catch {}
if (-not $f3) { $f3 = "UNKNOWN" }

# 2. Dong goi & Ma hoa XOR + Base64 (XOR key rieng biet de giau toan bo du lieu)
$rawString = "$f1|$f2|$f3"
$rawBytes = [System.Text.Encoding]::UTF8.GetBytes($rawString)
$xorKey = [System.Text.Encoding]::UTF8.GetBytes("VLASER_HWID_OBFUSCATE_2026")

$encBytes = New-Object byte[] $rawBytes.Length
for ($i = 0; $i -lt $rawBytes.Length; $i++) {
    $encBytes[$i] = $rawBytes[$i] -bxor $xorKey[$i % $xorKey.Length]
}

# Tao chuoi Ma Kich Hoat Machine Code: VLSR-XXXXX...
$b64 = [Convert]::ToBase64String($encBytes)
$machineCode = "VLSR-" + $b64

# 3. Hien thi ra man hinh don gian, chuyen nghiep
Write-Host ""
Write-Host "----------------------------------------------------------------------" -ForegroundColor Green
Write-Host "  MA KICH HOAT MAY TINH CUA BAN (REQUEST CODE):" -ForegroundColor Yellow
Write-Host "----------------------------------------------------------------------" -ForegroundColor Green
Write-Host ""
Write-Host "  $machineCode" -ForegroundColor White
Write-Host ""
Write-Host "----------------------------------------------------------------------" -ForegroundColor Green
Write-Host ""

# Copy thang vao Clipboard de khach chi viec Paste gui Zalo/Email
try {
    Set-Clipboard -Value $machineCode
    Write-Host "  (Da tu dong sao chep ma vao Clipboard! Ban co the Paste luon)" -ForegroundColor Gray
} catch {}

# Luu them file ma_may.txt phong khi khach can luu lai
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($scriptDir)) { $scriptDir = (Get-Location).Path }
$outFile = Join-Path $scriptDir "ma_may.txt"

$content = @"
======================================================================
VLASER SYSTEM ACTIVATION REQUEST CODE
Ngay tao: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")
======================================================================
Ma kich hoat:
$machineCode
======================================================================
Hay gui toan bo ma nay cho Nha Cung Cap de nhan file license.key
"@

$saved = $false
try {
    [System.IO.File]::WriteAllText($outFile, $content, [System.Text.Encoding]::UTF8)
    $saved = $true
} catch {
    try {
        $desktopPath = [Environment]::GetFolderPath("Desktop")
        $outFile = Join-Path $desktopPath "ma_may.txt"
        [System.IO.File]::WriteAllText($outFile, $content, [System.Text.Encoding]::UTF8)
        $saved = $true
    } catch {}
}

if ($saved) {
    Write-Host "  Da luu thong tin tai: $outFile" -ForegroundColor Green
}
Write-Host ""
Write-Host ">>> GUI MA TREN CHO NHA CUNG CAP DE NHAN FILE LICENSE.KEY <<<" -ForegroundColor Yellow
Write-Host ""
