# ===============================================================================
# VLASER LICENSE KEY GENERATOR - INTERACTIVE WIZARD
# ===============================================================================

[CmdletBinding()]
param(
    [string]$MachineCode,
    [string]$Customer,
    [int]$Days = 0,
    [switch]$Forever,
    [string]$OutputFile
)

Clear-Host
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "         CHƯƠNG TRÌNH TẠO LICENSE BẢN QUYỀN (VLASER KEY GENERATOR)    " -ForegroundColor Yellow
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host ""

function Decode-MachineCode([string]$code) {
    $clean = $code.Trim()
    if ($clean.StartsWith("VLSR-")) {
        $clean = $clean.Substring(5)
    }
    
    $encBytes = [Convert]::FromBase64String($clean)
    $xorKey = [System.Text.Encoding]::UTF8.GetBytes("VLASER_HWID_OBFUSCATE_2026")
    
    $decBytes = New-Object byte[] $encBytes.Length
    for ($i = 0; $i -lt $encBytes.Length; $i++) {
        $decBytes[$i] = $encBytes[$i] -bxor $xorKey[$i % $xorKey.Length]
    }
    
    $rawString = [System.Text.Encoding]::UTF8.GetString($decBytes)
    $parts = $rawString.Split("|")
    if ($parts.Length -ge 3) {
        return @{
            Guid = $parts[0]
            Vol = $parts[1]
            Mac = $parts[2]
        }
    }
    return $null
}

$guid = ""
$vol = ""
$mac = ""

# 1. Nhap ma kich hoat do khach gui
if ([string]::IsNullOrWhiteSpace($MachineCode)) {
    Write-Host "1. DÁN MÃ KÍCH HOẠT (MACHINE CODE) DO KHÁCH GỬI VÀO ĐÂY:" -ForegroundColor Yellow
    Write-Host "   (Dạng: VLSR-XXXXX... hoặc đường dẫn file ma_may.txt / hwid.txt)" -ForegroundColor Gray
    $rawInput = Read-Host "   Nhập vào đây"
    $rawInput = $rawInput.Trim().Trim('"').Trim("'")

    if (Test-Path $rawInput) {
        $content = Get-Content $rawInput -Raw
        if ($content -match "VLSR-[A-Za-z0-9+/=]+") {
            $MachineCode = $matches[0]
        } else {
            # Hỗ trợ đọc ngược file txt kiểu cũ
            $lines = Get-Content $rawInput
            foreach ($l in $lines) {
                if ($l -match "machine_guid=(.*)") { $guid = $matches[1].Trim() }
                if ($l -match "volume_serial_c=(.*)") { $vol = $matches[1].Trim() }
                if ($l -match "mac_addr=(.*)") { $mac = $matches[1].Trim() }
            }
        }
    } else {
        $MachineCode = $rawInput
    }
}

if (-not [string]::IsNullOrWhiteSpace($MachineCode) -and [string]::IsNullOrWhiteSpace($guid)) {
    try {
        $decoded = Decode-MachineCode $MachineCode
        if ($decoded) {
            $guid = $decoded.Guid
            $vol = $decoded.Vol
            $mac = $decoded.Mac
        } else {
            Write-Host "❌ Mã kích hoạt không hợp lệ!" -ForegroundColor Red
            pause
            exit 1
        }
    } catch {
        Write-Host "❌ Lỗi giải mã Machine Code: $($_.Exception.Message)" -ForegroundColor Red
        pause
        exit 1
    }
}

if ([string]::IsNullOrWhiteSpace($guid)) {
    Write-Host "❌ Không xác định được thông tin máy khách!" -ForegroundColor Red
    pause
    exit 1
}

Write-Host ""
Write-Host "----------------------------------------------------------------------" -ForegroundColor Green
Write-Host "  ĐÃ NHẬN DIỆN MÁY KHÁCH HÀNG THÀNH CÔNG!" -ForegroundColor Yellow
Write-Host "----------------------------------------------------------------------" -ForegroundColor Green
Write-Host ""

# 2. Tên khách hàng
if ([string]::IsNullOrWhiteSpace($Customer)) {
    $Customer = Read-Host "2. Nhập tên khách hàng / đơn vị nghiệm thu (mặc định: Khách hàng Vlaser)"
    if ([string]::IsNullOrWhiteSpace($Customer)) {
        $Customer = "Khách hàng Vlaser"
    }
}

# 3. Thời hạn bản quyền
if ($Forever.IsPresent) {
    $Days = 36500
} elseif ($Days -le 0) {
    $daysInput = Read-Host "3. Nhập số ngày bản quyền (mặc định: 365 ngày). Nhập 0 => vĩnh viễn"
    if ([string]::IsNullOrWhiteSpace($daysInput)) {
        $Days = 365
    } elseif ([int]::TryParse($daysInput, [ref]$Days) -eq $false -or $Days -le 0) {
        $Days = 36500
    }
}

# 4. Nơi lưu file license.key
if ([string]::IsNullOrWhiteSpace($OutputFile)) {
    $toolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    if ([string]::IsNullOrWhiteSpace($toolsDir)) { $toolsDir = (Get-Location).Path }
    $defaultSave = Join-Path $toolsDir "license.key"

    Write-Host "4. Nơi lưu file license.key:"
    Write-Host "   Nhấn Enter để lưu tại: $defaultSave" -ForegroundColor Gray
    $saveInput = Read-Host "   Hoặc nhập đường dẫn muốn lưu"
    if ([string]::IsNullOrWhiteSpace($saveInput)) {
        $OutputFile = $defaultSave
    } else {
        $OutputFile = $saveInput.Trim().Trim('"').Trim("'")
    }
}

# Đảm bảo thư mục lưu tồn tại
$targetDir = Split-Path -Parent $OutputFile
if (-not [string]::IsNullOrWhiteSpace($targetDir) -and -not (Test-Path $targetDir)) {
    try { New-Item -ItemType Directory -Path $targetDir -Force | Out-Null } catch {}
}

# 5. Sinh gói License
$now = [DateTime]::UtcNow
$exp = $now.AddDays($Days)
$expTs = [int64]([DateTimeOffset]$exp).ToUnixTimeSeconds()

$payload = [ordered]@{
    "customer" = $Customer
    "device_fingerprint" = [ordered]@{
        "machine_guid" = $guid
        "mac_addr" = $mac
        "volume_serial_c" = $vol
    }
    "expires_at_utc" = $exp.ToString("yyyy-MM-ddTHH:mm:ssZ")
    "expires_timestamp_utc" = $expTs
    "features" = @(
        "MULTI_LIDAR_FUSION",
        "OBJECT_TRACKING",
        "RAIL_SAFETY_ALARM",
        "TCP_RECEIVER_9100",
        "BLACKBOX_RECORD"
    )
    "issued_at_utc" = $now.ToString("yyyy-MM-ddTHH:mm:ssZ")
    "license_id" = "VLASER-$([DateTimeOffset]::UtcNow.ToUnixTimeSeconds())"
    "version" = "1.0"
}

function Escape-JsonString([string]$s) {
    $sb = New-Object System.Text.StringBuilder
    foreach ($ch in $s.ToCharArray()) {
        switch ($ch) {
            '"'  { [void]$sb.Append('\"') }
            '\'  { [void]$sb.Append('\\') }
            "`b" { [void]$sb.Append('\b') }
            "`f" { [void]$sb.Append('\f') }
            "`n" { [void]$sb.Append('\n') }
            "`r" { [void]$sb.Append('\r') }
            "`t" { [void]$sb.Append('\t') }
            default {
                $code = [int][char]$ch
                if ($code -lt 32) {
                    [void]$sb.AppendFormat('\u{0:x4}', $code)
                } else {
                    [void]$sb.Append($ch)
                }
            }
        }
    }
    return '"' + $sb.ToString() + '"'
}

function ConvertTo-CanonicalJson($obj) {
    if ($null -eq $obj) { return "null" }
    if ($obj -is [bool]) { return $(if ($obj) { "true" } else { "false" }) }
    if ($obj -is [int] -or $obj -is [int64] -or $obj -is [double] -or $obj -is [decimal]) {
        return [Convert]::ToString($obj, [System.Globalization.CultureInfo]::InvariantCulture)
    }
    if ($obj -is [string]) { return (Escape-JsonString $obj) }
    if ($obj -is [System.Collections.IDictionary]) {
        $sortedKeys = $obj.Keys | Sort-Object
        $pairs = @()
        foreach ($k in $sortedKeys) {
            $pairs += (Escape-JsonString ([string]$k)) + ":" + (ConvertTo-CanonicalJson $obj[$k])
        }
        return "{" + ($pairs -join ",") + "}"
    }
    if ($obj -is [System.Collections.IEnumerable]) {
        $items = @()
        foreach ($item in $obj) {
            $items += (ConvertTo-CanonicalJson $item)
        }
        return "[" + ($items -join ",") + "]"
    }
    return (Escape-JsonString ([string]$obj))
}

$canonicalJson = ConvertTo-CanonicalJson $payload
$secretBytes = [System.Text.Encoding]::UTF8.GetBytes("VLASER_RAILWAY_SAFETY_OFFLINE_SECRET_KEY_2026")
$hmac = New-Object System.Security.Cryptography.HMACSHA256(,$secretBytes)
$sigBytes = $hmac.ComputeHash([System.Text.Encoding]::UTF8.GetBytes($canonicalJson))
$sigHex = -join ($sigBytes | ForEach-Object { "{0:x2}" -f $_ })

$full = [ordered]@{
    "payload" = $payload
    "signature" = $sigHex
}

$fullJson = $full | ConvertTo-Json -Depth 5

try {
    [System.IO.File]::WriteAllText($OutputFile, $fullJson, [System.Text.Encoding]::UTF8)
    Write-Host ""
    Write-Host "======================================================================" -ForegroundColor Green
    Write-Host " ✅ TẠO THÀNH CÔNG FILE LICENSE.KEY!" -ForegroundColor Yellow
    Write-Host "  -> Đơn vị sở hữu : $Customer" -ForegroundColor Cyan
    Write-Host "  -> Thời hạn      : $($exp.ToString('yyyy-MM-dd HH:mm:ss')) UTC ($Days ngày)" -ForegroundColor Cyan
    Write-Host "  -> File lưu tại  : $OutputFile" -ForegroundColor Green
    Write-Host "======================================================================" -ForegroundColor Green
    Write-Host ">>> BƯỚC TIẾP THEO: Gửi file license.key này cho khách hàng! <<<" -ForegroundColor Yellow
} catch {
    Write-Host "❌ Lỗi ghi file: $($_.Exception.Message)" -ForegroundColor Red
}
Write-Host ""
