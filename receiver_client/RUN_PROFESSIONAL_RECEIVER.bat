<# :
@echo off
title PROFESSIONAL RECEIVER - LIDAR RAILWAY SAFETY SUITE
color 0B
cls

echo ===============================================================================
echo     HỆ THỐNG RECEIVER CHUYÊN SÂU - GIÁM SÁT RÀO CHẮN ĐƯỜNG TÀU (LIDAR)
echo ===============================================================================
echo.

rem Kiem tra quyen Administrator de tu dong mo Firewall (neu chua co)
net session >nul 2>&1
if %errorlevel% == 0 (
    netsh advfirewall firewall add rule name="LiDAR Professional Receiver (TCP 9100)" dir=in action=allow protocol=TCP localport=9100 >nul 2>&1
)

rem Neu co truyen tham so file .bin (Keo tha file .bin vao day de giai ma)
if not "%~1"=="" (
    if /i "%~x1"==".bin" (
        echo [GIAI MA FILE SU CO NHI PHAN] Dang phan tich: %~nx1
        echo -------------------------------------------------------------------------------
        where python >nul 2>&1
        if %errorlevel% == 0 (
            python "%~dp0decode_incident_bin.py" "%~1" --csv --json
        ) else (
            powershell -NoProfile -ExecutionPolicy Bypass -Command "& '%~dp0decode_incident_ps.ps1' -File '%~1'"
        )
        echo.
        echo Da phan tich xong. Nhan phim bat ky de thoat...
        pause >nul
        exit /b
    )
)

rem Chay truc tiep PowerShell bang cach goi chinh file nay
powershell -NoProfile -ExecutionPolicy Bypass -Command "Invoke-Expression ([System.IO.File]::ReadAllText('%~f0'))"

echo.
echo Chuong trinh da dung. Nhan phim bat ky de thoat...
pause >nul
exit /b
#>

# ==================== PHAN MA POWERSHELL DÙNG CHO CLIENT ====================
param(
    [int]$Port = 9100
)

$ErrorActionPreference = 'Stop'

Write-Host "===============================================================================" -ForegroundColor Cyan
Write-Host " [THONG TIN KET NOI RECEIVER CHUYEN SAU]" -ForegroundColor Yellow
$ipList = Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.InterfaceAlias -notmatch 'Loopback' -and $_.IPAddress -notmatch '^169\.254\.' } | Select-Object -ExpandProperty IPAddress
foreach ($ip in $ipList) {
    Write-Host "  -> IP cua may nay: $ip  (Port: $Port)" -ForegroundColor Green
}
Write-Host " * Huong dan: Nhap IP nay vao muc 'Visual Config' tren Web UI cua LiDAR Gateway." -ForegroundColor Gray
Write-Host " * Keo tha bat ky file .bin su co nao vao file nay de tu dong giai ma toa do." -ForegroundColor Gray
Write-Host "===============================================================================`n" -ForegroundColor Cyan

$listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Any, $Port)
$listener.Start()
Write-Host "Dang lang nghe luong du lieu su kien qua TCP port $Port..." -ForegroundColor Green
Write-Host "San sang nhan du lieu tu LidarWebGateway. Nhan Ctrl+C de dung.`n" -ForegroundColor Yellow

try {
    while ($true) {
        $client = $listener.AcceptTcpClient()
        $remote = $client.Client.RemoteEndPoint.ToString()
        Write-Host "[$(Get-Date -Format 'HH:mm:ss')] 🔗 Da ket noi tu Gateway: $remote" -ForegroundColor Cyan

        try {
            $stream = $client.GetStream()
            $reader = [System.IO.StreamReader]::new($stream, [System.Text.Encoding]::UTF8, $false, 4096, $true)
            $writer = [System.IO.StreamWriter]::new($stream, [System.Text.UTF8Encoding]::new($false), 4096, $true)
            $writer.NewLine = "`n"
            $writer.AutoFlush = $true

            while ($client.Connected) {
                $line = $reader.ReadLine()
                if ($null -eq $line) { break }
                if ([string]::IsNullOrWhiteSpace($line)) { continue }

                try {
                    $message = $line | ConvertFrom-Json
                } catch {
                    continue
                }

                $nowMs = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()

                # Phan hoi xac nhan ACK
                if ($message.eventId) {
                    $ack = @{
                        type = 'ack'
                        eventId = [string]$message.eventId
                        receivedAtUnixMs = $nowMs
                    } | ConvertTo-Json -Compress
                    $writer.WriteLine($ack)
                }

                if ($message.type -eq 'object_event') {
                    $obj = $message.object
                    $timeStr = Get-Date -Format 'HH:mm:ss.fff'
                    $tid = if ($message.objectId) { $message.objectId } else { $obj.id }
                    $speedKmh = [Math]::Round([Math]::Sqrt($obj.vx * $obj.vx + $obj.vy * $obj.vy) * 3.6, 1)
                    $maxSpeedKmh = if ($obj.maxSpeedKmh) { [Math]::Round([double]$obj.maxSpeedKmh, 1) } else { $speedKmh }
                    $x = [Math]::Round([double]$obj.x, 2)
                    $y = [Math]::Round([double]$obj.y, 2)
                    $w = [Math]::Round([double]$obj.width, 2)
                    $d = [Math]::Round([double]$obj.depth, 2)
                    $pts = $obj.pointCount

                    # Xac dinh cap do nguy hiem
                    $isCritical = ([Math]::Abs($x) -lt 1.5)
                    $levelTag = if ($isCritical) { "[CRITICAL - TRONG RAY]" } else { "[WARNING - TIEN GAN]" }
                    $color = if ($isCritical) { "Red" } else { "Yellow" }

                    $critStr = if ($obj.criticalX) { " (Giuaray: X=$([Math]::Round([double]$obj.criticalX,2)), Y=$([Math]::Round([double]$obj.criticalY,2)))" } else { "" }

                    Write-Host "[$timeStr] 🎯 MUC TIEU #$tid $levelTag" -ForegroundColor $color -NoNewline
                    Write-Host " | Vi tri: X=${x}m, Y=${y}m$critStr | Toc do: ${speedKmh} km/h (Max: ${maxSpeedKmh} km/h) | Size: ${w}x${d}m | $pts tia laser" -ForegroundColor White
                }
            }

            $reader.Dispose()
            $writer.Dispose()
            $stream.Dispose()
        } catch {
            Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Mat ket noi voi Gateway: $($_.Exception.Message)" -ForegroundColor Yellow
        } finally {
            $client.Close()
            Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Ngat ket noi: $remote`n" -ForegroundColor DarkGray
        }
    }
} finally {
    $listener.Stop()
}
