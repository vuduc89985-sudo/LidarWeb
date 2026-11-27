param(
    [Parameter(Mandatory=$true)]
    [string]$File
)

if (-not (Test-Path $File)) {
    Write-Host "[ERROR] Khong tim thay file: $File" -ForegroundColor Red
    return


$fs = [System.IO.File]::OpenRead($File)
$br = New-Object System.IO.BinaryReader($fs)

try {
    # Begin parsing logic
    $magicBytes = $br.ReadBytes(4)
    $magic = [System.Text.Encoding]::ASCII.GetString($magicBytes)
    if ($magic -ne 'LDRB') {
        Write-Host "[ERROR] Magic header khong hop le: $magic. Khong phai file LDRB!" -ForegroundColor Red
        return
    

    $totalFrames = $br.ReadUInt32()
    Write-Host "===============================================================================" -ForegroundColor Cyan
    Write-Host " 📁 TEP SU CO NHI PHAN: $(Split-Path $File -Leaf)" -ForegroundColor Green
    Write-Host " 🎞️ Tong so frame su co: $totalFrames frames (bao gom ca 10s pre-buffer)" -ForegroundColor Yellow
    Write-Host "===============================================================================" -ForegroundColor Cyan

    $csvOut = "$File.extracted_points.csv"
    $csvLines = New-Object System.Collections.Generic.List[string]
    $csvLines.Add("frame,sensor,x,y,z,intensity")

    $maxAlarm = 0
    $targetsList = @()

    for ($i = 0; $i -lt $totalFrames; $i++) {
        $metaLen = $br.ReadUInt32()
        $metaBytes = $br.ReadBytes($metaLen)
        $metaJson = [System.Text.Encoding]::UTF8.GetString($metaBytes)
        $meta = $metaJson | ConvertFrom-Json

        $n1 = $br.ReadUInt32()
        $pts1Bytes = $br.ReadBytes($n1 * 16)

        $n2 = $br.ReadUInt32()
        $pts2Bytes = $br.ReadBytes($n2 * 16)

        $al = 0
        if ($meta.alarmLevel) { $al = [int]$meta.alarmLevel 
        if ($al -gt $maxAlarm) { $maxAlarm = $al 

        if ($meta.targets) {
            foreach ($tg in $meta.targets) {
                if ($tg.id -and -not ($targetsList -contains $tg.id)) { $targetsList += $tg.id 
            
        

        if ($i -lt 5 -or $al -ge 3) {
            $dangerText = "SAFE"
            $color = "Green"
            if ($al -ge 3) { $dangerText = "CRITICAL (NGUY HIEM TRONG RAY)"; $color = "Red" 
            elseif ($al -eq 2) { $dangerText = "WARNING (CANH BAO)"; $color = "Yellow" 

            Write-Host "▶ Frame #$($i+1) | Muc an toan: [$al] $dangerText | L1: $n1 pts | L2: $n2 pts" -ForegroundColor $color
            if ($meta.targets) {
                $tNames = @()
                foreach ($tg in $meta.targets) { $tNames += "Muc tieu #" + $tg.id 
                Write-Host "    └─ $($tNames -join ', ')" -ForegroundColor Gray
            

            if ($i -eq 0 -or $al -ge 3) {
                $lim = [Math]::Min($n1, 25)
                for ($k = 0; $k -lt $lim; $k++) {
                    $off = $k * 16
                    $px = [System.BitConverter]::ToSingle($pts1Bytes, $off)
                    $py = [System.BitConverter]::ToSingle($pts1Bytes, $off + 4)
                    $pz = [System.BitConverter]::ToSingle($pts1Bytes, $off + 8)
                    $pi = [System.BitConverter]::ToSingle($pts1Bytes, $off + 12)
                    $csvLines.Add("$($i+1),1,$px,$py,$pz,$pi")
                
            
        
    

    [System.IO.File]::WriteAllLines($csvOut, $csvLines)
    Write-Host "`n-------------------------------------------------------------------------------" -ForegroundColor Cyan
    Write-Host "📋 TONG KET: Muc nguy hiem cao nhat: CAP $maxAlarm | Muc tieu: $($targetsList -join ', ')" -ForegroundColor Yellow
    Write-Host "💾 Da trich xuat mau toa do diem laser tho ra file CSV: $csvOut" -ForegroundColor Green
    Write-Host "-------------------------------------------------------------------------------" -ForegroundColor Cyan
 catch {
    Write-Host "[ERROR] Exception encountered: $_" -ForegroundColor Red
 finally {
    $br.Close()
    $fs.Close()


