#!/usr/bin/env python3
"""EC20 NMEA 卫星锁定监听器

用法：
  sudo python3 scripts/watch_nmea.py            # 默认监听 5 分钟
  sudo python3 scripts/watch_nmea.py 600        # 监听 10 分钟
  sudo python3 scripts/watch_nmea.py 300 /dev/ttyUSB1   # 指定端口

输出：每 30 秒一行摘要，区分"真实跟踪"和"占位 SNR"
- 真实跟踪：GSV 中 prn+elev+azim+snr 全非空且 snr>30
- 占位 SNR：snr 在 28-40 范围但 elev/azim 空（引擎异常/天线收不到信号的特征）

判定标准（记忆 [[ec20-gnssconfig-bug]]）：
- 真实跟踪卫星 >= 4 + GGA fix_quality=1 → 定位成功
- 占位 SNR 卫星 >= 3 + 真实跟踪 <= 1 → 天线问题
"""
import serial, time, sys

PORT = "/dev/ttyUSB_NMEA"
BAUD = 9600
DURATION = 300
INTERVAL = 30

def parse(lines):
    real_tracked = {}
    placeholder_sats = set()
    fix_quality = 0
    sats_in_fix = 0
    gsa_fix = "1"
    rmc_status = "V"
    gsv_total = 0
    for line in lines:
        line = line.strip()
        if line.startswith("$GPGSV") or line.startswith("$GNGSV"):
            parts = line.split(",")
            if len(parts) >= 4:
                try: gsv_total = int(parts[3])
                except: pass
            for i in range(4, len(parts)-4, 4):
                try:
                    prn = parts[i].strip()
                    elev = parts[i+1].strip()
                    azim = parts[i+2].strip()
                    snr_raw = parts[i+3].strip()
                    if not prn: continue
                    snr_str = snr_raw.split("*")[0]
                    if elev and azim and snr_str:
                        try:
                            snr = int(snr_str)
                            if snr > 30:
                                real_tracked[prn] = (snr, int(elev), int(azim))
                        except: pass
                    elif snr_str:
                        try:
                            snr = int(snr_str)
                            if 28 <= snr <= 40:
                                placeholder_sats.add(prn)
                        except: pass
                except: pass
        elif line.startswith("$GPGGA") or line.startswith("$GNGGA"):
            parts = line.split(",")
            if len(parts) >= 8:
                try: fix_quality = int(parts[6]) if parts[6] else 0
                except: pass
                try: sats_in_fix = int(parts[7]) if parts[7] else 0
                except: pass
        elif line.startswith("$GPGSA") or line.startswith("$GNGSA"):
            parts = line.split(",")
            if len(parts) >= 3:
                gsa_fix = parts[2]
        elif line.startswith("$GPRMC") or line.startswith("$GNRMC"):
            parts = line.split(",")
            if len(parts) >= 3:
                rmc_status = parts[2]
    return real_tracked, placeholder_sats, fix_quality, sats_in_fix, gsa_fix, rmc_status, gsv_total

def main():
    global PORT, DURATION
    if len(sys.argv) >= 2:
        DURATION = int(sys.argv[1])
    if len(sys.argv) >= 3:
        PORT = sys.argv[2]

    print(f"监听 {PORT} 持续 {DURATION}s，每 {INTERVAL}s 输出一次摘要")
    print(f"{'时间':>5} {'定位':>5} {'GSA':>4} {'RMC':>6} {'GGA锁':>5} {'真跟踪':>6} {'占位':>4} {'可见':>4}  真实跟踪卫星(SNR/仰角/方位)")
    print("-" * 110)

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    start = time.time()
    next_report = start + INTERVAL
    buf = []
    while time.time() - start < DURATION:
        n = ser.in_waiting
        if n:
            data = ser.read(n).decode(errors="replace")
            for line in data.split("\n"):
                line = line.strip()
                if line.startswith("$GP") or line.startswith("$GN"):
                    buf.append(line)
        if time.time() >= next_report:
            real, ph, fq, sif, gsa, rmc, total = parse(buf)
            t = int(time.time() - start)
            fix_str = "✓GPS" if fq==1 else ("DGPS" if fq==2 else "—")
            rmc_str = "A有效" if rmc=="A" else "V无效"
            detail = ", ".join(f"{p}:{v[0]}/{v[1]}°/{v[2]}°" for p,v in sorted(real.items(), key=lambda x:-x[1][0])[:6]) or "—"
            print(f"{t:>4}s {fix_str:>5} {gsa:>4} {rmc_str:>6} {sif:>5} {len(real):>6} {len(ph):>4} {total:>4}  {detail}")
            buf = []
            next_report = time.time() + INTERVAL
        else:
            time.sleep(0.05)
    ser.close()

    # 循环结束后强制输出最后一次摘要（避免 DURATION=INTERVAL 时漏掉）
    if buf:
        real, ph, fq, sif, gsa, rmc, total = parse(buf)
        t = int(time.time() - start)
        fix_str = "✓GPS" if fq==1 else ("DGPS" if fq==2 else "—")
        rmc_str = "A有效" if rmc=="A" else "V无效"
        detail = ", ".join(f"{p}:{v[0]}/{v[1]}°/{v[2]}°" for p,v in sorted(real.items(), key=lambda x:-x[1][0])[:6]) or "—"
        print(f"{t:>4}s {fix_str:>5} {gsa:>4} {rmc_str:>6} {sif:>5} {len(real):>6} {len(ph):>4} {total:>4}  {detail}")

    print()
    print("=" * 110)
    real, ph, fq, sif, gsa, rmc, total = parse(buf)
    if fq >= 1 and sif >= 4:
        print(f"✅ 定位成功：{sif} 颗卫星锁定，fix_quality={fq}")
    elif len(real) >= 4:
        print(f"⚠️  引擎跟踪到 {len(real)} 颗卫星但 GGA 未报告定位（冷启动中？延长监听）")
    elif len(ph) >= 3:
        print(f"❌ 天线信号不足：{len(ph)} 颗占位 SNR，仅 {len(real)} 颗真实跟踪")
        print("   根因：天线接触不良 / 增益不足 / 室内信号弱")
        print("   修复：① 物理重拔插天线 ② 移到窗外 ③ 重启 GNSS：AT+QGPSEND → AT+QGPS=1")
    else:
        print(f"⚠️  状态不确定：真实跟踪 {len(real)} 颗，占位 {len(ph)} 颗")

if __name__ == "__main__":
    main()
