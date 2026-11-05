import math
import struct
import time
import json
import sys
import subprocess
from importlib import invalidate_caches




class ImuParser:
    """HWT906（JY901 兼容）：55 TYPE d0..d7 SUM，共 11 字节"""

    def __init__(self):
        self.buf = bytearray()
        self.roll = self.pitch = self.yaw = 0.0
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.temp = 0.0
        self.acc_g = 0.0
        self.frames = 0
        self.errors = 0
        self.att_frames = 0
        self.times = {}

    @staticmethod
    def _s16(lo, hi):
        v = lo | (hi << 8)
        return v - 65536 if v >= 32768 else v

    def feed(self, data):
        self.buf += data
        while True:
            i = self.buf.find(0x55)
            if i < 0:
                self.buf.clear()
                return
            if i:
                del self.buf[:i]
            if len(self.buf) < 11:
                return
            if (sum(self.buf[0:10]) & 0xFF) != self.buf[10]:
                self.errors += 1
                del self.buf[:1]
                continue
            t, d = self.buf[1], self.buf[2:10]
            if t == 0x51:
                self.ax = self._s16(d[0], d[1]) / 2048.0
                self.ay = self._s16(d[2], d[3]) / 2048.0
                self.az = self._s16(d[4], d[5]) / 2048.0
                self.temp = self._s16(d[6], d[7]) / 100.0
                self.acc_g = math.sqrt(self.ax ** 2 + self.ay ** 2 + self.az ** 2)
            elif t == 0x52:
                self.gx = self._s16(d[0], d[1]) / 16.384
                self.gy = self._s16(d[2], d[3]) / 16.384
                self.gz = self._s16(d[4], d[5]) / 16.384
            elif t == 0x53:
                self.roll = self._s16(d[0], d[1]) * 180.0 / 32768.0
                self.pitch = self._s16(d[2], d[3]) * 180.0 / 32768.0
                self.yaw = self._s16(d[4], d[5]) * 180.0 / 32768.0
                self.att_frames += 1
            self.frames += 1
            self.times[t] = time.monotonic()
            del self.buf[:11]


# ----------------------------------------------------------------- 毫米波雷达
class RadarParser:
    """MR20：AA AA | ID_L ID_H | 8 字节负载 | 55 55   （共 14 字节）
       0x60A(0A 06) = 目标数量，负载 b0 = 数量
       0x60B(0B 06) = 目标信息，负载 8 字节：
           b0 目标ID
           Long = ((b1<<5) | (b2>>3)) * 0.1 - 500        [m]
           Lat  = (((b2&7)<<8) | b3)  * 0.1 - 102.3      [m]
           Vlong= ((b4<<2) | (b5>>6)) * 0.25 - 128       [m/s]
           Vlat = ((((b5&0x3F)<<3) | (b6>>5)) * 0.25) - 64 [m/s]
           b7 不参与当前解码
    """

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bytes_total = 0
        self.bad = 0
        self.n_obj = None
        self.count_time = None
        self.target = None          # dict（最近一条 0x60B）
        self.target_time = 0.0

    def feed(self, data):
        self.bytes_total += len(data)
        self.buf += data
        while True:
            i = self.buf.find(b"\xAA\xAA")
            if i < 0:
                self.buf[:] = self.buf[-1:] if self.buf.endswith(b"\xAA") else b""
                return
            if i:
                del self.buf[:i]
            if len(self.buf) < 14:
                return
            if self.buf[12] != 0x55 or self.buf[13] != 0x55:
                self.bad += 1
                del self.buf[:1]
                continue
            fid = self.buf[2] | (self.buf[3] << 8)
            payload = bytes(self.buf[4:12])
            del self.buf[:14]
            self.frames += 1
            if fid == 0x60A:
                self.n_obj = payload[0]
                self.count_time = time.monotonic()
                self.target = None if self.n_obj == 0 else self.target
            elif fid == 0x60B:
                self.target = self._parse_60b(payload)
                self.target_time = time.monotonic()

    @staticmethod
    def _parse_60b(b):
        raw_long = (b[1] << 5) | (b[2] >> 3)
        raw_lat = ((b[2] & 0x07) << 8) | b[3]
        lo = raw_long * 0.1 - 500.0
        la = raw_lat * 0.1 - 102.3
        return {
            "id": b[0],
            "long_m": round(lo, 2),
            "lat_m": round(la, 2),
            "distance_m": round(math.hypot(lo, la), 2),
            "angle_deg": round(math.degrees(math.atan2(la, lo)), 1),
            "v_long": round((((b[4] << 2) | (b[5] >> 6)) * 0.25) - 128.0, 2),
            "v_lat": round(((((b[5] & 0x3F) << 3) | (b[6] >> 5)) * 0.25) - 64.0, 2),
            "dyn": b[6] & 0x07,
        }


NAMES = ('MR20 雷达', 'RTK / ESP32', 'HWT906 IMU')
FIX = {0: '无定位', 1: '单点定位', 2: '差分定位', 4: 'RTK 固定解', 5: 'RTK 浮点解'}

def checksum(body):
    result = 0
    for byte in body:
        result ^= byte
    return result

def degrees(value, direction, positive, negative, limit):
    raw = float(value)
    whole, minutes = divmod(raw, 100)
    coordinate = whole + minutes / 60
    if not (0 <= minutes < 60 and 0 <= coordinate <= limit and direction in (positive, negative)):
        raise ValueError('坐标格式错误')
    return coordinate if direction == positive else -coordinate

class RtkParser:
    def __init__(self):
        self.buffer = bytearray()
        self.raw = ''
        self.fix = None
        self.last_fix = 0

    def feed(self, data):
        self.buffer.extend(data)
        while b'\n' in self.buffer:
            raw, _, rest = self.buffer.partition(b'\n')
            self.buffer = rest
            self.raw = raw.decode('ascii', errors='replace').strip()[-260:]
            if not raw.lstrip().startswith(b'$'):
                continue
            body, _, check = raw.strip()[1:].partition(b'*')
            try:
                if len(check) != 2 or int(check, 16) != checksum(body):
                    continue
                parts = body.decode('ascii').split(',')
                if len(parts) < 10 or len(parts[0]) != 5 or not parts[0].endswith('GGA'):
                    continue
                quality = int(parts[6])
                position = None
                if quality > 0:
                    position = (degrees(parts[2], parts[3], 'N', 'S', 90),
                                degrees(parts[4], parts[5], 'E', 'W', 180))
                self.fix = (quality, position, parts[7], parts[1])
                self.last_fix = time.monotonic()
            except (ValueError, OverflowError):
                continue
        if len(self.buffer) > 4096:
            del self.buffer[:-260]
            self.raw = '未收到换行，字节预览：' + bytes(self.buffer[-24:]).hex(' ')


# True：运行时打印模拟数据。接设备后改成 False，并填写实际端口。
DEMO = False
LEIDA_PORT = "COM3"     # 现在没接雷达
IMU_PORT = "COM7"
RTK_PORT = "COM5"
RTK_BAUD = 115200
PRINT_INTERVAL = 1.0

class SensorOutput:
    def __init__(self):
        self.radar = RadarParser()
        self.rtk = RtkParser()
        self.imu = ImuParser()
        self.sample_time = 0

    def feed(self, index, data):
        (self.radar, self.rtk, self.imu)[index].feed(data)

    def snapshot(self):
        now = time.monotonic()
        radar = self.radar
        count = radar.n_obj if radar.count_time is not None and now - radar.count_time < 2 else None
        target = radar.target if now - radar.target_time < 2 and count != 0 else None
        leida = {"n_obj": count, "target": target}

        imu = self.imu
        attitude = now - imu.times.get(0x53, -math.inf) < 2
        acceleration = now - imu.times.get(0x51, -math.inf) < 2
        angular_rate = now - imu.times.get(0x52, -math.inf) < 2
        imu_data = {
            "roll_deg": round(imu.roll, 2) if attitude else None,
            "pitch_deg": round(imu.pitch, 2) if attitude else None,
            "yaw_deg": round(imu.yaw, 2) if attitude else None,
            "acceleration_g": [imu.ax, imu.ay, imu.az] if acceleration else None,
            "angular_rate_deg_s": [imu.gx, imu.gy, imu.gz] if angular_rate else None,
        }

        quality, position, satellites, utc = (None, None, None, None)
        if self.rtk.fix is not None and now - self.rtk.last_fix < 3:
            quality, position, satellites, utc = self.rtk.fix
        rtk = {
            "fix_quality": quality,
            "fix_status": FIX.get(quality, "其他状态") if quality is not None else None,
            "latitude_deg": position[0] if position else None,
            "longitude_deg": position[1] if position else None,
            "satellites": int(satellites) if satellites and satellites.isdecimal() else None,
            "utc": utc,
            "raw": self.rtk.raw or None,
        }
        return {"Leida": leida, "IMU": imu_data, "RTK": rtk}

    def samples(self, now):
        if now - self.sample_time < 0.25:
            return
        self.sample_time = now
        self.feed(0, bytes.fromhex('AA AA 0A 06 01 00 00 00 00 00 00 00 55 55 AA AA 0B 06 01 9F AB F2 80 20 00 9E 55 55'))
        body = b'GNGGA,120000,3000.0000,N,12000.0000,E,4,18,0.8,12.0,M,0.0,M,,'
        self.feed(1, b'$' + body + ('*%02X\r\n' % checksum(body)).encode('ascii'))
        for kind, values in [(0x51, (32, -16, 2048, 2300)), (0x52, (0, 16, 32, 0)),
                             (0x53, (800, -320, int(7000 + 1500 * math.sin(now)), 0))]:
            frame = bytes([0x55, kind]) + struct.pack('<hhhh', *values)
            self.feed(2, frame + bytes([sum(frame) & 255]))


def main():
    output = SensorOutput()
    links = {}
    if DEMO:
        print("示例模式：以下为模拟数据。接实机请将 DEMO 改为 False。", file=sys.stderr, flush=True)
    else:
        try:
            import serial
        except ModuleNotFoundError as error:
            if error.name != "serial":
                raise
            print("缺少 pyserial，正在安装到当前 Python 环境…", file=sys.stderr, flush=True)
            try:
                subprocess.run(
                    [sys.executable, "-m", "pip", "install", "pyserial==3.5"],
                    stdout=sys.stderr,
                    check=True,
                )
                invalidate_caches()
                import serial
            except (OSError, subprocess.CalledProcessError, ImportError) as install_error:
                print(f"安装或加载 pyserial 失败：{install_error}", file=sys.stderr)
                return 1
        if not hasattr(serial, "Serial"):
            print("当前加载的 serial 模块没有 Serial 接口，请检查是否安装了 pyserial，或目录里是否有同名 serial.py。", file=sys.stderr)
            return 1
        settings = [(LEIDA_PORT, 115200), (RTK_PORT, RTK_BAUD), (IMU_PORT, 921600)]
        ports = [port.casefold() for port, _ in settings if port]
        if len(ports) != len(set(ports)):
            print("每个串口只能配置给一台设备。", file=sys.stderr)
            return 1
        for index, (port, baud) in enumerate(settings):
            if not port:
                continue
            if not isinstance(baud, int) or baud <= 0:
                print(f"{port} 请填写实际波特率。", file=sys.stderr)
                continue
            try:
                links[index] = serial.Serial(port, baud, timeout=0)
            except (OSError, ValueError) as error:
                print(f"{port} 打开失败：{error}", file=sys.stderr)
        if not links:
            print("没有可用串口，请检查文件顶部的配置。", file=sys.stderr)
            return 1

    next_print = 0
    try:
        while DEMO or links:
            now = time.monotonic()
            if DEMO:
                output.samples(now)
            for index, link in list(links.items()):
                try:
                    data = link.read(min(link.in_waiting, 32768))
                except OSError as error:
                    print(f"{link.port} 读取失败：{error}", file=sys.stderr)
                    link.close()
                    del links[index]
                    continue
                if data:
                    output.feed(index, data)
            if now >= next_print:
                print(json.dumps(output.snapshot(), ensure_ascii=False), flush=True)
                next_print = now + PRINT_INTERVAL
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        for link in links.values():
            link.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
