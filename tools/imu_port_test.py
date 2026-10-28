# -*- coding: utf-8 -*-
"""imu_port_test.py —— 直接测 /dev/art_imu 能不能打开、有没有数据

⚠️ 用途：art_imu_node 报 "Unable to open port" 就崩了（exit -6）。
   需要先分清是：
     (a) 串口被别的进程占着
     (b) 波特率不对，设备不回数据
     (c) 设备本身没上电 / 不工作

   本脚本按几个常见波特率分别尝试打开并读几百毫秒，
   看哪个波特率有数据。
"""
from __future__ import print_function

import os
import select
import sys
import termios
import time

PORT = '/dev/art_imu'
BAUDS = [500000, 460800, 230400, 115200, 921600, 256000]

# termios 里可用的波特率常量
BAUD_MAP = {
    115200: termios.B115200,
    230400: termios.B230400,
    460800: getattr(termios, 'B460800', None),
    500000: getattr(termios, 'B500000', None),
    921600: getattr(termios, 'B921600', None),
    256000: getattr(termios, 'B256000', None),
}


def try_baud(baud):
    const = BAUD_MAP.get(baud)
    if const is None:
        return (u'baudrate unsupported by termios', None)

    # 先确认没有别人占着
    try:
        fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        return (u'open failed: %s' % e, None)

    try:
        try:
            attrs = termios.tcgetattr(fd)
        except termios.error as e:
            return (u'tcgetattr failed: %s (maybe busy)' % e, None)

        # raw 模式
        attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                      termios.ISTRIP | termios.INLCR | termios.IGNCR |
                      termios.ICRNL | termios.IXON)
        attrs[1] &= ~termios.OPOST
        attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                      termios.ISIG | termios.IEXTEN)
        attrs[4] = const
        attrs[5] = const
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)

        # 读 800ms
        got = b''
        t0 = time.time()
        while time.time() - t0 < 0.8:
            r, _w, _x = select.select([fd], [], [], 0.1)
            if r:
                try:
                    chunk = os.read(fd, 256)
                except OSError:
                    break
                if chunk:
                    got += chunk
            if len(got) > 512:
                break
        return (None, got)
    finally:
        try:
            os.close(fd)
        except Exception:
            pass


def main():
    print('端口: %s' % PORT)
    try:
        st = os.stat(PORT)
        print('设备: major=%d minor=%d mode=%o'
              % (os.major(st.st_mode), os.minor(st.st_mode),
                 st.st_mode & 0o777))
    except OSError as e:
        print(u'[FAIL] stat failed: %s' % e)
        return 1

    results = []
    for b in BAUDS:
        err, data = try_baud(b)
        if err:
            print(u'  %-8d  %s' % (b, err))
        elif data:
            print(u'  %-8d  [OK] got %d bytes: %s ...'
                  % (b, len(data), data[:24].encode('hex')))
        else:
            print(u'  %-8d  opened OK but NO DATA' % b)
        results.append((b, err, data))
        time.sleep(0.2)

    print('')
    good = [b for b, e, d in results if d]
    if good:
        print(u'[OK] baudrates WITH data: %s' % good)
        print(u'     -> set baudrate %d in launch'
              % good[0])
    else:
        print(u'[!] no data at any baudrate')
        print(u'    -> device unpowered, or opened exclusively by another process')
    return 0


if __name__ == '__main__':
    sys.exit(main())
