# -*- coding: utf-8 -*-
"""imu_frame_probe.py —— 抓一段原始串口数据，找帧头帧尾

⚠️ 背景：厂商 `art_imu_01` 的解析代码要求帧格式
        0xA5 0x0A 0x01 ... 0x0D 0x0A（31 字节）
   但用它的节点读出来的数据是垃圾（orientation 恒为 identity，
   angular_velocity.x = -22.67）。所以要先确认：
        这个 IMU 到底发的是什么帧格式？

   本脚本抓 2 秒原始字节，统计：
     · 有没有 A5 0A 01 帧头
     · 帧间隔
     · 常见字节模式
"""
from __future__ import print_function

import os
import select
import sys
import termios
import time

PORT = '/dev/art_imu'
BAUD = 460800
BAUD_CONST = getattr(termios, 'B460800', None)


def main():
    if BAUD_CONST is None:
        print('termios has no B460800')
        return 1
    try:
        fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        print('open failed: %s' % e)
        return 1

    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                      termios.ISTRIP | termios.INLCR | termios.IGNCR |
                      termios.ICRNL | termios.IXON)
        attrs[1] &= ~termios.OPOST
        attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                      termios.ISIG | termios.IEXTEN)
        attrs[4] = attrs[5] = BAUD_CONST
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)

        data = b''
        t0 = time.time()
        while time.time() - t0 < 2.0:
            r, _w, _x = select.select([fd], [], [], 0.1)
            if r:
                try:
                    data += os.read(fd, 4096)
                except OSError:
                    break
        print('baud %d, captured %d bytes in 2s (%.0f B/s)'
              % (BAUD, len(data), len(data) / 2.0))
    finally:
        try:
            os.close(fd)
        except Exception:
            pass

    if not data:
        print('[!] no data')
        return 1

    print('')
    print('first 64 bytes:')
    print('  ' + data[:64].encode('hex'))
    print('')
    print('last 64 bytes:')
    print('  ' + data[-64:].encode('hex'))

    # ---- 找各种候选帧头 ----
    cands = [
        ('A5 0A 01', b'\xa5\x0a\x01'),           # art_imu 期望
        ('A5 5A',    b'\xa5\x5a'),               # WHEELTEC 常见
        ('55 AA',    b'\x55\xaa'),               # WHEELTEC/GNSS 常见
        ('AA 55',    b'\xaa\x55'),
        ('5A A5',    b'\x5a\xa5'),
        ('FD FC',    b'\xfd\xfc'),
        ('A5 0A',    b'\xa5\x0a'),
        ('0D 0A',    b'\x0d\x0a'),               # CRLF 结尾
    ]
    print('')
    print('pattern counts:')
    for name, pat in cands:
        cnt = data.count(pat)
        print('  %-10s  %4d  times' % (name, cnt))

    # ---- 如果找到 A5 0A 01，量帧间隔 ----
    pat = b'\xa5\x0a\x01'
    idx = []
    pos = data.find(pat)
    while pos >= 0:
        idx.append(pos)
        pos = data.find(pat, pos + 1)
    if len(idx) >= 2:
        gaps = [idx[i + 1] - idx[i] for i in range(len(idx) - 1)]
        print('')
        print('A5 0A 01 header found %d times' % len(idx))
        print('  frame gaps: min=%d max=%d (most common %d)'
              % (min(gaps), max(gaps),
                 max(set(gaps), key=gaps.count)))
        print('  => frame length appears to be %d bytes'
              % max(set(gaps), key=gaps.count))
        # 打印第一帧
        k = idx[0]
        print('  first frame: ' + data[k:k + 32].encode('hex'))
    elif len(idx) == 1:
        print('')
        print('[!] only 1 A5 0A 01 header in 2s -- frame format mismatch')
    else:
        print('')
        print('[!] NO A5 0A 01 header -- this is NOT the art_imu protocol')

    # ---- 尝试按 31 字节周期看规律 ----
    print('')
    print('byte-value histogram (top 8):')
    hist = {}
    for ch in data:
        hist[ch] = hist.get(ch, 0) + 1
    top = sorted(hist.items(), key=lambda kv: -kv[1])[:8]
    for v, c in top:
        print('  0x%02X  %d' % (v, c))
    return 0


if __name__ == '__main__':
    sys.exit(main())
