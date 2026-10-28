#!/bin/bash
# mic_diag.sh —— 停掉 ASR，检查 Astra 麦克风支持哪些采样率，并对比 16k/48k 录音电平
source /home/ub/Hajimi/LXH/env.sh 2>/dev/null
export ROS_MASTER_URI=http://192.168.8.213:11399

for pat in xf_asr_usb_node command_recognition motion_control; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
done
sleep 2

echo "=== Astra 麦克风支持的格式 ==="
arecord -D hw:CARD=S,DEV=0 --dump-hw-params 2>&1 | grep -E "FORMAT|RATE|CHANNELS|SAMPLE" | head -6

echo ""
echo "=== 录音对比（4s @16k / 4s @48k）==="
timeout 8 arecord -D plughw:CARD=S,DEV=0 -f S16_LE -r 16000 -c 1 -d 4 /tmp/r16.wav >/dev/null 2>&1
timeout 8 arecord -D plughw:CARD=S,DEV=0 -f S16_LE -r 48000 -c 1 -d 4 /tmp/r48.wav >/dev/null 2>&1
timeout 8 arecord -D plughw:CARD=S,DEV=0 -f S16_LE -r 16000 -c 2 -d 4 /tmp/r16s.wav >/dev/null 2>&1

cat > /tmp/lvl3.py <<'PY'
import wave, struct, math, sys
for name in sys.argv[1:]:
    try:
        w = wave.open(name, 'rb')
        n = w.getnframes(); rate = w.getframerate(); ch = w.getnchannels()
        d = w.readframes(n); w.close()
        v = struct.unpack('<%dh' % (len(d)//2), d)
        rms = math.sqrt(sum(x*x for x in v)/max(1, len(v)))
        pk = max(abs(x) for x in v) if v else 0
        print('  %-12s rate=%d ch=%d RMS=%6.0f peak=%6d' % (name.split('/')[-1], rate, ch, rms, pk))
    except Exception as e:
        print('  %-12s FAIL %r' % (name.split('/')[-1], e))
PY
python2 /tmp/lvl3.py /tmp/r16.wav /tmp/r48.wav /tmp/r16s.wav
