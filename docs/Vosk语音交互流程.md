# 语音交互流程示例（Vosk 离线版 · 2026-10-23）

> 引擎：Vosk 0.3.45（`/home/ub/talos_venv`）+ `vosk-model-small-cn-0.22`（66 MB）
> 麦克风：`plughw:CARD=S,DEV=0`（Astra），**不依赖 pulseaudio、不联网**
> 语法：`voicecache/vosk_grammar.json`（36 个字符 + 20 个候选词，Vosk 自动忽略词表外的词）
> 上层不变：仍向 `/voice_words` 发文本，`voice_dialog.py` / 意图匹配 / 离线播报都不用改

---

## 一、一次完整交互（实测记录）

```
车  🔈 主人，有什么我能帮你的
人  🎤 查询室内温度
    ├ Vosk 识别：查询室内温度（conf 1.00）
    └ 意图 temp（1.00）
车  🔈 当前室内温度 25.4 摄氏度
    └ 窗口续期 20 s

人  🎤 打开厨房风扇
    ├ Vosk：打开厨房风扇（1.00）→ 意图 fan_on
车  🔈 已经打开风扇了
    └ 窗口续期 20 s

人  🎤 打开卧室卧            ← 没说清
    └ 意图匹配失败 → “没听清，请再说一次”（窗口不续期）

人  🎤 打开卧室灯
    ├ Vosk：打开卧室灯（0.99）→ 意图 light_on
车  🔈 已经打开灯了

人  🎤 关闭卧室风扇          ← 不在指令库里
    └ “没听清，请再说一次”

人  🎤 关闭厨房风扇
    ├ Vosk：关闭厨房风扇（1.00）→ 意图 fan_off
车  🔈 风扇已经关掉了
```

---

## 二、时序（一条指令的完整链路）

```
人说话 ──► arecord 采集(0.125 s/块) ──► Vosk 端点检测 ──► 识别结果
                                                          │ ~0.3~0.8 s
                          asr_relay（置信度≥0.60、≥2 字、非回声窗口）
                                                          │
                                                 /voice_words
                                                          │
                                 voice_dialog：意图匹配(≥0.80) → 执行
                                                          │
                                       /voice/say → tts_play（本地 WAV，毫秒级）
```

总延迟：说完话到车开始回应约 **0.5~1.2 s**（实测 Vosk 0.35 s 加载，单句识别 <1 s）。

---

## 三、回声门控（防自激）

`asr_relay` 订阅 `/voice/say`，按文本长度估算播放时长（0.28 s/字 + 1.2 s）作为**静音窗口**，
窗口内的识别结果一律丢弃。实测有效：第一版没有门控时，车自己的“没听清，请再说一次”
被识别成用户说话，形成死循环；加门控后同一轮不再复现。

---

## 四、测试命令

```bash
# 1) 起 Vosk 识别（前台看结果）
/home/ub/talos_venv/bin/python -u ~/Hajimi/LXH/_tools/vosk_asr.py \
    --grammar ~/Hajimi/LXH/_tools/voicecache/vosk_grammar.json

# 2) 起中继（发布 /voice_words）
python2 -u ~/Hajimi/LXH/_tools/asr_relay.py \
    _grammar:=/home/ub/Hajimi/LXH/_tools/voicecache/vosk_grammar.json

# 3) 单独测对话（会话模型）
python2 -u ~/Hajimi/LXH/_tools/voice_dialog.py
```

---

## 五、与讯飞版的差异

| 维度 | 讯飞 BNF | Vosk（现在）|
|---|---|---|
| 联网 | 否（但引擎要独占麦克风、依赖 BNF 重装）| 否 |
| 文法 | 封闭 BNF，改一次要重装 + 重启 22 s | JSON 语法文件，改完直接重启（<1 s 加载）|
| 置信度 | 噪声 0~7、真指令 8~11，阈值窗口极窄 | 真指令 0.99~1.00，噪声多为 <0.6，容易区分 |
| 回声 | 依赖对话层清缓冲 | 中继层按播报时长静音，双重保护 |
| 依赖 | 讯飞节点 + 麦克风独占 | py3 子进程（200 MB 内存），麦克风独占 |

> 讯飞版本保留为回退：`ASR_BACKEND=xf` 启动任务即可切回。
