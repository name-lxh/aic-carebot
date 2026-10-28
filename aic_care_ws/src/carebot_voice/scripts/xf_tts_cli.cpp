/*
 * xf_tts_cli.cpp — 一次性讯飞离线语音合成命令行工具
 * ============================================================================
 * 智慧养老护理赛题 · 语音播报链路的关键补件
 *
 * 为什么需要它
 * ------------
 * 组委会提供的 `robot_voice/tts_subscribe.cpp` 是**常驻 ROS 节点**，存在三个
 * 导致它无法用于比赛播报的设计问题：
 *     1. 合成结果写到**固定文件名** `tts_sample.wav` —— 连续播报互相覆盖；
 *     2. `popen("play ...")` 之后只 `sleep(1)`，**不等播放结束** ——
 *        任务状态机会在语音还没播完时就推进到下一步；
 *     3. 没有任何完成回执 —— 上层无法知道播报何时真正结束。
 *
 * 本工具把「合成」抽成一个**一次性进程**：调用即同步完成合成并退出，
 * 由 voice_bridge.py 用唯一的文件名调用、并 `wait()` 播放进程，
 * 从而获得精确的播报时序，且支持播报队列。
 *
 * 用法
 * ----
 *     xf_tts_cli "<要朗读的文本>" /tmp/carebot_tts/say_1.wav
 *     # 退出码 0 = 成功，非 0 = 失败（错误码打印到 stderr）
 *
 * 构建
 * ----
 * 本文件放在 carebot_voice/scripts/ 下，**不参与 catkin 编译**，
 * 因为它需要链接讯飞 SDK（libmsc.so）与随包的头文件，而这些在 robot_voice 包里。
 * 请用配套脚本 scripts/build_xf_tts_cli.sh 编译：
 *
 *     ./build_xf_tts_cli.sh <robot_voice包路径> [输出目录]
 *
 * 然后把生成的可执行文件路径配到 voice_bridge 的 ~tts_bin 参数：
 *
 *     <param name="tts_bin" value="/home/ub/omniWheelCareRobot/rosCode/src/robot_voice/bin/xf_tts_cli"/>
 *
 * 参数说明
 * --------
 *     appid       必须与 libmsc.so 绑定一致（robot_voice 包用的是 f03b47f2）
 *     voice_name  发音人：xiaoyan（女声）/ xiaofeng（男声）
 *     speed       语速 0~100
 *     volume      音量 0~100
 *     pitch       音高 0~100
 *     rdn         数字发音方式 0~3
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

/* wav 音频头结构（与 robot_voice/tts_subscribe.cpp 保持一致） */
typedef struct _wave_pcm_hdr {
    char  riff[4];              /* = "RIFF" */
    int   size_8;               /* = FileSize - 8 */
    char  wave[4];              /* = "WAVE" */
    char  fmt[4];               /* = "fmt " */
    int   fmt_size;             /* = 16 */
    short format_tag;           /* = PCM : 1 */
    short channels;             /* = 通道数 : 1 */
    int   samples_per_sec;      /* = 16000 */
    int   avg_bytes_per_sec;    /* = samples_per_sec * bits / 8 */
    short block_align;          /* = bits / 8 */
    short bits_per_sample;      /* = 16 */
    char  data[4];              /* = "data" */
    int   data_size;            /* = 纯数据长度 */
} wave_pcm_hdr;

static wave_pcm_hdr default_wav_hdr = {
    { 'R', 'I', 'F', 'F' }, 0,
    { 'W', 'A', 'V', 'E' },
    { 'f', 'm', 't', ' ' }, 16,
    1, 1, 16000, 32000, 2, 16,
    { 'd', 'a', 't', 'a' }, 0
};

/* 文本合成：把 src_text 合成为 des_path 指向的 wav 文件 */
static int text_to_speech(const char *src_text, const char *des_path, const char *params)
{
    int          ret          = -1;
    FILE        *fp           = NULL;
    const char  *sessionID    = NULL;
    unsigned int audio_len    = 0;
    wave_pcm_hdr wav_hdr      = default_wav_hdr;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

    if (NULL == src_text || NULL == des_path) {
        fprintf(stderr, "params is error!\n");
        return ret;
    }

    fp = fopen(des_path, "wb");
    if (NULL == fp) {
        fprintf(stderr, "open %s error.\n", des_path);
        return ret;
    }

    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret) {
        fprintf(stderr, "QTTSSessionBegin failed, error code: %d.\n", ret);
        fclose(fp);
        return ret;
    }

    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret) {
        fprintf(stderr, "QTTSTextPut failed, error code: %d.\n", ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }

    fwrite(&wav_hdr, sizeof(wav_hdr), 1, fp);   /* 先占位写 wav 头，最后回填 */

    while (1) {
        const void *data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret)
            break;
        if (NULL != data) {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len;
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
        usleep(150 * 1000);                      /* 防止频繁占用 CPU */
    }

    if (MSP_SUCCESS != ret) {
        fprintf(stderr, "QTTSAudioGet failed, error code: %d.\n", ret);
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }

    /* 回填 wav 头的长度字段 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8, sizeof(wav_hdr.size_8), 1, fp);
    fseek(fp, 40, 0);
    fwrite(&wav_hdr.data_size, sizeof(wav_hdr.data_size), 1, fp);
    fclose(fp);

    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret)
        fprintf(stderr, "QTTSSessionEnd failed, error code: %d.\n", ret);

    return ret;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "用法: %s <文本> <输出wav路径> [appid] [发音人]\n"
            "示例: %s \"当前温度23.5度\" /tmp/tts.wav\n",
            prog, prog);
}

int main(int argc, char *argv[])
{
    if (argc < 3) {
        usage(argv[0]);
        return 2;
    }

    const char *text    = argv[1];
    const char *outfile = argv[2];
    const char *appid   = (argc > 3) ? argv[3] : "f03b47f2";
    const char *voice   = (argc > 4) ? argv[4] : "xiaoyan";

    char login_params[256];
    snprintf(login_params, sizeof(login_params), "appid = %s, work_dir = .", appid);

    char session_params[512];
    snprintf(session_params, sizeof(session_params),
             "voice_name = %s, text_encoding = utf8, sample_rate = 16000, "
             "speed = 50, volume = 80, pitch = 50, rdn = 2", voice);

    int ret = MSPLogin(NULL, NULL, login_params);
    if (MSP_SUCCESS != ret) {
        /* 登录失败通常是 appid 与 libmsc.so 不匹配；不直接退出，
           仍尝试合成（某些部署下离线合成无需登录）。 */
        fprintf(stderr, "MSPLogin failed, error code: %d (继续尝试合成).\n", ret);
    }

    ret = text_to_speech(text, outfile, session_params);
    if (MSP_SUCCESS != ret) {
        fprintf(stderr, "text_to_speech failed, error code: %d.\n", ret);
        MSPLogout();
        return 1;
    }

    MSPLogout();
    return 0;
}
