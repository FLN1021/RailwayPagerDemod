#include <jni.h>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <sstream>
#include <unistd.h>
#include <arpa/inet.h>
#include "demod.h"

#define BUF_SIZE 8192

static std::thread workerThread;
static std::atomic<bool> running(false);
static std::mutex msgMutex;
static std::vector<std::string> messageBuffer;

// JNI 环境全局变量
static JavaVM* g_vm = nullptr;
static jobject g_obj = nullptr; // Java 的 MainActivity 实例（全局引用）

extern "C" JNIEXPORT void JNICALL
Java_com_example_railwaypagerdemod_MainActivity_nativeStopClient(JNIEnv*, jobject) {
    running = false;
}

// worker线程：负责连接 TCP 并持续接收/解码
void clientThread(std::string host, int port) {
    int sockfd;
    struct sockaddr_in servaddr;
    uint8_t buffer[BUF_SIZE];
    ssize_t n;

    lowpassBaud.create(301, SAMPLE_RATE, BAUD_RATE * 5.0f);
    phaseDiscri.setFMScaling(SAMPLE_RATE / (2.0f * DEVIATION));

    // 1. socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::lock_guard<std::mutex> lock(msgMutex);
        messageBuffer.emplace_back("socket() failed\n");
        return;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = inet_addr(host.c_str());

    if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        std::lock_guard<std::mutex> lock(msgMutex);
        messageBuffer.emplace_back("connect() failed\n");
        close(sockfd);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(msgMutex);
        messageBuffer.emplace_back("Connected to " + host + ":" + std::to_string(port) + "\n");
    }

    running = true;

    // 改用SDRAngel的decimator函数
    DecimatorsU<qint32, quint8, SDR_RX_SAMP_SZ, 8, 127, true> m_decimatorsIQ;
    SampleVector m_convertBuffer(BUF_SIZE);

    // 信道滤波器 带宽12.5k
    Lowpass<FixReal> channel_filter_i, channel_filter_q;
    channel_filter_i.create(
        65,
        SAMPLE_RATE,
        6250
    );
    channel_filter_q.create(
        65,
        SAMPLE_RATE,
        6250
    );

    while (running && (n = read(sockfd, buffer, BUF_SIZE)) > 0) {

        auto it = m_convertBuffer.begin();

        // 1536ksps/32 = 48ksps，SUP情况下fc + 48k
        // 固定采样率或许不是好主意，但现在就先这样了...
        m_decimatorsIQ.decimate32_sup(&it, buffer, BUF_SIZE);

        // 这里有个FLL可能对于没有tcxo的rtlsdr会友好一些...

        for (auto p = m_convertBuffer.begin(); p != it; ++p) {
            // 信道滤波
            auto i_ds = channel_filter_i.filter(p->real());
            auto q_ds = channel_filter_q.filter(p->imag());

            // 类型转换
            // 这我也不知道该怎么归一化了...先写了个2048，貌似这个值不会让信号强度条溢出
            float fi = ((float) i_ds) / 2048.0f;
            float fq = ((float) q_ds) / 2048.0f;
            processOneSample(fi, fq);
        }

        if (is_message_ready) {
            for (int i = 0; i < msg->size(); i++) {
                std::ostringstream ss;
                char addr_buf[32];
                snprintf(addr_buf, sizeof(addr_buf), "%010d ", msg->at(i).addr);  // 10 + 10位补零
                ss << "[MSG] " << addr_buf << msg->at(i).numeric;
                {
                    std::lock_guard<std::mutex> lock(msgMutex);
                    messageBuffer.push_back(ss.str());
                }
            }
            is_message_ready = false;
        }
    }

    if (n < 0) {
        std::lock_guard<std::mutex> lock(msgMutex);
        messageBuffer.emplace_back("read() error\n");
    }

    close(sockfd);
    running = false;

    std::lock_guard<std::mutex> lock(msgMutex);
    messageBuffer.emplace_back("Connection closed\n");
}

// === JNI: 启动客户端 ===
extern "C"
JNIEXPORT void JNICALL
Java_com_example_railwaypagerdemod_MainActivity_startClientAsync(
        JNIEnv* env, jobject thiz, jstring host_, jstring port_) {

    const char* host = env->GetStringUTFChars(host_, nullptr);
    const char* portStr = env->GetStringUTFChars(port_, nullptr);
    int port = atoi(portStr);

    // 保存 Java 对象引用
    if (g_obj == nullptr) {
        env->GetJavaVM(&g_vm);
        g_obj = env->NewGlobalRef(thiz);
    }

    if (running) {
        // 已经在运行，忽略重复启动
        env->ReleaseStringUTFChars(host_, host);
        env->ReleaseStringUTFChars(port_, portStr);
        return;
    }

    workerThread = std::thread(clientThread, std::string(host), port);
    workerThread.detach();

    env->ReleaseStringUTFChars(host_, host);
    env->ReleaseStringUTFChars(port_, portStr);
}

// === JNI: 拉取一批最新消息 ===
extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_railwaypagerdemod_MainActivity_pollMessages(JNIEnv* env, jobject /*this*/) {
    std::lock_guard<std::mutex> lock(msgMutex);
    if (messageBuffer.empty())
        return env->NewStringUTF("");

    std::ostringstream ss;
    for (auto& msg : messageBuffer) ss << msg << "\n";
    messageBuffer.clear();

    return env->NewStringUTF(ss.str().c_str());
}

extern "C"
JNIEXPORT jfloat JNICALL
Java_com_example_railwaypagerdemod_MainActivity_getSignalStrength(JNIEnv*, jobject) {
    double value = magsqRaw;
    if (value > 0.5) value = 0.5;
    if (value < 0.0) value = 0.0;
    float percent = (float) value / 0.5;
    return percent; // 映射为 0~1
}