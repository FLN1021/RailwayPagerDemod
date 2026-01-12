#include "railway_pager_demod.h"
#include "demod.h"
#include "qtypes.h"
#include "dsp/decimatorsu.h"
// #include "dsp/decimators.h"

int main(void) {
    lowpassBaud.create(
        301,               // 滤波器阶数 (taps)
        SAMPLE_RATE,           // 采样率
        BAUD_RATE * 5.0f      // 截止频率
    );
    phaseDiscri.setFMScaling(SAMPLE_RATE / (2.0f * DEVIATION));


    int sockfd;
    struct sockaddr_in servaddr;
    uint8_t buffer[BUF_SIZE];
    ssize_t n;

    // 1. Create socket
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket error");
        exit(EXIT_FAILURE);
    }

    // 2. Fill server address
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(SERVER_PORT);
    servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);

    // 3. Connect to server
    if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        perror("connect error");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Connected to %s:%d — reading bytes...\n", SERVER_IP, SERVER_PORT);

    // // 4. Read loop
    // while ((n = read(sockfd, buffer, BUF_SIZE)) > 0) {
    //     for (int j = 0; j < BUF_SIZE; j += 2) {
    //         int8_t i = buffer[j];
    //         int8_t q = buffer[j+1];
    //         processOneSample(i, q);
    //     }
    //     if (is_message_ready) printf("[MSG] %s\n", numeric_msg.c_str());
    //     is_message_ready = false;
    // }

    // Test file output
    // FILE* iqFile = nullptr;
    // iqFile = fopen("test4.c16", "wb");
    // if (!iqFile) {
    //     perror("fopen");
    //     return false;
    // }
    // struct IQ8 {
    //     FixReal i;
    //     FixReal q;
    // };
    FILE* smpFile = nullptr;
    smpFile = fopen("test5.f64", "wb");
    FILE* sptFile = nullptr;
    sptFile = fopen("test5_spt.f64", "wb");

    const int DECIM = 5;
    int decim_counter = 0;
    uint32_t acc_i = 0, acc_q = 0;
    DecimatorsU<qint32, quint8, SDR_RX_SAMP_SZ, 8, 127, true> m_decimatorsIQ;
    SampleVector m_convertBuffer(BUF_SIZE);
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

    while ((n = read(sockfd, buffer, BUF_SIZE)) > 0) {

        auto it = m_convertBuffer.begin();

        // 1536ksps/32 = 48ksps，SUP情况下fc + 48k
        // 固定采样率或许不是好主意，但现在就先这样了...
        m_decimatorsIQ.decimate32_sup(&it, buffer, BUF_SIZE);
        // printf("size %ld\n",it - m_convertBuffer.begin());

        // todo: 进行一个FLL

        for (auto p = m_convertBuffer.begin(); p != it; ++p) {
            auto i_ds = p->real();
            auto q_ds = p->imag();
            // printf("%d %d\n", i_ds, q_ds);

            // LPF Channelize
            auto i_ch = channel_filter_i.filter(i_ds);
            auto q_ch = channel_filter_q.filter(q_ds);
            // IQ8 s{i_ch, q_ch};
            // printf("%d %d\n", i_ch, q_ch);
            // fwrite(&s, sizeof(s), 1, iqFile);

            // Type Conversion
            // 这我也不知道该怎么归一化了...这样可能会偏大，有超过128的
            float fi = ((float) i_ch) / 128.0f;
            float fq = ((float) q_ch) / 128.0f;
            processOneSample(fi, fq, smpFile, sptFile);
        }

        // for (int j = 0; j < n; j += 2) {
        //     acc_i += buffer[j];
        //     acc_q += buffer[j + 1];
        //     if (++decim_counter == DECIM) {
        //         int8_t i_ds = (int8_t)(((float) acc_i / DECIM) - 128);
        //         int8_t q_ds = (int8_t)(((float) acc_q / DECIM) - 128);
        //         // printf("%d %d\n", i_ds, q_ds);
        //         processOneSample(i_ds, q_ds);
        //         acc_i = acc_q = 0;
        //         decim_counter = 0;
        //     }
        // }

        if (is_message_ready) {
            for (int i = 0; i < msg->size(); i++) {
                if ((*msg)[i].numeric.empty())
                    continue;
                printf("Addr: %d | Func: %d | Numeric: %s | Alpha: %s\n",
                       (*msg)[i].addr, (*msg)[i].func, (*msg)[i].numeric.c_str(), (*msg)[i].alpha.c_str());
                printf("[MSG] %s\n", (*msg)[i].numeric.c_str());
            }
            is_message_ready = false;
        }
    }

    if (n < 0) perror("read error");

    close(sockfd);
    return 0;
}