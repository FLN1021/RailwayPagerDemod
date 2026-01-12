#include <jni.h>
#include <string>
#include <vector>
#include <algorithm>
#include <set>
#include <sstream>
#include "demod.h"

struct ParsedMessage {
    std::string vehicleId;
    std::string route;
    std::string latitude;
    std::string longitude;
    std::string trainNo;
    std::string speed;
    std::string mileage;
};

const char numeric_chars[] = "0123456789.U -)(";

int reverse4(int x) {
    int r = 0;
    for (int i = 0; i < 4; ++i)
        r |= ((x >> i) & 1) << (3 - i);
    return r;
}

std::vector<int> str_to_nibbles(const std::string &s) {
    std::vector<int> nibbles;
    for (char ch : s) {
        const char *pos = std::find(std::begin(numeric_chars), std::end(numeric_chars), ch);
        if (pos != std::end(numeric_chars))
            nibbles.push_back(pos - numeric_chars);
    }
    return nibbles;
}

std::string decode_gb2312(const std::string &numeric_field) {
    auto nibbles = str_to_nibbles(numeric_field);
    std::vector<std::string> results;

    for (bool reverse_bits : {false, true}) {
        for (bool high_first : {false, true}) {
            std::string tmp;
            for (size_t i = 0; i + 1 < nibbles.size(); i += 2) {
                int n1 = nibbles[i];
                int n2 = nibbles[i + 1];
                if (reverse_bits) { n1 = reverse4(n1); n2 = reverse4(n2); }
                unsigned char byte = high_first ? ((n1 << 4) | n2) : ((n2 << 4) | n1);
                tmp.push_back(byte);
            }
            if (std::any_of(tmp.begin(), tmp.end(), [](unsigned char c) { return c >= 0xA1; })) {
                results.push_back(tmp);
            }
        }
    }
    return std::string(results.at(1));
}

ParsedMessage p;
ParsedMessage parseMessage(const std::string &msg) {
    // 简化：用前缀区分地址类型
    if (msg.find("1234002") != std::string::npos && msg.size() > 50) { // 越界检查
        p.vehicleId = msg.substr(15, 8);
        std::string routeRaw = msg.substr(25, 16); // 一共四个字
        std::string latRaw = msg.substr(50, 8);
        std::string lonRaw = msg.substr(41, 9);

        try { // 防止stod异常字符导致崩溃
            double latitude = std::stod(latRaw.substr(0, 2) + "." + latRaw.substr(2));
            double longitude = std::stod(lonRaw.substr(0, 3) + "." + lonRaw.substr(3));
            p.latitude = std::to_string(latitude);
            p.longitude = std::to_string(longitude);
        } catch (const std::invalid_argument& e) {}
          catch (const std::out_of_range& e) {}

        p.route = decode_gb2312(routeRaw);
    }
    else if (msg.find("1234000") != std::string::npos) {
        std::string trimmed = msg;
        trimmed.erase(remove(trimmed.begin(), trimmed.end(), '\r'), trimmed.end());
        trimmed.erase(remove(trimmed.begin(), trimmed.end(), '\n'), trimmed.end());

        std::istringstream iss(trimmed);
        std::string dummy;
        iss >> dummy >> p.trainNo >> p.speed >> p.mileage;

        p.latitude = "0";
        p.longitude = "0";
    }

    return p;
}

extern "C"
JNIEXPORT jobject JNICALL
Java_com_example_railwaypagerdemod_MainActivity_decodeMessageNative(
        JNIEnv *env, jobject thiz, jstring jmsg) {

    const char *cmsg = env->GetStringUTFChars(jmsg, nullptr);
    p = parseMessage(std::string(cmsg));
    env->ReleaseStringUTFChars(jmsg, cmsg);

    jclass cls = env->FindClass("com/example/railwaypagerdemod/ParsedMessage");
    jmethodID ctor = env->GetMethodID(cls, "<init>",
                                      "(Ljava/lang/String;[BLjava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");

    jstring vehicleId = env->NewStringUTF(p.vehicleId.c_str());
    jstring latitude = env->NewStringUTF(p.latitude.c_str());
    jstring longitude = env->NewStringUTF(p.longitude.c_str());
    jstring trainNo = env->NewStringUTF(p.trainNo.c_str());
    jstring speed = env->NewStringUTF(p.speed.c_str());
    jstring mileage = env->NewStringUTF(p.mileage.c_str());
    jbyteArray route_arr = env->NewByteArray(p.route.size());
    env->SetByteArrayRegion(route_arr, 0, p.route.size(),
                            reinterpret_cast<const jbyte*>(p.route.data()));

    jclass byteArrCls = env->FindClass("[B");

    jobject obj = env->NewObject(cls, ctor,
                                 vehicleId, route_arr, latitude, longitude, trainNo, speed, mileage);

    env->DeleteLocalRef(vehicleId);
    env->DeleteLocalRef(latitude);
    env->DeleteLocalRef(longitude);
    env->DeleteLocalRef(trainNo);
    env->DeleteLocalRef(speed);
    env->DeleteLocalRef(mileage);

    return obj;
}
