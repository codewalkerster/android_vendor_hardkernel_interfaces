#include <aidl/vendor/hardkernel/hardware/rtc/IRtc.h>
#include <android/binder_manager.h>
#include <chrono>
#include <iostream>
#include <thread>

using ::aidl::vendor::hardkernel::hardware::rtc::IRtc;

int main (int argc, char **argv) {
    std::string instance = std::string(IRtc::descriptor) + "/default";
    std::shared_ptr<IRtc> rtc = IRtc::fromBinder(
            ndk::SpAIBinder(AServiceManager_getService(instance.c_str())));

    std::string rtc_time_secs;
    rtc->getTime(&rtc_time_secs);
    std::cout << "get rtc time - " << rtc_time_secs;

    return 0;
}
