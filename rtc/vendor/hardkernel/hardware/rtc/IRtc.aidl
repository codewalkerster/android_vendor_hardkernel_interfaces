package vendor.hardkernel.hardware.rtc;

@VintfStability
interface IRtc {
    long getTime();
    void setWakeupAlarm(in long triggerAtMillis);
}
