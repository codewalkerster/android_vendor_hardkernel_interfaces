package vendor.hardkernel.hardware.rtc;

@VintfStability
interface IRtc {
    String getTime();
    void setWakeupAlarm(in long triggerAtMillis);
}
