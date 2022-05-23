/*
 *    Copyright (c) 2019 Sangchul Go <luke.go@hardkernel.com>
 *
 *    OdroidThings is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    OdroidThings is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with OdroidThings.
 *    If not, see <http://www.gnu.org/licenses/>.
 */

#define LOG_TAG "vendor.hardkernel.hardware.odroidthings@1.0-hal"

#include <log/log.h>
#include <OdroidThings.h>
#include <hardware/odroidThings.h>
#include <dlfcn.h>

namespace vendor::hardkernel::hardware::odroidthings::V1_0::implementation {

OdroidThings *OdroidThings::sInstance = nullptr;

OdroidThings::OdroidThings() : mDevice(nullptr) {
    sInstance = this;
    mDevice = openHal();
    if (!mDevice) {
        ALOGE("Can't open HAL module");
    }
}

OdroidThings::~OdroidThings() {
    ALOGV("~OdroidThings()");
    if (mDevice == nullptr) {
        ALOGE("No valid device");
        return;
    }
    int err;
    if (0 != (err = mDevice->common.close(
                    reinterpret_cast<hw_device_t*>(mDevice)))) {
        ALOGE("Can't close odroidthing module, error: %d", err);
        return;
    }
    mDevice = nullptr;
}

IOdroidThings* OdroidThings::getInstance() {
    if (!sInstance) {
        sInstance = new OdroidThings();
    }
    return sInstance;
}

Return<void> OdroidThings::getPinNameList(getPinNameList_cb _hidl_cb) {
    hidl_vec<hidl_string> nameList;
    std::vector<std::string> pinNameList = mDevice->common_ops.getPinNameList();
    nameList.resize(pinNameList.size());
    for (size_t i=0; i < pinNameList.size(); i++) {
        nameList[i] = pinNameList[i];
    }
    _hidl_cb(nameList);

    return Void();
}

Return<void> OdroidThings::getListOf(uint8_t mode, getListOf_cb _hidl_cb) {
    hidl_vec<hidl_string> list;
    std::vector<std::string> pinList = mDevice->common_ops.getListOf(mode);
    list.resize(pinList.size());
    for (size_t i=0; i < pinList.size(); i++)
        list[i] = pinList[i];
    _hidl_cb(list);

    return Void();
}

Return<void> OdroidThings::setDirection(int32_t pin, Direction direction) {
    mDevice->gpio_ops.setDirection(pin, (direction_t) direction);

    return Void();
}

Return<void> OdroidThings::gpio_setValue(int32_t pin, bool value) {
    mDevice->gpio_ops.setValue(pin, value);

    return Void();
}

Return<bool> OdroidThings::gpio_getValue(int32_t pin) {
    return mDevice->gpio_ops.getValue(pin);
}

Return<void> OdroidThings::gpio_setActiveType(int32_t pin, int32_t activeType) {
    mDevice->gpio_ops.setActiveType(pin, activeType);

    return Void();
}

Return<void> OdroidThings::gpio_setEdgeTriggerType(int32_t pin, int32_t edgeTriggerType) {
    mDevice->gpio_ops.setEdgeTriggerType(pin, edgeTriggerType);

    return Void();
}

std::map<int, sp<Callback>> OdroidThings::callbackList = std::map<int, sp<Callback>>();

Return<void> OdroidThings::gpio_registerCallback(int32_t pin,
            const sp<Callback>& gpioCallback) {
    callbackList.insert(std::pair<int, sp<Callback>>(pin, gpioCallback));
    function_t cb;
    switch (pin) {
       CASE_CALLBACK(0);
       CASE_CALLBACK(1);
       CASE_CALLBACK(2);
       CASE_CALLBACK(3);
       CASE_CALLBACK(4);
       CASE_CALLBACK(5);
       CASE_CALLBACK(6);
       CASE_CALLBACK(7);
       CASE_CALLBACK(8);
       CASE_CALLBACK(9);
       CASE_CALLBACK(10);
       CASE_CALLBACK(11);
       CASE_CALLBACK(12);
       CASE_CALLBACK(13);
       CASE_CALLBACK(14);
       CASE_CALLBACK(15);
       CASE_CALLBACK(16);
       CASE_CALLBACK(17);
       CASE_CALLBACK(18);
       CASE_CALLBACK(19);
       CASE_CALLBACK(20);
       CASE_CALLBACK(21);
       CASE_CALLBACK(22);
       CASE_CALLBACK(23);
       CASE_CALLBACK(24);
       CASE_CALLBACK(25);
       CASE_CALLBACK(26);
       CASE_CALLBACK(27);
       CASE_CALLBACK(28);
       CASE_CALLBACK(29);
       CASE_CALLBACK(30);
       CASE_CALLBACK(31);
       CASE_CALLBACK(32);
       CASE_CALLBACK(33);
       CASE_CALLBACK(34);
       CASE_CALLBACK(35);
       CASE_CALLBACK(36);
       CASE_CALLBACK(37);
       CASE_CALLBACK(38);
       CASE_CALLBACK(39);
    }
    mDevice->gpio_ops.registerCallback(pin, cb);

    return Void();
}

Return<void> OdroidThings::gpio_unregisterCallback(int32_t pin) {
    std::map<int, sp<Callback>>::iterator it;
    it = callbackList.find(pin);
    if (it != callbackList.end()) {
        mDevice->gpio_ops.unregisterCallback(pin);
        callbackList.erase(it);
    }

    return Void();
}

Return<void> OdroidThings::pwm_open(int32_t pin) {
    mDevice->pwm_ops.open(pin);

    return Void();
}

Return<void> OdroidThings::pwm_close(int32_t pin) {
    mDevice->pwm_ops.close(pin);

    return Void();
}

Return<bool> OdroidThings::pwm_setEnable(int32_t pin, bool enabled) {
    ALOGD("pin - %d, enable - %s", pin, enabled?"true":"false");
    return mDevice->pwm_ops.setEnable(pin, enabled);
}

Return<bool> OdroidThings::pwm_setDutyCycle(int32_t pin, double cycle_rate) {
    ALOGD("pin - %d, cycle rate - %f", pin, cycle_rate);
    return mDevice->pwm_ops.setDutyCycle(pin, cycle_rate);
}

Return<bool> OdroidThings::pwm_setFrequency(int32_t pin, double frequency_hz) {
    ALOGD("pin - %d, cycle rate - %f", pin, frequency_hz);
    return mDevice->pwm_ops.setFrequency(pin, frequency_hz);
}

Return<void> OdroidThings::i2c_open(int32_t nameIdx, uint32_t address, int32_t idx) {
    mDevice->i2c_ops.open(nameIdx, address, idx);

    return Void();
}

Return<void> OdroidThings::i2c_close(int32_t idx) {
    mDevice->i2c_ops.close(idx);

    return Void();
}

Return<void> OdroidThings::i2c_readRegBuffer(int32_t idx, uint32_t reg, int32_t length, i2c_readRegBuffer_cb _hidl_cb) {
    Result result = Result::OK;

    _hidl_cb(result, mDevice->i2c_ops.readRegBuffer(idx, reg, length));
    return Void();
}

Return<Result> OdroidThings::i2c_writeRegBuffer(int32_t idx, uint32_t reg, const hidl_vec<uint8_t>& buffer, int32_t length) {
    return (Result)mDevice->i2c_ops.writeRegBuffer(idx, reg, buffer, length);
}

Return<void> OdroidThings::uart_open(int32_t idx) {
    mDevice->uart_ops.open(idx);
    return Void();
}

Return<void> OdroidThings::uart_close(int32_t idx) {
    mDevice->uart_ops.close(idx);
    return Void();
}

Return<bool> OdroidThings::uart_flush(int32_t idx, int32_t direction) {
    return mDevice->uart_ops.flush(idx, direction);
}

Return<bool> OdroidThings::uart_sendBreak(int32_t idx, int32_t duration) {
    return mDevice->uart_ops.sendBreak(idx, duration);
}

Return<bool> OdroidThings::uart_setBaudrate(int32_t idx, int32_t rate) {
    return mDevice->uart_ops.setBaudrate(idx, rate);
}

Return<bool> OdroidThings::uart_setDataSize(int32_t idx, int32_t size) {
    return mDevice->uart_ops.setDataSize(idx, size);
}

Return<bool> OdroidThings::uart_setHardwareFlowControl(int32_t idx, int32_t mode) {
    return mDevice->uart_ops.setHardwareFlowControl(idx, mode);
}

Return<bool> OdroidThings::uart_setParity(int32_t idx, int32_t mode) {
    return mDevice->uart_ops.setParity(idx, mode);
}

Return<bool> OdroidThings::uart_setStopBits(int32_t idx, int32_t bits) {
    return mDevice->uart_ops.setStopBits(idx, bits);
}

Return<void> OdroidThings::uart_read(int32_t idx, int32_t length, uart_read_cb _hidl_cb) {
    hidl_vec<uint8_t> list;
    std::vector<uint8_t> buffer = mDevice->uart_ops.read(idx, length);
    int32_t len = buffer.size();

    if (len > 0) {
        list.resize(len);
        for(int32_t i = 0; i < len; i++)
            list[i] = buffer[i];
        _hidl_cb(len, list);
    } else {
        _hidl_cb(len, list);
    }

    return Void();
}

Return<int32_t> OdroidThings::uart_write(int32_t idx, const hidl_vec<uint8_t>& buffer, int32_t length) {
    return mDevice->uart_ops.write(idx, buffer, length);
}

Return<void> OdroidThings::uart_registerCallback(int32_t idx,
        const sp<Callback>& uartCallback) {
    callbackList.insert(std::pair<int, sp<Callback>>(CALLBACK_OFFSET_UART + idx, uartCallback));
    function_t cb;
    switch (CALLBACK_OFFSET_UART + idx) {
        CASE_CALLBACK(40);
        CASE_CALLBACK(41);
        CASE_CALLBACK(42);
        CASE_CALLBACK(43);
    }
    mDevice->uart_ops.registerCallback(idx, cb);

    return Void();
}

Return<void> OdroidThings::uart_unregisterCallback(int32_t idx) {
    std::map<int, sp<Callback>>::iterator it;
    it = callbackList.find(CALLBACK_OFFSET_UART + idx);
    if (it != callbackList.end()) {
        mDevice->uart_ops.unregisterCallback(idx);
        callbackList.erase(it);
    }

    return Void();
}

Return<void> OdroidThings::spi_open(int32_t idx) {
    mDevice->spi_ops.open(idx);
    return Void();
}

Return<void> OdroidThings::spi_close(int32_t idx) {
    mDevice->spi_ops.close(idx);
    return Void();
}

Return<bool> OdroidThings::spi_setBitJustification(int32_t idx, uint8_t justification) {
    return mDevice->spi_ops.setBitJustification(idx, justification);
}

Return<bool> OdroidThings::spi_setBitsPerWord(int32_t idx, uint8_t bits) {
    return mDevice->spi_ops.setBitsPerWord(idx, bits);
}

Return<bool> OdroidThings::spi_setMode(int32_t idx, uint8_t mode) {
    return mDevice->spi_ops.setMode(idx, mode);
}

Return<bool> OdroidThings::spi_setCsChange(int32_t idx, bool cs) {
    return mDevice->spi_ops.setCsChange(idx, cs);
}

Return<bool> OdroidThings::spi_setDelay(int32_t idx, uint16_t delay) {
    return mDevice->spi_ops.setDelay(idx, delay);
}

Return<bool> OdroidThings::spi_setFrequency(int32_t idx, uint32_t frequency) {
    return mDevice->spi_ops.setFrequency(idx, frequency);
}

Return<void> OdroidThings::spi_transfer(int32_t idx, const hidl_vec<uint8_t>& txBuffer, int32_t length, spi_transfer_cb _hidl_cb) {
    hidl_vec<uint8_t> rxList;
    std::vector<uint8_t> rxBuffer = mDevice->spi_ops.transfer(idx, txBuffer, length);

    for(int32_t i = 0; i < length; i++)
        rxList[i] = rxBuffer[i];
    _hidl_cb(length, rxList);

    return Void();
}

Return<void> OdroidThings::spi_read(int32_t idx, int32_t length, spi_read_cb _hidl_cb) {
    hidl_vec<uint8_t> rxList;
    std::vector<uint8_t> rxBuffer = mDevice->spi_ops.read(idx, length);

    for(int32_t i = 0; i < length; i++)
        rxList[i] = rxBuffer[i];
    _hidl_cb(length, rxList);

    return Void();
}

Return<bool> OdroidThings::spi_write(int32_t idx, const hidl_vec<uint8_t>& txBuffer, int32_t length) {
    return mDevice->spi_ops.write(idx, txBuffer, length);
}

#if defined(__LP64__)
#define THINGS_PATH "/vendor/lib64/hw/odroidThings.so"
#else
#define THINGS_PATH "/vendor/lib/hw/odroidThings.so"
#endif

things_device_t* OdroidThings::openHal() {
    int err;
    const hw_module_t *hw_mdl = nullptr;
    ALOGD("Opening odroidThings hal library...");
    void *handle = dlopen(THINGS_PATH, RTLD_NOW);
    if ( handle == NULL) {
        ALOGE("module load err");
    }
    const char *sym = HAL_MODULE_INFO_SYM_AS_STR;
    hw_mdl = (hw_module_t*)dlsym(handle, sym);

    if (hw_mdl == nullptr) {
        ALOGE("No valid odroidThings module");
        return nullptr;
    }
    things_module_t const *module =
        reinterpret_cast<const things_module_t*>(hw_mdl);
    if (module->common.methods->open == nullptr) {
        ALOGE("No valid open method");
        return nullptr;
    }

    // initialize odroid things hal or TODO: be merge this function to open
    module->init();

    hw_device_t *device = nullptr;

    if (0 != (err = module->common.methods->open(hw_mdl, nullptr, &device))) {
        ALOGE("Can't open odroidThings methods, error: %d", err);
        return nullptr;
    }

    things_device_t* ot_device =
        reinterpret_cast<things_device_t*>(device);

    return ot_device;
}

} // namespace vendor::hardkernel::hardware::odroidthings::V1_0::implementation
