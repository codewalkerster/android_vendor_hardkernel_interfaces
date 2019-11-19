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

#include <unistd.h>

#include <hidl/HidlTransportSupport.h>
#include <log/log.h>
#include <utils/Errors.h>
#include <utils/StrongPointer.h>

#include "OdroidThings.h"

//#include <vendor/hardkernel/hardware/odroidthings/1.0/IOdroidThings.h>

using android::sp;

// libhwbinder:
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;

// Generated HIDL files
using vendor::hardkernel::hardware::odroidthings::V1_0::IOdroidThings;
using vendor::hardkernel::hardware::odroidthings::V1_0::implementation::OdroidThings;

// Main service entry point
int main() {
    ALOGE("odroid things service start");
    //Create an instance of our service class
    sp<IOdroidThings> service = OdroidThings::getInstance();

    configureRpcThreadpool(1/*?*/, true);

    if (service->registerAsService() != android::OK) {
        ALOGE("registerAsService failed");
        return 1;
    }

    // Join (forever) the thread pool we created for the service above
    joinRpcThreadpool();

    // We don't ever actually expect to return, so return an error if we do get here
    return 2;
}
