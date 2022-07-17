#ifndef __BLE_LED_SERVICE_H__
#define __BLE_LED_SERVICE_H__

#include "mbed-os\features\FEATURE_BLE\targets\TARGET_ESP32AT_BLE\Esp32AtGattServer.h"
#include "mbed.h"
#include "mbed-os\features\FEATURE_BLE\targets\TARGET_ESP32AT_BLE\Esp32AtGap.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"

class LEDService {
public:
    const static uint16_t LED_SERVICE_UUID              = 0xA000;
    const static uint16_t LED_STATE_CHARACTERISTIC_UUID = 0xA001;

    LEDService(BLEDevice &_ble, bool initialValueForLEDCharacteristic) :
        ble(_ble), ledState(LED_STATE_CHARACTERISTIC_UUID, &initialValueForLEDCharacteristic)
    {
        GattCharacteristic *charTable[] = {&ledState};
        GattService         ledService(LED_SERVICE_UUID, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

        ble.gattServer().addService(ledService);
    }

    GattAttribute::Handle_t getValueHandle() const
    {
        return ledState.getValueHandle();
    }

private:
    BLEDevice                         &ble;
    ReadWriteGattCharacteristic<bool> ledState;
};

#endif /* #ifndef __BLE_LED_SERVICE_H__ */