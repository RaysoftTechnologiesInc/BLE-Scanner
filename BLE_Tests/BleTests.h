#ifndef BLETESTS_H__
#define BLETESTS_H__

#if BLE_FEATURE_GATT_SERVER

#include "platform/mbed_assert.h"

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"

class BleTests
{
private:
    /* data */
public:
    BleTests(BLE &_ble, uint8_t _testVal = 16):
                    ble(_ble), 
                    testVal(_testVal),
                    bleTestVal(
                        GattCharacteristic::UUID_BATTERY_LEVEL_CHAR,
                        &testVal,
                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NONE
                    )
{
    GattCharacteristic *charTable[] = { &bleTestVal };
    GattService bleTest(
        GattService::UUID_BATTERY_SERVICE,
        charTable,
        sizeof(charTable) / sizeof(charTable[0])
    );

    ble.gattServer().addService(bleTest);
}
    

    void updateTestVal(uint8_t newVal)
    {
        testVal = newVal;
        ble.gattServer().write(
            bleTestVal.getValueHandle(),
            &testVal,
            1
        );
    }
protected:
    BLE &ble;
    uint8_t testVal;
    ReadOnlyGattCharacteristic<uint8_t> bleTestVal;
};

// BleTests::BleTests(BLE &_ble, uint8_t _testVal = 16): 
//                     ble(_ble), 
//                     testVal(_testVal),
//                     bleTestVal(
//                         GattCharacteristic::UUID_BATTERY_LEVEL_CHAR,
//                         &testVal,
//                         GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NONE
//                     )
// {
//     GattCharacteristic *charTable[] = { &bleTestVal };
//     GattService bleTest(
//         GattService::UUID_BATTERY_SERVICE,
//         charTable,
//         sizeof(charTable) / sizeof(charTable[0])
//     )

//     ble.gattServer().addService(bleTest);
// }



#endif 

#endif 
