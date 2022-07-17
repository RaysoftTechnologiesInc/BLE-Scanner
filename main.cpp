#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "gap/Gap.h"
#include "gap/AdvertisingDataParser.h"
#include "pretty_printer.h"
#include "BleScanner.h"

/**
 *  This example demonstrates all the basic setup required
 *  to scan advertising packets from peripherals.
 *
 *  It contains a single class(BleScanner) that performs scans.
 *
 */

events::EventQueue event_queue;

DigitalOut led1(LED1);
DigitalOut wifi_pwr(ESP_PWR);
DigitalOut wifi_en(ESP_EN);
DigitalOut boot(ESP_BOOT);
DigitalOut WDT(uWDI);

void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    boot = 1;
    //wifi_en = 1;
    wifi_pwr = 1;
    BLE &ble = BLE::Instance();

    /* this will inform us off all events so we can schedule their handling
     * using our event queue */
    ble.onEventsToProcess(schedule_ble_events);

    BleScanner demo(ble, event_queue);

    while (1) {
        demo.run();
        wait_ms(3000);
    };

    return 0;
}

































// #include <events/mbed_events.h>
// #include <mbed.h>
// #include "ble/BLE.h"
// #include "pretty_printer.h"

// static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

// DigitalOut led1(LED1);
// DigitalOut wifi_pwr(ESP_PWR);
// DigitalOut wifi_en (ESP_EN);
// DigitalOut boot(ESP_BOOT);
// DigitalOut WDT(uWDI);

// /** @deprecated This demo is deprecated and no replacement is currently available.
//  */
// MBED_DEPRECATED_SINCE(
//    "mbed-os-5.11",
//    "This demo is deprecated and no replacement is currently available."
// )
// class BeaconDemo : ble::Gap::EventHandler {
// public:
//     BeaconDemo(BLE &ble, events::EventQueue &event_queue) :
//         _ble(ble),
//         _event_queue(event_queue),
//         _adv_data_builder(_adv_buffer) { }

//     void start() {
//         _ble.gap().setEventHandler(this);

//         _ble.init(this, &BeaconDemo::on_init_complete);

//         _event_queue.dispatch_forever();
//     }

// private:
//     /**
//      * iBeacon payload builder.
//      *
//      * This data structure contains the payload of an iBeacon. The payload is
//      * built at construction time and application code can set up an iBeacon by
//      * injecting the raw field into the GAP advertising payload as a
//      * GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA.
//      */
//     union Payload {
//         /**
//          * Raw data of the payload.
//          */
//         uint8_t raw[25];
//         struct {
//             /**
//              * Beacon manufacturer identifier.
//              */
//             uint16_t companyID;

//             /**
//              * Packet ID; Equal to 2 for an iBeacon.
//              */
//             uint8_t ID;

//             /**
//              * Length of the remaining data presents in the payload.
//              */
//             uint8_t len;

//             /**
//              * Beacon UUID.
//              */
//             uint8_t proximityUUID[16];

//             /**
//              * Beacon Major group ID.
//              */
//             uint16_t majorNumber;

//             /**
//              * Beacon minor ID.
//              */
//             uint16_t minorNumber;

//             /**
//              * Tx power received at 1 meter; in dBm.
//              */
//             uint8_t txPower;
//         };

//         /**
//          * Assemble an iBeacon payload.
//          *
//          * @param[in] uuid Beacon network ID. iBeacon operators use this value
//          * to group their iBeacons into a single network, a single region and
//          * identify their organization among others.
//          *
//          * @param[in] majNum Beacon major group ID. iBeacon exploitants may use
//          * this field to divide the region into subregions, their network into
//          * subnetworks.
//          *
//          * @param[in] minNum Identifier of the Beacon in its subregion.
//          *
//          * @param[in] transmitPower Measured transmit power of the beacon at 1
//          * meter. Scanners use this parameter to approximate the distance
//          * to the beacon.
//          *
//          * @param[in] companyIDIn ID of the beacon manufacturer.
//          */
//         Payload(
//             const uint8_t *uuid,
//             uint16_t majNum,
//             uint16_t minNum,
//             uint8_t transmitPower,
//             uint16_t companyIDIn
//         ) : companyID(companyIDIn),
//             ID(0x02),
//             len(0x15),
//             majorNumber(__REV16(majNum)),
//             minorNumber(__REV16(minNum)),
//             txPower(transmitPower)
//         {
//             memcpy(proximityUUID, uuid, 16);
//         }
//     };

//     /** Callback triggered when the ble initialization process has finished */
//     void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
//         if (params->error != BLE_ERROR_NONE) {
//             printf("Ble initialization failed.");
//             return;
//         }

//         print_mac_address();

//         start_advertising();
//     }

//     void start_advertising() {
//         /* Create advertising parameters and payload */

//         ble::AdvertisingParameters adv_parameters(
//             ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
//             ble::adv_interval_t(ble::millisecond_t(1000))
//         );

//         _adv_data_builder.setFlags();

//         /**
//          * The Beacon payload has the following composition:
//          * 128-Bit / 16byte UUID = E2 0A 39 F4 73 F5 4B C4 A1 2F 17 D1 AD 07 A9 61
//          * Major/Minor  = 0x1122 / 0x3344
//          * Tx Power     = 0xC8 = 200, 2's compliment is 256-200 = (-56dB)
//          *
//          * Note: please remember to calibrate your beacons TX Power for more accurate results.
//          */
//         static const uint8_t uuid[] = { 0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4,
//                                         0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61 };
//         uint16_t major_number = 1122;
//         uint16_t minor_number = 3344;
//         uint16_t tx_power     = 0xC8;
//         uint16_t comp_id      = 0x004C;

//         Payload ibeacon(uuid, major_number, minor_number, tx_power, comp_id);

//         _adv_data_builder.setManufacturerSpecificData(ibeacon.raw);

//         /* Setup advertising */

//         ble_error_t error = _ble.gap().setAdvertisingParameters(
//             ble::LEGACY_ADVERTISING_HANDLE,
//             adv_parameters
//         );

//         if (error) {
//             print_error(error, "_ble.gap().setAdvertisingParameters() failed");
//             return;
//         }

//         error = _ble.gap().setAdvertisingPayload(
//             ble::LEGACY_ADVERTISING_HANDLE,
//             _adv_data_builder.getAdvertisingData()
//         );

//         if (error) {
//             print_error(error, "_ble.gap().setAdvertisingPayload() failed");
//             return;
//         }

//         /* Start advertising */

//         error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

//         if (error) {
//             print_error(error, "_ble.gap().startAdvertising() failed");
//             return;
//         }
//     }

// private:
//     /* Event handler */

//     void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
//         _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
//     }

// private:
//     BLE &_ble;
//     events::EventQueue &_event_queue;

//     uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
//     ble::AdvertisingDataBuilder _adv_data_builder;
// };

// /** Schedule processing of events from the BLE middleware in the event queue. */
// void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
//     event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
// }

// int main()
// {
//     boot = 1;
//     wifi_en = 1;
//     wifi_pwr = 1;
//     BLE &ble = BLE::Instance();
//     ble.onEventsToProcess(schedule_ble_events);

//     BeaconDemo demo(ble, event_queue);
//     demo.start();

//     return 0;
// }














// #include <events/mbed_events.h>
// #include <mbed.h>
// #include "ble/BLE.h"
// #include "ble/Gap.h"
// #include "ble/services/BatteryService.h"
// //#include "pretty_printer.h"

// DigitalOut led1(LED1);
// DigitalOut wifi_pwr(ESP_PWR);
// DigitalOut wifi_en (ESP_EN);
// DigitalOut boot(ESP_BOOT);
// DigitalOut WDT(uWDI);

// const static char DEVICE_NAME[] = "BATTERY";

// static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

// class BatteryDemo : ble::Gap::EventHandler {
// public:
//     BatteryDemo(BLE &ble, events::EventQueue &event_queue) :
//         _ble(ble),
//         _event_queue(event_queue),
//         _battery_uuid(GattService::UUID_BATTERY_SERVICE),
//         _battery_level(50),
//         _battery_service(ble, _battery_level),
//         _adv_data_builder(_adv_buffer) { }

//     void start() {
//         _ble.gap().setEventHandler(this);

//         _ble.init(this, &BatteryDemo::on_init_complete);

//         _event_queue.call_every(500, this, &BatteryDemo::blink);
//         _event_queue.call_every(1000, this, &BatteryDemo::update_sensor_value);

//         _event_queue.dispatch_forever();
//     }

// private:
//     /** Callback triggered when the ble initialization process has finished */
//     void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
//         if (params->error != BLE_ERROR_NONE) {
//             printf("Ble initialization failed.");
//             return;
//         }

//         //print_mac_address();

//         start_advertising();
//     }

//     void start_advertising() {
//         /* Create advertising parameters and payload */

//         ble::AdvertisingParameters adv_parameters(
//             ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
//             ble::adv_interval_t(ble::millisecond_t(100))
//         );

//         _adv_data_builder.setFlags();
//         _adv_data_builder.setLocalServiceList(mbed::make_Span(&_battery_uuid, 1));
//         _adv_data_builder.setName(DEVICE_NAME);

//         /* Setup advertising */

//         ble_error_t error = _ble.gap().setAdvertisingParameters(
//             ble::LEGACY_ADVERTISING_HANDLE,
//             adv_parameters
//         );

//         if (error) {
//             printf("_ble.gap().setAdvertisingParameters() failed");
//             return;
//         }

//         error = _ble.gap().setAdvertisingPayload(
//             ble::LEGACY_ADVERTISING_HANDLE,
//             _adv_data_builder.getAdvertisingData()
//         );

//         if (error) {
//             printf("_ble.gap().setAdvertisingPayload() failed");
//             return;
//         }

//         /* Start advertising */

//         error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

//         if (error) {
//             printf("_ble.gap().startAdvertising() failed");
//             return;
//         }
//     }

//     void update_sensor_value() {
//         if (_ble.gap().getState().connected) {
//             _battery_level++;
//             if (_battery_level > 100) {
//                 _battery_level = 20;
//             }

//             _battery_service.updateBatteryLevel(_battery_level);
//         }
//     }

//     void blink(void) {
//         led1 = !led1;
//     }

// private:
//     /* Event handler */

//     void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
//         _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
//     }

// private:
//     BLE &_ble;
//     events::EventQueue &_event_queue;

//     UUID _battery_uuid;

//     uint8_t _battery_level;
//     BatteryService _battery_service;

//     uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
//     ble::AdvertisingDataBuilder _adv_data_builder;
// };

// /** Schedule processing of events from the BLE middleware in the event queue. */
// void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
//     event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
// }

// int main()
// {
//     boot = 1;
//     wifi_en = 1;
//     wifi_pwr = 1;
//     BLE &ble = BLE::Instance();
//     ble.onEventsToProcess(schedule_ble_events);

//     BatteryDemo demo(ble, event_queue);
//     demo.start();

//     return 0;
// }




// #include "mbed.h"
// #include "BLE.h"
// #include "ble/Gap.h"
// #include "events/EventQueue.h"
// #include "platform/NonCopyable.h"



// #include "BLE Utils/gatt_client_process.h"
// #include "mbed-trace/mbed_trace.h"
// #include "BatteryService.h"
// #include "BLE.h"
// #include "ble/Gap.h"
// #include "ble/GapAdvertisingParams.h"
// #include "ble/GapAdvertisingData.h"
// #include "ble/GattClient.h"
// #include "ble/DiscoveredService.h"
// #include "ble/DiscoveredCharacteristic.h"
// #include "ble/CharacteristicDescriptorDiscovery.h"

// #include "BLEProcess.h"


// //const static char DEVICE_NAME[] = "Battery";

// DigitalOut led1(LED1);
// DigitalOut wifi_pwr(ESP_PWR);
// DigitalOut wifi_en (ESP_EN);
// DigitalOut boot(ESP_BOOT);
// DigitalOut WDT(uWDI);


// /**
//  * Handle discovery of the GATT server.
//  *
//  * First the GATT server is discovered in its entirety then each readable
//  * characteristic is read and the client register to characteristic
//  * notifications or indication when available. The client report server
//  * indications and notification until the connection end.
//  */
// class GattClientProcess : private mbed::NonCopyable<GattClientProcess>,
//                           public ble::Gap::EventHandler,
//                           public GattClient::EventHandler {

//     // Internal typedef to this class type.
//     // It is used as a shorthand to pass member function as callbacks.
//     typedef GattClientProcess Self;

//     typedef CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t
//         DiscoveryCallbackParams_t;

//     typedef CharacteristicDescriptorDiscovery::TerminationCallbackParams_t
//         TerminationCallbackParams_t;

//     typedef DiscoveredCharacteristic::Properties_t Properties_t;

// public:

//     /**
//      * Construct an empty client process.
//      *
//      * The function start() shall be called to initiate the discovery process.
//      */
//     GattClientProcess() :
//         _client(NULL),
//         _connection_handle(),
//         _characteristics(NULL),
//         _it(NULL),
//         _descriptor_handle(0),
//         _ble_interface(NULL),
//         _event_queue(NULL) {
//     }

//     ~GattClientProcess()
//     {
//         stop();
//     }

//     void init(BLE &ble_interface, events::EventQueue &event_queue)
//     {
//         _ble_interface = &ble_interface;
//         _event_queue = &event_queue;
//         _client = &_ble_interface->gattClient();

//         _ble_interface->gap().setEventHandler(this);
//     }

//     /**
//      * Start the discovery process.
//      *
//      * @param[in] client The GattClient instance which will discover the distant
//      * GATT server.
//      * @param[in] connection_handle Reference of the connection to the GATT
//      * server which will be discovered.
//      */
//     void start()
//     {
//         // setup the event handlers called during the process
//         _client->onDataWritten().add(as_cb(&Self::when_descriptor_written));
//         _client->onHVX().add(as_cb(&Self::when_characteristic_changed));

//         // The discovery process will invoke when_service_discovered when a
//         // service is discovered, when_characteristic_discovered when a
//         // characteristic is discovered and when_service_discovery_ends once the
//         // discovery process has ended.
//         _client->onServiceDiscoveryTermination(as_cb(&Self::when_service_discovery_ends));
//         ble_error_t error = _client->launchServiceDiscovery(
//             _connection_handle,
//             as_cb(&Self::when_service_discovered),
//             as_cb(&Self::when_characteristic_discovered)
//         );

//         if (error) {
//             printf("Error %u returned by _client->launchServiceDiscovery.\r\n", error);
//             return;
//         }

//         // register as a handler for GattClient events
//         _client->setEventHandler(this);

//         // this might not result in a new value but if it does we will be informed through
//         // an call in the event handler we just registered
//         _client->negotiateAttMtu(_connection_handle);

//         printf("Client process started: initiate service discovery.\r\n");
//     }

//     /**
//      * Stop the discovery process and clean the instance.
//      */
//     void stop()
//     {
//         if (!_client) {
//             return;
//         }

//         // unregister event handlers
//         _client->onDataWritten().detach(as_cb(&Self::when_descriptor_written));
//         _client->onHVX().detach(as_cb(&Self::when_characteristic_changed));
//         _client->onServiceDiscoveryTermination(NULL);

//         // remove discovered characteristics
//         clear_characteristics();

//         // clean up the instance
//         _connection_handle = 0;
//         _characteristics = NULL;
//         _it = NULL;
//         _descriptor_handle = 0;

//         printf("Client process stopped.\r\n");
//     }

// private:
//     /**
//      * Event handler invoked when a connection is established.
//      *
//      * This function setup the connection handle to operate on then start the
//      * discovery process.
//      */
//     virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event)
//     {
//         _connection_handle = event.getConnectionHandle();
//         _event_queue->call(mbed::callback(this, &Self::start));
//     }

//     /**
//      * Stop the discovery process and clean the instance.
//      */
//     virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event)
//     {
//         if (_client && event.getConnectionHandle() == _connection_handle) {
//             stop();
//         }
//     }

//     /**
//      * Implementation of GattClient::EventHandler::onAttMtuChange event
//      */
//     virtual void onAttMtuChange(
//         ble::connection_handle_t connectionHandle,
//         uint16_t attMtuSize
//     ) {
//         printf(
//             "ATT_MTU changed on the connection %d to a new value of %d.\r\n",
//             connectionHandle,
//             attMtuSize
//             /* maximum size of an attribute written in a single operation is one less */
//         );
//     }

// private:
// ////////////////////////////////////////////////////////////////////////////////
// // Service and characteristic discovery process.

//     /**
//      * Handle services discovered.
//      *
//      * The GattClient invokes this function when a service has been discovered.
//      *
//      * @see GattClient::launchServiceDiscovery
//      */
//     void when_service_discovered(const DiscoveredService *discovered_service)
//     {
//         // print information of the service discovered
//         printf("Service discovered: value = ");
//         print_uuid(discovered_service->getUUID());
//         printf(", start = %u, end = %u.\r\n",
//             discovered_service->getStartHandle(),
//             discovered_service->getEndHandle()
//         );
//     }

//     /**
//      * Handle characteristics discovered.
//      *
//      * The GattClient invoke this function when a characteristic has been
//      * discovered.
//      *
//      * @see GattClient::launchServiceDiscovery
//      */
//     void when_characteristic_discovered(const DiscoveredCharacteristic *discovered_characteristic)
//     {
//         // print characteristics properties
//         printf("\tCharacteristic discovered: uuid = ");
//         print_uuid(discovered_characteristic->getUUID());
//         printf(", properties = ");
//         print_properties(discovered_characteristic->getProperties());
//         printf(
//             ", decl handle = %u, value handle = %u, last handle = %u.\r\n",
//             discovered_characteristic->getDeclHandle(),
//             discovered_characteristic->getValueHandle(),
//             discovered_characteristic->getLastHandle()
//         );

//         // add the characteristic into the list of discovered characteristics
//         bool success = add_characteristic(discovered_characteristic);
//         if (!success) {
//             printf("Error: memory allocation failure while adding the discovered characteristic.\r\n");
//             _client->terminateServiceDiscovery();
//             stop();
//             return;
//         }
//     }

//     /**
//      * Handle termination of the service and characteristic discovery process.
//      *
//      * The GattClient invokes this function when the service and characteristic
//      * discovery process ends.
//      *
//      * @see GattClient::onServiceDiscoveryTermination
//      */
//     void when_service_discovery_ends(Gap::Handle_t connection_handle)
//     {
//         if (!_characteristics) {
//             printf("No characteristics discovered, end of the process.\r\n");
//             return;
//         }

//         printf("All services and characteristics discovered, process them.\r\n");

//         // reset iterator and start processing characteristics in order
//         _it = NULL;
//         _event_queue->call(mbed::callback(this, &Self::process_next_characteristic));
//     }

// ////////////////////////////////////////////////////////////////////////////////
// // Processing of characteristics based on their properties.

//     /**
//      * Process the characteristics discovered.
//      *
//      * - If the characteristic is readable then read its value and print it. Then
//      * - If the characteristic can emit notification or indication then discover
//      * the characteristic CCCD and subscribe to the server initiated event.
//      * - Otherwise skip the characteristic processing.
//      */
//     void process_next_characteristic(void)
//     {
//         if (!_it) {
//             _it = _characteristics;
//         } else {
//             _it = _it->next;
//         }

//         while (_it) {
//             Properties_t properties = _it->value.getProperties();

//             if (properties.read()) {
//                 read_characteristic(_it->value);
//                 return;
//             } else if(properties.notify() || properties.indicate()) {
//                 discover_descriptors(_it->value);
//                 return;
//             } else {
//                 printf(
//                     "Skip processing of characteristic %u\r\n",
//                     _it->value.getValueHandle()
//                 );
//                 _it = _it->next;
//             }
//         }

//         printf("All characteristics discovered have been processed.\r\n");
//     }

//     /**
//      * Initate the read of the characteristic in input.
//      *
//      * The completion of the operation will happens in when_characteristic_read()
//      */
//     void read_characteristic(const DiscoveredCharacteristic &characteristic)
//     {
//         printf("Initiating read at %u.\r\n", characteristic.getValueHandle());
//         ble_error_t error = characteristic.read(
//             0, as_cb(&Self::when_characteristic_read)
//         );

//         if (error) {
//             printf(
//                 "Error: cannot initiate read at %u due to %u\r\n",
//                 characteristic.getValueHandle(), error
//             );
//             stop();
//         }
//     }

//     /**
//      * Handle the reception of a read response.
//      *
//      * If the characteristic can emit notification or indication then start the
//      * discovery of the the characteristic descriptors then subscribe to the
//      * server initiated event by writing the CCCD discovered. Otherwise start
//      * the processing of the next characteristic discovered in the server.
//      */
//     void when_characteristic_read(const GattReadCallbackParams *read_event)
//     {
//         printf("\tCharacteristic value at %u equal to: ", read_event->handle);
//         for (size_t i = 0; i <  read_event->len; ++i) {
//             printf("0x%02X ", read_event->data[i]);
//         }
//         printf(".\r\n");

//         Properties_t properties = _it->value.getProperties();

//         if(properties.notify() || properties.indicate()) {
//             discover_descriptors(_it->value);
//         } else {
//             process_next_characteristic();
//         }
//     }

//     /**
//      * Initiate the discovery of the descriptors of the characteristic in input.
//      *
//      * When a descriptor is discovered, the function when_descriptor_discovered
//      * is invoked.
//      */
//     void discover_descriptors(const DiscoveredCharacteristic &characteristic)
//     {
//         printf("Initiating descriptor discovery of %u.\r\n", characteristic.getValueHandle());

//         _descriptor_handle = 0;
//         ble_error_t error = characteristic.discoverDescriptors(
//             as_cb(&Self::when_descriptor_discovered),
//             as_cb(&Self::when_descriptor_discovery_ends)
//         );

//         if (error) {
//             printf(
//                 "Error: cannot initiate discovery of %04X due to %u.\r\n",
//                 characteristic.getValueHandle(), error
//             );
//             stop();
//         }
//     }

//     /**
//      * Handle the discovery of the characteristic descriptors.
//      *
//      * If the descriptor found is a CCCD then stop the discovery. Once the
//      * process has ended subscribe to server initiated events by writing the
//      * value of the CCCD.
//      */
//     void when_descriptor_discovered(const DiscoveryCallbackParams_t* event)
//     {
//         printf("\tDescriptor discovered at %u, UUID: ", event->descriptor.getAttributeHandle());
//         print_uuid(event->descriptor.getUUID());
//         printf(".\r\n");

//         if (event->descriptor.getUUID() == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG) {
//             _descriptor_handle = event->descriptor.getAttributeHandle();
//             _client->terminateCharacteristicDescriptorDiscovery(
//                 event->characteristic
//             );
//         }
//     }

//     /**
//      * If a CCCD has been found subscribe to server initiated events by writing
//      * its value.
//      */
//     void when_descriptor_discovery_ends(const TerminationCallbackParams_t *event) {
//         // shall never happen but happen with android devices ...
//         // process the next charateristic
//         if (!_descriptor_handle) {
//             printf("\tWarning: characteristic with notify or indicate attribute without CCCD.\r\n");
//             process_next_characteristic();
//             return;
//         }

//         Properties_t properties = _it->value.getProperties();

//         uint16_t cccd_value =
//             (properties.notify() << 0) | (properties.indicate() << 1);

//         ble_error_t error = _client->write(
//             GattClient::GATT_OP_WRITE_REQ,
//             _connection_handle,
//             _descriptor_handle,
//             sizeof(cccd_value),
//             reinterpret_cast<uint8_t*>(&cccd_value)
//         );

//         if (error) {
//             printf(
//                 "Error: cannot initiate write of CCCD %u due to %u.\r\n",
//                 _descriptor_handle, error
//             );
//             stop();
//         }
//     }

//     /**
//      * Called when the CCCD has been written.
//      */
//     void when_descriptor_written(const GattWriteCallbackParams* event)
//     {
//         // should never happen
//         if (!_descriptor_handle) {
//             printf("\tError: received write response to unsolicited request.\r\n");
//             stop();
//             return;
//         }

//         printf("\tCCCD at %u written.\r\n", _descriptor_handle);
//         _descriptor_handle = 0;
//         process_next_characteristic();
//     }

//     /**
//      * Print the updated value of the characteristic.
//      *
//      * This function is called when the server emits a notification or an
//      * indication of a characteristic value the client has subscribed to.
//      *
//      * @see GattClient::onHVX()
//      */
//     void when_characteristic_changed(const GattHVXCallbackParams* event)
//     {
//         printf("Change on attribute %u: new value = ", event->handle);
//         for (size_t i = 0; i < event->len; ++i) {
//             printf("0x%02X ", event->data[i]);
//         }
//         printf(".\r\n");
//     }

//     struct DiscoveredCharacteristicNode {
//         DiscoveredCharacteristicNode(const DiscoveredCharacteristic &c) :
//             value(c), next(NULL) { }

//         DiscoveredCharacteristic value;
//         DiscoveredCharacteristicNode *next;
//     };

//     /**
//      * Add a discovered characteristic into the list of discovered characteristics.
//      */
//     bool add_characteristic(const DiscoveredCharacteristic *characteristic)
//     {
//         DiscoveredCharacteristicNode* new_node =
//             new(std::nothrow) DiscoveredCharacteristicNode(*characteristic);

//         if (!new_node) {
//             printf("Error while allocating a new characteristic.\r\n");
//             return false;
//         }

//         if (_characteristics == NULL) {
//             _characteristics = new_node;
//         } else {
//             DiscoveredCharacteristicNode* c = _characteristics;
//             while(c->next) {
//                 c = c->next;
//             }
//             c->next = new_node;
//         }

//         return true;
//     }

//     /**
//      * Clear the list of discovered characteristics.
//      */
//     void clear_characteristics(void)
//     {
//         DiscoveredCharacteristicNode *c= _characteristics;

//         while (c) {
//             DiscoveredCharacteristicNode *n = c->next;
//             delete c;
//             c = n;
//         }
//     }

//     /**
//      * Helper to construct an event handler from a member function of this
//      * instance.
//      */
//     template<typename ContextType>
//     FunctionPointerWithContext<ContextType> as_cb(
//         void (Self::*member)(ContextType context)
//     ) {
//         return makeFunctionPointer(this, member);
//     }

//     /**
//      * Print the value of a UUID.
//      */
//     static void print_uuid(const UUID &uuid)
//     {
//         const uint8_t *uuid_value = uuid.getBaseUUID();

//         // UUIDs are in little endian, print them in big endian
//         for (size_t i = 0; i < uuid.getLen(); ++i) {
//             printf("%02X", uuid_value[(uuid.getLen() - 1) - i]);
//         }
//     }

//     /**
//      * Print the value of a characteristic properties.
//      */
//     static void print_properties(const Properties_t &properties)
//     {
//         const struct {
//             bool (Properties_t::*fn)() const;
//             const char* str;
//         } prop_to_str[] = {
//             { &Properties_t::broadcast, "broadcast" },
//             { &Properties_t::read, "read" },
//             { &Properties_t::writeWoResp, "writeWoResp" },
//             { &Properties_t::write, "write" },
//             { &Properties_t::notify, "notify" },
//             { &Properties_t::indicate, "indicate" },
//             { &Properties_t::authSignedWrite, "authSignedWrite" }
//         };

//         printf("[");
//         for (size_t i = 0; i < (sizeof(prop_to_str) / sizeof(prop_to_str[0])); ++i) {
//             if ((properties.*(prop_to_str[i].fn))()) {
//                 printf(" %s", prop_to_str[i].str);
//             }
//         }
//         printf(" ]");
//     }

//     GattClient *_client;
//     Gap::Handle_t _connection_handle;
//     DiscoveredCharacteristicNode *_characteristics;
//     DiscoveredCharacteristicNode *_it;
//     GattAttribute::Handle_t _descriptor_handle;
//     BLE *_ble_interface;
//     events::EventQueue *_event_queue;
// };


// int main() {

//     boot = 1;
//     wifi_pwr = 1;
//     ThisThread::sleep_for(100);
//     BLE &ble_interface = BLE::Instance();
//     events::EventQueue event_queue;
//     BLEProcess ble_process(event_queue, ble_interface);
//     GattClientProcess gatt_client_process;

//     // Register GattClientProcess::init in the ble_process; this function will
//     // be called once the ble_interface is initialized.
//     ble_process.on_init(
//         mbed::callback(&gatt_client_process, &GattClientProcess::init)
//     );

//     // bind the event queue to the ble interface, initialize the interface
//     // and start advertising
//     ble_process.start();

//     // Process the event queue.
//     event_queue.dispatch_forever();

//     return 0;
// }


































































// static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

// class BleTestDemo : ble::Gap::EventHandler {
// public:
//     BleTestDemo(BLE &ble, events::EventQueue &event_queue) :
//         _ble(ble),
//         _event_queue(event_queue),
//         _battery_uuid(GattService::UUID_BATTERY_SERVICE),
//         _battery_level(50),
//         _battery_service(ble, _battery_level),
//         _adv_data_builder(_adv_buffer) { }

//     void start() {
//         _ble.gap().setEventHandler(this);

//         _ble.init(this, &BleTestDemo::on_init_complete);

//         _event_queue.call_every(500, this, &BleTestDemo::blink);
//         _event_queue.call_every(1000, this, &BleTestDemo::update_sensor_value);

//         _event_queue.dispatch_forever();
//     }

// private:
//     /** Callback triggered when the ble initialization process has finished */
//     void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
//         if (params->error != BLE_ERROR_NONE) {
//             printf("Ble initialization failed.");
//             return;
//         }

//         start_advertising();
//     }

//     void start_advertising() {
//         /* Create advertising parameters and payload */

//         ble::AdvertisingParameters adv_parameters(
//             ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
//             ble::adv_interval_t(ble::millisecond_t(1000))
//         );

//         _adv_data_builder.setFlags();
//         _adv_data_builder.setLocalServiceList(mbed::make_Span(&_battery_uuid, 1));
//         _adv_data_builder.setName(DEVICE_NAME);

//         /* Setup advertising */

//         ble_error_t error = _ble.gap().setAdvertisingParameters(
//             ble::LEGACY_ADVERTISING_HANDLE,
//             adv_parameters
//         );

//         if (error) {
//             printf("_ble.gap().setAdvertisingParameters() failed");
//             return;
//         }

//         error = _ble.gap().setAdvertisingPayload(
//             ble::LEGACY_ADVERTISING_HANDLE,
//             _adv_data_builder.getAdvertisingData()
//         );

//         if (error) {
//             printf("_ble.gap().setAdvertisingPayload() failed");
//             return;
//         }

//         /* Start advertising */

//         error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

//         if (error) {
//             printf("_ble.gap().startAdvertising() failed");
//             return;
//         }
//     }

//     void update_sensor_value() {
//         if (_ble.gap().getState().connected) {
//             _battery_level++;
//             if (_battery_level > 100) {
//                 _battery_level = 20;
//             }

//             _battery_service.updateBatteryLevel(_battery_level);
//         }
//     }

//     void blink(void) {
//         led1 = !led1;
//     }

// private:
//     /* Event handler */

//     void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
//         _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
//     }

// private:
//     BLE &_ble;
//     events::EventQueue &_event_queue;

//     UUID _battery_uuid;

//     uint8_t _battery_level;
//     BatteryService _battery_service;

//     uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
//     ble::AdvertisingDataBuilder _adv_data_builder;
// };

// /** Schedule processing of events from the BLE middleware in the event queue. */
// void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
//     event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
// }
// int main()
// {
//     boot = 1;
//     wifi_pwr = 1;
//     //wifi_en = 1;


//     BLE &ble = BLE::Instance();
//     ble.onEventsToProcess(schedule_ble_events);
//     BleTestDemo demo(ble, event_queue);
//     demo.start();

//     while(true)
//     {
//         //_socket->sendto(socketAddress, data, 18);
//         //sock->sendto(socketAddress, data, 18);
//         //led1 = !led1;
//         //WDT = !WDT;
//         ThisThread::sleep_for(100);
//     }
//     return 0;
// }





































































//  #include "mbed.h"
// #include "ESP32.h"
// #include "BLE.h"
// //#include "ble/Gap.h"

// DigitalOut led1(LED1);
// DigitalOut wifi_pwr(ESP_PWR);
// DigitalOut wifi_en (ESP_EN);
// DigitalOut boot(ESP_BOOT);
// DigitalOut WDT(uWDI);
// void PutEspInBootModeAndPause(void);

// int main (void)
// {
//     PutEspInBootModeAndPause();
//     // boot = 1;
//     // wifi_en = 1;
//     // wifi_pwr = 1;
//     // TCPSocket *_socket = new TCPSocket();
//     // UDPSocket *sock = new UDPSocket();

//     // NetworkInterface *net = NetworkInterface::get_default_instance();
//     // // sock->set_blocking(true);

//     //  int result = net->connect();
//     // // //int ret = _socket->open(net);
//     // // int res = sock->open(net);
//     // // char buf[50] = {0};
//     // const char *hostname = "192.168.1.63";
//     // //const char *data = "Hello from Client\n";
//     // // SocketAddress socketAddress(hostname, 8080);
//     // // //int ret = _socket->connect(socketAddress);
//     // // res = sock->connect(socketAddress);

//     // ESP32 esp(ESP_EN, NC, ESP_TX, ESP_RX, true, ESP_RTS, ESP_CTS, 115200);

//     // _socket->recvfrom(&socketAddress,buf, 5);
//     // //printf("Connecting to network!\n");



//     // // if (result != 0) {
//     // // // printf("Error! net->connect() returned: %d\n", result);
//     // //     int n  = result;
//     // //     for(int i = 0; i < 10; i++)
//     // //     {

//     // //         led1 = !led1;
//     // //         ThisThread::sleep_for(50);
//     // //     }
//     // //  }
//     // //int n  = result;

//     // const char *ap = "CXNK003C0E77";
//     // const char *password = "123456789";

//     // const char * random_addr = "11:22:33:44:55";
//     // const char * data = "Hello World from MicroRaptor!!!";

//     // esp.connect(ap, password);
//     // esp.open("TCP", 1, hostname, 8080, 0);
//     // esp.send(1, data, 18);





//     // while(true)
//     // {
//     //     //_socket->sendto(socketAddress, data, 18);
//     //     //sock->sendto(socketAddress, data, 18);
//     //     led1 = !led1;
//     //     WDT = !WDT;
//     //     ThisThread::sleep_for(100);
//     // }
// }

// void PutEspInBootModeAndPause(void)
// {
//     DigitalOut *espEnable   = new DigitalOut(ESP_EN, 0);
//     DigitalOut *espPower    = new DigitalOut(ESP_PWR, 0);
//     DigitalOut *espBoot     = new DigitalOut(ESP_BOOT, 0);

//     espBoot->write(0);
//     espEnable->write(0);
//     ThisThread::sleep_for(100);
//     espPower->write(1);
//     ThisThread::sleep_for(100);
//     espEnable->write(1);

//     for (;;) {
//         //Timers::watchdogTimer->Kick(stateEngineWatchdogSourceId);
//         printf(".");
//         ThisThread::sleep_for(1000);
//     }

//     delete espBoot;
//     delete espPower;
//     delete espEnable;
// }