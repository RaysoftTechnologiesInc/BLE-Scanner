#include "BleScanner.h"
#include "platform/Span.h"

BleScanner::BleScanner(BLE &ble, events::EventQueue &event_queue):
    _ble(ble),
    _gap(ble.gap()),
    _event_queue(event_queue),
    _led1(LED1, 0),
    _set_index(2),
{
}

BleScanner::~BleScanner()
{
}


//called to handle the dvertising packets coming in from the peripherals, 
//it also parses the packets to retrieve the data we want
void BleScanner::onAdvertisingReport(const ble::AdvertisingReportEvent &event)
{
    ble::AdvertisingDataParser adv_parser(event.getPayload());
    auto data = event.getPayload();
    uint8_t payload[data.size()];
    for (int i = 0; i < data.size(); ++i) {
        payload[i] = data[i];
    }

    while (adv_parser.hasNext()) {
        ble::AdvertisingDataParser::element_t field = adv_parser.next();

        if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME)
        {
                char name[field.value.size()];
                printf("We found our device\r\n");
                printf("Complete Local Name: ");
                mbed::Span<const uint8_t> name1 = field.value;
                for(int i = 0; i < field.value.size(); i++)
                {
                    name[i] = field.value[i];
                    printf("%c", name[i]);
                }
                printf("\r\n");
        }
    }
    
        adv_parser.reset();
}

void BleScanner::onScanTimeout(const ble::ScanTimeoutEvent &)
{

    printf("Stopped scanning early due to timeout parameter\r\n");

}

void BleScanner::end_scan_mode()
{
    ble_error_t error = _ble.gap().stopScan();

    if (error) {
        printf("Error caused by Gap::stopScan");
    }
}

void BleScanner::blink(void)
{
    _led1 = !_led1;
}

void BleScanner::on_init_complete(BLE::InitializationCompleteCallbackContext *event)
{
    if (event->error) {
        printf(  "Error during the initialisation");
        return;
    }
   
    _event_queue.call(this, &BleScanner::scan_mode_start);
}

void BleScanner::scan()
{
    ble_error_t error;
    
    /* start scanning, onAdvertisingReport is called in the event of any advertising packets being encountered.
     * and scan requests responses */
    error = _ble.gap().startScan(scan_params.duration);
    if (error) {
        printf( "Error caused by Gap::startScan");
        return;
    }

    //print the scan parameters
    printf("Scanning started (interval: %dms, window: %dms, timeout: %dms).\r\n",
           scan_params.interval.valueInMs(), scan_params.window.valueInMs(), scan_params.duration.valueInMs());


}

void BleScanner::scan_mode_start()
{
    _event_queue.call(this, &BleScanner::scan);
    //_demo_duration.start();

    _on_duration_end_id = _event_queue.call_in(
                              MODE_DURATION_MS,
                              this,
                              &BleScanner::end_scan_mode
                          );

    printf("\r\n");
}

void BleScanner::run()
{
    if (_ble.hasInitialized()) {
        printf("Ble instance already initialised.\r\n");
        return;
    }

    /* handle gap events */
    _ble.gap().setEventHandler(this);

    ble_error_t error = _ble.init(this, &BleScanner::on_init_complete);
    if (error) {
        printf("Error returned by BLE::init");
        return;
    }

    /* to show we're running we'll blink every 500ms */
    _event_queue.call_every(500, this, &BleScanner::blink);

    /* this will not return until shutdown */
    _event_queue.dispatch_forever();
}



