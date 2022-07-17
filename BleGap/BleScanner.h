#ifndef BLE_SCANNER_H_
#define BLE_SCANNER_H_


#include <events/mbed_events.h>
#include <mbed.h>
#include "BLE.h"
#include "gap/Gap.h"
#include "gap/AdvertisingDataParser.h"


static const size_t MODE_DURATION_MS      = 6000;

class BleScanner : private mbed::NonCopyable<BleScanner>, public ble::Gap::EventHandler
{
private:
    EventQueue event_queue;

    virtual void onAdvertisingReport(const ble::AdvertisingReportEvent &event);
    virtual void onScanTimeout(const ble::ScanTimeoutEvent&);
    void end_scan_mode();
    void blink(void);
    


    BLE                &_ble;
    ble::Gap           &_gap;
    events::EventQueue &_event_queue;
    DigitalOut          _led1;
    bool                _is_in_scanning_mode;
    bool                _is_connecting;
    int                 _on_duration_end_id;
    size_t              _set_index;


public:
    BleScanner(BLE& ble, events::EventQueue &event_queue);
    ~BleScanner();

    void on_init_complete(BLE::InitializationCompleteCallbackContext *event);
    void ble_scanner_start();
    void scan();
    void scan_mode_start();
    void run();
};

#endif