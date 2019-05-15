/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "ble/services/EnvironmentalService.h"
#include "pretty_printer.h"
#include "x_cube_mems.h"


AxesRaw_TypeDef MAGNETIC;

const static char DEVICE_NAME[] = "Weather";
volatile float TEMPERATURE_C = 20;
volatile float HUMIDITY = 50;
volatile float PRESSURE = 1000;
volatile float WIND_DIRECTION = 0;

static volatile uint16_t hum = 0;                  
static volatile  int16_t tem = 0;
static volatile uint32_t pre = 0;
static volatile uint16_t win = 0;

//Serial pc(USBTX, USBRX);

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

class myCustomService
{

public:
    /**
     * Constructor.
     *
     * param[in] _ble
     *               Reference to the underlying BLEDevice.
     * param[in] humidity percentage (16-bit unsigned, 2 decimals).
     *               initial value for the humidity value.
     * param[in] temperature in degrees Celsius (16-bit signed, 2 decimals).
     *               initial value for the temperature
     */
    myCustomService (BLEDevice &_ble) :
        ble(_ble),

        humiditychar(GattCharacteristic::UUID_HUMIDITY_CHAR, (uint8_t *)&hum,
                     sizeof(uint16_t), sizeof(uint16_t),
                     GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),

        temperaturechar(GattCharacteristic::UUID_TEMPERATURE_CHAR, (uint8_t *)&tem,
                        sizeof(int16_t), sizeof(int16_t),
                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),

        pressurechar(GattCharacteristic::UUID_PRESSURE_CHAR, (uint8_t * )&pre,
                     sizeof(uint32_t), sizeof(uint32_t),
                     GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY),

        winddirectionchar(0x2A71, (uint8_t * )&win,
                          sizeof(uint16_t), sizeof(uint16_t),
                          GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)


    {
        // Setup Service

        GattCharacteristic *charTable[] = {&humiditychar, &temperaturechar, &pressurechar, &winddirectionchar  };

        GattService      myService(0x181A, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

        ble.addService(myService);
    }


    /* Set a new 16-bit value for the humidity measurement.  */
    void updateHumidity(uint16_t humidity)
    {
        ble.updateCharacteristicValue(humiditychar.getValueAttribute().getHandle(), (uint8_t *)&humidity, sizeof(uint16_t));
    }

    /* Set a new 16-bit value for the temperature measurement.  */
    void updateTemperature(int16_t temperature)
    {
        ble.updateCharacteristicValue(temperaturechar.getValueAttribute().getHandle(), (uint8_t *)&temperature, sizeof(int16_t));
    }

    void updatePressure( uint32_t pressure )
    {
        ble.updateCharacteristicValue(pressurechar.getValueAttribute().getHandle(), (uint8_t *)&pressure, sizeof(uint32_t));
    }

    void updateWinddirection(uint16_t winddirection)
    {
        ble.updateCharacteristicValue(winddirectionchar.getValueAttribute().getHandle(), (uint8_t *)&winddirection, sizeof(uint16_t));
    }


private:
    BLE                         &ble;
    GattCharacteristic          humiditychar;
    GattCharacteristic          temperaturechar;
    GattCharacteristic          winddirectionchar;
    GattCharacteristic          pressurechar;
    


};

class WeatherStation : ble::Gap::EventHandler
{
public:
    WeatherStation(BLE &ble, X_CUBE_MEMS* sensors, events::EventQueue &event_queue) :
        _ble(ble),
        _sensors(sensors),
        _event_queue(event_queue),
        _led1(LED1, 1),
        _connected(false),
        _uuid(0x181A),
        _service(ble),
        _adv_data_builder(_adv_buffer) { }

    void start()
    {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &WeatherStation::on_init_complete);

        _event_queue.call_every(500, this, &WeatherStation::blink);
        _event_queue.call_every(1000, this, &WeatherStation::update_sensor_value);

        _event_queue.dispatch_forever();
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params)
    {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }

        print_mac_address();

        start_advertising();
    }

    void start_advertising()
    {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        _adv_data_builder.setFlags();
        _adv_data_builder.setAppearance(ble::adv_data_appearance_t::GENERIC_THERMOMETER);
        _adv_data_builder.setLocalServiceList(mbed::make_Span(&_uuid, 1));
        _adv_data_builder.setName(DEVICE_NAME);

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
                                ble::LEGACY_ADVERTISING_HANDLE,
                                adv_parameters
                            );

        if (error) {
            printf("_ble.gap().setAdvertisingParameters() failed\r\n");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
                    ble::LEGACY_ADVERTISING_HANDLE,
                    _adv_data_builder.getAdvertisingData()
                );

        if (error) {
            printf("_ble.gap().setAdvertisingPayload() failed\r\n");
            return;
        }

        /* Start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            printf("_ble.gap().startAdvertising() failed\r\n");
            return;
        }
    }

    void update_sensor_value()
    {
        if (_connected) {
            // Do blocking calls or whatever is necessary for sensor polling.
            // In our case, we simply update the HRM measurement.

            _sensors->hts221.GetTemperature((float *)&TEMPERATURE_C);
            _sensors->hts221.GetHumidity((float *)&HUMIDITY);
            _sensors->lps25h.GetPressure((float *)&PRESSURE);
            _sensors->lis3mdl.GetAxes((AxesRaw_TypeDef *)&MAGNETIC);
            
            printf( "temp:\t%.2f\r\nhumid:\t%.2f\r\npress:\t%.2f\r\n\r\n",
            TEMPERATURE_C, HUMIDITY, PRESSURE );
            
            int16_t  new_tem = TEMPERATURE_C;
            uint16_t new_hum = HUMIDITY;
            uint32_t new_pre = PRESSURE;
            
            printf( "temp_correct:\t%d\r\n",  new_tem );
            printf( "humid_correct:\t%u\r\n", new_hum );
            printf( "press_correct:\t%u\r\n", new_pre );

            new_tem      = new_tem*100;            //2 decimals
            new_hum      = new_hum*100;            //2 decimals
            new_pre      = new_pre*1000;           //hPa to Pa + 1 decimal

            //Calcule the direction where the system is pointing relative to North.
            //I have used a simple empirical method to distinguish between 8 directions.
            if (MAGNETIC.AXIS_X < 140) WIND_DIRECTION = 0; //North
            else if (MAGNETIC.AXIS_X >= 140 && MAGNETIC.AXIS_X < 200 && -MAGNETIC.AXIS_Y > 250 ) WIND_DIRECTION = 45;  //Northeast
            else if (MAGNETIC.AXIS_X >= 140 && MAGNETIC.AXIS_X < 200 && -MAGNETIC.AXIS_Y < 250 ) WIND_DIRECTION = 315; //Northwest
            else if (MAGNETIC.AXIS_X >= 200 && MAGNETIC.AXIS_X < 280 && -MAGNETIC.AXIS_Y > 250 ) WIND_DIRECTION = 90;  //East
            else if (MAGNETIC.AXIS_X >= 200 && MAGNETIC.AXIS_X < 280 && -MAGNETIC.AXIS_Y < 250 ) WIND_DIRECTION = 270; //Weast
            else if (MAGNETIC.AXIS_X >= 280 && MAGNETIC.AXIS_X < 380 && -MAGNETIC.AXIS_Y > 250 ) WIND_DIRECTION = 135; //Southeast
            else if (MAGNETIC.AXIS_X >= 280 && MAGNETIC.AXIS_X < 380 && -MAGNETIC.AXIS_Y < 250 ) WIND_DIRECTION = 225; //Soutwest
            else if (MAGNETIC.AXIS_X >= 380) WIND_DIRECTION = 180; //South
            
            WIND_DIRECTION *= 100;

            _service.updatePressure(new_pre);
            _service.updateHumidity(new_hum);
            _service.updateTemperature(new_tem);
            _service.updateWinddirection(WIND_DIRECTION);
        }
    }

    void blink(void)
    {
        _led1 = !_led1;
    }

private:
    /* Event handler */

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&)
    {
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _connected = false;
    }

    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event)
    {
        if (event.getStatus() == BLE_ERROR_NONE) {
            _connected = true;
        }
    }

private:
    BLE &_ble;
    X_CUBE_MEMS *_sensors;
    events::EventQueue &_event_queue;
    DigitalOut _led1;

    bool _connected;

    UUID _uuid;

    myCustomService _service;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    BLE &ble = BLE::Instance();
    X_CUBE_MEMS *Sensors = X_CUBE_MEMS::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    WeatherStation demo(ble, Sensors, event_queue);
    demo.start();

    return 0;
}

