#include "RTClib.h"
#include "BluetoothA2DPSink.h"

#define I2C_SDA_PIN  21 // default
#define I2C_SCL_PIN  22 // default

#define I2S_BCK_PIN  19
#define I2S_DATA_PIN 23
#define I2S_LRCK_PIN 18 // WS

#define BL_NAME      "Trashbox"

RTC_DS3231        rtc;
BluetoothA2DPSink a2dpSink;

char daysOfTheWeek[7][4] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

void setup()
{
    Serial.begin(115200);
    // delay(3000); // wait for console opening

    // Initialise RTC
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        while (true)
            ;
    }

    if (rtc.lostPower())
    {
        Serial.println("RTC lost power, lets set the time!");
        // rtc.adjust(DateTime(2022, 9, 10, 20, 48, 0)); // Change this to current time + 40 sec upload time
    }

    // Start bluetooth audio
    i2s_pin_config_t i2sPinConfig = {.bck_io_num   = I2S_BCK_PIN,
                                     .ws_io_num    = I2S_LRCK_PIN,
                                     .data_out_num = I2S_DATA_PIN,
                                     .data_in_num  = I2S_PIN_NO_CHANGE};
    a2dpSink.set_pin_config(i2sPinConfig);
    a2dpSink.start(BL_NAME); // To turn bluetooth on
    // a2dpSink.end(); // To turn bluetooth off
}

void loop()
{
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    delay(3000);
}
