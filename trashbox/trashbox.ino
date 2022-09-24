#include "RTClib.h"
#include "BluetoothA2DPSink.h"

#define I2C_SDA_PIN  21 // default
#define I2C_SCL_PIN  22 // default

#define I2S_BCK_PIN  19
#define I2S_DATA_PIN 23
#define I2S_LRCK_PIN 18 // WS

#define BL_NAME      "Trashbox"

TwoWire           i2c = TwoWire(0);
RTC_DS3231        rtc;
BluetoothA2DPSink a2dpSink;

char daysOfTheWeek[7][4] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

void setup()
{
    Serial.begin(115200);
    // delay(3000); // wait for console opening

    // Initialise RTC
    i2c.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000ul);

    if (!rtc.begin(&i2c))
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

    // Setup bluetooth audio
    i2s_pin_config_t i2sPinConfig = {.bck_io_num   = I2S_BCK_PIN,
                                     .ws_io_num    = I2S_LRCK_PIN,
                                     .data_out_num = I2S_DATA_PIN,
                                     .data_in_num  = I2S_PIN_NO_CHANGE};
    a2dpSink.set_pin_config(i2sPinConfig);

    static i2s_config_t i2sConfig = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = 44100, // updated automatically by A2DP
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = 0,   // default interrupt priority
        .dma_buf_count        = 16,  // 8 = default
        .dma_buf_len          = 256, // 64 = default
        .use_apll             = true,
        .tx_desc_auto_clear   = true // avoiding noise in case of data unavailability
    };
    a2dpSink.set_i2s_config(i2sConfig);

    // Start bluetooth audio
    a2dpSink.start(BL_NAME, false); // To turn bluetooth on, disable auto reconnect
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
