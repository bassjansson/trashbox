#include <RTClib.h>
#include <BluetoothA2DPSink.h>
#include <FastLED.h>

#define I2C_SDA_PIN  21 // default
#define I2C_SCL_PIN  22 // default

#define I2S_BCK_PIN  19
#define I2S_DATA_PIN 23
#define I2S_LRCK_PIN 18 // WS

#define IR_SENS_PIN  25
#define LED_DATA_PIN 27

#define NUM_OF_LEDS  30
#define BRIGHTNESS   255

#define BL_NAME      "Trashbox"

TwoWire           i2c = TwoWire(0);
RTC_DS3231        rtc;
BluetoothA2DPSink a2dpSink;

char daysOfTheWeek[7][4] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

CRGB leds[NUM_OF_LEDS];

float audioRMS = 0.0001f;

void readDataStream(const uint8_t * data, uint32_t length)
{
    int16_t * samples     = (int16_t *)data;
    uint32_t  sampleCount = length / 2;

    float rms = 0.0f;

    for (int i = 0; i < sampleCount; ++i)
    {
        float s = samples[i] / 32768.0f;
        rms += s * s;
    }

    audioRMS = sqrtf(rms / sampleCount) + 0.0001f;
}

void setup()
{
    // Setup serial communication
    Serial.begin(115200);
    // delay(3000); // wait for console opening


    // Setup IR sensor
    pinMode(IR_SENS_PIN, INPUT_PULLUP);
    digitalWrite(IR_SENS_PIN, HIGH);


    // Setup LEDs
    // FastLED.addLeds<WS2811, LED_DATA_PIN, RGB>(leds, NUM_OF_LEDS);
    // FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_OF_LEDS);  // GRB ordering is typical
    // FastLED.addLeds<WS2813, LED_DATA_PIN, RGB>(leds, NUM_OF_LEDS);
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_OF_LEDS); // GRB ordering is typical
    FastLED.setBrightness(BRIGHTNESS);


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
    a2dpSink.set_stream_reader(readDataStream);

    // Start bluetooth audio
    bool autoConnect = true;
    a2dpSink.start(BL_NAME, autoConnect); // To turn bluetooth on
    // a2dpSink.end(); // To turn bluetooth off
}

void loop()
{
    /*
    // Print RTC time
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
    */


    // Print IR reading
    static float ir = 0.0f;
    const float  f  = 0.9f;

    ir = ir * f + (1.0f - f) * digitalRead(IR_SENS_PIN);
    // Serial.println(ir * 5.0f);

    static int lastIR   = 0;
    int        irN      = digitalRead(IR_SENS_PIN);
    int        irChange = irN != lastIR;
    lastIR              = irN;


    // Update LEDs
    static int move = 0;
    move            = irChange ? !move : move;

    const float range  = 30.0f;
    float       rmsDb  = log10f(audioRMS) * 20.0f;
    float       rmsFlt = (rmsDb + range) / range;
    if (rmsFlt < 0.0f)
        rmsFlt = 0.0f;
    if (rmsFlt > 1.0f)
        rmsFlt = 1.0f;

    Serial.print(audioRMS * 5.0f);
    Serial.print(",");
    Serial.println(rmsFlt * 5.0f);

    for (int i = 0; i < NUM_OF_LEDS; ++i)
    {
        if (move)
            leds[i].setHSV(int(rmsFlt * 2.0f * 255) % 256, 255, rmsFlt * 255);
        else
            leds[i] = CRGB::Black;
    }

    FastLED.show();


    // Delay a little bit
    delay(25);
}
