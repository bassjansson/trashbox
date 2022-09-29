#include <RTClib.h>
#include <FastLED.h>

#include "BluetoothA2DPSinkTB.hpp"

#define I2C_SDA_PIN  21 // default
#define I2C_SCL_PIN  22 // default

#define I2S_BCK_PIN  19
#define I2S_DATA_PIN 23
#define I2S_LRCK_PIN 18 // WS

#define IR_SENS_PIN  25
#define LED_DATA_PIN 27
#define BUTTON_PIN   35

#define NUM_OF_LEDS  30
#define BRIGHTNESS   64

#define BLOCK_SIZE   1024 // 4096, 2 channel 16-bit, somehow constant
#define AUTO_CONNECT false

#define BUTT_TIMEOUT (60 * 1)  // seconds
#define CONN_TIMEOUT (60 * 10) // seconds

#define INIT_VOLUME  80 // percent

#define BLEEP_VOLUME 0.05f  // 0 to 1
#define BLEEP_FREQ   220.0f // Hertz
#define BLEEP_DUR    0.15f  // seconds

#define AUD_FB_RANGE 30.0f // dB

#define DAY_START_HR 8  // hours
#define DAY_END_HR   22 // hours

#define NUM_OF_SOUND 4

#define BL_NAME      "Trashbox"

TwoWire             i2c = TwoWire(0);
RTC_DS3231          rtc;
BluetoothA2DPSinkTB a2dpSink;

char daysOfTheWeek[7][4] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

CRGB leds[NUM_OF_LEDS];

float audioRMS = 0.0001f;

int16_t * soundData[NUM_OF_SOUND];
size_t    soundDataSize[NUM_OF_SOUND];

enum MainState
{
    WAIT_FOR_TRASH = 0,
    WAIT_FOR_BUTTON,
    WAIT_FOR_CONNECT,
    WAIT_FOR_DISCONNECT
};

MainState mainState = WAIT_FOR_TRASH;

uint32_t buttonTimeout  = 0;
uint32_t connectTimeout = 0;

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

    // Serial.println(length); // To know the block size
}

float tri(float phase)
{
    return fabsf(fmodf(phase + 0.75f, 1.0f) * 2.0f - 1.0f) * 2.0f - 1.0f;
}

float bell(float phase)
{
    float x = fmodf(phase, 1.0f) * 2.0f - 1.0f;
    return 1.0f - x * x;
}

void generateSounds()
{
    // Common
    uint16_t sampleRate  = a2dpSink.sample_rate();
    float    volume      = BLEEP_VOLUME;
    float    mainFreq    = BLEEP_FREQ; // Hertz
    float    noteTime    = BLEEP_DUR;  // seconds
    size_t   noteTimeInt = (size_t)(noteTime * sampleRate / BLOCK_SIZE + 0.5f) * BLOCK_SIZE;
    float    noteTimeAct = (float)noteTimeInt / sampleRate;
    int      s;
    float    t, freq, env, phas;
    int16_t  samp;


    // Sound 0 = trash thrown in
    s = 0;

    soundDataSize[s] = noteTimeInt * 6 + BLOCK_SIZE * 4; // 3 notes and 4 empty blocks
    soundData[s]     = (int16_t *)ps_malloc(sizeof(int16_t) * soundDataSize[s] * 2);

    for (int i = 0; i < soundDataSize[s]; ++i)
    {
        t    = (float)i / sampleRate;
        freq = t < noteTimeAct * 6.0f ? mainFreq : 0.0f;
        env  = bell(t / (noteTimeAct * 6.0f));
        phas = freq * t;
        samp = (int16_t)((tri(0.5f * phas) * 1.5f + tri(1.25f * phas) + tri(1.5f * phas) * 0.75f + tri(2.0f * phas) * 0.5f)
                         / 2.0f * env * volume * 32767);

        soundData[s][i * 2 + 0] = samp; // left
        soundData[s][i * 2 + 1] = samp; // right
    }


    // Sound 1 = wait for connect
    s = 1;

    soundDataSize[s] = noteTimeInt * 6 + BLOCK_SIZE * 4; // 3 notes and 4 empty blocks
    soundData[s]     = (int16_t *)ps_malloc(sizeof(int16_t) * soundDataSize[s] * 2);

    for (int i = 0; i < soundDataSize[s]; ++i)
    {
        t    = (float)i / sampleRate;
        freq = t < noteTimeAct * 6.0f ? mainFreq : 0.0f;
        env  = bell(t / (noteTimeAct * 3.0f));
        samp = (int16_t)(tri(freq * t) * env * volume * 32767);

        soundData[s][i * 2 + 0] = samp; // left
        soundData[s][i * 2 + 1] = samp; // right
    }


    // Sound 2 = connected
    s = 2;

    soundDataSize[s] = noteTimeInt * 3 + BLOCK_SIZE * 4; // 3 notes and 4 empty blocks
    soundData[s]     = (int16_t *)ps_malloc(sizeof(int16_t) * soundDataSize[s] * 2);

    for (int i = 0; i < soundDataSize[s]; ++i)
    {
        t    = (float)i / sampleRate;
        freq = t < noteTimeAct
                 ? mainFreq
                 : (t < noteTimeAct * 2.0f ? mainFreq * 1.25f : (t < noteTimeAct * 3.0f ? mainFreq * 1.5f : 0.0f));
        env  = bell(t / noteTimeAct);
        samp = (int16_t)(tri(freq * t) * env * volume * 32767);

        soundData[s][i * 2 + 0] = samp; // left
        soundData[s][i * 2 + 1] = samp; // right
    }


    // Sound 3 = disconnected
    s = 3;

    soundDataSize[s] = noteTimeInt * 3 + BLOCK_SIZE * 4; // 3 notes and 4 empty blocks
    soundData[s]     = (int16_t *)ps_malloc(sizeof(int16_t) * soundDataSize[s] * 2);

    for (int i = 0; i < soundDataSize[s]; ++i)
    {
        t    = (float)i / sampleRate;
        freq = t < noteTimeAct ? mainFreq * 1.5f
                               : (t < noteTimeAct * 2.0f ? mainFreq * 1.25f : (t < noteTimeAct * 3.0f ? mainFreq : 0.0f));
        env  = bell(t / noteTimeAct);
        samp = (int16_t)(tri(freq * t) * env * volume * 32767);

        soundData[s][i * 2 + 0] = samp; // left
        soundData[s][i * 2 + 1] = samp; // right
    }
}

void playSoundTask(void * parameter)
{
    // Serial.println("Start sound");

    int s = *((int *)parameter);

    int           blocks     = soundDataSize[s] / BLOCK_SIZE;
    uint16_t      sampleRate = a2dpSink.sample_rate();
    unsigned long start      = millis();
    unsigned long wait;

    for (int b = 0; b < blocks; ++b)
    {
        a2dpSink.write_audio((uint8_t *)soundData[s] + b * BLOCK_SIZE * 2 * 2, BLOCK_SIZE * 2 * 2);

        wait = start + b * 1000ul * BLOCK_SIZE / sampleRate - 1ul - millis();

        if (wait < 1000ul)
            vTaskDelay(wait / portTICK_PERIOD_MS);
    }

    // Serial.println("End sound");

    vTaskDelete(NULL);
}

void playSound(int sound = 0)
{
    static int snd = 0;

    snd = sound;

    xTaskCreatePinnedToCore(playSoundTask,   // Function that should be called
                            "playSoundTask", // Name of the task (for debugging)
                            4096,            // Stack size (bytes)
                            (void *)&snd,    // Parameter to pass
                            10,              // Task priority
                            NULL,            // Task handle
                            0                // Core you want to run the task on (0 or 1)
    );
}

void waitForTrash(uint32_t time)
{
    // Update LEDs
    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i] = CRGB::Black;
    FastLED.show();


    // Check trash sensors
    if (digitalRead(IR_SENS_PIN) == LOW) // TODO: more sensors will be added here
    {
        // Log state
        Serial.println("User threw in trash.");

        // Set timer
        buttonTimeout = time + BUTT_TIMEOUT;

        // Play sound
        playSound(0);

        // Switch state
        mainState = WAIT_FOR_BUTTON;
    }
}

void waitForButton(uint32_t time)
{
    // Check timer
    if (time > buttonTimeout)
    {
        // Log state
        Serial.println("Button timeout reached.");

        // Turn LEDs off
        for (int i = 0; i < NUM_OF_LEDS; ++i)
            leds[i] = CRGB::Black;
        FastLED.show();

        // Restart ESP
        ESP.restart();
    }


    // Update LEDs
    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i] = CRGB::Green;
    FastLED.show();


    // Check start bluetooth button
    if (digitalRead(BUTTON_PIN) == LOW) // TODO: button debouncing needs to be added here
    {
        // Log state
        Serial.println("User pressed button.");

        // Set timer
        connectTimeout = time + CONN_TIMEOUT;

        // Start bluetooth
        a2dpSink.start(BL_NAME, AUTO_CONNECT);

        // Play sound
        playSound(1);

        // Switch state
        mainState = WAIT_FOR_CONNECT;
    }
}

void waitForConnect(uint32_t time)
{
    // Check timer
    if (time > connectTimeout)
    {
        // Log state
        Serial.println("Connect timeout reached.");

        // Turn LEDs off
        for (int i = 0; i < NUM_OF_LEDS; ++i)
            leds[i] = CRGB::Black;
        FastLED.show();

        // End audio sink
        a2dpSink.end(true);

        // Restart ESP
        ESP.restart();
    }


    // Update LEDs
    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i] = CRGB::Blue;
    FastLED.show();


    // Check bluetooth connection
    if (a2dpSink.is_connected())
    {
        // Log state
        Serial.println("User connected.");

        // Set initial volume
        a2dpSink.set_volume(INIT_VOLUME);

        // Play sound
        playSound(2);

        // Switch state
        mainState = WAIT_FOR_DISCONNECT;
    }
}

void waitForDisconnect(uint32_t time)
{
    // Check timer
    if (time > connectTimeout)
    {
        // Log state
        Serial.println("Connect timeout reached.");

        // Turn LEDs off
        for (int i = 0; i < NUM_OF_LEDS; ++i)
            leds[i] = CRGB::Black;
        FastLED.show();

        // End audio sink
        a2dpSink.end(true);

        // Restart ESP
        ESP.restart();
    }


    // Update LEDs
    float rmsDb  = log10f(audioRMS) * 20.0f;
    float rmsFlt = (rmsDb + AUD_FB_RANGE) / AUD_FB_RANGE;

    if (rmsFlt < 0.0f)
        rmsFlt = 0.0f;
    if (rmsFlt > 1.0f)
        rmsFlt = 1.0f;

    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i].setHSV(int(rmsFlt * 2.0f * 255) % 256, 255, rmsFlt * 255);

    FastLED.show();


    // Check bluetooth connection
    if (!a2dpSink.is_connected())
    {
        // Log state
        Serial.println("User disconnected.");

        // Play sound
        playSound(3);

        // Switch state
        mainState = WAIT_FOR_CONNECT;
    }
}

void setup()
{
    // Setup serial communication
    Serial.begin(115200);
    // delay(3000); // wait for console opening


    // Setup IR sensor and button
    pinMode(IR_SENS_PIN, INPUT_PULLUP);
    digitalWrite(IR_SENS_PIN, HIGH);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(BUTTON_PIN, HIGH);


    // Setup LEDs
    // FastLED.addLeds<WS2811, LED_DATA_PIN, RGB>(leds, NUM_OF_LEDS);
    // FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_OF_LEDS);  // GRB ordering is typical
    // FastLED.addLeds<WS2813, LED_DATA_PIN, RGB>(leds, NUM_OF_LEDS);
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_OF_LEDS); // GRB ordering is typical
    FastLED.setBrightness(BRIGHTNESS);
    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i] = CRGB::Black;
    FastLED.show();


    // Setup RTC
    i2c.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000ul);

    if (!rtc.begin(&i2c))
    {
        Serial.println("Couldn't find RTC");
        while (true)
            ;
    }

    if (rtc.lostPower())
    {
        // Serial.println("RTC lost power, lets set the time!");
        // rtc.adjust(DateTime(2022, 9, 10, 20, 48, 0)); // Change this to current time + 40 sec upload time
    }


    // Setup audio sink
    i2s_pin_config_t i2sPinConfig = {.bck_io_num   = I2S_BCK_PIN,
                                     .ws_io_num    = I2S_LRCK_PIN,
                                     .data_out_num = I2S_DATA_PIN,
                                     .data_in_num  = I2S_PIN_NO_CHANGE};

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

    a2dpSink.set_pin_config(i2sPinConfig);
    a2dpSink.set_i2s_config(i2sConfig);
    a2dpSink.set_stream_reader(readDataStream);
    a2dpSink.init_i2s_only();


    // Generate sounds
    generateSounds();

    // Update LEDs
    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i] = CRGB::Red;
    FastLED.show();

    // Log state
    Serial.println("Trashbox ready for trash.");

    // Play sound
    playSound(3);

    // Wait a second
    delay(1000);

    // Set initial state
    mainState = WAIT_FOR_TRASH;
}

void loop()
{
    // Get current time from RTC
    DateTime now = rtc.now();

    // Set volume divider depending on day or night
    uint8_t hour        = now.hour();
    bool    isItDaytime = hour >= DAY_START_HR && hour < DAY_END_HR;
    a2dpSink.set_volume_divider(isItDaytime ? 1 : 2);

    // Switch through main states
    uint32_t time = now.secondstime();

    switch (mainState)
    {
        default:
            waitForTrash(time);
            break;
        case WAIT_FOR_TRASH:
            waitForTrash(time);
            break;
        case WAIT_FOR_BUTTON:
            waitForButton(time);
            break;
        case WAIT_FOR_CONNECT:
            waitForConnect(time);
            break;
        case WAIT_FOR_DISCONNECT:
            waitForDisconnect(time);
            break;
    }

    // Delay a little bit
    delay(25);
}
