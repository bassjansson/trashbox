#include <RTClib.h>
#include <FastLED.h>
#include <SPIFFS.h>

#include "BluetoothA2DPSinkTB.hpp"

// This code is used on a LilyGO TTGO T8 ESP32 board.
// The pins which are available on this board are:
// 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
// Do not use any of the other pins!

#define I2C_SDA_PIN  21 // default
#define I2C_SCL_PIN  22 // default

#define I2S_BCK_PIN  19
#define I2S_DATA_PIN 23
#define I2S_LRCK_PIN 18 // WS

#define LED_DATA_PIN 27
#define BUTTON_PIN   26

#define IR_SENS1_PIN 25
// #define IR_SENS2_PIN 33 // uncomment to use, works immediately
// #define IR_SENS3_PIN 32 // uncomment to use, works immediately

#define NUM_OF_LEDS  30
#define BRIGHTNESS   128 // 0 to 255

#define BLOCK_SIZE   1024 // 4096, 2 channel 16-bit, somehow constant
#define AUTO_CONNECT false

#define BTTN_TIMEOUT (40)      // seconds
#define CONN_TIMEOUT (80)      // seconds
#define PLAY_TIMEOUT (60 * 10) // seconds

#define INIT_VOLUME  80 // percent

#define BLEEP_VOLUME 0.05f  // 0 to 1
#define BLEEP_FREQ   220.0f // Hertz
#define BLEEP_DUR    0.15f  // seconds

#define AUD_FB_LPF   0.6f  // 0 to 1
#define AUD_FB_RANGE 40.0f // dB

#define DAY_START_HR 8  // hours
#define DAY_END_HR   22 // hours

#define NUM_OF_SOUND 4

#define BL_NAME      "Trashbox"

TwoWire             i2c = TwoWire(0);
RTC_DS3231          rtc;
BluetoothA2DPSinkTB a2dpSink;

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
uint32_t playTimeout    = 0;

struct Counters
{
    uint32_t countStartUnixTime = 0;
    uint32_t programBootCounter = 0;

    uint32_t trashDetectedCounter = 0;
    uint32_t buttonPressedCounter = 0;

    uint32_t buttonTimeoutCounter  = 0;
    uint32_t connectTimeoutCounter = 0;
    uint32_t playTimeoutCounter    = 0;

    uint32_t btConnectedCounter    = 0;
    uint32_t btDisconnectedCounter = 0;
};

Counters counters;

bool isSpiffsMounted = false;

void readAndPrintCounters()
{
    if (!isSpiffsMounted)
        return;

    File file = SPIFFS.open("/counters.bin", FILE_READ);

    if (!file || file.isDirectory())
    {
        Serial.println("Failed to open counters file..");
        return;
    }

    if (!file.read((uint8_t *)(&counters), sizeof(counters)) == sizeof(counters))
    {
        Serial.println("Failed to read counters file..");
        file.close();
        return;
    }

    DateTime dt(counters.countStartUnixTime);

    Serial.println();
    Serial.print("<<< Counters since ");
    Serial.print(dt.day(), DEC);
    Serial.print('-');
    Serial.print(dt.month(), DEC);
    Serial.print('-');
    Serial.print(dt.year(), DEC);
    Serial.print(' ');
    Serial.print(dt.hour(), DEC);
    Serial.print(':');
    Serial.print(dt.minute(), DEC);
    Serial.print(':');
    Serial.print(dt.second(), DEC);
    Serial.println(" >>>");
    Serial.print("           Program boots:  ");
    Serial.println(counters.programBootCounter);
    Serial.print("        Trash detections:  ");
    Serial.println(counters.trashDetectedCounter);
    Serial.print("          Button presses:  ");
    Serial.println(counters.buttonPressedCounter);
    Serial.print("         Button timeouts:  ");
    Serial.println(counters.buttonTimeoutCounter);
    Serial.print("        Connect timeouts:  ");
    Serial.println(counters.connectTimeoutCounter);
    Serial.print("           Play timeouts:  ");
    Serial.println(counters.playTimeoutCounter);
    Serial.print("      Bluetooth connects:  ");
    Serial.println(counters.btConnectedCounter);
    Serial.print("   Bluetooth disconnects:  ");
    Serial.println(counters.btDisconnectedCounter);
    Serial.println();

    file.close();
}

void writeCounters()
{
    if (!isSpiffsMounted)
        return;

    File file = SPIFFS.open("/counters.bin", FILE_WRITE);

    if (!file || file.isDirectory())
    {
        Serial.println("Failed to open counters file..");
        return;
    }

    if (!file.write((uint8_t *)(&counters), sizeof(counters)) == sizeof(counters))
    {
        Serial.println("Failed to write counters file..");
        file.close();
        return;
    }

    file.close();
}

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

    rms = log10f(sqrtf(rms / sampleCount) + 0.0001f) * 20.0f;
    rms = (rms + AUD_FB_RANGE) / AUD_FB_RANGE;

    if (rms < 0.0f)
        rms = 0.0f;
    if (rms > 1.0f)
        rms = 1.0f;

    audioRMS = audioRMS * AUD_FB_LPF + (1.0f - AUD_FB_LPF) * rms;

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
    if (leds[0].r > 2)
        leds[0].r -= 3;
    else
        leds[0].r = 0;

    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i].setRGB(leds[0].r, leds[0].r / 8.0f, 0);

    FastLED.show();


    // Read trash sensors
    bool isTrashDetected = digitalRead(IR_SENS1_PIN) == LOW;

#ifdef IR_SENS2_PIN
    isTrashDetected = isTrashDetected || digitalRead(IR_SENS2_PIN) == LOW;
#endif

#ifdef IR_SENS3_PIN
    isTrashDetected = isTrashDetected || digitalRead(IR_SENS3_PIN) == LOW;
#endif


    // Check if trash is detected
    if (isTrashDetected)
    {
        // Log state
        Serial.println("User threw in trash.");

        // Increment counter
        counters.trashDetectedCounter++;
        writeCounters();

        // Set timer
        buttonTimeout = time + BTTN_TIMEOUT;

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

        // Increment counter
        counters.buttonTimeoutCounter++;
        writeCounters();

        // Turn LEDs off
        for (int i = 0; i < NUM_OF_LEDS; ++i)
            leds[i] = CRGB::Black;
        FastLED.show();

        // Restart ESP
        ESP.restart();
    }


    // Update LEDs
    int   t = BLEEP_DUR * 6 * 2 * 1000;
    float c = sinf(millis() % t / (float)t * 2.0f * M_PI) * 0.5f + 0.5f;
    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i].setRGB(50, 50 + c * 205, 25);
    FastLED.show();


    // Read start button
    static int debounceCnt = 0;
    debounceCnt            = digitalRead(BUTTON_PIN) == LOW ? debounceCnt + 1 : 0;
    bool isButtonPressed   = debounceCnt >= 3;


    // Check if start button is pressed
    if (isButtonPressed)
    {
        // Log state
        Serial.println("User pressed button.");

        // Increment counter
        counters.buttonPressedCounter++;
        writeCounters();

        // Set timer
        connectTimeout = time + CONN_TIMEOUT;
        playTimeout    = time + PLAY_TIMEOUT;

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
    if (time > playTimeout || time > connectTimeout)
    {
        // Log state
        Serial.println(time > playTimeout ? "Play timeout reached." : "Connect timeout reached.");

        // Increment counter
        if (time > playTimeout)
            counters.playTimeoutCounter++;
        else
            counters.connectTimeoutCounter++;
        writeCounters();

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
    int   t = BLEEP_DUR * 6 * 1000;
    float c = sinf(millis() % t / (float)t * 2.0f * M_PI) * 0.5f + 0.5f;
    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i].setRGB(50, 25, 25 + c * 230);
    FastLED.show();


    // Check bluetooth connection
    if (a2dpSink.is_connected())
    {
        // Log state
        Serial.println("User connected.");

        // Increment counter
        counters.btConnectedCounter++;
        writeCounters();

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
    if (time > playTimeout)
    {
        // Log state
        Serial.println("Play timeout reached.");

        // Increment counter
        counters.playTimeoutCounter++;
        writeCounters();

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
        leds[i].setHSV(int(audioRMS * 1.75f * 255 + millis() / 250.0f) % 256, 255, audioRMS * 200 + 55);
    FastLED.show();


    // Check bluetooth connection
    if (!a2dpSink.is_connected())
    {
        // Log state
        Serial.println("User disconnected.");

        // Increment counter
        counters.btDisconnectedCounter++;
        writeCounters();

        // Set timer
        connectTimeout = time + CONN_TIMEOUT;

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


    // Setup SPIFFS
    if (SPIFFS.begin(true))
        isSpiffsMounted = true;
    else
        Serial.println("Failed to mount SPIFFS..");


    // Setup sensors
    pinMode(IR_SENS1_PIN, INPUT_PULLUP);
    digitalWrite(IR_SENS1_PIN, HIGH);

#ifdef IR_SENS2_PIN
    pinMode(IR_SENS2_PIN, INPUT_PULLUP);
    digitalWrite(IR_SENS2_PIN, HIGH);
#endif

#ifdef IR_SENS3_PIN
    pinMode(IR_SENS3_PIN, INPUT_PULLUP);
    digitalWrite(IR_SENS3_PIN, HIGH);
#endif


    // Setup button
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


    // Read and print counters
    counters.countStartUnixTime = rtc.now().unixtime();
    readAndPrintCounters();

    // Generate sounds
    generateSounds();

    // Update LEDs
    for (int i = 0; i < NUM_OF_LEDS; ++i)
        leds[i].setRGB(255, 32, 0);
    FastLED.show();

    // Log state
    Serial.println("Trashbox ready for trash.");

    // Increment counter
    counters.programBootCounter++;
    writeCounters();

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
