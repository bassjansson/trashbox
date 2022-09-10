#include "BluetoothA2DPSink.h"

#define I2C_SDA_PIN  21
#define I2C_SCL_PIN  22

#define I2S_BCK_PIN  19
#define I2S_DATA_PIN 23
#define I2S_LRCK_PIN 18 // WS

#define BL_NAME      "Trashbox"

BluetoothA2DPSink a2dpSink;

void setup()
{
    i2s_pin_config_t i2sPinConfig = {.bck_io_num   = I2S_BCK_PIN,
                                     .ws_io_num    = I2S_LRCK_PIN,
                                     .data_out_num = I2S_DATA_PIN,
                                     .data_in_num  = I2S_PIN_NO_CHANGE};
    a2dpSink.set_pin_config(i2sPinConfig);
}

void loop()
{
    a2dpSink.start(BL_NAME);
    delay(30000);
    a2dpSink.end();
    delay(30000);
}
