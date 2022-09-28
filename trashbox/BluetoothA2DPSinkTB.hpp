#pragma once

#include <BluetoothA2DPSink.h>

class BluetoothA2DPSinkTB : public BluetoothA2DPSink
{
public:
    size_t write_audio(const uint8_t * data, size_t size) override
    {
        return i2s_write_data(data, size);
    }
};
