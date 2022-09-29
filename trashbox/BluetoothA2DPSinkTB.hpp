#pragma once

#include <BluetoothA2DPSink.h>

class BluetoothA2DPSinkTB : public BluetoothA2DPSink
{
public:
    size_t write_audio(const uint8_t * data, size_t size) override
    {
        return i2s_write_data(data, size);
    };

    void init_i2s_only()
    {
        init_i2s();

        ESP_LOGI(BT_AV_TAG, "i2s_start");
        if (i2s_start(i2s_port) != ESP_OK)
        {
            ESP_LOGE(BT_AV_TAG, "i2s_start");
        }
    };

    void start(const char * name, bool auto_reconnect) override
    {
        set_auto_reconnect(auto_reconnect, AUTOCONNECT_TRY_NUM);

        ESP_LOGD(BT_AV_TAG, "%s", __func__);
        log_free_heap();

        if (is_start_disabled)
        {
            ESP_LOGE(BT_AV_TAG, "re-start not supported after end(true)");
            return;
        }

        // store parameters
        if (name)
        {
            this->bt_name = name;
        }
        ESP_LOGI(BT_AV_TAG, "Device name will be set to '%s'", this->bt_name);

        // Initialize NVS
        init_nvs();

        // reconnect management
        is_autoreconnect_allowed = true; // allow automatic reconnect
        if (reconnect_status == AutoReconnect)
        {
            get_last_connection();
            memcpy(peer_bd_addr, last_connection, ESP_BD_ADDR_LEN);

            // force disconnect first so that a subsequent connect has a chance to work
#ifdef A2DP_EXPERIMENTAL_DISCONNECT_ON_START
            ESP_LOGW(BT_AV_TAG, "A2DP_EXPERIMENTAL_DISCONNECT_ON_START");
            disconnect();
#else
            // trigger timeout
            delay(reconnect_delay);
#endif
        }

        // setup i2s
        // init_i2s(); // Do not init i2s
        player_init = false; // reset player

        // setup bluetooth
        init_bluetooth();

        // create application task
        app_task_start_up();

        // Bluetooth device name, connection mode and profile set up
        app_work_dispatch(ccall_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0);

        // handle security pin
        if (is_pin_code_active)
        {
            // Set default parameters for Secure Simple Pairing
            esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
            esp_bt_io_cap_t   iocap      = ESP_BT_IO_CAP_IO;
            esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
            // invokes callbacks
            esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
            esp_bt_pin_code_t pin_code;
            esp_bt_gap_set_pin(pin_type, 0, pin_code);
        }
        else
        {
            // Set default parameters for Secure Simple Pairing
            esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
            esp_bt_io_cap_t   iocap      = ESP_BT_IO_CAP_NONE;
            esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
            // no callbacks
            esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
            esp_bt_pin_code_t pin_code;
            esp_bt_gap_set_pin(pin_type, 0, pin_code);
        }

        log_free_heap();
    };

    void end(bool release_memory = false) override
    {
        // reconnect should not work after end
        is_autoreconnect_allowed = false;
        BluetoothA2DPCommon::end(release_memory);

        // stop I2S
        if (is_i2s_output)
        {
            // Do not uninstall i2s
            // ESP_LOGI(BT_AV_TAG, "uninstall i2s");
            // if (i2s_driver_uninstall(i2s_port) != ESP_OK)
            // {
            //     ESP_LOGE(BT_AV_TAG, "Failed to uninstall i2s");
            // }
            // else
            // {
            player_init = false;
            // }
        }

        log_free_heap();
    };

    void handle_audio_state(uint16_t event, void * p_param) override
    {
        ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
        esp_a2d_cb_param_t * a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP audio state: %s", to_str(a2d->audio_stat.state));

        // callback on state change
        audio_state = a2d->audio_stat.state;
        if (audio_state_callback != nullptr)
        {
            audio_state_callback(a2d->audio_stat.state, audio_state_obj);
        }

        if (is_i2s_output)
        {
            if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state)
            {
                m_pkt_cnt = 0;
                // Do not start i2s
                // ESP_LOGI(BT_AV_TAG, "i2s_start");
                // if (i2s_start(i2s_port) != ESP_OK)
                // {
                //     ESP_LOGE(BT_AV_TAG, "i2s_start");
                // }
            }
            else if (ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND == a2d->audio_stat.state
                     || ESP_A2D_AUDIO_STATE_STOPPED == a2d->audio_stat.state)
            {
                // Do not stop i2s
                // ESP_LOGW(BT_AV_TAG, "i2s_stop");
                // i2s_stop(i2s_port);
                // i2s_zero_dma_buffer(i2s_port);
            }

            if (audio_state_callback_post != nullptr)
            {
                audio_state_callback_post(a2d->audio_stat.state, audio_state_obj_post);
            }
        }
    };
};
