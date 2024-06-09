#include <cstdint>
#include <driver/gpio.h>
#include <esp_log.h>
#include <mutex>

#include "RBControl_encoder.hpp"
#include "RBControl_manager.hpp"
#include "RBControl_pinout.hpp"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

#define TAG "RbEncoder"

#define ENC_COUNT static_cast<int>(MotorId::MAX)
#define PCNT_H_LIM_VAL 1
#define PCNT_L_LIM_VAL -1
#define INC_PER_REVOLUTION 2 //PCNT increments per 1 engine revolution
#define ESP_INTR_FLAG_DEFAULT 0
#define ENC_DEBOUNCE_US 20 //[microseconds]
#define MAX_ENGINE_PERIOD_US 100000 //engine period limit separating zero result [us]
#define MIN_ENGINE_PERIOD_US 1000 //engine period limit separating zero results [us]

namespace rb {

const gpio_num_t ENCODER_PINS[ENC_COUNT * 2] = {
    ENC1A,
    ENC1B,
    ENC2A,
    ENC2B,
    ENC3A,
    ENC3B,
    ENC4A,
    ENC4B,
    ENC5A,
    ENC5B,
    ENC6A,
    ENC6B,
    ENC7A,
    ENC7B,
    ENC8A,
    ENC8B,
};

bool IRAM_ATTR Encoder::isrPcnt(pcnt_unit_handle_t, const pcnt_watch_event_data_t* event_data, void* cookie) {
    Encoder* enc = (Encoder*)cookie;
    const Manager::Event ev = {
        .type = Manager::EVENT_ENCODER_PCNT,
        .data = {
            .encoderPcnt = {
                .id = enc->m_id,
                .watchpoint = event_data->watch_point_value,
                .timestamp = esp_timer_get_time(),
            },
        },
    };

    if (enc->m_manager.queueFromIsr(&ev)) {
        portYIELD_FROM_ISR();
    }

    return true;
}

Encoder::Encoder(rb::Manager& man, rb::MotorId id)
    : m_manager(man)
    , m_id(id) {
    if (m_id >= MotorId::MAX) {
        ESP_LOGE(TAG, "Invalid encoder index %d, using 0 instead.", (int)m_id);
        m_id = MotorId::M1;
    }

    m_counter = 0;
    m_counter_time_us_last = esp_timer_get_time();
    m_counter_time_us_diff = 0;

    m_target_direction = 0;
    m_target_callback = NULL;
    m_target = 0;
}

Encoder::~Encoder() {
}

void Encoder::install() {
    const gpio_num_t encA = ENCODER_PINS[static_cast<int>(m_id) * 2];
    const gpio_num_t encB = ENCODER_PINS[static_cast<int>(m_id) * 2 + 1];

    pcnt_unit_config_t unit_config = {
        .low_limit = PCNT_L_LIM_VAL,
        .high_limit = PCNT_H_LIM_VAL,
        .flags = {},
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &m_pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(m_pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = encA,
        .level_gpio_num = encB,
        .flags = {},
    };
    ESP_ERROR_CHECK(pcnt_new_channel(m_pcnt_unit, &chan_a_config, &m_channel_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = encB,
        .level_gpio_num = encA,
        .flags = {},
    };
    ESP_ERROR_CHECK(pcnt_new_channel(m_pcnt_unit, &chan_b_config, &m_channel_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(m_channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(m_channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(m_channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(m_channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m_pcnt_unit, PCNT_H_LIM_VAL));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m_pcnt_unit, PCNT_L_LIM_VAL));

    pcnt_event_callbacks_t cbs = {
        .on_reach = Encoder::isrPcnt,
    };

    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(m_pcnt_unit, &cbs, this));

    ESP_ERROR_CHECK(pcnt_unit_enable(m_pcnt_unit));

    ESP_ERROR_CHECK(pcnt_unit_clear_count(m_pcnt_unit));

    ESP_ERROR_CHECK(pcnt_unit_start(m_pcnt_unit));
}

void Encoder::onPcntIsr(int watchpoint, int64_t timestamp) {
    std::function<void(Encoder&)> callback;

    m_time_mutex.lock();

    m_counter_time_us_diff = timestamp - m_counter_time_us_last;
    m_counter_time_us_last = timestamp;

    m_counter.fetch_add(watchpoint);

    if (m_target_direction != 0) {
        const auto val = value();
        if ((m_target_direction > 0 && val >= m_target) || (m_target_direction < 0 && val <= m_target)) {
            m_manager.setMotors().power(m_id, 0).set(true);
            m_target_direction = 0;
            callback = m_target_callback;
        }
    }

    m_time_mutex.unlock();

    if (callback)
        callback(*this);
}

int32_t Encoder::value() {
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(m_pcnt_unit, &count));
    return m_counter.load() + count;
}

void Encoder::reset() {
    m_counter = 0;
}

void Encoder::driveToValue(int32_t positionAbsolute, uint8_t power, std::function<void(Encoder&)> callback) {
    if (power == 0)
        return;

    ESP_LOGD(TAG, "driveToValue %ld %ld %d %p", positionAbsolute, this->value(), power, callback.target<void*>());

    const auto current = this->value();
    if (current == positionAbsolute)
        return;

    m_time_mutex.lock();
    if (m_target_direction != 0 && m_target_callback) {
        m_target_callback(*this);
    }
    m_target_callback = callback;
    m_target = positionAbsolute;
    m_target_direction = (positionAbsolute > current ? 1 : -1);
    m_manager.motor(m_id).power(static_cast<int8_t>(power) * m_target_direction);
    m_time_mutex.unlock();
}

void Encoder::drive(int32_t positionRelative, uint8_t power, std::function<void(Encoder&)> callback) {
    driveToValue(value() + positionRelative, power, callback);
}

};
