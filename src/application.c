#include <application.h>

// How long should radio listen for messages after boot or a button press
#define INITIAL_LISTEN_INTERVAL (1 * 60 * 1000)

// How often send periodic meter updates
#define USAGE_REPORT_INTERVAL (30 * 60 * 1000)

// Defaults
#define BATTERY_UPDATE_INTERVAL (60 * 60 * 1000)
#define TEMPERATURE_MEASURE_INTERVAL_SECOND (30)
#define TEMPERATURE_PUBLISH_DELTA (1.0f)
#define TEMPERATURE_PUBLISH_TIMEOUT_SECOND (15 * 60)

// LED instance
twr_led_t led;

// Button instance
twr_button_t button;

// Event Counter
uint32_t last_counter = 0;
void pulse_counter_event_handler(twr_module_sensor_channel_t channel, twr_pulse_counter_event_t event, void *event_param)
{
    (void) channel;
    (void) event_param;

    if(event == TWR_PULSE_COUNTER_EVENT_UPDATE && channel == TWR_MODULE_SENSOR_CHANNEL_A) 
    {
        twr_log_info("Publishing meter counter update.");

        uint32_t current_counter = twr_pulse_counter_get(TWR_MODULE_SENSOR_CHANNEL_A);
        uint32_t relative_counter = current_counter - last_counter;

        // Send absolute usage
        float_t absolute_usage = current_counter / (float_t)100;
        twr_radio_pub_float("usage/-/total", &absolute_usage);

        // Send relative usage only if changed
        //if(relative_counter > 0)
        //{
        float_t relative_usage = relative_counter / (float_t)100;
        twr_radio_pub_float("usage/-/relative", &relative_usage);
        //}

        last_counter = current_counter;

        twr_led_pulse(&led, 200);
    }
}

// Thermometer
twr_tmp112_t tmp112;
float publish_temperature = NAN;
twr_tick_t temperature_publish_timeout = 0;

void tmp112_event_handler(twr_tmp112_t *self, twr_tmp112_event_t event, void *event_param)
{
    float temperature;

    if (event == TWR_TMP112_EVENT_UPDATE)
    {
        if (twr_tmp112_get_temperature_celsius(self, &temperature))
        {
            if ((fabsf(temperature - publish_temperature) >= TEMPERATURE_PUBLISH_DELTA) || (temperature_publish_timeout < twr_scheduler_get_spin_tick()))
            {
                twr_log_info("Publishing temperature update.");

                // Used same MQTT topic as for Temperature Tag with alternate i2c address
                twr_radio_pub_temperature(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &temperature);

                publish_temperature = temperature;

                temperature_publish_timeout = twr_tick_get() + (TEMPERATURE_PUBLISH_TIMEOUT_SECOND * 1000);
            }
        }
    }
}

// Battery
void battery_event_handler(twr_module_battery_event_t event, void *event_param)
{
    (void) event_param;

    float voltage;

    if (event == TWR_MODULE_BATTERY_EVENT_UPDATE)
    {
        if (twr_module_battery_get_voltage(&voltage))
        {
            twr_log_info("Publishing battery voltage update.");

            twr_radio_pub_battery(&voltage);
        }
    }
}

twr_scheduler_task_id_t listening_stopped_task_id;

void listening_stopped_handler(void* param) 
{
    twr_log_info("Listening for usage configuration stopped.");

    twr_led_set_mode(&led, TWR_LED_MODE_OFF);
}

void start_listening()
{
    twr_led_set_mode(&led, TWR_LED_MODE_ON);

    int listening_interval_seconds = INITIAL_LISTEN_INTERVAL/1000;

    twr_radio_listen(INITIAL_LISTEN_INTERVAL);
    twr_radio_pub_int("core/-/listening-timeout", &listening_interval_seconds);

    twr_log_info("Waiting for counter update for next %i seconds...", listening_interval_seconds);

    twr_scheduler_plan_from_now(listening_stopped_task_id, INITIAL_LISTEN_INTERVAL);
}

void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    if (event == TWR_BUTTON_EVENT_PRESS)
    {
        start_listening();
    }
}

void counter_set_handler_int(uint64_t *id, const char *topic, void *value, void *param)
{
    twr_log_info("Set counter triggered [int].");

    uint32_t new_counter = *(uint32_t *)value;
    float_t usage = new_counter / (float_t)100;

    twr_pulse_counter_set(TWR_MODULE_SENSOR_CHANNEL_A, new_counter);
    last_counter = new_counter;

    twr_radio_pub_float("usage/-/total", &usage);

    twr_log_info("Usage counter set to %li (%0.2f).", new_counter, usage);
}

void counter_set_handler_float(uint64_t *id, const char *topic, void *value, void *param)
{
    twr_log_info("Set counter triggered [float].");

    float_t usage = *(float_t*)value;
    uint32_t new_counter = usage * 100;

    twr_pulse_counter_set(TWR_MODULE_SENSOR_CHANNEL_A, new_counter);
    last_counter = new_counter;

    twr_radio_pub_float("usage/-/total", &usage);

    twr_log_info("Usage counter set to %li (%0.2f).", new_counter, usage);
}

// Subscribe to counter changes
twr_radio_sub_t subs[] = {
    {"usage/-/total/float", TWR_RADIO_SUB_PT_FLOAT, counter_set_handler_float, NULL},
    {"usage/-/total/set", TWR_RADIO_SUB_PT_INT, counter_set_handler_int, NULL},
};

void application_init(void)
{
    // Initialize logging
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, false);
    twr_led_set_mode(&led, TWR_LED_MODE_ON);

    // Initialize button
    twr_button_init(&button, TWR_GPIO_BUTTON, TWR_GPIO_PULL_DOWN, false);
    twr_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize battery
    twr_module_battery_init();
    twr_module_battery_set_event_handler(battery_event_handler, NULL);
    twr_module_battery_set_update_interval(BATTERY_UPDATE_INTERVAL);

    // Initialize thermometer sensor on core module
    twr_tmp112_init(&tmp112, TWR_I2C_I2C0, 0x49);
    twr_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);
    twr_tmp112_set_update_interval(&tmp112, TEMPERATURE_PUBLISH_TIMEOUT_SECOND);

    // Radio
    twr_radio_init(TWR_RADIO_MODE_NODE_SLEEPING);
    twr_radio_set_subs(subs, sizeof(subs)/sizeof(subs[0]));

    // Meter
    twr_pulse_counter_init(TWR_MODULE_SENSOR_CHANNEL_A, TWR_PULSE_COUNTER_EDGE_FALL);
	twr_pulse_counter_set_event_handler(TWR_MODULE_SENSOR_CHANNEL_A, pulse_counter_event_handler, NULL);
    twr_pulse_counter_set_update_interval(TWR_MODULE_SENSOR_CHANNEL_A, USAGE_REPORT_INTERVAL);

    // Register task to notify end of listening
    listening_stopped_task_id = twr_scheduler_register(listening_stopped_handler, NULL, TWR_TICK_INFINITY);

    twr_radio_pairing_request("gas-meter", FW_VERSION);

    // Wait for optional counter configuration
    start_listening();
}
