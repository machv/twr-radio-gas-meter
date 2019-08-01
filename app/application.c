#include <application.h>

// How long should radio listen for messages after boot or a button press
#define INITIAL_LISTEN_INTERVAL (1 * 60 * 1000)

// How often send periodic meter updates
#define USAGE_REPORT_INTERVAL (5 * 60 * 1000)

// Defaults
#define BATTERY_UPDATE_INTERVAL (60 * 60 * 1000)
#define TEMPERATURE_MEASURE_INTERVAL_SECOND (30)
#define TEMPERATURE_PUBLISH_DELTA (1.0f)
#define TEMPERATURE_PUBLISH_TIMEOUT_SECOND (15 * 60)

// LED instance
bc_led_t led;

// Button instance
bc_button_t button;

// Event Counter
uint32_t last_counter = 0;
void pulse_counter_event_handler(bc_module_sensor_channel_t channel, bc_pulse_counter_event_t event, void *event_param)
{
    (void) channel;
    (void) event_param;

    if(event == BC_PULSE_COUNTER_EVENT_UPDATE && channel == BC_MODULE_SENSOR_CHANNEL_A) 
    {
        bc_log_info("Publishing meter counter update.", channel);

        uint32_t current_counter = bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_A);
        uint32_t relative_counter = current_counter - last_counter;

        // Send absolute usage
        float_t absolute_usage = current_counter / (float_t)100;
        bc_radio_pub_float("usage/-/total", &absolute_usage);

        // Send relative usage only if changed
        //if(relative_counter > 0)
        //{
        float_t relative_usage = relative_counter / (float_t)100;
        bc_radio_pub_float("usage/-/relative", &relative_usage);
        //}

        last_counter = current_counter;

        bc_led_pulse(&led, 200);
    }
}

// Thermometer
bc_tmp112_t tmp112;
float publish_temperature = NAN;
bc_tick_t temperature_publish_timeout = 0;

void tmp112_event_handler(bc_tmp112_t *self, bc_tmp112_event_t event, void *event_param)
{
    float temperature;

    if (event == BC_TMP112_EVENT_UPDATE)
    {
        if (bc_tmp112_get_temperature_celsius(self, &temperature))
        {
            if ((fabsf(temperature - publish_temperature) >= TEMPERATURE_PUBLISH_DELTA) || (temperature_publish_timeout < bc_scheduler_get_spin_tick()))
            {
                bc_log_info("Publishing temperature update.");

                // Used same MQTT topic as for Temperature Tag with alternate i2c address
                bc_radio_pub_temperature(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &temperature);

                publish_temperature = temperature;

                temperature_publish_timeout = bc_tick_get() + (TEMPERATURE_PUBLISH_TIMEOUT_SECOND * 1000);
            }
        }
    }
}

// Battery
void battery_event_handler(bc_module_battery_event_t event, void *event_param)
{
    (void) event_param;

    float voltage;

    if (event == BC_MODULE_BATTERY_EVENT_UPDATE)
    {
        if (bc_module_battery_get_voltage(&voltage))
        {
            bc_log_info("Publishing battery voltage update.");

            bc_radio_pub_battery(&voltage);
        }
    }
}

bc_scheduler_task_id_t listening_stopped_task_id;

void listening_stopped_handler(void* param) 
{
    bc_log_info("Listening for usage configuration stopped.");

    bc_led_set_mode(&led, BC_LED_MODE_OFF);
}

void start_listening()
{
    bc_led_set_mode(&led, BC_LED_MODE_ON);

    int listening_interval_seconds = INITIAL_LISTEN_INTERVAL/1000;

    bc_radio_listen(INITIAL_LISTEN_INTERVAL);
    bc_radio_pub_int("core/-/listening-timeout", &listening_interval_seconds);

    bc_log_info("Waiting for counter update for next %i seconds...", listening_interval_seconds);

    bc_scheduler_plan_from_now(listening_stopped_task_id, INITIAL_LISTEN_INTERVAL);
}

void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    if (event == BC_BUTTON_EVENT_PRESS)
    {
        start_listening();
    }
}

void counter_set_handler(uint64_t *id, const char *topic, void *value, void *param)
{
    bc_log_info("Set counter triggered [int].");

    uint32_t new_counter = *(uint32_t *)value;
    float_t usage = new_counter / (float_t)100;

    bc_pulse_counter_set(BC_MODULE_SENSOR_CHANNEL_A, new_counter);
    last_counter = new_counter;

    bc_radio_pub_float("usage/-/total", &usage);

    bc_log_info("Usage counter set to %i (%0.2f).", new_counter, usage);
}

void counter_set_handler_float(uint64_t *id, const char *topic, void *value, void *param)
{
    bc_log_info("Set counter triggered [float].");

    float_t usage = *(float_t*)value;
    uint32_t new_counter = usage * 100;

    bc_pulse_counter_set(BC_MODULE_SENSOR_CHANNEL_A, new_counter);
    last_counter = new_counter;

    bc_radio_pub_float("usage/-/total", &usage);

    bc_log_info("Usage counter set to %i (%0.2f).", new_counter, usage);
    
}

// Subscribe to counter changes
bc_radio_sub_t subs[] = {
    {"usage/-/total/set", BC_RADIO_SUB_PT_INT, counter_set_handler, NULL},
    {"usage/-/total/setfloat", BC_RADIO_SUB_PT_FLOAT, counter_set_handler_float, NULL},
};

void application_init(void)
{
    // Initialize logging
    bc_log_init(BC_LOG_LEVEL_DUMP, BC_LOG_TIMESTAMP_ABS);

    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_set_mode(&led, BC_LED_MODE_ON);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize battery
    bc_module_battery_init();
    bc_module_battery_set_event_handler(battery_event_handler, NULL);
    bc_module_battery_set_update_interval(BATTERY_UPDATE_INTERVAL);

    // Initialize thermometer sensor on core module
    bc_tmp112_init(&tmp112, BC_I2C_I2C0, 0x49);
    bc_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);
    bc_tmp112_set_update_interval(&tmp112, TEMPERATURE_PUBLISH_TIMEOUT_SECOND);

    // Radio
    bc_radio_init(BC_RADIO_MODE_NODE_SLEEPING);
    bc_radio_set_subs(subs, sizeof(subs)/sizeof(subs[0]));

    // Meter
    bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_A, BC_PULSE_COUNTER_EDGE_FALL);
	bc_pulse_counter_set_event_handler(BC_MODULE_SENSOR_CHANNEL_A, pulse_counter_event_handler, NULL);
    bc_pulse_counter_set_update_interval(BC_MODULE_SENSOR_CHANNEL_A, USAGE_REPORT_INTERVAL);

    // Register task to notify end of listening
    listening_stopped_task_id = bc_scheduler_register(listening_stopped_handler, NULL, BC_TICK_INFINITY);

    bc_radio_pairing_request("gas-meter", VERSION);

    // Wait for optional counter configuration
    start_listening();
}
