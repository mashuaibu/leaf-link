#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <math.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define SHUT_OFF_V  3900
#define LOW_V       4150
#define HIGH_V      5400    // Makes no difference for battery powered devices

#define BAT_MON_CHANNEL 20
#define ADC_PV_CHANNEL 3

double correction_ratio = 2;

#define SENSOR_POWER_SEL_NODE DT_ALIAS(SENSOR_PWR_SEL)
#define ADC_PV_EN_NODE DT_ALIAS(ADC_PV_EN)
#define OPENC_EN_NODE DT_ALIAS(OPENC_EN)

/* GPIO device and pin definitions */
static const struct gpio_dt_spec sensor_power_sel = GPIO_DT_SPEC_GET(DT_ALIAS(sensorpwrsel), gpios);
static const struct gpio_dt_spec adc_pv_en = GPIO_DT_SPEC_GET(DT_ALIAS(adcpven), gpios);
static const struct gpio_dt_spec openc_en = GPIO_DT_SPEC_GET(DT_ALIAS(opencen), gpios);

/* ADC device and channel configuration */
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

uint16_t buf;

struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf),
    .calibrate = true,
};

static uint16_t Vbat = 0;
static uint16_t Vpv = 0;

typedef enum {
    STATE_START,
    STATE_LOW_BATTERY,
    STATE_NORMAL,
    STATE_BATTERY_FULL
} power_state_t;

static power_state_t current_state = STATE_START;

#define STACK_SIZE_POWER 1024
#define PRIORITY 5

void update_measurement() {
    gpio_pin_set_dt(&openc_en, 0); // open circuit measurement
    //pm_device_runtime_get(adc_channels[0].dev);
    //pm_state_force(0, &(struct pm_state_info){PM_STATE_ACTIVE, 0, 0}); // Ensure active state
    k_sleep(K_MSEC(10)); // Allow ADC to wake up
    gpio_pin_configure_dt(&adc_pv_en, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&adc_pv_en, 1);
    k_sleep(K_MSEC(1000));

    int32_t bat_value = 0, pv_value = 0;
    int err;

    for (size_t i = 0; i < ARRAY_SIZE(adc_channels); i++) {
        err = adc_channel_setup_dt(&adc_channels[i]);
        if (err < 0) {
            LOG_ERR("Could not reconfigure ADC channel #%d (%d)", i, err);
            continue;
        }
        buf = 0;
        adc_sequence_init_dt(&adc_channels[i], &sequence);
        err = adc_read_dt(&adc_channels[i], &sequence);
        if (err < 0) {
            LOG_ERR("Could not read ADC channel #%d (%d)", i, err);
            continue;
        }

        if (i == 0) { bat_value = (int32_t)buf; }
        if (i == 1) { pv_value = (int32_t)buf; }
    }

    adc_raw_to_millivolts_dt(&adc_channels[0], &bat_value);
    adc_raw_to_millivolts_dt(&adc_channels[1], &pv_value);

    Vbat = (uint16_t) (correction_ratio * bat_value);
    Vpv = (uint16_t) (correction_ratio * pv_value);
    LOG_INF("V battery: %d", Vbat);
    LOG_INF("V PV: %d", Vpv);

    gpio_pin_set_dt(&adc_pv_en, 0);
    gpio_pin_configure(adc_pv_en.port, adc_pv_en.pin, GPIO_DISCONNECTED);
   // pm_device_runtime_put(adc_channels[0].dev);
   // openc is enabled/disabled after this depending on the state
}

void power_monitor_event() {
    while (true) {
       // pm_state_force(0, &(struct pm_state_info){PM_STATE_ACTIVE, 0, 0}); // Keep system awake
        update_measurement();

        switch (current_state) {
        case STATE_START:
            LOG_INF("STATE_START");
            gpio_pin_set_dt(&sensor_power_sel, 0);
            gpio_pin_set_dt(&openc_en, 1);
            current_state = (Vbat <= LOW_V) ? STATE_LOW_BATTERY : (Vbat > HIGH_V) ? STATE_BATTERY_FULL : STATE_NORMAL;
            //break;
            continue;
        case STATE_LOW_BATTERY:
            LOG_INF("STATE_LOW_BATTERY");
            gpio_pin_set_dt(&sensor_power_sel, 0);
            gpio_pin_set_dt(&openc_en, 1);
            if (Vbat > LOW_V) current_state = STATE_NORMAL;
            break;
        case STATE_NORMAL:
            LOG_INF("STATE_NORMAL");
            gpio_pin_set_dt(&sensor_power_sel, 1);
            gpio_pin_set_dt(&openc_en, 1);
            if (Vbat < SHUT_OFF_V) current_state = STATE_LOW_BATTERY;
            else if (Vbat > HIGH_V) current_state = STATE_BATTERY_FULL;
            break;
        case STATE_BATTERY_FULL:
            LOG_INF("STATE_BATTERY_FULL");
            gpio_pin_set_dt(&sensor_power_sel, 1);
            gpio_pin_set_dt(&openc_en, 0);
            if (Vbat < HIGH_V) current_state = STATE_NORMAL;
            break;
        default:
            LOG_ERR("Unknown state");
            break;
        }
        
       // pm_state_force(0, &(struct pm_state_info){PM_STATE_SUSPEND_TO_IDLE, 0, 0}); // Allow light sleep
        k_sleep(K_MINUTES(1));
    }
}

void power_monitor_thread(void *arg1, void *arg2, void *arg3) {
    if (!gpio_is_ready_dt(&sensor_power_sel) || !gpio_is_ready_dt(&adc_pv_en) || !gpio_is_ready_dt(&openc_en)) {
        LOG_ERR("GPIOs not ready!");
        return;
    }

    gpio_pin_configure_dt(&sensor_power_sel, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&adc_pv_en, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&openc_en, GPIO_OUTPUT_ACTIVE);

    gpio_pin_set_dt(&sensor_power_sel, 0);
    gpio_pin_set_dt(&adc_pv_en, 0);
    gpio_pin_set_dt(&openc_en, 0);
    k_msleep(1000);

    int err;
    for (size_t i = 0; i < ARRAY_SIZE(adc_channels); i++) {
        if (!adc_is_ready_dt(&adc_channels[i])) {
            LOG_ERR("ADC controller %s not ready", adc_channels[i].dev->name);
            return;
        }

        err = adc_channel_setup_dt(&adc_channels[i]);
        if (err < 0) {
            LOG_ERR("Could not setup ADC channel #%d (%d)", i, err);
            return;
        }
    }

    LOG_INF("Power Monitor initialized");
    power_monitor_event();
}