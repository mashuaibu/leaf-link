#include <soc.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <drivers/bme68x_sensor_api.h>
#include "bme68x.h"
#include "bme68x_iaq.h"
#include "lorawan.h"
#include "structures.h"
#include "powerMonitor.c"
#include "AsTAR_config.h"

static uint64_t last_status_send_time = 0;
atomic_t successful_send_count = ATOMIC_INIT(0);
atomic_t fail_send_count = ATOMIC_INIT(0);


#define TX_BUFFER_SIZE 51
#define PORT 0 //Port 0: TX packet acknowledgements
static uint8_t tx_buffer[TX_BUFFER_SIZE];

#define NVS_PARTITION         storage_partition
#define NVS_PARTITION_DEVICE  FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET  FIXED_PARTITION_OFFSET(NVS_PARTITION)
#define NVS_DEVNONCE_ID 1

static struct iaq_output iaq_output;

static const struct device *dev = DEVICE_DT_GET_ONE(bosch_bme68x_sensor_api);
static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

#define ADC_RESOLUTION 12
#define ADC_CHANNEL_ID 2
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT

/* Event handling */
enum lorawan_event {
    LORAWAN_EVENT_JOIN,
    LORAWAN_EVENT_JOIN_SUCCESS,
    LORAWAN_EVENT_JOIN_FAILURE,
    LORAWAN_EVENT_TX_DONE,
    LORAWAN_EVENT_TX_FAILED,
    LORAWAN_EVENT_RX_RECEIVED,
    LORAWAN_EVENT_SENT_DATA
};
static volatile enum lorawan_event event;

// Start ************************* Variables AsTAR++ *************************
// Note - all voltages are mV
static uint32_t sleepTimer = 300;                   // Sleep Timer holds the time to sleep 3600(s) = 1h
static uint16_t oldV = 0;                           // Last measured voltage
static uint16_t newV = 0;                           // Current measured voltage
static int16_t deltaV = 0;                          // newV - oldV
static uint16_t optimumV = 4000;                    // Target capacitor voltage

static uint16_t solarV = 0;                                // To hold V_solar reading
static uint16_t beginSleeping_Vcap = 0;                    // Capacitor Voltage when the node begin sleeping


//Variables are only dedicated to NightOptimisation algorithms
static uint16_t nighttimeVSwing = 0;
static bool nighttimeFlag = false;

uint32_t nightDurationRollingEstimate =  50400;         // In seconds, starting at 14 hours night length (accurate start point for 26 May)
uint32_t timeSinceSunset = 0;                           // In seconds
uint32_t timeSinceSunrise = 0;

bool Flag_JustWakeup = 0;                               // Flag indicates that the node just wakes up - When the node just wakes up, the "Sleep_timer" is very high => this can help mitagate issue
// END ************************* Variables AsTAR++ *************************

/* Function declarations */
static void lorawan_event_handler(void *arg1, void *arg2, void *arg3);
static void send_lora_data(void);
static int climate_data(void);
static int status_data(void);
static void lorawan_datarate_changed(enum lorawan_datarate dr);
static void lorawan_dl_callback(uint8_t port, bool data_pending, int16_t rssi, int8_t snr, uint8_t len, const uint8_t *hex_data);
static void iaq_output_handler(const struct bme68x_iaq_sample *iaq_sample);
// Declare Functions Using in AsTAR++
void setSuspensionHandler();
void Schedule(void);


//2. Create a Dedicated Thread
#define EVENT_HANDLER_STACK_SIZE 6144
#define BME680_STACK_SIZE 8192
#define EVENT_HANDLER_PRIORITY 10


static K_THREAD_STACK_DEFINE(lorawan_event_stack, EVENT_HANDLER_STACK_SIZE);
static struct k_thread lorawan_event_thread_data;
static K_THREAD_STACK_DEFINE(bme680_stack, BME680_STACK_SIZE);
static struct k_thread bme680_thread_data;
static K_THREAD_STACK_DEFINE(power_stack, STACK_SIZE_POWER);
static struct k_thread power_monitor_tid;


static uint8_t dev_eui[] = LORAWAN_DEV_EUI;
static uint8_t join_eui[] = LORAWAN_JOIN_EUI;
static uint8_t app_key[] = LORAWAN_APP_KEY;
static struct lorawan_join_config join_cfg = {
    .mode = LORAWAN_ACT_OTAA,
    .dev_eui = dev_eui,
    .otaa = {
        .join_eui = join_eui,
        .app_key = app_key,
        .nwk_key = app_key,
        //.dev_nonce = 0
    }
};
static struct lorawan_downlink_cb downlink_cb = {
	.port = LW_RECV_PORT_ANY,
	.cb = lorawan_dl_callback
};

static void lorawan_dl_callback(uint8_t port, bool data_pending,
			int16_t rssi, int8_t snr,
			uint8_t len, const uint8_t *hex_data)
{
	LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm", port, data_pending, rssi, snr);
	if (hex_data) {
		LOG_HEXDUMP_INF(hex_data, len, "Payload: ");
	}
}

static void lorawan_datarate_changed(enum lorawan_datarate dr)
{
	uint8_t unused, max_size;

	lorawan_get_payload_sizes(&unused, &max_size);
	LOG_INF("New Datarate: DR_%d, Max Payload %d", dr, max_size);
}


/* Handle LoRaWAN join response and events */
static void lorawan_event_handler(void *arg1, void *arg2, void *arg3) {
    int ret;
    while (1)
    {
        switch (event) {
            case LORAWAN_EVENT_JOIN:
                ret = 0;
                LOG_INF("Attempting to join the network");
                lorawan_set_region(LORAWAN_REGION_EU868);
                //lorawan_set_datarate(LORAWAN_DR_0);
                lorawan_enable_adr(true);
                lorawan_set_conf_msg_tries(3);
                lorawan_set_class(LORAWAN_CLASS_A);
                lorawan_register_downlink_callback(&downlink_cb);
                lorawan_register_dr_changed_callback(lorawan_datarate_changed);
                /* Start LoRaWAN stack */
                if (lorawan_start() < 0) {
                    k_sleep(K_MSEC(5000));
                    //lorawan_event_handler(LORAWAN_EVENT_JOIN);
                    break;
                }
                do {
                ret = lorawan_join(&join_cfg);
                    if (ret < 0) {
                        LOG_ERR("lorawan_join to network failed: %d", ret);
                        k_sleep(K_MSEC(5000));
                    }
                } while (ret != 0);
                //lorawan_event_handler(LORAWAN_EVENT_JOIN_SUCCESS);
                event = LORAWAN_EVENT_JOIN_SUCCESS;
                break;

            case LORAWAN_EVENT_JOIN_SUCCESS:
                LOG_INF("Join successful!");
                event = LORAWAN_EVENT_SENT_DATA;
                k_sleep(K_SECONDS(10));
                break;

            case LORAWAN_EVENT_TX_DONE:
                LOG_INF("LoRaWAN TX completed successfully.");
                newV = Vbat;
                solarV = Vpv;
                while (newV <= shutOffVoltage) { 
                    sleepTimer = LowVolt_SleepTime;
                    k_sleep(K_SECONDS(sleepTimer));
                    newV = Vbat;
                    solarV = Vpv;
                    oldV = newV;
                }  
                Schedule(); 
                LOG_INF("DEBUG: newV=%d, solarV=%d, beginSleeping_Vcap=%d, nighttimeVSwing=%d, timeSinceSunset=%d, nightDurationRollingEstimate=%d, optimumV=%d, sleepTimer=%d", 
                  newV, solarV, beginSleeping_Vcap, nighttimeVSwing, timeSinceSunset, nightDurationRollingEstimate, optimumV, sleepTimer);      
                event = LORAWAN_EVENT_SENT_DATA;
                k_sleep(K_SECONDS(sleepTimer));
                oldV = newV;
                break;

            case LORAWAN_EVENT_TX_FAILED:
                LOG_ERR("LoRaWAN TX failed. Retrying next time");
                event = LORAWAN_EVENT_TX_DONE;
                break;

            case LORAWAN_EVENT_RX_RECEIVED:
                LOG_INF("Downlink message received.");
                break;
            
            case LORAWAN_EVENT_SENT_DATA:
                send_lora_data();
                break;

            default:
                LOG_ERR("Unknown LoRaWAN event");
                break;
        }
    }
}

static void send_lora_data(void) {
  int ret;

  uint64_t current_time = k_uptime_get();
  uint64_t three_hours_ms = 3UL * 60UL * 60UL * 1000UL; // Define three hours in milliseconds

  // Send every 3 hours a status update
  int packet_len = 0;
  if ((current_time - last_status_send_time) >= three_hours_ms) {
    packet_len = status_data();
    last_status_send_time = current_time;
  } else {
    packet_len = climate_data();
  }

  LOG_INF("Payload: %s, Length: %d", tx_buffer, packet_len);

  if (packet_len < 0 || packet_len >= TX_BUFFER_SIZE) {
      LOG_ERR("Failed to format payload");
      event = LORAWAN_EVENT_TX_FAILED;
      atomic_inc(&fail_send_count);
      return;
  }

  ret = lorawan_send(15, tx_buffer, packet_len, LORAWAN_MSG_CONFIRMED);
  if (ret < 0) {
      LOG_ERR("lorawan_send() failed with error: %d", ret);
      event = LORAWAN_EVENT_TX_FAILED;
      atomic_inc(&fail_send_count);
      return;
  }
  event = LORAWAN_EVENT_TX_DONE;
  atomic_inc(&successful_send_count);
  return;
}

static int climate_data(void){
  uint16_t task_rate_min = (uint16_t) (sleepTimer / 60);
  int packet_len = snprintf((char *)tx_buffer, TX_BUFFER_SIZE,
                            "%d, %d.%02u, %d.%02u, %d, %d, %d, %d, %d",
                            Vbat,
                            iaq_output.temperature.q, iaq_output.temperature.r,
                            iaq_output.humidity.q, iaq_output.humidity.r,
                            iaq_output.raw_pressure.q,
                            iaq_output.raw_gas_res.q,
                            task_rate_min,
                            Vpv,
                            optimumV);
  return packet_len;
}

// success send, failed send, up-time
static int status_data(void){
  uint64_t uptime_ms = k_uptime_get();
  uint32_t successfull = atomic_get(&successful_send_count);
  uint32_t failed = atomic_get(&fail_send_count);

  int packet_len = snprintf((char *)tx_buffer, TX_BUFFER_SIZE,
                            "%d, %d, %d",
                            successfull,
                            failed,
                            (int) uptime_ms);
  return packet_len;
}


static inline void fixed_point_init(float const x, size_t const precision,
				    struct fixed_point *fixed_point)
{
	int32_t x_scaled = x * precision;
	fixed_point->q = x_scaled / precision;
	if (fixed_point->q < 0) {
		fixed_point->r = (fixed_point->q * precision) - x_scaled;
	} else {
		fixed_point->r = x_scaled - (fixed_point->q * precision);
	}
}

static void iaq_output_init(struct bme68x_iaq_sample const *iaq_sample,
			    struct iaq_output *iaq_output)
{
	/* degC to degC, centidegrees precision. */
	fixed_point_init(iaq_sample->raw_temperature, 100, &iaq_output->raw_temperature);
	fixed_point_init(iaq_sample->temperature, 100, &iaq_output->temperature);
	/* Pa to kPa, Pa precision */
	fixed_point_init(iaq_sample->raw_pressure / 100.0f, 1000, &iaq_output->raw_pressure);
	/* percent to percent, centipercent precision. */
	fixed_point_init(iaq_sample->raw_humidity, 100, &iaq_output->raw_humidity);
	fixed_point_init(iaq_sample->humidity, 100, &iaq_output->humidity);
	/* Ohm to kOhm, Ohm precision. */
	fixed_point_init(iaq_sample->raw_gas_res / 1000.0f, 1000, &iaq_output->raw_gas_res);
	/* IAQ scaled to [0,500]. */
	iaq_output->iaq = (uint16_t)iaq_sample->iaq;
	iaq_output->iaq_accuracy = iaq_sample->iaq_accuracy;
	/* Unscaled IAQ, range unknown. */
	//iaq_output->static_iaq = (uint32_t)iaq_sample->static_iaq;
	/* ppm. */
	iaq_output->co2_equivalent = (uint32_t)iaq_sample->co2_equivalent;
	iaq_output->co2_accuracy = iaq_sample->co2_accuracy;
	/* ppm, 1/100 ppm precision. */
	fixed_point_init(iaq_sample->voc_equivalent, 100, &iaq_output->voc_equivalent);
	iaq_output->voc_accuracy = iaq_sample->voc_accuracy;
	iaq_output->stab_status = iaq_sample->stab_status;
	iaq_output->run_status = iaq_sample->run_status;
}

/* Initialize and run the BME680 IAQ loop */
static void bme680_thread(void *arg1, void *arg2, void *arg3) {
    while (1)
    {
        struct bme68x_dev bme68x_dev = {0};
        int ret = bme68x_sensor_api_init(dev, &bme68x_dev);
        if (ret == 0) {
            ret = bme68x_init(&bme68x_dev);
        }

        if (ret != 0) {
            LOG_ERR("BME680 initialization failed: %d", ret);
            return;
        }

        ret = bme68x_iaq_init();
        if (ret != 0) {
            LOG_ERR("BME680 IAQ initialization failed: %d", ret);
            return;
        }
        bme68x_iaq_run(&bme68x_dev, iaq_output_handler); // ends if something goes wrong or sensor disconnect
        k_sleep(K_MINUTES(5)); //sleep until next startup
        while (current_state == STATE_LOW_BATTERY)
        {
          k_sleep(K_MINUTES(5));
        }
    }
}


#if defined(CONFIG_BATTERY)
  // Time until next transmission if BATTERY Powered
  // sleepTimer defines the sleeping time

  void Schedule(void) {
    if (newV <= ZONE1_THRESHOLD_MV) {
        sleepTimer = SLEEP_ZONE1_SEC;
    }
    else if (newV <= ZONE2_THRESHOLD_MV) {
        sleepTimer = SLEEP_ZONE2_SEC;
    }
    else if (newV <= ZONE3_THRESHOLD_MV) {
        sleepTimer = SLEEP_ZONE3_SEC;
    }
    else {
        // Above maximum expected voltage, maintain fastest schedule
        sleepTimer = SLEEP_ZONE3_SEC;
    }
  }
#else
// Time until next transmission if BATTERY-LESS Powered
  void Schedule(void) {
  uint32_t sleepDelta;
  deltaV = newV-oldV;

  // When the node wakes up?
  if (nighttimeFlag) {
    if (solarV >= wakeupThreshold_V * shutOffVoltage)
      timeSinceSunrise +=sleepTimer;
    else timeSinceSunrise = 0;
    if(timeSinceSunrise >= nightVRiseTimeThreshold)
    {  
      if (timeSinceSunset >= 14400) { // Threshold of 4 hours to count towards night estimate
        nightDurationRollingEstimate = (((100 - weightingNewNightLength) * nightDurationRollingEstimate)/100) + ((weightingNewNightLength * timeSinceSunset)/100);
      }
      nighttimeFlag = false;
      optimumV = daytimeOptimumV;
      timeSinceSunset = 0;
      Flag_JustWakeup = 1;
    }
  } else{
    timeSinceSunset = 0;
  } 
 
  // When the node sleeps?
  if (solarV < sleepThreshold_V * shutOffVoltage){
    timeSinceSunset += sleepTimer;
    if (timeSinceSunset >= nightVLossTimeThreshold) {
      if (!nighttimeFlag) 
      {
        sleepTimer = 300;
        beginSleeping_Vcap = newV;
        nighttimeVSwing = beginSleeping_Vcap - (1.1*shutOffVoltage);
      }
      nighttimeFlag = true;
    }
  }

  if(nighttimeFlag) {
    // optimumV = daytimeOptimumV - ((nighttimeVSwing * timeSinceSunset)/nightDurationRollingEstimate);
    if(beginSleeping_Vcap > ((nighttimeVSwing * timeSinceSunset)/nightDurationRollingEstimate)){
      optimumV = beginSleeping_Vcap - ((nighttimeVSwing * timeSinceSunset)/nightDurationRollingEstimate);
    } else{
      optimumV = 4505;
    }
    if (optimumV < (1.1 * shutOffVoltage)) optimumV = (1.1 * shutOffVoltage);
  }
  // ****** End night optimisations ********


  // Start *************** Specify Kp ***************
  double kp1, kp2;
  kp1 = (newV - optimumV)/50;
  if (kp1 < 1.5) kp1 = 1.5;
  kp2 = (optimumV - newV)/50;
  if (kp2 < 1.5) kp2 = 1.5;
  // Stop *************** Specify Kp ***************

  if(newV >= maxVoltage)                                //state = "Very High";
  {  
    sleepTimer = maxRate;
  } 
  else if ((newV < maxVoltage) && (newV > optimumV))      //state = "high";  => "high" means: newV > optimumV
  {
    if (deltaV >= 0) sleepTimer = sleepTimer / kp1;
    if (deltaV < 10) {
      sleepDelta = sleepTimer / 10;
      if (sleepDelta < 1) sleepDelta = 1; // Account for rounding
      if (nighttimeFlag){
        sleepTimer -= sleepDelta;         // Track decreasing optimumV
      } else {
        sleepTimer += sleepDelta;
      }
    }

    if (sleepTimer < maxRate) sleepTimer = maxRate;
    if (nighttimeFlag && (sleepTimer < nighttimeMaxRate)) sleepTimer = nighttimeMaxRate;
    if (sleepTimer > minRate) sleepTimer = minRate;
  }
  else if (newV < optimumV)               //state = "low";                      // "Low" means: newV < optimumV
  {
    if (deltaV > 0) {
      sleepDelta = sleepTimer / 10;
      if (sleepDelta < 1) sleepDelta = 1; // Account for rounding
      sleepTimer -= sleepDelta;
    }
    if (deltaV <= 0) sleepTimer = sleepTimer * kp2; 
    if (sleepTimer < maxRate) sleepTimer = maxRate;
    if (nighttimeFlag && (sleepTimer < nighttimeMaxRate)) sleepTimer = nighttimeMaxRate;
    if (sleepTimer > minRate) sleepTimer = minRate;
  }
  else  {
    // When newV = optimumV => do nothing (New rate = old rate)
    // state = "optimum";      // optimum means: newV = optimumV
  }


  // ************************* Sleep_Timer when Node just wakes up *************************
  /**
   * When the node just wakes up, the "Sleep_timer" is very high => this can help mitagate this
   */
  if (Flag_JustWakeup)
  {
    sleepTimer = 1800;
    Flag_JustWakeup = 0;
  }
  // ************************* Sleep_Timer when Node just wake up *************************

  //*********** TEST *******************
  // sleepTimer = 30; //*************************//////////////////////////////////88888888888888888888887777777777777777777
}
#endif
static void iaq_output_handler(const struct bme68x_iaq_sample *iaq_sample) {
    iaq_output_init(iaq_sample, &iaq_output);
    LOG_INF("-- IAQ output --");
    LOG_INF("Temperature: %d.%02u degC", iaq_output.temperature.q, iaq_output.temperature.r);
    LOG_INF("Pressure: %d.%03u HPa", iaq_output.raw_pressure.q, iaq_output.raw_pressure.r);
    LOG_INF("Humidity: %d.%02u %%", iaq_output.raw_humidity.q, iaq_output.raw_humidity.r);
    LOG_INF("Gas resistance: %d.%03u kOhm", iaq_output.raw_gas_res.q, iaq_output.raw_gas_res.r);
    LOG_INF("IAQ: %u", iaq_output.iaq);
}


// ********** ************************* MAIN *********************************************

int main(void) {
  k_thread_create(&power_monitor_tid,power_stack, K_THREAD_STACK_SIZEOF(power_stack), power_monitor_thread, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
  LOG_INF("Initializing devices...");
	//int ret;
    if (!device_is_ready(dev) || !device_is_ready(lora_dev)) {
        LOG_ERR("One or more devices are not ready");
        return 0;
    }
    k_sleep(K_MINUTES(2)); // make sure batteryvoltage is measured
    // if the battery is below the shutOffVoltage, the sensor should keep sleeping, startup asks a high power consumption
    while (Vbat < shutOffVoltage)
    {
      k_sleep(K_SECONDS(LowVolt_SleepTime));
      /* Perform a cold reboot won't be able to join otherwise*/
      sys_reboot(SYS_REBOOT_COLD);
    }
    // Start the LoRaWAN event handler thread
    event = LORAWAN_EVENT_JOIN;
    k_thread_create(&bme680_thread_data, bme680_stack,
                    K_THREAD_STACK_SIZEOF(bme680_stack),
                    bme680_thread,
                    NULL, NULL, NULL,
                    EVENT_HANDLER_PRIORITY, 0, K_NO_WAIT);
    
    newV = Vbat;
    solarV = Vpv;
    k_thread_create(&lorawan_event_thread_data, lorawan_event_stack,
                    K_THREAD_STACK_SIZEOF(lorawan_event_stack),
                    lorawan_event_handler,
                    NULL, NULL, NULL,
                    EVENT_HANDLER_PRIORITY, 0, K_NO_WAIT);

    /* Main thread has done everything it should do*/
    k_sleep(K_FOREVER);
}
