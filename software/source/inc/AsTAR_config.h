/* AsTAR++ Config file */


// Start ************************* Variables AsTAR++ *************************
// Note - all voltages are mV

#if defined(CONFIG_BATTERY)

    /* LiPo Battery Schedule Algorithm
    * Zones (by newV in millivolts):
    *   Zone 1: newV <= 3600 mV -> sleeptime = 60000 seconds
    *   Zone 2: 3600 < newV <= 4000 mV -> sleeptime = 1500 seconds
    *   Zone 3: 4000 < newV <= 4200 mV -> sleeptime = 300 seconds
    *   Above Zone 3: sleeptime = 300 seconds (max frequency)
    */
    // Thresholds (millivolts)
    #define ZONE1_THRESHOLD_MV 3900U
    #define ZONE2_THRESHOLD_MV 4000U
    #define ZONE3_THRESHOLD_MV 4200U
    
    // Sleep durations (seconds)
    #define SLEEP_ZONE1_SEC 3600U
    #define SLEEP_ZONE2_SEC 900U
    #define SLEEP_ZONE3_SEC 300U
 
    #define LowVolt_SleepTime 7200          // When new voltage < shutoff voltage, the node sleeps and wakes up in 720*10s =120 minutes

    #define shutOffVoltage 3900            // The voltage at which execution should be suspended.
#else
    // Battery-less setup
    #define LowVolt_SleepTime   7200        // When new voltage < shutoff voltage, the node sleeps and wakes up in 720*10s = 120 minutes

    #define maxRate             300         // The maximum execution rate (seconds, lower = faster)    - 1s
    #define minRate             7200        // The minimum execution rate (seconds, lower = faster)    - 120 minutes = 2 hours
    #define shutOffVoltage      3900        // The voltage at which execution should be suspended.
    #define maxVoltage          5500     

    #define wakeupThreshold_V   1.1         // x * shutOffVoltage
    #define sleepThreshold_V    1.1         // x * shutOffVoltage

    //Variables are only dedicated to NightOptimisation algorithms
    #define nighttimeMaxRate    300         // Outdoor nodes
    #define daytimeOptimumV     4000 

    #define nightVLossTimeThreshold 300     // In seconds, declare night if only voltage losses are recorded for this time period
    #define nightVRiseTimeThreshold 300     // In seconds, declare day if only V_solar rises are recorded for this time period
    #define weightingNewNightLength 30      // Percentage weighting for newly recorded night length to total night length
#endif


// Current lowest voltage (millivolts) is 3850 mV // below this value, the Adc is not working properly