/*
 * We'll print only integer values to not impose additional configuration
 * for supporting floats in format string specifiers.
 */
struct fixed_point {
	/* Integer part. */
	int32_t q;
	/* Fractional digits (variable precision). */
	uint32_t r;
};
struct iaq_output {
	/*
	 * Temperature measured by BME680/688 in degree Celsius.
	 * Precision 1/100 degC (2-digits remainder).
	 */
	struct fixed_point raw_temperature;
	/*
	 * Pressure measured by the BME680/688 in kPa.
	 * Precision 1 Pa (3-digits remainder).
	 */
	struct fixed_point raw_pressure;
	/*
	 * Relative directly measured by the BME680/688 in %.
	 * Precision 1/100 percent (2-digits remainder).
	 */
	struct fixed_point raw_humidity;
	/*
	 * Gas resistance measured by the BME680/688 in kOhm.
	 * Precision 1 Ohm (3-digits remainder).
	 */
	struct fixed_point raw_gas_res;
	/*
	 * Sensor heat compensated temperature in degrees Celsius.
	 * Precision 1/100 degC (2-digits remainder).
	 */
	struct fixed_point temperature;
	/*
	 * Sensor heat compensated relative humidity in %.
	 * Precision 1/100 percent (2-digits remainder).
	 */
	struct fixed_point humidity;
	/* Scaled IAQ [0,500]. */
	uint16_t iaq;
	enum bme68x_iaq_accuracy iaq_accuracy;
	/* Unscaled IAQ, range unknown. */
	uint32_t static_iaq;
	/* CO2 equivalent estimate in ppm. */
	uint32_t co2_equivalent;
	enum bme68x_iaq_accuracy co2_accuracy;
	/*
	 * VOC estimate in ppm.
	 * Precision 1/100 ppm (2-digits remainder).
	 */
	struct fixed_point voc_equivalent;
	enum bme68x_iaq_accuracy voc_accuracy;
	enum bme68x_iaq_status stab_status;
	enum bme68x_iaq_status run_status;
};
