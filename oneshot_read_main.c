/*

* SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD

*

* SPDX-License-Identifier: Apache-2.0

*/

#include <stdio.h>

#include <stdlib.h>

#include <string.h>

#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "soc/soc_caps.h"

#include "esp_log.h"

#include "esp_adc/adc_oneshot.h"

#include "esp_adc/adc_cali.h"

#include "esp_adc/adc_cali_scheme.h"

#include <math.h>



const static char *TAG = "MOISTURE_SENSOR";



/*---------------------------------------------------------------

ADC General Macros

---------------------------------------------------------------*/

//ADC1 Channels

#if CONFIG_IDF_TARGET_ESP32

#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_0

#else

#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_2

#endif



#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12

#define NO_OF_SAMPLES 5



static int adc_raw;

static int voltage;

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

static void example_adc_calibration_deinit(adc_cali_handle_t handle);



/**

* @brief Maps the raw ADC value to a moisture percentage using calibration coefficients.

* @param voltage_value The voltage reading from the sensor.

* @return The estimated moisture percentage (from 0 to 100).

*/

float moisture_map(float voltage_value)

{
// Coeficientes de tu análisis de regresión polinomial en Python.
const float a0 = 93.92003116191847;
const float a1 = 3.83561791e-02;
const float a2 = 4.28297943e-06;

// The formula for the second-degree polynomial regression: y = a0 + a1*x + a2*x^2

// where y is the moisture percentage and x is the voltage.

float moisture_percentage = a0 + a1 * voltage_value + a2 * (voltage_value * voltage_value);
// Ensure the percentage is within the 0-100% range

if (moisture_percentage < 9.93) {
moisture_percentage = 0;
}
if (moisture_percentage > 93.88) {
moisture_percentage = 100;
}
return moisture_percentage;

}



void app_main(void)

{

//-------------ADC1 Init---------------//

adc_oneshot_unit_handle_t adc1_handle;

adc_oneshot_unit_init_cfg_t init_config1 = {

.unit_id = ADC_UNIT_1,

};

ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));



//-------------ADC1 Config---------------//

adc_oneshot_chan_cfg_t config = {

.atten = EXAMPLE_ADC_ATTEN,

.bitwidth = ADC_BITWIDTH_DEFAULT,

};

ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));



//-------------ADC1 Calibration Init---------------//

adc_cali_handle_t adc1_cali_chan0_handle = NULL;

bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);



int adc_raw_sum = 0;

int voltage_sum = 0;

int adc_raw_avg = 0;

float voltage_avg = 0.0;

float moisture_percentage = 0.0;



while (1) {

adc_raw_sum = 0;

voltage_sum = 0;



// Take multiple samples and sum them for averaging

for (int i = 0; i < NO_OF_SAMPLES; i++) {

ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw));

adc_raw_sum += adc_raw;



if (do_calibration1_chan0) {

ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));

voltage_sum += voltage;

}

vTaskDelay(pdMS_TO_TICKS(50)); // Short delay between individual readings

}



// Calculate the average

adc_raw_avg = adc_raw_sum / NO_OF_SAMPLES;


if (do_calibration1_chan0) {

voltage_avg = (float)voltage_sum / NO_OF_SAMPLES;

}



// Calculate moisture percentage using the calibrated function

// Correcting the function call to use Volts instead of millivolts

moisture_percentage = moisture_map(voltage_avg / 1000.0);



// Print the average values and the calculated moisture percentage

ESP_LOGI(TAG, "ADC Raw Average: %d | Voltage Avg (V): %.2f | Moisture: %.2f%%", adc_raw_avg, voltage_avg/1000.0, moisture_percentage);



// Delay between each group of readings

vTaskDelay(pdMS_TO_TICKS(1000));

}



// Tear Down

ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

if (do_calibration1_chan0) {

example_adc_calibration_deinit(adc1_cali_chan0_handle);

}

}



/*---------------------------------------------------------------

ADC Calibration

---------------------------------------------------------------*/

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)

{

adc_cali_handle_t handle = NULL;

esp_err_t ret = ESP_FAIL;

bool calibrated = false;



#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

if (!calibrated) {

ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");

adc_cali_curve_fitting_config_t cali_config = {

.unit_id = unit,

.chan = channel,

.atten = atten,

.bitwidth = ADC_BITWIDTH_DEFAULT,

};

ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);

if (ret == ESP_OK) {

calibrated = true;

}

}

#endif



#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

if (!calibrated) {

ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");

adc_cali_line_fitting_config_t cali_config = {

.unit_id = unit,

.atten = atten,

.bitwidth = ADC_BITWIDTH_DEFAULT,

};

ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);

if (ret == ESP_OK) {

calibrated = true;

}

}

#endif



*out_handle = handle;

if (ret == ESP_OK) {

ESP_LOGI(TAG, "Calibration Success");

} else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {

ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");

} else {

ESP_LOGE(TAG, "Invalid arg or no memory");

}



return calibrated;

}



static void example_adc_calibration_deinit(adc_cali_handle_t handle)

{

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");

ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));



#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");

ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));

#endif

}