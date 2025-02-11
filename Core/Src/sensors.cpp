

#include "sensors.h"
#include "main.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>


//define PITCH_ENCODER_BITS 17
//#define PITCH_ENCODER_BITS 24


uint8_t pb1_value = 0;
uint8_t pb2_value = 0;
uint8_t pb1_update = 0;
uint8_t pb2_update = 0;

// Weather Station
uint8_t rx_buff[128];
uint8_t index_buff = 0;
uint8_t ws_receive_flag = 0;
uint8_t aRxBuffer[256];
//uint8_t ws_rx_byte;
uint8_t ws_rx_byte[4];


// RPM counters

uint32_t wheel_rpm_counter = 0;
uint32_t rotor_rpm_counter = 0;

float wind_dirs[4];
float wind_speeds[4];
int wind_dir_last_idx = 0;
int wind_speed_last_idx = 0;

float wind_direction_n180_0_p180(float wind_direction_0_360)
{
	float wind_direction_corrected = wind_direction_0_360;

	if (wind_direction_0_360 > 180 && wind_direction_0_360 < 360)
	{
		wind_direction_corrected = wind_direction_0_360 - 360;
	}
	return wind_direction_corrected;
}


#define END_OF_MOY_DIRECTION 50
uint16_t moy_direction_ctu = 0;
uint8_t new_moy_wind_direction = 0;
void ReadWeatherStation()
{
	if (!ws_receive_flag)
		return;
	ws_receive_flag = 0;

	static char frame_begin[] = "$IIMWV";

	__disable_irq();

	index_buff = 0;
	static uint8_t ws_message[128] = {0};
	memcpy(ws_message, rx_buff, sizeof(ws_message));

	__enable_irq();

	// HAL_UART_Transmit(&huart2, ws_message, strlen(ws_message), HAL_MAX_DELAY);

	if (strlen((char*)ws_message) >= 6)
	{
		char begin_frame[7]={0};
		memcpy(begin_frame, ws_message, 6);
		if (0 == strcmp(begin_frame, frame_begin))
		{
			char wind_dir_msg[6] = {0};
			char wind_speed_msg[6] = {0};

			memcpy(wind_dir_msg, &ws_message[7], 5);
			memcpy(wind_speed_msg, &ws_message[15], 5);

			float wind_dir = atof(wind_dir_msg);
			float wind_speed = atof(wind_speed_msg);

			// Wind speed to m/s
#define KNOTS_TO_MS 0.514444f
			wind_speed = KNOTS_TO_MS * wind_speed;

			// Wind direction correction from 0->360 to -180->180
			wind_dir = wind_direction_n180_0_p180(wind_dir);


			//FILTRAGE ET CALCULS MOYENNE
#define MOY_LENGTH 20
static float wind_dir_log[MOY_LENGTH] = {0};

			//log des dernières valeurs de direction du vent
			//on décale la liste pour scrap la valeur la plus vielle et insérer la nouvelle valeur
			if (moy_direction_ctu <= MOY_LENGTH) {
				for (int i = 0; i < MOY_LENGTH - 1; i++) {
					wind_dir_log[i+1] = wind_dir_log[i];
				}
				wind_dir_log[0] = wind_dir;
			}

			//calcul de la moyenne de direction du vent
			float moy_wind_direction = 0;
			for(int i = 0; i < MOY_LENGTH; i++) {
				moy_wind_direction += wind_dir_log[i];
			}

			sensor_data.wind_direction_avg = moy_wind_direction / MOY_LENGTH;


			//wind_dirs[wind_dir_last_idx++] = wind_dir;
			wind_speeds[wind_speed_last_idx++] = wind_speed;
			//if (wind_dir_last_idx >= 4) wind_dir_last_idx = 0;
			if (wind_speed_last_idx >= 4) wind_speed_last_idx = 0;
			// Do the averages
			//sensor_data.wind_direction_avg = (wind_dirs[0] + wind_dirs[1] + wind_dirs[2] + wind_dirs[3]) / 4.0f;
			sensor_data.wind_speed_avg = (wind_speeds[0] + wind_speeds[1] + wind_speeds[2] + wind_speeds[3]) / 4.0f;

			sensor_data.wind_direction = wind_dir;// - 120.0f;
			sensor_data.wind_speed = wind_speed;

			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		}
	}
}
/*
void get_wind_speed_dir(float* wind_dir) {

}
*/
void ReadTorqueLoadcellADC()
{
	ADC_ChannelConfTypeDef sConfigChannel8 = {0};
	sConfigChannel8.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfigChannel8.Channel = ADC_CHANNEL_8;
	sConfigChannel8.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfigChannel8) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t adc_torque = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);


	ADC_ChannelConfTypeDef sConfigChannel9 = {0};
	sConfigChannel9.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfigChannel9.Channel = ADC_CHANNEL_9;
	sConfigChannel9.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfigChannel9) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t adc_loadcell = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	static const float IAA_VDC_TO_ADC_V = 3.3f / 5.0f;
	static const float ADC_TO_TORQUE = IAA_VDC_TO_ADC_V * (5.0f/5.095f) * 160.0f / 4095.0f;
	// sensor_data.torque = (float)adc_torque * ADC_TO_TORQUE;
	// sensor_data.torque = (float)adc_torque;

	static const float ADC_TO_LOADCELL = IAA_VDC_TO_ADC_V * (5.0f/5.095f) * 500.0f / 4095.0f;
	// sensor_data.loadcell = (float)adc_loadcell * ADC_TO_LOADCELL;
	// sensor_data.loadcell = (float)adc_loadcell * ADC_TO_TORQUE;

	// sensor_data.loadcell = 0.0f;
	sensor_data.loadcell = (float)adc_torque * ADC_TO_LOADCELL;
	sensor_data.torque = (float)adc_loadcell * ADC_TO_TORQUE;
}

/*
float ReadTorqueADC()
{

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
    	Error_Handler();
    }


    HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t adc_result = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	static const float IAA_VDC_TO_ADC_V = 3.3f / 5.0f;
	static const float ADC_TO_TORQUE = IAA_VDC_TO_ADC_V * (5.0f/5.095f) * 160.0f / 4095.0f;
	return (float)adc_result * ADC_TO_TORQUE;
	// return adc_result;
}

float ReadLoadcellADC()
{

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 2;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}


	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t adc_result = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	static const float IAA_VDC_TO_ADC_V = 3.3f / 5.0f;
	static const float ADC_TO_LOADCELL = IAA_VDC_TO_ADC_V * (5.0f/5.095f) * 500.0f / 4095.0f;
	return (float)adc_result * ADC_TO_LOADCELL;
}
*/

void ReadWheelRPM() // 500ms interval
{
#define RPM_WHEEL_CNT_TIME_INVERSE 2.0f // Same as dividing by 500ms
#define WHEEL_CNT_PER_ROT 48.0f
		static const float wheel_counter_to_rpm_constant = (RPM_WHEEL_CNT_TIME_INVERSE / WHEEL_CNT_PER_ROT) * 60.0f;

		float wheel_rpm = (float)wheel_rpm_counter * wheel_counter_to_rpm_constant;

		wheel_rpm_counter = 0;
		sensor_data.wheel_rpm = wheel_rpm;
}

void CalcVehicleSpeed()
{
#define WHEEL_DIAMETER 6.2f;
	static const float wheel_rpm_to_speed = PI * WHEEL_DIAMETER;

	sensor_data.vehicle_speed = sensor_data.wheel_rpm * wheel_rpm_to_speed;
}

void ReadRotorRPM() // 100 ms interval
{
	// Process rpm counters
#define ROTOR_CNT_PER_ROT 360.0f
#define RPM_ROTOR_CNT_TIME_INVERSE 10.0f // Same as dividing by 100ms
		static const float rotor_counter_to_rpm_constant = (RPM_ROTOR_CNT_TIME_INVERSE / ROTOR_CNT_PER_ROT) * 60.0f;

		// sensor_data.wheel_rpm = (float)wheel_rpm_counter * wheel_counter_to_rpm_constant;
		// sensor_data.wheel_rpm = (float)wheel_rpm_counter;
		float rotor_rpm = (float)rotor_rpm_counter * rotor_counter_to_rpm_constant;
		rotor_rpm_counter = 0;

#define RPM_ROTOR_ABR_IGNORE_CNT 4
		static int ignore_counter = 0;
		if (abs(sensor_data.rotor_rpm - rotor_rpm) > 500)
		{
			ignore_counter++;
			if (ignore_counter == RPM_ROTOR_ABR_IGNORE_CNT)
			{
				ignore_counter = 0;
				// Force update the value
				sensor_data.rotor_rpm = rotor_rpm;
			}
			// Ignore
			return;
		}
		ignore_counter = 0;


		sensor_data.rotor_rpm = rotor_rpm;
}

double calculate_moy_uint32(uint32_t *data, uint32_t size) {
	double sum = 0;
	for (int i = 0; i < size; i++) {
		sum += data[i];
	}
	return sum / size;
}

double calculate_std_dev_uint32(double moy, uint32_t *data, uint32_t size) {
	double sum = 0;
	for (int i = 0; i < size; i++) {
		sum += pow(((double) data[i]) - moy, 2);
	}
	return sqrt(sum / size); //double std_dev
}

int compare(const void *a, const void *b) {
    return (*(int *)a - *(int *)b);
}

double calculate_median_uint32(uint32_t *data, uint32_t size) {
	uint32_t data_temp[size] = {0};

	for (int i = 0; i < size; i++) {
		data_temp[i] = data[i];
	}

	qsort(data_temp, size, sizeof(uint32_t), compare);

    if (size % 2 == 0) {
        // Even number of elements: average of the two middle elements
        return (data_temp[size / 2 - 1] + data_temp[size / 2]) / 2.0;
    } else {
        // Odd number of elements: middle element
        return data_temp[size / 2];
    }
}

#define log_encoder_raw_data_size 10
uint32_t log_encoder_raw_data[log_encoder_raw_data_size] = {0};
uint32_t log_encoder_raw_data_filtered[log_encoder_raw_data_size] = {0};

uint32_t verify_new_encoder_raw_data(uint32_t encoder_raw_data) {
	//ajouter la nouvelle valeur dans la liste et on scrap la plus vieille
	for (int i = log_encoder_raw_data_size - 1; i > 0; i--) {
		log_encoder_raw_data[i] = log_encoder_raw_data[i - 1];
		log_encoder_raw_data_filtered[i] = log_encoder_raw_data_filtered[i - 1];
	}
	log_encoder_raw_data[0] = encoder_raw_data;

	//moyenne
	double moy = calculate_moy_uint32(log_encoder_raw_data, log_encoder_raw_data_size);

	//écart type
	double std_dev = calculate_std_dev_uint32(moy, log_encoder_raw_data, log_encoder_raw_data_size);
	double lower_bound = moy - 1 * std_dev;
	double upper_bound = moy + 1 * std_dev;

	//si la valeur est mauvaise on la remplace par la moyenne
	if ((encoder_raw_data < lower_bound) || (encoder_raw_data > upper_bound)) {
		encoder_raw_data = (uint32_t) calculate_median_uint32(log_encoder_raw_data, log_encoder_raw_data_size);
	}
	log_encoder_raw_data_filtered[0] = encoder_raw_data;

	return log_encoder_raw_data_filtered[0];
}


#define PITCH_ENCODER_BITS 12
uint32_t ReadPitchEncoder()
{
	HAL_GPIO_WritePin(Pitch_Clock_GPIO_Port, Pitch_Clock_Pin, GPIO_PIN_RESET);
	delay_us(1);
	HAL_GPIO_WritePin(Pitch_Clock_GPIO_Port, Pitch_Clock_Pin, GPIO_PIN_SET);
	delay_us(1);

	// SSI works from 100kHz to about 2MHz
	uint32_t encoder_raw_data = 0;
	for(int i = 0; i < PITCH_ENCODER_BITS; i++)
	{

		HAL_GPIO_WritePin(Pitch_Clock_GPIO_Port, Pitch_Clock_Pin, GPIO_PIN_RESET);
		delay_us(1);

		encoder_raw_data <<= 1;

		if (HAL_GPIO_ReadPin(Pitch_Data_GPIO_Port, Pitch_Data_Pin)) {
			encoder_raw_data |= 1;
		}



		HAL_GPIO_WritePin(Pitch_Clock_GPIO_Port, Pitch_Clock_Pin, GPIO_PIN_SET);
		delay_us(1);

	}

	encoder_raw_data = verify_new_encoder_raw_data(encoder_raw_data);

	return encoder_raw_data;
}

uint32_t ReadMastEncoder()
{

	uint32_t mast_data = 0;
	for(int i = 0; i < 22; ++i)
	{
		mast_data <<= 1;

		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);
		//for (int i = 0; i < 20; ++i) {} // Wait 10 us
		// delay_us(10);

		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_SET);
		//for (int i = 0; i < 20; ++i) {} // Wait 10 us
		// delay_us(10);

		mast_data |= HAL_GPIO_ReadPin(Mast_Data_GPIO_Port, Mast_Data_Pin);
	}

	return mast_data;

	//HAL_GPIO_TogglePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin);
	//return 0;
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14) // PD_14 -- PB2
    {
    	//HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
    	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    	pb2_value = 1;

    	pb2_update = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_15) // PD_15 -- PB1
    {
    	//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    	// HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    	pb1_value = 1;

    	pb1_update = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_0) // Rotor RPM
    {
    	rotor_rpm_counter++;
    }
    else if (GPIO_Pin == GPIO_PIN_1) // Wheel RPM
    {
    	wheel_rpm_counter++;
    }
    else if (GPIO_Pin == GPIO_PIN_3) // Limit 1
    {
    	sensor_data.limit1 = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_4) // Limit 2
    {
    	sensor_data.limit2 = 1;
    }

}


// This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    // Transmit one byte with 100 ms timeout
    // HAL_UART_Transmit(&huart5, &byte, 1, 100);
	//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

	/*
	static char buf[128];
	static uint8_t index_buf = 0;
	buf[index_buf++] = ws_rx_byte;
	if (index_buf == 30)
 	{
		  buf[index_buf] = 0;
		  index_buf = 0;
		  HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
	}
	*/

	//HAL_UART_Transmit(&huart2, &ws_rx_byte, sizeof(ws_rx_byte), HAL_MAX_DELAY);

	// static char test_buffer[128] = {0};
	// static uint8_t test_index = 0;

	// rx_buff[index_buff] = ws_rx_byte;
	//index_buff++;

	rx_buff[index_buff++] = ws_rx_byte[0];
	// rx_buff[index_buff++] = ws_rx_byte[1];

	if(ws_rx_byte[0] == '\n')
	{
		rx_buff[index_buff++] = '\0';
		ws_receive_flag = 1;
	}
	/*
	rx_buff[index_buff++] = ws_rx_byte[1];
	if(ws_rx_byte[1] == '\n')
	{
		rx_buff[index_buff++] = '\0';
		index_buff = 0;
		ws_receive_flag = 1;
		// HAL_UART_Transmit(&huart2, test_buffer, strlen(test_buffer), HAL_MAX_DELAY);
	}*/
	//rx_buff[index_buff++] = ws_rx_byte[1];
	/*if(ws_rx_byte[1] == '\n')
	{
		rx_buff[index_buff] = '\0';
		index_buff = 0;
		ws_receive_flag = 1;
		// HAL_UART_Transmit(&huart2, test_buffer, strlen(test_buffer), HAL_MAX_DELAY);
	}*/


    // Restart interrupt for next byte
	HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&huart5, &ws_rx_byte[0], 1);
	if (ret != HAL_OK)
	{
		// Do something to reset the UART ?
		// HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_SET);
		if (ret == HAL_BUSY)
		{
			// Already waiting for bytes ??
			HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
		}
	}

	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_RESET);
  }
}

