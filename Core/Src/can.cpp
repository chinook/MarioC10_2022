

#include "can.h"
#include "main.h"

#include "stm32f4xx_hal.h"


// CAN variables
uint8_t txData[8];
uint8_t rxData[8];
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t txMailbox;

uint8_t button_press(uint32_t can_data) {
	if ((can_data & 0xFF) == CAN_STATUS_PRESS) {
		return 1;
	} else if ((can_data & 0xFF) == CAN_STATUS_UNPRESS) {
		return 0;
	} else {
		return -1;
	}
}

uint8_t old_status_button_hgg = 0;
uint8_t status_button_hgg_toggle_tmp = 0;
uint8_t status_button_hgg = 0;

uint8_t old_status_button_hg = 0;
uint8_t status_button_hg_toggle_tmp = 0;
uint8_t status_button_hg = 0;

uint8_t old_status_button_hd = 0;
uint8_t status_button_hd_toggle_tmp = 0;
uint8_t status_button_hd = 0;

uint8_t old_status_button_hdd = 0;
uint8_t status_button_hdd_toggle_tmp = 0;
uint8_t status_button_hdd = 0;

uint8_t old_status_button_mg = 0;
uint8_t status_button_mg_toggle_tmp = 0;
uint8_t status_button_mg = 0;

uint8_t old_status_button_md = 0;
uint8_t status_button_md_toggle_tmp = 0;
uint8_t status_button_md = 0;

uint8_t old_status_button_bgg = 0;
uint8_t status_button_bgg_toggle_tmp = 0;
uint8_t status_button_bgg = 0;

uint8_t old_status_button_bg = 0;
uint8_t status_button_bg_toggle_tmp = 0;
uint8_t status_button_bg = 0;

uint8_t old_status_button_bd = 0;
uint8_t status_button_bd_toggle_tmp = 0;
uint8_t status_button_bd = 0;

uint8_t old_status_button_bdd = 0;
uint8_t status_button_bdd_toggle_tmp = 0;
uint8_t status_button_bdd = 0;

void button_toggle(uint32_t can_data, uint8_t *status_button_x, uint8_t *old_status_button_x, uint8_t *status_button_x_toggle_tmp) {
	if (*old_status_button_x != (can_data & 0xFF)) {
		*status_button_x_toggle_tmp += 1;
	}
	if (*status_button_x_toggle_tmp >= 2) {
		*status_button_x_toggle_tmp = 0;
		if (*status_button_x == CAN_STATUS_PRESS) {
			*status_button_x = CAN_STATUS_UNPRESS;
		} else {
			*status_button_x = CAN_STATUS_PRESS;
		}
	}
	*old_status_button_x = (can_data & 0xFF);
}


void ProcessCanMessage()
{
	/*
	typedef union BytesToType_
	{
		struct
		{
			uint8_t bytes[4];
		};
		int32_t int_val;
		uint32_t uint_val;
		float float_val;
	} BytesToType;
	static BytesToType bytesToType;
	*/


	// Indicate CAN working with CAN led
	HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);

	// Technically CAN data can be 8 bytes but we only send 4-bytes data to the motor driver
	// uint32_t upper_can_data = rxData[4] | (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24);
	uint32_t can_data = rxData[0] | (rxData[1] << 8) | (rxData[2] << 16) | (rxData[3] << 24);

	if (pRxHeader.StdId == CAN_ID_STATE_DRIVEMOTOR_PITCH_MODE)
	{
		sensor_data.feedback_pitch_mode = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == CAN_ID_STATE_DRIVEMOTOR_MAST_MODE)
	{
		sensor_data.feedback_mast_mode = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == CAN_ID_DRIVEMOTOR_PITCH_MOVE_DONE)
	{
		pitch_done = 1;
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_PITCH_FAULT_STALL)
	{
		sensor_data.pitch_fault_stall = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_MAST_FAULT_STALL)
	{
		sensor_data.mast_fault_stall = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == CAN_ID_CMD_VOLANT_MANUAL_ROPS)
	{
		uint8_t rops_data = (can_data & 0xFF);
		if (rops_data == ROPS_ENABLE)
			b_rops = 1;
		else if (rops_data == ROPS_DISABLE)
			b_rops = 0;
		else
		{
			// Unknown value received for ROPS command
			// Assume it was meant to activate ROPS
			b_rops = 1;
		}
	}
	else if (pRxHeader.StdId == CAN_ID_STATE_DRIVEMOTOR_ROPS)
	{
		uint8_t rops_data = (can_data & 0xFF);
		sensor_data.feedback_pitch_rops = rops_data;
		/*
		if (rops_data == ROPS_ENABLE)
			sensor_data.feedback_pitch_rops = 1;
		else if (rops_data == ROPS_DISABLE)
			sensor_data.feedback_pitch_rops = 0;
		else
			sensor_data.feedback_pitch_rops = rops_data;
		*/
	} else if (pRxHeader.StdId == CAN_ID_CMD_MARIO_PITCH_MODE) {

		/*
		if (old_cmd_mario_pitch_mode != can_data) {
			cmd_mario_pitch_mode_toggle++;
		}
		if (cmd_mario_pitch_mode_toggle >= 2) {
			cmd_mario_pitch_mode_toggle = 0;
			if (test_buttons_volant == MOTOR_MODE_AUTOMATIC) {
				test_buttons_volant = MOTOR_MODE_MANUAL;
			} else {
				test_buttons_volant = MOTOR_MODE_AUTOMATIC;
			}
		}
		old_cmd_mario_pitch_mode = can_data;*/
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_HGG) {
		status_button_hgg = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_HG) {
		//status_button_hg = button_press(can_data);
		button_toggle(can_data, &status_button_hg, &old_status_button_hg, &status_button_hg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_HD) {
		status_button_hd = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_HDD) {
		status_button_hdd = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_MG) {
		status_button_mg = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_MD) {
		status_button_md = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_BGG) {
		status_button_bgg = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_BG) {
		status_button_bg = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_BD) {
		status_button_bd = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}
	else if (pRxHeader.StdId == CAN_ID_STATUS_BUTTON_BDD) {
		status_button_bdd = button_press(can_data);
		//button_toggle(can_data, &status_button_hgg_toggle, old_status_button_hgg, status_button_hgg_toggle_tmp);
	}

	else
	{
		// Unknown CAN ID
	}
}

void CAN_ReceiveFifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifo)
{
	uint32_t num_messages = HAL_CAN_GetRxFifoFillLevel(hcan, fifo);
	for (int i = 0; i < num_messages; ++i)
	{
		if (HAL_CAN_GetRxMessage(hcan, fifo, &pRxHeader, rxData) != HAL_OK)
		{
			Error_Handler();
		}

		ProcessCanMessage();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
	CAN_ReceiveFifoCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
	CAN_ReceiveFifoCallback(hcan, CAN_RX_FIFO1);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
	// HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
	HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
}

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size, uint8_t with_priority)
{
	// CAN_TxHeaderTypeDef msg;
	pTxHeader.StdId = id;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.DLC = size; // Number of bytes to send
	pTxHeader.TransmitGlobalTime = DISABLE;

	uint8_t found_mailbox = 0;
	for (int i = 0; i < 4; ++i)
	{
		// Check that mailbox is available for tx
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
		{
			found_mailbox = 1;
			break;
		}
		// Otherwise wait until free mailbox
		// for (int j = 0; j < 500; ++j) {}
		delay_us(2);
	}
	if (!found_mailbox)
	{
		// HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	}

	if (with_priority)
	{
		// If message is important, make sure no other messages are queud to ensure it will be sent after any other
		// values that could override it.
		for (int i = 0; i < 10; ++i)
		{
			// Check that all 3 mailboxes are empty
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3)
				break;
			// Otherwise wait until 3 free mailbox
			// for (int j = 0; j < 500; ++j) {}
			delay_us(50);
		}
	}

	uint32_t mb;
	HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, buf, &mb);
	if (ret != HAL_OK)
	{
		HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_SET);
		return ret;
	}

	// Successful transmit
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	// Update the error led if had a successful can write
	HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_RESET);
	// ToggleLed(LED_CAN);
	return ret;
}


/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_3TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /*
  CAN_FilterTypeDef filter_fifo0;
      	// All common bits go into the ID register
      filter_fifo0.FilterIdHigh = MARIO_FIFO0_RX_FILTER_ID_HIGH;
      filter_fifo0.FilterIdLow = MARIO_FIFO0_RX_FILTER_ID_LOW;

      	// Which bits to compare for filter
      filter_fifo0.FilterMaskIdHigh = MARIO_FIFO0_RX_FILTER_MASK_HIGH;
      filter_fifo0.FilterMaskIdLow = MARIO_FIFO0_RX_FILTER_MASK_LOW;

      filter_fifo0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
      filter_fifo0.FilterBank = 18; // Which filter to use from the assigned ones
      filter_fifo0.FilterMode = CAN_FILTERMODE_IDMASK;
      filter_fifo0.FilterScale = CAN_FILTERSCALE_32BIT;
      filter_fifo0.FilterActivation = CAN_FILTER_ENABLE;
      filter_fifo0.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
  	if (HAL_CAN_ConfigFilter(&hcan1, &filter_fifo0) != HAL_OK)
  	{
  	  Error_Handler();
  	}
  	*/
/*
    CAN_FilterTypeDef filter_all;
    	// All common bits go into the ID register
    filter_all.FilterIdHigh = 0x0000;
    filter_all.FilterIdLow = 0x0000;

    	// Which bits to compare for filter
    filter_all.FilterMaskIdHigh = 0x0000;
    filter_all.FilterMaskIdLow = 0x0000;

    filter_all.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter_all.FilterBank = 18; // Which filter to use from the assigned ones
    filter_all.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_all.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_all.FilterActivation = CAN_FILTER_ENABLE;
    filter_all.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
	if (HAL_CAN_ConfigFilter(&hcan1, &filter_all) != HAL_OK)
	{
	  Error_Handler();
	}
*/


  	CAN_FilterTypeDef sf_fifo0;
  	// All common bits go into the ID register
  	sf_fifo0.FilterIdHigh = MARIO_FIFO0_RX_FILTER_ID_HIGH;
  	sf_fifo0.FilterIdLow = MARIO_FIFO0_RX_FILTER_ID_LOW;

  	// Which bits to compare for filter
  	sf_fifo0.FilterMaskIdHigh = 0x0000;
  	sf_fifo0.FilterMaskIdLow = (FIFO0_RX_FILTER_MASK_LOW & 0x07FF);

  	sf_fifo0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  	sf_fifo0.FilterBank = 2; // Which filter to use from the assigned ones
  	sf_fifo0.FilterMode = CAN_FILTERMODE_IDMASK;
  	sf_fifo0.FilterScale = CAN_FILTERSCALE_32BIT;
  	sf_fifo0.FilterActivation = CAN_FILTER_ENABLE;
  	sf_fifo0.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
  	if (HAL_CAN_ConfigFilter(&hcan1, &sf_fifo0) != HAL_OK)
  	{
  	  Error_Handler();
  	}


  	CAN_FilterTypeDef sf_fifo1;
  	// All common bits go into the ID register
  	sf_fifo1.FilterIdHigh = FIFO1_RX_FILTER_ID_HIGH;
  	sf_fifo1.FilterIdLow = FIFO1_RX_FILTER_ID_LOW;

  	// Which bits to compare for filter
  	sf_fifo1.FilterMaskIdHigh = 0x0000;
  	sf_fifo1.FilterMaskIdLow = (FIFO1_RX_FILTER_MASK_LOW & 0x07FF);

  	sf_fifo1.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  	sf_fifo1.FilterBank = 3; // Which filter to use from the assigned ones
  	sf_fifo1.FilterMode = CAN_FILTERMODE_IDMASK;
  	sf_fifo1.FilterScale = CAN_FILTERSCALE_32BIT;
  	sf_fifo1.FilterActivation = CAN_FILTER_ENABLE;
  	sf_fifo1.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
  	if (HAL_CAN_ConfigFilter(&hcan1, &sf_fifo1) != HAL_OK)
  	{
  	  Error_Handler();
  	}


  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  	{
  		Error_Handler();
  	}
  	// if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE) != HAL_OK)
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  	// if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  	{
  		Error_Handler();
  	}

  /* USER CODE END CAN1_Init 2 */

}
