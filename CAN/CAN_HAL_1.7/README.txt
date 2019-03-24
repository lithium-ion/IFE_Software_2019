How to use CAN with version 1.7.0 of STM32F1 HAL libraries:

CubeMX setup:
1. In CubeMx project, click "connectivity" and open CAN configuration. Enable CAN master mode and change the CAN bit timing parameters.
2. In CAN configuration, open NVIC settings and enable CAN RX0 interrupts
3. Generate code with version 1.7.0 of HAL libraries 

Hardware setup:
1. Ground the standby pin or pull it low if connected to a GPIO pin. 

HAL setup:
1. Declare the following global structures and variables (change names if needed):
	CAN_TxHeaderTypeDef   TxHeader;
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t               TxData[8];
	uint8_t               RxData[8];
	uint32_t              TxMailbox;

2. Modify the following TxHeader variables:
	TxHeader.StdId = 0x321; 				// CAN standard ID
	TxHeader.ExtId = 0x01; 					// CAN extended ID
	TxHeader.RTR = CAN_RTR_DATA; 			// CAN frame type
	TxHeader.IDE = CAN_ID_STD; 				// CAN ID type
	TxHeader.DLC = 8; 						// CAN frame length in bytes
	TxHeader.TransmitGlobalTime = DISABLE;	// CAN timestamp in TxData[6] and TxData[7]
	
3. Add the following filter settings to MX_CAN_Init() and enable the filter:
	sFilterConfig.FilterBank = 0;							// filter number (0-13)
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		// mask mode or identifier mode		
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;		
	sFilterConfig.FilterIdHigh = 0x0000;					// received ID must match filter ID for each bit specified by filter mask
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;				// specifies which bits of the received ID to compare to the filter ID
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;		// receive FIFO (0 or 1, must match chosen interrupt!)
	sFilterConfig.FilterActivation = ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	
4. Add "HAL_CAN_Start(&hcan);" to MX_CAN_Init() (change name of CAN handle if needed)

5. Add "HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);" to MX_CAN_Init()
	
	
To transmit, modify TxData and TxHeader if necessary and use the following command:
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	
To receive, create the following callback function:
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
To receive the pending message, run the following command:
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
