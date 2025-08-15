/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

unsigned long numTestsMRAM0 = 0;
unsigned long  numTestsMRAM1 = 0;
unsigned long  numTestsMRAM2 = 0;
unsigned long  numTestsMRAM3 = 0;
unsigned long  numTestsMRAM4 = 0;
unsigned long  numTestsMRAM5 = 0;

uint32_t numFailsMRAM0 = 0;
uint32_t numFailsMRAM1 = 0;
uint32_t numFailsMRAM2 = 0;
uint32_t numFailsMRAM3 = 0;
uint32_t numFailsMRAM4 = 0;
uint32_t numFailsMRAM5 = 0;

uint8_t currChip = 0;
uint8_t currState = 0;
uint8_t consoleMode = 1;

uint8_t wren = 0x06;
uint8_t configure_code_1[] = {0x71,0x00,0x00,0x05,0x05}; // code for configuring avalanche AS1016A04
uint8_t configure_code_2[] = {0x71,0x00,0x00,0x05,0x02}; // code for configuring NETSOL S3A1604V0M
uint8_t Rx[1] = {'\0'};
uint8_t read = 0;
char testDetailCmd[3] = { '?', '\r', '\n'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_LPUART1_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void WriteMem(uint8_t chip, uint32_t addr, uint16_t length, uint8_t* data);
void ReadMem(uint8_t chip, uint32_t addr, uint16_t length, uint8_t* buffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void clrstr(uint8_t* str, int len){
	for(int i=0; i<len; i++){
		*(str+i)='\0';
	}
}

// wait in nanoseconds
void delay_ns(uint32_t ns){
	//get system clock frequency
	uint32_t system_clock_hz = SystemCoreClock;
	//calculate the number of cycles required for this delay
	uint64_t cycles_needed_64 = (uint64_t)system_clock_hz * ns;

	//Now divide by the number of nanoseconds in a second
	uint32_t cycles_needed = cycles_needed_64/1000000000;

	uint32_t start_cycle = DWT->CYCCNT;
	while((DWT->CYCCNT-start_cycle)<cycles_needed);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){                                       // Weakly defined function, used for callback in the ISR in this case
	                                                                                          // Better practice to have it in a separate file
	HAL_UART_Receive_IT(&hlpuart1, Rx, 1); //It will then self-sustain

    if(Rx[0]==testDetailCmd[read])                                                    //strcmp may be wrongly used here
      {
      	read++;
      }
    else
      {
    	  if(Rx[0]==testDetailCmd[0]){
    		  read = 1;
    	  }
    	  else{
    	      	read=0;
    	  }
      }
      if(read>=2)
      {
      	read=0;
      	if(consoleMode){
      		uint8_t data[76];
      		clrstr(data, 76);
      		HAL_UART_Transmit(&hlpuart1,"Memory Test Latest Results: \r\n\n" ,31,40); //SET TO /0 SO AS TO REMOVE EXTRANEOUS STUFF
      		clrstr(data, 76);
      		sprintf(data, "MRAM0 tests completed:\t%lu\r\nMRAM0 failures:\t\t%u\r\n", numTestsMRAM0, numFailsMRAM0);
      		HAL_UART_Transmit(&hlpuart1,data,strlen(data),40);
      		clrstr(data, 76);
      		sprintf(data, "MRAM1 tests completed:\t%lu\r\nMRAM1 failures:\t\t%u\r\n", numTestsMRAM1, numFailsMRAM1);
      		HAL_UART_Transmit(&hlpuart1,data,strlen(data),40);
      		clrstr(data, 76);
      		sprintf(data, "MRAM2 tests completed:\t%lu\r\nMRAM2 failures:\t\t%u\r\n", numTestsMRAM2, numFailsMRAM2);
      		HAL_UART_Transmit(&hlpuart1,data,strlen(data),40);
      		clrstr(data, 76);
      		sprintf(data, "MRAM3 tests completed:\t%lu\r\nMRAM3 failures:\t\t%u\r\n", numTestsMRAM3, numFailsMRAM3);
      		HAL_UART_Transmit(&hlpuart1,data,strlen(data),40);
      		clrstr(data, 76);
      		sprintf(data, "MRAM4 tests completed:\t%lu\r\nMRAM4 failures:\t\t%u\r\n", numTestsMRAM4, numFailsMRAM4);
      		HAL_UART_Transmit(&hlpuart1,data,strlen(data),40);
      		clrstr(data, 76);
      		sprintf(data, "MRAM5 tests completed:\t%lu\r\nMRAM5 failures:\t\t%u\r\n", numTestsMRAM5, numFailsMRAM5);
      		HAL_UART_Transmit(&hlpuart1,data,strlen(data),40);
      		clrstr(data, 76);

      		HAL_UART_Transmit(&hlpuart1,"\nCurrently testing chip ",24,40);
      		switch(currChip){
      		case 0:
      			HAL_UART_Transmit(&hlpuart1,"MRAM0\r\n",7,40);
      			break;
      		case 1:
      			HAL_UART_Transmit(&hlpuart1,"MRAM1\r\n",7,40);
      			break;
      		case 2:
      			HAL_UART_Transmit(&hlpuart1,"MRAM2\r\n",7,40);
      			break;
      		case 3:
      			HAL_UART_Transmit(&hlpuart1,"MRAM3\r\n",7,40);
      			break;
      		case 4:
      			HAL_UART_Transmit(&hlpuart1,"MRAM4\r\n",7,40);
      			break;
      		case 5:
      			HAL_UART_Transmit(&hlpuart1,"MRAM5\r\n",7,40);
      			break;
      		}
      		HAL_UART_Transmit(&hlpuart1,"Current state: ",15,40);
      		switch(currState){
      		case 0:
      			HAL_UART_Transmit(&hlpuart1,"UP:w0\r\n\r\n",9,40);
      			break;
      		case 1:
      			HAL_UART_Transmit(&hlpuart1,"UP:r0 w1\r\n\r\n",12,40);
      			break;
      		case 2:
      			HAL_UART_Transmit(&hlpuart1,"UP:r1 w0\r\n\r\n",12,40);
      			break;
      		case 3:
      			HAL_UART_Transmit(&hlpuart1,"DOWN:r0 w1\r\n\r\n",14,40);
      			break;
      		case 4:
      			HAL_UART_Transmit(&hlpuart1,"DOWN:r1 w0\r\n\r\n",14,40);
      			break;
      		case 5:
      			HAL_UART_Transmit(&hlpuart1,"DOWN:r0\r\n\r\n",11,40);
      			break;
      		}
      	}
      	else{                       // The rest of the function won't be used
      		uint8_t data[28];

      		data[0] = (numTestsMRAM0>>24)&0x000000FF;
      		data[1] = (numTestsMRAM0>>16)&0x000000FF;
      		data[2] = (numTestsMRAM0>>8)&0x000000FF;
      		data[3] = numTestsMRAM0&0x000000FF;              //for MRAM0
      		data[4] = (numTestsMRAM1>>24)&0x000000FF;
      		data[5] = (numTestsMRAM1>>16)&0x000000FF;
      		data[6] = (numTestsMRAM1>>8)&0x000000FF;
      		data[7] = numTestsMRAM1&0x000000FF;              //for MRAM1
      		data[8] = (numTestsMRAM2>>24)&0x000000FF;
      		data[9] = (numTestsMRAM2>>16)&0x000000FF;
      		data[10] = (numTestsMRAM2>>8)&0x000000FF;
      		data[11] = numTestsMRAM2&0x000000FF;             //for MRAM2
      		data[12] = (numTestsMRAM3>>24)&0x000000FF;
      		data[13] = (numTestsMRAM3>>16)&0x000000FF;
      		data[14] = (numTestsMRAM3>>8)&0x000000FF;
      		data[15] = numTestsMRAM3&0x000000FF;             //for MRAM3


      		data[16] = (numFailsMRAM0>>8)&0x00FF;
      		data[17] = numFailsMRAM0&0x00FF;
      		data[18] = (numFailsMRAM1>>8)&0x00FF;
      		data[19] = numFailsMRAM1&0x00FF;
      		data[20] = (numFailsMRAM2>>8)&0x00FF;
      		data[21] = numFailsMRAM2&0x00FF;
      		data[22] = (numFailsMRAM3>>8)&0x00FF;
      		data[23] = numFailsMRAM3&0x00FF;


      		data[24] = currChip;
      		data[25] = currState;


      		data[26] = '\r';
      		data[27] = '\n';

      		HAL_UART_Transmit(&hlpuart1,data,28,30);

      	}
      }
      Rx[0] = '\0';

}

void WriteMem(uint8_t chip, uint32_t addr, uint16_t length, uint8_t* data){
	length += 4; //Account for the header
	uint16_t selectedPin = 0;
	switch(chip){
	case 0:
		selectedPin = 1024;
		break;
	case 1:
		selectedPin = 4096;
		break;
	case 2:
		selectedPin = 16384;
		break;
	case 3:
		selectedPin = 32768;
		break;
	case 4:
		selectedPin = 512;
		break;
	case 5:
		selectedPin = 2048;
		break;
	}

	/*if(chip>1){ //If it's FRAM we need to re-enable writes after every write
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET); //Set WRite ENable
		  HAL_SPI_Transmit(&hspi1, &wren, 1, 10);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	}*/

	uint8_t TxBuff[length];
	TxBuff[0] = 0x02;
	TxBuff[1] = (addr>>16)&0x0000FF;
	TxBuff[2] = (addr>>8)&0x0000FF;
	TxBuff[3] = (addr)&0x0000FF;                                                                   //we only need the first 24 bits (16Mb)

	for(int i=4; i<length; i++){
		TxBuff[i] = *(data+(i-4)); //Loop through all the data and put it into the Tx buffer.
	}

	HAL_GPIO_WritePin(GPIOE, selectedPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)TxBuff, length, 100);
	HAL_GPIO_WritePin(GPIOE, selectedPin, GPIO_PIN_SET);
}

void ReadMem(uint8_t chip, uint32_t addr, uint16_t length, uint8_t* buffer){
	length += 4; //Account for the header

	uint16_t selectedPin = 0;
	switch(chip){
	case 0:
		selectedPin = 1024;
		break;
	case 1:
		selectedPin = 4096;
		break;
	case 2:
		selectedPin = 16384;
		break;
	case 3:
		selectedPin = 32768;
		break;
	case 4:
		selectedPin = 512;
		break;
	case 5:
		selectedPin = 2048;
		break;
	}

	uint8_t TxBuff[length];
	TxBuff[0] = 0x03;
	TxBuff[1] = (addr>>16)&0x0000FF;
	TxBuff[2] = (addr>>8)&0x0000FF;
	TxBuff[3] = (addr)&0x0000FF;
	for(int j = 4; j< length; j++){
		TxBuff[j] = '\0';
	}
	uint8_t RxBuff[length];

	HAL_GPIO_WritePin(GPIOE, selectedPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)TxBuff, (uint8_t *)RxBuff, length, length);
	HAL_GPIO_WritePin(GPIOE, selectedPin, GPIO_PIN_SET);

	for(int i=4; i<length; i++){
		*(buffer+(i-4)) = RxBuff[i]; //Load all the received data into the proper location
	}

}



// this is the Memory test for 4Mb MR25H40MDF
void MemTest(int chip, uint32_t* fails, unsigned long* num_reads){

    uint8_t zero = 0;                 //Maybe make a 6-deep array of ints to track the failures in each march element?   //failure is redundant
    uint8_t one = 255;
    uint8_t readByte = 0;
    uint8_t writeByte = 0;

    currState = 0;
    HAL_UART_Transmit(&hlpuart1,(uint8_t*)"currState 0\r\n",13,50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    //UP(w0);
    for(int x=0; x<524288; x++){ //Loop through each memory location
    	WriteMem(chip, x, 1, &zero); //Write a 0 at each address                                     // the final 0 is suspicious, it may cause error bc it should be a pointer
    }

    currState = 1;
    HAL_UART_Transmit(&hlpuart1,(uint8_t*)"currState 1\r\n",13,50);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    //UP(r0, w1);
    for(int y=0; y<524288; y++){ 		//Loop through each memory location
    	for(int z=0; z<8; z++){ 		//Read 0, write 1 at each bit
    		ReadMem(chip, y, 1, &readByte);

    		(*num_reads) ++;
    		writeByte = readByte;
    		writeByte |= (0x01<<(7-z)); //Copy the byte, except we set the MSb to 1
    		*fails += (readByte>>(7-z))&0x01; //Read the MSb and if it's not 0, increment failures

        	WriteMem(chip, y, 1, &writeByte); //Read 0, write 1 at each bit
    	}
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);


    currState = 2;
    HAL_UART_Transmit(&hlpuart1,(uint8_t*)"currState 2\r\n",13,50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //UP(r1, w0);
    for(int y=0; y<524288; y++){ 		//Loop through each memory location
    	for(int z=0; z<8; z++){ 		//Read 1, write 0 at each bit
    		ReadMem(chip, y, 1, &readByte);

    		(*num_reads) ++;

    		writeByte = readByte;
    		writeByte &= ~(0x01<<(7-z)); //Copy the byte, except we set the MSb to 0
    		*fails += ((~readByte)>>(7-z))&0x01; //Read the MSb and if it's not 1, increment failures
        	WriteMem(chip, y, 1, &writeByte);

    	}
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

    currState = 3;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    //DOWN(r0, w1);
    for(int y=524287; y>=0; y--){ 		//Loop through each memory location
    	for(int z=0; z<8; z++){ 		//Read 0, write 1 at each bit
    		ReadMem(chip, y, 1, &readByte);

    		(*num_reads) ++;

    		writeByte = readByte;
    		writeByte |= (0x01<<z); //Copy the byte, except we set the LSb to 1
    		*fails += (readByte>>z)&0x01; //Read the LSb and if it's not 0, increment failures
        	WriteMem(chip, y, 1, &writeByte);
    	}
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

    currState = 4;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //DOWN(r1, w0);
    for(int y=524287; y>=0; y--){ 		//Loop through each memory location
    	for(int z=0; z<8; z++){			//Read 1, write 0 at each bit
    		ReadMem(chip, y, 1, &readByte);

    		(*num_reads) ++;

    		writeByte = readByte;
    		writeByte &= ~(0x01<<z); //Copy the byte, except we set the LSb to 0
    		*fails += ((~readByte)>>z)&0x01; //Read the LSb and if it's not 1, increment failures
        	WriteMem(chip, y, 1, &writeByte);
    	}
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

    currState = 5;
    //DOWN(r0);
    for(int y=524287; y>=0; y--){ 		//Loop through each memory location
    	ReadMem(chip, y, 1, &readByte);

    	(*num_reads)++;

    	if(readByte>0){
    		(*fails)++;					//If any bytes are non-zero, increment failures
    	}
    }

    return;
}


// Memory test for memory with 16M memory (The remaining three MRAMs)
void MemTest_16(int chip, uint32_t* fails, unsigned long* num_reads){

    uint8_t zero = 0;                 //Maybe make a 6-deep array of ints to track the failures in each march element?   //failure is redundant
    uint8_t readByte = 0;
    uint8_t writeByte = 0;

    currState = 0;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    //UP(w0);
    for(int x=0; x<2097152; x++){ //Loop through each memory location
    	WriteMem(chip, x, 1, &zero); //Write a 0 at each address                                     // the final 0 is suspicious, it may cause error bc it should be a pointer
    }

    /*uint8_t data[16];
    ReadMem(1, 100, 1, &readByte);
    sprintf(data, "read value:\t\t%u\r\n", readByte);
    HAL_UART_Transmit(&hlpuart1,data,sizeof(data),100);*/



    currState = 1;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    //UP(r0, w1);
    for(int y=0; y<2097152; y++){ 		//Loop through each memory location
    	for(int z=0; z<8; z++){ 		//Read 0, write 1 at each bit
    		ReadMem(chip, y, 1, &readByte);
    		(*num_reads)++;
    		writeByte = readByte;
    		writeByte |= (0x01<<(7-z)); //Copy the byte, except we set the MSb to 1
    		*fails += (readByte>>(7-z))&0x01; //Read the MSb and if it's not 0, increment failures
        	WriteMem(chip, y, 1, &writeByte); //Read 0, write 1 at each bit
    	}
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);


    currState = 2;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //UP(r1, w0);
    for(int y=0; y<2097152; y++){ 		//Loop through each memory location
    	for(int z=0; z<8; z++){ 		//Read 1, write 0 at each bit
    		ReadMem(chip, y, 1, &readByte);
    		(*num_reads)++;
    		writeByte = readByte;
    		writeByte &= ~(0x01<<(7-z)); //Copy the byte, except we set the MSb to 0
    		*fails += ((~readByte)>>(7-z))&0x01; //Read the MSb and if it's not 1, increment failures
        	WriteMem(chip, y, 1, &writeByte);
    	}
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

    currState = 3;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    //DOWN(r0, w1);
    for(int y=2097151; y>=0; y--){ 		//Loop through each memory location
    	for(int z=0; z<8; z++){ 		//Read 0, write 1 at each bit
    		ReadMem(chip, y, 1, &readByte);
    		(*num_reads)++;
    		writeByte = readByte;
    		writeByte |= (0x01<<z); //Copy the byte, except we set the LSb to 1
    		*fails += (readByte>>z)&0x01; //Read the LSb and if it's not 0, increment failures
        	WriteMem(chip, y, 1, &writeByte);

    	}
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

    currState = 4;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //DOWN(r1, w0);
    for(int y=2097151; y>=0; y--){ 		//Loop through each memory location
    	for(int z=0; z<8; z++){			//Read 1, write 0 at each bit
    		ReadMem(chip, y, 1, &readByte);
    		(*num_reads)++;
    		writeByte = readByte;
    		writeByte &= ~(0x01<<z); //Copy the byte, except we set the LSb to 0
    		*fails += ((~readByte)>>z)&0x01; //Read the LSb and if it's not 1, increment failures
        	WriteMem(chip, y, 1, &writeByte);
    	}
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

    currState = 5;
    //DOWN(r0);
    for(int y=2097151; y>=0; y--){ 		//Loop through each memory location
    	ReadMem(chip, y, 1, &readByte);
    	(*num_reads)++;
    	if(readByte>0){
    		(*fails)++;					//If any bytes are non-zero, increment failures
    	}
    }

    return;

}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT =0;



  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET); //Set WRite ENable
  HAL_SPI_Transmit(&hspi1, &wren, 1, 10); // to enable write on the MRAM chip
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  HAL_UART_Receive_IT(&hlpuart1, Rx, 1);

  HAL_Delay(1); //short delay between transactions

  //We have to configure the mode of "Avalanche AS1016A04/Avalanche AS1004A04" such that we get the back-to-back mode
  //Then we don't need wren every time we want to write
  HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, configure_code_1, 5, 20);
  HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
/*
  //we have to configure the mode of "NETSOL S3A1606VOM" such that we get the back-to-back mode
  //Then we don't need wren every time we want to write
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, configure_code_2, 5, 20);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
*/

  HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET); //Set WRite ENable
  HAL_SPI_Transmit(&hspi1, &wren, 1, 10); // to enable write on the MRAM chip
  HAL_GPIO_WritePin(GPIOE,  GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);


  HAL_UART_Transmit(&hlpuart1, (uint8_t*)"\r\n", 2, 1000);//Get the interrupt running
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  // check if the code is actually running

  int pos = 0;
  uint8_t writeByte = 0;
  uint8_t readByte = 0;
  int chip = 0;
  uint8_t data[37];





  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    /*currChip = 0;
    MemTest(0, &numFailsMRAM0,&numTestsMRAM0);

    currChip = 1;
    MemTest(1, &numFailsMRAM1, &numTestsMRAM1);

    currChip = 2;
    MemTest(2, &numFailsMRAM2, &numTestsMRAM2);

    currChip = 3;
    MemTest(3, &numFailsMRAM3, &numTestsMRAM3);

    currChip = 4;
    MemTest(4, &numFailsMRAM4, &numTestsMRAM4);

    currChip = 5;
    MemTest(5, &numFailsMRAM5, &numTestsMRAM5);*/




    WriteMem(chip, pos, 1, &writeByte); //Read 0, write 1 at each bit
    ReadMem(chip, pos, 1, &readByte);

    sprintf(data, "chip number: %u  read value:\t\t%u\r\n",chip,readByte);
    HAL_UART_Transmit(&hlpuart1,data,sizeof(data),20);
    clrstr(data,37);

    chip++;
    pos++;
    if (chip==6){
    	chip=0;
    	writeByte++;
    }
    if (writeByte==256){
    	writeByte=0;
    }
    HAL_Delay(1000);
    if (writeByte!=readByte){
    	numFailsMRAM0++;
    }

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET); //May need this if code is regenerated
  /*Configure GPIO pin Output Level*/
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET); //And this
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|SMPS_V1_Pin|SMPS_EN_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE10 PE11 PE12
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OverCurrent_Pin SMPS_PG_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin|SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PowerSwitchOn_Pin SMPS_V1_Pin SMPS_EN_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin|SMPS_V1_Pin|SMPS_EN_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
