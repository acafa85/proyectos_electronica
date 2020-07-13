/*
 * @brief UART interrupt example with ring buffers
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "chip.h"
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "display.h"
#include <stdlib.h>
#include <string.h>
#include "stdio.h"
#include "board.h"
#include "MPU6050RegDef.h"
#include "MPU6050.h"
#include "semphr.h"
#include "math.h"
#include "botones.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define DEFAULT_I2C          I2C0

#define BUZZ 8 //puerto 2


#define I2C_IOX_BUS          DEFAULT_I2C

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000

#define TICKRATE_HZ1 (1000)	/* interrumpe cada 1ms */
volatile uint32_t msTicks; // counter for 1ms SysTicks

#define I2C_INCLINOMETRO 			LPC_I2C0
#define IRQ_INCLINOMETRO 			I2C0_IRQn
#define HANDLER_INCLINOMETRO 	    I2C0_IRQHandler

xTaskHandle xINCLINOMETRO; // Handler para tarea que maneja el inclinometro

#define DEBOUNCE 50  	//tiempo para el antirrebote (ms)
// Acc Full Scale Range  +-2G 4G 8G 16G
enum Ascale
{
    AFS_2G=0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

// Gyro Full Scale Range +-250 500 1000 2000 Degrees per second
enum Gscale
{
    GFS_250DPS=0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

// Sensor datas
float ax,ay,az;
float gx,gy,gz;
int16_t accelData[3],gyroData[3],tempData;
float accelBias[3] = {0, 0, 0};  // Bias corrections for acc
float gyroBias[3] = {0, 0, 0};   // Bias corrections for gyro

// Specify sensor full scale range
int Ascale = AFS_2G;
int Gscale = GFS_250DPS;

// Scale resolutions per LSB for the sensors
float aRes, gRes;
void delay_ms_init(uint32_t);

float DESVIO = 5;   // valor de calibracion inicial que uso, puesto que la IMU no esta nunca perfectamente horizontal
int referencia=0; // Variable usada de referencia

// Calculates Acc resolution
void getAres()
{
    switch(Ascale)
    {
        case AFS_2G:
            aRes = 2.0/32768.0;
            break;
        case AFS_4G:
            aRes = 4.0/32768.0;
            break;
        case AFS_8G:
            aRes = 8.0/32768.0;
            break;
        case AFS_16G:
            aRes = 16.0/32768.0;
            break;
    }
}

// Calculates Gyro resolution
void getGres()
{
    switch(Gscale)
    {
        case GFS_250DPS:
            gRes = 250.0/32768.0;
            break;
        case GFS_500DPS:
            gRes = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            gRes = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            gRes = 2000.0/32768.0;
            break;
    }
}



static int mode_poll;   /* Poll/Interrupt mode flag */
static I2C_ID_T i2cDev = DEFAULT_I2C; /* Currently active I2C device */

#define I2C_SLAVE_IOX_ADDR          0x68

int threshold=45;

/* Data area for slave operations */

static uint8_t iox_data[2]; /* PORT0 input port, PORT1 Output port */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#define TICKRATE_HZ1 (1000)	/* 10 ticks per second */
volatile uint32_t msTicks; // counter for 1ms SysTicks

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}


/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	Board_I2C_Init(id);

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}




/*-------- IO Expansion slave device implementation ----------*/
/* Update IN/OUT port states to real devices */
void i2c_iox_update_regs(int ops)
{
	if (ops & 1) { /* update out port */
		Board_LED_Set(0, iox_data[1] & 1);
		Board_LED_Set(1, iox_data[1] & 2);
		Board_LED_Set(2, iox_data[1] & 4);
		Board_LED_Set(3, iox_data[1] & 8);
	}

	if (ops & 2) { /* update in port */
		iox_data[0] = (uint8_t) Buttons_GetStatus();
	}
}


void I2C1_IRQHandler(void)
{
	i2c_state_handling(I2C1);
}

/**
 * @brief	I2C0 Interrupt handler
 * @return	None
 */


void I2C0_IRQHandler(void)
{
	i2c_state_handling(I2C0);
	xTaskResumeFromISR(xINCLINOMETRO);
}

float pitchAngle = 0;
float rollAngle = 0;

float offset=0; //variable para guardar la inclinacion a restar cuando actualizo el cero al oprimir ambos botones

/*****************************************************************************
 * Tipos, enumerativos y variables Privadas
 ****************************************************************************/


const char mensaje1[] = "Hola Mundo\r\n";

//Variables Globales

int debounce;

char pulsador;

char flag_cambio_ref=0; //flag global para avisar que se presionaron los dos botones al mismo tiempo y debo cambiar
                        // la referencia de 0°


void vENTRADA( void *pvParameters)
{
	for(;;)
	{

		//Reviso los botones
		//para ver si hay que actualizar el
		//Threshold

		switch(botones())
			{
				case 1:
						//Si se presiona B3
					vTaskDelay(250/portTICK_RATE_MS);// Delay for 250 mseconds.
					    threshold++;
					Board_LED_Toggle(0);

						break;
				case 2:
					vTaskDelay(250/portTICK_RATE_MS);// Delay for 250 mseconds.
						//Si se presiona B4
						threshold--;
					Board_LED_Toggle(0);
						break;

				case 3:
					vTaskDelay(250/portTICK_RATE_MS);// Delay for 250 mseconds.
						//Si se presiona B3 y B4 juntos!
						flag_cambio_ref=1;
						Board_LED_Toggle(0);
						break;
			}


	}
}




/*************************************************************************/
/* Tarea para INCLINOMETRO */
/*************************************************************************/
static void vINCLINOMETRO(void *pvParameters) {

	float angulo_partedecimal;
	int angulo_ENTERO;
	char Cadena_angulo[2];
	offset = 0;

	portTickType xLastWakeTime;

	/* xTaskGetTickCount() devuelve la cuenta de ticks actuales -> Referencia inicial */
	xLastWakeTime = xTaskGetTickCount();
	while (1) {

		presentacion_threshold();
		presentacion_referencia();

		for(int i=0; i<100;i++){
	    	complementaryFilter(&pitchAngle, &rollAngle);
	    	vTaskDelayUntil( &xLastWakeTime, ( 5 / portTICK_RATE_MS ) );    //Fija la cantidad de ticks exactos a partir de la refencia inicial, xLastWakeTime se va actualizando =>     Ideal para tareas periodicas
		}


		rollAngle=rollAngle + DESVIO;

		    	  //Si la parte decimal es > 0.5º: Ej.: 15.7º, entonces tomamos como angulo resultante: 16º
	    	         //Si la parte decimal es < 0.5º: Ej.: 15.3º, entonces tomamos como angulo resultante: 15º
                    angulo_ENTERO=(int) rollAngle;
	    	        angulo_partedecimal=(float) rollAngle - angulo_ENTERO;
       	            if (fabs(angulo_partedecimal>0.5))
	    	            angulo_ENTERO = angulo_ENTERO + 1;




       	            if(flag_cambio_ref == 1){
       	            	offset = angulo_ENTERO;
       	            	referencia = angulo_ENTERO;
       	            	flag_cambio_ref=0;
       	            	limpiarpantalla();
       	            }

       	            angulo_ENTERO = angulo_ENTERO - offset;

	    	          DEBUGOUT(" _______________\r\n");

	      	          //Transformo los valores enteros del angulo resultante a ASCII.
	    	          sprintf(Cadena_angulo,"%d",angulo_ENTERO);

	    	          presentacion_angulo(Cadena_angulo);


	     	          DEBUGOUT("| Roll : %d   \r\n",angulo_ENTERO);
	      	          DEBUGOUT("|_______________\r\n\r\n");
	      	          DEBUGOUT("| Ref. : %d   \r\n",referencia);
	      	          DEBUGOUT("|_______________\r\n\r\n");

	      	        if(angulo_ENTERO>threshold || angulo_ENTERO<-threshold)
	      	        	Chip_GPIO_SetPinState(LPC_GPIO, 2, BUZZ, 1);
	      	        else
	      	        	Chip_GPIO_SetPinState(LPC_GPIO, 2, BUZZ, 0);
	      	          vTaskDelay(250/portTICK_RATE_MS);// Delay for 250 mseconds.

	}
}




/*************************************************************************/
/*******************************  M A I N  *******************************/
/*************************************************************************/
int main(void) {

	SystemCoreClockUpdate();
	Board_Init();
	Board_LED_Set(0, true);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, BUZZ, 0);

	i2c_app_init(I2C0, SPEED_400KHZ);

	delay_ms_init(3);

	inicializacion_lcd();

	 delay_ms_init(1);
     whoAmI();                           // Communication test: WHO_AM_I register reading
 	 calibrate(accelBias,gyroBias);      // Calibrate MPU6050 and load biases into bias registers
	 DEBUGOUT("Calibration is completed. \r\n");
	 delay_ms_init(1);
	 init();                             // Initialize the sensor
	 delay_ms_init(1);
	 DEBUGOUT("MPU6050 is initialized for operation.. \r\n\r\n");
	 delay_ms_init(500);


	/* Tarea para INCLINOMETRO tarea periodica*/
	xTaskCreate(vINCLINOMETRO, (signed char *) "vINCLINOMETRO",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY +1UL),&xINCLINOMETRO);

	xTaskCreate(vENTRADA, (signed char *) "vENTRADA",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY +1UL),
				NULL);

	NVIC_SetPriority(IRQ_INCLINOMETRO, 1);
	NVIC_EnableIRQ(IRQ_INCLINOMETRO);

	// Arranco el scheduler
	vTaskStartScheduler();

	for( ;; );
	return 0;
}


void delay_lcd(unsigned int tiempo)
{
	vTaskDelay(tiempo/portTICK_RATE_MS);  //tiempo [ms]
}



void complementaryFilter(float* pitch, float* roll)
{
    /* Get actual acc value */
    readAccelData(accelData);
    getAres();
    ax = accelData[0]*aRes - accelBias[0];
    ay = accelData[1]*aRes - accelBias[1];
    az = accelData[2]*aRes - accelBias[2];

    /* Get actual gyro value */
    readGyroData(gyroData);
    getGres();
    gx = gyroData[0]*gRes;  // - gyroBias[0];      // Results are better without extracting gyroBias[i]
    gy = gyroData[1]*gRes;  // - gyroBias[1];
    gz = gyroData[2]*gRes;  // - gyroBias[2];

    float pitchAcc, rollAcc;

    /* Integrate the gyro data(deg/s) over time to get angle */
    *pitch += gx * dt;  // Angle around the X-axis
    *roll -=  gy * dt;  // Angle around the Y-axis

    /* Turning around the X-axis results in a vector on the Y-axis
    whereas turning around the Y-axis results in a vector on the X-axis. */
    pitchAcc = atan2f(accelData[1], accelData[2])*180/PI;
    rollAcc  = atan2f(accelData[0], accelData[2])*180/PI;

    /* Apply Complementary Filter */
    *pitch = *pitch * 0.98 + pitchAcc * 0.02;
    *roll  = *roll  * 0.98 + rollAcc  * 0.02;
}

void calibrate(float* dest1, float* dest2)
{
    uint8_t data[12];       // data array to hold acc and gyro x,y,z data
    uint16_t fifo_count, packet_count, count;
    int32_t accel_bias[3] = {0,0,0};
    int32_t gyro_bias[3] = {0,0,0};
    float aRes = 2.0/32768.0;
    float gRes = 250.0/32768.0;
    uint16_t accelsensitivity = 16384; // = 1/aRes = 16384 LSB/g
    //uint16_t gyrosensitivity = 131;    // = 1/gRes = 131 LSB/dps

    reset();     // Reset device

    /* Get stable time source */
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);    // PLL with X axis gyroscope reference is used to improve stability
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);    // Disable accel only low power mode
    delay_ms_init(2);

    /* Configure device for bias calculation */
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x04);    // Reset FIFO
    delay_ms_init(15);

    /* Configure accel and gyro for bias calculation */
    writeByte(MPU6050_ADDRESS, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity

    /* Configure FIFO to capture accelerometer and gyro data for bias calculation */
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable accelerometer and gyro for FIFO  (max size 1024 bytes in MPU-6050)
    delay_ms_init(80);                                    // Sample rate is 1 kHz, accumulates 80 samples in 80 milliseconds.
    // accX: 2 byte, accY: 2 byte, accZ: 2 byte. gyroX: 2 byte, gyroY: 2 byte, gyroZ: 2 byte.   12*80=960 byte < 1024 byte

    /* At end of sample accumulation, turn off FIFO sensor read */
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);             // Disable FIFO
    readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]);  // Read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;                          // The number of sets of full acc and gyro data for averaging. packet_count = 80 in this case

    for(count=0; count<packet_count; count++)
    {
        int16_t accel_temp[3]={0,0,0};
        int16_t gyro_temp[3]={0,0,0};
        readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging

        /* Form signed 16-bit integer for each sample in FIFO */
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        /* Sum individual signed 16-bit biases to get accumulated signed 32-bit biases */
        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }

    /* Normalize sums to get average count biases */
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    /* Remove gravity from the z-axis accelerometer bias calculation */
    if(accel_bias[2] > 0) {accel_bias[2] -= (int32_t) accelsensitivity;}
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    /* Output scaled accelerometer biases for manual subtraction in the main program */
    dest1[0] = accel_bias[0]*aRes;
    dest1[1] = accel_bias[1]*aRes;
    dest1[2] = accel_bias[2]*aRes;

    /* Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup */
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    /* Push gyro biases to hardware registers */
    writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
    writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
    writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
    writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

    /* Construct gyro bias in deg/s for later manual subtraction */
    dest2[0] = gyro_bias[0]*gRes;
    dest2[1] = gyro_bias[1]*gRes;
    dest2[2] = gyro_bias[2]*gRes;
}


void readAccelData(int16_t* dest)
{
    uint8_t rawData[6];  // x,y,z acc data
    readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);   // read six raw data registers sequentially and write them into data array

    /* Turn the MSB LSB into signed 16-bit value */
    dest[0] = (int16_t)(((int16_t)rawData[0]<<8) | rawData[1]);  // ACCEL_XOUT
    dest[1] = (int16_t)(((int16_t)rawData[2]<<8) | rawData[3]);  // ACCEL_YOUT
    dest[2] = (int16_t)(((int16_t)rawData[4]<<8) | rawData[5]);  // ACCEL_ZOUT
}

void readGyroData(int16_t* dest)
{
    uint8_t rawData[6];  // x,y,z gyro data
    readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);   // read the six raw data registers sequentially and write them into data array

    /* Turn the MSB LSB into signed 16-bit value */
    dest[0] = (int16_t)(((int16_t)rawData[0]<<8) | rawData[1]);  // GYRO_XOUT
    dest[1] = (int16_t)(((int16_t)rawData[2]<<8) | rawData[3]);  // GYRO_YOUT
    dest[2] = (int16_t)(((int16_t)rawData[4]<<8) | rawData[5]);  // GYRO_ZOUT
}

void reset()
{
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);  // set bit7 to reset the device
    delay_ms_init(100);    // delay_ms_init 100 ms to stabilize
}

// Initializes MPU6050 with the following config:
// PLL with X axis gyroscope reference
// Sample rate: 200Hz for gyro and acc
// Interrupts are disabled
void init()
{
    /* Wake up the device */
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  // wake up the device by clearing the sleep bit (bit6)
    delay_ms_init(100); // delay_ms_init 100 ms to stabilize

    /* Get stable time source */
    // PLL with X axis gyroscope reference is used to improve stability
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);

    /* Configure Gyroscope and Accelerometer */
    // Disable FSYNC, acc bandwidth: 44 Hz, gyro bandwidth: 42 Hz
    // Sample rates: 1kHz, maximum delay_ms_init: 4.9ms (which is pretty good for a 200 Hz maximum rate)
    writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

    /* Set sample rate = gyroscope output rate/(1+SMPLRT_DIV) */
    // SMPLRT_DIV=4 and sample rate=200 Hz (compatible with config above)
    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);

    /* Accelerometer configuration */
    uint8_t temp = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, temp & ~0xE0);      // Clear self-test bits [7:5]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, temp & ~0x18);      // Clear AFS bits [4:3]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, temp | Ascale<<3);  // Set full scale range

    /* Gyroscope configuration */
    temp = readByte(MPU6050_ADDRESS, GYRO_CONFIG);
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, temp & ~0xE0);      // Clear self-test bits [7:5]
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, temp & ~0x18);      // Clear FS bits [4:3]
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, temp | Gscale<<3);  // Set full scale range
}

// Communication test: WHO_AM_I register reading
void whoAmI()
{
    uint8_t whoAmI = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);   // Should return 0x68
    delay_ms_init(1);

    if(whoAmI==0x68)
    {
    	DEBUGOUT("MPU6050 is online... \r\n");
    }
    else
    {
    	DEBUGOUT("Could not connect to MPU6050 \r\nCheck the connections... \r\n");
    }
    DEBUGOUT("I AM 0x%x \r\n",whoAmI);
}


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	uint8_t data_write[2];
    data_write[0]=subAddress;           // I2C sends MSB first. Namely  >>|subAddress|>>|data|
    data_write[1]=data;
    //i2c.write(address,data_write,2,0);  // i2c.write(int address, char* data, int length, bool repeated=false);
    Chip_I2C_MasterSend(i2cDev, address, data_write, 2);
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data_read[1];  // will store the register data
	uint8_t data_write[1];
    data_write[0]=subAddress;
    //i2c.write(address,data_write,1,1);  // have not stopped yet
    //i2c.read(address,data_read,1,0);    // read the data and stop
    Chip_I2C_MasterSend(i2cDev, address, data_write, 1);
    Chip_I2C_MasterRead(i2cDev, address, data_read, 1);
	return data_read[0];
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t byteNum, uint8_t* dest)
{
	uint8_t data[14],data_write[1];
    data_write[0]=subAddress;
    Chip_I2C_MasterSend(i2cDev, address, data_write, 1);
    Chip_I2C_MasterRead(i2cDev, address, data, byteNum);

    for(int i=0;i<byteNum;i++)         // equate the addresses
        dest[i]=data[i];
}



//Rutina que se encarga de muestrear
//los botones B3 y B4 de la placa.
//Retorna:
//			0 -> no hay botón
//			1 -> se pulsó B3
//			2 -> se pulsó B4
//			3 -> se pulsaron B3 y B4
int botones(void)
{
	static int estado=0;
	int tmp;
	int ret=0;
	portTickType xLastWakeTime;

	switch (estado)
	{
		case 0:
				tmp=Chip_GPIO_ReadPortBit(LPC_GPIO, 0, B3);
				/* xTaskGetTickCount() devuelve la cuenta de ticks actuales -> Referencia inicial */
				xLastWakeTime = xTaskGetTickCount();
				vTaskDelayUntil( &xLastWakeTime, ( DEBOUNCE / portTICK_RATE_MS ) );
				if(tmp==false)
				{
					tmp=Chip_GPIO_ReadPortBit(LPC_GPIO, 2, B4);
					/* xTaskGetTickCount() devuelve la cuenta de ticks actuales -> Referencia inicial */
				    xLastWakeTime = xTaskGetTickCount();
					vTaskDelayUntil( &xLastWakeTime, ( DEBOUNCE / portTICK_RATE_MS ) );
					if(tmp==false)
					{//se oprimieron los dos botones!!
						ret=3;
						break;
					}
					else
						ret=1; //se oprimio solo B3
					 	break;
				}
				else if (tmp==true)
				{
					estado=1;
				}
				break;

		case 1:
				tmp=Chip_GPIO_ReadPortBit(LPC_GPIO, 2, B4);
				/* xTaskGetTickCount() devuelve la cuenta de ticks actuales -> Referencia inicial */
				xLastWakeTime = xTaskGetTickCount();
				vTaskDelayUntil( &xLastWakeTime, ( DEBOUNCE / portTICK_RATE_MS ) );
				if(tmp==false)
				{
					tmp=Chip_GPIO_ReadPortBit(LPC_GPIO, 0, B3);
					/* xTaskGetTickCount() devuelve la cuenta de ticks actuales -> Referencia inicial */
					xLastWakeTime = xTaskGetTickCount();
					vTaskDelayUntil( &xLastWakeTime, ( DEBOUNCE / portTICK_RATE_MS ) );
					if(tmp==false)
					{//se oprimieron los dos botones!!
						ret=3;
						break;
					}
					else
						ret=2; // se oprimio solo B4
				}
				else if (tmp==true)
				{
					estado=0;
				}
				break;
	}
	return ret;
}


void presentacion_threshold(void)
{
 char *p;
 char Cadena_angulo[2];
 char ref[]="THRESHOLD=";
 char i;
 p=ref;
 goto_xy(0,0);
 while(*p)
 {
  imprimecaracter(p);
  p++;
 }
 //Transformo los valores enteros del angulo threshold a ASCII.
 sprintf(Cadena_angulo,"%d",threshold);
 p=Cadena_angulo;
 goto_xy(0,10);
  while((*p)!=0)
  {
   imprimecaracter(p);
   p++;
  }
  for(i=0;i<4;i++,p++)
  {
   *p=' ';
   imprimecaracter(p);
  }

}

void presentacion_angulo(char *CAD_ANGULO)
{
 char *p;
 char i;
 char ref[]="ROLL=";
 p=ref;
 goto_xy(1,0);
 while(*p)
 {
  imprimecaracter(p);
  p++;
 }
 //Transformo los valores enteros del angulo threshold a ASCII.
 p=CAD_ANGULO;
 goto_xy(1,5);
  while((*p)!=0)
  {
   imprimecaracter(p);
   p++;
  }
  for(i=0;i<1;i++,p++)
  {
     *p=' ';
     imprimecaracter(p);
  }
}

void presentacion_referencia(void)
{
 char *p;
 char Cadena_angulo[2];
 char ref[]="REF=";
 char i;
 p=ref;
 goto_xy(1,9);
 while(*p)
 {
  imprimecaracter(p);
  p++;
 }
 //Transformo los valores enteros del angulo threshold a ASCII.
 sprintf(Cadena_angulo,"%d",referencia);
 p=Cadena_angulo;
 goto_xy(1,13);
  while((*p)!=0)
  {
   imprimecaracter(p);
   p++;
  }

  for(i=0;i<2;i++,p++)
  {
   *p=' ';
   imprimecaracter(p);
  }
}

