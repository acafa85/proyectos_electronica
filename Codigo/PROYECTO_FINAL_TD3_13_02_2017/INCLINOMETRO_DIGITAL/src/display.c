#include "FreeRTOS.h"
#include "stdio.h"
#include "lpc_types.h"
#include "display.h"
#include "string.h"
#include "stdio.h"



#define CANT_MUESTRAS 20

// FUNCIONES DEL DISPLAY

LPC_GPIO_T *pGPIO;

void inicializacion_lcd(void)
{
 //Inicio de inicializacion del lcd
 delay_ms_init(50);
 instruccion_init(0x38, 3);		//Primera instruccion para configurarlo en 8 bits con 2 líneas
 instruccion_init(0x0C, 3);		//display en ON, con cursor en OFF
 instruccion_init(0x01, 3);     	//Display clear
 instruccion_init(0x06, 3);
 //Fin de Inicializacion del lcd
}


void limpiarpantalla (void)
{
	instruccion(0x01,3);
}

void goto_xy(unsigned char fila, unsigned char columna)
{
 char dir;
 if (!fila)
 {
 		  dir= 0x80 + columna;
   		  instruccion (dir,1); //fila 0
 }
 else
 {
 		  dir= 0xC0 + columna; //fila 1
 		  instruccion (dir,1);
 }
}

void instruccion(unsigned int inst, unsigned int tiempo)
{
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 2, 0x000000FF);

	Chip_GPIO_SetPinState(LPC_GPIO, 2, RS, 0);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, RW, 0);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, E, 1);


	delay_lcd(1);

	// mando el dato

	Chip_GPIO_SetPortOutHigh(LPC_GPIO, 2, inst);

	Chip_GPIO_SetPinState(LPC_GPIO, 2, E, 0);

	delay_lcd(tiempo);
}

void escribestring(char * p)
{
 unsigned int columna=0;
 unsigned int dir;
 limpiarpantalla();
 while(*p)
 {
  imprimecaracter(p);
  columna++;
  p++;
  if (columna==16)
  {
   dir= 0xC0; //Paso a fila 1, columna 0
   instruccion (dir,1);
   columna=0;
  }
 }
}

void imprimecaracter(char *z)
{
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 2, 0x000000FF);

	Chip_GPIO_SetPinState(LPC_GPIO, 2, RS, 1);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, RW, 0);

	Chip_GPIO_SetPinState(LPC_GPIO, 2, E, 1);

	delay_lcd(3);


	Chip_GPIO_SetPortOutHigh(LPC_GPIO, 2, *z);

    Chip_GPIO_SetPinState(LPC_GPIO, 2, E, 0);

    delay_lcd(3);
}


void Display_angulo_ENTERO(char *Cadena_angulo)
{
 char *p=Cadena_angulo;
 goto_xy(1,0);
 char i;
 while((*p)!=0)
 {
  imprimecaracter(p);
  p++;
 }
 goto_xy(1,3);
   for(i=0;i<13;i++,p++)
  {
   *p=' ';
   imprimecaracter(p);
  }
}

void Display(char *Cadena)
{
 int i;
 char *p=Cadena;
 int FIN=0;
 goto_xy(0,7);
 while(!FIN)
 {
  imprimecaracter(p);
  p++;
  if((*p)=='.')
  {
   for(i=0;i<2;i++,p++) imprimecaracter(p);
   p++;
   *p='[';
   imprimecaracter(p);
   p++;
   *p='g';
   imprimecaracter(p);
   p++;
   *p=']';
   imprimecaracter(p);

   FIN=1;
  }
 }
}


void instruccion_init(unsigned int inst, unsigned int tiempo)
{
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 2, 0x000000FF);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, RS, 0);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, RW, 0);
	Chip_GPIO_SetPinState(LPC_GPIO, 2, E, 1);


	delay_ms_init(1);

	// mando el dato
	Chip_GPIO_SetPortOutHigh(LPC_GPIO, 2, inst);

	Chip_GPIO_SetPinState(LPC_GPIO, 2, E, 0);

	delay_ms_init(tiempo);
}

void delay_ms_init (unsigned int ms)
{
 uint16_t delay;
 volatile uint32_t i;
 for (delay = ms; delay >0 ; delay--)
//1ms loop with -Os optimisation
  {
  for (i=3500; i >0;i--){};
  }
}

