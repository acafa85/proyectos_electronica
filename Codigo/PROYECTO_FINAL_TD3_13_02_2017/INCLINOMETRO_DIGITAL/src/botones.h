#define		TRUE		1
#define		FALSE		0
#define		ENTRADA		0
#define		SALIDA		1
#define		B3			18  //puerto 0
#define		B4			13  //puerto 2

//Estados y tiempos del pulsador
#define		DEBOUNCE	50		//tiempo para el antirrebote (ms)
#define		SUELTO		0
#define		ESPEROBAJO	1
#define		PULSADO		2
#define		ESPEROALTO	3

//Prototipos

int boton(int);
int botones(void);
void delay_ms_init (unsigned int);
