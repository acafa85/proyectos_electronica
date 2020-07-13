#define RS  10 //puerto 2
#define RW  11 //puerto 2
#define E   12 //puerto 2

#define D0  0  //puerto 2
#define D1  1  //puerto 2
#define D2  2  //puerto 2
#define D3  3  //puerto 2
#define D4  4  //puerto 2
#define D5  5  //puerto 2
#define D6  6  //puerto 2
#define D7  7  //puerto 2



void delay_lcd(unsigned int tiempo);
void instruccion(unsigned int inst, unsigned int tiempo);
void goto_xy(unsigned char fila, unsigned char columna);
void escribestring(char * p);
void imprimecaracter(char * z);
void presentacion_threshold(void);
void presentacion_referencia(void);
void Display(char *);
void inicializacion_lcd(void);
void limpiarpantalla(void);
void instruccion_init(unsigned int inst, unsigned int tiempo);

void delay_ms_init (unsigned int ms);

void Display_angulo_ENTERO(char *);

void presentacion_angulo(char *);
