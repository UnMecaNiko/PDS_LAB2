| /* Conexiones:
         PF0: SDA
         PF1: SCL
         MPU direccion:
             AD0 alto: 0x69
             AD0 bajo: 0x68

         PC10: Tx
         PC11: Rx
 */

// ** LIBRERIAS ***
#include <stdio.h>
//#include "STM32F7xx.h"
#include "math.h"

// ** VELOCIDADES ***
#define standar_mode 0x30420F13   // 100K Hz
#define fast_mode 0x10320309      // 400K Hz
#define fast_plus_mode 0x00200204 // 1M Hz

    float g = 9.80665;
float datta = 0;
int vector[6] = {}, aux, n, num;
float x[6] = {0, 0, 0, 0, 0, 0}, y[6] = {0, 0, 0, 0, 0, 0};
float x1[6] = {0, 0, 0, 0, 0, 0}, y1[6] = {0, 0, 0, 0, 0, 0};
float x2[6] = {0, 0, 0, 0, 0, 0}, y2[6] = {0, 0, 0, 0, 0, 0};
float x3[6] = {0, 0, 0, 0, 0, 0}, y3[6] = {0, 0, 0, 0, 0, 0};
float x4[6] = {0, 0, 0, 0, 0, 0}, y4[6] = {0, 0, 0, 0, 0, 0};
float x5[6] = {0, 0, 0, 0, 0, 0}, y5[6] = {0, 0, 0, 0, 0, 0};

float yn = 0, Yn = 0;
float yn1 = 0, Yn1 = 0;
float yn2 = 0, Yn2 = 0;
float yn3 = 0, Yn3 = 0;
float yn4 = 0, Yn4 = 0;
float yn5 = 0, Yn5 = 0;
float AFX, AFY, AFZ, GFX, GFY, GFZ;

float AFXB[6] = {0.000368782660913225, 0.00184391330456613, 0.00368782660913225, 0.00368782660913225, 0.00184391330456613, 0.000368782660913225};
float AFXA[5] = {-3.47889284435613, 5.00982620682061, -3.69953587956549, 1.39420141274668, -0.213797850496448};

float AFYB[6] = {0.00103001570366773, 0.00515007851833865, 0.0103001570366773, 0.0103001570366773, 0.00515007851833865, 0.00103001570366773};
float AFYA[5] = {-3.07592438993752, 4.02969061802066, -2.74853001277532, 0.967442805851792, -0.139718518642240};

float AFZB[6] = {0.000136635675470786, 0.000683178377353932, 0.00136635675470786, 0.00136635675470786, 0.000683178377353932, 0.000136635675470786};
float AFZA[5] = {-3.78205067623297, 5.83747836187128, -4.57774375976233, 1.81927233091800, -0.292583915178916};

float GFXB[6] = {0.000271926364218716, 0.00135963182109358, 0.00271926364218716, 0.00271926364218716, 0.00135963182109358, 0.000271926364218716};
float GFXA[5] = {-3.57986231458465, 5.27672475478631, -3.97529392234077, 1.52460525687643, -0.237472131082317};

float GFYB[6] = {0.00103001570366773, 0.00515007851833865, 0.0103001570366773, 0.0103001570366773, 0.00515007851833865, 0.00103001570366773};
float GFYA[5] = {-3.07592438993752, 4.02969061802066, -2.74853001277532, 0.967442805851792, -0.139718518642240};

float GFZB[6] = {0.00192107360788685, 0.00960536803943424, 0.0192107360788685, 0.0192107360788685, 0.00960536803943424, 0.00192107360788685};
float GFZA[5] = {-2.77471685134525, 3.38362254785068, -2.17547527726840, 0.728918082939890, -0.100874146724541};

// ** VARIABLES GLOBALES **
// Variables IMU 1

int16_t aceleracion_X;          // Variable donde almaceno la aceleracion X
float aceleracion_X_convertida; // Variable donde almacenare la aceleracion en m/s^2
int16_t aceleracion_Y;          // Variable donde almaceno la aceleracion Y
float aceleracion_Y_convertida; // Variable donde almacenare la aceleracion en m/s^2
int16_t aceleracion_Z;          // Variable donde almaceno la aceleracion Z
float aceleracion_Z_convertida; // Variable donde almacenare la aceleracion en m/s^2
int16_t gyroscopo_X;            // Variable donde almaceno el gyroscopo X
float gyroscopo_X_convertida;   // Variable donde almacenare el gyroscopo en rad/s
int16_t gyroscopo_Y;            // Variable donde almaceno el gyroscopo Y
float gyroscopo_Y_convertida;   // Variable donde almacenare el gyroscopo en rad/s
int16_t gyroscopo_Z;            // Variable donde almaceno el gyroscopo Z
float gyroscopo_Z_convertida;   // Variable donde almacenare el gyroscopo en rad/s

// Variables UART
short dato_recibido = 0;
short caracter1 = 0;
short caracter2 = 0; // Variables donde guardare la fragmentacion del valor a enviar
short caracter3 = 0;
short caracter4 = 0;
short caracter5 = 0;

// ** FUNCIONES ***
void escribir(char direccion_esclavo, char direccion_escribir, char dato)
{
    I2C2->CR2 |= I2C_CR2_AUTOEND;
    I2C2->CR2 &= ~I2C_CR2_RD_WRN;          // Modo escritura
    I2C2->CR2 |= (direccion_esclavo << 1); // Envio la direccion del esclavo
    I2C2->CR2 |= (2 << 16);                // Envio la cantidad de bytes de la comunicacion
    I2C2->CR2 |= I2C_CR2_START;            // Condicion de inicio
    while (!(I2C2->ISR & I2C_ISR_TXIS))
        ;
    I2C2->TXDR = direccion_escribir; // Envio la direccion del registro que quiero acceder
    while (!(I2C2->ISR & I2C_ISR_TXIS))
        ;
    I2C2->TXDR = dato; // Envio el dato a la direccion especificada anteriormente
    while (!(I2C2->ISR & I2C_ISR_TXE))
        ; // Verifico que envie la informacion correctamente
    while (!(I2C2->ISR & I2C_ISR_STOPF))
        ;                   // Tiempo de espera mientras el proceso se acaba
    I2C2->CR2 = 0x02000000; // CR2 lo vuelvo al inicio
}

char leer_datos(char direccion_esclavo, char direccion_leer)
{ // Me retorna un tipo de dato entero de 8 bits

    I2C2->CR2 &= ~I2C_CR2_RD_WRN;          // Modo Escritura para enviar la direccion del esclavo
    I2C2->CR2 |= (direccion_esclavo << 1); // Envio la direccion del esclavo
    I2C2->CR2 |= (1 << 16);                // Envio la cantidad de bytes que se transmitiran
    I2C2->CR2 |= I2C_CR2_AUTOEND;          // Activo auto end
    I2C2->CR2 |= I2C_CR2_START;            // Condicion de inicio
    while (!(I2C2->ISR & I2C_ISR_TXIS))
        ;
    I2C2->TXDR = direccion_leer; // Envio la direccion del registro a leer
    while (!(I2C2->ISR & I2C_ISR_TXE))
        ; // Verifico que todo se halla enviado correctamente
    while (!(I2C2->ISR & I2C_ISR_STOPF))
        ; // Espero mientras se detiene la comnucacion

    // Reset de la comunicacion
    I2C2->CR1 &= ~I2C_CR1_PE;
    while (I2C2->CR1 & I2C_CR1_PE)
        ;
    I2C2->CR1 |= I2C_CR1_PE; // Activo el periferico I2C

    I2C2->CR2 |= (direccion_esclavo << 1);
    I2C2->CR2 |= I2C_CR2_RD_WRN; // Modo lectura
    I2C2->CR2 |= (1 << 16);
    I2C2->CR2 |= I2C_CR2_START;
    while (!(I2C2->ISR & I2C_ISR_RXNE))
        ;                             // Verifico si se lleno el buffer de datos recibidos
    uint8_t informacion = I2C2->RXDR; // Obtengo la informacion del sensor
    while (!(I2C2->ISR & I2C_ISR_STOPF))
        ; // Espero hasta que se acabe la comunicacion
    I2C2->CR2 |= I2C_CR2_STOP;

    // Reset de la comunicacion
    I2C2->CR1 &= ~I2C_CR1_PE;
    while (I2C2->CR1 & I2C_CR1_PE)
        ;
    I2C2->CR1 |= I2C_CR1_PE; // Activo el periferico I2C
    return informacion;
}

void lectura(void)
{
    // Obtener medidas del sensor
    aceleracion_X = ((leer_datos(0x68, 0x3B) << 8) | leer_datos(0x68, 0x3C)); // Bits
    aceleracion_Y = ((leer_datos(0x68, 0x3D) << 8) | leer_datos(0x68, 0x3E)); // Bits
    aceleracion_Z = ((leer_datos(0x68, 0x3F) << 8) | leer_datos(0x68, 0x40)); // Bits

    gyroscopo_X = ((leer_datos(0x68, 0x43) << 8) | leer_datos(0x68, 0x44)); // Bits
    gyroscopo_Y = ((leer_datos(0x68, 0x45) << 8) | leer_datos(0x68, 0x46)); // Bits
    gyroscopo_Z = ((leer_datos(0x68, 0x47) << 8) | leer_datos(0x68, 0x48)); // Bits

    // Conversion a unidades de medida
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    aceleracion_X_convertida = ((((aceleracion_X * 2.0) / 32767.0) * g) + 10) * 100; // Calculo la aceleracion en 1G
    for (int i = 0; i < 6; i++)
    {
        yn = yn + x[i] * AFXB[i];
        Yn = Yn + y[i] * AFXA[i];
    }
    for (int i = 5; i >= 0; i--)
    {
        x[i] = x[i - 1];
        y[i] = y[i - 1];
    }
    x[0] = aceleracion_X_convertida;
    y[0] = yn - Yn;

    AFX = y[0];

    Yn = 0;
    yn = 0; // AFX =0;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    aceleracion_Y_convertida = ((((aceleracion_Y * 2.0) / 32767.0) * g) + 10) * 100; // Calculo la aceleracion en 1G
    for (int i = 0; i < 6; i++)
    {
        yn1 = yn1 + x1[i] * AFYB[i];
        Yn1 = Yn1 + y1[i] * AFYA[i];
    }
    for (int i = 5; i >= 0; i--)
    {
        x1[i] = x1[i - 1];
        y1[i] = y1[i - 1];
    }
    x1[0] = aceleracion_Y_convertida;
    y1[0] = yn1 - Yn1;

    AFY = y1[0];

    Yn1 = 0;
    yn1 = 0;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    aceleracion_Z_convertida = ((((aceleracion_Z * 2.0) / 32767.0) * g) + 10) * 100; // Calculo la aceleracion en 1G
    for (int i = 0; i < 6; i++)
    {
        yn2 = yn2 + x2[i] * AFZB[i];
        Yn2 = Yn2 + y2[i] * AFZA[i];
    }
    for (int i = 5; i >= 0; i--)
    {
        x2[i] = x2[i - 1];
        y2[i] = y2[i - 1];
    }
    x2[0] = aceleracion_Z_convertida;
    y2[0] = yn2 - Yn2;

    AFZ = y2[0];

    Yn2 = 0;
    yn2 = 0; // AFZ =0;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    gyroscopo_X_convertida = ((((gyroscopo_X * 250) / 32767.0)) + 250) * 10; // Calculo la aceleracion en grados/s^2
    for (int i = 0; i < 6; i++)
    {
        yn3 = yn3 + x3[i] * GFXB[i];
        Yn3 = Yn3 + y3[i] * GFXA[i];
    }
    for (int i = 5; i >= 0; i--)
    {
        x3[i] = x3[i - 1];
        y3[i] = y3[i - 1];
    }
    x3[0] = gyroscopo_X_convertida;
    y3[0] = yn3 - Yn3;

    GFX = y3[0];

    Yn3 = 0;
    yn3 = 0; // GFX =0;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    gyroscopo_Y_convertida = ((((gyroscopo_Y * 250) / 32767.0)) + 250) * 10; // Calculo la aceleracion en grados/s^2
    for (int i = 0; i < 6; i++)
    {
        yn4 = yn4 + x4[i] * GFYB[i];
        Yn4 = Yn4 + y4[i] * GFYA[i];
    }
    for (int i = 5; i >= 0; i--)
    {
        x4[i] = x4[i - 1];
        y4[i] = y4[i - 1];
    }
    x4[0] = gyroscopo_Y_convertida;
    y4[0] = yn4 - Yn4;

    GFY = y4[0];

    Yn4 = 0;
    yn4 = 0; // GFY =0;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    gyroscopo_Z_convertida = ((((gyroscopo_Z * 250) / 32767.0)) + 250) * 10; // Calculo la aceleracion en grados/s^2
    for (int i = 0; i < 6; i++)
    {
        yn5 = yn5 + x5[i] * GFZB[i];
        Yn5 = Yn5 + y5[i] * GFZA[i];
    }
    for (int i = 5; i >= 0; i--)
    {
        x5[i] = x5[i - 1];
        y5[i] = y5[i - 1];
    }
    x5[0] = gyroscopo_Z_convertida;
    y5[0] = yn5 - Yn5;

    GFZ = y5[0];

    Yn5 = 0;
    yn5 = 0; // GFZ =0;
}

void enviar_caracter(char caracter)
{ // Inicio de enviar un caracter
    UART4->TDR = caracter;
    while ((UART4->ISR &= 0x80) == 0)
    {
    }
} // Fin funcion de enviar un caracter

void dividir_datos(short resultado)
{ // Inicio de funcion de dividir datos
    caracter1 = resultado / 1000;
    caracter2 = (resultado / 100) % 10;
    caracter3 = (resultado % 100) / 10;
    caracter4 = (resultado % 10);
} // Fin funcion de dividir numeros

//**Enviar**
void enviar(int x, float datta)
{
    switch (x)
    {
    case 1: // 1
        enviar_caracter('X');
        break;
    case 2:                   // 2
        enviar_caracter('Y'); // 0978 0959 2036 2511 2509 2499
        break;
    case 3: // 3
        enviar_caracter('Z');
        break;
    case 4: // 4
        enviar_caracter('x');
        break;
    case 5: // 5
        enviar_caracter('y');
        break;
    case 6: // 6
        enviar_caracter('z');
        break;
    default:
        break;
    }
    dividir_datos(datta);
    enviar_caracter(caracter1 + 0x30);
    enviar_caracter(caracter2 + 0x30);
    enviar_caracter(caracter3 + 0x30);
    enviar_caracter(caracter4 + 0x30);
}

// *** INTERRUPCIONES ***
extern "C"
{
    void SysTick_Handler(void)
    {
        lectura();
        enviar(1, AFX);
        enviar(2, AFY);
        enviar(3, AFZ);
        enviar(4, GFX);
        enviar(5, GFY);
        enviar(6, GFZ);
        enviar_caracter('/');
        enviar_caracter('\n');

        //	enviar(1,aceleracion_X_convertida);
        //	enviar(2,aceleracion_Y_convertida);
        //	enviar(3,aceleracion_Z_convertida);
        //	enviar(4,gyroscopo_X_convertida);
        //	enviar(5,gyroscopo_Y_convertida);
        //	enviar(6,gyroscopo_Z_convertida);
        //	enviar_caracter('/');
        //	enviar_caracter('\n');
    }
}

// ** MAIN ****
int main(void)
{
    // * PUERTOS **
    RCC->AHB1ENR |= 0x24; // C y F

    // * PINES ****
    GPIOC->MODER |= 0x02;  // ALTERNATIVO pin 0, ALTERNATIVO pin 6
    GPIOC->AFR[0] |= 0x08; // Funcion alterna 8, Rx, pin 0 - Funcion alterna 2 Timer 3 canal 1 en el pin 6

    GPIOF->MODER |= 0x0A;   // FUNCION ALTERNANTE pin 0 y 1
    GPIOF->OTYPER |= 0x03;  // OPEN DRAIN pin 0 y 1
    GPIOF->PUPDR |= 0x05;   // PULL-UP pines 0 y 1
    GPIOF->OSPEEDR |= 0x0C; // HIGH-SPEED pin 0 y 1
    GPIOF->AFR[0] |= 0x44;  // FUNCION ALTERNANTE 4, PF1 (SCL) / PF0 (SDA)

    // ** UART ****
    //		NVIC_EnableIRQ(UART4_IRQn);

    RCC->AHB1ENR |= 0xFF;     // PUERTO B Y C
    RCC->APB1ENR |= 0x80001;  // Activo el reloj del UART 4
    GPIOC->MODER |= 0XA00000; // MODO ALTERNANTE PUERTO C
    GPIOC->AFR[1] |= 0X8800;  // SE DEFINE LA FUNCION ALTERANTE PINES P10 Y P11
    UART4->BRR = 0X683;       // VELOCIDAD DE 9600 BAUDIOS
    UART4->CR1 |= 0X2C;       //  HABILITO EL TRASMISOR EL RECEPTOR LA INTERUPCION
    UART4->CR1 |= 0X1;        // USAR ENABLE

    // NVIC_EnableIRQ(UART4_IRQn);
    //  * I2C ****
    RCC->APB1ENR |= 0x00400000; // Habilito el reloj del I2C2
    I2C2->TIMINGR |= standar_mode;
    I2C2->CR1 |= 0x01;          // Activo el periferico I2C
    escribir(0x68, 0x6B, 0x00); // Despertar IMU 2
                                // ****

    //*Systick**
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 200); // systic a 5 milis

    while (true)
    {
    }
}