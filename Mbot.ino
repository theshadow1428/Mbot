#include <Arduino_FreeRTOS.h>
#include "queue.h"
#include "semphr.h"
#include <MeMegaPi.h>

#define F_CPU 16000000UL
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define VELOCIDAD_RECTO 70
#define VELOCIDAD_GIRO 80
#define ANGULO_RECTO 90
#define ZONA_MUERTA 6
#define MAX_CORRECCION 135
#define GIRO_180_MS 3250
#define PERIODO_MOTORES_MS 20
#define BUFFER_SERIAL_LEN 8

QueueHandle_t angleQueue;
SemaphoreHandle_t semaforoImpacto;

MeMegaPiDCMotor motor_1(1);
MeMegaPiDCMotor motor_9(9);
MeMegaPiDCMotor motor_2(2);
MeMegaPiDCMotor motor_10(10);

volatile TickType_t ultimoImpacto = 0;

void vTaskSerial(void *pvParameters);
void vTaskMotores(void *pvParameters);
void detener(void);
void girarIzquierda(int16_t speed);
void girar_180(void);
void conducirSegunAngulo(int angulo);

int16_t parseAngulo(char *buffer)
{
    int16_t val = 0;
    uint8_t i = 0;

    while (buffer[i] >= '0' && buffer[i] <= '9')
    {
        val = (int16_t)(val * 10 + (buffer[i] - '0'));
        i++;
    }
    return val;
}

int16_t clamp16(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int16_t abs16(int16_t v)
{
    if (v < 0) return -v;
    return v;
}

int16_t map16(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
    return (int16_t)(out_min + (int32_t)(x - in_min) * (out_max - out_min) / (in_max - in_min));
}

void setup()
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0C = 0x06;
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
    DDRK &= ~(1 << PK3);
    PORTK |= (1 << PK3);
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT19);

    detener();

    angleQueue = xQueueCreate(10, sizeof(int16_t));

    semaforoImpacto = xSemaphoreCreateBinary();

    if ((angleQueue != NULL) && (semaforoImpacto != NULL))
    {
        xTaskCreate(vTaskSerial, "SERIAL TASK", 150, NULL, 1 ,NULL);

        xTaskCreate(vTaskMotores, "MOTORES TASK", 200, NULL, 2, NULL);

        sei();
    }
}

void vTaskSerial(void *pvParameters)
{
    char buffer[BUFFER_SERIAL_LEN];
    uint8_t idx = 0;
    int16_t angulo;
    BaseType_t qStatus;
    const TickType_t xTicksToWait = pdMS_TO_TICKS(10);
    (void)pvParameters;

    while (1)
    {
        if (UCSR0A & (1 << RXC0))
        {
            uint8_t byteRx = UDR0;

            if ((byteRx == '\n') || (byteRx == '\r'))
            {
                if (idx > 0)
                {
                    buffer[idx] = '\0';
                    angulo = parseAngulo(buffer);
                    qStatus = xQueueSend(angleQueue, &angulo, xTicksToWait);
                    (void)qStatus;
                    idx = 0;
                }
            }
            else if ((byteRx >= '0') && (byteRx <= '9'))
            {
                if (idx < (BUFFER_SERIAL_LEN - 1))
                {
                    buffer[idx++] = (char)byteRx;
                }
            }
            else
            {
                idx = 0;
            }
        }
        vTaskDelay(xTicksToWait);
    }
}

void vTaskMotores(void *pvParameters)
{
    int16_t angulo = ANGULO_RECTO;

    int16_t anguloRx;

    BaseType_t qStatus;

    const TickType_t periodo = pdMS_TO_TICKS(PERIODO_MOTORES_MS);

    const TickType_t xTicksToWait = pdMS_TO_TICKS(5);

    (void)pvParameters;

    while (1)
    {
        if (xSemaphoreTake(semaforoImpacto, 0) == pdPASS)
        {
            detener();
            vTaskDelay(pdMS_TO_TICKS(100));
            girar_180();
        }

        qStatus = xQueueReceive(angleQueue, &anguloRx, xTicksToWait);

        if (qStatus == pdPASS)
        {
            angulo = anguloRx;
        }

        conducirSegunAngulo(angulo);

        vTaskDelay(periodo);
    }
}

ISR(PCINT2_vect)
{
    TickType_t ahora = xTaskGetTickCountFromISR();

    if ((ahora - ultimoImpacto) > pdMS_TO_TICKS(200))
    {
        if (!(PINK & (1 << PK3)))
        {
            xSemaphoreGiveFromISR(semaforoImpacto, NULL);

            ultimoImpacto = ahora;
        }
    }
}

void detener(void)
{
    motor_1.run(0);
    motor_2.run(0);
    motor_9.run(0);
    motor_10.run(0);
}

void girarIzquierda(int16_t speed)
{
    motor_10.run(speed);
    motor_2.run(speed);
    motor_1.run(speed);
    motor_9.run(speed);
}

void girar_180(void)
{
    girarIzquierda(VELOCIDAD_GIRO);
    vTaskDelay(pdMS_TO_TICKS(GIRO_180_MS));
    detener();
    vTaskDelay(pdMS_TO_TICKS(150));
}

void conducirSegunAngulo(int angulo)
{
    angulo = clamp16(angulo, 0, 180);
    int error = angulo - ANGULO_RECTO;

    if (abs16(error) <= ZONA_MUERTA)
    {
        motor_10.run(-VELOCIDAD_RECTO);
        motor_2.run(-VELOCIDAD_RECTO);
        motor_1.run(VELOCIDAD_RECTO);
        motor_9.run(VELOCIDAD_RECTO);

        return;
    }

    int correccion = map16(error, -90, 90, -MAX_CORRECCION, MAX_CORRECCION);

    int velIzq = VELOCIDAD_RECTO + correccion;

    int velDer = VELOCIDAD_RECTO - correccion;

    velIzq = clamp16(velIzq, -255, 255);

    velDer = clamp16(velDer, -255, 255);

    motor_10.run(-velIzq);
    motor_2.run(-velIzq);
    motor_1.run(velDer);
    motor_9.run(velDer);
}

void loop()
{
}