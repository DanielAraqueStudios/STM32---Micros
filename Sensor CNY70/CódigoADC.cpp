#include <stdio.h>
#include "stm32f7xx.h"

// Definición de pines para los segmentos (A-G y DP)
// Se utilizan los pines PE0-PE7 para los segmentos
#define SEG_A_PIN  0
#define SEG_B_PIN  1
#define SEG_C_PIN  2
#define SEG_D_PIN  3
#define SEG_E_PIN  4
#define SEG_F_PIN  5
#define SEG_G_PIN  6
#define SEG_DP_PIN 7

// Definición de pines para los dígitos (D1-D4)
// En STM32, los números del display son:
#define DIGIT_1_PIN 0  // Dígito más a la izquierda (millares)
#define DIGIT_2_PIN 1  // Segundo dígito desde la izquierda (centenas)
#define DIGIT_3_PIN 2  // Tercer dígito desde la izquierda (decenas)
#define DIGIT_4_PIN 3  // Dígito más a la derecha (unidades)

// Variable para el contador
volatile uint16_t counter = 0;

// Variable para el voltaje leído
volatile uint16_t voltage = 0; // Corregir el tipo de dato a uint16_t
volatile double volts = 0; // Variable para almacenar el voltaje convertido

// Arreglo con los valores para mostrar cada número (0-9) en el display 7 segmentos
// Para displays de CÁTODO COMÚN, los segmentos activos están en 1 (lógica positiva)
const uint8_t seven_segment_digits[10] = {
    0b00111111,  // 0: Segmentos A, B, C, D, E, F encendidos
    0b00000110,  // 1: Segmentos B, C encendidos
    0b01011011,  // 2: Segmentos A, B, D, E, G encendidos
    0b01001111,  // 3: Segmentos A, B, C, D, G encendidos
    0b01100110,  // 4: Segmentos B, C, F, G encendidos
    0b01101101,  // 5: Segmentos A, C, D, F, G encendidos
    0b01111101,  // 6: Segmentos A, C, D, E, F, G encendidos
    0b00000111,  // 7: Segmentos A, B, C encendidos
    0b01111111,  // 8: Segmentos A, B, C, D, E, F, G encendidos
    0b01101111   // 9: Segmentos A, B, C, D, F, G encendidos
};




void adc(){

    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3); //enable clock for port A,B,C ,D

    RCC->APB2ENR|=(1<<8)|(1<<9)|(1<<10); //ENABLE ADC 1,2 AND 3 CLOCK

    // A PORTS AS ADC
    /*MODER 0 FOR 8 BITS ,4 MHZ, 112 CYCLES , ALLINGMENT RIGHT 
    0,3-  1V----->TURN ON BLUE LED
    1,3-  2V----->TURN ON GREEN LED
    2-    3V----->TURN ON RED LED
    */
    GPIOA->MODER|=(1<<1)|(1<<0); //PA0 as analog mode
    GPIOA->PUPDR|=(1<<1); //PA0 as pull down mode

// PORTS FOR A0 ADC CONFIGURATION
    ADC->CCR&= ~(0b11<<16); //F= 8MHZ
    ADC1->CR1&= ~(0b11<<24);//12 bit resolution
    ADC1->CR2|=(1<<0)|(1<<10); // turn on ADC & set end of conversion
    ADC1->CR2 &= ~(1<<11); //right alignment
    
    ADC1->SMPR2|=(0b111<<0); //480 cycles
    
    ADC1->SQR3 &= ~(0b11111<<0); //clear channel
    ADC1->SQR3|=(0b00000<<0); //channel 0


}







// Función para inicializar el sistema
void MySystemInit(void) {
    // Habilitar el reloj para GPIOD y GPIOE
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;


    
    // Configurar los pines de GPIOE (segmentos) como salidas push-pull con velocidad alta
    GPIOE->MODER &= ~(0xFFFF);          // Limpiar los bits de los pines PE0-PE7
    GPIOE->MODER |= 0x5555;             // Establecer modo salida (01) para pines PE0-PE7
    GPIOE->OTYPER &= ~(0xFF);           // Configurar como push-pull (0) para mejor driving
    GPIOE->OSPEEDR |= 0xAAAA;           // Velocidad alta (10) para reducir parpadeo
    
    // Configurar los pines de GPIOD (dígitos) como salidas push-pull con velocidad alta
    // Los pines de dígitos deben poder entregar suficiente corriente (sink)
    GPIOD->MODER &= ~(0xFF);            // Limpiar los bits de los pines PD0-PD3
    GPIOD->MODER |= 0x55;               // Establecer modo salida (01) para pines PD0-PD3
    GPIOD->OTYPER &= ~(0xF);            // Configurar como push-pull (0)
    GPIOD->OSPEEDR |= 0xAA;             // Velocidad alta (10) para mejor rendimiento

    // Inicializar TIM2 para el multiplexado rápido (actualización aproximadamente cada 0.25ms)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Habilitar reloj para TIM2
    
    // Configurar TIM2 para generar interrupciones cada 0.25ms para multiplexado
    // Reloj del sistema típicamente a 216MHz para STM32F7, con preescalador de 108-1 = 2MHz
    TIM2->PSC = 108 - 1;                // Preescalador para obtener 2MHz (216MHz/108)
    TIM2->ARR = 500 - 1;                // Auto-reload para 0.25ms (500 ciclos a 2MHz)
    TIM2->DIER |= TIM_DIER_UIE;         // Habilitar interrupción de actualización
    TIM2->CR1 |= TIM_CR1_CEN;           // Iniciar Timer2

    // Inicializar TIM3 para actualizar el contador cada 500ms
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Habilitar reloj para TIM3
    
    // Configurar TIM3 para generar interrupciones cada 500ms
    TIM3->PSC = 10800 - 1;              // Prescaler para obtener 20kHz (216MHz/10800)
    TIM3->ARR = 10000 - 1;              // Auto-reload para 500ms (10000 ciclos a 20kHz)
    TIM3->DIER |= TIM_DIER_UIE;         // Habilitar interrupción de actualización
    TIM3->CR1 |= TIM_CR1_CEN;           // Iniciar Timer3
    
    // Configurar NVIC para las interrupciones de los temporizadores
    NVIC_EnableIRQ(TIM2_IRQn);          // Habilitar interrupción TIM2 para multiplexado
    NVIC_EnableIRQ(TIM3_IRQn);          // Habilitar interrupción TIM3 para actualización del contador
    NVIC_SetPriority(TIM2_IRQn, 1);     // Alta prioridad para multiplexado (evitar parpadeo)
    NVIC_SetPriority(TIM3_IRQn, 2);     // Menor prioridad para actualización del contador
}

// Manejador de interrupción para TIM2 (multiplexado de dígitos)
extern "C" void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;        // Limpiar flag de interrupción
        
        // SOLUCIÓN FINAL SIMPLIFICADA: SOLO EL DISPLAY DERECHO PARA 0-9
        
        // Apagar todo primero
        GPIOD->ODR = 0;
        GPIOE->ODR = 0;
        
        // Forzar que solo se muestre el display de la derecha con el valor del contador
        if (counter < 10) {
            // Si es un valor de 0-9, SOLO mostrar en el display de la derecha
            GPIOE->ODR = seven_segment_digits[counter % 10];
            // DISPLAY DERECHO = PIN 3 = DIGIT_4_PIN
            GPIOD->ODR = 0x08;  // Binario 1000 = activar solo pin 3
        }
        else {
            // Para números de 10+, usar un contador estático para multiplexar
            static uint8_t display_index = 0;
            
            switch (display_index) {
                case 0:
                    // Display derecho (unidades)
                    GPIOE->ODR = seven_segment_digits[counter % 10];
                    GPIOD->ODR = 0x08;  // Encender solo pin 3
                    break;
                    
                case 1:
                    if (counter >= 10) {
                        // Display decenas
                        GPIOE->ODR = seven_segment_digits[(counter / 10) % 10];
                        GPIOD->ODR = 0x04;  // Encender solo pin 2
                    }
                    break;
                    
                case 2:
                    if (counter >= 100) {
                        // Display centenas
                        GPIOE->ODR = seven_segment_digits[(counter / 100) % 10];
                        GPIOD->ODR = 0x02;  // Encender solo pin 1
                    }
                    break;
                    
                case 3:
                    if (counter >= 1000) {
                        // Display millares 
                        GPIOE->ODR = seven_segment_digits[(counter / 1000) % 10];
                        GPIOD->ODR = 0x01;  // Encender solo pin 0
                    }
                    break;
            }
            
            // Avanzar al siguiente dígito
            display_index = (display_index + 1) % 4;
        }
    }
}

// Manejador de interrupción para TIM3 (actualización del contador)
extern "C" void TIM3_IRQHandler(void) {
    // Verificar si es una interrupción de actualización
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;        // Limpiar flag de interrupción
        
        // Incrementar el contador y limitarlo a 9999
        counter++;
        if (counter >= 10000) {
            counter = 0;
        }
    }
}

int main(void) {
    // Llamar a nuestra función de inicialización personalizada
    MySystemInit();




    
    


    // Iniciar con un valor bajo para pruebas
    counter = 0;
    
    // Inicializar los estados de salida
    GPIOE->ODR = 0;                     // Apagar todos los segmentos
    GPIOD->ODR = 0;                     // Inicializar todos los dígitos como apagados
    
    // Configurar velocidad de conteo
    TIM3->PSC = 10800 - 1;              // Mantener el mismo prescaler
    TIM3->ARR = 4000 - 1;               // Ajustar a 200ms

    // Bucle principal vacío - todo el trabajo lo hacen las interrupciones
    while (1) {
        // No se necesita hacer nada aquí, el conteo y visualización
			
			
			    //////////DANIEL´S CODE//////////////////

    adc(); // Inicializar ADC 

    ADC1->CR2|=(1<<30); //start conversion

    while(((ADC1->SR & (1<<1)) >> 1) == 0){

        ADC2->SR &= ~(1<<1);

    } //wait for conversion to complete (EOC flag)
        
        voltage = ADC1->DR; //read value
        
        volts = (voltage * 3.3) / 4095; //convert to volts
        
    



    //////////DANIEL´S CODE////////////////// 
			
			
			
        // se manejan automáticamente por las interrupciones de los temporizadores
    }
}
