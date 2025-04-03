#include <stdio.h>
#include "stm32f7xx.h"

// Define correct pin mapping based on test results
#define DIGIT_1_PIN 0  // Extremo izquierdo (PD0) - 0xFE activa este dígito
#define DIGIT_2_PIN 1  // Segundo desde izquierda (PD1) - 0xFD activa este dígito
#define DIGIT_3_PIN 2  // Tercero desde izquierda (PD2) - 0xFB activa este dígito
#define DIGIT_4_PIN 3  // Extremo derecho (PD3) - 0xF7 activa este dígito

// Variable para el contador (ahora hasta 9999)
volatile uint16_t counter = 0;

// Variable para controlar qué dígito se muestra (para multiplexación)
volatile uint8_t current_digit = 0;

// Valores para mostrar cada número (0-9) para display CÁTODO COMÚN
const uint8_t seven_segment_digits[10] = {
    0b00111111,  // 0: A, B, C, D, E, F 
    0b00000110,  // 1: B, C
    0b01011011,  // 2: A, B, D, E, G
    0b01001111,  // 3: A, B, C, D, G
    0b01100110,  // 4: B, C, F, G
    0b01101101,  // 5: A, C, D, F, G
    0b01111101,  // 6: A, C, D, E, F, G
    0b00000111,  // 7: A, B, C
    0b01111111,  // 8: A, B, C, D, E, F, G
    0b01101111   // 9: A, B, C, D, F, G
};

// Función para inicializar el sistema
void MySystemInit(void) {
    // Habilitar el reloj para GPIOD y GPIOE
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    
    // Configurar GPIOE (segmentos) como salidas
    GPIOE->MODER &= ~(0xFFFF);  // Limpiar bits para PE0-PE7
    GPIOE->MODER |= 0x5555;     // Establecer modo salida (01) para PE0-PE7
    
    // Configurar GPIOD (dígitos) como salidas
    GPIOD->MODER &= ~(0xFF);    // Limpiar bits para PD0-PD3
    GPIOD->MODER |= 0x55;       // Establecer modo salida (01) para PD0-PD3

    // Configurar TIM2 para el MULTIPLEXADO de dígitos (5ms por dígito = 200Hz)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 54 - 1;        // Preescalador para obtener 2MHz (216MHz/108)
    TIM2->ARR = 1000 - 1;       // Auto-reload para 5ms (1000 ciclos a 2MHz)
    TIM2->DIER |= TIM_DIER_UIE; // Habilitar interrupción de actualización
    TIM2->CR1 |= TIM_CR1_CEN;   // Iniciar Timer2

    // Configurar TIM3 para actualizar el CONTADOR (0.5 segundos)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 
    TIM3->PSC = 10800 - 1;      // Prescaler para obtener 20kHz
    TIM3->ARR = 1000 - 1;      // Auto-reload para 0.5 segundos
    TIM3->DIER |= TIM_DIER_UIE; // Habilitar interrupción de actualización
    TIM3->CR1 |= TIM_CR1_CEN;   // Iniciar Timer3
    
    // Configurar interrupciones
    NVIC_EnableIRQ(TIM2_IRQn);  // Para multiplexado de dígitos
    NVIC_EnableIRQ(TIM3_IRQn);  // Para actualización del contador
    
    // Prioridad más alta para el multiplexado (evita parpadeo)
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_SetPriority(TIM3_IRQn, 2);
}

// Manejador de interrupción para TIM2 (multiplexado de dígitos)
extern "C" void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;  // Limpiar flag de interrupción
        
        // Apagar todos los dígitos para evitar fantasmas
        GPIOD->ODR = 0xFF;  // Para cátodo común, 1 = apagado
        
        // Calcular qué dígito y valor mostrar
        uint16_t value = counter;
        uint8_t digit_value = 0;
        
        // Extraer el dígito correspondiente
        switch (current_digit) {
            case 0:  // Dígito de unidades (extremo derecho)
                digit_value = value % 10;
                GPIOE->ODR = seven_segment_digits[digit_value];
                GPIOD->ODR = 0xF7;  // Activar PD3 (extremo derecho)
                break;
                
            case 1:  // Dígito de decenas
                value /= 10;
                digit_value = value % 10;
                // Solo mostrar si el número es >= 10
                if (counter >= 10) {
                    GPIOE->ODR = seven_segment_digits[digit_value];
                    GPIOD->ODR = 0xFB;  // Activar PD2 (tercero desde izquierda)
                }
                break;
                
            case 2:  // Dígito de centenas
                value /= 100;
                digit_value = value % 10;
                // Solo mostrar si el número es >= 100
                if (counter >= 100) {
                    GPIOE->ODR = seven_segment_digits[digit_value];
                    GPIOD->ODR = 0xFD;  // Activar PD1 (segundo desde izquierda)
                }
                break;
                
            case 3:  // Dígito de millares (extremo izquierdo)
                value /= 1000;
                digit_value = value % 10;
                // Solo mostrar si el número es >= 1000
                if (counter >= 1000) {
                    GPIOE->ODR = seven_segment_digits[digit_value];
                    GPIOD->ODR = 0xFE;  // Activar PD0 (extremo izquierdo)
                }
                break;
        }
        
        // Avanzar al siguiente dígito para la próxima interrupción
        current_digit = (current_digit + 1) % 4;
    }
}

// Manejador de interrupción para TIM3 (actualización del contador)
extern "C" void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;
        
        // Incrementar contador de 0 a 9999
        counter++;
        if (counter > 9999) {
            counter = 0;
        }
    }
}

int main(void) {
    // Inicializar el sistema
    MySystemInit();
    
    // Iniciar con valor 0
    counter = 0;
    current_digit = 0;
    
    // Inicializar estados de salida
    GPIOE->ODR = 0;       // Todos segmentos apagados
    GPIOD->ODR = 0xFF;    // Todos dígitos apagados (para cátodo común, 1 = apagado)
    
    while (1) {
        // Todo el trabajo de visualización se maneja en las interrupciones
        // No necesitamos hacer nada en el bucle principal
    }
}
