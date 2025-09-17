#include <stdio.h>
#include "esp_timer.h"
#include "lookuptable.h"

// Función de regresión (sin cambios)
int regression_func(int x) {
    return (int)(4.28297943e-06 * x * x - 3.83561791e-02 * x + 93.92003116191847);
}

void app_main(void) {
    int test_val = 0;
    int result;
    int64_t start_time, end_time;

    // --- Medición de Tiempo: Método de Regresión (sin cambios) ---
    start_time = esp_timer_get_time();
    result = regression_func(test_val);
    end_time = esp_timer_get_time();
    printf("Resultado (Regresión): %d | Tiempo de ejecución: %lld us\n", result, (end_time - start_time));

    // --- Medición de Tiempo: Método de LUT con Bucle para Precisión ---
    const int NUM_ITERATIONS = 10000; // Repetiremos la operación 10,000 veces

    start_time = esp_timer_get_time();
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        result = lookup_table[test_val]; // La operación que queremos medir
    }
    end_time = esp_timer_get_time();

    // Calculamos el tiempo total y luego el promedio
    int64_t total_time_lut = end_time - start_time;
    double average_time_lut = (double)total_time_lut / NUM_ITERATIONS;

    // Imprimimos el resultado con decimales usando %f
    printf("Resultado (LUT):       %d | Tiempo de ejecución promedio: %f us\n", result, average_time_lut);
}