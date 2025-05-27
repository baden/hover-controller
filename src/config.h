#pragma once


#define PHASE_SCALE                     256     // 1 градус = 256 одиниць
#define _A(x) ((x) * PHASE_SCALE)               // Macro to convert degrees to scaled value
#define _toA(x) ((x) / PHASE_SCALE)             // Macro to convert scaled value to degrees


#define MAX_PHASE_STEP_PER_CYCLE             _A(60)  // ~60 градусів на цикл
#define FILTER_SHIFT                         3     // фільтр = 1/8, тобто FILTER_STRENGTH = 8
#define STALL_PHASE_MOVEMENT_THRESHOLD       _A(30)
#define STALL_PHASE_ERROR_THRESHOLD          _A(60)
#define STALL_COUNTER_LIMIT                  100
#define INTEGRATOR_LIMIT                     _A(500)

// PID коефіцієнти (підібрати експериментально)
#define KP                                   5
#define KI                                   0


