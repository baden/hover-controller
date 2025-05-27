#pragma once


#define PHASE_SCALE                     256     // 1 градус = 256 одиниць
#define _A(x) ((x) * PHASE_SCALE)               // Macro to convert degrees to scaled value
#define _toA(x) ((x) / PHASE_SCALE)             // Macro to convert scaled value to degrees

#define DEADZONE_THRESHOLD                  _A(180)     // Можливо менше ніж 120 градусів не має сенсу?
#define FULL_STOP_THRESHOLD                 _A(90)      // Якщо похибка менше 60 градусів, то зовсім зупиняємося

#define MAX_PHASE_STEP_PER_CYCLE            _A(100)  // ~60 градусів на цикл
#define FILTER_SHIFT                        2     // фільтр = 1/8, тобто FILTER_STRENGTH = 8
#define STALL_PHASE_MOVEMENT_THRESHOLD      _A(30)
#define STALL_PHASE_ERROR_THRESHOLD         _A(60)
#define STALL_COUNTER_LIMIT                 1000
#define INTEGRATOR_LIMIT                    _A(500)
#define MAX_AMPLITUDE                       300    // Максимальна амплітуда синусоїди
#define PHASE_ADVANCE                      _A(90)  // Фаза струму на 90 градусів вперед від ротора

#define MAX_PHASE_ERROR                    _A(180) // Максимальна похибка фази для PID контролера

// PID коефіцієнти (підібрати експериментально)
#define KP                                  (256 * 4)
#define KI                                  (256 * 0.02)
#define PID_SHIFT                           8

// Значення PID для мертвої зони
#define KP_DEADZONE                        (256 * 1)
#define KI_DEADZONE                        (256 * 0.005)
