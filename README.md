## Вирішив спробувати зробити програму для плати гіроскутера.

Їх по суті аж 6 штук десь є. По дві на гіроскутер. Ну дві є в мене прям зараз.

Ось знайшов два проєкти

- https://github.com/trondin/MM32SPIN05_Hoberboard_hack
- https://github.com/AILIFE4798/Hoverboard-Firmware-Hack-Gen2.x-MM32 (вже нема)
- https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x

Даташит на проц: https://github.com/SoCXin/MM32SPIN422C/blob/main/docs/DS_MM32SPIN422C_SC.pdf
Ага, китайською.

Я все внаглую стирів з https://github.com/trondin/MM32SPIN05_Hoberboard_hack

Але вже бачу шо в мене інша плата.

В мене ось оця (v1-2.8):

https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/blob/main/target_4%3DMM32SPIN0X/v1%3D2.8/master_front.jpg

Можливо почнемо звідси:

https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32/blob/main/HoverBoardGigaDevice/Inc/defines_2-4-1.h



# hover-controller


Gen2.4.1

master, master-slave uart 

halla        ( 0):    28 (PC15)
hallb        ( 1):    26 (PC13)
hallc        ( 2):    27 (PC14)
ledr         ( 3):     8 (PA11)
ledg         ( 4):    32 (PD3)
ledb         ( 5):    31 (PD2)
ledu         ( 6):    23 (PB10)
ledd         ( 7): 65535 (not set)
buzzer       ( 8): 65535 (not set)
button       ( 9):    22 (PB9)
latch        (10):    15 (PB2)      Тут начеб-то співпало
charge       (11): 65535 (not set)
vbat         (12):    14 (PB1)
itotal       (13): 65535 (not set)
tx           (14):    19 (PB6)      Тут начеб-то співпало
rx           (15):    17 (PB4)      Тут начеб-то співпало


Попередньо керування мосфетами:

PhAp - PA8
PhAn - PB13
PhBp - PA9
PhBn - PB14
PhCp - PA10
PhCn - PB15

Схоже це у більшості плат незмінне.

uint16_t pinstorage[64]={28, 26, 27, 8, 32, 31, 23, 65535, 65535, 22, 15, 65535, 14, 65535, 19, 17, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 56491, 31, 250, 0, 19200, 8192, 1, 30, 0, 10, 300, 1, 1, 42000, 32000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


slave, bluetooth uart header

halla        ( 0):    26 (PC13)
hallb        ( 1):    27 (PC14)
hallc        ( 2):    28 (PC15)
ledr         ( 3):     8 (PA11)
ledg         ( 4):    32 (PD3)
ledb         ( 5):    31 (PD2)
ledu         ( 6):    23 (PB10)
ledd         ( 7): 65535 (not set)
buzzer       ( 8):    22 (PB9)
button       ( 9): 65535 (not set)
latch        (10): 65535 (not set)
charge       (11): 65535 (not set)
vbat         (12): 65535 (not set)
itotal       (13): 65535 (not set)
tx           (14):    20 (PB7)
rx           (15):    21 (PB8)


uint16_t pinstorage[64]={26, 27, 28, 8, 32, 31, 23, 65535, 22, 65535, 65535, 65535, 65535, 65535, 20, 21, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 56491, 31, 250, 0, 19200, 8192, 1, 30, 0, 10, 300, 1, 1, 42000, 32000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};



## Прошивка

Треба встановити пак для pyocd.

```
pyocd pack install mm32spin05
```

