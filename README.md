### STM32-CAR-RCT6

### 引脚定义图

| STM32F103C8T6引脚定义表 |                 |      |           |            |                      |            |
| ----------------------- | --------------- | ---- | --------- | ---------- | -------------------- | ---------- |
| 引脚号                  | 引脚名称        | 类型 | I/O口电平 | 主功能     | 默认复用功能         | 重定义功能 |
| 1                       | VBAT            | S    |           | VBAT       |                      |            |
| 2                       | PC13-TAMPER-RTC | I/O  |           | PC13       |                      |            |
| 3                       | PC14-OSC32_IN   | I/O  |           | PC14       |                      |            |
| 4                       | PC15-OSC32_OUT  | I/O  |           | PC15       |                      |            |
| 5                       | OSC_IN          | I    |           | OSC_IN     |                      |            |
| 6                       | OSC_OUT         | O    |           | OSC_OUT    |                      |            |
| 7                       | NRST            | I/O  |           | NRST       |                      |            |
| 8                       | VSSA            | S    |           | VSSA       |                      |            |
| 9                       | VDDA            | S    |           | VDDA       |                      |            |
| 10                      | PA0-WKUP        | I/O  |           | PA0        |                      |            |
| 11                      | PA1             | I/O  |           | PA1        |                      |            |
| 12                      | PA2             | I/O  |           | PA2        | PWMB                 |            |
| 13                      | PA3             | I/O  |           | PA3        | PWMA                 |            |
| 14                      | PA4             | I/O  |           | PA4        |                      |            |
| 15                      | PA5             | I/O  |           | PA5        |                      |            |
| 16                      | PA6             | I/O  |           | PA6        | E2A                  |            |
| 17                      | PA7             | I/O  |           | PA7        | E2B                  |            |
| 18                      | PB0             | I/O  |           | PB0        | AIN1                 |            |
| 19                      | PB1             | I/O  |           | PB1        | AIN2                 |            |
| 20                      | PB2             | I/O  | FT        | PB2/BOOT1  |                      |            |
| 21                      | PB10            | I/O  | FT        | UART3-TX   | BLU_RX               |            |
| 22                      | PB11            | I/O  | FT        | UART3-RX   | BLU_TX               |            |
| 23                      | VSS_1           | S    |           | VSS_1      |                      |            |
| 24                      | VDD_1           | S    |           | VDD_1      |                      |            |
| 25                      | PB12            | I/O  | FT        | PB12       | OLED_SCL             |            |
| 26                      | PB13            | I/O  | FT        | PB13       | OLED_SDA             |            |
| 27                      | PB14            | I/O  | FT        | PB14       | BIN1                 |            |
| 28                      | PB15            | I/O  | FT        | PB15       | BIN2                 |            |
| 29                      | PA8             | I/O  | FT        | PA8        |                      |            |
| 30                      | PA9             | I/O  | FT        | PA9        |                      |            |
| 31                      | PA10            | I/O  | FT        | PA10       |                      |            |
| 32                      | PA11            | I/O  | FT        | PA11       | USER_LED             |            |
| 33                      | PA12            | I/O  | FT        | PA12       | BEEP                 |            |
| 34                      | PA13            | I/O  | FT        | JTMS/SWDIO |                      |            |
| 35                      | VSS_2           | S    |           | VSS_2      |                      |            |
| 36                      | VDD_2           | S    |           | VDD_2      |                      |            |
| 37                      | PA14            | I/O  | FT        | JTCK/SWCLK |                      |            |
| 38                      | PA15            | I/O  | FT        | JTDI       |                      |            |
| 39                      | PB3             | I/O  | FT        | JTDO       |                      |            |
| 40                      | PB4             | I/O  | FT        | NJTRST     |                      |            |
| 41                      | PB5             | I/O  |           | PB5        |                      |            |
| 42                      | PB6             | I/O  | FT        | PB6        | E1A                  |            |
| 43                      | PB7             | I/O  | FT        | PB7        | E1B                  |            |
| 44                      | BOOT0           | I    |           | BOOT0      |                      |            |
| 45                      | PB8             | I/O  | FT        | PB8        |                      |            |
| 46                      | PB9             | I/O  | FT        | PB9        |                      |            |
| 47                      | VSS_3           | S    |           | VSS_3      |                      |            |
| 48                      | VDD_3           | S    |           | VDD_3      |                      |            |
| 49                      | PC12            | I/O  | FT        | UART5 TX   | 连的是正点原子IMU_RX |            |
| 50                      | PD2             | I/O  | FT        | UART5 RX   | 连的是正点原子IMU_TX |            |
| 51                      | PC4             | I/O  | FT        |            | MPU6050_SCL          |            |
| 52                      | PC5             | I/O  | FT        |            | MPU6050_SDA          |            |