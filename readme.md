# MPU-BYJ

Use the relative displacement information of x-axis of MPU6050/6000 to control the rotation of BYJ48.

## Connections

MPU6050/6000 (USE MPU 6000, THE BLUE CHIP):

- VCC -> 5V
- GND -> GND
- SCL -> SCL
- SDA -> SDA

### BYJ48

- The white port (on BYJ) -> The white port (on stepping motor board) 

### Stepping Motor Board

- (-) -> GND
- (+) -> 5V
- INT1 -> DIGITAL 8
- INT2 -> DIGITAL 9
- INT3 -> DIGITAL 10
- INT4 -> DIGITAL 11

---

## Refences

https://github.com/jrowberg/i2cdevlib
https://www.instructables.com/id/BYJ48-Stepper-Motor/

Yilin Zhang 2019.01.15
