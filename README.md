# nturt_stm32_front_box

## Introduction

- Pins are denoted as `PIN`, e.g. `5V`, `PA0`, etc.

## Peripheral Configuration

### GPIO

#### Button (pull-down)

- RTD_button -> GPIO `PE15`
- Gear1 -> GPIO `PE8`
- Gear2 -> GPIO `PE12`
- Gear3 -> GPIO `PE10`

#### Microswitch (pull-down)

- BSE_micro -> GPIO `PF2`
- APPS_micro -> GPIO `PE9`

#### Hall Sensor (pull-down)

- HallL -> GPIO `PF3`
- HallR -> GPIO `PC8`

#### LED

###### In Box

- warn_led -> GPIO `PC9`
- error_led -> GPIO `PC12`
- can_led -> GPIO `PG2`
- reserverd_led -> GPIO `PG3`

###### On Dashboard

- VCU_light -> GPIO `PE7`
- RTD_light -> GPIO `PA0`
- Gear_light -> GPIO `PE13`

#### Other

- RTD_siren -> GPIO `PD0`

### ADC

#### Pedal

- BSE -> ADC3 IN6 `PF10`
- APPS1 -> ADC1 IN5 `PB1`
- APPS2 -> ADC3 IN0 `PC2_C`

#### Sunspension Travel

- SuspensionL -> ADC3 IN8 `PF6`
- SuspensionR -> ADC1 IN15 `PA3`

#### Oil Pressure

- Oil -> ADC1 IN18 `PA4`

#### Strain Guage

- Strain -> ADC1 IN19 `PA5`

### I<sup>2</sup>C

#### Temperature Sensor (as master)

- TempL `SCL` -> I2C1 `PB8`
- TempL `SDA` -> I2C1 `PB9`
- TempR `SCL` -> I2C5 `PC10`
- TempR `SDA` -> I2C5 `PC11`

#### Rapspberry Pi (as slave)

- RPi `SCL` -> I2C2 `PF0`
- RPi `SDA` -> I2C2 `PF1`

### SPI

#### Steering Encoder

- Encoder `SCK` -> SPI4 `PE2`
- Encoder `MISO` -> SPI4 `PE5`
- Encoder `MOSI` -> SPI4 `PE6`
- Encoder `SS` -> GPIO `PE4`

### CAN

- `TX` -> FDCAN3 `PD13`
- `RX` -> FDCAN3 `PD12`

<div style="page-break-after: always;"></div>

## Appendix

### Pinout

*The pinout is the same as nucleo-h743gz2*

#### Zio and Arduino-compatible headers

<img src="https://os.mbed.com/media/uploads/jeromecoutant/nucleo_h743zi2_zio_left_2019_10_9.png"  width="80%" height="40%">
<img src="https://os.mbed.com/media/uploads/jeromecoutant/nucleo_h743zi2_zio_right_2019_10_9.png"  width="80%" height="40%">

<div style="page-break-after: always;"></div>

#### CN11 CN12 headers

<img src="https://os.mbed.com/media/uploads/jeromecoutant/nucleo_h743zi2_morpho_left_2019_10_9.png"  width="90%" height="50%">
<img src="https://os.mbed.com/media/uploads/jeromecoutant/nucleo_h743zi2_morpho_right_2019_10_9.png"  width="90%" height="50%">

<div style="page-break-after: always;"></div>
