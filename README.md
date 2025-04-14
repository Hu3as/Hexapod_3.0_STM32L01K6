## <div align="center">Hexapod Based on STM32 Microcontroller</div>

![Hexapod_Render](src/Hexapod-Assembly.png)

Designed and built for an engineering thesis, this six-legged walking robot is controlled via Bluetooth or an nRF module. As part of the project, a PCB was designed and fabricated, and the software was developed. The robot's structure was 3D printed.

The repository includes KiCad files for the PCB and a project in STM Cube IDE.

<div style="text-align: center;">
<img src="src/Hexapod.jpg" alt="HexapodPhoto" width="800">
<img src="src/PCB_render.jpg" alt="PCBRender" width="400">
<img src="src/Top_PCB_View.png" alt="PCBTOP" width="800">
<img src="src/Bottom_PCB_View.png" alt="PCBBOT" width="800">
</div>

### Components used:

- <b>Microcontroller:</b> STM32 L031K6T6

- <b>PWM Driver:</b> PCA9685PW

- <b>Bluetooth:</b> HC-05

- <b>nRF:</b> NRF24L01+ 2.4GHz

- <b>Buck Converters:</b> 8.4V → 6V

- <b>Transistors, Regulators, resistors, capacitors and other primary electronic components</b>

- <b>PWM Servos:</b> MG90S
<div style="text-align: center;">
<img src="src/MCU_Schematic.png" alt="MCU" width="800">
<img src="src/Schemat_PCA9685.png" alt="PCA" width="800">
<img src="src/nRF_schematic.png" alt="nRF" width="800">
</div>

### <div align="center">Software </div>

The software operates through communication via I2C, SPI, and UART with the respective components: the PWM driver, nRF module, and Bluetooth. The software includes an algorithm responsible for moving the robot's legs in Cartesian space. The robot is capable of moving forward, backward, and rotating.

### Software Structure:

<div style="text-align: center;">
<img src="src/Software_Schematic.jpg" alt="SoftDiag" width="800">
</div>
