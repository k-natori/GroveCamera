# GroveCamera

GroveCamera is a skecth to transfer captured image from M5Camera to M5Stack or M5Paper, using Grove connectors and cable with UART.
Please see counterparts of this skecth for M5Stack and M5Paper:

https://github.com/k-natori/GroveCameraStack

https://github.com/k-natori/GroveCameraPaper

These sketches written for M5Camera X and M5Stack Basic, so should be modified for your devices. For example, you should specify M5Camera model in "define" section, should change RX and TX pins for other M5Stack models.

A schematic diagram of the cable connections is shown below. Please note that **M5Camera and M5Stack should NOT be connected to the PC at the same time** since the 5V line is connected.
## Grove cable connection
![Grove cable connection](HOW_TO_USE/01_Connections.png)

A schematic diagram of the contents and sequence of UART transmissions is shown below.
## UART communication
![UART communication](HOW_TO_USE/02_Communications.png)
