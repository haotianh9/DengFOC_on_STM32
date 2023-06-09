# DengFOC_on_STM32
My implementation of Field-Oriented Control(FOC) on STM32 following DengFOC's tutorial 

DengFOC's tutorial: [DengFOC](http://dengfoc.com/#/)


Hardware used: 

MCU Development Board: I first used STM32G4 Nucleo-64 board from [motor control kit](https://www.st.com/en/evaluation-tools/p-nucleo-ihm03.html) (used on open loop speed control case). When working on closed loop position control, I found SPI communication is not working on that board. Although spent lots of time on it, I found it hard to debug. Thus, I switched to bulepill from then. If anybody can figure out the SPI communication problem, please Email me or submit a pull request. Thank you!
Driver Board: [motor control kit](https://www.st.com/en/evaluation-tools/p-nucleo-ihm03.html)
Motor with encoder: iPower Motor GM3506 Brushless Gimbal Motor w/ AS5048A Encoder [link](https://www.robotshop.com/products/ipower-motor-gm3506-brushless-gimbal-motor-w-as5048a-encoder)
