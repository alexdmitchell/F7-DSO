# F7-DSO
DSO Project using the STM 32F746G Discovery Board - custom firmware and analogue front end PCB for my MSc project

Project template & initialisation code generated using STMCubeMX

IDE: Keil uVision 5 Professional

Graticule is displayed in a seperate LCD layer, converted from a JPEG image (drawn in AutoCAD) to c header file using the LCD Image Converter tool: http://www.riuson.com/lcd-image-converter

Features:

2.4 MS/s sampling rate achieved using the onboard ADC (ADC3) and DMA

8 bit resolution

Waveform displayed on the integrated LCD

Full touch screen control

Analogue front end PCB including relay switched 1:1/1:10 attenuation & AC/DC Coupling, Programmable Gain Amplifier with DAC for offset
        
Notes:

You will first need to regenerate the project files using CubeMX (to replace the multiple files in STCubeGenerated that were deleted to reduce file count); open STCubeGenerated.ioc and regenerate the project. Then open the project in uVision and recompile - note that the Pro licence is needed as the code is >32kB.

The input for ADC3 is PA2 on the Arduino header on the rear of the board - the full schematic for the complete analogue front end PCB is included.

