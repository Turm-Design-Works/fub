# fub.*~
**Foot Switch &amp; Expression Pedal MIDI interface**  
has 4 of 6.35 jack.  
LEFT 2 jacks | Foot Switch jack. (Dual Foot Switch is available.)  
RIGHT 2 jacks | Expression Pedal jack.  

It is using ATSAND21G18A microcom chip as core.  

![IMG_8484](https://github.com/Turm-Design-Works/fub/assets/75283624/7cfb2a1e-bb16-4ba0-9779-1d95b645c9d7)

# LICENSES
- **MIT** ( [view LICENSE](https://github.com/Turm-Design-Works/fub/blob/main/LICENSE) )  
2024 Turm Design Works LLC.

# How to install firmware  
- After soldering, connect USB & ARM writer (e. Atmel ICE, PICkit, j-link) to your PC.
- Connect ARM writer to PCB's empty holes is marked "R V G C D" by using debug wire etc.
> R = RESET, V = VTG 3.3V, G = GND, C = SWCLK, D = SWDIO
- Open microchip studio application, Tools -> Device Programming.
- Apply writer and read chip. Afterwards select and program .bin firmware.
