# fub.*~
**Foot Switch &amp; Expression Pedal MIDI interface**  
has 4 of 6.35 jack.  
LEFT 2 jacks | Foot Switch jack. (Dual Foot Switch is available.)  
RIGHT 2 jacks | Expression Pedal jack.  
  
It is using ATSAND21G18A microcom chip as core.  

![IMG_8484](https://github.com/Turm-Design-Works/fub/assets/75283624/7cfb2a1e-bb16-4ba0-9779-1d95b645c9d7)

# LICENSES  
**MIT** ( [view LICENSE](https://github.com/Turm-Design-Works/fub/blob/main/LICENSE) )  
2024 Turm Design Works LLC.

# How to install firmware  
It needs to install "microchip(Atmel) studio", and prepare SAM/ARM writer (e. Atmel ICE, PICkit, j-link) with 5 debug wires.  
  
1. Solder all parts.
2. Connect fub.*~ & SAM/ARM writer to your PC.
3. Connect ARM writer to fub.*~ PCB empty holes is marked "R V G C D" by using debug wire.
> R: RESET, V: VTG 3.3V, G: GND, C: SWCLK, D: SWDIO  
> Please follow the connection of writer outputs you use.  
> ![IMG_8499-480](https://github.com/Turm-Design-Works/fub/assets/75283624/27d6cd9a-7f3a-4b07-aff6-20a29d3e4ac2)  
4. Open microchip studio application.
> Move to "Tools" -> "Device Programming".  
> ![164-480](https://github.com/Turm-Design-Works/fub/assets/75283624/eec0082e-55fa-414e-b57e-c822774034f9)  
5. Apply writer and read chip.
> 1: Select writer you use, 2: Select Device "ATSAMD21G18A", 3: Select Interface "SWD",  
> 4: Click "Apply", 5: Click "Read"  
> ![165-480](https://github.com/Turm-Design-Works/fub/assets/75283624/e3a22117-b892-4407-9201-8208da38410e)  
6. Move to memories tab, program .hex firmware.
> 1: Click "Erase now", 2: Select .hex firmware, 3: Click "Program"  
> ![166-480](https://github.com/Turm-Design-Works/fub/assets/75283624/05dc8db6-569a-4951-8992-3f0553da26f9)  
7. Disconnect fub.*~ and ARM writer from PC. Done.
