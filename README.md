# X11 → SPI (ILI9341) — Orange Pi

---

## 🇧🇷 Português

### Descrição
Pseudo-driver em userland para espelhar X11 em displays SPI (testado em Orange Pi Zero 3 com ILI9341 240×320).  
Este projeto surgiu porque o driver framebuffer do ILI9341 foi removido no kernel 6.1.

Não é a solução mais eficiente, nem a mais bonita — mas funciona.

O código foi escrito por alguém que veio do Python e está aprendendo C agora.  
Não há planos de manutenção ou evolução.  
O repositório é público porque deu trabalho chegar até aqui e talvez ajude alguém.

### Requisitos
- Orange Pi com Debian/Ubuntu e X11 ativo  
- Display SPI (ex.: ILI9341 240×320)  
- `build-essential libx11-dev libxext-dev`

### Compilar
```bash
gcc -O2 -o screen++ screen++.c -lX11 -lXext -lpthread -lrt
```

### Uso
```bash
DISPLAY=:0 xhost +local:
DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0 ./screen++ <modo> <rotacao>
```

- modo: 1–4 (se as cores estiverem erradas)
- rotação: 0, 90, 180, 270

### Troubleshooting
- Cores erradas → troque o modo  
- Imagem estranha → confira a fiação SPI e o GND  

### Aviso
Código experimental, escrito durante aprendizado.  
Use por sua conta e risco.

---

## 🇺🇸 English

### Description
Userland pseudo-driver to mirror X11 to SPI displays (tested on Orange Pi Zero 3 with a 240×320 ILI9341).  
This project exists because the ILI9341 framebuffer driver was dropped in Linux kernel 6.1.

It is not the most efficient or elegant solution — it just works.

The code was written by someone coming from Python and currently learning C.  
There are no plans for maintenance or future improvements.  
It is public because it took real effort to get working and may help others.

### Requirements
- Orange Pi running Debian/Ubuntu with X11  
- SPI display (e.g. ILI9341 240×320)  
- `build-essential libx11-dev libxext-dev`

### Build
```bash
gcc -O2 -o screen++ screen++.c -lX11 -lXext -lpthread -lrt
```

### Usage
```bash
DISPLAY=:0 xhost +local:
DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0 ./screen++ <mode> <rotation>
```

- mode: 1–4 (try if colors are wrong)
- rotation: 0, 90, 180, 270

### Troubleshooting
- Wrong colors → change mode  
- Corrupted image → check SPI wiring and GND  

### Disclaimer
Experimental code written while learning C.  
Use at your own risk.

---

## License
MIT
