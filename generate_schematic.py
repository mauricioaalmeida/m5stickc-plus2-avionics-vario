import os
from PIL import Image, ImageDraw, ImageFont

w, h = 1920, 1080
img = Image.new('RGB', (w, h), '#FFFFFF')
draw = ImageDraw.Draw(img)

try:
    font_title = ImageFont.truetype('arialbd.ttf', 26)
    font_sub = ImageFont.truetype('arial.ttf', 14)
    font_box_title = ImageFont.truetype('arialbd.ttf', 16)
    font_pin_lbl = ImageFont.truetype('arialbd.ttf', 13)
    font_pin_sub = ImageFont.truetype('arial.ttf', 11)
    font_detail = ImageFont.truetype('arial.ttf', 12)
    font_warn_title = ImageFont.truetype('arialbd.ttf', 14)
    font_warn = ImageFont.truetype('arial.ttf', 12)
except:
    font_title = font_sub = font_box_title = font_pin_lbl = font_pin_sub = font_detail = font_warn_title = font_warn = ImageFont.load_default()

# Header banner
draw.rectangle([(0, 0), (w, 65)], fill='#0D47A1')
draw.text((35, 12), 'ESQUEMA ELÉTRICO E DE FIAÇÃO CORRETO - VARIÔMETRO ESP32', fill='#FFFFFF', font=font_title)
draw.text((35, 42), 'Validação Física: GPS NEO-6M, GY-87 (10-DOF), Amplificador LM0080A, WeAct E-Paper e 4 Botões', fill='#E3F2FD', font=font_sub)

def draw_module(x, y, bw, bh, title, header_color='#1E88E5', bg_color='#FAFAFA'):
    draw.rectangle([(x, y), (x+bw, y+bh)], fill=bg_color, outline='#37474F', width=2)
    draw.rectangle([(x, y), (x+bw, y+30)], fill=header_color)
    draw.text((x+10, y+6), title, fill='#FFFFFF', font=font_box_title)

# Palette
C_RED = '#D32F2F'
C_BLACK = '#212121'
C_YELLOW = '#F57F17'
C_GREEN = '#2E7D32'
C_BLUE = '#1565C0'
C_WHITE = '#546E7A'
C_ORANGE = '#E65100'
C_PURPLE = '#6A1B9A'

# 1. ESP32 (Center Box)
esp_x, esp_y, esp_w, esp_h = 720, 170, 480, 780
draw_module(esp_x, esp_y, esp_w, esp_h, 'WEMOS ESP32 (ESP-WROOM-32)', '#263238', '#ECEFF1')

# 2. GY-87 (Left Middle)
gy_x, gy_y, gy_w, gy_h = 40, 370, 360, 310
draw_module(gy_x, gy_y, gy_w, gy_h, 'PLACA GY-87 (10-DOF)', '#0277BD')

# 3. GPS (Left Bottom)
gps_x, gps_y, gps_w, gps_h = 40, 715, 360, 235
draw_module(gps_x, gps_y, gps_w, gps_h, 'GPS u-blox NEO-6M-0-001', '#2E7D32')

# 4. POWER (Top Left)
pwr_x, pwr_y, pwr_w, pwr_h = 40, 85, 520, 245
draw_module(pwr_x, pwr_y, pwr_w, pwr_h, 'SISTEMA DE ENERGIA (18650 + TP4056)', '#EF6C00')

# 5. AUDIO LM0080A (Top Right)
aud_x, aud_y, aud_w, aud_h = 1340, 85, 540, 245
draw_module(aud_x, aud_y, aud_w, aud_h, 'AMPLIFICADOR HC-LM0080A (8002A)', '#C2185B')

# 6. E-PAPER WEACT 2.13 (Right Middle)
ep_x, ep_y, ep_w, ep_h = 1340, 360, 540, 370
draw_module(ep_x, ep_y, ep_w, ep_h, 'DISPLAY WeAct Studio 2.13\" E-PAPER', '#512DA8')

# 7. 4 BUTTONS (Right Bottom)
btn_x, btn_y, btn_w, btn_h = 1340, 760, 540, 190
draw_module(btn_x, btn_y, btn_w, btn_h, '4 BOTÕES DE NAVEGAÇÃO', '#00695C')

# Pins on ESP32 Left Header
esp_pins_left = [
    ('5V / VBUS', 'Alimentação 5V', 220),
    ('GND',       'Terra comum',    270),
    ('3V3',       'Saída 3.3V',     320),
    ('GPIO 22',   'I2C SCL',        420),
    ('GPIO 21',   'I2C SDA',        480),
    ('GPIO 14',   'INT MPU6050',    540),
    ('GPIO 16',   'RX2 (GPS TXD)',  770),
    ('GPIO 17',   'TX2 (GPS RXD)',  830),
]

# Pins on ESP32 Right Header
esp_pins_right = [
    ('GPIO 25', 'Áudio PWM / DAC', 220),
    ('GPIO 18', 'VSPI SCK',        400),
    ('GPIO 23', 'VSPI MOSI',       445),
    ('GPIO 5',  'VSPI CS',         490),
    ('GPIO 27', 'D/C',             535),
    ('GPIO 26', 'RES / Reset',     580),
    ('GPIO 35', 'BUSY',            625),
    ('GPIO 32', 'BTN 0 (MENU)',    795),
    ('GPIO 15', 'BTN L (UP)',      835),
    ('GPIO 12', 'BTN M (SELECT)',  875),
    ('GPIO 4',  'BTN R (DOWN)',    915),
]

# Draw ESP32 Left Pins
for name, desc, py in esp_pins_left:
    draw.rectangle([(esp_x, py-12), (esp_x+105, py+12)], fill='#CFD8DC', outline='#455A64')
    draw.text((esp_x+8, py-7), name, fill='#000000', font=font_pin_lbl)
    draw.text((esp_x+115, py-6), desc, fill='#37474F', font=font_pin_sub)
    draw.ellipse([(esp_x-6, py-6), (esp_x+6, py+6)], fill='#0D47A1')

# Draw ESP32 Right Pins
for name, desc, py in esp_pins_right:
    draw.rectangle([(esp_x+esp_w-105, py-12), (esp_x+esp_w, py+12)], fill='#CFD8DC', outline='#455A64')
    draw.text((esp_x+esp_w-97, py-7), name, fill='#000000', font=font_pin_lbl)
    draw.text((esp_x+esp_w-215, py-6), desc, fill='#37474F', font=font_pin_sub)
    draw.ellipse([(esp_x+esp_w-6, py-6), (esp_x+esp_w+6, py+6)], fill='#0D47A1')

# Wire function
def draw_wire(points, color, width=3):
    draw.line(points, fill=color, width=width)
    for p in [points[0], points[-1]]:
        draw.ellipse([(p[0]-5, p[1]-5), (p[0]+5, p[1]+5)], fill=color)

# GY-87 Pins
gy_pins = [
    ('VCC-IN', 'Entrada 5V (regulador interno gera 3.3V)', 420, C_RED),
    ('GND',    'Terra comum',                              470, C_BLACK),
    ('SCL',    'Clock I2C',                                520, C_YELLOW),
    ('SDA',    'Dados I2C',                                570, C_GREEN),
    ('INTA',   'Interrupção Data-Ready MPU6050',           620, C_WHITE),
    ('3.3V',   'Saída regulada (NÃO CONECTAR)',            652, '#9E9E9E'),
    ('DRDY',   'Bússola / FSYNC (NÃO CONECTAR)',           672, '#9E9E9E'),
]

for pname, pdesc, py, pcol in gy_pins:
    draw.text((gy_x+15, py-8), pname, fill=pcol, font=font_pin_lbl)
    draw.text((gy_x+95, py-7), pdesc, fill='#616161', font=font_pin_sub)
    if not 'NÃO' in pdesc:
        draw.ellipse([(gy_x+gy_w-6, py-6), (gy_x+gy_w+6, py+6)], fill=pcol)

# GPS Pins
gps_pins = [
    ('VCC', 'Alimentação 5V (regulador 3.3V interno)', 760, C_RED),
    ('GND', 'Terra comum',                              805, C_BLACK),
    ('TXD', 'Transmite NMEA -> RX2 do ESP32',           850, C_YELLOW),
    ('RXD', 'Recebe config  <- TX2 do ESP32',           895, C_GREEN),
    ('PPS', 'Pulso 1s (NÃO CONECTAR)',                  925, '#9E9E9E'),
]

for pname, pdesc, py, pcol in gps_pins:
    draw.text((gps_x+15, py-8), pname, fill=pcol, font=font_pin_lbl)
    draw.text((gps_x+75, py-7), pdesc, fill='#616161', font=font_pin_sub)
    if not 'NÃO' in pdesc:
        draw.ellipse([(gps_x+gps_w-6, py-6), (gps_x+gps_w+6, py+6)], fill=pcol)

# LM0080A Pins
aud_pins = [
    ('POWER 5V+',     'Alimentação 5V',                                   130, C_RED),
    ('AGND',          'Terra do áudio e alimentação',                     170, C_BLACK),
    ('AUDIO INPUT',   'Sinal PWM/DAC <- GPIO 25',                         210, C_BLUE),
    ('MUTE',          'Deixar desconectado (ativo por padrão)',           245, '#9E9E9E'),
    ('AUDIO OUT +/-', 'Ligar no Alto-Falante (NUNCA aterrar OUT- no GND)',275, '#D81B60'),
]

for pname, pdesc, py, pcol in aud_pins:
    draw.text((aud_x+15, py-8), pname, fill=pcol, font=font_pin_lbl)
    draw.text((aud_x+140, py-7), pdesc, fill='#616161', font=font_pin_sub)
    if not 'NÃO' in pdesc and not '+/-' in pname:
        draw.ellipse([(aud_x-6, py-6), (aud_x+6, py+6)], fill=pcol)

# WeAct E-Paper Pins (8-wire pre-made cable)
ep_pins = [
    ('VCC (Vermelho)', '3.3V <- pino 3V3 do ESP32', 400, C_RED),
    ('GND (Preto)',    'GND <- pino GND do ESP32',  445, C_BLACK),
    ('SCL (Verde)',    'SPI SCK <- GPIO 18',        490, C_GREEN),
    ('SDA (Amarelo)',  'SPI MOSI <- GPIO 23',       535, C_YELLOW),
    ('CS (Azul)',      'Chip Select <- GPIO 5',     580, C_BLUE),
    ('D/C (Branco)',   'Data / Command <- GPIO 27', 625, C_WHITE),
    ('RES (Laranja)',  'Reset <- GPIO 26',          670, C_ORANGE),
    ('BUSY (Roxo)',    'Busy status -> GPIO 35',    715, C_PURPLE),
]

for pname, pdesc, py, pcol in ep_pins:
    draw.text((ep_x+15, py-8), pname, fill=pcol, font=font_pin_lbl)
    draw.text((ep_x+145, py-7), pdesc, fill='#616161', font=font_pin_sub)
    draw.ellipse([(ep_x-6, py-6), (ep_x+6, py+6)], fill=pcol)

# Buttons
btn_pins = [
    ('BTN 0 (MENU)',    'GPIO 32 (Fio Branco)',  795, C_WHITE),
    ('BTN L (UP)',      'GPIO 15 (Fio Branco)',  835, C_WHITE),
    ('BTN M (SELECT)',  'GPIO 12 (Fio Amarelo)', 875, C_YELLOW),
    ('BTN R (DOWN)',    'GPIO 4 (Fio Verde)',    915, C_GREEN),
]

for pname, pdesc, py, pcol in btn_pins:
    draw.text((btn_x+25, py-8), pname, fill=pcol, font=font_pin_lbl)
    draw.text((btn_x+180, py-7), pdesc, fill='#424242', font=font_pin_sub)
    draw.ellipse([(btn_x-6, py-6), (btn_x+6, py+6)], fill=pcol)

draw.text((btn_x+25, 935), 'Comum dos 4 botões: Todos interligados ao GND (Fio Preto)', fill='#004D40', font=font_pin_sub)

# Power Sub-boxes
draw.rectangle([(pwr_x+15, pwr_y+45), (pwr_x+220, pwr_y+135)], outline='#EF6C00', fill='#FFE0B2', width=1)
draw.text((pwr_x+25, pwr_y+53), 'BATERIA 18650 (3.7V)', fill='#E65100', font=font_box_title)
draw.text((pwr_x+25, pwr_y+80), 'Polo (+) -> B+ do TP4056', fill='#000', font=font_detail)
draw.text((pwr_x+25, pwr_y+103), 'Polo (-) -> B- do TP4056', fill='#000', font=font_detail)

draw.rectangle([(pwr_x+240, pwr_y+45), (pwr_x+505, pwr_y+150)], outline='#EF6C00', fill='#FFE0B2', width=1)
draw.text((pwr_x+250, pwr_y+53), 'MÓDULO TP4056 + CHAVE', fill='#E65100', font=font_box_title)
draw.text((pwr_x+250, pwr_y+80), 'OUT- -> Barramento Geral GND', fill='#000', font=font_detail)
draw.text((pwr_x+250, pwr_y+105), 'OUT+ -> Chave Liga/Desliga -> 5V', fill='#D32F2F', font=font_detail)
draw.text((pwr_x+250, pwr_y+128), '(Alimenta ESP32, GPS, GY-87 e Áudio)', fill='#616161', font=font_pin_sub)

# ==================== ROUTE WIRES ====================

# 1. 5V Power Rail (RED)
draw_wire([(pwr_x+505, 130), (640, 130)], C_RED, 4)
draw_wire([(640, 130), (640, 760)], C_RED, 4)
draw_wire([(640, 130), (aud_x, 130)], C_RED, 4)
draw_wire([(640, 220), (esp_x, 220)], C_RED, 4)
draw_wire([(640, 420), (gy_x+gy_w, 420)], C_RED, 3)
draw_wire([(640, 760), (gps_x+gps_w, 760)], C_RED, 3)

# 2. GND Power Rail (BLACK)
draw_wire([(pwr_x+505, 85), (665, 85)], C_BLACK, 4)
draw_wire([(665, 85), (665, 805)], C_BLACK, 4)
draw_wire([(665, 170), (aud_x, 170)], C_BLACK, 4)
draw_wire([(665, 270), (esp_x, 270)], C_BLACK, 4)
draw_wire([(665, 470), (gy_x+gy_w, 470)], C_BLACK, 3)
draw_wire([(665, 805), (gps_x+gps_w, 805)], C_BLACK, 3)
draw_wire([(esp_x+esp_w, 270), (1240, 270), (1240, 445), (ep_x, 445)], C_BLACK, 3)
draw_wire([(1240, 445), (1240, 935), (btn_x, 935)], C_BLACK, 3)

# 3. 3.3V Logic Rail to E-Paper VCC
draw_wire([(esp_x+esp_w, 320), (1260, 320), (1260, 400), (ep_x, 400)], C_RED, 3)

# 4. GY-87 Signal Wires (Left Side)
draw_wire([(gy_x+gy_w, 520), (480, 520), (480, 420), (esp_x, 420)], C_YELLOW, 3)
draw_wire([(gy_x+gy_w, 570), (510, 570), (510, 480), (esp_x, 480)], C_GREEN, 3)
draw_wire([(gy_x+gy_w, 620), (540, 620), (540, 540), (esp_x, 540)], C_WHITE, 3)

# 5. GPS Signal Wires (Left Bottom)
draw_wire([(gps_x+gps_w, 850), (480, 850), (480, 770), (esp_x, 770)], C_YELLOW, 3)
draw_wire([(gps_x+gps_w, 895), (510, 895), (510, 830), (esp_x, 830)], C_GREEN, 3)

# 6. Audio Signal Wire
draw_wire([(esp_x+esp_w, 220), (1280, 220), (1280, 210), (aud_x, 210)], C_BLUE, 3)

# 7. E-Paper Signal Wires (Right Side)
draw_wire([(esp_x+esp_w, 400), (1220, 400), (1220, 490), (ep_x, 490)], C_GREEN, 3)
draw_wire([(esp_x+esp_w, 445), (1200, 445), (1200, 535), (ep_x, 535)], C_YELLOW, 3)
draw_wire([(esp_x+esp_w, 490), (1220, 490), (1220, 580), (ep_x, 580)], C_BLUE, 3)
draw_wire([(esp_x+esp_w, 535), (1200, 535), (1200, 625), (ep_x, 625)], C_WHITE, 3)
draw_wire([(esp_x+esp_w, 580), (1220, 580), (1220, 670), (ep_x, 670)], C_ORANGE, 3)
draw_wire([(ep_x, 715), (1200, 715), (1200, 625), (esp_x+esp_w, 625)], C_PURPLE, 3)

# 8. Button Signal Wires
draw_wire([(btn_x, 795), (esp_x+esp_w, 795)], C_WHITE, 3)
draw_wire([(btn_x, 835), (esp_x+esp_w, 835)], C_WHITE, 3)
draw_wire([(btn_x, 875), (esp_x+esp_w, 875)], C_YELLOW, 3)
draw_wire([(btn_x, 915), (esp_x+esp_w, 915)], C_GREEN, 3)

# Warning Banner at Bottom
draw.rectangle([(40, 970), (w-40, 1055)], fill='#FFEBEE', outline='#C62828', width=2)
draw.text((60, 980), 'PONTOS CRÍTICOS DE MONTAGEM E SEGURANÇA ELÉTRICA:', fill='#B71C1C', font=font_warn_title)
draw.text((60, 1005), '1. ALIMENTAÇÃO DA GY-87: O pino VCC-IN deve ser ligado no 5V! O chip possui um regulador interno LDO de 3.3V. Ligar 3.3V no VCC-IN causa queda de tensão e trava o barramento I2C.', fill='#B71C1C', font=font_warn)
draw.text((60, 1025), '2. GPS NEO-6M: O pino TXD do GPS transmite para o pino RX2 (GPIO 16) do ESP32. O pino RXD do GPS recebe do TX2 (GPIO 17) do ESP32.', fill='#B71C1C', font=font_warn)
draw.text((60, 1040), '3. AMPLIFICADOR LM0080A: O alto-falante conecta EXCLUSIVAMENTE nos pinos OUT+ e OUT-. Nunca aterre nenhum terminal do falante ao GND (ponte BTL).', fill='#B71C1C', font=font_warn)

img.save('C:/Users/Mauricio/.gemini/antigravity/brain/6e41bc07-b5fb-4973-8ea6-7280d8d9fc05/diagrama_eletrico_correto.png')
print('Regenerated cleanly!')
