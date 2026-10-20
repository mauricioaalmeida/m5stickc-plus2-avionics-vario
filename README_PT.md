# ✈️ M5StickC Plus 2 Aviation Avionics & Audio Variometer

<p align="center">
  🌐 <a href="README.md">English</a> | <b>Português (Brasil)</b>
</p>

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32--PICO--V3--02-orange.svg)](https://platformio.org/)
[![M5Stack](https://img.shields.io/badge/Hardware-M5StickC%20Plus%202%20%28M5--Watch%29-red.svg)](https://m5stack.com/)
[![Sensor](https://img.shields.io/badge/Baro-GY--63%20%28MS5611%29-blue.svg)](https://www.te.com/)
[![BLE](https://img.shields.io/badge/Bluetooth-LK8EX1%2010Hz%20%28XCTrack%29-green.svg)](https://xctrack.org/)
[![License](https://img.shields.io/badge/License-MIT-brightgreen.svg)](LICENSE)

<p align="center">
  <img src="docs/IMG_20261014_192710.jpg" alt="M5StickC Plus 2 Aviation Avionics & Audio Variometer" width="720" />
  <br>
  <em><b>M5StickC Plus 2</b> executando o Primary Flight Display (PFD) com horizonte artificial, altímetro de fita, variômetro vertical e telemetria BLE ativa.</em>
</p>

Simulador de instrumentos aviônicos de aviação civil e variômetro acústico de alta precisão para voo livre (parapente, asa delta, planadores e ultraleves) embarcado no **M5StickC Plus 2 (M5-Watch)** com módulo barométrico **GY-63 (MS5611)** e telemetria Bluetooth Low Energy (BLE) integrada para **XCTrack** e **XCSoar**.

Inspirado no visual dos aviônicos clássicos e no projeto *DIY Avionics Simulator* de Mirko Pavleski, adaptado com renderização de alto desempenho com double-buffering (35-40 FPS sem cintilação).

---

## 🌟 Funcionalidades Principais

* **Cockpit Glass Primário (PFD)**:
  * Horizonte artificial com graduação dinâmica de arfagem (*pitch ladder* de 5° em 5°), ângulo de inclinação (*roll scale* com ponteiro triangular) e pista de pouso virtual tridimensional em perspectiva.
  * Mira aeronáutica de referência fixa (*yellow waterline crosshair*).
  * Fita rolante de altitude (*tape altimeter*) com tambor digital em fonte grande e legível.
  * Indicador de altitude relativa de decolagem (**AGL**).
  * Variômetro vertical em barra (*VSI - Vertical Speed Indicator*) com escala de subida/descida graduada.
* **Múltiplos Modos de Exibição**:
  * **Modo 0**: PFD Cockpit Completo (Horizonte + Fita de Altitude + VSI + Status BLE/Bateria).
  * **Modo 1**: Horizonte Artificial Fullscreen com Overlay HUD.
  * **Modo 2**: Altímetro Analógico Clássico de 3 ponteiros (centenas, milhares e dezenas de milhares de pés/metros) com janela digital central.
  * **Modo 3**: Menu Interativo de Calibração Manual de Altitude com ajuste fino passo a passo e gravação na memória Flash NVS.
* **Variômetro Sonoro Acústico Inteligente**:
  * Algoritmo de tons de subida térmicos (*lift beeps*) e alarme contínuo de afundamento severo (*sink alarm*).
  * **Deadband anti-ruído**: silêncio absoluto quando o dispositivo está parado sobre a mesa (sem beeps falsos).
  * Tecla de atalho dedicada para Mudo / Som instantâneo (**MUTE / SND**).
* **Calibração e Nivelamento Dinâmico**:
  * **Auto-Tara de Inicialização**: nivela a atitude de voo (0° pitch, 0° roll) na posição de repouso ao ligar.
  * **Tara de Nível Rápida**: segure o botão frontal a qualquer momento durante o voo para renivelar o horizonte à posição atual do braço.
  * **Tara de Altitude Relativa (AGL = 0m)**: zere a referência de decolagem com um único clique.
* **Telemetria Bluetooth Low Energy (BLE)**:
  * Transmissão contínua a **10Hz** de sentenças `$LK8EX1` (pressão barométrica bruta, altitude corrigida, velocidade vertical em cm/s, temperatura e voltagem da bateria).
  * Compatível com **XCTrack**, **XCSoar**, **Flyme**, **LK8000** e **SeeYou Navigator**.
  * Reconhecimento automático via Nordic UART Service (NUS).

---

## 🛠️ Hardware e Ligações

| Componente | Função | Conexão no M5StickC Plus 2 |
| :--- | :--- | :--- |
| **M5StickC Plus 2** | MCU ESP32-PICO-V3-02, Display 240x135, Buzzer, IMU 6-Eixos, Bateria | Placa Principal |
| **Módulo GY-63 (MS5611)** | Sensor barométrico de altíssima resolução (10-20 cm) | **Porta Grove A (Conector Vermelho/Preto)** |

### Pinagem da Porta Grove A (M5StickC Plus 2):
* **Fio Amarelo (SDA)**: `GPIO 32`
* **Fio Branco (SCL)**: `GPIO 33`
* **Fio Vermelho (VCC)**: `5V` *(alimentado automaticamente pelo circuito boost interno)*
* **Fio Preto (GND)**: `GND`

> 💡 **Nota sobre o Sensor**: O firmware possui autodetecção dos endereços I2C `0x77` e `0x76` a 100/400 kHz e reconexão automática em segundo plano. Se o sensor não estiver conectado, o dispositivo permanece estável, fixado na altitude de referência, sem oscilações e 100% silencioso.

---

## 🎮 Guia Completo dos Botões

O M5StickC Plus 2 possui 3 botões físicos:
* **Botão A (Frontal M5)**: GPIO 37
* **Botão B (Lateral Inferior)**: GPIO 39
* **Botão PWR / C (Lateral Superior / Ligar)**: GPIO 35

```
+------------------------------------------------------+
| [PWR/C] (Lateral Superior)                           |
|                                                      |
|   +-----------------------+      +---------------+   |
|   |                       |      |               |   |
|   |      TELA LCD         |      |    [Btn A]    |   |
|   |      240 x 135        |      |    (Frontal)  |   |
|   |                       |      |               |   |
|   +-----------------------+      +---------------+   |
|                                                      |
| [Btn B] (Lateral Inferior)                           |
+------------------------------------------------------+
```

### 1. Modos Normais de Voo (PFD / HUD / Dial)

| Botão | Ação | Função |
| :--- | :--- | :--- |
| **Botão A (Frontal M5)** | **Toque Curto (Click)** | **Mute / Unmute**: Liga ou desliga instantaneamente o áudio do variômetro (com bip e toast visual). |
| **Botão A (Frontal M5)** | **Manter Pressionado por 3s** | **ZERAR (AGL 0m / Nível)**: Zera a altitude relativa (`AGL = 0m`) e nivela o horizonte artificial na posição atual do braço. |
| **Botão A (Frontal M5)** | **Duplo Clique** | **Ligar / Desligar Modo DEMO**: Ativa/desativa simulação térmica. |
| **Botão B (Lateral)** | **Toque Curto (Click)** | **Alternar Telas**: Alterna em loop entre `PFD Cockpit` ➔ `HUD Horizonte` ➔ `Dial Analógico`. |
| **Botão B (Lateral)** | **Segurar (> 1s)** | **Menu de Calibração**: Abre o Menu Interativo de Ajuste Fino de Altitude. |
| **Botão B (Lateral)** | **Duplo Clique** | **Ligar / Desligar Modo DEMO** (Atalho alternativo). |
| **Botão PWR (Lateral)**| **Toque Curto (Click)** | **Zera Altitude AGL**: Atalho rápido para zerar a altitude relativa. |

### 2. Modo 3: Menu Interativo de Ajuste de Altitude

Ao entrar no Menu de Ajuste, a edição se inicia na altitude inteira atual:

| Botão | Ação | Função |
| :--- | :--- | :--- |
| **Botão A (Frontal)** | **Toque Curto (Click)** | **Aumenta exatamente 1 metro (+1m)** na altitude alvo. |
| **Botão B (Lateral)** | **Toque Curto (Click)** | **Diminui exatamente 1 metro (-1m)** na altitude alvo. |
| **Botão A (Frontal)** | **Segurar (> 1s)** | **SALVAR**: Grava o valor na memória Flash NVS, recalibra a pressão QNH correspondente e retorna ao PFD. |
| **Botão PWR (Lateral)**| **Toque Curto (Click)** | **CANCELAR**: Sai do menu sem salvar alterações. |

---

## 📱 Conexão com o XCTrack (Android)

1. Ligue o M5StickC Plus 2. A barra superior indicará `BLE:ADV`.
2. Abra o **XCTrack** no seu smartphone ou tablet Android.
3. Vá em **Preferências (Configurações)** ➔ **Conexão & Sensores** ➔ **Sensor Externo**.
4. Selecione **Bluetooth LE (Low Energy)**.
5. Toque em **Procurar Dispositivos** e selecione **`M5-VARIO`**.
6. O protocolo será detectado automaticamente como **Nordic UART / LK8EX1**.
7. Na tela do M5StickC Plus 2, o indicador mudará para **`BLE:XCT`** em verde, confirmando a transmissão de altitude e VSI em tempo real!

---

## 💻 Compilação e Gravação com PlatformIO

### Pré-requisitos
* [Visual Studio Code](https://code.visualstudio.com/) com a extensão **PlatformIO IDE** instalada.
* Cabo USB-C conectado ao M5StickC Plus 2.

### Comandos pelo Terminal
```bash
# Clone ou acesse a pasta do projeto
cd c:/_MAA/Desenv/ESP32C3_BLUETOOTH_AUDIO_VARIO-main

# Compilar o firmware para o ambiente M5StickC Plus 2
pio run -e m5stickc_plus2

# Gravar o firmware na placa conectada (ex: COM8)
pio run -e m5stickc_plus2 -t upload --upload-port COM8

# Abrir o monitor serial para verificar os logs de boot
pio device monitor -p COM8 -b 115200
```

---

## ⚡ Gravação Rápida em 1 Clique (PowerShell)

Para gravar diretamente sem precisar compilar tudo manualmente:

```powershell
# Gravação instantânea a 1.500.000 baud
.\flash_m5stickc.ps1 -port COM8
```

---

## 🏪 Como Gerar e Publicar no Marketplace do M5Stack (M5Burner)

O **M5Burner** é a loja e gerenciador oficial de firmwares da M5Stack, permitindo que qualquer usuário no mundo instale seu projeto com 1 clique!

### Passo 1: Gerar o arquivo binário único unificado (.bin)
O M5Burner requer uma imagem de firmware completa (ou os arquivos separados com seus respectivos endereços). Você pode gerar tudo automaticamente com um único comando:

```bash
# Gera o binário único mesclado (m5stickc_plus2_vario_m5burner.bin) e já registra no seu M5Burner local
py -3.11 generate_m5burner_bin.py
```

Ou, se preferir fazer manualmente:
1. Compile o projeto via PlatformIO:
   ```bash
   pio run -e m5stickc_plus2
   ```
2. O PlatformIO gera os binários em `.pio/build/m5stickc_plus2/`:
   * `bootloader.bin` (endereço `0x1000`)
   * `partitions.bin` (endereço `0x8000`)
   * `boot_app0.bin` (endereço `0xe000`)
   * `firmware.bin` (endereço `0x10000`)

3. Mescle em um único arquivo com o esptool:
   ```bash
   python -m esptool --chip esp32 merge-bin -o m5stickc_plus2_vario_m5burner.bin --flash-mode dio --flash-freq 40m --flash-size 8MB 0x1000 .pio/build/m5stickc_plus2/bootloader.bin 0x8000 .pio/build/m5stickc_plus2/partitions.bin 0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin 0x10000 .pio/build/m5stickc_plus2/firmware.bin
   ```

### Passo 2: Publicar no M5Burner
1. Baixe e instale o [M5Burner](https://m5stack.com/pages/download).
2. Abra o M5Burner e faça login com a sua conta M5Stack (canto superior direito).
3. No menu lateral, selecione a categoria do seu dispositivo: **StickC**.
4. Clique no botão superior direito **Publish** (ou **User Custom** / **Upload**).
5. Preencha as informações do projeto:
   * **Title**: `M5StickC Plus 2 Avionics Horizon & Variometer`
   * **Category**: `Aviation` / `Sports` / `Sensors`
   * **Device**: `M5StickC Plus 2`
   * **Description**: Descreva as funções (Horizonte Artificial, Vario GY-63, BLE LK8EX1 para XCTrack, calibração NVS).
   * **Screenshots / Cover**: Adicione fotos do dispositivo em funcionamento no relógio.
   * **Firmware File**: Anexe o arquivo `m5stickc_plus2_vario_m5burner.bin` gerado.
6. Clique em **Submit**. Após uma rápida validação da M5Stack, seu projeto estará disponível publicamente para a comunidade global!

---

## 🚀 Como Subir o Projeto para o seu GitHub

Para publicar seu repositório completo com código e documentação no seu perfil do GitHub:

```bash
# 1. Verifique o status dos arquivos
git status

# 2. Crie uma branch principal limpa (main)
git branch -M main

# 3. Adicione os arquivos
git add .

# 4. Faça o commit com mensagem descritiva
git commit -m "feat: M5StickC Plus2 Aviation Avionics Simulator, GY-63 Vario, and BLE LK8EX1"

# 5. Envie para o GitHub
git push -u origin main
```

---

## 📜 Licença e Créditos

* **Algoritmo de Horizonte Artificial**: Inspirado no projeto original *DIY Avionics Simulator* por [Mirko Pavleski](https://projecthub.arduino.cc/mircemk/diy-avionics-simulator-with-esp32-artificial-horizon-compass-altimeter-f39fac).
* **Filtro de Fusão Kalman 4D**: Algoritmo KF4D baseado em *ESP32_IMU_GPS_BARO_VARIO* por [har-in-air](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO).
* **Bibliotecas**: [M5Unified](https://github.com/m5stack/M5Unified), [M5GFX](https://github.com/m5stack/M5GFX), [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino).
* **Licença**: MIT. Sinta-se livre para usar, modificar e compartilhar!
