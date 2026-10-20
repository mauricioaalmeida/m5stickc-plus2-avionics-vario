"""
Script to build and merge firmware for M5StickC Plus2 for M5Burner
Creates a single monolithic .bin ready to flash at offset 0x0
"""

import os
import sys
import shutil
import json
import subprocess

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(PROJECT_DIR, ".pio", "build", "m5stickc_plus2")
OUTPUT_BIN = os.path.join(PROJECT_DIR, "m5stickc_plus2_vario_m5burner.bin")
BURNER_DIR = r"C:\Users\Mauricio\Downloads\M5Burner-v3-beta-win-x64"
BOOT_APP0 = os.path.expanduser(r"~\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin")

def main():
    print("=" * 60)
    print("Gerando .bin completo do M5StickC Plus2 para o M5Burner...")
    print("=" * 60)

    # 1. Verificar/Compilar projeto se necessario
    bootloader = os.path.join(BUILD_DIR, "bootloader.bin")
    partitions = os.path.join(BUILD_DIR, "partitions.bin")
    firmware = os.path.join(BUILD_DIR, "firmware.bin")

    if not (os.path.exists(bootloader) and os.path.exists(partitions) and os.path.exists(firmware)):
        print("Compilando firmware com PlatformIO...")
        res = subprocess.run(["pio", "run", "-e", "m5stickc_plus2"], cwd=PROJECT_DIR)
        if res.returncode != 0:
            print("Erro ao compilar com PlatformIO!")
            sys.exit(1)

    if not os.path.exists(BOOT_APP0):
        print(f"Erro: boot_app0.bin nao encontrado em {BOOT_APP0}")
        sys.exit(1)

    # 2. Executar merge-bin com esptool
    print("\nMesclando bootloader, partitions, boot_app0 e firmware...")
    merge_cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32",
        "merge-bin",
        "-o", OUTPUT_BIN,
        "--flash-mode", "dio",
        "--flash-freq", "40m",
        "--flash-size", "8MB",
        "0x1000", bootloader,
        "0x8000", partitions,
        "0xe000", BOOT_APP0,
        "0x10000", firmware
    ]

    res = subprocess.run(merge_cmd)
    if res.returncode != 0:
        print("Tentando com fallback de comando esptool...")
        # fallback with deprecated flags if older esptool
        fallback_cmd = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32",
            "merge_bin",
            "-o", OUTPUT_BIN,
            "--flash_mode", "dio",
            "--flash_freq", "40m",
            "--flash_size", "8MB",
            "0x1000", bootloader,
            "0x8000", partitions,
            "0xe000", BOOT_APP0,
            "0x10000", firmware
        ]
        res = subprocess.run(fallback_cmd)
        if res.returncode != 0:
            print("Falha ao mesclar binarios!")
            sys.exit(1)

    file_size = os.path.getsize(OUTPUT_BIN)
    print(f"\n[SUCESSO] Binario unico gerado:")
    print(f"  -> Arquivo: {OUTPUT_BIN}")
    print(f"  -> Tamanho: {file_size:,} bytes ({file_size / (1024*1024):.2f} MB)")
    print(f"  -> Endereco de gravacao no M5Burner: 0x0000 (0x0)")

    # 3. Integrar diretamente com M5Burner caso esteja presente
    if os.path.isdir(BURNER_DIR):
        print("\nM5Burner detectado em:", BURNER_DIR)
        burner_fw_dir = os.path.join(BURNER_DIR, "packages", "firmware")
        os.makedirs(burner_fw_dir, exist_ok=True)
        
        burner_bin_name = "m5stickc_plus2_avionics_vario.bin"
        target_fw_path = os.path.join(burner_fw_dir, burner_bin_name)
        shutil.copyfile(OUTPUT_BIN, target_fw_path)
        print(f"  -> Copiado para {target_fw_path}")

        # Atualizar local.json para aparecer como firmware disponivel no M5Burner
        local_json_path = os.path.join(BURNER_DIR, "packages", "dat", "local.json")
        if os.path.exists(local_json_path):
            try:
                with open(local_json_path, "r", encoding="utf-8") as f:
                    fw_list = json.load(f)

                # Verificar se ja existe nossa entrada
                existing_idx = None
                for idx, entry in enumerate(fw_list):
                    if entry.get("_id") == "custom_m5stickc_plus2_vario" or entry.get("name") == "Avionics Vario (M5StickC Plus2)":
                        existing_idx = idx
                        break

                custom_entry = {
                    "_id": "custom_m5stickc_plus2_vario",
                    "fid": "custom_m5stickc_plus2_vario",
                    "name": "Avionics Vario (M5StickC Plus2)",
                    "description": "Horizonte Artificial e Variometro com Bluetooth e Audio",
                    "cover": "17c3a34b8f0578d104f295df92b6d02a.png",
                    "category": "stickc",
                    "tags": ["Avionics", "Vario", "StickC-Plus2"],
                    "author": "Mauricio Almeida",
                    "github": "https://github.com/mauricioaalmeida/m5stickc-plus2-avionics-vario",
                    "download": 100,
                    "versions": [
                        {
                            "version": "v1.0.0",
                            "published_at": "2026-10-14",
                            "file": burner_bin_name,
                            "published": True,
                            "change_log": "Build completo integrado com 8MB flash"
                        }
                    ],
                    "skipBindingUiFlow2": True,
                    "odevice": True
                }

                if existing_idx is not None:
                    fw_list[existing_idx] = custom_entry
                else:
                    fw_list.insert(0, custom_entry)

                with open(local_json_path, "w", encoding="utf-8") as f:
                    json.dump(fw_list, f, indent=2, ensure_ascii=False)
                print("  -> Registrado com sucesso no catalogo local do M5Burner!")
                print("     (Aparecera no M5Burner na aba 'STICKC' pronto com botao 'Burn')")
            except Exception as e:
                print(f"  -> Aviso ao atualizar local.json do M5Burner: {e}")

    print("\nProcesso finalizado com sucesso!")

if __name__ == "__main__":
    main()
