#!/usr/bin/env bash
# Сборка и заливка obd_dash на ESP32-C3 (ESP32-2424S012) через arduino-cli.
# Запуск из Git Bash:  bash flash.sh            собрать + залить по USB (COM5)
#                      bash flash.sh build      только компиляция
#                      bash flash.sh bin        собрать и показать путь к .bin (для OTA)
#                      bash flash.sh monitor    serial-монитор (не работает при CDC — см. ниже)
#
# ОБНОВЛЕНИЕ ПО WiFi (OTA):
#   1) bash flash.sh bin   -> покажет путь к obd_dash.ino.bin
#   2) телефон -> сеть OBD-Dash-XXXX -> http://192.168.4.1/ -> "Обновить прошивку"
#   3) выбрать этот .bin -> Прошить. Плата перезагрузится сама.
#
# Serial-монитор: CDCOnBoot=cdc не отдаёт лог arduino-cli monitor.
#   Читать через pyserial:
#   "C:/Espressif/tools/python/v6.1/venv/Scripts/python.exe" -c \
#     "import serial,sys;s=serial.Serial('COM5',115200);[sys.stdout.write(s.read(999).decode('u8','replace')) for _ in iter(int,1)]"

set -e

CLI="/c/Users/Константин/Documents/Arduino/_tools/arduino-cli.exe"
SKETCH="/c/Users/Константин/Documents/Arduino/obd_dash"
PORT="COM5"

# ESP32C3 Dev Module. Ключевое:
#   CDCOnBoot=cdc            -> Serial по встроенному USB
#   PartitionScheme=min_spiffs -> 1.9MB app x2 + OTA (обновление по WiFi)
#   FlashMode=dio, FlashFreq=80, FlashSize=4M
FQBN="esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=min_spiffs,FlashMode=dio,FlashFreq=80,FlashSize=4M,UploadSpeed=921600,CPUFreq=160"

# версия = короткий git-хеш (+ звёздочка если есть незакоммиченные правки)
VER="$(git -C "$SKETCH" rev-parse --short HEAD 2>/dev/null || echo dev)"
git -C "$SKETCH" diff --quiet 2>/dev/null || VER="${VER}*"
VPROP="compiler.cpp.extra_flags=-DFW_VERSION=\"$VER\""
echo ">>> версия сборки: $VER"

case "${1:-flash}" in
  build)
    "$CLI" compile --fqbn "$FQBN" --build-property "$VPROP" "$SKETCH"
    ;;
  bin)
    OUT="$SKETCH/_build"
    "$CLI" compile --fqbn "$FQBN" --build-property "$VPROP" --output-dir "$OUT" "$SKETCH"
    echo
    echo ">>> Файл для OTA (версия $VER):"
    ls -la "$OUT"/*.ino.bin
    ;;
  monitor)
    "$CLI" monitor -p "$PORT" --config baudrate=115200
    ;;
  flash|*)
    "$CLI" compile --fqbn "$FQBN" --build-property "$VPROP" "$SKETCH"
    "$CLI" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"
    echo
    echo ">>> Залито по USB (версия $VER). Обновлять по WiFi:  bash flash.sh bin"
    ;;
esac
