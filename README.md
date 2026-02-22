# Sharpener-Gyro

```bash
mpremote connect /dev/ttyUSB0 cp main.py :main.py
mpremote connect /dev/ttyUSB0 cp reader.py :reader.py
mpremote connect /dev/ttyUSB0 cp index.html :index.html
mpremote connect /dev/ttyUSB0 cp settings.html :settings.html
mpremote connect /dev/ttyUSB0 exec "import machine; machine.reset()"
```

## Linux USB bridge (reuse the same HTML UI over serial)

This lets you use the existing `index.html` / `settings.html` UI locally in a browser,
while talking to the ESP32 over USB serial instead of Wi-Fi.

### Install bridge dependencies

```bash
python3 -m pip install -r requirements-linux-bridge.txt
```

### Run the bridge

```bash
python3 linux_bridge.py --port /dev/ttyUSB0 --http-port 8080
```

Then open:

- `http://127.0.0.1:8080/`

Notes:

- The ESP32 firmware must be the version that includes the serial JSON protocol (`status`, `stream`, `set_mode`, `recalibrate` commands).
- `linux_bridge.py` ignores normal ESP32 boot/log lines on serial and only parses JSON frames.
- If your port is different, pass `--port /dev/ttyACM0` (or the correct device).
