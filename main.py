# main.py
# ESP32 MicroPython: Wi-Fi AP + DNS catch-all + HTTP server
# Serves index.html, /angle, /recalibrate, and OS connectivity checks.
# Requires reader.py (AngleTracker) and index.html in the filesystem.

import network
import uasyncio as asyncio
import socket
import uos
import ujson
from machine import I2C, Pin

from reader import AngleTracker

# ---------- CONFIG ----------
SSID = "Knife Angle (192.168.4.1)"
PASSWORD = "knife angle"

AP_IP = "192.168.4.1"
NETMASK = "255.255.255.0"
GATEWAY = "192.168.4.1"
DNS_IP = "8.8.8.8"

ANGLE_MODE = "PITCH"     # "PITCH" or "ROLL"
I2C_ID = 0
I2C_SCL_PIN = 22
I2C_SDA_PIN = 21
I2C_FREQ_HZ = 400_000
READ_PERIOD_MS = 100      # sensor refresh cadence for background task

# ---------- Angle tracker ----------
i2c = I2C(I2C_ID, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ_HZ)
tracker = AngleTracker(i2c, angle_mode=ANGLE_MODE, calibration_delay_ms=1500)

print("Calibrating... keep sensor still.")
if tracker.recalibrate():
    print("Calibrated.")
else:
    print("Calibration failed (sensor read). Using default reference.")

# ---------- Wi-Fi AP ----------
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=SSID, password=PASSWORD, authmode=network.AUTH_WPA_WPA2_PSK)
ap.ifconfig((AP_IP, NETMASK, GATEWAY, DNS_IP))
print("AP started:", ap.ifconfig())


# ---------- DNS catch-all (all A queries → AP_IP) ----------
async def dns_catch_all(ip=AP_IP):
    addr = socket.getaddrinfo("0.0.0.0", 53)[0][-1]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception:
            pass
        s.bind(addr)
        s.setblocking(False)
        ip_bytes = bytes(int(x) & 0xFF for x in ip.split("."))
        while True:
            try:
                data, cli = s.recvfrom(512)
                if not data or len(data) < 12:
                    await asyncio.sleep_ms(1)
                    continue
                tid = data[0:2]
                flags = b"\x81\x80"
                qdcount = data[4:6]
                ancount = b"\x00\x01"
                nscount = b"\x00\x00"
                arcount = b"\x00\x00"
                header = tid + flags + qdcount + ancount + nscount + arcount
                question = data[12:]
                answer = b"\xC0\x0C" + b"\x00\x01" + b"\x00\x01" + b"\x00\x00\x00\x3C" + b"\x00\x04" + ip_bytes
                s.sendto(header + question + answer, cli)
            except OSError:
                await asyncio.sleep_ms(2)
    finally:
        try:
            s.close()
        except Exception:
            pass


# ---------- HTTP utilities ----------
async def send_response(writer, status, ctype, body):
    """Send a short text response (non-streamed)."""
    reason = {200: "OK", 204: "No Content", 404: "Not Found", 500: "Internal Server Error"}.get(status, "OK")
    if not isinstance(body, str):
        body = str(body)
    body_bytes = body.encode("utf-8")
    hdr = (
        "HTTP/1.1 {} {}\r\n"
        "Content-Type: {}\r\n"
        "Content-Length: {}\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
    ).format(status, reason, ctype, len(body_bytes))
    await writer.awrite(hdr)
    if status != 204:
        await writer.awrite(body_bytes)
    await asyncio.sleep_ms(50)


async def send_file(writer, path, ctype="text/html; charset=utf-8"):
    """Stream a file in chunks, reliable even for >8 KB."""
    try:
        size = uos.stat(path)[6]
        hdr = (
            "HTTP/1.1 200 OK\r\n"
            f"Content-Type: {ctype}\r\n"
            f"Content-Length: {size}\r\n"
            "Cache-Control: no-store\r\n"
            "Connection: close\r\n\r\n"
        )
        await writer.awrite(hdr)
        with open(path, "rb") as f:
            while True:
                chunk = f.read(512)
                if not chunk:
                    break
                await writer.awrite(chunk)
                await asyncio.sleep_ms(2)
        await asyncio.sleep_ms(100)  # allow TCP flush
    except Exception:
        await send_response(writer, 404, "text/plain; charset=utf-8", "file not found")


# ---------- HTTP handler ----------
async def handle_client(reader, writer):
    try:
        req = await reader.readline()
        if not req:
            await writer.aclose()
            return

        try:
            line = req.decode()
        except Exception:
            line = ""
        parts = line.split()
        method = parts[0] if len(parts) >= 1 else ""
        path = parts[1] if len(parts) >= 2 else "/"

        # read headers
        content_length = 0
        while True:
            h = await reader.readline()
            if not h or h == b"\r\n":
                break
            hl = h.decode().strip().lower()
            if hl.startswith("content-length:"):
                try:
                    content_length = int(hl.split(":", 1)[1].strip())
                except Exception:
                    content_length = 0
        if content_length:
            try:
                await reader.readexactly(content_length)
            except Exception:
                pass

        # ---- OS captive portal checks ----
        if method == "GET" and path in ("/generate_204", "/gen_204"):
            await send_response(writer, 204, "text/plain", "")
        elif method == "GET" and path in ("/hotspot-detect.html", "/library/test/success.html"):
            await send_response(writer, 200, "text/html", "Success")
        elif method == "GET" and path in ("/connecttest.txt", "/ncsi.txt"):
            txt = "Microsoft Connect Test" if path.endswith("connecttest.txt") else "Microsoft NCSI"
            await send_response(writer, 200, "text/plain", txt)

        # ---- application endpoints ----
        elif method == "GET" and (path == "/" or path.startswith("/index.html")):
            await send_file(writer, "index.html", "text/html; charset=utf-8")

        elif method == "GET" and path == "/angle":
            delta = tracker.get_delta()
            age_ms = tracker.get_last_age_ms()
            payload = {
                "delta": None if delta is None else delta,
                "age_ms": None if age_ms is None else int(age_ms),
            }
            body = ujson.dumps(payload)
            await send_response(writer, 200, "application/json; charset=utf-8", body)

        elif path == "/recalibrate" and method in ("POST", "GET"):
            ok = tracker.recalibrate()
            await send_response(writer, 200, "text/plain; charset=utf-8", "OK" if ok else "ERR")

        else:
            await send_response(writer, 404, "text/plain; charset=utf-8", "Not found")

    except Exception:
        try:
            await send_response(writer, 500, "text/plain; charset=utf-8", "Server error")
        except Exception:
            pass
    finally:
        await asyncio.sleep_ms(50)
        try:
            await writer.aclose()
        except Exception:
            pass


# ---------- Background sensor refresh ----------
async def periodic_read():
    while True:
        tracker.get_delta()
        await asyncio.sleep_ms(READ_PERIOD_MS)


# ---------- Main entry ----------
async def main():
    asyncio.create_task(periodic_read())
    asyncio.create_task(dns_catch_all(AP_IP))
    srv = await asyncio.start_server(handle_client, "0.0.0.0", 80)
    print("HTTP server on http://{}/".format(AP_IP))
    while True:
        await asyncio.sleep(3600)


try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
