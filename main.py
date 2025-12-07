# main.py
# ESP32 MicroPython: Wi-Fi AP + DNS catch-all + HTTP server
# Serves index.html, /angle, /recalibrate, and OS connectivity checks.
# Requires reader.py (AngleTracker) and index.html in the filesystem.

import network
import uasyncio as asyncio
import socket
import uos
import ujson
import gc
import utime
from machine import I2C, Pin

from reader import AngleTracker

# ---------- CONFIG ----------
SSID = "Knife Angle (192.168.4.1)"
PASSWORD = "knife angle"

AP_IP = "192.168.4.1"
NETMASK = "255.255.255.0"
GATEWAY = "192.168.4.1"
DNS_IP = "8.8.8.8"

ANGLE_MODE = "PITCH"     # "PITCH", "ROLL", or "YAW"
I2C_ID = 0
I2C_SCL_PIN = 22
I2C_SDA_PIN = 21
I2C_FREQ_HZ = 400_000
READ_PERIOD_MS = 50       # sensor refresh cadence for background task
VALID_ANGLE_MODES = ("PITCH", "ROLL", "YAW")
JITTER_WARN_MULTIPLIER = 2    # log if loop gap exceeds READ_PERIOD_MS * this
JITTER_LOG_COOLDOWN_MS = 1500 # throttle jitter logs
SENSOR_WARN_MS = 40           # log if a single sensor read exceeds this
GC_PERIOD_MS = 5000
GC_MIN_FREE_BYTES = 16000

# ---------- Angle tracker ----------
def _normalize_mode_name(mode):
    if not mode:
        return None
    if not isinstance(mode, str):
        try:
            mode = str(mode)
        except Exception:
            return None
    mode_upper = mode.strip().upper()
    return mode_upper if mode_upper in VALID_ANGLE_MODES else None


default_mode = _normalize_mode_name(ANGLE_MODE) or "PITCH"
i2c = I2C(I2C_ID, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ_HZ)
tracker = AngleTracker(i2c, angle_mode=default_mode, calibration_delay_ms=1500)
current_angle_mode = tracker.angle_mode

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

# ---------- Measurement + SSE state ----------
event_clients = set()
event_lock = asyncio.Lock()
latest_delta = None
latest_age_ms = None
latest_json = ujson.dumps({"delta": None, "age_ms": None})
latest_sse_data = "data: {}\n\n".format(latest_json)


def _round_delta(delta):
    if delta is None:
        return None
    try:
        return round(delta, 4)
    except Exception:
        return delta


def update_latest(delta, age_ms):
    global latest_delta, latest_age_ms, latest_json, latest_sse_data
    latest_delta = delta
    latest_age_ms = None if age_ms is None else int(age_ms)
    payload = {
        "delta": None if delta is None else _round_delta(delta),
        "age_ms": latest_age_ms,
    }
    latest_json = ujson.dumps(payload)
    latest_sse_data = "data: {}\n\n".format(latest_json)
    return payload


# initialize snapshot
update_latest(tracker.get_last_delta(), tracker.get_last_age_ms())


async def broadcast_latest():
    async with event_lock:
        if not event_clients:
            return
        targets = list(event_clients)
    data = latest_sse_data
    for writer in targets:
        asyncio.create_task(_send_sse(writer, data))


async def register_sse_client(writer):
    async with event_lock:
        event_clients.add(writer)


async def unregister_sse_client(writer, *, close_writer=True):
    async with event_lock:
        if writer in event_clients:
            event_clients.remove(writer)
    if close_writer:
        try:
            await writer.aclose()
        except Exception:
            pass


async def _send_sse(writer, data):
    try:
        await writer.awrite(data)
    except Exception:
        await unregister_sse_client(writer)


async def serve_sse(writer):
    hdr = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: keep-alive\r\n\r\n"
    )
    try:
        await writer.awrite(hdr)
        await writer.awrite(": ok\n\n")
    except Exception:
        return
    await register_sse_client(writer)
    try:
        if latest_sse_data:
            try:
                await writer.awrite(latest_sse_data)
            except Exception:
                await unregister_sse_client(writer)
                return
        # hold the connection open; broadcast_latest will push data
        while True:
            await asyncio.sleep(30)
            async with event_lock:
                if writer not in event_clients:
                    break
    finally:
        await unregister_sse_client(writer)


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
                # Yield so heavy DNS bursts do not starve other tasks.
                await asyncio.sleep_ms(1)
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
    global current_angle_mode
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
        body = b""
        if content_length:
            try:
                body = await reader.readexactly(content_length)
            except Exception:
                body = b""

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

        elif method == "GET" and (path == "/settings" or path.startswith("/settings.html")):
            await send_file(writer, "settings.html", "text/html; charset=utf-8")

        elif method == "GET" and path == "/events":
            await serve_sse(writer)
            return

        elif method == "GET" and path == "/angle":
            delta = tracker.get_last_delta()
            age_ms = tracker.get_last_age_ms()
            update_latest(delta, age_ms)
            await send_response(writer, 200, "application/json; charset=utf-8", latest_json)

        elif path == "/angle-mode" and method == "GET":
            payload = ujson.dumps({"mode": current_angle_mode})
            await send_response(writer, 200, "application/json; charset=utf-8", payload)

        elif path == "/angle-mode" and method == "POST":
            next_mode = None
            if body:
                try:
                    raw = ujson.loads(body)
                except Exception:
                    raw = None
                if isinstance(raw, dict):
                    next_mode = _normalize_mode_name(raw.get("mode"))
                elif isinstance(raw, str):
                    next_mode = _normalize_mode_name(raw)
            if not next_mode:
                await send_response(writer, 400, "application/json; charset=utf-8", ujson.dumps({"error": "invalid mode"}))
            else:
                if next_mode != current_angle_mode:
                    tracker.set_angle_mode(next_mode)
                    await tracker.recalibrate_async()
                    current_angle_mode = tracker.angle_mode
                    update_latest(tracker.get_last_delta(), tracker.get_last_age_ms())
                payload = ujson.dumps({"mode": current_angle_mode})
                await send_response(writer, 200, "application/json; charset=utf-8", payload)

        elif path == "/recalibrate" and method in ("POST", "GET"):
            ok = await tracker.recalibrate_async()
            update_latest(tracker.get_last_delta(), tracker.get_last_age_ms())
            await send_response(writer, 200, "text/plain; charset=utf-8", "OK" if ok else "ERR")

        else:
            await send_response(writer, 404, "text/plain; charset=utf-8", "Not found")

    except Exception:
        try:
            await send_response(writer, 500, "text/plain; charset=utf-8", "Server error")
        except Exception:
            pass


# ---------- Background sensor refresh ----------
async def periodic_read():
    last_loop_ms = utime.ticks_ms()
    last_warn_ms = last_loop_ms
    while True:
        start_read = utime.ticks_ms()
        delta = tracker.get_delta()
        read_elapsed = utime.ticks_diff(utime.ticks_ms(), start_read)
        if read_elapsed > SENSOR_WARN_MS:
            print("[sensor] slow read ms=", read_elapsed)
        age_ms = tracker.get_last_age_ms()
        update_latest(delta, age_ms)
        await broadcast_latest()
        now = utime.ticks_ms()
        gap_ms = utime.ticks_diff(now, last_loop_ms)
        last_loop_ms = now
        if gap_ms > READ_PERIOD_MS * JITTER_WARN_MULTIPLIER:
            since_warn = utime.ticks_diff(now, last_warn_ms)
            if since_warn >= JITTER_LOG_COOLDOWN_MS:
                print("[jitter] periodic_read gap_ms=", gap_ms)
                last_warn_ms = now
        await asyncio.sleep_ms(READ_PERIOD_MS)


async def periodic_gc():
    while True:
        await asyncio.sleep_ms(GC_PERIOD_MS)
        try:
            async with event_lock:
                has_clients = bool(event_clients)
            if has_clients:
                continue
            if hasattr(gc, "mem_free"):
                try:
                    if gc.mem_free() > GC_MIN_FREE_BYTES:
                        continue
                except Exception:
                    pass
            gc.collect()
        except Exception:
            pass


# ---------- Main entry ----------
async def main():
    asyncio.create_task(periodic_read())
    asyncio.create_task(dns_catch_all(AP_IP))
    asyncio.create_task(periodic_gc())
    srv = await asyncio.start_server(handle_client, "0.0.0.0", 80)
    print("HTTP server on http://{}/".format(AP_IP))
    while True:
        await asyncio.sleep(3600)


try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
