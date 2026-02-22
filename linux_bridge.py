#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import threading
import time
import webbrowser
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - handled at runtime
    serial = None

try:
    from aiohttp import web
except ImportError:  # pragma: no cover - handled at runtime
    web = None


ROOT_DIR = Path(__file__).resolve().parent
INDEX_HTML = ROOT_DIR / "index.html"
SETTINGS_HTML = ROOT_DIR / "settings.html"

VALID_MODES = {"AXIS_X", "AXIS_Y", "AXIS_Z"}
LEGACY_MODE_ALIASES = {
    "ROLL": "AXIS_X",
    "PITCH": "AXIS_Y",
    "YAW": "AXIS_Z",
}


def normalize_mode_name(value: Any) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        value = str(value)
    upper = value.strip().upper()
    if upper in VALID_MODES:
        return upper
    return LEGACY_MODE_ALIASES.get(upper)


def utc_ms() -> int:
    return int(time.time() * 1000)


@dataclass
class PendingCommand:
    matcher: Callable[[dict[str, Any]], bool]
    future: asyncio.Future[dict[str, Any]]
    cmd_name: str


class SerialBridge:
    def __init__(self, port: str, baud: int, logger: logging.Logger) -> None:
        self.port = port
        self.baud = baud
        self.log = logger

        self.loop: asyncio.AbstractEventLoop | None = None
        self._stop = False
        self._manager_task: asyncio.Task[None] | None = None

        self._serial: Any = None
        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()
        self._disconnect_lock = asyncio.Lock()
        self._command_lock = asyncio.Lock()
        self._write_lock = threading.Lock()
        self._pending_command: PendingCommand | None = None

        self._sse_clients: set[asyncio.Queue[dict[str, Any]]] = set()
        self._sse_lock = asyncio.Lock()

        self.latest_telemetry: dict[str, Any] = {
            "delta": None,
            "age_ms": None,
            "mode": None,
            "t_ms": None,
            "ok": None,
        }
        self.device_status: dict[str, Any] | None = None
        self.bridge_state: dict[str, Any] = {
            "connected": False,
            "port": self.port,
            "baud": self.baud,
            "connect_attempts": 0,
            "reconnect_count": 0,
            "last_connected_at_ms": None,
            "last_disconnected_at_ms": None,
            "last_frame_at_ms": None,
            "last_error": None,
            "last_log_line": None,
            "rx_json_count": 0,
            "rx_non_json_count": 0,
            "tx_count": 0,
            "last_response": None,
        }

    async def start(self) -> None:
        self.loop = asyncio.get_running_loop()
        self._stop = False
        if self._manager_task is None or self._manager_task.done():
            self._manager_task = asyncio.create_task(self._connection_manager())

    async def close(self) -> None:
        self._stop = True
        if self._manager_task:
            self._manager_task.cancel()
            try:
                await self._manager_task
            except asyncio.CancelledError:
                pass
            self._manager_task = None
        await self._disconnect("bridge shutdown")

    async def _connection_manager(self) -> None:
        backoff_s = 0.5
        while not self._stop:
            if self.bridge_state["connected"]:
                await asyncio.sleep(0.5)
                continue
            try:
                await self._connect_once()
                backoff_s = 0.5
            except Exception as exc:
                self.bridge_state["last_error"] = f"connect:{exc}"
                self.log.warning("Serial connect failed on %s: %s", self.port, exc)
                await asyncio.sleep(backoff_s)
                backoff_s = min(backoff_s * 2.0, 2.0)

    async def _connect_once(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")

        self.bridge_state["connect_attempts"] += 1
        ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=1.0)  # type: ignore[attr-defined]
        self._serial = ser
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="esp32-serial-reader",
            daemon=True,
        )
        self._reader_thread.start()

        now = utc_ms()
        self.bridge_state["connected"] = True
        self.bridge_state["last_connected_at_ms"] = now
        if self.bridge_state["reconnect_count"] == 0:
            self.log.info("Connected to %s @ %d", self.port, self.baud)
        else:
            self.log.info("Reconnected to %s @ %d", self.port, self.baud)
        self.bridge_state["reconnect_count"] += 1
        self.bridge_state["last_error"] = None

        # Prime the device state but don't fail the whole connect if commands timeout.
        try:
            await self.send_command({"cmd": "status"}, timeout=1.5)
        except Exception as exc:
            self.log.debug("Initial status request failed: %s", exc)
        try:
            await self.send_command({"cmd": "stream", "enabled": True}, timeout=1.5)
        except Exception as exc:
            self.log.debug("Initial stream enable failed: %s", exc)

    def _reader_loop(self) -> None:
        assert self.loop is not None
        while not self._reader_stop.is_set():
            ser = self._serial
            if ser is None:
                return
            try:
                raw = ser.readline()
            except Exception as exc:
                reason = f"serial read error: {exc}"
                self.loop.call_soon_threadsafe(
                    lambda r=reason: asyncio.create_task(self._disconnect(r))
                )
                return
            if not raw:
                continue
            self.loop.call_soon_threadsafe(self._on_serial_line, raw)

    def _on_serial_line(self, raw: bytes) -> None:
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return
        self.bridge_state["last_frame_at_ms"] = utc_ms()

        if not line.startswith("{"):
            self.bridge_state["rx_non_json_count"] += 1
            self.bridge_state["last_log_line"] = line
            # ESP32 prints logs and boot lines on the same serial output; ignore them.
            self.log.debug("SERIAL LOG: %s", line)
            return

        try:
            frame = json.loads(line)
        except json.JSONDecodeError:
            self.bridge_state["rx_non_json_count"] += 1
            self.bridge_state["last_log_line"] = line
            self.log.debug("SERIAL BAD JSON: %s", line)
            return
        if not isinstance(frame, dict):
            self.bridge_state["rx_non_json_count"] += 1
            return

        self.bridge_state["rx_json_count"] += 1
        asyncio.create_task(self._handle_json_frame(frame))

    async def _handle_json_frame(self, frame: dict[str, Any]) -> None:
        frame_type = str(frame.get("type") or "").strip().lower()

        if frame_type == "telemetry":
            self._update_from_telemetry(frame)
            await self._broadcast_angle_event()
        elif frame_type == "status":
            self._update_from_status(frame)
        elif frame_type in {"response", "error"}:
            self.bridge_state["last_response"] = {
                "type": frame.get("type"),
                "cmd": frame.get("cmd"),
                "ok": frame.get("ok"),
                "error": frame.get("error"),
                "at_ms": utc_ms(),
            }
            if frame_type == "error":
                self.bridge_state["last_error"] = str(frame.get("error") or "device error")

        pending = self._pending_command
        if pending and not pending.future.done():
            try:
                if pending.matcher(frame):
                    pending.future.set_result(frame)
            except Exception as exc:
                pending.future.set_exception(exc)

    def _update_from_telemetry(self, frame: dict[str, Any]) -> None:
        self.latest_telemetry["delta"] = frame.get("delta")
        self.latest_telemetry["age_ms"] = frame.get("age_ms")
        mode = normalize_mode_name(frame.get("mode"))
        if mode:
            self.latest_telemetry["mode"] = mode
        self.latest_telemetry["t_ms"] = frame.get("t_ms")
        self.latest_telemetry["ok"] = frame.get("ok")

    def _update_from_status(self, frame: dict[str, Any]) -> None:
        self.device_status = frame
        mode = normalize_mode_name(frame.get("mode"))
        if mode:
            self.latest_telemetry["mode"] = mode
        latest = frame.get("latest")
        if isinstance(latest, dict):
            if "delta" in latest:
                self.latest_telemetry["delta"] = latest.get("delta")
            if "age_ms" in latest:
                self.latest_telemetry["age_ms"] = latest.get("age_ms")

    async def _broadcast_angle_event(self) -> None:
        payload = self.get_angle_payload()
        async with self._sse_lock:
            clients = list(self._sse_clients)
        for queue in clients:
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            try:
                queue.put_nowait(payload)
            except asyncio.QueueFull:
                pass

    async def subscribe_events(self) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=4)
        async with self._sse_lock:
            self._sse_clients.add(queue)
        return queue

    async def unsubscribe_events(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        async with self._sse_lock:
            self._sse_clients.discard(queue)

    async def _disconnect(self, reason: str) -> None:
        async with self._disconnect_lock:
            was_connected = bool(self.bridge_state["connected"])
            self._reader_stop.set()

            if self._pending_command and not self._pending_command.future.done():
                self._pending_command.future.set_exception(RuntimeError(reason))
            self._pending_command = None

            ser = self._serial
            self._serial = None
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

            thread = self._reader_thread
            self._reader_thread = None
            if thread and thread.is_alive():
                thread.join(timeout=0.5)

            self.bridge_state["connected"] = False
            self.bridge_state["last_disconnected_at_ms"] = utc_ms()
            self.bridge_state["last_error"] = reason
            if was_connected:
                self.log.warning("Serial disconnected: %s", reason)

    def is_connected(self) -> bool:
        return bool(self.bridge_state["connected"] and self._serial is not None)

    def get_angle_payload(self) -> dict[str, Any]:
        return {
            "delta": self.latest_telemetry.get("delta"),
            "age_ms": self.latest_telemetry.get("age_ms"),
        }

    async def send_command(
        self,
        cmd: dict[str, Any],
        *,
        timeout: float,
    ) -> dict[str, Any]:
        if not self.is_connected():
            raise RuntimeError("serial device not connected")

        cmd_name = str(cmd.get("cmd") or "").strip().lower()
        if not cmd_name:
            raise ValueError("command must include 'cmd'")

        def matcher(frame: dict[str, Any]) -> bool:
            frame_type = str(frame.get("type") or "").lower()
            if cmd_name == "status":
                return frame_type == "status"
            if frame_type not in {"response", "error"}:
                return False
            return str(frame.get("cmd") or "").strip().lower() == cmd_name

        async with self._command_lock:
            if not self.is_connected():
                raise RuntimeError("serial device not connected")
            loop = asyncio.get_running_loop()
            future: asyncio.Future[dict[str, Any]] = loop.create_future()
            self._pending_command = PendingCommand(matcher=matcher, future=future, cmd_name=cmd_name)
            try:
                self._write_json_line(cmd)
                response = await asyncio.wait_for(future, timeout=timeout)
            finally:
                self._pending_command = None

        if str(response.get("type") or "").lower() == "error":
            raise RuntimeError(str(response.get("error") or "device error"))
        return response

    def _write_json_line(self, payload: dict[str, Any]) -> None:
        if not self.is_connected():
            raise RuntimeError("serial device not connected")
        line = json.dumps(payload, separators=(",", ":")) + "\n"
        data = line.encode("utf-8")
        ser = self._serial
        if ser is None:
            raise RuntimeError("serial device not connected")
        try:
            with self._write_lock:
                ser.write(data)
                ser.flush()
        except Exception as exc:
            if self.loop:
                reason = f"serial write error: {exc}"
                self.loop.call_soon_threadsafe(
                    lambda r=reason: asyncio.create_task(self._disconnect(r))
                )
            raise
        self.bridge_state["tx_count"] += 1

    async def request_status(self, timeout: float = 1.0) -> dict[str, Any] | None:
        if not self.is_connected():
            return self.device_status
        try:
            frame = await self.send_command({"cmd": "status"}, timeout=timeout)
            if isinstance(frame, dict):
                return frame
        except Exception as exc:
            self.bridge_state["last_error"] = f"status:{exc}"
        return self.device_status

    async def set_mode(self, mode: str, timeout: float = 5.0) -> str:
        normalized = normalize_mode_name(mode)
        if not normalized:
            raise ValueError("invalid mode")
        frame = await self.send_command({"cmd": "set_mode", "mode": normalized}, timeout=timeout)
        resolved = normalize_mode_name(frame.get("mode")) if isinstance(frame, dict) else None
        if resolved:
            self.latest_telemetry["mode"] = resolved
            return resolved
        return normalized

    async def recalibrate(self, timeout: float = 6.0) -> bool:
        frame = await self.send_command({"cmd": "recalibrate"}, timeout=timeout)
        return bool(frame.get("ok")) if isinstance(frame, dict) else False

    async def set_stream(self, enabled: bool, timeout: float = 1.5) -> bool:
        frame = await self.send_command({"cmd": "stream", "enabled": bool(enabled)}, timeout=timeout)
        return bool(frame.get("enabled", enabled)) if isinstance(frame, dict) else bool(enabled)

    def merged_status_snapshot(self) -> dict[str, Any]:
        device = json.loads(json.dumps(self.device_status)) if self.device_status is not None else {}
        latest = device.get("latest")
        if not isinstance(latest, dict):
            latest = {}
            device["latest"] = latest
        latest["delta"] = self.latest_telemetry.get("delta")
        latest["age_ms"] = self.latest_telemetry.get("age_ms")
        if not device.get("mode") and self.latest_telemetry.get("mode"):
            device["mode"] = self.latest_telemetry.get("mode")
        device["bridge"] = {
            "connected": self.is_connected(),
            "port": self.port,
            "baud": self.baud,
            "connect_attempts": self.bridge_state.get("connect_attempts"),
            "reconnect_count": self.bridge_state.get("reconnect_count"),
            "last_connected_at_ms": self.bridge_state.get("last_connected_at_ms"),
            "last_disconnected_at_ms": self.bridge_state.get("last_disconnected_at_ms"),
            "last_frame_at_ms": self.bridge_state.get("last_frame_at_ms"),
            "last_error": self.bridge_state.get("last_error"),
            "last_log_line": self.bridge_state.get("last_log_line"),
            "rx_json_count": self.bridge_state.get("rx_json_count"),
            "rx_non_json_count": self.bridge_state.get("rx_non_json_count"),
            "tx_count": self.bridge_state.get("tx_count"),
            "last_response": self.bridge_state.get("last_response"),
        }
        return device


async def file_response(path: Path) -> web.FileResponse:
    return web.FileResponse(path)


async def handle_root(request: web.Request) -> web.StreamResponse:
    return await file_response(INDEX_HTML)


async def handle_settings_page(request: web.Request) -> web.StreamResponse:
    return await file_response(SETTINGS_HTML)


def get_bridge(request: web.Request) -> SerialBridge:
    return request.app["serial_bridge"]  # type: ignore[return-value]


async def handle_angle(request: web.Request) -> web.Response:
    bridge = get_bridge(request)
    if bridge.get_angle_payload()["delta"] is None and bridge.is_connected():
        await bridge.request_status(timeout=0.8)
    return web.json_response(bridge.get_angle_payload(), headers={"Cache-Control": "no-store"})


async def handle_status(request: web.Request) -> web.Response:
    bridge = get_bridge(request)
    await bridge.request_status(timeout=1.0)
    return web.json_response(bridge.merged_status_snapshot(), headers={"Cache-Control": "no-store"})


async def handle_angle_mode_get(request: web.Request) -> web.Response:
    bridge = get_bridge(request)
    mode = normalize_mode_name(bridge.latest_telemetry.get("mode"))
    if mode is None:
        status = await bridge.request_status(timeout=1.0)
        mode = normalize_mode_name((status or {}).get("mode") if isinstance(status, dict) else None)
    if mode is None:
        mode = "AXIS_Y"
    return web.json_response({"mode": mode}, headers={"Cache-Control": "no-store"})


async def handle_angle_mode_post(request: web.Request) -> web.Response:
    bridge = get_bridge(request)
    mode: str | None = None
    try:
        raw = await request.json()
    except Exception:
        raw = None
    if isinstance(raw, dict):
        mode = normalize_mode_name(raw.get("mode"))
    elif isinstance(raw, str):
        mode = normalize_mode_name(raw)
    if not mode:
        return web.json_response({"error": "invalid mode"}, status=400)
    if not bridge.is_connected():
        return web.json_response({"error": "device not connected"}, status=503)
    try:
        resolved = await bridge.set_mode(mode)
    except Exception as exc:
        return web.json_response({"error": str(exc)}, status=502)
    return web.json_response({"mode": resolved}, headers={"Cache-Control": "no-store"})


async def handle_recalibrate(request: web.Request) -> web.Response:
    bridge = get_bridge(request)
    if not bridge.is_connected():
        return web.Response(text="ERR", status=503, content_type="text/plain")
    try:
        ok = await bridge.recalibrate()
    except Exception:
        ok = False
    return web.Response(text="OK" if ok else "ERR", content_type="text/plain", headers={"Cache-Control": "no-store"})


async def handle_events(request: web.Request) -> web.StreamResponse:
    bridge = get_bridge(request)
    resp = web.StreamResponse(
        status=200,
        headers={
            "Content-Type": "text/event-stream",
            "Cache-Control": "no-store",
            "Connection": "keep-alive",
        },
    )
    await resp.prepare(request)
    queue = await bridge.subscribe_events()

    try:
        await resp.write(b": ok\n\n")
        initial = bridge.get_angle_payload()
        await resp.write(f"data: {json.dumps(initial)}\n\n".encode("utf-8"))
        while True:
            try:
                payload = await asyncio.wait_for(queue.get(), timeout=25.0)
                await resp.write(f"data: {json.dumps(payload)}\n\n".encode("utf-8"))
            except asyncio.TimeoutError:
                await resp.write(b": keepalive\n\n")
    except (ConnectionResetError, asyncio.CancelledError, BrokenPipeError):
        pass
    finally:
        await bridge.unsubscribe_events(queue)
        try:
            await resp.write_eof()
        except Exception:
            pass
    return resp


async def on_startup(app: web.Application) -> None:
    bridge: SerialBridge = app["serial_bridge"]
    await bridge.start()


async def on_cleanup(app: web.Application) -> None:
    bridge: SerialBridge = app["serial_bridge"]
    await bridge.close()


def create_app(bridge: SerialBridge) -> web.Application:
    app = web.Application()
    app["serial_bridge"] = bridge
    app.router.add_get("/", handle_root)
    app.router.add_get("/index.html", handle_root)
    app.router.add_get("/settings", handle_settings_page)
    app.router.add_get("/settings.html", handle_settings_page)
    app.router.add_get("/angle", handle_angle)
    app.router.add_get("/events", handle_events)
    app.router.add_get("/status", handle_status)
    app.router.add_get("/angle-mode", handle_angle_mode_get)
    app.router.add_post("/angle-mode", handle_angle_mode_post)
    app.router.add_get("/recalibrate", handle_recalibrate)
    app.router.add_post("/recalibrate", handle_recalibrate)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve Sharpener-Gyro web UI locally and bridge it to ESP32 over USB serial."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port path (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host (default: 127.0.0.1)")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP bind port (default: 8080)")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    parser.add_argument("--open-browser", action="store_true", help="Open the UI in a browser on startup")
    return parser.parse_args()


def configure_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


def main() -> int:
    args = parse_args()
    configure_logging(args.verbose)

    if serial is None:
        print("Missing dependency: pyserial. Install with: pip install -r requirements-linux-bridge.txt")
        return 2
    if web is None:
        print("Missing dependency: aiohttp. Install with: pip install -r requirements-linux-bridge.txt")
        return 2
    if not INDEX_HTML.exists() or not SETTINGS_HTML.exists():
        print("index.html/settings.html not found in repo root; run from Sharpener-Gyro directory.")
        return 2

    url = f"http://{args.host}:{args.http_port}/"
    log = logging.getLogger("linux_bridge")
    bridge = SerialBridge(port=args.port, baud=args.baud, logger=log.getChild("serial"))
    app = create_app(bridge)

    log.info("Starting linux bridge at %s (serial=%s baud=%d)", url, args.port, args.baud)
    if args.open_browser:
        try:
            webbrowser.open(url)
        except Exception as exc:
            log.warning("Failed to open browser: %s", exc)

    web.run_app(app, host=args.host, port=args.http_port, print=None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
