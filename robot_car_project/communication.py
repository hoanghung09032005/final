"""
communication.py
-------------------
Giao thức:
    - Vào Auto      : "A
"
    - Vào Manual    : "M
"
    - Dừng khẩn cấp : "S
"
    - Lái tay       : "<F/B/L/R> <0-100>
"
    - Đặt tốc độ Auto: "V <0-100>
"
    - Heartbeat     : "H
"
    - Mode 3 Follow : "O
"  (Siêu âm - STM32 tự xử lý)
    - Mode 3 AI     : "O <x_err> <y_err>
" (PC gửi error từ AI)
    - Mode 4 AI Line: "I <error_x100>
"
"""

import queue
import socket
import threading
import time

import config

DIR_CODE = {
    "FORWARD": "F",
    "BACKWARD": "B",
    "LEFT": "L",
    "RIGHT": "R",
}

_RECONNECT_DELAYS = (1.5, 3.0, 5.0, 8.0, 10.0)


class CarLink:
    def __init__(self):
        self._sock = None
        self._thread = None
        self._send_lock = threading.Lock()
        self.connected = False
        self.events = queue.Queue()
        self.sent_count = 0
        self.recv_count = 0
        self._last_ip = None
        self._last_port = None
        self._manual_disconnect = True
        self._reconnect_enabled = True

    def connect(self, ip=None, port=None):
        if self.connected:
            return True
        ip = ip or config.CAR_IP
        port = port or config.CAR_PORT
        self._last_ip = ip
        self._last_port = port
        try:
            sock = socket.create_connection((ip, port), timeout=config.CONNECT_TIMEOUT)
            sock.settimeout(0.2)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._enable_keepalive(sock)
        except OSError as e:
            self._emit("error", f"Không nối được tới {ip}:{port} - {e}")
            self._emit("error", self._diagnose(e))
            self._emit("state", "disconnected")
            return False

        self._sock = sock
        self.connected = True
        self._manual_disconnect = False
        self.sent_count = 0
        self.recv_count = 0
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

        self._emit("info", f"Đã kết nối xe tại {ip}:{port}")
        self._emit("state", "connected")
        return True

    def connect_async(self, ip=None, port=None):
        threading.Thread(target=self.connect, args=(ip, port), daemon=True).start()

    @staticmethod
    def _enable_keepalive(sock):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            return
        try:
            if hasattr(socket, "SIO_KEEPALIVE_VALS"):
                sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 3000, 1000))
            else:
                if hasattr(socket, "TCP_KEEPIDLE"):
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
                if hasattr(socket, "TCP_KEEPINTVL"):
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
                if hasattr(socket, "TCP_KEEPCNT"):
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
        except OSError:
            pass

    @staticmethod
    def _diagnose(err):
        text = str(err).lower()
        if "refused" in text or getattr(err, "errno", None) == 111:
            return ("→ Máy tính TỚI ĐƯỢC ESP32 nhưng cổng 8080 đang đóng. ")
        if "timed out" in text or "timeout" in text or "unreachable" in text:
            return ("→ Không thấy ESP32 ở địa chỉ này. Kiểm tra IP và mạng WiFi.")
        return "→ Chạy chan_doan.py để biết chính xác tầng nào đang hỏng."

    def close(self, send_stop=True):
        self._manual_disconnect = True
        if not self.connected:
            return
        if send_stop:
            self.send_raw("S")
            time.sleep(0.05)
        self.connected = False
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self._sock.close()
        except OSError:
            pass
        self._sock = None
        self._emit("info", "Đã ngắt kết nối xe.")
        self._emit("state", "disconnected")

    def send_raw(self, text):
        if not self.connected or self._sock is None:
            return False
        try:
            with self._send_lock:
                self._sock.sendall((text + "
").encode("ascii", errors="ignore"))
            self.sent_count += 1
            return True
        except OSError as e:
            self._emit("error", f"Mất kết nối khi gửi lệnh: {e}")
            self._drop()
            return False

    def enter_manual(self):
        return self.send_raw("M")

    def drive(self, direction, speed):
        code = DIR_CODE.get(direction)
        if code is None:
            return False
        return self.send_raw(f"{code} {int(speed)}")

    def brake(self):
        return self.send_raw("F 0")

    def start_auto(self):
        return self.send_raw("A")

    def start_follow(self):
        """Kích hoạt Mode 3: Bám vật thể (Siêu âm - STM32 tự xử lý)."""
        return self.send_raw("O")

    def send_follow_error(self, x_err, y_err):
        """
        Gửi error từ AI Camera cho Mode 3.
        x_err: -100..100 (lệch ngang so với tâm)
        y_err: -100..100 (lệch dọc: âm = quá gần, dương = quá xa)
        """
        return self.send_raw(f"O {int(x_err)} {int(y_err)}")

    def start_ai_line(self):
        """Kích hoạt Mode 4: AI nhận diện line."""
        return self.send_raw("I 0")

    def send_ai_error(self, error_x100):
        """
        Gửi error (đã x100) cho Mode 4.
        error_x100: -400..400 tương ứng error -4.00..4.00
        """
        return self.send_raw(f"I {int(error_x100)}")

    def heartbeat(self):
        return self.send_raw("H")

    def set_auto_speed(self, speed):
        speed = max(0, min(100, int(speed)))
        return self.send_raw(f"V {speed}")

    def stop_all(self):
        return self.send_raw("S")

    def _rx_loop(self):
        buf = ""
        while self.connected:
            try:
                data = self._sock.recv(1024)
            except socket.timeout:
                continue
            except OSError as e:
                if self.connected:
                    self._emit("error", f"Lỗi đọc dữ liệu: {e}")
                    self._drop()
                return

            if not data:
                if self.connected:
                    self._emit("error", "Xe đã đóng kết nối.")
                    self._drop()
                return

            buf += data.decode("utf-8", errors="replace")
            while "
" in buf:
                line, buf = buf.split("
", 1)
                self._handle_line(line.strip())

            if len(buf) > 4096:
                self._emit("error", "Bộ đệm nhận dữ liệu tràn.")
                buf = ""

    def _handle_line(self, line):
        if not line:
            return
        if not line.startswith("[ESP32-S3]"):
            self.recv_count += 1

        if line.startswith("LOG,"):
            parts = line.split(",")
            if len(parts) < 7:
                return
            try:
                telemetry = {
                    "sensor": int(parts[1]),
                    "side_l": int(parts[2]),
                    "side_r": int(parts[3]),
                    "error": int(parts[4]) / 100.0,
                    "pwm_l": int(parts[5]),
                    "pwm_r": int(parts[6]),
                    "t": time.time(),
                }
                if len(parts) >= 8:
                    distance_x10 = int(parts[7])
                    telemetry["dist"] = (
                        distance_x10 / 10.0 if distance_x10 >= 0 else None
                    )
                if len(parts) >= 10:
                    temp_x10 = int(parts[8])
                    humidity_x10 = int(parts[9])
                    telemetry["temp_c"] = (
                        temp_x10 / 10.0 if temp_x10 > -32768 else None
                    )
                    telemetry["humidity_rh"] = (
                        humidity_x10 / 10.0 if humidity_x10 > -32768 else None
                    )
                self._emit("telemetry", telemetry)
            except ValueError:
                pass
        else:
            self._emit("info", line)

    def _drop(self):
        self.connected = False
        try:
            if self._sock:
                self._sock.close()
        except OSError:
            pass
        self._sock = None
        self._emit("state", "disconnected")
        self._maybe_auto_reconnect()

    def _maybe_auto_reconnect(self):
        if self._manual_disconnect or not self._reconnect_enabled:
            return
        ip, port = self._last_ip, self._last_port
        if not ip:
            return

        def _retry_loop():
            for delay in _RECONNECT_DELAYS:
                time.sleep(delay)
                if self._manual_disconnect or self.connected:
                    return
                self._emit("info", f"[TỰ ĐỘNG] Đang thử nối lại xe tại {ip}:{port}...")
                if self.connect(ip, port):
                    return
            self._emit(
                "error",
                "Tự động nối lại thất bại nhiều lần - kiểm tra nguồn/WiFi xe "
                "rồi bấm 'Kết nối xe' lại.",
            )

        threading.Thread(target=_retry_loop, daemon=True).start()

    def _emit(self, kind, payload):
        self.events.put((kind, payload))

    def poll_events(self):
        items = []
        while True:
            try:
                items.append(self.events.get_nowait())
            except queue.Empty:
                break
        return items


link = CarLink()


def send_drive_command(direction, speed, log_func=None):
    ok = link.drive(direction, speed)
    if log_func and not ok:
        log_func(f"[{time.strftime('%H:%M:%S')}] Chưa kết nối xe - lệnh {direction} bị bỏ.")
    return ok


def start_auto_patrol(log_func=None):
    ok = link.start_auto()
    if log_func:
        log_func("[LỆNH] Bắt đầu chế độ tự động" if ok else "[LỖI] Chưa kết nối xe.")
    return ok


def stop_auto_patrol(log_func=None):
    ok = link.stop_all()
    if log_func:
        log_func("[LỆNH] Dừng xe" if ok else "[LỖI] Chưa kết nối xe.")
    return ok
