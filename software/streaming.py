import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, List, Optional

import serial  # pip install pyserial


class StreamState(Enum):
    IDLE = auto()
    SENDING = auto()
    PAUSED = auto()
    ERROR = auto()
    DONE = auto()


@dataclass
class StreamError:
    line_index: int
    line_text: str
    error_code: str
    raw_line: str


def default_logger(msg: str) -> None:
    print(msg)


class GrblStreamer:
    """
    Simple grblHAL / Grbl streaming class:
    - Sends G-code lines one-by-one.
    - Waits for 'ok' before sending the next line.
    - Stops on 'error:<code>' and reports which line failed.
    - Tolerates extra info lines from the controller.

    Usage:
        streamer = GrblStreamer(
            port="COM11",
            baudrate=115200,
            lines=gcode_lines,
            log_callback=your_log_func,
            state_callback=your_state_func,
            error_callback=your_error_func,
            progress_callback=your_progress_func,
        )
        streamer.start()
    """

    def __init__(
        self,
        port: str,
        baudrate: int,
        lines: List[str],
        log_callback: Callable[[str], None] = default_logger,
        state_callback: Optional[Callable[[StreamState], None]] = None,
        error_callback: Optional[Callable[[StreamError], None]] = None,
        progress_callback: Optional[Callable[[int, int], None]] = None,
        timeout: float = 1.0,
        startup_drain_time: float = 1.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.raw_lines = lines  # original lines
        self.lines = self._preprocess_lines(lines)
        self.log = log_callback
        self.state_callback = state_callback
        self.error_callback = error_callback
        self.progress_callback = progress_callback

        self.timeout = timeout
        self.startup_drain_time = startup_drain_time

        self._ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self.state: StreamState = StreamState.IDLE
        self._line_index: int = 0  # index into self.lines
        self._lock = threading.Lock()

    # -----------------------------
    # Public API
    # -----------------------------
    def start(self) -> None:
        """Open the port and start streaming in background."""
        if self.state not in (StreamState.IDLE, StreamState.DONE, StreamState.ERROR):
            self.log("Cannot start: streamer is not idle.")
            return

        try:
            self._open_port()
        except Exception as e:
            self.log(f"Failed to open port {self.port}: {e}")
            self._set_state(StreamState.ERROR)
            if self.error_callback:
                err = StreamError(
                    line_index=-1,
                    line_text="",
                    error_code=f"PORT:{e}",
                    raw_line="",
                )
                self.error_callback(err)
            return

        self._stop_event.clear()
        self._line_index = 0
        self._set_state(StreamState.SENDING)

        self._rx_thread = threading.Thread(target=self._io_loop, daemon=True)
        self._rx_thread.start()

    def pause(self) -> None:
        """Send real-time pause ('!') and transition to PAUSED."""
        with self._lock:
            if self.state != StreamState.SENDING:
                return
            self._send_realtime(b"!")
            self._set_state(StreamState.PAUSED)

    def resume(self) -> None:
        """Send real-time resume ('~') and continue SENDING."""
        with self._lock:
            if self.state != StreamState.PAUSED:
                return
            self._send_realtime(b"~")
            self._set_state(StreamState.SENDING)
            # Resume by sending the current line if not yet acked
            if self._line_index < len(self.lines):
                self._send_current_line()

    def abort(self) -> None:
        """Send Ctrl-X, stop streaming, close port."""
        with self._lock:
            self._send_realtime(b"\x18")  # Ctrl-X
            self._stop_event.set()
            self._set_state(StreamState.IDLE)
        self._close_port()

    def join(self, timeout: Optional[float] = None) -> None:
        """Wait for background thread to exit."""
        t = self._rx_thread
        if t is not None:
            t.join(timeout=timeout)

    # -----------------------------
    # Internal helpers
    # -----------------------------
    def _set_state(self, s: StreamState) -> None:
        self.state = s
        if self.state_callback:
            self.state_callback(s)

    def _open_port(self) -> None:
        self.log(f"Opening serial port {self.port} @ {self.baudrate}...")
        self._ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
        )
        time.sleep(0.1)  # brief settle

    def _close_port(self) -> None:
        if self._ser and self._ser.is_open:
            self.log("Closing serial port.")
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def _send_line(self, line: str) -> None:
        if not self._ser or not self._ser.is_open:
            self.log("Port not open, cannot send line.")
            return
        data = (line + "\n").encode("ascii", errors="replace")
        self.log(f"SEND[{self._line_index + 1}/{len(self.lines)}]: {line}")
        self._ser.write(data)

    def _send_realtime(self, data: bytes) -> None:
        if not self._ser or not self._ser.is_open:
            return
        self._ser.write(data)

    def _preprocess_lines(self, lines: List[str]) -> List[str]:
        """
        - Strip whitespace
        - Drop empty lines
        - Remove '; comment' and '(comment)' blocks in a simple way
        """
        cleaned: List[str] = []
        for ln in lines:
            ln = ln.strip()
            if not ln:
                continue

            # strip ';' comments
            if ";" in ln:
                ln = ln.split(";", 1)[0].rstrip()

            # strip simple '(...)' comments fully if the line is just a comment
            if ln.startswith("(") and ln.endswith(")"):
                continue

            # very basic '(comment)' removal in-line
            # this is not a full parser but good enough for many cases
            while True:
                start = ln.find("(")
                end = ln.find(")", start + 1)
                if start != -1 and end != -1 and end > start:
                    ln = (ln[:start] + " " + ln[end + 1 :]).strip()
                else:
                    break

            if ln:
                cleaned.append(ln)
        return cleaned

    # -----------------------------
    # I/O Loop and parsing
    # -----------------------------
    def _io_loop(self) -> None:
        """Background thread: reads, parses, and drives the state machine."""
        self.log("I/O loop started.")
        try:
            # 1) drain startup text
            t_end = time.time() + self.startup_drain_time
            while time.time() < t_end and not self._stop_event.is_set():
                line = self._read_line()
                if line is None:
                    continue
                self.log(f"STARTUP: {line}")

            # 2) start sending the first line, if any
            with self._lock:
                if self.state == StreamState.SENDING and self._line_index < len(self.lines):
                    self._send_current_line()
                elif not self.lines:
                    self.log("No G-code lines to send.")
                    self._set_state(StreamState.DONE)
                    return

            # 3) main loop
            while not self._stop_event.is_set():
                line = self._read_line()
                if line is None:
                    continue
                self._handle_incoming_line(line)

        finally:
            self._close_port()
            self.log("I/O loop terminated.")

    def _read_line(self) -> Optional[str]:
        """Read one line (ASCII) from serial; return None on timeout."""
        if not self._ser or not self._ser.is_open:
            return None
        try:
            raw = self._ser.readline()  # reads until '\n' or timeout
        except Exception as e:
            self.log(f"Serial read error: {e}")
            self._stop_event.set()
            self._set_state(StreamState.ERROR)
            return None
        if not raw:
            return None
        try:
            s = raw.decode("ascii", errors="replace").strip("\r\n")
        except Exception:
            s = ""
        if s:
            self.log(f"RECV: {s}")
        return s

    def _handle_incoming_line(self, line: str) -> None:
        """
        Parse incoming line:
        - 'ok' → move to next line (if SENDING)
        - 'error:XXX' → stop and report
        - everything else → log as info
        """
        line_lc = line.lower()

        if line_lc == "ok":
            self._on_ok()
        elif line_lc.startswith("error:"):
            code = line[len("error:") :]
            self._on_error(code.strip(), raw_line=line)
        else:
            # Other info lines; just log
            # Could parse status reports ('<...>') here if desired
            pass

    def _on_ok(self) -> None:
        with self._lock:
            if self.state != StreamState.SENDING:
                # ignore 'ok' if not currently sending
                return

            self._line_index += 1
            if self.progress_callback:
                self.progress_callback(self._line_index, len(self.lines))

            if self._line_index >= len(self.lines):
                self.log("All lines acknowledged. Job DONE.")
                self._set_state(StreamState.DONE)
                self._stop_event.set()
                return

            # send next line
            self._send_current_line()

    def _on_error(self, error_code: str, raw_line: str) -> None:
        with self._lock:
            self.log(f"Controller reported error: {error_code}")
            self._set_state(StreamState.ERROR)
            self._stop_event.set()

            line_index = self._line_index
            line_text = ""
            if 0 <= line_index < len(self.lines):
                line_text = self.lines[line_index]

            err = StreamError(
                line_index=line_index,
                line_text=line_text,
                error_code=error_code,
                raw_line=raw_line,
            )
            if self.error_callback:
                self.error_callback(err)

    def _send_current_line(self) -> None:
        if 0 <= self._line_index < len(self.lines):
            self._send_line(self.lines[self._line_index])
        else:
            self.log("send_current_line: index out of range.")