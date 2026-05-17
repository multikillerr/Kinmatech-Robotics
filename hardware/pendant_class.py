from PyQt6.QtCore import QThread, pyqtSignal
import time
import glob
import serial


class PendantConnectionThread(QThread):
    """Auto-detect and maintain the UART link to the physical pendant."""

    connection_status = pyqtSignal(bool, str)
    command_received = pyqtSignal(str)
    log_message = pyqtSignal(str)

    _MIN_RETRY = 1.0
    _MAX_RETRY = 10.0

    def __init__(self, baud_rate=115200):
        super().__init__()
        self.baud_rate = baud_rate
        self.running = True
        self.serial_port = None
        self.port = None
        self._retry_delay = self._MIN_RETRY

    def run(self):
        """Main thread loop."""
        while self.running:
            if self.serial_port is None:
                self.connect_pendant()
            else:
                self._read_loop()

    def _read_loop(self):
        while self.running and self.serial_port is not None:
            try:
                if self.serial_port.in_waiting:
                    raw = self.serial_port.readline()
                    command = raw.decode(errors='replace').strip()
                    if command:
                        self.command_received.emit(command)
                else:
                    time.sleep(0.005)
            except (serial.SerialException, OSError) as exc:
                self.log_message.emit(f"Pendant read error: {exc}")
                self.disconnect_pendant()
                return
            except Exception as exc:
                self.log_message.emit(f"Pendant unexpected error: {exc}")
                self.disconnect_pendant()
                return

    def connect_pendant(self):
        """Attempt to connect to the pendant."""
        try:
            ports = (
                glob.glob('/dev/tty.usb*')
                + glob.glob('/dev/ttyUSB*')
                + glob.glob('/dev/ttyACM*')
                + glob.glob('COM*')
            )

            for port in ports:
                if not self.running:
                    return
                try:
                    serial_port = serial.Serial(port, self.baud_rate, timeout=1)
                    time.sleep(2)
                    serial_port.reset_input_buffer()
                    serial_port.write(b'test_pendant\n')
                    time.sleep(0.3)

                    if serial_port.in_waiting:
                        response = serial_port.readline().decode(errors='replace').strip()
                        if response == "pendant_ready":
                            self.serial_port = serial_port
                            self.port = port
                            self._retry_delay = self._MIN_RETRY
                            self.connection_status.emit(True, port)
                            self.log_message.emit(f"Pendant connected on {port}")
                            return

                    serial_port.close()
                except Exception:
                    continue

            self.connection_status.emit(False, "")
            for _ in range(int(self._retry_delay * 10)):
                if not self.running:
                    return
                time.sleep(0.1)
            self._retry_delay = min(self._retry_delay * 1.5, self._MAX_RETRY)

        except Exception as e:
            self.log_message.emit(f"Connection error: {str(e)}")
            self.connection_status.emit(False, "")
            time.sleep(1)

    def disconnect_pendant(self):
        """Disconnect from the pendant."""
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            self.port = None
            self.connection_status.emit(False, "")

    def send(self, data: bytes) -> bool:
        """Send raw bytes to the pendant."""
        if self.serial_port is None:
            return False
        try:
            self.serial_port.write(data)
            return True
        except Exception as exc:
            self.log_message.emit(f"Pendant send error: {exc}")
            self.disconnect_pendant()
            return False

    def is_connected(self) -> bool:
        return self.serial_port is not None

    def stop(self):
        """Stop the thread."""
        self.running = False
        self.disconnect_pendant()