"""POSIX nonblocking 8N1 serial transport; no dependency on pyserial."""
import errno
import fcntl
import os
import select
import termios
import time
import tty


class SerialTransport:
    def __init__(self, path, baudrate=115200):
        baud = getattr(termios, 'B'+str(baudrate), None)
        if baud is None:
            raise ValueError('unsupported baud rate')
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            tty.setraw(self.fd, termios.TCSANOW)
            settings = termios.tcgetattr(self.fd)
            settings[4] = settings[5] = baud
            settings[2] &= ~termios.CRTSCTS
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        except Exception:
            os.close(self.fd)
            raise

    def read(self):
        result = bytearray()
        for _ in range(8):
            if not select.select([self.fd], [], [], 0)[0]:
                break
            try:
                chunk = os.read(self.fd, 1024)
            except BlockingIOError:
                break
            if not chunk:
                raise OSError(errno.EIO, 'serial disconnected')
            result.extend(chunk)
        return bytes(result)

    def write(self, data, timeout=.02):
        deadline = time.monotonic()+timeout
        offset = 0
        while offset < len(data):
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], max(0, remaining))[1]:
                raise TimeoutError('serial write timeout')
            try:
                count = os.write(self.fd, data[offset:])
            except BlockingIOError:
                continue
            if count <= 0:
                raise OSError(errno.EIO, 'serial write failed')
            offset += count

    def close(self):
        os.close(self.fd)
