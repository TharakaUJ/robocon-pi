import threading
import time
import os

class PIDController:
    _instance = None
    _lock = threading.Lock()
    _log_file = './pid_log.txt'

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(PIDController, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, '_initialized') and self._initialized:
            return
        
        self._initialized = True
        self.kp = 0.1
        self.ki = 0.0
        self.kd = 0.0
        self.callback = None        
        self._load_last_params()

    def _log_params(self):
        with open(self._log_file, 'a') as f:
            ts = time.strftime('%Y-%m-%d %H:%M:%S')
            f.write(f"{ts},{self.kp},{self.ki},{self.kd}\n")

    def _load_last_params(self):
        if not os.path.exists(self._log_file):
            return
        try:
            with open(self._log_file, 'r') as f:
                lines = f.readlines()
                if lines:
                    last = lines[-1].strip().split(',')
                    if len(last) == 4:
                        _, kp, ki, kd = last
                        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
                    else:
                        self.kp, self.ki, self.kd = 0.1, 0.0, 0.0
        except Exception:
            pass

    def set_kp(self, value):
        self.kp = value
        self._log_params()
        if self.callback:
            self.callback('kp', value)

    def set_ki(self, value):
        self.ki = value
        self._log_params()
        if self.callback:
            self.callback('ki', value)

    def set_kd(self, value):
        self.kd = value
        self._log_params()
        if self.callback:
            self.callback('kd', value)

    def set_callback(self, callback):
        self.callback = callback

    def get_params(self):
        return {'kp': self.kp, 'ki': self.ki, 'kd': self.kd}