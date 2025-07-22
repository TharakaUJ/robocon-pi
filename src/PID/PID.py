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
        self.kpx = 0.1
        self.kpy = 0.1
        self.kpt = 0.1
        self.kdx = 0.0 
        self.kdy = 0.0
        self.kdt = 0.0
        self.kix = 0.0
        self.kiy = 0.0
        self.kit = 0.0
        self.callback = None
        self._load_last_params()

    def _log_params(self):
        with open(self._log_file, 'a') as f:
            ts = time.strftime('%Y-%m-%d %H:%M:%S')
            f.write(f"{ts},{self.kpx},{self.kpy},{self.kpt},{self.kdx},{self.kdy},{self.kdt},{self.kix},{self.kiy},{self.kit}\n")

    def _load_last_params(self):
        if not os.path.exists(self._log_file):
            return
        try:
            with open(self._log_file, 'r') as f:
                lines = f.readlines()
                if lines:
                    last = lines[-1].strip().split(',')
                    if len(last) == 10:
                        _, kpx, kpy, kpt, kdx, kdy, kdt, kix, kiy, kit = last
                        self.kpx, self.kpy, self.kpt = float(kpx), float(kpy), float(kpt)
                        self.kdx, self.kdy, self.kdt = float(kdx), float(kdy), float(kdt)
                        self.kix, self.kiy, self.kit = float(kix), float(kiy), float(kit)
                    else:
                        self.kpx, self.kpy, self.kpt = 0.1, 0.1, 0.1
                        self.kdx, self.kdy, self.kdt = 0.0, 0.0, 0.0
                        self.kix, self.kiy, self.kit = 0.0, 0.0, 0.0
        except Exception:
            pass

    def set_kpx(self, value):
        self.kpx = value
        self._log_params()
        if self.callback:
            self.callback('kpx', value)

    def set_kpy(self, value):
        self.kpy = value
        self._log_params()
        if self.callback:
            self.callback('kpy', value)

    def set_kpt(self, value):
        self.kpt = value
        self._log_params()
        if self.callback:
            self.callback('kpt', value)

    def set_kdx(self, value):
        self.kdx = value
        self._log_params()
        if self.callback:
            self.callback('kdx', value)

    def set_kdy(self, value):
        self.kdy = value
        self._log_params()
        if self.callback:
            self.callback('kdy', value)

    def set_kdt(self, value):
        self.kdt = value
        self._log_params()
        if self.callback:
            self.callback('kdt', value)

    def set_kix(self, value):
        self.kix = value
        self._log_params()
        if self.callback:
            self.callback('kix', value)

    def set_kiy(self, value):
        self.kiy = value
        self._log_params()
        if self.callback:
            self.callback('kiy', value)

    def set_kit(self, value):
        self.kit = value
        self._log_params()
        if self.callback:
            self.callback('kit', value)

      

    def set_callback(self, callback):
        self.callback = callback

    def get_params(self):
        return {'kpx': self.kpx, 'kpy': self.kpy, 'kpt': self.kpt, 'kdx': self.kdx, 'kdy': self.kdy, 'kdt': self.kdt, 'kix': self.kix, 'kiy': self.kiy, 'kit': self.kit}