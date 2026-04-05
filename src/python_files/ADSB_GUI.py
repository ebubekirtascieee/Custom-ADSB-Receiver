import sys, serial, threading, csv, datetime
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import pyModeS as pms

# ==========================================
# GLOBAL VARIABLES
# ==========================================
shared_threshold = np.zeros(4096, dtype=np.uint16)
shared_envelope = np.zeros(4096, dtype=np.uint16)
shared_digital_bits = np.zeros(4096, dtype=np.uint16)
shared_margin = 0
shared_noise_floor = 0
data_lock = threading.Lock()

# ==========================================
# 1. THE DATA MINER (Background USB Thread)
# ==========================================
def serial_thread():
    global shared_threshold, shared_envelope, shared_digital_bits, shared_margin, shared_noise_floor
    PORT = 'COM7'  # UPDATE TO YOUR COM PORT
    BAUD = 921600

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Connected to FPGA Oscilloscope on {PORT}!")
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    PACKET_SIZE = 16404

    while True:
        try:
            if ser.read(1) == b'\xAA' and ser.read(1) == b'\x55':
                payload = ser.read(PACKET_SIZE - 2)

                if len(payload) == (PACKET_SIZE - 2):
                    # Hardware button margin and noise floor from the header
                    margin = (payload[0] << 8) | payload[1]
                    noise = (payload[2] << 8) | payload[3]

                    # >>> RESTORED ADS-B DECODER LOGIC <<<
                    adsb_bytes = payload[4:18]
                    hex_msg = adsb_bytes.hex().upper()

                    if hex_msg != "0000000000000000000000000000":
                        df = pms.df(hex_msg)
                        crc = pms.crc(hex_msg)

                        # Handle pure ADS-B (GPS/Speed/Callsign)
                        if df == 17 or df == 18:
                            if crc == 0:
                                icao = pms.icao(hex_msg)
                                print(f"[+] DF-17 ADS-B CAUGHT! (Noise: {noise} | Margin: {margin})")
                                print(f"    ICAO ID : {icao}")
                            else:
                                print(f"[DEBUG] DF-17 Corrupted | CRC: FAIL (0x{crc:06X})")

                        # Handle Radar Interrogation Replies
                        elif df == 20 or df == 21:
                            icao = pms.icao(hex_msg)
                            if icao:
                                print(f"[*] RADAR PING | ICAO: {icao} | Type: DF-{df}")

                        else:
                            print(f"[-] OTHER FORMAT | DF: {df} | HEX: {hex_msg[:8]}...")
                    # >>> END DECODER LOGIC <<<

                    raw_words = np.frombuffer(payload[18:], dtype=np.dtype('>u4'))

                    digital_bits = (raw_words >> 24) & 0x0001
                    threshold_trace = (raw_words >> 12) & 0x0FFF
                    envelope_trace = raw_words & 0x0FFF

                    with data_lock:
                        shared_margin = margin
                        shared_noise_floor = noise
                        np.copyto(shared_threshold, threshold_trace)
                        np.copyto(shared_envelope, envelope_trace)
                        np.copyto(shared_digital_bits, digital_bits)

        except Exception as e:
            print(f"Serial Error: {e}")
            break

# ==========================================
# 2. THE PAINTER (2x2 Grid GUI)
# ==========================================
class SDRWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ADS-B Calibration & Trigger Scope")
        self.resize(1600, 900)

        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        layout = QtWidgets.QVBoxLayout(main_widget)

        top_bar = QtWidgets.QHBoxLayout()
        self.info_label = QtWidgets.QLabel("Waiting for FPGA Data...")
        self.info_label.setStyleSheet(
            "font-family: 'Consolas', monospace; font-size: 20px; font-weight: bold; color: #00FF00; background-color: #111; padding: 10px; border-radius: 5px;")
        top_bar.addWidget(self.info_label, stretch=1)

        self.freeze_btn = QtWidgets.QPushButton("⏸ FREEZE & SAVE CSV")
        self.freeze_btn.setCheckable(True)
        self.freeze_btn.setStyleSheet(
            "font-size: 18px; font-weight: bold; background-color: #444; color: white; padding: 10px;")
        self.freeze_btn.clicked.connect(self.toggle_freeze)
        top_bar.addWidget(self.freeze_btn)
        layout.addLayout(top_bar)

        self.is_frozen = False
        self.trigger_count = 0

        self.graph_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graph_layout)
        self.time_x_axis = np.linspace(0, (4096 / 20.25), 4096)

        # ROW 0, COL 0: Live Free-Running (For Button Calibration)
        self.live_plot = self.graph_layout.addPlot(row=0, col=0, title="Live Free-Running (Button Calibration View)")
        self.live_plot.showGrid(x=True, y=True, alpha=0.4)
        self.live_plot.setYRange(-100, 4200, padding=0)
        self.live_env_curve = self.live_plot.plot(pen=pg.mkPen('y', width=1))
        self.live_thresh_curve = self.live_plot.plot(pen=pg.mkPen('r', width=1))

        # ROW 0, COL 1: Triggered Macro
        self.macro_plot = self.graph_layout.addPlot(row=0, col=1, title="Triggered: Macro View")
        self.macro_plot.showGrid(x=True, y=True, alpha=0.4)
        self.macro_plot.setYRange(-100, 4200, padding=0)
        self.macro_env_curve = self.macro_plot.plot(pen=pg.mkPen('y', width=1))
        self.macro_thresh_curve = self.macro_plot.plot(pen=pg.mkPen('r', width=1))

        # ROW 1, COL 0: Triggered Zoom
        self.zoom_env_plot = self.graph_layout.addPlot(row=1, col=0, title="Triggered Zoom: Envelope vs Threshold")
        self.zoom_env_plot.showGrid(x=True, y=True, alpha=0.4)
        self.zoom_env_plot.setYRange(-100, 4200, padding=0)
        self.zoom_env_curve = self.zoom_env_plot.plot(pen=pg.mkPen('y', width=2))
        self.zoom_thresh_curve = self.zoom_env_plot.plot(pen=pg.mkPen('r', width=2))

        # ROW 1, COL 1: Triggered Digital Bits
        self.zoom_bit_plot = self.graph_layout.addPlot(row=1, col=1, title="Triggered: FPGA Hardware Bits")
        self.zoom_bit_plot.showGrid(x=True, y=True, alpha=0.4)
        self.zoom_bit_plot.setYRange(-0.1, 1.2, padding=0)
        self.zoom_bit_curve = self.zoom_bit_plot.plot(pen=pg.mkPen('c', width=2))

        # Link the triggered X-axes together
        self.zoom_env_plot.setXLink(self.zoom_bit_plot)
        self.macro_plot.setXLink(self.zoom_bit_plot)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(33)

    def toggle_freeze(self):
        self.is_frozen = self.freeze_btn.isChecked()
        if self.is_frozen:
            self.freeze_btn.setText("▶ RESUME")
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"adsb_capture_{timestamp}.csv"
            with data_lock:
                save_thr = np.copy(shared_threshold)
                save_env = np.copy(shared_envelope)
                save_bits = np.copy(shared_digital_bits)
            try:
                with open(filename, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["Time_us", "Dynamic_Threshold", "Envelope", "Digital_Bit"])
                    for i in range(4096):
                        writer.writerow([f"{i / 20.25:.3f}", save_thr[i], save_env[i], save_bits[i]])
            except Exception as e:
                pass
        else:
            self.freeze_btn.setText("⏸ FREEZE & SAVE CSV")

    def update_gui(self):
        with data_lock:
            local_thr = np.copy(shared_threshold)
            local_env = np.copy(shared_envelope)
            local_bits = np.copy(shared_digital_bits)
            local_margin = shared_margin
            local_noise = shared_noise_floor

        # Live Plot ALWAYS updates, even if frozen, so calibrate buttons can be used
        if not self.is_frozen:
            self.live_env_curve.setData(self.time_x_axis, local_env)
            self.live_thresh_curve.setData(self.time_x_axis, local_thr)

            high_samples = np.sum(local_bits)

            # Triggered plots ONLY update when active
            if high_samples > 30:
                self.trigger_count += 1
                self.macro_env_curve.setData(self.time_x_axis, local_env)
                self.macro_thresh_curve.setData(self.time_x_axis, local_thr)

                self.zoom_env_curve.setData(self.time_x_axis, local_env)
                self.zoom_thresh_curve.setData(self.time_x_axis, local_thr)
                self.zoom_bit_curve.setData(self.time_x_axis, local_bits)

                self.macro_plot.setTitle(f"Triggered: Macro View (Frame #{self.trigger_count})")

        # Status text updates regardless
        status_text = f"Offset Margin: {local_margin} | Base Noise: {local_noise} | High Samples: {np.sum(local_bits)}"
        if self.is_frozen:
            status_text += "  [ STATUS: FROZEN & SAVED ]"

        self.info_label.setText(status_text)


if __name__ == '__main__':
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    app = QtWidgets.QApplication(sys.argv)
    window = SDRWindow()
    window.show()
    sys.exit(app.exec())