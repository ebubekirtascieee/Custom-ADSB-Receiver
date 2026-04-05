import sys, serial, os, time
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWebEngineWidgets import QWebEngineView
import pyModeS as pms

os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu"

#Change to your current coordinates, needed for the plane location finding.
REF_LAT = 41.036
REF_LON = 29.361

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        body { margin:0; padding:0; background-color: #000; }
        #map { width: 100vw; height: 100vh; }
        .custom-popup .leaflet-popup-content-wrapper { 
            background: #111; color: #0f0; border: 1px solid #0f0; font-family: monospace;
        }
        .custom-popup .leaflet-popup-tip { background: #111; }
    </style>
</head>
<body>
    <div id="map"></div>
    <script>
        var map = L.map('map').setView([41.036, 29.361], 10);
        L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
            attribution: '&copy; OpenStreetMap &copy; CARTO'
        }).addTo(map);

        var markers = {};

        function updateMarker(icao, lat, lon, alt, callsign, speed, heading) {
            var planeHtml = `<div style="font-size: 24px; color: #00ff00; text-shadow: 0 0 8px #00ff00; transform: rotate(${heading}deg); transform-origin: center;">✈️</div>`;
            var icon = L.divIcon({className: 'dummy', html: planeHtml, iconSize: [24, 24], iconAnchor: [12, 12]});

            if (!markers[icao]) {
                markers[icao] = L.marker([lat, lon], {icon: icon}).addTo(map);
            } else {
                markers[icao].setLatLng([lat, lon]);
                markers[icao].setIcon(icon);
            }

            var label = `<b>ICAO: ${icao}</b><br>Call: ${callsign}<br>Alt: ${alt} ft<br>Spd: ${speed} kts<br>Hdg: ${heading}°`;
            markers[icao].bindPopup(label, {className: 'custom-popup'});
        }

        // --- NEW: Function to remove planes that fly out of range ---
        function removeMarker(icao) {
            if (markers[icao]) {
                map.removeLayer(markers[icao]);
                delete markers[icao];
            }
        }
    </script>
</body>
</html>
"""


class SerialDecoderThread(QtCore.QThread):
    log_signal = QtCore.pyqtSignal(str)
    plane_signal = QtCore.pyqtSignal(str, float, float, str, str, str, str)

    def __init__(self, port='COM7', baud=921600):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.local_cache = {}

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            self.log_signal.emit(f"[SYSTEM] Connected to FAST RADAR on {self.port}")
            self.log_signal.emit(f"[SYSTEM] Ensure FPGA S1 is toggled to Fast Mode (20-Byte)!")
        except Exception as e:
            self.log_signal.emit(f"[ERROR] Failed to open port: {e}")
            return

        PACKET_SIZE = 20

        while self.running:
            try:
                if ser.read(1) == b'\xAA' and ser.read(1) == b'\x55':
                    payload = ser.read(PACKET_SIZE - 2)

                    if len(payload) == (PACKET_SIZE - 2):
                        margin = (payload[0] << 8) | payload[1]
                        noise = (payload[2] << 8) | payload[3]

                        adsb_bytes = payload[4:18]
                        hex_msg = adsb_bytes.hex().upper()

                        if hex_msg != "0000000000000000000000000000":
                            df = pms.df(hex_msg)
                            crc = pms.crc(hex_msg)

                            # Error Correction for map data
                            if crc != 0:
                                fixed_hex = pms.adsb.correct_errors(hex_msg)
                                if fixed_hex:
                                    hex_msg = fixed_hex
                                    crc = 0

                            if df == 17 or df == 18:
                                if crc == 0:
                                    icao = pms.icao(hex_msg)
                                    tc = pms.adsb.typecode(hex_msg)

                                    self.log_signal.emit(
                                        f"[+] DF-17 GPS PKT | ICAO: {icao} | TC: {tc} | Margin: {margin} | Noise: {noise}")

                                    if icao not in self.local_cache:
                                        self.local_cache[icao] = {'callsign': 'UNKNOWN', 'alt': '---', 'speed': '---',
                                                                  'heading': '0'}

                                    if 1 <= tc <= 4:
                                        try:
                                            self.local_cache[icao]['callsign'] = pms.adsb.callsign(hex_msg).strip('_')
                                        except:
                                            pass

                                    if tc == 19:
                                        try:
                                            v = pms.adsb.velocity(hex_msg)
                                            if v:
                                                self.local_cache[icao]['speed'] = str(int(v[0]))
                                                self.local_cache[icao]['heading'] = str(int(v[1]))
                                        except:
                                            pass

                                    if 9 <= tc <= 18:
                                        try:
                                            self.local_cache[icao]['alt'] = str(pms.adsb.altitude(hex_msg))
                                            pos = pms.adsb.position_with_ref(hex_msg, REF_LAT, REF_LON)
                                            if pos:
                                                lat, lon = pos[0], pos[1]
                                                c = self.local_cache[icao]
                                                self.plane_signal.emit(icao, lat, lon, c['alt'], c['callsign'],
                                                                       c['speed'], c['heading'])
                                        except:
                                            pass
                                else:
                                    self.log_signal.emit(
                                        f"[DEBUG] DF-17 Corrupted | CRC: FAIL ({crc:06X}) | Margin: {margin}")

                            elif df == 20 or df == 21:
                                icao = pms.icao(hex_msg)
                                if icao:
                                    self.log_signal.emit(
                                        f"[*] RADAR PING | ICAO: {icao} | Type: DF-{df} | Margin: {margin}")
                                    if df == 20:
                                        try:
                                            alt = pms.common.altcode(hex_msg)
                                            self.log_signal.emit(f"    └─ Altitude: {alt} ft")
                                        except:
                                            pass

            except Exception as e:
                pass

    def stop(self):
        self.running = False
        self.wait()


class RadarDashboard(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ADS-B Live Radar Dashboard")
        self.resize(1200, 800)

        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        layout = QtWidgets.QHBoxLayout(main_widget)
        layout.setContentsMargins(0, 0, 0, 0)

        self.terminal = QtWidgets.QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setFixedWidth(450)
        self.terminal.setStyleSheet(
            "background-color: #111; color: #0f0; font-family: Consolas, monospace; font-size: 14px; padding: 10px; border-right: 2px solid #333;")
        layout.addWidget(self.terminal)

        self.map_view = QWebEngineView()
        self.map_view.setHtml(HTML_TEMPLATE)
        layout.addWidget(self.map_view)

        # --- TIMEOUT TRACKING DATA ---
        self.active_planes = {}  # Dictionary to track {icao: last_seen_timestamp}

        self.cleanup_timer = QtCore.QTimer()
        self.cleanup_timer.timeout.connect(self.check_timeouts)
        self.cleanup_timer.start(5000)  # Run the check every 5 seconds

        self.worker = SerialDecoderThread(port='COM7')
        self.worker.log_signal.connect(self.append_log)
        self.worker.plane_signal.connect(self.update_map_marker)
        self.worker.start()

    @QtCore.pyqtSlot(str)
    def append_log(self, text):
        self.terminal.append(text)
        scrollbar = self.terminal.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    @QtCore.pyqtSlot(str, float, float, str, str, str, str)
    def update_map_marker(self, icao, lat, lon, alt, callsign, speed, heading):
        # Update the timestamp whenever we get a valid map position
        self.active_planes[icao] = time.time()

        js_code = f"updateMarker('{icao}', {lat}, {lon}, '{alt}', '{callsign}', '{speed}', '{heading}');"
        self.map_view.page().runJavaScript(js_code)

    def check_timeouts(self):
        current_time = time.time()
        # Find any planes that haven't been updated in 60 seconds
        stale_icaos = [icao for icao, last_seen in self.active_planes.items() if current_time - last_seen > 60]

        for icao in stale_icaos:
            # 1. Remove from map via Javascript
            self.map_view.page().runJavaScript(f"removeMarker('{icao}');")
            # 2. Log it to the terminal
            self.append_log(f"[!] TIMEOUT | ICAO: {icao} lost contact. Removed from map.")
            # 3. Remove from our active tracking
            del self.active_planes[icao]
            # 4. Safely clear the worker's cache so it resets altitude/callsign if it returns later
            self.worker.local_cache.pop(icao, None)

    def closeEvent(self, event):
        self.worker.stop()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = RadarDashboard()
    window.show()
    sys.exit(app.exec_())