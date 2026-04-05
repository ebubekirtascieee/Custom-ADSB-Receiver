import sys, csv
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

class CSVPlaybackWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Offline Threshold Analyzer")
        self.resize(1600, 900)

        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        layout = QtWidgets.QVBoxLayout(main_widget)

        top_bar = QtWidgets.QHBoxLayout()
        self.load_btn = QtWidgets.QPushButton("📂 LOAD CSV")
        self.load_btn.clicked.connect(self.load_csv)
        top_bar.addWidget(self.load_btn)
        layout.addLayout(top_bar)

        self.graph_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graph_layout)

        # Macro
        self.macro_plot = self.graph_layout.addPlot(row=0, col=0, colspan=2, title="Macro View")
        self.macro_env = self.macro_plot.plot(pen=pg.mkPen('y', width=1))
        self.macro_thr = self.macro_plot.plot(pen=pg.mkPen('r', width=1))

        # Zoom Env + Thr
        self.zoom_env_plot = self.graph_layout.addPlot(row=1, col=0, title="Zoom: Envelope (Yellow) vs Threshold (Red)")
        self.zoom_env = self.zoom_env_plot.plot(pen=pg.mkPen('y', width=2))
        self.zoom_thr = self.zoom_env_plot.plot(pen=pg.mkPen('r', width=2))

        # Zoom Bits
        self.zoom_bit_plot = self.graph_layout.addPlot(row=1, col=1, title="Zoom: FPGA Hardware Bits")
        self.zoom_bit = self.zoom_bit_plot.plot(pen=pg.mkPen('c', width=2))

        self.zoom_env_plot.setXLink(self.zoom_bit_plot)
        self.macro_plot.setXLink(self.zoom_bit_plot)

    def load_csv(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open", "", "CSV Files (*.csv)")
        if file_path:
            data = np.loadtxt(file_path, delimiter=',', skiprows=1)
            time_us = data[:, 0]
            thresh = data[:, 1]
            envelope = data[:, 2]
            bits = data[:, 3]

            self.macro_env.setData(time_us, envelope)
            self.macro_thr.setData(time_us, thresh)
            self.zoom_env.setData(time_us, envelope)
            self.zoom_thr.setData(time_us, thresh)
            self.zoom_bit.setData(time_us, bits)
            self.zoom_bit_plot.setXRange(time_us[0], time_us[-1])

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = CSVPlaybackWindow()
    window.show()
    sys.exit(app.exec())