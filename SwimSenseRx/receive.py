
import serial
import time
import struct
import logging
from datetime import datetime
from collections import deque
from threading import Thread, Event
import csv
import os
import matplotlib.pyplot as plt
import heartpy
import matplotlib.widgets as mwidgets
from dotenv import load_dotenv
load_dotenv()

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)
logging.getLogger('matplotlib.font_manager').disabled = True
class SwimSenseReceiver:
    def __init__(self, port, baudrate=115200, buffer_size=100):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        self.serial_conn = None
        self.running = Event()
        self.data_buffer = deque(maxlen=buffer_size)
        self.thread = None
        self.last_id = -1
        self.columns = ["vx","vy","vz","qw","qx","qy","qz","hm","rssi","dt","id"]
        self.csv_buffer = []         # batch CSV writes
        self.csv_flush_every = 50

    def start(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.running.set()
            self.thread = Thread(target=self._read_data)
            self.thread.start()
            logger.info("Started SwimSenseReceiver on port %s", self.port)
        except serial.SerialException as e:
            logger.error("Error opening serial port: %s", e)

    def stop(self):
        self.running.clear()
        if self.thread:
            self.thread.join()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        logger.info("Stopped SwimSenseReceiver")

    def _read_data(self):
        while self.running.is_set():
            try:
                raw = self.serial_conn.readline()  # blocks up to timeout, no manual sleep
                if not raw:
                    continue
                dp = self._parse_data(raw)
                if not dp:
                    continue
                curr_id = dp.get("id")
                if curr_id is not None and curr_id == self.last_id:
                    continue
                self.last_id = curr_id
                self.data_buffer.append(dp)  # append as we go
            except serial.SerialException as e:
                logger.error("Serial error: %s", e)
                break

    def _parse_data(self, raw_data):
        try:
            data_str = raw_data.decode('utf-8').strip()
            data_parts = data_str.split(',')
            
            if len(data_parts) != len(self.columns):
                logger.warning("Unexpected data format: %s", data_str)
                return None
            parsed_data = dict(zip(self.columns, data_parts))
            return parsed_data
        except (ValueError, UnicodeDecodeError) as e:
            logger.error("Data parsing error: %s", e)
            return None

    def get_latest_data(self):
        return list(self.data_buffer)
    
    def clear_data(self):
        self.data_buffer.clear()

import numpy as np

if __name__ == "__main__":
    receiver = SwimSenseReceiver(port=os.getenv('USB_PORT'), baudrate=115200)
    receiver.start()

    # --- CSV setup ---
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = "./logs"; os.makedirs(log_dir, exist_ok=True)
    csv_file_path = f"{log_dir}/swimsense_data_{timestamp}.csv"

    # --- plotting setup (pre-create everything) ---
    plt.ion()
    fig, axs = plt.subplots(4, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Real-time SwimSense Data')

    time_window = 200  # last N samples (or switch to seconds if you have absolute time)
    t  = deque(maxlen=time_window)
    vx = deque(maxlen=time_window); vy = deque(maxlen=time_window); vz = deque(maxlen=time_window)
    qw = deque(maxlen=time_window); qx = deque(maxlen=time_window); qy = deque(maxlen=time_window); qz = deque(maxlen=time_window)
    hm = deque(maxlen=time_window); rssi = deque(maxlen=time_window)
    axs[0].set_ylim(-5, 5)   # velocity m/s
    axs[1].set_ylim(-1, 1)   # quaternion components
    axs[2].set_ylim(-120, 0) # RSSI dBm
    axs[3].set_ylim(0, 5000) # heart monitor
    # Pre-create lines once
    (vx_ln,) = axs[0].plot([], [], label="vx")
    (vy_ln,) = axs[0].plot([], [], label="vy")
    (vz_ln,) = axs[0].plot([], [], label="vz")
    axs[0].set_title('Velocity Components'); axs[0].legend(); axs[0].grid(True)

    (qw_ln,) = axs[1].plot([], [], label="qw")
    (qx_ln,) = axs[1].plot([], [], label="qx")
    (qy_ln,) = axs[1].plot([], [], label="qy")
    (qz_ln,) = axs[1].plot([], [], label="qz")
    axs[1].set_title('Quaternion Components'); axs[1].legend(); axs[1].grid(True)

    (rssi_ln,) = axs[2].plot([], [], label="rssi")
    axs[2].set_title('RSSI'); axs[2].legend(); axs[2].grid(True)

    (hm_ln,) = axs[3].plot([], [], label="hm")
    axs[3].set_title('Heart Monitor'); axs[3].legend(); axs[3].grid(True)
    axs[3].set_xlabel('Time (s)')

    # x-axis autoscale helper
    def update_xlim(ax):
        if len(t) >= 2:
            ax.set_xlim(t[0], t[-1])

    # time from dt (cumulative). Prefer device timestamp if you have one.
    curr_time = 0.0
    i = 0
    heartrate_bpm = 0
    rmssd = 0
    sdnn = 0

    button_labels = ['Freestyle', 'Butterfly', 'Breaststroke', 'Backstroke', 'Standing', "Treading"]
    button_axes = []
    buttons = []
    button_y = 0.01
    button_w = 0.13
    button_h = 0.05
    total_width = len(button_labels) * button_w + (len(button_labels) - 1) * 0.02
    start_x = 0.5 - total_width / 2
    
    for idx, label in enumerate(button_labels):
        ax_btn = plt.axes((start_x + idx * (button_w + 0.02), button_y, button_w, button_h))
        btn = mwidgets.Button(ax_btn, label, useblit=True, color='0.85', hovercolor='0.85')
        button_axes.append(ax_btn)
        buttons.append(btn)

    def make_onclick(lbl):
        def onclick(event):
            # Toggle or set a global variable for this button
            global swimmer_state
            swimmer_state = lbl
            print(f"Swimmer state set to: {swimmer_state}")
            
        return onclick

    for btn, lbl in zip(buttons, button_labels):
        btn.on_clicked(make_onclick(lbl))
    
    swimmer_state = "Unknown"

    with open(csv_file_path, mode='w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=receiver.columns + ["swimmer_state"])
        writer.writeheader()
        def on_close(event):
            if receiver.csv_buffer:
                writer.writerows(receiver.csv_buffer)
                csvfile.flush()
            logger.info("Closing plot and stopping receiver...")
            receiver.stop()
            exit(0)
            
        fig.canvas.mpl_connect('close_event', on_close)
        try:
            while True:
                # drain what the RX thread has accumulated (non-blocking snapshot)
                batch = receiver.get_latest_data()
                if not batch:
                    plt.pause(0.001)
                    continue

                # process entries (no buffer clear needed if you pop from left; here we just snapshot, then clear)
                receiver.clear_data()

                for entry in batch:
                    try:
                        # CSV: buffer writes, flush every N
                        entry["swimmer_state"] = swimmer_state
                        receiver.csv_buffer.append(entry)
                        if len(receiver.csv_buffer) >= receiver.csv_flush_every:
                            writer.writerows(receiver.csv_buffer)
                            receiver.csv_buffer.clear()
                            csvfile.flush()

                        # Parse fields
                        dt_ms = float(entry["dt"])  # ensure your sender sends dt in ms (or change to s)
                        curr_time += dt_ms / 1000.0
                        t.append(curr_time)

                        vx.append(float(entry["vx"])); vy.append(float(entry["vy"])); vz.append(float(entry["vz"]))
                        qw.append(float(entry["qw"])); qx.append(float(entry["qx"])); qy.append(float(entry["qy"])); qz.append(float(entry["qz"]))
                        hm.append(float(entry["hm"]))
                        rssi.append(float(entry["rssi"]))
                    except ValueError as e:
                        logger.warning("Bad entry skipped: %s", e)

                # Update line data (no cla/legend/grid)
                x = np.fromiter(t, float)  # cheap view into deque
                vx_ln.set_data(x, np.fromiter(vx, float))
                vy_ln.set_data(x, np.fromiter(vy, float))
                vz_ln.set_data(x, np.fromiter(vz, float))

                qw_ln.set_data(x, np.fromiter(qw, float))
                qx_ln.set_data(x, np.fromiter(qx, float))
                qy_ln.set_data(x, np.fromiter(qy, float))
                qz_ln.set_data(x, np.fromiter(qz, float))

                rssi_ln.set_data(x, np.fromiter(rssi, float))
                hm_ln.set_data(x, np.fromiter(hm, float))
                # Heart rate calculation using heartpy
                hr_window = 5 
                if len(hm) >= 50 and i % 20 == 0:  # need enough data points
                    try:
                        filtered_hm = np.clip(np.fromiter(hm, float), 0, 5000)  # Clip signal to expected range
                        wd, m = heartpy.process(filtered_hm, sample_rate=1.0/(np.mean(np.diff(x)) if len(x) > 1 else 1.0))
                        heartrate_bpm = m['bpm']
                        rmssd = m['rmssd']
                        sdnn = m['sdnn']
                        axs[3].set_title(f'Heart Monitor - HR: {heartrate_bpm:.1f} bpm, RMSSD: {rmssd:.1f} ms, SDNN: {sdnn:.1f} ms')
                    except Exception as e:
                        logger.warning("Could not read heart rate")
                
                # Keep y-lims sensible; set once or update lazily
                # axs[0].set_ylim(-5, 5)  # example if your ranges are known
                update_xlim(axs[0]); update_xlim(axs[1]); update_xlim(axs[2]); update_xlim(axs[3])
                fig.subplots_adjust(bottom=0.15)

                    
                # Ensure all axes legends are horizontal and at the bottom center
                for ax in axs:
                    ax.legend(loc='lower right', bbox_to_anchor=(1.0, 0), ncol=4)

                # Color button with color if swimmer_state matches
                for btn, lbl in zip(buttons, button_labels):
                    if swimmer_state == lbl:
                        btn.color = 'lightblue'
                        btn.hovercolor = 'lightblue'
                    else:
                        btn.color = '0.85'  # default gray
                        btn.hovercolor = '0.85'
                    btn.ax.set_facecolor(btn.color)
                
                # You can now use button_states['A'], button_states['B'], etc. as global variables
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                # tiny pause lets the UI event loop breathe without stalling
                plt.pause(0.001)
                #exit if window closed fig.canvas.mpl_connect('close_event', on_close)
                
                i += 1

        except KeyboardInterrupt:
            # flush remaining buffered rows on exit
            on_close(None)