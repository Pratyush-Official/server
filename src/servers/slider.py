#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import threading
import time
from collections import deque
from datetime import datetime
import os

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# ================= CONFIG =================
WINDOW_SEC = 2.0
SAMPLE_PERIOD = 0.01

GAIN_MIN = 0.0
GAIN_MAX = 10.0

SP_MIN = 0.0
SP_MAX = 10000.0

LOG_FILE = os.path.expanduser("~/pid_log.txt")  # ✅ absolute path
# ==========================================


# ================= ROS NODE =================
class PIDTunerNode(Node):

    def __init__(self):
        super().__init__('pid_tuner_gui')

        self.create_subscription(
            Float32MultiArray,
            'pid_tune',
            self.pid_callback,
            10
        )

        self.gain_pub = self.create_publisher(
            Float32MultiArray,
            'pid_gains',
            10
        )

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.setpoint = 500.0

        self.create_timer(0.02, self.publish_gains)

        self.start_time = time.time()
        self.max_len = int(WINDOW_SEC / SAMPLE_PERIOD)

        self.t = deque(maxlen=self.max_len)
        self.ipx = deque(maxlen=self.max_len)
        self.ipy = deque(maxlen=self.max_len)
        self.spx = deque(maxlen=self.max_len)
        self.spy = deque(maxlen=self.max_len)

    def pid_callback(self, msg):
        if len(msg.data) < 4:
            return

        now = time.time() - self.start_time
        self.t.append(now)
        self.ipx.append(msg.data[0])
        self.ipy.append(msg.data[1])
        self.spx.append(msg.data[2])
        self.spy.append(msg.data[3])

    def publish_gains(self):
        msg = Float32MultiArray()
        msg.data = [self.kp, self.ki, self.kd, self.setpoint]
        self.gain_pub.publish(msg)


# ================= GUI =================
class PIDTunerGUI:

    def __init__(self, node: PIDTunerNode):
        self.node = node
        self.sliders = {}
        self.steps = {}
        self.sp_history = []

        self.root = tk.Tk()
        self.root.title("PID Tuner")
        self.root.geometry("1300x820")

        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True)

        self._build_controls(main)
        self._build_plot(main)

        self.update_plot()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Controls ----------
    def _build_controls(self, parent):
        control = ttk.Frame(parent)
        control.pack(fill="x", padx=20, pady=10)

        ttk.Label(control, text="PID Controller", font=("Arial", 15)).pack(anchor="w")

        self._add_slider(control, "Kp", "kp")
        self._add_slider(control, "Ki", "ki")
        self._add_slider(control, "Kd", "kd")

        self._add_setpoint_entry(control)
        self._add_setpoint_history(control)
        self._add_logger(control)

    # ---------- Slider with FIXED arrow keys ----------
    def _add_slider(self, parent, label, attr):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=4)

        ttk.Label(row, text=label, width=4).pack(side="left")

        slider = ttk.Scale(
            row,
            from_=GAIN_MIN,
            to=GAIN_MAX,
            orient="horizontal",
            command=lambda v, a=attr: setattr(self.node, a, float(v))
        )
        slider.pack(side="left", fill="x", expand=True, padx=8)
        slider.set(getattr(self.node, attr))

        # ✅ GIVE FOCUS SO ARROWS WORK
        slider.bind("<Enter>", lambda e: slider.focus_set())

        step_var = tk.StringVar(value="0.001")
        step_entry = ttk.Entry(row, textvariable=step_var, width=7)
        step_entry.pack(side="left")
        ttk.Label(row, text="step").pack(side="left", padx=4)

        self.sliders[attr] = slider
        self.steps[attr] = step_var

        for key, sign in [
            ("<Up>", 1), ("<Right>", 1),
            ("<Down>", -1), ("<Left>", -1)
        ]:
            slider.bind(key, lambda e, a=attr, s=sign: self._adjust(a, s))

    def _adjust(self, attr, sign):
        slider = self.sliders[attr]
        try:
            step = float(self.steps[attr].get())
        except ValueError:
            return "break"

        val = slider.get() + sign * step
        val = max(GAIN_MIN, min(GAIN_MAX, val))
        slider.set(val)
        setattr(self.node, attr, val)
        return "break"

    # ---------- Setpoint ----------
    def _add_setpoint_entry(self, parent):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=6)

        ttk.Label(row, text="SP", width=4).pack(side="left")
        self.sp_var = tk.StringVar(value=str(self.node.setpoint))
        entry = ttk.Entry(row, textvariable=self.sp_var, width=12)
        entry.pack(side="left", padx=8)

        entry.bind("<Return>", self._set_setpoint)
        entry.bind("<FocusOut>", self._set_setpoint)

    def _set_setpoint(self, event=None):
        try:
            val = float(self.sp_var.get())
        except ValueError:
            return

        val = max(SP_MIN, min(SP_MAX, val))
        self.node.setpoint = val
        self.sp_var.set(str(val))

        if val not in self.sp_history:
            self.sp_history.insert(0, val)
            self._refresh_sp_buttons()

    # ---------- Setpoint History ----------
    def _add_setpoint_history(self, parent):
        self.sp_frame = ttk.LabelFrame(parent, text="Setpoint History")
        self.sp_frame.pack(fill="x", pady=6)

    def _refresh_sp_buttons(self):
        for w in self.sp_frame.winfo_children():
            w.destroy()

        for sp in self.sp_history[:8]:
            ttk.Button(
                self.sp_frame,
                text=str(sp),
                command=lambda v=sp: self._use_sp(v)
            ).pack(side="left", padx=4)

    def _use_sp(self, val):
        self.node.setpoint = val
        self.sp_var.set(str(val))

    # ---------- Logger (FIXED) ----------
    def _add_logger(self, parent):
        box = ttk.LabelFrame(parent, text="Log")
        box.pack(fill="x", pady=6)

        self.msg_var = tk.StringVar()
        ttk.Entry(box, textvariable=self.msg_var, width=60).pack(side="left", padx=6)

        ttk.Button(box, text="LOG", command=self._log).pack(side="left")

    def _log(self):
        try:
            with open(LOG_FILE, "a") as f:
                f.write(
                    f"{datetime.now().isoformat()}, "
                    f"kp={self.node.kp:.6f}, "
                    f"ki={self.node.ki:.6f}, "
                    f"kd={self.node.kd:.6f}, "
                    f"sp={self.node.setpoint}, "
                    f"msg={self.msg_var.get()}\n"
                )
                f.flush()
        except Exception as e:
            messagebox.showerror("Log error", str(e))
            return

        messagebox.showinfo("Logged", f"Saved to {LOG_FILE}")

    # ---------- Plot ----------
    def _build_plot(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill="both", expand=True)

        self.fig = Figure(figsize=(7, 5))
        self.ax_x = self.fig.add_subplot(211)
        self.ax_y = self.fig.add_subplot(212, sharex=self.ax_x)

        self.l_spx, = self.ax_x.plot([], [], label="SP X")
        self.l_ipx, = self.ax_x.plot([], [], label="IP X")
        self.l_spy, = self.ax_y.plot([], [], label="SP Y")
        self.l_ipy, = self.ax_y.plot([], [], label="IP Y")

        self.ax_x.legend()
        self.ax_y.legend()
        self.ax_x.grid(True)
        self.ax_y.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def update_plot(self):
        if self.node.t:
            self.l_spx.set_data(self.node.t, self.node.spx)
            self.l_ipx.set_data(self.node.t, self.node.ipx)
            self.l_spy.set_data(self.node.t, self.node.spy)
            self.l_ipy.set_data(self.node.t, self.node.ipy)

            self.ax_x.relim()
            self.ax_x.autoscale()
            self.ax_y.relim()
            self.ax_y.autoscale()
            self.canvas.draw_idle()

        self.root.after(10, self.update_plot)

    def on_close(self):
        self.root.quit()
        self.root.destroy()


# ================= MAIN =================
def main():
    rclpy.init()
    node = PIDTunerNode()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    PIDTunerGUI(node).root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
