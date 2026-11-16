"""Keyboard jog for the SCARA (firmware: src/manual.cpp).

Arrows jog X/Y along the base plate edges (Up = the plate's forward
direction) for as long as they are held, W/S jog Z, F flips the elbow, O/C open and close the gripper,
Space stops, H homes Z. Keys only act while this
window has focus; losing focus stops the arm. Close the PlatformIO serial
monitor first, it holds the port.

    .venv\\Scripts\\python host\\jog.py [--port COM7]
"""

import argparse
import queue
import threading
import tkinter as tk

import serial

SEND_MS = 50
POLL_MS = 20
# On some platforms key repeat sends release+press pairs; a release only
# counts if no press follows within this time.
RELEASE_DEBOUNCE_MS = 30

KEY_AXES = {
    "Left": (-1, 0, 0),
    "Right": (1, 0, 0),
    "Up": (0, 1, 0),
    "Down": (0, -1, 0),
    "w": (0, 0, 1),
    "s": (0, 0, -1),
}

# Sent once per key press, ignoring key repeat.
KEY_COMMANDS = {"f": "F", "o": "O", "c": "C"}


class Jog:
    def __init__(self, root, port, xy_speed, z_speed):
        self.root = root
        self.xy_speed = xy_speed
        self.z_speed = z_speed
        self.held = set()
        self.pending_release = {}
        self.moving = False
        self.command_held = set()
        self.lines = queue.Queue()

        self.ser = serial.Serial(port, 115200, timeout=0.1)
        threading.Thread(target=self.read_serial, daemon=True).start()

        root.title(f"SCARA jog ({port})")
        font = ("Consolas", 14)
        self.pos_var = tk.StringVar(value="waiting for arm...")
        tk.Label(root, textvariable=self.pos_var, font=font, justify="left").pack(
            padx=10, pady=(10, 4), anchor="w"
        )
        tk.Label(
            root,
            text="Arrows: X/Y   W/S: Z   F: flip elbow   O/C: gripper   Space: stop   H: home Z",
            font=("Consolas", 10),
        ).pack(padx=10, anchor="w")
        self.log = tk.Text(root, height=10, width=60, font=("Consolas", 10), state="disabled")
        self.log.pack(padx=10, pady=10, fill="both", expand=True)

        root.bind("<KeyPress>", self.on_press)
        root.bind("<KeyRelease>", self.on_release)
        root.bind("<FocusOut>", lambda e: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(SEND_MS, self.send_loop)
        root.after(POLL_MS, self.poll_lines)

    def key_name(self, event):
        k = event.keysym
        return k.lower() if len(k) == 1 else k

    def on_press(self, event):
        k = self.key_name(event)
        if k in self.pending_release:
            self.root.after_cancel(self.pending_release.pop(k))
        if k in KEY_AXES:
            self.held.add(k)
        elif k == "space":
            self.stop()
        elif k == "h":
            self.stop()
            self.send("H")
        elif k in KEY_COMMANDS and k not in self.command_held:
            self.command_held.add(k)
            self.send(KEY_COMMANDS[k])

    def on_release(self, event):
        k = self.key_name(event)
        self.command_held.discard(k)
        if k in KEY_AXES and k not in self.pending_release:
            self.pending_release[k] = self.root.after(
                RELEASE_DEBOUNCE_MS, lambda: self.release(k)
            )

    def release(self, k):
        self.pending_release.pop(k, None)
        self.held.discard(k)

    def stop(self):
        for after_id in self.pending_release.values():
            self.root.after_cancel(after_id)
        self.pending_release.clear()
        self.held.clear()
        self.command_held.clear()
        self.moving = False
        self.send("S")

    def velocity(self):
        x = sum(KEY_AXES[k][0] for k in self.held)
        y = sum(KEY_AXES[k][1] for k in self.held)
        z = sum(KEY_AXES[k][2] for k in self.held)
        norm = (x * x + y * y) ** 0.5 or 1.0
        return x / norm * self.xy_speed, y / norm * self.xy_speed, z * self.z_speed

    def send_loop(self):
        vx, vy, vz = self.velocity()
        if vx or vy or vz:
            self.send(f"V {vx:.2f} {vy:.2f} {vz:.2f}")
            self.moving = True
        elif self.moving:
            self.send("V 0 0 0")
            self.moving = False
        self.root.after(SEND_MS, self.send_loop)

    def send(self, line):
        try:
            self.ser.write((line + "\n").encode())
        except serial.SerialException as e:
            self.add_log(f"! write failed: {e}")

    def read_serial(self):
        while True:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                return
            if raw:
                self.lines.put(raw.decode(errors="replace").strip())

    def poll_lines(self):
        while not self.lines.empty():
            line = self.lines.get()
            parts = line.split()
            if parts[:1] == ["P"] and len(parts) == 6:
                x, y, z, t1, t2 = parts[1:]
                self.pos_var.set(
                    f"X {float(x):8.2f}  Y {float(y):8.2f}  Z {float(z):8.2f} mm\n"
                    f"shoulder {float(t1):6.1f}  elbow {float(t2):6.1f} deg"
                )
            elif line:
                self.add_log(line)
        self.root.after(POLL_MS, self.poll_lines)

    def add_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def close(self):
        try:
            self.send("S")
            self.ser.close()
        finally:
            self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--xy-speed", type=float, default=60.0, help="mm/s")
    ap.add_argument("--z-speed", type=float, default=20.0, help="mm/s")
    args = ap.parse_args()

    root = tk.Tk()
    Jog(root, args.port, args.xy_speed, args.z_speed)
    root.mainloop()


if __name__ == "__main__":
    main()
