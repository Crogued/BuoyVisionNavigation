import subprocess
import tkinter as tk
from tkinter import messagebox
import os
import signal
import shutil
from datetime import datetime

class VideoRecorderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Video Recorder with libcamera")

        self.recording = False
        self.pprocess = None
        self.temp_output = "video_temp.mp4"

        # Custom path where videos will be saved
        self.destination_dir = "/home"

        # Create directory if it doesn't exist
        os.makedirs(self.destination_dir, exist_ok=True)

        # GUI
        self.start_button = tk.Button(root, text="Start Recording", width=25, command=self.start_recording)
        self.start_button.pack(pady=10)

        self.stop_button = tk.Button(root, text="Stop Recording", width=25, state=tk.DISABLED, command=self.stop_recording)
        self.stop_button.pack(pady=10)

        self.quit_button = tk.Button(root, text="Exit", width=25, command=self.quit_app)
        self.quit_button.pack(pady=10)

    def start_recording(self):
        if self.pprocess is None:
            command = [
                "libcamera-vid",
                "--inline",
                "-t", "0",
                "--width", "1920",
                "--height", "1080",
                "--codec", "h264",
                "-o", self.temp_output
            ]

            self.pprocess = subprocess.Popen(command, preexec_fn=os.setsid)
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.recording = True
            print("Recording started...")

    def stop_recording(self):
        if self.pprocess is not None:
            print("Stopping recording...")
            os.killpg(os.getpgid(self.pprocess.pid), signal.SIGINT)
            self.pprocess.wait()
            self.pprocess = None
            self.recording = False
            self.start_button.config(state=tk.NORMAL)
            self.stop_button.config(state=tk.DISABLED)

            # Name based on date and time
            now = datetime.now()
            filename = now.strftime("video_%Y-%m-%d_%H-%M-%S.mp4")
            final_path = os.path.join(self.destination_dir, filename)

            try:
                shutil.move(self.temp_output, final_path)
                messagebox.showinfo("Success", f"Video saved at:\n{final_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Error saving video: {e}")

    def quit_app(self):
        if self.recording:
            self.stop_recording()
        self.root.quit()

# Initialization
root = tk.Tk()
app = VideoRecorderApp(root)
root.mainloop()
