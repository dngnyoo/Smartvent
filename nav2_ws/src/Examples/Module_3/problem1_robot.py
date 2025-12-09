
#!/usr/bin/env python3
"""
Problem 1 Solution
- Robot traces a 0.5 m square (side = 50 cm).
- If an obstacle is detected at ≤ 25 cm, the robot stops, rotates 180°, and
  continues tracing the square in the opposite direction (i.e., reverse the
  turn direction from CW to CCW or vice versa).
- Path trace is logged to CSV: problem1_pathtrace.csv with columns:
  index, encoder_value, distance (cm).
  Sampling rate: ≥ 2 Hz (we use 5 Hz by default).

Bonus (optional):
- Run with --camera to start an MJPEG camera stream at /video_feed .
  Example: python3 problem1_robot.py --camera
  Then open http://<raspberrypi-ip>:5000/
"""
import argparse
import csv
import sys
import time
import threading

# GoPiGo3 drivers
try:
    import easygopigo3 as easy
    import gopigo3 as gpg3
except Exception as e:
    print("Failed to import GoPiGo3 libraries:", e)
    sys.exit(1)

# Optional camera streaming (bonus)
try:
    import cv2
    from flask import Flask, Response, render_template_string
    CAMERA_AVAILABLE = True
except Exception:
    CAMERA_AVAILABLE = False


###############################################################################
# Camera streaming (bonus)
###############################################################################

def start_camera_server(stop_event, host="0.0.0.0", port=5000, cam_index=0):
    """
    Starts a very simple MJPEG server using Flask + OpenCV.
    Visit http://<host>:<port>/ for a test page, and /video_feed for MJPEG stream.
    """
    if not CAMERA_AVAILABLE:
        print("[Camera] OpenCV/Flask not found. Skipping camera server.")
        return

    app = Flask(__name__)

    PAGE_HTML = """
    <html><head><title>Robot Camera</title></head>
    <body>
      <h2>Robot Camera Stream</h2>
      <img src="/video_feed" style="max-width: 100%; height: auto;" />
    </body></html>
    """

    cap = cv2.VideoCapture(cam_index)

    def gen_frames():
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            ok, buffer = cv2.imencode('.jpg', frame)
            if not ok:
                continue
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    @app.route('/')
    def index():
        return render_template_string(PAGE_HTML)

    @app.route('/video_feed')
    def video_feed():
        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def run_app():
        # Use threaded server so it doesn't block the main thread
        app.run(host=host, port=port, debug=False, threaded=True, use_reloader=False)

    server_thread = threading.Thread(target=run_app, daemon=True)
    server_thread.start()
    print(f"[Camera] MJPEG server started at http://{host}:{port}/ (Ctrl+C to stop)")


###############################################################################
# Robot behavior
###############################################################################

class SquareTracer:
    def __init__(self, side_cm=50.0, obstacle_cm=25.0, log_path="problem1_pathtrace.csv",
                 sample_hz=5.0, drive_step_cm=1.0, speed_dps=250):
        """
        side_cm: length of each side of the square to trace
        obstacle_cm: if distance sensor reading <= obstacle_cm, perform the 180° avoidance
        log_path: CSV path for path trace
        sample_hz: logging rate (Hz) ≥ 2; default 5 Hz
        drive_step_cm: drive increment for obstacle polling (smaller = more reactive)
        speed_dps: wheel speed in degrees per second
        """
        self.side_cm = float(side_cm)
        self.obstacle_cm = float(obstacle_cm)
        self.log_path = log_path
        self.sample_dt = 1.0 / max(2.0, float(sample_hz))
        self.drive_step_cm = float(drive_step_cm)
        self.speed_dps = int(speed_dps)

        # High-level API for movement + distance sensor
        self.egpg = easy.EasyGoPiGo3()
        self.dist = self.egpg.init_distance_sensor()

        # Low-level API for encoders
        self.gpg = gpg3.GoPiGo3()
        # Zero encoders at start for cleaner logs
        self.gpg.offset_motor_encoder(self.gpg.MOTOR_LEFT, self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT))
        self.gpg.offset_motor_encoder(self.gpg.MOTOR_RIGHT, self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT))

        # Speed
        self.egpg.set_speed(self.speed_dps)

        # Logging
        self._log_index = 0
        self._last_sample_t = 0.0
        self._csv_file = open(self.log_path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["index", "encoder_value", "distance_cm"])

        # Turning direction: +90 (CW) by default; flips to -90 on obstacle
        self._turn_sign = +90

    def _read_distance_cm(self):
        # Distance sensor returns millimeters; convert to cm. Handle out-of-range gracefully.
        try:
            mm = self.dist.read_mm()
            if mm is None or mm <= 0:
                return float("inf")
            return mm / 10.0
        except Exception:
            return float("inf")

    def _read_encoder_avg(self):
        try:
            el = self.gpg.get_motor_encoder(self.gpg.MOTOR_LEFT)
            er = self.gpg.get_motor_encoder(self.gpg.MOTOR_RIGHT)
            return 0.5 * (float(el) + float(er))
        except Exception:
            return float("nan")

    def _maybe_log(self):
        now = time.time()
        if now - self._last_sample_t >= self.sample_dt:
            enc = self._read_encoder_avg()
            dcm = self._read_distance_cm()
            self._csv.writerow([self._log_index, f"{enc:.2f}", f"{dcm:.1f}"])
            self._csv_file.flush()
            self._log_index += 1
            self._last_sample_t = now

    def _safe_stop(self):
        try:
            self.egpg.stop()
        except Exception:
            pass

    def cleanup(self):
        # Always call this on exit
        self._safe_stop()
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass
        try:
            self.gpg.reset_all()
        except Exception:
            pass

    def _turn(self, degrees):
        # Turn using EasyGoPiGo3's IMU-less turning by encoders
        self.egpg.turn_degrees(degrees)

    def _drive_step_checked(self, cm):
        """
        Drive forward 'cm' centimeters and constantly check distance + log.
        Return True if obstacle encountered (≤ threshold), otherwise False.
        """
        # Non-blocking move in small increments to allow frequent checks
        # We use drive_cm with blocking=True but in sub-steps to poll in between.
        # Each sub-step also logs at ≥ 2 Hz.
        remaining = float(cm)
        while remaining > 0.0:
            step = min(remaining, self.drive_step_cm)
            # Perform the step
            self.egpg.drive_cm(step, True)
            # After the step, sample/log
            t_poll_start = time.time()
            while time.time() - t_poll_start < 0.05:  # brief window to enforce logging cadence
                self._maybe_log()
                time.sleep(0.005)

            # Check obstacle after each small step
            dcm = self._read_distance_cm()
            if dcm <= self.obstacle_cm:
                self._safe_stop()
                return True  # obstacle hit
            remaining -= step
        return False

    def trace_square(self):
        """
        Trace a 4-sided square. If obstacle is detected at any point while moving straight,
        rotate 180° and continue tracing with reversed turn direction.
        """
        try:
            for side in range(4):
                # Move along the side with obstacle checks
                obstacle_hit = self._drive_step_checked(self.side_cm)
                if obstacle_hit:
                    print(f"[SquareTracer] Obstacle ≤ {self.obstacle_cm} cm detected. Executing 180° turn and reversing direction.")
                    # 180 turn
                    self._turn(180)
                    # Flip turning direction for future corners
                    self._turn_sign = -self._turn_sign
                    # Continue the remaining square (this side is considered complete)
                # Corner turn
                self._turn(self._turn_sign)
                # Ensure we log at least once per corner
                self._maybe_log()

            # Final stop
            self._safe_stop()

        except KeyboardInterrupt:
            print("[SquareTracer] Stopped by user (Ctrl+C).")
        finally:
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(description="Problem 1: 0.5 m square with obstacle avoidance + path logging")
    parser.add_argument("--camera", action="store_true", help="Start MJPEG camera stream (bonus)")
    parser.add_argument("--side_cm", type=float, default=50.0, help="Square side length in cm (default: 50)")
    parser.add_argument("--obstacle_cm", type=float, default=25.0, help="Obstacle threshold in cm (default: 25)")
    parser.add_argument("--log", type=str, default="problem1_pathtrace.csv", help="CSV log file path")
    parser.add_argument("--sample_hz", type=float, default=5.0, help="Logging rate in Hz (≥2)")
    parser.add_argument("--step_cm", type=float, default=1.0, help="Drive sub-step increment in cm")
    parser.add_argument("--speed_dps", type=int, default=250, help="Motor speed in deg/s")
    args = parser.parse_args()

    # Optional camera server
    stop_event = threading.Event()
    if args.camera:
        start_camera_server(stop_event)

    tracer = SquareTracer(
        side_cm=args.side_cm,
        obstacle_cm=args.obstacle_cm,
        log_path=args.log,
        sample_hz=args.sample_hz,
        drive_step_cm=args.step_cm,
        speed_dps=args.speed_dps
    )

    try:
        tracer.trace_square()
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
