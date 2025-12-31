from pymavlink import mavutil
import time

# --- KONFIGURASI KONEKSI ---
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Terhubung ke Pixhawk:", master.target_system)

# --- FUNGSI UTAMA: KIRIM PWM ---
def set_rc_channel_pwm(pitch, roll, throttle, yaw, forward):
    """
    Mengirim sinyal override RC ke Pixhawk.
    Nilai PWM:
    1100 = Min (Mundur/Turun/Kiri Penuh)
    1500 = Stop (Netral)
    1900 = Max (Maju/Naik/Kanan Penuh)
    
    Mapping Channel ArduSub Standar:
    Ch 1 = Pitch (Tilt)
    Ch 2 = Roll  (Kiri/Kanan)
    Ch 3 = Throttle (Naik/Turun)
    Ch 4 = Yaw   (Putar)
    Ch 5 = Forward (Maju/Mundur)
    Ch 6 = Lateral (Roll)
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        pitch,      # Channel 1 (Pitch)
        roll,       # Channel 2 (Roll)
        throttle,   # Channel 3 (Throttle)
        yaw,        # Channel 4 (Yaw)
        forward,    # Channel 5 (Forward)
        0, 0, 0, # Channel 6-8
        0, 0, 0, 0, 0, 0, 0, 0 # Channel 9-16
    )

# --- SETUP AWAL ---

# 1. Set Mode MANUAL (ID 19)
# Pastikan mode MANUAL agar Pixhawk tidak mencoba menstabilkan diri (PID mati)
mode_id = 19
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    1, mode_id, 0, 0, 0, 0, 0
)
time.sleep(1)
print("Mode set to MANUAL")

# 2. ARM Vehicle
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("ARMING...")
time.sleep(2) # Tunggu motor siap

# --- DEFINISI GERAKAN DENGAN WAKTU ---
def move_for(seconds, pitch=1500, roll=1500, throttle=1500, yaw=1500, forward=1500):
    start_time = time.time()
    while time.time() - start_time < seconds:
        set_rc_channel_pwm(pitch, roll, throttle, yaw, forward)
        time.sleep(0.1) # Kirim data setiap 0.1 detik (10Hz)
    
    # Stop setelah waktu habis
    set_rc_channel_pwm(1500, 1500, 1500, 1500, 1500)

print("--- MULAI MISI ---")

# Inisialisasi: Pastikan semua STOP dulu (1500)
set_rc_channel_pwm(1500, 1500, 1500, 1500, 1500)
time.sleep(1)

print("TURUN - 5 Detik")
move_for(5, pitch=1500, roll=1500, throttle=1400, yaw=1500, forward = 1500)

print("MAJU - 7 Detik")
move_for(7, pitch=1500, roll=1500, throttle=1550, yaw=1500, forward = 1600)

print("MUNDUR - 7 Detik")
move_for(7, pitch=1500, roll=1500, throttle=1550, yaw=1500, forward = 1400)

print("NAIK - 5 Detik")
move_for(5, pitch=1500, roll=1500, throttle=1600, yaw=1500, forward = 1500)

# STOP TOTAL
print("STOP.")
set_rc_channel_pwm(1500, 1500, 1500, 1500, 1500)
