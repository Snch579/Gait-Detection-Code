import smbus
import time
import csv
import os
import numpy as np
import matplotlib.pyplot as plt

# === Setup ===
MPU_ADDR = 0x68
DATA_FILE = "gait_data.csv"
alpha = 0.8  # High-pass filter constant
step_threshold = 0.4  # Default step detection threshold

# === Initialize I2C
bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up MPU6050

# === Globals
gravity = [0.0, 0.0, 0.0]

# === Sensor Reading ===
def read_raw_data(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr+1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def read_accel():
    x = read_raw_data(0x3B) / 16384.0
    y = read_raw_data(0x3D) / 16384.0
    z = read_raw_data(0x3F) / 16384.0
    return [x, y, z]

def read_gyro():
    gx = read_raw_data(0x43) / 131.0
    gy = read_raw_data(0x45) / 131.0
    gz = read_raw_data(0x47) / 131.0
    return [gx, gy, gz]

def read_motion():
    global gravity
    accel = read_accel()
    gyro = read_gyro()
    linear_accel = [0, 0, 0]
    for i in range(3):
        gravity[i] = alpha * gravity[i] + (1 - alpha) * accel[i]
        linear_accel[i] = accel[i] - gravity[i]
    return {
        "accel": accel,
        "gyro": gyro,
        "linear": linear_accel
    }

# === Plot Gait Comparison ===
def plot_gait_vectors(recorded_mean, live_mean):
    features = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
    x = np.arange(len(features))
    
    plt.figure(figsize=(8,6))
    plt.plot(x, recorded_mean, marker='o', label='Recorded Gait')
    plt.plot(x, live_mean, marker='s', label='Live Gait')
    
    plt.xticks(x, features)
    plt.title('Gait Pattern Comparison')
    plt.ylabel('Sensor Value')
    plt.legend()
    plt.grid(True)
    plt.show()

# === Record Gait ===
def record_gait_cycle(threshold):
    user_input = input("Enter step detection threshold in g (e.g. 0.4): ")
    try:
        threshold = float(user_input)
    except ValueError:
        print("Invalid input. Using previous/default threshold.")

    num_steps = input("Enter number of steps to record (e.g. 3): ")
    try:
        num_steps = int(num_steps)
    except ValueError:
        print("Invalid input. Using default of 3 steps.")
        num_steps = 3

    print(f"Start walking... recording {num_steps} steps.")
    data = []
    step_count = 0
    prev_y = 0
    cooldown = 0

    while step_count < num_steps:
        motion = read_motion()
        ax, ay, az = motion["linear"]
        gx, gy, gz = motion["gyro"]
        timestamp = time.time()

        if ay > threshold and prev_y <= threshold and cooldown == 0:
            step_count += 1
            print(f" Step {step_count} detected.")
            cooldown = 10

        if step_count > 0 and cooldown == 10:  # Start recording only after first step
            data.append([ax, ay, az, gx, gy, gz, timestamp])

        prev_y = ay
        if cooldown > 0:
            cooldown -= 1

        time.sleep(0.05)

    with open(DATA_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'time'])
        writer.writerows(data)

    print(f"? {num_steps} steps recorded and saved.\n")
    return threshold
# === Compare Gait ===
def compare_to_recorded_gait(threshold):
    if not os.path.exists(DATA_FILE):
        print("? No recorded gait data found.")
        return

    try:
        num_steps = int(input("Enter number of steps to compare: "))
    except:
        print("Invalid step count. Defaulting to 3.")
        num_steps = 3

    print(f" Walk {num_steps} steps... comparing to recorded gait.")
    recorded_data = np.genfromtxt(DATA_FILE, delimiter=",", skip_header=1)
    recorded_data = recorded_data[~np.isnan(recorded_data).any(axis=1)]

    if len(recorded_data) == 0:
        print("? No valid data found.")
        return

    live_data = []
    step_count = 0
    prev_y = 0
    cooldown = 0

    while step_count < num_steps:
        motion = read_motion()
        ax, ay, az = motion["linear"]
        gx, gy, gz = motion["gyro"]
        sample = [ax, ay, az, gx, gy, gz]

        if ay > threshold and prev_y <= threshold and cooldown == 0:
            step_count += 1
            print(f" Step {step_count} detected.")
            cooldown = 10

        if step_count > 0 and cooldown == 10:
            live_data.append(sample)

        prev_y = ay
        if cooldown > 0:
            cooldown -= 1

        time.sleep(0.05)

    # Compare mean vectors
    recorded_mean = np.mean(recorded_data[:, :6], axis=0)
    live_mean = np.mean(np.array(live_data)[:, :6], axis=0)

    distance = np.linalg.norm(recorded_mean - live_mean)

    print(f"\n Overall gait difference distance: {distance:.4f}")

    if distance <= 60.0:
        print("? Gait appears NORMAL.\n")
    else:
        print(" Gait appears ABNORMAL.\n")

    plot_gait_vectors(recorded_mean, live_mean)

# === Clear Data ===
def clear_data():
    global step_threshold
    step_threshold = 0.4
    if os.path.exists(DATA_FILE):
        os.remove(DATA_FILE)
        print(" Gait data cleared. Threshold reset to 0.4g.\n")
    else:
        print(" No data to clear.\n")

# === Menu ===
def main_loop():
    global step_threshold
    while True:
        print("=== GAIT MONITOR MENU ===")
        print("1 - Record gait cycle")
        print("2 - Compare new gait to recorded gait")
        print("3 - Clear gait data")
        print("4 - Exit")
        choice = input("Enter your choice: ")

        if choice == "1":
            step_threshold = record_gait_cycle(step_threshold)
        elif choice == "2":
            compare_to_recorded_gait(step_threshold)
        elif choice == "3":
            clear_data()
        elif choice == "4":
            print(" Exiting. Stay healthy!")
            break
        else:
            print("? Invalid option.\n")

main_loop()
