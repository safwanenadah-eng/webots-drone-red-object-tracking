"""
TP2 : Drone stable + contrôle manuel + détection objet rouge
       + Machine à états : SCANNING -> APPROACHING -> HOVERING

CORRECTION DES SEUILS :
  - TARGET_RATIO_MIN   : 0.005 -> 0.02  (évite faux positifs)
  - TARGET_RATIO_CLOSE : 0.04  -> 0.25  (objet doit vraiment être proche)
  - Correction bug SCANNING : utilise 'detected' basé sur ratio > MIN
"""

from controller import Robot, Camera, InertialUnit, GPS, Gyro, Keyboard, LED
import numpy as np

# ================= PID =================
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P      = 3.0
K_ROLL_P          = 50.0
K_PITCH_P         = 30.0

def CLAMP(value, low, high):
    return max(low, min(value, high))

# ================= VITESSES MANUELLES =================
MANUAL_PITCH_SPEED = 1.0
MANUAL_YAW_SPEED   = 0.8
MANUAL_ROLL_SPEED  = 0.8

# ================= MACHINE À ÉTATS =================
SCANNING    = 0
APPROACHING = 1
HOVERING    = 2

SCAN_YAW_SPEED = 0.8
APPROACH_PITCH = -1.0

#
# ┌─────────────────────────────────────────────────────────┐
# │  SEUILS CORRIGÉS                                        │
# │                                                         │
# │  Ton ratio quand drone voit l'objet de loin : > 0.05   │
# │                                                         │
# │  TARGET_RATIO_MIN   = 0.02                              │
# │    → drone démarre APPROACHING seulement si ratio>0.02  │
# │    → évite de réagir à de petits reflets rouges         │
# │                                                         │
# │  TARGET_RATIO_CLOSE = 0.25                              │
# │    → HOVERING seulement si l'objet occupe 25% de        │
# │      l'image = drone vraiment au-dessus                 │
# │    → avant : 0.04 < 0.05 donc HOVERING immédiat ❌      │
# │    → maintenant : 0.25 >> 0.05 donc approche réelle ✅  │
# └─────────────────────────────────────────────────────────┘
#
TARGET_RATIO_MIN   = 0.02   # 2%  → détection validée
TARGET_RATIO_CLOSE = 0.25   # 25% → cible très proche → HOVERING

STATE_NAMES = {SCANNING:"SCANNING", APPROACHING:"APPROACHING", HOVERING:"HOVERING"}

# ================= DETECTION ROUGE =================
def detect_red_object(camera):
    width  = camera.getWidth()
    height = camera.getHeight()

    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((height, width, 4))

    b = img[:, :, 0].astype(np.int16)
    g = img[:, :, 1].astype(np.int16)
    r = img[:, :, 2].astype(np.int16)

    mask = (r > 100) & (r > 2 * g) & (r > 2 * b)

    area  = int(np.sum(mask))
    ratio = area / (width * height)

    if area == 0:
        return False, 0, 0, 0.0

    ys, xs = np.where(mask)
    cx = float(np.mean(xs))
    cy = float(np.mean(ys))

    # detected = True seulement si ratio dépasse le seuil minimum
    detected = (ratio > TARGET_RATIO_MIN)
    return detected, cx, cy, ratio

# ================= INIT =================
robot    = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera");        camera.enable(timestep)
imu    = robot.getDevice("inertial unit"); imu.enable(timestep)
gps    = robot.getDevice("gps");           gps.enable(timestep)
gyro   = robot.getDevice("gyro");          gyro.enable(timestep)
keyboard = robot.getKeyboard();            keyboard.enable(timestep)

front_left_led  = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")
camera_roll_motor  = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")

fl = robot.getDevice("front left propeller")
fr = robot.getDevice("front right propeller")
rl = robot.getDevice("rear left propeller")
rr = robot.getDevice("rear right propeller")

for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break

print("Drone prêt")
print("Flèches = manuel | relâcher = AUTO")
print("Seuils : MIN=%.2f | CLOSE=%.2f" % (TARGET_RATIO_MIN, TARGET_RATIO_CLOSE))

target_altitude = gps.getValues()[2] + 1.0
state = SCANNING

# ================= BOUCLE =================
while robot.step(timestep) != -1:

    time = robot.getTime()

    roll           = imu.getRollPitchYaw()[0]
    pitch          = imu.getRollPitchYaw()[1]
    altitude       = gps.getValues()[2]
    roll_velocity  = gyro.getValues()[0]
    pitch_velocity = gyro.getValues()[1]

    led_state = int(time) % 2
    front_left_led.set(led_state)
    front_right_led.set(1 - led_state)

    camera_roll_motor.setPosition(-0.115 * roll_velocity)
    camera_pitch_motor.setPosition(-0.1 * pitch_velocity)

    detected, cx, cy, ratio = detect_red_object(camera)

    width  = camera.getWidth()
    height = camera.getHeight()
    error_x = cx - width  / 2.0
    error_y = cy - height / 2.0

    # ── Clavier ──────────────────────────────────────────────
    roll_disturbance  = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance   = 0.0
    key_pressed       = False

    key = keyboard.getKey()
    while key > 0:
        if key == keyboard.UP:
            pitch_disturbance = -MANUAL_PITCH_SPEED; key_pressed = True
        elif key == keyboard.DOWN:
            pitch_disturbance =  MANUAL_PITCH_SPEED; key_pressed = True
        elif key == keyboard.RIGHT:
            yaw_disturbance   = -MANUAL_YAW_SPEED;  key_pressed = True
        elif key == keyboard.LEFT:
            yaw_disturbance   =  MANUAL_YAW_SPEED;  key_pressed = True
        elif key == keyboard.SHIFT + keyboard.RIGHT:
            roll_disturbance  = -MANUAL_ROLL_SPEED; key_pressed = True
        elif key == keyboard.SHIFT + keyboard.LEFT:
            roll_disturbance  =  MANUAL_ROLL_SPEED; key_pressed = True
        elif key == keyboard.SHIFT + keyboard.UP:
            target_altitude += 0.05
        elif key == keyboard.SHIFT + keyboard.DOWN:
            target_altitude -= 0.05
        key = keyboard.getKey()

    roll_disturbance  = CLAMP(roll_disturbance,  -1.5, 1.5)
    pitch_disturbance = CLAMP(pitch_disturbance, -1.5, 1.5)

    # ── Machine à états (seulement si pas de touche) ─────────
    if not key_pressed:

        if state == SCANNING:
            yaw_disturbance   = SCAN_YAW_SPEED
            pitch_disturbance = 0.0
            roll_disturbance  = 0.0
            # Transition seulement si ratio > MIN (vraie détection)
            if detected:
                print("[SCANNING -> APPROACHING] ratio=%.3f" % ratio)
                state = APPROACHING

        elif state == APPROACHING:
            if not detected:
                print("[APPROACHING -> SCANNING] cible perdue")
                state = SCANNING
            else:
                yaw_disturbance   = -MANUAL_YAW_SPEED * (error_x / (width  / 2.0))
                pitch_disturbance = APPROACH_PITCH
                roll_disturbance  = 0.0
                # Transition seulement si ratio > CLOSE (vraiment proche)
                if ratio > TARGET_RATIO_CLOSE:
                    print("[APPROACHING -> HOVERING] ratio=%.3f" % ratio)
                    state = HOVERING

        elif state == HOVERING:
            if not detected:
                print("[HOVERING -> SCANNING] cible perdue")
                state = SCANNING
            else:
                roll_disturbance  =  MANUAL_ROLL_SPEED  * (error_x / (width  / 2.0))
                pitch_disturbance =  MANUAL_PITCH_SPEED * (error_y / (height / 2.0))
                yaw_disturbance   = 0.0

    # ── PID ──────────────────────────────────────────────────
    roll_input  = K_ROLL_P  * CLAMP(roll,  -0.5, 0.5) + roll_velocity  + roll_disturbance
    pitch_input = K_PITCH_P * CLAMP(pitch, -0.5, 0.5) + pitch_velocity + pitch_disturbance
    yaw_input   = yaw_disturbance

    diff = CLAMP(target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
    vertical_input = K_VERTICAL_P * (diff ** 3)

    # ── Moteurs ──────────────────────────────────────────────
    fl.setVelocity(  K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input)
    fr.setVelocity(-(K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input))
    rl.setVelocity(-(K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input))
    rr.setVelocity(  K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input)

    # ── Debug ─────────────────────────────────────────────────
    mode_str = "MANUEL" if key_pressed else STATE_NAMES[state]
    print("[%s] alt=%.2f | ratio=%.4f | détecté=%s" % (mode_str, altitude, ratio, detected))