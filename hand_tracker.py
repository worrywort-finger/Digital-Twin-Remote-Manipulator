import cv2
import mediapipe as mp
import socket
import math 
import time

# ==========================================
# 1. NETWORK CONFIGURATION
# ==========================================
BLENDER_IP = "127.0.0.1"  
BLENDER_PORT = 9000       

ESP32_IP = "10.213.215.221" 
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ==========================================
# 2. MEDIAPIPE INITIALIZATION
# ==========================================
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(1)

# ==========================================
# 3. STATE MACHINE & KINEMATIC MEMORY
# ==========================================
is_tracking = True
system_mode = "NORMAL"  

memory_buffer = []      
playback_index = 0      

# Kinematic Smoothing (EMA Filter)
SMOOTHING_ALPHA = 0.1  
prev_pan_angle = 0.0
prev_tilt_angle = 0.0
prev_sensitivity = 1.0 # --- NEW: Smooths the Z-axis depth changes ---

# Expanded State Machine Variables
was_pinched = False
last_log_time = 0.0
display_text_override = ""

def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

print("Digital Twin System Online.")
print("CONTROLS:")
print(" [Open Hand]  : Active Tracking")
print(" [Fist]       : Pause Tracking")
print(" [Peace Sign] : Snap to Center (Homing)")
print(" [OK Sign]    : Log Waypoint to File")
print(" [ r ] Key    : Toggle Recording")
print(" [ p ] Key    : Trigger Playback")
print(" [ l ] Key    : Manual Hardware Log")
print(" [ q ] Key    : Quit")

# ==========================================
# 4. MAIN INFERENCE LOOP
# ==========================================
while True:
    success, img = cap.read()
    if not success: break
        
    img = cv2.flip(img, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    display_text_override = "" 

    if system_mode in ["NORMAL", "RECORDING"]:
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                
                lm = hand_landmarks.landmark
                
                # --- GESTURE 1: The Safety Clutch (Open/Fist) ---
                fingers_curled = (lm[8].y > lm[6].y and lm[12].y > lm[10].y and lm[16].y > lm[14].y and lm[20].y > lm[18].y)
                fingers_extended = (lm[8].y < lm[6].y and lm[12].y < lm[10].y and lm[16].y < lm[14].y and lm[20].y < lm[18].y)

                # --- GESTURE 2: Homing (Peace Sign) ---
                index_up = lm[8].y < lm[6].y
                middle_up = lm[12].y < lm[10].y
                ring_down = lm[16].y > lm[14].y
                pinky_down = lm[20].y > lm[18].y
                peace_sign = index_up and middle_up and ring_down and pinky_down

                # --- GESTURE 3: Waypoint Logging ("OK" Sign) ---
                pinch_dist = math.hypot(lm[4].x - lm[8].x, lm[4].y - lm[8].y)
                ring_up = lm[16].y < lm[14].y
                pinky_up = lm[20].y < lm[18].y
                is_pinching = (pinch_dist < 0.05) and middle_up and ring_up and pinky_up

                if peace_sign:
                    is_tracking = False
                    display_text_override = "HOMING (PEACE SIGN)"
                    smoothed_pan, smoothed_tilt = 0.0, 0.0
                    prev_pan_angle, prev_tilt_angle = 0.0, 0.0
                    servo_pan, servo_tilt = 90, 90
                    
                    sock.sendto(f"{smoothed_pan},{smoothed_tilt}".encode(), (BLENDER_IP, BLENDER_PORT))
                    sock.sendto(f"P:{servo_pan},T:{servo_tilt}".encode(), (ESP32_IP, ESP32_PORT))

                elif fingers_curled: 
                    is_tracking = False
                elif fingers_extended: 
                    is_tracking = True
                
                # ==========================================
                # THE KINEMATIC ENGINE (Tracking & Z-Axis)
                # ==========================================
                if is_tracking and not peace_sign:
                    tracker_node = lm[9]
                    
                    # --- NEW: Z-Axis Depth Calculation ---
                    # Measure distance from Wrist (Node 0) to Middle Knuckle (Node 9)
                    hand_size = math.hypot(lm[9].x - lm[0].x, lm[9].y - lm[0].y)
                    
                    # Clamp the size to prevent extreme glitching (usually between 0.15 and 0.45)
                    hand_size = max(0.15, min(hand_size, 0.45))
                    
                    # Map physical size to a Sensitivity Multiplier (0.3x to 1.8x speed)
                    target_sensitivity = map_range(hand_size, 0.15, 0.45, 0.3, 1.8)
                    
                    # Apply EMA Smoothing to the Z-axis so the gears don't shift too violently
                    current_sensitivity = (0.1 * target_sensitivity) + (0.9 * prev_sensitivity)
                    prev_sensitivity = current_sensitivity
                    
                    # --- NEW: Center-Relative Scaling ---
                    # Calculate how far the hand is from the absolute center of the screen (0.5, 0.5)
                    dx = tracker_node.x - 0.5
                    dy = tracker_node.y - 0.5
                    
                    # Multiply that distance by our depth gear, then add it back to the center
                    scaled_x = 0.5 + (dx * current_sensitivity)
                    scaled_y = 0.5 + (dy * current_sensitivity)

                    # Map the scaled coordinates to the servo angles
                    raw_pan = map_range(scaled_x, 0.0, 1.0, -60.0, 60.0)
                    raw_tilt = map_range(scaled_y, 0.0, 1.0, 60.0, -60.0)
                    
                    # Apply standard X/Y EMA smoothing
                    smoothed_pan = (SMOOTHING_ALPHA * raw_pan) + ((1.0 - SMOOTHING_ALPHA) * prev_pan_angle)
                    smoothed_tilt = (SMOOTHING_ALPHA * raw_tilt) + ((1.0 - SMOOTHING_ALPHA) * prev_tilt_angle)
                    
                    prev_pan_angle = smoothed_pan
                    prev_tilt_angle = smoothed_tilt
                    
                    servo_pan = int(smoothed_pan + 90)
                    servo_tilt = int(smoothed_tilt + 90)
                    
                    sock.sendto(f"{smoothed_pan},{smoothed_tilt}".encode(), (BLENDER_IP, BLENDER_PORT))
                    sock.sendto(f"P:{servo_pan},T:{servo_tilt}".encode(), (ESP32_IP, ESP32_PORT))
                    
                    if system_mode == "RECORDING":
                        memory_buffer.append((smoothed_pan, smoothed_tilt, servo_pan, servo_tilt))

                # Pinch Logging Routine (OK Sign)
                if is_pinching and not was_pinched and is_tracking:
                    with open("waypoints_log.txt", "a") as file:
                        file.write(f"Waypoint Logged -> Pan: {servo_pan}, Tilt: {servo_tilt}\n")
                    print(f"[DATA] Waypoint Logged: P:{servo_pan}, T:{servo_tilt}")
                    last_log_time = time.time()
                    was_pinched = True
                elif not is_pinching:
                    was_pinched = False
                
                if time.time() - last_log_time < 0.5:
                    cv2.circle(img, (int(lm[8].x * img.shape[1]), int(lm[8].y * img.shape[0])), 20, (0, 255, 255), cv2.FILLED)
                    display_text_override = "WAYPOINT SAVED"

                mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    elif system_mode == "PLAYBACK":
        if playback_index < len(memory_buffer):
            saved_pan, saved_tilt, saved_span, saved_stilt = memory_buffer[playback_index]
            sock.sendto(f"{saved_pan},{saved_tilt}".encode(), (BLENDER_IP, BLENDER_PORT))
            sock.sendto(f"P:{saved_span},T:{saved_stilt}".encode(), (ESP32_IP, ESP32_PORT))
            playback_index += 1
        else:
            system_mode = "NORMAL"

    # ==========================================
    # 5. VISUAL HUD & KEYBOARD HOOKS
    # ==========================================
    cv2.rectangle(img, (0, 0), (640, 80), (0, 0, 0), cv2.FILLED) # Made the black box slightly taller

    if display_text_override != "":
        cv2.putText(img, f"STATE: {display_text_override}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
    elif system_mode == "NORMAL":
        status_color = (0, 255, 0) if is_tracking else (0, 0, 255)
        status_text = "ACTIVE (OPEN HAND)" if is_tracking else "PAUSED (FIST)"
        cv2.putText(img, f"STATE: {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
    elif system_mode == "RECORDING":
        cv2.putText(img, f"RECORDING... [{len(memory_buffer)}]", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        cv2.circle(img, (600, 20), 10, (0, 0, 255), cv2.FILLED) 
    elif system_mode == "PLAYBACK":
        cv2.putText(img, f"PLAYBACK [{playback_index}/{len(memory_buffer)}]", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    # --- NEW: Draw the Z-Axis Gear Ratio on the HUD ---
    gear_text = f"SENSITIVITY GEAR: {prev_sensitivity:.2f}x"
    if prev_sensitivity < 0.7:
        gear_text += " (MICRO-PRECISION)"
        gear_color = (255, 255, 0) # Cyan
    elif prev_sensitivity > 1.3:
        gear_text += " (HIGH-SPEED)"
        gear_color = (0, 165, 255) # Orange
    else:
        gear_text += " (NORMAL)"
        gear_color = (255, 255, 255) # White
        
    cv2.putText(img, gear_text, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, gear_color, 2)

    cv2.imshow("Digital Twin - Spatial Tracking", img)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('l'): 
        with open("waypoints_log.txt", "a") as file:
            # We use a try block in case the system is paused and servo variables aren't initialized
            try:
                file.write(f"Waypoint Logged (Manual) -> Pan: {servo_pan}, Tilt: {servo_tilt}\n")
                print(f"[DATA] Manual Waypoint Logged: P:{servo_pan}, T:{servo_tilt}")
                display_text_override = "MANUAL WAYPOINT SAVED"
                last_log_time = time.time()
            except NameError:
                print("[ERROR] Cannot log waypoint while system is paused.")
    elif key == ord('q'): break
    elif key == ord('r'): 
        if system_mode == "NORMAL":
            system_mode = "RECORDING"
            memory_buffer.clear() 
        elif system_mode == "RECORDING":
            system_mode = "NORMAL"
    elif key == ord('p'): 
        if system_mode == "NORMAL" and len(memory_buffer) > 0:
            system_mode = "PLAYBACK"
            playback_index = 0
        elif system_mode == "PLAYBACK":
            system_mode = "NORMAL" 

cap.release()
cv2.destroyAllWindows()