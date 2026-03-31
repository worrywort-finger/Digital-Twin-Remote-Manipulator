import cv2
import mediapipe as mp
import socket
import math 
import time
from collections import deque # --- NEW: High-speed sliding window for the graph ---

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

mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.7)

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
prev_sensitivity = 1.0 

# Predictive Kinematics Variables
is_predicting = False      
PREDICTION_FACTOR = 1.5    
prev_tracker_x = 0.5
prev_tracker_y = 0.5

# --- NEW: Telemetry Oscilloscope Variables ---
GRAPH_WIDTH = 150 # Number of frames to display on the graph
raw_history = deque(maxlen=GRAPH_WIDTH)
smooth_history = deque(maxlen=GRAPH_WIDTH)
pred_history = deque(maxlen=GRAPH_WIDTH)

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
print(" [ a ] Key    : Toggle Autonomous Sentry Mode") 
print(" [ k ] Key    : Toggle Predictive Kinematics") 
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
    display_text_override = "" 

    # ---------------------------------------------------------
    # BRANCH A: HUMAN TELEOPERATION
    # ---------------------------------------------------------
    if system_mode in ["NORMAL", "RECORDING"]:
        results = hands.process(img_rgb)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                
                lm = hand_landmarks.landmark
                
                # Gestures
                fingers_curled = (lm[8].y > lm[6].y and lm[12].y > lm[10].y and lm[16].y > lm[14].y and lm[20].y > lm[18].y)
                fingers_extended = (lm[8].y < lm[6].y and lm[12].y < lm[10].y and lm[16].y < lm[14].y and lm[20].y < lm[18].y)

                index_up = lm[8].y < lm[6].y
                middle_up = lm[12].y < lm[10].y
                ring_down = lm[16].y > lm[14].y
                pinky_down = lm[20].y > lm[18].y
                peace_sign = index_up and middle_up and ring_down and pinky_down

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
                    
                    # Flatten graph during homing
                    raw_history.append(0.0)
                    smooth_history.append(0.0)
                    pred_history.append(0.0)

                elif fingers_curled: 
                    is_tracking = False
                elif fingers_extended: 
                    is_tracking = True
                
                # THE KINEMATIC ENGINE
                if is_tracking and not peace_sign:
                    tracker_node = lm[9]
                    
                    curr_x = tracker_node.x
                    curr_y = tracker_node.y
                    
                    if is_predicting:
                        vel_x = curr_x - prev_tracker_x
                        vel_y = curr_y - prev_tracker_y
                        
                        target_x = curr_x + (vel_x * PREDICTION_FACTOR)
                        target_y = curr_y + (vel_y * PREDICTION_FACTOR)
                        
                        target_x = max(0.0, min(target_x, 1.0))
                        target_y = max(0.0, min(target_y, 1.0))
                        
                        cv2.circle(img, (int(target_x * img.shape[1]), int(target_y * img.shape[0])), 8, (0, 255, 0), cv2.FILLED)
                    else:
                        target_x = curr_x
                        target_y = curr_y
                    
                    prev_tracker_x = curr_x
                    prev_tracker_y = curr_y

                    hand_size = math.hypot(lm[9].x - lm[0].x, lm[9].y - lm[0].y)
                    hand_size = max(0.15, min(hand_size, 0.45))
                    target_sensitivity = map_range(hand_size, 0.15, 0.45, 0.3, 1.8)
                    current_sensitivity = (0.1 * target_sensitivity) + (0.9 * prev_sensitivity)
                    prev_sensitivity = current_sensitivity
                    
                    dx = target_x - 0.5
                    dy = target_y - 0.5
                    
                    scaled_x = 0.5 + (dx * current_sensitivity)
                    scaled_y = 0.5 + (dy * current_sensitivity)

                    raw_pan = map_range(scaled_x, 0.0, 1.0, -60.0, 60.0)
                    raw_tilt = map_range(scaled_y, 0.0, 1.0, 60.0, -60.0)
                    
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

                    # --- NEW: Append Data to Graph History ---
                    # Calculate true raw angle (without prediction) for the red line
                    true_raw_dx = curr_x - 0.5
                    true_raw_x = 0.5 + (true_raw_dx * current_sensitivity)
                    true_raw_pan = map_range(true_raw_x, 0.0, 1.0, -60.0, 60.0)

                    raw_history.append(true_raw_pan)
                    smooth_history.append(smoothed_pan)
                    pred_history.append(raw_pan) # If predicting, raw_pan holds the projected vector

                if is_pinching and not was_pinched and is_tracking:
                    with open("waypoints_log.txt", "a") as file:
                        file.write(f"Waypoint Logged -> Pan: {servo_pan}, Tilt: {servo_tilt}\n")
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
            
            # Feed playback values to the graph
            raw_history.append(saved_pan)
            smooth_history.append(saved_pan)
            pred_history.append(saved_pan)
            
            playback_index += 1
        else:
            system_mode = "NORMAL"

    elif system_mode == "AUTONOMOUS":
        results_face = face_detection.process(img_rgb)
        if results_face.detections:
            detection = results_face.detections[0]
            bboxC = detection.location_data.relative_bounding_box
            face_x = bboxC.xmin + (bboxC.width / 2)
            face_y = bboxC.ymin + (bboxC.height / 2)
            
            raw_pan = map_range(face_x, 0.0, 1.0, -60.0, 60.0)
            raw_tilt = map_range(face_y, 0.0, 1.0, 60.0, -60.0)
            
            smoothed_pan = (SMOOTHING_ALPHA * raw_pan) + ((1.0 - SMOOTHING_ALPHA) * prev_pan_angle)
            smoothed_tilt = (SMOOTHING_ALPHA * raw_tilt) + ((1.0 - SMOOTHING_ALPHA) * prev_tilt_angle)
            
            prev_pan_angle = smoothed_pan
            prev_tilt_angle = smoothed_tilt
            
            servo_pan = int(smoothed_pan + 90)
            servo_tilt = int(smoothed_tilt + 90)
            
            sock.sendto(f"{smoothed_pan},{smoothed_tilt}".encode(), (BLENDER_IP, BLENDER_PORT))
            sock.sendto(f"P:{servo_pan},T:{servo_tilt}".encode(), (ESP32_IP, ESP32_PORT))
            
            # Feed face values to graph
            raw_history.append(raw_pan)
            smooth_history.append(smoothed_pan)
            pred_history.append(raw_pan)

            ih, iw, _ = img.shape
            x, y, w, h = int(bboxC.xmin * iw), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2) 
            cv2.line(img, (x + int(w/2), y), (x + int(w/2), y + h), (0, 0, 255), 1) 
            cv2.line(img, (x, y + int(h/2)), (x + w, y + int(h/2)), (0, 0, 255), 1)

    # ==========================================
    # 5. VISUAL HUD & KEYBOARD HOOKS
    # ==========================================
    cv2.rectangle(img, (0, 0), (640, 100), (0, 0, 0), cv2.FILLED) 

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
    elif system_mode == "AUTONOMOUS":
        cv2.putText(img, "STATE: AUTONOMOUS TARGET LOCK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if system_mode != "AUTONOMOUS":
        gear_text = f"SENSITIVITY: {prev_sensitivity:.2f}x"
        cv2.putText(img, gear_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        pred_text = "PREDICTION: ACTIVE (AHEAD OF HAND)" if is_predicting else "PREDICTION: OFF (LAGGING)"
        pred_color = (0, 255, 0) if is_predicting else (0, 0, 255)
        cv2.putText(img, pred_text, (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, pred_color, 2)
    else:
        cv2.putText(img, "SENSITIVITY & PREDICTION: OVERRIDDEN", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # --- NEW: Draw Live Telemetry Oscilloscope ---
    ih, iw, _ = img.shape
    gh = 100 # Graph height
    gw = GRAPH_WIDTH
    gx = iw - gw - 10 # Bottom right X
    gy = ih - gh - 10 # Bottom right Y

    # Draw Box and Centerline
    cv2.rectangle(img, (gx, gy), (gx + gw, gy + gh), (0, 0, 0), cv2.FILLED)
    cv2.line(img, (gx, gy + int(gh/2)), (gx + gw, gy + int(gh/2)), (50, 50, 50), 1)

    # Draw Legend
    cv2.putText(img, "RAW", (gx + 5, gy + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
    cv2.putText(img, "SMOOTH", (gx + 40, gy + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
    cv2.putText(img, "PREDICT", (gx + 95, gy + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

    # Helper function to plot lines
    def plot_data(history, color):
        if len(history) > 1:
            for i in range(1, len(history)):
                # Map -60/60 degrees to pixel height
                y1 = int(map_range(history[i-1], -60.0, 60.0, gy + gh, gy))
                x1 = gx + i - 1
                y2 = int(map_range(history[i], -60.0, 60.0, gy + gh, gy))
                x2 = gx + i
                cv2.line(img, (x1, y1), (x2, y2), color, 1)

    # Render lines (OpenCV uses BGR colors)
    plot_data(raw_history, (0, 0, 255))   # Red
    plot_data(smooth_history, (255, 0, 0)) # Blue
    if is_predicting:
        plot_data(pred_history, (0, 255, 0))  # Green

    cv2.imshow("Digital Twin - Spatial Tracking", img)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('l'): 
        with open("waypoints_log.txt", "a") as file:
            try:
                file.write(f"Waypoint Logged (Manual) -> Pan: {servo_pan}, Tilt: {servo_tilt}\n")
                last_log_time = time.time()
            except NameError:
                pass
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
    elif key == ord('a'): 
        if system_mode in ["NORMAL", "AUTONOMOUS"]:
            system_mode = "AUTONOMOUS" if system_mode == "NORMAL" else "NORMAL"
    elif key == ord('k'): 
        is_predicting = not is_predicting

cap.release()
cv2.destroyAllWindows()