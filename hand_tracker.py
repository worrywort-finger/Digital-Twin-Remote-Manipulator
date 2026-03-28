import cv2
import mediapipe as mp
import socket


#NETWORK CONFIGURATION

BLENDER_IP = "127.0.0.1"  # Localhost
BLENDER_PORT = 9000       # Must match the Blender listening port

#ESP32 address 
ESP32_IP = "10.134.160.221"  #change this later after configuring the real address
ESP32_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


#MEDIAPIPE INITIALIZATION

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Initialize the primary webcam
cap = cv2.VideoCapture(1)


#KINEMATIC MAPPING FUNCTION
is_tracking = True
def map_range(value, in_min, in_max, out_min, out_max):
    """Proportionally scales a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

print("Initializing Vision System. Press 'q' in the video window to quit.")


#MAIN INFERENCE LOOP

while True:
    success, img = cap.read()
    if not success:
        print("Error: Failed to capture video feed.")
        break
        
    # Flip the image horizontally for an intuitive "selfie" mirror view
    img = cv2.flip(img, 1)
    
    # MediaPipe requires RGB color space, but OpenCV captures in BGR
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            
            #GESTURE DETECTION LOGIC
            # Check if fingertips (8, 12, 16, 20) are below their lower joints (6, 10, 14, 18)
            fingers_curled = (
                hand_landmarks.landmark[8].y > hand_landmarks.landmark[6].y and 
                hand_landmarks.landmark[12].y > hand_landmarks.landmark[10].y and 
                hand_landmarks.landmark[16].y > hand_landmarks.landmark[14].y and 
                hand_landmarks.landmark[20].y > hand_landmarks.landmark[18].y
            )
            
            # Check if all fingers are extended above their joints
            fingers_extended = (
                hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y and 
                hand_landmarks.landmark[12].y < hand_landmarks.landmark[10].y and 
                hand_landmarks.landmark[16].y < hand_landmarks.landmark[14].y and 
                hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y
            )

            # State Machine Toggle
            if fingers_curled:
                is_tracking = False
            elif fingers_extended:
                is_tracking = True
            
            if is_tracking:
                # Extract Landmark 9: Middle Finger MCP (Knuckle)
                tracker_node = hand_landmarks.landmark[9]
                
                # Calculate Pan (X-Axis): Left/Right
                # Camera X goes 0.0 (Left) to 1.0 (Right). We map this to -45 to 45 degrees.
                pan_angle = map_range(tracker_node.x, 0.0, 1.0, -60.0, 60.0)
                
                # Calculate Tilt (Y-Axis): Up/Down
                # Camera Y goes 0.0 (Top) to 1.0 (Bottom). 
                # We map 0.0 (Top) to 30 degrees, and 1.0 (Bottom) to -30 degrees to invert the axis naturally.
                tilt_angle = map_range(tracker_node.y, 0.0, 1.0, 60.0, -60.0)
                
                #Virtual Twin Payload
                # Format and Broadcast to Blender 
                blender_message = f"{pan_angle},{tilt_angle}"
                sock.sendto(blender_message.encode(), (BLENDER_IP, BLENDER_PORT))
                
                # Calculate Physical Servo Angles (0 to 180, centered at 90)
                # The negative sign (-) inverts the physical hardware axis to match the camera
                servo_pan = int((pan_angle) + 90)
                servo_tilt = int((tilt_angle) + 90)
                
                # Format explicitly so the C code knows which number is which (e.g., "P:90,T:45")
                esp32_message = f"P:{servo_pan},T:{servo_tilt}"
                sock.sendto(esp32_message.encode(), (ESP32_IP, ESP32_PORT))
                
                # Print the dual-cast status to the terminal for debugging
                print(f"Blender: {pan_angle:.1f}°, {tilt_angle:.1f}° | ESP32: {esp32_message}")
            
            
            # Draw the neural network mesh over the hand for visual debugging
            mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    if is_tracking:
        cv2.putText(img, "STATE: ACTIVE (OPEN HAND)", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(img, "STATE: PAUSED (FIST)", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
    # Display the live feed
    cv2.imshow("Digital Twin - Spatial Tracking", img)
    
    # Break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up hardware resources
cap.release()
cv2.destroyAllWindows()