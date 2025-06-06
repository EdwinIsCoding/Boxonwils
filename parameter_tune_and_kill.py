import socket

# IP address and port of Arduino Giga
UDP_IP = "192.168.0.113"
UDP_PORT = 55500

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_kill():
    """Function to send a kill message to stop the Arduino remotely via UDP."""

    message = "Stop"
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
    print(f"Kill message sent to {UDP_IP}:{UDP_PORT}")

def send_pid(mode):
    """Function to modify line-following and wall-following PID values. 
    This function allows the user to input PID values and send them to the Arduino via UDP.
    This allows for real-time tuning of the PID controllers."""

    # Add sending of Ka (alignment error constant) if mode is wall following
    mode_name = "Wall-Following" if mode == "WALL" else "Line-Following"
    print(f"Enter PID values for {mode_name}:")
    try:
        kp = float(input("  Enter P: "))
        ki = float(input("  Enter I: "))
        kd = float(input("  Enter D: "))
        if mode_name == 'Wall-Following':
            ka = float(input("  Enter A: "))
            message = f"{mode}_PID,{kp},{ki},{kd},{ka}"
            sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
            print(f"{mode_name} PID values sent: P={kp}, I={ki}, D={kd},A={ka}")
        else:
            message = f"{mode}_PID,{kp},{ki},{kd}"
            sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
            print(f"{mode_name} PID values sent: P={kp}, I={ki}, D={kd}")
    # 
    except ValueError:
        print("Invalid input. Please enter numerical values.")

def send_single_pid_param(controller, param):
    """Function to send a single PID parameter (P, I, or D) to the specified controller (Line or Wall)."""

    label = {'p': 'P', 'i': 'I', 'd': 'D'}[param]
    try:
        value = float(input(f"Enter new {label} value for {controller}-Following controller: "))
        message = f"{controller}SET{label},{value}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print(f"{label} value sent to {controller}-Following: {value}")
    except ValueError:
        print("Invalid input. Please enter a number.")

def send_base_speed():
    """Function to send a base speed value to the Arduino for controlling the robot's speed. The base speed is an integer between 0 and 800."""
    try:
        speed = int(input("Enter base speed (0–800): "))
        if 0 <= speed <= 800:
            message = f"BASE_SPEED,{speed}"
            sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
            print(f"Base speed sent: {speed}")
        else:
            print("Value out of range. Please enter between 0 and 800.")
    except ValueError:
        print("Invalid input. Please enter an integer.")

def send_turn_duration():
    """Sends a turn duration value to the Arduino for the predefined 90deg turns.
    The turn duration is an integer representing the time in milliseconds.
    This function simplifies fine-tuning the turn duration."""

    try:
        duration = int(input("Enter turn duration: "))
        message = f"TURN_DURATION,{duration}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print(f"Base speed sent: {duration}")
    except ValueError:
        print("Invalid input. Please enter an integer.")


def send_line_threshold():
    """Sends a line threshold value to the Arduino for line-following behaviour, allowing for binary representations of reflectance sensor readings."""
    
    try:
        linethreshold = int(input('Enter line threshold values: '))
        message = f"LINE_THRESHOLD,{linethreshold}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print(f"Line threshold sent: {linethreshold}")
    except ValueError:
        print("Invalid input. Please enter a number.")

def toggle_print_flags():
    """Function to toggle print flags on the Arduino for debugging purposes."""

    message = "TOGGLE_PRINT_FLAGS"
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
    print("Print flags toggled on Arduino.")

def send_modes(mode):
    """This function sends numbers representing modes. Each mode corresponds to a specific action with custom PID values and other parameters.
    This allows the user to quickly switch between different behaviors of the robot for rapid testing and tuning."""

    if mode == 1: # Straight line-following
        # Base speed = 400, line-following PID = 1, 0, 0.1
        message = "LINE_PID,1,0,0.1"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        message = "BASE_SPEED,400"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print("Mode 1: Base speed set to 400, Line-Following PID set to P=1, I=0, D=0.1")
    elif mode == 2: # Straight line-following with light turns
        # Base speed = 200, line-following PID = 2, 0, 0
        message = "LINE_PID,2,0,0"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        message = "BASE_SPEED,400"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print("Mode 2: Base speed set to 400, Line-Following PID set to P=2, I=0, D=0")
    elif mode == 3: # Line-following with sharp turns
        # Base speed = 250, line-following PID = 10, 0, 0
        message = "LINE_PID,10,0,0"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        message = "BASE_SPEED,250"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print("Mode 3: Base speed set to 250, Line-Following PID set to P=10, I=0, D=0")
    elif mode == 4: # Line-following with sharp turns 2
        # Base speed = 200, line-following PID = 10, 0, 0
        message = "LINE_PID,10,0,0"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        message = "BASE_SPEED,200"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print("Mode 4: Base speed set to 200, Line-Following PID set to P=10, I=0, D=0")
    elif mode == 5: # Line-following in the dark
        # Base speed = 300, line-following PID = 8, 0, 0
        message = "LINE_PID,8,0,0"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        message = "BASE_SPEED,300"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print("Mode 5: Base speed set to 300, Line-Following PID set to P=8, I=0, D=0")
    elif mode == 6: # Wall-following with PID values
        # Base speed = 400, wall-following PID = 100, 0, 5, 60
        message = "WALL_PID,100,0,5,60"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        message = "BASE_SPEED,350"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print("Mode 6: Base speed set to 350, Wall-Following PID set to P=100, I=0, D=5, A=60")


# User instructions
print("UDP Command Interface Running")
print("Press:")
print("  'k'  - Send Kill message")
print("  'w'  - Set Wall-Following PID values")
print("  'l'  - Set Line-Following PID values")
print("  'wp', 'wi', 'wd' - Set P/I/D of Wall-Following controller")
print("  'lp', 'li', 'ld' - Set P/I/D of Line-Following controller")
print("  'bs' - Set Base Speed (0–800)")
print("  'td' - Set Turn Duration (in milliseconds)")
print("  'lth' - Set Line Threshold")
print("  'tp' - Toggle Print Flags on Arduino")
print("  '1' to '6' - Set predefined modes with custom PID values")
print("  'q'  - Quit the program")

# Main loop to handle user input
while True:
    user_input = input("Enter command: ").strip().lower()

    if user_input == 'k':
        send_kill()
    elif user_input == 'w':
        send_pid("WALL")
    elif user_input == 'l':
        send_pid("LINE")
    elif user_input in ['wp', 'wi', 'wd']:
        send_single_pid_param("WALL", user_input[1])
    elif user_input in ['lp', 'li', 'ld']:
        send_single_pid_param("LINE", user_input[1])
    elif user_input == 'bs':
        send_base_speed()
    elif user_input == 'td':
        send_turn_duration()
    elif user_input == 'lth':
        send_line_threshold()
    elif user_input == 'tp':
        toggle_print_flags()
    elif user_input.isdigit():
        mode = int(user_input)
        if 1 <= mode <= 9:
            send_modes(mode)
        else:
            print("Invalid mode. Please enter a number between 1 and 6.")
    elif user_input == 'q':
        print("Exiting program.")
        break
    else:
        print("Unknown command. Please try again.")