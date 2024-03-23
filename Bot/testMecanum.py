import RPi.GPIO as GPIO
import time

# Mecanum wheel motor pins
motor1_pins = (1, 2, 3)  # Motor 1 pins (e.g., IN1, IN2, ENA)
motor2_pins = (4, 5, 6)  # Motor 2 pins (e.g., IN3, IN4, ENB)
motor3_pins = (7, 8, 9)  # Motor 3 pins (e.g., IN5, IN6, ENC)
motor4_pins = (10, 11, 12)  # Motor 4 pins (e.g., IN7, IN8, END)

# Pan-tilt camera servo pins
pan_pin = 13
tilt_pin = 14

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(pan_pin, GPIO.OUT)
GPIO.setup(tilt_pin, GPIO.OUT)

# Set up PWM for pan and tilt servos
pan_pwm = GPIO.PWM(pan_pin, 50)  # 50 Hz frequency
tilt_pwm = GPIO.PWM(tilt_pin, 50)  # 50 Hz frequency

# Mecanum wheel setup
def setup_motors():
    for pins in (motor1_pins, motor2_pins, motor3_pins, motor4_pins):
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)

def set_motor_speed(pins, speed):
    GPIO.output(pins[2], GPIO.HIGH)  # Enable motor
    GPIO.output(pins[0], GPIO.LOW if speed >= 0 else GPIO.HIGH)
    GPIO.output(pins[1], GPIO.HIGH if speed >= 0 else GPIO.LOW)

    speed = min(abs(speed), 100)  # Limit speed to 0-100
    pwm = GPIO.PWM(pins[2], 1000)  # 1 kHz frequency
    pwm.start(speed)

# Pan-tilt camera control
def set_pan(angle):
    duty_cycle = (angle / 18) + 2
    pan_pwm.ChangeDutyCycle(duty_cycle)

def set_tilt(angle):
    duty_cycle = (angle / 18) + 2
    tilt_pwm.ChangeDutyCycle(duty_cycle)

# Example usage
if __name__ == "__main__":
    try:
        setup_motors()
        pan_pwm.start(0)
        tilt_pwm.start(0)

        while True:
            # Control mecanum wheels
            set_motor_speed(motor1_pins, 50)  # Set speed for motor 1
            set_motor_speed(motor2_pins, 50)  # Set speed for motor 2
            set_motor_speed(motor3_pins, 50)  # Set speed for motor 3
            set_motor_speed(motor4_pins, 50)  # Set speed for motor 4

            # Control pan-tilt camera
            set_pan(90)  # Set pan angle (0-180)
            set_tilt(45)  # Set tilt angle (0-180)

            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting program")
    finally:
        GPIO.cleanup()  # Clean up GPIO on exit
