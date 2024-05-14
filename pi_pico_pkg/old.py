import time
from machine import Pin, PWM, I2C
import uasyncio as asyncio
from micro_ros import RCLCNode, ROSString, ROSUInt8MultiArray, Publisher, Subscriber

# General constants
TIMER_COMPARE_VALUE_50HZ = 60000
TIMER_COMPARE_VALUE_1000HZ = 48000
TIMER_COMPARE_VALUE_PWM_EXP = 4096
LED_PIN = 25
HALOGEN_PIN = 22
PCA0_I2C_ADDRESS = 0x20
PCA1_I2C_ADDRESS = 0x21
PCA2_I2C_ADDRESS = 0x22

# Manipulation constants for command handling
CLAW_DEVICE = 33
SCREWDRIVER = 34
SOLENOID = 35
PUMP = 36
END_EFFECTOR = 37
ACTUATOR = 38
HALOGEN = 39

# Science constants for command handling
FRONT_SCOOP = 0
MIDDLE_SCOOP = 1
BACK_SCOOP = 2
SUBSURFACE_COLLECTOR = 3
PROBE_ACTUATOR = 4

class PCA9555:
    def __init__(self, address):
        self.address = address
        self.ports = [0] * 16
        self.config = 0xFF
        self.output = 0x00
        self.input = 0x00

PCA0 = PCA9555(PCA0_I2C_ADDRESS)
PCA1 = PCA9555(PCA1_I2C_ADDRESS)
PCA2 = PCA9555(PCA2_I2C_ADDRESS)

class ServoDev:
    def __init__(self, reverse, pin):
        self.reverse = reverse
        self.pin = pin
        self.pwm = PWM(Pin(pin))

SERVO1_L = ServoDev(False, 2)
SERVO2_L = ServoDev(False, 3)
SERVO3_L = ServoDev(False, 4)
SERVO4_L = ServoDev(False, 5)
SERVO5_L = ServoDev(False, 6)
SERVO6_L = ServoDev(False, 7)
SERVO7_L = ServoDev(True, 0)
SERVO8_L = ServoDev(True, 1)
SERVO9_L = ServoDev(True, 2)
SERVO10_L = ServoDev(True, 3)
SERVO11_L = ServoDev(True, 4)
SERVO12_L = ServoDev(True, 5)

# Helper function to drive a servo
def drive_servo(servo, duty_cycle):
    servo.pwm.duty_u16(int(duty_cycle * 65535))

# ROS2 node setup
node = RCLCNode('pico_node')

# Publisher setup
publisher = node.create_publisher(ROSString, 'pico_feedback')

# Command handler
def command_handler(msg):
    device = msg.data[0]
    id = msg.data[1]
    direction = msg.data[2]
    pwm_cycle = msg.data[3]
    duty_cycle = pwm_cycle / 100.0

    if device == CLAW_DEVICE:
        node.loginfo("Claw device pinged!")
        node.loginfo(f"Duty cycle is at {pwm_cycle}%")
        node.loginfo(f"Direction is {direction}")
        set_end_effector_direction(PCA0, direction)
        set_end_effector_speed(duty_cycle)
    
    elif device == SCREWDRIVER:
        node.loginfo("Screwdriver pinged!")
        node.loginfo(f"Duty cycle is at {pwm_cycle}%")
        set_screwdriver_direction(PCA0, direction)
        set_screwdriver_speed(duty_cycle)

    elif device == SOLENOID:
        node.loginfo("Poker device pinged!")
        node.loginfo("Poker pressed" if direction else "Poker retracted")
        solenoid_on_off(direction)

    elif device == PUMP:
        node.loginfo("Pump device pinged!")
        node.loginfo(f"Pump device id: {id}")
        if direction == 1:
            duty_cycle = 0.13
        elif direction == 2:
            duty_cycle = 0.015
        else:
            duty_cycle = 0
        drive_servo(globals()[f'SERVO{id+2}_L'], duty_cycle)

    elif device == ACTUATOR:
        node.loginfo("Actuator pinged!!!")
        node.loginfo(f"Actuator device id: {id}")
        if id == PROBE_ACTUATOR:
            duty_cycle = 0.05 + duty_cycle * 0.05
            node.loginfo("PROBING!!!")
            drive_servo(SERVO1_L, duty_cycle)
        elif id == SUBSURFACE_COLLECTOR:
            duty_cycle = 0.04 + duty_cycle * 0.055
            node.loginfo("DRILLING!!!")
            drive_servo(SERVO5_L, duty_cycle)
        elif id == FRONT_SCOOP:
            duty_cycle = 0.05 + duty_cycle * 0.05
            node.loginfo("Front scoop!!!")
            drive_servo(SERVO6_L, duty_cycle)
        elif id == MIDDLE_SCOOP:
            duty_cycle = 0.05 + duty_cycle * 0.05
            node.loginfo("Middle scoop!!!")
            drive_servo(SERVO11_L, duty_cycle)
        elif id == BACK_SCOOP:
            duty_cycle = 0.05 + duty_cycle * 0.05
            node.loginfo("Back scoop!!!")
            drive_servo(SERVO12_L, duty_cycle)

    elif device == HALOGEN:
        if id:
            node.loginfo("HALOGEN ON!!")
            Pin(HALOGEN_PIN, Pin.OUT).on()
        else:
            node.loginfo("HALOGEN OFF!!!")
            Pin(HALOGEN_PIN, Pin.OUT).off()

    else:
        node.loginfo("Unknown device pinged!")
        node.loginfo("No action taken...")

# Subscriber setup
subscriber = node.create_subscription(ROSUInt8MultiArray, 'pico_sub', command_handler)

async def main():
    gpio_init(LED_PIN, Pin.OUT)
    Pin(LED_PIN, Pin.OUT).on()
    
    while True:
        await asyncio.sleep(1)

node.init(main())



TIMER_MAX_VALUE_1000HZ = 48000 - 1
TIMER_MAX_VALUE_50HZ = 60000 - 1
PWM_EXPANDER_TIMER_MAX_VALUE = 4096 - 1

# Hypothetical library for PCA9555
class PCA9555:
    def __init__(self, i2c, address=0x20):
        self.i2c = i2c
        self.address = address

    def set_pin(self, pin):
        pass  # Replace with actual implementation

    def clear_pin(self, pin):
        pass  # Replace with actual implementation

def solenoid_init():
    pwm13 = machine.PWM(machine.Pin(13))
    pwm13.freq(1000)
    pwm13.duty_u16(0)

def solenoid_on_off(on_off):
    pwm13 = machine.PWM(machine.Pin(13))
    if on_off:
        pwm13.duty_u16(TIMER_MAX_VALUE_1000HZ)
    else:
        pwm13.duty_u16(0)

def screwdriver_init():
    pwm12 = machine.PWM(machine.Pin(12))
    pwm12.freq(1000)
    pwm12.duty_u16(0)

def set_screwdriver_direction(pca, dir):
    if dir:
        pca.set_pin(0)
    else:
        pca.clear_pin(0)

def set_screwdriver_speed(pwm):
    pwm12 = machine.PWM(machine.Pin(12))
    timer_compare_value = int(pwm * TIMER_MAX_VALUE_1000HZ)
    if timer_compare_value > TIMER_MAX_VALUE_1000HZ:
        pwm12.duty_u16(TIMER_MAX_VALUE_1000HZ)
    elif timer_compare_value > 0:
        pwm12.duty_u16(timer_compare_value)
    else:
        pwm12.duty_u16(0)

def end_effector_init():
    pwm14 = machine.PWM(machine.Pin(14))
    pwm14.freq(1000)
    pwm14.duty_u16(0)

def set_end_effector_direction(pca, dir):
    if dir:
        pca.set_pin(2)
    else:
        pca.clear_pin(2)

def set_end_effector_speed(pwm):
    pwm14 = machine.PWM(machine.Pin(14))
    timer_compare_value = int(pwm * TIMER_MAX_VALUE_1000HZ)
    if timer_compare_value > TIMER_MAX_VALUE_1000HZ:
        pwm14.duty_u16(TIMER_MAX_VALUE_1000HZ)
    elif timer_compare_value > 0:
        pwm14.duty_u16(timer_compare_value)
    else:
        pwm14.duty_u16(0)

class SERVO_DEV:
    def __init__(self, pin_num, i2c=False):
        self.pin_num = pin_num
        self.i2c = i2c

def servo_init(servo):
    if servo.i2c:
        return  # Handled by pwm_exp_init in original C code
    else:
        pwm = machine.PWM(machine.Pin(servo.pin_num))
        pwm.freq(50)
        pwm.duty_u16(0)

def drive_servo(servo, pwm):
    if servo.i2c:
        timer_compare_value = int(pwm * PWM_EXPANDER_TIMER_MAX_VALUE)
        channel_num = servo.pin_num
        if timer_compare_value > PWM_EXPANDER_TIMER_MAX_VALUE:
            pwm_exp_set_pwm(channel_num, PWM_EXPANDER_TIMER_MAX_VALUE)
        elif timer_compare_value > 0:
            pwm_exp_set_pwm(channel_num, timer_compare_value)
        else:
            pwm_exp_set_pwm(channel_num, 0)
    else:
        pwm_device = machine.PWM(machine.Pin(servo.pin_num))
        timer_compare_value = int(pwm * TIMER_MAX_VALUE_50HZ)
        if timer_compare_value > TIMER_MAX_VALUE_50HZ:
            pwm_device.duty_u16(TIMER_MAX_VALUE_50HZ)
        elif timer_compare_value > 0:
            pwm_device.duty_u16(timer_compare_value)
        else:
            pwm_device.duty_u16(0)

# Assuming pwm_exp_set_pwm is defined elsewhere for I2C-based PWM control
def pwm_exp_set_pwm(channel, value):
    pass  # Replace with actual implementation
