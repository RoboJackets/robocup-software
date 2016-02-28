from math import *

rad_s_to_rpm = 60.0 / (2 * pi)

# g*cm^2 -> kg*m^2
# g*cm^2/1000/10000 = kg*m^2


class MotorModel:
    def __init__(self):
        self.vbatt = 12.483  # Battery voltage
        self.kr = 379  # Speed constant, rpm/V
        self.kt = .0255  # Torque constant, Nm/A
        self.sample_time = 0.005  # Time per step, s
        self.oversample = 50  # Fine steps per sample time
        self.speed = 0  # Motor speed, rad/s
        self.rbatt = 0.39  # Battery internal resistance, ohms
        self.rmot = 1.2  # Winding resistance, phase-to-phase, ohms
        self.angle = 0  # Cumulative rotor angle, rad
        self.load = 120 / 1e7  # Load inertia, kg*m^2
        self.kosc = 2.7 / 1e7  # Oscillation amplitude coefficient, Nm/(m/s)^2
        self.cmd = 0  # FPGA command, 0..127
        self.ticks_per_rev = 1440  # Encoder ticks per motor revolution
        self.friction = 0  # Constant (kinetic) friction, Nm
        self.enc_delta = 0  # Encoder count change since last sample
        self.cmd_scale = 127.0  # Command value for 100% duty cycle

        # Outputs from each step
        self.torque = 0  # Total torque, Nm
        self.accel = 0  # Angular acceleration, rad/s^2

    def step(self):
        dtime = self.sample_time / self.oversample
        last_angle = self.angle
        for i in range(self.oversample):
            self.fine_step(dtime)

        self.enc_delta = (self.angle - last_angle) / (2 *
                                                      pi) * self.ticks_per_rev

    def fine_step(self, dtime):
        # PWM duty cycle
        self.pwm_duty = self.cmd / self.cmd_scale

        # Back-EMF
        self.vmot = self.speed / (self.kr / rad_s_to_rpm)

        # Winding current
        #FIXME - This use of rbatt may not work with PWM
        self.imot = (self.vbatt * self.pwm_duty - self.vmot) / (
            self.rmot + self.rbatt)

        # Measured supply voltage
        self.vs = self.vbatt - self.imot * self.rbatt

        # Torque exerted by the motor
        self.torque = self.kt * self.imot

        if False:
            # Friction
            if self.speed == 0:
                #FIXME - Static friction
                pass
            # Kinetic friction
            elif self.speed > 0:
                self.torque -= self.friction
            else:
                self.torque += self.friction
        self.torque -= self.friction

        # Wobbling shaft/rotor
        self.torque += self.speed * self.speed * self.kosc * sin(self.angle)

        self.accel = self.torque / self.load

        # Update motor state
        self.angle += self.speed * dtime + 0.5 * self.accel * dtime * dtime
        self.speed += self.accel * dtime
