from enum import auto, Enum

from commands2 import Command, cmd, PIDSubsystem
from phoenix6 import utils
from phoenix6.configs import CANrangeConfiguration, TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, HardwareLimitSwitchConfigs, ProximityParamsConfigs, CurrentLimitsConfigs
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX, CANrange
from phoenix6.signals import NeutralModeValue, ForwardLimitValue, ForwardLimitSourceValue

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from typing import Final
from constants import Constants
from subsystems import StateSubsystem
from subsystems.aiming.io import AimingIO
from math import *
from wpimath.geometry import Pose2d
from wpilib import DriverStation


"""
TO-DO:

 - Hardstops
    - If the proposed angle from the turret is 180 degrees or more from the turret's current position, have it rotate in the opposite direction instead
        - If the proposed angle is past the hardstop, rotate the robot instead
"""

# Using intake-subsystem.py as a reference
class TurretSubsytem(PIDSubsystem):
    """
    Responsible for aiming horizontally with the turret and vertically with the variable hood.
    """

    # On the reference there's something about a CANrange here and I don't know what that means so I'm leaving it.

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.AimingConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.AimingConstants.GEAR_RATIO))
                     .with_current_limits(CurrentLimitsConfigs().with_supply_current_limit_enable(True).with_supply_current_limit(Constants.AimingConstants.SUPPLY_CURRENT))
                     )
    
    def __init__(self, io: AimingIO):
        super.__init__() # Change PID controller and Initial position if needed

        self._io: Final[AimingIO] = io
        self._inputs = AimingIO.AimingIOInputs()

        self._motorDisconnectedAlert = Alert("Turret motor is disconnected.", Alert.AlertType.kError)

        self.currentPose = Pose2d() # Might break idk

    def periodic(self):

        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)

        # Log inputs to PyKit
        Logger.processInputs("Turret", self._inputs)

        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

    def getAngleToHub(self, distance, angle, in_radians = True):
        if not in_radians:
            angle = radians(angle)
        horizontal_distance = distance * sin(degrees(angle))
        vertical_distance_to_tag = distance * cos(degrees(angle))
        vertical_distance_to_hub = vertical_distance_to_tag + Constants.AimingConstants.APRILTAGTOHUBCENTRE
        proposed_angle = degrees(atan(horizontal_distance / vertical_distance_to_hub))
        return proposed_angle

    def aimTowardsGoal(self, goal):
        # If the robot position is in the alliance side, call getANgleToHub before aiming
        # If the robot is in the neutral zone, have it determine what side of the zone it's on so it knows the target to aim at
        if self.currentPose.x <= Constants.AimingConstants.REDALLIANCEBORDER and self.currentPose.x >= Constants.AimingConstants.BLUEALLIANCEBORDER:
            if self.currentPose.y > Constants.AimingConstants.VERTICALMIDLINE:
                print("Shoot to depot")
            else:
                print("Shoot to outpost")
        elif (DriverStation.getAlliance == DriverStation.Alliance.kBlue and self.currentPose.x < Constants.AimingConstants.BLUEALLIANCEBORDER) or (DriverStation.getAlliance == DriverStation.Alliance.kRed and self.currentPose.x > Constants.AimingConstants.REDALLIANCEBORDER):
            self.target_angle = self.getAngleToHub() # Figure out the values idk

        