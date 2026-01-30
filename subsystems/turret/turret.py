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
from subsystems.turret.io import AimingIO
from math import *
from wpimath.geometry import Pose2d
from wpilib import DriverStation
from robot_state import RobotState


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

        self.velocityRequest = DutyCycleOut(0)

        self.independentAngle = 0
        self.goal = ""

    def periodic(self):

        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)

        # Log inputs to PyKit
        Logger.processInputs("Turret", self._inputs)

        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

        self.currentAngle = RobotState.get_current_pose().rotation + self.independentAngle

        if self.goal:
            self.rotateTowardsGoal(self.goal)

    """
    # SMELLY CODE THAT IT TURNS OUT WE ACTUALLY NEVER NEEDED

    def getAngleToHub(self, distance, angle, in_radians = True):
        if not in_radians:
            angle = radians(angle)
        horizontal_distance = distance * sin(degrees(angle))
        vertical_distance_to_tag = distance * cos(degrees(angle))
        vertical_distance_to_hub = vertical_distance_to_tag + Constants.AimingConstants.APRILTAGTOHUBCENTRE
        proposed_angle = degrees(atan(horizontal_distance / vertical_distance_to_hub))
        return proposed_angle
    
    def getAngleToPassGoal(self, distance, angle, in_radians = True):
        if not in_radians:
            angle = radians(angle)
    """
        
    def getAngleToGoal(self):
        # If the robot position is in the alliance side, call getANgleToHub before aiming
        # If the robot is in the neutral zone, have it determine what side of the zone it's on so it knows the target to aim at
        match self.goal.lower():
            case "hub":
                xdist = abs(RobotState.get_current_pose().X - Constants.GoalLocations.BLUEHUB.X) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(RobotState.get_current_pose().X - Constants.GoalLocations.REDHUB.X)
                ydist = abs(RobotState.get_current_pose().Y - Constants.GoalLocations.BLUEHUB.Y) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(RobotState.get_current_pose().Y - Constants.GoalLocations.REDHUB.Y)
            case "outpost":
                xdist = abs(RobotState.get_current_pose().X - Constants.GoalLocations.BLUEOUTPOSTPASS.X) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(RobotState.get_current_pose().X - Constants.GoalLocations.REDOUTPOSTPASS.X)
                ydist = abs(RobotState.get_current_pose().Y - Constants.GoalLocations.BLUEOUTPOSTPASS.Y) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(RobotState.get_current_pose().Y - Constants.GoalLocations.REDOUTPOSTPASS.Y)
            case "depot":
                xdist = abs(RobotState.get_current_pose().X - Constants.GoalLocations.BLUEDEPOTPASS.X) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(RobotState.get_current_pose().X - Constants.GoalLocations.REDDEPOTPASS.X)
                ydist = abs(RobotState.get_current_pose().Y - Constants.GoalLocations.BLUEDEPOTPASS.Y) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(RobotState.get_current_pose().Y - Constants.GoalLocations.REDDEPOTPASS.Y)
            case "_":
                raise TypeError("Turret aiming target must be \"hub\", \"outpost\", or \"depot\"")
        target_angle = atan(ydist / xdist)
        return target_angle

    def rotateTowardsGoal(self):
        # This function might not work because it probably isn't periodic so it'll only set the output once and then not check if the angle is correct until it's called again (which is when the target changes)
        targetAngle = self.getAngleToGoal()

        if not (self.currentAngle >= targetAngle - Constants.TurretConstants.ANGLEDEADBAND and self.currentAngle <= targetAngle + Constants.TurretConstants.ANGLEDEADBAND):
            self.velocityRequest.output = -1 if self.currentAngle > targetAngle else 1
        else:
            self.velocityRequest.output = 0

"""
WORK LEFT TO DO

Debug rotateTowardsGoal to be periodically checking
Add logic for hardstop
Make sure that rotation actually works (IndependentAngle is unlimited, but robotAngle may or may not be (in other words, it may limit from 0 to 360 degrees rather than just counting total rotation))
Use CANcoder to figure out the independent rotation of the turret
"""