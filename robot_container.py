import os

import commands2
import commands2.button
from commands2 import cmd, InstantCommand
from commands2.button import CommandXboxController, Trigger
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from phoenix6 import swerve, utils
from wpilib import DriverStation, SendableChooser, XboxController, SmartDashboard, getDeployDirectory
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.units import rotationsToRadians
from robot_config import currentRobot, has_subsystem  # Robot detection (Larry vs Comp)

from constants import Constants
from generated.tuner_constants import TunerConstants
from subsystems.climber import ClimberSubsystem
from subsystems.climber.io import ClimberIOTalonFX, ClimberIOSim
from subsystems.intake import IntakeSubsystem
from subsystems.superstructure import Superstructure
from subsystems.swerve import SwerveSubsystem
from subsystems.swerve.requests import DriverAssist
from subsystems.vision import VisionSubsystem
from typing import Optional
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs
from pykit.logger import Logger


class RobotContainer:
    def __init__(self) -> None:
        # Log which robot we're running on (for debugging)
        print(f"Initializing RobotContainer for: {currentRobot.name}")
        self._max_speed = TunerConstants.speed_at_12_volts
        self._max_angular_rate = rotationsToRadians(1)

        self._driver_controller = commands2.button.CommandXboxController(0)
        self._function_controller = commands2.button.CommandXboxController(1)

        
        # Initialize subsystems as None - will be created conditionally
        self.climber: Optional[ClimberSubsystem] = None
        self.intake: Optional[IntakeSubsystem] = None
        self.drivetrain: Optional[SwerveSubsystem] = None
        self.vision: Optional[VisionSubsystem] = None
        match Constants.currentMode:
            case Constants.Mode.REAL:
                # Real robot, instantiate hardware IO implementations
                if has_subsystem("drivetrain"):
                    self.drivetrain = TunerConstants.create_drivetrain()
                self.vision = VisionSubsystem(
                    self.drivetrain,
                    Constants.VisionConstants.FRONT,
                    Constants.VisionConstants.LAUNCHER,
                )

                # Create climber only if it exists on this robot
                if has_subsystem("climber"):
                    # Create climber motor config
                    # Note: Constants.ClimberConstants values are automatically selected based on detected robot
                    climber_motor_config = (TalonFXConfiguration()
                        .with_slot0(Constants.ClimberConstants.GAINS)
                        .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                        .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ClimberConstants.GEAR_RATIO))
                    )
                    
                    # Create climber real hardware IO
                    # Note: Constants.CanIDs.CLIMB_TALON is automatically set based on detected robot (Larry vs Comp)
                    climber_io = ClimberIOTalonFX(
                        Constants.CanIDs.CLIMB_TALON,  # Different CAN ID for Larry vs Comp
                        Constants.ClimberConstants.SERVO_PORT,
                        climber_motor_config
                    )
                    
                    # Create climber subsystem with real hardware IO
                    self.climber = ClimberSubsystem(climber_io)
                    print("Climber, Present")
                else:
                    print("Climber subsystem not available on this robot")

            case Constants.Mode.SIM:
                # Sim robot, instantiate physics sim IO implementations (if available)
                self.drivetrain = TunerConstants.create_drivetrain()
                self.vision = VisionSubsystem(
                    self.drivetrain,
                    Constants.VisionConstants.FRONT,
                    Constants.VisionConstants.LAUNCHER,
                )

                # Create climber only if it exists on this robot
                if has_subsystem("climber"):
                    # Create climber subsystem with simulation IO
                    self.climber = ClimberSubsystem(ClimberIOSim())
                    print("Climber, Present")
                else:
                    print("Climber subsystem not available on this robot")

        self.superstructure = Superstructure(
            self.drivetrain, self.vision, self.climber, self.intake
        )

        self._setup_swerve_requests()
        self._pathplanner_setup()
        self._setup_controller_bindings()

    def _pathplanner_setup(self):
        # Register NamedCommands
        NamedCommands.registerCommand("Default", self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT))
        NamedCommands.registerCommand("Score in Hub", self.superstructure.set_goal_command(Superstructure.Goal.SCORE))
        NamedCommands.registerCommand("Climb Ready", self.superstructure.set_goal_command(Superstructure.Goal.CLIMBREADY))
        NamedCommands.registerCommand("Climb", self.superstructure.set_goal_command(Superstructure.Goal.CLIMB))

        # Build AutoChooser
        self._auto_chooser = SendableChooser()

        for auto in os.listdir(os.path.join(getDeployDirectory(), 'pathplanner', 'autos')):
            auto = auto.removesuffix(".auto")
            if auto ==".DS_Store":
                continue
            self._auto_chooser.addOption(auto, PathPlannerAuto(auto, False))
            self._auto_chooser.addOption(auto + " (Mirrored)", PathPlannerAuto(auto, True))
        self._auto_chooser.setDefaultOption("None", cmd.none())
        self._auto_chooser.onChange(
            lambda _: self._set_correct_swerve_position()
        )
        self._auto_chooser.addOption("Basic Leave",
            self.drivetrain.apply_request(lambda: self._robot_centric.with_velocity_x(1)).withTimeout(1.0)
        )
        SmartDashboard.putData("Selected Auto", self._auto_chooser)

    def _set_correct_swerve_position(self) -> None:
        chooser_selected = self._auto_chooser.getSelected()
        try:
            self.drivetrain.reset_pose(self._flip_pose_if_needed(chooser_selected._startingPose))
            self.drivetrain.reset_rotation(chooser_selected._startingPose.rotation() + self.drivetrain.get_operator_forward_direction())
        except AttributeError:
            pass

    @staticmethod
    def _flip_pose_if_needed(pose: Pose2d) -> Pose2d:
        if (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed:
            flipped_x = Constants.FIELD_LAYOUT.getFieldLength() - pose.X()
            flipped_y = Constants.FIELD_LAYOUT.getFieldWidth() - pose.Y()
            flipped_rotation = Rotation2d(pose.rotation().radians()) + Rotation2d.fromDegrees(180)
            return Pose2d(flipped_x, flipped_y, flipped_rotation)
        return pose

    def _setup_swerve_requests(self):
        self._field_centric = (
            swerve.requests.FieldCentric()
            .with_deadband(0)
            .with_rotational_deadband(0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        )

        self._robot_centric: swerve.requests.RobotCentric = (
            swerve.requests.RobotCentric()
            .with_deadband(0)
            .with_rotational_deadband(0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        )

        self._driver_assist: DriverAssist = (
            DriverAssist()
            .with_deadband(self._max_speed * 0.01)
            .with_rotational_deadband(self._max_angular_rate * 0.02)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_translation_pid(Constants.AutoAlignConstants.TRANSLATION_P, Constants.AutoAlignConstants.TRANSLATION_I, Constants.AutoAlignConstants.TRANSLATION_D)
            .with_heading_pid(Constants.AutoAlignConstants.HEADING_P, Constants.AutoAlignConstants.HEADING_I, Constants.AutoAlignConstants.HEADING_D)
        )
        
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

    @staticmethod
    def rumble_command(controller: CommandXboxController, duration: float, intensity: float):
        return cmd.sequence(
            InstantCommand(lambda: controller.setRumble(XboxController.RumbleType.kBothRumble, intensity)),
            cmd.waitSeconds(duration),
            InstantCommand(lambda: controller.setRumble(XboxController.RumbleType.kBothRumble, 0))
        )

    def _setup_controller_bindings(self) -> None:
        hid = self._driver_controller.getHID()
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: self._field_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
            )
        )

        self._driver_controller.leftBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._robot_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
            )
        )

        if self.intake is not None:
            self._driver_controller.rightBumper().whileTrue(
                self.intake.set_desired_state_command(self.intake.SubsystemState.INTAKE)
            ).onFalse(
                self.intake.set_desired_state_command(self.intake.SubsystemState.STOP)
            )
            self._driver_controller.leftBumper().whileTrue(
                self.intake.set_desired_state_command(self.intake.SubsystemState.OUTPUT)
            ).onFalse(
                self.intake.set_desired_state_command(self.intake.SubsystemState.STOP)
            )
        else:
            print("Intake subsystem not available on this robot, unable to bind intake buttons")

        self._driver_controller.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(Rotation2d(-hid.getLeftY(), -hid.getLeftX()))
            )
        )

        
        Trigger(lambda: self._driver_controller.getLeftTriggerAxis() > 0.75).onTrue(
            self.drivetrain.runOnce(lambda: self._driver_assist.with_target_pose(self.drivetrain.get_tower_target(self.drivetrain.TowerSide.LEFT)))
        ).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._driver_assist
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
            )
        )

        Trigger(lambda: self._driver_controller.getRightTriggerAxis() > 0.75).onTrue(
            self.drivetrain.runOnce(lambda: self._driver_assist.with_target_pose(self.drivetrain.get_tower_target(self.drivetrain.TowerSide.RIGHT)))
        ).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._driver_assist
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
            )
        )

        self._driver_controller.start().onTrue(self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric()))

        goal_bindings = {
            self._function_controller.y(): self.superstructure.Goal.SCORE,
            self._function_controller.x(): self.superstructure.Goal.PASSDEPOT,
            self._function_controller.b(): self.superstructure.Goal.PASSOUTPOST,
            self._function_controller.a(): self.superstructure.Goal.DEFAULT,
            self._function_controller.leftBumper(): self.superstructure.Goal.CLIMBREADY,
            self._function_controller.rightBumper(): self.superstructure.Goal.CLIMB,
        }

        """ 
        Leaving last year's goal bindings in for reference

        for button, goal in goal_bindings.items():
            if goal is self.superstructure.Goal.L3_ALGAE or goal is self.superstructure.Goal.NET or goal is self.superstructure.Goal.L2_ALGAE or goal is self.superstructure.Goal.PROCESSOR:
                (button.whileTrue(
                    self.superstructure.set_goal_command(goal)
                    .alongWith(self.intake.set_desired_state_command(self.intake.SubsystemState.ALGAE_INTAKE)))
                    .onFalse(self.intake.set_desired_state_command(self.intake.SubsystemState.ALGAE_HOLD)))
            else:
                button.onTrue(self.superstructure.set_goal_command(goal))

        self._function_controller.leftBumper().onTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.FUNNEL),
                self.intake.set_desired_state_command(self.intake.SubsystemState.FUNNEL_INTAKE),
            )
        ).onFalse(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.DEFAULT),
                self.intake.set_desired_state_command(self.intake.SubsystemState.STOP)
            )
        )

        (self._function_controller.leftBumper() & self._function_controller.back()).whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.FLOOR),
                self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_INTAKE),
            )
        ).onFalse(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.DEFAULT),
                self.intake.set_desired_state_command(self.intake.SubsystemState.STOP),
            )
        )

        (self._function_controller.povLeft() | self._function_controller.povUpLeft() | self._function_controller.povDownLeft()).onTrue(
            cmd.parallel(
                self.climber.set_desired_state_command(self.climber.SubsystemState.CLIMB_OUT),
                self.superstructure.set_goal_command(self.superstructure.Goal.CLIMB)
            )
        ).onFalse(self.climber.set_desired_state_command(self.climber.SubsystemState.STOP))

        (self._function_controller.povRight() | self._function_controller.povUpRight() | self._function_controller.povDownRight()).onTrue(
            cmd.parallel(
                self.climber.set_desired_state_command(self.climber.SubsystemState.CLIMB_IN),
                self.superstructure.set_goal_command(self.superstructure.Goal.CLIMB)
            )
        ).onFalse(self.climber.set_desired_state_command(self.climber.SubsystemState.STOP))

        self._function_controller.povUp().onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.FINISH)
        )

        self._function_controller.rightBumper().whileTrue(
            self.intake.set_desired_state_command(self.intake.SubsystemState.OUTPUT)
        ).onFalse(
            self.intake.set_desired_state_command(self.intake.SubsystemState.STOP)
        )

        (self._function_controller.rightBumper() & self._function_controller.start()).onTrue(
            self.intake.set_desired_state_command(self.intake.SubsystemState.L1_OUTPUT)
        ).onFalse(
            self.intake.set_desired_state_command(self.intake.SubsystemState.STOP)
        )
        """

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()
    
    def get_climber(self) -> Optional[ClimberSubsystem]:
        """Get the climber subsystem if it exists on this robot."""
        return self.climber
    
    def get_intake(self) -> Optional[IntakeSubsystem]:
        """Get the intake subsystem if it exists on this robot."""
        return self.intake
    
    def has_climber(self) -> bool:
        """Check if climber subsystem exists on this robot."""
        return self.climber is not None
    
    def has_intake(self) -> bool:
        """Check if intake subsystem exists on this robot."""
        return self.intake is not None