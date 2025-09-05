f// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import frc.robot.utils.TunableNumber;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.SwerveModuleLimit;
import lib.ironpulse.swerve.sim.SwerveSimConfig;
import lib.ironpulse.swerve.sjtu6.SwerveSJTU6Config;
import lib.ironpulse.utils.Logging;
import lib.ntext.NTParameter;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class RobotConstants {
  // basic constants
  public static final boolean disableHAL = false;
  public static final double LOOPER_DT = 1 / 50.0;
  public static final boolean TUNING = true;
  public static final boolean DriverCamera = true;
  public static final boolean useReplay = false;
  public static final String PARAMETER_TAG = "Param";
  public static String CANIVORE_CAN_BUS_NAME = "6941Canivore0";
  public static String CLIMBER_CAN_BUS = "rio";

  // auto robot config
  public static RobotConfig AUTO_ROBOT_CONFIG;
  static {
    try {
      AUTO_ROBOT_CONFIG = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Logging.error("Constants", "Failed to load AUTO_ROBOT_CONFIG. %s", e.getMessage());
    }
  }

  /**
   * Constants related to the robot's indicators, such as LEDs.
   */
  public static class IndicatorConstants {
    public static final int LED_PORT = 0;
    public static final int LED_BUFFER_LENGTH = 30;
  }

  /**
   * Constants specific to the swerve drivetrain configuration.
   */
  public static class SwerveConstants {
    // tolerance seconds, for swerve to reset in auto.
    // current set to 0.01s (10ms) since no path would consume <10ms.
    public static final double AUTO_SWERVE_TOLERANCE_SECS = 0.01;

    // pigeon id
    public static final int PIGEON_ID = 14;

    // swerve driving
    /**
     * Gearing between the drive motor output shaft and the wheel.
     */
    public static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
    /**
     * Gearing between the steer motor output shaft and the azimuth gear.
     */
    public static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;

    /**
     * Max Voltage Output in voltage.
     */
    public static final Measure<VoltageUnit> MAX_VOLTAGE = Volts.of(12.0);
    /**
     * Radius of the wheel in meters.
     */
    public static final Measure<DistanceUnit> wheelRadius = Meters.of(0.0479);
    /**
     * The max speed of the swerve (should not larger than speedAt12Volts)
     */
    public static final Measure<LinearVelocityUnit> maxSpeed = MetersPerSecond.of(3.5);//4.5
    /**
     * The max angular speed of the swerve.
     */
    public static final Measure<AngularVelocityUnit> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);


    public static final Measure<LinearVelocityUnit> speedAt12Volts = maxSpeed;
    /**
     * The stator current at which the wheels start to slip
     */
    public static final Measure<CurrentUnit> slipCurrent = Amps.of(150.0);
    /**
     * Theoretical free speed (m/s) at 12v applied output;
     */

    // ffw & wheel c
    public static final Measure<DistanceUnit> wheelCircumferenceMeters = Meters
        .of(wheelRadius.magnitude() * 2 * Math.PI);
    public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.69522,
        2.3623, 0.19367);

    public static final double statorCurrent = 110;
    public static final double supplyCurrent = 50;
    public static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0;//0.5
    public static final double deadband = 0.05;
    public static final double rotationalDeadband = 0.05;

    /**
     * Swerve steering gains
     */
    private static final Slot0Configs steerGains = new Slot0Configs().withKP(120)// 120
        .withKI(0.2)// 0.2
        .withKD(0.005)// 0.005
        .withKS(0).withKV(0).withKA(0);

    /**
     * Swerve driving gains
     */
    private static final Slot0Configs driveGains = new Slot0Configs().withKP(1).withKI(0).withKD(0)
        .withKS(0).withKV(0.12).withKA(0);
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    /**
     * The closed-loop output type to use for the drive motors; This affects the
     * PID/FF gains for the drive motors
     */
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    /**
     * The closed-loop output type to use for the steer motors; This affects the
     * PID/FF gains for the steer motors
     */

    private static final double STEER_INERTIA = 0.00001;
    /**
     * Simulation only
     */
    private static final double DRIVE_INERTIA = 0.001;
    /**
     * Simulation only
     */
    private static final Measure<VoltageUnit> steerFrictionVoltage = Volts.of(0.25);
    /**
     * Simulation only
     */
    private static final Measure<VoltageUnit> driveFrictionVoltage = Volts.of(0.25);
    /**
     * Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
     */
    private static final double COUPLE_RATIO = 3.5;
    private static final boolean STEER_MOTOR_REVERSED = true;
    public static final LegacySwerveModuleConstantsFactory ConstantCreator = new LegacySwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO).withSteerMotorGearRatio(STEER_GEAR_RATIO)
        .withWheelRadius(wheelRadius.in(Inches)).withSlipCurrent(slipCurrent.magnitude())
        .withSteerMotorGains(steerGains).withDriveMotorGains(driveGains)
        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
        .withSpeedAt12VoltsMps(speedAt12Volts.magnitude()).withSteerInertia(STEER_INERTIA)
        .withDriveInertia(DRIVE_INERTIA)
        .withSteerFrictionVoltage(steerFrictionVoltage.magnitude())
        .withDriveFrictionVoltage(driveFrictionVoltage.magnitude())
        .withFeedbackSource(LegacySwerveModuleConstants.SteerFeedbackType.SyncCANcoder)
        .withCouplingGearRatio(COUPLE_RATIO).withSteerMotorInverted(STEER_MOTOR_REVERSED);
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
    private static final int FRONT_LEFT_ENCODER_ID = 10;
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.20849609375;
    private static final Measure<DistanceUnit> frontLeftXPos = Meters.of(0.29);
    private static final Measure<DistanceUnit> frontLeftYPos = Meters.of(0.29);
    public static final LegacySwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
        FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID,
        FRONT_LEFT_ENCODER_OFFSET, frontLeftXPos.magnitude(), frontLeftYPos.magnitude(), false);
    // Front Right
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_ENCODER_ID = 11;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = -0.30615234375;
    private static final Measure<DistanceUnit> frontRightXPos = Meters.of(0.29);
    private static final Measure<DistanceUnit> frontRightYPos = Meters.of(-0.29);
    public static final LegacySwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
        FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID,
        FRONT_RIGHT_ENCODER_OFFSET, frontRightXPos.magnitude(), frontRightYPos.magnitude(),
        true);
    // Back Left
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_STEER_MOTOR_ID = 1;
    private static final int BACK_LEFT_ENCODER_ID = 0;
    private static final double BACK_LEFT_ENCODER_OFFSET = 0.032958984375;
    private static final Measure<DistanceUnit> backLeftXPos = Meters.of(-0.29);
    private static final Measure<DistanceUnit> backLeftYPos = Meters.of(0.29);
    public static final LegacySwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
        BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID,
        BACK_LEFT_ENCODER_OFFSET, backLeftXPos.magnitude(), backLeftYPos.magnitude(), false);
    // Back Right
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
    private static final int BACK_RIGHT_ENCODER_ID = 20;
    private static final double BACK_RIGHT_ENCODER_OFFSET = -0.377685546875;
    private static final Measure<DistanceUnit> backRightXPos = Meters.of(-0.29);
    private static final Measure<DistanceUnit> backRightYPos = Meters.of(-0.29);
    public static final LegacySwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
        BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID,
        BACK_RIGHT_ENCODER_OFFSET, backRightXPos.magnitude(), backRightYPos.magnitude(), true);
    // Swerve Module
    public static LegacySwerveModuleConstants[] modules = {FrontLeft, FrontRight, BackLeft, BackRight};
    public static final Translation2d[] modulePlacements = new Translation2d[]{
        new Translation2d(SwerveConstants.FrontLeft.LocationX,
            SwerveConstants.FrontLeft.LocationY),
        new Translation2d(SwerveConstants.FrontRight.LocationX,
            SwerveConstants.FrontRight.LocationY),
        new Translation2d(SwerveConstants.BackLeft.LocationX,
            SwerveConstants.BackLeft.LocationY),
        new Translation2d(SwerveConstants.BackRight.LocationX,
            SwerveConstants.BackRight.LocationY)};

    public static final String kSwerveTag = "Swerve";
    public static final String kSwerveModuleTag = "Swerve/SwerveModule";
    public static final double kSwerveHalfWidth = 0.6 / 2.0;
    public static SwerveModuleLimit kDefaultSwerveModuleLimit = SwerveModuleLimit.builder()
        // v (mps) = 6000.0 (foc max omega in rpm) / 60.0 / reduction (drive gear ratio 7) * circumference
        .maxDriveVelocity(MetersPerSecond.of(4.559797259))
        .maxDriveAcceleration(MetersPerSecondPerSecond.of(20.0))
        // omega (rps) = 6000.0 (foc max omega in rpm) / 60.0 / reduction
        .maxSteerAngularVelocity(RotationsPerSecond.of(6000.0 / 60.0 / 22.0))
        // accelerate in 0.1s // TODO: sysid needed
        .maxSteerAngularAcceleration(RotationsPerSecondPerSecond.of(6000.0 / 60.0 / 22.0 / 0.05))
        .build();
    public static SwerveLimit kDefaultSwerveLimit = SwerveLimit.builder()
        .maxLinearVelocity(MetersPerSecond.of(4.5))
        .maxSkidAcceleration(MetersPerSecondPerSecond.of(15.0))
        // must be smaller than 4.5 / (dist * sqrt(2)) to be actually effective
        .maxAngularVelocity(DegreesPerSecond.of(700.0))
        // accelerate in 0.2s, also must be smaller than the defined module limit to be actually effective
        .maxAngularAcceleration(DegreesPerSecondPerSecond.of(2000.0))
        .build();


    public static SwerveConfig.SwerveModuleConfig kModuleCompFL = SwerveConfig.SwerveModuleConfig.builder()
        .name("FL")
        .location(new Translation2d(kSwerveHalfWidth, kSwerveHalfWidth))
        .driveMotorId(4)
        .steerMotorId(3)
        .encoderId(10)
        .driveMotorEncoderOffset(Degree.of(0))
        .steerMotorEncoderOffset(Rotations.of(-0.247802))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompFR = SwerveConfig.SwerveModuleConfig.builder()
        .name("FR")
        .location(new Translation2d(kSwerveHalfWidth, -kSwerveHalfWidth))
        .driveMotorId(6)
        .steerMotorId(5)
        .encoderId(11)
        .driveMotorEncoderOffset(Degree.of(0))
        .steerMotorEncoderOffset(Rotations.of(-0.295898))
        .driveInverted(true)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompBL = SwerveConfig.SwerveModuleConfig.builder()
        .name("BL")
        .location(new Translation2d(-kSwerveHalfWidth, kSwerveHalfWidth))
        .driveMotorId(2)
        .steerMotorId(1)
        .encoderId(0)
        .driveMotorEncoderOffset(Degree.of(0))
        .steerMotorEncoderOffset(Rotations.of(0.04003906))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
    public static SwerveConfig.SwerveModuleConfig kModuleCompBR = SwerveConfig.SwerveModuleConfig.builder()
        .name("BR")
        .location(new Translation2d(-kSwerveHalfWidth, -kSwerveHalfWidth))
        .driveMotorId(8)
        .steerMotorId(7)
        .encoderId(20)
        .driveMotorEncoderOffset(Degree.of(0))
        .steerMotorEncoderOffset(Rotations.of(-0.3833007))
        .driveInverted(true)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
    public static SwerveSimConfig kSimConfig = SwerveSimConfig.builder()
        .name("Swerve")
        .dtS(LOOPER_DT)
        .wheelDiameter(Inch.of(4.1))
        .driveGearRatio(7.0)
        .steerGearRatio(20.0)
        .driveMotor(DCMotor.getKrakenX60Foc(1))
        .driveMomentOfInertia(KilogramSquareMeters.of(0.04))
        .driveStdDevPos(0.0000001)
        .driveStdDevVel(0.000001)
        .steerMotor(DCMotor.getKrakenX60Foc(1))
        .steerMomentOfInertia(KilogramSquareMeters.of(0.01))
        .steerStdDevPos(0.0000001)
        .steerStdDevVel(0.000001)
        .defaultSwerveLimit(kDefaultSwerveLimit)
        .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
        .moduleConfigs(new SwerveConfig.SwerveModuleConfig[]{
            kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
        })
        .build();
    public static SwerveSJTU6Config kRealConfig = SwerveSJTU6Config.builder()
        .name("Swerve")
        .dtS(LOOPER_DT)
        .wheelDiameter(Inch.of(4.1))
        .driveGearRatio(6.7460317460317460317460317460317)
        .steerGearRatio(21.428571428571428571428571428571)
        .defaultSwerveLimit(kDefaultSwerveLimit)
        .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
        .moduleConfigs(new SwerveConfig.SwerveModuleConfig[]{
            kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
        })
        .odometryFrequency(Hertz.of(100))
        .driveStatorCurrentLimit(Amps.of(100))
        .steerStatorCurrentLimit(Amps.of(40))
        .canivoreCanBusName(CANIVORE_CAN_BUS_NAME)
        .pigeonId(PIGEON_ID)
        .build();


    @NTParameter(tableName = "Params" + "/" + kSwerveModuleTag)
    private final static class SwerveModuleParams {
      private final static class Drive {
        static final double kP = 8.0;
        static final double kI = 0.14;
        static final double kD = 0.0;
        static final double kS = 0.0;
        static final double kV = 0.1247;
        static final double kA = 0.01215;
        static final boolean isBrake = true;
      }

      private final static class Steer {
        static final double kP = 10;
        static final double kI = 0.001;
        static final double kD = 0.15;
        static final double kS = 0.005;
        static final boolean isBrake = true;
      }
    }
  }

  /**
   * Constants specific to the reef aim mechanism.
   */
  public static final class ReefAimConstants {
    public static final TunableNumber HEXAGON_DANGER_ZONE_OFFSET = new TunableNumber("AIM/HEXAGON_DANGER_ZONE_OFFSET", 0.24);
    public static final TunableNumber MAX_DISTANCE_REEF_LINEUP = new TunableNumber("AIM/maxLineupDistance", 0.75);
    public static final Measure<DistanceUnit> PIPE_TO_TAG = Meters.of(0.164308503);
    public static final TunableNumber ROBOT_TO_PIPE_METERS = new TunableNumber("AIM/ROBOT_TO_PIPE_METERS", 0.59);
    public static final TunableNumber X_TOLERANCE_METERS = new TunableNumber("AIM/X_TOLERANCE_METERS", 0.01);
    public static final TunableNumber Y_TOLERANCE_METERS = new TunableNumber("AIM/Y_TOLERANCE_METERS", 0.01);
    public static final TunableNumber RAISE_LIMIT_METERS = new TunableNumber("AIM/RAISE_LIMIT_METERS", 1);
    public static final TunableNumber OMEGA_TOLERANCE_DEGREES = new TunableNumber("AIM/OMEGA_TOLERANCE_DEGREES", 1);
    public static final Measure<LinearVelocityUnit> MAX_AIMING_SPEED = MetersPerSecond.of(3.5);
    public static final Measure<LinearAccelerationUnit> MAX_AIMING_ACCELERATION = MetersPerSecondPerSecond.of(10);
    public static final TunableNumber Edge_Case_Max_Delta = new TunableNumber("AIM/MAX DELTA", 0.2);
    public static final TunableNumber ROBOT_TO_ALGAE_METERS = new TunableNumber("AIM/ROBOT_TO_ALGAE_METERS", 0.489);
    public static final TunableNumber ALGAE_TO_TAG_METERS = new TunableNumber("AIM/ALGAE_TO_TAG_METERS", 0);
    public static final TunableNumber HEXAGON_DANGER_DEGREES = new TunableNumber("AIM/HEXAGON_DANGER_DEGREES", 45);
  }

  /**
   * Constants related to the beambreak subsystem.
   */
  public static class BeamBreakConstants {
    public static final int ENDEFFECTORARM_CORAL_BEAMBREAK_ID = 1;
    public static final int ENDEFFECTORARM_ALGAE_BEAMBREAK_ID = 2;
    public static final int INTAKE_BEAMBREAK_ID = 3;
  }

  /**
   * Constants related to the robot's intake subsystem.
   */
  public static class SuperstructureConstants {
    /**
     * Tunable parameters for intake pose toggling functionality.
     * These constants control the alternate intake angles for specific superstructure states.
     */
    @NTParameter(tableName = "Params/Superstructure")
    public static final class IntakeToggleParams {
      // Stow states alternate intake angles
      static final double CORAL_STOW_ALT_INTAKE = 110.0;
      static final double ALGAE_STOW_ALT_INTAKE = 80.0;
      
      // Score states alternate intake angles  
      static final double L1_SHOOT_SIDE_ALT_INTAKE = 40.0;
      static final double L2_ALT_INTAKE = 110.0;
      static final double L3_ALT_INTAKE = 110.0;
      static final double L4_ALT_INTAKE = 110.0;
      static final double NET_SCORE_ALT_INTAKE = 40.0;
      
      // Basic states alternate intake angles
      static final double IDLE_ALT_INTAKE = 110.0;
      static final double AVOID_ALT_INTAKE = 110.0;
    }
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 15;
    public static final int INDEX_MOTOR_ID = 19;
    public static final int INTAKE_PIVOT_MOTOR_ID = 16;
    public static final int INTAKE_PIVOT_ENCODER_ID = 17;
    public static final double INTAKE_PIVOT_ROTOR_ENCODER_RATIO = 45 / 11 * 56 / 20 * 56 / 8;

    //Constants for intake roller
    public static final int STATOR_CURRENT_LIMIT_AMPS = 60;
    public static final int SUPPLY_CURRENT_LIMIT_AMPS = 60;
    public static final boolean IS_BRAKE = false;
    public static final boolean IS_INTAKER_INVERT = true;
    public static final boolean IS_INDEXER_INVERT = false;
    public static final boolean INDEX_FOLLOWER_INVERT = true; 
    public static final double REDUCTION = 1;
    public static final double moi = 0;//inertia for simulation
    public static final double ROLLER_RATIO = 1;
    public static final double INTAKE_DANGER_ZONE = 90;
    public static final TunableNumber ROLLER_AMPS_HAS_CORAL = new TunableNumber("INTAKE_ROLLER/rollerAmpsHasCoral", 55);
    public static final TunableNumber INTAKE_PIVOT_JERK = new TunableNumber("INTAKE_PIVOT/jerk", 0);
    public static final TunableNumber INTAKE_PIVOT_TOLERANCE = new TunableNumber("INTAKE_PIVIOT/tolerance", 3.5);

    public static final double INTAKE_PIVOT_ENCODER_OFFSET = -0.160156-0.25;
    //Motion constants for intake roller
    public static final TunableNumber INTAKE_VOLTAGE = new TunableNumber("INTAKE_ROLLER/intakeVoltage", 15.0);
    public static final TunableNumber INDEX_ROLLER_VOLTAGE = new TunableNumber("INTAKE_ROLLER/indexRollerVoltage", 15.0);
    public static final TunableNumber INDEX_OUTTAKE_VOLTAGE = new TunableNumber("INTAKE_ROLLER/indexRollerVoltage", -12);
    public static final TunableNumber OUTTAKE_VOLTAGE = new TunableNumber("INTAKE_ROLLER/outtakeVoltage", -6.0);
    public static final TunableNumber SHOOT_VOLTAGE = new TunableNumber("INTAKE_ROLLER/shootVoltage", -2.5);
    public static final TunableNumber INTAKE_HOLD_VOLTAGE = new TunableNumber("INTAKE_ROLLER/intakeHoldVoltage", 1.5);


    //Constants for intake pivot
    public static double PIVOT_RATIO = (double) (12 * 50) / 11;

    /**
     * Constants for the intake pivot motor gains in the intake subsystem.
     */
    public static class IntakePivotGainsClass {
      public static final TunableNumber INTAKE_PIVOT_KP = new TunableNumber("INTAKE_PIVOT PID/kp", 4.7);
      public static final TunableNumber INTAKE_PIVOT_KI = new TunableNumber("INTAKE_PIVOT PID/ki", 0.5);
      public static final TunableNumber INTAKE_PIVOT_KD = new TunableNumber("INTAKE_PIVOT PID/kd", 0.02);
      public static final TunableNumber INTAKE_PIVOT_KS = new TunableNumber("INTAKE_PIVOT PID/ks", 0);
      public static final TunableNumber INTAKE_PIVOT_KG = new TunableNumber("INTAKE_PIVOT PID/kg", -0.035);

      public static final TunableNumber INTAKE_PIVOT_CRUISEVEL = new TunableNumber("INTAKE_PIVOT/cruiseVel", 100);
      public static final TunableNumber INTAKE_PIVOT_ACC = new TunableNumber("INTAKE_PIVOT/acc", 100);
    }
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 52;
    public static final double CLIMBER_RATIO = 5 * 5 * 4;
    public static final int CLIMBER_ROLLER_MOTOR_ID = 55;
  }

  /**
   * Constants related to the elevator subsystem.
   */
  public static class ElevatorConstants {
    public static final int LEFT_ELEVATOR_MOTOR_ID = 50;
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 51;

    public static final double ELEVATOR_SPOOL_DIAMETER = 0.04 + 0.003; //0.04m for spool diameter, 0.003 for rope diameter
    public static final double ELEVATOR_GEAR_RATIO = 3.0;
    public static final double ELEVATOR_DANGER_ZONE = 0.4180619200456253;
    public static final double ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED = 0.0;
    public static final int ELEVATOR_ZEROING_FILTER_SIZE = 5;
    public static final TunableNumber ELEVATOR_GOAL_TOLERANCE = new TunableNumber("Elevator/GoalTolerance", 0.03);


    // Dynamic Motion Magic configs - separate for up and down movement
    public static final TunableNumber motionAccelerationUp = new TunableNumber("Elevator/MotionAccelerationUp",
        600);
    public static final TunableNumber motionCruiseVelocityUp = new TunableNumber("Elevator/MotionCruiseVelocityUp",
        250);
    public static final TunableNumber motionJerkUp = new TunableNumber("Elevator/MotionJerkUp",
        0.0);

    public static final TunableNumber motionAccelerationDown = new TunableNumber("Elevator/MotionAccelerationDown",
        200);
    public static final TunableNumber motionCruiseVelocityDown = new TunableNumber("Elevator/MotionCruiseVelocityDown",
        160);
    public static final TunableNumber motionJerkDown = new TunableNumber("Elevator/MotionJerkDown",
        0.0);
    public static final TunableNumber MAX_EXTENSION_METERS = new TunableNumber("Elevator/maxExtension",
        1.475);
    public static final TunableNumber ELEVATOR_ZEROING_CURRENT = new TunableNumber("Elevator/zeroingCurrent",
        50);
    public static final TunableNumber SAFE_HEIGHT_FLIP = new TunableNumber("Elevator/safeHeightFlip", 0.5);

    public static final TunableNumber FLYBY_HEIGHT = new TunableNumber("Elevator/FlybyHeight", 1.43);
    // SysId characterization constants
    public static final TunableNumber SYSID_RAMP_RATE_VOLTS_PER_SEC = new TunableNumber("Elevator/PID/SysId/rampRateVoltsPerSec", 1);
    public static final TunableNumber SYSID_DYNAMIC_VOLTAGE = new TunableNumber("Elevator/PID/SysId/dynamicVoltage", 5);
    public static final TunableNumber SYSID_TIMEOUT_SECONDS = new TunableNumber("Elevator/PID/SysId/timeoutSeconds", 10.0);

  }

  /**
   * Constants for the elevator motor gains.
   */
  public static class ElevatorGainsClass {
    public static final TunableNumber ELEVATOR_KP = new TunableNumber("Elevator/PID/kp", 3.75);
    public static final TunableNumber ELEVATOR_KI = new TunableNumber("Elevator/PID/ki", 0);
    public static final TunableNumber ELEVATOR_KD = new TunableNumber("Elevator/PID/kd", 0);
    public static final TunableNumber ELEVATOR_KA = new TunableNumber("Elevator/PID/ka", 0.0068);
    public static final TunableNumber ELEVATOR_KV = new TunableNumber("Elevator/PID/kv", 0.1308);// 0.107853495
    public static final TunableNumber ELEVATOR_KS = new TunableNumber("Elevator/PID/ks", 0.13);
    public static final TunableNumber ELEVATOR_KG = new TunableNumber("Elevator/PID/kg", 0.32);//0.3
  }

    public static class LimelightConstants {
        public static final String LIMELIGHT_LEFT = "limelight-leftf";
        public static final String LIMELIGHT_RIGHT = "limelight-rightf";
        public static final double AREA_THRESHOLD = 0.1;
        public static final TunableNumber OCULUS_RESET_AMBIGUITY_THRESHOLD = new TunableNumber("LIMELIGHT/oculusResetAmbiguityThreshold", 0.15);
    }

    public static class PhotonvisionConstants {
        public static final String[] PV_CAMERA_NAMES = {"pv-cam1"};
        public static final boolean[] SNAPSHOT_ENABLED = {true};
        public static final int SNAPSHOT_PERIOD = 5;
    }

  /**
   * Constants related to the EndEffectorArm subsystem.
   */
  public static class EndEffectorArmConstants {
    // Motor IDs
    public static final int END_EFFECTOR_ARM_PIVOT_MOTOR_ID = 21;
    public static final int END_EFFECTOR_ARM_ROLLER_MOTOR_ID = 22;
    public static final int END_EFFECTOR_ARM_ENCODER_ID = 23;

    // Roller motor configuration
    public static final int STATOR_CURRENT_LIMIT_AMPS = 80;
    public static final int SUPPLY_CURRENT_LIMIT_AMPS = 40;
    public static final boolean IS_BRAKE = true;
    public static final boolean IS_INVERT = false;
    // Pivot motor configuration
    public static final double ROTOR_SENSOR_RATIO = 1.0 / 8 * 90 / 18 * 60;



    @NTParameter(tableName = "Params/EndEffectorArm")
    public static final class EndEffectorArmParams {
      // Encoder and position constants
      static final double encoderOffset = 0.65625;
      static final double maxAngleDegrees = 370.0;
      static final double pivotTolerance = 3.5;

      // Motion Magic parameters
      static final double motionMagicCruiseVelocity = 1200.0; // degrees/second
      static final double motionMagicAcceleration = 10000.0; // degrees/second^2
      static final double motionMagicJerk = 60000.0; // degrees/second^3

      // Roller voltages for different operations
      static final double coralIntakeVoltage = 12.0;
      static final double coralOuttakeVoltage = -6.0;
      static final double coralPreshootVoltage = -10.0;
      static final double algaeIntakeVoltage = 8.0;
      static final double algaePreshootVoltage = -12.0;
      static final double coralHoldVoltage = 0.5;
      static final double algaeHoldVoltage = 1.5;
      static final double coralShootVoltage = -12.0;
      static final double coralShootVoltageL1 = -2.0;
      static final double algaeNetShootVoltage = -11.0;
      static final double algaeProcessorShootVoltage = -4.0;
      static final double coralShootDelayTime = 0.2;

      // Pivot motor gains
      static final double pivotKP = 220;
      static final double pivotKI = 0.0;
      static final double pivotKD = 6.0;
      static final double pivotKA = 0.001; // Use calculated theoretical value
      static final double pivotKV = 0.2145; // Use calculated theoretical value
      static final double pivotKS = 0;
      static final double pivotKG = -0.18;

      // Roller motor gains (currently open loop)
      static final double rollerKP = 0.0;
      static final double rollerKI = 0.0;
      static final double rollerKD = 0.0;
      static final double rollerKA = 0.0;
      static final double rollerKV = 0.0;
      static final double rollerKS = 0.0;
    }

    /**
     * @deprecated Use EndEffectorArmParams instead. Legacy constants for backward compatibility.
     */
    @Deprecated
    public static final TunableNumber END_EFFECTOR_ARM_ENCODER_OFFSET = new TunableNumber("EEARM/Pivot/encoderOffset", 0.65625);
    @Deprecated
    public static final TunableNumber MAX_ANGLE_DEGREES = new TunableNumber("EEARM/Pivot/maxAngleDegrees", 370.0);
    @Deprecated
    public static final TunableNumber END_EFFECTOR_ARM_PIVIOT_TOLERANCE = new TunableNumber("EEARM/Pivot/tolerance", 3.5);

    // Roller voltages for different operations
    @Deprecated
    public static final TunableNumber CORAL_INTAKE_VOLTAGE = new TunableNumber("EEARM/Roller/coralIntakeVoltage", 12.0);
    @Deprecated
    public static final TunableNumber CORAL_OUTTAKE_VOLTAGE = new TunableNumber("EEARM/Roller/coralOuttakeVoltage", -6.0);
    @Deprecated
    public static final TunableNumber CORAL_PRESHOOT_VOLTAGE = new TunableNumber("EEARM/Roller/coralPreShootVoltage", -10.0);
    @Deprecated
    public static final TunableNumber ALGAE_INTAKE_VOLTAGE = new TunableNumber("EEARM/Roller/algaeIntakeVoltage", 8.0);
    @Deprecated
    public static final TunableNumber ALGAE_PRESHOOT_VOLTAGE = new TunableNumber("EEARM/Roller/algaePreShootVoltage", -12.0);
    @Deprecated
    public static final TunableNumber CORAL_HOLD_VOLTAGE = new TunableNumber("EEARM/Roller/coralHoldVoltage", 0.5);
    @Deprecated
    public static final TunableNumber ALGAE_HOLD_VOLTAGE = new TunableNumber("EEARM/Roller/algaeHoldVoltage", 1.5);
    @Deprecated
    public static final TunableNumber CORAL_SHOOT_VOLTAGE = new TunableNumber("EEARM/Roller/coralShootVoltage", -11.0);
    @Deprecated
    public static final TunableNumber CORAL_SHOOT_VOLTAGE_L1 = new TunableNumber("EEARM/Roller/coralShootVoltageL1", -2.0);
    @Deprecated
    public static final TunableNumber ALGAE_NET_SHOOT_VOLTAGE = new TunableNumber("EEARM/Roller/algaeNetShootVoltage", -12.0);
    @Deprecated
    public static final TunableNumber ALGAE_PROCESSOR_SHOOT_VOLTAGE = new TunableNumber("EEARM/Roller/algaeProcessorShootVoltage", -4.0);
    @Deprecated
    public static final TunableNumber NET_SHOOT_DELAY_TIME = new TunableNumber("EEARM/Roller/netShootDelayTime", 0.2);
    @Deprecated
    public static final TunableNumber PROCESSOR_SHOOT_DELAY_TIME = new TunableNumber("EEARM/Roller/processorShootDelayTime", 0.5);
    @Deprecated
    public static final TunableNumber L1_SHOOT_SIDE_EJECT_DELAY_TIME = new TunableNumber("EEARM/Roller/l1ShootSideEjectDelayTime", 1);




 
  }

  /**
   * Constants for QuestNav subsystem (VR headset pose tracking)
   */
  public static class QuestNavConstants {
    // Standard deviations for pose estimation (in meters and degrees)
    public static final TunableNumber STD_DEV_X = new TunableNumber("QuestNav/StdDevX", 0.02);
    public static final TunableNumber STD_DEV_Y = new TunableNumber("QuestNav/StdDevY", 0.02);
    public static final TunableNumber STD_DEV_ROT_DEG = new TunableNumber("QuestNav/StdDevRotDeg", 2.0);

    // Transform from robot center to Quest headset (in meters and degrees)
    // TODO: Measure and update these values based on actual Quest mounting position
    public static final TunableNumber ROBOT_TO_QUEST_X = new TunableNumber("QuestNav/RobotToQuestX", 0.12);
    public static final TunableNumber ROBOT_TO_QUEST_Y = new TunableNumber("QuestNav/RobotToQuestY", 0.337);
    public static final TunableNumber ROBOT_TO_QUEST_ROT_DEG = new TunableNumber("QuestNav/RobotToQuestRotDeg", 10.0);

    // Enable/disable vision updates
    public static final TunableNumber ENABLE_VISION_UPDATES = new TunableNumber("QuestNav/EnableVisionUpdates", 1.0);

    // Pose reset validation thresholds
    public static final TunableNumber MAX_RESET_DISTANCE_METERS = new TunableNumber("QuestNav/MaxResetDistanceMeters", 0.01);
    public static final TunableNumber MAX_RESET_ANGLE_DEGREES = new TunableNumber("QuestNav/MaxResetAngleDegrees", 1);
  }
}
