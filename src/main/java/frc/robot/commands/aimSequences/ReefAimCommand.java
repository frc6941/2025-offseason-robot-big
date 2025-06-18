package frc.robot.commands.aimSequences;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.display.Display;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import static frc.robot.RobotConstants.ReefAimConstants;

public class ReefAimCommand extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
    private final ProfiledPIDController xPID = new ProfiledPIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get(),
            new TrapezoidProfile.Constraints(
                    ReefAimConstants.MAX_AIMING_SPEED.magnitude(),
                    ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude()));
    private final ProfiledPIDController yPID = new ProfiledPIDController(
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
            RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get(),
            new TrapezoidProfile.Constraints(
                    ReefAimConstants.MAX_AIMING_SPEED.magnitude(),
                    ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude()));
    private final BooleanSupplier stop;
    private final CommandXboxController driverController;
    private final IndicatorSubsystem indicatorSubsystem;
    private boolean rightReef; // true if shooting right reef
    private boolean xFinished = false;
    private boolean yFinished = false;
    private boolean omegaFinished = false;
    private Pose2d robotPose, tagPose, destinationPose, finalDestinationPose;
    private Translation2d translationalVelocity, controllerVelocity;


    public ReefAimCommand(BooleanSupplier stop, CommandXboxController driverController,
                          IndicatorSubsystem indicatorSubsystem) {
        addRequirements(swerve);
        this.stop = stop;
        this.driverController = driverController;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void initialize() {
        robotPose = swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());

        // Calculate destination
        tagPose = AimGoalSupplier.getNearestTag(robotPose);

        // Choose target based on game piece
        if (destinationSupplier.getCurrentGamePiece() == DestinationSupplier.GamePiece.ALGAE_INTAKING) {
            finalDestinationPose = AimGoalSupplier.getFinalAlgaeTarget(tagPose);
        } else {
            rightReef = destinationSupplier.getCurrentBranch();
            finalDestinationPose = AimGoalSupplier.getFinalCoralTarget(tagPose, rightReef);
        }

        // Now that finalDestinationPose is set, we can get the drive target
        destinationPose = AimGoalSupplier.getDriveTarget(robotPose, finalDestinationPose);

        // Get velocities in field coordinates & Flip velocities if needed
        double vx = (AllianceFlipUtil.shouldFlip() ? -1 : 1) * swerve.getLocalizer().getSmoothedVelocity().getX();
        double vy = (AllianceFlipUtil.shouldFlip() ? -1 : 1) * swerve.getLocalizer().getSmoothedVelocity().getY();

        // PID init with field-relative velocities
        xPID.reset(vx);
        yPID.reset(vy);

        indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
    }

    @Override
    public void execute() {
        xPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ReefAimConstants.MAX_AIMING_SPEED.magnitude(),
                        ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude()));
        yPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ReefAimConstants.MAX_AIMING_SPEED.magnitude(),
                        ReefAimConstants.MAX_AIMING_ACCELERATION.magnitude()));
        if (RobotConstants.TUNING) {
            xPID.setPID(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
            yPID.setPID(RobotConstants.SwerveConstants.AimGainsClass.AIM_KP.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KI.get(),
                    RobotConstants.SwerveConstants.AimGainsClass.AIM_KD.get());
        }

        robotPose = swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp());
        destinationPose = AimGoalSupplier.getDriveTarget(robotPose, finalDestinationPose);

        xPID.setGoal(destinationPose.getTranslation().getX());
        yPID.setGoal(destinationPose.getTranslation().getY());
        swerve.setLockHeading(true);
        swerve.setHeadingTarget(destinationPose.getRotation().getDegrees());

        double xPIDOutput = xPID.calculate(robotPose.getX());
        double yPIDOutput = yPID.calculate(robotPose.getY());

        if (AllianceFlipUtil.shouldFlip()) {
            xPIDOutput = -xPIDOutput;
            yPIDOutput = -yPIDOutput;
        }

        controllerVelocity = new Translation2d(
                Math.abs(driverController.getLeftY()) < RobotConstants.SwerveConstants.deadband ?
                        0 : -driverController.getLeftY() * RobotConstants.SwerveConstants.maxSpeed.magnitude(),
                Math.abs(driverController.getLeftX()) < RobotConstants.SwerveConstants.deadband ?
                        0 : -driverController.getLeftX() * RobotConstants.SwerveConstants.maxSpeed.magnitude());
        translationalVelocity = controllerVelocity.getX() < -0.5 ?
                new Translation2d(0, 0) : new Translation2d(xPIDOutput, yPIDOutput);

        swerve.drive(translationalVelocity.plus(controllerVelocity), 0.0, true, false);
        Display.getInstance().setAimingTarget(destinationPose);

        Logger.recordOutput("ReefAimCommand/tagPose", tagPose);
        Logger.recordOutput("ReefAimCommand/destinationPose", destinationPose);
        Logger.recordOutput("ReefAimCommand/finalDestinationPose", finalDestinationPose);
        Logger.recordOutput("ReefAimCommand/translationalVelocity", translationalVelocity);
        Logger.recordOutput("ReefAimCommand/controllerVelocity", controllerVelocity);
        Logger.recordOutput("ReefAimCommand/shouldFlip", AllianceFlipUtil.shouldFlip());
    }

    @Override
    public boolean isFinished() {
        xFinished = Math.abs(robotPose.getX() - finalDestinationPose.getX()) < ReefAimConstants.X_TOLERANCE_METERS.get();
        yFinished = Math.abs(robotPose.getY() - finalDestinationPose.getY()) < ReefAimConstants.Y_TOLERANCE_METERS.get();
        omegaFinished = swerve.aimingReady(ReefAimConstants.OMEGA_TOLERANCE_DEGREES.get(), 5);
        Logger.recordOutput("ReefAimCommand/xFinished", xFinished);
        Logger.recordOutput("ReefAimCommand/yFinished", yFinished);
        Logger.recordOutput("ReefAimCommand/omegaFinished", omegaFinished);
        Logger.recordOutput("ReefAimCommand/emergencyStopped", stop.getAsBoolean());
        return (xFinished && yFinished && omegaFinished) || stop.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0.0, true, false);
        swerve.setLockHeading(false);
        if (!interrupted) indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);
        else indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}