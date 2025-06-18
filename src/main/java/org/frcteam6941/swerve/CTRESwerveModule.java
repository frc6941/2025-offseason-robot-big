package org.frcteam6941.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

// Represents a swerve module using CTRE hardware components
public class CTRESwerveModule implements SwerveModuleBase {
    private final int moduleNumber;
    private final CTRESwerveIO module;

    // Constructor to initialize the swerve module with an ID, constants, and CAN bus name
    public CTRESwerveModule(int id, LegacySwerveModuleConstants constants, String canbusName) {
        moduleNumber = id;
        module = new CTRESwerveIO(constants, canbusName);
        module.getSteerMotor().getConfigurator().apply(new Slot0Configs()
                .withKP(RobotConstants.SwerveConstants.steerGainsClass.STEER_KP.get())
                .withKI(RobotConstants.SwerveConstants.steerGainsClass.STEER_KI.get())
                .withKD(RobotConstants.SwerveConstants.steerGainsClass.STEER_KD.get())
                .withKA(RobotConstants.SwerveConstants.steerGainsClass.STEER_KA.get())
                .withKV(RobotConstants.SwerveConstants.steerGainsClass.STEER_KV.get())
                .withKS(RobotConstants.SwerveConstants.steerGainsClass.STEER_KS.get()));
        module.getDriveMotor().getConfigurator().apply(new Slot0Configs()
                .withKP(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KP.get())
                .withKI(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KI.get())
                .withKD(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KD.get())
                .withKA(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KA.get())
                .withKV(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KV.get())
                .withKS(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KS.get()));
    }

    // Returns the module number of this swerve module
    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    // Gets the current state of the swerve module including speed and angle
    @Override
    public SwerveModuleState getState() {
        return module.getCurrentState();
    }

    // Retrieves the position of the swerve module
    @Override
    public SwerveModulePosition getPosition() {
        return module.getInternalState();
    }

    // Updates the signals for the swerve module, typically called in a periodic loop
    @Override
    public void updateSignals() {
        SwerveModulePosition pos = module.getPosition(true);
    }

    // Sets the desired state for the swerve module, including speed and angle, with options for open loop control and motion override
    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        //        IF YOU WANT TO ADJUST SWERVE PID, DEANNOTATE FOLLOWING LINES:
        //        module.getSteerMotor().getConfigurator().apply(new Slot0Configs()
        //                .withKP(RobotConstants.SwerveConstants.steerGainsClass.STEER_KP.get())
        //                .withKI(RobotConstants.SwerveConstants.steerGainsClass.STEER_KI.get())
        //                .withKD(RobotConstants.SwerveConstants.steerGainsClass.STEER_KD.get())
        //                .withKA(RobotConstants.SwerveConstants.steerGainsClass.STEER_KA.get())
        //                .withKV(RobotConstants.SwerveConstants.steerGainsClass.STEER_KV.get())
        //                .withKS(RobotConstants.SwerveConstants.steerGainsClass.STEER_KS.get()));
        //        module.getDriveMotor().getConfigurator().apply(new Slot0Configs()
        //                .withKP(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KP.get())
        //                .withKI(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KI.get())
        //                .withKD(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KD.get())
        //                .withKA(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KA.get())
        //                .withKV(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KV.get())
        //                .withKS(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KS.get()));

        module.apply(desiredState, isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity);
        Logger.recordOutput("swerve/modules/" + moduleNumber + "/desiredState", desiredState);
        Logger.recordOutput("swerve/modules/" + moduleNumber + "/currentState", module.getCurrentState());
    }
}
