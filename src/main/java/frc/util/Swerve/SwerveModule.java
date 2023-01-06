package frc.util.Swerve;


import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.RobotConstants.ModuleConstants;


public class SwerveModule {

private final TalonFX driveMotor;
private final TalonFX turningMotor;


private final PIDController turningPidController;

private final AnalogInput absoluteEncoder; //DutyCycleEncoder absoluteEncoder;
private final boolean absoluteEncoderReversed;
private final double absoluteEncoderOffsetRad;



public SwerveModule(int turningMotorId, int driveMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);//new DutyCycleEncoder(absoluteEncoderId);

    driveMotor = new TalonFX(driveMotorId);
    turningMotor = new TalonFX(turningMotorId);

    driveMotor.configFactoryDefault();
    turningMotor.configFactoryDefault();

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    turningMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setNeutralMode(NeutralMode.Coast);
    //driveMotor.configOpenloopRamp(0.7);
    driveMotor.configNeutralDeadband(0.02);
    turningMotor.configNeutralDeadband(0.02);
    turningMotor.configOpenloopRamp(0.05);

    
    
    double p = RobotConstants.getModulePID().p;
    double i = RobotConstants.getModulePID().i;
    double d = RobotConstants.getModulePID().d;
    

    turningPidController = new PIDController(p,i,d,0.01);
    
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    this.resetEncoders();
    
    
}



public double getDrivePosition() {
    return driveMotor.getSelectedSensorPosition();
}

public double getTurningPosition() {
    return Math.PI * (turningMotor.getSelectedSensorPosition() / ModuleConstants.kTurningEncoderRot2Rad);
}

public double getDriveVelocity() {
    return ((driveMotor.getSelectedSensorVelocity() / 2048.0) * ModuleConstants.kDriveMotorGearRatio) * (Math.PI * ModuleConstants.kWheelDiameterMeters) * 10.0;
}

public double getTurningVelocity() {
    return turningMotor.getSelectedSensorVelocity();
}

public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / 5.0;
    
    angle *= 2.0 * Math.PI;
    angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    angle -= absoluteEncoderOffsetRad;
    return angle; 
}
    



public void resetEncoders() {
    this.driveMotor.setSelectedSensorPosition(0);
    
    this.turningMotor.setSelectedSensorPosition((getAbsoluteEncoderRad() / Math.PI) * ModuleConstants.kTurningEncoderRot2Rad);
   
   
   
}

public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
}

public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(ControlMode.PercentOutput,state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    double pidOutput = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
    turningMotor.set(ControlMode.PercentOutput,pidOutput);
   // SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] PID OUTPUT", pidOutput);
    //SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] State Radianss ", state.angle.getRadians());
    //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    
}

public void updateDashboard(){
    SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] Current POS", getTurningPosition());
    SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] abs", getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] abs raw", absoluteEncoder.getVoltage() / 5.0);
    //SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getChannel() + "] averageAbsolute", realAbsoluteEncoderAverage);
}

public void setBrakeMode(boolean brake){
    if(brake){
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
    } else{
        this.driveMotor.setNeutralMode(NeutralMode.Coast);
    }
   
}

public void stop() {
    driveMotor.set(ControlMode.PercentOutput,0);
    turningMotor.set(ControlMode.PercentOutput,0);
}

}


