package frc.util.Swerve; 

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriveConstants;
import frc.util.PIDConstants;
import frc.util.SimNavx;
import frc.util.SimPID;
import frc.util.Vect;

//Wheels are a array numbered 0-3 from front to back, with even numbers on the left side when
//  facing forward.
public class SwerveDrive {

    private SimNavx gyro; 
    private int wheelCount = 4;
    private SwerveModule[] modules = new SwerveModule[wheelCount];; 
    private boolean isFieldOriented = true;
    private int moduleZeroed = 0;
    private double autoStartAngle = 0;
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(0));

    public SwerveDrive(int[] steeringPorts, int[] drivingPorts,
        int[] encoderPorts, boolean[] drivingReversed, boolean [] turningReversed, double[] moduleOffsets, boolean[] absoluteReversed){
            
            for(int i =0; i < wheelCount; i++){
                this.modules[i] = new SwerveModule(steeringPorts[i], drivingPorts[i], drivingReversed[i], turningReversed[i], encoderPorts[i], moduleOffsets[i],  absoluteReversed[i]);
            }

            this.gyro = new SimNavx();
           

            
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        modules[0].setDesiredState(desiredStates[0]);
        modules[1].setDesiredState(desiredStates[1]);
        modules[2].setDesiredState(desiredStates[2]);
        modules[3].setDesiredState(desiredStates[3]);
    }

    

    /**
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
    
  public void drive(double xSpeed, double ySpeed, double turningSpeed){
    ChassisSpeeds chassisSpeeds;

    xSpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

    turningSpeed *= DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    if (this.isFieldOriented) {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, this.getRotation2d());
    } else {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    this.setModuleStates(moduleStates);



  }

  public void updateOdometry(){
    this.gyro.update();
    odometer.update(getRotation2d(), modules[0].getState(), modules[1].getState(), modules[2].getState(),
    modules[3].getState());

    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    
    for(int i = 0; i < modules.length; i++){
      modules[i].updateDashboard();
    }
  }


  public double getModuleTLRaw(){
    return modules[0].getAbsoluteEncoderRad();
  }

  public double getModuleTRRaw(){
    return modules[1].getAbsoluteEncoderRad();
  }

  public double getModuleBLRaw(){
    return modules[2].getAbsoluteEncoderRad();
  }

  public double getModuleBRRaw(){
    return modules[3].getAbsoluteEncoderRad();
  }


public double getGyroSpeed(){
  return this.gyro.getGyroSpeed();
}


public void zeroHeading() {
  gyro.reset();
}


public Pose2d getPose() {
  return odometer.getPoseMeters();
}

public void resetOdometry(){
  odometer.resetPosition(new Pose2d(0, 0, new Rotation2d()),getRotation2d());
}

public void resetOdometry(Pose2d pose) {
  odometer.resetPosition(pose, getRotation2d());
}

public void resetOdometry(double x, double y){
  odometer.resetPosition(new Pose2d(x, y,  getRotation2d()), getRotation2d());
}

public void setAutoStartAngle(double angle){
  this.autoStartAngle = angle;
}

public double getX(){
  return this.odometer.getPoseMeters().getX();
}

public double getY(){
  return this.odometer.getPoseMeters().getY();
}



public void setFieldOriented(boolean fieldOriented){
  this.isFieldOriented = fieldOriented;
}

public void setBrakeMode(boolean brake){
  for(int i = 0; i < modules.length; i++){
    modules[i].setBrakeMode(brake);
  }
}


public double getHeading() {
//System.out.println("HEADING" + this.autoStartAngle);
  return gyro.getAngle() + this.autoStartAngle;
}

public Rotation2d getRotation2d() {
  return Rotation2d.fromDegrees(getHeading());
  
}

public double getGyroAngle(){
  return getHeading();
}


}
  
