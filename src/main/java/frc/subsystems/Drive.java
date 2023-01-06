package frc.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.imaging.SimLimelight.LimelightTargetType;
import frc.robot.RobotConstants;
import frc.subsystems.LEDStrip.LEDColourState;
import frc.util.SimPID;
import frc.util.Vect;


public class Drive extends SubsystemBase{

    private static Drive instance;
    private DriveState currentDriveState;
    private SimPID headingPID;
    private SimPID xPID;
    private SimPID yPID;
    private double forward;
    private double strafe;
    private double azimuth;
    private boolean lockAngleFlag = true;
    private boolean resetHeading = true;
    private LEDColourState desiredLedState = LEDColourState.OFF;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter turningLimiter;
    private double aimingOffset = 1;
    private double aimingPresetAngle = 0;
    private boolean colorSensorEnabled = false;


    public Drive (){
        super();
    }

    public enum DriveState {OUTPUT,  LIMELIGHT_GOAL_LOCK, MANUAL,DRIVE_BY, PRESET_LOCK};

    public void setState(DriveState newState){
        this.currentDriveState = newState;
    }

    public DriveState getState(){
        return this.currentDriveState;
    }

    public void setOutput(double iforward, double istrafe, double iazimuth){
        this.forward = iforward;
        this.strafe = istrafe;
        this.azimuth = iazimuth;
    }

    public void setTargetAngle(double targetAngle){
        this.aimingPresetAngle = targetAngle;
    }

    public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}

		return instance;
	}

    public void firstCycle(){
        
        this.currentDriveState = DriveState.OUTPUT; 
        this.headingPID =  new SimPID(RobotConstants.getHeadingPID());
    
        this.headingPID.setIRange(2);
        
        
        this.xPID = new SimPID(RobotConstants.getDrivePID());
        this.yPID = new SimPID(RobotConstants.getDrivePID());

        this.xLimiter = new SlewRateLimiter(1.25);
		this.yLimiter = new SlewRateLimiter(1.25);
		this.turningLimiter = new SlewRateLimiter(4.0);


        this.yLimiter.calculate(0);
        this.xLimiter.calculate(0);
        this.turningLimiter.calculate(0);

        this.yLimiter.reset(0);
        this.xLimiter.reset(0);
        this.turningLimiter.reset(0);

        this.xPID.setIRange(0.15);
        this.yPID.setIRange(0.15);

        this.xPID.setMaxOutput(1.0);
        this.yPID.setMaxOutput(1.0);

        this.lockAngleFlag = true;
    }

    public void calculate(){
        SmartDashboard.putNumber("GYRO HEADING", this.io.getHeading());
        SmartDashboard.putNumber("GYRO ANGLE", this.io.getGyroAngle());
        SmartDashboard.putNumber("GYRO SPEED", this.io.getGyroSpeed());
        switch (this.currentDriveState){
            case OUTPUT:
                this.desiredLedState = LEDColourState.OFF;
                this.io.drive(this.forward, this.strafe, this.azimuth);
                this.io.setFieldOriented(true); 
                this.io.setDriveBrakeMode(false);  
                break;
            case PRESET_LOCK:
                this.desiredLedState = LEDColourState.OFF;
                this.io.setFieldOriented(true); 
                this.io.setDriveBrakeMode(false); 
                this.headingPID.enableContinuousInput(-180, 180);
                this.headingPID.setDesiredValue(this.aimingPresetAngle);
                this.io.drive(this.forward, this.strafe, this.headingPID.calcPID(this.io.getHeading()));
                break;
            case MANUAL:     
                this.desiredLedState = LEDColourState.OFF;
                this.io.drive(this.forward, this.strafe, this.azimuth);
                this.io.setFieldOriented(false); 
                this.io.setDriveBrakeMode(false);   
                break; 
            case LIMELIGHT_GOAL_LOCK:
                this.io.setFieldOriented(true);
                this.io.setDriveBrakeMode(true);
                if(this.io.getVisionTargetExists()){
                    double theta = this.io.getHeading() - this.io.getVisionTargetX();
                    if(this.isBallCorrect()){
                        this.headingPID.setDesiredValue(theta + this.getAimOffset());
                    } else if(this.io.getIntakeTopLightSensor()) {
                        this.headingPID.setDesiredValue(theta + this.getAimOffset() + RobotConstants.WRONG_BALL_OFFSET);
                    } else {
                        this.headingPID.setDesiredValue(theta + this.getAimOffset());
                    }
                    
                    if(this.isAimed()){
                        this.io.updateGoalPose(Units.feetToMeters(this.io.getVisionDistanceFeet()), this.io.getVisionTargetX());
                        if(this.io.getVisionDistanceFeet() > 11.0 && this.io.getVisionDistanceFeet() < 22.5){
                            this.desiredLedState = LEDColourState.VISION_AIMED;
                        } else {
                            this.desiredLedState = LEDColourState.VISION_TOO_CLOSE;
                        }
                    } else {
                        this.desiredLedState = LEDColourState.VISION_TARGET;
                    }
                    this.io.drive(this.forward, this.strafe, this.headingPID.calcPID(this.io.getHeading()));
                } else {
                    double goalAngle = Math.toDegrees(Math.atan2(this.io.getGoalPose().getY() - this.io.getRobotPose().getY(), 
                    this.io.getGoalPose().getX() - this.io.getRobotPose().getX()));
                    SmartDashboard.putNumber("Goal Angle: ", goalAngle);
                    this.headingPID.enableContinuousInput(-180, 180);
                    this.headingPID.setDesiredValue(goalAngle);
                    
                    this.io.drive(this.forward, this.strafe,this.azimuth); //this.headingPID.calcPID(this.io.getHeading()));
                    this.desiredLedState = LEDColourState.VISION_NO_TARGET;
                }
                break;
            case DRIVE_BY:
                this.io.setFieldOriented(true);
                this.io.setDriveBrakeMode(true);
                if(this.io.getVisionTargetExists()){
                    this.io.updateGoalPose(Units.feetToMeters(this.io.getVisionDistanceFeet()), this.io.getVisionTargetX());
                    Vect deltaToGoal = new Vect(this.io.getGoalPose());
                    Vect robotVelocity = new Vect(this.io.getXVelocity() ,this.io.getYVelocity());
                    double deltaTime = this.aimAhead(deltaToGoal, robotVelocity,this.io.getShotVelocity());

                    Vect aimPoint;
                    if(deltaTime > 0){
                        aimPoint = deltaToGoal.add(robotVelocity.scalarMult(deltaTime+0.0/1000.0));
                    } else {
                        aimPoint = deltaToGoal;
                    }

                    double goalAngle = Math.toDegrees(Math.atan2(aimPoint.getY() - this.io.getRobotPose().getY(), 
                    aimPoint.getX() - this.io.getRobotPose().getX()));
                    SmartDashboard.putNumber("Goal Angle: ", goalAngle);
                    this.headingPID.enableContinuousInput(-180, 180);
                    this.headingPID.setDesiredValue(goalAngle);
                    
                    double boundedAngle = MathUtil.inputModulus(this.io.getHeading(), -180, 180);
                    
                    if(Math.abs(goalAngle - boundedAngle) < 3){
                        if(this.io.getVisionDistanceFeet() > 10.5){
                            this.desiredLedState = LEDColourState.VISION_AIMED;
                        } else {
                            this.desiredLedState = LEDColourState.VISION_TOO_CLOSE;
                        }
                    } else {
                        this.desiredLedState = LEDColourState.VISION_TARGET;
                    }
                    this.io.drive(this.forward*0.2, this.strafe*0.2, this.headingPID.calcPID(this.io.getHeading()));
                } else {
                    double goalAngle = Math.toDegrees(Math.atan2(this.io.getGoalPose().getY() - this.io.getRobotPose().getY(), 
                    this.io.getGoalPose().getX() - this.io.getRobotPose().getX()));
                    SmartDashboard.putNumber("Goal Angle: ", goalAngle);
                    this.headingPID.enableContinuousInput(-180, 180);
                    this.headingPID.setDesiredValue(goalAngle);
                    
                    this.io.drive(this.forward*0.2, this.strafe*0.2, this.headingPID.calcPID(this.io.getHeading()));
                    this.desiredLedState = LEDColourState.VISION_NO_TARGET;
                }
                break;
            default:
                break;
            
        }
        
        

    }


    public boolean isAimed(){
        if(this.currentDriveState == DriveState.LIMELIGHT_GOAL_LOCK){
            if(this.io.getVisionTargetExists()){
                if(this.io.getVisionTargetX() - this.getAimOffset() < 3.0 && (this.io.getVisionTargetX() - this.getAimOffset()) > -3.0){ // Must be within 2 degrees of target
                    return true;
                } else {
                    return false;
                }
            } else { // NO TARGETS!??
                return false;
            }
        } else {
            return false;
        }
    }

    public boolean isStill(){
        return (Math.abs(this.io.getGyroSpeed()) < 0.1) && (Math.abs(this.io.getXVelocity()) < 0.05) 
         && (Math.abs(this.io.getYVelocity()) < 0.05);
    }

    public LEDColourState getDesiredLedState() {
        return this.desiredLedState;
    }

    public double getAimOffset(){
        if(this.io.getVisionTargetExists()){
            if(this.io.getVisionDistanceFeet() < 10.6){
                return 2.4;
            } else if(this.io.getVisionDistanceFeet() > 19.0){
                return 0.5;
            } else {
                return  -0.165 * this.io.getVisionDistanceFeet() + 4.15;
            }
        } else{
            return 0;
        }
    }

    

    

    public void resetGyro(){
        this.io.zeroHeading();
        this.io.resetAutoStartAngle();
        this.lockAngleFlag = true;
    }

    public boolean DriveToPointLimeLightAngle(double x, double y, double theta, double minVelocity, 
    double maxVelocity, double turnSpeed, double eps) {

        this.io.setLimelightTarget(LimelightTargetType.GOAL);
        if(this.io.getVisionTargetExists()){
            return DriveToPoint(x, y, this.io.getHeading() - this.io.getVisionTargetX() + this.getAimOffset(), minVelocity, maxVelocity, turnSpeed, eps);
        } else {
            return DriveToPoint(x, y, theta, minVelocity, maxVelocity, turnSpeed, eps);
        }
    }



    public boolean DriveToPoint(double x, double y, double theta, double minVelocity, double maxVelocity,
			double turnSpeed, double eps) {
	
		this.headingPID.setMaxOutput(turnSpeed);
        this.headingPID.setFinishedRange(0.5);
        this.headingPID.setIRange(2);
		this.headingPID.setDesiredValue(theta);
        this.xPID.setFinishedRange(eps);
        this.yPID.setFinishedRange(eps);
        this.io.setFieldOriented(true);

		double yError = y - this.io.getRobotPose().getY();
        double xError = x - this.io.getRobotPose().getX();

       
		
        Vect vectError = new Vect(xError, yError);
        

        double dist = vectError.mag();

        vectError = vectError.unit();
      
        vectError = vectError.scalarMult(maxVelocity);

        this.yPID.setMaxOutput(Math.abs(vectError.getY())); // scales so always at least 1 is going the max velocity 
        this.xPID.setMaxOutput(Math.abs(vectError.getX()));

		double yOutput = this.yPID.calcPIDError(yError);
        double xOutput = this.xPID.calcPIDError(xError);


        Vect vectOutput = new Vect(xOutput, yOutput);

        if(vectOutput.mag() < minVelocity){ // if the overall vector speed is less than the minimum 
            vectOutput = vectOutput.unit(); // get the unit vector

            vectOutput = vectOutput.scalarMult(minVelocity); // scale it to the min
        }

        yOutput = vectOutput.getY();
        xOutput = vectOutput.getX();
        double turningOutput = this.headingPID.calcPID(this.io.getHeading());

        yOutput = this.yLimiter.calculate(yOutput);
        xOutput = this.xLimiter.calculate(xOutput);
        turningOutput = this.turningLimiter.calculate(turningOutput);


		boolean isDone = false;
		if (minVelocity <= 0.01) {
			if (this.xPID.isDone() && this.yPID.isDone()) {
                
				isDone = true;
                this.io.drive(0, 0, 0);
                this.turningLimiter.reset(0);
                this.xLimiter.reset(0);
                this.yLimiter.reset(0);
				// slew ramp reset
			} else { // not done so drive
                this.io.drive(xOutput, yOutput, turningOutput);
            }
		} else if (dist < eps) { 
			isDone = true;
		} else{
            this.io.drive(xOutput, yOutput, turningOutput);
        }

     
		return isDone;
	}


    public void setColorSensorEnabled(boolean enabled){
        this.colorSensorEnabled = enabled;
    }


    public double aimAhead(Vect deltaP, Vect deltaV, double muzzleV){
        double a = (deltaV.dot(deltaV)) - muzzleV*muzzleV;
        double b = 2.0 * deltaV.dot(deltaP);
        double c = deltaP.dot(deltaP);
        
        double desc= b*b - 4.0*a*c;
        
        if(desc > 0){
            return 2.0*c / (Math.sqrt(desc) - b);
        } else {
            return -1.0;
        }
    }

    

    public void resetRateLimit(){
        this.yLimiter.reset(0);
        this.xLimiter.reset(0);
        this.turningLimiter.reset(0);
    }

    public boolean isBallCorrect(){
        if(this.colorSensorEnabled){
            return this.io.getCorrectBall();
        } else {
            return true;
        }
    }


   public void disable(){
        this.io.drive(0, 0, 0);
   }
}