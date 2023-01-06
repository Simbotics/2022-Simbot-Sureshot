package frc.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.imaging.SimLimelight;
import frc.imaging.SimLimelightTarget;
import frc.imaging.SimLimelight.LimelightTargetType;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriveConstants;
import frc.util.PIDConstants;
import frc.util.Vect;
import frc.util.Swerve.SwerveDrive;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.util.Color;


public class IO {

	private static IO instance;

	//Sensors

	private DriverStation driverStation;
	private PowerDistribution pdp;
	


	private double lastTime = 0.0;
	private double deltaTime = 10.0;

	private boolean firstCycle = true;

	private DriverStation.Alliance alliance;
	private double matchTime;

	private long SystemTimeAtAutonStart = 0;
	private long timeSinceAutonStarted = 0;

	private Vect lastPose = new Vect(0,0);
	private Vect GoalPose = new Vect(0,0);
	private double xVelocity = 0;
	private double yVelocity = 0;
	


	private int[] turningEncPorts;
	private double[] moduleOffsets;

	
	private SimLimelight limelight;

	private SimLimelight limelightColourSensor;

	private DigitalInput intakeBottomLightSensor;
	private DigitalInput intakeTopLightSensor;
	

	//MOTORS 

	//private TalonFX intake;
	private VictorSP intake;
	//private TalonFX indexerBottom;
	private VictorSP indexerBottom;
	private TalonFX indexerTop;

	private TalonFX shooterTop;
	private TalonFX shooterBottom;

	private TalonFX climberLeft;
	private TalonFX climberRight;

	private int[] drivingPorts;
	private int[] steeringPorts;

	private Solenoid intakeWristUp;
	private Solenoid intakeWristDown;

	private Solenoid hangarUp;
	private Solenoid hangarDown;

	private Solenoid shooterExtendOut;
	private Solenoid shooterExtendIn;

	private Solenoid shooterLockIn;
	private Solenoid shooterLockOut;
	
	private SwerveDrive swerveDrive;

	private Spark ledStrip;

	private StatorCurrentLimitConfiguration oldCurrentLimitConfig = new StatorCurrentLimitConfiguration(false, 0, 0, 0);

	private double climberOldRampRate = 0.5;
	private boolean intakeWrist = false;
	private boolean climberCylinder = false;
	private boolean shooterHoodCylinder = false;
	private boolean shooterLockCylinder = true;
	private SlewRateLimiter intakeLimiter;

	

	private IO() {
		this.pdp = new PowerDistribution();
		this.driverStation = DriverStation.getInstance();
		
		this.limelight = SimLimelight.getInstance(0);
		this.limelightColourSensor = SimLimelight.getInstance(1);
		this.intakeBottomLightSensor = new DigitalInput(7);
		this.intakeTopLightSensor = new DigitalInput(6);
		this.turningEncPorts = new int[]{0,1,2,3};

		// Motors
		this.intake = new VictorSP(2);
		this.indexerBottom = new VictorSP(3);
		
		this.indexerTop = new TalonFX(15);
		this.shooterTop = new TalonFX(2);
		this.shooterBottom = new TalonFX(3);

		this.climberLeft = new TalonFX(13);
		this.climberRight = new TalonFX(14);
		
		this.steeringPorts = new int[]{4,5,6,7};
		this.drivingPorts = new int[]{8,9,10,11};

		// LEDS
		this.ledStrip = new Spark(1);

		this.intakeLimiter = new SlewRateLimiter(1.5);
		// Solenoids

		this.intakeWristUp = new Solenoid(PneumaticsModuleType.CTREPCM , 0);
		this.intakeWristDown = new Solenoid(PneumaticsModuleType.CTREPCM , 1);

		this.hangarUp = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
		this.hangarDown = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

		this.shooterExtendIn = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
		this.shooterExtendOut = new Solenoid(PneumaticsModuleType.CTREPCM, 5);

		this.shooterLockIn = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
		this.shooterLockOut = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

	

		


		int[] steeringPorts = new int[]{DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackRightTurningMotorPort};
        int[] drivingPorts = new int[]{DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackRightDriveMotorPort};
        int[] encoderPorts = new int[]{DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, DriveConstants.kFrontRightDriveAbsoluteEncoderPort, DriveConstants.kBackLeftDriveAbsoluteEncoderPort, DriveConstants.kBackRightDriveAbsoluteEncoderPort};
        boolean[] drivingReversed = new boolean[]{DriveConstants.kFrontLeftDriveMotorReversed, DriveConstants.kFrontRightDriveMotorReversed, DriveConstants.kBackLeftDriveMotorReversed, DriveConstants.kBackRightDriveMotorReversed};
        boolean[] turningReversed = new boolean[]{DriveConstants.kFrontLeftTurningEncoderReversed, DriveConstants.kFrontRightTurningEncoderReversed, DriveConstants.kBackLeftTurningEncoderReversed, DriveConstants.kBackRightTurningEncoderReversed};
        double[] moduleOffsets = new double[]{DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad};
        boolean[] absoluteReversed = new boolean[]{DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed, DriveConstants.kFrontRightDriveAbsoluteEncoderReversed, DriveConstants.kBackLeftDriveAbsoluteEncoderReversed, DriveConstants.kBackRightDriveAbsoluteEncoderReversed};

        this.swerveDrive = new SwerveDrive(steeringPorts, drivingPorts, encoderPorts, drivingReversed, turningReversed, moduleOffsets, absoluteReversed);

		this.configureSpeedControllers();


		

		this.reset();
	}

	public static IO getInstance() {
		if (instance == null) {
			instance = new IO();
		}
		return instance;
	}

	public void reset() {
		this.firstCycle = true;
		//this.resetIndexerBottomEncoder();
		this.resetClimbEncoder();
				

	
	}

	public void configureSpeedControllers() {
		shooterTop.configFactoryDefault();
		shooterBottom.configFactoryDefault();
		//intake.configFactoryDefault();
		//indexerBottom.configFactoryDefault();
		indexerTop.configFactoryDefault();
		//TODO not sure whether the top/bottom motor is inverted
		shooterBottom.setInverted(false);

		shooterTop.setNeutralMode(NeutralMode.Coast);
		shooterBottom.setNeutralMode(NeutralMode.Coast);
		shooterTop.configPeakOutputReverse(0);
		shooterBottom.configPeakOutputReverse(0);
		//intake.setNeutralMode(NeutralMode.Coast);
		//indexerBottom.setNeutralMode(NeutralMode.Brake);
		indexerTop.setNeutralMode(NeutralMode.Brake);
		indexerTop.setInverted(true);

		this.climberRight.setNeutralMode(NeutralMode.Brake);
		this.climberLeft.setNeutralMode(NeutralMode.Brake);
		this.climberLeft.setSensorPhase(false);
		this.climberLeft.setInverted(false);
		this.climberRight.follow(this.climberLeft);
		this.climberRight.setInverted(true);
		this.climberLeft.configVoltageCompSaturation(11);
		this.climberLeft.enableVoltageCompensation(true);

		this.climberLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 30, 35, 100));
		//intake.configOpenloopRamp(0.4);
		//indexerBottom.configOpenloopRamp(0.0);
		//indexerBottom.configClosedloopRamp(0.2);
		this.climberLeft.configOpenloopRamp(0.5);

		this.climberRight.setStatusFramePeriod(StatusFrame.Status_1_General, 300);
		this.climberRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 300);

		//this.indexerBottom.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 300);
		this.indexerTop.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 300);
		//this.intake.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
		
	}

	public void resetClimbEncoder(){
		this.climberLeft.setSelectedSensorPosition(0);
	}


	public void resetIndexerTopEncoder(){
		this.indexerTop.setSelectedSensorPosition(0);
	}

	public double getIndexerTopEncoder(){
		return this.indexerTop.getSelectedSensorPosition();
	}

	public double getClimberCurrent(){
		return (this.pdp.getCurrent(12) + this.pdp.getCurrent(13)) / 2.0;
	}

	


	public void setClimberCurrentLimit(boolean enabled, double limit, double peak, double peakTime){
		StatorCurrentLimitConfiguration newCurrent = new StatorCurrentLimitConfiguration(enabled, limit, peak, peakTime);
		if(newCurrent.enable != oldCurrentLimitConfig.enable || newCurrent.currentLimit != oldCurrentLimitConfig.currentLimit
		 ||newCurrent.triggerThresholdCurrent != oldCurrentLimitConfig.triggerThresholdCurrent ||newCurrent.triggerThresholdTime != oldCurrentLimitConfig.triggerThresholdTime){
			this.climberLeft.configStatorCurrentLimit(newCurrent);
		 } 

		 this.oldCurrentLimitConfig = newCurrent;
	}

	public void setClimberRampRate(double rampRate){
		if(this.climberOldRampRate != rampRate){
			this.climberLeft.configOpenloopRamp(rampRate);
		}
		this.climberOldRampRate = rampRate;
	}

	public void update() {

		
		if (this.lastTime == 0.0) {
			this.deltaTime = 10;
			this.lastTime = System.currentTimeMillis();
		} else {
			this.deltaTime = System.currentTimeMillis() - this.lastTime;
			this.lastTime = System.currentTimeMillis();
		}

		if (this.driverStation.isAutonomous()) {
			this.timeSinceAutonStarted = System.currentTimeMillis() - this.SystemTimeAtAutonStart;
			SmartDashboard.putNumber("12_Time Since Auto Start:", this.timeSinceAutonStarted);
		}

		if (this.firstCycle) {
			this.firstCycle = false;
		
		
		

		
		}
		
		this.alliance = this.driverStation.getAlliance();
		this.matchTime = this.driverStation.getMatchTime();

		

		
		this.swerveDrive.updateOdometry();


		this.xVelocity = (this.swerveDrive.getPose().getX() - this.lastPose.getX()) * (1000.0 / deltaTime) ; // Convert to meters per second 
		this.yVelocity = (this.swerveDrive.getPose().getY() - this.lastPose.getY()) * (1000.0 / deltaTime);

		this.lastPose = new Vect(this.swerveDrive.getPose().getX(), this.swerveDrive.getPose().getY());


		this.limelight.calculate();
		if(this.limelight.getTargetExists()){
			SmartDashboard.putNumber("VISION DISTANCE", this.getVisionDistanceFeet());
		}
		SmartDashboard.putBoolean("LIGHT SENSOR", this.getIntakeBottomLightSensor());
		SmartDashboard.putBoolean("LIGHT SENSOR TOP", this.getIntakeTopLightSensor());
		SmartDashboard.putNumber("HANGER", this.climberLeft.getSelectedSensorPosition());
		SmartDashboard.putNumber("HANGER SPEED", this.getClimberSpeed());
		SmartDashboard.putNumber("TOP SHOOTER RPM", this.getShooterTopRPM());
		SmartDashboard.putNumber("BOTTOM SHOOTER RPM", this.getShooterBottomRPM());
		//SmartDashboard.putNumber("INDEXER RPM", this.getIndexerBottomRPM());
		

		SmartDashboard.putNumber("X VELOCITY", this.xVelocity);
		SmartDashboard.putNumber("Y VELOCITY", this.yVelocity);
		SmartDashboard.putNumber("X Position", this.swerveDrive.getX());
		SmartDashboard.putNumber("Y Position", this.swerveDrive.getY());

		SmartDashboard.putBoolean("Blue Ball", this.getBlueBall());
		SmartDashboard.putBoolean("Red Ball", this.getRedBall());

		SmartDashboard.putNumber("Goal X", this.GoalPose.getX());
		SmartDashboard.putNumber("Goal Y", this.GoalPose.getY());
		SmartDashboard.putString("ALLIANCE: ",this.alliance.toString());
	
	}

	public double getMatchTimeLeft() {
		return this.matchTime;
	}

	public void resetAutonTimer() {
		this.SystemTimeAtAutonStart = System.currentTimeMillis();
	}


	// PDP //

	public double getVoltage() {
		return this.pdp.getVoltage();
	}

	public double getCurrent(int port) {
		return this.pdp.getCurrent(port);
	}

	public double getDeltaTime(){
		return this.deltaTime;
	}

	
	// LIMELIGHT
	public boolean getVisionTargetExists() {
		return this.limelight.getTargetExists();
	}

	public void setPipeline(int pipeline){
		this.limelight.setPipeline(pipeline);
	}

	public double getVisionTargetAngle() {
		return this.limelight.getVisionTargetAngle();
	}

	public void setLimelightTarget(LimelightTargetType type) {
		this.limelight.setLimelight(type);
	}

	public LimelightTargetType getLimelightTarget() {
		return this.limelight.getLimelightState();
	}

	public double getVisionTargetX() {
		return this.limelight.getTargetX();
	}

	public double getVisionTargetRotation() {
		return this.limelight.getTargetRotation();
	}

	public double getVisionTargetY() {
		return this.limelight.getTargetY();
	}

	public double getVisionTargetArea() {
		return this.limelight.getTargetArea();
	}

	public double getVisionDistanceInches(){
		//d = (h2-h1) / tan(a1+a2)
		if(this.limelight.getTargetExists()){
			return 26.69 + (RobotConstants.GOAL_HEIGHT_INCHES - RobotConstants.CAMERA_HEIGHT_INCHES) / (Math.tan(Math.toRadians(RobotConstants.CAMERA_TILT_DEGREES + this.limelight.getTargetY())));
		} else {
			return 72.0;
		}
	}	

	public double getVisionDistanceFeet(){
		return getVisionDistanceInches() / 12.0;
	}

	
	//MOTORS

	// Intake
	public boolean getIntakeBottomLightSensor() {
		return this.intakeBottomLightSensor.get();
	}

	public boolean getIntakeTopLightSensor() {
		return this.intakeTopLightSensor.get();
	}

	public boolean getRedBall(){
		return this.limelightColourSensor.getTargetExists() && this.intakeTopLightSensor.get();
	}

	public boolean getBlueBall(){
		return !this.limelightColourSensor.getTargetExists() && this.intakeTopLightSensor.get();
	}

	public boolean getCorrectBall(){
		if(this.alliance == Alliance.Red){
			return getRedBall();
		} else {
			return getBlueBall();
		}
		
	}

	


	public int getIndexerBallCount(){
		if(this.intakeBottomLightSensor.get() && this.intakeTopLightSensor.get()){
			return 2;
		} else if(this.intakeTopLightSensor.get()){
			return 1;
		} else {
			return 0;
		}
	}

	public int[] getTurningEncPorts(){
		return this.turningEncPorts;
	}

	public double[] getModuleOffsets(){
		return this.moduleOffsets;
	}

	public void setIntakeMotor(double speed){
		//this.intake.set(ControlMode.PercentOutput, speed);
		this.intake.set(this.intakeLimiter.calculate(speed));
		
	}

	public void setIntakeWrist(boolean out){
		if(this.intakeWrist != out){
			this.intakeWristUp.set(!out);
			this.intakeWristDown.set(out);
			this.intakeWrist = out;
		}
		
	}
	public void setHangar(boolean out){
		if(this.climberCylinder != out){
			this.hangarUp.set(!out);
			this.hangarDown.set(out);
			this.climberCylinder = out;
		}
		

	}

	public double getHangerPostion(){
		return this.climberLeft.getSelectedSensorPosition();
	}
	public void setShooterLock(boolean out){
		if(this.shooterLockCylinder != out){
			this.shooterLockIn.set(!out);
			this.shooterLockOut.set(out);
			this.shooterLockCylinder = out;
		}
	
	}

	public void setShooterExtend(boolean out){
		if(this.shooterHoodCylinder != out){
			this.shooterExtendIn.set(!out);
			this.shooterExtendOut.set(out);
			this.shooterHoodCylinder = out;
		}
		
	}
	
	public void setIndexerBottom(double output){
		//this.indexerBottom.set(ControlMode.PercentOutput, output);
		this.indexerBottom.set(output);
	}

	public void setIndexerTop(double output){
		this.indexerTop.set(ControlMode.PercentOutput, output);
	}
	
	public void setShooterTopMotor(double speed){
		this.shooterTop.set(ControlMode.PercentOutput, speed);
		
	}


	public void setShooterBottomMotor(double speed){
		this.shooterBottom.set(ControlMode.PercentOutput, speed);
		
	}
	public void resetClimberEncoder(){
		this.climberLeft.setSelectedSensorPosition(0);
	}
	
	public void configureIndexerTopPID(PIDConstants constants){
		this.indexerTop.config_kP(0, constants.p);
		this.indexerTop.config_kI(0, constants.i);
		this.indexerTop.config_kD(0, constants.d);
		this.indexerTop.config_IntegralZone(0, 3000);
		this.indexerTop.configAllowableClosedloopError(0, (int) constants.eps);
		this.indexerTop.config_kF(0, constants.ff);
	}
	public void configureShooterTopPID(PIDConstants constants){
		this.shooterTop.config_kP(0, constants.p);
		this.shooterTop.config_kI(0, constants.i);
		this.shooterTop.config_kD(0, constants.d);
		this.shooterTop.config_IntegralZone(0, 3000);
		this.shooterTop.configAllowableClosedloopError(0, (int) constants.eps);
		this.shooterTop.config_kF(0, constants.ff);
	}

	public void configureShooterBottomPID(PIDConstants constants){
		this.shooterBottom.config_kP(0, constants.p);
		this.shooterBottom.config_kI(0, constants.i);
		this.shooterBottom.config_kD(0, constants.d);
		this.shooterBottom.config_IntegralZone(0, 3000);
		this.shooterBottom.configAllowableClosedloopError(0, (int) constants.eps);
		this.shooterBottom.config_kF(0, constants.ff);
	}

	private void setShooterTopVelocity(double speed){
		this.shooterTop.selectProfileSlot(0, 0);
		this.shooterTop.set(ControlMode.Velocity, speed);
		
		
	}
	

	private void setShooterBottomVelocity(double speed){
		this.shooterBottom.set(ControlMode.Velocity, speed);
		
	}
	
	private void setIndexerTopVelocity(double speed){
		this.indexerTop.set(ControlMode.Velocity, speed);
		
	}
	public void setindexerTopRPM(double rpm){
		this.setIndexerTopVelocity((rpm * RobotConstants.SHOOTER_TICKS_PER_REV / 60.0) / 10.0);
		
	}

	public void setShooterTopRPM(double rpm){
		this.setShooterTopVelocity((rpm * RobotConstants.SHOOTER_TICKS_PER_REV / 60.0) / 10.0);
		
	}

	public void setShooterBottomRPM(double rpm){
		this.setShooterBottomVelocity((rpm * RobotConstants.SHOOTER_TICKS_PER_REV / 60.0) / 10.0);
		
	}

	public void setClimberOutput(double output){
		this.climberLeft.set(ControlMode.PercentOutput, output);
	}

	public double getShooterTopRPM(){
		return ((this.shooterTop.getSelectedSensorVelocity() / RobotConstants.SHOOTER_TICKS_PER_REV * 60.0) * 10.0); 
	}

	public double getShooterBottomRPM(){
		return ((this.shooterBottom.getSelectedSensorVelocity() / RobotConstants.SHOOTER_TICKS_PER_REV * 60.0) * 10.0); 
	}

	
	public double getIndexerTopRPM(){
		return ((this.indexerTop.getSelectedSensorVelocity() / RobotConstants.SHOOTER_TICKS_PER_REV * 60.0) * 10.0); 
	}

	public double getClimberSpeed(){
		return this.climberLeft.getSelectedSensorVelocity();
	}
	public int[] getSteeringPorts(){
		return this.steeringPorts;
	}

	public int[] getDrivingPorts(){
		return this.drivingPorts;
	}

	

	public void drive(double xSpeed, double ySpeed, double turningSpeed){
		this.swerveDrive.drive(xSpeed, ySpeed, turningSpeed);
	}

	public void setFieldOriented(boolean fieldOriented){
		this.swerveDrive.setFieldOriented(fieldOriented);
	}

	public void setDriveBrakeMode(boolean brake){
		this.swerveDrive.setBrakeMode(brake);
	}

	public double getHeading(){
		return this.swerveDrive.getHeading();
	}

	public double getGyroAngle(){
		return this.swerveDrive.getGyroAngle();
	}

	public double getGyroSpeed(){
		return this.swerveDrive.getGyroSpeed();
	}

	public void zeroHeading(){
		this.swerveDrive.zeroHeading();
	}

	public double getXVelocity(){
		return this.xVelocity;
	}

	public double getYVelocity(){
		return this.yVelocity;
	}

	public Pose2d getRobotPose(){
		return this.swerveDrive.getPose();
	}

	public double getShotVelocity(){
		return 4.36; // use current shooter rpms to roughly estimate this later
	}


	public void updateGoalPose(double distanceToGoal, double angleToGoal){
		this.GoalPose = new Vect(this.swerveDrive.getPose().getX(), this.swerveDrive.getPose().getY()).add(new Vect(distanceToGoal,this.getHeading()+angleToGoal,0));
	}

	public Vect getGoalPose(){
		return this.GoalPose;
	}
	
	public void setDrivePose(double x, double y, double theta){
		this.swerveDrive.zeroHeading();
		this.swerveDrive.setAutoStartAngle(theta);
		this.swerveDrive.resetOdometry(x, y);

	}

	public void resetAutoStartAngle(){
		this.swerveDrive.setAutoStartAngle(0);
	}

	public void setLEDStrip(double pattern){
		ledStrip.set(pattern);
	}


	public void stopAll() {
		this.intake.set(0.0);
		this.shooterTop.set(ControlMode.PercentOutput, 0);
		this.shooterBottom.set(ControlMode.PercentOutput, 0);
		this.setClimberOutput(0);
		// shut off things here
	}
	
}