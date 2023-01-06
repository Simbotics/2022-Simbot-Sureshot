package frc.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.imaging.SimLimelight.LimelightTargetType;
import frc.robot.RobotConstants;
import frc.subsystems.LEDStrip.LEDColourState;
import frc.util.Vect;

public class Shooter extends SubsystemBase  {
    private static Shooter instance;
    private ShooterState currentShooterState;
    private HoodState currentHoodState;
    private HoodState prevHoodState;
    private double shooterTopLastTargetRPM;
    private double shooterTopTargetRPM;
    private double shooterBottomLastTargetRPM;
    private double shooterBottomTargetRPM;
    private LEDColourState desiredLedState = LEDColourState.OFF;
    
    private int lockingCycles = 0;
    private int extendingCycles = 0;

    public enum ShooterState{OFF, FENDER, INTAKING, LAUNCH_PAD, LIMELIGHT_SHOOTING,LIMELIGHT_MOVING,LOW_GOAL}
    
    public enum HoodState{OPEN,OPENING,CLOSED,CLOSING}

    private Shooter() {
        super();
        this.shooterTopTargetRPM = 0;
        this.shooterTopLastTargetRPM = 0;
        this.shooterBottomTargetRPM = 0;
        this.shooterBottomLastTargetRPM = 0;
        if(RobotConstants.TUNING_PID){
            SmartDashboard.putNumber("Shooter_Top_Target_RPM", 5000);
            SmartDashboard.putNumber("Shooter_Bottom_Target_RPM", 5000);
        }
    }
    

    public static Shooter getInstance(){
        if (instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    public void setState(ShooterState newState){
        this.currentShooterState = newState;
    }

    public void setHoodState(HoodState newState){
        this.currentHoodState = newState;
    }

    public ShooterState getShooterState(){
        return this.currentShooterState;
    }

    public HoodState getHoodState(){
        return this.currentHoodState;
    }




    public void firstCycle(){
        this.currentShooterState = ShooterState.OFF;
        this.currentHoodState = HoodState.CLOSED;
        this.prevHoodState = HoodState.CLOSED;
        this.io.configureShooterBottomPID(RobotConstants.getShooterBottomPID());
        this.io.configureShooterTopPID(RobotConstants.getShooterTopPID());
      
    }

    public void calculate(){
        switch (this.currentShooterState){
            case FENDER:
                if(this.currentHoodState != HoodState.OPENING && this.currentHoodState != HoodState.OPEN){
                    this.currentHoodState = HoodState.OPENING;
                }
                this.shooterTopTargetRPM = RobotConstants.FENDER_TOP_RPM;
                this.shooterBottomTargetRPM = RobotConstants.FENDER_BOTTOM_RPM;
                if(this.isShooterAtSpeed()){
                    this.desiredLedState = LEDColourState.SHOOTER_AT_RPM;
                } else {
                    this.desiredLedState = LEDColourState.SHOOTER_NOT_AT_RPM;
                }
                this.io.setShooterTopRPM(this.shooterTopTargetRPM);
                this.io.setShooterBottomRPM(this.shooterBottomTargetRPM);
                break;
            case LAUNCH_PAD:
                if(this.currentHoodState != HoodState.CLOSING && this.currentHoodState != HoodState.CLOSED){
                    this.currentHoodState = HoodState.CLOSING;
                }
                this.shooterTopTargetRPM = RobotConstants.LAUNCH_PAD_TOP_RPM;
                this.shooterBottomTargetRPM = RobotConstants.LAUNCH_PAD_BOTTOM_RPM;
                if(this.isShooterAtSpeed()){
                    this.desiredLedState = LEDColourState.SHOOTER_PRESET;
                } else {
                    this.desiredLedState = LEDColourState.SHOOTER_NOT_AT_RPM;
                }
                this.io.setShooterTopRPM(this.shooterTopTargetRPM);
                this.io.setShooterBottomRPM(this.shooterBottomTargetRPM);
                break;
            case LIMELIGHT_SHOOTING:
                this.io.setLimelightTarget(LimelightTargetType.GOAL);
                if(this.currentHoodState != HoodState.CLOSING && this.currentHoodState != HoodState.CLOSED){
                    this.currentHoodState = HoodState.CLOSING;
                }
                if(!RobotConstants.TUNING_PID){
                    if (io.getVisionTargetExists()) {
                        this.shooterTopTargetRPM = shooterTopRPMBasedOnDistance(this.io.getVisionDistanceFeet());
                        this.shooterBottomTargetRPM = shooterBottomRPMBasedOnDistance(this.io.getVisionDistanceFeet());
                        this.shooterTopLastTargetRPM = this.shooterTopTargetRPM;
                        this.shooterBottomLastTargetRPM = this.shooterBottomTargetRPM;
                        if(this.isShooterAtSpeed()){
                            this.desiredLedState = LEDColourState.SHOOTER_AT_RPM;
                        } else {
                            this.desiredLedState = LEDColourState.SHOOTER_NOT_AT_RPM;
                        }
                    } else {
                        this.shooterTopTargetRPM = this.shooterTopLastTargetRPM;
                        this.shooterBottomTargetRPM = this.shooterBottomLastTargetRPM;
                    }
                    if(this.shooterTopTargetRPM > RobotConstants.TOP_SHOOTER_MAX_RPM){
                        this.shooterTopTargetRPM = RobotConstants.TOP_SHOOTER_MAX_RPM;
                    }
                    if(this.shooterBottomTargetRPM > RobotConstants.BOTTOM_SHOOTER_MAX_RPM){
                        this.shooterBottomTargetRPM = RobotConstants.BOTTOM_SHOOTER_MAX_RPM;
                    }
                    io.setShooterTopRPM(this.shooterTopTargetRPM);
                    io.setShooterBottomRPM(this.shooterBottomTargetRPM);
                } else {
                    
                    this.shooterTopTargetRPM = SmartDashboard.getNumber("Shooter_Top_Target_RPM", 5000);
                    this.shooterBottomTargetRPM = SmartDashboard.getNumber("Shooter_Bottom_Target_RPM", 5000);
                    if(this.shooterTopTargetRPM > RobotConstants.TOP_SHOOTER_MAX_RPM){
                        this.shooterTopTargetRPM = RobotConstants.TOP_SHOOTER_MAX_RPM;
                    }
                    if(this.shooterBottomTargetRPM > RobotConstants.BOTTOM_SHOOTER_MAX_RPM){
                        this.shooterBottomTargetRPM = RobotConstants.BOTTOM_SHOOTER_MAX_RPM;
                    }
                    if(this.isShooterAtSpeed()){
                        this.desiredLedState = LEDColourState.SHOOTER_AT_RPM;
                    } else {
                        this.desiredLedState = LEDColourState.SHOOTER_NOT_AT_RPM;
                    }
                    this.io.setShooterTopRPM(this.shooterTopTargetRPM);
                    this.io.setShooterBottomRPM(this.shooterBottomTargetRPM);
                   
                }
                break;
            case LIMELIGHT_MOVING:
                this.io.setLimelightTarget(LimelightTargetType.GOAL);
                if(this.currentHoodState != HoodState.CLOSING && this.currentHoodState != HoodState.CLOSED){
                    this.currentHoodState = HoodState.CLOSING;
                }
               
                if(io.getVisionTargetExists()) {
                    this.shooterTopTargetRPM = shooterTopRPMBasedOnDistance(this.io.getVisionDistanceFeet()) + shooterRunningAdjustment();
                    this.shooterBottomTargetRPM = shooterBottomRPMBasedOnDistance(this.io.getVisionDistanceFeet() + shooterRunningAdjustment());
                    this.shooterTopLastTargetRPM = this.shooterTopTargetRPM;
                    this.shooterBottomLastTargetRPM = this.shooterBottomTargetRPM;
                    if(this.isShooterAtSpeed()){
                        this.desiredLedState = LEDColourState.SHOOTER_AT_RPM;
                    } else {
                        this.desiredLedState = LEDColourState.SHOOTER_NOT_AT_RPM;
                    }
                } else {
                    this.shooterTopTargetRPM = this.shooterTopLastTargetRPM;
                    this.shooterBottomTargetRPM = this.shooterBottomLastTargetRPM;
                }
                    if(this.shooterTopTargetRPM > RobotConstants.TOP_SHOOTER_MAX_RPM){
                        this.shooterTopTargetRPM = RobotConstants.TOP_SHOOTER_MAX_RPM;
                    }
                    if(this.shooterBottomTargetRPM > RobotConstants.BOTTOM_SHOOTER_MAX_RPM){
                        this.shooterBottomTargetRPM = RobotConstants.BOTTOM_SHOOTER_MAX_RPM;
                    }
                    io.setShooterTopRPM(this.shooterTopTargetRPM);
                    io.setShooterBottomRPM(this.shooterBottomTargetRPM);
                
                break;
            case LOW_GOAL:
                if(this.currentHoodState != HoodState.CLOSING && this.currentHoodState != HoodState.CLOSED){
                    this.currentHoodState = HoodState.CLOSING;
                }
                this.shooterTopTargetRPM = RobotConstants.LOW_GOAL_TOP_RPM;
                this.shooterBottomTargetRPM = RobotConstants.LOW_GOAL_BOTTOM_RPM;
                if(this.isShooterAtSpeed()){
                    this.desiredLedState = LEDColourState.SHOOTER_LOW_GOAL;
                } else {
                    this.desiredLedState = LEDColourState.SHOOTER_NOT_AT_RPM;
                }
                this.io.setShooterTopRPM(this.shooterTopTargetRPM);
                this.io.setShooterBottomRPM(this.shooterBottomTargetRPM);
                break;

            case INTAKING:
                if(this.currentHoodState != HoodState.CLOSING && this.currentHoodState != HoodState.CLOSED){
                    this.currentHoodState = HoodState.CLOSING;
                }
                this.shooterTopTargetRPM = RobotConstants.EJECT_TOP_RPM;
                this.shooterBottomTargetRPM = RobotConstants.EJECT_TOP_RPM;
                this.io.setShooterTopRPM(this.shooterTopTargetRPM);
                this.io.setShooterBottomRPM(this.shooterBottomTargetRPM);
                break;
            default:
                break;
            case OFF:
                this.io.setShooterBottomMotor(0);
                this.io.setShooterTopMotor(0);
                this.io.setLimelightTarget(LimelightTargetType.DRIVING);
                this.desiredLedState = LEDColourState.OFF;
               
                   
                
                break;
        }



        switch(this.currentHoodState){
            case CLOSED:
                this.io.setShooterExtend(false);
                this.io.setShooterLock(true);
                break;
            case CLOSING:
                if(this.prevHoodState != HoodState.CLOSING){
                    extendingCycles = 0;
                    lockingCycles = 0;
                }
                this.io.setShooterExtend(false);
                if(extendingCycles < 50){
                    this.io.setShooterLock(false);
                    this.extendingCycles++;
                } else {
                    this.io.setShooterLock(true);
                    if(lockingCycles < 25){
                        lockingCycles++;
                    } else{
                        this.currentHoodState = HoodState.CLOSED;
                    }
                }
                break;
            case OPEN:
                this.io.setShooterExtend(true);
                this.io.setShooterLock(false);
                break;
            case OPENING:
                if(this.prevHoodState != HoodState.OPENING){
                    extendingCycles = 0;
                    lockingCycles = 0;
                }
                this.io.setShooterLock(false);
                if(lockingCycles < 35){
                    this.io.setShooterExtend(false);
                    this.lockingCycles++;
                } else {
                    this.io.setShooterExtend(true);
                    if(extendingCycles < 50){
                        extendingCycles++;
                    } else{
                        this.currentHoodState = HoodState.OPEN;
                    }
                }
                break;
            default:
                break;

        }
        this.prevHoodState = this.currentHoodState;
        SmartDashboard.putString("HOOD STATE", this.currentHoodState.toString());
    }
    
    public LEDColourState getDesiredLedState() {
        return this.desiredLedState;
    }

    private boolean isTopShooterAtSpeed(){
        return Math.abs(this.shooterTopTargetRPM - this.io.getShooterTopRPM()) < 100.0;
    }

    private boolean isBottomShooterAtSpeed(){
        return Math.abs(this.shooterBottomTargetRPM - this.io.getShooterBottomRPM()) < 100.0;
    }

    public boolean isShooterAtSpeed(){
        return this.isBottomShooterAtSpeed() && this.isTopShooterAtSpeed();
    }
       
    public double shooterBottomRPMBasedOnDistance(double distance) {
        if(distance > 11.0){
            return (1.85 * distance * distance) + (9.0 * distance) + 1740;
        } else {
            return (1.85 * distance * distance) + (9.0 * distance) + 1755;
        }
    }

    public double shooterTopRPMBasedOnDistance(double distance) {
        if(distance > 11.0){
            return (1.08 * distance * distance) + (30.0 * distance) + 1570.0;//(1.02 * distance * distance) + (26.4 * distance) + 1626;
        } else {
            return (1.08 * distance * distance) + (30.0 * distance) + 1585.0;//(1.02 * distance * distance) + (26.4 * distance) + 1626;
        }  
    }

    public double shooterRunningAdjustment(){
        Vect robotVelocity = new Vect(this.io.getXVelocity(), this.io.getYVelocity());
        robotVelocity.rotate(this.io.getHeading() + this.io.getVisionTargetX());
        return -200.0 * robotVelocity.getX();

    }

    public void disable(){
        this.io.setShooterTopMotor(0.0);
        this.io.setShooterBottomMotor(0.0);
    }
}

