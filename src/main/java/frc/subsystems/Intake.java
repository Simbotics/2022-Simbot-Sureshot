package frc.subsystems;

import java.lang.System.Logger.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.subsystems.LEDStrip.LEDColourState;

public class Intake extends SubsystemBase{

    private static Intake instance;
    private IntakeState currentIntakeState = IntakeState.OFF_UP;

    private boolean ballFlag = false;
    private boolean haveWeEverSeenABall = false;
    private LEDColourState desiredLedState = LEDColourState.OFF;
    private double indexerEncoderPosition = 0;
    private int cyclesShooting2Balls = 0; 
    private int cyclesToShoot = 50;
    private int cyclesToWait = 10;


    private boolean ejectingWasTrue = false;
    private int cyclesEjectingTop = 0;

    private boolean colorSensorEnabled = false;

 

   
    

    /**
     * Base constructor. Pass the responsibility of defining io to the abstract parent.
     * Consider using the firstCycle when defining behaviour.
     * This function is "private" because it is never called directly outside this file. 
     * To adhere by the Singleton pattern we get getInstance and allow it to define the new
     * instance of this class.
     * 
     * @return Intake object
     * @see    getInstance to build a new instance of this class.
     */
    private Intake() {
        super();
    }

    /**
     * Returns a new object of the Intake class.
     * If there is no previously defined Intake object, a new one is created and returned.
     * 
     * @return Intake object.
     */
    public static Intake getInstance() {
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
	}

    /**
     * Enum defining all visible intake states that we can set the intake to.
     * Available for use by all outside classes.
     * 
     * @see getIntakeState to get the active intake state.
     * @see setIntakeState to set a new active intake state.
     */
    public enum IntakeState { OFF_UP, OFF_DOWN,INTAKING_DOWN, INTAKING_UP, OUTTAKING_UP, OUTTAKING_DOWN, INDEXING, 
                            SHOOTING_DOWN, SHOOTING_UP, SHOOTING_DOWN_TWO_BALLS, SHOOTING_UP_TWO_BALLS, FENDER_SHOOTING_UP, FENDER_SHOOTING_DOWN, 
                            AUTO_SHOOTING, AUTO_OUTAKING, OFF_DOWN_AUTO}

    


    
    /**
     * Sets the current intake state.
     * 
     * @param newState    The intake state that you want to set active.
     * @see   IntakeState Enum defining available intake states.
     */
    public void setState(IntakeState newState) {
        this.currentIntakeState = newState;
        
    }

  
    /**
     * Returns the active intake state
     * 
     * @return             Active intake state.
     * @see    IntakeState Enum defining available intake states.
     */
    public IntakeState getState() {
        return this.currentIntakeState;
    }

    


    @Override
    public void firstCycle() {
        
       // this.io.configureIndexerBottomPID(RobotConstants.getIndexerPID());
        this.io.configureIndexerTopPID(RobotConstants.getIndexerPID());
    }

    @Override
    public void calculate() {

        switch (this.currentIntakeState){
            case INTAKING_DOWN:
                this.io.setIntakeWrist(true);
                if(this.io.getIndexerBallCount() == 2){
                    this.io.setIntakeMotor(0.0);
                    this.io.setIndexerBottom(0);
                    if(!this.isBallCorrect()){ // we must eject
                        this.io.setindexerTopRPM(3000);
                    } else { // we are good
                        this.io.setIndexerTop(0); 
                        this.desiredLedState = LEDColourState.TWO_BALLS;  
                    }
                } else if(this.io.getIndexerBallCount() == 1){
                    this.io.setIntakeMotor(0.65);
                    this.io.setIndexerBottom(1.0);
                    if(!this.isBallCorrect()){ // we must eject
                        this.io.setindexerTopRPM(3000);
                    } else { // we are good
                        this.io.setIndexerTop(0); 
                        this.desiredLedState = LEDColourState.ONE_BALL;   
                    }
                }else{
                    //this.io.setindexerBottomRPM(6000);
                    this.io.setIndexerBottom(1.0);
                    this.io.setindexerTopRPM(3000);
                    this.io.setIntakeMotor(0.65);
                    this.desiredLedState = LEDColourState.INTAKING;
                }
                break;
            case INTAKING_UP:
                this.io.setIntakeMotor(1.0);
                this.io.setIntakeWrist(false);
                cyclesShooting2Balls = 0; 
                this.desiredLedState = LEDColourState.INTAKING;
                if(this.io.getIntakeBottomLightSensor() && !this.io.getIntakeTopLightSensor()){
                    //this.io.setindexerBottomRPM(3000);
                    this.io.setIndexerBottom(0.5);
                } else {
                    this.io.setIndexerBottom(0.0);
                }
                break;
            case OFF_DOWN:
                this.io.setIntakeMotor(0.0);
                this.io.setIntakeWrist(true);
                this.io.setIndexerBottom(0.0);
                cyclesShooting2Balls = 0; 
                if(this.io.getIndexerBallCount() == 0){
                    this.desiredLedState = LEDColourState.OFF;
                    this.io.setIndexerTop(0);
                } else if(this.io.getIndexerBallCount() == 1) {
                    if(!this.isBallCorrect()){ // we must eject
                        this.io.setindexerTopRPM(3000);
                    } else { // we are good
                        this.io.setIndexerTop(0); 
                        this.desiredLedState = LEDColourState.ONE_BALL;   
                    }
                } else {
                    if(!this.isBallCorrect()){ // we must eject
                        this.io.setindexerTopRPM(3000);
                    } else { // we are good
                        this.io.setIndexerTop(0); 
                        this.desiredLedState = LEDColourState.TWO_BALLS;
                        this.currentIntakeState = IntakeState.OFF_UP;   
                    }
                }
                break;
            case OFF_DOWN_AUTO:
                this.io.setIntakeMotor(0.0);
                this.io.setIntakeWrist(true);
                this.io.setIndexerBottom(0.0);
                this.io.setIndexerTop(0.0);
                cyclesShooting2Balls = 0; 
                if(this.io.getIndexerBallCount() == 0){
                    this.desiredLedState = LEDColourState.OFF;
                } else if(this.io.getIndexerBallCount() == 1) {
                    this.desiredLedState = LEDColourState.ONE_BALL;
                } else {
                    this.desiredLedState = LEDColourState.TWO_BALLS;
                }
                break;
            case OFF_UP:
                this.io.setIntakeMotor(0.0);
                this.io.setIntakeWrist(false);
                this.io.setIndexerBottom(0.0);
                cyclesShooting2Balls = 0; 
                this.desiredLedState = LEDColourState.OFF;
                if(this.io.getIndexerBallCount() == 0){
                    this.desiredLedState = LEDColourState.OFF;
                    this.io.setIndexerTop(0);
                } else if(this.io.getIndexerBallCount() == 1) {
                    if(!this.isBallCorrect()){ // we must eject
                        this.io.setindexerTopRPM(3000);
                    } else { // we are good
                        this.io.setIndexerTop(0); 
                        this.desiredLedState = LEDColourState.ONE_BALL;   
                    }
                } else {
                    if(!this.isBallCorrect()){ // we must eject
                        this.io.setindexerTopRPM(3000);
                    } else { // we are good
                        this.io.setIndexerTop(0); 
                        this.desiredLedState = LEDColourState.TWO_BALLS;   
                    }
                }
                break;
            case OUTTAKING_DOWN:
                this.desiredLedState = LEDColourState.OUTTAKING;
                this.io.setIntakeMotor(-0.3);
                this.io.setIntakeWrist(true);
                //this.io.setindexerBottomRPM(-3000);
                this.io.setIndexerBottom(-0.5);
                this.io.setindexerTopRPM(-3000);
                cyclesShooting2Balls = 0; 
               
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            case OUTTAKING_UP:
                this.desiredLedState = LEDColourState.OUTTAKING;
                this.io.setIntakeMotor(-0.3);
                this.io.setIntakeWrist(false);
                //this.io.setindexerBottomRPM(-3000);
                this.io.setIndexerBottom(-0.5);
                this.io.setindexerTopRPM(-3000);
                cyclesShooting2Balls = 0; 
                
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            case INDEXING:
                //this.io.setindexerBottomRPM(3000);
                this.io.setIndexerBottom(0.5);
                this.io.setindexerTopRPM(3000);
                break;
            case SHOOTING_DOWN:
              //  this.io.setindexerBottomRPM(6000);
                this.io.setIndexerBottom(1.0);
                if(this.isBallCorrect()){ // score
                    this.io.setindexerTopRPM(3000);
                } else { // we are not good
                    if(this.io.getVisionTargetX() > (RobotConstants.WRONG_BALL_OFFSET-2.0) || !this.io.getIntakeTopLightSensor()){
                        this.io.setindexerTopRPM(3000);
                    } else {
                        this.io.setIndexerTop(0);  
                    }
                    
                }
                this.io.setIntakeMotor(0.0);
                cyclesShooting2Balls = 0; 
                this.io.setIntakeWrist(true);
                
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            case SHOOTING_UP:
                //this.io.setindexerBottomRPM(6000);
                this.io.setIndexerBottom(1.0);
                if(this.isBallCorrect()){ // score
                    this.io.setindexerTopRPM(3000);
                } else { // we are not good
                    if(this.io.getVisionTargetX() > (RobotConstants.WRONG_BALL_OFFSET-2.0) || !this.io.getIntakeTopLightSensor()){
                        this.io.setindexerTopRPM(3000);
                    } else {
                        this.io.setIndexerTop(0);  
                    }
                    
                }
                this.io.setIntakeMotor(0.0);
                cyclesShooting2Balls = 0; 
                this.io.setIntakeWrist(false);
                
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            case SHOOTING_DOWN_TWO_BALLS:
                
                this.io.setIntakeMotor(0.0);
                this.io.setIntakeWrist(true);

                if(cyclesShooting2Balls < cyclesToShoot){ // keep shooting
                    //this.io.setindexerBottomRPM(4000);
                    this.io.setIndexerBottom(0.66);
                    this.io.setindexerTopRPM(4000);
                } else if(cyclesShooting2Balls < cyclesToShoot + cyclesToWait){ // we need to wait
                   
                    this.io.setIndexerBottom(0);
                    this.io.setIndexerTop(0.0);
                } else { // done waiting 
                   this.currentIntakeState = IntakeState.SHOOTING_DOWN;
                }

                cyclesShooting2Balls++;

                
                
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            case SHOOTING_UP_TWO_BALLS:
                this.io.setIntakeMotor(0.0);
                this.io.setIntakeWrist(false);

                if(cyclesShooting2Balls < cyclesToShoot){ // keep shooting
                    //this.io.setindexerBottomRPM(4000);
                    this.io.setIndexerBottom(0.66);
                    this.io.setindexerTopRPM(4000);
                } else if(cyclesShooting2Balls < cyclesToShoot + cyclesToWait){ // we need to wait
                    
                    this.io.setIndexerBottom(0);
                    this.io.setIndexerTop(0.0);
                } else { // done waiting 
                this.currentIntakeState = IntakeState.SHOOTING_UP;
                }

                cyclesShooting2Balls++;

                
                
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            case FENDER_SHOOTING_DOWN:
               // this.io.setindexerBottomRPM(6000);
               this.io.setIndexerBottom(1.0);
                this.io.setindexerTopRPM(2000);
                this.io.setIntakeMotor(0.0);
                this.io.setIntakeWrist(true);
                cyclesShooting2Balls = 0; 
                
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            case FENDER_SHOOTING_UP:
               // this.io.setindexerBottomRPM(6000);
                this.io.setIndexerBottom(1.0);
                this.io.setindexerTopRPM(2000);
                this.io.setIntakeMotor(0.0);
                this.io.setIntakeWrist(false);
                cyclesShooting2Balls = 0; 
                
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
            case AUTO_SHOOTING:
                this.io.setIndexerBottom(1.0);
                this.io.setindexerTopRPM(2000);
                this.io.setIntakeMotor(0.0);
                this.io.setIntakeWrist(true);
                cyclesShooting2Balls = 0; 
                
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            case AUTO_OUTAKING:
                this.desiredLedState = LEDColourState.OUTTAKING;
                this.io.setIntakeMotor(-0.3);
                this.io.setIntakeWrist(true);
                //this.io.setindexerBottomRPM(-3000);
                this.io.setIndexerBottom(-0.3);
                this.io.setindexerTopRPM(-3000);
                cyclesShooting2Balls = 0; 
            
                this.ballFlag = false;
                this.haveWeEverSeenABall = false;
                break;
            default:
                break;

        }

        
        SmartDashboard.putString("INTAKE STATE", this.currentIntakeState.toString());
        SmartDashboard.putNumber("BALL STATE", this.io.getIndexerBallCount());
        //SmartDashboard.putNumber("INDEXER POS", this.io.getIndexerBottomEncoder());
        
    }

    public LEDColourState getDesiredLedState() {
        return this.desiredLedState;
    }



    public boolean isIntakeUp(){
        return this.currentIntakeState == IntakeState.INTAKING_UP || this.currentIntakeState == IntakeState.SHOOTING_UP || this.currentIntakeState == IntakeState.SHOOTING_UP_TWO_BALLS
        || this.currentIntakeState == IntakeState.OFF_UP || this.currentIntakeState == IntakeState.OUTTAKING_UP || this.currentIntakeState == IntakeState.FENDER_SHOOTING_UP;
    }

    public void setCyclesToWaitBetweenShots(int cycles){
        this.cyclesToWait = cycles;
    }

    public void setColorSensorEnabled(boolean enabled){
        this.colorSensorEnabled = enabled;
    }

    public boolean isBallCorrect(){
        if(this.colorSensorEnabled){
            return this.io.getCorrectBall();
        } else {
            return true;
        }
    }

    
    

    @Override
    public void disable() {
        // TODO Auto-generated method stub
        this.io.setIntakeMotor(0.0);
       
    }
    
}
