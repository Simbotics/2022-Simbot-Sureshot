
package frc.subsystems;

import frc.io.IO;


public class LEDStrip {

    private static LEDStrip instance;
    private IO io;

    public enum LEDColourState {
        INTAKING,ONE_BALL,TWO_BALLS,OFF,OUTTAKING,
        CLIMBER_MANUAL_SET,CLIMBER_MANUAL,CLIMBER_EXTENDING,CLIMBER_DONE_EXTENDING,CLIMBER_RETRACTING,CLIMBER_HOLDING,CLIMBER_ON_BAR,CLIMBER_TRAVERSAL,
        VISION_NO_TARGET,VISION_TARGET,VISION_AIMED,VISION_TOO_CLOSE,
        SHOOTER_AT_RPM, SHOOTER_NOT_AT_RPM, SHOOTER_LOW_GOAL, SHOOTER_PRESET

    }

    private LEDStrip() {
        this.io = IO.getInstance();
    }

    public static LEDStrip getInstance() {
        if (instance == null) {
            instance = new LEDStrip();
        }
        return instance;
    }

    public void setLed(LEDColourState state) {
        switch (state) {
            case CLIMBER_DONE_EXTENDING:
            this.io.setLEDStrip(0.77); // green
                break;
            case CLIMBER_EXTENDING:
                this.io.setLEDStrip(0.61); // red
                break;
            case CLIMBER_HOLDING:
                this.io.setLEDStrip(0.67); // gold
                break;
            case CLIMBER_MANUAL:
                this.io.setLEDStrip(0.57); // hot pink
                break;
            case CLIMBER_MANUAL_SET:
                this.io.setLEDStrip(0.77); // green
                break;
            case CLIMBER_ON_BAR:
                this.io.setLEDStrip(0.77); // green
                break;
            case CLIMBER_RETRACTING:
                this.io.setLEDStrip(0.61); // red
                break;
            case CLIMBER_TRAVERSAL:
                this.io.setLEDStrip(-0.87); // confetti
                break;
            case INTAKING:
                this.io.setLEDStrip(0.77); // green
                break;
            case OUTTAKING:
                this.io.setLEDStrip(0.61); // red
                break;
            case OFF:
                this.io.setLEDStrip(0.99); // black
                break;
            case ONE_BALL:
                this.io.setLEDStrip(0.87); // blue
                break;
            case SHOOTER_AT_RPM:
                this.io.setLEDStrip(0.77); // green
                break;
            case SHOOTER_NOT_AT_RPM:
                this.io.setLEDStrip(0.61); // red
                break;
            case TWO_BALLS:
                this.io.setLEDStrip(-0.09); // violet
                break;
            case VISION_AIMED:
                this.io.setLEDStrip(0.77); // green
                break;
            case VISION_NO_TARGET:
                this.io.setLEDStrip(0.61); // red
                break;
            case VISION_TARGET:
                this.io.setLEDStrip(0.65); // orange
                break;
            case VISION_TOO_CLOSE:
                this.io.setLEDStrip(-0.11); // STROBE RED!!
                break;
            case SHOOTER_LOW_GOAL:
                this.io.setLEDStrip(-0.05); // STROBE!!
                break;
            case SHOOTER_PRESET:
                this.io.setLEDStrip(-0.07); // STROBE!!
                break;
            default:
                break;
        
        }

    }
}
