package frc.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;

public class SimAnalogEncoder {

    private final AnalogInput analogInput;

    private int count = 0;
    private int prev = 0;
    private int prevRaw = 0;
    private int speed = 0;
    private int offset = 0;
    private ArrayList<Integer> pastSpeeds;
    private int maxEncoderVal = 0;
    private final int PAST_SPEEDS_SIZE = 5;
    private int rolloverRange = 300;


    public SimAnalogEncoder(final int analogInputPort, int offset, int rolloverRange, int maxEncoderVal) {
        this.analogInput = new AnalogInput(analogInputPort);
        this.offset = offset;
        this.pastSpeeds = new ArrayList<Integer>();
        this.rolloverRange = rolloverRange;
        this.maxEncoderVal = maxEncoderVal;
    }

    public SimAnalogEncoder(final int analogInputPort, int offset, int rolloverRange) {
        this.analogInput = new AnalogInput(analogInputPort);
        this.offset = offset;
        this.pastSpeeds = new ArrayList<Integer>();
        this.rolloverRange = rolloverRange;
        this.maxEncoderVal = 4096;
    }

    public SimAnalogEncoder(final int analogInputPort, int offset){
        this.analogInput = new AnalogInput(analogInputPort);
        this.offset = offset;
        this.pastSpeeds = new ArrayList<Integer>();
        this.maxEncoderVal = 4096;
    }

    public SimAnalogEncoder(final int analogInputPort){
        this.analogInput = new AnalogInput(analogInputPort);
        this.offset = 0;
        this.pastSpeeds = new ArrayList<Integer>();
        this.maxEncoderVal = 4096;
    }

    public int getRawWithOffset(){
        return (analogInput.getValue() - offset) % this.maxEncoderVal;
    }

    public int getRaw(){
        return analogInput.getValue();
    }

    public int get(){
        if(count < 0) return ((this.count - 1) * this.maxEncoderVal) + this.getRawWithOffset();

        return (this.maxEncoderVal * this.count) + this.getRawWithOffset();
    }

    public int speed(){
        return this.getRawWithOffset() - this.prev;
    }

    public double voltage(){
        return analogInput.getVoltage();
    }

    public void zero(){
        this.count = 0;
        this.offset = this.getRawWithOffset();
    }

    public double getRunningAverageSpeed() {
		double sum = 0.0;
		for (Integer speed : this.pastSpeeds) {
			sum = sum + speed;
		}
		return sum / this.pastSpeeds.size();
	}

    public void update(){
        int curr = this.getRawWithOffset();
        this.speed = curr - this.prev;
        

        if (this.pastSpeeds.size() >= PAST_SPEEDS_SIZE) {
			this.pastSpeeds.remove(0);
		}
        this.pastSpeeds.add(this.prev);

        if(inUpperRolloverRange(this.prevRaw) && inLowerRolloverRange(this.getRaw())){
            this.count++;
        }

        if(inLowerRolloverRange(this.prevRaw) && inUpperRolloverRange(this.getRaw())){
            this.count--;
        }
        
        this.prev = curr;
        this.prevRaw = this.getRaw();  
    }

    public boolean inUpperRolloverRange(int value){
        return value >= this.maxEncoderVal - this.rolloverRange;
    }

    public boolean inLowerRolloverRange(int value){
        return value <= this.rolloverRange;
    }

    public int getCount(){
        return this.count;
    }
}