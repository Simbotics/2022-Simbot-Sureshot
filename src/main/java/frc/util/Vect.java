package frc.util;

/*
 * Basic Vector library 
 * 
 * @author Jason
 */
public class Vect {
    private double x;
    private double y;

    public Vect(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vect(double mag, double angle, int none) {
        double c = Math.toRadians(angle);
        this.x = Math.cos(c) * mag;
        this.y = Math.sin(c) * mag;
    }
	
    public double getX() {
        return x;	
    }
	
    public double getY() {
        return y;
    }

    public Vect(Vect clone) {
        this.x = clone.x;
        this.y = clone.y;
    }	

    public Vect add(Vect b) {
        return new Vect(this.x + b.x, this.y + b.y);
    }

    public Vect sub(Vect b) {
    	return new Vect(this.x - b.x, this.y - b.y);
    }
    
    public Vect scalarMult(double c){
        return new Vect (this.x * c, this.y *c);
    }
    
    public static Vect fromAngle(double c) {
    	c = Math.toRadians(c);
    	return new Vect(Math.cos(c), Math.sin(c));
    }

    public Vect unit() {
        double magnitude = this.mag();
        return new Vect(this.x / magnitude, this.y / magnitude);
    }

    public double magSq() {
        return this.dot(this);
    }
	
    public double dot(Vect b) {
        return this.x * b.x + this.y * b.y;
    }

    public double mag() {
        return Math.sqrt(this.magSq());
    }
    
    public double scalarProjectOnto(Vect b) {
    	return this.dot(b) / b.mag();
    }
    
    public Vect projectOnto(Vect b) {
        return b.scalarMult(this.dot(b) / b.magSq());
    }
    
    public Vect rotate(double degrees) {
    	double angle = Math.toRadians(degrees);
    	double x = this.x*Math.cos(angle)-this.y*Math.sin(angle);
    	double y = this.x*Math.sin(angle)+this.y*Math.cos(angle);
    	return new Vect(x,y);
    }
    
    public String toString() {
    	return "(" + x + "," + y + ")";
    }
}
