package frc.util;

public class Translation2d {

    private double x;
    private double y;

    public Translation2d() {
        this.x = 0;
        this.y = 0;
    }

    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Translation2d(Translation2d duplicate) {
        this.x = duplicate.x;
        this.y = duplicate.y;
    }

    public Translation2d translateBy(Translation2d translationVector) {
        return new Translation2d(this.x + translationVector.x, this.y + translationVector.y);
    }

    public Translation2d rotateBy(Rotation2d rotationVector) {
        return new Translation2d(this.x * rotationVector.getCosAngle() + this.y * -rotationVector.getSinAngle(),
                this.x * rotationVector.getSinAngle() + this.y * rotationVector.getCosAngle());
    }

    public double norm() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public Translation2d inverse(){
        return new Translation2d(-this.x, -this.y);
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }
}