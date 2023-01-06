package frc.util;
public class Rotation2d {
    protected static final double kEpsilon = 0.000000001;
    protected double cosAngle;
    protected double sinAngle;
    public Rotation2d() {
        this(1, 0, false);
    }
    public Rotation2d(double x, double y, boolean normalize) {
        cosAngle = x;
        sinAngle = y;
        if (normalize) {
            normalize();
        }
    }
    public Rotation2d(Rotation2d other) {
        cosAngle = other.cosAngle;
        sinAngle = other.sinAngle;
    }
    public static Rotation2d fromRadians(double angleRadians) {
        return new Rotation2d(Math.cos(angleRadians), Math.sin(angleRadians), false);
    }
    public static Rotation2d fromDegrees(double angleDegrees) {
        return fromRadians(Math.toRadians(angleDegrees));
    }
    public void normalize() {
        double magnitude = Math.hypot(cosAngle, sinAngle);
        if (magnitude > kEpsilon) {
            sinAngle /= magnitude;
            cosAngle /= magnitude;
        } else {
            sinAngle = 0;
            cosAngle = 0;
        }
    }
    public double getCosAngle() {
        return cosAngle;
    }
    public double getSinAngle() {
        return sinAngle;
    }
    public double tan() {
        if (cosAngle < kEpsilon) {
            if (sinAngle >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sinAngle / cosAngle;
    }
    public double getRadians() {
        return Math.atan2(sinAngle, cosAngle);
    }
    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }
    public Rotation2d rotateBy(Rotation2d other){
        return new Rotation2d(cosAngle * other.cosAngle - sinAngle * other.sinAngle,
                cosAngle * other.sinAngle + sinAngle * other.cosAngle, true);
    }
    public Rotation2d inverse() {
        return new Rotation2d(cosAngle, -sinAngle, false);
    }
}