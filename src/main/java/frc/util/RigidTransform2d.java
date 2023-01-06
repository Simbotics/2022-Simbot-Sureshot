package frc.util;

public class RigidTransform2d {

    private static final double kEps = 0.00000001;

    public static class Delta {
        public final double dx;
        public final double dy;
        public final double dtheta;

        public Delta(double dx, double dy, double dtheta) {
            this.dx = dx;
            this.dy = dy;
            this.dtheta = dtheta;
        }
    }

    protected Translation2d translation;
    protected Rotation2d rotation;

    public RigidTransform2d() {
        translation = new Translation2d();
        rotation = new Rotation2d();
    }

    public RigidTransform2d(Translation2d translationVector, Rotation2d rotationVector) {
        this.translation = translationVector;
        this.rotation = rotationVector;
    }

    public RigidTransform2d(RigidTransform2d other) {
        this.translation = new Translation2d(other.translation);
        this.rotation = new Rotation2d(other.rotation);
    }

    public static RigidTransform2d fromTranslation(Translation2d translation) {
        return new RigidTransform2d(translation, new Rotation2d());
    }

    public static RigidTransform2d fromRotation(Rotation2d rotation) {
        return new RigidTransform2d(new Translation2d(), rotation);
    }

    public Translation2d getTranslation() {
        return this.translation;
    }

    public void setTranslation(Translation2d translation) {
        this.translation = translation;
    }

    public Rotation2d getRotation() {
        return this.rotation;
    }

    public void setRotation(Rotation2d rotation) {
        this.rotation = rotation;
    }

    public RigidTransform2d inverse() {
        Rotation2d rotation_inverted = this.rotation.inverse();
        return new RigidTransform2d(this.translation.inverse(), rotation_inverted);
    }

    public RigidTransform2d transformBy(RigidTransform2d other) {
        return new RigidTransform2d(
                this.translation.rotateBy(this.rotation).translateBy(other.translation.rotateBy(other.rotation)),
                new Rotation2d());
    }

    public RigidTransform2d transformBy(Translation2d translationVector) {
        return new RigidTransform2d(this.translation.translateBy(translationVector), this.rotation);
    }

    public RigidTransform2d transformBy(Rotation2d rotationVector) {
        return new RigidTransform2d(this.translation, this.rotation.rotateBy(rotationVector));
    }

    public double getX() {
        return this.translation.rotateBy(this.rotation).getX();
    }

    public double getY() {
        return this.translation.rotateBy(this.rotation).getY();
    }
}