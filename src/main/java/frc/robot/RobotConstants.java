package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.io.Dashboard;
import frc.util.PIDConstants;

public class RobotConstants {

	public static final boolean USING_DASHBOARD = false;
	public static final boolean TUNING_PID = false;

	// Drive Size Constants (Feet)
	public static final double DRIVE_MAX_VELOCITY = 14.0;
	public static final double DRIVE_TICKS_PER_REV = 2048.0;
	



	

	
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0 * (10.5/10.0));
        public static final double kDriveMotorGearRatio = 1.0 / ((34.0/10.0) * (20.0/26.0) * (45.0/15.0));
        public static final double kTurningMotorGearRatio = 1.0 / ((24.0/8.0) * (72.0/14.0));
        public static final double kDriveEncoderRot2Meter = (2048.0 / kDriveMotorGearRatio) * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = (2048.0 / kTurningMotorGearRatio) / 2.0;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21.25);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22.25);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0));

        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 10;
        public static final int kFrontRightDriveMotorPort = 9;
        public static final int kBackRightDriveMotorPort = 11;

        public static final int kFrontLeftTurningMotorPort = 4;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 7;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

		public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.900447;//Math.PI + 0.937262;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -2.057;//0.276117;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.204;//- 1.965;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.983903;//Math.PI + 2.816922;

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14);
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

	//SHOOTER CONSTANTS
	
	
	public static final double SHOOTER_TICKS_PER_REV = 2048.0;
	
	public static final double BOTTOM_SHOOTER_MAX_RPM = 6320; 
	public static final double TOP_SHOOTER_MAX_RPM = 5880;

	public static final double FENDER_TOP_RPM = 2000.0;
	public static final double FENDER_BOTTOM_RPM = 2200.0;

	public static final double LOW_GOAL_TOP_RPM = 950.0;
	public static final double LOW_GOAL_BOTTOM_RPM = 950.0;

	public static final double EJECT_TOP_RPM = 950.0;
	public static final double EJECT_BOTTOM_RPM = 950.0;

	public static final double LAUNCH_PAD_TOP_RPM = 2400.0;
	public static final double LAUNCH_PAD_BOTTOM_RPM = 2400.0;


	public static final double WRONG_BALL_OFFSET = 20.0;




	// DRIVE ANGLES TO SNAP TO

	public static final double DRIVE_LEFT_WALL_ANGLE = 90.0;
	public static final double DRIVE_RIGHT_WALL_ANGLE = -90.0;
	public static final double DRIVE_FENDER_FRONT_LEFT = 21.0;
	public static final double DRIVE_FENDER_FRONT_RIGHT = -69.0;
	public static final double DRIVE_FENDER_BACK_LEFT = 111.0;
	public static final double DRIVE_FENDER_BACK_RIGHT = -159.0;
	public static final double DRIVE_CLIMB_ANGLE = 0.0;
	public static final double DRIVE_LAUNCHPAD_ANGLE = 12.0;
	

	//HANGER CONSTANTS 

	public static final double HANGER_EXTEND_OUTPUT = 1.0;
	public static final double HANGER_RETRACT_OUTPUT = -0.8;
	public static final double HANGER_HOLD_OUTPUT = -0.07;


	public static final double HANGER_INVERT_THRESHOLD = 15000;
	public static final double HANGER_EXTEND_POSITION = 230000;
	public static final double HANGER_MID_EXTEND_POSITION = 290000;
	public static final double HANGER_RETRACT_FULL_POSITION = 30000;
	public static final double HANGER_RETRACT_HOLD_POSITION = 0;
	public static final double HANGER_RETRACT_TRAVERSAL_POSITION = 234000;
	public static final double HANGER_RETRACT_SEMI_HIGH_POSITION = 125000;
	public static final double HANGER_CURRENT_THRESHOLD = 50.0;
	public static final double HANGER_SPEED_THRESHOLD = 250;


	
	//INDEXER CONSTANTS
	public static final double IndexerOffset = 25000;


	// PID CONSTANTS

	private static Dashboard dashboard = Dashboard.getInstance();

	private static PIDConstants climberHoldPID = new PIDConstants(0.00, 0, 0, 200);

	private static PIDConstants indexerPID = new PIDConstants(0.07, 0.0, 0.0, 0.06, 0);
 	private static PIDConstants shooterTopPID = new PIDConstants(0.19, 0.0, 0.0, 0.053 ,0);
	private static PIDConstants shooterBottomPID = new PIDConstants(0.19, 0.0, 0.0, 0.0465,0);
	
	private static PIDConstants modulePID = new PIDConstants(0.5, 0.0000, 0.0, 0.0);

	private static PIDConstants headingPID = new PIDConstants(0.013, 0.0002, 0.012, 0.0);
	private static PIDConstants drivePID = new PIDConstants(0.56, 0.00045, 0.001, 0.01);

	
	// Camera Constants
	public static final double CAMERA_HORIZONTAL_FOV = 59.6;
	public static final double CAMERA_VERTICAL_FOV = 49.7;
	public static final double CAMERA_IMAGE_WIDTH = 320.0;
	public static final double CAMERA_IMAGE_HEIGHT = 240.0;
	public static final double CAMERA_FOCAL_LENGTH_Y = (CAMERA_IMAGE_HEIGHT / 2)
			/ Math.tan(Math.toRadians(CAMERA_VERTICAL_FOV / 2));
	public static final double CAMERA_FOCAL_LENGTH_X = (CAMERA_IMAGE_WIDTH / 2)
			/ Math.tan(Math.toRadians(CAMERA_HORIZONTAL_FOV / 2));
	public static final double Y_ANGLE_PER_PIXEL = CAMERA_VERTICAL_FOV / CAMERA_IMAGE_HEIGHT;
	public static final double X_ANGLE_PER_PIXEL = CAMERA_HORIZONTAL_FOV / CAMERA_IMAGE_WIDTH;

	
	public static final double GOAL_HEIGHT_INCHES = 102.0;
	public static final double CAMERA_HEIGHT_INCHES = 33.187;
	public static final double CAMERA_TILT_DEGREES = 43.0;

	// SHOOTER DISTANCE VALUES
	public static PIDConstants getIndexerPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("INDEXER_PID", indexerPID);
		} else {
			return indexerPID;
		}
	}
	
	public static PIDConstants getShooterTopPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("TOP_SHOOTER_PID", shooterTopPID);
		} else {
			return shooterTopPID;
		}
	}
	public static PIDConstants getShooterBottomPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("BOTTOM_SHOOTER_PID", shooterBottomPID);
		} else {
			return shooterBottomPID;
		}
	}

	public static PIDConstants getModulePID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("MODULE_PID", modulePID);
		} else {
			return modulePID;
		}
	}

	public static PIDConstants getHeadingPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("HEADING_PID", headingPID);
		} else {
			return headingPID;
		}
	}

	public static PIDConstants getDrivePID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("DRIVE_PID", drivePID);
		} else {
			return drivePID;
		}
	}

	public static PIDConstants getClimberHoldPID() {
		if (USING_DASHBOARD) {
			return dashboard.getPIDConstants("CLIMBER_PID", climberHoldPID);
		} else {
			return climberHoldPID;
		}
	}
	

	// Pushes the values to the smart dashboard

	public static void pushValues() {
	
		//dashboard.putPIDConstants("TOP_SHOOTER_PID", shooterTopPID);
		//dashboard.putPIDConstants("BOTTOM_SHOOTER_PID", shooterBottomPID);
		//dashboard.putPIDConstants("HEADING_PID", headingPID);
		//dashboard.putPIDConstants("MODULE_PID", modulePID);	
		//dashboard.putPIDConstants("INDEXER_PID", indexerPID);
		//dashboard.putPIDConstants("DRIVE_PID", drivePID);
		//dashboard.putPIDConstants("CLIMBER_PID", climberHoldPID);
	}
}
