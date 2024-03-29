package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    /** Intake */
    public static final int INTAKE_LEFT_CAN_ID = 28;
    public static final int INTAKE_RIGHT_CAN_ID = 29;
    public static final int INTAKE_RIGHT_OMNI_CAN_ID = 23;
    public static final int INTAKE_LEFT_OMNI_CAN_ID = 21; 
    public static final int FRONT_STORAGE_ROLLER_CAN_ID = 30; 
    public static final int BACK_STORAGE_ROLLER_CAN_ID = 35; 
    public static final int SHOOTER_LEFT_CAN_ID = 13;
    public static final int SHOOTER_RIGHT_CAN_ID = 25;
    public static final int SHOOTER_HOOD_CAN_ID = 27;
    public static final int CLIMBER_RIGHT_CAN_ID = 36;
    public static final int CLIMBER_LEFT_CAN_ID = 37;

    public static final int INTAKE_PCH_ID = 15;
    public static final int PCH_CAN_ID = 9;


    public static final int CLIMBER_INITIAL_HOOK_PCH_ID = 8;
    public static final int CLIMBER_SECOND_HOOK_PCH_ID = 9;
    public static final int CLIMBER_ELEVATOR_PCH_ID = 14; 

    public static final double INTAKE_SPIN_SPEED = .75;  
    public static final double OMNIS_SPIN_SPEED = 0.65;
    public static final double FEEDER_INNER_SPEED = 1;
    public static final double FEEDER_OUTER_SPEED = 1;
    public static final double FEEDER_SHOOTING_SPEED = 0.75;
    public static final double SHOOTER_LOW_GOAL_PERCENT_OUTPUT = 0.35;
    public static final double SHOOTER_HIGH_GOAL_PERCENT_OUTPUT = 0.55;
    public static final double AUTO_DRIVE_DISTANCE = 0.25;
    public static final double SHOOTER_RESTING_VELOCITY = .25; //10 is placeholder value
    public static final double TOPROLLER_OPEN_LOOP = .6;
    public static final double TOPROLLER_OPEN_LOOP_LOWER_GOAL = .2;

    /**Closed Loop Constants */
    public static final double SHOOTER_VELOCITY_HIGH = 4350; //20 is placeholder value
    public static final double INTAKE_VELOCITY = 30;
    public static final double CLIMBER_START_POSITION = 0;
    public static final double TOPROLLER_VELOCITY = 1000;
    //CHANGE TO CORRECT NUMBERS

    public static final double stickDeadband = 0.04;

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(11.5);
        public static final double wheelBase = Units.inchesToMeters(11.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.6;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (8.16 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        //angleNeutralMode set to coast to make it easier to move robot when disabled--absolute encoders are great! during operation, steering is position contolled to 
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast; 
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 8;
            public static final double angleOffset = 101.25; //147.48
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 2;
            public static final double angleOffset = 82.96;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 5;
            public static final double angleOffset = 273.69;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 11;
            public static final double angleOffset = 328.00;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
