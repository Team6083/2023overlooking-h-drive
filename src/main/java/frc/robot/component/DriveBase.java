package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.System.NewAutoEngine;

public class DriveBase {

    // Port
    private static final int Lm1 = 1;// MotorController ID
    private static final int Lm2 = 2;
    private static final int Rm1 = 3;
    private static final int Rm2 = 4;
    private static final int Mm = 5;

    // Basis divebase
    public static WPI_TalonSRX leftMotor1;// Define motor
    public static WPI_TalonSRX leftMotor2;
    public static WPI_TalonSRX rightMotor1;
    public static WPI_TalonSRX rightMotor2;
    public static WPI_TalonSRX middleMotor;

    public static MotorControllerGroup leftmotor;
    public static MotorControllerGroup rightmotor;
    public static DifferentialDrive drive;// Use to simplified drivebase program

    // Sensor
    // Gyro: need install Library
    public static AHRS gyro; // To detect the current angle, and design which angle we want to match, then
                             // calculate to match the goal

    public static DifferentialDriveOdometry odometry;
    protected static RamseteController ramseteController = new RamseteController();
    protected static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.53);

    // Feedforward Controller
    protected static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.7, 0.1);

    protected static Field2d field = new Field2d();
    protected static Field2d trajField = new Field2d();

    // For PID
    public static double kP = 0.13;
    public static double kI = 0;
    public static double kD = 0;

    public static PIDController leftPID = new PIDController(kP, kI, kD);
    public static PIDController rightPID = new PIDController(kP, kI, kD);

    // Set the pulse of the encoder
    private static final double encoderPulse = 4096;

    private static final double gearing = 10.71;

    private static double LeftVolt;
    private static double RightVolt;

    private static double left;
    private static double right;
    private static double middle;

    private static double leftWheelSpeed;
    private static double rightWheelSpeed;

    public static void init() {
        leftMotor1 = new WPI_TalonSRX(Lm1);
        leftMotor2 = new WPI_TalonSRX(Lm2);
        rightMotor1 = new WPI_TalonSRX(Rm1);
        rightMotor2 = new WPI_TalonSRX(Rm2);
        middleMotor = new WPI_TalonSRX(Mm);

        leftmotor = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightmotor = new MotorControllerGroup(rightMotor1, rightMotor2);
        leftmotor.setInverted(true);
        rightmotor.setInverted(false);
        drive = new DifferentialDrive(leftmotor, rightmotor);

        // Reset encoder
        leftMotor1.configClearPositionOnQuadIdx(true, 10);
        rightMotor1.configClearPositionOnQuadIdx(true, 10);
        middleMotor.configClearPositionOnQuadIdx(true, 10);

        // Define gyro ID
        gyro = new AHRS(SPI.Port.kMXP);

        // Put path and status on field from PathWeaver on SmartDashboard
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0),
                positionToDistanceMeter(leftMotor1.getSelectedSensorPosition()),
                positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()));

        SmartDashboard.putData("field", field);
        SmartDashboard.putData("trajField", trajField);

        putDashboard();
    }

    // Normal drivebase
    public static void teleop() {

        left = -Robot.xbox.getLeftY() * 0.9;
        right = Robot.xbox.getRightY() * 0.9;
        middle = Robot.xbox.getLeftX() * 0.9;

        if (Robot.xbox.getLeftBumperPressed() || Robot.xbox.getRightBumperPressed()) {
            left = 1;
            right = 1;
            middle = 1;
        }

        drive.tankDrive(left, right);
        middleMotor.set(middle);
    }

    public static void directControl(double leftMotorInput, double rightMotorInput, double middleMotorInput) {
        left = leftMotorInput;
        right = rightMotorInput;
        middle = middleMotorInput;
        drive.tankDrive(left, right);
        middleMotor.set(middle);
    }

    public static void runTraj(Trajectory trajectory, double timeInsec) {

        // Set the goal of the robot in that second
        Trajectory.State goal = trajectory.sample(timeInsec);
        trajField.setRobotPose(goal.poseMeters);

        var currentPose = odometry.getPoseMeters();

        var chaspeed = ramseteController.calculate(currentPose, goal);

        // Convert chassis speed to wheel speed
        var wheelSpeeds = kinematics.toWheelSpeeds(chaspeed); // Left and right speed
        leftWheelSpeed = wheelSpeeds.leftMetersPerSecond; // Catch speed from wheelSpeed(with ctrl + left mice)
        rightWheelSpeed = wheelSpeeds.rightMetersPerSecond;

        leftPID.setSetpoint(leftWheelSpeed);
        rightPID.setSetpoint(rightWheelSpeed);

        // To make the number of the encoder become the motor's volt
        LeftVolt = leftPID.calculate(
                positionToDistanceMeter(leftMotor1.getSelectedSensorPosition() / NewAutoEngine.timer.get()), left)
                + feedforward.calculate(leftWheelSpeed);
        RightVolt = rightPID.calculate(
                positionToDistanceMeter(rightMotor1.getSelectedSensorPosition() / NewAutoEngine.timer.get()), right)
                + feedforward.calculate(rightWheelSpeed);

        leftmotor.setVoltage(LeftVolt);
        rightmotor.setVoltage(RightVolt);
        drive.feed();

        SmartDashboard.putNumber("errorPosX", currentPose.minus(goal.poseMeters).getX());// The distance between the
                                                                                         // target and the position
        SmartDashboard.putNumber("errorPosY", currentPose.minus(goal.poseMeters).getY());
    }

    public static void updateODO() {
        var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
        odometry.update(gyroAngle, positionToDistanceMeter(leftMotor1.getSelectedSensorPosition()),
                positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()));
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("heading", odometry.getPoseMeters().getRotation().getDegrees());

        kP = SmartDashboard.getNumber("kP", kP);
        kI = SmartDashboard.getNumber("kI", kI);
        kD = SmartDashboard.getNumber("kD", kD);

        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
        drive.feed();
    }

    // To set the position and rotation of the robot
    public static void setODOPose(Pose2d pose) {
        odometry.resetPosition(pose.getRotation(), positionToDistanceMeter(leftMotor1.getSelectedSensorPosition()),
                positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()), pose);
        field.setRobotPose(odometry.getPoseMeters());
    }

    public static void putDashboard() {
        SmartDashboard.putNumber("leftEncoder", leftMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightEncoder", rightMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("gyro", gyro.getAngle());
        SmartDashboard.putNumber("leftController_speed", left);// Motor's volt
        SmartDashboard.putNumber("rightController_speed", right);
        SmartDashboard.putNumber("middleController_speed", middle);
        SmartDashboard.putNumber("left_wheel_speed", left);// The wheel speed
        SmartDashboard.putNumber("right_wheel_speed", right);
        SmartDashboard.putNumber("middle_wheel_speed", right);
    }

    // Input the position of the encoder then calculate the distance(meter)
    public static double positionToDistanceMeter(double position) {
        double sensorRate = position / encoderPulse;
        double wheelRate = sensorRate / gearing;
        double positionMeter = 2 * Math.PI * Units.inchesToMeters(6) * wheelRate;
        return positionMeter;
    }

    // Here comes some mode to set up or update
    public static void resetEncoderOn() {
        leftMotor1.configClearPositionOnQuadIdx(true, 10);
        rightMotor1.configClearPositionOnQuadIdx(true, 10);
        middleMotor.configClearPositionOnQuadIdx(true, 10);
    }

    public static void resetEncoderOff() {
        leftMotor1.configClearPositionOnQuadIdx(false, 10);
        rightMotor1.configClearPositionOnQuadIdx(false, 10);
        middleMotor.configClearPositionOnQuadIdx(true, 10);
    }

    public static void resetGyro() {
        gyro.reset();
    }

    public static void resetPID() {
        leftPID.reset();
        rightPID.reset();
    }
}
