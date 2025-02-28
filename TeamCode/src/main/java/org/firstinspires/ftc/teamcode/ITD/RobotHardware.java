package org.firstinspires.ftc.teamcode.ITD;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {
    // Drivetrain
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    // Odometry & Positioning
    public IMU imu = null;
    public Odometry odometry = new Odometry();
    // Front Intake
    public DcMotor intakeMotor = null;
    public DcMotor intakeExtensionSlide = null;
    public Servo intakeServoLeft = null; // Servos raise intake up and down
    public Servo intakeServoRight = null;
    public Servo outtakeGuardServo = null;
    // Back Twin Towers
    public DcMotor backElevatorLeft = null;
    public DcMotor backElevatorRight = null;
    public TouchSensor elevatorLimitLeft = null;
    public TouchSensor elevatorLimitRight = null;

    public DigitalChannel horizontalLimit = null;
    // Sample Bucket
    public Servo bucketServoLeft = null;
    public Servo bucketServoRight = null;
    // Specimen Scoring
    public Servo clawServo = null;
    // Lights
    public RevBlinkinLedDriver lightStrip = null;

    public static Telemetry telemetry;
    
    static final double releasePosition = 0.5;
    static final double grabPosition = 0;
    static final double maxSlidePosition = 2800;
    static final boolean povDrive = false;
    static final boolean teamColor = false;
    static final double lServoIn = 0;
    static final double rServoIn = 1;
    static final double lServoInMod = 0.05;
    static final double rServoInMod = 0.95;
    static final double lServoMed = 0.45;
    static final double rServoMed = 0.55;
    static final double lServoOut = 0.68;
    static final double rServoOut = 0.32;
    static final double lServoOutOut = 0.05;
    static final double rServoOutOut = 0.95;
    static final double lServoOutIn = 0.62;
    static final double rServoOutIn = 0.38;
    static final double lServoOutInMod = 0.5;
    static final double rServoOutInMod = 0.5;
    static final double shutoffIntakeDelay = 1.0;
    static final double guardDownAfterOuttakeDownDelay = 0.5;
    static final double intakeForwardPower = 0.7;
    static final double intakeMaxReversePower = 0.5;
    static final double reverseStartPower = 0.2;
    static final double reverseEasePerSecond = 0.25;
    static final double guardServoDown = 0.29;
    static final double guardServoUp = 0.0;

    // Non Constant
    static double initialYaw;

    public RobotHardware(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl"); // Encoder = Odometry X (left, right)
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr"); // Encoder = Odometry Y (forward, back)
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        imu = hardwareMap.get(IMU.class, "imu");

        odometry.init(imu, leftBackDrive, rightFrontDrive);

        intakeServoLeft = hardwareMap.get(Servo.class, "insvl");
        intakeServoRight = hardwareMap.get(Servo.class, "insvr");
        bucketServoLeft = hardwareMap.get(Servo.class, "outsvl");
        bucketServoRight = hardwareMap.get(Servo.class, "outsvr");
        outtakeGuardServo = hardwareMap.get(Servo.class, "guardsvr");
        clawServo = hardwareMap.get(Servo.class, "sp");

        backElevatorLeft = hardwareMap.get(DcMotor.class, "lsl");
        backElevatorRight = hardwareMap.get(DcMotor.class, "lsr");
        intakeExtensionSlide = hardwareMap.get(DcMotor.class, "ext");
        intakeMotor = hardwareMap.get(DcMotor.class, "int");

        elevatorLimitLeft = hardwareMap.get(TouchSensor.class, "liml");
        elevatorLimitRight = hardwareMap.get(TouchSensor.class, "limr");
        horizontalLimit = hardwareMap.get(DigitalChannel.class, "limh");

        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "li");

        // Initialize IMU
        RevHubOrientationOnRobot imuOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(imuOrientation));

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders means apply proportion of max power, not max velocity - keep in mind
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backElevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backElevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backElevatorLeft.setDirection(DcMotor.Direction.FORWARD);
        backElevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backElevatorRight.setDirection(DcMotor.Direction.FORWARD);
        backElevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtensionSlide.setDirection(DcMotor.Direction.FORWARD);
        intakeExtensionSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        initialYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
