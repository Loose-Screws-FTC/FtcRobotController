import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.TouchSensor;

// Controller scheme: https://www.padcrafter.com/?templates=LM3+Scheme&plat=0&yButton=Override+slide+limit&leftTrigger=Vertical+slide+down&rightTrigger=Vertical+slide+up&leftBumper=Intake+spin+out&rightBumper=Intake+spin+in&col=%23242424%2C%23606A6E%2C%23FFFFFF&xButton=Light+pattern+1&bButton=Light+pattern+2&leftStickClick=&leftStick=Move&rightStickClick=Turn&rightStick=Change+team+color

@TeleOp(name="Linear Slide Adjustment", group="Linear OpMode")
public class AdjustArms extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private IMU imu = null;
    
    private Servo intakeServo0 = null;
    private Servo intakeServo1 = null;
    private Servo outtakeServo0 = null;
    private Servo outtakeServo1 = null;
    private Servo jawServo = null;
    private DcMotor linearSlideL = null;
    private DcMotor linearSlideR = null;
    private DcMotor intakeSlide = null;
    private DcMotor intakeMotor = null;
    private TouchSensor slideLimitL = null;
    private TouchSensor slideLimitR = null;
    //private DcMotor extensionSlide = null;
    //private Servo extensionServo = null;
    //private Servo sampleGrabber = null;
    private RevBlinkinLedDriver lightStrip = null;
    
    static double releasePosition = 0.5;
    static double grabPosition = 0;
    
    static double maxSlidePosition = 2700;

    private boolean povDrive = false;
    
    private double initialYaw;
    
    private boolean teamColor = false;
    
    private boolean overrideSlideLimit = false;
    
    static double lServoIn = 0;
    static double rServoIn = 1;
    static double lServoMed = 0.4;
    static double rServoMed = 0.6;
    
    static double lServoOut = 0.625;
    static double rServoOut = 0.375;
    
    static double lServoOutOut = 0;
    static double rServoOutOut = 1;
    static double lServoOutIn = 0.6;
    static double rServoOutIn = 0.4;
    
    private double intakeDownPower = 0.7;
    private double intakeUpPower = 0.3;
    
    private boolean isIntakeDown = false;
    
    private float bottomEncoderPositionL = 0;
    private float bottomEncoderPositionR = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        imu = hardwareMap.get(IMU.class, "imu");
        
        intakeServo0 = hardwareMap.get(Servo.class, "insvl");
        intakeServo1 = hardwareMap.get(Servo.class, "insvr");
        outtakeServo0 = hardwareMap.get(Servo.class, "outsvl");
        outtakeServo1 = hardwareMap.get(Servo.class, "outsvr");
        jawServo = hardwareMap.get(Servo.class, "sp");
        
        linearSlideL = hardwareMap.get(DcMotor.class, "lsl");
        linearSlideR = hardwareMap.get(DcMotor.class, "lsr");
        intakeSlide = hardwareMap.get(DcMotor.class, "ext");
        intakeMotor = hardwareMap.get(DcMotor.class, "int");
        
        slideLimitL = hardwareMap.get(TouchSensor.class, "liml");
        slideLimitR = hardwareMap.get(TouchSensor.class, "limr");
        
        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "li");

        // Initialize IMU

        RevHubOrientationOnRobot imuOrientation = new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(imuOrientation));
        
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        initialYaw = orientation.getYaw(AngleUnit.RADIANS);

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
        
        linearSlideL.setDirection(DcMotor.Direction.FORWARD);
        linearSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideR.setDirection(DcMotor.Direction.FORWARD);
        linearSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlide.setDirection(DcMotor.Direction.FORWARD);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        
        bottomEncoderPositionL = linearSlideL.getCurrentPosition();
        bottomEncoderPositionR = linearSlideR.getCurrentPosition();

        ((PwmControl) intakeServo0).setPwmEnable();
        ((PwmControl) intakeServo1).setPwmEnable();
        ((PwmControl) outtakeServo0).setPwmEnable();
        ((PwmControl) outtakeServo1).setPwmEnable();
        ((PwmControl) jawServo).setPwmEnable();
        
        intakeServo0.setPosition(lServoIn);
        intakeServo1.setPosition(rServoIn);
        
        outtakeServo0.setPosition(lServoOutIn);
        outtakeServo1.setPosition(rServoOutIn);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Check slide limits
            float currentEncoderL = linearSlideL.getCurrentPosition();
            float currentEncoderR = linearSlideR.getCurrentPosition();
            if (slideLimitL.isPressed()) {
              bottomEncoderPositionL = currentEncoderL;
            }
            if (slideLimitR.isPressed()) {
              bottomEncoderPositionR = currentEncoderR;
            }
            
            double powerMult = 0.5;
            if (gamepad1.y) {
              linearSlideR.setPower(-powerMult);
            } else if (gamepad1.a) {
              linearSlideR.setPower(powerMult);
            } else {
              linearSlideR.setPower(0);
            }
            
            if (gamepad1.dpad_up) {
              linearSlideL.setPower(powerMult);
            } else if (gamepad1.dpad_down) {
              linearSlideL.setPower(-powerMult);
            } else {
              linearSlideL.setPower(0);
            }
            
            double intakePower = isIntakeDown ? intakeDownPower : intakeUpPower;
            
            if (gamepad1.left_bumper) {
              intakeMotor.setPower(-intakePower);
            } else if (gamepad1.right_bumper) {
              intakeMotor.setPower(intakePower);
            } else {
              intakeMotor.setPower(0);
            }
            
            if (gamepad1.dpad_left) {
              intakeSlide.setPower(-1);
            } else if (gamepad1.dpad_right) {
              intakeSlide.setPower(1);
            } else {
              intakeSlide.setPower(0);
            }
            
            if (gamepad1.x) {
              outtakeServo0.setPosition(lServoOutIn);
              outtakeServo1.setPosition(rServoOutIn);
            } else if (gamepad1.b) {
              outtakeServo0.setPosition(lServoOutOut);
              outtakeServo1.setPosition(rServoOutOut);
            }
            
            double max, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            if (!povDrive) {
                YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
                double robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS) - initialYaw + Math.PI;
                double radians = Math.atan2(axial, lateral);
                double power = Math.sqrt(axial * axial + lateral * lateral);
                if (power > 0.05) {
                  power = Math.max(Math.pow(power, 2), 0.2);
                } else {
                  power = 0;
                }
                telemetry.addData("Power", power);
                radians -= robotYaw;
                if (radians > 2 * Math.PI) radians -= 2 * Math.PI;
                if (radians < 0) radians += 2 * Math.PI;
                axial = Math.sin(radians) * power;
                lateral = Math.cos(radians) * power;
            }

            leftFrontPower  = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower   = axial - lateral + yaw;
            rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Slide Position R", linearSlideR.getCurrentPosition() - bottomEncoderPositionR);
            telemetry.addData("Slide Position L", linearSlideL.getCurrentPosition() - bottomEncoderPositionL);
            telemetry.addData("Left Slide Limit", slideLimitL.isPressed());
            telemetry.addData("Right Slide Limit", slideLimitR.isPressed());
            telemetry.update();
        }
    }}
