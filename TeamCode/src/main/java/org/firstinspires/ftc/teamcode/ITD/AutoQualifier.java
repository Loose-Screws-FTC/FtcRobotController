package org.firstinspires.ftc.teamcode.ITD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Controller scheme: https://www.padcrafter.com/?templates=LM3+Scheme&plat=0&yButton=Override+slide+limit&leftTrigger=Vertical+slide+down&rightTrigger=Vertical+slide+up&leftBumper=Intake+spin+out&rightBumper=Intake+spin+in&col=%23242424%2C%23606A6E%2C%23FFFFFF&xButton=Light+pattern+1&bButton=Light+pattern+2&leftStickClick=&leftStick=Move&rightStickClick=Turn&rightStick=Change+team+color

// Want this scheme: https://www.padcrafter.com/?templates=Controller+Scheme+1&plat=0&col=%23242424%2C%23606A6E%2C%23FFFFFF&rightTrigger=Towers+Up&rightBumper=Specimen+Claw+Grab&leftBumper=Specimen+Claw+Release&leftTrigger=Towers+Down&leftStickClick=&bButton=Intake+Eject&xButton=Intake+Align+Vertical+Sample&yButton=Lower+Intake&aButton=Raise+Intake&rightStick=Rotational+Movement+%2F+Extend+%26+Retract+Intake&leftStick=Lateral+Movement+%28Field+Relative%29&dpadUp=Lower+Bucket&dpadDown=Raise+Bucket&dpadRight=Move+to+Human+Player&dpadLeft=Move+to+Submersible

@Autonomous(name = "Qualifier Auto", group = "Linear OpMode")
public class AutoQualifier extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware hardware;

    static double releasePosition = 0.5;
    static double grabPosition = 0;

    static double maxSlidePosition = -7500;

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
    private double intakeUpPower = 0.4;

    private boolean isIntakeDown = false;

    private float bottomEncoderPositionL = 0;
    private float bottomEncoderPositionR = 0;

    private double toDegrees(double rad) {
        double deg = rad * 180.0 / Math.PI;
        return deg;
    }

    private double wrapToPI(double dYaw) {
        if (dYaw < -180)
            dYaw += 360; // fix wraparound
        if (dYaw > 180)
            dYaw -= 360; // from +179 to -179

        return dYaw;
    }

    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        hardware.leftFrontDrive.setPower(leftFrontPower);
        hardware.rightFrontDrive.setPower(rightFrontPower);
        hardware.leftBackDrive.setPower(leftBackPower);
        hardware.rightBackDrive.setPower(rightBackPower);
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getDist(double x0, double y0, double x1, double y1) {
        return Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    }

    public int getNull(Waypoint[] waypoints) {
        for (int i = 0; i < waypoints.length; i++) {
            if (waypoints[i] == null)
                return i;
        }
        return -1;
    }

    // Method to get the robot's current yaw angle
    private double getFacing() {
        YawPitchRollAngles angles = hardware.imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void runOpMode() {
        hardware = new RobotHardware(hardwareMap);
        RobotHardware.telemetry = telemetry;

        hardware.backElevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backElevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backElevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backElevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

/*
coords = [
    [34, -28, 0],
    [2, -26, 180],
    [2, -54, 180],
    [-8, -54, 180],
    [-8, -10, 180],
    [-8, -54, 180],
    [-18, -54, 180],
    [-18, -10, 180],
    [-18, -54, 180],
    [-28, -54, 180],
    [-28, -10, 180],
    [-4, -10, 180],
    [-4, 0, 180]
]

for x, y, yaw in coords:
    x -= 21
    print(f"            new Waypoint({str(x): >3}, {str(y): >3}, {str(yaw): >3}, 0, 0, 0, true, 0, hardware),")

*/
        AutoStep[] steps = {
            // new Waypoint( 0,  0, 180, 0, 0, 0, true, 0, hardware),
            // new Waypoint( 0,  0,   0, 0, 0, 0, true, 0, hardware),
            // new DelayAutoStep(1, hardware),
            // new Waypoint(16, 16, 180, 0, 0, 0, true, 0, hardware),
            // new DelayAutoStep(1, hardware),
            // new Waypoint( 0,  0,   0, 0, 0, 0, true, 0, hardware)
            new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware), // Approach chamber
            new CustomAutoStep(() -> {}, () -> {
                double power = 0.2;
                hardware.leftFrontDrive.setPower(-power);
                hardware.rightFrontDrive.setPower(-power);
                hardware.leftBackDrive.setPower(-power);
                hardware.rightBackDrive.setPower(-power);
            }, 1),
            new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware), // Place specimen
            new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
            new Waypoint(-24, -8, 0, 0, 0, 0, false, 0, hardware),
            new Waypoint(-24, -8, 180, 0, 0, 0, false, 0, hardware),
            new DelayAutoStep(1, hardware),
            new CustomAutoStep(() -> {}, () -> {
                double power = 0.2;
                hardware.leftFrontDrive.setPower(-power);
                hardware.rightFrontDrive.setPower(-power);
                hardware.leftBackDrive.setPower(-power);
                hardware.rightBackDrive.setPower(-power);
            }, 2),
            new CustomAutoStep(() -> {}, () -> {
                hardware.leftFrontDrive.setPower(0);
                hardware.rightFrontDrive.setPower(0);
                hardware.leftBackDrive.setPower(0);
                hardware.rightBackDrive.setPower(0);
                hardware.clawServo.setPosition(hardware.releasePosition);
            }, 1),
            new Waypoint(-24, -8, 180, 0, 0, 0, true, -1700, hardware),
            new Waypoint(-24, -8, 0, 0, 0, 0, true, -1700, hardware),
            new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
            new CustomAutoStep(() -> {}, () -> {
                double power = 0.2;
                hardware.leftFrontDrive.setPower(-power);
                hardware.rightFrontDrive.setPower(-power);
                hardware.leftBackDrive.setPower(-power);
                hardware.rightBackDrive.setPower(-power);
            }, 1),
            new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
            new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
            // new Waypoint(-24, -8, 0, 0, 0, 0, false, 0, hardware),
            // new Waypoint(-24, -8, 180, 0, 0, 0, false, 0, hardware),
            // new DelayAutoStep(0.5, hardware),
            // new Waypoint(-24, 0, 180, 0, 0, 0, false, 0, hardware),
            // new CustomAutoStep(() -> {}, () -> {
            //     double power = 0.2;
            //     hardware.leftFrontDrive.setPower(-power);
            //     hardware.rightFrontDrive.setPower(-power);
            //     hardware.leftBackDrive.setPower(-power);
            //     hardware.rightBackDrive.setPower(-power);
            // }, 0.5),
            // new CustomAutoStep(() -> {}, () -> {
            //     hardware.clawServo.setPosition(hardware.grabPosition);
            // }, 0.5),
            // new Waypoint(-24, -8, 180, 0, 0, 0, true, 0, hardware),
            // new Waypoint(-24, -8, 0, 0, 0, 0, true, 0, hardware),
            // new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
            // new CustomAutoStep(() -> {}, () -> {
            //     double power = 0.2;
            //     hardware.leftFrontDrive.setPower(-power);
            //     hardware.rightFrontDrive.setPower(-power);
            //     hardware.leftBackDrive.setPower(-power);
            //     hardware.rightBackDrive.setPower(-power);
            // }, 0.5),
            // new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
            // new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
            // new Waypoint(-24, -8, 0, 0, 0, 0, false, 0, hardware),
            // new Waypoint(-24, -8, 180, 0, 0, 0, false, 0, hardware),
            // new DelayAutoStep(0.5, hardware),
            // new Waypoint(-24, 0, 180, 0, 0, 0, false, 0, hardware),
            // new CustomAutoStep(() -> {}, () -> {
            //     double power = 0.2;
            //     hardware.leftFrontDrive.setPower(-power);
            //     hardware.rightFrontDrive.setPower(-power);
            //     hardware.leftBackDrive.setPower(-power);
            //     hardware.rightBackDrive.setPower(-power);
            // }, 0.5),
            // new Waypoint(-24, 0, 180, 0, 0, 0, true, 0, hardware),
            // new DelayAutoStep(0.5, hardware),
            // new Waypoint(-24, -8, 180, 0, 0, 0, true, 0, hardware),
            // new Waypoint(-24, -8, 0, 0, 0, 0, true, 0, hardware),
            // new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
            // new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
            // new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
            // new Waypoint(-24, -8, 0, 0, 0, 0, false, 0, hardware),
            // new Waypoint(-24, -8, 180, 0, 0, 0, false, 0, hardware),
            // new DelayAutoStep(0.5, hardware),
            // new Waypoint(-24, 0, 180, 0, 0, 0, false, 0, hardware),
            // new CustomAutoStep(() -> {}, () -> {
            //     double power = 0.2;
            //     hardware.leftFrontDrive.setPower(-power);
            //     hardware.rightFrontDrive.setPower(-power);
            //     hardware.leftBackDrive.setPower(-power);
            //     hardware.rightBackDrive.setPower(-power);
            // }, 0.5),
            // new Waypoint(-24, 0, 180, 0, 0, 0, true, 0, hardware),
            // new DelayAutoStep(0.5, hardware),
            // new Waypoint(-24, -8, 180, 0, 0, 0, true, 0, hardware),
            // new Waypoint(-24, -8, 0, 0, 0, 0, true, 0, hardware),
            // new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
            // new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
            // new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
            new Waypoint(-36, 0, 0, 0, 0, 0, false, 0, hardware) // Park in observation zone
        };
        
            /*
            new Waypoint( 13, -28, 0, 0, 0, 0, true, 0, hardware),
            new Waypoint(-19, -26, 0, 0, 0, 0, true, 0, hardware),
            new Waypoint(-19, -54, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-29, -54, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-29, -10, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-29, -54, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-39, -54, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-39, -10, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-39, -54, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-49, -54, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-49, -10, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-25, -10, 180, 0, 0, 0, true, 0, hardware),
            new Waypoint(-25,   0, 180, 0, 0, 0, true, 0, hardware),
            */
//         AutoStep[] steps = {
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
// new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
// new Waypoint(-24, -8, 0, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, -8, 180, 0, 0, 0, false, 0, hardware),
// new DelayAutoStep(0.5),
// new Waypoint(-24, 0, 180, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, 0, 180, 0, 0, 0, true, 0, hardware),
// new DelayAutoStep(0.5),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
// new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
// new Waypoint(-24, -8, 0, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, -8, 180, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, 0, 180, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, 0, 180, 0, 0, 0, true, 0, hardware),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
// new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
// new Waypoint(-24, -8, 0, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, -8, 180, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, 0, 180, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, 0, 180, 0, 0, 0, true, 0, hardware),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
// new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
// new Waypoint(-24, -8, 0, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, -8, 180, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, 0, 180, 0, 0, 0, false, 0, hardware),
// new Waypoint(-24, 0, 180, 0, 0, 0, true, 0, hardware),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1700, hardware),
// new Waypoint(12, -30, 0, 0, 0, 0, true, -1250, hardware),
// new Waypoint(12, -25, 0, 0, 0, 0, false, -1250, hardware),
// new Waypoint(0, 0, 0, 0, 0, 0, false, 0, hardware)
//         };

        int currentStepI = 0;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // =======================================================================================
        // UPDATE LOOP
        // =======================================================================================


        double lastRuntime = runtime.seconds();
        double deltaTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            deltaTime = runtime.seconds() - lastRuntime;
            lastRuntime = runtime.seconds();
            hardware.odometry.update();

            AutoStep currentStep = steps[currentStepI];
            currentStep.run(deltaTime);
            if (currentStep.hasFinished()) {
                currentStepI++;
                if (currentStepI < steps.length) {
                    steps[currentStepI].start();
                } else {
                    break;
                }
            }

            telemetry.addData("Current Step I", currentStepI);
            telemetry.addData("-", "----------------------");
            telemetry.addData("X", hardware.odometry.positionX());
            telemetry.addData("Y", hardware.odometry.positionY());
            telemetry.addData("Yaw", hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Slide", hardware.backElevatorRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
