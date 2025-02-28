package org.firstinspires.ftc.teamcode.ITD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Controller scheme: https://www.padcrafter.com/?templates=LM3+Scheme&plat=0&yButton=Override+slide+limit&leftTrigger=Vertical+slide+down&rightTrigger=Vertical+slide+up&leftBumper=Intake+spin+out&rightBumper=Intake+spin+in&col=%23242424%2C%23606A6E%2C%23FFFFFF&xButton=Light+pattern+1&bButton=Light+pattern+2&leftStickClick=&leftStick=Move&rightStickClick=Turn&rightStick=Change+team+color

// Want this scheme: https://www.padcrafter.com/?templates=Controller+Scheme+1&plat=0&col=%23242424%2C%23606A6E%2C%23FFFFFF&rightTrigger=Towers+Up&rightBumper=Specimen+Claw+Grab&leftBumper=Specimen+Claw+Release&leftTrigger=Towers+Down&leftStickClick=&bButton=Intake+Eject&xButton=Intake+Align+Vertical+Sample&yButton=Lower+Intake&aButton=Raise+Intake&rightStick=Rotational+Movement+%2F+Extend+%26+Retract+Intake&leftStick=Lateral+Movement+%28Field+Relative%29&dpadUp=Lower+Bucket&dpadDown=Raise+Bucket&dpadRight=Move+to+Human+Player&dpadLeft=Move+to+Submersible

@TeleOp(name = "PID Calibration", group = "Linear OpMode")
public class PIDCalibration extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware hardware;



    @Override
    public void runOpMode() {
        hardware = new RobotHardware(hardwareMap);
        RobotHardware.telemetry = telemetry;

        Waypoint origin = new Waypoint(0, 0, 0, 0, 0, 0, false, 0, hardware);
        Waypoint moveTest = new Waypoint(0, 10, 0, 0, 0, 0, false, 0, hardware);
        Waypoint strafeTest = new Waypoint(20, 0, 90, 0, 0, 0, false, 0, hardware);
        Waypoint yawTest = new Waypoint(0, 0, 180, 0, 0, 0, false, 0, hardware);

        hardware.backElevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backElevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.backElevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backElevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        double p = 0.15;
        double i = 0;
        double d = 0.02;

        Waypoint target = origin;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            deltaTime = runtime.seconds() - lastRuntime;
            lastRuntime = runtime.seconds();
            hardware.odometry.update();

            origin.movePID.P = p;
            origin.movePID.I = i;
            origin.movePID.D = d;
            moveTest.movePID.P = p;
            moveTest.movePID.I = i;
            moveTest.movePID.D = d;
            strafeTest.movePID.P = p;
            strafeTest.movePID.I = i;
            strafeTest.movePID.D = d;

            if (gamepad1.guide) {
                p = 0;
                i = 0;
                d = 0;
            }

            if (gamepad1.dpad_up) {
                p += deltaTime * 0.1;
            }
            if (gamepad1.dpad_down) {
                p -= deltaTime * 0.1;
            }
            if (gamepad1.left_stick_y > 0.1) {
                i += gamepad1.left_stick_y * deltaTime * 0.05;
            }
            if (gamepad1.left_stick_y < -0.1) {
                i += gamepad1.left_stick_y * deltaTime * 0.05;
            }
            if (gamepad1.right_stick_y > 0.1) {
                d += gamepad1.right_stick_y * deltaTime * 0.05;
            }
            if (gamepad1.right_stick_y < -0.1) {
                d += gamepad1.right_stick_y * deltaTime * 0.05;
            }

            if (gamepad1.y) {
                target = moveTest;
                target.run(deltaTime);
            }
            if (gamepad1.a) {
                target = yawTest;
                target.run(deltaTime);
            }
            if (gamepad1.b) {
                target = strafeTest;
                target.run(deltaTime);
            }
            if (gamepad1.x) {
                target = origin;
                target.run(deltaTime);
            }

            if (!target.hasFinished()) {
                target.run(deltaTime);
            }
            else {
                hardware.leftFrontDrive.setPower(0);
                hardware.leftBackDrive.setPower(0);
                hardware.rightFrontDrive.setPower(0);
                hardware.rightBackDrive.setPower(0);
            }


            telemetry.addData("-", "----------------------");
            telemetry.addData("Target", target);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("Deltatime", deltaTime);
            telemetry.addData("-", "----------------------");
            telemetry.addData("X", hardware.odometry.positionX());
            telemetry.addData("Y", hardware.odometry.positionY());
            telemetry.addData("Yaw", hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Slide", hardware.backElevatorRight.getCurrentPosition());
            telemetry.update();

        }
    }
}
