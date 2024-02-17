package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
@TeleOp
public class combination extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo handleServo = hardwareMap.servo.get("handleServo");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo airplaneServo = hardwareMap.servo.get("airplaneServo");
        DcMotor dcmotorTop = hardwareMap.dcMotor.get("armMotorTop");
        DcMotor dcmotorBottom = hardwareMap.dcMotor.get("armMotorBottom");
        double handlePosition = 0.0;
        double clawPosition = 0.0;
        double airplanePosiiton = 0.0;
        double holdUp = 0.3;
        double time = 0.0;
        double ctime = 0.0;
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        boolean robotCentric = true;

        while (opModeIsActive()) {

            double frontLeftPower;
            double backLeftPower;
            double frontRightPower;
            double backRightPower;
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            boolean sup = gamepad1.dpad_up;
            boolean sdown = gamepad1.dpad_down;
            boolean copen = gamepad1.a;

            boolean hardCode = gamepad1.b;


            double armUp = gamepad1.right_trigger;
            double armDown = gamepad1.left_trigger;

            boolean launch = gamepad1.right_bumper;

            double motorPosition = 0;
            double speed = 0.4;

            if (gamepad1.left_bumper) {
                handlePosition = 0;
                clawPosition = 0;
                airplanePosiiton = 0;
            }
            /*if (gamepad1.dpad_up) holdUp += 0.0001;
            if (gamepad1.dpad_down) holdUp -= 0.0001;
            telemetry.addData("Hold Up Value: Press Dpad Up and Down", holdUp);*/

            dcmotorTop.setPower((-armUp + armDown)/2);
            dcmotorBottom.setPower((armUp - armDown)/2);

            if (armUp == 0 && armDown == 0) {
                dcmotorTop.setPower(holdUp);
                dcmotorBottom.setPower(holdUp);
            }
            telemetry.addData("Arm Position: Press X and B ", motorPosition);

            if (sup) {
                if (handleServo.getPosition() < 2)  handlePosition += 0.008;
            }
            else if (sdown) {
                if (handleServo.getPosition() > 0) handlePosition -= 0.008;
            }

            if (hardCode) handlePosition = 0.35;
            ctime += 1;
            if (copen) {
                if (clawServo.getPosition() == 0 && ctime > 20) {
                    clawPosition = 0.27;
                    ctime = 0;
                }
                else if (clawServo.getPosition() == 0.27 && ctime > 20) {
                    clawPosition = 0;
                    ctime = 0;
                }
            }
            telemetry.addData("Handle Position", handlePosition);
            telemetry.addData("Pressing A", copen);
            telemetry.addData("Toggle Time Claw", ctime);

            if (launch) {
                airplanePosiiton = 2;
            }

            handleServo.setPosition(handlePosition);
            clawServo.setPosition(clawPosition);
            airplaneServo.setPosition(airplanePosiiton);
            telemetry.addData("Tilt Position: Press Y", handlePosition);
            telemetry.addData("Claw Position: Press A", clawPosition);
            telemetry.addData("Airplane Position: Press Right Bumper", airplanePosiiton);

            if (gamepad1.left_stick_button) robotCentric = !robotCentric;

            if (!robotCentric) {
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                x *= Math.cos(-botHeading) - y * Math.sin(-botHeading);
                y *= Math.sin(-botHeading) + y * Math.cos(-botHeading);

                x *= 1.1;  // Counteract imperfect strafing
            }

            telemetry.addData("Robot Centric (Press Left Stick Button)", robotCentric);
            telemetry.update();
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
