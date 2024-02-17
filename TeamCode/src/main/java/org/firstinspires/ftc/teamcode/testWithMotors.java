package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class testWithMotors extends LinearOpMode{
    public void runOpMode() {
        Servo handleServo = hardwareMap.servo.get("handleServo");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        double handlePosition = 0.0;
        double clawPosition = 0.0;
        double holdUp = 0.00;
        waitForStart();
        while (opModeIsActive()) {

            boolean sup = gamepad1.y;
            boolean sdown = gamepad1.a;

            boolean copen = gamepad1.dpad_right;
            boolean cclose = gamepad1.dpad_left;

            if (gamepad1.right_stick_button) {
                handlePosition = 0;
                clawPosition = 0;
            }

            if (gamepad1.dpad_up) holdUp += 0.01;
            if (gamepad1.dpad_down) holdUp -= 0.01;
            telemetry.addData("Hold Up Value", holdUp);

            if (sup) {
                handlePosition += 0.005;
            }
            else if (sdown) {
                handlePosition -= 0.005;
            }

            if (copen) {
                clawPosition += 0.005;
            }
            else if (cclose) {
                clawPosition -= 0.005;
            }
            handleServo.setPosition(handlePosition);
            clawServo.setPosition(clawPosition);
            telemetry.addData("Servo Position", handlePosition);
            telemetry.addData("Claw Position", clawPosition);
            telemetry.update();
        }
    }

}
