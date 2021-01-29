package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

@TeleOp (name = "willTeleOp4", group = "Teleop")
public class willTeleOp4 extends BasicOpMode_Linear
{

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotor flyWheel;
    private DcMotor ringLift;
    private Servo wobbleLift;
    private Servo wobbleGrab;
    private Servo ringGrab;
    private Servo ringShoot;

    @Override

    public void runOpMode()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        flyWheel = hardwareMap.dcMotor.get("flyWheel");
        intake = hardwareMap.dcMotor.get("intake");
        wobbleLift = hardwareMap.servo.get("wobbleLift");
        wobbleGrab = hardwareMap.servo.get("wobbleGrab");
        ringShoot = hardwareMap.servo.get("ringShoot");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        ringShoot.setPosition(0.5);

        wobbleLift.setPosition(.5);
        wobbleGrab.setPosition(1);

        waitForStart();

        while(opModeIsActive()) {

            ringShoot.setPosition(.5);

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

            if(gamepad1.right_bumper) {
                frontLeft.setPower(v1 * .05);
                frontRight.setPower(v2 * .05);
                backLeft.setPower(v3 * .05);
                backRight.setPower(v4 * .05);
            }
            if(gamepad2.right_trigger > .5){
                ringShoot.setPosition(1);
                sleep(400);
            }
            else{
                ringShoot.setPosition(0.5);
            }
            if (gamepad2.left_trigger > .5){
                wobbleGrab.setPosition(1);
            }
            if (gamepad2.dpad_up){
                wobbleLift.setPosition(0);
            }
            if (gamepad2.dpad_down){
                wobbleLift.setPosition(1);
            }
            if (gamepad2.dpad_left){
                wobbleGrab.setPosition(1);
            }
            if (gamepad2.dpad_right){
                wobbleGrab.setPosition(.5);
            }
            if (gamepad2.a){
                flyWheel.setPower(.8);
            }
            if(gamepad2.b){
                flyWheel.setPower(0);
            }
            idle();
        }

    }

}


