package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

@Disabled
@TeleOp (name = "willTeleOp2", group = "Tutorials")
public class willTeleOp2 extends BasicOpMode_Linear
{

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    //private DcMotor armMotor;
    //private Servo armServo;
    //private CRServo slideServo;

    @Override
    public void runOpMode() //throws InterruptedException
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        //armMotor = hardwareMap.dcMotor.get("armMotor");
        //armServo = hardwareMap.servo.get("armServo");
        //slideServo = hardwareMap.crservo.get("slideServo");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.left_stick_y > .1) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
            }
            if (gamepad1.left_stick_y < -.1) {
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(-1);
            }
            if (gamepad1.left_stick_x > .1) {
                frontLeft.setPower(-1);
                frontRight.setPower(1);
                backLeft.setPower(-1);
                backRight.setPower(1);
            }
            if (gamepad1.left_stick_x < -.1) {
                frontLeft.setPower(1);
                frontRight.setPower(-1);
                backLeft.setPower(1);
                backRight.setPower(-1);
            }
            if (gamepad1.dpad_right)
            {
                frontLeft.setPower(-1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(-1);
            }
            if (gamepad1.dpad_left)
            {
                frontLeft.setPower(1);
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(1);
            }
            //slideServo.setPower(gamepad2.left_stick_y);

            /*
            armMotor.setPower(-gamepad2.right_stick_y);

            if(gamepad2.dpad_up)
            {
                armMotor.setPower(.7);
            }
            if(gamepad2.dpad_down)
            {
                armMotor.setPower(-.7);
            }
            if(gamepad2.left_bumper)
            {
                armServo.setPosition(1);
            }
            if(gamepad2.right_bumper)
            {
                armServo.setPosition(.5);
            }
            */
            else
            {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

            }

            idle();
        }

    }

}


