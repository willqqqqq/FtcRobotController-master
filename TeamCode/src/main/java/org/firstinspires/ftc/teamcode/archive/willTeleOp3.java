package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

@Disabled
@TeleOp (name = "willTeleOp3", group = "Teleop")
public class willTeleOp3 extends BasicOpMode_Linear
{

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    //private DcMotor flyLeft;
    //private DcMotor flyRight;
    //private Servo armServo;
    //private CRServo slideServo;

    @Override
    public void runOpMode()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        //flyLeft = hardwareMap.dcMotor.get("flyLeft");
        //flyRight = hardwareMap.dcMotor.get("flyRight");
        //armServo = hardwareMap.servo.get("armServo");
        //slideServo = hardwareMap.crservo.get("slideServo");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        //flyRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();



        while(opModeIsActive()) {

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

            idle();
        }

    }

}


