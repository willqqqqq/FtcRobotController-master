package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

@Disabled
@TeleOp (name = "shootingTele", group = "Tutorials")
public class shootingTele extends BasicOpMode_Linear
{

    private DcMotor flyLeft;
    private DcMotor flyRight;

    //private DcMotor armMotor;
    //private Servo armServo;

    @Override
    public void runOpMode() //throws InterruptedException
    {
        flyLeft = hardwareMap.dcMotor.get("frontLeft");
        flyRight = hardwareMap.dcMotor.get("frontRight");
        //armServo = hardwareMap.servo.get("armServo");
        //slideServo = hardwareMap.crservo.get("slideServo");
        flyRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.a){
                flyLeft.setPower(1);
                flyRight.setPower(1);
            }


            idle();
        }

    }

}


