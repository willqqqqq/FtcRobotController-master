package org.firstinspires.ftc.teamcode.archive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

@Autonomous(name = "shoot", group = "Autonomous")
public class shoot extends BasicOpMode_Linear
{

    private DcMotor fly1;
    private DcMotor fly2;

    @Override
    public void runOpMode() //throws InterruptedException
    {
        fly1 = hardwareMap.dcMotor.get("fly1");
        fly2 = hardwareMap.dcMotor.get("fly2");
        //fly2.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        //-------------------------------------------------------------------
        spinTime(100000,1);
        //-------------------------------------------------------------------
    }

    public void spin (double power) {
        fly1.setPower(power);
        fly2.setPower(power);

    }

    public void spinTime (long time,double power) {
        fly1.setPower(power);
        fly2.setPower(power);

        sleep(time);

        fly1.setPower(0);
        fly2.setPower(0);
    }

}
