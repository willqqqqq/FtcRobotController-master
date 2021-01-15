package org.firstinspires.ftc.teamcode.archive.subsystemsCommands;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class willDriveSubsystem extends SubsystemBase {

    private MecanumDrive drive;

    private Motor frontLeft, frontRight, backLeft, backRight;

    private GyroEx gyroSensor;

    private RevIMU imu;

    private boolean fieldCentric = false;

    public willDriveSubsystem(Motor frontL, Motor frontR, Motor backL, Motor backR, RevIMU revimu){

        frontLeft = frontL;
        frontRight = frontR;
        backLeft = backL;
        backRight = backR;
        fieldCentric = true;

        imu = revimu;


        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        


    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading){
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, imu.getHeading(), true);
    }

}
