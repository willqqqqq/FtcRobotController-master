package org.firstinspires.ftc.teamcode.archive;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.archive.subsystemsCommands.willDriveCommand;
import org.firstinspires.ftc.teamcode.archive.subsystemsCommands.willDriveSubsystem;
@Disabled
@TeleOp(name = "willTeleOp5")
public class willTeleOp5 extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private GyroEx gyroSensor;
    private willDriveSubsystem driveSubsystem;
    private willDriveCommand driveCommand;

    private GamepadEx m_driverOp;

    @Override
    public void initialize() {

        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");


        m_driverOp = new GamepadEx(gamepad1);

        //driveSubsystem = new willDriveSubsystem(frontLeft, frontRight, backLeft, backRight);
        driveCommand = new willDriveCommand(driveSubsystem,  m_driverOp:: getLeftY, m_driverOp:: getLeftX, m_driverOp:: getRightX);

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);

    }
}
