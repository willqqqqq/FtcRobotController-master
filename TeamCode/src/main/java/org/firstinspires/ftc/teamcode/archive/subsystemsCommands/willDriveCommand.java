package org.firstinspires.ftc.teamcode.archive.subsystemsCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class willDriveCommand extends CommandBase {

    private willDriveSubsystem driveSubsystem;
    private DoubleSupplier strafe, forward, turn, heading;

    public willDriveCommand(willDriveSubsystem driveSubsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){

        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.heading = heading;

        addRequirements(driveSubsystem);

    }
    @Override
    public void execute(){

        driveSubsystem.drive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble(), heading.getAsDouble());

    }
}
