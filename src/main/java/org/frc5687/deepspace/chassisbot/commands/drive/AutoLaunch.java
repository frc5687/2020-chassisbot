package org.frc5687.deepspace.chassisbot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.chassisbot.Robot;
import org.frc5687.deepspace.chassisbot.commands.drive.AutoDrive;

public class AutoLaunch extends CommandGroup {
    public AutoLaunch(Robot robot) {
        addSequential(new AutoDrive(robot.getSparkMaxDriveTrain(),robot.getIMU(),24,.5,false, true, 0, "", 2000));
    }
}
