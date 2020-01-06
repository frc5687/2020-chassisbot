package org.frc5687.deepspace.chassisbot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.subsystems.DriveTrain;

public class AutoAlign extends PIDCommand {

    public AutoAlign(DriveTrain driveTrain, AHRS imu, double angle, double speed, long timeout, double tolerance) {
       super(
               new PIDController(Constants.Auto.Align.kP, Constants.Auto.Align.kI, Constants.Auto.Align.kD),
               driveTrain::getYaw,
               angle,
               output -> driveTrain.cheesyDrive(speed, output, false, true),
               driveTrain
       );
       getController().enableContinuousInput(Constants.Auto.MIN_IMU_ANGLE,Constants.Auto.MAX_IMU_ANGLE);
       getController().setTolerance(tolerance);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

//    @Override
//    public void end(boolean interrupt) {
//        super.end(interrupt);
//    }

}



