package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public interface SysIdDrivetrain {
    public List<WPI_TalonFX> getLeftMotors();

    public List<WPI_TalonFX> getRightMotors();

    public double getLeftPosition();

    public double getRightPosition();

    public double getLeftVelocity();

    public double getRightVelocity();

    public double getGyroAngle();

    public double getGyroRate();
}
