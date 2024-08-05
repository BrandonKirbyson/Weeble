package org.firstinspires.ftc.teamcode.util.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;

public class GyroDrive {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private DriveState lastState = DriveState.STOPPED;
    private DriveState state = DriveState.STOPPED;

    private YawPitchRollAngles angles;

    private final Pose pose = new Pose();

    private boolean lastBalanced = false;

    private double targetVel = 0;
    private double rotationVel = 0;

    private boolean emergencyStop = false;

    public GyroDrive(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initIMU(IMU imu) {
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
    }

    public Pose getPose() {
        return pose;
    }

    public void emergencyStop() {
        emergencyStop = true;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void drive(double drivePower, double turnPower, boolean fast) {
        targetVel = drivePower * (fast ? SpeedConstants.FastDrive : SpeedConstants.Drive);
        rotationVel = turnPower * (fast ? SpeedConstants.FastTurn : SpeedConstants.Turn);
    }

    public void setTargetAngle(double target) {
    }

    public void update(YawPitchRollAngles angles) {
        this.angles = angles;

        if (!isBalanced() || emergencyStop) {
            if (!isBalanced() && emergencyStop) emergencyStop = false;
            stopMotors();
            state = DriveState.STOPPED;
            lastBalanced = false;
            return;
        } else if (!lastBalanced) {
            lastBalanced = true;
        }

        double currentAngle = angles.getPitch(AngleUnit.DEGREES);

        //Use LQRConstants
        /*
 u[0] = ( k[0]*(encoder_set_point  - average_theta)
          +k[1]*(pitch_set_point-pitch)
          +k[2]*(velocity_set_point-average_RPM)
          +k[3]*(pitch_dot_set_point-pitch_dot));

  u[1] = (k1[0]*yaw +k1[1]*yaw_dot) ;
  v[0] = (0.5 *(u[0] + u[1])); // for right motor
  v[1] = (0.5 *(u[0] - u[1])); // for left motor
         */


//        setPower(outputPower + rotSpeed, outputPower - rotSpeed);

        updateState();

        FtcDashboardManager.addData("Angle", currentAngle);
        FtcDashboardManager.addData("AngleError", currentAngle);
        FtcDashboardManager.addData("Turn", rotationVel);
//        FtcDashboardManager.addData("Output", outputPower);
        overlayRobot();
    }

    private void updateState() {
        lastState = state;
        if (isBalanced()) {
            if (targetVel == 0) {
                state = DriveState.IDLE;
            } else {
                state = DriveState.DRIVING;
            }
        } else {
            state = DriveState.STOPPED;
        }
    }

    private void setPower(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public boolean isBalanced() {
        return Math.abs(angles.getPitch(AngleUnit.DEGREES) - BalanceConstants.TargetAngle) < BalanceConstants.MaxAngle;
    }

    public void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public DriveState getLastState() {
        return lastState;
    }

    public DriveState getState() {
        return state;
    }

    private void overlayRobot() {
        FtcDashboardManager.getPacket().fieldOverlay()
                .setFill("white")
                .setStroke("green")
                .fillCircle(pose.x, pose.y, 5)
                .strokeLine(pose.x, pose.y, Math.cos(Math.toRadians(pose.heading)) * 4, Math.sin(Math.toRadians(pose.heading)) * 4);
    }
}
