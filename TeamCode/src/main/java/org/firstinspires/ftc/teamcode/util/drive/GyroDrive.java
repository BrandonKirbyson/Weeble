package org.firstinspires.ftc.teamcode.util.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.head.HeadConstants;
import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;

public class GyroDrive {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private DriveState lastState = DriveState.STOPPED;
    private DriveState state = DriveState.STOPPED;

    private YawPitchRollAngles angles;

    private final LQRController controller;

    private final Pose pose = new Pose();

    private boolean lastBalanced = false;

    private double targetVel = 0;
    private double currentVel = 0;

    private double rotationVel = 0;

    private double lastPos = 0;
    private double currentPos = 0;

    private double lastTime = 0;

    private boolean emergencyStop = false;

    public GyroDrive(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new LQRController();
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

    public void update(YawPitchRollAngles angles, double pitchRate, double headAngle) {
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

        if (LQRConstants.UpdateLQRGains) {
            controller.updateK();
            LQRConstants.UpdateLQRGains = false;
        }

        updateCurrentVelocity();

        double currentAngle = angles.getPitch(AngleUnit.DEGREES);

        RealMatrix currentState = getCurrentState(currentAngle, pitchRate, currentVel);

        RealMatrix targetState = getTargetState(headAngle);

        double[] output = controller.calculateOutputPowers(currentState, targetState);

        setPower(output[0] + rotationVel, output[1] - rotationVel);

        updateState();

        FtcDashboardManager.addData("Angle", currentAngle);
        FtcDashboardManager.addData("AngleError", currentAngle);
        FtcDashboardManager.addData("PitchRate", pitchRate);
        FtcDashboardManager.addData("CurrentVelocity", currentVel);
        FtcDashboardManager.addData("TargetVelocity", targetVel);
        FtcDashboardManager.addData("Turn", rotationVel);
        FtcDashboardManager.addData("Output", output[0] + " | " + output[1]);
        FtcDashboardManager.addData("K", controller.getK());
        overlayRobot();
        lastTime = System.currentTimeMillis();
    }

    private void updateCurrentVelocity() {
        currentPos = (double) (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2 / LQRConstants.TICKS_PER_M;

        currentVel = (currentPos - lastPos) / (System.currentTimeMillis() - lastTime);

        lastPos = currentPos;
    }

    public RealMatrix getCurrentState(double angle, double pitchRate, double currentVelocity) {
        // Construct the state vector [pitchAngle, pitchRate, currentVelocity, ...]
        return MatrixUtils.createRealMatrix(new double[][]{
                {angle},
                {pitchRate},
                {currentPos},
                {currentVelocity}
        });
    }

    private RealMatrix getTargetState(double headAngle) {
        double targetAngle = LQRConstants.TargetAngle + (headAngle - HeadConstants.xCenter) * BalanceConstants.HeadAngleConversion;

        return MatrixUtils.createRealMatrix(new double[][]{
                {targetAngle}, // pitch angle
                {0}, // pitch rate
                {lastPos}, // position
                {targetVel} // velocity
        });
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
