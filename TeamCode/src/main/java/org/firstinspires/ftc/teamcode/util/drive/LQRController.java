package org.firstinspires.ftc.teamcode.util.drive;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class LQRController {
    private RealMatrix K;
    private Future<RealMatrix> futureK;
    private final ExecutorService executor;

    public LQRController() {
        calculateLQRGain();

        executor = Executors.newSingleThreadExecutor();
    }

    public RealMatrix getK() {
        if (futureK != null && futureK.isDone()) {
            try {
                K = futureK.get();
                futureK = null;
            } catch (InterruptedException | ExecutionException e) {
                //noinspection CallToPrintStackTrace
                e.printStackTrace();
            }
        }
        return K;
    }

    public double[] calculateOutputPowers(RealMatrix currentState, RealMatrix targetState) {
        // Calculate the state error
        RealMatrix error = targetState.subtract(currentState);

        // Calculate the control input (u = -K * error)
        RealMatrix controlInput = K.multiply(error).scalarMultiply(-1);

        // Assume controlInput contains power values for left and right motors
        double leftPower = controlInput.getEntry(0, 0);
        double rightPower = controlInput.getEntry(1, 0);

        // Return the motor powers as a 2-element array
        return new double[]{leftPower, rightPower};
    }

    public RealMatrix getCurrentState(YawPitchRollAngles angles, double currentVelocity) {
        double pitchAngle = angles.getPitch(AngleUnit.RADIANS);  // Pitch angle in radians
        double pitchRate = 0;
        // Construct the state vector [pitchAngle, pitchRate, currentVelocity, ...]
        return MatrixUtils.createRealMatrix(new double[][]{
                {pitchAngle},
                {pitchRate},
                {currentVelocity}
        });
    }

    private RealMatrix calculateLQRGain() {
        RealMatrix A = MatrixUtils.createRealMatrix(LQRConstants.A);
        RealMatrix B = MatrixUtils.createRealMatrix(LQRConstants.B);
        RealMatrix Q = MatrixUtils.createRealDiagonalMatrix(LQRConstants.Q);
        RealMatrix R = MatrixUtils.createRealMatrix(new double[][]{{LQRConstants.R}});

        // Solve the Riccati equation
        RiccatiEquationSolver solver = new RiccatiEquationSolverImpl(A, B, Q, R);
        return solver.getK();
    }

    public void updateK() {
        if (futureK == null || futureK.isDone()) {
            futureK = executor.submit(this::calculateLQRGain);
        }
    }

    public void shutdown() {
        if (executor != null && !executor.isShutdown()) {
            executor.shutdown();
        }
    }
}