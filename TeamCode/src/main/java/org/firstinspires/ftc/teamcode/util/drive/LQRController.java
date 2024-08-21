package org.firstinspires.ftc.teamcode.util.drive;

import org.firstinspires.ftc.teamcode.util.lib.FtcDashboardManager;
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
        K = calculateLQRGain();

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
        if (K == null) return new double[]{0, 0};

        // Calculate the state error
        RealMatrix error = targetState.subtract(currentState);

        // Calculate the control input (u = -K * error)
        RealMatrix controlInput = K.multiply(error).scalarMultiply(-1);

        // Assume controlInput contains power values for left and right motors
        FtcDashboardManager.addData("control", controlInput);
        double power = controlInput.getEntry(0, 0) / 100;

        // Return the motor powers as a 2-element array
        return new double[]{power, power};
    }

    private RealMatrix calculateLQRGain() {
        RealMatrix A = MatrixUtils.createRealMatrix(LQRConstants.getA());
        RealMatrix B = MatrixUtils.createRealMatrix(LQRConstants.getB());
        RealMatrix Q = MatrixUtils.createRealDiagonalMatrix(LQRConstants.getQ());
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