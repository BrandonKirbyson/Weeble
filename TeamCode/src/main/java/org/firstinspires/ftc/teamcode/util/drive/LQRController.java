package org.firstinspires.ftc.teamcode.util.drive;

import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;

public class LQRController {
    private RealMatrix K;

    public LQRController() {
        calculateLQRGain();
    }

    public RealMatrix getK() {
        return K;
    }

    public void calculateLQRGain() {
        RealMatrix A = MatrixUtils.createRealMatrix(LQRConstants.A);
        RealMatrix B = MatrixUtils.createRealMatrix(LQRConstants.B);
        RealMatrix Q = MatrixUtils.createRealDiagonalMatrix(LQRConstants.Q);
        RealMatrix R = MatrixUtils.createRealMatrix(new double[][]{{LQRConstants.R}});

        // Solve the Riccati equation
        RiccatiEquationSolver solver = new RiccatiEquationSolverImpl(A, B, Q, R);
        K = solver.getK();
    }
}
