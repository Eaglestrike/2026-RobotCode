/*
 * Original C++ ballistic-solver:
 * MIT License
 * Copyright (c) 2026 ujinf74
 * https://github.com/ujinf74/ballistic-solver
 *
 * Java port:
 * Copyright (c) 2026 Jiarui Wu
 */

package frc.robot.subsystems.shooter.solver;

import java.io.IOException;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.solver2.BallisticClient;
import frc.robot.subsystems.shooter.solver2.BallisticClient.ServerSolverResult;

/**
 * Ballistic Solver - Computes launch angles to intercept moving targets
 * under gravity and quadratic air drag.
 * 
 * Ported from ballistic-solver-main (C++ implementation)
 * Uses RK4 integration + gradient descent optimization
 */
public class BallisticSolver implements ShooterSolver {

    // ================================================================
    // Vec3 - Simple 3D vector
    // ================================================================
    public static class Vec3 {
        public double x, y, z;

        public Vec3() {
            this(0, 0, 0);
        }

        public Vec3(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vec3 plus(Vec3 other) {
            return new Vec3(x + other.x, y + other.y, z + other.z);
        }

        public Vec3 minus(Vec3 other) {
            return new Vec3(x - other.x, y - other.y, z - other.z);
        }

        public Vec3 times(double s) {
            return new Vec3(s * x, s * y, s * z);
        }

        public double dot(Vec3 other) {
            return x * other.x + y * other.y + z * other.z;
        }

        public double norm() {
            return Math.sqrt(dot(this));
        }

        public Vec3 copy() {
            return new Vec3(x, y, z);
        }
    }

    // ================================================================
    // State for RK4
    // ================================================================
    private static class State {
        Vec3 r; // position
        Vec3 v; // velocity

        State() {
            r = new Vec3();
            v = new Vec3();
        }

        State copy() {
            State s = new State();
            s.r = r.copy();
            s.v = v.copy();
            return s;
        }
    }

    // ================================================================
    // Solver Result (internal)
    // ================================================================
    public static class SolverResult {
        public double theta = 0; // launch pitch angle (radians)
        public double phi = 0; // launch yaw angle (radians)
        public double miss = Double.MAX_VALUE; // miss distance (meters)
        public double tStar = 0; // time of closest approach (seconds)
        public int iterations = 0; // solver iterations used
        public boolean success = false; // whether miss <= tolerance
    }

    // ================================================================
    // Solver mode
    // ================================================================
    public enum SolverMode {
        LOCAL,
        CLIENT
    }

    // ================================================================
    // Instance fields (configurable per-robot)
    // ================================================================
    private final BallisticConstants params;
    private SolverMode mode;
    private BallisticClient ballisticClient;

    private final DoubleSubscriber lowRpmSub = DogLog.tunable("Shooter/LowRpm", 1350.0, x -> {});
    private final DoubleSubscriber mediumRpmSub = DogLog.tunable("Shooter/MediumRpm", 2000.0, x -> {});
    private final DoubleSubscriber highRpmSub = DogLog.tunable("Shooter/HighRpm", 4500.0, x -> {});
    private final DoubleSubscriber quadraticDragSub =
            DogLog.tunable("Shooter/QuadraticDrag", BallisticConstants.DEFAULT_DRAG_COEFFICIENT, x -> {});
    private final DoubleSubscriber rpmToInitialSpeedScale =
            DogLog.tunable("Shooter/RpmToInitialSpeedScale", 1.0, x -> {});

    /** Fit points for RPM→initialSpeed quadratic: v = A*rps² + B*rps + C (rps = RPM/60). */
    private static final double DEFAULT_FIT_RPM_1 = 1350.0;
    private static final double DEFAULT_FIT_RPM_2 = 2000.0;
    private static final double DEFAULT_FIT_RPM_3 = 4500.0;
    private final DoubleSubscriber fitRpm1 = DogLog.tunable("Shooter/QuadraticFit/Rpm1", DEFAULT_FIT_RPM_1, x -> {});
    private final DoubleSubscriber fitVel1 = DogLog.tunable("Shooter/QuadraticFit/Vel1",
            ShooterConstants.flywheelRPSToBallVel(DEFAULT_FIT_RPM_1 / 60.0), x -> {});
    private final DoubleSubscriber fitRpm2 = DogLog.tunable("Shooter/QuadraticFit/Rpm2", DEFAULT_FIT_RPM_2, x -> {});
    private final DoubleSubscriber fitVel2 = DogLog.tunable("Shooter/QuadraticFit/Vel2",
            ShooterConstants.flywheelRPSToBallVel(DEFAULT_FIT_RPM_2 / 60.0), x -> {});
    private final DoubleSubscriber fitRpm3 = DogLog.tunable("Shooter/QuadraticFit/Rpm3", DEFAULT_FIT_RPM_3, x -> {});
    private final DoubleSubscriber fitVel3 = DogLog.tunable("Shooter/QuadraticFit/Vel3",
            ShooterConstants.flywheelRPSToBallVel(DEFAULT_FIT_RPM_3 / 60.0), x -> {});

    // private final DoubleSubscriber spinLiftFactorSub = DogLog.tunable("Shooter/SpinLiftFactor", 0.0, x -> {});

    /** Create a BallisticSolver with the specified mode. */
    public BallisticSolver(SolverMode mode) {
        this.params = new BallisticConstants();
        this.params.highArc = true;
        this.mode = mode;

        if (mode == SolverMode.CLIENT) {
            tryConnectClient();
        } else {
            ballisticClient = null;
            System.out.println("[BallisticSolver] Using LOCAL solver on roboRIO");
        }
    }

    /** Create a BallisticSolver defaulting to LOCAL mode. */
    public BallisticSolver() {
        this(SolverMode.LOCAL);
    }

    public SolverMode getMode() {
        return mode;
    }

    public void setMode(SolverMode newMode) {
        if (newMode == mode)
            return;
        mode = newMode;
        System.out.println("[BallisticSolver] Mode set to " + mode);
        if (newMode == SolverMode.CLIENT) {
            tryConnectClient();
        } else {
            closeClient();
        }
    }

    /**
     * Attempt to connect the TCP client. Called from setMode and reconnect logic.
     */
    public void tryConnectClient() {
        if (ballisticClient != null)
            return;
        try {
            ballisticClient = new BallisticClient("10.1.14.11", 5805);
            System.out.println("[BallisticSolver] Connected to remote server");
        } catch (IOException e) {
            ballisticClient = null;
            System.err.println("[BallisticSolver] Cannot connect to server: " + e.getMessage());
        }
    }

    private void closeClient() {
        if (ballisticClient != null) {
            ballisticClient.close();
            ballisticClient = null;
        }
    }

    /** Returns true if mode is CLIENT but the TCP connection is not established. */
    public boolean needsReconnect() {
        return mode == SolverMode.CLIENT && ballisticClient == null;
    }

    public double getRpmForPreset(BallisticConstants.SpeedPreset preset) {
        return switch (preset) {
            case LOW -> lowRpmSub.get();
            case MEDIUM -> mediumRpmSub.get();
            case HIGH -> highRpmSub.get();
            default -> mediumRpmSub.get();
        };
    }

    /**
     * Fit quadratic v = A*rps² + B*rps + C from 3 tunable points (RPM, vel).
     * Returns {A, B, C} or null if singular (fall back to default curve).
     */
    private double[] fitQuadraticFromPoints() {
        double r1 = fitRpm1.get() / 60.0;
        double r2 = fitRpm2.get() / 60.0;
        double r3 = fitRpm3.get() / 60.0;
        double v1 = fitVel1.get();
        double v2 = fitVel2.get();
        double v3 = fitVel3.get();
        // Solve [r² r 1] * [A;B;C] = v for each point. 3x3 linear system.
        double m11 = r1 * r1, m12 = r1, m13 = 1;
        double m21 = r2 * r2, m22 = r2, m23 = 1;
        double m31 = r3 * r3, m32 = r3, m33 = 1;
        double det = m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31);
        if (Math.abs(det) < 1e-12) {
            return null;
        }
        double invDet = 1.0 / det;
        double a = invDet * (v1 * (m22 * m33 - m23 * m32) - m12 * (v2 * m33 - v3 * m23) + m13 * (v2 * m32 - v3 * m22));
        double b = invDet * (m11 * (v2 * m33 - v3 * m23) - v1 * (m21 * m33 - m23 * m31) + m13 * (m21 * v3 - m31 * v2));
        double c = invDet * (m11 * (m22 * v3 - m32 * v2) - m12 * (m21 * v3 - m31 * v2) + v1 * (m21 * m32 - m22 * m31));
        return new double[] { a, b, c };
    }

    /** Evaluate fitted quadratic at rps, or fallback to ShooterConstants curve if fit is singular. */
    private double rpsToLaunchVelocity(double rps) {
        double[] abc = fitQuadraticFromPoints();
        if (abc != null) {
            double v = abc[0] * rps * rps + abc[1] * rps + abc[2];
            return Math.max(v, 0.0);
        }
        return ShooterConstants.flywheelRPSToBallVel(rps);
    }

    public double getLaunchVelocityForPreset(BallisticConstants.SpeedPreset preset) {
        double rps = getRpmForPreset(preset) / 60.0;
        return rpsToLaunchVelocity(rps) * rpmToInitialSpeedScale.get();
    }

    /** Quadratic drag coefficient (tunable via Shooter/QuadraticDrag). */
    public double getDragCoefficient() {
        return quadraticDragSub.get();
    }

    // ================================================================
    // ShooterSolver interface implementation
    // ================================================================
    @Override
    public ShotData calculateShot(Pose3d launcherPosition, ChassisSpeeds fieldSpeeds,
            Translation3d actualTarget) {
        params.spinLiftFactor = 0.0; // spinLiftFactorSub.get() commented out
        // ---- 1. Relative position: target w.r.t. launcher in field frame ----
        Translation3d deltaField = actualTarget.minus(launcherPosition.getTranslation());
        double relX = deltaField.getX();
        double relY = deltaField.getY();
        double relZ = deltaField.getZ();

        // ---- 2. Relative velocity ----
        double relVelX = -fieldSpeeds.vxMetersPerSecond;
        double relVelY = -fieldSpeeds.vyMetersPerSecond;

        // ---- 3. Distance → speed preset → launch velocity & RPM (from tunables) ----
        double horizontalDistance = Math.sqrt(relX * relX + relY * relY);
        BallisticConstants.SpeedPreset preset = BallisticConstants.getSpeedPresetForDistance(horizontalDistance);
        double launchVelocity = getLaunchVelocityForPreset(preset); // scale applied inside getter
        double rpm = getRpmForPreset(preset);

        // ---- 4. Solve based on mode ----
        if (mode == SolverMode.CLIENT) {
            if (ballisticClient != null) {
                return solveViaClient(relX, relY, relZ, relVelX, relVelY, launchVelocity, rpm, actualTarget);
            }
            return ShotData.INVALID;
        }
        return solveLocally(relX, relY, relZ, relVelX, relVelY, launchVelocity, rpm, actualTarget, horizontalDistance);
    }

    private ShotData solveViaClient(double relX, double relY, double relZ,
            double relVelX, double relVelY,
            double launchVelocity, double rpm, Translation3d actualTarget) {
        try {
            ServerSolverResult serverResult = ballisticClient
                    .sendSolve(new BallisticClient.ServerSolverParams(relX, relY, relZ, relVelX, relVelY, 0,
                            launchVelocity, true));

            if (serverResult.success || Double.isFinite(serverResult.azimuth)) {
                double shootingAngleDeg = Math.toDegrees(serverResult.elevation);
                double turretAngleDeg = MathUtil.inputModulus(Math.toDegrees(serverResult.azimuth), -180, 180);

                return new ShotData(
                        Rotation2d.fromDegrees(shootingAngleDeg),
                        Rotation2d.fromDegrees(turretAngleDeg),
                        rpm,
                        actualTarget,
                        serverResult.success);
            }
        } catch (Exception e) {
            System.err.println("[BallisticSolver] Client error, will reconnect: " + e.getMessage());
            closeClient();
        }

        return ShotData.INVALID;
    }

    private ShotData solveLocally(double relX, double relY, double relZ,
            double relVelX, double relVelY,
            double launchVelocity, double rpm, Translation3d actualTarget,
            double horizontalDistance) {
        Vec3 relPos = new Vec3(relX, relY, relZ);
        Vec3 relVel = new Vec3(relVelX, relVelY, 0);

        SolverResult result = solve(relPos, relVel, launchVelocity, quadraticDragSub.get(), params);

        if (result.success || Double.isFinite(result.theta)) {
            double shootingAngleDeg = Math.toDegrees(result.theta);
            double turretAngleDeg = MathUtil.inputModulus(Math.toDegrees(result.phi), -180, 180);

            return new ShotData(
                    Rotation2d.fromDegrees(shootingAngleDeg),
                    Rotation2d.fromDegrees(turretAngleDeg),
                    rpm,
                    actualTarget,
                    result.success);
        }

        double fallbackTurretDeg = Math.toDegrees(Math.atan2(relY, relX));
        double fallbackShootingDeg = Math.toDegrees(Math.atan2(relZ, horizontalDistance))
                + horizontalDistance * 1.5;

        return new ShotData(
                Rotation2d.fromDegrees(fallbackShootingDeg),
                Rotation2d.fromDegrees(MathUtil.inputModulus(fallbackTurretDeg, -180, 180)),
                rpm,
                actualTarget,
                false);
    }

    // ================================================================
    // Utility functions
    // ================================================================
    private static double wrapPi(double a) {
        a = ((a + Math.PI) % (2.0 * Math.PI));
        if (a < 0)
            a += 2.0 * Math.PI;
        return a - Math.PI;
    }

    private static Vec3 anglesToDir(double theta, double phi) {
        double ct = Math.cos(theta);
        return new Vec3(ct * Math.cos(phi), ct * Math.sin(phi), Math.sin(theta));
    }

    private static double[] vecToAngles(Vec3 r) {
        double n = r.norm();
        if (n < 1e-12) {
            return new double[] { 0, 0 };
        }
        Vec3 d = r.times(1.0 / n);
        double theta = Math.asin(MathUtil.clamp(d.z, -1.0, 1.0));
        double phi = Math.atan2(d.y, d.x);
        return new double[] { theta, phi };
    }

    // ================================================================
    // RK4 Integration with quadratic drag
    // ================================================================
    private static void deriv(State s, double kDrag, double g, Vec3 wind, State ds) {
        ds.r = s.v.copy();

        Vec3 a = new Vec3(0, 0, -g);
        Vec3 vRelAir = s.v.minus(wind);
        double speed = vRelAir.norm();

        if (speed > 1e-12 && kDrag != 0.0) {
            a = a.minus(vRelAir.times(kDrag * speed));
        }
        ds.v = a;
    }

    private static void rk4Step(State s, double h, double kDrag, double g, Vec3 wind) {
        State k1 = new State(), k2 = new State(), k3 = new State(), k4 = new State();
        State tmp;

        deriv(s, kDrag, g, wind, k1);

        tmp = s.copy();
        tmp.r = tmp.r.plus(k1.r.times(0.5 * h));
        tmp.v = tmp.v.plus(k1.v.times(0.5 * h));
        deriv(tmp, kDrag, g, wind, k2);

        tmp = s.copy();
        tmp.r = tmp.r.plus(k2.r.times(0.5 * h));
        tmp.v = tmp.v.plus(k2.v.times(0.5 * h));
        deriv(tmp, kDrag, g, wind, k3);

        tmp = s.copy();
        tmp.r = tmp.r.plus(k3.r.times(h));
        tmp.v = tmp.v.plus(k3.v.times(h));
        deriv(tmp, kDrag, g, wind, k4);

        s.r = s.r.plus(k1.r.plus(k2.r.times(2)).plus(k3.r.times(2)).plus(k4.r).times(h / 6.0));
        s.v = s.v.plus(k1.v.plus(k2.v.times(2)).plus(k3.v.times(2)).plus(k4.v).times(h / 6.0));
    }

    // ================================================================
    // Target position at time t
    // ================================================================
    private static Vec3 targetPos(Vec3 relPos0, Vec3 relVel, double t) {
        return relPos0.plus(relVel.times(t));
    }

    private static Vec3 relVec(Vec3 projPos, Vec3 relPos0, Vec3 relVel, double t) {
        return projPos.minus(targetPos(relPos0, relVel, t));
    }

    // ================================================================
    // Find closest approach
    // ================================================================
    private static double[] findClosestApproach(
            Vec3 projVel0, Vec3 relPos0, Vec3 relVel,
            double kDrag, BallisticConstants params) {

        State prev = new State();
        prev.r = new Vec3();
        prev.v = projVel0.copy();

        double tPrev = 0;
        Vec3 rel0 = relVec(prev.r, relPos0, relVel, 0);
        Vec3 relVel0 = prev.v.minus(relVel);
        double qPrev = rel0.dot(relVel0);

        boolean isLow = !params.highArc;

        Vec3 bestRelMiss = rel0;
        double bestT = 0;

        while (tPrev < params.tMax) {
            State curr = prev.copy();
            rk4Step(curr, params.dt, kDrag, params.getEffectiveG(), params.wind);
            double tCurr = tPrev + params.dt;

            Vec3 relCurr = relVec(curr.r, relPos0, relVel, tCurr);
            Vec3 relVelCurr = curr.v.minus(relVel);
            double qCurr = relCurr.dot(relVelCurr);

            boolean allowCheck = isLow || curr.v.z <= 0;

            if (allowCheck && qPrev <= 0 && qCurr >= 0) {
                double tau = params.dt * (-qPrev) / (qCurr - qPrev + 1e-12);
                tau = MathUtil.clamp(tau, 0, params.dt);

                State stStar = prev.copy();
                if (tau > 1e-15) {
                    rk4Step(stStar, tau, kDrag, params.getEffectiveG(), params.wind);
                }
                double tStar = tPrev + tau;
                bestRelMiss = relVec(stStar.r, relPos0, relVel, tStar);
                bestT = tStar;
                break;
            }

            prev = curr;
            tPrev = tCurr;
            qPrev = qCurr;
        }

        if (bestT == 0 && tPrev > 0) {
            bestRelMiss = relVec(prev.r, relPos0, relVel, tPrev);
            bestT = tPrev;
        }

        return new double[] { bestRelMiss.x, bestRelMiss.y, bestRelMiss.z, bestT };
    }

    // ================================================================
    // Vacuum arc solution (initial guess)
    // ================================================================
    private static double[] vacuumArcAnglesToPoint(Vec3 R, double v0, boolean highArc, double g) {
        double Rxy = Math.sqrt(R.x * R.x + R.y * R.y);
        double phi = Math.atan2(R.y, R.x);

        if (Rxy < 1e-12) {
            double theta = (R.z >= 0 ? 1 : -1) * (Math.PI / 4.0);
            return new double[] { theta, phi, 1 };
        }

        double v2 = v0 * v0;
        double disc = v2 * v2 - g * (g * Rxy * Rxy + 2.0 * R.z * v2);

        if (disc < 0) {
            double theta = Math.atan2(R.z, Rxy);
            return new double[] { theta, phi, 0 };
        }

        double s = Math.sqrt(disc);
        double num = highArc ? (v2 + s) : (v2 - s);
        double tanTheta = num / (g * Rxy);
        double theta = Math.atan(tanTheta);

        if (!Double.isFinite(theta)) {
            theta = Math.atan2(R.z, Rxy);
            return new double[] { theta, phi, 0 };
        }

        return new double[] { theta, phi, 1 };
    }

    // ================================================================
    // Initial guess with lead
    // ================================================================
    private static double[] initialGuess(Vec3 relPos0, Vec3 relVel, double v0, boolean highArc, double g) {
        double t0 = relPos0.norm() / Math.max(v0, 1e-6);
        Vec3 R = relPos0.plus(relVel.times(t0));
        return vacuumArcAnglesToPoint(R, v0, highArc, g);
    }

    // ================================================================
    // Compute residual
    // ================================================================
    private static double[] computeResidual(
            double theta, double phi,
            Vec3 relPos0, Vec3 relVel,
            double v0, double kDrag, BallisticConstants P) {

        Vec3 dir = anglesToDir(theta, phi);
        Vec3 projVel0 = dir.times(v0);

        double[] closestResult = findClosestApproach(projVel0, relPos0, relVel, kDrag, P);
        Vec3 relMissAtStar = new Vec3(closestResult[0], closestResult[1], closestResult[2]);
        double tStar = closestResult[3];
        double miss = relMissAtStar.norm();

        Vec3 aim = targetPos(relPos0, relVel, tStar);
        Vec3 aimCorr = aim.minus(relMissAtStar.times(P.beta));

        double[] angles0 = vacuumArcAnglesToPoint(aim, v0, P.highArc, P.getEffectiveG());
        double[] angles1 = vacuumArcAnglesToPoint(aimCorr, v0, P.highArc, P.getEffectiveG());

        double th0 = angles0[0], ph0 = angles0[1];
        double th1 = angles1[0], ph1 = angles1[1];

        if (angles0[2] == 0 || angles1[2] == 0) {
            double[] a0 = vecToAngles(aim);
            double[] a1 = vecToAngles(aimCorr);
            th0 = a0[0];
            ph0 = a0[1];
            th1 = a1[0];
            ph1 = a1[1];
        }

        double F0 = th1 - th0;
        double F1 = wrapPi(ph1 - ph0);

        return new double[] { F0, F1, miss, tStar };
    }

    // ================================================================
    // Main solver
    // ================================================================
    public static SolverResult solve(
            Vec3 relPos0,
            Vec3 relVel,
            double v0,
            double kDrag,
            BallisticConstants params) {

        BallisticConstants P = (params != null) ? params : new BallisticConstants();
        SolverResult result = new SolverResult();

        // Validation
        if (!Double.isFinite(v0) || v0 <= 0 || !Double.isFinite(kDrag)) {
            return result;
        }

        // Initial guess
        double[] guess = initialGuess(relPos0, relVel, v0, P.highArc, P.getEffectiveG());
        double theta = MathUtil.clamp(guess[0], P.thetaMin, P.thetaMax);
        double phi = wrapPi(guess[1]);

        // Compute initial residual
        double[] res = computeResidual(theta, phi, relPos0, relVel, v0, kDrag, P);
        double miss = res[2];
        double tStar = res[3];

        double bestTheta = theta;
        double bestPhi = phi;
        double bestMiss = miss;
        double bestTStar = tStar;

        // Gradient descent iteration
        double stepSize = 0.1;

        for (int iter = 0; iter < P.maxIter; iter++) {
            result.iterations = iter + 1;

            if (miss <= P.tolMiss) {
                result.success = true;
                break;
            }

            // Finite difference gradient
            double h = 1e-4;

            double[] resThP = computeResidual(theta + h, phi, relPos0, relVel, v0, kDrag, P);
            double[] resThM = computeResidual(theta - h, phi, relPos0, relVel, v0, kDrag, P);
            double[] resPhP = computeResidual(theta, wrapPi(phi + h), relPos0, relVel, v0, kDrag, P);
            double[] resPhM = computeResidual(theta, wrapPi(phi - h), relPos0, relVel, v0, kDrag, P);

            double dMissDTheta = (resThP[2] - resThM[2]) / (2 * h);
            double dMissDPhi = (resPhP[2] - resPhM[2]) / (2 * h);

            // Gradient descent step
            double dTheta = -stepSize * dMissDTheta;
            double dPhi = -stepSize * dMissDPhi;

            // Line search
            double alpha = 1.0;
            boolean improved = false;

            for (int ls = 0; ls < 8; ls++) {
                double newTheta = MathUtil.clamp(theta + alpha * dTheta, P.thetaMin, P.thetaMax);
                double newPhi = wrapPi(phi + alpha * dPhi);

                double[] newRes = computeResidual(newTheta, newPhi, relPos0, relVel, v0, kDrag, P);
                double newMiss = newRes[2];

                if (newMiss < miss) {
                    theta = newTheta;
                    phi = newPhi;
                    miss = newMiss;
                    tStar = newRes[3];
                    improved = true;

                    if (miss < bestMiss) {
                        bestTheta = theta;
                        bestPhi = phi;
                        bestMiss = miss;
                        bestTStar = tStar;
                    }
                    break;
                }
                alpha *= 0.5;
            }

            if (!improved) {
                stepSize *= 0.5;
                if (stepSize < 1e-6)
                    break;
            }
        }

        result.theta = bestTheta;
        result.phi = bestPhi;
        result.miss = bestMiss;
        result.tStar = bestTStar;
        result.success = bestMiss <= P.tolMiss;

        return result;
    }

    // ================================================================
    // Convenience methods
    // ================================================================
    /**
     * Solve for launch angles given robot-relative target position and velocity
     */
    public static SolverResult solve(
            double targetX, double targetY, double targetZ,
            double targetVelX, double targetVelY, double targetVelZ,
            double launchSpeed, double dragCoeff) {

        Vec3 relPos = new Vec3(targetX, targetY, targetZ);
        Vec3 relVel = new Vec3(targetVelX, targetVelY, targetVelZ);

        return solve(relPos, relVel, launchSpeed, dragCoeff, null);
    }

}
