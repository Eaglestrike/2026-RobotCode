package frc.robot.subsystems.shooter.desmos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.solver.ShooterSolver;

/**
 * Desmos-derived ballistic solver — gravity-only (no drag) closed-form
 * solution.
 *
 * <h2>What it computes</h2>
 * Given the launcher's 3D field pose, the robot's field-relative velocity, and
 * a 3D
 * target position, this solver computes:
 * <ul>
 * <li><b>Hood angle (pitch)</b> — the vertical launch angle, biased steep
 * (⅓ apex angle + ⅔ × 90°) to ensure a high-arc trajectory with steep
 * descent.</li>
 * <li><b>Turret angle (yaw)</b> — the horizontal launch direction in field
 * frame,
 * accounting for robot motion during flight (lead angle).</li>
 * <li><b>Launch velocity v₀</b> — the required ball exit speed (m/s), converted
 * to
 * flywheel RPM via {@link ShooterConstants#ballVelToFlywheelRPS}.</li>
 * <li><b>Flight time t</b> — the time for the ball to reach the target.</li>
 * </ul>
 *
 * <h2>Algorithm overview</h2>
 * <ol>
 * <li>Compute the "apex solution" — the launch angles that place the target
 * exactly
 * at the parabola's apex (v_z = 0). This gives a reference pitch angle.</li>
 * <li>Choose the actual hood angle as a weighted blend toward 90° (steep
 * shot).</li>
 * <li>With the hood angle fixed, use bisection to find flight time t₁ such that
 * the
 * ball's parabolic height matches the target height at horizontal range
 * ℓ(t₁).</li>
 * <li>Compute the turret yaw to the lead-corrected target position at t₁.</li>
 * <li>Recover v₀ from the horizontal displacement equation.</li>
 * </ol>
 *
 * <h2>Known limitations / notes</h2>
 * <ul>
 * <li>No air drag — suitable as a fast initial estimate or for short-range
 * shots.</li>
 * <li>{@code weird_atan2} has a known discrepancy when x &lt; 0 and y &lt; 0
 * (adds −π/2 instead of −π). Left as-is pending field testing.</li>
 * </ul>
 *
 * <h2>Integration plan (Shooter / ShooterConstants)</h2>
 * <ul>
 * <li>{@link ShooterConstants#ballVelToFlywheelRPS} converts the computed v₀
 * (m/s) →
 * flywheel RPS → RPM for the {@link ShotData} record.</li>
 * <li>{@code Shooter.periodic()} can call {@code calculateShot()} every cycle
 * (or on a
 * background thread like the existing {@code BallisticSolver}) and feed the
 * resulting hood angle, turret angle, and RPM to the subsystem commands.</li>
 * <li>Hood angle is clamped to
 * [{@link ShooterConstants.HoodConstants#MIN_ANGLE},
 * {@link ShooterConstants.HoodConstants#MAX_ANGLE}] inside Shooter.</li>
 * <li>Future: pass this solver as the {@code ShooterSolver} implementation in
 * the
 * {@code Shooter} constructor to swap between Desmos and BallisticSolver at
 * runtime.</li>
 * </ul>
 */
public class DesmosSolver implements ShooterSolver {
    /** Blending ratio for hood angle between apex and 90° (steep) */
    private static double ratio = 1 / 3d;

    // ================================================================
    // Internal parameter bundle
    // ================================================================

    /**
     * Bundle of trajectory parameters passed between internal methods.
     * <ul>
     * <li>g — gravitational acceleration (m/s²)</li>
     * <li>hx, hy, hz — target position relative to launcher (meters)</li>
     * <li>vx, vy — robot field-relative velocity (m/s), subtracted from target
     * motion</li>
     * <li>a — hood (pitch) angle in degrees</li>
     * <li>b — turret (yaw) angle in degrees</li>
     * </ul>
     */
    private static class Params {
        double g;
        double hx, hy, hz;
        double vx, vy;
        double a, b;
    }

    // ================================================================
    // Residual function for bisection
    // ================================================================

    /**
     * Evaluates the residual f(t) = tan(a) · ℓ(t) − (hz + ½ g t²).
     *
     * At the correct flight time t, f(t) = 0, meaning the parabolic height at
     * horizontal range ℓ(t) equals the target height (including gravity drop).
     *
     * ℓ(t) = √((hx − vx·t)² + (hy − vy·t)²) is the horizontal distance to the
     * lead-corrected target at time t.
     */
    private static double f(double t, Params p) {
        double dx = p.hx - p.vx * t;
        double dy = p.hy - p.vy * t;
        double l = Math.sqrt(dx * dx + dy * dy);

        return Math.tan(p.a * Math.PI / 180) * l - (p.hz + 0.5 * p.g * t * t);
    }

    // ================================================================
    // Bisection root finder
    // ================================================================

    /**
     * Finds the root of f(t) = 0 in [tLo, tHi] using bisection.
     *
     * @return flight time, or −1 if the root is not bracketed or convergence fails.
     */
    private static double findRoot(double tLo, double tHi, Params p, double tol, int maxIter) {
        double fLo = f(tLo, p);
        double fHi = f(tHi, p);

        // NaN check — e.g. if hood angle is NaN from degenerate apex
        if (!Double.isFinite(fLo) || !Double.isFinite(fHi)) {
            System.out.printf("[DesmosSolver] NaN/Inf in residual: f(%.2f)=%f, f(%.2f)=%f, a=%.2f%n",
                    tLo, fLo, tHi, fHi, p.a);
            return -1;
        }

        if (fLo * fHi > 0.0) {
            System.out.printf(
                    "[DesmosSolver] Root not bracketed: f(%.2f)=%.4f, f(%.2f)=%.4f, a=%.2f°, hz=%.3f, hx=%.3f, hy=%.3f%n",
                    tLo, fLo, tHi, fHi, p.a, p.hz, p.hx, p.hy);
            return -1;
        }

        for (int i = 0; i < maxIter; i++) {
            double tMid = 0.5 * (tLo + tHi);
            double fMid = f(tMid, p);

            if (Math.abs(fMid) < tol || (tHi - tLo) < tol) {
                return tMid;
            }

            if (fLo * fMid < 0.0) {
                tHi = tMid;
                fHi = fMid;
            } else {
                tLo = tMid;
                fLo = fMid;
            }
        }

        System.out.println("[DesmosSolver] Root finder did not converge");
        return -1;
    }

    // ================================================================
    // Apex solution
    // ================================================================

    /** Internal container for the apex-trajectory solution. */
    private static class Apex {
        double v0, t;
        double a, b;
    }

    /**
     * Computes the launch angles and velocity that place the target exactly at the
     * parabola's apex (vertical velocity = 0 at target).
     *
     * <p>
     * At the apex: t_apex = √(2 hz / g), so the ball rises exactly hz meters.
     * The pitch angle depends only on the vertical rise (hz) and horizontal range
     * to the lead-corrected target at t_apex. Yaw points toward that lead-corrected
     * position.
     * </p>
     *
     * <p>
     * <b>Key insight:</b> pitch must always be in [0°, 90°] (launch upward).
     * We compute it from the always-positive horizontal distance and vertical
     * velocity, avoiding sign issues when the target is in a "negative" quadrant.
     * </p>
     */
    private static Apex findApexAngle(Params p) {
        double k = Math.sqrt(p.g / (2 * p.hz)); // 1 / t_apex
        double tApex = 1.0 / k; // = sqrt(2 hz / g)

        // Lead-corrected target position at t_apex (horizontal components)
        double dxApex = p.hx - p.vx * tApex;
        double dyApex = p.hy - p.vy * tApex;

        // Yaw: direction to lead-corrected target (full atan2, handles all quadrants)
        double b_apex = Math.atan2(dyApex, dxApex);

        // Horizontal range (always positive) and vertical velocity at launch
        double lApex = Math.sqrt(dxApex * dxApex + dyApex * dyApex);
        double vzApex = Math.sqrt(2 * p.hz * p.g); // vertical speed needed to reach hz

        // Pitch: angle between horizontal and launch velocity vector
        // atan2(vz, v_horiz) — both positive, so result is in [0, π/2]
        double vHoriz = lApex / tApex; // horizontal speed needed
        double a_apex = Math.atan2(vzApex, vHoriz);

        Apex a = new Apex();
        a.v0 = Math.sqrt(vHoriz * vHoriz + vzApex * vzApex);
        a.t = tApex;
        a.a = a_apex * 180.0 / Math.PI;
        a.b = b_apex * 180.0 / Math.PI;
        return a;
    }

    // ================================================================
    // Results container
    // ================================================================

    /** Raw output of the internal solver (before conversion to ShotData). */
    public static class Results {
        /** Flight time to target (seconds) */
        double t;
        /** Required ball exit velocity (m/s) */
        double v0;
        /** Turret yaw angle in field frame (degrees) */
        double turretAngle;
        /** Hood pitch angle (degrees, 0 = horizontal, 90 = straight up) */
        double hoodAngle;
    }

    // ================================================================
    // Custom atan2 variant
    // ================================================================

    // ================================================================
    // Core solver
    // ================================================================

    /**
     * Computes turret angle, hood angle, initial velocity, and flight time for a
     * gravity-only (no drag) ballistic trajectory to a target.
     *
     * @param vx robot field-relative velocity X (m/s)
     * @param vy robot field-relative velocity Y (m/s)
     * @param tx horizontal offset to target X (m)
     * @param ty horizontal offset to target Y (m)
     * @param hz height difference: target Z minus launcher Z (m)
     * @return solver results (angles, velocity, flight time)
     */
    public static Results findLaunchParams(double vx, double vy, double tx, double ty, double hz) {
        // Guard: apex-based approach requires the target to be above the launcher (hz >
        // 0).
        // If the target is at or below launcher height, we cannot place it at the
        // parabola's
        // apex (sqrt of negative). Return an invalid result so calculateShot returns
        // INVALID.
        if (hz <= 0.0) {
            Results r = new Results();
            r.t = -1;
            r.v0 = Double.NaN;
            r.hoodAngle = 0;
            r.turretAngle = 0;
            return r;
        }

        Params p = new Params();
        p.g = 9.81;
        p.hx = tx;
        p.hy = ty;
        p.hz = hz;

        p.vx = vx;
        p.vy = vy;

        // Step 1: Find the "apex solution" — launch angles that place target at
        // parabola apex
        Apex apex = findApexAngle(p);

        // Step 2: Blend hood angle toward 90° for a steep descent trajectory
        p.a = ratio * apex.a + (1 - ratio) * 90.0;

        // Sanity-check: if apex angle was NaN (degenerate geometry), bail out
        if (!Double.isFinite(p.a)) {
            Results r = new Results();
            r.t = -1;
            r.v0 = Double.NaN;
            r.hoodAngle = 0;
            r.turretAngle = 0;
            return r;
        }

        // Step 3: Bisect for flight time with the chosen hood angle
        double t1 = findRoot(0.0, 10.0, p, 1e-8, 50);

        // Step 4: Compute turret yaw to the lead-corrected target position at t1
        // Using standard atan2 for correct quadrant handling in all cases.
        p.b = Math.atan2(p.hy - p.vy * t1, p.hx - p.vx * t1) * 180.0 / Math.PI;

        // Step 5: Recover launch velocity from horizontal displacement
        double v0 = ((p.hx - p.vx * t1) * Math.cos(p.b * Math.PI / 180)
                + (p.hy - p.vy * t1) * Math.sin(p.b * Math.PI / 180)) / (t1 * Math.cos(p.a * Math.PI / 180));

        Results r = new Results();
        r.t = t1;
        r.v0 = v0;
        r.hoodAngle = p.a;
        r.turretAngle = p.b;
        return r;
    }

    /** Overload with hardcoded height for backwards compatibility. */
    public static Results findLaunchParams(double vx, double vy, double tx, double ty) {
        return findLaunchParams(vx, vy, tx, ty, 1.822);
    }

    // ================================================================
    // ShooterSolver interface implementation
    // ================================================================

    private static int CACHE_ITERS = 5;
    private ShotData cachedResult;
    private int cacheIter = CACHE_ITERS;

    /**
     * Computes a shot solution conforming to the {@link ShooterSolver} interface.
     *
     * <p>
     * Coordinate convention: the target and launcher are in WPILib's field frame
     * (X forward, Y left, Z up). The solver works in a relative frame where
     * (tx, ty) is the horizontal displacement from launcher to target, and hz is
     * the vertical displacement (target Z - launcher Z). Robot velocity is passed
     * through so the solver accounts for the robot's motion during flight.
     * </p>
     *
     * <p>
     * v0 is converted to flywheel RPM via
     * {@link ShooterConstants#ballVelToFlywheelRPS(double)} * 60.
     * </p>
     *
     * @param launcherPose 3D field-relative pose of the ball exit point
     * @param fieldSpeeds  field-relative chassis speeds (vx, vy in m/s)
     * @param actualTarget 3D field-relative target position
     * @return {@link ShotData} with hood angle, turret angle (field-frame), RPM,
     *         and validity flag
     */
    @Override
    public ShotData calculateShot(Pose3d launcherPose, ChassisSpeeds fieldSpeeds, Translation3d actualTarget) {
        if (cacheIter++ == CACHE_ITERS) {
            cacheIter = 1;
            return cachedResult;
        }

        // Relative position: target w.r.t. launcher
        double tx = actualTarget.getX() - launcherPose.getX();
        double ty = actualTarget.getY() - launcherPose.getY();
        double hz = actualTarget.getZ() - launcherPose.getZ();

        // Robot velocity (field-relative) — solver expects the velocity to subtract
        // from target motion, which is the robot's own velocity
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        Results r = findLaunchParams(vx, vy, tx, ty, hz);

        // Check for solver failure (bisection returned -1)
        if (r.t < 0 || !Double.isFinite(r.v0) || r.v0 <= 0) {
            return cachedResult = ShotData.INVALID;
        }

        DogLog.log("Shooter/Launch velocity", r.v0);

        // Convert ball exit velocity (m/s) -> flywheel RPS -> RPM
        double flywheelRPS = ShooterConstants.ballVelToFlywheelRPS(r.v0);
        double rpm = flywheelRPS * 60.0;

        return cachedResult = new ShotData(
                Rotation2d.fromDegrees(r.hoodAngle),
                Rotation2d.fromDegrees(r.turretAngle),
                rpm,
                actualTarget,
                true);
    }

    /**
     * Sets the blending ratio for hood angle between the apex solution and 90° (steep shot).
     * By default, ratio = ⅓ (one-third of the way from apex to 90°).  
     * @param ratio The blending ratio (0.0 = full 90°, 1.0 = full apex)
     */
    public static void setHoodRatio(double ratio) {
        DesmosSolver.ratio = ratio;
    }
}
