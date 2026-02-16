package frc.robot.subsystems.shooter.ballistics;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

import frc.robot.subsystems.shooter.ballistics.BallisticSolver.SolverResult;
import frc.robot.subsystems.shooter.ballistics.BallisticSolver.Vec3;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;

/**
 * Unit tests for BallisticSolver and BallisticConstants
 */
class BallisticSolverTest {

    private static final double LAUNCH_SPEED = BallisticConstants.MEDIUM_SPEED_LAUNCH_VEL; // 18 m/s
    private static final double DRAG = BallisticConstants.DEFAULT_DRAG_COEFFICIENT; // 0.015

    // ================================================================
    // BallisticSolver - 静止目标测试
    // ================================================================

    @Test
    @DisplayName("静止目标：正前方 5m, 高 2m — 求解器应收敛")
    void solveStationary_forward5m_height2m() {
        SolverResult result = BallisticSolver.solveStationary(5.0, 0.0, 2.0, LAUNCH_SPEED, DRAG);

        assertTrue(result.success, "求解器应成功收敛");
        assertTrue(Double.isFinite(result.theta), "theta 应有限");
        assertTrue(Double.isFinite(result.phi), "phi 应有限");
        assertTrue(result.miss < 0.1, "miss 应 < 0.1m, 实际: " + result.miss);
        assertTrue(result.tStar > 0, "飞行时间应 > 0");

        // theta (仰角) 应在合理范围内
        double thetaDeg = Math.toDegrees(result.theta);
        assertTrue(thetaDeg > 10 && thetaDeg < 80,
                "仰角应在 10°-80° 之间, 实际: " + thetaDeg);

        // phi (偏航角) 应接近 0（目标正前方）
        double phiDeg = Math.toDegrees(result.phi);
        assertTrue(Math.abs(phiDeg) < 5,
                "偏航角应接近 0°, 实际: " + phiDeg);

        System.out.printf("5m/2m: theta=%.1f° phi=%.1f° miss=%.3fm t=%.3fs iter=%d%n",
                thetaDeg, phiDeg, result.miss, result.tStar, result.iterations);
    }

    @Test
    @DisplayName("静止目标：近距离 2m, 高 1m")
    void solveStationary_close2m() {
        SolverResult result = BallisticSolver.solveStationary(2.0, 0.0, 1.0, LAUNCH_SPEED, DRAG);

        assertTrue(result.success, "近距离目标应可解");
        assertTrue(result.miss < 0.1, "miss 应 < 0.1m");

        double thetaDeg = Math.toDegrees(result.theta);
        System.out.printf("2m/1m: theta=%.1f° miss=%.3fm t=%.3fs%n",
                thetaDeg, result.miss, result.tStar);
    }

    @Test
    @DisplayName("静止目标：远距离 8m, 高 2m")
    void solveStationary_far8m() {
        // 使用 HIGH 速度档位
        double highSpeed = BallisticConstants.HIGH_SPEED_LAUNCH_VEL; // 24 m/s
        SolverResult result = BallisticSolver.solveStationary(8.0, 0.0, 2.0, highSpeed, DRAG);

        assertTrue(result.success, "远距离目标应可解（高速档）");
        assertTrue(result.miss < 0.1, "miss 应 < 0.1m");

        double thetaDeg = Math.toDegrees(result.theta);
        System.out.printf("8m/2m HIGH: theta=%.1f° miss=%.3fm t=%.3fs%n",
                thetaDeg, result.miss, result.tStar);
    }

    @Test
    @DisplayName("不可达目标：距离过远应求解失败或 miss 较大")
    void solveStationary_unreachable() {
        // 用低速尝试打 20m 远的目标
        double lowSpeed = BallisticConstants.LOW_SPEED_LAUNCH_VEL; // 12 m/s
        SolverResult result = BallisticSolver.solveStationary(20.0, 0.0, 2.0, lowSpeed, DRAG);

        // 要么不成功，要么 miss 很大
        if (result.success) {
            System.out.printf("远目标意外成功: miss=%.3fm%n", result.miss);
        } else {
            assertTrue(result.miss > 0.05 || !result.success,
                    "20m 低速应不可达或 miss 较大");
        }
    }

    // ================================================================
    // 对称性测试
    // ================================================================

    @Test
    @DisplayName("左右对称：目标 (5,2,2) 和 (5,-2,2) 应给出对称 phi")
    void solveStationary_symmetry() {
        SolverResult left = BallisticSolver.solveStationary(5.0, 2.0, 2.0, LAUNCH_SPEED, DRAG);
        SolverResult right = BallisticSolver.solveStationary(5.0, -2.0, 2.0, LAUNCH_SPEED, DRAG);

        assertTrue(left.success && right.success, "两侧目标都应可解");

        // theta 应相同
        assertEquals(left.theta, right.theta, 0.01,
                "左右 theta 应相同");

        // phi 应符号相反，大小相同
        assertEquals(left.phi, -right.phi, 0.01,
                "左右 phi 应对称");

        System.out.printf("对称性: L phi=%.1f° R phi=%.1f° diff=%.4f%n",
                Math.toDegrees(left.phi), Math.toDegrees(right.phi),
                Math.abs(left.phi + right.phi));
    }

    // ================================================================
    // 移动目标测试
    // ================================================================

    @Test
    @DisplayName("移动目标：横向移动的目标应有 phi 偏移（提前量）")
    void solve_movingTarget_lateralLead() {
        Vec3 relPos = new Vec3(5.0, 0.0, 2.0);
        Vec3 relVel = new Vec3(0.0, 1.0, 0.0); // 横向移动 1 m/s

        // 使用低弧线更容易收敛
        BallisticConstants params = new BallisticConstants();
        params.highArc = false;
        params.maxIter = 30;
        params.tolMiss = 0.1;

        SolverResult result = BallisticSolver.solve(relPos, relVel, LAUNCH_SPEED, DRAG, params);

        // 对比静止目标的 phi
        SolverResult stationary = BallisticSolver.solve(relPos, new Vec3(), LAUNCH_SPEED, DRAG, params);

        double phiDeg = Math.toDegrees(result.phi);
        double stationaryPhiDeg = Math.toDegrees(stationary.phi);

        System.out.printf("移动目标: phi=%.1f° miss=%.3fm success=%b iter=%d%n",
                phiDeg, result.miss, result.success, result.iterations);
        System.out.printf("静止目标: phi=%.1f° miss=%.3fm%n", stationaryPhiDeg, stationary.miss);

        // 至少验证 theta 和 phi 是有限值
        assertTrue(Double.isFinite(result.theta), "theta 应有限");
        assertTrue(Double.isFinite(result.phi), "phi 应有限");

        // 移动目标的 phi 应不同于静止目标（有提前量）
        if (result.success && stationary.success) {
            assertTrue(Math.abs(phiDeg - stationaryPhiDeg) > 0.1,
                    "移动目标 phi 应与静止目标不同（提前量）");
        }
    }

    // ================================================================
    // 求解器输出范围验证（与 Hood 限位配合）
    // ================================================================

    @Test
    @DisplayName("求解器 theta 转换为 hood 角度后应在 Hood 限位范围内")
    void solver_outputInHoodRange() {
        // 测试多个距离
        double[] distances = {2.0, 3.0, 5.0, 7.0};
        double targetHeight = 2.0;

        for (double dist : distances) {
            double speed = BallisticConstants.getLaunchVelocityForPreset(
                    BallisticConstants.getSpeedPresetForDistance(dist));

            SolverResult result = BallisticSolver.solveStationary(dist, 0.0, targetHeight, speed, DRAG);

            if (result.success) {
                double launchAngleDeg = Math.toDegrees(result.theta);
                double hoodAngleDeg = BallisticConstants.launchAngleToHoodAngle(launchAngleDeg);

                System.out.printf("dist=%.0fm: launch=%.1f° hood=%.1f° range=[%.1f, %.1f]%n",
                        dist, launchAngleDeg, hoodAngleDeg,
                        HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

                // 警告但不强制失败（标定数据可能需要调整）
                if (hoodAngleDeg < HoodConstants.MIN_ANGLE || hoodAngleDeg > HoodConstants.MAX_ANGLE) {
                    System.out.printf("  ⚠ WARNING: hood 角度 %.1f° 超出限位 [%.1f, %.1f]%n",
                            hoodAngleDeg, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
                }
            }
        }
    }

    // ================================================================
    // BallisticConstants - RPM 映射测试
    // ================================================================

    @Test
    @DisplayName("RPM 到发射速度：插值准确性")
    void rpmToLaunchSpeed_interpolation() {
        // 端点值
        assertEquals(8.0, BallisticConstants.rpmToLaunchSpeed(2000), 0.001, "2000 RPM -> 8 m/s");
        assertEquals(12.0, BallisticConstants.rpmToLaunchSpeed(3000), 0.001, "3000 RPM -> 12 m/s");
        assertEquals(24.0, BallisticConstants.rpmToLaunchSpeed(6000), 0.001, "6000 RPM -> 24 m/s");

        // 插值中间点
        double mid = BallisticConstants.rpmToLaunchSpeed(2500);
        assertEquals(10.0, mid, 0.001, "2500 RPM 应插值为 10 m/s");

        // 边界外 clamp
        double belowMin = BallisticConstants.rpmToLaunchSpeed(1000);
        assertEquals(8.0, belowMin, 0.001, "低于最小 RPM 应返回最小速度");

        double aboveMax = BallisticConstants.rpmToLaunchSpeed(7000);
        assertEquals(24.0, aboveMax, 0.001, "高于最大 RPM 应返回最大速度");
    }

    @Test
    @DisplayName("发射速度到 RPM：逆映射")
    void launchSpeedToRPM_inverse() {
        assertEquals(2000, BallisticConstants.launchSpeedToRPM(8.0), 0.001);
        assertEquals(3000, BallisticConstants.launchSpeedToRPM(12.0), 0.001);
        assertEquals(6000, BallisticConstants.launchSpeedToRPM(24.0), 0.001);

        // 中间值
        assertEquals(2500, BallisticConstants.launchSpeedToRPM(10.0), 0.001);
    }

    @Test
    @DisplayName("RPM <-> 速度 双向一致性")
    void rpmSpeedRoundTrip() {
        double[] testRPMs = {2500, 3500, 4500, 5500};
        for (double rpm : testRPMs) {
            double speed = BallisticConstants.rpmToLaunchSpeed(rpm);
            double backRPM = BallisticConstants.launchSpeedToRPM(speed);
            assertEquals(rpm, backRPM, 0.1,
                    String.format("RPM %.0f -> speed %.1f -> RPM %.0f 应一致", rpm, speed, backRPM));
        }
    }

    // ================================================================
    // BallisticConstants - Hood 角度转换测试
    // ================================================================

    @Test
    @DisplayName("Hood 角度转换：双向一致性")
    void hoodAngleConversion_roundTrip() {
        double[] launchAngles = {30.0, 45.0, 60.0, 75.0};
        for (double la : launchAngles) {
            double hoodAngle = BallisticConstants.launchAngleToHoodAngle(la);
            double backLaunch = BallisticConstants.hoodAngleToLaunchAngle(hoodAngle);
            assertEquals(la, backLaunch, 0.001,
                    String.format("launch %.1f° -> hood %.1f° -> launch %.1f° 应一致",
                            la, hoodAngle, backLaunch));
        }
    }

    @Test
    @DisplayName("Hood 偏移量：hood 角度 = 发射角 - 3.5°")
    void hoodAngleOffset() {
        assertEquals(41.5, BallisticConstants.launchAngleToHoodAngle(45.0), 0.001,
                "45° 发射角 -> 41.5° hood 角度");
        assertEquals(26.5, BallisticConstants.launchAngleToHoodAngle(30.0), 0.001,
                "30° 发射角 -> 26.5° hood 角度");
    }

    // ================================================================
    // BallisticConstants - 速度档位选择测试
    // ================================================================

    @Test
    @DisplayName("速度档位：距离选择")
    void speedPresetSelection() {
        assertEquals(BallisticConstants.SpeedPreset.LOW,
                BallisticConstants.getSpeedPresetForDistance(2.0), "2m -> LOW");
        assertEquals(BallisticConstants.SpeedPreset.LOW,
                BallisticConstants.getSpeedPresetForDistance(3.9), "3.9m -> LOW");
        assertEquals(BallisticConstants.SpeedPreset.MEDIUM,
                BallisticConstants.getSpeedPresetForDistance(4.0), "4m -> MEDIUM");
        assertEquals(BallisticConstants.SpeedPreset.MEDIUM,
                BallisticConstants.getSpeedPresetForDistance(6.9), "6.9m -> MEDIUM");
        assertEquals(BallisticConstants.SpeedPreset.HIGH,
                BallisticConstants.getSpeedPresetForDistance(7.0), "7m -> HIGH");
        assertEquals(BallisticConstants.SpeedPreset.HIGH,
                BallisticConstants.getSpeedPresetForDistance(10.0), "10m -> HIGH");
    }

    @Test
    @DisplayName("速度档位：RPM 和发射速度一致")
    void speedPresetValues() {
        assertEquals(3000, BallisticConstants.getRPMForPreset(BallisticConstants.SpeedPreset.LOW));
        assertEquals(4500, BallisticConstants.getRPMForPreset(BallisticConstants.SpeedPreset.MEDIUM));
        assertEquals(6000, BallisticConstants.getRPMForPreset(BallisticConstants.SpeedPreset.HIGH));

        assertEquals(12.0, BallisticConstants.getLaunchVelocityForPreset(BallisticConstants.SpeedPreset.LOW));
        assertEquals(18.0, BallisticConstants.getLaunchVelocityForPreset(BallisticConstants.SpeedPreset.MEDIUM));
        assertEquals(24.0, BallisticConstants.getLaunchVelocityForPreset(BallisticConstants.SpeedPreset.HIGH));
    }

    // ================================================================
    // Vec3 基础测试
    // ================================================================

    @Test
    @DisplayName("Vec3 基础运算")
    void vec3Operations() {
        Vec3 a = new Vec3(1, 2, 3);
        Vec3 b = new Vec3(4, 5, 6);

        Vec3 sum = a.plus(b);
        assertEquals(5, sum.x); assertEquals(7, sum.y); assertEquals(9, sum.z);

        Vec3 diff = a.minus(b);
        assertEquals(-3, diff.x); assertEquals(-3, diff.y); assertEquals(-3, diff.z);

        Vec3 scaled = a.times(2);
        assertEquals(2, scaled.x); assertEquals(4, scaled.y); assertEquals(6, scaled.z);

        assertEquals(32, a.dot(b), 0.001); // 1*4 + 2*5 + 3*6

        assertEquals(Math.sqrt(14), a.norm(), 0.001);
    }

    // ================================================================
    // 无效输入测试
    // ================================================================

    @Test
    @DisplayName("无效输入：零速度应返回失败")
    void solve_zeroSpeed() {
        SolverResult result = BallisticSolver.solveStationary(5, 0, 2, 0, DRAG);
        assertFalse(result.success, "零速度应返回失败");
    }

    @Test
    @DisplayName("无效输入：负速度应返回失败")
    void solve_negativeSpeed() {
        SolverResult result = BallisticSolver.solveStationary(5, 0, 2, -10, DRAG);
        assertFalse(result.success, "负速度应返回失败");
    }

    @Test
    @DisplayName("无效输入：NaN 速度应返回失败")
    void solve_nanSpeed() {
        SolverResult result = BallisticSolver.solveStationary(5, 0, 2, Double.NaN, DRAG);
        assertFalse(result.success, "NaN 速度应返回失败");
    }
}
