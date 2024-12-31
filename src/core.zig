//! This module contains core structs and maths function used throughout the library.

pub const Vec2f = @Vector(2, f32);
pub const Vec3f = @Vector(3, f32);
pub const Vec4f = @Vector(4, f32);
pub const Vec6f = @Vector(6, f32);
pub const Vec8f = @Vector(8, f32);
pub const Vec12f = @Vector(12, f32);
pub const Vec16f = @Vector(16, f32);
const std = @import("std");
const rand = std.rand;
const zero_2f = Vec2f{ 0, 0 };
const zero_3f = Vec2f{ 0, 0, 0 };
const zero_4f = Vec2f{ 0, 0, 0, 0 };
var rng = rand.Xoshiro256.init(0);

/// A circular region in 2D Euclidean space.
//  I.e., the region B := { b in R^2 | d(centre, b) < radius }
pub const Ball2f = struct {
    centre: Vec2f,
    radius: f32 = 0,
    const Self = @This();

    pub fn overlapsBall(self: Self, b: Ball2f) bool {
        if (self.radius == 0 or b.radius == 0) return false;
        return squaredSum(self.centre - b.centre) < (self.radius + b.radius) * (self.radius + b.radius);
    }

    pub fn overlapsBox(self: Self, b: Box2f) bool {
        // TODO: implement this?
        _ = self;
        _ = b;
        return false;
    }

    pub fn overlapsBallsX8(self: Self, balls: [8]Ball2f) [8]bool {
        const dx_vec = Vec8f{
            balls[0].centre[0],
            balls[1].centre[0],
            balls[2].centre[0],
            balls[3].centre[0],
            balls[4].centre[0],
            balls[5].centre[0],
            balls[6].centre[0],
            balls[7].centre[0],
        } - @as(Vec8f, @splat(self.centre[0]));

        const dy_vec = Vec8f{
            balls[0].centre[1],
            balls[1].centre[1],
            balls[2].centre[1],
            balls[3].centre[1],
            balls[4].centre[1],
            balls[5].centre[1],
            balls[6].centre[1],
            balls[7].centre[1],
        } - @as(Vec8f, @splat(self.centre[1]));

        const r_sums_vec = Vec8f{
            self.radius + balls[0].radius,
            self.radius + balls[1].radius,
            self.radius + balls[2].radius,
            self.radius + balls[3].radius,
            self.radius + balls[4].radius,
            self.radius + balls[5].radius,
            self.radius + balls[6].radius,
            self.radius + balls[7].radius,
        };

        // NOTE: Actually performs worse than the naive method when compiled with --release=fast
        //       Using @sqrt was just as fast as using the inequality shown below
        // return @sqrt(dx_vec * dx_vec + dy_vec * dy_vec) < r_sums_vec;
        return dx_vec * dx_vec + dy_vec * dy_vec < r_sums_vec * r_sums_vec;
    }
};

/// An axis-aligned rectangular region in 2D Euclidean space.
//  I.e., the region B: = { b in R^2 | (|centre - b[0]| < 0.5 * w) and (|centre - b[1]| < 0.5 * h)}
pub const Box2f = struct {
    centre: Vec2f,
    width: f32,
    height: f32,
};

pub fn reseedRng(s: usize) void {
    rng.seed(s);
}

pub fn getRandUniform(min: f32, max: f32) f32 {
    return min + (max - min) * rng.random().float(f32);
}

pub fn norm(v: Vec2f) f32 {
    return @sqrt(squaredSum(v));
}

/// Returns the norm (length) of the vector v.
pub fn squaredSum(v: anytype) f32 {
    return innerProd(v, v);
}

pub fn innerProd(a: anytype, b: anytype) f32 {
    return @reduce(.Add, a * b);
}

/// Returns true if all elements of vectors a and b are equal.
pub fn vecsEqual(a: anytype, b: anytype) bool {
    return @reduce(.And, a == b);
}

pub inline fn asf32(x: anytype) f32 {
    return @floatFromInt(x);
}

pub inline fn asu32(x: anytype) u32 {
    return @intFromFloat(x);
}

// pub fn getDistTo4(a: Vec2f, b1: Vec2f, b2: Vec2f, b3: Vec2f, b4: Vec2f) Vec4f {
//     return Vec4f{ norm(a - b1), norm(a - b2), norm(a - b3), norm(a - b4) };
// }

// // gets the distance between a and points b1 - b4
// pub fn getDistTo4Vec(a: Vec2f, bx_vec: Vec4f, by_vec: Vec4f) Vec4f {
//     const ax_vec: Vec4f = @splat(a[0]);
//     const ay_vec: Vec4f = @splat(a[1]);
//     const dx_vec = bx_vec - ax_vec;
//     const dy_vec = by_vec - ay_vec;
//     return @sqrt(dx_vec * dx_vec + dy_vec * dy_vec);
// }

/// Returns a unit length vector in the same direction as v, or a zero vector if it has zero length.
pub fn normalised2f(v: Vec2f) Vec2f {
    const n = norm(v);
    return if (n > 0) scaledVec2f(1 / norm(v), v) else zero_2f;
}

pub fn scaledVec2f(alpha: f32, v: anytype) Vec2f {
    return Vec2f{ alpha * v[0], alpha * v[1] };
}

// solves for t that minimises || u + t * v ||
pub fn solveForMinDist(u: Vec2f, v: Vec2f) f32 {
    const denom = innerProd(v, v);
    return if (denom == 0) 0 else -innerProd(u, v) / denom;
}

// returns the minimum value of || u_a + t * v_a - (b_a - t * v_a) || ^ 2 with no restriction on t
pub fn getMinDistSquared(pos_a: Vec2f, vel_a: Vec2f, pos_b: Vec2f, vel_b: Vec2f) f32 {
    const u = pos_a - pos_b;
    const v = vel_a - vel_b;
    const t = solveForMinDist(u, v);
    const d = u + scaledVec2f(t, v);
    return squaredSum(d);
}

// same as the above but clamps range of t to [t_min, t_max]
pub fn getMinDistSquaredForInterval(pos_a: Vec2f, vel_a: Vec2f, pos_b: Vec2f, vel_b: Vec2f, t_min: f32, t_max: f32) f32 {
    const u = pos_a - pos_b;
    const v = vel_a - vel_b;
    var t = solveForMinDist(u, v);
    t = @max(t_min, @min(t, t_max));
    const d = u + scaledVec2f(t, v);
    return squaredSum(d);
}

pub fn intervalsOverlap(a1: f32, a2: f32, b1: f32, b2: f32) bool {
    return a1 < b1 and b1 < a2 or a1 < b2 and b2 < a2 or b1 < a1 and a2 < b2;
}

pub fn boxesOverlap(a1: Vec2f, a2: Vec2f, b1: Vec2f, b2: Vec2f) bool {
    const x_overlap = intervalsOverlap(a1[0], a2[0], b1[0], b2[0]);
    const y_overlap = intervalsOverlap(a1[1], a2[1], b1[1], b2[1]);
    return x_overlap and y_overlap;
}

/// Returns a ball that encompasses both input balls.
pub fn getEncompassingBall(a: Ball2f, b: Ball2f) Ball2f {
    const d = a.centre - b.centre;
    const dist = norm(d);
    if (b.radius == 0 or (dist + b.radius < a.radius)) return a; // a encompasses b
    if (a.radius == 0 or (dist + a.radius < b.radius)) return b; // b encompasses a
    // otherwise, create a new circle that encompasses both a and b
    return .{
        .radius = 0.5 * (dist + a.radius + b.radius),
        .centre = scaledVec2f(0.5, (a.centre + b.centre)) + scaledVec2f(0.5 * (a.radius - b.radius) / dist, d),
    };
}

const testing = std.testing;
const tolerance = 0.0001;

test "vec equals" {
    const u = Vec2f{ 1, 0 };
    const v = Vec2f{ 0, 1 };
    const w = Vec2f{ 1, 1 };

    try testing.expectEqual(false, vecsEqual(u, v));
    try testing.expectEqual(false, vecsEqual(u, w));
    try testing.expectEqual(false, vecsEqual(v, w));
}

test "vec norm" {
    const u = Vec2f{ 1, 0 };
    const v = Vec2f{ 0, 1 };
    const w = Vec2f{ 1, 1 };

    try testing.expectApproxEqAbs(1.0, norm(u), tolerance);
    try testing.expectApproxEqAbs(1.0, norm(v), tolerance);
    try testing.expectApproxEqAbs(std.math.sqrt2, norm(w), tolerance);
}

test "closest dist" {
    const pos_a = Vec2f{ 1, 0 };
    const pos_b = Vec2f{ 0, 1 };

    const zero_result = getMinDistSquared(zero_2f, zero_2f, zero_2f, zero_2f);
    try testing.expectApproxEqAbs(0, zero_result, tolerance);

    const static_result = getMinDistSquared(pos_a, zero_2f, pos_b, zero_2f);
    try testing.expectApproxEqAbs(2, static_result, tolerance);

    const parallel_result = getMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 2, 0 });
    try testing.expectApproxEqAbs(1, parallel_result, tolerance);

    const opposite_result = getMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ -1, 0 });
    try testing.expectApproxEqAbs(1, opposite_result, tolerance);

    const perpendicular_result = getMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 0, 1 });
    try testing.expectApproxEqAbs(0, perpendicular_result, tolerance);
}

test "closest dist interval" {
    const pos_a = Vec2f{ 1, 0 };
    const pos_b = Vec2f{ 0, 1 };

    const parallel_result = getMinDistSquaredForInterval(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 2, 0 }, 0, 100);
    try testing.expectApproxEqAbs(1, parallel_result, tolerance);

    const opposite_result = getMinDistSquaredForInterval(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ -1, 0 }, -1.5, -0.5);
    try testing.expectApproxEqAbs(1, opposite_result, tolerance);

    const perpendicular_result = getMinDistSquaredForInterval(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 0, 1 }, -3, 3);
    try testing.expectApproxEqAbs(0, perpendicular_result, tolerance);
}

test "intervals overlap" {
    const check_1 = intervalsOverlap(1, 4, 0, 5);
    try testing.expectEqual(true, check_1);
    const check_2 = intervalsOverlap(1, 4, 0, 2);
    try testing.expectEqual(true, check_2);
    const check_3 = intervalsOverlap(1, 4, 2, 3);
    try testing.expectEqual(true, check_3);
    const check_4 = intervalsOverlap(1, 4, 3, 5);
    try testing.expectEqual(true, check_4);
    const check_5 = intervalsOverlap(1, 4, -1, 0);
    try testing.expectEqual(false, check_5);
    const check_6 = intervalsOverlap(1, 4, 5, 6);
    try testing.expectEqual(false, check_6);
}

test "boxes overlap" {
    const check_1 = boxesOverlap(Vec2f{ 0, 0 }, Vec2f{ 1, 1 }, Vec2f{ 0.5, 0.5 }, Vec2f{ 1.5, 1.5 });
    try testing.expectEqual(true, check_1);
    const check_2 = boxesOverlap(Vec2f{ 0, 0 }, Vec2f{ 1, 1 }, Vec2f{ 0.5, 0.5 }, Vec2f{ 0.75, 0.75 });
    try testing.expectEqual(true, check_2);
    const check_3 = boxesOverlap(Vec2f{ 0, 0 }, Vec2f{ 1, 1 }, Vec2f{ 1.5, 1.5 }, Vec2f{ 0.75, 0.75 });
    try testing.expectEqual(true, check_3);
    const check_4 = boxesOverlap(Vec2f{ 0, 0 }, Vec2f{ 1, 1 }, Vec2f{ 1.5, 1.5 }, Vec2f{ 1.75, 1.75 });
    try testing.expectEqual(false, check_4);
}

test "check balls overlap" {
    const a = Ball2f{ .centre = Vec2f{ 0, 0 }, .radius = 1.0 };
    const b1 = Ball2f{ .centre = Vec2f{ 0.5, 0.5 }, .radius = 0.1 };
    const b2 = Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.6 };
    const b3 = Ball2f{ .centre = Vec2f{ 1.0, 1.0 }, .radius = 0.4 };
    const b4 = Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.5 };

    const check_1 = a.overlapsBall(b1);
    try testing.expectEqual(true, check_1);
    const check_2 = a.overlapsBall(b2);
    try testing.expectEqual(true, check_2);
    const check_3 = a.overlapsBall(b3);
    try testing.expectEqual(false, check_3);
    const check_4 = a.overlapsBall(b4);
    try testing.expectEqual(false, check_4);
}

test "check balls overlap 8x" {
    const a = Ball2f{ .centre = Vec2f{ 0, 0 }, .radius = 1.0 };
    const balls = [_]Ball2f{
        Ball2f{ .centre = Vec2f{ 0.5, 0.5 }, .radius = 0.1 },
        Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.6 },
        Ball2f{ .centre = Vec2f{ 1.0, 1.0 }, .radius = 0.4 },
        Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.5 },
        Ball2f{ .centre = Vec2f{ -0.8, -0.8 }, .radius = 0.1 },
        Ball2f{ .centre = Vec2f{ -1.5, 0.0 }, .radius = 0.6 },
        Ball2f{ .centre = Vec2f{ 1.0, 1.0 }, .radius = 0.5 },
        Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.0 },
    };

    const check_result = a.overlapsBallsX8(balls);
    try testing.expectEqual([8]bool{ true, true, false, false, false, true, true, false }, check_result);
}
