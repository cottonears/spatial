//! This module contains core maths functions used throughout the library.

const std = @import("std");
pub const Vec2f = @Vector(2, f32);
pub const zero2f = Vec2f{ 0, 0 };

/// Gets an array containing the range: 0, 1, ... len - 1.
pub fn getRange(comptime T: type, comptime len: u24) [len]T {
    @setEvalBranchQuota(len + 1);
    var range: [len]T = undefined;
    for (0..len) |i| range[i] = @intCast(i);
    return range;
}

/// Gets an array containing the reversed range: len - 1, len - 2, ... 0.
pub fn getReversedRange(comptime T: type, comptime len: u24) [len]T {
    @setEvalBranchQuota(len + 1);
    var range: [len]T = undefined;
    for (0..len) |i| range[i] = @intCast(len - i - 1);
    return range;
}

/// Generates the sequence S := { x ^ (2 * n) | n in [n_start, n_end) }.
pub fn getPow2nSequence(comptime base: u8, comptime n_start: u8, comptime n_end: u8) @Vector(n_end - n_start, usize) {
    std.debug.assert(n_end > n_start);
    var seq: [n_end - n_start]usize = undefined;
    inline for (n_start..n_end, 0..) |n, i| seq[i] = try std.math.powi(usize, base, 2 * n);
    return seq;
}

/// Converts the integer k to a 32 bit float. Saves a bit of typing.
pub fn asf32(k: anytype) f32 {
    return @floatFromInt(k);
}

/// Returns || v ||, the Euclidean norm of the vector v.
pub fn norm(v: anytype) f32 {
    return @sqrt(squaredSum(v));
}

/// Returns < v, v >, the dot product of v with itself.
pub fn squaredSum(v: anytype) f32 {
    return dotProduct(v, v);
}

/// Returns < a, b >, the dot product a and b.
pub fn dotProduct(a: anytype, b: anytype) f32 {
    return @reduce(.Add, a * b);
}

/// Returns true if all elements of vectors a and b are equal.
pub fn vecsEqual(a: anytype, b: anytype) bool {
    return @reduce(.And, a == b);
}

/// Returns a unit length vector in the same direction as v, or a zero vector if it has zero length.
pub fn normalised(v: anytype) @TypeOf(v) {
    const n = norm(v);
    return if (n > 0) scaledVec(1 / norm(v), v) else zero2f;
}

/// Returns a scaled version of the input vector.
pub fn scaledVec(alpha: f32, v: anytype) @TypeOf(v) {
    const alpha_vec: @TypeOf(v) = @splat(alpha);
    return alpha_vec * v;
}

// TODO: check if this generalises to higher dimensions and modify parameters if it does.
// solves for t that minimises || u + t * v ||
fn solveTimeThatMinimisesDist(u: Vec2f, v: Vec2f) f32 {
    const denom = dotProduct(v, v);
    return if (denom == 0) 0 else -dotProduct(u, v) / denom;
}

/// Returns (t, d_min^2) between two points moving with constant velocity (no restriction on t).
/// Each point is described by the equation: r(t) = u + t * v (u and v are vectors, t is a real number).
pub fn solveMinDistSquared(u_a: Vec2f, v_a: Vec2f, u_b: Vec2f, v_b: Vec2f) Vec2f {
    const u = u_a - u_b;
    const v = v_a - v_b;
    const t: f32 = solveTimeThatMinimisesDist(u, v);
    const d = u + scaledVec(t, v);
    return .{ t, squaredSum(d) };
}

/// Returns (t, d_min^2) between two points moving with constant velocity; t clamped to [t_min, t_max].
/// Each point is described by the equation: r(t) = u + t * v (u and v are vectors, t is a real number).
pub fn solveMinDistSquaredClamp(u_a: Vec2f, v_a: Vec2f, u_b: Vec2f, v_b: Vec2f, t_min: f32, t_max: f32) Vec2f {
    const u = u_a - u_b;
    const v = v_a - v_b;
    var t = solveTimeThatMinimisesDist(u, v);
    t = @max(t_min, @min(t, t_max));
    const d = u + scaledVec(t, v);
    return .{ t, squaredSum(d) };
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

    const zero_result = solveMinDistSquared(zero2f, zero2f, zero2f, zero2f);
    try testing.expectApproxEqAbs(0, zero_result[1], tolerance);

    const static_result = solveMinDistSquared(pos_a, zero2f, pos_b, zero2f);
    try testing.expectApproxEqAbs(2, static_result[1], tolerance);

    const parallel_result = solveMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 2, 0 });
    try testing.expectApproxEqAbs(1, parallel_result[1], tolerance);

    const opposite_result = solveMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ -1, 0 });
    try testing.expectApproxEqAbs(1, opposite_result[1], tolerance);

    const perpendicular_result = solveMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 0, 1 });
    try testing.expectApproxEqAbs(0, perpendicular_result[1], tolerance);
}

test "closest dist interval" {
    const pos_a = Vec2f{ 1, 0 };
    const pos_b = Vec2f{ 0, 1 };

    const parallel_result = solveMinDistSquaredClamp(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 2, 0 }, 0, 100);
    try testing.expectApproxEqAbs(1, parallel_result[1], tolerance);

    const opposite_result = solveMinDistSquaredClamp(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ -1, 0 }, -1.5, -0.5);
    try testing.expectApproxEqAbs(1, opposite_result[1], tolerance);

    const perpendicular_result = solveMinDistSquaredClamp(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 0, 1 }, -3, 3);
    try testing.expectApproxEqAbs(0, perpendicular_result[1], tolerance);
}
