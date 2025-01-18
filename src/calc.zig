//! This module contains core maths functions used throughout the library.
//! TODO: add support for doubles and higher-dimensional vectors
const std = @import("std");
pub const Vec2f = @Vector(2, f32);
const zero_2f = Vec2f{ 0, 0 };

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

/// Returns the Euclidean norm of the vector v.
pub fn norm(v: anytype) f32 {
    return @sqrt(squaredSum(v));
}

/// Returns the norm (length) of the vector v.
pub fn squaredSum(v: anytype) f32 {
    return innerProd(v, v);
}

/// Computes the inner product of two vectors of f32s.
pub fn innerProd(a: anytype, b: anytype) f32 {
    return @reduce(.Add, a * b);
}

/// Returns true if all elements of vectors a and b are equal.
pub fn vecsEqual(a: anytype, b: anytype) bool {
    return @reduce(.And, a == b);
}

/// Returns a unit length vector in the same direction as v, or a zero vector if it has zero length.
pub fn normalised(v: anytype) @TypeOf(v) {
    const n = norm(v);
    return if (n > 0) scaledVec(1 / norm(v), v) else zero_2f;
}

/// Returns a scaled version of the input vector.
pub fn scaledVec(alpha: f32, v: anytype) @TypeOf(v) {
    const alpha_vec: @TypeOf(v) = @splat(alpha);
    return alpha_vec * v;
}

// TODO: check if this generalises to higher dimensions and modify parameters if it does.
// solves for t that minimises || u + t * v ||
pub fn solveForMinDist(u: Vec2f, v: Vec2f) f32 {
    const denom = innerProd(v, v);
    return if (denom == 0) 0 else -innerProd(u, v) / denom;
}

// returns the minimum value of || u_a + t * v_a - (b_a - t * v_a) || ^ 2 with no restriction on t
pub fn getMinDistSquared(pos_a: Vec2f, vel_a: Vec2f, pos_b: Vec2f, vel_b: Vec2f) f32 {
    const u = pos_a - pos_b;
    const v = vel_a - vel_b;
    const t: f32 = solveForMinDist(u, v);
    const d = u + scaledVec(t, v);
    return squaredSum(d);
}

// same as the above but clamps range of t to [t_min, t_max]
pub fn getMinDistSquaredForInterval(pos_a: Vec2f, vel_a: Vec2f, pos_b: Vec2f, vel_b: Vec2f, t_min: f32, t_max: f32) f32 {
    const u = pos_a - pos_b;
    const v = vel_a - vel_b;
    var t = solveForMinDist(u, v);
    t = @max(t_min, @min(t, t_max));
    const d = u + scaledVec(t, v);
    return squaredSum(d);
}

// NOTE: Doesn't handle case where half_width = 0 (implying the open ball is the empty set) sensibly.
pub fn intervalsOverlap(centre_a: f32, half_width_a: f32, centre_b: f32, half_width_b: f32) bool {
    return @abs(centre_a - centre_b) < half_width_a + half_width_b;
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
