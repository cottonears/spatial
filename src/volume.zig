const std = @import("std");
const calc = @import("calc.zig");
const Vec2f = calc.Vec2f;

/// A circular region in 2D Euclidean space.
/// B := { b in R^2 : ||c, b|| < radius }.
pub const Ball2f = struct {
    centre: Vec2f,
    radius: f32,
};

/// An axis-aligned rectangular region in 2D Euclidean space.
///  B: = { b in R^2 : |c[0] - b[0]| < half_width and |c[1] - b[1]| < half_height }.
pub const Box2f = struct {
    centre: Vec2f,
    half_width: f32,
    half_height: f32,

    pub fn getCorners(self: Box2f) [2]Vec2f {
        const offset = Vec2f{ self.half_width, self.half_height };
        return [2]Vec2f{ self.centre - offset, self.centre + offset };
    }
};

// TODO: Investigate how performance is affected if volumes in overlap to be null.

/// Returns true if the two balls overlap.
/// Assumes neither of them is empty.
pub fn checkOverlapBallBall(a: Ball2f, b: Ball2f) bool {
    const vec_diff = a.centre - b.centre;
    const r_sum = a.radius + b.radius;
    return calc.squaredSum(vec_diff) < r_sum * r_sum;
}

/// Returns true if the two boxes overlap.
/// Assumes neither of them is empty.
pub fn checkOverlapBoxBox(a: Box2f, b: Box2f) bool {
    const vec_diff = a.centre - b.centre;
    const width_sum = a.half_width + b.half_width;
    const height_sum = a.half_height + b.half_height;
    return @abs(vec_diff[0]) < width_sum and @abs(vec_diff[1]) < height_sum;
}

/// Returns true if the ball and box overlap.
/// Assumes neither of them is empty.
pub fn checkOverlapBallBox(a: Ball2f, b: Box2f) bool {
    // TODO: implement this!
    _ = a;
    _ = b;
    return false;
}

/// Returns true if the two volumes overlap.
/// Assumes neither of them is empty
pub fn checkVolumesOverlap(a: anytype, b: anytype) bool {
    return switch (@TypeOf(a)) {
        Ball2f => if (@TypeOf(b) == Ball2f) checkOverlapBallBall(a, b) else checkOverlapBallBox(a, b),
        Box2f => if (@TypeOf(b) == Box2f) checkOverlapBoxBox(a, b) else checkOverlapBallBox(b, a),
        else => unreachable, // overlap check has not been implemented for this volume
    };
}

/// Returns a ball that encompasses both input balls.
pub fn getEncompassingBall(a: ?Ball2f, b: ?Ball2f) ?Ball2f {
    if (a == null) return b;
    if (b == null) return a;
    const vec_sum = a.?.centre + b.?.centre;
    const vec_diff = a.?.centre - b.?.centre;
    const r_sum = a.?.radius + b.?.radius;
    const r_diff = a.?.radius - b.?.radius;
    const centre_dist = calc.norm(vec_diff);
    const centre_offset = if (centre_dist > 0) calc.scaledVec(0.5 * r_diff / centre_dist, vec_diff) else Vec2f{ 0, 0 };
    const enc_centre = calc.scaledVec(0.5, vec_sum) + centre_offset;
    const enc_radius = 0.5 * (centre_dist + r_sum);
    return .{ .centre = enc_centre, .radius = enc_radius };
}

/// Returns a box that encompasses both input boxes.
pub fn getEncompassingBox(a: ?Box2f, b: ?Box2f) ?Box2f {
    if (a == null) return b;
    if (b == null) return a;
    const x_min = @min(a.?.centre[0] - a.?.half_width, b.?.centre[0] - b.?.half_width);
    const x_max = @max(a.?.centre[0] + a.?.half_width, b.?.centre[0] + b.?.half_width);
    const y_min = @min(a.?.centre[1] - a.?.half_height, b.?.centre[1] - b.?.half_height);
    const y_max = @max(a.?.centre[1] + a.?.half_height, b.?.centre[1] + b.?.half_height);
    const half_width = 0.5 * (x_max - x_min);
    const half_height = 0.5 * (y_max - y_min);
    const enc_centre = Vec2f{ x_min + half_width, y_min + half_height };
    return .{ .centre = enc_centre, .half_width = half_width, .half_height = half_height };
}

// TODO: make this more robust
pub fn getEncompassingVolume(T: type, a: anytype, b: anytype) ?T {
    return switch (T) {
        Ball2f => getEncompassingBall(a, b),
        Box2f => getEncompassingBox(a, b),
        else => unreachable,
    };
}

const testing = std.testing;

test "check balls overlap" {
    const a = Ball2f{ .centre = Vec2f{ 0, 0 }, .radius = 1.0 };
    const b1 = Ball2f{ .centre = Vec2f{ 0.5, 0.5 }, .radius = 0.1 };
    const b2 = Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.6 };
    const b3 = Ball2f{ .centre = Vec2f{ 1.0, 1.0 }, .radius = 0.4 };
    const b4 = Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.5 };

    const check_1 = checkOverlapBallBall(a, b1);
    try testing.expectEqual(true, check_1);
    const check_2 = checkOverlapBallBall(a, b2);
    try testing.expectEqual(true, check_2);
    const check_3 = checkOverlapBallBall(a, b3);
    try testing.expectEqual(false, check_3);
    const check_4 = checkOverlapBallBall(a, b4);
    try testing.expectEqual(false, check_4);
}

test "boxes overlap" {
    const a = Box2f{ .centre = Vec2f{ 0.5, 0.5 }, .half_width = 0.5, .half_height = 0.5 };
    const b1 = Box2f{ .centre = Vec2f{ 0.75, 0.75 }, .half_width = 0.5, .half_height = 0.5 };
    const b2 = Box2f{ .centre = Vec2f{ 0.625, 0.625 }, .half_width = 0.125, .half_height = 0.125 };
    const b3 = Box2f{ .centre = Vec2f{ 1.125, 1.125 }, .half_width = 0.375, .half_height = 0.375 };
    const b4 = Box2f{ .centre = Vec2f{ 1.625, 1.625 }, .half_width = 0.125, .half_height = 0.125 };
    const check_1 = checkOverlapBoxBox(a, b1);
    const check_2 = checkOverlapBoxBox(a, b2);
    const check_3 = checkOverlapBoxBox(a, b3);
    const check_4 = checkOverlapBoxBox(a, b4);
    try testing.expectEqual(true, check_1);
    try testing.expectEqual(true, check_2);
    try testing.expectEqual(true, check_3);
    try testing.expectEqual(false, check_4);
}

// TODO: write further encompassing + overlap tests
// currently this case is bugged
// 1. A = (c = { 0.029, 0.378 }, r = 0.199), B = (c = { 0.123, 0.355 }, r = 0.031);  C = (c = { -0.006, 0.387 }, r = 0.163)
