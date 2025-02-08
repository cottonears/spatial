//! This module contains definitions of volume types and related functions.
const std = @import("std");
const calc = @import("calc.zig");
const Vec2f = calc.Vec2f;

/// A circular region in 2D Euclidean space.
/// B := { b in R^2 : ||c, b|| < radius }.
pub const Ball2f = struct {
    centre: Vec2f,
    radius: f32,
    const Self = @This();

    pub fn isEmpty(self: Self) bool {
        return self.radius == 0;
    }

    pub fn getBoundingBox(self: Self) Box2f {
        return Box2f{
            .centre = self.centre,
            .half_height = self.radius,
            .half_width = self.radius,
        };
    }
};

/// An axis-aligned rectangular region in 2D Euclidean space.
///  B: = { b in R^2 : |c[0] - b[0]| < half_width and |c[1] - b[1]| < half_height }.
pub const Box2f = struct {
    centre: Vec2f,
    half_width: f32,
    half_height: f32,
    const Self = @This();

    pub fn isEmpty(self: Self) bool {
        return self.half_width == 0 or self.half_height == 0;
    }

    pub fn getBoundingBall(self: Self) Ball2f {
        return Ball2f{
            .centre = self.centre,
            .radius = calc.norm(Vec2f{ self.half_width, self.half_heigh }),
        };
    }

    pub fn getCorners(self: Self) [2]Vec2f {
        const offset = Vec2f{ self.half_width, self.half_height };
        return [2]Vec2f{ self.centre - offset, self.centre + offset };
    }
};

/// Returns true if the two volumes overlap.
/// Assumes neither of them is empty
pub fn checkVolumesOverlap(a: anytype, b: anytype) bool {
    return switch (@TypeOf(a)) {
        Ball2f => if (@TypeOf(b) == Ball2f) checkOverlapBallBall(a, b) else checkOverlapBallBox(a, b),
        Box2f => if (@TypeOf(b) == Box2f) checkOverlapBoxBox(a, b) else checkOverlapBallBox(b, a),
        else => unreachable, // overlap check has not been implemented for this volume
    };
}

/// Returns a volume (of type T) that encompasses both a and b.
pub fn getEncompassingVolume(T: type, a: anytype, b: anytype) T {
    switch (T) {
        Ball2f => {
            const ball_a: Ball2f = if (@TypeOf(a) == Ball2f) a else a.getBoundingBall();
            const ball_b: Ball2f = if (@TypeOf(b) == Ball2f) b else b.getBoundingBall();
            return getEncompassingBall(ball_a, ball_b);
        },
        Box2f => {
            const box_a: Box2f = if (@TypeOf(a) == Box2f) a else a.getBoundingBox();
            const box_b: Box2f = if (@TypeOf(b) == Box2f) b else b.getBoundingBox();
            return getEncompassingBox(box_a, box_b);
        },
        else => unreachable, // T is of unsupported type
    }
}

/// Returns the time of the first collision for two volumes moving with constant velocity.
/// Time will be a number in range [0, float-max] (float-max => no collision).
pub fn solveForCollision(a: anytype, v_a: Vec2f, b: anytype, v_b: Vec2f) f32 {
    return switch (@TypeOf(a)) {
        // TODO: add support for boxes and mixed-volume checks
        Ball2f => solveCollisionBallBall(a, v_a, b, v_b),
        else => unreachable, // overlap check has not been implemented for this volume
    };
}

/// Returns a ball that encompasses both input balls.
fn getEncompassingBall(a: Ball2f, b: Ball2f) Ball2f {
    if (a.isEmpty()) return b;
    if (b.isEmpty()) return a;
    const d = a.centre - b.centre;
    const r_diff = a.radius - b.radius;
    const dist = calc.norm(d);
    if (b.radius == 0 or (dist <= r_diff)) return a; // a encompasses b
    if (a.radius == 0 or (dist <= -r_diff)) return b; // b encompasses a
    return .{
        .radius = 0.5 * (dist + a.radius + b.radius),
        .centre = calc.scaledVec(0.5, a.centre + b.centre) + calc.scaledVec(0.5 * r_diff / dist, d),
    };
}

/// Returns a box that encompasses both input boxes.
fn getEncompassingBox(a: Box2f, b: Box2f) Box2f {
    if (a.isEmpty()) return b;
    if (b.isEmpty()) return a;
    const x_min = @min(a.centre[0] - a.half_width, b.centre[0] - b.half_width);
    const x_max = @max(a.centre[0] + a.half_width, b.centre[0] + b.half_width);
    const y_min = @min(a.centre[1] - a.half_height, b.centre[1] - b.half_height);
    const y_max = @max(a.centre[1] + a.half_height, b.centre[1] + b.half_height);
    const half_width = 0.5 * (x_max - x_min);
    const half_height = 0.5 * (y_max - y_min);
    const enc_centre = Vec2f{ x_min + half_width, y_min + half_height };
    return .{ .centre = enc_centre, .half_width = half_width, .half_height = half_height };
}

/// Returns true if the two balls overlap.
/// Assumes neither of them is empty (radius > 0).
fn checkOverlapBallBall(a: Ball2f, b: Ball2f) bool {
    const vec_diff = a.centre - b.centre;
    const r_sum = a.radius + b.radius;
    return calc.squaredSum(vec_diff) < r_sum * r_sum;
}

/// Returns true if the two boxes overlap.
/// Assumes neither of them is empty (width and height are non-zero).
fn checkOverlapBoxBox(a: Box2f, b: Box2f) bool {
    const vec_diff = a.centre - b.centre;
    const width_sum = a.half_width + b.half_width;
    const height_sum = a.half_height + b.half_height;
    return @abs(vec_diff[0]) < width_sum and @abs(vec_diff[1]) < height_sum;
}

/// Returns true if the ball and box overlap.
/// Assumes neither of them is empty.
fn checkOverlapBallBox(a: Ball2f, b: Box2f) bool {
    const vec_diff = b.centre - a.centre;
    const norm_d = calc.norm(vec_diff);
    if (norm_d <= a.radius) return true;
    // Find the point on the edge of the circle in the direction of b.
    // Translate the point to get its position relative to b.
    // Finally, check if this edge point is within b.
    const edge_pt = calc.scaledVec(a.radius / norm_d, vec_diff) - vec_diff;
    return @abs(edge_pt[0]) < b.half_width and @abs(edge_pt[1]) < b.half_height;
}

// TODO: optimise this! Might be better to check min-dist between a and b first?
fn solveCollisionBallBall(a: Ball2f, v_a: Vec2f, b: Ball2f, v_b: Vec2f) f32 {
    const u = a.centre - b.centre;
    const u_dot_u = calc.dotProduct(u, u);
    const rad_sum_squared = (a.radius + b.radius) * (a.radius + b.radius);
    if (u_dot_u < rad_sum_squared) { // balls overlap at t = 0
        return 0;
    }
    const v = v_a - v_b;
    const coeff_0 = u_dot_u - rad_sum_squared;
    const coeff_1 = 2 * calc.dotProduct(u, v);
    const coeff_2 = calc.dotProduct(v, v);
    const square_term = coeff_1 * coeff_1 - 4 * coeff_2 * coeff_0;
    if (square_term < 0 or coeff_2 == 0) { // no real solutions
        return std.math.floatMax(f32);
    }
    // otherwise return the t closest
    const t_1 = (-coeff_1 + @sqrt(square_term)) / (2 * coeff_2);
    const t_2 = (-coeff_1 - @sqrt(square_term)) / (2 * coeff_2);
    if (t_1 < 0 and t_2 < 0) return std.math.floatMax(f32);
    if (t_1 >= 0 and t_2 >= 0) return @min(t_1, t_2);
    return if (t_1 < 0) t_2 else t_1;
}

// fn solveCollisionBallBox(a: Ball2f, v_a: Vec2f, b: Box2f, v_b: Vec2f) f32 {
//     // TODO: implement this!
//     const u = a.centre - b.centre;
//     const u_dot_u = calc.dotProduct(u, u);
//     const rad_sum_squared = (a.radius + a.radius) * (b.half_height + b.half_width);
//     if (u_dot_u < rad_sum_squared) { // balls overlap at t = 0
//         return 0;
//     }
//     const v = v_a - v_b;
//     const coeff_0 = u_dot_u - rad_sum_squared;
//     const coeff_1 = 2 * calc.dotProduct(u, v);
//     const coeff_2 = calc.dotProduct(v, v);
//     const square_term = coeff_1 * coeff_1 - 4 * coeff_2 * coeff_0;
//     if (square_term < 0 or coeff_2 == 0) { // no real solutions
//         return std.math.floatMax(f32);
//     }
//     // otherwise return the t closest
//     const t_1 = (-coeff_1 + @sqrt(square_term)) / (2 * coeff_2);
//     const t_2 = (-coeff_1 - @sqrt(square_term)) / (2 * coeff_2);
//     if (t_1 < 0 and t_2 < 0) return std.math.floatMax(f32);
//     if (t_1 >= 0 and t_2 >= 0) return @min(t_1, t_2);
//     return if (t_1 < 0) t_2 else t_1;
// }

// fn solveCollisionBoxBox(a: Box2f, v_a: Vec2f, b: Box2f, v_b: Vec2f) f32 {

//     //
//     // check when x overlaps
//     // check when y overlaps
//     //
// }

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

test "test encompassing balls" {
    const a: Ball2f = .{ .centre = calc.zero2f, .radius = 0.199 };
    const b: Ball2f = .{ .centre = calc.zero2f, .radius = 0.031 };
    const c = getEncompassingVolume(Ball2f, a, b);
    try testing.expectEqual(c.centre, a.centre);
    try testing.expectEqual(c.radius, @max(a.radius, b.radius));
}

test "encompassing boxes" {
    const a: Ball2f = .{ .centre = calc.zero2f, .radius = 0.139 };
    const b: Ball2f = .{ .centre = calc.zero2f, .radius = 0.735 };
    const c = getEncompassingVolume(Box2f, a, b);
    try testing.expectEqual(c.centre, @max(a.centre, b.centre));
    try testing.expectEqual(c.half_height, @max(a.radius, b.radius));
    try testing.expectEqual(c.half_width, @max(a.radius, b.radius));
}

test "test ball collisions" {
    const a = Ball2f{ .centre = Vec2f{ 0.0, 0.0 }, .radius = 1 };
    const b = Ball2f{ .centre = Vec2f{ 0.0, 5.0 }, .radius = 1 };
    const t = solveCollisionBallBall(a, Vec2f{ 0, 0 }, b, Vec2f{ 0, -0.1 });
    std.debug.print("solved for t = {d:.3}\n", .{t});
}
