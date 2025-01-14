const std = @import("std");
const core = @import("core.zig");
const Vec2f = core.Vec2f;

/// A circular region in 2D Euclidean space.
///  I.e., the region B := { b in R^2 : ||c, b|| < R }.
pub const Ball2f = struct {
    centre: Vec2f,
    radius: f32 = 0,
    const Self = @This();

    /// Returns a ball that encompasses both input balls.
    pub fn getEncompassing(a: Ball2f, b: Ball2f) Ball2f {
        const d = a.centre - b.centre;
        const dist = core.norm(d);
        if (b.radius == 0 or (dist + b.radius < a.radius)) return a; // a encompasses b
        if (a.radius == 0 or (dist + a.radius < b.radius)) return b; // b encompasses a
        // otherwise, create a new circle that encompasses both a and b
        return .{
            .radius = 0.5 * (dist + a.radius + b.radius),
            .centre = core.scaledVec(0.5, (a.centre + b.centre)) + core.scaledVec(0.5 * (a.radius - b.radius) / dist, d),
        };
    }

    /// Returns true if this ball is empty.
    pub fn isEmpty(self: Self) bool {
        return self.radius == 0;
    }

    /// Returns true if the two balls overlap.
    pub fn overlapsOther(self: Self, other: Ball2f) bool {
        if (self.radius == 0 or other.radius == 0) return false;
        const r_sum = self.radius + other.radius;
        return core.squaredSum(self.centre - other.centre) < r_sum * r_sum;
    }

    // One way of doing this would be to do the following (might be a bit inefficient):
    // - define d as the line joining the centres
    // - check where the box's edges intersect d
    // - overlap if the intersection point is within the ball's radius of its centre
    // pub fn overlapsBox(self: Self, b: Box2f) bool {
    //     return false;
    // }
};

/// An axis-aligned rectangular region in 2D Euclidean space.
///  I.e., the region B: = { b in R^2 : |c_y - b_y| < 0.5 * HW and |c_y - b_y| < 0.5 * HH }.
pub const Box2f = struct {
    centre: Vec2f,
    half_width: f32 = 0.0,
    half_height: f32 = 0.0,

    // TODO: unfinished implementation - fix this!
    pub fn getEncompassing(a: Box2f, b: Box2f) Box2f {
        if (b.half_width == 0 or b.half_height == 0) return a; // a encompasses b
        if (a.half_width == 0 or a.half_height == 0) return b; // b encompasses a
        return .{
            .centre = core.scaledVec(0.5, (a.centre + b.centre)),
            .half_width = a.half_width + b.half_width,
            .half_height = a.half_height + b.half_height,
        };
    }

    /// Returns true if this box is empty.
    pub fn isEmpty(self: Box2f) bool {
        return self.half_width == 0 or self.half_height == 0;
    }

    /// Returns true if the two boxes overlap.
    pub fn overlapsOther(self: Box2f, other: Box2f) bool {
        const d = self.centre - other.centre;
        const width_sum = self.half_width + other.half_width;
        const height_sum = self.half_height + other.half_height;
        return @abs(d[0]) < width_sum and @abs(d[1]) < height_sum;
    }
};

const testing = std.testing;

test "check balls overlap" {
    const a = Ball2f{ .centre = Vec2f{ 0, 0 }, .radius = 1.0 };
    const b1 = Ball2f{ .centre = Vec2f{ 0.5, 0.5 }, .radius = 0.1 };
    const b2 = Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.6 };
    const b3 = Ball2f{ .centre = Vec2f{ 1.0, 1.0 }, .radius = 0.4 };
    const b4 = Ball2f{ .centre = Vec2f{ 1.5, 0.0 }, .radius = 0.5 };

    const check_1 = a.overlapsOther(b1);
    try testing.expectEqual(true, check_1);
    const check_2 = a.overlapsOther(b2);
    try testing.expectEqual(true, check_2);
    const check_3 = a.overlapsOther(b3);
    try testing.expectEqual(false, check_3);
    const check_4 = a.overlapsOther(b4);
    try testing.expectEqual(false, check_4);
}

test "boxes overlap" {
    const a = Box2f{ .centre = Vec2f{ 0.5, 0.5 }, .half_width = 0.5, .half_height = 0.5 };
    const b1 = Box2f{ .centre = Vec2f{ 0.75, 0.75 }, .half_width = 0.5, .half_height = 0.5 };
    const b2 = Box2f{ .centre = Vec2f{ 0.625, 0.625 }, .half_width = 0.125, .half_height = 0.125 };
    const b3 = Box2f{ .centre = Vec2f{ 1.125, 1.125 }, .half_width = 0.375, .half_height = 0.375 };
    const b4 = Box2f{ .centre = Vec2f{ 1.625, 1.625 }, .half_width = 0.125, .half_height = 0.125 };
    const check_1 = a.overlapsOther(b1);
    const check_2 = a.overlapsOther(b2);
    const check_3 = a.overlapsOther(b3);
    const check_4 = a.overlapsOther(b4);
    try testing.expectEqual(true, check_1);
    try testing.expectEqual(true, check_2);
    try testing.expectEqual(true, check_3);
    try testing.expectEqual(false, check_4);
}
