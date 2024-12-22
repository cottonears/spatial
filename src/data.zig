//! This module contains spatial data structures.

const std = @import("std");
const core = @import("core.zig");
const svg = @import("svg.zig");

const Vec2f = core.Vec2f;
const Vec3f = core.Vec3f;
const Vec4f = core.Vec4f;
const Vec6f = core.Vec6f;
const Vec8f = core.Vec8f;
const Vec12f = core.Vec12f;
const Vec16f = core.Vec16f;
const Ball2f = core.Ball2f;
const Box2f = core.Box2f;

pub const LeafNode = struct {
    bodies: std.ArrayList(Ball2f) = undefined, // IDEA: try using a MultiArrayList here and compare performance
    bball: Ball2f = undefined,
    // bball_centre: Vec2f,
    // bball_radius: f32 = 0,
    // bbox_width: f32 = 0.0, TODO: add these back in
    // bbox_height: f32 = 0.0,
    bounds_require_update: bool = false,
    const Self = @This();

    /// Initialises the LeafNode and ensures it has the desired capacity.
    pub fn init(allocator: std.mem.Allocator, centre: Vec2f, capacity: u16) !Self {
        const bodies = try std.ArrayList(Ball2f).initCapacity(allocator, capacity);
        return .{
            .bball = Ball2f{ .centre = centre, .radius = 0.0 },
            .bodies = bodies,
        };
    }

    /// Destroys the LeafNode and frees any allocated memory.
    pub fn deinit(self: *Self) void {
        self.bodies.deinit();
    }

    /// Returns the number of items stored in the LeafNode
    pub fn length(self: Self) u16 {
        return @intCast(self.bodies.items.len);
    }

    /// Attempts to add a body to this node and returns its index if successful.
    pub fn addBody(self: *Self, b: Ball2f) !u8 {
        // IDEA: might be worth having an option to record a log of capacity expansions
        try self.bodies.append(b);
        self.bounds_require_update = true; // self.bounds_require_update or body is not enclosed by existing bounds
        return @intCast(self.bodies.items.len);
    }

    /// Gets a copy of the body with the specified index
    pub fn getBody(self: Self, index: u8) Ball2f {
        return self.bodies.items[index];
    }

    /// Removes the body at the specified index and returns it.
    pub fn removeBody(self: *Self, index: u8) Ball2f {
        const b = self.bodies.swapRemove(index);
        self.bounds_require_update = true; // or body is not within margin of the enclosed bounds
        return b;
    }

    /// Updates the bounding volume for this node.
    pub fn updateBounds(self: *Self) void {
        if (self.length() > 0) {
            self.bball = self.bodies.items[0];
            for (1..self.length()) |i| {
                // IDEA: it might be more efficient to do an in-place update below
                self.bball = core.getEncompassingBall(self.bball, self.bodies.items[i]);
                // IDEA: figuring out the encompassing ball is a little complicated
                //       It might be more efficient to update the bounding box in the loop
                //       Then we can fit the encompassing ball based on that afterwards
            }
        }
        self.bounds_require_update = false;
    }

    pub fn clear(self: *Self) !void {
        try self.bodies.resize(0);
        self.bball.radius = 0;
    }

    // checks if any overlap is possible
    pub fn isOverlapPossible(self: Self, centre: Vec2f, radius: f32) bool {
        return (self.length() == 0) or self.bball.overlapsBall(
            Ball2f{ .centre = centre, .radius = radius },
        );
    }

    // TODO: get this working nicely with slices
    // pub fn getOverlappingIndexes(self: Self, slice: []u8, start_index: u8) !void {
    //     for (self.bodies.items) |b| {
    //         if (core.norm(self.position - b.position) < self.bball_radius + b.radius) try list.append(b);
    //     }
    // }
};

const testing = std.testing;
const tolerance = 0.0001;

test "test leaf node" {
    var leaf = try LeafNode.init(testing.allocator, Vec2f{ 0, 0 }, 16);
    defer leaf.deinit();
    const a = Ball2f{ .centre = Vec2f{ 1, 1 }, .radius = 0 };
    const b = Ball2f{ .centre = Vec2f{ 3, 3 }, .radius = 0 };
    const a_index = leaf.addBody(a);
    const b_index = leaf.addBody(b);
    const update_needed_before = leaf.bounds_require_update;
    leaf.updateBounds();
    const update_needed_after = leaf.bounds_require_update;
    const overlap_check = leaf.isOverlapPossible(Vec2f{ 3, 4 }, 0);

    try testing.expectEqual(2, leaf.length());
    try testing.expectEqual(0, a_index);
    try testing.expectEqual(1, b_index);
    try testing.expectEqual(true, update_needed_before);
    try testing.expectEqual(true, update_needed_after);
    try testing.expectApproxEqAbs(2.0, leaf.bball.radius, tolerance);
    try testing.expectApproxEqAbs(2.0, leaf.bball.centre[0], tolerance);
    try testing.expectApproxEqAbs(2.0, leaf.bball.centre[1], tolerance);
    try testing.expectEqual(false, overlap_check);
}
