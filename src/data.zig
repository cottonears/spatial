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

/// Combines a node number and body index to produce a 'tree index'.
/// This is a number that universally identifies a body within its tree.
pub inline fn getTreeIndex(node_number: u16, body_index: u8) u24 {
    const node_num_u24: u24 = @intCast(node_number);
    return (node_num_u24 << 8) + body_index;
}

/// Retrieves the 'node number' from a tree index.
pub inline fn getNodeNumberIndex(tree_index: u24) u16 {
    const node_num_u24: u24 = tree_index >> 8;
    return @intCast(node_num_u24);
}

/// Retrieves the 'body index' from a tree index.
pub inline fn getBodyIndex(tree_index: u24) u8 {
    const body_index_u24: u24 = tree_index & 0x0000FF;
    return @intCast(body_index_u24);
}

pub const LeafNode = struct {
    node_number: u16,
    bodies: std.ArrayList(Ball2f), // IDEA: try using a MultiArrayList here and compare performance
    bball: Ball2f,
    // bball_centre: Vec2f,
    // bball_radius: f32 = 0,
    // bbox_width: f32 = 0.0, TODO: add these back in
    // bbox_height: f32 = 0.0,
    bounds_require_update: bool = false,
    const Self = @This();

    /// Initialises the LeafNode and ensures it has the desired capacity.
    pub fn init(allocator: std.mem.Allocator, node_number: u16, centre: Vec2f, capacity: u8) !Self {
        const bodies = try std.ArrayList(Ball2f).initCapacity(allocator, capacity);
        return .{
            .bball = Ball2f{ .centre = centre },
            .bodies = bodies,
            .node_number = node_number,
        };
    }

    /// Destroys the LeafNode and frees any allocated memory.
    pub fn deinit(self: *Self) void {
        self.bodies.deinit();
    }

    /// Returns the number of items stored in the LeafNode
    pub fn length(self: Self) u8 {
        return @intCast(self.bodies.items.len);
    }

    /// Attempts to add a body to this node and returns its tree index if successful.
    /// Not thread safe.
    pub fn addBody(self: *Self, b: Ball2f) !u24 {
        // IDEA: might be worth having an option to record a log of capacity expansions
        const i: u24 = @intCast(self.bodies.items.len);
        std.debug.assert(i < 256);
        try self.bodies.append(b);
        self.bounds_require_update = true; // self.bounds_require_update or body is not enclosed by existing bounds
        return getTreeIndex(self.node_number, @intCast(i));
    }

    /// Gets a copy of the body with the specified index
    pub fn getBody(self: Self, tree_index: u24) Ball2f {
        const body_index = getBodyIndex(tree_index);
        std.debug.assert(body_index < self.length());
        return self.bodies.items[body_index];
    }

    /// Removes the body at the specified index and returns it.
    /// NOTE: Invalidates all body indexes for this leaf-node!
    pub fn removeBody(self: *Self, index: u24) Ball2f {
        const body_index = getBodyIndex(index);
        std.debug.assert(body_index < self.length());
        const body = self.bodies.swapRemove(body_index);
        self.bounds_require_update = true; // or body is not within margin of the enclosed bounds
        return body;
    }

    /// Updates the bounding volume for this node.
    pub fn updateBounds(self: *Self) void {
        if (!self.bounds_require_update or self.length() == 0) return;
        self.bball = self.bodies.items[0];
        for (1..self.length()) |i| {
            // IDEA: it might be more efficient to do an in-place update below
            self.bball = core.getEncompassingBall(self.bball, self.bodies.items[i]);
            // IDEA: figuring out the encompassing ball is a little complicated
            //       It might be more efficient to update the bounding box in the loop
            //       Then we can fit the encompassing ball based on that afterwards
        }
        self.bounds_require_update = false;
    }

    // IDEA: why don't we call this resize and allow users to leave things (i.e., static bodies) in there?
    pub fn clear(self: *Self) !void {
        try self.bodies.resize(0);
        self.bball.radius = 0;
    }

    // checks if any overlap is possible
    pub fn isOverlapPossible(self: Self, b: Ball2f) bool {
        return self.length() > 0 and self.bball.overlapsBall(b);
    }

    pub fn getOverlappingIndexes(self: Self, buff: []u24, b: Ball2f) ![]u24 {
        if (!self.isOverlapPossible(b)) return &[0]u24{};
        std.debug.assert(self.bodies.items.len < std.math.maxInt(u8));
        var buff_index: u24 = 0;
        for (0..self.bodies.items.len) |i| {
            if (self.bodies.items[i].overlapsBall(b)) {
                const tree_index = getTreeIndex(self.node_number, @intCast(i));
                buff[buff_index] = tree_index;
                buff_index += 1;
            }
        }
        return buff[0..buff_index];
    }
};

const testing = std.testing;
const tolerance = 0.0001;

test "test leaf node" {
    var leaf = try LeafNode.init(testing.allocator, 7, Vec2f{ 0, 0 }, 16);
    defer leaf.deinit();
    const a = Ball2f{ .centre = Vec2f{ 1, 1 } };
    const b = Ball2f{ .centre = Vec2f{ 3, 3 } };
    const la = try leaf.addBody(a);
    const lb = try leaf.addBody(b);
    try testing.expectEqual(2, leaf.length());
    try testing.expectEqual(0x700, la);
    try testing.expectEqual(0x701, lb);
    try testing.expectEqual(true, leaf.bounds_require_update);

    leaf.updateBounds();
    var index_buff: [8]u24 = undefined;
    const query_ball = Ball2f{ .centre = Vec2f{ 3, 4 }, .radius = 1.1 };
    const overlap_1 = leaf.isOverlapPossible(query_ball);
    const overlap_tis = try leaf.getOverlappingIndexes(&index_buff, query_ball);
    try testing.expectEqual(false, leaf.bounds_require_update);
    try testing.expectApproxEqAbs(std.math.sqrt2, leaf.bball.radius, tolerance);
    try testing.expectApproxEqAbs(2.0, leaf.bball.centre[0], tolerance);
    try testing.expectApproxEqAbs(2.0, leaf.bball.centre[1], tolerance);
    try testing.expectEqual(true, overlap_1);
    try testing.expectEqual(1, overlap_tis.len);

    try leaf.clear();
    try testing.expectEqual(0, leaf.length());
    try testing.expectApproxEqAbs(0, leaf.bball.radius, tolerance);
    const overlap_2 = leaf.isOverlapPossible(query_ball);
    try testing.expectEqual(false, overlap_2);
}

//let's call this the 'naive implementation' and get it working first
pub const QuadNode1 = struct {
    node_number: u16,
    bball: Ball2f = undefined,
    child_ptrs: ChildUnion = undefined,
    empty: bool = true, // true if there are no bodies within any leaf node successors
    const ChildUnion = union(enum) { QuadNode: [4]*Self, LeafNode: [4]*LeafNode };
    const Self = @This();

    // Iterates (depth first) through children and updates their bounding volume info.
    // Updates this Node's bounds info if necessary.
    pub fn updateBounds(self: *Self) void {
        self.empty = true;
        switch (self.child_ptrs) {
            .QuadNode => |quads| {
                self.empty = true;
                for (0..4) |i| {
                    quads[i].updateBounds();
                    if (!quads[i].empty) {
                        self.bball = if (self.empty) quads[i].bball else core.getEncompassingBall(self.bball, quads[i].bball);
                        self.empty = false;
                    }
                    // IDEA: it might be more efficient to do an in-place update above
                }
            },
            .LeafNode => |leaves| {
                for (0..4) |i| {
                    leaves[i].updateBounds();
                    if (leaves[i].length() > 0) {
                        self.bball = if (self.empty) leaves[i].bball else core.getEncompassingBall(self.bball, leaves[i].bball);
                        self.empty = false;
                    }
                }
            },
        }
    }

    // checks if any overlap is possible
    pub fn isOverlapPossible(self: Self, b: Ball2f) bool {
        return !self.empty and self.bball.overlapsBall(b);
    }

    pub fn getOverlappingIndexes(self: Self, buff: []u24, b: Ball2f) ![]u24 {
        if (!self.isOverlapPossible(b)) return &[0]u24{};
        var b_index: u24 = 0;
        switch (self.child_ptrs) {
            .QuadNode => |quad_children| {
                for (quad_children) |quad| {
                    const q_slice = try quad.getOverlappingIndexes(buff[b_index..], b);
                    b_index += @intCast(q_slice.len);
                }
            },
            .LeafNode => |leaf_children| {
                for (leaf_children) |leaf| {
                    const q_slice = try leaf.getOverlappingIndexes(buff[b_index..], b);
                    b_index += @intCast(q_slice.len);
                }
            },
        }
        return buff[0..b_index];
    }

    // TODO: implement the below
    //pub fn writeSvg(Self: self, allocator: std.mem.Allocator, fname: []const u8) !void {}
};

test "quad node 1" {
    var buckets: [4]LeafNode = undefined;
    buckets[0] = try LeafNode.init(std.testing.allocator, 0, Vec2f{ -5, 5 }, 10);
    buckets[1] = try LeafNode.init(std.testing.allocator, 1, Vec2f{ 5, 5 }, 10);
    buckets[2] = try LeafNode.init(std.testing.allocator, 2, Vec2f{ -5, -5 }, 10);
    buckets[3] = try LeafNode.init(std.testing.allocator, 3, Vec2f{ 5, 5 }, 10);
    defer for (0..4) |i| buckets[i].deinit();
    var qn = QuadNode1{
        .node_number = 4,
        .bball = Ball2f{ .centre = Vec2f{ 0, 0 } },
        .child_ptrs = QuadNode1.ChildUnion{ .LeafNode = [4]*LeafNode{
            &buckets[0],
            &buckets[1],
            &buckets[2],
            &buckets[3],
        } },
    };

    const a = Ball2f{ .centre = Vec2f{ 1.0, 0.0 }, .radius = 1.0 };
    const b = Ball2f{ .centre = Vec2f{ -2.0, -2.0 }, .radius = 0.8 };
    const c = Ball2f{ .centre = Vec2f{ -6.0, -5.0 }, .radius = 1.2 };
    std.debug.print("quad-node {}; bounding ball = {any}\n", .{ qn.node_number, qn.bball });
    const index_a = try buckets[1].addBody(a);
    qn.updateBounds();
    std.debug.print("quad-node {}; bounding ball = {any}\n", .{ qn.node_number, qn.bball });
    const index_b = try buckets[2].addBody(b);
    qn.updateBounds();
    std.debug.print("quad-node {}; bounding ball = {any}\n", .{ qn.node_number, qn.bball });
    const index_c = try buckets[2].addBody(c);
    qn.updateBounds();
    std.debug.print("quad-node {}; bounding ball = {any}\n", .{ qn.node_number, qn.bball });

    var index_buff: [8]u24 = undefined;
    const query_region = Ball2f{ .centre = Vec2f{ 3, 3 }, .radius = 1 };
    const may_overlap_1 = qn.isOverlapPossible(query_region);
    const overlap_indexes_1 = try qn.getOverlappingIndexes(&index_buff, query_region);

    // TODO: below is giving unexpected results, write image to svg and investigate
    try testing.expectEqual(false, may_overlap_1);
    try testing.expectEqual(0, overlap_indexes_1.len);
    try testing.expectEqual(false, index_a == index_b);
    try testing.expectEqual(false, index_a == index_c);
    try testing.expectEqual(false, index_b == index_c);

    const query_region_2 = Ball2f{ .centre = Vec2f{ -3.5, 3.5 }, .radius = 0.5 };
    const overlap_indexes_2 = try qn.getOverlappingIndexes(index_buff[overlap_indexes_1.len..], query_region_2);
    try testing.expectEqual(0, overlap_indexes_2.len);
}
