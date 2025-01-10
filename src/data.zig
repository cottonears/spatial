const std = @import("std");
const core = @import("core.zig");
const Vec2f = core.Vec2f;
const Vec4f = core.Vec4f;
const Vec8f = core.Vec8f;
const Vec16f = core.Vec16f;
const Ball2f = core.Ball2f;

/// Raises a number to the power of (2 * n).
fn pow2n(comptime base: u8, comptime n: u8) usize {
    var s: usize = 1;
    inline for (0..n) |_| s = s * base * base;
    return s;
}

/// Gets the sequence S := { x ^ (2 * n) | n in {n_start, ..., n_end - 1} }.
/// If make_pow0_zero is true, x ^ 0 is replaced with 0.
fn getPow2nSequence(
    comptime base: u8,
    comptime n_start: u8,
    comptime n_end: u8,
    comptime make_pow0_zero: bool,
) [n_end - n_start]usize {
    var seq: [n_end - n_start]usize = undefined;
    inline for (n_start..n_end, 0..) |n, i| {
        seq[i] = if (make_pow0_zero and n == 0) 0 else pow2n(base, n);
    }
    return seq;
}

/// Gets an array containing the reversed range: len - 1, len - 2, ... 0.
fn getReversedRange(comptime T: type, comptime len: T) [len]T {
    var range: [len]T = undefined;
    inline for (0..len) |i| range[i] = len - i - 1;
    return range;
}

/// Returns log2(len) for values of len that are supported by the SquareTree struct.
fn getMinimialIntegerForRange(comptime len: u32) type {
    switch (len) {
        4 => return u2,
        16 => return u4,
        64 => return u6,
        256 => return u8,
        1024 => return u10,
        4096 => return u12,
        16384 => return u14,
        65536 => return u16,
        262144 => return u18,
        1048576 => return u20,
        4194304 => return u22,
        16777216 => return u24,
        else => unreachable, // unsupported
    }
}

/// Gets a 'shift-amount' that can be used to mutiply/divide by (base * base) using a bit-shift.
fn getShiftForBase(comptime base_num: u8) comptime_int {
    switch (base_num) {
        2 => return @as(u2, 2),
        4 => return @as(u3, 4),
        8 => return @as(u3, 6),
        16 => return @as(u4, 8),
        else => {
            @compileError("Unsupported base number (must be a small power of two)");
        },
    }
}

/// A data structure that covers a square region (size * size) of 2D Euclidean space.
pub fn SquareTree(comptime base_number: u8, comptime depth: u8) type {
    return struct {
        allocator: std.mem.Allocator,
        origin: Vec2f,
        size: f32,
        size_per_level: [depth]f32,
        scale_per_level: [depth]f32,
        bodies: [num_leaves]std.ArrayListUnmanaged(Ball2f) = undefined,
        bounds: [depth][]Ball2f = undefined,
        num_bodies: usize = 0,

        pub const NodeIndex = getMinimialIntegerForRange(num_leaves);
        pub const BodyIndex = struct {
            leaf_index: NodeIndex,
            body_number: u16, // TODO: make this type a comptime parameter
        };
        pub const nodes_in_level = getPow2nSequence(base_number, 1, depth + 1, false); // the number of nodes in each level
        pub const nodes_above_level = getPow2nSequence(base_number, 0, depth, true); // cumulative number of nodes above each level
        pub const num_leaves = nodes_in_level[depth - 1]; // number of nodes in the last (leaf) level
        pub const num_parents = nodes_above_level[depth - 1]; // the number of non-leaf nodes
        pub const num_nodes = num_parents + num_leaves; // total number of nodes
        pub const reverse_levels = getReversedRange(u8, depth);
        const base: NodeIndex = @intCast(base_number);
        const base_squared: NodeIndex = base * base;
        const level_bitshift = getShiftForBase(base_number);

        const Self = @This();

        pub fn init(allocator: std.mem.Allocator, leaf_capacity: u32, origin: Vec2f, size: f32) !Self {
            if (depth < 2) {
                @compileError("SquareTree depth must be greater than 1");
            }
            var leaf_lists: [num_leaves]std.ArrayListUnmanaged(Ball2f) = undefined;
            for (0..num_leaves) |i| {
                leaf_lists[i] = try std.ArrayListUnmanaged(Ball2f).initCapacity(allocator, leaf_capacity);
            }
            var bballs: [depth][]Ball2f = undefined;
            var size_per_lvl: [depth]f32 = undefined;
            var scale_per_lvl: [depth]f32 = undefined;
            var last_size = size;
            for (0..depth) |lvl| {
                bballs[lvl] = try allocator.alloc(Ball2f, nodes_in_level[lvl]);
                size_per_lvl[lvl] = last_size / base;
                scale_per_lvl[lvl] = 1.0 / size_per_lvl[lvl];
                last_size = last_size / base;
            }

            return Self{
                .allocator = allocator,
                .bodies = leaf_lists,
                .bounds = bballs,
                .origin = origin,
                .size = size,
                .size_per_level = size_per_lvl,
                .scale_per_level = scale_per_lvl,
            };
        }

        pub fn deinit(self: *Self) void {
            for (0..num_leaves) |i| self.bodies[i].deinit(self.allocator);
            for (0..depth) |lvl| self.allocator.free(self.bounds[lvl]);
        }

        /// Converts a node index to the index of its predecessor (1 or more levels higher).
        inline fn getPredeccessorIndex(index: NodeIndex, lvl_diff: u8) NodeIndex {
            return index >> @truncate(lvl_diff * level_bitshift);
        }

        /// Converts a node index to the index its first successor (1 or more levels lower).
        inline fn getSuccessorIndex(index: NodeIndex, lvl_diff: u8) NodeIndex {
            return index << @truncate(lvl_diff * level_bitshift);
        }

        /// Gets the indexes of immediate children of the specified parent node.
        /// Assumes the provided parent_index does not refer to a leaf node.
        fn getChildIndexes(buff: []NodeIndex, parent_index: NodeIndex) []NodeIndex {
            std.debug.assert(buff.len >= base_squared);
            const first_child_index = getSuccessorIndex(parent_index, 1);
            for (0..base_squared) |i| {
                buff[i] = first_child_index + @as(NodeIndex, @intCast(i));
            }
            return buff[0..base_squared];
        }

        /// Prints some useful information about this specific variety of square tree.
        pub fn printTypeInfo() void {
            std.debug.print("■ SquareTree({}, {}) info:\n", .{ base, depth });
            std.debug.print("   Index type: {} ({} bytes)\n", .{ @typeInfo(NodeIndex), @sizeOf(NodeIndex) });
            std.debug.print("   Number leaf nodes: {d}\n", .{num_leaves});
            std.debug.print("   Total nodes: {d}\n", .{num_nodes});
            std.debug.print("   Nodes per level: {any}\n", .{nodes_in_level});
            std.debug.print("   Nodes above level: {any}\n", .{nodes_above_level});
            std.debug.print("   Size: {d} kB\n", .{@sizeOf(Self) / 1000});
        }

        /// Returns true if the specified point is within the region indexed by this tree, false otherwise.
        pub fn isPointWithinRegion(self: Self, point: Vec2f) bool {
            const d = point - self.origin;
            return 0 <= d[0] and d[0] < self.size and 0 <= d[1] and d[1] < self.size;
        }

        // TODO: see if the performance of the below function can be improved?
        /// Gets the index of the leaf node the query point lies within.
        /// Asserts the point is within the region covered by this tree.
        pub fn getLeafIndexForPoint(self: Self, point: Vec2f) NodeIndex {
            const d = point - self.origin;
            var index: NodeIndex = 0;
            for (0..depth) |lvl| {
                const row = @as(NodeIndex, @intFromFloat(self.scale_per_level[lvl] * d[1])) % base;
                const col = @as(NodeIndex, @intFromFloat(self.scale_per_level[lvl] * d[0])) % base;
                index = (index << @truncate(level_bitshift)) + row * base + col;
            }
            return index;
        }

        /// Gets the position of a point offset from the identified node's origin
        /// NOTE: This function is slow due to integer divisions: don't use in performance critical code!.
        pub fn getNodePoint(self: Self, level: u8, index: NodeIndex, x_offset: f32, y_offset: f32) !Vec2f {
            var point = self.origin;
            for (0..level + 1) |lvl| {
                const lvl_index = getPredeccessorIndex(index, @intCast(level - lvl));
                const lvl_size = self.size_per_level[lvl];
                const row = (lvl_index / base) % base;
                const col = lvl_index % base;
                const x_lvl = if (lvl < level) core.asf32(col) else x_offset + core.asf32(col);
                const y_lvl = if (lvl < level) core.asf32(row) else y_offset + core.asf32(row);
                point += Vec2f{ lvl_size * x_lvl, lvl_size * y_lvl };
            }
            return point;
        }

        /// Gets the origin point for the identified node.
        pub fn getNodeOrigin(self: Self, level: u8, index: NodeIndex) !Vec2f {
            return self.getNodePoint(level, index, 0.0, 0.0);
        }

        /// Gets the centre point of the identified node.
        pub fn getNodeCentre(self: Self, level: u8, index: NodeIndex) !Vec2f {
            return self.getNodePoint(level, index, 0.5, 0.5);
        }

        /// Gets the corner opposite the origin for the identified node.
        pub fn getNodeCorner(self: Self, level: u8, index: NodeIndex) !Vec2f {
            return self.getNodePoint(level, index, 1.0, 1.0);
        }

        /// Updates the bounding volumes within the tree.
        pub fn updateBounds(self: *Self) void {
            for (reverse_levels) |lvl| {
                if (lvl == depth - 1) { // leaf nodes
                    for (0..num_leaves) |i| {
                        var ball = Ball2f{ .centre = Vec2f{ 0, 0 }, .radius = 0 }; // start with an empty ball
                        for (0..self.bodies[i].items.len) |j| {
                            ball = core.getEncompassingBall(ball, self.bodies[i].items[j]);
                        }
                        self.bounds[lvl][i] = ball;
                    }
                } else { // parent nodes
                    var child_index_buff: [base_squared]NodeIndex = undefined;
                    for (0..self.bounds[lvl].len) |i| {
                        var ball = Ball2f{ .centre = Vec2f{ 0, 0 }, .radius = 0 }; // start with an empty ball
                        const child_indexes = getChildIndexes(&child_index_buff, @intCast(i));
                        for (child_indexes) |j| {
                            ball = core.getEncompassingBall(ball, self.bounds[lvl + 1][j]);
                        }
                        self.bounds[lvl][i] = ball;
                    }
                }
            }
        }

        pub fn getOverlappingBodies(self: Self, buff: []BodyIndex, query_ball: Ball2f) ![]BodyIndex {
            var search_buff: [num_leaves]NodeIndex = undefined;
            for (0..base_squared) |i| search_buff[i] = @intCast(i);
            var search_len: usize = @intCast(base_squared);
            var next_buff: [num_leaves]NodeIndex = undefined;
            var next_len: usize = 0;
            // iterate through all parent nodes
            for (0..depth - 1) |lvl| {
                for (search_buff[0..search_len]) |i| {
                    if (query_ball.overlapsBall(self.bounds[lvl][i])) {
                        next_len += getChildIndexes(next_buff[next_len..], i).len;
                    }
                }
                @memcpy(search_buff[0..next_len], next_buff[0..next_len]);
                search_len = next_len;
                next_len = 0;
            }
            // iterate through leaf nodes
            var buff_index: usize = 0;
            for (search_buff[0..search_len]) |leaf_index| {
                for (self.bodies[leaf_index].items, 0..) |b, i| {
                    if (query_ball.overlapsBall(b)) {
                        buff[buff_index] = BodyIndex{
                            .leaf_index = leaf_index,
                            .body_number = @intCast(i),
                        };
                        buff_index += 1;
                    }
                }
            }
            return if (buff_index > 0) buff[0..buff_index] else &[0]BodyIndex{};
        }

        pub fn addBody(self: *Self, ball: Ball2f) !BodyIndex {
            const leaf_index = self.getLeafIndexForPoint(ball.centre);
            const body_num = self.bodies[leaf_index].items.len;
            // TODO: add the body to a neighbouring leaf if this leaf has reached capacity?
            try self.bodies[leaf_index].append(self.allocator, ball);
            self.num_bodies += 1;
            return BodyIndex{ .leaf_index = leaf_index, .body_number = @intCast(body_num) };
        }

        pub fn getBody(self: Self, body_index: BodyIndex) Ball2f {
            return self.bodies[body_index.leaf_index].items[body_index.body_number];
        }

        // IDEA: The below will invalidate indexing, which is inconvenient.
        //       We could prevent this by swapping with an empty ball maybe?
        //       Add will need to be updated to search for the first empty position, but that should be OK.
        pub fn removeBody(self: *Self, body_index: BodyIndex) Ball2f {
            const b = self.bodies[body_index.leaf_index].swapRemove(body_index.body_number);
            self.num_bodies -= 1;
            return b;
        }
    };
}

const testing = std.testing;
const tolerance: f32 = 0.0001;

test "square tree init" {
    const Tree2x8 = SquareTree(2, 8);
    var qt = try Tree2x8.init(testing.allocator, 8, Vec2f{ 0, 0 }, 1.0);
    defer qt.deinit();

    const Tree4x4 = SquareTree(4, 4);
    var ht = try Tree4x4.init(testing.allocator, 8, Vec2f{ 0, 0 }, 1.0);
    defer ht.deinit();
}

test "quad tree indexing" {
    const tree_depth = 3;
    const QuadTree4 = SquareTree(2, tree_depth);
    var qt = try QuadTree4.init(testing.allocator, 0, Vec2f{ 0, 0 }, 8.0);
    defer qt.deinit();

    const pt_check_1 = qt.isPointWithinRegion(Vec2f{ 0, 0 });
    const pt_check_2 = qt.isPointWithinRegion(Vec2f{ 5, 7 });
    const pt_check_3 = qt.isPointWithinRegion(Vec2f{ -1, 3 });
    const pt_check_4 = qt.isPointWithinRegion(Vec2f{ 8.0, 8.0 });
    try testing.expectEqual(true, pt_check_1);
    try testing.expectEqual(true, pt_check_2);
    try testing.expectEqual(false, pt_check_3);
    try testing.expectEqual(false, pt_check_4);

    const pt_a = Vec2f{ 4.1, 4.1 };
    const index_a = qt.getLeafIndexForPoint(pt_a);
    const parent_a = QuadTree4.getPredeccessorIndex(index_a, 1);
    const grandparent_a = QuadTree4.getPredeccessorIndex(index_a, 2);
    try testing.expectEqual(48, index_a);
    try testing.expectEqual(12, parent_a);
    try testing.expectEqual(3, grandparent_a);

    const pt_b = Vec2f{ 7.99, 7.99 };
    const ln_num_b = qt.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(QuadTree4.num_leaves - 1, ln_num_b);
    for (0..QuadTree4.num_leaves) |n| {
        const pred_0 = QuadTree4.getPredeccessorIndex(@intCast(n), 0);
        const pred_1 = QuadTree4.getPredeccessorIndex(@intCast(n), 1);
        const pred_2 = QuadTree4.getPredeccessorIndex(@intCast(n), 2);
        try testing.expectEqual(n, pred_0);
        try testing.expectEqual(n / 16, pred_2);
        try testing.expectEqual(n / 4, pred_1);
    }
}

test "hex tree indexing" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(4, tree_depth);
    var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 }, 8.0);
    defer tree.deinit();

    const pt_check_1 = tree.isPointWithinRegion(Vec2f{ 0, 0 });
    const pt_check_2 = tree.isPointWithinRegion(Vec2f{ 5, 7 });
    const pt_check_3 = tree.isPointWithinRegion(Vec2f{ -1, 3 });
    const pt_check_4 = tree.isPointWithinRegion(Vec2f{ 8.0, 8.0 });
    try testing.expectEqual(true, pt_check_1);
    try testing.expectEqual(true, pt_check_2);
    try testing.expectEqual(false, pt_check_3);
    try testing.expectEqual(false, pt_check_4);

    const pt_a = Vec2f{ 4.1, 2.1 };
    const index_a = tree.getLeafIndexForPoint(pt_a);
    const parent_a = HexTree2.getPredeccessorIndex(index_a, 1);
    try testing.expectEqual(96, index_a);
    try testing.expectEqual(6, parent_a);

    const pt_b = Vec2f{ 7.99, 7.99 };
    const index_b = tree.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(HexTree2.num_leaves - 1, index_b);

    for (0..HexTree2.num_leaves) |n| {
        const parent_index = HexTree2.getPredeccessorIndex(@intCast(n), 1);
        const parent_child0_index = HexTree2.getSuccessorIndex(parent_index, 1);
        try testing.expectEqual(n / 16, parent_index);
        try testing.expect(@abs(n - parent_child0_index) < 16);
    }
}

test "hex tree overlap" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(4, tree_depth);
    var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 }, 1.0);
    defer tree.deinit();

    const a = Ball2f{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 };
    const b = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 };
    const c = Ball2f{ .centre = Vec2f{ 0.2, 0.9 }, .radius = 0.1 };
    const index_a = try tree.addBody(a);
    const index_b = try tree.addBody(b);
    const index_c = try tree.addBody(c);
    tree.updateBounds();

    var index_buff: [8]HexTree2.BodyIndex = undefined;
    const query_region_1 = Ball2f{ .centre = Vec2f{ 0.9, 0.5 }, .radius = 0.1 };
    const overlap_indexes_1 = try tree.getOverlappingBodies(&index_buff, query_region_1);
    try testing.expectEqual(0, overlap_indexes_1.len);
    try testing.expectEqual(false, index_a.leaf_index == index_b.leaf_index);
    try testing.expectEqual(false, index_a.leaf_index == index_c.leaf_index);
    try testing.expectEqual(false, index_b.leaf_index == index_c.leaf_index);

    const query_region_2 = Ball2f{ .centre = Vec2f{ -3.5, -3.5 }, .radius = 1.0 };
    const overlap_indexes_2 = try tree.getOverlappingBodies(&index_buff, query_region_2);
    try testing.expectEqual(0, overlap_indexes_2.len);

    const query_region_3 = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 2.0 }; // outside the tree's region, but overlapping
    const overlap_3_bfs = try tree.getOverlappingBodies(&index_buff, query_region_3);
    try testing.expectEqual(3, overlap_3_bfs.len);
}

test "square tree add remove" {
    const QuadTree = SquareTree(2, 4);
    var qt = try QuadTree.init(testing.allocator, 8, Vec2f{ 0, 0 }, 1.0);
    defer qt.deinit();

    const test_bodies = [_]Ball2f{
        Ball2f{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 },
        Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 },
        Ball2f{ .centre = Vec2f{ 0.2, 0.9 }, .radius = 0.1 },
    };

    var indexes: [3]QuadTree.BodyIndex = undefined;
    for (test_bodies, 0..) |b, i| indexes[i] = try qt.addBody(b);
    try testing.expectEqual(3, qt.num_bodies);

    for (indexes, 0..) |b_index, i| {
        const get_body = qt.getBody(b_index);
        const removed_body = qt.removeBody(b_index);
        try testing.expectEqual(test_bodies[i].centre, get_body.centre);
        try testing.expectEqual(test_bodies[i].radius, get_body.radius);
        try testing.expectEqual(test_bodies[i].centre, removed_body.centre);
        try testing.expectEqual(test_bodies[i].radius, removed_body.radius);
    }
    try testing.expectEqual(0, qt.num_bodies);
}

// test "square tree get point" {
//     const QuadTree = SquareTree(4, 2);
//     var qt = try QuadTree.init(testing.allocator, 8, Vec2f{ 0, 0 }, 1.0);
//     defer qt.deinit();

//     for (0..2) |lvl| {
//         std.debug.print("\nLevel = {d}\n", .{lvl});
//         for (0..QuadTree.nodes_in_level[lvl]) |i| {
//             const pt = try qt.getNodeOrigin(@truncate(lvl), @truncate(i));
//             std.debug.print("{X}: point = {d:.3}\n", .{ i, pt });
//         }
//     }
// }
