const std = @import("std");
const core = @import("core.zig");

const Vec2f = core.Vec2f;
const Vec4f = core.Vec4f;
const Vec8f = core.Vec8f;
const Vec16f = core.Vec16f;
const Ball2f = core.Ball2f;

/// Raises a number to the power of (2 * n). Can be used at comptime.
fn pow2n(comptime base: u8, comptime n: u8) usize {
    var s: usize = 1;
    inline for (0..n) |_| s = s * base * base;
    return s;
}

fn getNodesPerLevel(comptime base: u8, comptime depth: u8) [depth]usize {
    var npl: [depth]usize = undefined;
    inline for (0..depth) |d| npl[d] = pow2n(base, 1 + @as(u32, @intCast(d)));
    return npl;
}

fn getNodesAboveLevel(comptime base: u8, comptime depth: u8) [depth]usize {
    const npl = getNodesPerLevel(base, depth);
    var nodes_above: [depth]usize = undefined;
    var sum: usize = 0;
    inline for (0..depth) |d| {
        nodes_above[d] = sum;
        sum += npl[d];
    }
    return nodes_above;
}

fn getSizePerLevel(comptime base: u8, comptime depth: u8, comptime size: f32) [depth]f32 {
    var size_per_lvl: [depth]f32 = undefined;
    var last_size = size;
    inline for (0..depth) |d| {
        size_per_lvl[d] = last_size / base;
        last_size = last_size / base;
    }
    return size_per_lvl;
}

fn getScalePerLevel(comptime base: u8, comptime depth: u8, comptime size: f32) [depth]f32 {
    const size_per_lvl = getSizePerLevel(base, depth, size);
    var scale_per_lvl: [depth]f32 = undefined;
    inline for (0..depth) |i| scale_per_lvl[i] = 1.0 / size_per_lvl[i];
    return scale_per_lvl;
}

fn getReversedRange(comptime len: u8) [len]u8 {
    var range: [len]u8 = undefined;
    inline for (0..len) |i| range[i] = len - i - 1;
    return range;
}

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
        else => unreachable,
    }
}

// TODO: there still seems to be a bug in getNodePoint - get this working properly!
// TODO: check if having grid-size as a comptime parameter actually improves performance
/// A data structure that covers a square region (size * size) of 2D Euclidean space.
pub fn SquareTree(comptime base_number: u8, comptime depth: u8, comptime square_size: f32) type {
    return struct {
        allocator: std.mem.Allocator,
        origin: Vec2f,
        bodies: [num_leaves]std.ArrayList(Ball2f) = undefined,
        bounds: [depth][]Ball2f = undefined,

        pub const NodeIndex = getMinimialIntegerForRange(num_leaves);
        pub const BodyIndex = struct {
            leaf_index: NodeIndex,
            body_number: u8, // TODO: make this a comptime parameter
        };
        pub const nodes_in_level = getNodesPerLevel(base_number, depth); // the number of nodes in each level
        pub const nodes_above_level = getNodesAboveLevel(base_number, depth); // cumulative number of nodes above each level
        pub const num_leaves = nodes_in_level[depth - 1]; // number of nodes in the last (leaf) level
        pub const num_parents = nodes_above_level[depth - 1]; // the number of non-leaf nodes
        pub const num_nodes = num_parents + num_leaves; // total number of nodes
        pub const reverse_levels = getReversedRange(depth);
        const base: NodeIndex = @intCast(base_number);
        const base_squared: NodeIndex = base * base;
        const size_per_level = getSizePerLevel(base, depth, square_size); // size of each square per level
        const scale_per_level = getScalePerLevel(base, depth, square_size); // reciprocal of the above
        const Self = @This();

        pub fn init(allocator: std.mem.Allocator, leaf_capacity: u32, origin: Vec2f) !Self {
            var leaf_lists: [num_leaves]std.ArrayList(Ball2f) = undefined;
            for (0..num_leaves) |i| {
                leaf_lists[i] = try std.ArrayList(Ball2f).initCapacity(allocator, leaf_capacity);
            }
            var bballs: [depth][]Ball2f = undefined;
            for (0..depth) |lvl| {
                bballs[lvl] = try allocator.alloc(Ball2f, nodes_in_level[lvl]);
            }
            return Self{
                .allocator = allocator,
                .bodies = leaf_lists,
                .bounds = bballs,
                .origin = origin,
            };
        }

        pub fn deinit(self: *Self) void {
            for (0..num_leaves) |i| self.bodies[i].deinit();
            for (0..depth) |lvl| self.allocator.free(self.bounds[lvl]);
        }

        /// Prints some useful information about this specific variety of square tree.
        pub fn printTypeInfo() void {
            std.debug.print("■ SquareTree({}, {}, {}) info:\n", .{ base, depth, square_size });
            std.debug.print("   Index type: {} ({} bytes)\n", .{ @typeInfo(NodeIndex), @sizeOf(NodeIndex) });
            std.debug.print("   Number leaf nodes: {d}\n", .{num_leaves});
            std.debug.print("   Total nodes: {d}\n", .{num_nodes});
            std.debug.print("   Nodes per level: {any}\n", .{nodes_in_level});
            std.debug.print("   Nodes above level: {any}\n", .{nodes_above_level});
            std.debug.print("   Size per level: {any}\n", .{size_per_level});
            std.debug.print("   Size: {d} kB\n", .{@sizeOf(Self) / 1000});
        }

        /// Converts a leaf index to the of its successor at the specified level
        pub fn getPredeccessorIndex(level: u8, leaf_index: NodeIndex) NodeIndex {
            const lvl_diff = depth - level - 1;
            return leaf_index >> @truncate(lvl_diff * base);
        }

        /// Gets the indexes of the immediate children of the identified node
        pub fn getChildIndexes(level: u8, level_index: NodeIndex) [base * base]NodeIndex {
            std.debug.assert(level < depth - 1);
            var child_indexes: [base * base]NodeIndex = undefined;
            for (0..base_squared) |i| {
                child_indexes[i] = (level_index << base) + @as(NodeIndex, @intCast(i));
            }
            return child_indexes;
        }

        /// Returns true if the specified point is within the region indexed by this tree, false otherwise.
        pub fn isPointWithinRegion(self: Self, point: Vec2f) bool {
            const d = point - self.origin;
            return 0 <= d[0] and d[0] < square_size and 0 <= d[1] and d[1] < square_size;
        }

        /// Gets the index of the leaf node the query point lies within.
        /// Asserts the point is within the region covered by this tree.
        pub fn getLeafIndexForPoint(self: Self, point: Vec2f) NodeIndex {
            const d = point - self.origin;
            var index: NodeIndex = 0;
            for (0..depth) |lvl| {
                const row = @as(NodeIndex, @intFromFloat(scale_per_level[lvl] * d[1])) % base;
                const col = @as(NodeIndex, @intFromFloat(scale_per_level[lvl] * d[0])) % base;
                index = (index << base) + row * base + col; // TODO: this is not correct for base = 8 (should bitshift x 6)
            }
            return index;
        }

        /// Gets the position of a point offset from the identified node's origin
        pub fn getNodePoint(self: Self, level: u8, index: NodeIndex, x_offset: f32, y_offset: f32) Vec2f {
            var point = self.origin;
            if (level == 0) {
                const row = index / base;
                const col = index % base;
                return Vec2f{
                    size_per_level[0] * (x_offset + core.asf32(col)),
                    size_per_level[0] * (y_offset + core.asf32(row)),
                };
            }
            for (0..level + 1) |lvl| {
                // NOTE: This is probably quite slow in practice due to integer divisions.
                const lvl_index = getPredeccessorIndex(@intCast(lvl), index);
                const lvl_size = size_per_level[lvl];
                const row = (lvl_index / base) % base;
                const col = lvl_index % base;
                const x_lvl = if (lvl < level) core.asf32(col) else x_offset + core.asf32(col);
                const y_lvl = if (lvl < level) core.asf32(row) else y_offset + core.asf32(row);
                point += Vec2f{ lvl_size * x_lvl, lvl_size * y_lvl };
            }
            return point;
        }

        /// Gets the origin point for the identified node.
        pub fn getNodeOrigin(self: Self, level: u8, index: NodeIndex) Vec2f {
            return self.getNodePoint(level, index, 0.0, 0.0);
        }

        /// Gets the centre point of the identified node.
        pub fn getNodeCentre(self: Self, level: u8, index: NodeIndex) Vec2f {
            return self.getNodePoint(level, index, 0.5, 0.5);
        }

        /// Gets the corner opposite the origin for the identified node.
        pub fn getNodeCorner(self: Self, level: u8, index: NodeIndex) Vec2f {
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
                } else { // higher-level nodes
                    for (0..self.bounds[lvl].len) |i| {
                        var ball = Ball2f{ .centre = Vec2f{ 0, 0 }, .radius = 0 }; // start with an empty ball
                        const child_indexes = getChildIndexes(lvl, @intCast(i));
                        for (child_indexes) |j| {
                            ball = core.getEncompassingBall(ball, self.bounds[lvl + 1][j]);
                        }
                        self.bounds[lvl][i] = ball;
                    }
                }
            }
        }

        fn getOverlappingChildren(self: Self, buff: []BodyIndex, b: Ball2f, level: u8, level_index: NodeIndex) []BodyIndex {
            var buff_index: usize = 0;
            if (level == depth - 1) { // leaf node
                for (self.bodies[level_index].items, 0..) |a, i| {
                    if (a.overlapsBall(b)) {
                        buff[buff_index] = BodyIndex{
                            .leaf_index = level_index,
                            .body_number = @intCast(i),
                        };
                        buff_index += 1;
                    }
                }
            } else { // parent node
                const child_indexes = getChildIndexes(level, level_index);
                for (child_indexes) |i| {
                    if (self.bounds[level + 1][i].overlapsBall(b)) {
                        buff_index += getOverlappingChildren(self, buff[buff_index..], b, level + 1, i).len;
                    }
                }
            }
            // TODO: consider truncating buff_index if it is too large for the buffer?
            return if (buff_index > 0) buff[0..buff_index] else &[0]BodyIndex{};
        }

        pub fn getOverlappingBodies(self: Self, buff: []BodyIndex, b: Ball2f) []BodyIndex {
            var buff_index: usize = 0;
            for (0..nodes_in_level[0]) |n| {
                const overlaps = self.getOverlappingChildren(buff[buff_index..], b, 0, @intCast(n));
                buff_index += overlaps.len;
            }
            return if (buff_index > 0) buff[0..buff_index] else &[0]BodyIndex{};
        }

        pub fn addBody(self: *Self, ball: Ball2f) !BodyIndex {
            const leaf_index = self.getLeafIndexForPoint(ball.centre);
            const body_num = self.bodies[leaf_index].items.len;
            // TODO: add the body to a neighbouring leaf if this leaf has reached capacity
            try self.bodies[leaf_index].append(ball);
            return BodyIndex{ .leaf_index = leaf_index, .body_number = @intCast(body_num) };
        }
    };
}

const testing = std.testing;
const tolerance: f32 = 0.0001;

test "square tree init" {
    const Tree2x8 = SquareTree(2, 8, 2000.0);
    var qt = Tree2x8{ .allocator = std.testing.allocator, .origin = Vec2f{ 0, 0 } };
    defer qt.deinit();

    const Tree4x4 = SquareTree(4, 4, 2000.0);
    var ht = Tree4x4{ .allocator = std.testing.allocator, .origin = Vec2f{ 0, 0 } };
    defer ht.deinit();
}

test "quad tree indexing" {
    const tree_depth = 3;
    const QuadTree4 = SquareTree(2, tree_depth, 8.0);
    var qt = try QuadTree4.init(testing.allocator, 0, Vec2f{ 0, 0 });
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
    const parent_a = QuadTree4.getPredeccessorIndex(1, index_a);
    const grandparent_a = QuadTree4.getPredeccessorIndex(0, index_a);
    try testing.expectEqual(48, index_a);
    try testing.expectEqual(12, parent_a);
    try testing.expectEqual(3, grandparent_a);

    const pt_b = Vec2f{ 7.99, 7.99 };
    const ln_num_b = qt.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(QuadTree4.num_leaves - 1, ln_num_b);
    for (0..QuadTree4.num_leaves) |n| {
        const ln_num = QuadTree4.getPredeccessorIndex(0, @intCast(n));
        try testing.expectEqual(n / 16, ln_num);
    }
}

test "hex tree indexing" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(4, tree_depth, 8.0);
    var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 });
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
    const parent_a = HexTree2.getPredeccessorIndex(0, index_a);
    try testing.expectEqual(96, index_a);
    try testing.expectEqual(6, parent_a);

    const pt_b = Vec2f{ 7.99, 7.99 };
    const index_b = tree.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(HexTree2.num_leaves - 1, index_b);

    for (0..HexTree2.num_leaves) |n| {
        const parent_index = HexTree2.getPredeccessorIndex(0, @intCast(n));
        try testing.expectEqual(n / 16, parent_index);
    }
}

test "hex tree overlap" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(4, tree_depth, 1.0);
    var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 });
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
    const overlap_indexes_1 = tree.getOverlappingBodies(&index_buff, query_region_1);
    try testing.expectEqual(0, overlap_indexes_1.len);
    try testing.expectEqual(false, index_a.leaf_index == index_b.leaf_index);
    try testing.expectEqual(false, index_a.leaf_index == index_c.leaf_index);
    try testing.expectEqual(false, index_b.leaf_index == index_c.leaf_index);

    const query_region_2 = Ball2f{ .centre = Vec2f{ -3.5, -3.5 }, .radius = 1.0 };
    const overlap_indexes_2 = tree.getOverlappingBodies(&index_buff, query_region_2);
    try testing.expectEqual(0, overlap_indexes_2.len);

    const query_region_3 = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 2.0 }; // outside the tree's region, but overlapping
    const overlap_indexes_3 = tree.getOverlappingBodies(&index_buff, query_region_3);
    try testing.expectEqual(3, overlap_indexes_3.len);
    //std.debug.print("overlap 3 = {any}.\n", .{overlap_indexes_3});
}
