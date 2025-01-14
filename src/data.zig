const std = @import("std");
const core = @import("core.zig");
const volume = @import("volume.zig");
const Vec2f = core.Vec2f;
const Ball2f = volume.Ball2f;
const Box2f = volume.Box2f;

// TODO: Fix bug where tree will not clean up empty positions (maybe remove this altogether?)
// IDEA: It might be worth inserting the root node at level 0 in the below struct.
//       This could be useful for performing top-level chceks if multiple trees are used.
/// A data structure that covers a square region (size * size) of 2D Euclidean space.
pub fn SquareTree(
    comptime base_num: u8, // Each non-leaf node will have base * base children.
    comptime depth: u8, // The depth of the tree.
    comptime ArrayIndex: type, // The type that will be used to index the arrays that store data in leaf nodes.
    comptime VolumeType: anytype, // The type of volume that will be stored in the tree.
) type {
    return struct {
        allocator: std.mem.Allocator,
        origin: Vec2f,
        size: f32,
        size_per_level: [depth]f32,
        scale_per_level: [depth]f32,
        bounds: [depth][]VolumeType = undefined,
        leaf_arrays: [num_leaves]std.ArrayListUnmanaged(VolumeType) = undefined,
        leaf_empty_positions: [num_leaves]ArrayIndex = undefined,
        num_bodies: usize = 0,

        pub const NodeIndex = std.math.IntFittingRange(0, num_leaves - 1);
        pub const BodyIndex = packed struct {
            leaf_index: NodeIndex,
            body_number: ArrayIndex,
        };
        const Self = @This();

        comptime { // check the arguments are suppored
            if (!(base_num == 2 or base_num == 4 or base_num == 8 or base_num == 16)) @compileError("base value not supported");
            if (depth < 2) @compileError("depth must be greater than 1");
        }
        pub const nodes_in_level = core.getPow2nSequence(base_num, 1, depth + 1);
        pub const num_leaves = nodes_in_level[depth - 1]; // number of nodes in the last (leaf) level
        pub const num_nodes = @reduce(.Add, nodes_in_level);
        pub const reverse_levels = core.getReversedRange(u8, depth);
        const base: NodeIndex = @intCast(base_num);
        const base_squared: NodeIndex = base * base;
        const level_bitshift = core.getMultDivShift(base_squared);

        pub fn init(allocator: std.mem.Allocator, leaf_capacity: u32, origin: Vec2f, size: f32) !Self {
            if (size <= 0.0) return error.InvalidSize;

            var bballs: [depth][]VolumeType = undefined;
            var size_per_lvl: [depth]f32 = undefined;
            var scale_per_lvl: [depth]f32 = undefined;
            var last_size = size;
            for (0..depth) |lvl| {
                bballs[lvl] = try allocator.alloc(VolumeType, nodes_in_level[lvl]);
                size_per_lvl[lvl] = last_size / base;
                scale_per_lvl[lvl] = 1.0 / size_per_lvl[lvl];
                last_size = last_size / base;
            }

            var leaf_arrays: [num_leaves]std.ArrayListUnmanaged(VolumeType) = undefined;
            const leaf_empty_positions = [_]ArrayIndex{std.math.maxInt(ArrayIndex)} ** num_leaves;
            for (0..num_leaves) |i| {
                leaf_arrays[i] = try std.ArrayListUnmanaged(VolumeType).initCapacity(allocator, leaf_capacity);
            }

            return Self{
                .allocator = allocator,
                .bounds = bballs,
                .leaf_arrays = leaf_arrays,
                .leaf_empty_positions = leaf_empty_positions,
                .origin = origin,
                .size = size,
                .size_per_level = size_per_lvl,
                .scale_per_level = scale_per_lvl,
            };
        }

        pub fn deinit(self: *Self) void {
            for (0..num_leaves) |i| self.leaf_arrays[i].deinit(self.allocator);
            for (0..depth) |lvl| self.allocator.free(self.bounds[lvl]);
        }

        /// Gets the index of the identified node's predecessor (0 or more levels higher).
        fn getPredeccessorIndex(index: NodeIndex, lvl_diff: u8) NodeIndex {
            return index >> @truncate(lvl_diff * level_bitshift);
        }

        /// Gets the index of a successor of the identified node (0 or more levels lower).
        /// The index returned is for the 'first-successor' at the specified level (i.e., the lowest index for a successor at that level).
        fn getSuccessorIndex(index: NodeIndex, lvl_diff: u8) NodeIndex {
            return index << @truncate(lvl_diff * level_bitshift);
        }

        /// Gets the indexes of all immediate children of the specified parent node.
        /// Assumes the provided parent_index does not refer to a leaf node.
        fn getChildIndexes(buff: []NodeIndex, parent_index: NodeIndex) []NodeIndex {
            std.debug.assert(buff.len >= base_squared);
            const first_child_index = getSuccessorIndex(parent_index, 1);
            for (0..base_squared) |i| {
                buff[i] = first_child_index + @as(NodeIndex, @intCast(i));
            }
            return buff[0..base_squared];
        }

        // TODO: replace the info in this function with a string literal - it should be known at comptime.
        /// Prints some useful information about this specific variety of square tree.
        pub fn printTypeInfo() void {
            std.debug.print(
                "■ SquareTree({}, {}, {}, {}) info:\n",
                .{ base, depth, ArrayIndex, VolumeType },
            );
            std.debug.print(
                "   Body Index = {} + {} ({} bytes)\n",
                .{ NodeIndex, ArrayIndex, @sizeOf(BodyIndex) },
            );
            std.debug.print("   Nodes per level: {any}\n", .{nodes_in_level});
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
        /// NOTE: This function is slow due to integer divisions: don't use in performance critical code!
        pub fn getNodePoint(self: Self, level: u8, index: NodeIndex, x_offset: f32, y_offset: f32) !Vec2f {
            var point = self.origin;
            for (0..level + 1) |lvl| {
                const lvl_index = getPredeccessorIndex(index, @intCast(level - lvl));
                const lvl_size = self.size_per_level[lvl];
                const row: f32 = @floatFromInt((lvl_index / base) % base);
                const col: f32 = @floatFromInt(lvl_index % base);
                point += Vec2f{ lvl_size * col, lvl_size * row };
            }
            const offset = core.scaledVec(self.size_per_level[level], Vec2f{ x_offset, y_offset });
            return point + offset;
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

        /// Adds a body to the tree and returns its index.
        pub fn addBody(self: *Self, ball: VolumeType) !BodyIndex {
            const i = self.getLeafIndexForPoint(ball.centre);
            var body_num: ArrayIndex = undefined;
            // if there's an empty position in the array, fi it
            if (self.leaf_empty_positions[i] < self.leaf_arrays[i].items.len) {
                body_num = self.leaf_empty_positions[i];
                self.leaf_arrays[i].items[body_num] = ball;
                // need to update the empty position for this array
                self.leaf_empty_positions[i] = std.math.maxInt(ArrayIndex);
                for (body_num..self.leaf_arrays[i].items.len) |j| {
                    if (self.leaf_arrays[i].items[j].isEmpty()) {
                        self.leaf_empty_positions[i] = @intCast(j);
                        break;
                    }
                }
            } else { // no empty positions in the array, append body at the end
                // TODO: add the body to a neighbouring leaf if this leaf's array has reached capacity?
                body_num = @intCast(self.leaf_arrays[i].items.len);
                try self.leaf_arrays[i].append(self.allocator, ball);
            }
            self.num_bodies += 1;
            return BodyIndex{ .leaf_index = i, .body_number = @intCast(body_num) };
        }

        /// Gets the body at the specified index.
        pub fn getBody(self: Self, body_index: BodyIndex) Ball2f {
            return self.leaf_arrays[body_index.leaf_index].items[body_index.body_number];
        }

        /// Removes the body at the specified index.
        /// The index may be re-used when another body is added to the same leaf node.
        pub fn deleteBody(self: *Self, k: BodyIndex) void {
            self.leaf_arrays[k.leaf_index].items[k.body_number].radius = 0;
            if (k.body_number < self.leaf_empty_positions[k.leaf_index]) {
                self.leaf_empty_positions[k.leaf_index] = k.body_number;
            }
            self.num_bodies -= 1;
        }

        /// Removes all bodies from the tree.
        pub fn clearAllBodies(self: *Self) void {
            self.leaf_empty_positions = [_]ArrayIndex{std.math.maxInt(ArrayIndex)} ** num_leaves;
            for (0..num_leaves) |i| self.leaf_arrays[i].shrinkRetainingCapacity(0);
        }

        /// Updates the bounding volumes within the tree.
        pub fn updateBounds(self: *Self) void {
            for (reverse_levels) |lvl| {
                if (lvl == depth - 1) { // leaf nodes
                    for (0..num_leaves) |i| {
                        var ball = VolumeType{ .centre = Vec2f{ 0, 0 } }; // start with an empty ball
                        for (0..self.leaf_arrays[i].items.len) |j| {
                            ball = VolumeType.getEncompassing(ball, self.leaf_arrays[i].items[j]);
                        }
                        self.bounds[lvl][i] = ball;
                    }
                } else { // parent nodes
                    var child_index_buff: [base_squared]NodeIndex = undefined;
                    for (0..self.bounds[lvl].len) |i| {
                        var ball = VolumeType{ .centre = Vec2f{ 0, 0 } }; // start with an empty ball
                        const child_indexes = getChildIndexes(&child_index_buff, @intCast(i));
                        for (child_indexes) |j| {
                            ball = VolumeType.getEncompassing(ball, self.bounds[lvl + 1][j]);
                        }
                        self.bounds[lvl][i] = ball;
                    }
                }
            }
        }

        /// Retrieves the indexes of bodies that might overlap.
        pub fn getOverlappingBodies(self: Self, buff: []BodyIndex, query_vol: VolumeType) ![]BodyIndex {
            var search_buff: [num_leaves]NodeIndex = undefined;
            for (0..base_squared) |i| search_buff[i] = @intCast(i);
            var search_len: usize = @intCast(base_squared);
            var next_buff: [num_leaves]NodeIndex = undefined;
            var next_len: usize = 0;
            // iterate through all parent nodes
            for (0..depth - 1) |lvl| {
                for (search_buff[0..search_len]) |i| {
                    if (query_vol.overlapsOther(self.bounds[lvl][i])) {
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
                for (self.leaf_arrays[leaf_index].items, 0..) |b, i| {
                    if (query_vol.overlapsOther(b)) {
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
    };
}

// IDEA: When using square-trees in practice it is probably a good idea to do something like the following:
// 1) Use a coarse tree for static objects.
// 2) Store slow - medium speed objects in a fine tree (that overlaps the static tree).
// 3) Don't bother storing very fast objects in a tree.
// Doing 1 is quite obvious, since we don't want to check if static objects overlap at runtime.
// We can check all category 2 objects for collisions with category 1 and 2 objects.
// Doing 3 is advantageous if we don't want to check collisions of category 3 objects with each other.
// Category 3 objects, and instantaneous rays, will have huge bounding volumes due to their speed.
// This could result in a large amount of potential collisions being returned for each of them.
// It will probably be more efficient to process these potential collisions to find the first actual collision,
// allowing us to ignore later potential collisions.

const testing = std.testing;
const tolerance: f32 = 0.0001;

test "square tree init" {
    const Tree2x8 = SquareTree(2, 8, u8, Ball2f);
    var qt = try Tree2x8.init(testing.allocator, 8, Vec2f{ 0, 0 }, 1.0);
    defer qt.deinit();

    const Tree4x4 = SquareTree(4, 4, u8, Ball2f);
    var ht = try Tree4x4.init(testing.allocator, 8, Vec2f{ 0, 0 }, 1.0);
    defer ht.deinit();
}

test "quad tree indexing" {
    const tree_depth = 3;
    const QuadTree3 = SquareTree(2, tree_depth, u10, Ball2f);
    var qt = try QuadTree3.init(testing.allocator, 0, Vec2f{ 0, 0 }, 8.0);
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
    const parent_a = QuadTree3.getPredeccessorIndex(index_a, 1);
    const grandparent_a = QuadTree3.getPredeccessorIndex(index_a, 2);
    try testing.expectEqual(48, index_a);
    try testing.expectEqual(12, parent_a);
    try testing.expectEqual(3, grandparent_a);

    const pt_b = Vec2f{ 7.99, 7.99 };
    const ln_num_b = qt.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(QuadTree3.num_leaves - 1, ln_num_b);
    for (0..QuadTree3.num_leaves) |n| {
        const pred_0 = QuadTree3.getPredeccessorIndex(@intCast(n), 0);
        const pred_1 = QuadTree3.getPredeccessorIndex(@intCast(n), 1);
        const pred_2 = QuadTree3.getPredeccessorIndex(@intCast(n), 2);
        try testing.expectEqual(n, pred_0);
        try testing.expectEqual(n / 16, pred_2);
        try testing.expectEqual(n / 4, pred_1);
    }
}

test "hex tree indexing" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(4, tree_depth, u8, Ball2f);
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

test "hex tree overlap ball" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(4, tree_depth, u8, Ball2f);
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

test "hex tree overlap box" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(4, tree_depth, u8, Box2f);
    var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 }, 1.0);
    defer tree.deinit();

    const a = Box2f{ .centre = Vec2f{ 0.2, 0.0 }, .half_width = 0.4, .half_height = 0.4 };
    const b = Box2f{ .centre = Vec2f{ 0.2, 0.5 }, .half_width = 0.2, .half_height = 0.2 };
    const c = Box2f{ .centre = Vec2f{ 0.2, 0.9 }, .half_width = 0.1, .half_height = 0.1 };
    const index_a = try tree.addBody(a);
    const index_b = try tree.addBody(b);
    const index_c = try tree.addBody(c);
    tree.updateBounds();

    var index_buff: [8]HexTree2.BodyIndex = undefined;
    const query_region_1 = Box2f{ .centre = Vec2f{ 0.9, 0.5 }, .half_width = 0.1, .half_height = 0.1 };
    const overlap_indexes_1 = try tree.getOverlappingBodies(&index_buff, query_region_1);
    try testing.expectEqual(0, overlap_indexes_1.len);
    try testing.expectEqual(false, index_a.leaf_index == index_b.leaf_index);
    try testing.expectEqual(false, index_a.leaf_index == index_c.leaf_index);
    try testing.expectEqual(false, index_b.leaf_index == index_c.leaf_index);

    // const query_region_2 = Ball2f{ .centre = Vec2f{ -3.5, -3.5 }, .radius = 1.0 };
    // const overlap_indexes_2 = try tree.getOverlappingBodies(&index_buff, query_region_2);
    // try testing.expectEqual(0, overlap_indexes_2.len);

    // const query_region_3 = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 2.0 }; // outside the tree's region, but overlapping
    // const overlap_3_bfs = try tree.getOverlappingBodies(&index_buff, query_region_3);
    // try testing.expectEqual(3, overlap_3_bfs.len);
}

test "square tree add remove" {
    const QuadTree = SquareTree(2, 4, u4, Ball2f);
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
        qt.deleteBody(b_index);
        try testing.expectEqual(test_bodies[i].centre, get_body.centre);
        try testing.expectEqual(test_bodies[i].radius, get_body.radius);
    }
    try testing.expectEqual(0, qt.num_bodies);
}
