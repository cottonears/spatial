const std = @import("std");
const calc = @import("calc.zig");
const Vec2f = calc.Vec2f;

/// A data structure that covers a square region (size * size) of 2D Euclidean space.
pub fn SquareTree(
    comptime base_num: u8, // Each node has base_num * base_num children.
    comptime tree_depth: u8, // The number of layers in the tree structure.
    comptime DataIndex: type, // Type used to index data stored in leaf nodes.
    comptime VolumeType: anytype, // Type of bounding volume stored in the tree.
) type {
    return struct {
        allocator: std.mem.Allocator,
        origin: Vec2f,
        size: f32,
        size_per_level: [tree_depth]f32,
        scale_per_level: [tree_depth]f32,
        bounding_volumes: [tree_depth][]VolumeType = undefined,
        leaf_data: [num_leaves]std.ArrayListUnmanaged(VolumeType) = undefined,
        num_bodies: usize = 0,

        // check arguments are supported
        comptime {
            if (!(base_num == 2 or base_num == 4 or base_num == 8 or base_num == 16)) @compileError("unsupported base_num");
            if (tree_depth < 2) @compileError("depth must be greater than 1");
            if (base_num * tree_depth > 20) @compileError("base_num * depth exceeds limit (20)"); // things get seg-faulty above this limit =(
        }

        pub const NodeIndex = std.math.IntFittingRange(0, num_leaves - 1);
        pub const BodyIndex = packed struct {
            leaf_index: NodeIndex,
            data_index: DataIndex,
        };
        const Self = @This();
        pub const base: NodeIndex = @intCast(base_num);
        pub const base_squared: NodeIndex = base * base;
        pub const depth = tree_depth;
        pub const num_nodes = @reduce(.Add, nodes_in_level);
        pub const num_leaves = nodes_in_level[tree_depth - 1];
        pub const nodes_in_level = calc.getPow2nSequence(base_num, 1, tree_depth + 1);
        pub const leaf_indexes = calc.getRange(NodeIndex, num_leaves);
        pub const reverse_levels = calc.getReversedRange(u8, tree_depth);
        const level_bitshift = std.math.log2(base_squared);

        pub fn init(allocator: std.mem.Allocator, leaf_capacity: u32, origin: Vec2f, size: f32) !Self {
            if (size <= 0.0) return error.InvalidSize;

            var vols: [depth][]VolumeType = undefined;
            var size_per_lvl: [depth]f32 = undefined;
            var scale_per_lvl: [depth]f32 = undefined;
            var last_size = size;
            for (0..depth) |lvl| {
                vols[lvl] = try allocator.alloc(VolumeType, nodes_in_level[lvl]);
                size_per_lvl[lvl] = last_size / base;
                scale_per_lvl[lvl] = 1.0 / size_per_lvl[lvl];
                last_size = last_size / base;
            }

            var leaf_arrays: [num_leaves]std.ArrayListUnmanaged(VolumeType) = undefined;
            for (leaf_indexes) |i| {
                leaf_arrays[i] = try std.ArrayListUnmanaged(VolumeType).initCapacity(allocator, leaf_capacity);
            }

            return Self{
                .allocator = allocator,
                .bounding_volumes = vols,
                .leaf_data = leaf_arrays,
                .origin = origin,
                .size = size,
                .size_per_level = size_per_lvl,
                .scale_per_level = scale_per_lvl,
            };
        }

        pub fn deinit(self: *Self) void {
            for (0..num_leaves) |i| self.leaf_data[i].deinit(self.allocator);
            for (0..depth) |lvl| self.allocator.free(self.bounding_volumes[lvl]);
        }

        /// Gets a string with some useful information about this variety of square tree.
        pub fn getTypeInfo(buff: []u8) ![]u8 {
            var buff_len: usize = 0;
            buff_len += (try std.fmt.bufPrint(
                buff,
                "■ SquareTree({}, {}, {}, {}) info:\n  Body Index = {} + {} ({} bytes)\n",
                .{ base, depth, DataIndex, VolumeType, NodeIndex, DataIndex, @sizeOf(BodyIndex) },
            )).len;
            buff_len += (try std.fmt.bufPrint(
                buff[buff_len..],
                "  Nodes per level: {any}\n  Struct size: {d} kB\n",
                .{ nodes_in_level, @sizeOf(Self) / 1000 },
            )).len;
            return buff[0..buff_len];
        }

        /// Gets the position of a point offset from the identified node's origin.
        pub fn getNodePoint(self: Self, level: u8, index: NodeIndex, x_offset: f32, y_offset: f32) !Vec2f {
            var point = self.origin;
            for (0..level + 1) |lvl| {
                const lvl_index = getPredeccessorIndex(index, @intCast(level - lvl));
                const lvl_size = self.size_per_level[lvl];
                const row: f32 = @floatFromInt((lvl_index / base) % base);
                const col: f32 = @floatFromInt(lvl_index % base);
                point += Vec2f{ lvl_size * col, lvl_size * row };
            }
            const offset = calc.scaledVec(self.size_per_level[level], Vec2f{ x_offset, y_offset });
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

        /// Returns true if the specified point is within the region indexed by this tree, false otherwise.
        pub fn isPointWithinRegion(self: Self, point: Vec2f) bool {
            const d = point - self.origin;
            return 0 <= d[0] and d[0] < self.size and 0 <= d[1] and d[1] < self.size;
        }

        /// Gets the index of the leaf node the query point lies within.
        /// Assumes the point is within the region covered by this tree.
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

        /// Adds a body to the tree and returns its index.
        pub fn addBody(self: *Self, ball: VolumeType) !BodyIndex {
            const leaf = self.getLeafIndexForPoint(ball.centre);
            const array_index: DataIndex = @intCast(self.leaf_data[leaf].items.len);
            if (array_index == std.math.maxInt(DataIndex)) {
                return error.DataIndexRangeExceeded;
            }
            try self.leaf_data[leaf].append(self.allocator, ball);
            self.num_bodies += 1;
            return BodyIndex{ .leaf_index = leaf, .data_index = array_index };
        }

        /// Gets the body at the specified index.
        pub fn getBody(self: Self, index: BodyIndex) ?VolumeType {
            const ld_slice = self.leaf_data[index.leaf_index].items;
            return if (ld_slice.len > index.data_index) ld_slice[index.data_index] else null;
        }

        /// Removes all bodies from the tree.
        /// Does not update bounding volumes.
        pub fn clearAllBodies(self: *Self) void {
            for (0..num_leaves) |i| self.leaf_data[i].shrinkRetainingCapacity(0);
            self.num_bodies = 0;
        }

        /// Updates all bounding volumes within the tree.
        pub fn updateBounds(self: *Self) void {
            for (reverse_levels) |lvl| {
                if (lvl == depth - 1) { // leaf nodes
                    for (0..num_leaves) |i| {
                        var ball = VolumeType{ .centre = Vec2f{ 0, 0 } }; // start with an empty volume
                        for (0..self.leaf_data[i].items.len) |j| {
                            ball = VolumeType.getEncompassing(ball, self.leaf_data[i].items[j]);
                        }
                        self.bounding_volumes[lvl][i] = ball;
                    }
                } else { // parent nodes
                    var child_index_buff: [base_squared]NodeIndex = undefined;
                    for (0..nodes_in_level[lvl]) |i| {
                        var ball = VolumeType{ .centre = Vec2f{ 0, 0 } }; // start with an empty volume
                        const child_indexes = getChildIndexes(&child_index_buff, @intCast(i));
                        for (child_indexes) |j| {
                            ball = VolumeType.getEncompassing(ball, self.bounding_volumes[lvl + 1][j]);
                        }
                        self.bounding_volumes[lvl][i] = ball;
                    }
                }
            }
        }

        /// Returns indexes of all stored bodies that lie within the query volume.
        pub fn findOverlaps(self: Self, buff: []BodyIndex, query_vol: VolumeType) []BodyIndex {
            var buff_1: [num_leaves]NodeIndex = undefined;
            var buff_2: [num_leaves]NodeIndex = undefined;
            var search_slice: []NodeIndex = getChildIndexes(&buff_1, 0);
            var next_slice: []NodeIndex = buff_2[0..];
            // iterate through all parent nodes
            for (0..depth - 1) |lvl| {
                var next_offset: usize = 0;
                for (search_slice) |i| {
                    if (query_vol.overlapsOther(self.bounding_volumes[lvl][i])) {
                        next_offset += getChildIndexes(next_slice[next_offset..], i).len;
                    }
                }
                const swap_slice = next_slice;
                next_slice.ptr = search_slice.ptr;
                search_slice = swap_slice[0..next_offset];
            }
            // iterate through leaf nodes
            var buff_len: usize = 0;
            for (search_slice) |i| {
                for (0.., self.leaf_data[i].items) |j, b| {
                    if (query_vol.overlapsOther(b)) {
                        buff[buff_len] = BodyIndex{ .leaf_index = i, .data_index = @intCast(j) };
                        buff_len += 1;
                    }
                }
            }
            // TODO: find out if there is a cleaner way of doing the below?
            return if (buff_len > 0) buff[0..buff_len] else &[0]BodyIndex{};
        }

        /// Returns indexes of stored bodies within the query volume with index > prior_index.
        pub fn findOverlapsAfterIndex(
            self: Self,
            buff: []BodyIndex,
            query_vol: VolumeType,
            prior_index: BodyIndex,
        ) []BodyIndex {
            var buff_1: [num_leaves]NodeIndex = undefined;
            var buff_2: [num_leaves]NodeIndex = undefined;
            var search_slice: []NodeIndex = getChildIndexes(&buff_1, 0);
            var next_slice: []NodeIndex = buff_2[0..];
            const pl_index = prior_index.leaf_index;
            // iterate through all parent nodes
            for (0..depth - 1) |lvl| {
                const lvl_diff: u8 = @intCast(depth - 1 - lvl);
                const lvl_start = getPredeccessorIndex(pl_index, lvl_diff);
                var next_offset: usize = 0;
                for (search_slice) |i| {
                    if (lvl_start <= i and query_vol.overlapsOther(self.bounding_volumes[lvl][i])) {
                        next_offset += getChildIndexes(next_slice[next_offset..], i).len;
                    }
                }
                const swap_slice = next_slice;
                next_slice.ptr = search_slice.ptr;
                search_slice = swap_slice[0..next_offset];
            }
            // iterate through leaf nodes
            var buff_len: usize = 0;
            for (search_slice) |i| {
                // NOTE: The below code is ugly, but it performs reasonably well.
                //       In testing it ran about 10% faster than the 'cleaner' version it replaced.
                //       The cleaner version was much closer to the same loop in findOverlaps.
                //       It might be possible to further improve performance.
                const ld_slice = self.leaf_data[i].items;
                if (i < pl_index or ld_slice.len == 0) continue;
                if (i > pl_index) {
                    for (0..ld_slice.len) |j| {
                        if (query_vol.overlapsOther(ld_slice[j])) {
                            buff[buff_len] = BodyIndex{ .leaf_index = i, .data_index = @intCast(j) };
                            buff_len += 1;
                        }
                    }
                } else { // TODO: check if these can be combined?
                    if (prior_index.data_index == ld_slice.len - 1) continue;
                    for (prior_index.data_index + 1..ld_slice.len) |j| {
                        if (query_vol.overlapsOther(ld_slice[j])) {
                            buff[buff_len] = BodyIndex{ .leaf_index = i, .data_index = @intCast(j) };
                            buff_len += 1;
                        }
                    }
                }
            }
            return if (buff_len > 0) buff[0..buff_len] else &[0]BodyIndex{};
        }

        /// Gets the index of the identified node's predecessor (0 or more levels higher).
        fn getPredeccessorIndex(index: NodeIndex, lvl_diff: u8) NodeIndex {
            return index >> @truncate(lvl_diff * level_bitshift);
        }

        /// Gets the index of a successor of the identified node (0 or more levels lower).
        /// The index returned is for the 'firstsuccessor' at the specified level (i.e., the lowest index for a successor at that level).
        fn getSuccessorIndex(index: NodeIndex, lvl_diff: u8) NodeIndex {
            return index << @truncate(lvl_diff * level_bitshift);
        }

        /// Gets the indexes of all immediate children of the specified parent node.
        fn getChildIndexes(buff: []NodeIndex, parent_index: NodeIndex) []NodeIndex {
            const first_child_index = parent_index << level_bitshift;
            for (0..base_squared) |i| {
                buff[i] = first_child_index + @as(NodeIndex, @intCast(i));
            }
            return buff[0..base_squared];
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
const volume = @import("volume.zig");
const Ball2f = volume.Ball2f;
const Box2f = volume.Box2f;
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
    const QuadTree3 = SquareTree(2, 3, u10, Ball2f);
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
    for (QuadTree3.leaf_indexes) |n| {
        const pred_0 = QuadTree3.getPredeccessorIndex(n, 0);
        const pred_1 = QuadTree3.getPredeccessorIndex(n, 1);
        const pred_2 = QuadTree3.getPredeccessorIndex(n, 2);
        try testing.expectEqual(n, pred_0);
        try testing.expectEqual(n / 16, pred_2);
        try testing.expectEqual(n / 4, pred_1);
    }
}

test "hex tree indexing" {
    const HexTree2 = SquareTree(4, 2, u8, Ball2f);
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

    for (HexTree2.leaf_indexes) |n| {
        const parent_index = HexTree2.getPredeccessorIndex(n, 1);
        const parent_child0_index = HexTree2.getSuccessorIndex(parent_index, 1);
        try testing.expectEqual(n / 16, parent_index);
        try testing.expect(@abs(n - parent_child0_index) < 16);
    }
}

test "hex tree overlap ball" {
    const HexTree2 = SquareTree(4, 2, u8, Ball2f);
    var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 }, 1.0);
    defer tree.deinit();

    const a = Ball2f{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 };
    const b = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 };
    const c = Ball2f{ .centre = Vec2f{ 0.2, 0.7 }, .radius = 0.1 };
    const index_a = try tree.addBody(a);
    const index_b = try tree.addBody(b);
    const index_c = try tree.addBody(c);
    tree.updateBounds();

    var index_buff: [8]HexTree2.BodyIndex = undefined;
    const query_region_1 = Ball2f{ .centre = Vec2f{ 0.9, 0.5 }, .radius = 0.1 };
    const overlap_indexes_1 = tree.findOverlaps(&index_buff, query_region_1);
    try testing.expectEqual(0, overlap_indexes_1.len);
    try testing.expectEqual(false, index_a.leaf_index == index_b.leaf_index);
    try testing.expectEqual(false, index_a.leaf_index == index_c.leaf_index);
    try testing.expectEqual(false, index_b.leaf_index == index_c.leaf_index);

    const query_region_2 = Ball2f{ .centre = Vec2f{ -3.5, -3.5 }, .radius = 1.0 };
    const overlap_indexes_2 = tree.findOverlaps(&index_buff, query_region_2);
    try testing.expectEqual(0, overlap_indexes_2.len);

    const query_region_3 = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 2.0 }; // outside the tree's region, but overlapping
    const overlap_indexes_3a = tree.findOverlaps(&index_buff, query_region_3);
    const overlap_indexes_3b = tree.findOverlapsAfterIndex(&index_buff, query_region_3, index_a);
    try testing.expectEqual(3, overlap_indexes_3a.len);
    try testing.expectEqual(2, overlap_indexes_3b.len);
}

test "hex tree overlap box" {
    const HexTree2 = SquareTree(4, 2, u8, Box2f);
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
    const overlap_indexes_1 = tree.findOverlaps(&index_buff, query_region_1);
    try testing.expectEqual(0, overlap_indexes_1.len);
    try testing.expectEqual(false, index_a.leaf_index == index_b.leaf_index);
    try testing.expectEqual(false, index_a.leaf_index == index_c.leaf_index);
    try testing.expectEqual(false, index_b.leaf_index == index_c.leaf_index);

    const query_region_2 = Box2f{ .centre = Vec2f{ -3.5, -3.5 }, .half_width = 1.0, .half_height = 1.0 };
    const overlap_indexes_2 = tree.findOverlaps(&index_buff, query_region_2);
    try testing.expectEqual(0, overlap_indexes_2.len);

    const query_region_3 = Box2f{ .centre = Vec2f{ 0.2, 0.5 }, .half_width = 1.0, .half_height = 1.0 };
    const overlap_3a = tree.findOverlaps(&index_buff, query_region_3);
    const overlap_3b = tree.findOverlapsAfterIndex(&index_buff, query_region_3, index_a);
    try testing.expectEqual(3, overlap_3a.len);
    try testing.expectEqual(2, overlap_3b.len);
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
    for (0.., test_bodies) |i, b| indexes[i] = try qt.addBody(b);
    try testing.expectEqual(3, qt.num_bodies);

    for (0.., indexes) |i, b_index| {
        const get_body = qt.getBody(b_index) orelse unreachable;
        try testing.expectEqual(test_bodies[i].centre, get_body.centre);
        try testing.expectEqual(test_bodies[i].radius, get_body.radius);
    }

    qt.clearAllBodies();
    try testing.expectEqual(0, qt.num_bodies);
}
