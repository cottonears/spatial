const std = @import("std");
const core = @import("core.zig");

const Vec2f = core.Vec2f;
const Vec4f = core.Vec4f;
const Vec8f = core.Vec8f;
const Vec16f = core.Vec16f;
const Ball2f = core.Ball2f;

/// Raises a number to the power of (2 * n). Can be used at comptime.
fn pow2n(comptime base: u5, comptime n: u8) usize {
    var s: usize = 1;
    inline for (0..n) |_| s = s * base * base;
    return s;
}

fn getNodesPerLevel(comptime base: u5, comptime depth: u8) [depth]usize {
    var npl: [depth]usize = undefined;
    inline for (0..depth) |d| npl[d] = pow2n(base, 1 + @as(u32, @intCast(d)));
    return npl;
}

fn getNodesAboveLevel(comptime base: u5, comptime depth: u8) [depth]usize {
    const npl = getNodesPerLevel(base, depth);
    var nodes_above: [depth]usize = undefined;
    var sum: usize = 0;
    inline for (0..depth) |d| {
        nodes_above[d] = sum;
        sum += npl[d];
    }
    return nodes_above;
}

fn getSizePerLevel(comptime base: u5, comptime depth: u8, comptime size: f32) [depth]f32 {
    var size_per_lvl: [depth]f32 = undefined;
    var last_size = size;
    inline for (0..depth) |d| {
        size_per_lvl[d] = last_size / base;
        last_size = last_size / base;
    }
    return size_per_lvl;
}

fn getScalePerLevel(comptime base: u5, comptime depth: u8, comptime size: f32) [depth]f32 {
    const size_per_lvl = getSizePerLevel(base, depth, size);
    var scale_per_lvl: [depth]f32 = undefined;
    inline for (0..depth) |i| scale_per_lvl[i] = 1.0 / size_per_lvl[i];
    return scale_per_lvl;
}

fn getReversedRange(comptime len: u5) [len]u5 {
    var range: [len]u5 = undefined;
    inline for (0..len) |i| range[i] = len - i - 1;
    return range;
}

// IDEA: Rework indexing so that we just use unsigned integers to identify leaf nodes.
//       Queries for higher level nodes can be supported by storing node info in a 2D array.
//       This could make usage more convenient by eliminating slices + arrays - may also improve performance.
// IDEA: the first implementation updates bounds as soon as things are added, might be good to add a lazy option in future.
// TODO: check if having grid-size as a comptime parameter actually improves performance
/// A data structure that covers a square region (size * size) of 2D Euclidean space.
pub fn SquareTree2(comptime index_type: type, comptime depth: u5, comptime square_size: f32) type {
    const base: u5 = switch (index_type) {
        u8 => 8 / depth,
        u16 => 16 / depth,
        u24 => 24 / depth,
        u32 => 32 / depth,
        else => unreachable,
    };

    return struct {
        allocator: std.mem.Allocator,
        origin: Vec2f,
        bodies: [num_leaves]std.ArrayList(Ball2f) = undefined,
        bounds: [depth][]Ball2f = undefined,

        pub const nodes_in_level = getNodesPerLevel(base, depth); // the number of nodes in each level
        pub const nodes_above_level = getNodesAboveLevel(base, depth); // cumulative number of nodes above each level
        pub const num_leaves = nodes_in_level[depth - 1]; // number of nodes in the last (leaf) level
        pub const num_nodes = nodes_above_level[depth - 1] - num_leaves; // total number of nodes
        pub const num_parents = num_nodes - num_leaves; // the number of non-leaf nodes
        pub const reverse_levels = getReversedRange(depth);
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
            std.debug.print("■ SquareTree({}, {}, {}) info:\n", .{ index_type, depth, square_size });
            std.debug.print("   Nodes per level: {any}\n", .{nodes_in_level});
            std.debug.print("   Nodes above level: {any}\n", .{nodes_above_level});
            std.debug.print("   Size per level: {any}\n", .{size_per_level});
            std.debug.print("   Scale per level: {any}\n", .{scale_per_level});
            std.debug.print("   Number leaf nodes: {d}\n", .{num_leaves});
            std.debug.print("   Total number nodes: {d}\n", .{num_nodes});
            std.debug.print("   Size: {d} kB\n", .{@sizeOf(Self) / 1000});
            std.debug.print("   Index type: {} ({} bytes)\n", .{ @typeInfo(index_type), @sizeOf(index_type) });
        }

        /// Converts a leaf index to the 'level index' of its successor at the specified level
        pub fn getPredeccessorIndex(level: u5, leaf_index: index_type) index_type {
            const lvl_diff: u3 = @truncate(depth - level - 1);
            return @intCast(leaf_index >> (lvl_diff * @as(u3, @intCast(base))));
        }

        /// Gets the indexes of the immediate children of the identified node
        pub fn getChildIndexes(level: u5, level_index: index_type) [base * base]index_type {
            std.debug.assert(level < depth - 1);
            var child_indexes: [base * base]index_type = undefined;
            for (0..base * base) |i| child_indexes[i] = (level_index << base) + i;
            return child_indexes;
        }

        /// Returns true if the specified point is within the region indexed by this tree, false otherwise.
        pub fn isPointWithinRegion(self: Self, point: Vec2f) bool {
            const d = point - self.origin;
            return 0 <= d[0] and d[0] < square_size and 0 <= d[1] and d[1] < square_size;
        }

        /// Gets the path to the leaf node the query point lies within.
        /// Asserts the point is within the region covered by this tree.
        pub fn getLeafIndexForPoint(self: Self, point: Vec2f) index_type {
            std.debug.assert(self.isPointWithinRegion(point));
            var d_remaining = point - self.origin;
            var index: index_type = 0;
            for (0..depth) |lvl| {
                const row: index_type = @intFromFloat(scale_per_level[lvl] * d_remaining[1]);
                const col: index_type = @intFromFloat(scale_per_level[lvl] * d_remaining[0]);
                index = (index << base) + row * base + col;
                d_remaining -= Vec2f{ // TODO: see if we can get rid of this with clever truncation ?
                    size_per_level[lvl] * core.asf32(col),
                    size_per_level[lvl] * core.asf32(row),
                };
                // std.debug.print(
                //     "{}. d_lvl = {d:.2}, lvl_scale = {d:.3}; row = {}, col = {}\n",
                //     .{ lvl, d_remaining, scale_per_level[lvl], row, col },
                // );
            }
            return index;
        }

        /// Gets the position of a point offset from the identified node's origin
        pub fn getNodePoint(self: Self, path_slice: []const index_type, x_offset: f32, y_offset: f32) Vec2f {
            var d_lvl = self.origin;
            for (path_slice, 0..) |index, lvl| {
                // NOTE: This is probably quite slow in practice due to integer divisions.
                const lvl_size = size_per_level[lvl];
                const row = index / base;
                const col = index % base;
                d_lvl += Vec2f{
                    lvl_size * core.asf32(col),
                    lvl_size * core.asf32(row),
                };
            }
            d_lvl += Vec2f{
                size_per_level[path_slice.len - 1] * x_offset,
                size_per_level[path_slice.len - 1] * y_offset,
            };
            return d_lvl;
        }

        /// Gets the origin point for the identified node.
        pub fn getNodeOrigin(self: Self, path_slice: []const index_type) Vec2f {
            return self.getNodePoint(path_slice, 0.0, 0.0);
        }

        /// Gets the centre point of the identified node.
        pub fn getNodeCentre(self: Self, path_slice: []const index_type) Vec2f {
            return self.getNodePoint(path_slice, 0.5, 0.5);
        }

        /// Gets the corner opposite the origin for the identified node.
        pub fn getNodeCorner(self: Self, path_slice: []const index_type) Vec2f {
            return self.getNodePoint(path_slice, 1.0, 1.0);
        }

        /// Updates the bounding volumes within the tree.
        pub fn updateBounds(self: *Self) void {
            while (reverse_levels) |lvl| {
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
                        const child_indexes = getChildIndexes(lvl, i);
                        for (child_indexes) |j| {
                            ball = core.getEncompassingBall(ball, ball.self.bounds[lvl + 1][j]);
                        }
                        self.bounds[lvl][i] = ball;
                    }
                }
            }
        }

        // TODO: unfortunately, we can't actually return information that will help retrieve the specific ball here
        //      What we really want to do is have a way of keeping track of where the ball is in the structure (e.g., leaf index)
        //      And also have a way of easily checking potential collisions (without double counting).
        //      There must be some way of doing this?
        //      For now we are just going to return the indexes of all leaves that overlap with the ball
        fn getOverlappingChildren(self: Self, buff: []u32, b: Ball2f, level: u5, level_index: index_type) []index_type {
            var buff_index: u32 = 0;
            if (level == depth - 1) {
                for (self.bodies[level_index].items) |a| {
                    if (a.overlapsBall(b)) {
                        buff[buff_index] = level_index; // TODO: fix degeneracy here!
                        buff_index += 1;
                    }
                }
            } else {
                const child_indexes = getChildIndexes(level, level_index);
                for (child_indexes) |i| {
                    if (self.bounds[level + 1][i].overlapsBall(b)) {
                        buff_index += getOverlappingChildren(self, buff[buff_index..], b, level + 1, i).len;
                    }
                }
            }
            // TODO: consider truncating buff_index if it is too large for the buffer?
            return if (buff_index > 0) buff[0..buff_index] else &[0]u32{};
        }

        pub fn getOverlappingIndexes(self: Self, buff: []u32, b: Ball2f) []u32 {
            var buff_index: u32 = 0;
            for (0..nodes_in_level[0]) |n| {
                buff_index += self.getOverlappingChildren(buff, buff_index, b, 0, n);
            }
            return if (buff_index > 0) buff[0..buff_index] else &[0]u32{};
        }

        // pub fn addBody(self: *Self, ball: Ball2f) !u32 {
        //     const path = self.getPathForPoint(ball.centre);
        //     const leaf_num = getLeafNumber(path);
        //     const leaf_index = leaf_num - num_parents;
        //     try self.leaf_lists[leaf_index].append(ball);
        //     return leaf_num;
        // }
    };
}

const testing = std.testing;
const tolerance: f32 = 0.0001;

test "square tree init" {
    const Tree2x8 = SquareTree2(u16, 8, 2000.0);
    var qt = Tree2x8{ .allocator = std.testing.allocator, .origin = Vec2f{ 0, 0 } };
    defer qt.deinit();

    const Tree4x4 = SquareTree2(u16, 4, 2000.0);
    var ht = Tree4x4{ .allocator = std.testing.allocator, .origin = Vec2f{ 0, 0 } };
    defer ht.deinit();
}

test "quad tree indexing" {
    const tree_depth = 4;
    const QuadTree4 = SquareTree2(u8, tree_depth, 8.0);
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
    const parent_a = QuadTree4.getPredeccessorIndex(0, index_a);
    std.debug.print("Index for {d:.3}: {X}; parent = {X}\n", .{ pt_a, index_a, parent_a });

    const pt_b = Vec2f{ 7.99, 7.99 };
    const ln_num_b = qt.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(QuadTree4.num_leaves - 1, ln_num_b);
    for (0..QuadTree4.num_leaves) |n| {
        const ln_num = QuadTree4.getPredeccessorIndex(2, @intCast(n));
        try testing.expectEqual(n / 4, ln_num);
    }
}

// test "hex tree indexing" {
//     const tree_depth = 2;
//     const HexTree2 = SquareTree(u4, tree_depth, 8.0);
//     var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 });
//     defer tree.deinit();

//     const pt_check_1 = tree.isPointWithinRegion(Vec2f{ 0, 0 });
//     const pt_check_2 = tree.isPointWithinRegion(Vec2f{ 5, 7 });
//     const pt_check_3 = tree.isPointWithinRegion(Vec2f{ -1, 3 });
//     const pt_check_4 = tree.isPointWithinRegion(Vec2f{ 8.0, 8.0 });
//     try testing.expectEqual(true, pt_check_1);
//     try testing.expectEqual(true, pt_check_2);
//     try testing.expectEqual(false, pt_check_3);
//     try testing.expectEqual(false, pt_check_4);

//     const pt_a = Vec2f{ 4.1, 2.1 };
//     const path_to_a = tree.getPathForPoint(pt_a);
//     const ln_num_a1 = HexTree2.getLeafNumber(path_to_a);
//     const ln_num_a2 = HexTree2.getNodeNumber(path_to_a[0..]);
//     const node_num_parent_a = HexTree2.getNodeNumber(path_to_a[0..1]);
//     try testing.expectEqual(112, ln_num_a1);
//     try testing.expectEqual(112, ln_num_a2);
//     try testing.expectEqual(6, node_num_parent_a);

//     const pt_b = Vec2f{ 7.99, 7.99 };
//     const path_vec_b = tree.getPathForPoint(pt_b);
//     const ln_num_b = HexTree2.getLeafNumber(path_vec_b);
//     try testing.expectEqual(HexTree2.num_nodes - 1, ln_num_b);
//     const first_leaf = HexTree2.num_nodes - HexTree2.num_leaves;
//     for (first_leaf..HexTree2.num_nodes) |n| {
//         const path: @Vector(tree_depth, u4) = HexTree2.getPathToLeafNode(@intCast(n));
//         const ln_num = HexTree2.getLeafNumber(path);
//         try testing.expectEqual(n, ln_num);
//     }
// }

// test "hex bounds" {
//     const tree_depth = 2;
//     const HexTree2 = SquareTree(u4, tree_depth, 1.0);
//     var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 });
//     defer tree.deinit();

//     const a = Ball2f{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 };
//     const b = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 };
//     const c = Ball2f{ .centre = Vec2f{ 0.2, 0.9 }, .radius = 0.1 };
//     const index_a = try tree.addBody(a);
//     const index_b = try tree.addBody(b);
//     const index_c = try tree.addBody(c);
//     tree.updateBounds();
//     std.debug.print("index_a = {}; index_b = {}; index_c = {}.\n", .{ index_a, index_b, index_c });

//     for (0..HexTree2.num_nodes) |n| {
//         if (tree.node_empty[n]) continue;
//         std.debug.print("Node {d} bball = {any}\n", .{ n, tree.node_bballs[n] });
//     }
// }

// test "hex tree overlap" {
//     const tree_depth = 2;
//     const HexTree2 = SquareTree(u4, tree_depth, 1.0);
//     var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 });
//     defer tree.deinit();

//     const a = Ball2f{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 };
//     const b = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 };
//     const c = Ball2f{ .centre = Vec2f{ 0.2, 0.9 }, .radius = 0.1 };
//     const index_a = try tree.addBody(a);
//     const index_b = try tree.addBody(b);
//     const index_c = try tree.addBody(c);
//     tree.updateBounds();
//     std.debug.print("index_a = {}; index_b = {}; index_c = {}.\n", .{ index_a, index_b, index_c });

//     var index_buff: [8]u32 = undefined;
//     const query_region_1 = Ball2f{ .centre = Vec2f{ 0.9, 0.5 }, .radius = 0.1 };
//     const overlap_indexes_1 = tree.getOverlappingIndexes(&index_buff, query_region_1);
//     try testing.expectEqual(0, overlap_indexes_1.len);
//     try testing.expectEqual(false, index_a == index_b);
//     try testing.expectEqual(false, index_a == index_c);
//     try testing.expectEqual(false, index_b == index_c);

//     const query_region_2 = Ball2f{ .centre = Vec2f{ -3.5, -3.5 }, .radius = 1.0 };
//     const overlap_indexes_2 = tree.getOverlappingIndexes(&index_buff, query_region_2);
//     try testing.expectEqual(0, overlap_indexes_2.len);

//     const query_region_3 = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 2.0 }; // outside the tree's region, but overlapping
//     const overlap_indexes_3 = tree.getOverlappingIndexes(&index_buff, query_region_3);
//     try testing.expectEqual(3, overlap_indexes_3.len);
//     std.debug.print("overlap 3 = {any}.\n", .{overlap_indexes_3});
// }

// pub const LeafNode = struct {
//     node_number: u16,
//     bodies: std.ArrayList(Ball2f), // IDEA: try using a MultiArrayList here and compare performance
//     bball: Ball2f,
//     // bball_centre: Vec2f,
//     // bball_radius: f32 = 0,
//     // bbox_width: f32 = 0.0, TODO: add these back in
//     // bbox_height: f32 = 0.0,
//     bounds_require_update: bool = false,
//     const Self = @This();

//     /// Initialises the LeafNode and ensures it has the desired capacity.
//     pub fn init(allocator: std.mem.Allocator, node_number: u16, centre: Vec2f, capacity: u8) !Self {
//         const bodies = try std.ArrayList(Ball2f).initCapacity(allocator, capacity);
//         return .{
//             .bball = Ball2f{ .centre = centre },
//             .bodies = bodies,
//             .node_number = node_number,
//         };
//     }

//     /// Destroys the LeafNode and frees any allocated memory.
//     pub fn deinit(self: *Self) void {
//         self.bodies.deinit();
//     }

//     /// Returns the number of items stored in the LeafNode
//     pub fn length(self: Self) u8 {
//         return @intCast(self.bodies.items.len);
//     }

//     /// Attempts to add a body to this node and returns its tree index if successful.
//     /// Not thread safe.
//     pub fn addBody(self: *Self, b: Ball2f) !u24 {
//         // IDEA: might be worth having an option to record a log of capacity expansions
//         const i: u24 = @intCast(self.bodies.items.len);
//         std.debug.assert(i < 256);
//         try self.bodies.append(b);
//         self.bounds_require_update = true; // self.bounds_require_update or body is not enclosed by existing bounds
//         return getTreeIndex(self.node_number, @intCast(i));
//     }

//     /// Gets a copy of the body with the specified index
//     pub fn getBody(self: Self, tree_index: u24) Ball2f {
//         const body_index = getBodyIndex(tree_index);
//         std.debug.assert(body_index < self.length());
//         return self.bodies.items[body_index];
//     }

//     /// Removes the body at the specified index and returns it.
//     /// NOTE: Invalidates all body indexes for this leaf-node!
//     pub fn removeBody(self: *Self, index: u24) Ball2f {
//         const body_index = getBodyIndex(index);
//         std.debug.assert(body_index < self.length());
//         const body = self.bodies.swapRemove(body_index);
//         self.bounds_require_update = true; // or body is not within margin of the enclosed bounds
//         return body;
//     }
