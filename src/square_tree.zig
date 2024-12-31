const std = @import("std");
const core = @import("core.zig");

const Vec2f = core.Vec2f;
const Vec4f = core.Vec4f;
const Vec8f = core.Vec8f;
const Vec16f = core.Vec16f;
const Ball2f = core.Ball2f;

/// Raises a number to the power of (2 * n). Can be used at comptime.
fn pow2n(comptime base: u5, comptime n: u8) u32 {
    var s: u32 = 1;
    inline for (0..n) |_| s = s * base * base;
    return s;
}

fn getNodesPerLevel(comptime base: u5, comptime depth: u8) [depth]u32 {
    var npl: [depth]u32 = undefined;
    inline for (0..depth) |d| npl[d] = pow2n(base, 1 + @as(u32, @intCast(d)));
    return npl;
}

fn getNodesAboveLevel(comptime base: u5, comptime depth: u8) [depth]u32 {
    const npl = getNodesPerLevel(base, depth);
    var nodes_above: [depth]u32 = undefined;
    var sum: u32 = 0;
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
// IDEA: Should we add a custom type for node-number as well? This would allow for more efficient memory (+cache) usage.
// IDEA: the first implementation updates bounds as soon as things are added, might be good to add a lazy option in future.
// TODO: check if having grid-size as a comptime parameter actually improves performance
/// A data structure that covers a square region (size * size) of 2D Euclidean space.
pub fn SquareTree(comptime base_type: type, comptime depth: u8, comptime square_size: f32) type {
    const base: u5 = switch (base_type) {
        u2 => 2, // 4 children per node
        u4 => 4, // 16 children per node
        u8 => 8, // 64 children per node
        u16 => 16, // 256 children per node
        else => unreachable,
    };

    return struct {
        allocator: std.mem.Allocator,
        origin: Vec2f,
        leaf_lists: [num_leaves]std.ArrayList(Ball2f) = undefined,
        node_bballs: [num_nodes]Ball2f = undefined,
        node_empty: [num_nodes]bool = [_]bool{true} ** num_nodes,

        pub const nodes_in_level = getNodesPerLevel(base, depth); // the number of nodes in each level
        pub const nodes_above_level = getNodesAboveLevel(base, depth); // cumulative number of nodes above each level
        pub const num_leaves = nodes_in_level[depth - 1]; // number of nodes in the last (leaf) level
        pub const num_nodes = nodes_above_level[depth - 1] + num_leaves; // total number of nodes
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
            return Self{
                .allocator = allocator,
                .leaf_lists = leaf_lists,
                .origin = origin,
            };
        }

        pub fn deinit(self: *Self) void {
            for (0..num_leaves) |i| self.leaf_lists[i].deinit();
        }

        /// Prints some useful information about this specific variety of square tree.
        pub fn printTypeInfo() void {
            std.debug.print("■ SquareTree({}, {}, {}) info:\n", .{ base_type, depth, square_size });
            std.debug.print("   Nodes per level: {any}\n", .{nodes_in_level});
            std.debug.print("   Nodes above level: {any}\n", .{nodes_above_level});
            std.debug.print("   Size per level: {any}\n", .{size_per_level});
            std.debug.print("   Scale per level: {any}\n", .{scale_per_level});
            std.debug.print("   Number leaf nodes: {d}\n", .{num_leaves});
            std.debug.print("   Total number nodes: {d}\n", .{num_nodes});
            std.debug.print("   Size: {d} kB\n", .{@sizeOf(Self) / 1000});
            std.debug.print("   Path index type: {} ({} bytes)\n", .{ @typeInfo([depth]base_type), @sizeOf([depth]base_type) });
        }

        /// Returns true if the specified point is within the region indexed by this tree, false otherwise.
        pub fn isPointWithinRegion(self: Self, point: Vec2f) bool {
            const d = point - self.origin;
            return 0 <= d[0] and d[0] < square_size and 0 <= d[1] and d[1] < square_size;
        }

        /// Returns the 'node number' for the leaf node at the end of the specified path.
        /// This is an integer in the range 0..num_nodes that uniquely identifies a node.
        pub fn getLeafNumber(path_array: [depth]base_type) u32 {
            var sum: u32 = 0;
            for (0..depth) |lvl| sum = (sum << base) + path_array[lvl];
            return nodes_above_level[depth - 1] + sum;
        }

        /// Returns the 'node number' for the node at the end of the specified path.
        /// This is an integer in the range 0..num_nodes that uniquely identifies a node.
        pub fn getNodeNumber(path_slice: []const base_type) u32 {
            std.debug.assert(0 < path_slice.len and path_slice.len <= depth);
            var sum: u32 = 0;
            for (path_slice) |i| sum = (sum << base) + i;
            return nodes_above_level[path_slice.len - 1] + sum;
        }

        /// Gets the path to the leaf node at the specified index.
        pub fn getPathToLeafNode(node_number: u32) [depth]base_type {
            std.debug.assert(nodes_above_level[depth - 1] <= node_number and node_number < num_nodes);
            const leaf_index = node_number - nodes_above_level[depth - 1];
            var path: [depth]base_type = undefined;
            for (0..depth) |lvl| {
                const lvl_diff: u5 = @intCast(depth - 1 - lvl);
                path[lvl] = @truncate(leaf_index >> (lvl_diff * base));
            }
            return path;
        }

        /// Gets the path to the identified node.
        pub fn getPathToNode(buff: []base_type, node_number: u32) []base_type {
            std.debug.assert(node_number < num_nodes);
            const node_level = Self.getNodeLevel(node_number);
            const level_index = node_number - nodes_above_level[node_level];
            for (0..(node_level + 1)) |lvl| {
                const lvl_diff: u5 = @intCast(node_level - lvl);
                buff[lvl] = @truncate(level_index >> (lvl_diff * base));
            }
            return buff[0..(node_level + 1)];
        }

        /// Gets the path to the leaf node the query point lies within.
        /// Asserts the point is within the region covered by this tree.
        pub fn getPathForPoint(self: Self, point: Vec2f) [depth]base_type {
            std.debug.assert(self.isPointWithinRegion(point));
            var d_remaining = point - self.origin;
            var path: [depth]base_type = undefined;
            for (0..depth) |lvl| {
                const row: u16 = @intFromFloat(scale_per_level[lvl] * d_remaining[1]);
                const col: u16 = @intFromFloat(scale_per_level[lvl] * d_remaining[0]);
                path[lvl] = @truncate(row * base + col);
                d_remaining -= Vec2f{
                    size_per_level[lvl] * core.asf32(col),
                    size_per_level[lvl] * core.asf32(row),
                };
                // std.debug.print(
                //     "{}. d_lvl = {d:.2}, lvl_scale = {d:.3}; row = {}, col = {}\n",
                //     .{ lvl, d_remaining, scale_per_level[lvl], row, col },
                // );
            }
            return path;
        }

        /// Returns the level for the specified node number
        pub fn getNodeLevel(node_number: u32) u8 {
            for (0..(depth - 1)) |lvl| {
                if (node_number < nodes_above_level[lvl + 1]) {
                    return @intCast(lvl);
                }
            }
            return depth - 1;
        }

        /// Gets the position of a point offset from the identified node's origin
        pub fn getNodePoint(self: Self, path_slice: []const base_type, x_offset: f32, y_offset: f32) Vec2f {
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
        pub fn getNodeOrigin(self: Self, path_slice: []const base_type) Vec2f {
            return self.getNodePoint(path_slice, 0.0, 0.0);
        }

        /// Gets the centre point of the identified node.
        pub fn getNodeCentre(self: Self, path_slice: []const base_type) Vec2f {
            return self.getNodePoint(path_slice, 0.5, 0.5);
        }

        /// Gets the corner opposite the origin for the identified node.
        pub fn getNodeCorner(self: Self, path_slice: []const base_type) Vec2f {
            return self.getNodePoint(path_slice, 1.0, 1.0);
        }

        /// Gets the node numbers for children of the specified node
        pub fn getChildNodeNumbers(node_number: u32) [base * base]u32 {
            std.debug.assert(node_number < num_parents);
            const parent_lvl = getNodeLevel(node_number);
            const lvl_index = node_number - nodes_above_level[parent_lvl];
            const child_start_num = nodes_above_level[parent_lvl + 1] + (lvl_index * base * base);
            var child_numbers: [base * base]u32 = undefined;
            for (0..base * base) |i| child_numbers[i] = child_start_num + @as(u32, @intCast(i));
            return child_numbers;
        }

        /// Updates the bounding volumes within the tree.
        pub fn updateBounds(self: *Self) void {
            // first fit bounding volumes around the contents of each leaf nodes' list.
            for (0..num_leaves) |i| {
                if (self.leaf_lists[i].items.len == 0) {
                    self.node_empty[num_parents + i] = true;
                    continue;
                }
                var bball = self.leaf_lists[i].items[0];
                for (1..self.leaf_lists[i].items.len) |j| {
                    bball = core.getEncompassingBall(bball, self.leaf_lists[i].items[j]);
                }
                self.node_bballs[num_parents + i] = bball;
                self.node_empty[num_parents + i] = false;
            }
            // then iterate through the higher levels of the tree and update their bounding volumes.
            var lvl_index: i16 = depth - 2;
            while (lvl_index >= 0) {
                const lvl: u8 = @intCast(lvl_index);
                const start_num = nodes_above_level[lvl];
                const end_num = start_num + nodes_in_level[lvl];
                for (start_num..end_num) |p| {
                    self.node_empty[p] = true;
                    const child_nums = getChildNodeNumbers(@intCast(p));
                    for (child_nums) |c| {
                        if (self.node_empty[c]) continue;
                        if (self.node_empty[p]) {
                            self.node_bballs[p] = self.node_bballs[c];
                            self.node_empty[p] = false;
                        } else {
                            self.node_bballs[p] = core.getEncompassingBall(self.node_bballs[p], self.node_bballs[c]);
                        }
                    }
                }
                lvl_index -= 1;
            }
        }

        // TODO: unfortunately, we can't actually return information that will help retrieve the specific ball here
        //      What we really want to do is have a way of keeping track of where the ball is in the structure (e.g., leaf index)
        //      And also have a way of easily checking potential collisions (without double counting).
        //      There must be some way of doing this?
        //      For now we are just going to return the indexes of all leaves that overlap with the ball
        fn getOverlappingChildren(self: Self, buff: []u32, b: Ball2f, node_number: u32) []u32 {
            var buff_index: u32 = 0;
            if (node_number < num_parents) { // i.e., it is a 'real parent node'
                const children = getChildNodeNumbers(node_number);
                for (children) |c| {
                    if (self.node_empty[c]) continue;
                    if (self.node_bballs[c].overlapsBall(b)) {
                        const child_overlaps = self.getOverlappingChildren(buff[buff_index..], b, @intCast(c));
                        buff_index += @intCast(child_overlaps.len);
                    }
                }
            } else { // leaf node
                const leaf_index = node_number - num_parents;
                for (self.leaf_lists[leaf_index].items) |leaf_ball| {
                    if (b.overlapsBall(leaf_ball)) {
                        buff[buff_index] = leaf_index; // TODO: figure out what to do about this garbageville situation
                        buff_index += 1;
                    }
                }
            }
            // TODO: consider truncatin buff_index if it is too large for the buffer?
            return if (buff_index > 0) buff[0..buff_index] else &[0]u32{};
        }

        pub fn getOverlappingIndexes(self: Self, buff: []u32, b: Ball2f) []u32 {
            var buff_index: u32 = 0;
            for (0..nodes_in_level[0]) |n| {
                if (self.node_empty[n]) continue;
                if (self.node_bballs[n].overlapsBall(b)) {
                    const child_overlaps = self.getOverlappingChildren(buff[buff_index..], b, @intCast(n));
                    buff_index += @intCast(child_overlaps.len);
                }
            }
            return if (buff_index > 0) buff[0..buff_index] else &[0]u32{};
        }

        pub fn addBody(self: *Self, ball: Ball2f) !u32 {
            const path = self.getPathForPoint(ball.centre);
            const leaf_num = getLeafNumber(path);
            const leaf_index = leaf_num - num_parents;
            try self.leaf_lists[leaf_index].append(ball);
            return leaf_num;
        }
    };
}

// IDEA: Rework indexing so that we just use unsigned integers to identify leaf nodes.
//       Queries for higher level nodes can be supported by storing node info in a 2D array.
//       This could make usage more convenient by eliminating slices + arrays - may also improve performance.
// IDEA: Should we add a custom type for node-number as well? This would allow for more efficient memory (+cache) usage.
// IDEA: the first implementation updates bounds as soon as things are added, might be good to add a lazy option in future.
// TODO: check if having grid-size as a comptime parameter actually improves performance
/// A data structure that covers a square region (size * size) of 2D Euclidean space.
const testing = std.testing;
const tolerance: f32 = 0.0001;

test "pow2n" {
    const b0 = pow2n(2, 0);
    const b1 = pow2n(2, 1);
    const b2 = pow2n(2, 2);
    const b3 = pow2n(2, 3);
    const b30 = pow2n(2, 15);
    try testing.expectEqual(1, b0);
    try testing.expectEqual(4, b1);
    try testing.expectEqual(16, b2);
    try testing.expectEqual(64, b3);
    try testing.expectEqual(1073741824, b30);

    const t0 = pow2n(3, 0);
    const t1 = pow2n(3, 1);
    const t2 = pow2n(3, 2);
    const t3 = pow2n(3, 3);
    try testing.expectEqual(1, t0);
    try testing.expectEqual(9, t1);
    try testing.expectEqual(81, t2);
    try testing.expectEqual(729, t3);

    const q0 = pow2n(4, 0);
    const q1 = pow2n(4, 1);
    const q2 = pow2n(4, 2);
    const q3 = pow2n(4, 3);
    try testing.expectEqual(1, q0);
    try testing.expectEqual(16, q1);
    try testing.expectEqual(256, q2);
    try testing.expectEqual(4096, q3);
}

test "square tree init" {
    const Tree2x2 = SquareTree(u2, 9, 2000.0);
    var qt: Tree2x2 = Tree2x2{ .allocator = std.testing.allocator, .origin = Vec2f{ 0, 0 } };
    defer qt.deinit();

    const Tree4x4 = SquareTree(u4, 4, 2000.0);
    var ht: Tree4x4 = Tree4x4{ .allocator = std.testing.allocator, .origin = Vec2f{ 0, 0 } };
    defer ht.deinit();
}

test "quad tree indexing" {
    const tree_depth = 3;
    const QuadTree3 = SquareTree(u2, tree_depth, 8.0);
    var qt = try QuadTree3.init(testing.allocator, 0, Vec2f{ 0, 0 });
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
    const path_to_a = qt.getPathForPoint(pt_a);
    const ln_num_a1 = QuadTree3.getLeafNumber(path_to_a);
    const ln_num_a2 = QuadTree3.getNodeNumber(path_to_a[0..]);
    const node_num_parent_a = QuadTree3.getNodeNumber(path_to_a[0..2]);
    try testing.expectEqual(68, ln_num_a1);
    try testing.expectEqual(68, ln_num_a2);
    try testing.expectEqual(16, node_num_parent_a);

    const pt_b = Vec2f{ 7.99, 7.99 };
    const path_vec_b = qt.getPathForPoint(pt_b);
    const ln_num_b = QuadTree3.getLeafNumber(path_vec_b);
    try testing.expectEqual(QuadTree3.num_nodes - 1, ln_num_b);
    const first_leaf = QuadTree3.num_nodes - QuadTree3.num_leaves;
    for (first_leaf..QuadTree3.num_nodes) |n| {
        const path: @Vector(tree_depth, u2) = QuadTree3.getPathToLeafNode(@intCast(n));
        const ln_num = QuadTree3.getLeafNumber(path);
        try testing.expectEqual(n, ln_num);
    }
}

test "hex tree indexing" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(u4, tree_depth, 8.0);
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
    const path_to_a = tree.getPathForPoint(pt_a);
    const ln_num_a1 = HexTree2.getLeafNumber(path_to_a);
    const ln_num_a2 = HexTree2.getNodeNumber(path_to_a[0..]);
    const node_num_parent_a = HexTree2.getNodeNumber(path_to_a[0..1]);
    try testing.expectEqual(112, ln_num_a1);
    try testing.expectEqual(112, ln_num_a2);
    try testing.expectEqual(6, node_num_parent_a);

    const pt_b = Vec2f{ 7.99, 7.99 };
    const path_vec_b = tree.getPathForPoint(pt_b);
    const ln_num_b = HexTree2.getLeafNumber(path_vec_b);
    try testing.expectEqual(HexTree2.num_nodes - 1, ln_num_b);
    const first_leaf = HexTree2.num_nodes - HexTree2.num_leaves;
    for (first_leaf..HexTree2.num_nodes) |n| {
        const path: @Vector(tree_depth, u4) = HexTree2.getPathToLeafNode(@intCast(n));
        const ln_num = HexTree2.getLeafNumber(path);
        try testing.expectEqual(n, ln_num);
    }
}

test "hex bounds" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(u4, tree_depth, 1.0);
    var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 });
    defer tree.deinit();

    const a = Ball2f{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 };
    const b = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 };
    const c = Ball2f{ .centre = Vec2f{ 0.2, 0.9 }, .radius = 0.1 };
    const index_a = try tree.addBody(a);
    const index_b = try tree.addBody(b);
    const index_c = try tree.addBody(c);
    tree.updateBounds();
    std.debug.print("index_a = {}; index_b = {}; index_c = {}.\n", .{ index_a, index_b, index_c });

    for (0..HexTree2.num_nodes) |n| {
        if (tree.node_empty[n]) continue;
        std.debug.print("Node {d} bball = {any}\n", .{ n, tree.node_bballs[n] });
    }
}

test "hex tree overlap" {
    const tree_depth = 2;
    const HexTree2 = SquareTree(u4, tree_depth, 1.0);
    var tree = try HexTree2.init(testing.allocator, 4, Vec2f{ 0, 0 });
    defer tree.deinit();

    const a = Ball2f{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 };
    const b = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 };
    const c = Ball2f{ .centre = Vec2f{ 0.2, 0.9 }, .radius = 0.1 };
    const index_a = try tree.addBody(a);
    const index_b = try tree.addBody(b);
    const index_c = try tree.addBody(c);
    tree.updateBounds();
    std.debug.print("index_a = {}; index_b = {}; index_c = {}.\n", .{ index_a, index_b, index_c });

    var index_buff: [8]u32 = undefined;
    const query_region_1 = Ball2f{ .centre = Vec2f{ 0.9, 0.5 }, .radius = 0.1 };
    const overlap_indexes_1 = tree.getOverlappingIndexes(&index_buff, query_region_1);
    try testing.expectEqual(0, overlap_indexes_1.len);
    try testing.expectEqual(false, index_a == index_b);
    try testing.expectEqual(false, index_a == index_c);
    try testing.expectEqual(false, index_b == index_c);

    const query_region_2 = Ball2f{ .centre = Vec2f{ -3.5, -3.5 }, .radius = 1.0 };
    const overlap_indexes_2 = tree.getOverlappingIndexes(&index_buff, query_region_2);
    try testing.expectEqual(0, overlap_indexes_2.len);

    const query_region_3 = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 2.0 }; // outside the tree's region, but overlapping
    const overlap_indexes_3 = tree.getOverlappingIndexes(&index_buff, query_region_3);
    try testing.expectEqual(3, overlap_indexes_3.len);
    std.debug.print("overlap 3 = {any}.\n", .{overlap_indexes_3});
}

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
