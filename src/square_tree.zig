const std = @import("std");
const core = @import("core.zig");

const Vec2f = core.Vec2f;
const Vec4f = core.Vec4f;
const Vec8f = core.Vec8f;
const Vec16f = core.Vec16f;
const Ball2f = core.Ball2f;

/// Raises a number to the power of (2 * n). Can be used at comptime.
fn pow2n(comptime base: u8, comptime n: u8) u32 {
    var s: u32 = 1;
    inline for (0..n) |_| s = s * base * base;
    return s;
}

fn getNodesPerLevel(comptime base: u8, comptime depth: u8) [depth]u32 {
    var npl: [depth]u32 = undefined;
    inline for (0..depth) |d| npl[d] = pow2n(base, 1 + @as(u32, @intCast(d)));
    return npl;
}

fn getNodesAboveLevel(comptime base: u8, comptime depth: u8) [depth]u32 {
    const npl = getNodesPerLevel(base, depth);
    var nodes_above: [depth]u32 = undefined;
    var sum: u32 = 0;
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

// IDEA: the first implementation updates bounds as soon as things are added, might be good to add a lazy option in future.
// TODO: keep developing the below struct and add to data + compare performance with naive implementation.
// TODO: check if having size as a comptime parameter actually improves performance
/// A data structure that covers a square region (size * size) of 2D Euclidean space.
/// There are base * base children on each level.
pub fn SquareTree(comptime base: u8, comptime depth: u8, comptime size: f32) type {
    if (base % 2 > 0) {
        // TODO: check if the restriction to even numbers still necessary.
        @compileError("SquareTree only supports even bases from 2 - 16.");
    }
    const node_index = switch (base) {
        2 => u2, // 4 children per node
        4 => u4, // 16 children per node
        6 => u6, // 36 children per node
        8 => u6, // 64 children per node
        10 => u7, // 100 children per node
        12 => u8, // 144 children per node
        14 => u8, // 196 children per node
        16 => u8, // 256 children per node
        else => unreachable,
    };
    const nodex_index_plus = switch (base) {
        2 => u3, // 4 children per node
        4 => u5, // 16 children per node
        6 => u6, // 36 children per node
        8 => u7, // 64 children per node
        10 => u7, // 100 children per node
        12 => u8, // 144 children per node
        14 => u8, // 196 children per node
        16 => u9, // 256 children per node
        else => unreachable,
    };
    const PathIndexVec = @Vector(depth, node_index);
    const PathIndexSlice = []node_index;
    // IDEA: Should we add a custom type for node-number as well?
    //       This would allow for more efficient memory usage
    const nodes_in_level = getNodesPerLevel(base, depth); // the number of nodes in each level
    const nodes_above_level = getNodesAboveLevel(base, depth); // cumulative number of nodes above each level
    const size_per_level = getSizePerLevel(base, depth, size); // size of each square per level
    const scale_per_level = getScalePerLevel(base, depth, size); // reciprocal of the above
    const num_leaf_nodes = nodes_in_level[depth - 1]; // number of nodes in the last (leaf) level
    const num_nodes = nodes_above_level[depth - 1] + num_leaf_nodes; // total number of nodes

    return struct {
        allocator: std.mem.Allocator,
        origin: Vec2f,
        leaf_lists: [num_leaf_nodes]std.ArrayList(Ball2f),
        node_bballs: [num_nodes]Ball2f = undefined,
        node_lens: [num_nodes]u32 = [_]u32{0} ** num_nodes, // the number of descendants within each node
        const Self = @This();

        pub fn init(allocator: std.mem.Allocator, leaf_capacity: u32, origin: Vec2f) !Self {
            var leaf_lists: [num_leaf_nodes]std.ArrayList(Ball2f) = undefined;
            for (0..num_leaf_nodes) |i| {
                leaf_lists[i] = try std.ArrayList(Ball2f).initCapacity(allocator, leaf_capacity);
            }
            return Self{
                .allocator = allocator,
                .leaf_lists = leaf_lists,
                .origin = origin,
            };
        }

        pub fn deinit(self: *Self) void {
            for (0..num_leaf_nodes) |i| self.leaf_lists[i].deinit();
        }

        /// Returns true if the specified point is within the region indexed by this tree, false otherwise.
        pub fn isPointWithinRegion(self: Self, point: Vec2f) bool {
            const d = point - self.origin;
            return 0 <= d[0] and d[0] < size and 0 <= d[1] and d[1] < size;
        }

        /// Gets the index of the path to the leaf node the query point lies within.
        /// Asserts the point is within the region covered by this tree.
        pub fn getPathVecForPoint(self: Self, point: Vec2f) PathIndexVec {
            std.debug.assert(self.isPointWithinRegion(point));
            var index: PathIndexVec = undefined;
            var d_lvl = point - self.origin;
            for (0..depth) |lvl| {
                // TODO: this loop can overflow in some cases - fix this!
                //       May want to introduce a new type (within this struct) for arithmetic operations on path index elements.
                const row: nodex_index_plus = @intFromFloat(scale_per_level[lvl] * d_lvl[1]);
                const col: nodex_index_plus = @intFromFloat(scale_per_level[lvl] * d_lvl[0]);
                index[lvl] = @intCast(row * base + col);
                d_lvl = d_lvl - Vec2f{
                    size_per_level[lvl] * core.asf32(col),
                    size_per_level[lvl] * core.asf32(row),
                };
            }
            return index;
        }

        /// Returns the 'node number' for the node at the specfied level in the provided path index.
        /// This is an integer in the range 0..num_nodes that uniquely identifies a node.
        pub fn getNodeNumber(_: Self, path_vec: PathIndexVec, level: u8) u32 {
            std.debug.assert(level < depth);
            var sum: u32 = 0;
            for (0..(level + 1)) |lvl| {
                sum = @as(u32, @intCast(base * base)) * sum + path_vec[lvl];
            }
            return nodes_above_level[level] + sum;
        }

        pub fn getLeafNodeNumber(self: Self, path_vec: PathIndexVec) u32 {
            return getNodeNumber(self, path_vec, depth - 1);
        }

        pub fn getNodeNumberFromSlice(_: Self, path_slice: PathIndexSlice) u32 {
            std.debug.assert(0 < path_slice.len and path_slice.len <= depth);
            var sum: u32 = 0;
            for (path_slice) |i| {
                sum = @as(u32, @intCast(base * base)) * sum + i;
            }
            return nodes_above_level[node_index.len - 1] + sum;
        }

        /// Gets the path index for the specified leaf node.
        // pub fn getPathIndexForLeafNode(_: Self, node_number: u32) PathIndexVec {
        //     std.debug.assert(node_number >= nodes_above_level[depth - 1]);
        //     const lvl_index = node_number - nodes_above_level[depth - 1];
        //     const squares_before = lvl_index / @as(u32, @intCast(base * base));
        //     const squares_before = lvl_index / @as(u32, @intCast(base * base));
        //     // ...
        //     return;
        // }

        /// Converts an index vector to a node number identifying the last element in the path.
        fn getNodePoint(self: Self, iv: PathIndexVec, level: u8, x_offset: f32, y_offset: f32) Vec2f {
            _ = level; // TODO: make use of this!
            var d_lvl = self.origin;
            for (0..depth) |lvl| {
                // NOTE: This is probably quite slow in practice due to integer divisions.
                const lvl_size = size_per_level[lvl];
                const row = iv[lvl] / base;
                const col = iv[lvl] % base;
                const lvl_offset = Vec2f{
                    lvl_size * (x_offset + core.asf32(col)),
                    lvl_size * (y_offset + core.asf32(row)),
                };
                d_lvl += lvl_offset;
            }
            return d_lvl;
        }

        pub fn getNodeOrigin(self: Self, iv: PathIndexVec, level: u8) Vec2f {
            return self.getNodePoint(iv, level, 0.0, 0.0);
        }

        pub fn getNodeCentre(self: Self, iv: PathIndexVec, level: u8) Vec2f {
            return self.getNodePoint(iv, level, 0.5, 0.5);
        }

        pub fn getNodeOppositeCorner(self: Self, iv: PathIndexVec, level: u8) Vec2f {
            return self.getNodePoint(iv, level, 1.0, 1.0);
        }

        pub fn printInfo(_: Self) void {
            std.debug.print("SquareTree info:\n", .{});
            std.debug.print("Number nodes: {d}\n", .{num_nodes});
            std.debug.print("Nodes per level: {any}\n", .{nodes_in_level});
            std.debug.print("Size per level: {any}\n", .{size_per_level});
            std.debug.print("Leaf scale: {d:3}\n", .{scale_per_level});
        }

        pub fn numberNodes(_: Self) u32 {
            return num_nodes;
        }
    };
}

const svg = @import("svg.zig");
const testing = std.testing;

test "pow2n" {
    const b0 = pow2n(2, 0);
    const b1 = pow2n(2, 1);
    const b2 = pow2n(2, 2);
    const b3 = pow2n(2, 3);
    try testing.expectEqual(1, b0);
    try testing.expectEqual(4, b1);
    try testing.expectEqual(16, b2);
    try testing.expectEqual(64, b3);

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

test "quad tree indexing" {
    var my_qt = try SquareTree(2, 3, 8.0).init(testing.allocator, 4, Vec2f{ 0, 0 });
    defer my_qt.deinit();
    //my_qt.printInfo();

    const pt_check_1 = my_qt.isPointWithinRegion(Vec2f{ 0, 0 });
    const pt_check_2 = my_qt.isPointWithinRegion(Vec2f{ 5, 7 });
    const pt_check_3 = my_qt.isPointWithinRegion(Vec2f{ -1, 3 });
    const pt_check_4 = my_qt.isPointWithinRegion(Vec2f{ 8.0, 8.0 });
    try testing.expectEqual(true, pt_check_1);
    try testing.expectEqual(true, pt_check_2);
    try testing.expectEqual(false, pt_check_3);
    try testing.expectEqual(false, pt_check_4);

    const pt_a = Vec2f{ 4.8, 2.5 };
    const index_a = my_qt.getPathVecForPoint(pt_a);
    const ln_num_a = my_qt.getLeafNodeNumber(index_a);
    const ln_origin = my_qt.getNodeOrigin(index_a, 2);
    const ln_dist_a = core.norm(pt_a - ln_origin);
    try testing.expectEqual(44, ln_num_a);
    try testing.expect(ln_dist_a <= @sqrt(2.0));

    const pt_b = Vec2f{ 7.99, 7.99 };
    const index_b = my_qt.getPathVecForPoint(pt_b);
    const ln_num_b = my_qt.getLeafNodeNumber(index_b);
    const ln_origin_b = my_qt.getNodeOrigin(index_b, 2);
    const ln_dist_b = core.norm(pt_b - ln_origin_b);
    try testing.expectEqual(ln_num_b, my_qt.numberNodes() - 1);
    try testing.expect(ln_dist_b <= @sqrt(2.0));
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

//     /// Updates the bounding volume for this node.
//     pub fn updateBounds(self: *Self) void {
//         if (!self.bounds_require_update or self.length() == 0) return;
//         self.bball = self.bodies.items[0];
//         for (1..self.length()) |i| {
//             // IDEA: it might be more efficient to do an in-place update below
//             self.bball = core.getEncompassingBall(self.bball, self.bodies.items[i]);
//             // IDEA: figuring out the encompassing ball is a little complicated
//             //       It might be more efficient to update the bounding box in the loop
//             //       Then we can fit the encompassing ball based on that afterwards
//         }
//         self.bounds_require_update = false;
//     }

//     // IDEA: why don't we call this resize and allow users to leave things (i.e., static bodies) in there?
//     pub fn clear(self: *Self) !void {
//         try self.bodies.resize(0);
//         self.bball.radius = 0;
//     }

//     // checks if any overlap is possible
//     pub fn isOverlapPossible(self: Self, b: Ball2f) bool {
//         return self.length() > 0 and self.bball.overlapsBall(b);
//     }

//     pub fn getOverlappingIndexes(self: Self, buff: []u24, b: Ball2f) ![]u24 {
//         if (!self.isOverlapPossible(b)) return &[0]u24{};
//         std.debug.assert(self.bodies.items.len < std.math.maxInt(u8));
//         var buff_index: u24 = 0;
//         for (0..self.bodies.items.len) |i| {
//             if (self.bodies.items[i].overlapsBall(b)) {
//                 const tree_index = getTreeIndex(self.node_number, @intCast(i));
//                 buff[buff_index] = tree_index;
//                 buff_index += 1;
//             }
//         }
//         return buff[0..buff_index];
//     }
// };
