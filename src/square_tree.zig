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

fn getNodesPerLevelArray(comptime base: u8, comptime depth: u8) [depth]u32 {
    var npl: [depth]u32 = undefined;
    inline for (0..depth) |d| npl[d] = pow2n(base, 1 + @as(u32, @intCast(d)));
    return npl;
}

fn getLevelStartIndexes(comptime base: u8, comptime depth: u8) [depth]u32 {
    const npl = getNodesPerLevelArray(base, depth);
    var start_indexes: [depth]u32 = undefined;
    var sum: u32 = 0;
    inline for (0..depth) |d| {
        start_indexes[d] = sum;
        sum += npl[d];
    }
    return start_indexes;
}

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

    const npl_quad = getNodesPerLevelArray(2, 4);
    try testing.expectEqual(4, npl_quad[0]); // index 0 is refers to the level just below the root (not the root itself!)
    try testing.expectEqual(16, npl_quad[1]); //
    try testing.expectEqual(64, npl_quad[2]);
    try testing.expectEqual(256, npl_quad[3]);

    var my_qt = try SquareTree(2, 4, Vec2f{ 0, 0 }, 1000).init(testing.allocator);
    defer my_qt.deinit();
}

// TODO: keep developing the below struct and add to data + compare performance with naive implementation.
/// A data structure that covers a square region (size * size) of 2D Euclidean space.
/// There are base * base children on each level.
pub fn SquareTree(base: u8, depth: u8, origin: Vec2f, size: f32) type {
    _ = origin;
    _ = size;
    const nodes_per_level = getNodesPerLevelArray(base, depth);

    return struct {
        allocator: std.mem.Allocator = undefined,
        //node_bounding_balls: [number_nodes]Ball2f = undefined,
        const Self = @This();

        pub fn init(allocator: std.mem.Allocator) !Self {
            std.debug.print("init square tree; nodes-per-level: {any}\n", .{nodes_per_level});
            return Self{ .allocator = allocator };
        }

        pub fn deinit(self: *Self) void {
            //self.allocator.free(memory: anytype)
            _ = self;
        }
    };
}
