//! Square trees that are too large (more than around 1 million nodes) crash the program.
//! This module exists to try and replicate the problem in a smaller example (no luck so far).
//!
const std = @import("std");
const testing = std.testing;

pub const Square = struct {
    centre_x: f32 = 0.0,
    centre_y: f32 = 0.0,
    size: f32 = 1.0,
};

pub fn SquareStruct(comptime size: u32) type {
    const num_values: u64 = @as(u64, @intCast(size)) * @as(u64, @intCast(size));

    return struct {
        id: usize,
        values: [num_values]Square = undefined,
        const Self = @This();

        pub fn numberValues() u64 {
            return num_values;
        }
    };
}

pub fn SquareStruct1(comptime size: u32) type {
    const num_values: u64 = @as(u64, @intCast(size)) * @as(u64, @intCast(size));

    return struct {
        id: usize,
        values: [num_values]Square = undefined,
        const Self = @This();

        pub fn numberValues() u64 {
            return num_values;
        }
    };
}

test "okay" {
    const Square100 = SquareStruct(100);
    try testing.expectEqual(100 * 100, Square100.numberValues());
    const my_square = Square100{
        .id = 7,
    };
    std.debug.print(
        "square100 has id {d} and {d} values; size = {d} kB.\n",
        .{ my_square.id, Square100.numberValues(), @sizeOf(Square100) / 1000 },
    );
}

test "crash" {
    const Square8000 = SquareStruct(8000);
    try testing.expectEqual(8000 * 8000, Square8000.numberValues());
    const my_square = Square8000{
        .id = 42,
    };
    std.debug.print(
        "square8000 has id {d} and {d} values; size = {d} kB.\n",
        .{ my_square.id, Square8000.numberValues(), @sizeOf(Square8000) / 1000 },
    );
}
