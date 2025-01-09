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

pub fn SquareStruct2(comptime size: u32) type {
    const num_values: u64 = @as(u64, @intCast(size)) * @as(u64, @intCast(size));

    return struct {
        id: usize,
        values: [num_values]std.ArrayList(Square) = undefined,
        const Self = @This();

        pub fn numberValues() u64 {
            return num_values;
        }
    };
}

test "struct 1 - 400" {
    const Square1_400 = SquareStruct1(400);
    try testing.expectEqual(400 * 400, Square1_400.numberValues());
    const my_square = Square1_400{
        .id = 401,
    };
    std.debug.print(
        "square1_400 has id {d} and {d} values; size = {d} kB.\n",
        .{ my_square.id, Square1_400.numberValues(), @sizeOf(Square1_400) / 1000 },
    );
}

test "struct 1 - 5000" {
    const Square1_5000 = SquareStruct1(5000);
    try testing.expectEqual(5000 * 5000, Square1_5000.numberValues());
    const my_square = Square1_5000{
        .id = 5001,
    };
    std.debug.print(
        "square1_5000 has id {d} and {d} values; size = {d} kB.\n",
        .{ my_square.id, Square1_5000.numberValues(), @sizeOf(Square1_5000) / 1000 },
    );
}

test "struct 2 - 400" {
    const Square2_400 = SquareStruct2(400);
    try testing.expectEqual(400 * 400, Square2_400.numberValues());
    const my_square = Square2_400{
        .id = 402,
    };
    std.debug.print(
        "square2_400 has id {d} and {d} values; size = {d} kB.\n",
        .{ my_square.id, Square2_400.numberValues(), @sizeOf(Square2_400) / 1000 },
    );
}

test "struct 2 - 5000" {
    const Square2_5000 = SquareStruct2(5000);
    try testing.expectEqual(5000 * 5000, Square2_5000.numberValues());
    const my_square = Square2_5000{
        .id = 5002,
    };
    std.debug.print(
        "square2_5000 has id {d} and {d} values; size = {d} kB.\n",
        .{ my_square.id, Square2_5000.numberValues(), @sizeOf(Square2_5000) / 1000 },
    );
}
