const std = @import("std");

// TODO: Build out the library's ABI in this file and test it from C.

export fn add(a: i32, b: i32) i32 {
    return a + b;
}
