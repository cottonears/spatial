const std = @import("std");
const core = @import("core.zig");
const data = @import("data.zig");
const svg = @import("svg.zig");
const Vec2f = core.Vec2f;
const Vec3f = core.Vec3f;
const Vec4f = core.Vec4f;
const Vec6f = core.Vec6f;
const Vec8f = core.Vec8f;
const Vec12f = core.Vec12f;
const Vec16f = core.Vec16f;
const Ball2f = core.Ball2f;
const Box2f = core.Box2f;
const fmt = std.fmt;
const fs = std.fs;
const os = std.os;
const time = std.time;
const random_len = 8000;
const num_trials = 10;
var random_floats: [random_len]f32 = undefined;
var random_vecs: [random_len]Vec2f = undefined;
var random_data_generated = false;

pub fn main() !void {
    var num_bodies: u32 = 10000;
    if (os.argv.len > 1) {
        const slice = std.mem.span(os.argv[1]);
        num_bodies = try fmt.parseInt(u32, slice, 0);
    }

    try runAllBenchmarks(num_bodies);
}

fn runAllBenchmarks(n: u32) !void {
    genRandomFloats();
    std.debug.print("Running all benchmarks for {} bodies...\n", .{n});
    benchmarkVectorMaths();
    // TOOD: add benchmarks
}

fn benchmarkUpdateBounds() void {
    // TODO: implement + test this
}

fn benchmarkVectorMaths() void {
    const a = core.Ball2f{ .centre = Vec2f{ 0.0, 0.0 }, .radius = 0.5 };
    var result_vec_1: @Vector(random_len, bool) = undefined;
    var result_vec_2: @Vector(random_len, bool) = undefined;
    var any_diff: bool = false;
    var v1_time: u64 = 0;
    var v2_time: u64 = 0;

    for (0..num_trials) |_| {
        const t_0 = time.microTimestamp();
        for (0..(random_len / 8)) |i| {
            const b0 = Ball2f{ .centre = random_vecs[8 * i + 0], .radius = random_floats[8 * i + 0] };
            const b1 = Ball2f{ .centre = random_vecs[8 * i + 1], .radius = random_floats[8 * i + 1] };
            const b2 = Ball2f{ .centre = random_vecs[8 * i + 2], .radius = random_floats[8 * i + 2] };
            const b3 = Ball2f{ .centre = random_vecs[8 * i + 3], .radius = random_floats[8 * i + 3] };
            const b4 = Ball2f{ .centre = random_vecs[8 * i + 4], .radius = random_floats[8 * i + 4] };
            const b5 = Ball2f{ .centre = random_vecs[8 * i + 5], .radius = random_floats[8 * i + 5] };
            const b6 = Ball2f{ .centre = random_vecs[8 * i + 6], .radius = random_floats[8 * i + 6] };
            const b7 = Ball2f{ .centre = random_vecs[8 * i + 7], .radius = random_floats[8 * i + 7] };
            result_vec_1[8 * i + 0] = a.overlapsBall(b0);
            result_vec_1[8 * i + 1] = a.overlapsBall(b1);
            result_vec_1[8 * i + 2] = a.overlapsBall(b2);
            result_vec_1[8 * i + 3] = a.overlapsBall(b3);
            result_vec_1[8 * i + 4] = a.overlapsBall(b4);
            result_vec_1[8 * i + 5] = a.overlapsBall(b5);
            result_vec_1[8 * i + 6] = a.overlapsBall(b6);
            result_vec_1[8 * i + 7] = a.overlapsBall(b7);
        }
        const t_1 = time.microTimestamp();
        for (0..(random_len / 8)) |i| {
            const b0 = Ball2f{ .centre = random_vecs[8 * i + 0], .radius = random_floats[8 * i + 0] };
            const b1 = Ball2f{ .centre = random_vecs[8 * i + 1], .radius = random_floats[8 * i + 1] };
            const b2 = Ball2f{ .centre = random_vecs[8 * i + 2], .radius = random_floats[8 * i + 2] };
            const b3 = Ball2f{ .centre = random_vecs[8 * i + 3], .radius = random_floats[8 * i + 3] };
            const b4 = Ball2f{ .centre = random_vecs[8 * i + 4], .radius = random_floats[8 * i + 4] };
            const b5 = Ball2f{ .centre = random_vecs[8 * i + 5], .radius = random_floats[8 * i + 5] };
            const b6 = Ball2f{ .centre = random_vecs[8 * i + 6], .radius = random_floats[8 * i + 6] };
            const b7 = Ball2f{ .centre = random_vecs[8 * i + 7], .radius = random_floats[8 * i + 7] };
            const overlap_result = a.overlapsBallsX8([_]Ball2f{ b0, b1, b2, b3, b4, b5, b6, b7 });
            result_vec_2[8 * i + 0] = overlap_result[0];
            result_vec_2[8 * i + 1] = overlap_result[1];
            result_vec_2[8 * i + 2] = overlap_result[2];
            result_vec_2[8 * i + 3] = overlap_result[3];
            result_vec_2[8 * i + 4] = overlap_result[4];
            result_vec_2[8 * i + 5] = overlap_result[5];
            result_vec_2[8 * i + 6] = overlap_result[6];
            result_vec_2[8 * i + 7] = overlap_result[7];
        }
        const t_2 = time.microTimestamp();
        const differences = result_vec_1 != result_vec_2;
        const trial_diff = @reduce(.Or, differences);
        any_diff = any_diff or trial_diff;
        v1_time += @intCast(t_1 - t_0);
        v2_time += @intCast(t_2 - t_1);
    }
    std.debug.print(
        "Benchmark results for squaredSum:\nVersion 1 took {} us, Version 2 took {} us; differences = {}.\n",
        .{ v1_time, v2_time, any_diff },
    );
}

fn genRandomFloats() void {
    if (random_data_generated) return;
    std.debug.print("Generating {} random floats...\n", .{random_len});
    for (0..random_len) |i| random_floats[i] = core.getRandUniform(-1, 1);
    for (0..random_len) |i| {
        const x_index = (i + 2) % random_len;
        const y_index = (i + 7) % random_len;
        random_vecs[i] = Vec2f{ random_floats[x_index], random_floats[y_index] };
    }
    random_data_generated = true;
}

// integration tests below
const testing = std.testing;
const out_dir_name = "test-out";
const canvas_width = 1000;
const canvas_height = 1000;
const canvas_scale = 250;
const palette_size = 8;
var dashed_styles: [palette_size]svg.ShapeStyle = undefined;
var solid_styles: [palette_size]svg.ShapeStyle = undefined;
var test_canvas: svg.SvgCanvas = undefined;

fn init() !void {
    // delete test-out and create a new copy
    var cwd = fs.cwd();
    std.debug.print("Creating fresh {s} directory...\n", .{out_dir_name});
    try cwd.deleteTree(out_dir_name);
    try cwd.makeDir(out_dir_name);
    // set up a canvas for drawing images on
    std.debug.print(
        "Creating svg canvas ({}x{}) and random palette ({} colours)...\n",
        .{ canvas_width, canvas_height, palette_size },
    );
    test_canvas = svg.SvgCanvas.init(std.testing.allocator, canvas_width, canvas_height, canvas_scale);
    var palette = try svg.RandomHslPalette.init(std.testing.allocator, palette_size);
    for (0.., palette.hsl_colours) |i, hsl| {
        dashed_styles[i] = svg.ShapeStyle{ .stroke_hsl = hsl, .stroke_dashed = true };
        solid_styles[i] = svg.ShapeStyle{ .stroke_hsl = hsl };
    }
    palette.deinit();
    // generate random data for tests
    genRandomFloats();
}

fn deinit() void {
    test_canvas.deinit();
}

test "encompassing balls" {
    try init();
    defer deinit();
    for (0..8) |i| {
        const a = Ball2f{ .centre = random_vecs[i], .radius = @abs(0.5 * random_floats[i]) };
        const b = Ball2f{ .centre = random_vecs[i + 1], .radius = @abs(0.5 * random_floats[i + 1]) };
        const c = core.getEncompassingBall(a, b);
        try test_canvas.addCircle(testing.allocator, a.centre, a.radius, solid_styles[0]);
        try test_canvas.addCircle(testing.allocator, b.centre, b.radius, solid_styles[2]);
        try test_canvas.addCircle(testing.allocator, c.centre, c.radius, dashed_styles[6]);
        // write an image to file
        var fname_buff: [128]u8 = undefined;
        const fpath = try fmt.bufPrint(
            &fname_buff,
            "{s}/encompassing-balls-{}.html",
            .{ out_dir_name, i + 1 },
        );
        try test_canvas.writeHtml(testing.allocator, fpath, true);
    }
}
