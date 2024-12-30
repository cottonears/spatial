const std = @import("std");
const core = @import("core.zig");
const data = @import("data.zig");
const square_tree = @import("square_tree.zig");
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
const random_len = 10000;
const num_trials = 10;
var random_floats: [random_len]f32 = undefined;
var random_vecs: [random_len]Vec2f = undefined;
var random_data_generated = false;

pub fn main() !void {
    var num_bodies: u32 = random_len;
    if (os.argv.len > 1) {
        const slice = std.mem.span(os.argv[1]);
        num_bodies = try fmt.parseInt(u32, slice, 0);
    }

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    defer {
        const check = gpa.deinit();
        std.debug.print("gpa deinit check: {}\n", .{check});
    }
    try runAllBenchmarks(allocator, num_bodies);
}

fn runAllBenchmarks(allocator: std.mem.Allocator, n: u32) !void {
    genRandomFloats();
    std.debug.print("Running all benchmarks for {} bodies...\n", .{n});
    //benchmarkVectorMaths();
    // TOOD: add more benchmarks
    try benchmarkSquareTreeIndexing(allocator);
    std.debug.print("Benchmarks complete!\n", .{});
}

fn benchmarkUpdateBounds() void {
    // TODO: implement + test this
}
fn benchmarkSquareTreeIndexing(allocator: std.mem.Allocator) !void {
    std.debug.print("Starting square-tree benchmark...", .{});
    const tree_depth = 4;
    const HexTree4 = square_tree.SquareTree(u4, tree_depth, 1.0);
    var tree = try HexTree4.init(allocator, 8, Vec2f{ 0, 0 });
    defer tree.deinit();

    var vec_sum_1: @Vector(tree_depth, u32) = [_]u32{0} ** tree_depth;
    var vec_sum_2: @Vector(tree_depth, u32) = [_]u32{0} ** tree_depth;
    var ln_sum_1: usize = 0;
    var ln_sum_2: usize = 0;

    const t_0 = time.microTimestamp();
    for (0..random_len) |i| {
        const p_1 = tree.getPathForPoint(random_vecs[i]);
        const ln_1 = HexTree4.getLeafNumber(p_1);
        vec_sum_1 += p_1;
        ln_sum_1 += ln_1;
    }
    const t_1 = time.microTimestamp();
    for (0..random_len) |i| {
        const p_2 = tree.getPathForPoint(random_vecs[i]);
        const ln_2 = HexTree4.getNodeNumber(p_2[0..]);
        vec_sum_2 += p_2;
        ln_sum_2 += ln_2;
    }
    const t_2 = time.microTimestamp();

    const differences = vec_sum_1 != vec_sum_2;
    const any_diff = @reduce(.Or, differences) or ln_sum_1 != ln_sum_2;
    std.debug.print(
        "Benchmark results for path-to-point ({} points):\nV1 took {} us, V2 took {} us; differences = {}.\n",
        .{ random_len, t_1 - t_0, t_2 - t_1, any_diff },
    );
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
    for (0..random_len) |i| random_floats[i] = core.getRandUniform(0, 1);
    for (0..random_len) |i| {
        const x_index = (i + 2) % random_len;
        const y_index = (i + 7) % random_len;
        random_vecs[i] = Vec2f{ random_floats[x_index], random_floats[y_index] };
    }
    random_data_generated = true;
}

fn getHexStringNoSpaces(comptime T: type, slice: []const T, buff: []u8) ![]u8 {
    var buff_offset: usize = 0;
    for (0..slice.len) |i| {
        buff_offset += (try std.fmt.bufPrint(buff[buff_offset..], "{X}", .{slice[i]})).len;
    }
    return buff[0..buff_offset];
}

// integration tests below
const testing = std.testing;
const out_dir_name = "test-out";
const canvas_width = 2000;
const canvas_height = 2000;
const canvas_scale = 1000;
const palette_size = 8;
const tolerance = 0.00001;
var dashed_styles: [palette_size]svg.ShapeStyle = undefined;
var solid_styles: [palette_size]svg.ShapeStyle = undefined;
var test_canvas: svg.Canvas = undefined;

fn initTesting(delete_out_dir: bool) !void {
    // delete test-out and create a new copy
    if (delete_out_dir) {
        var cwd = fs.cwd();
        std.debug.print("Creating fresh {s} directory...\n", .{out_dir_name});
        try cwd.deleteTree(out_dir_name);
        try cwd.makeDir(out_dir_name);
    }
    // set up a canvas for drawing images on
    std.debug.print(
        "Creating svg canvas ({}x{}) and random palette ({} colours)...\n",
        .{ canvas_width, canvas_height, palette_size },
    );
    test_canvas = try svg.Canvas.init(std.testing.allocator, canvas_width, canvas_height, canvas_scale);
    var palette = try svg.RandomHslPalette.init(std.testing.allocator, palette_size);
    for (0.., palette.hsl_colours) |i, hsl| {
        dashed_styles[i] = svg.ShapeStyle{ .stroke_hsl = hsl, .stroke_dashed = true };
        solid_styles[i] = svg.ShapeStyle{ .stroke_hsl = hsl };
    }
    palette.deinit();
    // generate random data for tests
    genRandomFloats();
}

fn deinitTesting() void {
    test_canvas.deinit();
}

test "encompassing balls" {
    try initTesting(true);
    defer deinitTesting();
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

test "hex tree 3" {
    try initTesting(false);
    defer deinitTesting();
    const tree_depth = 3;
    const HexTree3 = square_tree.SquareTree(u4, tree_depth, 1.0);
    HexTree3.printTypeInfo();
    var tree = try HexTree3.init(testing.allocator, 8, Vec2f{ 0, 0 });
    defer tree.deinit();

    // check the random points are binned correctly
    const max_dist_expected = 0.5 * @sqrt(2 * (0.015625 * 0.015625)); // to the centre of a leaf node
    var max_dist_found: f32 = 0.0;
    for (0..random_len) |i| {
        const point = random_vecs[i];
        const path = tree.getPathForPoint(point);
        const centre = tree.getNodeCentre(&path);
        const dist = core.norm(point - centre);
        if (i < 8 or dist > max_dist_expected) { // print details of the first few points, or any that are far from their leaf node's centre
            std.debug.print(
                "{}. query point = {d:.3}; path = {X}, node-centre = {d:.3}, distance = {d:.4}\n",
                .{ i, point, path, centre, dist },
            );
        }
        max_dist_found = @max(dist, max_dist_found);
    }
    std.debug.print(
        "max distance found was {d:.4} (expected limit = {d:.4})\n",
        .{ max_dist_found, max_dist_expected },
    );
    try testing.expect(max_dist_found < max_dist_expected + tolerance);

    // create an svg image depicting the tree
    var text_buff: [16]u8 = undefined;
    var path_buff: [tree_depth]u4 = undefined;
    for (HexTree3.reverse_levels) |lvl| {
        var style = solid_styles[2 * lvl];
        style.stroke_width = @truncate(tree_depth - lvl);
        const font_size: u8 = 9 + 3 * (tree_depth - lvl);
        const lvl_start = HexTree3.nodes_above_level[lvl];
        const lvl_end = lvl_start + HexTree3.nodes_in_level[lvl];
        for (lvl_start..lvl_end) |n| {
            const path = HexTree3.getPathToNode(&path_buff, @intCast(n));
            const node_origin = tree.getNodeOrigin(path);
            const node_centre = tree.getNodeCentre(path);
            const node_corner = tree.getNodeCorner(path);
            const node_label = try getHexStringNoSpaces(u4, path, &text_buff);
            try test_canvas.addRectangle(testing.allocator, node_origin, node_corner, style);
            try test_canvas.addText(testing.allocator, node_centre, node_label, font_size, style.stroke_hsl);
        }
    }
    var fname_buff: [128]u8 = undefined;
    const fpath = try fmt.bufPrint(&fname_buff, "{s}/hex-tree-{}.html", .{ out_dir_name, tree_depth });
    try test_canvas.writeHtml(testing.allocator, fpath, true);
}
