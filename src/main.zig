const std = @import("std");
const calc = @import("calc.zig");
const data = @import("data.zig");
const svg = @import("svg.zig");
const vol = @import("volume.zig");
const fmt = std.fmt;
const fs = std.fs;
const os = std.os;
const time = std.time;

const Vec2f = calc.Vec2f;
const Ball2f = vol.Ball2f;
const Box2f = vol.Box2f;
const benchmark_len = 20000;
const test_len = 1000;

var num_trials: u32 = 10;
var random_floats: [benchmark_len]f32 = undefined;
var random_vecs: [benchmark_len]Vec2f = undefined;
var random_balls: [benchmark_len]Ball2f = undefined;
var random_boxes: [benchmark_len]Box2f = undefined;
var random_data_generated = false;

pub fn main() !void {
    if (os.argv.len > 1) {
        const slice = std.mem.span(os.argv[1]);
        num_trials = try fmt.parseInt(u16, slice, 0);
    }
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    defer {
        const check = gpa.deinit();
        std.debug.print("gpa deinit check: {}\n", .{check});
    }
    try runAllBenchmarks(allocator);
}

fn runAllBenchmarks(allocator: std.mem.Allocator) !void {
    genRandomData(0);
    std.debug.print("Running micro-benchmarks for {} bodies...\n", .{benchmark_len});
    try benchmarkIndexing(allocator);
    benchmarkOverlapChecks();

    std.debug.print("\nRunning tree benchmarks for {} bodies...\n\n", .{benchmark_len});
    try benchmarkSquareTree(2, 6, u8, Ball2f, allocator);
    try benchmarkSquareTree(4, 3, u8, Ball2f, allocator);
    try benchmarkSquareTree(2, 6, u8, Box2f, allocator);
    try benchmarkSquareTree(4, 3, u8, Box2f, allocator);
    std.debug.print("Benchmarks complete!\n", .{});
}

fn benchmarkOverlapChecks() void {
    var overlap_count_1: u32 = 0;
    var overlap_count_2: u32 = 0;

    const t_0 = time.microTimestamp();
    const first_ball = random_balls[0];
    for (0..num_trials) |_| {
        for (random_balls) |b| {
            const overlap = vol.checkVolumesOverlap(first_ball, b);
            overlap_count_1 += if (overlap) 1 else 0;
        }
    }
    const t_1 = time.microTimestamp();

    const first_box = random_boxes[0];
    for (0..num_trials) |_| {
        for (random_boxes) |b| {
            const overlap = vol.checkVolumesOverlap(first_box, b);
            overlap_count_2 += if (overlap) 1 else 0;
        }
    }
    const t_2 = time.microTimestamp();

    std.debug.print(
        "Overlapping intervals benchmark: found {}/{} overlaps. Method 1 (balls) took {} us; method 2 (boxes) took {} us.\n",
        .{ overlap_count_1, overlap_count_2, t_1 - t_0, t_2 - t_1 },
    );
}

fn benchmarkSquareTree(
    comptime base_num: u4,
    comptime depth: u4,
    comptime DataIndex: type,
    comptime VolumeType: type,
    allocator: std.mem.Allocator,
) !void {
    const TreeType = data.SquareTree(base_num, depth, DataIndex, VolumeType);
    var type_info_buff: [512]u8 = undefined;
    const type_info = try TreeType.getTypeInfo(&type_info_buff);
    std.debug.print("{s}", .{type_info});
    const leaf_cap = 2 * benchmark_len / TreeType.num_nodes;
    var qt = try TreeType.init(allocator, leaf_cap, Vec2f{ 0, 0 }, 1.0);
    defer qt.deinit();

    const random_bodies = getRandomBodies(VolumeType);
    var body_indexes: [benchmark_len]TreeType.BodyIndex = undefined;
    var overlap_buff: [2 * benchmark_len]TreeType.BodyIndex = undefined;
    var overlap_count_1: u32 = 0;
    var overlap_count_2: u32 = 0;

    const t_0 = time.microTimestamp();
    for (0..num_trials) |_| {
        qt.clearAllBodies();
        for (random_bodies, 0..) |ball, i| body_indexes[i] = try qt.addBody(ball);
    }
    const t_1 = time.microTimestamp();
    for (0..num_trials) |_| {
        qt.updateBounds();
    }
    const t_2 = time.microTimestamp();
    for (0..num_trials) |_| {
        for (random_bodies) |b| {
            const overlaps_found = qt.findOverlaps(&overlap_buff, b);
            overlap_count_1 += @truncate(overlaps_found.len);
        }
    }
    const t_3 = time.microTimestamp();
    for (0..num_trials) |_| {
        for (body_indexes) |i| {
            const overlaps_found = qt.findOverlapsFromIndex(&overlap_buff, i);
            overlap_count_2 += @truncate(overlaps_found.len);
        }
    }
    const t_4 = time.microTimestamp();

    const scale_to_avg_ms = 1.0 / (calc.asf32(num_trials) * 1000.0);
    const add_ms = scale_to_avg_ms * calc.asf32(t_1 - t_0);
    const update_ms = scale_to_avg_ms * calc.asf32(t_2 - t_1);
    const overlap1_ms = scale_to_avg_ms * calc.asf32(t_3 - t_2);
    const overlap2_ms = scale_to_avg_ms * calc.asf32(t_4 - t_3);
    std.debug.print(
        "Benchmark results for {} {}s: method 1 found {} overlaps, method 2 found {} overlaps.\n",
        .{ qt.num_bodies, VolumeType, overlap_count_1 / num_trials, overlap_count_2 / num_trials },
    );
    std.debug.print(
        "Average times: add = {d:.3} ms, update = {d:.3} ms , overlap 1 = {d:.3} ms, overlap 2 = {d:.3} ms.\n\n",
        .{ add_ms, update_ms, overlap1_ms, overlap2_ms },
    );
}

fn benchmarkIndexing(allocator: std.mem.Allocator) !void {
    const Tree16x2 = data.SquareTree(4, 2, u8, Ball2f);
    var ht = try Tree16x2.init(allocator, 8, Vec2f{ 0, 0 }, 1.0);
    defer ht.deinit();
    var ln_sum_1: usize = 0;
    var ln_sum_2: usize = 0;

    const t_0 = time.microTimestamp();
    for (0..num_trials) |_| {
        for (0..benchmark_len) |i| ln_sum_1 += ht.getLeafIndexForPoint(random_vecs[i]);
    }
    const t_1 = time.microTimestamp();
    for (0..num_trials) |_| {
        for (0..benchmark_len) |i| ln_sum_2 += ht.getNodeIndexForPoint(Tree16x2.depth - 1, random_vecs[i]);
    }
    const t_2 = time.microTimestamp();

    std.debug.print(
        "Index benchmark: indexed {} points in {}/{} us. Leaf index sums = {}/{}.\n",
        .{ num_trials * benchmark_len, t_1 - t_0, t_2 - t_1, ln_sum_1, ln_sum_2 },
    );
}

fn getRandUniform(rng: *std.rand.Xoshiro256, min: f32, max: f32) f32 {
    return min + (max - min) * rng.random().float(f32);
}

fn genRandomData(seed: usize) void {
    var rng = std.rand.Xoshiro256.init(seed);
    for (0..benchmark_len) |i| random_floats[i] = getRandUniform(&rng, 0, 1);
    for (0..benchmark_len) |i| {
        const x_index = (i + 2) % benchmark_len;
        const y_index = (i + 7) % benchmark_len;
        const radius = 0.004 * random_floats[i];
        const half_width = if (i % 2 == 0) 0.7854 * radius else radius;
        const half_height = if (i % 2 != 0) 0.7854 * radius else radius;
        random_vecs[i] = Vec2f{ random_floats[x_index], random_floats[y_index] };
        random_balls[i] = Ball2f{
            .centre = random_vecs[i],
            .radius = radius,
        };
        random_boxes[i] = Box2f{
            .centre = random_vecs[i],
            .half_width = half_width,
            .half_height = half_height,
        };
    }
    random_data_generated = true;
}

fn getRandomBodies(comptime T: type) []T {
    return switch (T) {
        Ball2f => &random_balls,
        Box2f => &random_boxes,
        else => unreachable,
    };
}

// integration tests below
const testing = std.testing;
const out_dir_name = "test-out";
const canvas_width = 1000;
const canvas_height = 1000;
const canvas_scale = 400;
const palette_size = 9;
const tolerance = 0.00001;
var dashed_styles: [palette_size]svg.ShapeStyle = undefined;
var solid_styles: [palette_size]svg.ShapeStyle = undefined;
var test_canvas: svg.Canvas = undefined;

const BallGrid = data.SquareTree(4, 2, u16, Ball2f);
var ball_grid: BallGrid = undefined;
const BoxGrid = data.SquareTree(4, 2, u16, Box2f);
var box_grid: BoxGrid = undefined;

fn initTesting(delete_out_dir: bool) !void {
    if (!random_data_generated) { // generate random data for tests
        genRandomData(0);
        dashed_styles[0] = svg.ShapeStyle{ .stroke_hsl = [_]u9{ 0, 0, 0 }, .stroke_dashed = true };
        solid_styles[0] = svg.ShapeStyle{ .stroke_hsl = [_]u9{ 0, 0, 0 } };
        var palette = try svg.RandomHslPalette.init(std.testing.allocator, palette_size - 1);
        for (1.., palette.hsl_colours) |i, hsl| {
            dashed_styles[i] = svg.ShapeStyle{ .stroke_hsl = hsl, .stroke_dashed = true };
            solid_styles[i] = svg.ShapeStyle{ .stroke_hsl = hsl };
        }
        palette.deinit();
    }
    if (delete_out_dir) { // delete test-out and create a new copy
        var cwd = fs.cwd();
        std.debug.print("Creating fresh {s} directory...\n", .{out_dir_name});
        try cwd.deleteTree(out_dir_name);
        try cwd.makeDir(out_dir_name);
    }
    // create new canvas + grid
    test_canvas = try svg.Canvas.init(std.testing.allocator, canvas_width, canvas_height, canvas_scale);
    ball_grid = try BallGrid.init(testing.allocator, 16, Vec2f{ 0, 0 }, 1.0);
    box_grid = try BoxGrid.init(testing.allocator, 16, Vec2f{ 0, 0 }, 1.0);
}

fn deinitTesting() void {
    test_canvas.deinit();
    ball_grid.deinit();
    box_grid.deinit();
}

fn getNodeLabel(buff: []u8, lvl: u8, index: u32) ![]const u8 {
    return switch (lvl) {
        0 => try std.fmt.bufPrint(buff, "{X:0>1}", .{index}),
        1 => try std.fmt.bufPrint(buff, "{X:0>2}", .{index}),
        2 => try std.fmt.bufPrint(buff, "{X:0>3}", .{index}),
        3 => try std.fmt.bufPrint(buff, "{X:0>4}", .{index}),
        4 => try std.fmt.bufPrint(buff, "{X:0>5}", .{index}),
        else => unreachable,
    };
}

test "draw encompassing balls" {
    try initTesting(true);
    defer deinitTesting();
    for (0..test_len / 20) |i| {
        const a = Ball2f{
            .centre = random_vecs[3 * i],
            .radius = 0.25 * random_floats[7 * i],
        };
        const b = Ball2f{
            .centre = random_vecs[11 * i],
            .radius = 0.25 * random_floats[13 * i],
        };
        const c = vol.getEncompassingVolume(Ball2f, a, b);
        try test_canvas.addCircle(testing.allocator, a.centre, a.radius, solid_styles[1]);
        try test_canvas.addCircle(testing.allocator, b.centre, b.radius, solid_styles[3]);
        try test_canvas.addCircle(testing.allocator, c.?.centre, c.?.radius, dashed_styles[7]);
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

test "draw encompassing boxes" {
    try initTesting(false);
    defer deinitTesting();
    for (0..test_len / 100) |i| {
        const a = Box2f{
            .centre = random_vecs[3 * i],
            .half_width = 0.25 * random_floats[5 * i],
            .half_height = 0.25 * random_floats[7 * i],
        };
        const b = Box2f{
            .centre = random_vecs[11 * i],
            .half_width = 0.25 * random_floats[13 * i],
            .half_height = 0.25 * random_floats[17 * i],
        };
        const c = vol.getEncompassingVolume(Box2f, a, b);
        const corners_a = a.getCorners();
        const corners_b = b.getCorners();
        const corners_c = c.?.getCorners();
        try test_canvas.addRectangle(testing.allocator, corners_a[0], corners_a[1], solid_styles[1]);
        try test_canvas.addRectangle(testing.allocator, corners_b[0], corners_b[1], solid_styles[3]);
        try test_canvas.addRectangle(testing.allocator, corners_c[0], corners_c[1], dashed_styles[7]);
        // write an image to file
        var fname_buff: [128]u8 = undefined;
        const fpath = try fmt.bufPrint(
            &fname_buff,
            "{s}/encompassing-boxes-{}.html",
            .{ out_dir_name, i + 1 },
        );
        try test_canvas.writeHtml(testing.allocator, fpath, true);
    }
}

test "draw ball grid" {
    try initTesting(false);
    defer deinitTesting();

    // create an svg canvas depicting the grid structure
    var text_buff: [16]u8 = undefined;
    for (BallGrid.reverse_levels) |lvl| {
        var style = solid_styles[(2 * lvl) % palette_size];
        style.stroke_width = @truncate(BallGrid.depth - lvl);
        const font_size: u8 = 6 + 4 * (BallGrid.depth - lvl);
        const lvl_end_index = BallGrid.nodes_in_level[lvl];
        for (0..lvl_end_index) |i| {
            const index: BallGrid.NodeIndex = @intCast(i);
            const node_origin = try ball_grid.getNodeOrigin(lvl, index);
            const node_centre = try ball_grid.getNodeCentre(lvl, index);
            const node_corner = try ball_grid.getNodeCorner(lvl, index);
            const node_label = try getNodeLabel(&text_buff, lvl, index);
            try test_canvas.addRectangle(testing.allocator, node_origin, node_corner, style);
            try test_canvas.addText(testing.allocator, node_centre, node_label, font_size, style.stroke_hsl);
        }
    }

    //  add some data to the test grid and record indexes of all bodies that overlap with others
    var body_indexes: [test_len]BallGrid.BodyIndex = undefined;
    for (0.., random_balls[0..test_len]) |i, b| body_indexes[i] = try ball_grid.addBody(b);
    ball_grid.updateBounds();
    var overlap_set = std.AutoHashMap(BallGrid.BodyIndex, void).init(testing.allocator);
    defer overlap_set.deinit();
    for (body_indexes) |i| {
        var overlap_buff: [test_len]BallGrid.BodyIndex = undefined;
        const overlaps_found = ball_grid.findOverlapsFromIndex(&overlap_buff, i);
        if (overlaps_found.len == 0) continue;
        try overlap_set.put(i, {});
        for (overlaps_found) |j| try overlap_set.put(j, {});
    }

    // draw circles on the canvas; write SVG file.
    for (body_indexes) |i| {
        const body = ball_grid.getBody(i) orelse unreachable;
        const style = if (overlap_set.contains(i)) solid_styles[6] else solid_styles[8];
        try test_canvas.addCircle(testing.allocator, body.centre, body.radius, style);
    }
    var fname_buff: [128]u8 = undefined;
    const fpath = try fmt.bufPrint(
        &fname_buff,
        "{s}/ball-grid-{}-{}.html",
        .{ out_dir_name, BallGrid.base, BallGrid.depth },
    );
    try test_canvas.writeHtml(testing.allocator, fpath, true);
}

test "draw box grid" {
    try initTesting(false);
    defer deinitTesting();

    // create an svg canvas depicting the grid structure
    var text_buff: [16]u8 = undefined;
    for (BoxGrid.reverse_levels) |lvl| {
        var style = solid_styles[(2 * lvl) % palette_size];
        style.stroke_width = @truncate(BoxGrid.depth - lvl);
        const font_size: u8 = 6 + 4 * (BoxGrid.depth - lvl);
        const lvl_end_index = BoxGrid.nodes_in_level[lvl];
        for (0..lvl_end_index) |i| {
            const index: BoxGrid.NodeIndex = @intCast(i);
            const node_origin = try box_grid.getNodeOrigin(lvl, index);
            const node_centre = try box_grid.getNodeCentre(lvl, index);
            const node_corner = try box_grid.getNodeCorner(lvl, index);
            const node_label = try getNodeLabel(&text_buff, lvl, index);
            try test_canvas.addRectangle(testing.allocator, node_origin, node_corner, style);
            try test_canvas.addText(testing.allocator, node_centre, node_label, font_size, style.stroke_hsl);
        }
    }

    //  add some data to the test grid and record indexes of all bodies that overlap with others
    var body_indexes: [test_len]BoxGrid.BodyIndex = undefined;
    for (0.., random_boxes[0..test_len]) |i, b| body_indexes[i] = try box_grid.addBody(b);
    box_grid.updateBounds();
    var overlap_set = std.AutoHashMap(BoxGrid.BodyIndex, void).init(testing.allocator);
    defer overlap_set.deinit();
    for (body_indexes) |i| {
        var overlap_buff: [test_len]BoxGrid.BodyIndex = undefined;
        const overlaps_found = box_grid.findOverlapsFromIndex(&overlap_buff, i);
        if (overlaps_found.len == 0) continue;
        try overlap_set.put(i, {});
        for (overlaps_found) |j| try overlap_set.put(j, {});
    }

    // draw rectangles on the canvas; write SVG file.
    for (body_indexes) |i| {
        const box = box_grid.getBody(i) orelse unreachable;
        const box_corners = box.getCorners();
        const style = if (overlap_set.contains(i)) solid_styles[6] else solid_styles[8];
        try test_canvas.addRectangle(testing.allocator, box_corners[0], box_corners[1], style);
    }
    var fname_buff: [128]u8 = undefined;
    const fpath = try fmt.bufPrint(
        &fname_buff,
        "{s}/box-grid-{}-{}.html",
        .{ out_dir_name, BoxGrid.base, BoxGrid.depth },
    );
    try test_canvas.writeHtml(testing.allocator, fpath, true);
}

test "integration test 1" {
    try initTesting(false);
    defer deinitTesting();

    // check the random points are indexed correctly
    const leaf_size = ball_grid.size_per_level[BallGrid.depth - 1];
    const max_dist_expected = 0.5 * @sqrt(2 * (leaf_size * leaf_size));
    var max_dist_found: f32 = 0.0;
    for (0..test_len) |i| {
        const point = random_vecs[i];
        const index = ball_grid.getLeafIndexForPoint(point);
        const centre = try ball_grid.getNodeCentre(BallGrid.depth - 1, index);
        const dist = calc.norm(point - centre);
        if (dist > max_dist_expected) {
            std.debug.print(
                "{}. query point = {d:.3}; index = {X}, node-centre = {d:.3}, distance = {d:.4}\n",
                .{ i, point, index, centre, dist },
            );
        }
        max_dist_found = @max(dist, max_dist_found);
    }
    std.debug.print(
        "max distance found was {d:.4} (expected limit = {d:.4})\n",
        .{ max_dist_found, max_dist_expected },
    );
    try testing.expect(max_dist_found < max_dist_expected + tolerance);

    // store some random points in the grid
    var body_indexes: [test_len]BallGrid.BodyIndex = undefined;
    for (0..test_len) |i| body_indexes[i] = try ball_grid.addBody(random_balls[i]);
    ball_grid.updateBounds();

    // find the overlapping bodies and record their indexes
    var overlap_set_1 = std.AutoHashMap(BallGrid.BodyIndex, void).init(testing.allocator);
    defer overlap_set_1.deinit();
    for (body_indexes) |i| {
        var buff: [test_len]BallGrid.BodyIndex = undefined;
        const overlaps_found = ball_grid.findOverlapsFromIndex(&buff, i);
        if (overlaps_found.len > 0) {
            try overlap_set_1.put(i, {});
            for (overlaps_found) |j| try overlap_set_1.put(j, {});
        }
    }

    // find overlaps using the alternative function
    var overlap_set_2 = std.AutoHashMap(BallGrid.BodyIndex, void).init(testing.allocator);
    defer overlap_set_2.deinit();
    for (random_balls[0..test_len], body_indexes) |b, i| {
        var buff: [test_len]BallGrid.BodyIndex = undefined;
        const overlaps_found = ball_grid.findOverlaps(&buff, b);
        if (overlaps_found.len > 0) {
            for (overlaps_found) |j| {
                if (i.leaf_index == j.leaf_index and i.data_index == j.data_index) continue;
                try overlap_set_2.put(j, {});
            }
        }
    }

    // cross-check overlap results for consistency
    var iter_1 = overlap_set_1.keyIterator();
    while (iter_1.next()) |i| try testing.expect(overlap_set_2.contains(i.*));
    try testing.expectEqual(overlap_set_1.count(), overlap_set_2.count());
}
