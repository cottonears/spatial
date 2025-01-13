const std = @import("std");
const core = @import("core.zig");
const data = @import("data.zig");
const svg = @import("svg.zig");
const fmt = std.fmt;
const fs = std.fs;
const os = std.os;
const time = std.time;

const Vec2f = core.Vec2f;
const Ball2f = core.Ball2f;
const Box2f = core.Box2f;
const benchmark_len = 10000;
const test_len = 1000;

var num_trials: u16 = 10;
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
    genRandomData();
    std.debug.print("Running all benchmarks for {} bodies...\n", .{benchmark_len});
    try benchmarkIndexing(allocator);
    try benchmarkSquareTree(allocator);
    benchmarkOverlapChecks();
    std.debug.print("Benchmarks complete!\n", .{});
}

fn benchmarkSquareTree(allocator: std.mem.Allocator) !void {
    const tree_depth = 5;
    const TreeType = data.SquareTree(2, tree_depth, u6);
    const leaf_cap = 2 * benchmark_len / TreeType.num_nodes;
    var qt = try TreeType.init(allocator, leaf_cap, Vec2f{ 0, 0 }, 1.0);
    defer qt.deinit();
    var overlap_buff: [benchmark_len]TreeType.BodyIndex = undefined;
    var overlap_count: u32 = 0;

    const t_0 = time.microTimestamp();
    for (0..num_trials) |_| {
        for (random_balls) |ball| _ = try qt.addBody(ball);
    }
    const t_1 = time.microTimestamp();
    for (0..num_trials) |_| {
        qt.updateBounds();
    }
    const t_2 = time.microTimestamp();
    for (0..num_trials) |_| {
        for (random_balls) |b| {
            const overlaps_found = try qt.getOverlappingBodies(&overlap_buff, b);
            overlap_count += @truncate(overlaps_found.len);
        }
    }
    const t_3 = time.microTimestamp();

    std.debug.print(
        "Overlapping balls benchmark: found {} overlaps. Add took {} us, update took {} us, overlap checks took {} ms.\n",
        .{ overlap_count, t_1 - t_0, t_2 - t_1, @divFloor(t_3 - t_2, 1000) },
    );
}

fn benchmarkOverlapChecks() void {
    var overlap_count_1: u32 = 0;
    var overlap_count_2: u32 = 0;

    const t_0 = time.microTimestamp();
    const first_ball = random_balls[0];
    for (0..num_trials) |_| {
        for (random_balls) |b| {
            const overlap = first_ball.overlapsBall(b);
            overlap_count_1 += if (overlap) 1 else 0;
        }
    }
    const t_1 = time.microTimestamp();

    const first_box = random_boxes[0];
    for (0..num_trials) |_| {
        for (random_boxes) |b| {
            const overlap = first_box.overlapsBox(b);
            overlap_count_2 += if (overlap) 1 else 0;
        }
    }
    const t_2 = time.microTimestamp();

    std.debug.print(
        "Overlapping intervals benchmark: found {}/{} overlaps. Method 1 took {} us; method 2 took {} us.\n",
        .{ overlap_count_1, overlap_count_2, t_1 - t_0, t_2 - t_1 },
    );
}

fn benchmarkIndexing(allocator: std.mem.Allocator) !void {
    const tree_depth = 2;
    const HexTree4 = data.SquareTree(4, tree_depth, u8);
    var ht = try HexTree4.init(allocator, 8, Vec2f{ 0, 0 }, 1.0);
    defer ht.deinit();
    var ln_sum_1: usize = 0;

    const t_0 = time.microTimestamp();
    for (0..benchmark_len) |i| {
        const ln_1 = ht.getLeafIndexForPoint(random_vecs[i]);
        ln_sum_1 += ln_1;
    }
    const t_1 = time.microTimestamp();

    std.debug.print(
        "Index benchmark: indexed {} points in {} us. Leaf index sum = {}.\n",
        .{ benchmark_len, t_1 - t_0, ln_sum_1 },
    );
}

fn genRandomData() void {
    for (0..benchmark_len) |i| random_floats[i] = core.getRandUniform(0, 1);
    for (0..benchmark_len) |i| {
        const x_index = (i + 2) % benchmark_len;
        const y_index = (i + 7) % benchmark_len;
        const radius = 0.01 * random_floats[i];
        const half_width = if (i % 2 == 0) 0.7854 * radius else radius;
        const half_height = if (i % 2 != 0) 0.7854 * radius else radius;
        random_vecs[i] = Vec2f{ random_floats[x_index], random_floats[y_index] };
        random_balls[i] = Ball2f{ .centre = random_vecs[i], .radius = radius, };
        random_boxes[i] = Box2f{ .centre = random_vecs[i], .half_width = half_width, .half_height = half_height, };
    }
    random_data_generated = true;
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
    if (!random_data_generated) { // generate random data for tests
        genRandomData();
        var palette = try svg.RandomHslPalette.init(std.testing.allocator, palette_size);
        for (0.., palette.hsl_colours) |i, hsl| {
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
    // always set up a canvas for drawing images on
    test_canvas = try svg.Canvas.init(std.testing.allocator, canvas_width, canvas_height, canvas_scale);
}

fn deinitTesting() void {
    test_canvas.deinit();
}

test "encompassing balls" {
    try initTesting(true);
    defer deinitTesting();
    for (0..test_len / 10) |i| {
        const a = Ball2f{ .centre = random_vecs[i], .radius = @abs(0.5 * random_floats[i]) };
        const b = Ball2f{ .centre = random_vecs[i + 1], .radius = @abs(0.5 * random_floats[i + 1]) };
        const c = Ball2f.getEncompassingBall(a, b);
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

test "square tree" {
    try initTesting(false);
    defer deinitTesting();
    const base_num = 4;
    const tree_depth = 2;
    const TreeType = data.SquareTree(base_num, tree_depth, u16);
    var st = try TreeType.init(testing.allocator, 2, Vec2f{ 0, 0 }, 1.0);
    defer st.deinit();

    // check the random points are binned correctly
    const leaf_size = st.size_per_level[tree_depth - 1];
    const max_dist_expected = 0.5 * @sqrt(2 * (leaf_size * leaf_size));
    var max_dist_found: f32 = 0.0;
    for (0..test_len) |i| {
        const point = random_vecs[i];
        const index = st.getLeafIndexForPoint(point);
        const centre = try st.getNodeCentre(tree_depth - 1, index);
        const dist = core.norm(point - centre);
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

    // create an svg image depicting the grid structure of the tree
    var text_buff: [16]u8 = undefined;
    for (TreeType.reverse_levels) |lvl| {
        var style = solid_styles[(2 * lvl) % palette_size];
        style.stroke_width = @truncate(tree_depth - lvl);
        const font_size: u8 = 12 + 12 * (tree_depth - lvl);
        const lvl_end_index = TreeType.nodes_in_level[lvl];
        for (0..lvl_end_index) |i| {
            const index: TreeType.NodeIndex = @intCast(i);
            const node_origin = try st.getNodeOrigin(lvl, index);
            const node_centre = try st.getNodeCentre(lvl, index);
            const node_corner = try st.getNodeCorner(lvl, index);
            const node_label = try getNodeLabel(&text_buff, lvl, index);
            try test_canvas.addRectangle(testing.allocator, node_origin, node_corner, style);
            try test_canvas.addText(testing.allocator, node_centre, node_label, font_size, style.stroke_hsl);
        }
    }
    // find the overlapping bodies and add them to the image
    var overlap_buff: [benchmark_len / 8]TreeType.BodyIndex = undefined;
    var overlap_count: u32 = 0;
    for (0..test_len) |i| _ = try st.addBody(random_balls[i]);
    st.updateBounds();
    for (0..test_len) |i| {
        const query_body = random_balls[i];
        const overlaps_found = try st.getOverlappingBodies(&overlap_buff, query_body);
        if (overlaps_found.len == 1) { // self intersection
            try test_canvas.addCircle(testing.allocator, query_body.centre, query_body.radius, solid_styles[7]);
            continue;
        }
        // draw all overlaps in a different colour
        for (overlaps_found) |overlap_index| {
            const overlap_body = st.leaf_arrays[overlap_index.leaf_index].items[overlap_index.body_number];
            if (@reduce(.And, overlap_body.centre == query_body.centre)) continue; // assumed to be the same body
            try test_canvas.addCircle(testing.allocator, query_body.centre, query_body.radius, solid_styles[5]);
            try test_canvas.addCircle(testing.allocator, overlap_body.centre, overlap_body.radius, solid_styles[5]);
        }
        overlap_count += @truncate(overlaps_found.len);
    }

    var fname_buff: [128]u8 = undefined;
    const fpath = try fmt.bufPrint(&fname_buff, "{s}/square-tree-{}-{}.html", .{ out_dir_name, base_num, tree_depth });
    try test_canvas.writeHtml(testing.allocator, fpath, true);
}
