const std = @import("std");
const core = @import("core.zig");
const tree = @import("tree.zig");
const svg = @import("svg.zig");
const fmt = std.fmt;
const fs = std.fs;
const os = std.os;
const time = std.time;

const Vec2f = core.Vec2f;
const Vec3f = core.Vec3f;
const Vec4f = core.Vec4f;
const Vec6f = core.Vec6f;
const Vec8f = core.Vec8f;
const Vec12f = core.Vec12f;
const Vec16f = core.Vec16f;
const Ball2f = core.Ball2f;
const Box2f = core.Box2f;
const benchmark_len = 10000;
const test_len = 400;

var num_trials: u16 = 10;
var random_floats: [benchmark_len]f32 = undefined;
var random_vecs: [benchmark_len]Vec2f = undefined;
var random_balls: [benchmark_len]Ball2f = undefined;
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
    try benchmarkOverlap(allocator);
    std.debug.print("Benchmarks complete!\n", .{});
}

fn benchmarkOverlap(allocator: std.mem.Allocator) !void {
    const tree_depth = 3;
    const TreeType = tree.SquareTree(2, tree_depth, 1.0);
    const leaf_cap = 4 * benchmark_len / (TreeType.num_leaves + TreeType.num_parents);
    var qt = try TreeType.init(allocator, leaf_cap, Vec2f{ 0, 0 });
    defer qt.deinit();

    const t_0 = time.microTimestamp();
    for (random_balls) |ball| _ = try qt.addBody(ball);
    const t_1 = time.microTimestamp();
    qt.updateBounds();
    const t_2 = time.microTimestamp();
    var overlap_buff: [benchmark_len]TreeType.BodyIndex = undefined;
    var overlap_count: u32 = 0;
    for (random_balls) |b| {
        const overlaps_found = qt.getOverlappingBodies(&overlap_buff, b);
        overlap_count += @truncate(overlaps_found.len);
    }
    const t_3 = time.microTimestamp();

    std.debug.print(
        "Overlap benchmark: found {} overlaps in {} us. Time breakdown: adding {} us, updating {} us, overlapping {} us.\n",
        .{ overlap_count, t_3 - t_0, t_1 - t_0, t_2 - t_1, t_3 - t_2 },
    );
}

fn benchmarkIndexing(allocator: std.mem.Allocator) !void {
    const tree_depth = 4;
    const HexTree4 = tree.SquareTree(4, tree_depth, 1.0);
    var ht = try HexTree4.init(allocator, 8, Vec2f{ 0, 0 });
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
        random_vecs[i] = Vec2f{ random_floats[x_index], random_floats[y_index] };
        random_balls[i] = Ball2f{ .centre = random_vecs[i], .radius = 0.01 * random_floats[i] };
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
    }
    if (delete_out_dir) { // delete test-out and create a new copy
        var cwd = fs.cwd();
        std.debug.print("Creating fresh {s} directory...\n", .{out_dir_name});
        try cwd.deleteTree(out_dir_name);
        try cwd.makeDir(out_dir_name);
    }
    // always set up a canvas for drawing images on
    test_canvas = try svg.Canvas.init(std.testing.allocator, canvas_width, canvas_height, canvas_scale);
    var palette = try svg.RandomHslPalette.init(std.testing.allocator, palette_size);
    for (0.., palette.hsl_colours) |i, hsl| {
        dashed_styles[i] = svg.ShapeStyle{ .stroke_hsl = hsl, .stroke_dashed = true };
        solid_styles[i] = svg.ShapeStyle{ .stroke_hsl = hsl };
    }
    palette.deinit();
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

test "square tree" {
    try initTesting(false);
    defer deinitTesting();
    const base_num = 4;
    const tree_depth = 2;
    const TreeType = tree.SquareTree(base_num, tree_depth, 1.0);
    TreeType.printTypeInfo();
    var ht = try TreeType.init(testing.allocator, 8, Vec2f{ 0, 0 });
    defer ht.deinit();

    // check the random points are binned correctly
    const max_dist_expected = 0.5; // * @sqrt(2 * (0.0625 * 0.0625)); // to the centre of a leaf node
    var max_dist_found: f32 = 0.0;
    for (0..test_len) |i| {
        const point = random_vecs[i];
        const index = ht.getLeafIndexForPoint(point);
        const centre = ht.getNodeCentre(tree_depth - 1, index);
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
        var style = solid_styles[2 * lvl];
        style.stroke_width = @truncate(tree_depth - lvl);
        const font_size: u8 = 9 + 3 * (tree_depth - lvl);
        const lvl_end_index = TreeType.nodes_in_level[lvl];
        for (0..lvl_end_index) |i| {
            const index: TreeType.NodeIndex = @intCast(i);
            const node_origin = ht.getNodeOrigin(lvl, index);
            const node_centre = ht.getNodeCentre(lvl, index);
            const node_corner = ht.getNodeCorner(lvl, index);
            const node_label = try std.fmt.bufPrint(&text_buff, "{X}", .{index});
            try test_canvas.addRectangle(testing.allocator, node_origin, node_corner, style);
            try test_canvas.addText(testing.allocator, node_centre, node_label, font_size, style.stroke_hsl);
        }
    }
    // find the overlapping bodies and add them to the image
    var overlap_buff: [benchmark_len / 8]TreeType.BodyIndex = undefined;
    var overlap_count: u32 = 0;
    for (0..test_len) |i| _ = try ht.addBody(random_balls[i]);
    ht.updateBounds();
    for (0..test_len) |i| {
        const query_body = random_balls[i];
        const overlaps_found = ht.getOverlappingBodies(&overlap_buff, query_body);
        if (overlaps_found.len == 1) { // self intersection
            try test_canvas.addCircle(testing.allocator, query_body.centre, query_body.radius, solid_styles[1]);
            continue;
        }
        // draw all overlaps in a different colour
        for (overlaps_found) |overlap_index| {
            const overlap_body = ht.bodies[overlap_index.leaf_index].items[overlap_index.body_number];
            if (@reduce(.And, (overlap_body.centre == query_body.centre))) continue; // assumed to be the same body
            try test_canvas.addCircle(testing.allocator, query_body.centre, query_body.radius, solid_styles[6]);
            try test_canvas.addCircle(testing.allocator, overlap_body.centre, overlap_body.radius, solid_styles[6]);
        }
        overlap_count += @truncate(overlaps_found.len);
    }

    var fname_buff: [128]u8 = undefined;
    const fpath = try fmt.bufPrint(&fname_buff, "{s}/square-tree-{}-{}.html", .{ out_dir_name, base_num, tree_depth });
    try test_canvas.writeHtml(testing.allocator, fpath, true);
}
