const std = @import("std");
const core = @import("core.zig");
const tree = @import("tree.zig");
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
var num_trials: u16 = 10;
var random_floats: [random_len]f32 = undefined;
var random_vecs: [random_len]Vec2f = undefined;
var random_balls: [random_len]Ball2f = undefined;
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
    std.debug.print("Running all benchmarks for {} bodies...\n", .{random_len});
    try benchmarkTreeIndexing(allocator);
    try benchmarkOverlap(allocator);
    std.debug.print("Benchmarks complete!\n", .{});
}

fn benchmarkOverlap(allocator: std.mem.Allocator) !void {
    const tree_depth = 3;
    const TreeType = tree.SquareTree(2, tree_depth, 1.0);
    const leaf_cap = 4 * random_len / (TreeType.num_leaves + TreeType.num_parents);
    var qt = try TreeType.init(allocator, leaf_cap, Vec2f{ 0, 0 });
    defer qt.deinit();

    const t_0 = time.microTimestamp();
    for (0..random_len) |i| {
        const ball = Ball2f{ .centre = random_vecs[i], .radius = 0.001 * random_floats[i] };
        _ = try qt.addBody(ball);
    }
    const t_1 = time.microTimestamp();
    qt.updateBounds();
    const t_2 = time.microTimestamp();
    var overlap_buff: [random_len]TreeType.index_type = undefined;
    var overlap_count: u32 = 0;
    for (random_balls) |b| {
        const overlaps_found = qt.getOverlappingIndexes(&overlap_buff, b);
        overlap_count += @truncate(overlaps_found.len);
    }
    const t_3 = time.microTimestamp();
    std.debug.print(
        "Found {} overlaps in {} us; time breakdown:\n",
        .{ overlap_count, t_3 - t_0 },
    );
    std.debug.print(
        "Adding took {} us; updating took {} us; overlap check took {} us.\n",
        .{ t_1 - t_0, t_2 - t_1, t_3 - t_2 },
    );
}

fn benchmarkTreeIndexing(allocator: std.mem.Allocator) !void {
    std.debug.print("Starting square-tree benchmark...\n", .{});
    const tree_depth = 4;
    const HexTree4 = tree.SquareTree(4, tree_depth, 1.0);
    var ht = try HexTree4.init(allocator, 8, Vec2f{ 0, 0 });
    defer ht.deinit();
    var ln_sum_1: usize = 0;

    const t_0 = time.microTimestamp();
    for (0..random_len) |i| {
        const ln_1 = ht.getLeafIndexForPoint(random_vecs[i]);
        ln_sum_1 += ln_1;
    }
    const t_1 = time.microTimestamp();

    const any_diff = ln_sum_1 != 0; // TODO: change number
    ln_sum_1 -= random_len * HexTree4.num_parents;
    std.debug.print(
        "1. Benchmark results for path-to-point ({} points):\nTook {} us, differences = {}, ln-sum = {}.\n",
        .{ random_len, t_1 - t_0, any_diff, ln_sum_1 },
    );
}

fn genRandomData() void {
    if (random_data_generated) return;
    std.debug.print("Generating {} random floats...\n", .{random_len});
    for (0..random_len) |i| random_floats[i] = core.getRandUniform(0, 1);
    for (0..random_len) |i| {
        const x_index = (i + 2) % random_len;
        const y_index = (i + 7) % random_len;
        random_vecs[i] = Vec2f{ random_floats[x_index], random_floats[y_index] };
        random_balls[i] = Ball2f{ .centre = random_vecs[i], .radius = 0.001 * random_floats[i] };
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

    genRandomData(); // generate random data for tests
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

test "hex tree" {
    try initTesting(false);
    defer deinitTesting();
    const tree_depth = 3;
    const HexTree3 = tree.SquareTree(4, tree_depth, 1.0);
    HexTree3.printTypeInfo();
    var ht = try HexTree3.init(testing.allocator, 8, Vec2f{ 0, 0 });
    defer ht.deinit();

    // check the random points are binned correctly
    const max_dist_expected = 0.5 * @sqrt(2 * (0.015625 * 0.015625)); // to the centre of a leaf node
    var max_dist_found: f32 = 0.0;
    for (0..random_len) |i| {
        const point = random_vecs[i];
        const index = ht.getLeafIndexForPoint(point);
        const centre = ht.getNodeCentre(tree_depth, index);
        const dist = core.norm(point - centre);
        if (i < 8 or dist > max_dist_expected) { // print details of the first few points, or any that are far from their leaf node's centre
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

    // create an svg image depicting the tree
    var text_buff: [16]u8 = undefined;
    for (HexTree3.reverse_levels) |lvl| {
        var style = solid_styles[2 * lvl];
        style.stroke_width = @truncate(tree_depth - lvl);
        const font_size: u8 = 9 + 3 * (tree_depth - lvl);
        const lvl_end_index = HexTree3.nodes_in_level[lvl];
        for (0..lvl_end_index) |i| {
            // TODO: add some way of retrieving points for non-leaf nodes to get this working again
            const index = HexTree3.getPredeccessorIndex(lvl, @intCast(i));
            const node_origin = ht.getNodeOrigin(lvl, index);
            const node_centre = ht.getNodeCentre(lvl, index);
            const node_corner = ht.getNodeCorner(lvl, index);
            const node_label = try std.fmt.bufPrint(&text_buff, "{X}", .{index});
            try test_canvas.addRectangle(testing.allocator, node_origin, node_corner, style);
            try test_canvas.addText(testing.allocator, node_centre, node_label, font_size, style.stroke_hsl);
        }
    }
    var fname_buff: [128]u8 = undefined;
    const fpath = try fmt.bufPrint(&fname_buff, "{s}/hex-tree-{}.html", .{ out_dir_name, tree_depth });
    try test_canvas.writeHtml(testing.allocator, fpath, true);
}
