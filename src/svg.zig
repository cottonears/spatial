const std = @import("std");
const mem = std.mem;
const Vec2f = @Vector(2, f32);

const DEFAULT_HUE_RANGE = [_]u9{ 0, 360 };
const DEFAULT_SAT_RANGE = [_]u9{ 50, 70 };
const DEFAULT_LT_RANGE = [_]u9{ 40, 60 };

// TODO: the following would be nice:
// - layers with default colouring + styles
// - automatic scaling and offset to fit contents (would require storing elements in some intermediate representation)
/// A struct that can be used to compose images from basic shapes. Useful for testing.
pub const Canvas = struct {
    height: f32,
    width: f32,
    scale: f32,
    offset: Vec2f,
    str: std.ArrayList(u8),
    //palette: RandomHslPalette = undefined,
    const Self = @This();

    pub fn init(allocator: mem.Allocator, width: f32, height: f32, scale: f32) !Self {
        return .{
            .height = height,
            .width = width,
            .scale = scale,
            .offset = Vec2f{ 0.5 * width, 0.5 * height },
            .str = try std.ArrayList(u8).initCapacity(allocator, 10000),
            //.palette = RandomHslPalette.initDefault(),
        };
    }

    pub fn deinit(self: *Self) void {
        self.str.deinit();
    }

    fn getCanvasCoords(self: Self, v: Vec2f) Vec2f {
        return self.offset + Vec2f{ self.scale * v[0], self.scale * v[1] };
    }

    pub fn addLine(self: *Self, allocator: mem.Allocator, start: Vec2f, end: Vec2f, style: ShapeStyle) !void {
        const start_c = self.getCanvasCoords(start);
        const end_c = self.getCanvasCoords(end);
        var sbuff: [256]u8 = undefined;
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<line x1=\"{d:.3}\" y1=\"{d:.3}\" x2=\"{d:.3}\" y2=\"{d:.3}\" {s}/>",
            .{ start_c[0], start_c[1], end_c[0], end_c[1], try style.getElementString(&sbuff) },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(element_str);
    }

    pub fn addRectangle(self: *Self, allocator: mem.Allocator, start: Vec2f, end: Vec2f, style: ShapeStyle) !void {
        const start_c = self.getCanvasCoords(start);
        const diff_c = self.getCanvasCoords(end - start);
        var sbuff: [256]u8 = undefined;
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<rect x=\"{d:.3}\" y=\"{d:.3}\" width=\"{d:.3}\" height=\"{d:.3}\" {s}/>",
            .{ start_c[0], start_c[1], diff_c[0], diff_c[1], try style.getElementString(&sbuff) },
        );
        defer allocator.free(element_str);
        //std.debug.print("appending rectangle to canvas:\n {s}\n", .{element_str});
        try self.str.appendSlice(element_str);
    }

    pub fn addCircle(self: *Self, allocator: mem.Allocator, centre: Vec2f, radius: f32, style: ShapeStyle) !void {
        const centre_c = self.getCanvasCoords(centre);
        const radius_c = self.scale * radius;
        var sbuff: [256]u8 = undefined;
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<circle cx=\"{d:.3}\" cy=\"{d:.3}\" r=\"{d:.3}\" {s}/>",
            .{ centre_c[0], centre_c[1], radius_c, try style.getElementString(&sbuff) },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(element_str);
    }

    pub fn addPolygon(self: *Self, allocator: mem.Allocator, points: []Vec2f, style: ShapeStyle) !void {
        var pts_list = std.ArrayList(u8).init(allocator);
        defer pts_list.deinit();
        for (points) |p| {
            const p_c = self.getCanvasCoords(p);
            var buff: [32]u8 = undefined;
            const slice = buff[0..];
            const str = try std.fmt.bufPrint(slice, "{d:.3},{d:.3} ", .{ p_c[0], p_c[1] });
            try pts_list.appendSlice(str);
        }
        var sbuff: [256]u8 = undefined;
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<polygon points=\"{s}\" {s}/>",
            .{ pts_list.items[0..], try style.getElementString(&sbuff) },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(element_str);
    }

    pub fn addText(self: *Self, allocator: mem.Allocator, centre: Vec2f, text: []const u8, font_size: u8, fill_hsl: [3]u9) !void {
        const centre_c = self.getCanvasCoords(centre);
        var sbuff: [128]u8 = undefined;
        const style_str = try std.fmt.bufPrint(
            &sbuff,
            "font-size=\"{d}\" fill=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" text-anchor=\"middle\"",
            .{ font_size, fill_hsl[0], fill_hsl[1], fill_hsl[2] },
        );
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<text x=\"{d:.3}\" y=\"{d:.3}\" {s}>{s}</text>",
            .{ centre_c[0], centre_c[1], style_str, text },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(element_str);
    }

    // caller owns the returned memory
    pub fn getSvg(self: Self, allocator: mem.Allocator) ![]u8 {
        const svg_start = try std.fmt.allocPrint(
            allocator,
            "<svg width=\"{d:.3}\" height=\"{d:.3}\" xmlns=\"http://www.w3.org/2000/svg\">",
            .{ self.width, self.height },
        );
        defer allocator.free(svg_start);

        var text = std.ArrayList(u8).init(allocator);
        try text.appendSlice(svg_start);
        try text.appendSlice(self.str.items);
        try text.appendSlice("\n</svg>");

        return text.toOwnedSlice();
    }

    pub fn writeHtml(self: *Self, allocator: mem.Allocator, filename: []const u8, clear: bool) !void {
        const html_start = "<!DOCTYPE html>\n<html><body>";
        const html_end = "</body></html>";
        const svg_body = try self.getSvg(allocator);
        defer testing.allocator.free(svg_body);
        var file = try std.fs.cwd().createFile(filename, .{});
        defer file.close();
        var buf_writer = std.io.bufferedWriter(file.writer());
        const writer = buf_writer.writer();
        try writer.print("{s}\n{s}\n{s}", .{ html_start, svg_body, html_end });
        try buf_writer.flush();
        if (clear) try self.str.resize(0);
    }
};

pub const ShapeStyle = struct {
    fill_active: bool = false,
    fill_hsl: [3]u9 = undefined,
    fill_opacity: f32 = 1.0,
    stroke_active: bool = true,
    stroke_hsl: [3]u9 = undefined,
    stroke_width: u4 = 2,
    stroke_opacity: f32 = 1.0,
    stroke_dashed: bool = false,

    pub fn getElementString(style: ShapeStyle, buff_ptr: *[256]u8) ![]u8 {
        var len: usize = 0;
        if (style.fill_active) {
            len = (try std.fmt.bufPrint(
                buff_ptr,
                "fill=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" ",
                .{ style.fill_hsl[0], style.fill_hsl[1], style.fill_hsl[2] },
            )).len;
        } else {
            len = (try std.fmt.bufPrint(
                buff_ptr,
                "fill=\"none\" ",
                .{},
            )).len;
        }
        if (style.stroke_active) {
            len += (try std.fmt.bufPrint(
                buff_ptr[len..],
                "stroke=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" stroke-width=\"{}\" ",
                .{ style.stroke_hsl[0], style.stroke_hsl[1], style.stroke_hsl[2], style.stroke_width },
            )).len;

            if (style.stroke_opacity < 1.0) {
                len += (try std.fmt.bufPrint(
                    buff_ptr[len..],
                    "stroke-opacity=\"{d:.3}\" ",
                    .{style.stroke_opacity},
                )).len;
            }
            if (style.stroke_dashed) {
                len += (try std.fmt.bufPrint(
                    buff_ptr[len..],
                    "stroke-dasharray=\"{},{}\" ",
                    .{ 2 * style.stroke_width, 2 * style.stroke_width },
                )).len;
            }
        }
        return buff_ptr[0..len];
    }
};

pub const RandomHslPalette = struct {
    h_min: u9 = 0,
    h_max: u9 = 360,
    s_min: u9 = 0,
    s_max: u9 = 100,
    l_min: u9 = 0,
    l_max: u9 = 100,
    allocator: mem.Allocator,
    rng: std.rand.DefaultPrng,
    hsl_colours: [][3]u9 = undefined,
    const Self = @This();

    pub fn init(allocator: mem.Allocator, num_colours: u8) !Self {
        var pal = Self{
            .h_min = DEFAULT_HUE_RANGE[0],
            .h_max = DEFAULT_HUE_RANGE[1],
            .s_min = DEFAULT_SAT_RANGE[0],
            .s_max = DEFAULT_SAT_RANGE[1],
            .l_min = DEFAULT_LT_RANGE[0],
            .l_max = DEFAULT_LT_RANGE[1],
            .allocator = allocator,
            .rng = std.rand.DefaultPrng.init(0),
        };
        pal.hsl_colours = try allocator.alloc([3]u9, num_colours);
        pal.fillWithRandomColours(pal.hsl_colours);
        return pal;
    }

    pub fn deinit(self: *Self) void {
        self.allocator.free(self.hsl_colours);
    }

    pub fn getRandomColour(self: *Self) [3]u9 {
        const h = self.h_min + self.rng.random().int(u9) % (self.h_max - self.h_min);
        const s = self.s_min + self.rng.random().int(u9) % (self.s_max - self.s_min);
        const l = self.l_min + self.rng.random().int(u9) % (self.l_max - self.l_min);
        return [_]u9{ h, s, l };
    }

    /// Fills the provided slice with random colours.
    /// The colours will be evenly spaced within the hue range, but have the same lightness + saturation.
    pub fn fillWithRandomColours(self: *Self, col_slice: [][3]u9) void {
        const hue_range = self.h_max - self.h_min;
        var col = self.getRandomColour();
        for (0..col_slice.len) |i| {
            const hue_inc = @as(u9, @intCast(i *% hue_range / col_slice.len));
            col[0] = self.h_min + (col[0] +% hue_inc) % hue_range;
            col_slice[i] = col;
        }
    }
};

// testing code
const testing = std.testing;
const canvas_width: f32 = 800;
const canvas_height: f32 = 600;
const gen_debug_output = true;

test "random colours" {
    var pal = try RandomHslPalette.init(std.testing.allocator, 4);
    defer pal.deinit();
    for (0..4) |i| {
        const current_hsl = pal.hsl_colours[i];
        const next_hsl = pal.hsl_colours[(i + 1) % 4];
        try testing.expect(current_hsl[0] != next_hsl[0]); // hue shifted
        try testing.expect(current_hsl[1] == next_hsl[1]); // unchanged
        try testing.expect(current_hsl[2] == next_hsl[2]); // unchanged
    }
}

test "paint" { // creates a canvas and paint it
    var points = [_]Vec2f{
        [_]f32{ 0, 100 },
        [_]f32{ 200, 300 },
        [_]f32{ 100, 150 },
    };
    var canvas = try Canvas.init(testing.allocator, canvas_width, canvas_height, 1.0);
    defer canvas.deinit();
    var pal = try RandomHslPalette.init(std.testing.allocator, 4);
    defer pal.deinit();

    const style_0 = ShapeStyle{ .stroke_hsl = pal.hsl_colours[0], .stroke_dashed = true };
    const style_1 = ShapeStyle{ .stroke_hsl = pal.hsl_colours[1] };
    const style_2 = ShapeStyle{ .stroke_hsl = pal.hsl_colours[2] };
    const style_3 = ShapeStyle{ .stroke_hsl = pal.hsl_colours[3] };
    try canvas.addRectangle(testing.allocator, [_]f32{ -200, 0 }, [_]f32{ -200, 200 }, style_0);
    try canvas.addPolygon(testing.allocator, points[0..], style_1);
    try canvas.addCircle(testing.allocator, [_]f32{ 0, 200 }, 10.0, style_2);
    try canvas.addLine(testing.allocator, [_]f32{ -200, -100 }, [_]f32{ 200, 300 }, style_3);
    if (gen_debug_output) try canvas.writeHtml(testing.allocator, "test-out/svg-paint.html", true);
}
