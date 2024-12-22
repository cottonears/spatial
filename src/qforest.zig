const std = @import("std");
const m = @import("m.zig");
const spatial = @import("spatial.zig");
const canvas = @import("canvas.zig");
const math = std.math;
const mem = std.mem;
const Vec2f = m.Vec2f;
const Vec4f = m.Vec4f;

const Body = spatial.Body;

// TODO:
// 1. Get the spatial structure working for a single quad-tree
// 2. Implement tree-sizing based on global speed limit
// 3. Figure out a plan for indexing in the tree
//

pub const LeafBucket = struct {
    bodies: std.ArrayList(Body) = undefined, // IDEA: try using a MultiArrayList here and compare performance
    bball_centre: Vec2f,
    bball_radius: f32 = 0,
    // bbox_width: f32 = 0.0, TODO: add these back in
    // bbox_height: f32 = 0.0,
    bounds_require_update: bool = false,
    const Self = @This();

    pub fn init(allocator: mem.Allocator, centre: Vec2f, capacity: u16) !Self {
        const bodies = try std.ArrayList(Body).initCapacity(allocator, capacity);
        return .{
            .bball_centre = centre,
            .bodies = bodies,
        };
    }

    pub fn deinit(self: *Self) void {
        self.bodies.deinit();
    }

    pub fn length(self: Self) u16 {
        return @intCast(self.bodies.items.len);
    }

    // IDEA:
    // When bodies are added to a leaf node we can do a cheap update of the bounding radius by just performing one expansion
    // If a body is removed, we may need to recompute the bounding ball from scratch.
    // However, in many cases recomputation can be avoided by checking if the removed ball was at the perimiter of the bounding ball.
    // On second thoughts, this might be a lot of effort for very little gain - maybe shelve it for now.
    // TODO: should return an index for the newly-added body
    pub fn addBody(self: *Self, b: Body) !void {
        // TODO: record a log of capacity expansions
        try self.bodies.append(b);
        self.bounds_require_update = true;
    }

    // TODO: add this once indexing has been figured out
    //pub fn removeBody(id : u32) void {}

    pub fn updateBounds(self: *Self) void {
        for (self.bodies.items) |b| {
            // IDEA: figuring out the encompassing ball is a little complicated
            //       It might be more efficient to update the bounding box in the loop
            //       Then we can fit the encompassing ball based on that afterwards
            const c = spatial.getEncompassingBall(self.bball_centre, self.bball_radius, b.position, b.radius);
            self.bball_centre = Vec2f{ c[0], c[1] };
            self.bball_radius = c[2];
        }
        self.bounds_require_update = false;
    }

    // pub fn getBody(self: Self, index: u32) !void {}

    // pub fn removeBodyWithIndex(self: Self, index: u32) !void {
    //  need to think about how to keep max_radius + max_velocity_squared updated
    // }

    pub fn clear(self: *Self) !void {
        try self.bodies.resize(0);
        self.bounding_radius = 0;
    }

    // checks if any overlap is possible
    pub fn isOverlapPossible(self: Self, centre: Vec2f, radius: f32) bool {
        return spatial.checkBallsOverlap(self.bball_centre, self.bball_radius, centre, radius);
    }

    // TODO: see if performance is significantly affected when we use a bounded array or plain slice instead of a list
    pub fn getPotentialOverlapsForBody(self: Self, list: *std.ArrayList(Body), a: Body) !void {
        for (self.bodies.items) |b| {
            if (a.id == b.id) continue;
            if (m.norm(a.position - b.position) < a.radius + b.radius) try list.append(b);
        }
    }

    pub fn getPotentialOverlapsAtPoint(self: Self, list: *std.ArrayList(Body), pos: Vec2f, radius: f32) !void {
        for (self.bodies.items) |b| {
            if (m.norm(pos - b.position) < radius + b.radius) try list.append(b);
        }
    }
};

// IDEA: can we make each level of these nodes contiguous in memory?
// Or maybe its better to ensure just the 4 children of a given
//  we could also use indexing to identify children, rather than pointers (will save memory)
// spatial node with 4 children
pub const QuadNode = struct {
    const ChildUnion = union(enum) { QuadNodes: [4]*QuadNode, BodyBuckets: [4]*LeafBucket };
    const Self = @This();
    bball_centre: Vec2f,
    bball_radius: f32 = 0,
    child_ptrs: ChildUnion = undefined,
    // IDEA: might be useful to consider having a bounding box in addition to, or in place of, the bounding ball
    // bounding_width: f32,
    // bounding_height: f32,
    // TODO: add velocity at some point for continuous collision detection

    // iterates (depth first) through children and updates their bounding volume info, updates this Node's bounds info if necessary.
    // returns true if this node's bounds were updated, false otherwise
    pub fn updateBounds(self: *Self) void {
        var max_child_radius: f32 = 0.0;
        switch (self.child_ptrs) {
            .QuadNodes => |quads| {
                for (quads) |q| {
                    q.updateBounds();
                    max_child_radius = @max(max_child_radius, q.bball_radius);
                }
            },
            .BodyBuckets => |buckets| {
                for (buckets) |b| {
                    b.updateBounds();
                    max_child_radius = @max(max_child_radius, b.bball_radius);
                }
            },
        }
    }

    pub fn drawOnCanvas(self: Self, svg_canvas: *canvas.SvgCanvas) !void {
        // TODO: implement
        _ = self;
        _ = svg_canvas;
    }

    pub fn mayOverlapBall(self: Self, centre: Vec2f, radius: f32, start_index: u32) bool {
        _ = start_index; // TODO: add this optimisation
        return spatial.checkBallsOverlap(self.bball_centre, self.bball_radius, centre, radius);
    }

    // gets the indexes of bodies that overlap with a ball
    pub fn appendOverlapIndexesBall(self: Self, indexes: *std.ArrayList(u32), start_index: u32, centre: Vec2f, radius: f32) !void {
        _ = start_index;
        switch (self.child_ptrs) {
            .QuadNodes => |quad_children| {
                _ = quad_children;
                // for (quad_children) |quad| {
                //     _ = quad.getOverlapIndexesBall(centre, indexes, radius, start_index);
                // }
            },
            .BodyBuckets => |bucket_children| {
                for (bucket_children) |bucket| {
                    if (!bucket.isOverlapPossible(centre, radius)) continue;
                    for (bucket.bodies.items, 0..bucket.bodies.items.len) |b, i| {
                        if (spatial.checkBallsOverlap(b.position, b.radius, centre, radius)) {
                            try indexes.append(@intCast(i));
                        }
                    }
                }
            },
        }
    }
};

const testing = std.testing;

test "quad node 1" {
    var overlap_indexes = try std.ArrayList(u32).initCapacity(testing.allocator, 20);
    defer overlap_indexes.deinit();

    const a = Body{
        .id = 0,
        .position = Vec2f{ 1.0, 0.0 },
        .radius = 1.0,
    };
    const b = Body{
        .id = 1,
        .position = Vec2f{ 2.0, 2.0 },
        .radius = 0.8,
    };

    var buckets: [4]LeafBucket = undefined;
    buckets[0] = try LeafBucket.init(std.testing.allocator, Vec2f{ -5, 5 }, 10);
    buckets[1] = try LeafBucket.init(std.testing.allocator, Vec2f{ 5, 5 }, 10);
    buckets[2] = try LeafBucket.init(std.testing.allocator, Vec2f{ -5, -5 }, 10);
    buckets[3] = try LeafBucket.init(std.testing.allocator, Vec2f{ 5, 5 }, 10);
    defer for (0..4) |i| buckets[i].deinit();
    var qn = QuadNode{
        .bball_centre = m.zero_2f,
        .child_ptrs = QuadNode.ChildUnion{ .BodyBuckets = [4]*LeafBucket{
            &buckets[0],
            &buckets[1],
            &buckets[2],
            &buckets[3],
        } },
    };

    try buckets[1].addBody(a);
    try buckets[1].addBody(b);
    qn.updateBounds();

    const centre_1 = Vec2f{ 3.0, 3.0 };
    const radius_1 = 1.0;
    const may_overlap_1 = qn.mayOverlapBall(centre_1, radius_1, 0);
    try qn.appendOverlapIndexesBall(&overlap_indexes, 0, centre_1, radius_1);
    try testing.expectEqual(true, may_overlap_1);
    try testing.expectEqual(1, overlap_indexes.items.len);

    // const test_pt_2 = Body{ .id = 8, .position = Vec2f{ 3.5, 3.5 }, .radius = 0.5 };
    // try qn.fillWithPotentialOverlaps(&overlap_list, test_pt_2);
    // try testing.expectEqual(0, overlap_list.items.len);

    // try qn.fillWithPotentialOverlaps(&overlap_list, b);
    // try testing.expectEqual(0, overlap_list.items.len);

    //qn.updateChildBounds();
    //my_node.updateChildBounds()

    std.debug.print("quad-node = {any}\n", .{qn});
}
