# Code to define the environment room and rectangles used to define it
# maintained by {jmorton2,kmenda}@stanford.edu

@enum MapLayout baseline one_sided_corridor two_sided_corridor

# Define constants  -- all units in m
const RW = 5. # room width
mutable struct ROBOT_W_struct
    val::Float64 # robot width
end
const DEFAULT_ROBOT_W = 1.0
const ROBOT_W = ROBOT_W_struct(DEFAULT_ROBOT_W)

# Define rectangle type for constructing hallway
# corners: 4x2 np array specifying
#		   bottom-left, top-left,
#		   top-right, bottom-right corner
# walls: length 4 list of bools specifying
#		 if left, top, right, bottom sides are
#		 open (False) or walls (True)
mutable struct Rectangle
    corners::Array{Float64, 2}
    walls::Array{Bool, 1}
    segments::Array{LineSegment, 1}
    width::Float64
    height::Float64
    midpoint::SVec2
    area::Float64
    xl::Float64
    xu::Float64
    yl::Float64
    yu::Float64

    function Rectangle(
        corners::Array{Float64, 2},
        walls::Array{Bool, 1};
        goal_idx::Int=0,
        stair_idx::Int=0
        )

        retval = new()

        retval.corners = corners
        retval.walls = walls

        retval.width = corners[3, 1] - corners[2, 1]
        retval.height = corners[2, 2] - corners[1, 2]
        mean_vals = mean(corners, dims=1)
        retval.midpoint = SVec2(mean_vals[1, 1], mean_vals[1, 2])
        
        # compute area in which robot could be initialized
        retval.xl = corners[2, 1]
        retval.xu = corners[3, 1]
        retval.yl = corners[1, 2]
        retval.yu = corners[2, 2]
        if walls[1]
            retval.width -= ROBOT_W.val/2
            retval.xl += ROBOT_W.val/2
        end
        if walls[2]
            retval.height -= ROBOT_W.val/2
            retval.yu -= ROBOT_W.val/2
        end
        if walls[3]
            retval.width -= ROBOT_W.val/2
            retval.xu -= ROBOT_W.val/2
        end
        if walls[4]
            retval.height -= ROBOT_W.val/2
            retval.yl += ROBOT_W.val/2
        end
        @assert retval.width > 0.0 && retval.height > 0.0 "Negative width or height"
        retval.area = retval.width * retval.height
        
        retval.segments = [LineSegment(corners[i, :], corners[i+1, :], (goal_idx == i), (stair_idx == i)) for i =1:3 if walls[i]]
        if walls[4]
            push!(retval.segments, LineSegment(corners[1, :], corners[4, :], (goal_idx == 4), (stair_idx == 4)))
        end

        retval
    end
end

# Randomly initializes the robot in this rectangle
function init_pos(rect::Rectangle, rng)
    w = rect.xu - rect.xl
    h = rect.yu - rect.yl
    return SVec2(rand(rng)*w + rect.xl, rand(rng)*h + rect.yl)
end

# Determines if pos (center of robot) is within the rectangle
function in_rectangle(rect::Rectangle, pos::SVec2)
    corners = rect.corners
    xlims = SVec2(rect.xl - MARGIN, rect.xu + MARGIN)
    ylims = SVec2(rect.yl - MARGIN, rect.yu + MARGIN)
    if xlims[1] < pos[1] < xlims[2]
        if ylims[1] < pos[2] < ylims[2]
            return true
        end
    end
    return false
end

# determines if pos (center of robot) is intersecting with a wall
# returns: -2, -Inf if center of robot not in room
#          -1, -Inf if not in wall contact
#          0~3, violation mag, indicating which wall has contact
#          if multiple, returns largest violation
function wall_contact(rect::Rectangle, pos::SVec2)
    if !(in_rectangle(rect, pos))
        return -2, -Inf
    end
    corners = rect.corners
    xlims = SVec2(corners[2, 1], corners[3, 1])
    ylims = SVec2(corners[1, 2], corners[2, 2])

    contact = -1
    contact_mag = -Inf
    if pos[1] - ROBOT_W.val/2 <= xlims[1] + MARGIN && rect.walls[1]
        # in contact with left wall
        new_contact_mag = abs(pos[1] - ROBOT_W.val/2 - xlims[1])
        if new_contact_mag > contact_mag
            contact_mag = new_contact_mag
            contact = 1
        end
    end
    if pos[2] + ROBOT_W.val/2 + MARGIN >= ylims[2] && rect.walls[2]
        # in contact with top wall
        new_contact_mag = abs(pos[2] - ROBOT_W.val/2 - xlims[2])
        if new_contact_mag > contact_mag
            contact_mag = new_contact_mag
            contact = 2
        end
    end
    if pos[1] + ROBOT_W.val/2 + MARGIN >= xlims[2] && rect.walls[3]
        # in contact with right wall
        new_contact_mag = abs(pos[1] - ROBOT_W.val/2 - xlims[2])
        if new_contact_mag > contact_mag
            contact_mag = new_contact_mag
            contact = 3
        end
    end
    if pos[2] - ROBOT_W.val/2 <= ylims[1] + MARGIN && rect.walls[4]
        # in contact with bottom wall
        new_contact_mag = abs(pos[2] - ROBOT_W.val/2 - xlims[1])
        if new_contact_mag > contact_mag
            contact_mag = new_contact_mag
            contact = 4
        end
    end

    return contact, contact_mag
end

# Render rectangle based on segments
function render(rect::Rectangle, ctx::CairoContext)
    for seg in rect.segments
        render(seg, ctx)
    end
end

# generate consecutive rectangles that make up the room
# all rectangles share a full "wall" with an adjacent rectangle
# shared walls are not solid - just used to specify geometry
mutable struct Room
    rectangles::Array{Rectangle, 1}
    areas::Array{Float64, 1}
    goal_rect::Int  # Index of rectangle with goal state
    goal_wall::Int  # Index of wall that leads to goal
    stair_rect::Int # Index of rectangle with stairs
    stair_wall::Int # Index of wall that leads to stairs

    function Room(sspace; layout::MapLayout=baseline, config::Int=1, num_rooms::Int=4)

        retval = new()

        # Initialize array of rectangles
        rectangles = []

        if layout == one_sided_corridor
            wall_length = 10.0
            for i in 1:num_rooms
                corners = Float64[
                    (i-1)*wall_length  0.0;
                    (i-1)*wall_length  wall_length;
                    i*wall_length      wall_length;
                    i*wall_length      0.0
                ]
                goal_idx = (i == num_rooms) ? 3 : 0
                push!(rectangles, Rectangle(corners, [true, true, true, false],
                      goal_idx=goal_idx, stair_idx=0))
            end
            lower_corners = Float64[
                0.0                   -wall_length;
                0.0                    0.0;
                num_rooms*wall_length  0.0;
                num_rooms*wall_length -wall_length
            ]
            push!(rectangles, Rectangle(lower_corners, [true, false, true, true],
                  goal_idx=0, stair_idx=3))
            retval.goal_rect  = num_rooms
            retval.goal_wall  = 3
            retval.stair_rect = num_rooms + 1
            retval.stair_wall = 3

            retval.rectangles = rectangles
            retval.areas = [r.area for r in rectangles]
            return retval

        elseif layout == two_sided_corridor
            wall_length = 10.0
            # n upper rooms, n-1 lower rooms: asymmetry enables localization
            n_lower = num_rooms - 1
            # Upper rooms (n): open bottom (connects to corridor)
            for i in 1:num_rooms
                goal_idx = (i == num_rooms) ? 3 : 0
                push!(rectangles, Rectangle(Float64[
                    (i-1)*wall_length  wall_length;
                    (i-1)*wall_length  2*wall_length;
                    i*wall_length      2*wall_length;
                    i*wall_length      wall_length
                ], [true, true, true, false], goal_idx=goal_idx, stair_idx=0))
            end
            # Lower rooms (n-1): same width W as top rooms, but two leftmost are merged into one 2W room
            for i in 1:n_lower
                x_start = (i == 1) ? 0.0           : i * wall_length
                x_end   = (i == 1) ? 2*wall_length : (i + 1) * wall_length
                stair_idx = (i == n_lower) ? 3 : 0
                push!(rectangles, Rectangle(Float64[
                    x_start  -wall_length;
                    x_start   0.0;
                    x_end     0.0;
                    x_end    -wall_length
                ], [true, false, true, true], goal_idx=0, stair_idx=stair_idx))
            end
            # Central corridor: spans full upper width, open top and bottom, end walls only
            push!(rectangles, Rectangle(Float64[
                0.0                    0.0;
                0.0                    wall_length;
                num_rooms*wall_length  wall_length;
                num_rooms*wall_length  0.0
            ], [true, false, true, false], goal_idx=0, stair_idx=0))
            retval.goal_rect  = num_rooms           # last upper room
            retval.goal_wall  = 3
            retval.stair_rect = num_rooms + n_lower # last lower room
            retval.stair_wall = 3

            retval.rectangles = rectangles
            retval.areas = [r.area for r in rectangles]
            return retval
        end

        # Define different configurations for stair and goal locations (baseline)
        goal_idxs = [0, 0, 0, 0]
        stair_idxs = [0, 0, 0, 0]
        if config == 2
            retval.goal_rect = 1
            retval.goal_wall = 4
            retval.stair_rect = 2
            retval.stair_wall = 1
        elseif config == 3
            retval.goal_rect = 4
            retval.goal_wall = 3
            retval.stair_rect = 2
            retval.stair_wall = 1
        else
            retval.goal_rect = 4
            retval.goal_wall = 3
            retval.stair_rect = 4
            retval.stair_wall = 4
        end
        goal_idxs[retval.goal_rect] = retval.goal_wall
        stair_idxs[retval.stair_rect] = retval.stair_wall

        # Rectangle 1
        corners = round_corners(sspace,[[-20-RW -20]; [-20-RW 0-RW]; [-20+RW 0-RW]; [-20+RW -20]])
        walls = [true, false, true, true] # top wall shared
        push!(rectangles, Rectangle(corners, walls, goal_idx=goal_idxs[1], stair_idx=stair_idxs[1]))

        # Rectangle 2
        corners = round_corners(sspace,[[-20-RW 0-RW]; [-20-RW 0+RW]; [-20+RW 0+RW]; [-20+RW 0-RW]])
        walls = [true, true, false, false] # bottom, right wall shared
        push!(rectangles, Rectangle(corners, walls, goal_idx=goal_idxs[2], stair_idx=stair_idxs[2]))

        # Rectangle 3
        corners = round_corners(sspace,[[-20+RW 0-RW]; [-20+RW 0+RW]; [10 0+RW]; [10 0-RW]])
        walls = [false, true, false, true] # left wall shared
        push!(rectangles, Rectangle(corners, walls, goal_idx=goal_idxs[3], stair_idx=stair_idxs[3]))

        # Rectangle 4
        corners = round_corners(sspace,[[10 0-RW]; [10 0+RW]; [10+RW 0+RW]; [10+RW 0-RW]])
        walls = [false, true, true, true] # left wall shared
        push!(rectangles, Rectangle(corners, walls, goal_idx=goal_idxs[4], stair_idx=stair_idxs[4]))

        retval.rectangles = rectangles
        retval.areas = [r.area for r in rectangles]
        
        retval
    end
end

# Sample from multinomial distribution
# eventually this should be replaced with categorical
function multinomial_sample(p::AbstractVector{Float64}, rng::AbstractRNG)
    rand_num = rand(rng)
    for i = 1:length(p)
        if rand_num < sum(p[1:i])
            return i
        end
    end
end

# Initialize the robot randomly in the room
# Randomly select a rectangle weighted by initializable area
function init_pos(r::Room, rng::AbstractRNG)
    norm_areas = r.areas/sum(r.areas)
    rect = multinomial_sample(norm_areas, rng)
    return init_pos(r.rectangles[rect], rng)
end

# Determines if pos is in contact with a wall
# returns bool indicating contact
function wall_contact(r::Room, pos::SVec2)
    for (i, rect) in enumerate(r.rectangles)
        wc, _ = wall_contact(rect, pos)
        if wc >= 0
            return true
        end
    end
    return false
end

# Determines if pos is in contact with a specific wall
# returns true if true
function contact_wall(r::Rectangle, wall::Int, pos::SVec2)
    wc,_ = wall_contact(r, pos)
    return wc == wall
end    

# Determines if pos (center of robot) is within the room
function in_room(r::Room, pos::SVec2)
    for rect in r.rectangles
        if in_rectangle(rect, pos)
            return true
        end
    end
    return false
end 

# Attempts to translate from pos0 in direction heading for des_step without violating boundaries
function legal_translate(r::Room, pos0::SVec2, heading::SVec2, des_step::Float64)
    if des_step == 0.0
        return pos0
    end

    R = ROBOT_W.val/2
    pos1 = pos0 + des_step * heading + R * sign.(heading)

    fs = des_step
    for rect in r.rectangles
        for seg in rect.segments
            if (seg.p1[1] == seg.p2[1]) ?
                (pos0[1] - seg.p1[1]) * (pos1[1] - seg.p2[1]) > 0 :
                (pos0[2] - seg.p1[2]) * (pos1[2] - seg.p2[2]) > 0 
                continue
            end
            new_fs = furthest_step(seg, pos0, heading, R)
            if new_fs < fs
                fs = new_fs
            end
        end
    end
    pos1 = pos0 + fs * heading
    if !in_room(r, pos1)
        return pos0
    else
        return pos1
    end
end

# Maximum step the robot center can take from pos0 in direction heading
# before the robot body (radius ROBOT_W.val/2) contacts any wall.
# Correctly handles wall corners by checking endpoint circles via furthest_step.
# Returns 0.0 if the robot is already at or past a wall (e.g. due to noise).
function max_safe_step(r::Room, pos0::SVec2, heading::SVec2)::Float64
    R    = ROBOT_W.val / 2
    # Build a far lookahead point to use as a direction filter (same logic as
    # legal_translate). Segments where both pos0 and pos1 lie on the same side
    # are behind or far outside the ray and must be skipped — otherwise
    # furthest_step returns 0 for endpoints behind the robot (via clamped
    # negative circle intersections), which would wrongly zero out max_safe_step.
    pos1 = pos0 + 1000.0 * heading + R * sign.(heading)
    fs   = Inf
    for rect in r.rectangles
        for seg in rect.segments
            if (seg.p1[1] == seg.p2[1]) ?
                (pos0[1] - seg.p1[1]) * (pos1[1] - seg.p2[1]) > 0 :
                (pos0[2] - seg.p1[2]) * (pos1[2] - seg.p2[2]) > 0
                continue
            end
            new_fs = furthest_step(seg, pos0, heading, R)
            if new_fs < fs
                fs = new_fs
            end
        end
    end
    return max(0.0, fs)
end

# computes the length of a ray from robot center to closest segment
# from p0 pointing in direction heading
# inputs: p0: array specifying initial point
#         heading: array specifying heading unit vector
#         R: robot radius [m]
# outputs: ray_length [m]
function ray_length(r::Room, pos0::SVec2, heading::SVec2)
    rl = Inf
    pos1 = Inf .* heading
    for rect in r.rectangles
        for seg in rect.segments
            if (seg.p1[1] == seg.p2[1]) ?
                (pos0[1] - seg.p1[1]) * (pos1[1] - seg.p2[1]) > 0 :
                (pos0[2] - seg.p1[2]) * (pos1[2] - seg.p2[2]) > 0 
                continue
            end
            new_rl = ray_length(seg, pos0, heading)
            if new_rl < rl
                rl = new_rl
            end
        end
    end
    return rl
end

# Render room based on individual rectangles
function render(r::Room, ctx::CairoContext)
    for rect in r.rectangles
        render(rect, ctx)
    end
end
