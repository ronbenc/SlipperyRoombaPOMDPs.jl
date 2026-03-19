"""
Visualizes how BananaStateDistribution is shaped by wall interactions.
Six scenarios: open baseline, run-through-wall, oblique approach,
parallel-near-wall glide, L-corner bypass, and tight corridor.

The red arrow points to the unconstrained desired next position (ignoring walls).
The red star marks that desired position. Samples show where the robot actually lands.

Run from repo root:
    julia --project=SlipperyRoombaPOMDPs SlipperyRoombaPOMDPs/wall_interaction_vis.jl
"""

include("src/RoombaPOMDPs.jl")
using .RoombaPOMDPs
using POMDPs, Distributions, Random, Plots

const N    = 3000
const rng  = MersenneTwister(42)
const DT   = 0.5
const VMAX = Inf

const THETA_STD = 0.05
const TRANS_COE = 0.8
const NOISE_STR = "(θ_std=$(THETA_STD), trans=$(TRANS_COE))"

mdp  = RoombaMDP(theta_std=THETA_STD, trans_noise_coeff=TRANS_COE, v_max=Inf, om_max=Inf)
room = mdp.room

function draw_walls!(plt, room)
    for rect in room.rectangles
        for seg in rect.segments
            plot!(plt, [seg.p1[1], seg.p2[1]], [seg.p1[2], seg.p2[2]];
                color     = :black,
                linewidth = 2.5,
                label     = "")
        end
    end
end

# Unconstrained desired next position — where the robot wants to go ignoring walls.
# Arrow + star at this point show the intent; samples show what actually happens.
function desired_next(s::RoombaState, a::RoombaAct)
    v      = clamp(a.v, 0.0, VMAX)
    th_new = s.theta + a.omega * DT
    step   = v * DT
    return s.x + step * cos(th_new), s.y + step * sin(th_new)
end

scenarios = [
    (
        title = "1: Radius gap $NOISE_STR\n(1 m from wall, ray_length=1.0 > max_safe_step=0.5)",
        s     = RoombaState(0.0, 4.0, 0.0, 0.0),
        a     = RoombaAct(2.0, π),
        xlims = (-3.0, 3.0),
        ylims = (2.5, 6.5),
    ),
    (
        title = "2: Run through wall $NOISE_STR\n(desired lands 0.2 m past top wall, max_safe_step=0.3)",
        s     = RoombaState(0.0, 4.2, 0.0, 0.0),
        a     = RoombaAct(2.0, π),
        xlims = (-2.5, 2.5),
        ylims = (2.5, 7.0),
    ),
    (
        title = "3: Oblique approach $NOISE_STR\n(rotate to π/3 toward top wall — asymmetric clip)",
        s     = RoombaState(5.0, 2.0, 0.0, 0.0),
        a     = RoombaAct(2.0, 2π/3),
        xlims = (2.0, 10.0),
        ylims = (0.0, 6.5),
    ),
    (
        title = "4: Parallel glide near wall $NOISE_STR\n(east, 1.2 m below top wall — lopsided)",
        s     = RoombaState(-3.0, 3.8, 0.0, 0.0),
        a     = RoombaAct(2.0, 0.0),
        xlims = (-6.0, 4.0),
        ylims = (1.5, 6.5),
    ),
    (
        title = "5: L-corner bypass $NOISE_STR\n(rotate to π/4 NE toward corner at (-15,-5) — split)",
        s     = RoombaState(-16.5, -7.0, 0.0, 0.0),
        a     = RoombaAct(2.0, π/2),
        xlims = (-22.0, -10.0),
        ylims = (-12.0, -2.0),
    ),
    (
        title = "6: Tight corridor (R4) $NOISE_STR\n(rotate north, 5 m wide — both walls clip)",
        s     = RoombaState(12.5, -1.0, 0.0, 0.0),
        a     = RoombaAct(2.0, π),
        xlims = (8.0, 17.0),
        ylims = (-4.0, 6.0),
    ),
    (
        title = "7: L-corner run-through $NOISE_STR\n(rotate to π/4, desired past right wall)",
        s     = RoombaState(-16.5, -6.0, 0.0, 0.0),
        a     = RoombaAct(7.0, π/2),
        xlims = (-22.0, -10.0),
        ylims = (-12.0, -2.0),
    ),
]

plots_list = map(scenarios) do sc
    d  = POMDPs.transition(mdp, sc.s, sc.a)
    xs = Float64[]
    ys = Float64[]
    for _ in 1:N
        sp = rand(rng, d)
        push!(xs, sp.x)
        push!(ys, sp.y)
    end

    plt = scatter(xs, ys;
        title        = sc.title,
        xlabel       = "x [m]",
        ylabel       = "y [m]",
        xlims        = sc.xlims,
        ylims        = sc.ylims,
        aspect_ratio = :equal,
        markersize   = 2,
        markeralpha  = 0.3,
        label        = "samples",
    )

    draw_walls!(plt, room)

    # Start position: circle scaled to robot radius
    R = RoombaPOMDPs.ROBOT_W.val / 2
    θc = range(0, 2π, length=64)
    plot!(plt, sc.s.x .+ R .* cos.(θc), sc.s.y .+ R .* sin.(θc);
        color     = :green,
        linewidth = 2,
        label     = "start",
    )
    # Orientation indicator: line from center to edge in direction s.theta
    plot!(plt, [sc.s.x, sc.s.x + R * cos(sc.s.theta)],
               [sc.s.y, sc.s.y + R * sin(sc.s.theta)];
        color     = :green,
        linewidth = 2,
        label     = "",
    )

    # Arrow from start to unconstrained desired next position
    dx, dy = desired_next(sc.s, sc.a)
    quiver!(plt, [sc.s.x], [sc.s.y];
        quiver    = ([dx - sc.s.x], [dy - sc.s.y]),
        linewidth = 2,
        linecolor = :red,
        label     = "",
    )

    # Star at the unconstrained desired position
    scatter!(plt, [dx], [dy];
        markersize  = 7,
        markercolor = :red,
        markershape = :star5,
        label       = "desired",
    )

    plt
end

p = plot(plots_list..., layout=(4, 2), size=(1000, 1600))
savefig(p, joinpath(@__DIR__, "wall_interaction.png"))
println("Saved wall_interaction.png")
display(p)
