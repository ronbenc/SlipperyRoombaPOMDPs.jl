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
const VMAX = 2.0

const THETA_STD = 1.2
const TRANS_COE = 0.8
const NOISE_STR = "(θ_std=$(THETA_STD), trans=$(TRANS_COE))"

mdp  = RoombaMDP(theta_std=THETA_STD, trans_noise_coeff=TRANS_COE)
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
        title = "1: Open baseline $NOISE_STR\n(center of corridor, no walls nearby)",
        s     = RoombaState(0.0, 0.0, 0.0, 0.0),
        a     = RoombaAct(2.0, 0.3),
        xlims = (-4.0, 8.0),
        ylims = (-4.0, 4.0),
    ),
    (
        title = "2: Run through wall $NOISE_STR\n(desired lands 0.5 m past top wall)",
        s     = RoombaState(0.0, 4.5, π/2, 0.0),
        a     = RoombaAct(2.0, 0.0),
        xlims = (-2.5, 2.5),
        ylims = (2.5, 7.0),
    ),
    (
        title = "3: Oblique approach $NOISE_STR\n(θ=π/3 toward top wall — asymmetric clip)",
        s     = RoombaState(5.0, 2.0, π/3, 0.0),
        a     = RoombaAct(2.0, 0.0),
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
        title = "5: L-corner bypass $NOISE_STR\n(NE toward corner at (-15,-5) — split)",
        s     = RoombaState(-16.5, -7.0, π/4, 0.0),
        a     = RoombaAct(2.0, 0.0),
        xlims = (-22.0, -10.0),
        ylims = (-12.0, -2.0),
    ),
    (
        title = "6: Tight corridor (R4) $NOISE_STR\n(north, 5 m wide — both walls clip)",
        s     = RoombaState(12.5, -1.0, π/2, 0.0),
        a     = RoombaAct(2.0, 0.0),
        xlims = (8.0, 17.0),
        ylims = (-4.0, 6.0),
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

    # Start position
    scatter!(plt, [sc.s.x], [sc.s.y];
        markersize  = 8,
        markercolor = :green,
        markershape = :circle,
        label       = "start",
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

p = plot(plots_list..., layout=(3, 2), size=(1000, 1200))
savefig(p, joinpath(@__DIR__, "wall_interaction.png"))
println("Saved wall_interaction.png")
display(p)
