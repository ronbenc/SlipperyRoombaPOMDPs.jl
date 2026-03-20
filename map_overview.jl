"""
Print all available SlipperyRoombaPOMDP room layouts to a single PNG.
- Row 1: 3 baseline (L-shaped) configurations
- Row 2: one_sided_corridor with 2 and 3 rooms
Green = goal wall, Red = stairs wall, Black = other walls.

Run from repo root:
    julia --project=SlipperyRoombaPOMDPs SlipperyRoombaPOMDPs/map_overview.jl
"""

include("src/RoombaPOMDPs.jl")
using .RoombaPOMDPs
using Plots

function draw_walls!(plt, room)
    for rect in room.rectangles
        for seg in rect.segments
            color = seg.goal ? :green : seg.stairs ? :red : :black
            plot!(plt, [seg.p1[1], seg.p2[1]], [seg.p1[2], seg.p2[2]];
                color=color, linewidth=2.5, label="")
        end
    end
end

function room_plot(room, title)
    plt = plot(; title=title, aspect_ratio=:equal,
                 xlabel="x [m]", ylabel="y [m]", legend=false)
    draw_walls!(plt, room)
    return plt
end

baseline_plots = [
    room_plot(RoombaMDP(config=c).room,
        ["Baseline config 1", "Baseline config 2", "Baseline config 3"][c])
    for c in 1:3
]

corridor_plots = [
    room_plot(Room(ContinuousRoombaStateSpace(), layout=one_sided_corridor, num_rooms=n),
              "One-sided corridor, $n rooms")
    for n in [2, 3, 4]
]

two_sided_plots = [
    room_plot(Room(ContinuousRoombaStateSpace(), layout=two_sided_corridor, num_rooms=n),
              "Two-sided corridor, $n rooms")
    for n in [2, 3, 4]
]

p = plot(baseline_plots..., corridor_plots..., two_sided_plots...,
         layout=(3, 3), size=(1500, 1350))
savefig(p, joinpath(@__DIR__, "all_maps.png"))
println("Saved all_maps.png")
