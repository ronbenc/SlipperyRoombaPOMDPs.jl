"""
Visual verification of BananaStateDistribution.
Generates scatter plots of sampled next states from the same starting state
under various noise settings, to confirm the banana shape.

Run from repo root:
    julia --project=SlipperyRoombaPOMDPs SlipperyRoombaPOMDPs/banana_vis.jl
"""

include("src/RoombaPOMDPs.jl")
using .RoombaPOMDPs
using POMDPs, Distributions, Random, Plots

const N   = 3000
const rng = MersenneTwister(42)

# Starting state: at origin, heading East (theta=0)
s = RoombaState(0.0, 0.0, 0.0, 0.0)
# Action: moderate forward speed + turning => arc motion
a = RoombaAct(1.0, 0.5)

configs = [
    (theta_std=0.5,   trans_noise_coeff=0.01, title="θ noise dominant"),
    (theta_std=0.01,  trans_noise_coeff=0.5,  title="trans noise dominant"),
    (theta_std=0.3,   trans_noise_coeff=0.3,  title="both — banana"),
    (theta_std=0.1,   trans_noise_coeff=0.1,  title="both — small noise"),
]

# Deterministic mean next state (no noise) — used as arrow target in all panels
mdp_det = RoombaMDP(theta_std=0.01, trans_noise_coeff=0.01)
sp_mean = rand(MersenneTwister(0), POMDPs.transition(mdp_det, s, a))  # effectively deterministic

plots_list = map(configs) do cfg
    mdp = RoombaMDP(theta_std=cfg.theta_std, trans_noise_coeff=cfg.trans_noise_coeff)
    d   = POMDPs.transition(mdp, s, a)
    xs  = Float64[]
    ys  = Float64[]
    for _ in 1:N
        sp = rand(rng, d)
        push!(xs, sp.x)
        push!(ys, sp.y)
    end

    plt = scatter(xs, ys;
        title        = "$(cfg.title)\n(θ_std=$(cfg.theta_std), trans=$(cfg.trans_noise_coeff))",
        xlabel       = "x [m]",
        ylabel       = "y [m]",
        aspect_ratio = :equal,
        markersize   = 2,
        markeralpha  = 0.3,
        label        = "samples",
    )

    # Start point
    scatter!(plt, [s.x], [s.y];
        markersize  = 8,
        markercolor = :green,
        markershape = :circle,
        label       = "start",
    )

    # Arrow from start to deterministic mean next state
    quiver!(plt, [s.x], [s.y];
        quiver      = ([sp_mean.x - s.x], [sp_mean.y - s.y]),
        linewidth   = 2,
        linecolor   = :red,
        label       = "",
    )

    # Mark the mean next state
    scatter!(plt, [sp_mean.x], [sp_mean.y];
        markersize  = 7,
        markercolor = :red,
        markershape = :star5,
        label       = "desired",
    )

    plt
end

p = plot(plots_list..., layout=(2, 2), size=(900, 700))
savefig(p, joinpath(@__DIR__, "banana_distribution.png"))
println("Saved banana_distribution.png")
display(p)
