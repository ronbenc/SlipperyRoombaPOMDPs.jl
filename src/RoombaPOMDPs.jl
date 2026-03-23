module RoombaPOMDPs

using POMDPs
using Distributions
using StaticArrays
using Parameters
using POMDPTools
using Statistics
using Graphics
using Cairo
using Random
using LinearAlgebra
using ParticleFilters

import POMDPTools: render

export
    MapLayout,
    baseline,
    one_sided_corridor,
    two_sided_corridor,
    Room,
    RoombaState,
    RoombaAct,
    RoombaMDP,
    RoombaPOMDP,
    BananaStateDistribution,
    RoombaModel,
    Bumper,
    BumperPOMDP,
    Lidar,
    LidarPOMDP,
    DiscreteLidar,
    DiscreteLidarPOMDP,
    RoombaParticleFilter,
    get_goal_xy,
    wrap_to_pi,
    ContinuousRoombaStateSpace,
    DiscreteRoombaStateSpace,
    render

include("line_segment_utils.jl")
include("env_room.jl")
include("roomba_env.jl")
include("filtering.jl")

end
