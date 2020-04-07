module GRECO
using LinearAlgebra
using NearestNeighbors
using LightGraphs

const HomM = Array{Float64, 2}
include("PRM_functions.jl")
include("misc.jl")

function PRM(s,g,O)
    # Description: this functions obtains the shortest path using the original
    # probabilistic road map Planning using random sampling and connecting k-nearest
    # points in free space. The collissions checks are done for a robot of radious 1m
    # and for spheres and cylinders as obstacles. The shortest path is found using
    # dijkstra algorithm. It accepts s and g
    # as initial and final positions, these will be changed in a future version for
    # 4x4 homogeneous transformations
    # Alejandro Palacio 03/23/2020

    # Establish map limits
    Buffer=5;
    Map=MapBounds(s,g,O,Buffer)
    #Map=[round(min(O[:,1]...))-Max_buffer round(max(O[:,1]...))+Max_buffer;
    #round(min(O[:,2]...))-Max_buffer round(max(O[:,2]...))+Max_buffer;
    #0 round(max(O[:,1]...))+Max_buffer]

    # Generate RoadMapPRM
    SamplePoints, Start_node, End_node, Link_length=RoadMapPRM(s,g,O,Map);

    # Optimal path with
    Path, x, len=get_shortest_path(Start_node,End_node,Link_length,1,2)

    # Convert path into Tuple
    Frames=framepath(Path,SamplePoints)
    TotalCost=len;

    return Frames, TotalCost
end

export
    PRM
end # module
