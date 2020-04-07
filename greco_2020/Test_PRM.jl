# Description: test script for robot planning algorithms for a robot of radious 1m
using Pkg
Pkg.activate(@__DIR__)
Pkg.instantiate()
using GRECO, Revise
using Plots, Random
using MeshCat, GeometryTypes, CoordinateTransformations
# use include("src/GRECO.jl") # use if module alejandro is modified

# Some functions provided by Dr. Hassan Poonawala for visualization (pathvisualization.jl)
function sampleh() # random homogenous matrix
  randvec=rand(1,3)*2*pi
  rotmat = [1 0.0 0.0; 0.0 cos(randvec[1]) -sin(randvec[1]);0.0 sin(randvec[1]) cos(randvec[1])] *[cos(randvec[1]) 0.0 -sin(randvec[1]); 0.0 1.0 0.0;sin(randvec[1]) 0.0 cos(randvec[1])]*[1 0.0 0.0; 0.0 cos(randvec[2]) -sin(randvec[2]);0.0 sin(randvec[2]) cos(randvec[2])]
  h = zeros(Float64,4,4)
  h[1:3,1:3] = rotmat;
  h[1:3,4] = randn(3,1)*100
  return h
end

function plot_cylinder(vis,c1,c2,radius,mat,name="")
    geom = Cylinder(Point3f0(c1),Point3f0(c2),convert(Float32,radius))
    setobject!(vis["cyl"][name],geom,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
end

function plot_sphere(vis,c1,radius,mat,name="")
    geom = HyperSphere(Point3f0(c1), convert(Float32,radius))
    setobject!(vis["sph"][name],geom,mat)
end

function add_obs!(vis,obs) # visualize obstacles
    for i in 1:size(obs,1)

        if obs[i,5] == 0.0

          plot_sphere(vis,[obs[i,1],obs[i,2],obs[i,3]],obs[i,4],MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"sph$i")
        else
          plot_cylinder(vis,[obs[i,1],obs[i,2],0],[obs[i,1],obs[i,2],obs[i,3]],obs[i,4],MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"cyl_$i")
        end
    end
end

function draw_robot(vis,h,name="")  # visualize frame as quadrotor
    robot_obj = HyperRectangle(Vec(0.0,0.0,0.0),Vec(1.0,1.0,1.0))
    setobject!(vis["robot"][name],robot_obj,MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0)));
    settransform!(vis["robot"][name], compose(Translation(h[1:3,4]),LinearMap(h[1:3,1:3])))
end

# Example selector: 1 to 3
SF=3;
if SF==1
    s=[0.0 0.0 0.0]';
    g=[20.0 20.0 5.0]';
    O=[10.0 10.0 5.0 2 0;
        5.0 5.0 8.0 3 0;
        14.0 7.0 10.0 2.0 0;
        6.0 16.0 9.0 3.0 0;
        10.0 10.0 3.0 2.0 1;
        18.0 18.0 12.0 1.0 1;
        17.0 19.0 12.0 1.0 1
        19.0 17.0 12.0 1.0 1]
elseif SF==2
    s=[0.0 0.0 0.0]';
    g=[0.0 0.0 23.0]';

    O=[5.0 5.0 10.0 1.0 1;
        5.0 0.0 10.0 1.0 1;
        5.0 -5.0 10.0 1.0 1;
        -5.0 5.0 10.0 1.0 1;
        -5.0 0.0 10.0 1.0 1;
        -5.0 -5.0 10.0 1.0 1;
        0.0 5.0 10.0 1.0 1;
        0.0 -5.0 10.0 1.0 1;
        0.0 0.0 15.0 5.0 0]
    elseif SF==3
        # Start node
        s=[0.0 0.0 0.0]';

        # End node
        g=[4.0 4.0 1.0]';

        # Obstacles: each row represents an obstacle
        O=[2 2 2 0.2 0;         # sphere
            2 2 1.5 0.5 1];      # cylinder
end


# Path with PRM
Frames,Cost=PRM(s,g,O);
traj_sol=Frames;

# Visualize everything: obs + solution path
vis = Visualizer()
open(vis)
# obstacles:
add_obs!(vis,O)
# visualize robot at each coordinate frame on path:
for i in 1:length(traj_sol)
  draw_robot(vis,traj_sol[i],"$i")
end
