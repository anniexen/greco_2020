function RoadMapPRM(s,g,O,Map)
    # Check of initial and final outside obstacles
    if isColliding(s,O)>0
        error("Start point is on obstacle")
    end
    if isColliding(g,O)>0
        error("End point is on obstacle")
    end

    # Sampling of free space
    NumSamples=1000;     # Number of desired samples in free space
    SamplePoints=hcat(s,g);
    while size(SamplePoints,2)<NumSamples+2
        # Random sample in space
        Sample=[rand(1)*abs(Map[1,2]-Map[1,1]);rand(1)*abs(Map[2,2]-Map[2,1]);rand(1)*abs(Map[3,2]-Map[3,1])];
        # If no collision, add to samples
        if isColliding(Sample,O)==0
            SamplePoints=hcat(SamplePoints,Sample)
        end
    end

    # Connection of sampled points
    Start_node=Array{Int64}(undef,1,1)
    End_node=Array{Int64}(undef,1,1)
    Link_length=Array{Float64}(undef,1,1)
    SamplePoints=reshape(SamplePoints,3,NumSamples+2)
    kdtree = KDTree(SamplePoints)       # k nearest points using NearestNeighbors package online
    for i=1:size(SamplePoints,2)
        k=10;    # Number of closest samples
        ps=SamplePoints[:,i];
        idxs, close_dists = knn(kdtree, ps, k, true)
        for j=1:k
            if close_dists[j]>0
                if isEdgeColliding(ps,SamplePoints[:,idxs[j]],O)==0
                    Start_node=vcat(Start_node, i)
                    End_node=vcat(End_node,idxs[j])
                    Link_length=vcat(Link_length, close_dists[j]);
                end
            end
        end
    end
    Start_node=Start_node[2:end,:]
    End_node=End_node[2:end,:]
    Link_length=Link_length[2:end,:]

    return SamplePoints, Start_node, End_node, Link_length
end

function isColliding(p,O)
    collide=zeros(size(O,1),1)
    for i=1:size(O,1)
        r_obs=O[i,4];   # radious of object
        r_rob=1;        # radious of robot

        if O[i,5]==0    # if sphere
            d_obs=dist(O[i,1:3],p)  # distance between point and sphere
            if d_obs<=r_rob+r_obs;
                collide[i]=1;
            end
        elseif O[i,5]==1 && p[3]<=O[i,3]  # if cylinder
            d_obs=sqrt((p[1]-O[i,1])^2 + (p[2]-O[i,2])^2)
            if d_obs<=r_rob+r_obs;
                collide[i]=1;
            end
        end
    end
    return sum(collide)
end

function isEdgeColliding(p1,p2,O)
    # Local planner to search for collissions, Incremental approach
    d_points=dist(p1,p2);
    N_inc=round(d_points/1);    # step of at least 1m
    collide=0;
    p_inc=p1;
    for i=1:N_inc
        p_inc=d_points/N_inc*(p2-p1)+p_inc; # Incremental vector
        if isColliding(p_inc,O)>0           # if incremental point collides, break
            collide=1
            break
        end
    end
    return collide
end

function dist(p1,p2)
    # The dist(p1,p2) function returns a real number corresponding to Euclidean
    # distances between the points f1 and f2.
    d = sqrt((p1[1]-p2[1])^2 + (p1[2]-p2[2])^2 + (p1[3]-p2[3])^2)
end

function frame_def(d)::HomM
    # Description: return the homogeneous transformation matrix for a position d
    f=[1.0 0.0 0.0 d[1];0.0 1.0 0.0 d[2]; 0.0 0.0 1.0 d[3]; 0.0 0.0 0.0 1.0]
    return f
end

function framepath(Path,SamplePoints)
    F=Array{Float64,2}[]

    for j=1:size(Path,1)
        d=SamplePoints[:,Path[j]]
        Frame_node=frame_def(d)
        push!(F,Frame_node)
    end

    Frames=Tuple(F)
    return Frames
end

function MapBounds(p1,p2,O,Buffer)
    max_x=max(max(O[:,1]...),p1[1],p2[1])
    min_x=min(min(O[:,1]...),p1[1],p2[1])

    max_y=max(max(O[:,2]...),p1[2],p2[2])
    min_y=min(min(O[:,2]...),p1[2],p2[2])

    max_z=max(max(O[:,3]...),p1[3],p2[3])
    min_z=0

    Max=[max_x;max_y;max_z]
    Min=[min_x;min_y;min_z]
    Range=Max-Min;
    Map=hcat(Min-repeat([Buffer],3,1), Max+repeat([Buffer],3,1))
    Range=[Range+2*repeat([Buffer],3,1);2*pi;pi;2*pi]
    return Map
end
