function [D] = CreateDistanceMatrix(shape,type,div)
%CREATEDISTANCEMATRIX Summary of this function goes here
%   Detailed explanation goes here
global geodesic_library;                
geodesic_library = 'geodesic_release';      %"release" is faster and "debug" does additional checks

if nargin < 2
    type = 'exact';
end
if nargin < 3
    div = 2;
end
V = [shape.X,shape.Y,shape.Z];
mesh = geodesic_new_mesh([shape.X,shape.Y,shape.Z],shape.TRIV);    %initilize new mesh
N = length(shape.X);
D = zeros(N);
source_points = cell(N,1);
for vertex_id = 1:N          %create all points 
    source_points{vertex_id} = geodesic_create_surface_point('vertex',vertex_id, V(vertex_id,:));
end

for source_id = 1:N          %create distance matrix 
    algorithm = geodesic_new_algorithm(mesh, type,div);      %initialize new geodesic algorithm
    geodesic_propagate(algorithm, {source_points{source_id}});
    [~,D(source_id,:)] = geodesic_distance_and_source(algorithm);
    geodesic_delete(algorithm);
end




