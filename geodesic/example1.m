% this simple exampe shows the general principles of geodesic toolbox
% Danil Kirsanov, 09/2007 

global geodesic_library;                
geodesic_library = 'geodesic_release';      %"release" is faster and "debug" does additional checks
rand('state', 0);                         %comment this statement if you want to produce random mesh every time

N = 10000;                                  %number of points in a mesh
% [vertices,faces] = create_hedgehog_mesh(N, 0.1);   %create "noisy sphere" mesh; "vertices" contains 3D vertex coordinates; "faces" contains vertex id's for every triangle
% %[vertices,faces] = create_flat_triangular_mesh(0.1, 0); N = length(vertices);  %rectangular mesh for sanity check
% shape.X = vertices(:,1);
% shape.Y = vertices(:,2);
% shape.Z = vertices(:,3);
% shape.TRIV = faces;
shape = ply_to_shape('wolf.ply');
tic;
D = CreateDistanceMatrix(shape,'exact', 4);
toc;
c = 'smacof';
feat = meshfeaturepoints(shape,D,c);
mds=meshmds(shape,D,c);
mds_result_out_file = fullfile(['rmeshout' c '.ply']);
plywrite(mds_result_out_file,TRI',V');
N = meshneighbors(shape);
[F max] = meshtipfunctional(shape,N,D);
trisurf(mds.TRIV,mds.X,mds.Y,mds.Z,F, 'FaceColor', 'interp', 'EdgeColor', 'k');       %plot the mesh
K = convhulln(V');