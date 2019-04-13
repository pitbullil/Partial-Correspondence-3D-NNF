function [matches] = DDIS_sparse_correspondences(null_name,null_path,part_name,part_path,params)

if ~isfield(params,'surface_recon') params.surface_recon = 'LOAD'; end
if ~isfield(params,'keypoints_str')params.keypoints_str = 'ALL'; end
if ~isfield(params,'normals')     params.normals = 'MESHNORMS';end
if ~isfield(params,'distances')     params.distances = 'GEODESIC';end
if ~isfield(params,'feature')    
    params.feature.type = 'FPFH';    params.feature.rad = 3.00;
end
if ~isfield(params,'similarity') params.similarity = 'TRIDIS'; end
if ~isfield(params,'deformation_frac')     params.deformation_frac = 0.5;end
if ~isfield(params,'geodesic_disc_frac')    params.geodesic_disc_frac = 60.0;end
if ~isfield(params,'nnrej') params.nnrej=1;end
if ~isfield(params,'threads') params.threads = 6;end
if ~isfield(params,'use_greedy_opt') params.use_greedy_opt = true;end
delafter = false
M = fullfile(null_path,null_name);
if exist(([M '.ply']), 'file') == 0
 delafter = true
    try 
        shape = off_to_shape([M '.off']);
        shape_to_ply([M '.ply'],shape)
    catch
        error('1st model does not exist')
    end
end
N = fullfile(part_path,part_name);
if exist(([N '.ply']), 'file') == 0
    try 
        shape = off_to_shape([N '.off']);
        shape_to_ply([N '.ply'],shape)
    catch
        error('1st model does not exist')
    end
end
N = [N '.ply'];
M = [M '.ply'];

out_dir = strjoin({params.similarity,...
    num2str(params.geodesic_disc_frac,'%3.1f'),...
    'DF',num2str(params.deformation_frac,'%3.1f'),...
    'NNR',num2str(params.nnrej,'%2.2f'),...
    'FR',num2str(params.feature.rad,'%2.2f')},'_')
match = [part_name '_' null_name '.csv']
if strcmp(params.set,'cuts') | strcmp(params.set,'holes')
    match =[part_name '.csv'];
end   
match_file = fullfile(out_dir,match);

if (params.use_greedy_opt) match_file = fullfile(out_dir,'greedy3',match); end;

    tic;
    [stat,res]=system(strjoin({params.code_path,...
        N, ...
        M, ... 
        ['surface=' params.surface_recon] , ...
        ['feature=' params.feature.type], ...
        ['keypoint=' params.keypoints_str], ...
        params.normals, ... 
        ['template=' params.distances],... 
        'mesh=SUPER', ... 
        ['similarity=' params.similarity], ... 
        ['DIVFRAC=' num2str(params.deformation_frac)], ...
        'RMODE=FRACQ', ...
        ['FRAC=' num2str(params.feature.rad)], ...
        'NN=1', ...
        ['RTHRESH=' num2str(params.geodesic_disc_frac)], ...
        'NNREJ=1'}),'-echo') ;
    if delafter    
        delete(M,N);
    end
    matches = dlmread(match_file)+1;
end

