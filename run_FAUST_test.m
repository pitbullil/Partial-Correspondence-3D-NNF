featurecodepath =  fullfile(pwd);%set to the repository root dir
params.code_path = fullfile(featurecodepath,'nnf_sparse_code','build','Release','3DDIS_OMP.exe');
assert(exist(params.code_path, 'file') == 2,'not in code root folder')
addpath(genpath(featurecodepath))
FAUST_ROOT = 'D:\OneDrive\MPI-FAUST\MPI-FAUST\test'
cd(FAUST_ROOT)
training_root = '.\challenge_pairs\'; % relative path  from FAUST_ROOT to directory containing challenge info text files
challenge = 'intra' % name of the challenge. only intra is really addressed by our method
fid = fopen(['.\challenge_pairs\' challenge '_challenge.txt']);
pairs = textscan(fid,'%d_%d');
formatSpec = 'test_scan_%03d.ply'
scan_root = '.\scansLR\';%relative path to Low resolution scans
temp_dir = '.\temp\'
dense_match_folder = 'TRIDIS_60.0_DF_0.5_NNR_1.00_FR_3.00\dense\'
mkdir(dense_match_folder);
times_sparse =zeros(1,numel(pairs{1}));
times =zeros(1,numel(pairs{1}));

for i=[1:60]
    t=tic;
    src = sprintf(formatSpec,pairs{1}(i))
    tgt = sprintf(formatSpec,pairs{2}(i))
    N = ply_to_shape([scan_root tgt]);
    M = ply_to_shape([scan_root src]);

    params.surface_recon = 'LOAD';
    params.keypoints_str = 'ALL';
    params.normals = 'MESHNORMS';
    params.distances = 'GEODESIC';
    params.feature.type = 'FPFH';
    params.similarity = 'TRIDIS';
    params.feature.rad = 3.00;
    params.deformation_frac = 0.5;
    params.geodesic_disc_frac = 60.0;
    params.nnrej=1;
    params.threads = 6;
    params.use_greedy_opt = true;
    params.set = 'FAUST'

    tic;
    [matches_sparse] = DDIS_sparse_correspondences(src(1:end-4),scan_root,tgt(1:end-4),scan_root,params);
    %matches_sparse_full = [map_N(matches_sparse(:,1)),map_M(matches_sparse(:,2))];
    diam=sqrt(sum(calc_tri_areas(M)));
    times_sparse(i)=toc;
    % using ground truth indices
    %- Construct C matrix using ground-truth correspondence and compare to A\B
    main_params.delta = 100;
    main_params.num_eigen = 90;
    main_params.shot.num_bins = 10;
    main_params.shot.rad=0.04;
    main_params.shot.rad=main_params.shot.rad*diam;
    main_params.shot.scalar =3;
    main_params.mu.sparse =5e-1;
    main_params.mu.dense = 0.7;
    main_params.max_iter = 100;
    main_params.manopt_maxiter = 100;
    main_params.icp_maxiter = 0;
    main_params.num_deltas = round(N.n/10);
    main_params.delta_radius = 1/sqrt(main_params.num_deltas)*diam;

    main_params.init_delta_radius = 1/sqrt(size(pi*matches_sparse,1))*diam;
    main_params.verbose = true;
    main_params.num_reiterate = 0;

    tic;matches = RPFM(M,N,main_params,matches_sparse);
    matches = [(1:N.n)' matches];
    dlmwrite(fullfile(dense_match_folder,[src(1:end-4) '_' tgt(1:end-4) '.csv']),matches);
    times(i)=toc

end
save([challenge 'times'],'times','times_sparse')


