featurecodepath =  fullfile(pwd);%set to the repository root dir
ddis_params.code_path = fullfile(featurecodepath,'nnf_sparse_code','build','Release','3DDIS_OMP.exe');
addpath(genpath(featurecodepath))
close all
clear all
models_dir = 'C:\Users\narbel\OneDrive\SHREC16ATest\';%path to SHREC'16 Partial matching root dir (obtain from http://www.dais.unive.it/~shrec2016/dataset.php)
set = 'holes'; %replace to holes if necessary
null_model_dir = fullfile(models_dir,'null');
part_model_dir = fullfile(models_dir,set);
files = dir(fullfile(part_model_dir,'*.off'));
matches_folder =    fullfile(models_dir,'.\TRIDIS_60.0_DF_0.5_NNR_1.00_FR_3.00\dense\');

%compute match errors
visual_dir = fullfile(matches_folder,'visual');
mkdir(visual_dir)
for fi = [1:numel(files)]
    fprintf('-------------------------------------\nProcessing shape %d of %d\n',fi, numel(files));
    clear M;
    clear N;
    
    fname = [files(fi).name];
    [~, name, ~] = fileparts(fname);

    null_model = regexp(name,'[^_0-9]*','match');
    null_model = [null_model{2} ''];
    
    M = load_off(fullfile(null_model_dir, [null_model ,'.off']));    
    N = load_off(fullfile(part_model_dir, [name,'.off']));
    
    outfile = fullfile(visual_dir,[name '.jpg']);
    matches =  dlmread(fullfile(matches_folder, [name,'.csv']));
    vis_partial_dense_matches(N, M, matches(:,2),fullfile(visual_dir,name))
    close
end
%% Compute performance indices
