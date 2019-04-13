featurecodepath =  fullfile(pwd);%set to the repository root dir
assert(exist(fullfile(featurecodepath,'build','Release','3DDIS_OMP.exe'), 'file') == 2,'not in code root folder')
addpath(genpath(featurecodepath))
close all
clear all
models_dir = 'D:\OneDrive\SHREC16ATest\';%path to SHREC'16 Partial matching root dir (obtain from http://www.dais.unive.it/~shrec2016/dataset.php)
set = 'cuts'; %replace to holes if necessary
null_model_dir = fullfile(models_dir,'null');
part_model_dir = fullfile(models_dir,set);
files = dir(fullfile(part_model_dir,'*.off'));
matches_folder =    fullfile(models_dir,'.\TRIDIS_60.0_DF_0.5_NNR_1.00_FR_3.00\greedy3\');

%compute match errors
visual_dir = fullfile(matches_folder,'visual');
mkdir(visual_dir)
for fi = 1:numel(files)
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
    try
    matches = dlmread(fullfile(matches_folder, [name,'.csv']))+1;
    vis_partial_matches(N, M, matches(:,1), matches(:,2),outfile);
    close
    catch
        continue
    end
end
%% Compute performance indices
