function [ cs ] = my_calc_shot( model,main_params )
%MY_CALC_SHOT Summary of this function goes here
%   Detailed explanation goes here
    cs = calc_shot([model.VERT]', model.TRIV', 1:model.n, main_params.shot.num_bins,main_params.shot.rad, main_params.shot.scalar)';

end

