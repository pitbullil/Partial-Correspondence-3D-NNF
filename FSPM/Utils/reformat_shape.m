function [ model_ ] = reformat_shape( model )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
model_.VERT = model.VERT;
model_.S = model.S;
model_.m = size(model.TRIV,1);
model_.n = size(model.VERT,1);
model_.evecs = model.evecs_new;
model_.TRIV = model.TRIV;


end

