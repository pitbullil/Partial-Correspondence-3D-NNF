 function [ shape ] = colorize_mesh( shape,C )
%COLORIZE_MESH Summary of this function goes here
%   Detailed explanation goes here
if isa(C,'char')
    P = pcread(C);
    C = P.Color;
    clear P;
end
if length(C) ~= shape.n
        error('color and location size mismatch');
end
shape.color = uint8(255*C);
end

