function [barmap] = shape_point_to_barycentric_map(S)
%SHAPE_POINT_TO_BARYCENTRIC_MAP Summary of this function goes here
%   Detailed explanation goes here
if ~isfield(S,'X')
    S.X = S.VERT(:,1);
end

barmap = zeros(length(S.X),4);
for i=1:length(S.X)
    [row,col]=find(S.TRIV==i);
    barmap(i,:)=[row(1) S.TRIV(row(1),:)==i];
end

end