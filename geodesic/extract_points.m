function [pointcloud] = extract_points(shape,inds)
%EXTRACT_POINT Summary of this function goes here
%   Detailed explanation goes here
pointcloud = pointCloud([shape.X(inds,:),shape.Y(inds,:),shape.Z(inds,:)]);
end

