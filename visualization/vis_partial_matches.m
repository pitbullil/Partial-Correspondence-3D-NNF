function [h] = vis_partial_matches( X, Y, xin, yin,outfile)
%VIS_PARTIAL_MATCHES Summary of this function goes here
%   Detailed explanation goes here
if ~exist('pos')
    pos = [1/2 1/2 1/2 1/2];
end

gray1=gray(256);
colormap(gray1)
colorsY = create_colormap(Y,Y);
colorsY = colorsY(yin,:);
h=figure('units','normalized','outerposition',pos);
ax1=subplot(1,2,1);
trisurf(Y.TRIV, Y.VERT(:,1), Y.VERT(:,2), Y.VERT(:,3), 255*ones(Y.n, 1), 'EdgeAlpha', 0), hold on,

%scatter3(Y.VERT(:,1), Y.VERT(:,2), Y.VERT(:,3), 30, 0.8*ones(Y.n, 3), 'filled'),
axis equal, axis off,
hold on,
scatter3(Y.VERT(yin,1), Y.VERT(yin,2), Y.VERT(yin,3), 60, colorsY, 'filled'),
         axis equal;
    shading interp
    lighting phong, camlight,
    %alpha 0.5

    set(gca, 'xgrid', 'off')
    set(gca, 'ygrid', 'off')
    set(gca, 'zgrid', 'off')
set(gca,'Visible','off')

ax2=subplot(1,2,2);
white = 240 * [1 1 1];
%colormap white
trisurf(X.TRIV, X.VERT(:,1), X.VERT(:,2), X.VERT(:,3), 240*ones(X.n, 1), 'EdgeAlpha', 0), hold on,
    lighting phong, camlight,

axis equal;
    shading interp
    set(gca, 'xgrid', 'off')
    set(gca, 'ygrid', 'off')
    set(gca, 'zgrid', 'off')
set(gca,'Visible','off')
%alpha 0.5

axis equal, axis off,
scatter3(X.VERT(xin,1), X.VERT(xin,2), X.VERT(xin,3), 60, 1*ones(1,3), 'filled'), hold on
scatter3(X.VERT(xin,1), X.VERT(xin,2), X.VERT(xin,3), 60, colorsY, 'filled'),

%hlink = linkprop([ax1,ax2],{'CameraPosition','CameraUpVector'});
rotate3d on
pause
if exist('outfile')
    saveas(gcf,outfile);
    p = pointCloud(X.VERT(xin,:),'Color',colorsY);
    pcwrite(p,[outfile(1:end-3) '.ply'])
        p = pointCloud(Y.VERT(yin,:),'Color',colorsY);
    pcwrite(p,[outfile(1:end-3) '_base.ply'])

end
close(h);
end
