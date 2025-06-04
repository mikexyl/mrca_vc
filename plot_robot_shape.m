function h = plot_robot_shape(ax, dim, pos, radius, color, varargin)
% plot_robot_shape - Helper to plot a robot as ellipse (2D) or ellipsoid (3D)
%   h = plot_robot_shape(ax, dim, pos, radius, color, Name, Value, ...)
%   ax: axes handle
%   dim: 2 or 3
%   pos: position vector
%   radius: scalar or vector (2x1 or 3x1)
%   color: color spec (string or RGB)
%   varargin: additional plotting properties

if dim == 2
    % Default values
    edgeColor = 'black';
    faceAlpha = 0.5;
    lineStyle = '-';
    lineWidth = 2;
    faceColor = color;
    % Parse varargin
    for i = 1:2:length(varargin)
        switch lower(varargin{i})
            case 'edgecolor', edgeColor = varargin{i+1};
            case 'facecolor', faceColor = varargin{i+1};
            case 'facealpha', faceAlpha = varargin{i+1};
            case 'linestyle', lineStyle = varargin{i+1};
            case 'linewidth', lineWidth = varargin{i+1};
        end
    end
    h = plot_ellipse_2D(ax, pos, radius, 0, ...
        'EdgeColor', edgeColor, ...
        'FaceColor', faceColor, ...
        'FaceAlpha', faceAlpha, ...
        'LineStyle', lineStyle, ...
        'LineWidth', lineWidth);
else
    % 3D
    edgeColor = color;
    faceColor = color;
    faceAlpha = 1;
    % Parse varargin
    for i = 1:2:length(varargin)
        switch lower(varargin{i})
            case 'edgecolor', edgeColor = varargin{i+1};
            case 'facecolor', faceColor = varargin{i+1};
            case 'facealpha', faceAlpha = varargin{i+1};
        end
    end
    h = plot_ellipsoid_3D(ax, pos, radius, 0, ...
        'EdgeColor', edgeColor, ...
        'FaceColor', faceColor, ...
        'FaceAlpha', faceAlpha);
end
end
