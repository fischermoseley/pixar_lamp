function animate(t,R,C)

%axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched

% sets the axes to fit everything at all times
%axis([min(min(R(1:2:end,:))) max(max(R(1:2:end,:))) min(min(R(2:2:end,:))) max(max(R(2:2:end,:)))]);

% initialize the lines with the first column of R
legline = drawlines([],R(:,1),C);

speed = 1;                                      % playback speed
tic                                             % start counter
while toc < t(end)/speed                        % while there's real time left
    tsim = toc*speed;                           % determine the simulation time to draw
    Rint = interp1(t',R',tsim', 'linear')';     % interpolate to get coordinates at that time
    drawlines(legline,Rint,C);                  % draw the lines at that time
end
drawlines(legline,R(:,end),C);                  % be sure to end on the last frame

end

function legline = drawlines(legline,Rvec,C)

% Format the X and Y data
X = reshape(Rvec(2*C-1),[],2);
Y = reshape(Rvec(2*C),[],2);

if isempty(legline)
    % this is how to initialize line objects with multiple segments
    legline = line(X',Y','linewidth', 2,'color','blue', 'visible', 'on', 'marker','.','markersize',10, 'markeredgecolor','red');
else
    % this is how to update line objects with multiple segments
    set(legline,{'xdata'},num2cell(X,2),{'ydata'}, num2cell(Y,2), 'visible', 'on');
end

drawnow % if you don't call this function, it will only draw the last frame
end
