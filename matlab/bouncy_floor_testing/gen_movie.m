function movie = gen_movie(t, q)
    x1 = q(1,:);
    x2 = q(2,:);
    
    vw = VideoWriter('gottem.avi', 'Uncompressed AVI');
    vw.FrameRate = 60;
    open(vw);
    
    for i = 1:length(t)
        plot(0, x1(i), 'ob');
        hold on
        plot(0, x2(i), 'or');
        ylim([-3, 3])
        xlim([-1, 1])
        hold off
        frame = getframe;
        movie(i) = frame;
        writeVideo(vw, frame);
    end
    close(vw);
end