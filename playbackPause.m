function playbackPause(xtraj,doPlaybackPause)
   if doPlaybackPause
      pause(xtraj.tspan(2)/5);  % speed up 5 times
   end
end