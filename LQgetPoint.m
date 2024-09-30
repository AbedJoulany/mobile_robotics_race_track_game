function [LQ,p] = LQgetPoint(LQ)
   % takes out the first element and resturns it to the caller, with the
   % changed queue
   % global qdata qhead qtail
   if (LQ.qhead ~= LQ.qtail) % QisEmpty
      p = LQ.qdata(LQ.qhead,:);
      LQ.qhead = LQ.qhead + 1;
   else
      error('Queue is empty');
   end
end