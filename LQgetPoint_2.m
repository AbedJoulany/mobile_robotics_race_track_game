function [LQ,p] = LQgetPoint_2(LQ)
   % takes out the last element and resturns it to the caller, with the
   % changed queue
   % global qdata qhead qtail
   if (LQ.qhead ~= LQ.qtail) % QisEmpty
      LQ.qtail = LQ.qtail - 1;
      p = LQ.qdata(LQ.qtail,:);
   else
      error('Queue is empty');
   end
end