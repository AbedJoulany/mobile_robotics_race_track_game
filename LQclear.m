function LQ = LQclear(LQ)
   % returns an empty queue
   %global qhead qtail
   LQ.qdata = zeros(LQ.qsize,LQ.qdim);
   LQ.qhead = 1;
   LQ.qtail = 1;
end