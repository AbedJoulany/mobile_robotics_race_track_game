function LQ = LQinsert(LQ,p)
   % insert the data point p as the last element of LQ.
   % global qsize qbulk qdata qhead qtail qdim
   [i,j] = size(p);
   if  isempty(LQ.qdim)
      error('queue is not initialized, Use LQueue(size,dim).');
   else
      if ( i ~= 1 | j > LQ.qdim)
         error(['data point size is ' num2str(i) ' x ' num2str(j) ' and it should be 1 x ' num2str(qdim) '.']);
      end
   end
   if LQ.qtail >= LQ.qsize % not enough space
      % move the whole data down the queue
      newdata = LQ.qdata(LQ.qhead:LQ.qtail,1:LQ.qdim);
      % increase it by qbulk
      LQ.qdata = [newdata ; zeros(LQ.qbulk,LQ.qdim)];
      LQ.qsize = length(LQ.qdata);
      LQ.qhead = 1;
      LQ.qtail = length(newdata);
   end
   LQ.qdata(LQ.qtail,:) = p;
   LQ.qtail = LQ.qtail + 1;
   
end