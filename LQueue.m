function LQ = LQueue(bulk,dim)
   % returns a new queue with element vectors of length dim
   % the initial size is bulk, it will grow on demand (see LQinsert)
   % LQ members are qsize qbulk qdata qhead qtail qdim
   % There is also an efficient version that implements one global queue
   % (see the function Queue).
   
   if bulk < 10
      bulk = 10;
   end
   if bulk > 5000
      bulk = 5000;
   end;
   
   LQ.qbulk = round(bulk); % ensure integer size
   
   LQ.qsize = LQ.qbulk; % memory is reserved for this initial number of elements
   LQ.qdim = dim;
   LQ.qdata = zeros(LQ.qsize,dim);
   % elements will be inserted at the tail and got out from the head
   LQ.qhead = 1;
   LQ.qtail = 1;
end