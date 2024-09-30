function LQshow(LQ)
   %global qsize qbulk qdata qhead qtail qdim
   my_disp('----------------------');
   disp(['Queue size is ' num2str(LQ.qsize) ' by ' num2str(LQ.qdim)]);
   disp(['number of data elements is ' num2str(LQ.qtail - LQ.qhead)]);
   if LQisEmpty(LQ)
      disp('Queus is empty');
   else
      disp('data is: (pres ENTER to continue)');
      pause
      LQ.qdata(LQ.qhead:LQ.qtail-1,:)
   end
end