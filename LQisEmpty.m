function a = LQisEmpty(LQ)
   % returns 1 if Q is an empty queue
   %global qhead qtail
   a = (LQ.qhead == LQ.qtail);
end