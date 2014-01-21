function varargout = aclock(request,bufnum);
%ACLOCK Start an "atomic" stopwatch timer.
%  The purpose of ACLOCK is to record the amount of CPU time used by your
%  operations, i.e.
%     to = cputime; operation; cputime-to
%  However, in the event that your operation is less than the 5 times the granularity 
%  of resolution computed by CPUTIME, ACLOCK just returns the elapsed time
%  instead, i.e.
%     to = clock; operation; etime(clock,to)
%
%  Usage:
%  ACLOCK('TIC') starts the stopwatch timer.
%  ACLOCK('TOC') by itself prints the elapsed time (in seconds) since
%  ACLOCK('TIC') was used.  T = ACLOCK('TOC'); saves the elapsed time
%  in T, instead of printing it out.
%
%  B = ACLOCK('TIC') optionally retuns a buffer number associated with the stopwatch.
%  T = ACLOCK('TOC',B) retuns the elapsed time associated with that particular buffer.
%
%  This feature allows multiple calls to ACLOCK to each report the correct
%  elapsed time. 
%
%  Example:
%  buf1 = aclock('tic'); rand(1000);
%  buf2 = aclock('tic'); rand(1000);
%  aclock('toc',buf1);
%  aclock('toc',buf2);
%
% History
% Date          Who        Comment
% ----------    ---        -----------------------------------
% 11-27-2004    rme        Created and written.  
  
persistent atomicBuffer availableBuffer;

maxBuffers = 100;

% initialize the buffer
if isempty(atomicBuffer); 
  atomicBuffer = cell(maxBuffers,2);
  availableBuffer = 1;
end;

switch lower(request);
case 'tic';
 atomicBuffer{availableBuffer,1} = clock;      % store "clock" time
 atomicBuffer{availableBuffer,2} = cputime;    % store "cpu" time
 if nargout; 
   varargout{1} = availableBuffer;             % return current buffer number
 end;
 availableBuffer = availableBuffer+1;          % increment buffer number
 if availableBuffer > maxBuffers;
   availableBuffer = 1;                        % wrap index to beginning of ring buffer
 end;
case 'toc';
 if ~exist('bufnum','var') || isempty(bufnum); % check if user supplied buffer number
   bufnum = availableBuffer - 1;               % set to buffer number returned by previous call
   if bufnum < 1;
     bufnum = maxBuffers;                      % wrap around to end of ringbuffer
   end;
 end;
 dtclock = etime(clock,atomicBuffer{bufnum,1});% elapsed "clock" time
 dtcpu   = cputime - atomicBuffer{bufnum,2};   % elapsed "cpu" time
 % cputime only has a granularity of 0.01s on my linux box.  return elapsed cputime
 % if the operation took longer than this minimum granularity, otherwise just return
 % the elapsed time.
 if dtcpu > 0.05; elapsed_time = dtcpu; else; elapsed_time = dtclock;  end;
 if nargout;
   varargout{1} = elapsed_time;                % output the elapsed time
 else;
   elapsed_time                                % print elapsed time to screen
 end;
otherwise; 
 error('uknown behavior');                     % error
end;
