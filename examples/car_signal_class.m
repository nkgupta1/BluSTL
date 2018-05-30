classdef car_signal_class < STLC_lti
    
    methods
        function SC = car_signal_class()
            
            A = [0 0 ;
                 0 0];
            Bu = [1 0;
                  0 1];

            Bw = [0 0;
                  0 0]; 

            C = [0 0];
            Du = [0 0];
            Dw = [0 0];

            SC = SC@STLC_lti(A,Bu,Bw,C,Du,Dw);
            
        end
        
        function obj = get_objective(Sys, X, Y, U, W, rho,wr,wt1)
            % last term penalizes inputs late in execution
            % https://yalmip.github.io/command/sdpfun/
            % a = sdpfun(find(max(abs(U))),'max');
            %obj = norm(sum(abs(U),2), Sys.nrm)-wr*norm(sum(rho,2), Sys.nrm) + a;
            obj = norm(sum(abs(U),2), Sys.nrm)-wr*norm(sum(rho,2), Sys.nrm) + max(find(sum(abs(double(U))>0.1)));
        end  
        
    end
    
    
end