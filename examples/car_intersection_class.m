classdef car_intersection_class < STLC_lti
    
    methods
        function SC = car_intersection_class()
            
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
        
            
        
    end
    
    
end