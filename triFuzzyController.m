function out = triFuzzyController(inVals, inMFParams, outMFParams, rules)
    % Implement a generic fuzzy controller that implements Simplified PIE
    % with CA defuzzification, supporting an arbitray number of triangular
    % input MFs
    % TODO, implement Center of Sums and
    % Max-Min Inference method rather than PIE and CA
    % Inputs: 
    %   inVals: Input singleton points, 1 for each fuzzy input parameter
    %   inMFParams:  Mx1{Nx3} Input Triangular MF parameters, Mx1 Cell array of Nx3 
    %       input MF parameters, N is number of input MFs, M is number of
    %       input Fuzzy input parameters
    %   outMFParams: Nx3 Output MF parameters, N is number of output MFs
    %       of length 3 [c_slow, c_medium, c_fast]
    %   rules: 1xN struct, containing indices of input and output fuzzy values
    % Outputs: Fan speed RPM

    % Use fuzzy singleton, so only take the input MFs at the input value to
    % find the input MF contributions w
    for ii = 1:length(inMFParams)
        MFParams = inMFParams{ii};
        for jj = 1:size(MFParams,1)
            if jj == 1 % First MF
                w(ii,jj) = Ltriangle(inVals(ii),MFParams(jj,1),MFParams(jj,2),MFParams(jj,3));
            elseif jj == size(MFParams,1) % Last MF
                w(ii,jj) = Rtriangle(inVals(ii),MFParams(jj,1),MFParams(jj,2),MFParams(jj,3));
            else % Middle MFs
                w(ii,jj) = Ctriangle(inVals(ii),MFParams(jj,1),MFParams(jj,2),MFParams(jj,3));
            end
        end
    end

    % Correlate input and output MFs using the rules
    for rule = 1:length(rules)
        % Rules are format: if param1 and param2 [and param3] then outputmf
        rw(rule) = 100; %this is set unreasonably high as at the end of the below loop rw should never be this value, it should be 1 or less
        for param = 1:length(inMFParams)
            rw(rule) = min( w(param, rules(rule).inputMFs(param)), rw(rule)); % take the minimum of the contributing weight of each parameter in the rule
        end
    end

    
    % Apply the rule weights to each output MF and sum to compute CA defuzzification
    % out = (w_cold*c_fast + w_cool*c_medium + w_warm*c_slow) ./ (w_cold+w_cool+w_warm);
    out = sum(rw'.*outMFParams([rules.outputMF],2) ./ sum(rw)); % Rule weights * output MF centers / sum(rule weights)
    

end

function out = Ltriangle(x,a,b,c)
    out = max(min(1,(c-x)/(c-b)),0);
end

function out = Ctriangle(x,a,b,c)
    out = max(min((x-a)/(b-a),(c-x)/(c-b)),0);
end

function out = Rtriangle(x,a,b,c)
    out = max(min((x-a)/(b-a),1),0);
end