function X = madmm_l21(x0,functions,params)

% set reg term: g(x) = |v(X)|_2,1 (sum of norm over columns)
functions.fun_g = @(Z)sum(sqrt(sum(functions.fun_v(Z).^2,1)));

T = params.max_iter; %number of iterations

% set the manifold type
problem.M = params.manifold;
options.verbosity = 0;
if isfield(params,'manopt_maxiter'), options.maxiter = params.manopt_maxiter; end;

% set the "shrinkage" parameter
c = params.lambda / params.rho;


X = x0;
Z = functions.fun_v(x0);
U = zeros(size(Z));


original_cost = @(x)functions.fun_f(x) + params.lambda*functions.fun_g(x);
keep_cost = zeros(T,1);

if params.is_plot, fig = animatedline; title('cost'); end

for step = 1:T   
    
    keep_cost(step) = original_cost(X);    
        
    if params.is_plot, addpoints(fig,step,keep_cost(step)); drawnow(); end

    
    %disp(['cost: ' num2str(keep_cost(step))]);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % X-step - solve a smooth minimization problem on the manifold    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    problem.cost = @(x)functions.fun_f(x) + params.rho*functions.fun_h(x,Z,U);
    problem.egrad = @(x)functions.dfun_f(x) + params.rho*functions.dhdx(x,Z,U);
    % checkgradient(problem);

    
    X = conjugategradient(problem,X,options);
    
    %disp('Finished step X.');    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Z-step     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Z = prox_l21(functions.fun_v(X)+U,c);    
    %disp('Finished step Z.'); 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % U-update 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    U = U + functions.fun_v(X) - Z;
    
    
end


%if params.is_plot, figure(15), subplot(121),colormap; imagesc(x0), subplot(122), imagesc(X); colormap; end
    if params.is_plot, clearpoints(fig); drawnow(); end


end

