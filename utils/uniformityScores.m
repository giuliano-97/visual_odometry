function scores = uniformityScores(candidates_and_keypoints, optionalArgs)
    arguments
        candidates_and_keypoints (:,2) double= random('Uniform',1,600,600,2);
        optionalArgs.Sigma double = 30;
        optionalArgs.Test = false;
    end
    % Defining covariance matrix and maximum value in individual
    % distribution
    sigma_cov = diag(optionalArgs.Sigma*[1,1]);
    maximum_dist_value = mvnpdf([0,0],[0,0],sigma_cov);
    
%     % Defnining functionc
%     tic
%     func = @(X) zeros(size(X,1),1);
%     tic
%     for i = 1:size(candidates_and_keypoints,1)
%         func = @(X) func(X) + mvnpdf(X,candidates_and_keypoints(i,:),...
%             sigma_cov);
%     end
% 
%     func = @(X) func(X)-maximum_dist_value;
%     func = @(X) func(X)/max(func(X));
%     scores = func(candidates_and_keypoints);
%     toc
    tic
    row_mvnpdf = @(u_x, u_y) (@(x) mvnpdf(x, [u_x,u_y], sigma_cov));
    
    funcs = @(X) bsxfun(row_mvnpdf, X(:,1), X(:,2));
    
    distribs = funcs(candidates_and_keypoints);
    
    penalty_func = @(x,y) sum(distribs([x,y]));
    tbl = table(candidates_and_keypoints(:,1), candidates_and_keypoints(:,2));
    
    scores = rowfun(penalty_func, tbl);
    scores = table2array(scores);
    scores = scores/max(scores);
    toc

    %% Displaying results for testing
    if optionalArgs.Test 
        [X1,X2] = meshgrid((1:600)',(1:600)');
        X = table(X1(:), X2(:));
        
        tic
        result = rowfun(penalty_func, X);
        toc

        Z = reshape(table2array(result),600,600);
        Z = Z/max(Z,[],'all');
        surf(X1,X2,Z);
    end
end