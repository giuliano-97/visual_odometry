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
    
    % Defnining function
    tic
    func = @(X) zeros(size(X,1),1);
    for i = 1:size(candidates_and_keypoints,1)
        func = @(X) func(X) + mvnpdf(X,candidates_and_keypoints(i,:),...
            sigma_cov);
    end
%     func = @(X) func(X)-maximum_dist_value;
    func = @(X) func(X)/max(func(X));
    

    % calculating scores
    scores = func(candidates_and_keypoints);
    toc
    
    
    %% Displaying results for testing
    if optionalArgs.Test 
        [X1,X2] = meshgrid((1:600)',(1:600)');
        X = [X1(:), X2(:)];

        tic
        result = func(X);
        toc

        tic
        candidate_scores = func(candidates_and_keypoints);
        toc

        Z = reshape(result,600,600);
        surf(X1,X2,Z);
    end
end