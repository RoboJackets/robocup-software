% RBF_BallTracker.m
%
% Rao-Blackwellized Particle Filter for ball tracking in RoboCup
%
% Philip Rogers, progers7@gatech.edu, Oct 3rd 2009
%
% Reference: Map-Based Multiple Model Tracking of a Moving Object
%            Authors: Cody Kwok and Dieter Fox.
%            DOI: 10.1007/b106671
%
% Description of tracker:
%  This tracker uses a "Rao-Blackwellized" particle filter which means
%  the non-linear portions of the state are handeled using a particle
%  filter, and the linear portions are handeled using a linear filter
%  (in this case, I am using the Extended Kalman Filter.)
%  
%  There are 2 "modes" (and can be many more) that the particles are
%  distributed over.  Currently, one mode is relatively good at robustly
%  tracking the rolling ball, and the second mode helps handle the
%  non-linearities that occur during fast changes in velocity (such as
%  kicking or bouncing off of things.)
%
function RBF_BallTracker()
    % initializations
    clc; delete(timerfind); % clean up any existing timers
    dt = 0.05; % simulation change in time, in seconds
    [ballPlot,trackPlot,bestTrackPlot] = initDisplay(); % create display
    global ballModes, ballModes = createModes(); % create ball modes
    global balls, balls = initRBF(); % create RBF Filter
    global ballVisible, ballVisible = true;
    global ballObserved, ballObserved.x = [0; 0; 0; 0];

    % print short instructions
    disp('Rao-Blackwellized Particle Filter for ball tracking in RoboCup');
    disp(' how to use: Press any key to "hide" the ball (simulate lost segmentation)');
    disp(' click in the dark green area to completely stop the tracker.');
    
    % create tracker
    set(gcf,'WindowButtonMotionFcn',{@moveBall,ballPlot});
    simulationTimer = timer('TimerFcn', ...
        @(h,ev) trackerLoop(h,ev,ballPlot,trackPlot,bestTrackPlot,dt),...
        'ExecutionMode','fixedRate','Period',dt);
    start(simulationTimer);
    disp('starting...');
    
    % misc user interfacing
    set(gcf,'ButtonDownFcn',{@stopSimulation,simulationTimer});
    set(gcf,'KeyPressFcn',@makeBallInvisible);
end

% initDisplay()
% setup a plot for a rectangular soccer field with an orange ball.
% returns 2 objects corresponding to the ball and track plots.
function [ballPlot,trackPlot,bestTrackPlot] = initDisplay()
    figure(1); clf; hold on; axis equal;
    fieldSize = [-3.025,3.025,-2.025,2.025]; % field dimensions, in meters.
    axis(fieldSize);
    whitebg('g');
    trackPlot = plot(0,0,'.','MarkerEdgeColor',[0,0.5,0]);
    bestTrackPlot = plot(0,0,'x','MarkerEdgeColor','w');
    title('Rao-Blackwellized Particle Filter for ball tracking in RoboCup');
    ballSize = [0.12 0.12]; % ball size, in meters
    ballPlot = rectangle('Position',[0 0 ballSize],'Curvature',[1,1],...
        'FaceColor','r','LineStyle','none');
end

% initRBF()
% creates the ball tracker state
function balls = initRBF()
    N = 10; % number of ball particles
    balls = cell(N);
    for n = 1 : N
        balls{n}.x = [0; 0; 0; 0]; % EKF state: x(m), y(m), vx(m/s), vy(m/s)
        balls{n}.P = diag([0.0 0.0 0.0 0.0]); % EKF covariance matrix
        balls{n}.m = 1; % mode
        balls{n}.w = 1/N; % particle filter weight
    end
end

% moveBall
% called when the mouse moves, updates the position of the ball.
function moveBall(h,ev,ballPlot)
    global ballVisible; global ballObserved;
    mPoint = get(gca,'CurrentPoint');
    ballPos = get(ballPlot,'Position');
    if(ballVisible)
        set(ballPlot,'Position',[mPoint(1,1)-ballPos(3)/2,mPoint(1,2)-ballPos(4)/2,ballPos(3),ballPos(4)]);
    else
        ballObserved.x(1:2) = mPoint(1,1:2);
        %set(ballPlot,'Position',[-99,-99,ballPos(3),ballPos(4)]);
    end
end

% trackerLoop
% main loop
function trackerLoop(h,ev,ballPlot,trackPlot,bestTrackPlot,dt)
    global balls; global ballVisible; global ballObserved; global ballModes;
    N = size(balls,1); % number of ball particles
    M = size(ballModes,1); % number of ball modes
    ballPos = get(ballPlot,'Position');
    if(ballVisible)
        % determine the observed state [x; y; vx; vy]
        ballPos(1) = ballPos(1)+ballPos(3)/2; % correct for size of ball
        ballPos(2) = ballPos(2)+ballPos(4)/2; % (graphic is offset by 1/2)
        dy = ballPos(2)-ballObserved.x(2);
        dx = ballPos(1)-ballObserved.x(1);
        ballObserved.x(1) = ballPos(1);
        ballObserved.x(2) = ballPos(2);
        ballObserved.x(3) = dx/dt;
        ballObserved.x(4) = dy/dt;
    end
    
    z = ballObserved.x; % observation
    R = diag([0.001 0.001 0.001 0.001]); % R = observation noise
    
    if(ballVisible)
        newBalls = cell(N*M);
        newBallIndex = 0;
        for n = 1 : N % for each ball particle
            weightSum = 0;
            for m = 1 : M % for each ball mode
                newBallIndex = newBallIndex + 1;
                
                % ** EKF Predict **
                [xhat,F,P] = ballModes{m}.motionModel(balls{n},dt);
                
                % ** EKF Update **
                % calculate innovation (measurement residual)
                Y = z - xhat; % Y = z-h(X), h(X)=X;
                % calculate innovation covariance
                H = [...
                        1 0 0 0; % dh(X)/dxhat
                        0 1 0 0; % dh(X)/dyhat
                        0 0 1 0; % dh(X)/dvxhat
                        0 0 0 1; % dh(X)/dvyhat
                    ];           % H = dh(X)/xhat
                S = H*P*H' + R;
                % calculate optimal kalman gain
                K = P*H'*inv(S);
                % update state estimate
                xhat = xhat + K*Y;
                % update covariance estimate
                P = (eye(4) - K*H)*P;

                % calculate weight
                probM = ballModes{balls{n}.m}.transitionProb(m); % probability of switching modes
                obsCov = (S+R + (S+R)').*0.5; % prevent non-pos-semi-def covariance
                probZ = mvnpdf(z,xhat,obsCov); % probability of observation
                weightSum = weightSum+probZ*probM;
                newBalls{newBallIndex}.w = probZ*probM;
                
                % set updated variables
                newBalls{newBallIndex}.x = xhat;
                newBalls{newBallIndex}.P = P;
                newBalls{newBallIndex}.m = m;
            end
            for m = 1 : M % for each ball mode, multiply weights
                newBalls{newBallIndex-m+1}.w = newBalls{newBallIndex-m+1}.w*weightSum;
            end
        end
    else % ball not observed
        newBalls = cell(N);
        for n = 1 : N % for each ball particle
            % draw new mode: m ~ p(m|mprev,ballprev)
            cumSum = cumsum(ballModes{balls{n}.m}.transitionProb);
            randTransition = rand();
            m = 1;
            while(cumSum(m) < randTransition)
                m = m + 1;
            end
            
            % ** EKF Predict **
            [xhat,F,P] = ballModes{m}.motionModel(balls{n},dt);
            newBalls{n}.x = xhat;
            newBalls{n}.P = P;
            newBalls{n}.m = m;
            newBalls{n}.w = 1/N;
        end
    end
    
    % resample N balls from newBalls
    balls = resampleBalls(newBalls, N, 1.0*N);
    
    % plot particles drawn from each distribution
    numPlotParticles = 100;
    X = zeros(N*numPlotParticles,size(newBalls{n}.x,1));
    meanX = [0;0;0;0];
    for n = 1 : N
        meanX = meanX + newBalls{n}.x;
        PDraw = (newBalls{n}.P + newBalls{n}.P').*0.5; % prevent non-positive-semi-definite P1;
        X((n-1)*numPlotParticles+1:n*numPlotParticles,:) = mvnrnd(newBalls{n}.x,PDraw,numPlotParticles);
    end
    set(trackPlot,'xdata',X(:,1),'ydata', X(:,2));
    meanX = meanX ./ N;
    set(bestTrackPlot,'xdata',meanX(1),'ydata', meanX(2));
end

% stopSimulation(timer)
% stops the simulation by stopping the timer object.
function stopSimulation(h,ev,timer)
    stop(timer);
    disp('stopped.');
end

% makeBallInvisible
% simple function to set the visibility of the ball to on/off
function makeBallInvisible(h,ev)
    global ballVisible;
    ballVisible = ~ballVisible;
end

% createModes()
% creates the "modes" the ball can have.  This includes transition
% probabilities to other modes, and a motion model for an EKF.
function ballModes = createModes()
    ballModes = cell(2);
    
    % Ball mode for no interactions (just standard velocity motion update)
    ballModes{1}.name = 'none';
    ballModes{1}.transitionProb = [0.9 0.1]; % probability for transitioning to other modes
    ballModes{1}.motionModel = @motionModelNone;
    function [xhat,F,P] = motionModelNone(ball,dt)
        % setup variables
        % (note: state X = [x; y; vx; vy])
        x = ball.x(1); y = ball.x(2); vx = ball.x(3); vy = ball.x(4);
        Q = dt.*diag([0.1 0.1 0.1 0.1]); % Q = process noise
        % ** EKF Predict **
        % predict next state.
        xNew = x + vx*dt;   % f(x) = x + vx*dt
        yNew = y + vy*dt;   % f(y) = y + vy*dt
        vxNew = vx;         % f(vx) = vx
        vyNew = vy;         % f(vy) = vy
        xhat = [xNew; yNew; vxNew; vyNew]; % xhat = f(X)
        % predict estimated covariance.
        F = [... 
                1 0 dt 0; % df(X)/dx
                0 1 0 dt; % df(X)/dy
                0 0 1 0;  % df(X)/dvx
                0 0 0 1;  % df(X)/dvy
            ];            % F = df(X)/dX
        P = F*ball.P*F' + Q;
    end

    % Ball mode for deflections (sharp changes in position&velocity)
    ballModes{2}.name = 'deflection';
    ballModes{2}.transitionProb = [1.0 0.0]; % probability for transitioning to other modes
    ballModes{2}.motionModel = @motionModelDeflection;
    function [xhat,F,P] = motionModelDeflection(ball,dt)
        % setup variables
        % (note: state X = [x; y; vx; vy])
        x = ball.x(1); y = ball.x(2); vx = ball.x(3); vy = ball.x(4);
        Q = dt.*diag([1.0001 1.0001 1.0001 1.0001]); % Q = process noise
        % ** EKF Predict **
        % predict next state.
        xNew = x + vx*dt;   % f(x) = x + vx*dt
        yNew = y + vy*dt;   % f(y) = y + vy*dt
        vxNew = vx;         % f(vx) = vx
        vyNew = vy;         % f(vy) = vy
        xhat = [xNew; yNew; vxNew; vyNew]; % xhat = f(X)
        % predict estimated covariance.
        F = [... 
                1 0 dt 0; % df(X)/dx
                0 1 0 dt; % df(X)/dy
                0 0 1 0;  % df(X)/dvx
                0 0 0 1;  % df(X)/dvy
            ];            % F = df(X)/dX
        P = F*ball.P*F' + Q;
    end
end

% resampleBalls
% standard resample procedure.
function newBalls = resampleBalls(balls, N, nThresh)
    M = size(balls,1); % current number of particles (N is final number)
    
    % Generate N uniform subintervals of (0, 1]
    intervals = 0:(1/N):1;
    % Draw a sample from each interval
    U = intervals(1:N) + (1/N)*rand(1,N);
    weightSum = 0;
    for m = 1 : M, weightSum = weightSum + balls{m}.w; end

    % Normalize weights and calculate effective particle count
    Neff = 0;
    for m = 1 : M
        if(weightSum==0)
            balls{m}.w = 1/N;
        else
            balls{m}.w = balls{m}.w / weightSum;
        end
        Neff = Neff + (balls{m}.w).^2;
    end
    Neff = 1/Neff;

    % Calculate the cumulative sum of the weights
    cumulativeW = zeros(1,M);
    weightSum = 0;
    for m = 1 : M
        weightSum = weightSum + balls{m}.w;
        cumulativeW(m) = weightSum;
    end
    
    if(Neff < nThresh || M~=N)
        % Select the weight index which is responsible for each U value of 
        % the cumulative sum
        resampleIndex = zeros(1,N);
        for n = 1 : N
            j = 1;
            while cumulativeW(j) < U(n)
                j = j + 1;
            end
            resampleIndex(n) = j;
        end
        % Create new particle array from existing particles, and sampled indexes
        newBalls = cell(N);
        for n = 1 : N
            newBalls{n}.x = balls{resampleIndex(n)}.x;
            newBalls{n}.P = balls{resampleIndex(n)}.P;
            newBalls{n}.m = balls{resampleIndex(n)}.m;
            newBalls{n}.w = 1/N;
        end
    else
        newBalls = balls;
    end
end