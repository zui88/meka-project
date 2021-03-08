%% Signature
% inputs    inputImgSize:    the input size the network is specified (input layer)
%           imageDispay :    if the captured picture should be dispayed or not 

%function detect-function(inputImgSize,imageDisplay)
function detectFunction()
%#codegen

    % %% standart input values if no are given		  
    % % nargin returns the number of function input arguments given int the
    % % call to the currently executing function.
    % switch nargin
    %   case 0
    %     inputImgSize = [];
    %     imageDisplay = [];
    %   case 1
    %     imageDisplay = [];
    %   case 2
    %   otherwise
    %     error('2 inputs are accepted.')
    % end
    % if isempty(inputImgSize)
    %   inputImgSize = [416 416];
    % end
    % if isempty(lwRadiation)
    %   imageDisplay = true;
    % end

    %% init       -> everything set up

    % Function will be called several times. Just the first time the
    % variable will be initialized mynet. Comparable to static in C or C++
    persistent mynet

    % create a jetson object to access the hardware
    % access data req.
    %hwobj = jetson('192.168.1.6', 'jetson', '1111');
    hwobj = jetson;
    % https://de.mathworks.com/help/supportpkg/nvidia/ref/webcam.webcam.html?searchHighlight=webcam&s_tid=srchtitle
    w     = webcam(hwobj,1);
    % just for now results will be displayes
    d     = imageDisplay(hwobj);

    % load the trained network
    if isempty(mynet)
      % todo: yet to specify
      mynet = coder.loadDeepLearningNetwork('yoloNetwork.mat');
    end

    %% doom loop  -> there is no escape
    % must fit in int32 variable
    for i = 1:1e5

      %% main code  -> for detection  
      img = snapshot(w);
      img = imresize(img, [224 224]);

      [bboxes scores labels] = detect(mynet, img, "Threshold", 0.6);

      [~, idx] = max(scores);

      % annotate detecttions in the image
      if ~isempty(bboxes)
        outImg = insertObjectAnnotation(img,...
            "Rectangle", bboxes(idx,:), labels{idx});
      else
        outImg = img;
      end

      image(d, outImg);

    end
    
end
