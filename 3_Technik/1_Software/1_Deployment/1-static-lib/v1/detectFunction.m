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
    hwobj      = jetson;
    cam        = camera(hwobj, "vi-output, imx219 6-0010", [1280 720]);
    % just for now results will be displayes
    display    = imageDisplay(hwobj);

    % load the trained network
    if isempty(mynet)
      % todo: yet to specify
      mynet = coder.loadDeepLearningNetwork('yoloNetwork.mat');
    end

    %% doom loop  -> there is no escape
    % must fit in int32 variable
    for i = 1:100

      %% main code  -> for detection  
      img = snapshot(cam);
      img = imresize(img, [224 224]);

      [bboxes scores labels] = detect(mynet, img, "Threshold", 0.1);

      % first output will be discarded
      [~, idx] = max(scores);

      % annotate detecttions in the image
      if ~isempty(bboxes)
        outImg = insertObjectAnnotation(img,...
            "Rectangle", bboxes(idx,:), labels{idx});
      else
          message = {'Nothing Detected'};
          outImg = insertText(img, [10 20], message,...
              'FontSize', 18,'BoxColor', [255 0 0],...     % rot
              'BoxOpacity',0.4,'TextColor',[255 255 255]); % weiss
         %outImg = img;
      end
      
      image(display, outImg);

   end
    
end
