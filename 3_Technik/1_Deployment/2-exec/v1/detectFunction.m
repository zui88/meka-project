
function detectFunction()
%#codegen

    persistent mynet
    persistent hwobj
    persistent cam
    persistent display

    if isempty(mynet) || isempty(hwobj) || isempty(cam) || isempty(display)
      hwobj      = jetson;
      cam        = camera(hwobj, "vi-output, imx219 6-0010", [1280 720]);
      display    = imageDisplay(hwobj);
      mynet      = coder.loadDeepLearningNetwork('yoloNetwork.mat');
    end

      img = snapshot(cam);
      img = imresize(img, [224 224]);

      [bboxes scores labels] = detect(mynet, img, "Threshold", 0.5);

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
      end
      
      image(display, outImg);
    
end
