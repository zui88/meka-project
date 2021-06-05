## @desciption Algorithmus beachtet nur ein Objekt,
##             das mit dem hoechsten Score
## @return x_ x Position von Bounding Box (BB)
## @return y_ y Position von Bounding Box (BB)
## @return width_ Breiter der BB
## @return height_ Hohe der BB

function [x_ y_ width_ height_ score_] = detectFunction()
%#codegen

    persistent mynet
    persistent hwobj
    persistent cam

    if isempty(mynet) || isempty(hwobj) || isempty(cam)
      hwobj      = jetson;
      cam        = camera(hwobj, "vi-output, imx219 6-0010", [1280 720]);
      mynet      = coder.loadDeepLearningNetwork('yoloNetwork.mat');
    end

      img = snapshot(cam);
      img = imresize(img, [224 224]);

      [bboxes scores] = detect(mynet, img, "Threshold", 0.5);

      % only the one with the highest score
      [score_ idx] = max(scores);

      if ~isempty(bboxes)
        tmp_bbox = bboxes(idx,:);
        x_ = tmp_bbox(1);
        y_ = tmp_bbox(2);
        width_  = tmp_bbox(3);
        height_ = tmp_bbox(4);
      else
        % nothing detected
        x_ = -1;
        y_ = -1;
        width_  = -1;
        height_ = -1;
        score_  = single(-1);
      end
end
