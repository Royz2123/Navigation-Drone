fixed = imread('C:\Users\gadyz\Desktop\Roy\t1.png');
moving = imread('C:\Users\gadyz\Desktop\Roy\t30.png');

imshowpair(fixed, moving,'Scaling','joint')

[optimizer, metric] = imregconfig('multimodal');

optimizer.InitialRadius = 0.009;
optimizer.Epsilon = 1.5e-4;
optimizer.GrowthFactor = 1.01;
optimizer.MaximumIterations = 300;

tform = imregtform(moving, fixed, 'affine', optimizer, metric);

movingRegistered = imwarp(moving,tform,'OutputView',imref2d(size(fixed)));

imshowpair(fixed, movingRegistered,'Scaling','joint')
