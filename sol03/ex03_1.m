%% load model
[V,F] = openOFF('model.off', '');

%% close figure
close all;

%% rotate / translate vertices

figure
display(V, F)

% rotate first around x, then z
figure
% 先x轴转45度，然后z轴120度
V1 = transform(V,  [deg2rad(45) 0 0           ]', [0 0 0]');
V1 = transform(V1, [0,          0 deg2rad(120)]', [0 0 0]');
display(V1, F)

% rotate first around z, then x --> gives different result, as the order
% in which rotations are applied matters
figure
% 先z轴120度，后x轴转45度
V2 = transform(V,  [0           0 deg2rad(120)]', [0 0 0]');
V2 = transform(V2, [deg2rad(45) 0 0           ]', [0 0 0]');
display(V2, F)

% translate
figure
V3 = transform(V,  [0 0 0]', [0.5 0.2 0.1]');
display(V3, F)

%% display model
function display(V,F)
    C = 0.3*ones(size(V,1),3);
    patch('Vertices', V, 'Faces', F, 'FaceVertexCData', C);
    axis equal;
    shading interp;
    camlight right;
    camlight left;
end