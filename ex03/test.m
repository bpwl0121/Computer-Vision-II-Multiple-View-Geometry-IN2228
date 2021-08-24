

%% load model
[V,F] = openOFF('model.off', '');

%% close figure
close all;

%% rotate / translate vertices

figure
display(V, F)
%% display model
function display(V,F)
    C = 0.3*ones(size(V,1),3);
    patch('Vertices', V, 'Faces', F, 'FaceVertexCData', C);
    axis equal;
    shading interp;
    camlight right;
    camlight left;
end


%% display model
function display(V,F)
    C = 0.3*ones(size(V,1),3);
    patch('Vertices', V, 'Faces', F, 'FaceVertexCData', C);
    axis equal;
    shading interp;
    camlight right;
    camlight left;
end