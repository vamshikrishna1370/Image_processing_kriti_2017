alex = alexnet;
layers = alex.Layers
%%
layers(23) = fullyConnectedLayer(2);
layers(25) = classificationLayer
%%
allImages = imageDatastore('TrainingData', 'IncludeSubfolders', true, 'LabelSource', 'foldernames');
[trainingImages, testImages] = splitEachLabel(allImages, 0.8, 'randomize');
%%
opts = trainingOptions('sgdm', 'InitialLearnRate', 0.1, 'MaxEpochs', 20, 'MiniBatchSize', 64,'OutputFcn',@(info)stopIfAccuracyNotImproving(info,2));

trainingImages.ReadFcn = @readFunctionTrain;
myNet = trainNetwork(trainingImages, layers, opts);

%%
testImages.ReadFcn = @readFunctionTrain;
predictedLabels = classify(myNet, testImages);
accuracy = mean(predictedLabels == testImages.Labels)
