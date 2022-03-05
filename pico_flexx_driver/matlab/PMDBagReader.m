classdef PMDBagReader < handle
    properties
        bag
        mono8Msgs
        depthMsgs
        noiseMsgs
        res
    end
    methods
        function obj = PMDBagReader(filename)
            obj.bag = rosbag(filename);
            obj.mono8Msgs = readMessages(select(obj.bag, 'Topic', '/monstar/image_mono8'));
            obj.depthMsgs = readMessages(select(obj.bag, 'Topic', '/monstar/image_depth'));
            obj.noiseMsgs = readMessages(select(obj.bag, 'Topic', '/monstar/image_noise'));
            obj.res = [352, 287];
        end

        function a = noise(obj, i)
            a = double(reshape(typecast(obj.noiseMsgs{i}.Data, 'single'), obj.res(1), obj.res(2)));
        end
        function a = depth(obj, i)
            a = double(reshape(typecast(obj.depthMsgs{i}.Data, 'single'), obj.res(1), obj.res(2)));
        end
        function a = img(obj, i)
            a = double(reshape(typecast(obj.mono8Msgs{i}.Data, 'uint8'), obj.res(1), obj.res(2)));
        end
    end
end