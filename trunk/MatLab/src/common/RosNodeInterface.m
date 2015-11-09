classdef RosNodeInterface < handle
    properties
    end
    
    methods(Abstract,Static)
        runNode
        subscribeToTopics
        createPublishers
        createServices
    end
    
end

