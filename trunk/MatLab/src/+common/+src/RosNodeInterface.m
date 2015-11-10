classdef RosNodeInterface < handle
    
    properties
    end
    
    methods(Abstract)
        subscribeToTopics(instance)
        createPublishers(instance)
        runNode(instance)
        createServices(instance)
    end
end

