classdef BaseRosNode < RosNodeInterface & robotics.ros.Node
    
    properties(SetAccess = protected)
        subscriberMap
        publisherMap
        servicesClientMap
        servicesServersMap
    end
    
    methods
        function baseRosNodeInstance = BaseRosNode(name)
            baseRosNodeInstance = baseRosNodeInstance@robotics.ros.Node(name);
            baseRosNodeInstance.subscriberMap = containers.Map;
            baseRosNodeInstance.publisherMap = containers.Map;
            baseRosNodeInstance.servicesClientMap = containers.Map;
            baseRosNodeInstance.servicesServersMap = containers.Map;
        end
    end
    
    methods(Access = protected)
        function addSubscribedTopic(instance,topicName,topicMsg,callBackFunction)
            subscriber = robotics.ros.Subscriber(instance,topicName,topicMsg,callBackFunction);
            instance.subscriberMap(topicName) = subscriber;
        end
        
        function addPublisherClient(instance,topicName,topicMsg)
            publisher = robotics.ros.Publisher(instance,topicName,topicMsg);
            instance.publisherMap(topicName) = publisher;
        end    
    end
    
    methods(Static)
        function runNode
            throw(MethodNotImplementedException('runNode','BaseRosNode'));
        end
        
        function subscribeToTopics
            throw(MethodNotImplementedException('subscribeToTopics','BaseRosNode'));
        end
        
        function createPublishers
            throw(MethodNotImplementedException('createPublishers','BaseRosNode'));
        end
        
        function createServices
            throw(MethodNotImplementedException('createServices','BaseRosNode'));
        end
    end
    
end

