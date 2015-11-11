classdef BaseRosNode < robotics.ros.Node & common.src.RosNodeInterface
    
    properties
        subscriberMap
        publisherMap
        servicesClientMap
        servicesServersMap
    end
    
    methods
        %Constructor
        function baseRosNodeInstance = BaseRosNode(name)
            baseRosNodeInstance = baseRosNodeInstance@robotics.ros.Node(name);
            baseRosNodeInstance.subscriberMap = containers.Map;
            baseRosNodeInstance.publisherMap = containers.Map;
            baseRosNodeInstance.servicesClientMap = containers.Map;
            baseRosNodeInstance.servicesServersMap = containers.Map;
        end
    
        function addSubscribedTopic(instance,topicName,topicMsg,callBackFunction)
                subscriber = robotics.ros.Subscriber(instance,...
                    topicName,topicMsg,callBackFunction);
                instance.subscriberMap(topicName) = subscriber;
        end
        
        function addPublisherClient(instance,topicName,topicMsg)
            publisher = robotics.ros.Publisher(instance,topicName,topicMsg);
            instance.publisherMap(topicName) = publisher;
        end
        
        function addServiceClient(instance,serviceName)
            serviceClient = robotics.ros.ServiceClient(instance,serviceName);
            instance.servicesClientMap(serviceName) =  serviceClient;
        end
        
        function addServiceServer(instance,serviceName,serviceType...
            ,callBackFunction)
                serviceServer = robotics.ros.ServiceServer...
                    (instance,serviceName,serviceType,callBackFunction);
                instance.servicesServersMap(serviceName) = serviceServer;
        end
        
        function subscribeToTopics(~)
            throw(common.src.excpetions.MethodNotImplementedException...
                ('subscribeToTopics','BaseRosNode'));
        end
        
        function createPublishers(~)
            throw(common.src.excpetions.MethodNotImplementedException...
            ('createPublishers','BaseRosNode'));
        end
        
        function runNode(~)
            throw(common.src.excpetions.MethodNotImplementedException...
                ('runNode','BaseRosNode'));
        end
        
        function createServices(~)
            throw(common.src.excpetions.MethodNotImplementedException...
                ('createServices','BaseRosNode'));
        end
    end
end