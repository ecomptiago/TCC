classdef MethodNotImplementedException < MException
    methods
        function methodNotImplementedInstance = MethodNotImplementedException(methodName, baseClassName)
            methodNotImplementedInstance = methodNotImplementedInstance@MException('customException:Method:NotImplemented',sprintf('Method %s has to be implemented in classes derivated from class %s', methodName, baseClassName));
        end    
    end
end

