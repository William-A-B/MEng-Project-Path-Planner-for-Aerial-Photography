classdef Node
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    properties
        position
        parent
        cost
    end
    methods
        function obj = Node(position, parent, cost)
            obj.position = position;
            obj.parent = parent;
            obj.cost = cost;
        end
    end
end

