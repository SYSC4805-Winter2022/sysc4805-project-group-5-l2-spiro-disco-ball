local Node = {
    value = nil
    prevNode = nil
    nextNode = nil
}

function Node:delete()
    self.prevNode.nextNode = self.nextNode
    self.nextNode.prevNode = self.prevNode
end

function Node:insert(newNode)
    --[[
    if newNode.nextNode != nil || newNode.prevNode != nil then
        error("trying to insert a node that is already in a list") 
    end
    ]]
    
    newNode.nextNode = self.nextNode
    newNode.prevNode = self
    if self.nextNode != nil then
        self.nextNode.prevNode = newNode
    end
    self.nextNode = newNode
end
