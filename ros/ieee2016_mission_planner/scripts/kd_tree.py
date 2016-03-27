import numpy as np
class Node():
    def __init__(self,point,linked_object):
        self.point = np.array(point)
        self.lesser = None
        self.greater = None
        self.d = len(point)
        self.linked_object = linked_object
        # Set to true if the this node is equal to it's parent node on that axis
        self.equal_flag = False

    def __repr__(self):
        if self.linked_object is None:
            return str(self.point)
        return str(self.linked_object) + ":" + str(self.point)

    def dist_to_point(self, point):
        point = np.array(point)
        return np.linalg.norm(self.point-point)

    def search(self, point, axis):
        axis = self.next_axis(axis)

        #print axis,self,"LESSER:",self.lesser,"GREATER:",self.greater
        if point[axis] < self.point[axis]:
            if self.lesser is None:
                # This check needs to be here since we lump axis equal to each other into the greater than branch.
                if self.greater is not None:
                    other_closest = self.greater.search(point,axis)
                    if other_closest[0] < self.dist_to_point(point):
                        return other_closest

                return [self.dist_to_point(point),self]
            else:
                closest = self.lesser.search(point,axis)
                # If this node is closer then the closest then return it as the new closest.
                node_dist = self.dist_to_point(point)
                if node_dist < closest[0]:
                    return [node_dist,self]

                # Is hyperplane in the hypersphere
                if abs(self.point[axis] - point[axis]) < closest[0]:
                    if self.greater is None: return closest
                    other_closest = self.greater.search(point,axis)
                    if other_closest[0] < closest[0]:
                        return other_closest
                    else:
                        return closest

        elif point[axis] >= self.point[axis]:
            #print axis,"+"
            if self.greater is None:
                # if self.lesser is not None:
                #     other_closest = self.lesser.search(point,axis)
                #     if other_closest[0] < self.dist_to_point(point):
                #         return other_closest

                return [self.dist_to_point(point),self]
            else:
                closest = self.greater.search(point,axis)
                # If this node is closer then the closest then return it as the new closest.
                node_dist = self.dist_to_point(point)
                if node_dist < closest[0]:
                    return [node_dist,self]

                # Is hyperplane in the hypersphere
                if abs(self.point[axis] - point[axis]) < closest[0]:
                    if self.lesser is None: return closest
                    other_closest = self.lesser.search(point,axis)
                    if other_closest[0] < closest[0]:
                        return other_closest
                    else:
                        return closest
        return closest

    def next_axis(self,axis):
        axis += 1
        if axis == self.d: axis = 0
        return axis

    def insert(self, new_node, axis):
        axis = self.next_axis(axis)
        if new_node.point[axis] < self.point[axis]:
            if self.lesser is None:
                self.lesser = new_node
                return
            self.lesser.insert(new_node,axis)
        elif new_node.point[axis] >= self.point[axis]:
            if self.greater is None:
                self.greater = new_node
                return
            self.greater.insert(new_node,axis)

        return

    def reset_node(self):
        self.greater = None
        self.lesser = None

class KDTree():
    def __init__(self, duplicate_tolerance = .001):
        # Init with the number of dimensions
        self.base_node = None
        self.nodes = []

        # When checking for duplicates, use this as the tolerance when identifying the same point
        self.duplicate_tolerance = duplicate_tolerance
        #print self.duplicate_tolerance
    def __repr__(self):
        return str(self.nodes)

    def insert(self, point, linked_object=None):
        n = Node(point,linked_object)

        # If this is the first node we are trying to insert make it the base.
        if self.base_node is None:
            self.nodes.append(n) 
            self.base_node = n
            return
        
        self.nodes.append(n)
        self.base_node.insert(n,-1)

    def insert_unique(self, point, linked_object=None):
        n = Node(point,linked_object)

        # If this is the first node we are trying to insert make it the base.
        if self.base_node is None: 
            self.nodes.append(n)
            self.base_node = n
            return

        if self.base_node.search(np.array(point),-1)[0] >= self.duplicate_tolerance:
            self.nodes.append(n)
            self.base_node.insert(n,-1)

    def insert_unique_average(self, point, linked_object=None, new_weight=.1):
        # Behaves similarly to insert_unique but will remake the tree with a weighted average of the new point and old point.
        n = Node(point,linked_object)
        # If this is the first node we are trying to insert make it the base.
        if self.base_node is None: 
            self.nodes.append(n)
            self.base_node = n
            return
        
        # Check if there are any close points, if not add them like normal
        closest_node = self.base_node.search(np.array(point),-1)
        #print closest_node
        if closest_node[0] >= self.duplicate_tolerance:
            #print "adding new."
            self.nodes.append(n)
            self.base_node.insert(n,-1)
        else:
            #print closest_node[1].linked_object
            #print linked_object
            if closest_node[1].linked_object != linked_object:
                #print "objects not same."
                # self.nodes.append(n)
                # self.base_node.insert(n,-1)
                return

            avg_point = (1-new_weight) * self.nodes[self.nodes.index(closest_node[1])].point + (new_weight) * n.point
            n = Node(avg_point,linked_object)
            self.nodes.remove(closest_node[1])
            self.nodes.append(n)
            #print "Averaging new node"
            # Go back through and re-add all the nodes to the tree
            self.base_node = None
            for node in self.nodes:
                node.reset_node()
                if self.base_node == None:
                    self.base_node = node
                    continue
                self.base_node.insert(node,-1)

    def search(self, point):
        if self.base_node is None: 
            print "Empty Tree!"
            return

        return self.base_node.search(np.array(point),-1)

    # def insert_list(self, list):
    #     '''
    #     Generates a new tree with the new list. List elements should be Node objects.
    #     '''
    #     for l in new_list:
    #         self.insert(l.point,linked_object=l.linked_object)
    #     return temp_tree

if __name__ == "__main__":
    k = KDTree(.5)
    k.insert([1],"red")
    k.insert([2],"blue")
    k.insert_unique_average([2.1],"red")
    print k.nodes
