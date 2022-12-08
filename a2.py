class Heap:
    def __init__(self,array=[]):
        self.array = array # underlying list of the Heap
        self.size = len(array) # size of Heap
        self.build_heap() # calling build_heap() on the input list passed in the Heap class

    def heap_up(self,i):
        if i==0: # if the element is the root
            return
        parent = self.array[(i-1)//2]
        element = self.array[i]
        if parent>element: # if the parent of the element is greater than itself, swap the two and call heap_up() on the parent index
            self.array[(i-1)//2],self.array[i] = self.array[i],self.array[(i-1)//2]
            self.heap_up((i-1)//2)

    def heap_down(self,i):
        minindex = i
        leftchild = 2*i+1
        rightchild = 2*i+2

        if leftchild<self.size and self.array[leftchild]<self.array[minindex]: # if the element is greater than its left child (if it exists)
            minindex = leftchild
        if rightchild<self.size and self.array[rightchild]<self.array[minindex]: # if the element is greater than its right child (if it exists)
            minindex = rightchild

        if minindex!=i: # if element is greater than one of its children, swap the two and call heap_down on the index of the child in 'array'
            self.array[minindex],self.array[i] = self.array[i],self.array[minindex]
            self.heap_down(minindex)

    def build_heap(self):
        i = self.size - 1
        while i>=0: # Call heap_down() on every element in 'array' from last to first
            self.heap_down(i)
            i -= 1

    def push(self,element): # add the element in the end of the list and call heap_up() on the last index
        self.array.append(element)
        self.size += 1
        self.heap_up(self.size-1)

    def top(self):
        return self.array[0] # root of the Heap is present at the start of the array

    def pop(self): # first swap the min element(root) with the last element of 'array', then pop the last element and call heap_down() on the root index
        self.array[-1],self.array[0] = self.array[0],self.array[-1]
        self.size -= 1
        self.array.pop()
        self.heap_down(0)

def listCollisions(M,x,v,m,T):
    n = len(M) # no of objects
    time_of_col = [0]*(n-1) # time_of_col[i] denotes the time of next collsion of ith and (i+1)th object (initialised with 0)
    for i in range(n-1):
        if v[i]>v[i+1]: # if relative velocity > 0
            time_of_col[i] = (x[i+1]-x[i])/(v[i]-v[i+1]) # time of collision = relative position/relative velocity

    time_of_movement = [0]*n # time_of_movement[i] denotes the last time the ith object was moved and its position was updated
    L = [] # initializing the output list
    h = Heap([(time_of_col[i], i, x[i]+time_of_col[i]*v[i]) for i in range(n-1) if time_of_col[i]>0]) # creating a heap of the all the initial possible collsions
    no_of_collisions = 0

    while no_of_collisions<m and h.size!=0:
        (t,i,pos) = h.top()
        h.pop()

        if t>T: break # break the loop if time becomes greater than T
        if t!=time_of_col[i] : continue # if the time stored in time_of_col[i] doesn't match t, then ignore this element of the heap as this doesn't indicate a valid collision
        
        # L.append((float("%0.4f"%(t)),i,float("%0.4f"%(pos))))
        L.append((t,i,pos)) # append the collision to the output list

        u1,u2 = v[i],v[i+1]
        x[i] = x[i+1] = pos # updating the position of the colliding objects
        v[i] = ((M[i]-M[i+1])*u1 + 2*M[i+1]*u2)/(M[i]+M[i+1]) # updating velcity of the ith object
        v[i+1] = (2*M[i]*u1 + (M[i+1]-M[i])*u2)/(M[i]+M[i+1]) # updating velocit of the (i+1)th object
        time_of_movement[i] = time_of_movement[i+1] = t # updating the time of movement of these two objects to the current time

        if i>0 and v[i-1]>v[i]: 
            # if the collision of (i-1)th and ith object is possible (relative velocity>0), first update the position and time of movement of (i-1)th object and add the collision data in the heap
            x[i-1] += v[i-1]*(t-time_of_movement[i-1])
            time_of_movement[i-1] = t
            increment = (x[i] - x[i-1])/(v[i-1]-v[i])
            time_of_col[i-1] = t+increment
            h.push((t+increment,i-1,x[i-1]+v[i-1]*increment))

        if i<n-2 and v[i+1]>v[i+2]:
            # if the collision of (i+1)th and (i+2)th object is possible (relative velocity>0), first update the position and time of movement of (i+2)th object and add the collision data in the heap
            x[i+2] += v[i+2]*(t-time_of_movement[i+2])
            time_of_movement[i+2] = t
            increment = (x[i+2] - x[i+1])/(v[i+1]-v[i+2])
            time_of_col[i+1] = t+increment
            h.push((t+increment,i+1,x[i+1]+v[i+1]*increment))

        no_of_collisions += 1
    return L