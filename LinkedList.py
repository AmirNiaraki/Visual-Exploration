import math 

# Representation of a node 
class Node: 
	def __init__(self, data): 
		self.data = data 
		self.next = None

# Function to insert node 
def insert(root, item): 
	temp = Node(item) 
	
	if (root == None): 
		root = temp 
	else : 
		ptr = root 
		while (ptr.next != None): 
			ptr = ptr.next
		ptr.next = temp 
	
	return root 

def display(root): 
	while (root != None) : 
		print(root.data, end = " ") 
		root = root.next
		
def arrayToList(arr, n): 
	root = None
	for i in range(0, n, 1): 
		root = insert(root, arr[i]) 
	
	return root 

# Driver code 
if __name__=='__main__': 
	arr = [(0,0),(5,5)] 
	n = len(arr) 
	root = arrayToList(arr, n) 
	display(root) 
# This code is contributed by Srathore 
