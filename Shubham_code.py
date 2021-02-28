#importing the necessary libraries
from shapely.geometry import Polygon
import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.image as mpimg 
import math # for using floor, round and other math functions
from queue import Queue  #using queue in BFS in the Wavefront Planner

# Defining each obstacle with four vertices cordinate # cordinates are taken as per the given figure in question
obstacle1 = Polygon([(1,2), (2,2), (2,3), (1, 3)])
obstacle2 = Polygon([(1,-1), (2,-1), (2,0), (1, 0)])
obstacle3 = Polygon([(-2,-1), (-1,-1), (-1,2), (-2, 2)])

# Length and width were not given so I have assumed the values which satisfy the given figure in the question
W1 = 0.35;   #half width of link 1   
L1 = 0.05;   #half thickness of link 1

W2 = 0.35;   #half width of link 2
L2 = 0.05;   #half thickness of link 2


# Making Polygon as a obstacle
obs1 = [(1,2), (2,2), (2,3), (1, 3)]
obs1.append(obs1[0])   #repeating the first point to create a 'closed loop'
xs1, ys1 = zip(*obs1)  #creating lists of x and y values

obs2 = [(1,-1), (2,-1), (2,0), (1, 0)]
obs2.append(obs2[0])  #repeating the first point to create a 'closed loop'
xs2, ys2 = zip(*obs2) #creating lists of x and y values


obs3 = [(-2,-1), (-1,-1), (-1,2), (-2, 2)]
obs3.append(obs3[0])  #repeating the first point to create a 'closed loop'
xs5, ys5 = zip(*obs3) #creating lists of x and y valuwes


theta1 = 60
theta2 = 300
i = (theta1*(np.pi))/180   #converting theta1 degree to radians for theta1
j = (theta2*(np.pi))/180   #converting theta2 degree to radians for theta1
# Finding corresponding four vertices of the each link_1 and link_2
# Link_1 has four vertices pt1, pt2 ,pt3 and pt4 
pt1_x = -L1*np.sin(i) 
pt1_y = L1*np.cos(i)
pt4_x = L1*np.sin(i)
pt4_y = -L1*np.cos(i)
pt2_x = (2*W1*np.cos(i)) - L1*np.sin(i)
pt2_y = (2*W1*np.sin(i)) + L1*np.cos(i)
pt3_x = (2*W1*np.cos(i)) + L1*np.sin(i)
pt3_y = (2*W1*np.sin(i)) - L1*np.cos(i)

# Link_2 has four vertices pt5, pt6 ,pt7 and pt8 
pt5_x = -L2*np.sin(i+j) + (2*W1*np.cos(i)) 
pt5_y =  L2*np.cos(i+j) + (2*W1*np.sin(i)) 
pt8_x =  L2*np.sin(i+j) + (2*W1*np.cos(i))
pt8_y = -L2*np.cos(i+j) + (2*W1*np.sin(i))
pt6_x = (2*W2*np.cos(i+j)) - L2*np.sin(i+j) + (2*W1*np.cos(i))
pt6_y = (2*W2*np.sin(i+j)) + L2*np.cos(i+j) + (2*W1*np.sin(i))
pt7_x = (2*W2*np.cos(i+j)) + L2*np.sin(i+j) + (2*W1*np.cos(i))
pt7_y = (2*W2*np.sin(i+j)) - L2*np.cos(i+j) + (2*W1*np.sin(i))


# for first Link (Link_1) storing the vertices in a 2D array
first_link = [(pt1_x , pt1_y), (pt2_x, pt2_y), (pt3_x, pt3_y), (pt4_x, pt4_y)]
first_link.append(first_link[0]) 
xs3, ys3 = zip(*first_link) 

# for second Link (Link_2) storing the vertices in a 2D array
second_link = [(pt5_x , pt5_y), (pt6_x, pt6_y), (pt7_x, pt7_y), (pt8_x, pt8_y)]
second_link.append(second_link[0]) 
xs4, ys4 = zip(*second_link) 

# Drawing the plot of robot world to display the robot and some polygonal obstacle_1 and obstacle_2
plt.figure()
plt.grid()
plt.fill(xs1,ys1, 'r') 
plt.fill(xs2,ys2, 'b')
plt.fill(xs5,ys5, 'g')
plt.fill(xs3,ys3, 'k')
plt.fill(xs4,ys4, 'k')
plt.xlabel('X -(distnace)') 
plt.ylabel('Y -(distnace)') 
plt.title('2 R Manipulator(in black) and Robot World with obstacles(in red, blue and green)') 
plt.show() 


# Part2 of Assignment finding the Robot End Effector WorkSpace Code
# An array named theta1 storing the angles from 0 to 2PI in increment of 0.1 radians 
theta1 = np.arange(0, 2*(np.pi), np.pi/180)
theta2 = np.arange(0, 2*(np.pi), np.pi/180) #similarly for theta2



# Drawing Configuration Space  #################################
N = 45;   
world = np.zeros((N+1, N+1)) # world is 45 X 45 Matrix Grid with each entry as 0  #1 based indexing 



# Checking collision with obstacle 2
THETA1 = [] # array containing the values of theta1 where collision occurs with the obstacle of link 
THETA2 = [] # array containing the values of theta2 where collision occurs with the obstacle of link

for i in theta1:
    for j in theta2:
    	# find the corresponding coordinate of vertices of link_1 and Link_2 in ground frame
	    pt1_x = -L1*np.sin(i) 
	    pt1_y = L1*np.cos(i)
	    pt4_x = L1*np.sin(i)
	    pt4_y = -L1*np.cos(i)
	    pt2_x = (2*W1*np.cos(i)) - L1*np.sin(i)
	    pt2_y = (2*W1*np.sin(i)) + L1*np.cos(i)
	    pt3_x = (2*W1*np.cos(i)) + L1*np.sin(i)
	    pt3_y = (2*W1*np.sin(i)) - L1*np.cos(i)

	    pt5_x = -L2*np.sin(i+j) + (2*W1*np.cos(i)) 
	    pt5_y =  L2*np.cos(i+j) + (2*W1*np.sin(i)) 
	    pt8_x =  L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt8_y = -L2*np.cos(i+j) + (2*W1*np.sin(i))	    
	    pt6_x = (2*W2*np.cos(i+j)) - L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt6_y = (2*W2*np.sin(i+j)) + L2*np.cos(i+j) + (2*W1*np.sin(i))
	    pt7_x = (2*W2*np.cos(i+j)) + L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt7_y = (2*W2*np.sin(i+j)) - L2*np.cos(i+j) + (2*W1*np.sin(i))

	    rod1 = Polygon([(pt1_x, pt1_y), (pt2_x, pt2_y), (pt3_x, pt3_y), (pt4_x, pt4_y)])
	    rod2 = Polygon([(pt5_x, pt5_y), (pt6_x, pt6_y), (pt7_x, pt7_y), (pt8_x, pt8_y)])
	    if rod1.intersects(obstacle2)  or rod2.intersects(obstacle2) :
	        THETA1.append(i*(180/(np.pi)))
	        THETA2.append(j*(180/(np.pi)))
	        world[math.floor(i*(22.5/(np.pi)))][math.floor(j*(22.5/(np.pi)))] = 1  
        

# plt.plot(THETA1, THETA2, 'bo') 


# Checking collision with obstacle 3
# THETA1 = [] # array containing the values of theta1 where collision occurs with the obstacle of link 
# THETA2 = [] # array containing the values of theta2 where collision occurs with the obstacle of link

for i in theta1:
    for j in theta2:
    	# find the corresponding coordinate of vertices of link_1 and Link_2 in ground frame
	    pt1_x = -L1*np.sin(i) 
	    pt1_y = L1*np.cos(i)
	    pt4_x = L1*np.sin(i)
	    pt4_y = -L1*np.cos(i)
	    pt2_x = (2*W1*np.cos(i)) - L1*np.sin(i)
	    pt2_y = (2*W1*np.sin(i)) + L1*np.cos(i)
	    pt3_x = (2*W1*np.cos(i)) + L1*np.sin(i)
	    pt3_y = (2*W1*np.sin(i)) - L1*np.cos(i)

	    pt5_x = -L2*np.sin(i+j) + (2*W1*np.cos(i)) 
	    pt5_y =  L2*np.cos(i+j) + (2*W1*np.sin(i)) 
	    pt8_x =  L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt8_y = -L2*np.cos(i+j) + (2*W1*np.sin(i))	    
	    pt6_x = (2*W2*np.cos(i+j)) - L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt6_y = (2*W2*np.sin(i+j)) + L2*np.cos(i+j) + (2*W1*np.sin(i))
	    pt7_x = (2*W2*np.cos(i+j)) + L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt7_y = (2*W2*np.sin(i+j)) - L2*np.cos(i+j) + (2*W1*np.sin(i))

	    rod1 = Polygon([(pt1_x, pt1_y), (pt2_x, pt2_y), (pt3_x, pt3_y), (pt4_x, pt4_y)])
	    rod2 = Polygon([(pt5_x, pt5_y), (pt6_x, pt6_y), (pt7_x, pt7_y), (pt8_x, pt8_y)])
	    if rod1.intersects(obstacle3)  or rod2.intersects(obstacle3) :
	        THETA1.append(i*(180/(np.pi)))
	        THETA2.append(j*(180/(np.pi)))
	        world[math.floor(i*(22.5/(np.pi)))][math.floor(j*(22.5/(np.pi)))] = 1  
        

plt.plot(THETA1, THETA2, 'go') 


plt.grid()
plt.xlabel('\u03B81  (in degrees)') 
plt.ylabel('\u03B82  (in degrees)')  
plt.title('Configuration Space ') 

plt.show()




#### Wavefront planner ######################
# Obstacle are given with 2 
for i in range(0, 360, int(math.floor(360/N)) ):
    for j in range(0, 360, int(math.floor(360/N))):
    	if world[int(i/8)][int(j/8)] == 1:
    		obs1 = [(i,j), (i+(360/N)-1,j), (i+(360/N)-1,j+(360/N)-1), (i, j+(360/N)-1)]
    		obs1.append(obs1[0])   #repeating the first point to create a 'closed loop'
    		xs1, ys1 = zip(*obs1)  #creating lists of x and y values
    		plt.fill(xs1,ys1, 'r', alpha=0.7)

    	else:

            obs1 = [(i,j), (i+(360/N)-1,j), (i+(360/N)-1,j+(360/N)-1), (i, j+(360/N)-1)]
            obs1.append(obs1[0])   #repeating the first point to create a 'closed loop'
            xs1, ys1 = zip(*obs1)  #creating lists of x and y values
            plt.fill(xs1,ys1, 'g', alpha=0.7) 
             

    	
    	
# print(world)

def CheckZeroInMat(world):
    for i in range(0, 45, 1):
        for j in range(0, 45, 1):
            if world[i][j] == 0:
                return 1
    return 0

def LiesInRange(x, y):
    if x >= 0 and x < N and y >= 0 and y < N: 
        return 1
    return 0


neighbour = [[1,0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1],[1, -1]]



que = Queue()

# print("\n\nqueue = ")


i = 300- 12
j = 240 -8

world[math.floor(300*N/360)][math.floor(240*N/360)] = 2;
obs1 = [(i,j), (i+(360/N)-1,j), (i+(360/N)-1,j+(360/N)-1), (i, j+(360/N)-1)]
obs1.append(obs1[0])   #repeating the first point to create a 'closed loop'
xs1, ys1 = zip(*obs1)  #creating lists of x and y values
plt.fill(xs1,ys1, 'b', alpha=0.7)

que.put([math.floor(300*N/360),math.floor(240*N/360), 3]);

while que.empty() == 0 : 
    val = que.get()
    x = val[0]; y = val[1]; dist = val[2];
    if x == math.floor(60*N/360) and y == math.floor(60*N/360) : 
        break; 
    # print("Queue(", x, y, dist , ")")
    if LiesInRange(x + 1, y) and world[x + 1][y] == 0: 
        world[x + 1][y] = dist
        que.put([x+1, y, dist+1])
    if LiesInRange(x , y+1) and world[x][y+1] == 0: 
        world[x][y+1] = dist
        que.put([x, y+1, dist+1]);
    if LiesInRange(x-1 , y) and world[x-1][y] == 0: 
        world[x-1][y] = dist;
        que.put([x-1, y, dist+1]);
    if LiesInRange(x , y-1) and world[x][y-1] == 0: 
        world[x][y-1] = dist;
        que.put([x, y-1, dist+1]);
    if LiesInRange(x + 1, y+1) and world[x + 1][y+1] == 0: 
        world[x + 1][y+1] = dist
        que.put([x+1, y+1, dist+1])
    if LiesInRange(x-1 , y+1) and world[x-1][y+1] == 0: 
        world[x-1][y+1] = dist;
        que.put([x-1, y+1, dist+1]);
    if LiesInRange(x-1 , y-1) and world[x-1][y-1] == 0: 
        world[x-1][y-1] = dist;
        que.put([x-1, y-1, dist+1]);
    if LiesInRange(x+1 , y-1) and world[x+1][y-1] == 0: 
        world[x+1][y-1] = dist;
        que.put([x+1, y-1, dist+1]);  


# 0 to 45
for i in range(3, 40, 1): 
    for j in range(3, 40, 1):
        # print("Hello _i = ", i, j)
        x = (i)*int(360/N) -4
        y = (j)*int(360/N) -4
        plt.annotate('%d'%(world[i][j]), xy=(x-2 , y-2))
        
# plt.show()

# starting 
x = math.floor(60*N/360)
y = math.floor(300*N/360)
# q_goal = [math.floor(300), math.floor(240)]
# q_start = [math.floor(60), math.floor(300)]
x_final = math.floor(300*N/360) 
y_final = math.floor(240*N/360) 

count = 100
while x != x_final and y != y_final and count > 0:
    i = 8*x - 8 
    j = 8*y- 8
    print("i = ",i , "j = ", j )
    count = count - 1
    obs1 = [(i,j), (i+(360/N)-1,j), (i+(360/N)-1,j+(360/N)-1), (i, j+(360/N)-1)]
    obs1.append(obs1[0])   #repeating the first point to create a 'closed loop'
    xs1, ys1 = zip(*obs1)  #creating lists of x and y values
    plt.fill(xs1,ys1, 'c', alpha=0.7)

    if LiesInRange(x + 1, y) and world[x + 1][y] < world[x][y] and world[x + 1][y] >= 2 : 
        x = x+1
        continue
    if LiesInRange(x , y+1) and world[x][y+1] < world[x][y] and world[x][y+1] >= 2: 
        y = y+1
        continue
    if LiesInRange(x-1 , y) and world[x-1][y] < world[x][y] and world[x - 1][y] >= 2: 
        x = x -1
        continue
    if LiesInRange(x , y-1) and world[x][y-1] < world[x][y] and world[x][y-1] >= 2: 
        y = y-1
        continue
    if LiesInRange(x + 1, y+1) and world[x + 1][y+1] < world[x][y] and world[x + 1][y+1] >= 2: 
        x = x + 1
        continue 
        y = y+1
    if LiesInRange(x-1 , y+1) and world[x-1][y+1] < world[x][y] and world[x - 1][y+1] >= 2: 
        x = x -1
        continue
        y = y+1
    if LiesInRange(x-1 , y-1) and world[x-1][y-1] < world[x][y] and world[x - 1][y-1] >= 2: 
        x = x - 1
        continue
        y = y -1
    if LiesInRange(x+1 , y-1) and world[x+1][y-1] < world[x][y] and world[x + 1][y-1] >= 2: 
        x = x + 1
        y = y-1
        continue


plt.show()
