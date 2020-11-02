I have implemented A* search algorithm to traverse through a digital map representation of Mendon ponds park. I was provided with the image of the map and the elevation details of the terrain in a text file.
I have implemented A* search algorithm in Python. I used the Pillow library of python to read, load, open and access the image in an ‘Pixel Access’ object. With the pixel access object I manipulated the data in such a way that, if you were to traverse the actual park, then from point A to point B, my search will guide you at every step and would take you to point B in shortest path, with considering the factors such as, the overall distance of the next point to point B, the type of terrain of the next point assigning a speed factor, the elevation difference between the current and next point(uphill or downhill). This is the description of my heuristic function.
For overall distance of the point to point B, I used Euclidean distance formula between the next point and point B, with the pixel co-ordinates.
For speed factor based on terrain I used the following table
Terrain Type	Speed factor
Open Land	1
Rough Meadow	6
Easy Movement Forest	4
Slow Run Forest	5
Walk Forest	7
Impassable Vegetation	9
Lake/Swamp/Marsh	8
Paved road	2
Footpath	3
Out of Bounds	10000000000000

For the elevation difference, I calculated the difference in elevation between the next point and current point.
Then adding up all these values gives me my cost function.
I was able to traverse a classic event in under the acceptable time limit of 2 seconds. In classic event I traversed from the start location to end location though the list of controls in sequence of the route.

For the search for other person, I have considered the other person as a teenage kid.
I have changed the heuristic values as:
Terrain Type	Speed factor
Open Land	1
Rough Meadow	7
Easy Movement Forest	5
Slow Run Forest	6
Walk Forest	8
Impassable Vegetation	10
Lake/Swamp/Marsh	10
Paved road	1
Footpath	1
Out of Bounds	10000000000000

Other factors are the same.

![alt text](https://github.com/AkhilPawar55/A-/blob/master/AnotherPersonWhiteRoute.png?raw=true)
Another person traversing white route

The route path taken by other person is shown in magenta and my path is shown in red. You can see that the teenage kid quickly traverses through open land and on footpaths but has hard time in slow walk forests and rough meadow.
