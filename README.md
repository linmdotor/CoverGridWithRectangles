# Cover Grid With Rectangles
Algorithm to cover an area with the minimun number of non colliding rectangles

Description
-----------
We have to solve the following problem:

We have an area (represented by a 2-dimensional array) with some of his positions in blank and other marked as occupied.

Something like this:
{1,0,0,0,0,1},
{0,0,0,0,0,1},
{0,0,0,0,0,1},
{0,0,0,1,1,1},
{0,0,0,1,1,1},
(0s represent the blanks, and 1s the occupied)

We must find one of the optimal solutions that fills all the blanks with the minimal number of rectangles. These rectangles can't overlap each other.

In this case...
- 3 rectangles:
	1-(from [0][1] to [0][4])
	2-(from [1][0] to [2][4])
	3-(from [3][0] to [4][2])

This will not be used in runtime, thus optimizing the speed or cost of the algorithm is not a priority. 

-------------------------------------------------------
Lin M. Dotor © 2017
