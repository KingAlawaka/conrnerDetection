#include "stdafx.h"
#include "main.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include <thread>
#include <vector>
#include <sqlite\sqlite3.h>
#include <time.h>

#define w 400

using namespace cv;
using namespace std;


#pragma region Global Variables

Mat src, src_gray;
Mat dst, detected_edges;
Mat new_image; // color enhaced image
Mat read_image;//image that read from the file
Mat intersection_image;

int edgeThresh = 1;
//int lowThreshold =  22;// 22;//for eye
//int lowThreshold = 66;//43;//for road
int lowThreshold = 12;//for test
int const max_lowThreshold = 200;
int ratios = 3;
int kernel_size = 3;
char* window_name = "Edge Map";
char* window_name2 = "Color Enhancement";

double alpha = 1; /**< Simple contrast control alpha value [1.0-3.0]*/
int beta = 0;  /**< Simple brightness control beta value [0-100]*/

int imageRows = 0;
int imageColumns = 0;

ofstream edgeMapFile;
ofstream particalPathfile;

vector<vector<int> > v2d;
vector<int> tempVector; //col
vector<vector<int> > edgeMapVector; //rows

vector<vector<int> > leftSeedPoints;
vector<vector<int> > rightSeedPoints;
vector<vector<int> > upSeedPoints;
vector<vector<int> > downSeedpoint;

vector<int> pathCoordinatesX;
vector<int> pathCoordinatesY;

//Mat image = imread("im0013.JPG");
//Mat image = imread("test2.PNG");
//Mat image = imread("road3.jpg");
//Mat image = imread("corner_test_fixed_bend/test.png");
//Mat image = imread("imageTest/blocksTest.png");
//Mat image = imread("test3/noise/im1_n_1.png");
//Mat image = imread("mathlab1.PNG");
Mat image = imread("iphone images/IMG_0017.jpg");

sqlite3 *db;
char *zErrMsg = 0;
int rc;
char *sql;

int similarMovementRetValue = 0;
char direction;


vector<Point2f> pts_src;
vector<Point2f> pts_dst;

#pragma endregion


#pragma region Methods()

void insertCornerPoints(int particalID, int x, int y);
void CheckCornerPoints(int particalID, int x, int y,int blockCategory, char direction);
int checkEmptySpace(int i, int j, char direction);

/*Possible Return types U UL UR D DL DR L R */
/*Only four possible movement types L R U D*/

void CheckCornerPoints(int particalID, int x, int y, int blockCategory,char direction)
{
	/* blockCategory 
		=1	three consecutive blocks
		=2	four blocks and looking for less than 10 count for all four directions
		=3  three blocks and less than 1 to 2 counts atleast 3 directions
		=4	five blocks or more filled*/
	int l = 0;
	int r = 0;
	int u = 0;
	int d = 0;
	int directionCount = 0;
	int case1Limit = 3;//5
	int case2Limit = 3;//4
	int case3Limit = 2;//2
	int case4Limit = 3;

	int case2Limit2 = 0;//4
	int case3Limit2 = 0;//2
	int case4Limit2 = 0;

	int left = 0;
	int right = 0;
	int up = 0;
	int down = 0;


	switch (blockCategory)
	{
	case 1:
		switch (direction)
		{
		case 'U':
			l = checkEmptySpace(x, y, 'L');
			r = checkEmptySpace(x, y, 'R');
			if (l <= case1Limit && r <= case1Limit)
			{
				insertCornerPoints(particalID, x, y);
			}
			else
			{
				CheckCornerPoints(particalID, x, y, 3, direction);
			}
			break;
		case 'D':
			l = checkEmptySpace(x, y, 'L');
			r = checkEmptySpace(x, y, 'R');
			if (l <= case1Limit && r <= case1Limit)
			{
				insertCornerPoints(particalID, x, y);
			}
			else
			{
				CheckCornerPoints(particalID, x, y, 3, direction);
			}
			break;
		case 'R':
			u = checkEmptySpace(x, y, 'U');
			d = checkEmptySpace(x, y, 'D');
			if (u <= case1Limit && d <= case1Limit)
			{
				insertCornerPoints(particalID, x, y);
			}
			else
			{
				CheckCornerPoints(particalID, x, y, 3, direction);
			}
			break;
		case 'L':
			u = checkEmptySpace(x, y, 'U');
			d = checkEmptySpace(x, y, 'D');
			if (u <= case1Limit && d <= case1Limit)
			{
				insertCornerPoints(particalID, x, y);
			}
			else
			{
				CheckCornerPoints(particalID, x, y, 3, direction);
			}
			break;
		default:
			break;
		}
		break;
	case 2:
		l = checkEmptySpace(x, y, 'L');
		r = checkEmptySpace(x, y, 'R');
		d = checkEmptySpace(x, y, 'D');
		u = checkEmptySpace(x, y, 'U');
		/*if(l<=10 && r <= 10 && d <= 10 && u <= 10)
			insertCornerPoints(particalID, x, y);*/
		if (l <= case2Limit)
		{
			directionCount++;
			left = 1;
		}
			

		if (r <= case2Limit)
		{
			directionCount++;
			right = 1;
		}

		if (d <= case2Limit)
		{
			directionCount++;
			down = 1;
		}

		if (u <= case2Limit)
		{
			directionCount++;
			up = 1;
		}

		if (directionCount >= 3)
		{
			if(directionCount != 4 && left == 1 && right == 1 && up == 1 && down <= case2Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 && left == 1 && right == 1 && down == 1 &&  up <= case2Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 && left == 1 &&  down == 1 && up == 1 && right <= case2Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 &&  down == 1 && right == 1 && up == 1 && left <= case2Limit2)
				insertCornerPoints(particalID, x, y);
			else 
				insertCornerPoints(particalID, x, y);
		}
		break;
	case 3:
		l = checkEmptySpace(x, y, 'L');
		r = checkEmptySpace(x, y, 'R');
		d = checkEmptySpace(x, y, 'D');
		u = checkEmptySpace(x, y, 'U');

		if (l <= case3Limit)
			directionCount++;

		if (r <= case3Limit)
			directionCount++;

		if (d <= case3Limit)
			directionCount++;

		if (u <= case3Limit)
			directionCount++;

		if (directionCount >= 3)
		{
			if (directionCount != 4 && left == 1 && right == 1 && up == 1 && down <= case3Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 && left == 1 && right == 1 && down == 1 && up <= case3Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 && left == 1 && down == 1 && up == 1 && right <= case3Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 && down == 1 && right == 1 && up == 1 && left <= case3Limit2)
				insertCornerPoints(particalID, x, y);
			else
				insertCornerPoints(particalID, x, y);
		}
		break;
	case 4:
		l = checkEmptySpace(x, y, 'L');
		r = checkEmptySpace(x, y, 'R');
		d = checkEmptySpace(x, y, 'D');
		u = checkEmptySpace(x, y, 'U');
		/*if(l<=10 && r <= 10 && d <= 10 && u <= 10)
		insertCornerPoints(particalID, x, y);*/
		if (l <= case4Limit)
			directionCount++;

		if (r <= case4Limit)
			directionCount++;

		if (d <= case4Limit)
			directionCount++;

		if (u <= case4Limit)
			directionCount++;

		if (directionCount >= 3)
		{
			if (directionCount != 4 && left == 1 && right == 1 && up == 1 && down <= case4Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 && left == 1 && right == 1 && down == 1 && up <= case4Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 && left == 1 && down == 1 && up == 1 && right <= case4Limit2)
				insertCornerPoints(particalID, x, y);
			else if (directionCount != 4 && down == 1 && right == 1 && up == 1 && left <= case4Limit2)
				insertCornerPoints(particalID, x, y);
			else
				insertCornerPoints(particalID, x, y);
		}
		break;
	default:
		break;
	}
}

int checkEmptySpace(int i, int j, char direction)
{
	cout << "check empty " << to_string(i) << "\t" << to_string(j) << endl;
	int temp = 0;
	int count = 1;
	switch (direction) {
	case 'U':
		while (temp != 255)
		{
			if (i - count >= 0)
			{
				//cout << "check empty " << to_string(i) << "\t" << to_string(j) << endl;
				temp = edgeMapVector[i - count][j];
				count++;
			}
			else
			{
				break;
			}

		}
		break;
	case 'D':
		while (temp != 255)
		{
			if (i + count < edgeMapVector.size())
			{
				//cout << "check empty " << to_string(i) << "\t" << to_string(j) << endl;
				temp = edgeMapVector[i + count][j];
				count++;
			}
			else
			{
				break;
			}

		}
		break;
	case 'L':
		while (temp != 255)
		{
			if (j - count >= 0)
			{
				//cout << "check empty " << to_string(i) << "\t" << to_string(j) << endl;
				temp = edgeMapVector[i][j - count];
				count++;
			}
			else
			{
				break;
			}

		}
		break;
	case 'R':
		while (temp != 255)
		{
			if (j + count < edgeMapVector[0].size())
			{
				//cout << "check empty " << to_string(i) << "\t" << to_string(j) << endl;
				temp = edgeMapVector[i][j + count];
				count++;
			}
			else
			{
				break;
			}
		}
		break;
	}
	cout << "check empty " << direction << " ret" << endl;
	return count-2; //in the loop we have count++ so then loop exit count is +1 and also we start from count=1 to remove that need to add -2 in the last return

}

char* isValidMoveAvailable(int pid, int i, int j, char particalDirection)
{

#pragma region CheckCorner Logic without movement condition

	int countOfBlocks=0;

	if (i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j - 1] == 255)
	{
		countOfBlocks++;
	}

	if (i < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255)
	{
		countOfBlocks++;
	}

	if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i + 1][j - 1] == 255)
	{
		countOfBlocks++;
	}

	if (i - 1 < edgeMapVector.size() && j < edgeMapVector[0].size() && i - 1 >= 0 && j >= 0 && edgeMapVector[i - 1][j] == 255)
	{
		countOfBlocks++;
	}

	if (i + 1 < edgeMapVector.size() && j < edgeMapVector[0].size() && i + 1 >= 0 && j >= 0 && edgeMapVector[i + 1][j] == 255)
	{
		countOfBlocks++;
	}

	if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i - 1][j + 1] == 255)
	{
		countOfBlocks++;
	}

	if (i < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255)
	{
		countOfBlocks++;
	}

	if (i + 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j + 1] == 255)
	{
		countOfBlocks++;
	}

	if (countOfBlocks == 4)
	{
		switch (direction)
		{
			case 'U':
			if (i - 1 < edgeMapVector.size() && j < edgeMapVector[0].size() && i - 1 >= 0 && j >= 0 && edgeMapVector[i - 1][j] == 255)
				CheckCornerPoints(pid, i, j, 2, direction);
			break;
			case 'D':
				if (i + 1 < edgeMapVector.size() && j < edgeMapVector[0].size() && i + 1 >= 0 && j >= 0 && edgeMapVector[i + 1][j] == 255)
					CheckCornerPoints(pid, i, j, 2, direction);
				break;
			case 'R':
				if (i < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255)
					CheckCornerPoints(pid, i, j, 2, direction);
				break;
			case 'L':
				if (i < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255)
					CheckCornerPoints(pid, i, j, 2, direction);
				break;
		}
		
	}
	else if (countOfBlocks >= 5)
	{
		switch (direction)
		{
		case 'U':
			if (i - 1 < edgeMapVector.size() && j < edgeMapVector[0].size() && i - 1 >= 0 && j >= 0 && edgeMapVector[i - 1][j] == 255)
				CheckCornerPoints(pid, i, j, 4, direction);
			break;
		case 'D':
			if (i + 1 < edgeMapVector.size() && j < edgeMapVector[0].size() && i + 1 >= 0 && j >= 0 && edgeMapVector[i + 1][j] == 255)
				CheckCornerPoints(pid, i, j, 4, direction);
			break;
		case 'R':
			if (i < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255)
				CheckCornerPoints(pid, i, j, 4, direction);
			break;
		case 'L':
			if (i < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255)
				CheckCornerPoints(pid, i, j, 4, direction);
			break;
		}
		
	}
	else if (countOfBlocks == 3)
	{
		switch (direction)
		{
		case 'U':
			if (i - 1 < edgeMapVector.size() && j < edgeMapVector[0].size() && i - 1 >= 0 && j >= 0 && edgeMapVector[i - 1][j] == 255)
				CheckCornerPoints(pid, i, j, 3, direction);
			break;
		case 'D':
			if (i + 1 < edgeMapVector.size() && j < edgeMapVector[0].size() && i + 1 >= 0 && j >= 0 && edgeMapVector[i + 1][j] == 255)
				CheckCornerPoints(pid, i, j, 3, direction);
			break;
		case 'R':
			if (i < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255)
				CheckCornerPoints(pid, i, j, 3, direction);
			break;
		case 'L':
			if (i < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255)
				CheckCornerPoints(pid, i, j, 3, direction);
			break;
		}
	}
#pragma endregion

	switch (direction) {
	case 'U':
		if (i - 1 <= 0 || j + 1 >= edgeMapVector[0].size() || j - 1 <= 0)
		{
			return "0";
		}
		else
		{

			if (j < edgeMapVector[0].size() && i - 1 < edgeMapVector.size() && edgeMapVector[i - 1][j] != 255)
			{
				//cout << to_string(i) << "\t" << to_string(j) << "\tU_4" << endl;
				return "U";
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i][j + 1] != 255 && edgeMapVector[i][j - 1] != 255)
			{
				CheckCornerPoints(pid, i, j, 1, 'U');
				int c1 = checkEmptySpace(i, j, 'L');
				int c2 = checkEmptySpace(i, j, 'R');

				if (c1 >= c2)
				{
					direction = 'L';
					return "L";
				}
				else
				{
					direction = 'R';
					return "R";
				}

			}//5
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i][j + 1] != 255)
			{
				direction = 'R';
				CheckCornerPoints(pid, i, j, 2, 'U');
				/*insertCornerPoints(pid, i, j + 1);*/
				//cout << to_string(i) << "\t" << to_string(j) << "\tR_41" << endl;
				return "R";//5
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i][j - 1] != 255)
			{
				direction = 'L';
				CheckCornerPoints(pid, i, j, 2, 'U');
				/*insertCornerPoints(pid, i, j - 1);*/
				//cout << to_string(i) << "\t" << to_string(j) << "\tL_42" << endl;
				return "L";//5
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i - 1][j - 1] != 255)
			{
				CheckCornerPoints(pid, i, j, 3, 'U');
				//cout << to_string(i) << "\t" << to_string(j) << "\tUL_43" << endl;
				return "UL";//4
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j + 1] != 255)
			{
				CheckCornerPoints(pid, i, j, 3, 'U');
				//cout << to_string(i) << "\t" << to_string(j) << "\tUR_44" << endl;
				return "UR";//4
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i][j + 1] != 255)
			{
				direction = 'R';
				CheckCornerPoints(pid, i, j, 3, 'U');
				/*insertCornerPoints(pid, i, j + 1);*/
				//cout << to_string(i) << "\t" << to_string(j) << "\tR_45" << endl;
				return "R";//4
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i][j - 1] != 255)
			{
				direction = 'L';
				CheckCornerPoints(pid, i, j, 3, 'U');
				/*insertCornerPoints(pid, i, j - 1);*/
				//cout << to_string(i) << "\t" << to_string(j) << "\tL_46" << endl;
				return "L";//4
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i][j - 1] == 255)
			{
				direction = 'D';
				CheckCornerPoints(pid, i, j, 3, 'U');
				/*insertCornerPoints(pid, i + 1, j);*/
				//cout << to_string(i) << "\t" << to_string(j) << "\tD_47" << endl;
				return "D";//3
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j + 1] != 255)
			{

				//cout << to_string(i) << "\t" << to_string(j) << "\tUR_48" << endl;
				return "UR";//3
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i - 1][j - 1] != 255)
			{
				//cout << to_string(i) << "\t" << to_string(j) << "\tUL_49" << endl;
				return "UL";//3
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i - 1][j - 1] != 255)
			{
				//cout << to_string(i) << "\t" << to_string(j) << "\tUL_491" << endl;
				return "UL";//3
			}
			else if (i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i - 1][j + 1] != 255)
			{
				//cout << to_string(i) << "\t" << to_string(j) << "\tUR_492" << endl;
				return "UR";//3
			}
			else if (j + 1 < edgeMapVector[0].size() && i + 1 < edgeMapVector.size() && edgeMapVector[i + 1][j + 1] != 255)
			{
				direction = 'D';
				//cout << to_string(i) << "\t" << to_string(j) << "\tDR_493" << endl;
				return "DR";
			}
			else if (j - 1 < edgeMapVector[0].size() && i + 1 < edgeMapVector.size() && edgeMapVector[i + 1][j - 1] != 255)
			{
				direction = 'D';
				//cout << to_string(i) << "\t" << to_string(j) << "\tDL_494" << endl;
				return "DL";
			}
			else if (j < edgeMapVector[0].size() && i + 1 < edgeMapVector.size() && edgeMapVector[i + 1][j] != 255)
			{
				direction = 'D';
				/*insertCornerPoints(pid, i + 1, j);*/
				//cout << to_string(i) << "\t" << to_string(j) << "\tD_495" << endl;
				return "D";
			}
			else
				return "0";
		}
		break;
	case 'D':
		if (i + 1 >= edgeMapVector.size() || j + 1 >= edgeMapVector[0].size() || j - 1 <= 0)
		{
			return "0";
		}
		else if (j < edgeMapVector[0].size() && i + 1 < edgeMapVector.size() && edgeMapVector[i + 1][j] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tD_3" << endl;
			return "D";
		}
		else if (i + 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i][j + 1] != 255 && edgeMapVector[i][j - 1] != 255)
		{
			CheckCornerPoints(pid, i, j, 1, 'D');

			int c1 = checkEmptySpace(i, j, 'L');
			int c2 = checkEmptySpace(i, j, 'R');

			if (c1 >= c2)
			{
				direction = 'L';
				return "L";
			}
			else
			{
				direction = 'R';
				return "R";
			}

		}//5
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i][j - 1] != 255)
		{
			direction = 'L';
			CheckCornerPoints(pid, i, j, 2, 'D');
			/*insertCornerPoints(pid, i, j - 1);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tL_31" << endl;
			return "L";//4
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i][j + 1] != 255)
		{
			direction = 'R';
			CheckCornerPoints(pid, i, j, 2, 'D');
			/*insertCornerPoints(pid, i, j + 1);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tR_32" << endl;
			return "R";//4
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i + 1][j + 1] != 255)
		{
			CheckCornerPoints(pid, i, j, 3, 'D');
			//cout << to_string(i) << "\t" << to_string(j) << "\tDR_33" << endl;
			return "DR";//4
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i + 1][j - 1] != 255)
		{
			CheckCornerPoints(pid, i, j, 3, 'D');
			//cout << to_string(i) << "\t" << to_string(j) << "\tDL_34" << endl;
			return "DL";//4
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i][j + 1] != 255)
		{
			direction = 'R';
			CheckCornerPoints(pid, i, j, 3, 'D');
			/*insertCornerPoints(pid, i, j + 1);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tR_35" << endl;
			return "R";//4
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i][j - 1] != 255)
		{
			direction = 'L';
			CheckCornerPoints(pid, i, j, 3, 'D');
			/*insertCornerPoints(pid, i, j - 1);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tL_36" << endl;
			return "L";//4
		}
		else if (i + 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && j + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i][j - 1] == 255)
		{
			direction = 'U';
			CheckCornerPoints(pid, i, j, 3, 'D');
			/*insertCornerPoints(pid, i - 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tU_37" << endl;
			return "U";//3
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i + 1][j + 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tDR_38" << endl;
			return "DR";//3
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i + 1][j - 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tDL_39" << endl;
			return "DL";//3
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i + 1][j + 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tDR_391" << endl;
			return "DR";//3
		}
		else if (i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && j - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i + 1][j - 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tDL_392" << endl;
			return "DL";//3
		}
		else if (j + 1 < edgeMapVector[0].size() && i - 1 < edgeMapVector.size() && edgeMapVector[i - 1][j + 1] != 255)
		{
			direction = 'U';
			//cout << to_string(i) << "\t" << to_string(j) << "\tUR_393" << endl;
			return "UR";
		}
		else if (j - 1 < edgeMapVector[0].size() && i - 1 < edgeMapVector.size() && edgeMapVector[i - 1][j - 1] != 255)
		{
			direction = 'U';
			//cout << to_string(i) << "\t" << to_string(j) << "\tUL_394" << endl;
			return "UL";
		}
		else if (j < edgeMapVector[0].size() && i - 1 < edgeMapVector.size() && edgeMapVector[i - 1][j] != 255)
		{
			direction = 'U';
			/*insertCornerPoints(pid, i + 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tU_395" << endl;
			return "U";
		}
		else
			return "0";
		break;
	case 'L':
		if (j - 1 <= 0 || i + 1 >= edgeMapVector.size() || i - 1 <= 0)
		{
			return "0";
		}
		else if (j - 1 < edgeMapVector[0].size() && i < edgeMapVector.size() && edgeMapVector[i][j - 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tL_2" << endl;
			return "L";
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i + 1][j] != 255 && edgeMapVector[i - 1][j] != 255)
		{
			CheckCornerPoints(pid, i, j, 1, 'L');
			int c1 = checkEmptySpace(i, j, 'U');
			int c2 = checkEmptySpace(i, j, 'D');

			if (c1 >= c2)
			{
				direction = 'U';
				return "U";
			}
			else
			{
				direction = 'D';
				return "D";
			}

		}//5
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i + 1][j - 1] && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i - 1][j] != 255)
		{
			direction = 'U';
			CheckCornerPoints(pid, i, j, 2, 'L');
			/*insertCornerPoints(pid, i - 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tU_21" << endl;
			return "U";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i + 1][j - 1] && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i + 1][j] != 255)
		{
			direction = 'D';
			CheckCornerPoints(pid, i, j, 2, 'L');
			/*insertCornerPoints(pid, i + 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tD_22" << endl;
			return "D";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i + 1][j] != 255)
		{
			direction = 'D';
			CheckCornerPoints(pid, i, j, 3, 'L');
			/*insertCornerPoints(pid, i + 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tD_23" << endl;
			return "D";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i - 1][j] != 255)
		{
			direction = 'U';
			CheckCornerPoints(pid, i, j, 3, 'L');
			/*insertCornerPoints(pid, i - 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tU_24" << endl;
			return "U";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i - 1][j - 1] != 255)
		{
			CheckCornerPoints(pid, i, j, 3, 'L');
			//cout << to_string(i) << "\t" << to_string(j) << "\tUL_25" << endl;
			return "UL";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i + 1][j - 1] != 255)
		{
			CheckCornerPoints(pid, i, j, 3, 'L');
			//cout << to_string(i) << "\t" << to_string(j) << "\tDL_26" << endl;
			return "DL";//4
		}
		else if (i - 1 < edgeMapVector.size() && i + 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i - 1 >= 0 && i + 1 >= 0 && j - 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i][j - 1] == 255)
		{
			CheckCornerPoints(pid, i, j, 3, 'L');
			direction = 'R';
			/*insertCornerPoints(pid, i, j + 1);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tR_27" << endl;
			return "R";//3
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i + 1][j - 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tDL_28" << endl;
			return "DL";//3
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i - 1][j - 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tUL_29" << endl;
			return "UL";//3
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i + 1][j - 1] == 255 && edgeMapVector[i - 1][j - 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tUL_291" << endl;
			return "UL";//3
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j - 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j - 1 >= 0 && edgeMapVector[i][j - 1] == 255 && edgeMapVector[i - 1][j - 1] == 255 && edgeMapVector[i + 1][j - 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tDL_292" << endl;
			return "DL";//3
		}
		else if (j + 1 < edgeMapVector[0].size() && i + 1 < edgeMapVector.size() && edgeMapVector[i + 1][j + 1] != 255)
		{
			direction = 'R';
			//cout << to_string(i) << "\t" << to_string(j) << "\tDR_293" << endl;
			return "DR";
		}
		else if (j + 1 < edgeMapVector[0].size() && i - 1 < edgeMapVector.size() && edgeMapVector[i - 1][j + 1] != 255)
		{
			direction = 'R';
			//cout << to_string(i) << "\t" << to_string(j) << "\tUR_294" << endl;
			return "UR";
		}
		else if (j + 1 < edgeMapVector[0].size() && i < edgeMapVector.size() && edgeMapVector[i][j + 1] != 255)
		{
			direction = 'R';
			/*insertCornerPoints(pid, i, j + 1);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tR_295" << endl;
			return "R";
		}
		else
			return "0";
		break;
	case 'R':
		if (j + 1 >= edgeMapVector[0].size() || i + 1 >= edgeMapVector.size() || i - 1 <= 0)
		{
			return "0";
		}
		if (j + 1 < edgeMapVector[0].size() && i < edgeMapVector.size() && edgeMapVector[i][j + 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tR_1" << endl;
			return "R"; //0
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i + 1][j] != 255)
		{
			direction = 'D';
			CheckCornerPoints(pid, i, j, 2, 'R');
			/*insertCornerPoints(pid, i + 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tD_11" << endl;
			return "D";//5
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i - 1][j] != 255)
		{
			direction = 'U';
			CheckCornerPoints(pid, i, j, 2, 'R');
			/*insertCornerPoints(pid, i - 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tU_12" << endl;
			return "U";//5
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i + 1][j] != 255 && edgeMapVector[i - 1][j] != 255)
		{
			CheckCornerPoints(pid, i, j, 1, 'R');
			int c1 = checkEmptySpace(i, j, 'U');
			int c2 = checkEmptySpace(i, j, 'D');

			if (c1 >= c2)
			{
				direction = 'U';
				return "U";
			}
			else
			{
				direction = 'D';
				return "D";
			}

		}//5
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i - 1][j + 1] != 255)
		{
			CheckCornerPoints(pid, i, j, 3, 'R');
			//cout << to_string(i) << "\t" << to_string(j) << "\tUR_13" << endl;
			return "UR";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i + 1][j + 1] != 255)
		{
			CheckCornerPoints(pid, i, j, 3, 'R');
			//cout << to_string(i) << "\t" << to_string(j) << "\tDR_14" << endl;
			return "DR";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i - 1][j] != 255)
		{
			direction = 'U';
			CheckCornerPoints(pid, i, j, 3, 'R');
			/*insertCornerPoints(pid, i - 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tU_15" << endl;
			return "U";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i - 1][j] != 255)
		{
			direction = 'U';
			CheckCornerPoints(pid, i, j, 3, 'R');
			/*insertCornerPoints(pid, i - 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tU_16" << endl;
			return "U";//4
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i + 1][j] != 255)
		{
			direction = 'D';
			CheckCornerPoints(pid, i, j, 3, 'R');
			/*insertCornerPoints(pid, i + 1, j);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tD_17" << endl;
			return "D";//4
		}
		else if (i - 1 < edgeMapVector.size() && i + 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i - 1 >= 0 && i + 1 >= 0 && j + 1 >= 0 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i][j + 1] == 255)
		{
			direction = 'L';
			CheckCornerPoints(pid, i, j, 3, 'R');
			/*insertCornerPoints(pid, i, j - 1);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tL_18" << endl;
			return "L";//3

		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i + 1][j + 1] == 255 && edgeMapVector[i - 1][j + 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tUR_19" << endl;
			return "UR";//3
		}
		else if (i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && j + 1 < edgeMapVector[0].size() && i + 1 >= 0 && i - 1 >= 0 && j + 1 >= 0 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i - 1][j + 1] == 255 && edgeMapVector[i + 1][j + 1] != 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tDR_191" << endl;
			return "DR";//3
		}
		else if (j + 1 < edgeMapVector[0].size() && i - 1 < edgeMapVector.size() && i + 1 < edgeMapVector.size() && edgeMapVector[i - 1][j + 1] != 255 && edgeMapVector[i + 1][j] == 255 && edgeMapVector[i][j + 1] == 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tUR_192" << endl;
			return "UR";//3
		}
		else if (j + 1 < edgeMapVector[0].size() && i + 1 < edgeMapVector.size() && i - 1 < edgeMapVector.size() && edgeMapVector[i + 1][j + 1] != 255 && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i][j + 1] == 255)
		{
			//cout << to_string(i) << "\t" << to_string(j) << "\tDR_193" << endl;
			return "DR";//3
		}
		//else if (j + 1 < edgeMapVector[0].size() && i - 1 < edgeMapVector.size() && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i+1][j-1] != 255)
		//{
		//	//cout << to_string(i) << "\t" << to_string(j) << "D_13" << endl;
		//	return "DL";//0
		//}		
		//else if (j < edgeMapVector[0].size() && i + 1 < edgeMapVector.size() && edgeMapVector[i - 1][j] == 255 && edgeMapVector[i][j + 1] == 255 && edgeMapVector[i - 1][j-1] != 255)
		//{
		//	//cout << to_string(i) << "\t" << to_string(j) << "U_14" << endl;
		//	return "UL";//0
		//}
		else if (j - 1 < edgeMapVector[0].size() && i + 1 < edgeMapVector.size() && edgeMapVector[i + 1][j - 1] != 255)
		{
			direction = 'L';
			//cout << to_string(i) << "\t" << to_string(j) << "\tDL_194" << endl;
			return "DL";//1
		}
		else if (j - 1 < edgeMapVector[0].size() && i - 1 < edgeMapVector.size() && edgeMapVector[i - 1][j - 1] != 255)
		{
			direction = 'L';
			//cout << to_string(i) << "\t" << to_string(j) << "\tUL_195" << endl;
			return "UL";//1
		}
		else if (j - 1 < edgeMapVector[0].size() && i < edgeMapVector.size() && edgeMapVector[i][j - 1] != 255)
		{
			direction = 'L';
			/*insertCornerPoints(pid, i, j - 1);*/
			//cout << to_string(i) << "\t" << to_string(j) << "\tL_196" << endl;
			return "L";//1
		}
		else
			return "0";
		break;
	default:
		return "0";
	}
}

void CannyThreshold(int, void*)
{
	/// Reduce noise with a kernel 3x3
	Mat blurImage;
	blur(src_gray, blurImage, Size(3, 3));

	Mat mask;
	
	/// Canny detector
	Canny(blurImage, detected_edges, lowThreshold, lowThreshold*ratios, kernel_size);
	lowThreshold = 70; // lower end
	Canny(blurImage, mask, lowThreshold, lowThreshold*ratios, kernel_size, true);
	for (int i = 0; i < mask.rows; i++)
	{
		for (int j = 0; j < mask.cols; j++)
		{
			if (mask.at<uchar>(i, j) == 255)
				mask.at<uchar>(i, j) = 0;
			else
				mask.at<uchar>(i, j) = 255;
		}
	}
	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);
	
	detected_edges.copyTo(dst, mask);
	//detected_edges.copyTo(dst, mask);
	
	
	//imshow("src", src);
	//imshow("detected edges", detected_edges );
	//imshow("dst", dst );
	//waitKey(0);
	//imshow("Gray scale image",src_gray);
	//cout<<"number of channels"<<dst.channels()<<endl;
}

void colorEnhancement()
{
	new_image = Mat::zeros(image.size(), image.type());

	/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
	for (int y = 0; y < image.rows; y++)
	{
		for (int x = 0; x < image.cols; x++)
		{
			for (int c = 0; c < 3; c++)
			{
				new_image.at<Vec3b>(y, x)[c] =
					saturate_cast<uchar>(alpha*(image.at<Vec3b>(y, x)[c]) + beta);
			}
		}
	}

	/// Create Windows
	/*namedWindow("Original RGB Image", 1);
	namedWindow(window_name2, 1);*/

	/// Show stuff
	/*imshow("Original RGB Image", image);
	imshow(window_name2, new_image);*/
}

void MyFilledCircle(Mat &img, Point center)
{
	int thickness = -1;
	int lineType = 8;

	circle(img,
		center,
		5,
		Scalar(0, 255, 255),
		thickness,
		lineType);
}

void MyFilledCircle2(Mat &img, Point center)
{
	int thickness = -1;
	int lineType = 8;

	circle(img,
		center,
		5,
		Scalar(255,255,0),
		thickness,
		lineType);
}

void particalPathMarker(Mat &img, Point center)
{
	int thickness = 1;
	int lineType = 8;

	circle(img,
		center,
		1,
		Scalar(0, 255, 0),
		thickness,
		lineType);
}

void writeMatToFile(cv::Mat& m, const char* filename)
{
	try
	{
		cout << "File Write Start" << endl;
		//   //ofstream fout(filename);
		   //edgeMapFile.crea
		edgeMapFile.open("edgeMap.txt");
		////edgeMapFile<<XR<<"_"<<YR<<"_"<<ZR<<endl;
		//cout<<m.channels()<<endl;

		if (!edgeMapFile)
		{
			cout << "File Not Opened" << endl;  return;
		}

		for (int i = 0; i < detected_edges.rows; i++)
		{
			for (int j = 0; j < detected_edges.cols; j++)
			{
				//fout<<m.at<float>(i,j)<<"\t";
				edgeMapFile << (float)detected_edges.at<uchar>(i, j) << "\t";
				tempVector.push_back((int)detected_edges.at<uchar>(i, j));
				//cout<<"hello"<<"\t";
			}
			edgeMapFile << endl;
			edgeMapVector.push_back(tempVector);
			tempVector.clear();
		}
		edgeMapFile.close();
		cout << "File Write Done" << endl;
		cout << "rows " << edgeMapVector.size() << endl;
		cout << "col " << edgeMapVector[0].size() << endl;
		/*cout << "0,0 " << edgeMapVector[0][0] << endl;
		cout << "0,1 " << edgeMapVector[0][1] << endl;
		cout << "0,2 " << edgeMapVector[0][2] << endl;

		cout << "1,0 " << edgeMapVector[1][0] << endl;
		cout << "1,1 " << edgeMapVector[1][1] << endl;
		cout << "1,2 " << edgeMapVector[1][2] << endl;

		cout << "2,0 " << edgeMapVector[2][0] << endl;
		cout << "2,1 " << edgeMapVector[2][1] << endl;
		cout << "2,2 " << edgeMapVector[2][2] << endl;*/

		//cout << "0,3 " << edgeMapVector[0][3] << endl;
		//cout << "0,4 " << edgeMapVector[0][4] << endl;
		//cout << "0,5 " << edgeMapVector[0][5] << endl;
		//cout << "0,6 " << edgeMapVector[0][6] << endl;
		//cout << "0,7 " << edgeMapVector[0][7] << endl;
		//cout << "0,8 " << edgeMapVector[0][8] << endl;

		//fout.close();
	}
	catch (Exception e)
	{
		cout << e.msg << endl;
	}
}

void FileToMatObject(cv::Mat& m, cv::Mat& t, const char* filename)
{
	ifstream infile("edgeMap.txt");
	string s;
	int countLines = 0;
	vector<string> tokens;
	/*while (infile >>s)
	{
		cout<<s<<endl;
	}*/
	for (std::string line; getline(infile, line); )
	{
		//cout<<line<<endl;
		istringstream iss(line);
		copy(istream_iterator<string>(iss),
			istream_iterator<string>(),
			back_inserter(tokens));
		countLines++;
		//cout<<"Count of the lines "<<countLines<<endl;
	}
	//cout<<"Count of the lines "<<countLines<<endl;
	imageRows = countLines;
	cout << "Vector size " << tokens.size() << endl;
	imageColumns = tokens.size() / imageRows;
	int vectorCounter = 0;

	//Mat ut(imageRows,imageColumns,m.channels);
	//CV_Assert(t.depth() == CV_8U);
	Mat a(imageRows, imageColumns, CV_8UC1);

	for (int i = 0; i < imageRows; i++)
	{
		for (int j = 0; j < imageColumns; j++)
		{
			if (tokens.size() > vectorCounter)
			{
				a.at<uchar>(i, j) = stoi(tokens.at(vectorCounter));
			}
			vectorCounter++;
		}
	}
	cout << "done" << endl;
	//imshow("Intersections",a);
}

void FileToMatObject2(cv::Mat& m, cv::Mat& t, const char* filename)
{
	//ifstream infile("edgeMapRemove.txt");//eye remove unwanted parts 
	ifstream infile("edgeMapRemoveRoad2.txt"); //road unwanted parts removed
	string s;
	int countLines = 0;
	vector<string> tokens;
	/*while (infile >>s)
	{
		cout<<s<<endl;
	}*/
	for (std::string line; getline(infile, line); )
	{
		//cout<<line<<endl;
		istringstream iss(line);
		copy(istream_iterator<string>(iss),
			istream_iterator<string>(),
			back_inserter(tokens));
		countLines++;
		//cout<<"Count of the lines 2 "<<countLines<<endl;
	}
	//cout<<"Count of the lines "<<countLines<<endl;
	imageRows = countLines;
	cout << "Vector size " << tokens.size() << endl;
	imageColumns = tokens.size() / imageRows;
	int vectorCounter = 0;

	//Mat ut(imageRows,imageColumns,m.channels);
	//CV_Assert(t.depth() == CV_8U);
	Mat b(imageRows, imageColumns, CV_8UC1);
	cout << "c count" << b.channels() << endl;
	for (int i = 0; i < imageRows; i++)
	{
		for (int j = 0; j < imageColumns; j++)
		{
			if (tokens.size() > vectorCounter)
			{
				b.at<uchar>(i, j) = stoi(tokens.at(vectorCounter));
			}
			vectorCounter++;
		}
	}
	cout << "done" << endl;
	imshow("After Remove", b);
}

void clearUnwantedEdges(Mat m)
{
	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			if ((int)m.at<uchar>(i, j) < 500)
			{
				m.at<uchar>(i, j) = 0;
			}
			//fout<<m.at<float>(i,j)<<"\t";
			//edgeMapFile<<(float)m.at<uchar>(i,j)<<"\t";
			//cout<<"hello"<<"\t";
		}
	}

	/*imshow("afterRemove",m);*/
}

void vectorPrint()
{
	cout << "col" << detected_edges.cols << endl;
	cout << "rows" << detected_edges.rows << endl;
	cout << "capacity" << edgeMapVector.size() << endl;
	cout << "capacity" << edgeMapVector[0].size() << endl;
}

static int callback(void *NotUsed, int argc, char **argv, char **azColName)
{
	int i;
	for (i = 0; i < argc; i++)
	{
		cout << azColName[i] << "\t" << argv[i] << "\t";
		//cout << azColName[i] << "\t" << argv[i] << "\t";

	}
	cout << endl;
	return 0;
}

// 1 true 0 false -1 invalid
static int callbackCheckSimilarMovements(void *NotUsed, int argc, char **argv, char **azColName)
{
	/*int i;
	for (i = 0; i < argc; i++)
	{
		cout << azColName[i] << "\t" << argv[i] << "\t";
	}
	cout << endl;*/

	if (argc == 1)
	{
		if (atoi(argv[0]) >= 5)
		{
			similarMovementRetValue = 1;
		}
		else
		{
			similarMovementRetValue = 0;
		}
	}
	else
	{
		similarMovementRetValue = -1;
	}

	return 0;
}

static int callBackShowIntersections(void *NotUsed, int argc, char **argv, char **azColName)
{
	//MyFilledCircle(atom_image, Point(692, 6));
	//MyFilledCircle(atom_image, Point(605, 17));
	//MyFilledCircle(atom_image, Point(123, 30));
	//MyFilledCircle(atom_image, Point(123, 41));
	//MyFilledCircle(atom_image, Point(596, 120));
	//MyFilledCircle(atom_image, Point(694, 120));
	//MyFilledCircle(atom_image, Point(73, 427));


	MyFilledCircle(intersection_image, Point(atoi(argv[1]), atoi(argv[0])));

	return 0;
}

static int callBackShowCornerPoints(void *NotUsed, int argc, char **argv, char **azColName)
{
	//MyFilledCircle(atom_image, Point(692, 6));
	//MyFilledCircle(atom_image, Point(605, 17));
	//MyFilledCircle(atom_image, Point(123, 30));
	//MyFilledCircle(atom_image, Point(123, 41));
	//MyFilledCircle(atom_image, Point(596, 120));
	//MyFilledCircle(atom_image, Point(694, 120));
	//MyFilledCircle(atom_image, Point(73, 427));


	MyFilledCircle2(intersection_image, Point(atoi(argv[1]), atoi(argv[0])));

	return 0;
}

static int callBackShowParticalPaths(void *NotUsed, int argc, char **argv, char **azColName)
{
	particalPathMarker(intersection_image, Point(atoi(argv[1]), atoi(argv[0])));
	return 0;
}

static int callBackSaveParticalCoordinates(void *NotUsed, int argc, char **argv, char **azColName)
{
	pathCoordinatesX.push_back(atoi(argv[0]));
	pathCoordinatesY.push_back(atoi(argv[1]));
	return 0;
}

void emptyTable()
{
	string sqlQuery = "delete from data_log_tbl;";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	sqlQuery = "UPDATE SQLITE_SEQUENCE SET SEQ = 0 WHERE NAME = 'data_log_tbl';";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	sqlQuery = "DELETE FROM partical_direction_tbl;";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	sqlQuery = "UPDATE SQLITE_SEQUENCE SET SEQ = 0 WHERE NAME = 'partical_direction_tbl';";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	sqlQuery = "DELETE FROM corner_points_tbl;";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	sqlQuery = "UPDATE SQLITE_SEQUENCE SET SEQ = 0 WHERE NAME = 'corner_points_tbl';";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		fprintf(stdout, "Table Reset\n");
	}
}

//void insertData(int particalID,int source_x,int source_y,int dest_x,int dest_y)
void insertData(int particalID, int source_x, int source_y, int dest_x, int dest_y, string dir)
{
	//string name = "ajith";
	string sqlQuery = "insert into data_log_tbl (partical_id,source_x,source_y,dest_x,dest_y) values ('" + to_string(particalID) + "','" + to_string(source_x) + "','" + to_string(source_y) + "','" + to_string(dest_x) + "','" + to_string(dest_y) + "');";
	//string sqlQuery = "insert into data_log2_tbl (partical_id,source_x,source_y,dest_x,dest_y,direction) values ('"+to_string(particalID)+"','"+ to_string(source_x)+"','"+ to_string(source_y)+"','"+ to_string(dest_x)+"','"+ to_string(dest_y)+"','"+dir+"');";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		//fprintf(stdout, "inserted\n");
	}
}

void insertParticalDirection(int particalID, string dir)
{
	string sqlQuery = "insert into partical_direction_tbl (partical_id,direction) values ('" + to_string(particalID) + "','" + dir + "');";
	//string sqlQuery = "insert into data_log2_tbl (partical_id,source_x,source_y,dest_x,dest_y,direction) values ('"+to_string(particalID)+"','"+ to_string(source_x)+"','"+ to_string(source_y)+"','"+ to_string(dest_x)+"','"+ to_string(dest_y)+"','"+dir+"');";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		//fprintf(stdout, "inserted\n");
	}
}

void insertCornerPoints(int particalID, int x, int y)
{
	string sqlQuery = "insert into corner_points_tbl (partical_id,x,y) values ('" + to_string(particalID) + "','" + to_string(x) + "','" + to_string(y) + "');";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callback, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		//fprintf(stdout, "inserted\n");
	}
}

void selectAll()
{
	sql = "select * from data_log_tbl;";

	rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		fprintf(stdout, "select\n");
	}
}

void checkSimilarMovements(int dest_x, int dest_y)
{
	string sqlQuery = "select count(id) as rowCount from data_log_tbl where dest_x =" + to_string(dest_x) + " and dest_y=" + to_string(dest_y) + ";";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callbackCheckSimilarMovements, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		//fprintf(stdout, "select\n");
	}
}

void test(string name)
{
	//insertData(name);
	selectAll();
}

void movePartical(int pID, int x,int y, char particalDirection)
//void movePartical(int pID, int startingPoint, char particalDirection)
{
	try
	{
		/*particalPathfile.open("path.txt");
		if (!edgeMapFile)
		{
		cout << "File Not Opened" << endl;  return;
		}*/


		/*int el;*/
		//FILE *stream;
		/*particalPathfile << "Partical ID,X,Y,Direction"<< "\n";*/
		//stream = fopen("edgeMap.txt", "r");

		int i = x;
		int j = y;
		char* retValue = "0";
		int source_x = 0;
		int source_y = 0;
		int dest_x = 0;
		int dest_y = 0;
		int particalID = pID;
		int counter = 0;
		bool executionStop = false;
		int tempVal = 0;
		direction = particalDirection;

		if (direction == 'R')
		{
			i = x;
			j = y;
			retValue = isValidMoveAvailable(pID, i, j, direction);
		}
		else if (direction == 'L')
		{
			/*i = startingPoint;
			j = edgeMapVector[0].size() - 1;*/
			i = x;
			j = y;
			retValue = isValidMoveAvailable(pID, i, j, direction);
		}
		else if (direction == 'U')
		{
			/*i = edgeMapVector.size() - 1;
			j = startingPoint;*/
			i = x;
			j = y;
			retValue = isValidMoveAvailable(pID, i, j, direction);
		}
		else if (direction == 'D')
		{
			/*i = 0;
			j = startingPoint;*/
			i = x;
			j = y;
			retValue = isValidMoveAvailable(pID, i, j, direction);
		}


		while (retValue != "0" && executionStop != true)
		{
			counter++;
			if (retValue == "U")
			{
				source_x = i;
				source_y = j;
				i = i - 1;
				dest_x = i;
				dest_y = j;
				//insertData(particalID, source_x, source_y, dest_x, dest_y);
				insertData(particalID, source_x, source_y, dest_x, dest_y, retValue);

				if (direction == 'R')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'L')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'U')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'D')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
			}
			else if (retValue == "D")
			{
				source_x = i;
				source_y = j;
				i = i + 1;
				dest_x = i;
				dest_y = j;
				//insertData(particalID, source_x, source_y, dest_x, dest_y);
				insertData(particalID, source_x, source_y, dest_x, dest_y, retValue);

				if (direction == 'R')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'L')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'U')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'D')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
			}
			else if (retValue == "L")
			{
				source_x = i;
				source_y = j;
				j = j - 1;
				dest_x = i;
				dest_y = j;
				//insertData(particalID, source_x, source_y, dest_x, dest_y);
				insertData(particalID, source_x, source_y, dest_x, dest_y, retValue);

				if (direction == 'R')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'L')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'U')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'D')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
			}
			else if (retValue == "R")
			{
				source_x = i;
				source_y = j;
				j = j + 1;
				dest_x = i;
				dest_y = j;
				//insertData(particalID, source_x, source_y, dest_x, dest_y);
				insertData(particalID, source_x, source_y, dest_x, dest_y, retValue);

				if (direction == 'R')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'L')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'U')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'D')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
			}
			else if (retValue == "UL")
			{
				source_x = i;
				source_y = j;
				i = i - 1;
				j = j - 1;
				dest_x = i;
				dest_y = j;
				//insertData(particalID, source_x, source_y, dest_x, dest_y);
				insertData(particalID, source_x, source_y, dest_x, dest_y, retValue);

				if (direction == 'R')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'L')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'U')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'D')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
			}
			else if (retValue == "UR")
			{
				source_x = i;
				source_y = j;
				i = i - 1;
				j = j + 1;
				dest_x = i;
				dest_y = j;
				//insertData(particalID, source_x, source_y, dest_x, dest_y);
				insertData(particalID, source_x, source_y, dest_x, dest_y, retValue);

				if (direction == 'R')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'L')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'U')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'D')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
			}
			else if (retValue == "DL")
			{
				source_x = i;
				source_y = j;
				i = i + 1;
				j = j - 1;
				dest_x = i;
				dest_y = j;
				//insertData(particalID, source_x, source_y, dest_x, dest_y);
				insertData(particalID, source_x, source_y, dest_x, dest_y, retValue);

				if (direction == 'R')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'L')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'U')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'D')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
			}
			else if (retValue == "DR")
			{
				source_x = i;
				source_y = j;
				i = i + 1;
				j = j + 1;
				dest_x = i;
				dest_y = j;
				//insertData(particalID, source_x, source_y, dest_x, dest_y);
				insertData(particalID, source_x, source_y, dest_x, dest_y, retValue);

				if (direction == 'R')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'L')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'U')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
				else if (direction == 'D')
				{
					retValue = isValidMoveAvailable(pID, i, j, direction);
				}
			}

			checkSimilarMovements(dest_x, dest_y);

			if (similarMovementRetValue == 1 || similarMovementRetValue == -1)
			{
				executionStop = true;
			}
		}

		cout << "Partical Movement Done" << endl;
		/*cout << "done path file" << endl;
		particalPathfile << endl;
		particalPathfile.close();*/
	}
	catch (Exception e)
	{
		cout << e.msg << endl;
	}

}

void showIntersections()
{
	//string sqlQuery = "select dest_x, dest_y from data_log_tbl x, partical_direction_tbl pd where x.partical_id = pd.partical_id group by x.dest_x, x.dest_y having count(distinct x.partical_id) = 3 and count(distinct pd.direction) > 1;";
	string sqlQuery = "select dest_x, dest_y from data_log_tbl group by dest_x, dest_y having count(distinct partical_id) >=3 ;";
	//string sqlQuery = "Select x2.dest_x,x2.dest_y from data_log_tbl x1, data_log_tbl x2 where Abs(x1.dest_x - x2.dest_x) < 3 and Abs(x1.dest_y - x2.dest_y) < 3 and x1.partical_id != x2.partical_id Group by x1.partical_id Having count(*) > 1;";
	//string sqlQuery = "Select x1.dest_x,x1.dest_y from data_log_tbl x1, data_log_tbl x2 where Abs(x1.dest_x - x2.dest_x) < 3 and Abs(x1.dest_y - x2.dest_y) < 3 and x1.partical_id != x2.partical_id Group by x1.partical_id Having count(*) > 1;";
	//string sqlQuery = "Select  distinct x1.dest_x, x1.dest_y from data_log_tbl x1, data_log_tbl x2 where Abs(x1.dest_x - x2.dest_x) < 3 and Abs(x1.dest_y - x2.dest_y) < 3 and x1.partical_id != x2.partical_id;";
	rc = sqlite3_exec(db, sqlQuery.c_str(), callBackShowIntersections, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		fprintf(stdout, "select\n");
	}
}

void showCornerPoints()
{
	string sqlQuery = "select distinct x, y from corner_points_tbl;";
	//string sqlQuery = "Select x2.dest_x,x2.dest_y from data_log_tbl x1, data_log_tbl x2 where Abs(x1.dest_x - x2.dest_x) < 3 and Abs(x1.dest_y - x2.dest_y) < 3 and x1.partical_id != x2.partical_id Group by x1.partical_id Having count(*) > 1;";
	//string sqlQuery = "Select x1.dest_x,x1.dest_y from data_log_tbl x1, data_log_tbl x2 where Abs(x1.dest_x - x2.dest_x) < 3 and Abs(x1.dest_y - x2.dest_y) < 3 and x1.partical_id != x2.partical_id Group by x1.partical_id Having count(*) > 1;";
	//string sqlQuery = "Select  distinct x1.dest_x, x1.dest_y from data_log_tbl x1, data_log_tbl x2 where Abs(x1.dest_x - x2.dest_x) < 3 and Abs(x1.dest_y - x2.dest_y) < 3 and x1.partical_id != x2.partical_id;";
	rc = sqlite3_exec(db, sqlQuery.c_str(), callBackShowCornerPoints, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		fprintf(stdout, "select\n");
	}
}

void showParticalPaths(int id)
{
	string sqlQuery = "select dest_x, dest_y from data_log_tbl where partical_id = " + to_string(id) + ";";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callBackShowParticalPaths, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		fprintf(stdout, "select\n");
	}
}

void getParticalPathCoordinates(int id)
{
	string sqlQuery = "select dest_x,dest_y from data_log_tbl where partical_id = " + to_string(id) + ";";

	rc = sqlite3_exec(db, sqlQuery.c_str(), callBackSaveParticalCoordinates, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL Error %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
	}
	else
	{
		fprintf(stdout, "select\n");
	}
}

void writeParticalPathToFile(int id)
{
	try
	{
		ifstream infile("edgeMap.txt");
		string s;
		int countLines = 0;
		vector<string> tokens;

		for (std::string line; getline(infile, line); )
		{
			//cout<<line<<endl;
			istringstream iss(line);
			copy(istream_iterator<string>(iss),
				istream_iterator<string>(),
				back_inserter(tokens));
			countLines++;
		}
		imageRows = countLines;
		imageColumns = tokens.size() / imageRows;
		int vectorCounter = 0;

		Mat a(imageRows, imageColumns, CV_8UC1);

		for (int i = 0; i < imageRows; i++)
		{
			for (int j = 0; j < imageColumns; j++)
			{
				if (tokens.size() > vectorCounter)
				{
					a.at<uchar>(i, j) = stoi(tokens.at(vectorCounter));
				}
				vectorCounter++;
			}
		}

		while (!pathCoordinatesX.empty() && !pathCoordinatesY.empty())
		{
			int x = pathCoordinatesX.back();
			int y = pathCoordinatesY.back();

			a.at<uchar>(x, y) = id;
			pathCoordinatesX.pop_back();
			pathCoordinatesY.pop_back();
		}

		cout << "File Write Start" << endl;
		edgeMapFile.open("edgeMap_" + to_string(id) + ".txt");

		if (!edgeMapFile)
		{
			cout << "File Not Opened" << endl;  return;
		}

		for (int i = 0; i < a.rows; i++)
		{
			for (int j = 0; j < a.cols; j++)
			{
				edgeMapFile << (float)a.at<uchar>(i, j) << "\t";
			}
			edgeMapFile << endl;
		}
		edgeMapFile.close();
		cout << "File Write Done" << endl;
	}
	catch (Exception e)
	{
		cout << e.msg << endl;
	}
}

void findSeedPoints()
{
	int i = 0;
	int j = 0;
	vector<int> temp;

#pragma region Left to right

	//left seed points only rows j=0
	i = 0;
	j = 0;

	for (i; i < edgeMapVector.size(); i++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(i);
			cout << "L" << i << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector.size() && v1 - 1 > 0)
		{
			t.push_back(v1 - 1);
			t.push_back(0);
			leftSeedPoints.push_back(t);
		}
		else
		{
			t.push_back(v1);
			t.push_back(0);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector.size() && v1 + 1 > 0)
		{
			t.push_back(v1 + 1);
			t.push_back(0);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(0);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1);
		}
	}
	temp.clear();

	//L1

	i = 0;
	j = (int)edgeMapVector[0].size()/3;

	for (i; i < edgeMapVector.size(); i++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(i);
			cout << "L_1" << i << endl;
		}
	}

	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector.size() && v1 - 1 > 0)
		{
			t.push_back(v1 - 1);
			t.push_back(j);
			leftSeedPoints.push_back(t);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector.size() && v1 + 1 > 0)
		{
			t.push_back(v1 + 1);
			t.push_back(j);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1);
		}
	}
	temp.clear();

	//L2
	i = 0;
	j = (int)edgeMapVector[0].size() / 1.5;

	for (i; i < edgeMapVector.size(); i++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(i);
			cout << "L_2" << i << endl;
		}
	}

	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector.size() && v1 - 1 > 0)
		{
			t.push_back(v1 - 1);
			t.push_back(j);
			leftSeedPoints.push_back(t);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector.size() && v1 + 1 > 0)
		{
			t.push_back(v1 + 1);
			t.push_back(j);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			leftSeedPoints.push_back(t);
			//leftSeedPoints.push_back(v1);
		}
	}
	temp.clear();

#pragma endregion

#pragma region Right to Left
	//right seed points only rows j=last pixel 
	i = 0;
	j = edgeMapVector[0].size() - 1;

	for (i; i < edgeMapVector.size(); i++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(i);
			cout << "R" << i << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector.size() && v1 - 1 > 0)
		{
			t.push_back(v1 - 1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1 - 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector.size() && v1 + 1 > 0)
		{
			t.push_back(v1 + 1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1);
		}
	}
	temp.clear();

	//R1
	i = 0;
	j = (int)(edgeMapVector[0].size() - 1)/4;

	for (i; i < edgeMapVector.size(); i++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(i);
			cout << "R_1" << i << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector.size() && v1 - 1 > 0)
		{
			t.push_back(v1 - 1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1 - 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector.size() && v1 + 1 > 0)
		{
			t.push_back(v1 + 1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1);
		}
	}
	temp.clear();

	//R2

	i = 0;
	j = (int)(edgeMapVector[0].size() - 1) / 2;

	for (i; i < edgeMapVector.size(); i++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(i);
			cout << "R_2" << i << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector.size() && v1 - 1 > 0)
		{
			t.push_back(v1 - 1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1 - 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector.size() && v1 + 1 > 0)
		{
			t.push_back(v1 + 1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(v1);
			t.push_back(j);
			rightSeedPoints.push_back(t);
			//rightSeedPoints.push_back(v1);
		}
	}
	temp.clear();

#pragma endregion

#pragma region Up to down
	//up seed point only cols i=0
	i = 0;
	j = 0;

	for (j; j < edgeMapVector[0].size(); j++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(j);
			cout << "U" << j << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector[0].size() && v1 - 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 - 1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1 - 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector[0].size() && v1 + 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 + 1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1);
		}
	}
	temp.clear();

	//U1
	i = (int) edgeMapVector.size()/3;
	j = 0;

	for (j; j < edgeMapVector[0].size(); j++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(j);
			cout << "U_1" << j << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector[0].size() && v1 - 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 - 1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1 - 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector[0].size() && v1 + 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 + 1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1);
		}
	}
	temp.clear();

	//U2
	
	i = (int)edgeMapVector.size() / 1.5;
	j = 0;

	for (j; j < edgeMapVector[0].size(); j++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(j);
			cout << "U_2" << j << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector[0].size() && v1 - 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 - 1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1 - 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector[0].size() && v1 + 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 + 1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1 + 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			upSeedPoints.push_back(t);
			//upSeedPoints.push_back(v1);
		}
	}
	temp.clear();

#pragma endregion

#pragma region Down to Up

	//down seed point only cols i= last pixel
	i = edgeMapVector.size() - 1;
	j = 0;

	for (j; j < edgeMapVector[0].size(); j++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(j);
			cout << "D" << j << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector[0].size() && v1 - 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 - 1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1 - 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector[0].size() && v1 + 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 + 1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1 + 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1);
		}
	}
	temp.clear();

	//D1
	i = (int) (edgeMapVector.size() - 1)/4;
	j = 0;

	for (j; j < edgeMapVector[0].size(); j++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(j);
			cout << "D_1" << j << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector[0].size() && v1 - 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 - 1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1 - 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector[0].size() && v1 + 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 + 1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1 + 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1);
		}
	}
	temp.clear();

	//D2
	i = (int)(edgeMapVector.size() - 1) / 2;
	j = 0;

	for (j; j < edgeMapVector[0].size(); j++)
	{
		if (edgeMapVector[i][j] == 255)
		{
			temp.push_back(j);
			cout << "D_2" << j << endl;
		}
	}


	while (!temp.empty())
	{
		int v1 = temp.back();
		temp.pop_back();
		vector<int> t;

		if (v1 - 1 < edgeMapVector[0].size() && v1 - 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 - 1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1 - 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1);
		}

		if (v1 + 1 < edgeMapVector[0].size() && v1 + 1 > 0)
		{
			t.push_back(i);
			t.push_back(v1 + 1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1 + 1);
		}
		else
		{
			t.push_back(i);
			t.push_back(v1);
			downSeedpoint.push_back(t);
			//downSeedpoint.push_back(v1);
		}
	}
	temp.clear();

#pragma endregion

}

#pragma endregion


int main(int argc, char** argv)
{
	clock_t tStart = clock();
	/*char* direction = isValidMoveAvailable(1, 2, "ER");
	cout << direction << endl;*/
	colorEnhancement();
	/// Load an image
	src = new_image;
	

	if (!src.data)
	{
		return -1;
	}

	/// Create a matrix of the same type and size as src (for dst)
	dst.create(src.size(), src.type());

	/// Convert the image to grayscale
	cvtColor(src, src_gray, CV_BGR2GRAY);

	/// Create a window
	namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	/// Create a Trackbar for user to enter threshold
	//createTrackbar( "Alpha: ", window_name2, &alpha_slider, alpha_slider_max, colorEnhancement );
	//createTrackbar( "Beta: ", window_name2, &beta_slider, beta_slider_max, colorEnhancement );
	createTrackbar("Min Threshold: ", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);

	/// Show the image

	CannyThreshold(0, 0);

	writeMatToFile(dst, "edgeMap.txt");
	findSeedPoints();
	intersection_image =  dst; //show partical paths
	//intersection_image = detected_edges;//to show corners
	//FileToMatObject(dst,read_image,"edgeMap.txt");
	//FileToMatObject2(dst,read_image,"edgeMap.txt");
	//imshow("before Remove",dst);

	//vectorPrint();

	//open DB
	rc = sqlite3_open("partical_method_db.db", &db);
	if (rc)
	{
		fprintf(stderr, "Cant open DB %s\n", sqlite3_errmsg(db));
		return 0;
	}
	else
	{
		fprintf(stdout, "DB opened\n");
	}

	//insert some SQL
	//insertData("ajith");
	//selectAll();

	emptyTable();

#pragma region satelite image test
	////L and R 0-587	U and D 0-806

	/*thread L_R_1(movePartical, 1, 60, 'R');
   thread L_R_2(movePartical, 10, 120, 'R');
	thread L_R_3(movePartical, 11, 180, 'R');
	thread L_R_4(movePartical, 12, 240, 'R');
	thread L_R_5(movePartical, 13, 300, 'R');
	thread L_R_6(movePartical, 14, 360, 'R');
	thread L_R_7(movePartical, 15, 420, 'R');
	thread L_R_8(movePartical, 16, 480, 'R');
	thread L_R_9(movePartical, 17, 540, 'R');
	thread L_R_10(movePartical, 18, 5, 'R');


	thread R_L_1(movePartical, 2, 60, 'L');
	thread R_L_2(movePartical, 20, 120, 'L');
	thread R_L_3(movePartical, 21, 180, 'L');
	thread R_L_4(movePartical, 22, 240, 'L');
	thread R_L_5(movePartical, 23, 300, 'L');
	thread R_L_6(movePartical, 24, 360, 'L');
	thread R_L_7(movePartical, 25, 420, 'L');
	thread R_L_8(movePartical, 26, 480, 'L');
	thread R_L_9(movePartical, 27, 540, 'L');
	thread R_L_10(movePartical,28, 6, 'L');


	thread D_U_1(movePartical, 3, 80, 'U');
	thread D_U_2(movePartical, 30, 160, 'U');
	thread D_U_3(movePartical, 31, 240, 'U');
	thread D_U_4(movePartical, 32, 320, 'U');
	thread D_U_5(movePartical, 33, 400, 'U');
	thread D_U_6(movePartical, 34, 480, 'U');
	thread D_U_7(movePartical, 35, 560, 'U');
	thread D_U_8(movePartical, 36, 640, 'U');
	thread D_U_9(movePartical, 37, 720, 'U');
	thread D_U_10(movePartical,38, 800, 'U');

	thread U_D_1(movePartical, 4, 80, 'D');
	thread U_D_2(movePartical, 40, 160, 'D');
	thread U_D_3(movePartical, 41, 240, 'D');
	thread U_D_4(movePartical, 42, 320, 'D');
	thread U_D_5(movePartical, 43, 400, 'D');
	thread U_D_6(movePartical, 44, 480, 'D');
	thread U_D_7(movePartical, 45, 560, 'D');
	thread U_D_8(movePartical, 46, 640, 'D');
	thread U_D_9(movePartical, 47, 720, 'D');
	thread U_D_10(movePartical,48, 800, 'D');*/


	//L_R_1.join();
	//L_R_2.join();
	//L_R_3.join();
	//L_R_4.join();
	//L_R_5.join();
	//L_R_6.join();
	//L_R_7.join();
	//L_R_8.join();
	//L_R_9.join();
	//L_R_10.join();


	//R_L_1.join();
	//R_L_2.join();
	//R_L_3.join();
	//R_L_4.join();
	//R_L_5.join();
	//R_L_6.join();
	//R_L_7.join();
	//R_L_8.join();
	//R_L_9.join();
	//R_L_10.join();


	//D_U_1.join();
	//D_U_2.join();
	//D_U_3.join();
	//D_U_4.join();
	//D_U_5.join();
	//D_U_6.join();
	//D_U_7.join();
	//D_U_8.join();
	//D_U_9.join();
	//D_U_10.join();


	//U_D_1.join();
	//U_D_2.join();
	//U_D_3.join();
	//U_D_4.join();
	//U_D_5.join();
	//U_D_6.join();
	//U_D_7.join();
	//U_D_8.join();
	//U_D_9.join();
	//U_D_10.join();

#pragma endregion 

#pragma region retinal Image test

  ////L and R 0-605	U and D 0-700

  /*thread L_R_1(movePartical, 1, 60, 'R');
  thread L_R_2(movePartical, 10, 120, 'R');
  thread L_R_3(movePartical, 11, 180, 'R');
  thread L_R_4(movePartical, 12, 240, 'R');
  thread L_R_5(movePartical, 13, 300, 'R');
  thread L_R_6(movePartical, 14, 360, 'R');
  thread L_R_7(movePartical, 15, 420, 'R');
  thread L_R_8(movePartical, 16, 480, 'R');
  thread L_R_9(movePartical, 17, 540, 'R');
  thread L_R_10(movePartical, 18, 5, 'R');


  thread R_L_1(movePartical, 2, 60, 'L');
  thread R_L_2(movePartical, 20, 120, 'L');
  thread R_L_3(movePartical, 21, 180, 'L');
  thread R_L_4(movePartical, 22, 240, 'L');
  thread R_L_5(movePartical, 23, 300, 'L');
  thread R_L_6(movePartical, 24, 360, 'L');
  thread R_L_7(movePartical, 25, 420, 'L');
  thread R_L_8(movePartical, 26, 480, 'L');
  thread R_L_9(movePartical, 27, 540, 'L');
  thread R_L_10(movePartical, 28, 6, 'L');


  thread D_U_1(movePartical, 3, 80, 'U');
  thread D_U_2(movePartical, 30, 160, 'U');
  thread D_U_3(movePartical, 31, 240, 'U');
  thread D_U_4(movePartical, 32, 320, 'U');
  thread D_U_5(movePartical, 33, 400, 'U');
  thread D_U_6(movePartical, 34, 480, 'U');
  thread D_U_7(movePartical, 35, 560, 'U');
  thread D_U_8(movePartical, 36, 640, 'U');
  thread D_U_9(movePartical, 37, 10, 'U');
  thread D_U_10(movePartical, 38, 280, 'U');

  thread U_D_1(movePartical, 4, 80, 'D');
  thread U_D_2(movePartical, 40, 160, 'D');
  thread U_D_3(movePartical, 41, 240, 'D');
  thread U_D_4(movePartical, 42, 320, 'D');
  thread U_D_5(movePartical, 43, 400, 'D');
  thread U_D_6(movePartical, 44, 480, 'D');
  thread U_D_7(movePartical, 45, 560, 'D');
  thread U_D_8(movePartical, 46, 640, 'D');
  thread U_D_9(movePartical, 47, 20, 'D');
  thread U_D_10(movePartical, 48, 690, 'D');


  L_R_1.join();
  L_R_2.join();
  L_R_3.join();
  L_R_4.join();
  L_R_5.join();
  L_R_6.join();
  L_R_7.join();
  L_R_8.join();
  L_R_9.join();
  L_R_10.join();


  R_L_1.join();
  R_L_2.join();
  R_L_3.join();
  R_L_4.join();
  R_L_5.join();
  R_L_6.join();
  R_L_7.join();
  R_L_8.join();
  R_L_9.join();
  R_L_10.join();


  D_U_1.join();
  D_U_2.join();
  D_U_3.join();
  D_U_4.join();
  D_U_5.join();
  D_U_6.join();
  D_U_7.join();
  D_U_8.join();
  D_U_9.join();
  D_U_10.join();


  U_D_1.join();
  U_D_2.join();
  U_D_3.join();
  U_D_4.join();
  U_D_5.join();
  U_D_6.join();
  U_D_7.join();
  U_D_8.join();
  U_D_9.join();
  U_D_10.join();*/
#pragma endregion

 /* movePartical(1,111, 'R');
  insertParticalDirection(1,"R");
  movePartical(2, 92, 'L');
  insertParticalDirection(2,"L");
  movePartical(3, 178, 'D');
  insertParticalDirection(3,"D");
  movePartical(4, 159, 'U');
  insertParticalDirection(4,"U");
  showParticalPaths(1);
  showParticalPaths(2);
  showParticalPaths(3);
  showParticalPaths(4);*/

	direction = 'R';
	int seedpointCounter = 1;
	while (!leftSeedPoints.empty())
	{
		direction = 'R';
		vector<int> tempVal;
		tempVal = leftSeedPoints.back();
		if (tempVal.size() == 4)
		{
			cout << "L " << tempVal[2] << " " << tempVal[3] << endl;
			insertParticalDirection(seedpointCounter, "R");
			movePartical(seedpointCounter, tempVal[2], tempVal[3], direction);
			leftSeedPoints.pop_back();
			seedpointCounter++;
		}
		else
		{
			cout << "L " << tempVal[0] << " " << tempVal[1] << endl;
			insertParticalDirection(seedpointCounter, "R");
			movePartical(seedpointCounter, tempVal[0], tempVal[1], direction);
			leftSeedPoints.pop_back();
			seedpointCounter++;
		}
		
	}

	//movePartical(1,62,direction);
	////seedpointCounter++;
	////movePartical(2,88, direction);
	//
	direction = 'L';

	while (!rightSeedPoints.empty())
	{
		direction = 'L';
		vector<int> tempVal;
		tempVal = rightSeedPoints.back();
		if (tempVal.size() == 4)
		{
			cout << "R " << tempVal[2] << " " << tempVal[3] << endl;
			insertParticalDirection(seedpointCounter, "L");
			movePartical(seedpointCounter, tempVal[2], tempVal[3], direction);
			rightSeedPoints.pop_back();
			seedpointCounter++;
		}
		else
		{
			cout << "R " << tempVal[0] << " " << tempVal[1] << endl;
			insertParticalDirection(seedpointCounter, "L");
			movePartical(seedpointCounter, tempVal[0], tempVal[1], direction);
			rightSeedPoints.pop_back();
			seedpointCounter++;
		}
	}
	//
	//
	direction = 'U';

	while (!downSeedpoint.empty())
	{
		direction = 'U';
		vector<int> tempVal;
		tempVal = downSeedpoint.back();
		if (tempVal.size() == 4)
		{
			cout << "D " << tempVal[2] << " " << tempVal[3] << endl;
			insertParticalDirection(seedpointCounter, "U");
			movePartical(seedpointCounter, tempVal[2], tempVal[3], direction);
			downSeedpoint.pop_back();
			seedpointCounter++;
		}
		else
		{
			cout << "D " << tempVal[0] << " " << tempVal[1] << endl;
			insertParticalDirection(seedpointCounter, "U");
			movePartical(seedpointCounter, tempVal[0], tempVal[1], direction);
			downSeedpoint.pop_back();
			seedpointCounter++;
		}
	}
	//
	direction = 'D';
	//
	while (!upSeedPoints.empty())
	{
		direction = 'D';
		vector<int> tempVal;
		tempVal = upSeedPoints.back();
		if (tempVal.size() == 4)
		{
			cout << "U " << tempVal[2] << " " << tempVal[3] << endl;
			insertParticalDirection(seedpointCounter, "D");
			movePartical(seedpointCounter, tempVal[2], tempVal[3], direction);
			upSeedPoints.pop_back();
			seedpointCounter++;
		}
		else
		{
			cout << "U " << tempVal[0] << " " << tempVal[1] << endl;
			insertParticalDirection(seedpointCounter, "D");
			movePartical(seedpointCounter, tempVal[0], tempVal[1], direction);
			upSeedPoints.pop_back();
			seedpointCounter++;
		}
		
	}

	/*for (int a = 1; a <= seedpointCounter; a++)
	{
		showParticalPaths(a);
	}*/
	//showParticalPaths(13);

	//selectAll();

	showIntersections();

	//showCornerPoints();

	/*Size size(edgeMapVector.size() * 2, edgeMapVector[0].size() * 2);
	Mat resizedImg;
	resize(intersection_image, resizedImg, size);*/

	/*MyFilledCircle2(intersection_image, Point(384, 160));
	MyFilledCircle2(intersection_image, Point(399, 169));
	MyFilledCircle2(intersection_image, Point(402, 169));
	MyFilledCircle2(intersection_image, Point(384, 187));*/
	//MyFilledCircle2(intersection_image, Point(384, 187));
	//MyFilledCircle2(intersection_image, Point(384, 157));
	//MyFilledCircle2(intersection_image, Point(58, 180));
	//MyFilledCircle2(intersection_image, Point(52, 186));
	//MyFilledCircle2(intersection_image, Point(61, 323));

	//MyFilledCircle2(intersection_image, Point(63, 191));
	//MyFilledCircle(intersection_image, Point(43, 54));
	//MyFilledCircle(intersection_image, Point(144, 67));
	//MyFilledCircle(intersection_image, Point(109, 276));
	//MyFilledCircle(intersection_image, Point(302, 217));
	//MyFilledCircle(intersection_image, Point(217, 140));
	//MyFilledCircle(intersection_image, Point(32, 15));
	//MyFilledCircle(intersection_image, Point(150, 106));
	//MyFilledCircle(intersection_image, Point(108, 264));
	//MyFilledCircle(intersection_image, Point(208, 134));

	imshow("Intersections", intersection_image);
	imshow("Edge Map", detected_edges);


	//for (int a = 1; a <= seedpointCounter; a++)
	//{
	   // getParticalPathCoordinates(a);
	   // writeParticalPathToFile(a);
	   // pathCoordinatesX.clear();
	   // pathCoordinatesY.clear();
	//}




   // imshow("Original", detected_edges);
	sqlite3_close(db);




	printf("Time taken: %.2f s\n", ((double)(clock() - tStart) / CLOCKS_PER_SEC));
	//cout << "Number of Particals " << seedpointCounter-1 << endl;
	/// Wait until user exit program by pressing a key
	waitKey(0);

	return 0;
}

