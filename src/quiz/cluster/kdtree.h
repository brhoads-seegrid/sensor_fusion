#ifndef KDTREE_INCLUDED
#define KDTREE_INCLUDED



/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <vector>
using namespace std;


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node **node, uint depth, vector<float> point, int id) {
		if (*node == NULL) {
			*node = new Node(point, id);
		} else {
			uint cd = depth % 2;
			if (point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(vector<int> &ids, vector<float> target, float distanceTol, Node* node, uint depth)
	{
		if (node != NULL) {
			if ((node->point[0]>=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol)) &&
			    (node->point[1]>=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol))) {
					float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+
					                      (node->point[1]-target[1])*(node->point[1]-target[1]));
					if (distance <= distanceTol) ids.push_back(node->id);
				}

				if ((target[depth%2] - distanceTol) < node->point[depth%2])
					searchHelper(ids, target, distanceTol, node->left, depth+1);
				if ((target[depth%2] + distanceTol) > node->point[depth%2])
					searchHelper(ids, target, distanceTol, node->right, depth + 1);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(ids, target, distanceTol, root, 0);
		return ids;
	}
};



#endif


