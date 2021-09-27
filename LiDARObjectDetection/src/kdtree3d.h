#include "render/render.h"
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>

// Structure to represent node of kd tree

template <typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node( PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node<PointT>** node, uint depth, PointT point, int id){
		if(*node == NULL) 
			*node = new Node<PointT>(point, id);
		else
		{
			uint cd = depth % 3;
			if (depth % 3 == 0){
				if(point.x < ((*node)->point.x))
					insertHelper(&((*node)->left), depth+1, point, id);
				else
					insertHelper(&((*node)->right), depth+1, point, id);
			}
			if (depth % 3 == 1){
				if(point.y < ((*node)->point.y))
					insertHelper(&((*node)->left), depth+1, point, id);
				else
					insertHelper(&((*node)->right), depth+1, point, id);
			}
			if (depth % 3 == 2){
				if(point.z < ((*node)->point.z))
					insertHelper(&((*node)->left), depth+1, point, id);
				else
					insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
		

	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);


	}

	void searchHelper(PointT target, std::vector<int> &ids, Node<PointT>* node, int depth, float distanceTol){

		if(node != NULL){
			if(node->point.x>=(target.x-distanceTol) && node->point.x<=(target.x+distanceTol) && node->point.y>=(target.y-distanceTol) && node->point.y<=(target.y+distanceTol)&& node->point.z>=(target.z-distanceTol) && node->point.z<=(target.z+distanceTol)){
				float distance = sqrt((node->point.x-target.x)*(node->point.x-target.x) + (node->point.y-target.y)*(node->point.y-target.y) + (node->point.z-target.z)*(node->point.z-target.z));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}
			if (depth % 3 == 0){
				if(target.x - distanceTol < node->point.x)
					searchHelper(target, ids, node->left, depth+1, distanceTol);
				if(target.x + distanceTol > node->point.x)
					searchHelper(target, ids, node->right, depth+1, distanceTol);
			}
			if (depth % 3 == 1){
				if(target.y - distanceTol < node->point.y)
					searchHelper(target, ids, node->left, depth+1, distanceTol);
				if(target.y + distanceTol > node->point.y)
					searchHelper(target, ids, node->right, depth+1, distanceTol);
			}
			if (depth % 3 == 2){
				if(target.z - distanceTol < node->point.z)
					searchHelper(target, ids, node->left, depth+1, distanceTol);
				if(target.z + distanceTol > node->point.z)
					searchHelper(target, ids, node->right, depth+1, distanceTol);
			}
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{

		std::vector<int> ids;
		searchHelper(target, ids, root, 0, distanceTol);
		return ids;
	}
	

};




