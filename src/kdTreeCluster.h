#ifndef KDTREECLUSTER_H_
#define KDTREECLUSTER_H_

#include "render/render.h"

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

	
	void insertTree(Node** node, uint depth, std::vector<float> point, int id){
		// the function should create a new node and place correctly with in the root 
		if(*node == NULL)
      	{
        	*node = new Node(point, id);
      	}

      	else 
      	{
			// Calculate current depth
        	uint cd = depth%3;
			if (point[cd] < ((*node)->point[cd]))
				insertTree(&((*node)->left), depth+1, point, id);
			else
				insertTree(&((*node)->right), depth+1, point, id);				
      	}

	}
	
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		insertTree(&root, 0, point, id);
	}
	

	void searchTree(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids){
		if (node!=NULL ){
			if((node->point[0]>=target[0]-distanceTol)&&(node->point[0]<=target[0]+distanceTol)&&
			(node->point[1]>=target[1]-distanceTol)&&(node->point[1]<=target[1]+distanceTol)&&
			(node->point[2]>=target[2]-distanceTol)&&(node->point[2]<=target[2]+distanceTol)){
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1])
				+(node->point[2]-target[2])*(node->point[2]-target[2]));
				if(distance<=distanceTol)
					ids.push_back(node->id);
			}

			// check across boundary
			if((target[depth%3]-distanceTol)<node->point[depth%3])
				searchTree(target, node->left, depth+1, distanceTol, ids);
			if((target[depth%3]+distanceTol)>node->point[depth%3])
				searchTree(target, node->right, depth+1, distanceTol, ids);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchTree(target, root, 0, distanceTol,ids);
		return ids;
	}
	

};

void clusterPoints(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, 
float distanceTol,int maxSize);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,int minSize, int maxSize);


#endif