#ifndef KDTREE_H
#define KDTREE_H

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

	void insert(std::vector<float> point, int id)
	{
		insert(root, point, 0, id);
	}

	void insert(Node *&root, const std::vector<float> &point, int depth, const int &id) {
		int split = depth % 3;

		if(!root) root = new Node(point, id);
		else if(root->point[split] > point[split]) 
			insert(root->left, point, depth+1, id);
		else
			insert(root->right, point, depth+1, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::vector<float> box(6,0); // boundary of the tolerance box
		box[0] = target[0] - distanceTol; // lefter
		box[1] = target[0] + distanceTol; // righter
		box[2] = target[1] - distanceTol; // lower
		box[3] = target[1] + distanceTol; // upper
		box[4] = target[2] - distanceTol; // back
		box[5] = target[2] + distanceTol; // front
		searchHelper(root, 0, ids, box, target, distanceTol);

		return ids;
	}

	void searchHelper(Node *root, int depth, std::vector<int> &ids, std::vector<float> &box, const std::vector<float> &target, const float &distanceTol) {
		if(!root) return;

		int split = depth % 3;
		
		float dist = sqrt((root->point[0]-target[0])*(root->point[0]-target[0]) + (root->point[1]-target[1])*(root->point[1]-target[1]) + 
																					(root->point[2]-target[2])*(root->point[2]-target[2]));
		if(dist <= distanceTol)
			ids.push_back(root->id);

		if(box[split*2] < root->point[split])
			searchHelper(root->left, depth+1, ids, box, target, distanceTol);
		if(box[split*2+1] > root->point[split])
			searchHelper(root->right, depth+1, ids, box, target, distanceTol);

	}

};


#endif /* KDTREE_H */

