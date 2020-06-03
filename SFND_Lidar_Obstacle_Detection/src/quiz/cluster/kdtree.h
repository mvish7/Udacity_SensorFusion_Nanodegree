/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId): point(arr), id(setId), left(NULL), right(NULL)
	{

	}
};

struct KdTree
{
    Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert_helper_fun(Node* &node, int depth, std::vector<float> point,int id)
    {

	    int dim = 0;

	    if (node==NULL)
        {
            node = new Node(point, id);
        }
	    else
        {
            dim = depth%2;

            if (point[dim] < (node-> point[dim]))
                insert_helper_fun(node->left, depth+1, point, id);
            else
                insert_helper_fun(node->right, depth+1, point, id);
        }

    }

	void insert(std::vector<float> point, int id)
	{
		// Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		insert_helper_fun(root, 0, point, id);



	}

	bool in_box(std::vector<float>box_coords, std::vector<float> node)
    {
        return ((box_coords[0] < node[0]) && (node[0] < box_coords[1])) &&
               ((box_coords[2] < node[1]) && (node[1] < box_coords[3]));
    }

	void search_helper_fun(Node* &node,  std::vector<float> target, float distanceTol, int depth,  std::vector<int>& ids, std::vector<float>box_coords)
    {
	    if (node != NULL)
        {
            int dim = depth%2;
            float box_dim_min, box_dim_max;
            // dereference the target val??
            if (in_box(box_coords, node->point))
            {
                float dist_bet_pts = sqrt(std::pow((node->point[0] - target[0]), 2) + std::pow((node->point[1]-target[1]), 2));
                if (dist_bet_pts<distanceTol)
                {
                    ids.push_back(node->id);

                }
            }

            //box_coords has [x_min, x_max, y_min, y_max]
            //below we are fetching x_max and y_max for comparison
            if(dim==0)
            {
                 box_dim_min = box_coords[0];
                 box_dim_max = box_coords[1];
            }

            if(dim==1)
            {
                 box_dim_min = box_coords[2];
                 box_dim_max = box_coords[3];
            }

            if(box_dim_min < node->point[dim])
            {
                //if (node->left != NULL)
                search_helper_fun(node->left, target, distanceTol, depth+1, ids, box_coords);

            }
            if(box_dim_max > node->point[dim])
            {
                //if (node->right != NULL)
                search_helper_fun(node->right, target, distanceTol, depth+1, ids, box_coords);
            }
        }

    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		int depth = 0;

		std::vector<float> box_coords;
		box_coords.push_back(target[0]-distanceTol);
		box_coords.push_back(target[0]+distanceTol);
		box_coords.push_back(target[1]-distanceTol);
		box_coords.push_back(target[1]+distanceTol);

		search_helper_fun(root, target, distanceTol, depth, ids, box_coords);
		return ids;
	}
	

};




