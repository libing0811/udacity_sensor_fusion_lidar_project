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
  	int level;

	Node(std::vector<float> arr, int setId, int setLevel)
	:	point(arr), id(setId), left(NULL), right(NULL),level(setLevel)
	{}
};


struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

  /*
  	void InitBalanceTree(typename pcl::PointCloud<pcl::PointXZYI>::Ptr cloud){
    	std::vector<pcl::PointXZYI> left;
      std::vector<pcl::PointXZYI> right;
      root= SortPoints(0, cloud->points, left, right);
      
    }
  
 	Node * SortPoints(int level, std::vector<pcl::PointXZYI> points, std::vector<pcl::PointXZYI> &left, std::vector<pcl::PointXZYI> & right){
      //there is something wrong with id, if used , must be corrected.
      
      if(points.size<3 && points.size>0){
      	for(int i=0;i<points.size();i++){
      		insert(points[i],i);
      	}
      }
      else{
        if(level%3==0)
              std::sort (points.begin(), points.end(), compare_x);

        if(level%3==1)
              std::sort (points.begin(), points.end(), compare_y);

        if(level%3==2)
              std::sort (points.begin(), points.end(), compare_z);

        int midIndex=points.size()/2;
        x=insert(points[midIndex], midIndex);
        
        x->left=SortPoints(level+1,points.begin(), points[midIndex-1],)
          
        x->right=SortPoints(level+1,points.begin(), points[midIndex-1],)
      }
    }
  
  bool compare_x (pcl::PointXZYI i, pcl::PointXZYI j) { return (i.x<j.x); }
  bool compare_y (pcl::PointXZYI i, pcl::PointXZYI j) { return (i.y<j.y); }
  bool compare_z (pcl::PointXZYI i, pcl::PointXZYI j) { return (i.z<j.z); }
  */
  
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
      insertHelper(&root, point,id, 0);
	}
  
  void insertHelper(Node **node, std::vector<float> point, int id, int level)
  {
      if(*node == NULL)
      {
        *node = new Node(point, id, level);
      }
      else{

        int index=level%3;
        if((*node)->point[index]>point[index]){
              //left
            insertHelper(&((*node)->left),point, id, level+1);
        }
        else{
              //right
            insertHelper(&((*node)->right),point, id, level+1);
        }
        /*
        if(level%3==0){
          //compare x
          if((*node)->point[0]>point[0]){
              //left
            insertHelper(&((*node)->left),point, id, level+1);
          }
          else{
              //right
            insertHelper(&((*node)->right),point, id, level+1);
          }
        }
        else if(level%3==1){
          //compare y
          if((*node)->point[1]>point[1]){
              //left
            insertHelper(&((*node)->left),point, id, level+1);
          }
          else{
              //right
            insertHelper(&((*node)->right),point, id, level+1);
          }
        }
        else{
          //compare z
          if((*node)->point[2]>point[2]){
              //left
            insertHelper(&((*node)->left),point, id, level+1);
          }
          else{
              //right
            insertHelper(&((*node)->right),point, id, level+1);
          }
        }*/
    }
  }
  
  Node * getNewNode(std::vector<float> point,int id, int level){
    return new Node(point, id, level);
  }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
    searchHelper(root,target,distanceTol,&ids);
		return ids;
	}
	
  
  void searchHelper(Node * node, std::vector<float>target, float distanceTol, std::vector<int> *ids_pointer){
    if(node==NULL)
        return;
    
    //find index
    int index= node->level%3;

    if(node->point[index]>target[index]+distanceTol){
      	//search left zone only
        searchHelper(node->left,target, distanceTol, ids_pointer);
    }
    else if(node->point[index]<target[index]-distanceTol){	
        //search right zone only
        searchHelper(node->right, target, distanceTol, ids_pointer);
    }
    else{		
        //compute disance between node and target
        float distance = sqrt((target[0]-node->point[0])*(target[0]-node->point[0])+(target[2]-node->point[2])*(target[2]-node->point[2])+(target[1]-node->point[1])*(target[1]-node->point[1]));
        if(distance < distanceTol){
        	ids_pointer->push_back(node->id);
        }
        
        //search both left and right branch
        searchHelper(node->left, target, distanceTol, ids_pointer);
        searchHelper(node->right, target, distanceTol, ids_pointer);
    }

    /*
    if(node->level%3==0){
    	//compare x
      if(node->point[0]>target[0]+distanceTol){
      		
        //search left zone only
        searchHelper(node->left,target, distanceTol, ids_pointer);
      }
      else if(node->point[0]<target[0]-distanceTol){
      		
        //search right zone only
        searchHelper(node->right, target, distanceTol, ids_pointer);
      }
      else{
      		
        //compute disance between node and target
        float distance = sqrt((target[0]-node->point[0])*(target[0]-node->point[0])+(target[2]-node->point[2])*(target[2]-node->point[2])+(target[1]-node->point[1])*(target[1]-node->point[1]));
        if(distance < distanceTol){
        	ids_pointer->push_back(node->id);
        }
        
        //search both left and right branch
      		
        searchHelper(node->left, target, distanceTol, ids_pointer);
        searchHelper(node->right, target, distanceTol, ids_pointer);
      }
    }
    else if(node->level%3==1){
    	//compare y
      if(node->point[1]>target[1]+distanceTol){
      		
        //search right zone only
        searchHelper(node->left,target, distanceTol, ids_pointer);
      }
      else if(node->point[1]<target[1]-distanceTol){
      		
        //search right zone only
        searchHelper(node->right, target, distanceTol, ids_pointer);
      }
      else{
      		
        //compute disance between node and target
        float distance = sqrt((target[0]-node->point[0])*(target[0]-node->point[0])+(target[2]-node->point[2])*(target[2]-node->point[2])+(target[1]-node->point[1])*(target[1]-node->point[1]));
        if(distance < distanceTol){
        	ids_pointer->push_back(node->id);
        }
        
        //search both left and right branch
      		
        searchHelper(node->left, target, distanceTol, ids_pointer);
        searchHelper(node->right, target, distanceTol, ids_pointer);
      }
    }
    else{
    	//compute z
      if(node->point[2]>target[2]+distanceTol){
      		
        //search right zone only
        searchHelper(node->left,target, distanceTol, ids_pointer);
      }
      else if(node->point[2]<target[2]-distanceTol){
      		
        //search right zone only
        searchHelper(node->right, target, distanceTol, ids_pointer);
      }
      else{
      		
        //compute disance between node and target
        float distance = sqrt((target[0]-node->point[0])*(target[0]-node->point[0])+(target[2]-node->point[2])*(target[2]-node->point[2])+(target[1]-node->point[1])*(target[1]-node->point[1]));
        if(distance < distanceTol){
        	ids_pointer->push_back(node->id);
        }
        
        //search both left and right branch
      		
        searchHelper(node->left, target, distanceTol, ids_pointer);
        searchHelper(node->right, target, distanceTol, ids_pointer);
      }
    }
    */
  }

};




