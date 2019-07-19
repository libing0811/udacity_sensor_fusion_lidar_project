// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
  
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
  
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());
  
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudRegion);
  
  
    //remove roof points
    std::vector<int> indices; 
   
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    //Calls the filtering method and returns the filtered point cloud indices.
    roof.filter(indices);
  
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(int point :indices)
      inliers->indices.push_back(point);
  
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);
    
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>);
  
    for(int index: inliers->indices)
    {
     planeCloud->points.push_back(cloud->points[index]);
    }
  
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacleCloud);
  
    /*
    pcl::ExtractIndices<pcl::PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planeCloud);
  
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstacleCloud);
    */
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
  
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    // TODO:: Fill in this function to find inliers for the cloud.
  
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
  
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneWithoutPCL(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
  	srand(time(NULL));
  
  	int pcCount=cloud->points.size();
  	//
  	for(int i=0;i<maxIterations ; i++){
   	 	//random choose 3 inliers to make the plane
      	std::unordered_set<int> inliers;
      	while(inliers.size()<3){
      		inliers.insert(rand()%pcCount);
      	}
      
        //
        auto itr=inliers.begin();
        float x1=cloud->points[*itr].x;
        float y1=cloud->points[*itr].y;
        float z1=cloud->points[*itr].z;
        
        itr++;      
        float x2=cloud->points[*itr].x;
        float y2=cloud->points[*itr].y;
        float z2=cloud->points[*itr].z;
        
        itr++;
        float x3=cloud->points[*itr].x;
        float y3=cloud->points[*itr].y;
        float z3=cloud->points[*itr].z;
        
        //
        float a=(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
        float b=(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
        float c=(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
        float d= -a*x1 - b*y1 - c*z1;
      
      	for(int j=0;j<pcCount;j++){
      		if(inliers.count(j)>0)
           	continue;
          
            //compute distance
            float distance=fabs(cloud->points[j].x*a+cloud->points[j].y*b+cloud->points[j].z*c+d)/(sqrt(a*a+b*b+c*c));
          
            if(distance<distanceThreshold)
                inliers.insert(j);
        
      	}
      
        if(inliers.size()>inliersResult.size())
            inliersResult=inliers;
        
    }

    //get RANSAC plane in inlierResult.
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    std::cout << "cloud points count " << cloud->points.size()<< std::endl;
  
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices); //get indices of clusters
    
    std::cout << "clusters count " << cluster_indices.size()<< std::endl;
  
    //extract point cloud
    for (pcl::PointIndices indice : cluster_indices)
    {
      typename pcl::PointCloud<PointT>::Ptr newCluster(new pcl::PointCloud<PointT>);
      
      for (int index : indice.indices)
      {
        newCluster->points.push_back(cloud->points[index]);
      }
      
      newCluster->width=newCluster->points.size();
      newCluster->height=1;
      newCluster->is_dense=true;
      
      clusters.push_back(newCluster);
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringWithoutPCL(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    std::vector<typename pcl::PointCloud<PointT>::Ptr> result;

    //init kd-tree
    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->points.size(); i++) {
      std::vector<float> point;
      point.push_back(cloud->points[i].x);
      point.push_back(cloud->points[i].y);
      point.push_back(cloud->points[i].z);
    	tree->insert(point,i); 
    }


    //clustering
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed (cloud->points.size(),false);
  
    for( int i =0 ;i < cloud->points.size();i++){
        //if point has been processed, skip
        if(processed[i]==true)
            continue;
        
        std::vector<int> cluster;
        
        proximity(cluster, i, processed , tree, cloud, clusterTolerance);

        //min size & max size 
        if(cluster.size()<minSize || cluster.size()>maxSize)
            continue;
        
        clusters.push_back(cluster);
    }
    
    //get clusters's id 
    for(std::vector<int> clusterIds : clusters)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int indice: clusterIds)
  			clusterCloud->points.push_back(cloud->points[indice]);
  		
        result.push_back(clusterCloud);
  	}

	return result;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(std::vector<int>& cluster,int id,std::vector<bool>& processed ,KdTree * tree, typename pcl::PointCloud<PointT>::Ptr cloud ,float distanceTol){
	
    cluster.push_back(id);
    processed[id]=true;
    
    std::vector<float> target;
    target.push_back(cloud->points[id].x);
    target.push_back(cloud->points[id].y);
    target.push_back(cloud->points[id].z);
    
    std::vector<int> neighbors=tree->search(target, distanceTol);
    for(int neighborIndex : neighbors){
        if(processed[neighborIndex]==false){
            proximity(cluster, neighborIndex,processed, tree, cloud, distanceTol);
        }
    }
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
