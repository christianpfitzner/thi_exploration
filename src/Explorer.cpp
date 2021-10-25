/* Copyright (c), Prof. Dr. Christian Pfitzner, All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
 */


#include <thi_exploration/Explorer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <thi_exploration/MapOperations.h>


using namespace thi; 


Explorer::Explorer(ros::NodeHandle nh)
{
    _nh = nh; 

    constexpr auto queue_size = 1; 

    _resize_map_pub   = _nh.advertise<nav_msgs::OccupancyGrid>("debug/resized_map",  queue_size); 
    _frontier_map_pub = _nh.advertise<nav_msgs::OccupancyGrid>("debug/frontier_map", queue_size); 


    _frontier_goal_pub = _nh.advertise<visualization_msgs::MarkerArray>("debug/frontier_pose", queue_size); 
}


Explorer::~Explorer(void)
{
    // default destructor for this class
}





void Explorer::process(void)
{
    // // resize map to speed up exploration
    // const nav_msgs::OccupancyGrid resizedMap      = this->resizeMap(_map, 0.1);

    ROS_ERROR_STREAM("Free cells:     " << thi::MapOperations::getFreeCellsInMap(    _map)); 
    ROS_ERROR_STREAM("occ  cells:     " << thi::MapOperations::getOccupiedCellsInMap(_map)); 
    ROS_ERROR_STREAM("unkn cells:     " << thi::MapOperations::getUnknownCellsInMap( _map)); 

    // find frontiers
    const nav_msgs::OccupancyGrid frontier_map    = this->findFrontiers(_map);
    
    ROS_ERROR_STREAM("frontier cells: " << thi::MapOperations::getFrontierCellsInMap(_map) ); 

    // group frontiers 
    std::vector< Frontier > frontiers = this->groupFrontiers(frontier_map);

    // // weight frontiers      
    // const std::vector<Frontier> frontier_weighted = this->weightFrontiers(frontier_grouped);
}


nav_msgs::OccupancyGrid Explorer::findFrontiers(const nav_msgs::OccupancyGrid map) const
{
    nav_msgs::OccupancyGrid frontier_map = thi::MapOperations::copy(map);  
    const auto map_size                  = frontier_map.info.width * frontier_map.info.height;  
    frontier_map.data.resize(map_size); 


    // iterate over all frontier cells 
    for (auto i = 0; i < map_size; i++)
    {
        const auto current_cell = map.data[i];

        if(current_cell == FREE)
        {
            const std::vector<int> neighbors = thi::MapOperations::getNeighborIndices(i, map, false); 

            if( thi::MapOperations::neighborIsUnknown(neighbors, map))      
                frontier_map.data[i] = FRONTIER; 
            else                                              
                frontier_map.data[i] = NO_FRONTIER;
        }
        else
        {
            frontier_map.data[i] = NO_FRONTIER; 
        }
    }

    // publish debug message if anyone is interested
    if(_frontier_map_pub.getNumSubscribers() > 0)
    {
    _frontier_map_pub.publish(frontier_map); 
    }

    return frontier_map; 
}



void Explorer::floodFill(nav_msgs::OccupancyGrid* map, unsigned int idx, int oldValue, int newValue)
{
    // check if index is valid
    if( !thi::MapOperations::idxInRange(idx, *map) )
    {
        return; 
    }

    // check if the current cell is the old value
    if(map->data[idx] != oldValue)
    {
        return; 
    }

    map->data[idx] = newValue; 

    // apply recursion
    const auto neighbors = thi::MapOperations::getNeighborIndices(idx, *map, true); 
    for (const auto n : neighbors)
    {
        floodFill(map, n, oldValue, newValue); 
    }
}



std::vector< Frontier > Explorer::groupFrontiers(nav_msgs::OccupancyGrid map) 
{
    const auto nr_of_cells = map.info.height*map.info.width; 

    auto group_idx = 111;             //!todo remove the magic number
    const auto group_idx_start = group_idx; 

    for (size_t i = 0; i < nr_of_cells; i++)
    {
        if(map.data[i] != FRONTIER)
            continue; 

        this->floodFill(&map, i, FRONTIER, group_idx); 
        group_idx++;         
    }


    const auto nr_of_groups = group_idx - group_idx_start; 


    // create a vector with the size of all groups
    std::vector< Frontier > frontiers; 
    frontiers.resize(nr_of_groups); 

    // init all frontiers
    for(auto i=0 ; i<nr_of_groups ; i++)
    {
        frontiers[i].setMap(_map); 
    }




    for (size_t i = 0; i <nr_of_cells ; i++)
    {
        const auto current_cell = map.data[i];

        if(current_cell == NO_FRONTIER)
            continue; 

        const auto current_group_idx = current_cell - group_idx_start;  // substract the starting value of the group
        
        frontiers[current_group_idx].group = current_group_idx; 
        frontiers[current_group_idx].addIndex(i); 
    }


    // output to user about the number of groups
    for(size_t i=0 ; i<frontiers.size() ; i++)
    {
        ROS_DEBUG_STREAM("Group " << i << ": " << frontiers[i].getSize()); 
    }


    visualization_msgs::MarkerArray pose_array; 
    

    // calculate the centroid cell for all frontiers
    auto id = 0; 
    for(const auto f : frontiers)
    {
        visualization_msgs::Marker pose; 
        pose.header.frame_id = "map"; 
        pose.header.stamp    = ros::Time(); 
        pose.ns              = "exploration"; 
        pose.id              = id++; 

        const auto centroid  = f.getCentroid();
        pose.pose.position.x = centroid.x; 
        pose.pose.position.y = centroid.y; 

        tf2::Quaternion q;
        q.setRPY( 90, 0, 0 );
        q.normalize(); 

        pose.pose.orientation.w = q[3]; 
        pose.pose.orientation.x = q[0]; 
        pose.pose.orientation.y = q[1]; 
        pose.pose.orientation.z = q[2]; 

        pose.type = visualization_msgs::Marker::SPHERE; 

        pose.scale.x = 0.1;
        pose.scale.y = 0.1;
        pose.scale.z = 0.1;
        pose.color.a = 1.0; 
        pose.color.r = 1.0;
        pose.color.g = 0.0;
        pose.color.b = 0.0;


        ROS_DEBUG_STREAM("centroid: " << pose.pose.position.x << " " << pose.pose.position.y); 

        pose_array.markers.push_back(pose); 
    }

    // // publish debug message if anyone is interested
    if(_frontier_map_pub.getNumSubscribers() > 0)
    {
        _frontier_goal_pub.publish(pose_array); 
    }



    return frontiers; 
}


std::vector<Frontier> Explorer::weightFrontiers(std::vector<Frontier> frontiers) const
{
    std::vector<Frontier> weighted_frontiers; 



    // get the current pose of the robot




    // calculate the euclidean distance between a frontier and the robot



    




    return weighted_frontiers; 
}