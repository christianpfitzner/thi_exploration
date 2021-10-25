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

bool Explorer::idxInRange(const unsigned int idx, const nav_msgs::OccupancyGrid map) const
{

    const auto map_size = map.info.width*map.info.height; 

    // check if the index within the range of the map
    if(idx < 0)        return false;    
    if(idx > map_size) return false; 

    return true; 
}



bool Explorer::neighborIsUnknown(const std::vector<int> neighbors, const nav_msgs::OccupancyGrid map) const
{
    for (auto i = 0; i < neighbors.size() ; i++)
    {
        if(map.data[neighbors[i]] == UNKNOWN)    return true; 
    }

    return false; 
}



nav_msgs::OccupancyGrid Explorer::copy(const nav_msgs::OccupancyGrid input) const 
{
    nav_msgs::OccupancyGrid output; 
    output.header.frame_id = input.header.frame_id; 
    output.header.seq      = input.header.seq; 
    output.header.stamp    = input.header.stamp; 
    output.info.height     = input.info.height; 
    output.info.width      = input.info.width;  
    output.info.height     = input.info.height; 
    output.info.resolution = input.info.resolution;
    
    return output; 
}




unsigned int Explorer::getNrCellsInMap(const nav_msgs::OccupancyGrid map, int state)
{
    auto counter = 0; 
    
    // iterate over all frontier cells 
    const auto nr_of_cells = map.info.height*map.info.width; 

    for (auto  i = 0; i < nr_of_cells; i++)
    {
        const auto current_cell = map.data[i];
        if(current_cell == state)
        {
            counter++; 
        }
    }

    return counter; 
}


std::vector<int> thi::Explorer::getNeighborIndices(const unsigned int idx, const nav_msgs::OccupancyGrid map, bool get8Neighbors /* =  false */) const
{
    std::vector<int> neighbors; 

    const auto width  = map.info.width; 
    const auto height = map.info.height; 

    // top
    {
        const auto top_idx     = idx - width; 
        if(this->idxInRange(top_idx, map))     neighbors.push_back(top_idx); 
        else                                   neighbors.push_back(thi::UNVALID); 
    }

    // bottom
    {
        const auto bottom_idx  = idx + width; 
        if(this->idxInRange(bottom_idx, map))  neighbors.push_back(bottom_idx); 
        else                                   neighbors.push_back(thi::UNVALID); 
    }
    
    // left
    {
        const auto left_idx    = idx - 1; 
        if(this->idxInRange(left_idx, map))    neighbors.push_back(left_idx); 
        else                                   neighbors.push_back(thi::UNVALID); 
    }

    // right
    {
        const auto right_idx   = idx + 1; 
        if(this->idxInRange(right_idx, map))   neighbors.push_back(right_idx); 
        else                                   neighbors.push_back(thi::UNVALID);
    }

    if(get8Neighbors == true)
    {
        // top left
        {
            const auto top_left_idx     = idx - width - 1; 
            if(this->idxInRange(top_left_idx, map))     neighbors.push_back(top_left_idx); 
            else                                        neighbors.push_back(thi::UNVALID); 
        }

        // top right
        {
            const auto top_right_idx     = idx - width + 1; 
            if(this->idxInRange(top_right_idx, map))    neighbors.push_back(top_right_idx); 
            else                                        neighbors.push_back(thi::UNVALID); 
        }

        // bottom left
        {
            const auto bottom_left_idx     = idx + width - 1; 
            if(this->idxInRange(bottom_left_idx, map))  neighbors.push_back(bottom_left_idx); 
            else                                        neighbors.push_back(thi::UNVALID); 
        }

        // bottom right
        {
            const auto bottom_right_idx     = idx + width + 1; 
            if(this->idxInRange(bottom_right_idx, map)) neighbors.push_back(bottom_right_idx); 
            else                                        neighbors.push_back(thi::UNVALID); 
        }

    }
    return neighbors; 
};



nav_msgs::OccupancyGrid Explorer::findFrontiers(const nav_msgs::OccupancyGrid map) const
{
    nav_msgs::OccupancyGrid frontier_map = copy(map);  
    const auto map_size                  = frontier_map.info.width * frontier_map.info.height;  
    frontier_map.data.resize(map_size); 


    // iterate over all frontier cells 
    const auto nr_of_cells = map.info.height*map.info.width; 

    for (auto i = 0; i < nr_of_cells; i++)
    {
        const auto current_cell = map.data[i];

        if(current_cell == FREE)
        {
            const std::vector<int> neighbors = this->getNeighborIndices(i, map); 

            if( this->neighborIsUnknown(neighbors, map))      frontier_map.data[i] = FRONTIER; 
            else                                              frontier_map.data[i] = NO_FRONTIER;
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
    if( !idxInRange(idx, *map) )
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
    const auto neighbors = this->getNeighborIndices(idx, *map, true); 
    for (const auto n : neighbors)
    {
        floodFill(map, n, oldValue, newValue); 
    }
}



std::vector< std::vector<Frontier> > Explorer::groupFrontiers(nav_msgs::OccupancyGrid map) 
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
    std::vector< std::vector<Frontier> > groups; 
    groups.resize(nr_of_groups); 



    for (size_t i = 0; i <nr_of_cells ; i++)
    {
        const auto current_cell = map.data[i];
        if(current_cell == NO_FRONTIER)
            continue; 


        const auto current_group_idx = current_cell - group_idx_start;  // substract the starting value of the group
        
        Frontier f; 
        f.group        = current_group_idx; 
        f.index_in_map = i; 
        groups[current_group_idx].push_back(f); 
    }


    // output to user about the number of groups
    for(size_t i=0 ; i<groups.size() ; i++)
    {
        ROS_DEBUG_STREAM("Group " << i << ": " << groups[i].size()); 
    }


    visualization_msgs::MarkerArray pose_array; 
    

    // calculate the centroid cell for all frontiers
    auto id = 0; 
    for(const auto group : groups)
    {
        visualization_msgs::Marker pose; 
        pose.header.frame_id = "map"; 
        pose.header.stamp    = ros::Time(); 
        pose.ns  = "my_namespace"; 
        pose.id = id++; 

        auto centroid_row = 0;
        auto centroid_col = 0;  
        for(auto i=0 ; i< group.size() ; i++)
        {
            Point2D_uint coord = getCoordFromIndex(map, group[i].index_in_map); 
            centroid_row      += coord.row; 
            centroid_col      += coord.col; 
        }

        pose.pose.position.x = centroid_row / group.size() * map.info.resolution; 
        pose.pose.position.y = centroid_col / group.size() * map.info.resolution; 

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
        pose.color.a = 1.0; // Don't forget to set the alpha!
        pose.color.r = 1.0;
        pose.color.g = 0.0;
        pose.color.b = 0.0;


        ROS_ERROR_STREAM("centroid: " << pose.pose.position.x << " " << pose.pose.position.y); 

        pose_array.markers.push_back(pose); 
    }


    

    // // publish debug message if anyone is interested
    if(_frontier_map_pub.getNumSubscribers() > 0)
    {
        _frontier_goal_pub.publish(pose_array); 
    }



    return groups; 
}
